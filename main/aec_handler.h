#pragma once

#include <esp_afe_sr_iface.h>
#include <esp_afe_sr_models.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <string.h>

#include <algorithm>

#include "esp_log.h"
#include "i2s_config.h"

class AECHandler {
 public:
  AECHandler()
      : afe_handle(nullptr),
        afe_data(nullptr),
        speaker_reference_buffer(nullptr),
        speaker_reference_mutex(nullptr),
        frame_size(0) {}

  bool init() {
    // Initialize mutex
    speaker_reference_mutex = xSemaphoreCreateMutex();
    if (!speaker_reference_mutex) {
      ESP_LOGE("AEC", "Failed to create mutex");
      return false;
    }

    // Get AFE handle
    afe_handle = const_cast<esp_afe_sr_iface_t*>(&ESP_AFE_VC_HANDLE);
    if (!afe_handle) {
      ESP_LOGE("AEC", "Failed to get AFE handle");
      return false;
    }

    // Configure AFE
    afe_config_t afe_config = AFE_CONFIG_DEFAULT();
    afe_config.aec_init = true;
    afe_config.se_init = true;
    afe_config.vad_init = false;
    afe_config.wakenet_init = false;
    afe_config.voice_communication_init = true;
    afe_config.voice_communication_agc_init = true;
    afe_config.voice_communication_agc_gain = 15;
    afe_config.afe_mode = SR_MODE_LOW_COST;
    afe_config.afe_perferred_core = 1;
    afe_config.afe_perferred_priority = 5;
    afe_config.afe_ringbuf_size = 50;
    afe_config.memory_alloc_mode = AFE_MEMORY_ALLOC_MORE_PSRAM;
    afe_config.pcm_config.total_ch_num = 2;  // Mic + Reference
    afe_config.pcm_config.mic_num = 1;       // Single mic
    afe_config.pcm_config.ref_num = 1;       // Single reference
    afe_config.pcm_config.sample_rate = SAMPLE_RATE;

    // Create AFE instance
    afe_data = afe_handle->create_from_config(&afe_config);
    if (!afe_data) {
      ESP_LOGE("AEC", "Failed to create AFE instance");
      return false;
    }

    // Get the frame size AFE expects
    frame_size = afe_handle->get_feed_chunksize(afe_data);
    if (frame_size <= 0) {
      ESP_LOGE("AEC", "Invalid frame size from AFE");
      return false;
    }

    ESP_LOGI("AEC", "AFE frame size: %d", frame_size);

    // Initialize reference buffer with AFE frame size
    speaker_reference_buffer = (int16_t*)heap_caps_malloc(
        frame_size * sizeof(int16_t), MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);

    if (!speaker_reference_buffer) {
      ESP_LOGE("AEC", "Failed to allocate reference buffer");
      return false;
    }

    memset(speaker_reference_buffer, 0, frame_size * sizeof(int16_t));
    ESP_LOGI("AEC", "AEC initialized successfully");
    return true;
  }

  void deinit() {
    if (afe_handle && afe_data) {
      afe_handle->destroy(afe_data);
      afe_data = nullptr;
      afe_handle = nullptr;
    }

    if (speaker_reference_buffer) {
      heap_caps_free(speaker_reference_buffer);
      speaker_reference_buffer = nullptr;
    }

    if (speaker_reference_mutex) {
      vSemaphoreDelete(speaker_reference_mutex);
      speaker_reference_mutex = nullptr;
    }
  }

  int16_t* processAudio(int16_t* mic_data, size_t* samples_processed) {
    if (!afe_handle || !afe_data || !mic_data) {
      ESP_LOGE("AEC", "Invalid state or input");
      return mic_data;
    }

    static int64_t last_error_time = 0;
    int64_t current_time =
        esp_timer_get_time() / 1000;  // Convert to milliseconds

    // Create interleaved buffer - exact size AFE expects
    int16_t* combined_buffer = (int16_t*)heap_caps_malloc(
        frame_size * 2 * sizeof(int16_t),  // 2 channels
        MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);

    if (!combined_buffer) {
      ESP_LOGE("AEC", "Failed to allocate combined buffer");
      return mic_data;
    }

    // Interleave mic and reference data
    bool has_reference = false;
    if (xSemaphoreTake(speaker_reference_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      if (speaker_reference_buffer) {
        has_reference = true;
        // Carefully interleave exactly frame_size samples
        for (int i = 0; i < frame_size; i++) {
          combined_buffer[i * 2] = mic_data[i];
          combined_buffer[i * 2 + 1] = speaker_reference_buffer[i];
        }
      }
      xSemaphoreGive(speaker_reference_mutex);
    }

    // If no reference data, fill with zeros
    if (!has_reference) {
      for (int i = 0; i < frame_size; i++) {
        combined_buffer[i * 2] = mic_data[i];
        combined_buffer[i * 2 + 1] = 0;
      }
      if (current_time - last_error_time > 1000) {  // Log once per second
        ESP_LOGW("AEC", "No reference data available");
        last_error_time = current_time;
      }
    }

    // Feed data to AFE
    int ret = afe_handle->feed(afe_data, combined_buffer);
    if (ret != 0) {
      if (current_time - last_error_time > 1000) {  // Log once per second
        ESP_LOGW("AEC", "AFE feed failed with status: %d", ret);
        if (ret == 512) {
          ESP_LOGW("AEC", "Feed size: %d, Frame size: %d", frame_size * 2,
                   frame_size);
        }
        last_error_time = current_time;
      }
    }

    // Try to fetch processed audio
    afe_fetch_result_t* result = nullptr;
    for (int retry = 0; retry < 2; retry++) {  // Try twice
      result = afe_handle->fetch(afe_data);
      if (result && result->data) {
        break;
      }
      vTaskDelay(1);  // Small delay before retry
    }

    heap_caps_free(combined_buffer);  // Free temporary buffer

    if (!result || !result->data) {
      if (current_time - last_error_time > 1000) {
        ESP_LOGE("AEC", "AFE fetch failed");
        last_error_time = current_time;
      }
      return mic_data;
    }

    *samples_processed = result->data_size / sizeof(int16_t);
    return (int16_t*)result->data;
  }

  void updateReferenceBuffer(const int16_t* speaker_data, size_t samples) {
    if (!speaker_data || samples == 0 || samples > frame_size) {
      return;
    }

    if (speaker_reference_mutex &&
        xSemaphoreTake(speaker_reference_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      if (speaker_reference_buffer) {
        memcpy(speaker_reference_buffer, speaker_data,
               samples * sizeof(int16_t));
      }
      xSemaphoreGive(speaker_reference_mutex);
    }
  }

 private:
  esp_afe_sr_iface_t* afe_handle;
  esp_afe_sr_data_t* afe_data;
  int16_t* speaker_reference_buffer;
  SemaphoreHandle_t speaker_reference_mutex;
  int frame_size;  // AFE's expected frame size
};