#pragma once

#include <esp_afe_sr_iface.h>
#include <esp_afe_sr_models.h>
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
        speaker_reference_mutex(nullptr) {}

  bool init() {
    // Initialize mutex first
    speaker_reference_mutex = xSemaphoreCreateMutex();
    if (!speaker_reference_mutex) {
      ESP_LOGE("AEC", "Failed to create speaker reference mutex");
      return false;
    }

    // Initialize reference buffer with increased size
    speaker_reference_buffer =
        (int16_t*)heap_caps_malloc(SAMPLES_PER_FRAME * sizeof(int16_t),
                                   MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
    if (!speaker_reference_buffer) {
      ESP_LOGE("AEC", "Failed to allocate speaker reference buffer");
      return false;
    }
    memset(speaker_reference_buffer, 0, SAMPLES_PER_FRAME * sizeof(int16_t));

    // Get AFE handle for voice communication
    afe_handle = const_cast<esp_afe_sr_iface_t*>(&ESP_AFE_VC_HANDLE);

    // Configure AFE with optimized settings for mono setup
    afe_config_t afe_config = AFE_CONFIG_DEFAULT();
    afe_config.aec_init = true;
    afe_config.se_init = true;
    afe_config.vad_init = false;
    afe_config.wakenet_init = false;
    afe_config.voice_communication_init = true;
    afe_config.voice_communication_agc_init = true;
    afe_config.voice_communication_agc_gain = 15;
    afe_config.afe_mode = SR_MODE_LOW_COST;
    afe_config.afe_perferred_core = 0;
    afe_config.afe_perferred_priority = 5;
    afe_config.afe_ringbuf_size = 320;  // Increased ring buffer size

    // Configure for single mic + reference as per documentation
    afe_config.pcm_config.total_ch_num = 2;  // Mic + Reference
    afe_config.pcm_config.mic_num = 1;       // Single mic
    afe_config.pcm_config.ref_num = 1;       // Single reference channel
    afe_config.pcm_config.sample_rate = SAMPLE_RATE;

    // Create AFE instance
    afe_data = afe_handle->create_from_config(&afe_config);
    if (!afe_data) {
      ESP_LOGE("AEC", "Failed to create AFE instance");
      return false;
    }

    // Get expected chunk size for proper buffer allocation
    int chunksize = afe_handle->get_feed_chunksize(afe_data);
    ESP_LOGI("AEC", "AFE chunk size: %d", chunksize);
    ESP_LOGI("AEC", "AFE initialized successfully");
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

  // Process mic input with reference signal
  int16_t* processAudio(int16_t* mic_data, size_t* samples_processed) {
    if (!afe_handle || !afe_data) {
      ESP_LOGE("AEC", "AFE not initialized");
      return mic_data;
    }

    // Create interleaved buffer with proper allocation
    int16_t* combined_buffer =
        (int16_t*)heap_caps_malloc(SAMPLES_PER_FRAME * 2 * sizeof(int16_t),
                                   MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);

    if (!combined_buffer) {
      ESP_LOGE("AEC", "Failed to allocate combined buffer");
      return mic_data;
    }

    // Interleave mic and reference data with synchronization
    if (xSemaphoreTake(speaker_reference_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      // Ensure proper interleaving as per documentation
      for (int i = 0; i < SAMPLES_PER_FRAME; i++) {
        combined_buffer[i * 2] = mic_data[i];  // Mic channel
        combined_buffer[i * 2 + 1] = speaker_reference_buffer[i];  // Reference
      }
      xSemaphoreGive(speaker_reference_mutex);
    }

    // Process through AFE
    int ret = afe_handle->feed(afe_data, combined_buffer);
    if (ret != 0) {
      ESP_LOGW("AEC", "AFE feed returned: %d", ret);
      heap_caps_free(combined_buffer);
      return mic_data;
    }

    // Fetch processed audio with immediate retry on failure
    afe_fetch_result_t* result = nullptr;
    int retry_count = 0;
    const int max_retries = 3;

    while (retry_count < max_retries) {
      result = afe_handle->fetch(afe_data);
      if (result && result->data) {
        break;
      }
      vTaskDelay(pdMS_TO_TICKS(1));
      retry_count++;
    }

    heap_caps_free(combined_buffer);

    if (!result || !result->data) {
      ESP_LOGE("AEC", "AFE fetch failed after %d retries", retry_count);
      return mic_data;
    }

    *samples_processed = result->data_size / sizeof(int16_t);
    return (int16_t*)result->data;
  }

  // Store reference signal from speaker with size validation
  void updateReferenceBuffer(const int16_t* speaker_data, size_t samples) {
    if (!speaker_data || samples == 0) {
      return;
    }

    if (speaker_reference_mutex &&
        xSemaphoreTake(speaker_reference_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      if (speaker_reference_buffer) {
        size_t copy_size = std::min(samples * sizeof(int16_t),
                                    SAMPLES_PER_FRAME * sizeof(int16_t));
        memcpy(speaker_reference_buffer, speaker_data, copy_size);
      }
      xSemaphoreGive(speaker_reference_mutex);
    }
  }

 private:
  esp_afe_sr_iface_t* afe_handle;
  esp_afe_sr_data_t* afe_data;
  int16_t* speaker_reference_buffer;
  SemaphoreHandle_t speaker_reference_mutex;
};