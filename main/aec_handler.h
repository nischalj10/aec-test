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
    // Get AFE handle first
    afe_handle = const_cast<esp_afe_sr_iface_t*>(&ESP_AFE_VC_HANDLE);
    if (!afe_handle) {
      ESP_LOGE("AEC", "Failed to get AFE handle");
      return false;
    }

    // Get the exact frame size AFE expects
    afe_config_t afe_config = AFE_CONFIG_DEFAULT();
    afe_config.pcm_config.total_ch_num = 2;  // Total channels (mic + ref)
    afe_config.pcm_config.mic_num = 1;       // One mic
    afe_config.pcm_config.ref_num = 1;       // One reference
    afe_config.pcm_config.sample_rate = SAMPLE_RATE;

    // Voice communication specific settings
    afe_config.aec_init = true;
    afe_config.se_init = true;
    afe_config.voice_communication_init = true;
    afe_config.wakenet_init = false;
    afe_config.vad_init = false;

    // Create AFE instance
    afe_data = afe_handle->create_from_config(&afe_config);
    if (!afe_data) {
      ESP_LOGE("AEC", "Failed to create AFE instance");
      return false;
    }

    // Get the actual frame size
    frame_size = afe_handle->get_feed_chunksize(afe_data);
    ESP_LOGI("AEC", "AFE frame size: %d samples", frame_size);

    // Allocate internal buffers
    size_t buffer_size = frame_size * sizeof(int16_t);
    mic_buffer = (int16_t*)heap_caps_malloc(buffer_size, MALLOC_CAP_INTERNAL);
    ref_buffer = (int16_t*)heap_caps_malloc(buffer_size, MALLOC_CAP_INTERNAL);
    if (!mic_buffer || !ref_buffer) {
      ESP_LOGE("AEC", "Failed to allocate buffers");
      return false;
    }

    return true;
  }

  int16_t* processAudio(const int32_t* input_samples, size_t num_samples,
                        size_t* processed_size) {
    if (!afe_data || !input_samples || num_samples == 0) {
      *processed_size = 0;
      return nullptr;
    }

    // Calculate sizes carefully
    const size_t samples_per_channel = frame_size;         // 256
    const size_t total_samples = samples_per_channel * 2;  // 512 (stereo)

    // Allocate buffer for stereo interleaved data
    int16_t* combined_buffer = (int16_t*)heap_caps_malloc(
        total_samples * sizeof(int16_t),  // 512 * 2 = 1024 bytes
        MALLOC_CAP_INTERNAL);

    if (!combined_buffer) {
      ESP_LOGE("AEC", "Failed to allocate combined buffer");
      return nullptr;
    }

    // Fill the interleaved buffer
    for (size_t i = 0; i < samples_per_channel; i++) {
      // Convert 32-bit to 16-bit and interleave
      combined_buffer[i * 2] = (int16_t)(input_samples[i] >> 14);  // Mic
      combined_buffer[i * 2 + 1] =
          0;  // Reference (initialize to silence for now)
    }

    // Log the actual data we're feeding
    ESP_LOGD("AEC", "Feeding %d total samples (%d per channel) to AFE",
             total_samples, samples_per_channel);

    // Feed stereo data to AFE
    int ret = afe_handle->feed(afe_data, combined_buffer);

    // Log detailed info about the feed operation
    if (ret != 0) {
      ESP_LOGW("AEC",
               "AFE feed failed: %d, Frame size: %d, Total samples: %d, Buffer "
               "size: %d bytes",
               ret, frame_size, total_samples, total_samples * sizeof(int16_t));
    }

    // Try to fetch processed data
    afe_fetch_result_t* result = nullptr;
    for (int retry = 0; retry < 2; retry++) {
      result = afe_handle->fetch(afe_data);
      if (result && result->data) break;
      vTaskDelay(pdMS_TO_TICKS(1));
    }

    // Clean up the combined buffer
    heap_caps_free(combined_buffer);

    // Handle the output
    if (!result || !result->data) {
      *processed_size = 0;
      return nullptr;
    }

    *processed_size = result->data_size / sizeof(int16_t);
    return (int16_t*)result->data;
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

  afe_fetch_result_t* getFetchResult() {
    if (!afe_handle || !afe_data) {
      return nullptr;
    }
    return afe_handle->fetch(afe_data);
  }

 private:
  esp_afe_sr_iface_t* afe_handle{nullptr};
  esp_afe_sr_data_t* afe_data{nullptr};
  int16_t* mic_buffer{nullptr};
  int16_t* ref_buffer{nullptr};
  int16_t* speaker_reference_buffer;
  SemaphoreHandle_t speaker_reference_mutex;
  int frame_size{0};
};