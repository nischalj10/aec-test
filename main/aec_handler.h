#pragma once

#include <esp_afe_sr_iface.h>
#include <esp_afe_sr_models.h>
#include <esp_timer.h>

#include "esp_log.h"
#include "i2s_config.h"

/**
 * SrAecHandler:
 *   - Uses ESP_AFE_SR_HANDLE (speech recognition pipeline),
 *   - Single mic + reference => AEC,
 *   - Also enable `se_init = true` for noise suppression (NS),
 *   - No VAD or wake word => so the pipeline always returns audio.
 */
class SrAecHandler {
 public:
  SrAecHandler() : afe_handle(nullptr), afe_data(nullptr), frame_size(0) {}

  bool init() {
    // Use SR handle
    afe_handle = const_cast<esp_afe_sr_iface_t*>(&ESP_AFE_SR_HANDLE);
    if (!afe_handle) {
      ESP_LOGE("SR_AEC", "Failed to get SR handle!");
      return false;
    }

    // Default config
    afe_config_t cfg = AFE_CONFIG_DEFAULT();
    // We feed 1 mic + 1 ref channel = 2 total
    cfg.pcm_config.sample_rate = SAMPLE_RATE;
    cfg.pcm_config.total_ch_num = 2;
    cfg.pcm_config.mic_num = 1;
    cfg.pcm_config.ref_num = 1;

    // Only AEC + NS => remove mild feedback
    cfg.aec_init = true;   // acoustic echo cancellation
    cfg.se_init = true;    // single-channel noise suppression
    cfg.vad_init = false;  // no voice gating
    cfg.wakenet_init = false;
    // We do not use voice_communication_init here
    cfg.voice_communication_init = false;
    cfg.afe_mode = SR_MODE_HIGH_PERF;
    cfg.afe_linear_gain = 0.25;

    // Create from config
    afe_data = afe_handle->create_from_config(&cfg);
    if (!afe_data) {
      ESP_LOGE("SR_AEC", "Failed to create AFE instance");
      return false;
    }

    // Each channel: "feed_chunksize" 16-bit samples
    frame_size = afe_handle->get_feed_chunksize(afe_data);
    ESP_LOGI("SR_AEC", "SR AFE frame_size per channel = %d", (int)frame_size);

    return true;
  }

  size_t getFrameSize() const { return frame_size; }

  /**
   * processAudio:
   *   - mic_32: pointer to frame_size 32-bit mic samples
   *   - ref_16: pointer to frame_size 16-bit reference data
   *   - out_size: number of 16-bit samples returned
   */
  int16_t* processAudio(const int32_t* mic_32, const int16_t* ref_16,
                        size_t* out_size) {
    *out_size = 0;
    if (!afe_data || !mic_32 || !ref_16) {
      return nullptr;
    }

    const size_t total_16 = frame_size * 2;  // mic + ref interleaved
    int16_t* interleaved = (int16_t*)heap_caps_malloc(
        total_16 * sizeof(int16_t), MALLOC_CAP_INTERNAL);
    if (!interleaved) {
      ESP_LOGE("SR_AEC", "OOM interleaved");
      return nullptr;
    }

    // Build interleaved buffer
    for (size_t i = 0; i < frame_size; i++) {
      // Convert 32-bit mic => 16-bit
      interleaved[2 * i + 0] = (int16_t)(mic_32[i] >> 14);
      // Reference is already 16-bit
      interleaved[2 * i + 1] = ref_16[i];
    }

    // Feed
    int fed = afe_handle->feed(afe_data, interleaved);
    heap_caps_free(interleaved);

    if (fed < 0) {
      ESP_LOGE("SR_AEC", "feed() error: %d", fed);
      return nullptr;
    }

    // Fetch
    afe_fetch_result_t* result = nullptr;
    for (int tries = 0; tries < 3; tries++) {
      result = afe_handle->fetch(afe_data);
      if (result && result->data) break;
      vTaskDelay(pdMS_TO_TICKS(1));
    }

    if (!result || !result->data) {
      return nullptr;  // No output yet
    }

    *out_size = result->data_size / sizeof(int16_t);
    return (int16_t*)result->data;
  }

  void deinit() {
    if (afe_data) {
      afe_handle->destroy(afe_data);
      afe_data = nullptr;
    }
    afe_handle = nullptr;
  }

 private:
  esp_afe_sr_iface_t* afe_handle;
  esp_afe_sr_data_t* afe_data;
  size_t frame_size;
};