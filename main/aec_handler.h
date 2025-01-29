#pragma once

#include <esp_afe_sr_iface.h>
#include <esp_afe_sr_models.h>
#include <esp_timer.h>

#include "esp_log.h"
#include "i2s_config.h"

/**
 * A simple AEC/NS pipeline using ESP_AFE_SR_HANDLE.
 * We feed 512 samples every ~32ms, fetch once, if no data => skip quickly.
 */
class SrAecHandler {
 public:
  SrAecHandler() : afe_handle(nullptr), afe_data(nullptr), frame_size(0) {}

  bool init() {
    afe_handle = const_cast<esp_afe_sr_iface_t*>(&ESP_AFE_SR_HANDLE);
    if (!afe_handle) {
      ESP_LOGE(TAG, "Failed to get SR handle!");
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
    cfg.afe_linear_gain = 0.3;

    afe_data = afe_handle->create_from_config(&cfg);
    if (!afe_data) {
      ESP_LOGE(TAG, "Failed to create AFE instance");
      return false;
    }

    frame_size = afe_handle->get_feed_chunksize(afe_data);
    ESP_LOGI(TAG, "AEC frame_size=%d (samples per channel)", (int)frame_size);
    return true;
  }

  size_t getFrameSize() const { return frame_size; }

  int feedInterleaved(const int16_t* interleaved) {
    if (!afe_data || !interleaved) return -1;
    return afe_handle->feed(afe_data, interleaved);
  }

  // Non-blocking single fetch attempt
  afe_fetch_result_t* fetchOnce() {
    if (!afe_data) return nullptr;
    return afe_handle->fetch(afe_data);
  }

  void deinit() {
    if (afe_data) {
      afe_handle->destroy(afe_data);
      afe_data = nullptr;
    }
    afe_handle = nullptr;
  }

 private:
  static constexpr const char* TAG = "SR_AEC";
  esp_afe_sr_iface_t* afe_handle;
  esp_afe_sr_data_t* afe_data;
  size_t frame_size;
};