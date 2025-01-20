#pragma once

#include <esp_afe_sr_iface.h>
#include <esp_afe_sr_models.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <string.h>

#include "esp_log.h"
#include "i2s_config.h"

/**
 * AECHandler:
 *   - Grabs the esp_afe_sr_iface_t for single-mic + AEC
 *   - Configures total_ch_num=2 => (mic + ref)
 *   - Only AEC (no VAD/wakenet)
 *   - We can query getFrameSize() to know how many 16-bit samples per channel
 */
class AECHandler {
 public:
  AECHandler() : afe_handle(nullptr), afe_data(nullptr), frame_size(0) {}

  bool init() {
    afe_handle = const_cast<esp_afe_sr_iface_t*>(&ESP_AFE_SR_HANDLE);
    if (!afe_handle) {
      ESP_LOGE("AEC", "Failed to get AFE handle!");
      return false;
    }

    // Prepare config
    afe_config_t cfg = AFE_CONFIG_DEFAULT();
    cfg.pcm_config.sample_rate = SAMPLE_RATE;
    cfg.pcm_config.total_ch_num = 2;  // mic(1) + ref(1)
    cfg.pcm_config.mic_num = 1;
    cfg.pcm_config.ref_num = 1;

    // Only enable AEC
    cfg.aec_init = true;
    cfg.se_init = true;
    cfg.vad_init = false;
    cfg.wakenet_init = false;
    cfg.voice_communication_init = false;
    cfg.afe_mode = SR_MODE_HIGH_PERF;

    // Create AFE
    afe_data = afe_handle->create_from_config(&cfg);
    if (!afe_data) {
      ESP_LOGE("AEC", "Failed to create AFE instance");
      return false;
    }

    // AFE returns chunk size in 16-bit samples *per channel*
    frame_size = afe_handle->get_feed_chunksize(afe_data);
    ESP_LOGI("AEC", "AFE frame_size per channel = %d (16-bit samples)",
             frame_size);

    return true;
  }

  size_t getFrameSize() const {
    return frame_size;  // # of 16-bit samples per channel
  }

  /**
   * Feed mic+ref to AFE, fetch processed data.
   *
   * @param mic_data   pointer to (frame_size) 32-bit mic samples
   * @param ref_data   pointer to (frame_size) 16-bit reference samples
   * @param out_size   (out) number of 16-bit samples returned
   * @return           pointer to 16-bit processed data (owned by AFE)
   */
  int16_t* processAudio(const int32_t* mic_data, const int16_t* ref_data,
                        size_t* out_size) {
    if (!afe_data || !mic_data || !ref_data) {
      *out_size = 0;
      return nullptr;
    }

    // We feed total_ch_num=2 => 2*frame_size 16-bit samples
    const size_t total_samples_16 = frame_size * 2;

    // Build interleaved buffer: [mic(16-bit), ref(16-bit)] x frame_size
    int16_t* interleaved = (int16_t*)heap_caps_malloc(
        total_samples_16 * sizeof(int16_t), MALLOC_CAP_INTERNAL);
    if (!interleaved) {
      ESP_LOGE("AEC", "OOM for interleaved");
      *out_size = 0;
      return nullptr;
    }

    for (size_t i = 0; i < frame_size; i++) {
      // Convert 32-bit mic to 16-bit
      interleaved[2 * i + 0] = (int16_t)(mic_data[i] >> 14);
      // Ref is already 16-bit
      interleaved[2 * i + 1] = ref_data[i];
    }

    // Feed
    int ret = afe_handle->feed(afe_data, interleaved);
    heap_caps_free(interleaved);

    if (ret < 0) {
      ESP_LOGE("AEC", "afe->feed() error: %d", ret);
    } else if ((size_t)ret != total_samples_16) {
      ESP_LOGW("AEC", "feed() consumed %d/%d samples", ret,
               (int)total_samples_16);
    }

    // Fetch
    afe_fetch_result_t* result = nullptr;
    for (int tries = 0; tries < 4; tries++) {
      result = afe_handle->fetch(afe_data);
      if (result && result->data) break;
      vTaskDelay(pdMS_TO_TICKS(1));
    }

    if (!result || !result->data) {
      // AFE might not output every feed() call if pipeline not ready.
      *out_size = 0;
      return nullptr;
    }

    // result->data is 16-bit, result->data_size is in bytes
    *out_size = result->data_size / sizeof(int16_t);
    return (int16_t*)result->data;
  }

  void deinit() {
    if (afe_handle && afe_data) {
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