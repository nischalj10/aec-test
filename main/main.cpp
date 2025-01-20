#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>

#include "aec_handler.h"  // Our SrAecHandler
#include "driver/i2s.h"
#include "esp_log.h"
#include "i2s_config.h"

static const char* TAG = "AEC_SR_DEMO";
static SrAecHandler sr_aec;

// Buffers
static int32_t* mic_buffer_32 = nullptr;  // read from mic (32-bit)
static int16_t* ref_buffer_16 = nullptr;  // software reference
static size_t frame_size_16 = 0;
static size_t mic_bytes_per_frame = 0;

static bool init_i2s(size_t mic_dma_len, size_t spk_dma_len);
static bool init_buffers(void);
static void cleanup(void);
static void audio_task(void* arg);

/*
   For amplitude limiting so we don't blow up the signal in an endless loop.
   You can tweak this to e.g. ±8000 or ±15000.
*/
static inline int16_t clampAmplitude(int32_t val) {
  const int32_t LIMIT = 10000;  // adjust as needed
  if (val > LIMIT) return LIMIT;
  if (val < -LIMIT) return -LIMIT;
  return (int16_t)val;
}

extern "C" void app_main(void) {
  ESP_LOGI(TAG,
           "Starting single-mic SR-handle AEC+NS with amplitude limiting...");

  // 1) Initialize AEC handler (SR pipeline)
  if (!sr_aec.init()) {
    ESP_LOGE(TAG, "Failed to init sr_aec");
    return;
  }

  frame_size_16 = sr_aec.getFrameSize();  // e.g. 256 or 512
  mic_bytes_per_frame = frame_size_16 * sizeof(int32_t);

  ESP_LOGI(TAG, "frame_size_16=%d => mic_bytes_per_frame=%d",
           (int)frame_size_16, (int)mic_bytes_per_frame);

  // 2) Set I2S buffer sizes
  i2s_mic_config.dma_buf_len = frame_size_16;
  i2s_speaker_config.dma_buf_len = frame_size_16;

  // 3) Initialize I2S
  if (!init_i2s(i2s_mic_config.dma_buf_len, i2s_speaker_config.dma_buf_len)) {
    ESP_LOGE(TAG, "I2S init failed");
    sr_aec.deinit();
    return;
  }

  // 4) Buffers
  if (!init_buffers()) {
    ESP_LOGE(TAG, "Init buffers failed");
    cleanup();
    return;
  }

  // 5) Launch audio task with bigger stack
  xTaskCreate(audio_task, "audio_task", 8192, NULL, configMAX_PRIORITIES - 2,
              NULL);

  ESP_LOGI(TAG, "AEC + NS loopback started");
}

static bool init_i2s(size_t mic_dma_len, size_t spk_dma_len) {
  ESP_LOGI(TAG, "Installing I2S drivers (mic_dma_len=%d, spk_dma_len=%d)",
           (int)mic_dma_len, (int)spk_dma_len);

  esp_err_t err = i2s_driver_install(I2S_PORT_MIC, &i2s_mic_config, 0, NULL);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Mic i2s_driver_install fail: %d", err);
    return false;
  }
  err = i2s_set_pin(I2S_PORT_MIC, &i2s_mic_pins);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Mic i2s_set_pin fail: %d", err);
    return false;
  }

  err = i2s_driver_install(I2S_PORT_SPEAKER, &i2s_speaker_config, 0, NULL);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Spk i2s_driver_install fail: %d", err);
    return false;
  }
  err = i2s_set_pin(I2S_PORT_SPEAKER, &i2s_speaker_pins);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Spk i2s_set_pin fail: %d", err);
    return false;
  }

  i2s_zero_dma_buffer(I2S_PORT_SPEAKER);

  ESP_LOGI(TAG, "I2S init done.");
  return true;
}

static bool init_buffers(void) {
  mic_buffer_32 = (int32_t*)heap_caps_malloc(mic_bytes_per_frame,
                                             MALLOC_CAP_8BIT | MALLOC_CAP_DMA);
  if (!mic_buffer_32) {
    ESP_LOGE(TAG, "OOM mic_buffer_32");
    return false;
  }

  ref_buffer_16 = (int16_t*)heap_caps_calloc(frame_size_16, sizeof(int16_t),
                                             MALLOC_CAP_INTERNAL);
  if (!ref_buffer_16) {
    ESP_LOGE(TAG, "OOM ref_buffer_16");
    return false;
  }

  ESP_LOGI(TAG, "Buffers allocated. mic=%d bytes, ref=%d samples",
           (int)mic_bytes_per_frame, (int)frame_size_16);
  return true;
}

static void cleanup(void) {
  if (mic_buffer_32) {
    heap_caps_free(mic_buffer_32);
    mic_buffer_32 = nullptr;
  }
  if (ref_buffer_16) {
    heap_caps_free(ref_buffer_16);
    ref_buffer_16 = nullptr;
  }
  i2s_driver_uninstall(I2S_PORT_MIC);
  i2s_driver_uninstall(I2S_PORT_SPEAKER);
  sr_aec.deinit();
}

static void audio_task(void* arg) {
  while (true) {
    // 1) Read from mic
    size_t bytes_read = 0;
    esp_err_t err = i2s_read(I2S_PORT_MIC, mic_buffer_32, mic_bytes_per_frame,
                             &bytes_read, pdMS_TO_TICKS(100));
    if (err != ESP_OK || bytes_read < mic_bytes_per_frame) {
      ESP_LOGW(TAG, "Mic read underflow: got %d/%d bytes (err=%d)",
               (int)bytes_read, (int)mic_bytes_per_frame, (int)err);
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    // 2) Process with SR-handle AEC+NS
    size_t out_len_16 = 0;
    int16_t* processed =
        sr_aec.processAudio(mic_buffer_32, ref_buffer_16, &out_len_16);
    if (!processed || out_len_16 == 0) {
      ESP_LOGW(TAG, "No AEC output yet");
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }

    // 3) Write to speaker in stereo, with amplitude clamp
    size_t stereo_samples = out_len_16 * 2;
    size_t stereo_bytes = stereo_samples * sizeof(int16_t);

    int16_t* stereo_buf =
        (int16_t*)heap_caps_malloc(stereo_bytes, MALLOC_CAP_INTERNAL);
    if (!stereo_buf) {
      ESP_LOGE(TAG, "OOM for stereo_buf");
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    // Limit amplitude & duplicate left/right
    for (size_t i = 0; i < out_len_16; i++) {
      int16_t sample_clamped = clampAmplitude(processed[i]);
      stereo_buf[2 * i + 0] = sample_clamped;  // L
      stereo_buf[2 * i + 1] = sample_clamped;  // R
    }

    size_t bytes_written = 0;
    i2s_write(I2S_PORT_SPEAKER, stereo_buf, stereo_bytes, &bytes_written,
              pdMS_TO_TICKS(100));
    heap_caps_free(stereo_buf);

    // 4) Copy the same processed data to ref_buffer for next iteration
    size_t samples_written = bytes_written / sizeof(int16_t);
    if (samples_written >= out_len_16) {
      memcpy(ref_buffer_16, processed, out_len_16 * sizeof(int16_t));
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}