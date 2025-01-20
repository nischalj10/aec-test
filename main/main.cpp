#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>

#include "aec_handler.h"
#include "driver/i2s.h"
#include "esp_log.h"
#include "i2s_config.h"

static const char* TAG = "AEC_APP";

static AECHandler aec;

// Dynamically sized buffers
static int32_t* mic_buffer_32 = NULL;  // for reading from the mic
static int16_t* ref_buffer_16 = NULL;  // for feeding the next AEC ref
static int16_t* aec_output_last =
    NULL;  // store last AEC output => "software reference"

static size_t g_frame_size = 0;  // 16-bit samples per channel
static size_t g_bytes_per_mic_frame =
    0;  // # of bytes to read from mic each iteration

static bool init_i2s(size_t mic_dma_len, size_t speaker_dma_len);
static bool init_buffers(void);
static void cleanup(void);
static void audio_task(void* arg);

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "Starting software-based AEC demo...");

  // 1) Initialize AEC
  if (!aec.init()) {
    ESP_LOGE(TAG, "AEC init failed");
    return;
  }

  // Query how many 16-bit samples per channel the AFE expects
  g_frame_size = aec.getFrameSize();
  // Example: 512 if your logs say "AFE frame_size per channel = 512"

  // For the microphone, we must read "g_frame_size * 32-bit" samples => that's
  // g_frame_size * 4 bytes
  g_bytes_per_mic_frame = g_frame_size * sizeof(int32_t);

  // 2) Adjust our I2S DMA buffer lengths to match the AFE chunk size
  //    For the mic: we want .dma_buf_len = g_frame_size so each read() yields
  //    exactly one chunk
  i2s_mic_config.dma_buf_len = g_frame_size;
  // For the speaker: we typically also match .dma_buf_len = g_frame_size for
  // consistent timing
  i2s_speaker_config.dma_buf_len = g_frame_size;

  // 3) Now initialize I2S with those updated configs
  if (!init_i2s(i2s_mic_config.dma_buf_len, i2s_speaker_config.dma_buf_len)) {
    ESP_LOGE(TAG, "I2S init failed");
    cleanup();
    return;
  }

  // 4) Allocate buffers
  if (!init_buffers()) {
    ESP_LOGE(TAG, "Buffer init failed");
    cleanup();
    return;
  }

  // 5) Start a task that reads mic, calls AEC, writes to speaker
  xTaskCreate(audio_task, "audio_task", 4096, NULL, configMAX_PRIORITIES - 2,
              NULL);
  ESP_LOGI(TAG, "AEC app started successfully");
}

// ----------------------------------------------------------------------------
// I2S Setup
// ----------------------------------------------------------------------------
static bool init_i2s(size_t mic_dma_len, size_t speaker_dma_len) {
  ESP_LOGI(TAG, "Installing I2S drivers...");

  // ---------- Microphone ----------
  esp_err_t err = i2s_driver_install(I2S_PORT_MIC, &i2s_mic_config, 0, NULL);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "MIC driver install failed: %d", err);
    return false;
  }
  err = i2s_set_pin(I2S_PORT_MIC, &i2s_mic_pins);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "MIC set pin failed: %d", err);
    return false;
  }

  // ---------- Speaker ----------
  err = i2s_driver_install(I2S_PORT_SPEAKER, &i2s_speaker_config, 0, NULL);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "SPK driver install failed: %d", err);
    return false;
  }
  err = i2s_set_pin(I2S_PORT_SPEAKER, &i2s_speaker_pins);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "SPK set pin failed: %d", err);
    return false;
  }

  // Clear speaker DMA
  i2s_zero_dma_buffer(I2S_PORT_SPEAKER);

  ESP_LOGI(TAG, "I2S init done. MIC dma_buf_len=%d, SPK dma_buf_len=%d",
           (int)mic_dma_len, (int)speaker_dma_len);
  return true;
}

// ----------------------------------------------------------------------------
// Buffers
// ----------------------------------------------------------------------------
static bool init_buffers(void) {
  // We'll read "frame_size" 32-bit samples each iteration =>
  // g_bytes_per_mic_frame
  mic_buffer_32 = (int32_t*)heap_caps_malloc(g_bytes_per_mic_frame,
                                             MALLOC_CAP_8BIT | MALLOC_CAP_DMA);
  if (!mic_buffer_32) {
    ESP_LOGE(TAG, "OOM for mic_buffer_32");
    return false;
  }

  // We'll feed AEC a 16-bit reference of length g_frame_size
  ref_buffer_16 = (int16_t*)heap_caps_calloc(g_frame_size, sizeof(int16_t),
                                             MALLOC_CAP_INTERNAL);
  if (!ref_buffer_16) {
    ESP_LOGE(TAG, "OOM for ref_buffer_16");
    return false;
  }

  // We'll keep track of last AEC output for the next reference
  aec_output_last = (int16_t*)heap_caps_calloc(g_frame_size, sizeof(int16_t),
                                               MALLOC_CAP_INTERNAL);
  if (!aec_output_last) {
    ESP_LOGE(TAG, "OOM for aec_output_last");
    return false;
  }

  ESP_LOGI(TAG,
           "Allocated mic_buffer_32(%d bytes), ref_buffer_16(%d samples), "
           "aec_output_last(%d samples)",
           (int)g_bytes_per_mic_frame, (int)g_frame_size, (int)g_frame_size);
  return true;
}

// ----------------------------------------------------------------------------
// Cleanup
// ----------------------------------------------------------------------------
static void cleanup(void) {
  if (mic_buffer_32) {
    heap_caps_free(mic_buffer_32);
    mic_buffer_32 = NULL;
  }
  if (ref_buffer_16) {
    heap_caps_free(ref_buffer_16);
    ref_buffer_16 = NULL;
  }
  if (aec_output_last) {
    heap_caps_free(aec_output_last);
    aec_output_last = NULL;
  }
  i2s_driver_uninstall(I2S_PORT_MIC);
  i2s_driver_uninstall(I2S_PORT_SPEAKER);
  aec.deinit();
}

// ----------------------------------------------------------------------------
// Main Audio Task
// ----------------------------------------------------------------------------
static void audio_task(void* arg) {
  while (true) {
    // 1) Read exactly one AFE chunk from the mic => g_frame_size 32-bit samples
    size_t bytes_read = 0;
    esp_err_t err = i2s_read(I2S_PORT_MIC, mic_buffer_32, g_bytes_per_mic_frame,
                             &bytes_read, pdMS_TO_TICKS(100));
    if (err != ESP_OK || bytes_read < g_bytes_per_mic_frame) {
      ESP_LOGW(TAG, "Mic read underflow: got %d/%d bytes (err=%d)",
               (int)bytes_read, (int)g_bytes_per_mic_frame, (int)err);
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    // 2) Let AEC process it: mic + "ref_buffer_16" => AEC output
    size_t out_size_16 = 0;
    int16_t* aec_out =
        aec.processAudio(mic_buffer_32, ref_buffer_16, &out_size_16);

    if (!aec_out || out_size_16 == 0) {
      // It's normal for the first few frames to produce no output (pipeline
      // delay).
      ESP_LOGW(TAG, "No AEC output yet");
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }

    // 3) Write the AEC output to the speaker in stereo
    //    The AEC is single-channel, so replicate L/R
    size_t stereo_samples = out_size_16 * 2;  // left + right
    size_t stereo_bytes = stereo_samples * sizeof(int16_t);

    int16_t* stereo_buf =
        (int16_t*)heap_caps_malloc(stereo_bytes, MALLOC_CAP_INTERNAL);
    if (!stereo_buf) {
      ESP_LOGE(TAG, "OOM for stereo_buf");
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    for (size_t i = 0; i < out_size_16; i++) {
      stereo_buf[2 * i + 0] = aec_out[i];  // left
      stereo_buf[2 * i + 1] = aec_out[i];  // right
    }

    size_t bytes_written = 0;
    i2s_write(I2S_PORT_SPEAKER, stereo_buf, stereo_bytes, &bytes_written,
              pdMS_TO_TICKS(100));
    heap_caps_free(stereo_buf);

    // 4) Store the just-played data into ref_buffer_16 for *next* iteration
    //    The “ideal” reference is the raw speaker data. This is just a demo
    //    approach.
    size_t samples_written = bytes_written / sizeof(int16_t);
    if (samples_written >= out_size_16) {
      memcpy(ref_buffer_16, aec_out, out_size_16 * sizeof(int16_t));
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}