#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "aec_handler.h"
#include "driver/i2s.h"
#include "esp_log.h"
#include "i2s_config.h"

static const char* TAG = "AEC_TEST";

// Global AEC handler instance
static AECHandler aec;

// Buffers for audio processing
static int32_t* i2s_read_buffer = nullptr;
static int16_t* process_buffer = nullptr;

// Initialize I2S peripherals
static bool init_i2s() {
  ESP_LOGI(TAG, "Initializing I2S...");

  // Initialize I2S for microphone
  esp_err_t err = i2s_driver_install(I2S_PORT_MIC, &i2s_mic_config, 0, nullptr);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to install I2S mic driver: %d", err);
    return false;
  }

  err = i2s_set_pin(I2S_PORT_MIC, &i2s_mic_pins);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set I2S mic pins: %d", err);
    return false;
  }

  // Initialize I2S for speaker
  err = i2s_driver_install(I2S_PORT_SPEAKER, &i2s_speaker_config, 0, nullptr);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to install I2S speaker driver: %d", err);
    return false;
  }

  err = i2s_set_pin(I2S_PORT_SPEAKER, &i2s_speaker_pins);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set I2S speaker pins: %d", err);
    return false;
  }

  // Clear the speaker output buffer
  i2s_zero_dma_buffer(I2S_PORT_SPEAKER);

  ESP_LOGI(TAG, "I2S initialized successfully");
  return true;
}

// Initialize audio processing buffers
static bool init_buffers() {
  // Allocate buffers matching I2S frame size
  i2s_read_buffer = (int32_t*)heap_caps_malloc(
      I2S_FRAME_SIZE * sizeof(int32_t), MALLOC_CAP_8BIT | MALLOC_CAP_DMA);

  process_buffer = (int16_t*)heap_caps_malloc(
      I2S_FRAME_SIZE * sizeof(int16_t), MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);

  if (!i2s_read_buffer || !process_buffer) {
    ESP_LOGE(TAG, "Failed to allocate audio buffers");
    return false;
  }

  ESP_LOGI(TAG, "Audio buffers initialized");
  return true;
}

static void cleanup() {
  if (i2s_read_buffer) {
    heap_caps_free(i2s_read_buffer);
    i2s_read_buffer = nullptr;
  }
  if (process_buffer) {
    heap_caps_free(process_buffer);
    process_buffer = nullptr;
  }

  i2s_driver_uninstall(I2S_PORT_MIC);
  i2s_driver_uninstall(I2S_PORT_SPEAKER);
}

static void convert_samples(int32_t* input, int16_t* output, size_t samples) {
  for (size_t i = 0; i < samples; i++) {
    // Convert 32-bit to 16-bit with scaling
    output[i] = input[i] >> 14;
  }
}

// Audio processing task
static void audio_processing_task(void* arg) {
  size_t bytes_read = 0;
  size_t bytes_written = 0;
  size_t samples_processed = 0;
  const TickType_t xMaxBlockTime = pdMS_TO_TICKS(100);
  const size_t read_size = I2S_FRAME_SIZE * sizeof(int32_t);

  ESP_LOGI(TAG, "Audio processing started");

  while (true) {
    // Read from I2S mic
    esp_err_t err = i2s_read(I2S_PORT_MIC, i2s_read_buffer, read_size,
                             &bytes_read, xMaxBlockTime);

    if (err != ESP_OK) {
      ESP_LOGE(TAG, "I2S read failed: %d", err);
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    if (bytes_read > 0) {
      size_t samples = bytes_read / sizeof(int32_t);
      convert_samples(i2s_read_buffer, process_buffer, samples);

      int16_t* processed_audio =
          aec.processAudio(process_buffer, &samples_processed);

      if (processed_audio && samples_processed > 0) {
        err = i2s_write(I2S_PORT_SPEAKER, processed_audio,
                        samples_processed * sizeof(int16_t), &bytes_written,
                        xMaxBlockTime);

        if (err != ESP_OK) {
          ESP_LOGE(TAG, "I2S write failed: %d", err);
        } else {
          aec.updateReferenceBuffer(processed_audio, samples_processed);
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "Starting AEC test application");

  // Initialize I2S
  if (!init_i2s()) {
    ESP_LOGE(TAG, "Failed to initialize I2S");
    cleanup();
    return;
  }

  // Initialize audio buffers
  if (!init_buffers()) {
    ESP_LOGE(TAG, "Failed to initialize audio buffers");
    cleanup();
    return;
  }

  // Initialize AEC
  if (!aec.init()) {
    ESP_LOGE(TAG, "Failed to initialize AEC");
    cleanup();
    return;
  }

  // Create audio processing task with higher priority
  xTaskCreate(audio_processing_task, "audio_proc", 8192, nullptr,
              configMAX_PRIORITIES - 2,  // High priority
              nullptr);

  ESP_LOGI(TAG, "AEC test application started successfully");
}