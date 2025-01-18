#include <driver/i2s.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "aec_handler.h"

static const char* TAG = "AEC_TEST";

// Task handles
static TaskHandle_t mic_task_handle = nullptr;
static TaskHandle_t speaker_task_handle = nullptr;

// Global AEC handler instance
static AECHandler aec;

// I2S buffers
static int32_t* i2s_read_buffer = nullptr;
static int16_t* i2s_write_buffer = nullptr;

// Callback for processed audio
static void processedAudioCallback(const int16_t* data, size_t samples,
                                   void* context) {
  if (!data || !samples) return;

  size_t bytes_written = 0;
  esp_err_t err = i2s_write(I2S_PORT_SPEAKER, data, samples * sizeof(int16_t),
                            &bytes_written, pdMS_TO_TICKS(100));

  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to write to speaker: %d", err);
  } else {
    // Keep track of written data for reference buffer
    if (bytes_written > 0) {
      aec.writeRefData((int16_t*)data, bytes_written / sizeof(int16_t));
    }
  }
}

// Microphone reading task
static void microphoneTask(void* arg) {
  const size_t read_size = SAMPLES_PER_FRAME * sizeof(int32_t);
  size_t bytes_read = 0;

  while (true) {
    // Read from I2S mic
    esp_err_t err = i2s_read(I2S_PORT_MIC, i2s_read_buffer, read_size,
                             &bytes_read, pdMS_TO_TICKS(100));

    if (err == ESP_OK && bytes_read > 0) {
      size_t samples = bytes_read / sizeof(int32_t);
      if (!aec.writeMicData(i2s_read_buffer, samples)) {
        ESP_LOGW(TAG, "Failed to write mic data to AEC");
        vTaskDelay(pdMS_TO_TICKS(1));
      }
    } else if (err != ESP_OK) {
      ESP_LOGW(TAG, "Failed to read from mic: %d", err);
      vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Small delay to prevent starving other tasks
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// Initialize I2S peripherals
static bool initI2S() {
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

// Initialize audio buffers
static bool initBuffers() {
  // Allocate I2S buffers with DMA capabilities
  i2s_read_buffer = (int32_t*)heap_caps_malloc(
      SAMPLES_PER_FRAME * sizeof(int32_t), MALLOC_CAP_DMA);

  i2s_write_buffer = (int16_t*)heap_caps_malloc(
      SAMPLES_PER_FRAME * sizeof(int16_t), MALLOC_CAP_DMA);

  if (!i2s_read_buffer || !i2s_write_buffer) {
    ESP_LOGE(TAG, "Failed to allocate I2S buffers");
    return false;
  }

  ESP_LOGI(TAG, "Audio buffers initialized with size: %d samples",
           SAMPLES_PER_FRAME);
  return true;
}

// Cleanup resources
static void cleanup() {
  // Stop AEC processing
  aec.stop();

  // Delete tasks
  if (mic_task_handle) {
    vTaskDelete(mic_task_handle);
    mic_task_handle = nullptr;
  }

  // Free buffers
  if (i2s_read_buffer) {
    heap_caps_free(i2s_read_buffer);
    i2s_read_buffer = nullptr;
  }

  if (i2s_write_buffer) {
    heap_caps_free(i2s_write_buffer);
    i2s_write_buffer = nullptr;
  }

  // Uninstall I2S drivers
  i2s_driver_uninstall(I2S_PORT_MIC);
  i2s_driver_uninstall(I2S_PORT_SPEAKER);
}

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "Starting AEC test application");

  // Initialize I2S
  if (!initI2S()) {
    ESP_LOGE(TAG, "Failed to initialize I2S");
    cleanup();
    return;
  }

  // Initialize audio buffers
  if (!initBuffers()) {
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

  // Set callback for processed audio
  aec.setProcessedAudioCallback(processedAudioCallback, nullptr);

  // Start AEC processing
  if (!aec.start()) {
    ESP_LOGE(TAG, "Failed to start AEC");
    cleanup();
    return;
  }

  // Create microphone task
  BaseType_t ret = xTaskCreatePinnedToCore(
      microphoneTask, "mic_task", 4096, nullptr,
      configMAX_PRIORITIES - 2,  // High priority but below AEC task
      &mic_task_handle,
      0  // Run on core 0
  );

  if (ret != pdPASS) {
    ESP_LOGE(TAG, "Failed to create microphone task");
    cleanup();
    return;
  }

  ESP_LOGI(TAG, "AEC test application started successfully");
}