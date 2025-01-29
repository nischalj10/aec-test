#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <opus.h>
#include <string.h>

#include "AudioBufferManager.h"
#include "ReferenceBufferManager.h"
#include "aec_handler.h"
#include "driver/i2s.h"
#include "esp_log.h"
#include "i2s_config.h"

static const char* TAG = "MAIN";

// Components
static ReferenceBufferManager* reference_buffer_manager = nullptr;
static AudioBufferManager* audio_buffer_manager = nullptr;

// Opus codec
static OpusEncoder* opus_encoder = nullptr;
static OpusDecoder* opus_decoder = nullptr;
static opus_int16* encoder_input_buffer = nullptr;
static uint8_t* encoder_output_buffer = nullptr;
static opus_int16* output_buffer = nullptr;

#define OPUS_OUT_BUFFER_SIZE 1276
#define OPUS_ENCODER_BITRATE 30000
#define OPUS_ENCODER_COMPLEXITY 0
#define MIC_CHANNELS 1
#define SPEAKER_CHANNELS 1
#define MAX_AEC_FRAME 512
#define OPUS_20MS_FRAME 320
#define FETCH_TASK_PRIORITY configMAX_PRIORITIES - 2
#define FEED_TASK_PRIORITY configMAX_PRIORITIES - 3

// Task handles
static TaskHandle_t micFeedTaskHandle = nullptr;
static TaskHandle_t aecFetchTaskHandle = nullptr;

static volatile bool feedTaskShouldStop = false;
static volatile bool fetchTaskShouldStop = false;

static SrAecHandler sr_aec;
static bool sr_aec_inited = false;

static bool init_opus() {
  // Initialize Opus encoder
  ESP_LOGI(TAG, "Initializing OPUS encoder...");
  int err = 0;
  opus_encoder = opus_encoder_create(SAMPLE_RATE, MIC_CHANNELS,
                                     OPUS_APPLICATION_VOIP, &err);
  if (err != OPUS_OK) {
    ESP_LOGE(TAG, "OpusEncoder create failed: %d", err);
    return false;
  }

  opus_encoder_ctl(opus_encoder, OPUS_SET_BITRATE(OPUS_ENCODER_BITRATE));
  opus_encoder_ctl(opus_encoder, OPUS_SET_COMPLEXITY(OPUS_ENCODER_COMPLEXITY));
  opus_encoder_ctl(opus_encoder, OPUS_SET_FORCE_CHANNELS(MIC_CHANNELS));

  encoder_input_buffer = (opus_int16*)heap_caps_malloc(
      OPUS_20MS_FRAME * sizeof(opus_int16),  // Only need 320 samples now
      MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
  encoder_output_buffer = (uint8_t*)heap_caps_malloc(
      OPUS_OUT_BUFFER_SIZE, MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);

  if (!encoder_input_buffer || !encoder_output_buffer) {
    ESP_LOGE(TAG, "Failed to allocate encoder buffers");
    return false;
  }

  ESP_LOGI(TAG, "OPUS encoder initialized successfully");

  // Initialize Opus decoder for speaker
  ESP_LOGI(TAG, "Initializing OPUS decoder...");
  int dec_err = 0;
  opus_decoder = opus_decoder_create(SAMPLE_RATE, SPEAKER_CHANNELS, &dec_err);
  if (dec_err != OPUS_OK) {
    ESP_LOGE(TAG, "OpusDecoder create failed: %d", dec_err);
    return false;
  }

  // Allocate output buffer in internal memory
  output_buffer = (opus_int16*)heap_caps_malloc(
      BUFFER_SAMPLES * SPEAKER_CHANNELS * sizeof(opus_int16),
      MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL  // Use internal memory for DMA
  );

  if (!output_buffer) {
    ESP_LOGE(TAG, "Failed to allocate output buffer");
    opus_decoder_destroy(opus_decoder);
    opus_decoder = nullptr;
    return false;
  }

  ESP_LOGI(TAG, "OPUS decoder initialized successfully");

  return true;
}

static bool init_i2s() {
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
  i2s_zero_dma_buffer(I2S_PORT_MIC);
  i2s_start(I2S_PORT_MIC);

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
  i2s_start(I2S_PORT_SPEAKER);
  return true;
}

static bool init_buffers() {
  audio_buffer_manager = new AudioBufferManager();
  if (!audio_buffer_manager) {
    ESP_LOGE(TAG, "Failed to create audio buffer");
    return false;
  }

  reference_buffer_manager = new ReferenceBufferManager();
  if (!reference_buffer_manager) {
    ESP_LOGE(TAG, "Failed to create reference buffer");
    return false;
  }

  return true;
}

static void cleanup() {
  feedTaskShouldStop = true;
  fetchTaskShouldStop = true;
  vTaskDelay(pdMS_TO_TICKS(100));

  if (micFeedTaskHandle) {
    vTaskDelete(micFeedTaskHandle);
    micFeedTaskHandle = nullptr;
  }
  if (aecFetchTaskHandle) {
    vTaskDelete(aecFetchTaskHandle);
    aecFetchTaskHandle = nullptr;
  }

  if (encoder_input_buffer) {
    heap_caps_free(encoder_input_buffer);
    encoder_input_buffer = nullptr;
  }
  if (encoder_output_buffer) {
    heap_caps_free(encoder_output_buffer);
    encoder_output_buffer = nullptr;
  }

  if (output_buffer) {
    free(output_buffer);
    output_buffer = nullptr;
  }

  // Cleanup Opus
  if (opus_encoder) {
    opus_encoder_destroy(opus_encoder);
    opus_encoder = nullptr;
  }
  if (opus_decoder) {
    opus_decoder_destroy(opus_decoder);
    opus_decoder = nullptr;
  }

  delete audio_buffer_manager;
  audio_buffer_manager = nullptr;
  delete reference_buffer_manager;
  reference_buffer_manager = nullptr;

  i2s_driver_uninstall(I2S_PORT_MIC);
  i2s_driver_uninstall(I2S_PORT_SPEAKER);
  sr_aec.deinit();
}

static void micFeedTask(void* arg) {
  ESP_LOGI(TAG, "Starting mic feed task on core %d", xPortGetCoreID());

  TickType_t lastWakeTime = xTaskGetTickCount();
  const TickType_t interval = pdMS_TO_TICKS(30);  // 32ms for 512 samples

  // Allocate buffers
  int32_t* mic_buf32 = (int32_t*)heap_caps_malloc(
      MAX_AEC_FRAME * sizeof(int32_t), MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
  int16_t* mic16 = (int16_t*)heap_caps_malloc(
      MAX_AEC_FRAME * sizeof(int16_t), MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
  int16_t* ref16 = (int16_t*)heap_caps_malloc(
      MAX_AEC_FRAME * sizeof(int16_t), MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
  int16_t* interleaved =
      (int16_t*)heap_caps_malloc(2 * MAX_AEC_FRAME * sizeof(int16_t),
                                 MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);

  if (!mic_buf32 || !mic16 || !ref16 || !interleaved) {
    ESP_LOGE(TAG, "Failed to allocate mic buffers");
    goto cleanup;
  }

  while (!feedTaskShouldStop) {
    // Read from I2S mic
    size_t bytes_read = 0;
    esp_err_t err =
        i2s_read(I2S_PORT_MIC, mic_buf32, MAX_AEC_FRAME * sizeof(int32_t),
                 &bytes_read, portMAX_DELAY);

    if (err != ESP_OK) {
      ESP_LOGW(TAG, "I2S read failed: %d", err);
      vTaskDelay(interval);
      continue;
    }

    // Convert mic samples (32->16 bit)
    for (int i = 0; i < MAX_AEC_FRAME; i++) {
      int32_t s = (mic_buf32[i] >> 14);
      mic16[i] = (int16_t)((s > 32767) ? 32767 : (s < -32768) ? -32768 : s);
    }

    // Get reference samples from the manager
    if (reference_buffer_manager &&
        reference_buffer_manager->getReference(ref16)) {
      // Interleave mic and reference
      for (int i = 0; i < MAX_AEC_FRAME; i++) {
        interleaved[2 * i] = mic16[i];      // Mic sample
        interleaved[2 * i + 1] = ref16[i];  // Reference sample
      }
    } else {
      ESP_LOGW(TAG, "Failed to get reference samples, using zeros");
      // Zero out reference channel
      for (int i = 0; i < MAX_AEC_FRAME; i++) {
        interleaved[2 * i] = mic16[i];  // Mic sample
        interleaved[2 * i + 1] = 0;     // Zero reference
      }
    }

    // Feed to AEC
    if (sr_aec_inited) {
      int ret = sr_aec.feedInterleaved(interleaved);
      if (ret < 0) {
        ESP_LOGW(TAG, "AEC feed error: %d", ret);
      }
    }

    vTaskDelayUntil(&lastWakeTime, interval);
  }

cleanup:
  heap_caps_free(mic_buf32);
  heap_caps_free(mic16);
  heap_caps_free(ref16);
  heap_caps_free(interleaved);
  ESP_LOGI(TAG, "Mic feed task stopped");
  vTaskDelete(NULL);
}

static void aecFetchTask(void* arg) {
  ESP_LOGI(TAG, "Starting AEC Fetch task on core %d", xPortGetCoreID());

  // // Rate limiting
  // uint32_t last_send_time = 0;
  // const uint32_t MIN_SEND_INTERVAL_MS = 15;  // Minimum 15ms between sends

  int16_t opus_input[OPUS_20MS_FRAME];

  while (!fetchTaskShouldStop) {
    // 1. Try to get AEC output
    afe_fetch_result_t* result = sr_aec.fetchOnce();

    if (!result || !result->data || result->data_size == 0) {
      ESP_LOGW(TAG, "No AEC Result");
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }

    if (result && result->data && result->data_size > 0) {
      // Add AEC output to buffer manager
      size_t samples = result->data_size / sizeof(int16_t);
      int16_t* aec_data = (int16_t*)result->data;

      if (!audio_buffer_manager->addSamples(aec_data, samples)) {
        ESP_LOGW(TAG, "Failed to add samples to buffer manager");
        continue;
      }

      // Process available complete Opus frames
      while (audio_buffer_manager->getOpusFrame(opus_input)) {
        // // Rate limiting check
        // uint32_t current_time = esp_timer_get_time() / 1000;
        // if (current_time - last_send_time < MIN_SEND_INTERVAL_MS) {
        //   break;  // Try again next iteration
        // }

        // Encode and send
        opus_int32 encoded_size =
            opus_encode(opus_encoder, opus_input, OPUS_20MS_FRAME,
                        encoder_output_buffer, OPUS_OUT_BUFFER_SIZE);

        if (encoded_size > 0) {
          // last_send_time = current_time;

          // Clear the output buffer before decoding
          memset(output_buffer, 0, 320 * SPEAKER_CHANNELS * sizeof(opus_int16));

          int frames_per_channel = opus_decode(
              opus_decoder, encoder_output_buffer, encoded_size, output_buffer,
              320,  // max frames per channel
              0     // no FEC
          );

          if (frames_per_channel < 0) {
            ESP_LOGE(TAG, "Opus decode failed: %d", frames_per_channel);
            return;
          }

          if (frames_per_channel > 0) {
            // Calculate total samples considering stereo
            int total_samples = frames_per_channel * SPEAKER_CHANNELS;

            // Apply software gain with proper bounds checking
            const float gain = 8.0f;
            const float smoothing = 0.9f;
            float last_sample = 0.0f;
            for (int i = 0; i < total_samples; i++) {
              float current = output_buffer[i] * gain;
              // Apply smoothing to reduce artifacts
              current =
                  (current * (1.0f - smoothing)) + (last_sample * smoothing);
              // Proper clipping
              if (current > 32767.0f) current = 32767.0f;
              if (current < -32768.0f) current = -32768.0f;
              output_buffer[i] = (int16_t)current;
              last_sample = current;
            }

            // Calculate bytes to write
            size_t bytes_to_write = total_samples * sizeof(opus_int16);
            size_t bytes_written = 0;

            // Write to I2S with timeout
            esp_err_t err = i2s_write(I2S_PORT_SPEAKER, output_buffer,
                                      bytes_to_write, &bytes_written,
                                      pdMS_TO_TICKS(50));  // 50ms timeout

            if (err != ESP_OK) {
              ESP_LOGE(TAG, "I2S write failed: %d", err);
            } else if (bytes_written != bytes_to_write) {
              ESP_LOGW(TAG, "Incomplete I2S write: %d/%d bytes", bytes_written,
                       bytes_to_write);
            } else {
              reference_buffer_manager->addSamples(encoder_input_buffer, 320);
            }
          }
        }
      }
    }
    // Small delay to prevent tight loop
    vTaskDelay(pdMS_TO_TICKS(5));
  }

  ESP_LOGI(TAG, "AEC fetch task stopped");
  vTaskDelete(NULL);
}

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "Starting AEC demo with Opus encoding...");

  // Initialize AEC
  if (!sr_aec.init()) {
    ESP_LOGE(TAG, "Failed to init AEC");
    return;
  }
  sr_aec_inited = true;

  if (!init_i2s()) {
    ESP_LOGE(TAG, "Failed to init I2S");
    return;
  }

  // Initialize Opus
  if (!init_opus()) {
    ESP_LOGE(TAG, "Failed to init Opus");
    cleanup();
    return;
  }

  // Initialize buffers
  if (!init_buffers()) {
    ESP_LOGE(TAG, "Failed to init buffers");
    cleanup();
    return;
  }

  // Reset stop flag
  feedTaskShouldStop = false;
  fetchTaskShouldStop = false;

  // Create fetch task first (higher priority)
  BaseType_t ret =
      xTaskCreatePinnedToCore(aecFetchTask, "aecFetch", 24576, NULL,
                              FETCH_TASK_PRIORITY, &aecFetchTaskHandle, 0);

  if (ret != pdPASS) {
    ESP_LOGE(TAG, "Failed to create fetch task");
    cleanup();
    return;
  }

  // Small delay to ensure fetch task is running
  vTaskDelay(pdMS_TO_TICKS(10));

  // Create mic feed task
  ret = xTaskCreatePinnedToCore(micFeedTask, "micFeed", 24576, NULL,
                                FEED_TASK_PRIORITY, &micFeedTaskHandle, 1);

  if (ret != pdPASS) {
    ESP_LOGE(TAG, "Failed to create mic task");
    cleanup();
    return;
  }

  ESP_LOGI(TAG, "AEC demo started successfully");

  while (true) {
    vTaskDelay(pdMS_TO_TICKS(1000));  // Just sleep forever
  }
}