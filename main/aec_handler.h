#pragma once

#include <esp_afe_sr_iface.h>
#include <esp_afe_sr_models.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>

#include "i2s_config.h"

class AECHandler {
 public:
  AECHandler()
      : mic_buffer(RING_BUFFER_SIZE),
        ref_buffer(RING_BUFFER_SIZE),
        afe_handle(nullptr),
        afe_data(nullptr),
        afe_task_handle(nullptr),
        audio_callback(nullptr),
        callback_context(nullptr),
        is_running(false),
        frame_size(0) {}

  ~AECHandler() {
    stop();

    if (afe_data && afe_handle) {
      afe_handle->destroy(afe_data);
      afe_data = nullptr;
    }
  }

  bool init() {
    // Get AFE handle for voice communication
    afe_handle = const_cast<esp_afe_sr_iface_t*>(&ESP_AFE_VC_HANDLE);
    if (!afe_handle) {
      ESP_LOGE("AEC", "Failed to get AFE handle");
      return false;
    }

    // Configure AFE
    afe_config_t config = AFE_CONFIG_DEFAULT();
    config.aec_init = true;
    config.se_init = true;
    config.vad_init = false;
    config.wakenet_init = false;
    config.voice_communication_init = true;
    config.voice_communication_agc_init = true;
    config.voice_communication_agc_gain = 15;
    config.afe_mode = SR_MODE_LOW_COST;
    config.afe_perferred_core = 1;
    config.afe_perferred_priority = 5;
    config.afe_ringbuf_size = 100;  // Increased for better buffering
    config.memory_alloc_mode = AFE_MEMORY_ALLOC_INTERNAL_PSRAM_BALANCE;
    config.pcm_config.total_ch_num = 2;  // Mic + Reference
    config.pcm_config.mic_num = 1;       // Single mic
    config.pcm_config.ref_num = 1;       // Single reference
    config.pcm_config.sample_rate = SAMPLE_RATE;

    // Create AFE instance
    afe_data = afe_handle->create_from_config(&config);
    if (!afe_data) {
      ESP_LOGE("AEC", "Failed to create AFE instance");
      return false;
    }

    // Get the frame size
    frame_size = afe_handle->get_feed_chunksize(afe_data);
    ESP_LOGI("AEC", "AFE frame size: %d samples", frame_size);

    if (frame_size != SAMPLES_PER_FRAME) {
      ESP_LOGW("AEC", "AFE frame size (%d) doesn't match configured size (%d)",
               frame_size, SAMPLES_PER_FRAME);
    }

    return true;
  }

  bool start() {
    if (is_running) return true;

    if (!afe_data || !afe_handle) {
      ESP_LOGE("AEC", "AEC not initialized");
      return false;
    }

    is_running = true;
    BaseType_t ret =
        xTaskCreatePinnedToCore(afeProcessingTask, "afe_proc", 8192, this,
                                configMAX_PRIORITIES - 1, &afe_task_handle,
                                1  // Run on core 1
        );

    if (ret != pdPASS) {
      ESP_LOGE("AEC", "Failed to create AFE processing task");
      is_running = false;
      return false;
    }

    return true;
  }

  void stop() {
    is_running = false;
    if (afe_task_handle) {
      vTaskDelete(afe_task_handle);
      afe_task_handle = nullptr;
    }
    mic_buffer.reset();
    ref_buffer.reset();
  }

  bool writeMicData(const int32_t* data, size_t samples) {
    if (!is_running || !data || !samples) return false;

    // Convert 32-bit samples to 16-bit
    int16_t* converted = (int16_t*)heap_caps_malloc(samples * sizeof(int16_t),
                                                    MALLOC_CAP_INTERNAL);
    if (!converted) return false;

    for (size_t i = 0; i < samples; i++) {
      converted[i] = data[i] >> 14;  // Convert to 16-bit
    }

    bool result = mic_buffer.write(converted, samples);
    heap_caps_free(converted);
    printf("writtenMicData");
    return result;
  }

  bool writeRefData(const int16_t* data, size_t samples) {
    if (!is_running || !data || !samples) return false;
    return ref_buffer.write(data, samples);
  }

  typedef void (*AudioCallback)(const int16_t* data, size_t samples,
                                void* context);
  void setProcessedAudioCallback(AudioCallback callback, void* context) {
    audio_callback = callback;
    callback_context = context;
  }

 private:
  static void afeProcessingTask(void* arg) {
    AECHandler* handler = (AECHandler*)arg;
    handler->processingLoop();
  }

  void processingLoop() {
    const size_t frame_samples = frame_size;
    const TickType_t min_period = pdMS_TO_TICKS(15);  // ~16kHz sampling
    TickType_t last_wake_time;
    int16_t* mic_data = nullptr;
    int16_t* ref_data = nullptr;
    int16_t* combined = nullptr;

    // Allocate processing buffers
    mic_data = (int16_t*)heap_caps_malloc(frame_samples * sizeof(int16_t),
                                          MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    ref_data = (int16_t*)heap_caps_malloc(frame_samples * sizeof(int16_t),
                                          MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    combined = (int16_t*)heap_caps_malloc(frame_samples * 2 * sizeof(int16_t),
                                          MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);

    if (!mic_data || !ref_data || !combined) {
      ESP_LOGE("AEC", "Failed to allocate processing buffers");
      goto cleanup;
    }

    last_wake_time = xTaskGetTickCount();
    ESP_LOGI("AEC", "Starting AFE processing loop");

    while (is_running) {
      printf("is_running loop");
      // Try to maintain consistent timing
      vTaskDelayUntil(&last_wake_time, min_period);

      // Read from ring buffers
      if (mic_buffer.read(mic_data, frame_samples) &&
          ref_buffer.read(ref_data, frame_samples)) {
        printf("read from mic");
        // Interleave mic and reference data
        for (size_t i = 0; i < frame_samples; i++) {
          combined[i * 2] = mic_data[i];      // Mic channel
          combined[i * 2 + 1] = ref_data[i];  // Reference channel
        }

        // Feed interleaved data to AFE
        int feed_result = afe_handle->feed(afe_data, combined);
        printf("before feed_result");
        if (feed_result == 0) {
          // Try to fetch processed audio
          printf("Try to fetch processed audio afe_fetch_result_t");
          afe_fetch_result_t* result = afe_handle->fetch(afe_data);
          if (result && result->data && audio_callback) {
            size_t output_samples = result->data_size / sizeof(int16_t);
            audio_callback((int16_t*)result->data, output_samples,
                           callback_context);
          }
        } else {
          static int64_t last_error_time = 0;
          int64_t current_time = esp_timer_get_time() / 1000;
          if (current_time - last_error_time > 1000) {  // Log once per second
            ESP_LOGW("AEC",
                     "AFE feed failed: %d, Frame size: %d, Total samples: %d",
                     feed_result, frame_size, frame_samples * 2);
            last_error_time = current_time;
          }
        }
      }
    }

  cleanup:
    if (mic_data) heap_caps_free(mic_data);
    if (ref_data) heap_caps_free(ref_data);
    if (combined) heap_caps_free(combined);

    ESP_LOGI("AEC", "AFE processing task stopped");
    vTaskDelete(NULL);
  }

 private:
  AudioRingBuffer mic_buffer;
  AudioRingBuffer ref_buffer;
  esp_afe_sr_iface_t* afe_handle;
  esp_afe_sr_data_t* afe_data;
  TaskHandle_t afe_task_handle;
  AudioCallback audio_callback;
  void* callback_context;
  volatile bool is_running;
  int frame_size;

  static constexpr const char* TAG = "AEC";
};