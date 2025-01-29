#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <stdint.h>
#include <string.h>

#include <algorithm>

#include "esp_log.h"

// Constants
#define OPUS_FRAME_SIZE 320    // Opus expects 320 samples (20ms @ 16kHz)
#define AEC_FRAME_SIZE 512     // AEC provides 512 samples
#define RING_BUFFER_SIZE 1920  // Buffer size in samples
#define MAX_PENDING_FRAMES \
  6  // Maximum number of 320-sample frames we can store

class AudioBufferManager {
 public:
  AudioBufferManager() : write_pos(0), read_pos(0), samples_available(0) {
    mutex = xSemaphoreCreateMutex();
  }

  ~AudioBufferManager() {
    if (mutex) {
      vSemaphoreDelete(mutex);
    }
  }

  // Add samples to the ring buffer
  bool addSamples(const int16_t* samples, size_t count) {
    if (!samples || count == 0 || count > RING_BUFFER_SIZE) {
      ESP_LOGW(TAG, "Invalid parameters or mutex not initialized");
      return false;
    }

    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
      return false;
    }

    // Check if we have enough space
    if (samples_available + count > RING_BUFFER_SIZE) {
      xSemaphoreGive(mutex);
      ESP_LOGW(TAG, "Ring buffer full - dropping samples");
      return false;
    }

    // Copy samples to ring buffer
    size_t first_copy =
        std::min(static_cast<size_t>(count), RING_BUFFER_SIZE - write_pos);
    memcpy(&ring_buffer[write_pos], samples, first_copy * sizeof(int16_t));

    if (first_copy < count) {
      // Wrap around and copy remaining
      memcpy(&ring_buffer[0], samples + first_copy,
             (count - first_copy) * sizeof(int16_t));
    }

    write_pos = (write_pos + count) % RING_BUFFER_SIZE;
    samples_available += count;

    xSemaphoreGive(mutex);
    return true;
  }

  // Get exactly OPUS_FRAME_SIZE samples if available
  bool getOpusFrame(int16_t* output) {
    if (!output) return false;

    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
      return false;
    }

    if (samples_available < OPUS_FRAME_SIZE) {
      xSemaphoreGive(mutex);
      return false;
    }

    // Copy samples to output buffer
    size_t first_read = std::min(static_cast<size_t>(OPUS_FRAME_SIZE),
                                 RING_BUFFER_SIZE - read_pos);
    memcpy(output, &ring_buffer[read_pos], first_read * sizeof(int16_t));

    if (first_read < OPUS_FRAME_SIZE) {
      // Wrap around and copy remaining
      memcpy(output + first_read, &ring_buffer[0],
             (OPUS_FRAME_SIZE - first_read) * sizeof(int16_t));
    }

    read_pos = (read_pos + OPUS_FRAME_SIZE) % RING_BUFFER_SIZE;
    samples_available -= OPUS_FRAME_SIZE;

    xSemaphoreGive(mutex);
    return true;
  }

  size_t available() const { return samples_available; }

 private:
  static constexpr const char* TAG = "AUDIO_BUFFER";

  int16_t ring_buffer[RING_BUFFER_SIZE];
  size_t write_pos;
  size_t read_pos;
  size_t samples_available;
  SemaphoreHandle_t mutex;
};