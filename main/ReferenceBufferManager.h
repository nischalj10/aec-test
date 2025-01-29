#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <string.h>

#include <algorithm>

#include "esp_log.h"

class ReferenceBufferManager {
 public:
  static constexpr size_t REF_BUFFER_SIZE = 512;  // Required size for AEC
  static constexpr const char* TAG = "REF_BUFFER";

  ReferenceBufferManager() : write_pos(0), total_samples(0) {
    mutex = xSemaphoreCreateMutex();
    memset(buffer, 0, sizeof(buffer));
  }

  ~ReferenceBufferManager() {
    if (mutex) {
      vSemaphoreDelete(mutex);
    }
  }

  // Add new samples to the circular buffer
  bool addSamples(const int16_t* samples, size_t count) {
    if (!samples || count == 0) {
      return false;
    }

    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
      ESP_LOGW(TAG, "Failed to take mutex");
      return false;
    }

    // Copy samples to ring buffer
    size_t first_copy = std::min(count, REF_BUFFER_SIZE - write_pos);
    memcpy(&buffer[write_pos], samples, first_copy * sizeof(int16_t));

    if (first_copy < count) {
      // Wrap around and copy remaining
      size_t remaining = count - first_copy;
      memcpy(&buffer[0], samples + first_copy, remaining * sizeof(int16_t));
      write_pos = remaining;
    } else {
      write_pos = (write_pos + first_copy) % REF_BUFFER_SIZE;
    }

    // Update total samples count
    total_samples += count;
    if (total_samples > REF_BUFFER_SIZE) {
      total_samples = REF_BUFFER_SIZE;
    }

    xSemaphoreGive(mutex);
    return true;
  }

  // Get the most recent 512 samples for AEC
  bool getReference(int16_t* output) {
    if (!output) {
      return false;
    }

    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
      ESP_LOGW(TAG, "Failed to take mutex");
      return false;
    }

    if (total_samples < REF_BUFFER_SIZE) {
      // Not enough samples yet, pad with zeros
      size_t valid_samples = total_samples;

      // Copy available samples
      if (write_pos >= valid_samples) {
        // All samples are contiguous
        memcpy(output, &buffer[write_pos - valid_samples],
               valid_samples * sizeof(int16_t));
      } else {
        // Samples wrap around
        size_t end_samples = valid_samples - write_pos;
        memcpy(output, &buffer[REF_BUFFER_SIZE - end_samples],
               end_samples * sizeof(int16_t));
        memcpy(output + end_samples, buffer, write_pos * sizeof(int16_t));
      }

      // Zero-pad the rest
      memset(output + valid_samples, 0,
             (REF_BUFFER_SIZE - valid_samples) * sizeof(int16_t));
    } else {
      // We have enough samples, get most recent 512
      if (write_pos >= REF_BUFFER_SIZE) {
        // Simple case - just copy the last 512 samples
        memcpy(output, &buffer[write_pos - REF_BUFFER_SIZE],
               REF_BUFFER_SIZE * sizeof(int16_t));
      } else {
        // Need to handle wrap-around
        size_t end_samples = REF_BUFFER_SIZE - write_pos;
        memcpy(output, &buffer[REF_BUFFER_SIZE - end_samples],
               end_samples * sizeof(int16_t));
        memcpy(output + end_samples, buffer, write_pos * sizeof(int16_t));
      }
    }

    xSemaphoreGive(mutex);
    return true;
  }

  size_t available() const { return total_samples; }

 private:
  int16_t buffer[REF_BUFFER_SIZE];
  size_t write_pos;
  size_t total_samples;
  SemaphoreHandle_t mutex;
};