#pragma once

#include <driver/i2s.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// Audio configuration
#define SAMPLE_RATE 16000
#define SAMPLES_PER_FRAME 256  // Matches AFE's expected frame size
#define I2S_BUFFER_SIZE (SAMPLES_PER_FRAME * 4)  // Buffer for 32-bit samples
#define DMA_BUFFER_COUNT 3

// Pin definitions
#define I2S_MCK_IO -1  // Not used
// Speaker - MAX98357A
#define I2S_BCK_IO_SPEAKER 7  // BCLK
#define I2S_WS_IO_SPEAKER 44  // LRC
#define I2S_DO_IO_SPEAKER 8   // DIN
// Microphone - INMP441
#define I2S_BCK_IO_MIC 3  // SCK
#define I2S_WS_IO_MIC 1   // WS
#define I2S_DI_IO_MIC 2   // SD

// I2S Port assignments
#define I2S_PORT_MIC I2S_NUM_1
#define I2S_PORT_SPEAKER I2S_NUM_0

// Ring buffer configuration
#define RING_BUFFER_SIZE (SAMPLES_PER_FRAME * 8)  // 8 frames of buffering

// Configure I2S for INMP441 Microphone
static const i2s_config_t i2s_mic_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = DMA_BUFFER_COUNT,
    .dma_buf_len = SAMPLES_PER_FRAME,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .mclk_multiple = I2S_MCLK_MULTIPLE_256};

// Configure I2S for MAX98357A Speaker
static const i2s_config_t i2s_speaker_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = DMA_BUFFER_COUNT,
    .dma_buf_len = SAMPLES_PER_FRAME,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .mclk_multiple = I2S_MCLK_MULTIPLE_256};

static const i2s_pin_config_t i2s_mic_pins = {.mck_io_num = I2S_MCK_IO,
                                              .bck_io_num = I2S_BCK_IO_MIC,
                                              .ws_io_num = I2S_WS_IO_MIC,
                                              .data_out_num = I2S_PIN_NO_CHANGE,
                                              .data_in_num = I2S_DI_IO_MIC};

static const i2s_pin_config_t i2s_speaker_pins = {
    .mck_io_num = I2S_MCK_IO,
    .bck_io_num = I2S_BCK_IO_SPEAKER,
    .ws_io_num = I2S_WS_IO_SPEAKER,
    .data_out_num = I2S_DO_IO_SPEAKER,
    .data_in_num = I2S_PIN_NO_CHANGE};

// Utility class for ring buffer operations
class AudioRingBuffer {
 public:
  AudioRingBuffer(size_t size) : buffer_size(size), read_ptr(0), write_ptr(0) {
    buffer = (int16_t*)heap_caps_malloc(size * sizeof(int16_t),
                                        MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    xSemaphore = xSemaphoreCreateMutex();
  }

  ~AudioRingBuffer() {
    if (buffer) {
      heap_caps_free(buffer);
    }
    if (xSemaphore) {
      vSemaphoreDelete(xSemaphore);
    }
  }

  bool write(const int16_t* data, size_t len) {
    if (!buffer || !data || !len) return false;

    if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(10)) != pdTRUE) {
      return false;
    }

    size_t space = available_write();
    if (space < len) {
      xSemaphoreGive(xSemaphore);
      return false;
    }

    for (size_t i = 0; i < len; i++) {
      buffer[write_ptr] = data[i];
      write_ptr = (write_ptr + 1) % buffer_size;
    }

    xSemaphoreGive(xSemaphore);
    return true;
  }

  bool read(int16_t* data, size_t len) {
    printf("in read");
    if (!buffer || !data || !len) return false;

    if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(10)) != pdTRUE) {
      return false;
    }

    size_t available = available_read();
    if (available < len) {
      xSemaphoreGive(xSemaphore);
      return false;
    }

    for (size_t i = 0; i < len; i++) {
      data[i] = buffer[read_ptr];
      read_ptr = (read_ptr + 1) % buffer_size;
    }

    xSemaphoreGive(xSemaphore);
    return true;
  }

  void reset() {
    if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(10)) == pdTRUE) {
      read_ptr = 0;
      write_ptr = 0;
      xSemaphoreGive(xSemaphore);
    }
  }

  size_t available_read() {
    return (write_ptr + buffer_size - read_ptr) % buffer_size;
  }

  size_t available_write() { return buffer_size - available_read() - 1; }

 private:
  int16_t* buffer;
  size_t buffer_size;
  volatile size_t read_ptr;
  volatile size_t write_ptr;
  SemaphoreHandle_t xSemaphore;
};