#pragma once

#include <driver/i2s.h>

#include "esp_log.h"

// Audio configuration
#define SAMPLE_RATE 16000
#define AFE_FRAME_SIZE 256   // AFE expects 320 samples
#define DMA_BUFFER_SIZE 256  // Match AFE frame size exactly
#define DMA_BUFFER_COUNT 4   // Number of DMA buffers

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

// Configure I2S for INMP441 Microphone
static const i2s_config_t i2s_mic_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = DMA_BUFFER_COUNT,
    .dma_buf_len = DMA_BUFFER_SIZE,  // IMPORTANT: Match AFE frame size
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
    .dma_buf_len = DMA_BUFFER_SIZE,  // IMPORTANT: Match AFE frame size
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