#pragma once

#include <driver/i2s.h>

#include "esp_log.h"

// Audio configuration - adjusted based on ESP AFE documentation
#define SAMPLE_RATE 16000
#define SAMPLES_PER_FRAME 320                   // Standard AFE frame size
#define DMA_FRAME_SIZE (SAMPLES_PER_FRAME * 2)  // Account for mic + ref channel

// Increased buffer sizes to prevent overflow
#define I2S_DMA_BUFFER_COUNT 16  // Increased for better buffering
#define I2S_DMA_BUFFER_LEN (SAMPLES_PER_FRAME * 2)  // Match AFE frame size

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
    .dma_buf_count = I2S_DMA_BUFFER_COUNT,
    .dma_buf_len = I2S_DMA_BUFFER_LEN,
    .use_apll = true,
    .fixed_mclk = SAMPLE_RATE * 256,
    .mclk_multiple = I2S_MCLK_MULTIPLE_256,  // Explicit MCLK multiplier
};

// Configure I2S for MAX98357A Speaker with modified buffer size
static const i2s_config_t i2s_speaker_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = I2S_DMA_BUFFER_COUNT,
    .dma_buf_len = I2S_DMA_BUFFER_LEN,
    .use_apll = true,
    .tx_desc_auto_clear = true,
    .fixed_mclk = SAMPLE_RATE * 256,
    .mclk_multiple = I2S_MCLK_MULTIPLE_256,  // Explicit MCLK multiplier
};

// I2S pin configurations remain the same
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