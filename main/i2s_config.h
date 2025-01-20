#pragma once

#include <driver/i2s.h>

#include "esp_log.h"

// The AFE often defaults to 512-sample frames per channel at 16 kHz, so we’ll
// configure I2S DMA to match that. But we’ll confirm at runtime by asking the
// AFE.

// Typical sample rate for AEC
#define SAMPLE_RATE 16000

// Pins (adjust for your wiring)
#define I2S_MCK_IO -1  // -1 if not using MCLK

// Speaker - MAX98357A
#define I2S_BCK_IO_SPEAKER 7
#define I2S_WS_IO_SPEAKER 44
#define I2S_DO_IO_SPEAKER 8

// Microphone - INMP441
#define I2S_BCK_IO_MIC 3
#define I2S_WS_IO_MIC 1
#define I2S_DI_IO_MIC 2

// I2S ports
#define I2S_PORT_MIC I2S_NUM_1
#define I2S_PORT_SPEAKER I2S_NUM_0

// ------------------- I2S Configs -------------------

// Microphone: INMP441 usually provides 32-bit data, left channel only.
static i2s_config_t i2s_mic_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    // .dma_buf_len will be set dynamically (see app_main) to match AFE frames
    .dma_buf_len = 512,  // placeholder; we’ll fix up at runtime
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .mclk_multiple = I2S_MCLK_MULTIPLE_256};

static const i2s_pin_config_t i2s_mic_pins = {.mck_io_num = I2S_MCK_IO,
                                              .bck_io_num = I2S_BCK_IO_MIC,
                                              .ws_io_num = I2S_WS_IO_MIC,
                                              .data_out_num = I2S_PIN_NO_CHANGE,
                                              .data_in_num = I2S_DI_IO_MIC};

// Speaker: Many I2S amps expect standard stereo (LR) 16-bit frames.
// We’ll replicate our mono data into L and R at runtime.
static i2s_config_t i2s_speaker_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    // .dma_buf_len will be set dynamically (see app_main) to match AFE frames
    .dma_buf_len = 512,  // placeholder; we’ll fix up at runtime
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .mclk_multiple = I2S_MCLK_MULTIPLE_256};

static const i2s_pin_config_t i2s_speaker_pins = {
    .mck_io_num = I2S_MCK_IO,
    .bck_io_num = I2S_BCK_IO_SPEAKER,
    .ws_io_num = I2S_WS_IO_SPEAKER,
    .data_out_num = I2S_DO_IO_SPEAKER,
    .data_in_num = I2S_PIN_NO_CHANGE};