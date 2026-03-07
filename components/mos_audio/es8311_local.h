/*
 * es8311_local.h — ES8311 driver (adapted for ESP-IDF new I2C master API)
 *
 * Adapted from:
 *   espressif/esp-bsp components/es8311/include/es8311.h
 *   SPDX-FileCopyrightText: 2015-2022 Espressif Systems (Shanghai) CO LTD
 *   SPDX-License-Identifier: Apache-2.0
 *
 * Changes vs upstream:
 *   - Uses driver/i2c_master.h (new API) instead of legacy driver/i2c.h
 *   - es8311_create() takes i2c_master_bus_handle_t instead of i2c_port_t
 */

#pragma once

#include "esp_types.h"
#include "esp_err.h"
#include "driver/i2c_master.h"

/* ES8311 I2C address: CE/AD0 pin low = 0x18, high = 0x19 */
#define ES8311_ADDRESS_0 0x18u
#define ES8311_ADDRESS_1 0x19u

#ifdef __cplusplus
extern "C" {
#endif

typedef void *es8311_handle_t;

typedef enum {
    ES8311_MIC_GAIN_MIN = -1,
    ES8311_MIC_GAIN_0DB,
    ES8311_MIC_GAIN_6DB,
    ES8311_MIC_GAIN_12DB,
    ES8311_MIC_GAIN_18DB,
    ES8311_MIC_GAIN_24DB,
    ES8311_MIC_GAIN_30DB,
    ES8311_MIC_GAIN_36DB,
    ES8311_MIC_GAIN_42DB,
    ES8311_MIC_GAIN_MAX
} es8311_mic_gain_t;

typedef enum {
    ES8311_FADE_OFF = 0,
    ES8311_FADE_4LRCK,
    ES8311_FADE_8LRCK,
    ES8311_FADE_16LRCK,
    ES8311_FADE_32LRCK,
    ES8311_FADE_64LRCK,
    ES8311_FADE_128LRCK,
    ES8311_FADE_256LRCK,
    ES8311_FADE_512LRCK,
    ES8311_FADE_1024LRCK,
    ES8311_FADE_2048LRCK,
    ES8311_FADE_4096LRCK,
    ES8311_FADE_8192LRCK,
    ES8311_FADE_16384LRCK,
    ES8311_FADE_32768LRCK,
    ES8311_FADE_65536LRCK
} es8311_fade_t;

typedef enum es8311_resolution_t {
    ES8311_RESOLUTION_16 = 16,
    ES8311_RESOLUTION_18 = 18,
    ES8311_RESOLUTION_20 = 20,
    ES8311_RESOLUTION_24 = 24,
    ES8311_RESOLUTION_32 = 32
} es8311_resolution_t;

typedef struct es8311_clock_config_t {
    bool mclk_inverted;
    bool sclk_inverted;
    bool mclk_from_mclk_pin; /* true: MCLK pin; false: SCLK/BCLK pin */
    int  mclk_frequency;     /* ignored if mclk_from_mclk_pin == false */
    int  sample_frequency;   /* Hz */
} es8311_clock_config_t;

/**
 * @brief Create ES8311 handle
 *
 * @param bus       I2C master bus handle (new API)
 * @param dev_addr  I2C address (ES8311_ADDRESS_0 or ES8311_ADDRESS_1)
 * @return handle, or NULL on failure
 */
es8311_handle_t es8311_create(i2c_master_bus_handle_t bus, uint16_t dev_addr);

/** @brief Delete ES8311 handle and free resources */
void es8311_delete(es8311_handle_t dev);

/**
 * @brief Initialize ES8311 (reset, clock, format, power-up DAC)
 *
 * After this call the ADC modulator is powered on (REG0E=0x02).
 * Call es8311_adc_power_down() immediately after to silence the ADC.
 */
esp_err_t es8311_init(es8311_handle_t dev,
                      const es8311_clock_config_t *clk_cfg,
                      es8311_resolution_t res_in,
                      es8311_resolution_t res_out);

/**
 * @brief Power down the ADC analog section (DAC remains active)
 *
 * Writes REG0E=0xFF (all analog sections powered down except DAC bias),
 * REG15=0x00 (ADC ramp/dmic off), REG17=0x00 (ADC volume mute).
 * Call this after es8311_init() when no microphone is connected.
 */
esp_err_t es8311_adc_power_down(es8311_handle_t dev);

/** @brief Configure sampling frequency (can be called at runtime) */
esp_err_t es8311_sample_frequency_config(es8311_handle_t dev,
                                         int mclk_frequency,
                                         int sample_frequency);

/** @brief Set output DAC volume (0-100) */
esp_err_t es8311_voice_volume_set(es8311_handle_t dev, int volume, int *volume_set);

/** @brief Get output DAC volume */
esp_err_t es8311_voice_volume_get(es8311_handle_t dev, int *volume);

/** @brief Mute/unmute DAC output */
esp_err_t es8311_voice_mute(es8311_handle_t dev, bool mute);

/** @brief Configure microphone path (set digital_mic=false for analog mic) */
esp_err_t es8311_microphone_config(es8311_handle_t dev, bool digital_mic);

/** @brief Set microphone PGA gain */
esp_err_t es8311_microphone_gain_set(es8311_handle_t dev, es8311_mic_gain_t gain_db);

/** @brief Dump all ES8311 registers to console (debug) */
void es8311_register_dump(es8311_handle_t dev);

#ifdef __cplusplus
}
#endif
