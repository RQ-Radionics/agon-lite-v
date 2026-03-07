/*
 * es8311_local.c — ES8311 driver (adapted for ESP-IDF new I2C master API)
 *
 * Adapted from:
 *   espressif/esp-bsp components/es8311/es8311.c
 *   SPDX-FileCopyrightText: 2015-2022 Espressif Systems (Shanghai) CO LTD
 *   SPDX-License-Identifier: Apache-2.0
 *
 * Changes vs upstream:
 *   - Uses driver/i2c_master.h (new API) instead of legacy driver/i2c.h
 *   - Added es8311_adc_power_down() to silence ADC on boards without mic
 */

#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"

#include "es8311_local.h"

/* ------------------------------------------------------------------ */
/* Register map                                                         */
/* ------------------------------------------------------------------ */
#define ES8311_RESET_REG00          0x00
#define ES8311_CLK_MANAGER_REG01    0x01
#define ES8311_CLK_MANAGER_REG02    0x02
#define ES8311_CLK_MANAGER_REG03    0x03
#define ES8311_CLK_MANAGER_REG04    0x04
#define ES8311_CLK_MANAGER_REG05    0x05
#define ES8311_CLK_MANAGER_REG06    0x06
#define ES8311_CLK_MANAGER_REG07    0x07
#define ES8311_CLK_MANAGER_REG08    0x08
#define ES8311_SDPIN_REG09          0x09
#define ES8311_SDPOUT_REG0A         0x0A
#define ES8311_SYSTEM_REG0B         0x0B
#define ES8311_SYSTEM_REG0C         0x0C
#define ES8311_SYSTEM_REG0D         0x0D
#define ES8311_SYSTEM_REG0E         0x0E
#define ES8311_SYSTEM_REG0F         0x0F
#define ES8311_SYSTEM_REG10         0x10
#define ES8311_SYSTEM_REG11         0x11
#define ES8311_SYSTEM_REG12         0x12
#define ES8311_SYSTEM_REG13         0x13
#define ES8311_SYSTEM_REG14         0x14
#define ES8311_ADC_REG15            0x15
#define ES8311_ADC_REG16            0x16
#define ES8311_ADC_REG17            0x17
#define ES8311_ADC_REG18            0x18
#define ES8311_ADC_REG19            0x19
#define ES8311_ADC_REG1A            0x1A
#define ES8311_ADC_REG1B            0x1B
#define ES8311_ADC_REG1C            0x1C
#define ES8311_DAC_REG31            0x31
#define ES8311_DAC_REG32            0x32
#define ES8311_DAC_REG33            0x33
#define ES8311_DAC_REG34            0x34
#define ES8311_DAC_REG35            0x35
#define ES8311_DAC_REG37            0x37
#define ES8311_GP_REG45             0x45

/* ------------------------------------------------------------------ */
/* Clock coefficient table                                              */
/* ------------------------------------------------------------------ */
struct _coeff_div {
    uint32_t mclk;
    uint32_t rate;
    uint8_t pre_div;
    uint8_t pre_multi;
    uint8_t adc_div;
    uint8_t dac_div;
    uint8_t fs_mode;
    uint8_t lrck_h;
    uint8_t lrck_l;
    uint8_t bclk_div;
    uint8_t adc_osr;
    uint8_t dac_osr;
};

static const struct _coeff_div coeff_div[] = {
    /* 8k */
    {12288000,  8000, 0x06, 0x00, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {18432000,  8000, 0x03, 0x01, 0x03, 0x03, 0x00, 0x05, 0xff, 0x18, 0x10, 0x10},
    {16384000,  8000, 0x08, 0x00, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    { 8192000,  8000, 0x04, 0x00, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    { 6144000,  8000, 0x03, 0x00, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    { 4096000,  8000, 0x02, 0x00, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    { 3072000,  8000, 0x01, 0x00, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    { 2048000,  8000, 0x01, 0x00, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    { 1536000,  8000, 0x03, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    { 1024000,  8000, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    /* 11.025k */
    {11289600, 11025, 0x04, 0x00, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    { 5644800, 11025, 0x02, 0x00, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    { 2822400, 11025, 0x01, 0x00, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    { 1411200, 11025, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    /* 12k */
    {12288000, 12000, 0x04, 0x00, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    { 6144000, 12000, 0x02, 0x00, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    { 3072000, 12000, 0x01, 0x00, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    { 1536000, 12000, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    /* 16k */
    {12288000, 16000, 0x03, 0x00, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {18432000, 16000, 0x03, 0x01, 0x03, 0x03, 0x00, 0x02, 0xff, 0x0c, 0x10, 0x10},
    {16384000, 16000, 0x04, 0x00, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    { 8192000, 16000, 0x02, 0x00, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    { 6144000, 16000, 0x03, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    { 4096000, 16000, 0x01, 0x00, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    { 3072000, 16000, 0x03, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    { 2048000, 16000, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    { 1536000, 16000, 0x03, 0x03, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    { 1024000, 16000, 0x01, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    /* 22.05k */
    {11289600, 22050, 0x02, 0x00, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    { 5644800, 22050, 0x01, 0x00, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    { 2822400, 22050, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    { 1411200, 22050, 0x01, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {  705600, 22050, 0x01, 0x03, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    /* 24k */
    {12288000, 24000, 0x02, 0x00, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {18432000, 24000, 0x03, 0x00, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    { 6144000, 24000, 0x01, 0x00, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    { 3072000, 24000, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    { 1536000, 24000, 0x01, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    /* 32k */
    {12288000, 32000, 0x03, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {18432000, 32000, 0x03, 0x02, 0x03, 0x03, 0x00, 0x02, 0xff, 0x0c, 0x10, 0x10},
    {16384000, 32000, 0x02, 0x00, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    { 8192000, 32000, 0x01, 0x00, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    { 6144000, 32000, 0x03, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    { 4096000, 32000, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    { 3072000, 32000, 0x03, 0x03, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    { 2048000, 32000, 0x01, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    { 1536000, 32000, 0x03, 0x03, 0x01, 0x01, 0x01, 0x00, 0x7f, 0x02, 0x10, 0x10},
    { 1024000, 32000, 0x01, 0x03, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    /* 44.1k */
    {11289600, 44100, 0x01, 0x00, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    { 5644800, 44100, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    { 2822400, 44100, 0x01, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    { 1411200, 44100, 0x01, 0x03, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    /* 48k */
    {12288000, 48000, 0x01, 0x00, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {18432000, 48000, 0x03, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    { 6144000, 48000, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    { 3072000, 48000, 0x01, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    { 1536000, 48000, 0x01, 0x03, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    /* 64k */
    {12288000, 64000, 0x03, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {16384000, 64000, 0x01, 0x00, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    { 8192000, 64000, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    { 4096000, 64000, 0x01, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    /* 96k */
    {12288000, 96000, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {18432000, 96000, 0x03, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    { 6144000, 96000, 0x01, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    { 3072000, 96000, 0x01, 0x03, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
};

static const char *TAG = "es8311_local";

/* ------------------------------------------------------------------ */
/* Device context                                                       */
/* ------------------------------------------------------------------ */
typedef struct {
    i2c_master_dev_handle_t dev_handle;
} es8311_dev_t;

/* ------------------------------------------------------------------ */
/* Low-level I2C read/write (new master API)                            */
/* ------------------------------------------------------------------ */
static esp_err_t es8311_write_reg(es8311_dev_t *dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(dev->dev_handle, buf, 2, pdMS_TO_TICKS(1000));
}

static esp_err_t es8311_read_reg(es8311_dev_t *dev, uint8_t reg, uint8_t *val)
{
    return i2c_master_transmit_receive(dev->dev_handle,
                                       &reg, 1,
                                       val, 1,
                                       pdMS_TO_TICKS(1000));
}

/* ------------------------------------------------------------------ */
/* Clock coefficient lookup                                             */
/* ------------------------------------------------------------------ */
static int get_coeff(uint32_t mclk, uint32_t rate)
{
    for (int i = 0; i < (int)(sizeof(coeff_div) / sizeof(coeff_div[0])); i++) {
        if (coeff_div[i].rate == rate && coeff_div[i].mclk == (uint32_t)mclk) {
            return i;
        }
    }
    return -1;
}

/* ------------------------------------------------------------------ */
/* Public: create / delete                                              */
/* ------------------------------------------------------------------ */
es8311_handle_t es8311_create(i2c_master_bus_handle_t bus, uint16_t dev_addr)
{
    es8311_dev_t *dev = (es8311_dev_t *)calloc(1, sizeof(es8311_dev_t));
    if (!dev) return NULL;

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = dev_addr,
        .scl_speed_hz    = 400000,
    };
    if (i2c_master_bus_add_device(bus, &dev_cfg, &dev->dev_handle) != ESP_OK) {
        free(dev);
        return NULL;
    }
    return (es8311_handle_t)dev;
}

void es8311_delete(es8311_handle_t handle)
{
    if (!handle) return;
    es8311_dev_t *dev = (es8311_dev_t *)handle;
    i2c_master_bus_rm_device(dev->dev_handle);
    free(dev);
}

/* ------------------------------------------------------------------ */
/* Sample frequency configuration                                       */
/* ------------------------------------------------------------------ */
esp_err_t es8311_sample_frequency_config(es8311_handle_t handle,
                                         int mclk_frequency,
                                         int sample_frequency)
{
    es8311_dev_t *dev = (es8311_dev_t *)handle;
    uint8_t regv;

    int coeff = get_coeff((uint32_t)mclk_frequency, (uint32_t)sample_frequency);
    if (coeff < 0) {
        ESP_LOGE(TAG, "No coefficient for MCLK=%d Hz, fs=%d Hz",
                 mclk_frequency, sample_frequency);
        return ESP_ERR_INVALID_ARG;
    }
    const struct _coeff_div *c = &coeff_div[coeff];

    /* REG02 */
    ESP_RETURN_ON_ERROR(es8311_read_reg(dev, ES8311_CLK_MANAGER_REG02, &regv), TAG, "i2c");
    regv &= 0x07;
    regv |= (uint8_t)((c->pre_div - 1) << 5);
    regv |= (uint8_t)(c->pre_multi << 3);
    ESP_RETURN_ON_ERROR(es8311_write_reg(dev, ES8311_CLK_MANAGER_REG02, regv), TAG, "i2c");

    /* REG03 */
    ESP_RETURN_ON_ERROR(es8311_write_reg(dev, ES8311_CLK_MANAGER_REG03,
                        (uint8_t)((c->fs_mode << 6) | c->adc_osr)), TAG, "i2c");
    /* REG04 */
    ESP_RETURN_ON_ERROR(es8311_write_reg(dev, ES8311_CLK_MANAGER_REG04, c->dac_osr), TAG, "i2c");
    /* REG05 */
    ESP_RETURN_ON_ERROR(es8311_write_reg(dev, ES8311_CLK_MANAGER_REG05,
                        (uint8_t)(((c->adc_div - 1) << 4) | (c->dac_div - 1))), TAG, "i2c");
    /* REG06 */
    ESP_RETURN_ON_ERROR(es8311_read_reg(dev, ES8311_CLK_MANAGER_REG06, &regv), TAG, "i2c");
    regv &= 0xE0;
    if (c->bclk_div < 19) {
        regv |= (uint8_t)((c->bclk_div - 1) << 0);
    } else {
        regv |= (uint8_t)(c->bclk_div << 0);
    }
    ESP_RETURN_ON_ERROR(es8311_write_reg(dev, ES8311_CLK_MANAGER_REG06, regv), TAG, "i2c");

    /* REG07 */
    ESP_RETURN_ON_ERROR(es8311_read_reg(dev, ES8311_CLK_MANAGER_REG07, &regv), TAG, "i2c");
    regv &= 0xC0;
    regv |= c->lrck_h;
    ESP_RETURN_ON_ERROR(es8311_write_reg(dev, ES8311_CLK_MANAGER_REG07, regv), TAG, "i2c");

    /* REG08 */
    ESP_RETURN_ON_ERROR(es8311_write_reg(dev, ES8311_CLK_MANAGER_REG08, c->lrck_l), TAG, "i2c");

    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/* Internal: clock config                                               */
/* ------------------------------------------------------------------ */
static esp_err_t es8311_clock_config(es8311_dev_t *dev,
                                     const es8311_clock_config_t *clk_cfg,
                                     es8311_resolution_t res)
{
    uint8_t reg06;
    uint8_t reg01 = 0x3F; /* Enable all clocks */
    int mclk_hz;

    if (clk_cfg->mclk_from_mclk_pin) {
        mclk_hz = clk_cfg->mclk_frequency;
    } else {
        mclk_hz = clk_cfg->sample_frequency * (int)res * 2;
        reg01 |= (1 << 7); /* Use BCLK as MCLK source */
    }
    if (clk_cfg->mclk_inverted) {
        reg01 |= (1 << 6);
    }
    ESP_RETURN_ON_ERROR(es8311_write_reg(dev, ES8311_CLK_MANAGER_REG01, reg01), TAG, "i2c");

    ESP_RETURN_ON_ERROR(es8311_read_reg(dev, ES8311_CLK_MANAGER_REG06, &reg06), TAG, "i2c");
    if (clk_cfg->sclk_inverted) {
        reg06 |= (1 << 5);
    } else {
        reg06 &= (uint8_t)~(1 << 5);
    }
    ESP_RETURN_ON_ERROR(es8311_write_reg(dev, ES8311_CLK_MANAGER_REG06, reg06), TAG, "i2c");

    return es8311_sample_frequency_config((es8311_handle_t)dev, mclk_hz, clk_cfg->sample_frequency);
}

/* ------------------------------------------------------------------ */
/* Internal: format config                                              */
/* ------------------------------------------------------------------ */
static esp_err_t es8311_fmt_config(es8311_dev_t *dev,
                                   es8311_resolution_t res_in,
                                   es8311_resolution_t res_out)
{
    uint8_t reg09 = 0, reg0a = 0;

    switch (res_in) {
    case ES8311_RESOLUTION_16: reg09 = (3 << 2); break;
    case ES8311_RESOLUTION_18: reg09 = (2 << 2); break;
    case ES8311_RESOLUTION_20: reg09 = (1 << 2); break;
    case ES8311_RESOLUTION_24: reg09 = (0 << 2); break;
    case ES8311_RESOLUTION_32: reg09 = (4 << 2); break;
    }
    switch (res_out) {
    case ES8311_RESOLUTION_16: reg0a = (3 << 2); break;
    case ES8311_RESOLUTION_18: reg0a = (2 << 2); break;
    case ES8311_RESOLUTION_20: reg0a = (1 << 2); break;
    case ES8311_RESOLUTION_24: reg0a = (0 << 2); break;
    case ES8311_RESOLUTION_32: reg0a = (4 << 2); break;
    }

    /* Set slave mode (clear REG00[6]) */
    uint8_t reg00;
    ESP_RETURN_ON_ERROR(es8311_read_reg(dev, ES8311_RESET_REG00, &reg00), TAG, "i2c");
    reg00 &= 0xBF;
    ESP_RETURN_ON_ERROR(es8311_write_reg(dev, ES8311_RESET_REG00, reg00), TAG, "i2c");

    ESP_RETURN_ON_ERROR(es8311_write_reg(dev, ES8311_SDPIN_REG09, reg09), TAG, "i2c");
    ESP_RETURN_ON_ERROR(es8311_write_reg(dev, ES8311_SDPOUT_REG0A, reg0a), TAG, "i2c");

    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/* Public: initialize                                                   */
/* ------------------------------------------------------------------ */
esp_err_t es8311_init(es8311_handle_t handle,
                      const es8311_clock_config_t *clk_cfg,
                      es8311_resolution_t res_in,
                      es8311_resolution_t res_out)
{
    es8311_dev_t *dev = (es8311_dev_t *)handle;

    /* Reset */
    ESP_RETURN_ON_ERROR(es8311_write_reg(dev, ES8311_RESET_REG00, 0x1F), TAG, "reset");
    vTaskDelay(pdMS_TO_TICKS(20));
    ESP_RETURN_ON_ERROR(es8311_write_reg(dev, ES8311_RESET_REG00, 0x00), TAG, "reset");
    ESP_RETURN_ON_ERROR(es8311_write_reg(dev, ES8311_RESET_REG00, 0x80), TAG, "power-on");

    /* Clock */
    ESP_RETURN_ON_ERROR(es8311_clock_config(dev, clk_cfg, res_out), TAG, "clk");

    /* I2S format */
    ESP_RETURN_ON_ERROR(es8311_fmt_config(dev, res_in, res_out), TAG, "fmt");

    /* Power up analog */
    ESP_RETURN_ON_ERROR(es8311_write_reg(dev, ES8311_SYSTEM_REG0D, 0x01), TAG, "pwr");
    ESP_RETURN_ON_ERROR(es8311_write_reg(dev, ES8311_SYSTEM_REG0E, 0x02), TAG, "pwr");
    /* Power up DAC */
    ESP_RETURN_ON_ERROR(es8311_write_reg(dev, ES8311_SYSTEM_REG12, 0x00), TAG, "dac");
    ESP_RETURN_ON_ERROR(es8311_write_reg(dev, ES8311_SYSTEM_REG13, 0x10), TAG, "dac");
    /* ADC equalizer bypass, cancel DC offset */
    ESP_RETURN_ON_ERROR(es8311_write_reg(dev, ES8311_ADC_REG1C, 0x6A), TAG, "adc");
    /* DAC equalizer bypass */
    ESP_RETURN_ON_ERROR(es8311_write_reg(dev, ES8311_DAC_REG37, 0x08), TAG, "dac");

    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/* Public: power down ADC (no microphone on board)                      */
/* ------------------------------------------------------------------ */
esp_err_t es8311_adc_power_down(es8311_handle_t handle)
{
    es8311_dev_t *dev = (es8311_dev_t *)handle;

    /* REG0E: power up/down register
     *   bits[5:4] = ADC analog power:  0b11 = power down ADC PGA + modulator
     *   bits[1:0] = bias/ref:          keep as-is
     * Safe value: 0x00 keeps everything up, but we want only DAC on.
     * Per ES8311 datasheet:
     *   REG0E[5] = 1  → power down ADC analog (ADC_LP)
     *   REG0E[4] = 1  → power down ADC modulator
     * Set 0x30 to power down both ADC parts while keeping bias (DAC needs bias). */
    ESP_RETURN_ON_ERROR(es8311_write_reg(dev, ES8311_SYSTEM_REG0E, 0x32), TAG, "adc_pd");

    /* REG15: ADC ramp rate / DMIC select — set to 0 (no ADC activity) */
    ESP_RETURN_ON_ERROR(es8311_write_reg(dev, ES8311_ADC_REG15, 0x00), TAG, "adc_pd");

    /* REG17: ADC volume — mute */
    ESP_RETURN_ON_ERROR(es8311_write_reg(dev, ES8311_ADC_REG17, 0x00), TAG, "adc_pd");

    ESP_LOGI(TAG, "ADC powered down (no mic on board)");
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/* Public: microphone config                                            */
/* ------------------------------------------------------------------ */
esp_err_t es8311_microphone_config(es8311_handle_t handle, bool digital_mic)
{
    es8311_dev_t *dev = (es8311_dev_t *)handle;
    uint8_t reg14 = 0x1A; /* Enable analog MIC, max PGA gain */
    if (digital_mic) {
        reg14 |= (1 << 6);
    }
    es8311_write_reg(dev, ES8311_ADC_REG17, 0xC8); /* ADC gain */
    return es8311_write_reg(dev, ES8311_SYSTEM_REG14, reg14);
}

esp_err_t es8311_microphone_gain_set(es8311_handle_t handle, es8311_mic_gain_t gain_db)
{
    es8311_dev_t *dev = (es8311_dev_t *)handle;
    return es8311_write_reg(dev, ES8311_ADC_REG16, (uint8_t)gain_db);
}

/* ------------------------------------------------------------------ */
/* Public: DAC volume / mute                                            */
/* ------------------------------------------------------------------ */
esp_err_t es8311_voice_volume_set(es8311_handle_t handle, int volume, int *volume_set)
{
    if (volume < 0) volume = 0;
    if (volume > 100) volume = 100;
    if (volume_set) *volume_set = volume;

    es8311_dev_t *dev = (es8311_dev_t *)handle;
    uint8_t reg32 = (volume == 0) ? 0 : (uint8_t)((volume * 256 / 100) - 1);
    return es8311_write_reg(dev, ES8311_DAC_REG32, reg32);
}

esp_err_t es8311_voice_volume_get(es8311_handle_t handle, int *volume)
{
    es8311_dev_t *dev = (es8311_dev_t *)handle;
    uint8_t reg32;
    ESP_RETURN_ON_ERROR(es8311_read_reg(dev, ES8311_DAC_REG32, &reg32), TAG, "i2c");
    *volume = (reg32 == 0) ? 0 : ((reg32 * 100) / 256) + 1;
    return ESP_OK;
}

esp_err_t es8311_voice_mute(es8311_handle_t handle, bool mute)
{
    es8311_dev_t *dev = (es8311_dev_t *)handle;
    uint8_t reg31;
    ESP_RETURN_ON_ERROR(es8311_read_reg(dev, ES8311_DAC_REG31, &reg31), TAG, "i2c");
    if (mute) {
        reg31 |= (1 << 6) | (1 << 5);
    } else {
        reg31 &= (uint8_t)~((1 << 6) | (1 << 5));
    }
    return es8311_write_reg(dev, ES8311_DAC_REG31, reg31);
}

/* ------------------------------------------------------------------ */
/* Public: register dump (debug)                                        */
/* ------------------------------------------------------------------ */
void es8311_register_dump(es8311_handle_t handle)
{
    es8311_dev_t *dev = (es8311_dev_t *)handle;
    for (int reg = 0; reg < 0x4A; reg++) {
        uint8_t val = 0;
        es8311_read_reg(dev, (uint8_t)reg, &val);
        ESP_LOGI(TAG, "REG[0x%02X] = 0x%02X", reg, val);
    }
}
