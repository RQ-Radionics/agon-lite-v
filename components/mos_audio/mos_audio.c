/*
 * mos_audio.c — Audio driver for Agon MOS (Olimex ESP32-P4-PC)
 *
 * Exact copy of the BSP audio init sequence from the Olimex production test:
 *   bsp_i2c_init()                  → i2c_new_master_bus (no extras)
 *   bsp_audio_init(NULL)            → i2s_new_channel + enable at 22050 mono
 *   bsp_audio_codec_speaker_init()  → es8311_codec_new + esp_codec_dev_open
 *
 * If the production test works, this works — no divergence allowed.
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/i2s_std.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"

#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "es8311_codec.h"

#include "mos_audio.h"

static const char *TAG = "mos_audio";

static i2s_chan_handle_t             s_i2s_tx      = NULL;
static i2s_chan_handle_t             s_i2s_rx      = NULL;
static const audio_codec_data_if_t *s_i2s_data_if = NULL;
static esp_codec_dev_handle_t        s_spk_dev     = NULL;
static i2c_master_bus_handle_t       s_i2c_bus     = NULL;
static bool                          s_bus_owned   = false;
static bool                          s_initialized = false;

/* ------------------------------------------------------------------ */
/* bsp_audio_init(NULL) — exact copy                                   */
/* ------------------------------------------------------------------ */
static esp_err_t audio_i2s_init(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(
        CONFIG_MOS_AUDIO_I2S_NUM, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;
    ESP_RETURN_ON_ERROR(
        i2s_new_channel(&chan_cfg, &s_i2s_tx, &s_i2s_rx),
        TAG, "i2s_new_channel");

    /* BSP_I2S_DUPLEX_MONO_CFG(22050) */
    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(22050),
        .slot_cfg = I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG(
                        I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = CONFIG_MOS_AUDIO_I2S_MCLK_GPIO,
            .bclk = CONFIG_MOS_AUDIO_I2S_BCLK_GPIO,
            .ws   = CONFIG_MOS_AUDIO_I2S_WS_GPIO,
            .dout = CONFIG_MOS_AUDIO_I2S_DOUT_GPIO,
            .din  = CONFIG_MOS_AUDIO_I2S_DIN_GPIO,
            .invert_flags = { false, false, false },
        },
    };

    ESP_RETURN_ON_ERROR(
        i2s_channel_init_std_mode(s_i2s_tx, &std_cfg), TAG, "i2s_init TX");
    ESP_RETURN_ON_ERROR(
        i2s_channel_enable(s_i2s_tx), TAG, "i2s_enable TX");
    ESP_RETURN_ON_ERROR(
        i2s_channel_init_std_mode(s_i2s_rx, &std_cfg), TAG, "i2s_init RX");
    ESP_RETURN_ON_ERROR(
        i2s_channel_enable(s_i2s_rx), TAG, "i2s_enable RX");

    audio_codec_i2s_cfg_t i2s_cfg = {
        .port      = CONFIG_MOS_AUDIO_I2S_NUM,
        .tx_handle = s_i2s_tx,
        .rx_handle = s_i2s_rx,
    };
    s_i2s_data_if = audio_codec_new_i2s_data(&i2s_cfg);
    ESP_RETURN_ON_FALSE(s_i2s_data_if, ESP_ERR_NO_MEM,
                        TAG, "audio_codec_new_i2s_data");
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/* bsp_audio_codec_speaker_init() — exact copy                         */
/* ------------------------------------------------------------------ */
static esp_err_t audio_codec_init(i2c_master_bus_handle_t bus)
{
    const audio_codec_gpio_if_t *gpio_if = audio_codec_new_gpio();
    ESP_RETURN_ON_FALSE(gpio_if, ESP_ERR_NO_MEM, TAG, "audio_codec_new_gpio");

    audio_codec_i2c_cfg_t i2c_cfg = {
        .port       = CONFIG_MOS_AUDIO_I2C_PORT,
        .addr       = ES8311_CODEC_DEFAULT_ADDR,
        .bus_handle = bus,
    };
    const audio_codec_ctrl_if_t *i2c_ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    ESP_RETURN_ON_FALSE(i2c_ctrl_if, ESP_ERR_NO_MEM,
                        TAG, "audio_codec_new_i2c_ctrl");

    esp_codec_dev_hw_gain_t gain = {
        .pa_voltage        = 5.0f,
        .codec_dac_voltage = 3.3f,
    };
    es8311_codec_cfg_t es8311_cfg = {
        .ctrl_if      = i2c_ctrl_if,
        .gpio_if      = gpio_if,
        .codec_mode   = ESP_CODEC_DEV_TYPE_OUT,
        .pa_pin       = CONFIG_MOS_AUDIO_PA_GPIO,
        .pa_reverted  = false,
        .master_mode  = false,
        .use_mclk     = true,
        .digital_mic  = false,
        .invert_mclk  = false,
        .invert_sclk  = false,
        .hw_gain      = gain,
    };
    const audio_codec_if_t *es8311_dev = es8311_codec_new(&es8311_cfg);
    ESP_RETURN_ON_FALSE(es8311_dev, ESP_ERR_NOT_FOUND, TAG, "es8311_codec_new");

    esp_codec_dev_cfg_t dev_cfg = {
        .dev_type  = ESP_CODEC_DEV_TYPE_IN_OUT,
        .codec_if  = es8311_dev,
        .data_if   = s_i2s_data_if,
    };
    s_spk_dev = esp_codec_dev_new(&dev_cfg);
    ESP_RETURN_ON_FALSE(s_spk_dev, ESP_ERR_NO_MEM, TAG, "esp_codec_dev_new");

    esp_codec_dev_sample_info_t fs = {
        .sample_rate     = 48000,
        .channel         = 2,
        .bits_per_sample = 16,
    };
    ESP_RETURN_ON_ERROR(
        esp_codec_dev_open(s_spk_dev, &fs), TAG, "esp_codec_dev_open");
    ESP_RETURN_ON_ERROR(
        esp_codec_dev_set_out_vol(s_spk_dev, 70), TAG, "set_out_vol");

    ESP_LOGI(TAG, "Audio initialized: 48000 Hz stereo 16-bit");
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/* Public API                                                           */
/* ------------------------------------------------------------------ */

esp_err_t mos_audio_init(void)
{
    ESP_RETURN_ON_FALSE(!s_initialized, ESP_ERR_INVALID_STATE,
                        TAG, "already initialized");

    /* bsp_i2c_init() — exact copy, no extras */
    i2c_master_bus_config_t i2c_bus_conf = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .sda_io_num = CONFIG_MOS_AUDIO_I2C_SDA_GPIO,
        .scl_io_num = CONFIG_MOS_AUDIO_I2C_SCL_GPIO,
        .i2c_port   = CONFIG_MOS_AUDIO_I2C_PORT,
    };
    ESP_RETURN_ON_ERROR(
        i2c_new_master_bus(&i2c_bus_conf, &s_i2c_bus),
        TAG, "i2c_new_master_bus");
    s_bus_owned = true;

    esp_err_t ret = audio_i2s_init();
    if (ret == ESP_OK) ret = audio_codec_init(s_i2c_bus);

    if (ret != ESP_OK && s_bus_owned) {
        i2c_del_master_bus(s_i2c_bus);
        s_i2c_bus   = NULL;
        s_bus_owned = false;
    } else if (ret == ESP_OK) {
        s_initialized = true;
    }
    return ret;
}

esp_err_t mos_audio_init_with_bus(i2c_master_bus_handle_t bus)
{
    ESP_RETURN_ON_FALSE(!s_initialized, ESP_ERR_INVALID_STATE,
                        TAG, "already initialized");
    ESP_RETURN_ON_FALSE(bus != NULL, ESP_ERR_INVALID_ARG,
                        TAG, "bus handle is NULL");

    s_i2c_bus   = bus;
    s_bus_owned = false;

    esp_err_t ret = audio_i2s_init();
    if (ret == ESP_OK) ret = audio_codec_init(bus);
    if (ret == ESP_OK) s_initialized = true;
    return ret;
}

int mos_audio_write(const int16_t *samples, size_t num_samples, uint32_t timeout_ms)
{
    if (!s_initialized || !samples || num_samples == 0) return -1;
    int ret = esp_codec_dev_write(s_spk_dev,
                                  (void *)samples,
                                  (int)(num_samples * sizeof(int16_t)));
    return (ret == ESP_CODEC_DEV_OK) ? (int)num_samples : -1;
}

esp_err_t mos_audio_set_volume(int volume)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    if (volume < 0)   volume = 0;
    if (volume > 100) volume = 100;
    return (esp_codec_dev_set_out_vol(s_spk_dev, volume) == ESP_CODEC_DEV_OK)
           ? ESP_OK : ESP_FAIL;
}

i2s_chan_handle_t mos_audio_get_tx_handle(void)
{
    return s_initialized ? s_i2s_tx : NULL;
}

void mos_audio_deinit(void)
{
    if (!s_initialized) return;

    esp_codec_dev_close(s_spk_dev);
    esp_codec_dev_delete(s_spk_dev);
    s_spk_dev = NULL;

    i2s_del_channel(s_i2s_tx);
    i2s_del_channel(s_i2s_rx);
    s_i2s_tx = s_i2s_rx = NULL;
    s_i2s_data_if = NULL;

    if (s_bus_owned && s_i2c_bus) {
        i2c_del_master_bus(s_i2c_bus);
    }
    s_i2c_bus   = NULL;
    s_bus_owned = false;

    s_initialized = false;
    ESP_LOGI(TAG, "mos_audio de-initialized");
}
