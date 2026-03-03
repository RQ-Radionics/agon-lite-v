/*
 * mos_audio.c — Audio driver for Agon MOS (Olimex ESP32-P4-PC)
 *
 * Uses the esp_codec_dev abstraction layer with the ES8311 codec driver,
 * mirroring the approach used in the Olimex production test firmware.
 * This avoids direct ES8311 register manipulation and relies on the
 * tested, upstream codec driver for correct init sequencing.
 *
 * Hardware connections (Olimex ESP32-P4-PC Rev B):
 *   I2C1: SDA=GPIO7, SCL=GPIO8  (400 kHz, shared with LT8912B)
 *   GPIO6: CODEC_PWR_DIS# (active LOW — pull LOW to power on codec)
 *   GPIO53: PA_EN (active HIGH — managed automatically by esp_codec_dev)
 *   I2S0: DOUT=GPIO9, WS=GPIO10, DIN=GPIO11, BCLK=GPIO12, MCLK=GPIO13
 *
 * Sample rate: 48000 Hz stereo 16-bit (matches Olimex production test).
 * The Agon VDP synthesizes audio internally — there is no PCM sample
 * streaming from MOS, so the sample rate is an implementation detail
 * with no compatibility constraints.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
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

#define MOS_AUDIO_SAMPLE_RATE   48000
#define MOS_AUDIO_CHANNELS      2       /* stereo — matches production test */
#define MOS_AUDIO_BITS          16

/* ------------------------------------------------------------------ */
/* Driver state                                                         */
/* ------------------------------------------------------------------ */
typedef struct {
    i2s_chan_handle_t             i2s_tx;
    i2s_chan_handle_t             i2s_rx;
    const audio_codec_data_if_t *data_if;
    esp_codec_dev_handle_t        spk;
    bool                          initialized;
    bool                          bus_owned;
} mos_audio_t;

static mos_audio_t s_audio = { 0 };

/* ------------------------------------------------------------------ */
/* Internal: I2S + codec init given a ready i2c_master_bus_handle_t    */
/* ------------------------------------------------------------------ */
static esp_err_t audio_init_common(i2c_master_bus_handle_t bus)
{
    /* --- I2S channels ------------------------------------------------ */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(
        CONFIG_MOS_AUDIO_I2S_NUM, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;
    ESP_RETURN_ON_ERROR(
        i2s_new_channel(&chan_cfg, &s_audio.i2s_tx, &s_audio.i2s_rx),
        TAG, "i2s_new_channel");

    /* BSP uses I2S_STD_CLK_DEFAULT_CONFIG + MONO slot at 22050 as the
     * init rate; esp_codec_dev_open() will reconfigure to the real rate.
     * We init at our target rate directly so there is no intermediate
     * reconfiguration step. */
    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(MOS_AUDIO_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
                        I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = CONFIG_MOS_AUDIO_I2S_MCLK_GPIO,
            .bclk = CONFIG_MOS_AUDIO_I2S_BCLK_GPIO,
            .ws   = CONFIG_MOS_AUDIO_I2S_WS_GPIO,
            .dout = CONFIG_MOS_AUDIO_I2S_DOUT_GPIO,
            .din  = CONFIG_MOS_AUDIO_I2S_DIN_GPIO,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };

    ESP_RETURN_ON_ERROR(
        i2s_channel_init_std_mode(s_audio.i2s_tx, &std_cfg),
        TAG, "i2s_init TX");
    ESP_RETURN_ON_ERROR(
        i2s_channel_init_std_mode(s_audio.i2s_rx, &std_cfg),
        TAG, "i2s_init RX");
    ESP_RETURN_ON_ERROR(
        i2s_channel_enable(s_audio.i2s_tx), TAG, "i2s_enable TX");
    ESP_RETURN_ON_ERROR(
        i2s_channel_enable(s_audio.i2s_rx), TAG, "i2s_enable RX");

    /* Wait for MCLK to stabilise before touching ES8311 over I2C.
     * The ES8311 requires MCLK to be running before it will ACK any
     * I2C transaction.  The I2S peripheral starts outputting MCLK as
     * soon as the channel is enabled; allow a few ms for the ES8311
     * internal oscillator to lock onto it. */
    vTaskDelay(pdMS_TO_TICKS(20));

    /* --- esp_codec_dev data interface -------------------------------- */
    audio_codec_i2s_cfg_t i2s_cfg = {
        .port      = CONFIG_MOS_AUDIO_I2S_NUM,
        .tx_handle = s_audio.i2s_tx,
        .rx_handle = s_audio.i2s_rx,
    };
    s_audio.data_if = audio_codec_new_i2s_data(&i2s_cfg);
    ESP_RETURN_ON_FALSE(s_audio.data_if, ESP_ERR_NO_MEM,
                        TAG, "audio_codec_new_i2s_data");

    /* --- esp_codec_dev control interface (I2C → ES8311) -------------- */
    const audio_codec_gpio_if_t *gpio_if = audio_codec_new_gpio();
    ESP_RETURN_ON_FALSE(gpio_if, ESP_ERR_NO_MEM, TAG, "audio_codec_new_gpio");

    audio_codec_i2c_cfg_t i2c_cfg = {
        .port       = CONFIG_MOS_AUDIO_I2C_PORT,
        .addr       = ES8311_CODEC_DEFAULT_ADDR,  /* 0x18 */
        .bus_handle = bus,
    };
    const audio_codec_ctrl_if_t *i2c_ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    ESP_RETURN_ON_FALSE(i2c_ctrl_if, ESP_ERR_NO_MEM, TAG, "audio_codec_new_i2c_ctrl");

    /* --- ES8311 codec interface --------------------------------------- */
    esp_codec_dev_hw_gain_t gain = {
        .pa_voltage      = 5.0f,
        .codec_dac_voltage = 3.3f,
    };
    es8311_codec_cfg_t es8311_cfg = {
        .ctrl_if      = i2c_ctrl_if,
        .gpio_if      = gpio_if,
        .codec_mode   = ESP_CODEC_DEV_TYPE_OUT,
        .pa_pin       = CONFIG_MOS_AUDIO_PA_GPIO,   /* GPIO53, active HIGH */
        .pa_reverted  = false,
        .master_mode  = false,
        .use_mclk     = true,
        .digital_mic  = false,
        .invert_mclk  = false,
        .invert_sclk  = false,
        .hw_gain      = gain,
    };
    const audio_codec_if_t *es8311_dev = es8311_codec_new(&es8311_cfg);
    ESP_RETURN_ON_FALSE(es8311_dev, ESP_ERR_NO_MEM, TAG, "es8311_codec_new");

    /* --- esp_codec_dev device handle --------------------------------- */
    esp_codec_dev_cfg_t dev_cfg = {
        .dev_type  = ESP_CODEC_DEV_TYPE_IN_OUT,
        .codec_if  = es8311_dev,
        .data_if   = s_audio.data_if,
    };
    s_audio.spk = esp_codec_dev_new(&dev_cfg);
    ESP_RETURN_ON_FALSE(s_audio.spk, ESP_ERR_NO_MEM, TAG, "esp_codec_dev_new");

    /* --- Open at target sample rate ---------------------------------- */
    esp_codec_dev_sample_info_t fs = {
        .sample_rate     = MOS_AUDIO_SAMPLE_RATE,
        .channel         = MOS_AUDIO_CHANNELS,
        .bits_per_sample = MOS_AUDIO_BITS,
    };
    ESP_RETURN_ON_ERROR(
        esp_codec_dev_open(s_audio.spk, &fs),
        TAG, "esp_codec_dev_open");

    ESP_RETURN_ON_ERROR(
        esp_codec_dev_set_out_vol(s_audio.spk, 70),
        TAG, "set_out_vol");

    s_audio.initialized = true;
    ESP_LOGI(TAG, "mos_audio initialized: %d Hz, %dch, %d-bit",
             MOS_AUDIO_SAMPLE_RATE, MOS_AUDIO_CHANNELS, MOS_AUDIO_BITS);
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/* Power-on helper (GPIO6 CODEC_PWR_DIS#, active LOW)                  */
/* ------------------------------------------------------------------ */
static esp_err_t codec_power_on(void)
{
#if CONFIG_MOS_AUDIO_PWR_GPIO >= 0
    gpio_config_t pwr_cfg = {
        .pin_bit_mask = (1ULL << CONFIG_MOS_AUDIO_PWR_GPIO),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&pwr_cfg), TAG, "pwr gpio config");
    gpio_set_level(CONFIG_MOS_AUDIO_PWR_GPIO, 0);   /* active LOW = power ON */
    vTaskDelay(pdMS_TO_TICKS(50));                  /* ≥20ms per ES8311 datasheet */
    ESP_LOGI(TAG, "Codec power ON (GPIO%d=LOW)", CONFIG_MOS_AUDIO_PWR_GPIO);
#endif
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/* Public API                                                           */
/* ------------------------------------------------------------------ */

esp_err_t mos_audio_init(void)
{
    ESP_RETURN_ON_FALSE(!s_audio.initialized, ESP_ERR_INVALID_STATE,
                        TAG, "already initialized");

    ESP_RETURN_ON_ERROR(codec_power_on(), TAG, "codec_power_on");

    /* Create I2C bus */
    i2c_master_bus_handle_t bus = NULL;
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port          = CONFIG_MOS_AUDIO_I2C_PORT,
        .sda_io_num        = CONFIG_MOS_AUDIO_I2C_SDA_GPIO,
        .scl_io_num        = CONFIG_MOS_AUDIO_I2C_SCL_GPIO,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_RETURN_ON_ERROR(
        i2c_new_master_bus(&bus_cfg, &bus),
        TAG, "i2c_new_master_bus");
    s_audio.bus_owned = true;

    esp_err_t ret = audio_init_common(bus);
    if (ret != ESP_OK) {
        i2c_del_master_bus(bus);
        s_audio.bus_owned = false;
    }
    return ret;
}

esp_err_t mos_audio_init_with_bus(i2c_master_bus_handle_t bus)
{
    ESP_RETURN_ON_FALSE(!s_audio.initialized, ESP_ERR_INVALID_STATE,
                        TAG, "already initialized");
    ESP_RETURN_ON_FALSE(bus != NULL, ESP_ERR_INVALID_ARG,
                        TAG, "bus handle is NULL");

    ESP_RETURN_ON_ERROR(codec_power_on(), TAG, "codec_power_on");

    s_audio.bus_owned = false;  /* caller owns the bus */
    return audio_init_common(bus);
}

int mos_audio_write(const int16_t *samples, size_t num_samples, uint32_t timeout_ms)
{
    if (!s_audio.initialized || !samples || num_samples == 0) return -1;

    /* esp_codec_dev_write takes byte count, returns ESP_CODEC_DEV_OK (0) on success */
    int ret = esp_codec_dev_write(s_audio.spk,
                                  (void *)samples,
                                  (int)(num_samples * sizeof(int16_t)));
    if (ret != ESP_CODEC_DEV_OK) return -1;
    return (int)num_samples;
}

esp_err_t mos_audio_set_volume(int volume)
{
    if (!s_audio.initialized) return ESP_ERR_INVALID_STATE;
    if (volume < 0)   volume = 0;
    if (volume > 100) volume = 100;
    int ret = esp_codec_dev_set_out_vol(s_audio.spk, volume);
    return (ret == ESP_CODEC_DEV_OK) ? ESP_OK : ESP_FAIL;
}

i2s_chan_handle_t mos_audio_get_tx_handle(void)
{
    return s_audio.initialized ? s_audio.i2s_tx : NULL;
}

void mos_audio_deinit(void)
{
    if (!s_audio.initialized) return;

    esp_codec_dev_close(s_audio.spk);
    esp_codec_dev_delete(s_audio.spk);
    s_audio.spk = NULL;

    i2s_channel_disable(s_audio.i2s_tx);
    i2s_channel_disable(s_audio.i2s_rx);
    i2s_del_channel(s_audio.i2s_tx);
    i2s_del_channel(s_audio.i2s_rx);

#if CONFIG_MOS_AUDIO_PWR_GPIO >= 0
    gpio_set_level(CONFIG_MOS_AUDIO_PWR_GPIO, 1);   /* active HIGH = power OFF */
#endif

    memset(&s_audio, 0, sizeof(s_audio));
    ESP_LOGI(TAG, "mos_audio de-initialized");
}
