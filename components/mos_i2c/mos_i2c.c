/*
 * mos_i2c.c - I2C driver for ESP32-MOS
 *
 * Replaces the eZ80 hardware I2C ISR (i2c.c) with the ESP-IDF i2c_master
 * driver (new API introduced in ESP-IDF v5.x).
 *
 * Public interface is identical to the original agon-mos i2c.h:
 *   mos_I2C_OPEN(frequency)
 *   mos_I2C_CLOSE()
 *   mos_I2C_WRITE(address, size, buf)
 *   mos_I2C_READ(address, size, buf)
 *
 * Thread safety: the ESP-IDF i2c_master driver serialises transactions
 * internally, so calls from different tasks are safe.
 */

#include "mos_i2c.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "mos_i2c";

/* ── Internal state ─────────────────────────────────────────────────────── */

static i2c_master_bus_handle_t s_bus_handle  = NULL;
static bool                    s_open        = false;
static int                     s_sda_gpio    = MOS_I2C_SDA_GPIO;
static int                     s_scl_gpio    = MOS_I2C_SCL_GPIO;

/* Timeout for a single transaction (ms) */
#define I2C_TIMEOUT_MS  2000

/* Maximum bytes per transaction (matches original I2C_MAX_BUFFERLENGTH) */
#define I2C_MAX_BYTES   32

/* ── Speed mapping ──────────────────────────────────────────────────────── */

static uint32_t speed_to_hz(uint8_t id)
{
    switch (id) {
        case MOS_I2C_SPEED_115200: return 115200;
        case MOS_I2C_SPEED_230400: return 230400;
        case MOS_I2C_SPEED_57600:  /* fall through */
        default:                   return 57600;
    }
}

/* ── ESP-IDF esp_err → MOS return code ─────────────────────────────────── */

static uint8_t esp_err_to_mos(esp_err_t err)
{
    switch (err) {
        case ESP_OK:                return MOS_I2C_OK;
        case ESP_ERR_TIMEOUT:       return MOS_I2C_ARB_LOST;
        case ESP_ERR_NOT_FOUND:     return MOS_I2C_NORESPONSE;
        case ESP_FAIL:              /* fall through */
        default:                    return MOS_I2C_BUS_ERROR;
    }
}

/* ── Public: set GPIO pins (must be called before mos_I2C_OPEN) ─────────── */

void mos_i2c_set_pins(int sda_gpio, int scl_gpio)
{
    s_sda_gpio = sda_gpio;
    s_scl_gpio = scl_gpio;
}

/* ── mos_I2C_OPEN ───────────────────────────────────────────────────────── */

void mos_I2C_OPEN(uint8_t frequency)
{
    if (s_open) {
        /* Already open — close first, then re-open with new frequency */
        mos_I2C_CLOSE();
    }

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port            = I2C_NUM_0,
        .sda_io_num          = s_sda_gpio,
        .scl_io_num          = s_scl_gpio,
        .clk_source          = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt   = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t err = i2c_new_master_bus(&bus_cfg, &s_bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus failed: %s", esp_err_to_name(err));
        s_bus_handle = NULL;
        return;
    }

    ESP_LOGI(TAG, "I2C opened on SDA=%d SCL=%d @ %lu Hz",
             s_sda_gpio, s_scl_gpio, (unsigned long)speed_to_hz(frequency));
    s_open = true;
    /* Store frequency in a global for use in device handles */
    /* (we create per-transaction device handles — see WRITE/READ) */
    (void)frequency; /* actual Hz used at device-handle creation time */
}

/* ── mos_I2C_CLOSE ──────────────────────────────────────────────────────── */

void mos_I2C_CLOSE(void)
{
    if (!s_open || s_bus_handle == NULL) return;

    i2c_del_master_bus(s_bus_handle);
    s_bus_handle = NULL;
    s_open       = false;
    ESP_LOGI(TAG, "I2C closed");
}

/* ── mos_I2C_WRITE ──────────────────────────────────────────────────────── */

uint8_t mos_I2C_WRITE(uint8_t i2c_address, uint8_t size, const char *buffer)
{
    if (!s_open || s_bus_handle == NULL) return MOS_I2C_BUS_ERROR;
    if (i2c_address > 127)              return MOS_I2C_NORESPONSE;
    if (size == 0)                      return MOS_I2C_OK;
    if (size > I2C_MAX_BYTES)           size = I2C_MAX_BYTES;

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = i2c_address,
        .scl_speed_hz    = 100000,   /* 100 kHz — safe default for all devices */
    };

    i2c_master_dev_handle_t dev;
    esp_err_t err = i2c_master_bus_add_device(s_bus_handle, &dev_cfg, &dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WRITE: add_device(0x%02X) failed: %s",
                 i2c_address, esp_err_to_name(err));
        return esp_err_to_mos(err);
    }

    err = i2c_master_transmit(dev, (const uint8_t *)buffer, size,
                              I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGD(TAG, "WRITE 0x%02X: %s", i2c_address, esp_err_to_name(err));
    }

    i2c_master_bus_rm_device(dev);
    return esp_err_to_mos(err);
}

/* ── mos_I2C_READ ───────────────────────────────────────────────────────── */

uint8_t mos_I2C_READ(uint8_t i2c_address, uint8_t size, char *buffer)
{
    if (!s_open || s_bus_handle == NULL) return MOS_I2C_BUS_ERROR;
    if (i2c_address > 127)              return MOS_I2C_NORESPONSE;
    if (size == 0)                      return MOS_I2C_OK;
    if (size > I2C_MAX_BYTES)           size = I2C_MAX_BYTES;

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = i2c_address,
        .scl_speed_hz    = 100000,
    };

    i2c_master_dev_handle_t dev;
    esp_err_t err = i2c_master_bus_add_device(s_bus_handle, &dev_cfg, &dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "READ: add_device(0x%02X) failed: %s",
                 i2c_address, esp_err_to_name(err));
        return esp_err_to_mos(err);
    }

    err = i2c_master_receive(dev, (uint8_t *)buffer, size, I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGD(TAG, "READ 0x%02X: %s", i2c_address, esp_err_to_name(err));
    }

    i2c_master_bus_rm_device(dev);
    return esp_err_to_mos(err);
}
