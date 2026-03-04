/*
 * esp_lcd_lt8912b.c — Lontium LT8912B MIPI DSI → HDMI bridge driver
 *
 * Fixed output: 1024×768@60Hz over HDMI.
 * DSI input:    2-lane RGB888, 1000 Mbps per lane.
 *
 * Register sequence derived from the Olimex ESP32-P4-PC production test BSP:
 *   components/esp-bsp/components/lcd/esp_lcd_lt8912b/esp_lcd_lt8912b.c
 *
 * That BSP uses the esp_lcd_panel_io abstraction, but the register values are
 * identical. We use i2c_master directly (same as our existing code) to avoid
 * pulling in the full panel IO stack.
 *
 * 1024×768@60Hz timing (Olimex BSP header esp_lcd_lt8912b.h):
 *   hact=1024 htotal=1184 hfp=48  hs=32  hbp=80
 *   vact=768  vtotal=790  vfp=3   vs=4   vbp=15
 *   pclk=56 MHz, h_polarity=1(+), v_polarity=0(-)
 *   settle reg 0x11 = 0x10
 *
 * LT8912B I2C sub-addresses (all on the same bus):
 *   0x48 — main  (digital/analog init, HPD status, output path)
 *   0x49 — CEC/DSI  (MIPI config, DDS clock, video timing)
 *   0x4A — AVI (AVI infoframe, audio IIS)
 *
 * EoTP note:
 *   The LT8912B does NOT tolerate MIPI DSI End-of-Transmission Packets.
 *   See esp_lcd_lt8912b_patch_dsi_eotp() — call after esp_lcd_new_dsi_bus().
 *
 * HPD note:
 *   Hardware HPD is read from GPIO (config->hpd_gpio) and confirmed via
 *   I2C register 0x48:0xC1 bit7.  Both must be asserted for "connected".
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"

#include "esp_lcd_lt8912b.h"

static const char *TAG = "lt8912b";

/* ------------------------------------------------------------------ */
/* LT8912B I2C addresses                                               */
/* ------------------------------------------------------------------ */
#define LT8912B_ADDR_MAIN       0x48    /* digital/analog/output       */
#define LT8912B_ADDR_CEC_DSI    0x49    /* MIPI config, DDS, timing    */
#define LT8912B_ADDR_AUDIO      0x4A    /* AVI infoframe, audio IIS    */

/* HPD status register (at ADDR_MAIN) — bit7 = cable connected */
#define LT8912B_REG_HPD_STATUS  0xC1

/* ------------------------------------------------------------------ */
/* Driver state                                                         */
/* ------------------------------------------------------------------ */
typedef struct {
    i2c_master_bus_handle_t  bus;
    i2c_master_dev_handle_t  dev_main;      /* 0x48 */
    i2c_master_dev_handle_t  dev_cec_dsi;   /* 0x49 */
    i2c_master_dev_handle_t  dev_audio;     /* 0x4A */
    int                      hpd_gpio;
    bool                     initialized;
    bool                     bus_owned;
} lt8912b_t;

static lt8912b_t s_lt = { 0 };

/* ------------------------------------------------------------------ */
/* I2C helpers                                                          */
/* ------------------------------------------------------------------ */
static esp_err_t lt_write(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(dev, buf, sizeof(buf), 100);
}

static esp_err_t lt_read(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *val)
{
    return i2c_master_transmit_receive(dev, &reg, 1, val, 1, 100);
}

/* ------------------------------------------------------------------ */
/* Init sequence — copied verbatim from Olimex BSP panel_lt8912b_init  */
/* ------------------------------------------------------------------ */

/* Step 1: Digital clock enable (ADDR_MAIN = 0x48)
 * BSP cmd_digital_clock_en[] — note 0x02 comes first in BSP order    */
static esp_err_t lt8912b_write_digital_clock_en(void)
{
    i2c_master_dev_handle_t m = s_lt.dev_main;
    /* BSP writes 0x02 first (lvds pll reset), then clock enables */
    ESP_RETURN_ON_ERROR(lt_write(m, 0x02, 0xF7), TAG, "dclk 0x02");
    ESP_RETURN_ON_ERROR(lt_write(m, 0x08, 0xFF), TAG, "dclk 0x08");
    ESP_RETURN_ON_ERROR(lt_write(m, 0x09, 0xFF), TAG, "dclk 0x09");
    ESP_RETURN_ON_ERROR(lt_write(m, 0x0A, 0xFF), TAG, "dclk 0x0A");
    ESP_RETURN_ON_ERROR(lt_write(m, 0x0B, 0x7C), TAG, "dclk 0x0B");
    ESP_RETURN_ON_ERROR(lt_write(m, 0x0C, 0xFF), TAG, "dclk 0x0C");
    return ESP_OK;
}

/* Step 2: Tx Analog (ADDR_MAIN = 0x48)
 * BSP cmd_tx_analog[] — 0x31/0x32/0x33 differ from Linux kernel       */
static esp_err_t lt8912b_write_tx_analog(void)
{
    i2c_master_dev_handle_t m = s_lt.dev_main;
    ESP_RETURN_ON_ERROR(lt_write(m, 0x31, 0xE1), TAG, "txan 0x31");
    ESP_RETURN_ON_ERROR(lt_write(m, 0x32, 0xE1), TAG, "txan 0x32");
    ESP_RETURN_ON_ERROR(lt_write(m, 0x33, 0x0C), TAG, "txan 0x33");
    ESP_RETURN_ON_ERROR(lt_write(m, 0x37, 0x00), TAG, "txan 0x37");
    ESP_RETURN_ON_ERROR(lt_write(m, 0x38, 0x22), TAG, "txan 0x38");
    ESP_RETURN_ON_ERROR(lt_write(m, 0x60, 0x82), TAG, "txan 0x60");
    return ESP_OK;
}

/* Step 3: Cbus Analog (ADDR_MAIN = 0x48) */
static esp_err_t lt8912b_write_cbus_analog(void)
{
    i2c_master_dev_handle_t m = s_lt.dev_main;
    ESP_RETURN_ON_ERROR(lt_write(m, 0x39, 0x45), TAG, "cbus 0x39");
    ESP_RETURN_ON_ERROR(lt_write(m, 0x3A, 0x00), TAG, "cbus 0x3A");
    ESP_RETURN_ON_ERROR(lt_write(m, 0x3B, 0x00), TAG, "cbus 0x3B");
    return ESP_OK;
}

/* Step 4: HDMI PLL Analog (ADDR_MAIN = 0x48) */
static esp_err_t lt8912b_write_hdmi_pll_analog(void)
{
    i2c_master_dev_handle_t m = s_lt.dev_main;
    ESP_RETURN_ON_ERROR(lt_write(m, 0x44, 0x31), TAG, "hpll 0x44");
    ESP_RETURN_ON_ERROR(lt_write(m, 0x55, 0x44), TAG, "hpll 0x55");
    ESP_RETURN_ON_ERROR(lt_write(m, 0x57, 0x01), TAG, "hpll 0x57");
    ESP_RETURN_ON_ERROR(lt_write(m, 0x5A, 0x02), TAG, "hpll 0x5A");
    return ESP_OK;
}

/* Step 5: MIPI Analog — P/N swap=false (ADDR_MAIN = 0x48) */
static esp_err_t lt8912b_write_mipi_analog(void)
{
    i2c_master_dev_handle_t m = s_lt.dev_main;
    ESP_RETURN_ON_ERROR(lt_write(m, 0x3E, 0xD6), TAG, "man 0x3E"); /* no P/N swap */
    ESP_RETURN_ON_ERROR(lt_write(m, 0x3F, 0xD4), TAG, "man 0x3F");
    ESP_RETURN_ON_ERROR(lt_write(m, 0x41, 0x3C), TAG, "man 0x41");
    return ESP_OK;
}

/* Step 6: MIPI basic set — 2 lanes, settle=0x10, no lane swap
 * (ADDR_CEC_DSI = 0x49)                                               */
static esp_err_t lt8912b_write_mipi_basic(void)
{
    i2c_master_dev_handle_t d = s_lt.dev_cec_dsi;
    ESP_RETURN_ON_ERROR(lt_write(d, 0x10, 0x01), TAG, "mbs 0x10"); /* term en */
    ESP_RETURN_ON_ERROR(lt_write(d, 0x11, 0x10), TAG, "mbs 0x11"); /* settle */
    /* 0x12 (trail) — BSP leaves commented out, skip */
    ESP_RETURN_ON_ERROR(lt_write(d, 0x13, 0x02), TAG, "mbs 0x13"); /* 2 lanes */
    ESP_RETURN_ON_ERROR(lt_write(d, 0x14, 0x00), TAG, "mbs 0x14"); /* debug mux */
    ESP_RETURN_ON_ERROR(lt_write(d, 0x15, 0x00), TAG, "mbs 0x15"); /* no lane swap */
    ESP_RETURN_ON_ERROR(lt_write(d, 0x1A, 0x03), TAG, "mbs 0x1A"); /* hshift 3 */
    ESP_RETURN_ON_ERROR(lt_write(d, 0x1B, 0x03), TAG, "mbs 0x1B"); /* vshift 3 */
    return ESP_OK;
}

/* Step 7: DDS config (ADDR_CEC_DSI = 0x49)
 * BSP uses strm_sw_freq_word = 0x93, 0x3E, 0x29 — NOT the Linux kernel
 * fixed values (0xFF, 0x56, 0x69).  The BSP values are correct.       */
static esp_err_t lt8912b_write_dds_config(void)
{
    i2c_master_dev_handle_t d = s_lt.dev_cec_dsi;

    ESP_RETURN_ON_ERROR(lt_write(d, 0x4E, 0x93), TAG, "dds 4E");
    ESP_RETURN_ON_ERROR(lt_write(d, 0x4F, 0x3E), TAG, "dds 4F");
    ESP_RETURN_ON_ERROR(lt_write(d, 0x50, 0x29), TAG, "dds 50");
    ESP_RETURN_ON_ERROR(lt_write(d, 0x51, 0x80), TAG, "dds 51 arm");
    ESP_RETURN_ON_ERROR(lt_write(d, 0x1E, 0x4F), TAG, "dds 1E");

    ESP_RETURN_ON_ERROR(lt_write(d, 0x1F, 0x5E), TAG, "dds 1F");
    ESP_RETURN_ON_ERROR(lt_write(d, 0x20, 0x01), TAG, "dds 20");
    ESP_RETURN_ON_ERROR(lt_write(d, 0x21, 0x2C), TAG, "dds 21");
    ESP_RETURN_ON_ERROR(lt_write(d, 0x22, 0x01), TAG, "dds 22");
    ESP_RETURN_ON_ERROR(lt_write(d, 0x23, 0xFA), TAG, "dds 23");
    ESP_RETURN_ON_ERROR(lt_write(d, 0x24, 0x00), TAG, "dds 24");
    ESP_RETURN_ON_ERROR(lt_write(d, 0x25, 0xC8), TAG, "dds 25");
    ESP_RETURN_ON_ERROR(lt_write(d, 0x26, 0x00), TAG, "dds 26");
    ESP_RETURN_ON_ERROR(lt_write(d, 0x27, 0x5E), TAG, "dds 27");
    ESP_RETURN_ON_ERROR(lt_write(d, 0x28, 0x01), TAG, "dds 28");
    ESP_RETURN_ON_ERROR(lt_write(d, 0x29, 0x2C), TAG, "dds 29");
    ESP_RETURN_ON_ERROR(lt_write(d, 0x2A, 0x01), TAG, "dds 2A");
    ESP_RETURN_ON_ERROR(lt_write(d, 0x2B, 0xFA), TAG, "dds 2B");
    ESP_RETURN_ON_ERROR(lt_write(d, 0x2C, 0x00), TAG, "dds 2C");
    ESP_RETURN_ON_ERROR(lt_write(d, 0x2D, 0xC8), TAG, "dds 2D");
    ESP_RETURN_ON_ERROR(lt_write(d, 0x2E, 0x00), TAG, "dds 2E");

    ESP_RETURN_ON_ERROR(lt_write(d, 0x42, 0x64), TAG, "dds 42");
    ESP_RETURN_ON_ERROR(lt_write(d, 0x43, 0x00), TAG, "dds 43");
    ESP_RETURN_ON_ERROR(lt_write(d, 0x44, 0x04), TAG, "dds 44");
    ESP_RETURN_ON_ERROR(lt_write(d, 0x45, 0x00), TAG, "dds 45");
    ESP_RETURN_ON_ERROR(lt_write(d, 0x46, 0x59), TAG, "dds 46");
    ESP_RETURN_ON_ERROR(lt_write(d, 0x47, 0x00), TAG, "dds 47");
    ESP_RETURN_ON_ERROR(lt_write(d, 0x48, 0xF2), TAG, "dds 48");
    ESP_RETURN_ON_ERROR(lt_write(d, 0x49, 0x06), TAG, "dds 49");
    ESP_RETURN_ON_ERROR(lt_write(d, 0x4A, 0x00), TAG, "dds 4A");
    ESP_RETURN_ON_ERROR(lt_write(d, 0x4B, 0x72), TAG, "dds 4B");
    ESP_RETURN_ON_ERROR(lt_write(d, 0x4C, 0x45), TAG, "dds 4C");
    ESP_RETURN_ON_ERROR(lt_write(d, 0x4D, 0x00), TAG, "dds 4D");
    ESP_RETURN_ON_ERROR(lt_write(d, 0x52, 0x08), TAG, "dds 52");
    ESP_RETURN_ON_ERROR(lt_write(d, 0x53, 0x00), TAG, "dds 53");
    ESP_RETURN_ON_ERROR(lt_write(d, 0x54, 0xB2), TAG, "dds 54");
    ESP_RETURN_ON_ERROR(lt_write(d, 0x55, 0x00), TAG, "dds 55");
    ESP_RETURN_ON_ERROR(lt_write(d, 0x56, 0xE4), TAG, "dds 56");
    ESP_RETURN_ON_ERROR(lt_write(d, 0x57, 0x0D), TAG, "dds 57");
    ESP_RETURN_ON_ERROR(lt_write(d, 0x58, 0x00), TAG, "dds 58");
    ESP_RETURN_ON_ERROR(lt_write(d, 0x59, 0xE4), TAG, "dds 59");
    ESP_RETURN_ON_ERROR(lt_write(d, 0x5A, 0x8A), TAG, "dds 5A");
    ESP_RETURN_ON_ERROR(lt_write(d, 0x5B, 0x00), TAG, "dds 5B");
    ESP_RETURN_ON_ERROR(lt_write(d, 0x5C, 0x34), TAG, "dds 5C");

    /* Commit: clear 0x51 to latch DDS values */
    ESP_RETURN_ON_ERROR(lt_write(d, 0x51, 0x00), TAG, "dds 51 commit");
    return ESP_OK;
}

/* Step 8: Video timing — 800×600@60Hz (ADDR_CEC_DSI = 0x49)
 *
 * Olimex BSP macro ESP_LCD_LT8912B_VIDEO_TIMING_800x600_60Hz():
 *   hact=800  htotal=1056 hfp=48  hs=128 hbp=88
 *   vact=600  vtotal=628  vfp=1   vs=4   vbp=23
 *   pclk=40 MHz, h_polarity=1(+), v_polarity=1(+)
 *
 * Sync polarity reg 0xAB (ADDR_MAIN):
 *   sync_polarity = h_polarity*0x02 + v_polarity*0x01 = 1*2 + 1*1 = 0x03
 */
static esp_err_t lt8912b_write_video_timing(void)
{
    i2c_master_dev_handle_t d = s_lt.dev_cec_dsi;

    ESP_RETURN_ON_ERROR(lt_write(d, 0x18, 128),           TAG, "vt hs");
    ESP_RETURN_ON_ERROR(lt_write(d, 0x19, 4),             TAG, "vt vs");
    ESP_RETURN_ON_ERROR(lt_write(d, 0x1C, 800 & 0xFF),   TAG, "vt hact_l");
    ESP_RETURN_ON_ERROR(lt_write(d, 0x1D, 800 >> 8),     TAG, "vt hact_h");
    ESP_RETURN_ON_ERROR(lt_write(d, 0x2F, 0x0C),          TAG, "vt fifo");
    ESP_RETURN_ON_ERROR(lt_write(d, 0x34, 1056 & 0xFF),  TAG, "vt htot_l");
    ESP_RETURN_ON_ERROR(lt_write(d, 0x35, 1056 >> 8),    TAG, "vt htot_h");
    ESP_RETURN_ON_ERROR(lt_write(d, 0x36, 628 & 0xFF),   TAG, "vt vtot_l");
    ESP_RETURN_ON_ERROR(lt_write(d, 0x37, 628 >> 8),     TAG, "vt vtot_h");
    ESP_RETURN_ON_ERROR(lt_write(d, 0x38, 23 & 0xFF),    TAG, "vt vbp_l");
    ESP_RETURN_ON_ERROR(lt_write(d, 0x39, 23 >> 8),      TAG, "vt vbp_h");
    ESP_RETURN_ON_ERROR(lt_write(d, 0x3A, 1 & 0xFF),     TAG, "vt vfp_l");
    ESP_RETURN_ON_ERROR(lt_write(d, 0x3B, 1 >> 8),       TAG, "vt vfp_h");
    ESP_RETURN_ON_ERROR(lt_write(d, 0x3C, 88 & 0xFF),    TAG, "vt hbp_l");
    ESP_RETURN_ON_ERROR(lt_write(d, 0x3D, 88 >> 8),      TAG, "vt hbp_h");
    ESP_RETURN_ON_ERROR(lt_write(d, 0x3E, 48 & 0xFF),    TAG, "vt hfp_l");
    ESP_RETURN_ON_ERROR(lt_write(d, 0x3F, 48 >> 8),      TAG, "vt hfp_h");

    /* sync_polarity = h_pol(1)*0x02 + v_pol(1)*0x01 = 0x03 */
    ESP_RETURN_ON_ERROR(lt_write(s_lt.dev_main, 0xAB, 0x03), TAG, "vt pol");
    return ESP_OK;
}

/* Step 9: AVI infoframe (ADDR_AUDIO=0x4A for most, ADDR_MAIN for 0xAB)
 * BSP _panel_lt8912b_send_avi_infoframe() — vic=0, aspect=16:9 (0x02) */
static esp_err_t lt8912b_write_avi_infoframe(void)
{
    i2c_master_dev_handle_t a = s_lt.dev_audio;
    /* enable null package */
    ESP_RETURN_ON_ERROR(lt_write(a, 0x3C, 0x41), TAG, "avi 3C");

    /* sync_polarity already written in video_timing (0xAB=0x02) */

    /* PB0 checksum: pb2=(0x02<<4)+0x08=0x28, pb4=0 → (0x5f-0x28-0)=0x37 */
    ESP_RETURN_ON_ERROR(lt_write(a, 0x43, 0x37), TAG, "avi pb0");
    ESP_RETURN_ON_ERROR(lt_write(a, 0x44, 0x10), TAG, "avi pb1 RGB888");
    ESP_RETURN_ON_ERROR(lt_write(a, 0x45, 0x28), TAG, "avi pb2 16:9");
    ESP_RETURN_ON_ERROR(lt_write(a, 0x46, 0x00), TAG, "avi pb3");
    ESP_RETURN_ON_ERROR(lt_write(a, 0x47, 0x00), TAG, "avi pb4 vic=0");
    return ESP_OK;
}

/* Step 10: MIPI RX logic reset + DDS reset (ADDR_MAIN = 0x48)
 * BSP _panel_lt8912b_mipi_rx_logic_reset() adds DDS reset (0x05) too  */
static esp_err_t lt8912b_write_rxlogicres(void)
{
    i2c_master_dev_handle_t m = s_lt.dev_main;
    /* MIPI RX reset */
    ESP_RETURN_ON_ERROR(lt_write(m, 0x03, 0x7F), TAG, "rxres hold");
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_RETURN_ON_ERROR(lt_write(m, 0x03, 0xFF), TAG, "rxres release");
    /* DDS reset */
    ESP_RETURN_ON_ERROR(lt_write(m, 0x05, 0xFB), TAG, "ddsres hold");
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_RETURN_ON_ERROR(lt_write(m, 0x05, 0xFF), TAG, "ddsres release");
    return ESP_OK;
}

/* Step 11: Audio IIS mode — HDMI mode (ADDR_MAIN = 0x48) */
static esp_err_t lt8912b_write_audio_iis_mode(void)
{
    return lt_write(s_lt.dev_main, 0xB2, 0x01); /* 0x01=HDMI, 0x00=DVI */
}

/* Step 12: Audio IIS enable (ADDR_AUDIO = 0x4A) */
static esp_err_t lt8912b_write_audio_iis_en(void)
{
    i2c_master_dev_handle_t a = s_lt.dev_audio;
    ESP_RETURN_ON_ERROR(lt_write(a, 0x06, 0x08), TAG, "iis 06");
    ESP_RETURN_ON_ERROR(lt_write(a, 0x07, 0xF0), TAG, "iis 07");
    ESP_RETURN_ON_ERROR(lt_write(a, 0x34, 0xD2), TAG, "iis 34");
    ESP_RETURN_ON_ERROR(lt_write(a, 0x0F, 0x2B), TAG, "iis 0F");
    return ESP_OK;
}

/* Step 13: LVDS bypass + core PLL reset (ADDR_MAIN = 0x48)
 * BSP cmd_lvds[] then _panel_lt8912b_lvds_output(false) + hdmi_output(true) */
static esp_err_t lt8912b_write_lvds_config(void)
{
    i2c_master_dev_handle_t m = s_lt.dev_main;

    /* Lvds Power Up */
    ESP_RETURN_ON_ERROR(lt_write(m, 0x44, 0x30), TAG, "lvds 44");
    ESP_RETURN_ON_ERROR(lt_write(m, 0x51, 0x05), TAG, "lvds 51");

    /* Lvds Bypass */
    ESP_RETURN_ON_ERROR(lt_write(m, 0x50, 0x24), TAG, "lvds 50");
    ESP_RETURN_ON_ERROR(lt_write(m, 0x51, 0x2D), TAG, "lvds 51b");
    ESP_RETURN_ON_ERROR(lt_write(m, 0x52, 0x04), TAG, "lvds 52");
    ESP_RETURN_ON_ERROR(lt_write(m, 0x69, 0x0E), TAG, "lvds 69a");
    ESP_RETURN_ON_ERROR(lt_write(m, 0x69, 0x8E), TAG, "lvds 69b");
    ESP_RETURN_ON_ERROR(lt_write(m, 0x6A, 0x00), TAG, "lvds 6A");
    ESP_RETURN_ON_ERROR(lt_write(m, 0x6C, 0xB8), TAG, "lvds 6C");
    ESP_RETURN_ON_ERROR(lt_write(m, 0x6B, 0x51), TAG, "lvds 6B");

    /* Core PLL reset pulse */
    ESP_RETURN_ON_ERROR(lt_write(m, 0x04, 0xFB), TAG, "pll_rst hold");
    ESP_RETURN_ON_ERROR(lt_write(m, 0x04, 0xFF), TAG, "pll_rst release");

    /* Scaler bypass, JEIDA format */
    ESP_RETURN_ON_ERROR(lt_write(m, 0x7F, 0x00), TAG, "lvds 7F");
    ESP_RETURN_ON_ERROR(lt_write(m, 0xA8, 0x13), TAG, "lvds A8");

    /* LVDS output disabled (BSP calls _lvds_output(false)) */
    ESP_RETURN_ON_ERROR(lt_write(m, 0x44, 0x31), TAG, "lvds off");

    /* HDMI output enabled (BSP calls _hdmi_output(true): reg 0x33=0x0e) */
    ESP_RETURN_ON_ERROR(lt_write(m, 0x33, 0x0E), TAG, "hdmi on");

    /* LVDS PLL reset (BSP _lvds_output(on) sequence for final bring-up) */
    ESP_RETURN_ON_ERROR(lt_write(m, 0x02, 0xF7), TAG, "lvds_pll hold");
    ESP_RETURN_ON_ERROR(lt_write(m, 0x02, 0xFF), TAG, "lvds_pll release");

    /* MIPI RX reset (second pulse, finalizes output path) */
    ESP_RETURN_ON_ERROR(lt_write(m, 0x03, 0xCB), TAG, "mipi_rx hold");
    ESP_RETURN_ON_ERROR(lt_write(m, 0x03, 0xFB), TAG, "mipi_rx hold2");
    ESP_RETURN_ON_ERROR(lt_write(m, 0x03, 0xFF), TAG, "mipi_rx release");

    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/* HPD GPIO config                                                      */
/* ------------------------------------------------------------------ */
static void lt8912b_hpd_gpio_init(int gpio)
{
    if (gpio < 0) return;
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << gpio),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
}

/* ------------------------------------------------------------------ */
/* Common init path                                                     */
/* ------------------------------------------------------------------ */
static esp_err_t lt8912b_init_common(int hpd_gpio)
{
    /* Chip ID check — retry up to 3 times with 10 ms delay to handle
     * the case where the chip is still powering up on first boot.
     * Chip ID registers: 0x00 = 0x12 (high), 0x01 = 0xB2 (low).        */
    uint8_t id_h = 0, id_l = 0;
    esp_err_t id_ret = ESP_ERR_TIMEOUT;
    for (int attempt = 0; attempt < 3; attempt++) {
        if (attempt > 0) vTaskDelay(pdMS_TO_TICKS(10));
        if (lt_read(s_lt.dev_main, 0x00, &id_h) == ESP_OK &&
            lt_read(s_lt.dev_main, 0x01, &id_l) == ESP_OK) {
            if (id_h == 0x12 && id_l == 0xB2) {
                id_ret = ESP_OK;
                break;
            }
        }
    }
    if (id_ret != ESP_OK) {
        ESP_LOGE(TAG, "Chip ID mismatch: got 0x%02X%02X, expected 0x12B2", id_h, id_l);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "LT8912B detected (ID 0x%02X%02X)", id_h, id_l);

    ESP_LOGI(TAG, "LT8912B init sequence start (800x600@60Hz HDMI)");

    ESP_RETURN_ON_ERROR(lt8912b_write_digital_clock_en(), TAG, "digital_clock_en");
    ESP_RETURN_ON_ERROR(lt8912b_write_tx_analog(),        TAG, "tx_analog");
    ESP_RETURN_ON_ERROR(lt8912b_write_cbus_analog(),      TAG, "cbus_analog");
    ESP_RETURN_ON_ERROR(lt8912b_write_hdmi_pll_analog(),  TAG, "hdmi_pll_analog");
    ESP_RETURN_ON_ERROR(lt8912b_write_mipi_analog(),      TAG, "mipi_analog");
    ESP_RETURN_ON_ERROR(lt8912b_write_mipi_basic(),       TAG, "mipi_basic");
    ESP_RETURN_ON_ERROR(lt8912b_write_dds_config(),       TAG, "dds_config");
    ESP_RETURN_ON_ERROR(lt8912b_write_video_timing(),     TAG, "video_timing");

    /* BSP writes video setup twice (before and after detect_input_mipi).
     * We skip the MIPI input detect read (it's diagnostic only) and just
     * write the timing registers twice to match the BSP sequence. */
    ESP_RETURN_ON_ERROR(lt8912b_write_video_timing(),     TAG, "video_timing2");

    ESP_RETURN_ON_ERROR(lt8912b_write_avi_infoframe(),    TAG, "avi_infoframe");
    ESP_RETURN_ON_ERROR(lt8912b_write_rxlogicres(),       TAG, "rxlogicres");
    ESP_RETURN_ON_ERROR(lt8912b_write_audio_iis_mode(),   TAG, "audio_iis_mode");
    ESP_RETURN_ON_ERROR(lt8912b_write_audio_iis_en(),     TAG, "audio_iis_en");
    ESP_RETURN_ON_ERROR(lt8912b_write_lvds_config(),      TAG, "lvds_config");

    s_lt.hpd_gpio = hpd_gpio;
    lt8912b_hpd_gpio_init(hpd_gpio);

    s_lt.initialized = true;
    ESP_LOGI(TAG, "LT8912B initialized — 800x600@60Hz HDMI");

    if (esp_lcd_lt8912b_is_connected()) {
        ESP_LOGI(TAG, "HDMI cable connected");
    } else {
        ESP_LOGW(TAG, "HDMI cable not detected");
    }
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/* Public API                                                           */
/* ------------------------------------------------------------------ */
esp_err_t esp_lcd_lt8912b_init(const esp_lcd_lt8912b_config_t *config)
{
    ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TAG, "config is NULL");
    ESP_RETURN_ON_FALSE(!s_lt.initialized, ESP_ERR_INVALID_STATE, TAG, "already initialized");

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port            = config->i2c_port,
        .sda_io_num          = config->sda_gpio,
        .scl_io_num          = config->scl_gpio,
        .clk_source          = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt   = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &s_lt.bus), TAG, "i2c_new_master_bus");
    s_lt.bus_owned = true;

    i2c_device_config_t dev_cfg = { .scl_speed_hz = 400000,
                                    .device_address = LT8912B_ADDR_MAIN };
    if (i2c_master_bus_add_device(s_lt.bus, &dev_cfg, &s_lt.dev_main) != ESP_OK) goto err;
    dev_cfg.device_address = LT8912B_ADDR_CEC_DSI;
    if (i2c_master_bus_add_device(s_lt.bus, &dev_cfg, &s_lt.dev_cec_dsi) != ESP_OK) goto err;
    dev_cfg.device_address = LT8912B_ADDR_AUDIO;
    if (i2c_master_bus_add_device(s_lt.bus, &dev_cfg, &s_lt.dev_audio) != ESP_OK) goto err;

    return lt8912b_init_common(config->hpd_gpio);

err:
    if (s_lt.dev_audio)   i2c_master_bus_rm_device(s_lt.dev_audio);
    if (s_lt.dev_cec_dsi) i2c_master_bus_rm_device(s_lt.dev_cec_dsi);
    if (s_lt.dev_main)    i2c_master_bus_rm_device(s_lt.dev_main);
    i2c_del_master_bus(s_lt.bus);
    memset(&s_lt, 0, sizeof(s_lt));
    return ESP_FAIL;
}

esp_err_t esp_lcd_lt8912b_init_with_bus(i2c_master_bus_handle_t bus,
                                         const esp_lcd_lt8912b_config_t *config)
{
    ESP_RETURN_ON_FALSE(bus    != NULL, ESP_ERR_INVALID_ARG, TAG, "bus is NULL");
    ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TAG, "config is NULL");
    ESP_RETURN_ON_FALSE(!s_lt.initialized, ESP_ERR_INVALID_STATE, TAG, "already initialized");

    s_lt.bus       = bus;
    s_lt.bus_owned = false;

    i2c_device_config_t dev_cfg = { .scl_speed_hz = 400000,
                                    .device_address = LT8912B_ADDR_MAIN };
    if (i2c_master_bus_add_device(s_lt.bus, &dev_cfg, &s_lt.dev_main) != ESP_OK) goto err;
    dev_cfg.device_address = LT8912B_ADDR_CEC_DSI;
    if (i2c_master_bus_add_device(s_lt.bus, &dev_cfg, &s_lt.dev_cec_dsi) != ESP_OK) goto err;
    dev_cfg.device_address = LT8912B_ADDR_AUDIO;
    if (i2c_master_bus_add_device(s_lt.bus, &dev_cfg, &s_lt.dev_audio) != ESP_OK) goto err;

    return lt8912b_init_common(config->hpd_gpio);

err:
    if (s_lt.dev_audio)   i2c_master_bus_rm_device(s_lt.dev_audio);
    if (s_lt.dev_cec_dsi) i2c_master_bus_rm_device(s_lt.dev_cec_dsi);
    if (s_lt.dev_main)    i2c_master_bus_rm_device(s_lt.dev_main);
    memset(&s_lt, 0, sizeof(s_lt));
    return ESP_FAIL;
}

bool esp_lcd_lt8912b_is_connected(void)
{
    if (!s_lt.initialized) return false;
    if (s_lt.hpd_gpio >= 0 && !gpio_get_level(s_lt.hpd_gpio)) return false;
    uint8_t val = 0;
    if (lt_read(s_lt.dev_main, LT8912B_REG_HPD_STATUS, &val) != ESP_OK) return false;
    return (val & 0x80) != 0;
}

i2c_master_bus_handle_t esp_lcd_lt8912b_get_i2c_bus(void)
{
    return s_lt.initialized ? s_lt.bus : NULL;
}

void esp_lcd_lt8912b_deinit(void)
{
    if (!s_lt.initialized) return;
    if (s_lt.dev_audio)   i2c_master_bus_rm_device(s_lt.dev_audio);
    if (s_lt.dev_cec_dsi) i2c_master_bus_rm_device(s_lt.dev_cec_dsi);
    if (s_lt.dev_main)    i2c_master_bus_rm_device(s_lt.dev_main);
    if (s_lt.bus_owned && s_lt.bus) i2c_del_master_bus(s_lt.bus);
    memset(&s_lt, 0, sizeof(s_lt));
    ESP_LOGI(TAG, "LT8912B de-initialized");
}

/* esp_lcd_lt8912b_patch_dsi_eotp() is in esp_lcd_lt8912b_eotp.c */
