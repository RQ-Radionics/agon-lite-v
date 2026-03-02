/*
 * esp_lcd_lt8912b.h — LT8912B MIPI DSI → HDMI bridge driver
 *
 * Lontium LT8912B: 2-lane MIPI DSI input, HDMI/DVI output.
 * Fixed output resolution: 640×480@60Hz (VESA standard timing).
 *
 * Hardware (Olimex ESP32-P4-PC Rev B):
 *   I2C1  SDA=GPIO23  SCL=GPIO22  (400 kHz)
 *   I2C addresses: 0x48 (main), 0x49 (MIPI/DDS), 0x4A (audio/AVI, unused)
 *   HPD input:  GPIO15  (HDMI hot-plug detect, active HIGH)
 *   DSI:        2 lanes, DATA0 + CLK, dedicated P4 DSI pads
 *   No reset GPIO (chip resets with ESP EN line)
 *
 * Usage:
 *   1. Create DSI bus with esp_lcd_new_dsi_bus()
 *   2. Call esp_lcd_lt8912b_init() to configure the chip via I2C
 *   3. Create DPI panel with esp_lcd_new_panel_dpi()
 *   4. Enable the panel with esp_lcd_panel_init() + esp_lcd_panel_disp_on_off()
 *
 * Reference:
 *   Linux kernel driver: drivers/gpu/drm/bridge/lontium-lt8912b.c
 *   Sequence: upstream torvalds/linux (reviewed, authoritative)
 */

#pragma once

#include "esp_err.h"
#include "esp_lcd_types.h"
#include "esp_lcd_mipi_dsi.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief LT8912B init configuration
 *
 * Pass this to esp_lcd_lt8912b_init().  The DSI bus must already be
 * created before calling init — the LT8912B I2C config is independent
 * of the DSI bus handle but must be configured before the DPI panel is
 * started.
 */
typedef struct {
    int  i2c_port;      /**< I2C port number (typically I2C_NUM_1) */
    int  sda_gpio;      /**< I2C SDA GPIO */
    int  scl_gpio;      /**< I2C SCL GPIO */
    int  hpd_gpio;      /**< HDMI HPD input GPIO, -1 to disable */
    bool hdmi_mode;     /**< true = HDMI output, false = DVI */
} esp_lcd_lt8912b_config_t;

/**
 * @brief Initialize the LT8912B chip via I2C.
 *
 * Verifies chip ID, then programs all registers for 640×480@60Hz output
 * via 2-lane MIPI DSI input.  Keeps the I2C bus open for HPD polling.
 *
 * Must be called after esp_lcd_new_dsi_bus() and before
 * esp_lcd_new_panel_dpi() / esp_lcd_panel_init().
 *
 * @param  config  LT8912B configuration (I2C port, GPIOs, mode)
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if chip not detected,
 *         or an esp_err_t from the I2C driver on communication error.
 */
esp_err_t esp_lcd_lt8912b_init(const esp_lcd_lt8912b_config_t *config);

/**
 * @brief Check whether an HDMI cable is connected.
 *
 * Reads the HPD status bit from the LT8912B (reg 0x48:0xC1 bit7).
 * If HPD GPIO monitoring is enabled this also reflects the GPIO state.
 *
 * @return true if HDMI cable detected, false otherwise or if not initialized.
 */
bool esp_lcd_lt8912b_is_connected(void);

/**
 * @brief Disable EoTP (End-of-Transmission Packet) on the DSI host.
 *
 * The LT8912B does NOT tolerate MIPI DSI EoTP packets — they confuse
 * the chip's MIPI receiver and prevent video output.  IDF enables EoTP
 * by default in esp_lcd_new_dsi_bus().
 *
 * Call this IMMEDIATELY after esp_lcd_new_dsi_bus() and BEFORE calling
 * esp_lcd_lt8912b_init() or esp_lcd_new_panel_dpi().
 *
 * This function accesses the private IDF DSI bus structure to reach the
 * LL HAL — it is tested against IDF v5.4.  If IDF internals change,
 * the build will fail at the struct member access, making the breakage
 * obvious.
 *
 * @param  dsi_bus  DSI bus handle returned by esp_lcd_new_dsi_bus()
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if dsi_bus is NULL
 */
esp_err_t esp_lcd_lt8912b_patch_dsi_eotp(esp_lcd_dsi_bus_handle_t dsi_bus);

/**
 * @brief De-initialize and release I2C resources.
 *
 * Call this only when tearing down the display subsystem entirely.
 */
void esp_lcd_lt8912b_deinit(void);

#ifdef __cplusplus
}
#endif
