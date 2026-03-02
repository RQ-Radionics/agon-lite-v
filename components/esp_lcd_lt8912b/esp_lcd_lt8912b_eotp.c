/*
 * esp_lcd_lt8912b_eotp.c — EoTP patch for the LT8912B HDMI bridge
 *
 * This is a SEPARATE compilation unit from esp_lcd_lt8912b.c because
 * mipi_dsi_priv.h (an IDF-internal header) defines:
 *
 *   #define TAG "lcd.dsi"
 *
 * which conflicts with the static TAG definition in the main driver file.
 * By isolating the #include of mipi_dsi_priv.h here, we avoid the macro
 * collision entirely.
 *
 * Function: esp_lcd_lt8912b_patch_dsi_eotp()
 *   Disables EoTP (End-of-Transmission Packet) on the DSI host after
 *   esp_lcd_new_dsi_bus() has been called.  The LT8912B cannot tolerate
 *   EoTP packets — they confuse its MIPI receiver.  IDF enables EoTP by
 *   default in esp_lcd_new_dsi_bus(), so we must undo that here.
 *
 * Implementation:
 *   Access the private esp_lcd_dsi_bus_t struct (via mipi_dsi_priv.h)
 *   to reach hal.host, then call the LL function directly.
 *   If IDF restructures these internals, this file will fail to compile,
 *   making the breakage obvious rather than silently producing bad video.
 */

/* mipi_dsi_priv.h must be included BEFORE any file that defines TAG,
 * because it defines TAG itself.  No other TAG-defining header is
 * included here. */
#include "mipi_dsi_priv.h"
#include "hal/mipi_dsi_host_ll.h"

#include "esp_err.h"
#include "esp_check.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_lt8912b.h"

/* Use the TAG defined by mipi_dsi_priv.h ("lcd.dsi") — it will appear
 * in log output as "lcd.dsi" for this specific log line, which is
 * acceptable since this is patching the DSI host directly. */

esp_err_t esp_lcd_lt8912b_patch_dsi_eotp(esp_lcd_dsi_bus_handle_t dsi_bus)
{
    ESP_RETURN_ON_FALSE(dsi_bus != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "lt8912b: dsi_bus is NULL for EoTP patch");

    /* Cast to private struct — defined in mipi_dsi_priv.h.
     * Layout: { int bus_id; mipi_dsi_hal_context_t hal; esp_pm_lock_handle_t pm_lock; }
     * hal.host is the dsi_host_dev_t* pointing to the DSI host registers. */
    esp_lcd_dsi_bus_t *bus = (esp_lcd_dsi_bus_t *)dsi_bus;
    mipi_dsi_host_ll_enable_tx_eotp(bus->hal.host, false, false);

    ESP_LOGI(TAG, "lt8912b: DSI EoTP disabled (LT8912B requires no EoTP)");
    return ESP_OK;
}
