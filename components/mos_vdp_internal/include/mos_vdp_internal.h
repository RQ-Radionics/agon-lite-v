/*
 * mos_vdp_internal.h — Internal VDP: Agon video commands → PSRAM framebuffer → HDMI
 *
 * This component implements an Agon-compatible VDP (Video Display Processor)
 * that renders directly to an RGB888 framebuffer in PSRAM, outputting through
 * the ESP32-P4 DPI controller → LT8912B MIPI DSI bridge → HDMI.
 *
 * It replaces the external TCP-based VDP (mos_vdp) for the Olimex ESP32-P4-PC
 * board which has HDMI output and a USB keyboard.
 *
 * Architecture:
 *   - Caller feeds VDU bytes (from MOS shell output) via mos_vdp_internal_putch()
 *   - Keyboard input arrives as PS/2 Set 2 scancodes via mos_vdp_internal_send_scancode()
 *   - An internal FreeRTOS task processes VDU commands and updates the framebuffer
 *   - The DPI controller DMA-streams the framebuffer to the LT8912B at 60Hz
 *
 * Supported VDU commands (Agon compatible):
 *   VDU 4  — text at text cursor
 *   VDU 5  — text at graphics cursor
 *   VDU 7  — beep (ignored)
 *   VDU 8  — backspace
 *   VDU 9  — tab (horizontal)
 *   VDU 10 — line feed
 *   VDU 11 — vertical tab (up)
 *   VDU 12 — CLS (clear screen)
 *   VDU 13 — carriage return
 *   VDU 14 — page mode on
 *   VDU 15 — page mode off
 *   VDU 16 — CLG (clear graphics)
 *   VDU 17 — COLOUR n (text colour)
 *   VDU 18 — GCOL mode, colour
 *   VDU 19 — define colour (palette)
 *   VDU 22 — MODE n (screen mode)
 *   VDU 23 — redefine character / system commands
 *   VDU 25 — PLOT k, x, y
 *   VDU 28 — text viewport
 *   VDU 29 — graphics origin
 *   VDU 30 — home cursor
 *   VDU 31 — cursor to x, y
 *   Printable ASCII 32-126 — draw character at cursor
 *
 * Board: Olimex ESP32-P4-PC
 *   DPI panel: 640×480 RGB888, double-buffered, ~1.8 MB PSRAM per buffer
 */

#pragma once

#include "esp_err.h"
#include "esp_lcd_panel_ops.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * mos_vdp_internal_init — start the internal VDP.
 *
 * @param dpi_panel  DPI panel handle returned by hdmi_init() / esp_lcd_new_panel_dpi().
 *                   The panel must already be enabled (esp_lcd_panel_init +
 *                   esp_lcd_panel_disp_on_off called). Pass NULL to disable
 *                   framebuffer output (useful for testing without hardware).
 * @return ESP_OK on success.
 */
esp_err_t mos_vdp_internal_init(esp_lcd_panel_handle_t dpi_panel);

/*
 * mos_vdp_internal_putch — feed one VDU byte to the internal VDP.
 *
 * Thread-safe. Can be called from any task. Bytes are queued and
 * processed by the render task.
 */
void mos_vdp_internal_putch(uint8_t c);

/*
 * mos_vdp_internal_getch — read one ASCII character from the keyboard.
 *
 * Blocks until a key is available. Returns -1 if the VDP is shutting down.
 * This is the equivalent of mos_vdp_getch() for the internal VDP.
 */
int mos_vdp_internal_getch(void);

/*
 * mos_vdp_internal_kbhit — returns true if a key is available.
 */
bool mos_vdp_internal_kbhit(void);

/*
 * mos_vdp_internal_send_scancode — feed a PS/2 Set 2 scancode byte.
 *
 * Called from mos_kbd callback for each byte of a PS/2 Set 2 sequence.
 * The VDP translates scancodes to ASCII + virtual key codes and generates
 * Agon KEY packets for the MOS layer.
 *
 * Thread-safe. Can be called from the USB host task (core 0).
 */
void mos_vdp_internal_send_scancode(uint8_t byte);

/*
 * mos_vdp_internal_connected — always true once init succeeds.
 *
 * Replaces mos_vdp_connected() when using the internal VDP.
 */
bool mos_vdp_internal_connected(void);

/*
 * mos_vdp_internal_flush — flush any pending output to the framebuffer.
 *
 * No-op for the internal VDP (rendering is synchronous), provided for
 * API compatibility with mos_vdp_flush().
 */
void mos_vdp_internal_flush(void);

#ifdef __cplusplus
}
#endif
