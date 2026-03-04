/*
 * mos_kbd.h — USB HID keyboard driver for ESP32-MOS
 *
 * Drives a USB keyboard connected via a FE1.1s USB hub using the ESP32
 * USB host stack (PHY0 / OTG20 High-Speed, GPIO24/25).
 *
 * Output: PS/2 Set 2 scancodes delivered via a registered callback.
 * Extended keys (arrow keys, Home, End, PgUp, PgDn, Ins, Del, etc.)
 * are prefixed with 0xE0 as per the PS/2 Set 2 specification.
 *
 * Break (key-release) codes are the make code prefixed with 0xF0
 * (or 0xE0 0xF0 for extended keys).
 *
 * Board: Olimex ESP32-P4-PC
 *   HUB_RST#: GPIO21 (active LOW — drive HIGH to enable hub)
 *   USB D-:   GPIO24 (OTG20 HS PHY0, same as production test)
 *   USB D+:   GPIO25 (OTG20 HS PHY0, same as production test)
 */

#pragma once

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * mos_kbd_scancode_cb_t
 *
 * Called from the USB event task (core 0) for each PS/2 Set 2 byte.
 * Multi-byte sequences (extended make: 0xE0 xx, extended break: 0xE0 0xF0 xx,
 * normal break: 0xF0 xx) are delivered as individual bytes in order.
 *
 * The callback must be fast and non-blocking.  Do not call mos_kbd_init()
 * or mos_kbd_deinit() from within the callback.
 */
typedef void (*mos_kbd_scancode_cb_t)(uint8_t byte);

/*
 * mos_kbd_init — start the USB HID keyboard driver.
 *
 * Deasserts HUB_RST# (GPIO21 HIGH), waits 100 ms for hub enumerate,
 * installs the USB host stack on PHY0 (peripheral_map = 0), and
 * starts the HID host driver.  All USB tasks are pinned to core 0.
 *
 * @param cb  Callback invoked for each PS/2 Set 2 scancode byte.
 *            Must not be NULL.
 * @return    ESP_OK on success, or an esp_err_t on failure.
 */
esp_err_t mos_kbd_init(mos_kbd_scancode_cb_t cb);

/*
 * mos_kbd_deinit — stop the USB HID keyboard driver and release resources.
 * Safe to call even if mos_kbd_init() failed.
 */
void mos_kbd_deinit(void);

#ifdef __cplusplus
}
#endif
