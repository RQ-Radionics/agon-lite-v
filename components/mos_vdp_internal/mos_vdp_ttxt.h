/*
 * mos_vdp_ttxt.h — Teletext (mode 7) backend for ESP32-P4 VDP internal
 *
 * Implements Agon mode 7 character-cell rendering:
 *   40 columns × 25 rows, 16×20 px cells, RGB888 framebuffer.
 *
 * Public API — called from mos_vdp_internal.cpp
 */

#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Teletext colour RGB888 values (6-bit palette: RRGGBB in 2-bit fields) */
#define TTXT_BLACK    0x00
#define TTXT_RED      0x30  /* R2 */
#define TTXT_GREEN    0x0C  /* G2 */
#define TTXT_YELLOW   0x3C  /* R2+G2 */
#define TTXT_BLUE     0x03  /* B2 */
#define TTXT_MAGENTA  0x33  /* R2+B2 */
#define TTXT_CYAN     0x0F  /* G2+B2 */
#define TTXT_WHITE    0x3F  /* R2+G2+B2 */

/* State flags */
#define TTXT_FLAG_FLASH    0x01
#define TTXT_FLAG_CONCEAL  0x02
#define TTXT_FLAG_HEIGHT   0x04  /* double-height active on this row */
#define TTXT_FLAG_GRAPH    0x08  /* graphics mode */
#define TTXT_FLAG_SEPARATE 0x10  /* separated block graphics */
#define TTXT_FLAG_HOLD     0x20  /* hold graphics */
#define TTXT_FLAG_DHLOW    0x40  /* we are on the lower row of a double-height pair */

/* A character is a control code if bits 6 and 5 are both zero */
#define TTXT_IS_CONTROL(c)  (((c) & 0x60) == 0)

/* Screen dimensions */
#define TTXT_COLS   40
#define TTXT_ROWS   25
#define TTXT_CELL_W 16

/*
 * ttxt_init — initialise the teletext engine.
 *
 * fb        : pointer to the RGB888 framebuffer (FB_W × FB_H × 3 bytes)
 * fb_w      : framebuffer width in pixels (must be >= TTXT_COLS * TTXT_CELL_W)
 * fb_h      : framebuffer height in pixels (used to choose cell height: 20 if >=500, else 19)
 *
 * Returns 0 on success, -1 on allocation failure.
 * Must be called each time mode 7 is entered (re-initialises all state).
 */
int  ttxt_init(uint8_t *fb, int fb_w, int fb_h);

/*
 * ttxt_deinit — free PSRAM allocations.  Call when leaving mode 7.
 */
void ttxt_deinit(void);

/*
 * ttxt_draw_char — write character c to the teletext screen at pixel position
 * (px, py).  px/py are rounded down to the nearest cell boundary.
 *
 * c is a raw byte as received from the VDU stream (ASCII or control code).
 * The function updates the screen buffer and redraws the affected cell(s).
 */
void ttxt_draw_char(int px, int py, uint8_t c);

/*
 * ttxt_get_char — return the raw byte stored at pixel position (px, py).
 */
uint8_t ttxt_get_char(int px, int py);

/*
 * ttxt_scroll — scroll the teletext window up by one row.
 */
void ttxt_scroll(void);

/*
 * ttxt_cls — clear the teletext screen (fill with spaces).
 */
void ttxt_cls(void);

/*
 * ttxt_flash — update flashing characters.
 * Call with phase=true to show, phase=false to hide flashing text.
 * Typically called at 1 Hz (every ~60 frames) by the timer callback.
 */
void ttxt_flash(bool phase);

/*
 * ttxt_repaint — redraw the entire screen from the screen buffer.
 * Used after a framebuffer switch or mode re-entry.
 */
void ttxt_repaint(void);

/*
 * ttxt_cell_w / ttxt_cell_h — current cell dimensions in pixels.
 * Valid after ttxt_init().
 */
int ttxt_cell_w(void);
int ttxt_cell_h(void);

#ifdef __cplusplus
}
#endif
