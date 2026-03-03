/*
 * mos_vdp_internal.cpp — Internal VDP: Agon VDU commands → PSRAM framebuffer → HDMI
 *
 * Renders Agon-compatible VDU commands to an RGB888 double-buffered framebuffer
 * in PSRAM, outputting through the ESP32-P4 DPI controller → LT8912B → HDMI.
 *
 * Supported:
 *   - Text rendering (100×75 chars, 8×8 font, 16 colours, 800×600 mode)
 *   - VDU 4/5/7/8/9/10/11/12/13/14/15/16/17/18/19/22/23/25/28/29/30/31
 *   - PS/2 Set 2 keyboard → ASCII + modifier decoding
 *
 * Board: Olimex ESP32-P4-PC
 * Output: 800×600@60Hz VESA over HDMI via LT8912B DSI bridge
 */

#include "mos_vdp_internal.h"

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_cache.h"

#include "vdp_font.h"

static const char *TAG = "vdp_int";

/* ------------------------------------------------------------------ */
/* Screen geometry                                                      */
/*                                                                      */
/* Physical framebuffer is always 800×600 — the maximum Agon mode.     */
/* All 18 Agon video modes are rendered within this framebuffer; lower  */
/* resolution modes use a sub-region (COLS/ROWS reflect active mode).  */
/* The DPI controller always outputs 800×600@60Hz VESA to the LT8912B. */
/* ------------------------------------------------------------------ */
#define FB_W            800
#define FB_H            600
#define FONT_W          8
#define FONT_H          8
#define BYTES_PER_PIX   3                     /* RGB888 */
#define FB_SIZE         (FB_W * FB_H * BYTES_PER_PIX)  /* 1440000 */

/* Active mode dimensions — updated by VDU 22 (mode switch).
 * Default mode 0 = 640×480 (Agon standard).
 * Maximum mode = 800×600.
 * COLS/ROWS are always derived from the active mode width/height.      */
static int s_mode_w = 640;   /* active mode pixel width  */
static int s_mode_h = 480;   /* active mode pixel height */
#define COLS            (s_mode_w / FONT_W)
#define ROWS            (s_mode_h / FONT_H)

/* ------------------------------------------------------------------ */
/* Agon 16-colour palette (defaultPalette10 mapped through colourLookup)
 *
 * FabGL levels: 0x00, 0x55, 0xAA, 0xFF
 * Palette indices used in mode 0 (defaultPalette10):
 *   0x00 0x20 0x08 0x28 0x02 0x22 0x0A 0x2A  (dark)
 *   0x15 0x30 0x0C 0x3C 0x03 0x33 0x0F 0x3F  (bright)
 *
 * Each entry encodes 2 bits per channel (R5:4, G3:2, B1:0) into 6 bits.
 * We expand: bits[5:4]→R, bits[3:2]→G, bits[1:0]→B, each 2-bit → level.
 */

static const uint8_t s_palette_lookup6[64][3] = {
    /* Index 0x00..0x3F: R = bits[5:4], G = bits[3:2], B = bits[1:0] */
    /* Level map: 0→0x00, 1→0x55, 2→0xAA, 3→0xFF */
#define L(x) ((x) == 0 ? 0x00 : (x) == 1 ? 0x55 : (x) == 2 ? 0xAA : 0xFF)
#define E(i) { L(((i)>>4)&3), L(((i)>>2)&3), L((i)&3) }
    E(0x00),E(0x01),E(0x02),E(0x03),E(0x04),E(0x05),E(0x06),E(0x07),
    E(0x08),E(0x09),E(0x0A),E(0x0B),E(0x0C),E(0x0D),E(0x0E),E(0x0F),
    E(0x10),E(0x11),E(0x12),E(0x13),E(0x14),E(0x15),E(0x16),E(0x17),
    E(0x18),E(0x19),E(0x1A),E(0x1B),E(0x1C),E(0x1D),E(0x1E),E(0x1F),
    E(0x20),E(0x21),E(0x22),E(0x23),E(0x24),E(0x25),E(0x26),E(0x27),
    E(0x28),E(0x29),E(0x2A),E(0x2B),E(0x2C),E(0x2D),E(0x2E),E(0x2F),
    E(0x30),E(0x31),E(0x32),E(0x33),E(0x34),E(0x35),E(0x36),E(0x37),
    E(0x38),E(0x39),E(0x3A),E(0x3B),E(0x3C),E(0x3D),E(0x3E),E(0x3F),
#undef E
#undef L
};

/* defaultPalette10: 16-colour mode 0 colour indices into s_palette_lookup6 */
static const uint8_t s_palette10[16] = {
    0x00, 0x20, 0x08, 0x28, 0x02, 0x22, 0x0A, 0x2A,   /* dark */
    0x15, 0x30, 0x0C, 0x3C, 0x03, 0x33, 0x0F, 0x3F,   /* bright */
};

/* Working palette — 16 entries RGB888; can be redefined via VDU 19 */
typedef struct { uint8_t r, g, b; } rgb888_t;

static rgb888_t s_palette[16];

static void palette_reset(void)
{
    for (int i = 0; i < 16; i++) {
        uint8_t idx = s_palette10[i];
        s_palette[i].r = s_palette_lookup6[idx][0];
        s_palette[i].g = s_palette_lookup6[idx][1];
        s_palette[i].b = s_palette_lookup6[idx][2];
    }
}

/* ------------------------------------------------------------------ */
/* Framebuffer state                                                    */
/*                                                                      */
/* Single-buffer strategy: always render into fbs[0] and call          */
/* esp_lcd_panel_draw_bitmap(fbs[0]) to flush dcache → PSRAM so the    */
/* DPI DMA sees updated pixels.  IDF detects color_data is within      */
/* fbs[0] and does only a cache writeback (no DMA copy, no buf swap).  */
/*                                                                      */
/* Double-buffering is not used: tearing is not visible at character   */
/* granularity, and it avoids the diverging-buffers problem that        */
/* occurs when rendering incrementally into alternating buffers.        */
/* ------------------------------------------------------------------ */
static esp_lcd_panel_handle_t s_panel = NULL;
static uint8_t *s_fb[2] = { NULL, NULL };   /* fb[0] = render target; fb[1] unused */

static SemaphoreHandle_t s_fb_mutex = NULL; /* protects framebuffer access */

static inline void fb_plot_pixel(int x, int y, rgb888_t c)
{
    if ((unsigned)x >= (unsigned)s_mode_w || (unsigned)y >= (unsigned)s_mode_h) return;
    /* Stride is always FB_W (physical framebuffer width = 800) */
    uint8_t *p = s_fb[0] + (y * FB_W + x) * BYTES_PER_PIX;
    p[0] = c.r; p[1] = c.g; p[2] = c.b;
}

static void fb_flush(void)
{
    /* Flush dcache → PSRAM so the DPI DMA sees updated pixels.
     * Pass fbs[0]: IDF detects it's within the panel fb range and does
     * cache writeback + sets cur_fb_index=0. No DMA copy. */
    if (!s_panel || !s_fb[0]) return;
    esp_lcd_panel_draw_bitmap(s_panel, 0, 0, FB_W, FB_H, s_fb[0]);
}

/* ------------------------------------------------------------------ */
/* Text cursor state                                                    */
/* ------------------------------------------------------------------ */
static int  s_col = 0, s_row = 0;
static int  s_fg  = 15;   /* bright white */
static int  s_bg  = 0;    /* black */

/* Viewport — default = full mode area; reset on mode change */
static int s_vp_left = 0, s_vp_top = 0;
static int s_vp_right = 79, s_vp_bottom = 59;  /* updated in mode_set() */

/* ------------------------------------------------------------------ */
/* Low-level character rendering                                        */
/* ------------------------------------------------------------------ */

static void draw_char(int col, int row, uint8_t c)
{
    const uint8_t *glyph = vdp_font_glyph(c);
    int px = col * FONT_W;
    int py = row * FONT_H;
    rgb888_t fg = s_palette[s_fg & 0xF];
    rgb888_t bg = s_palette[s_bg & 0xF];

    for (int r = 0; r < FONT_H; r++) {
        uint8_t bits = glyph[r];
        for (int b = 0; b < FONT_W; b++) {
            fb_plot_pixel(px + b, py + r, (bits & (0x80 >> b)) ? fg : bg);
        }
    }
}

static void clear_screen(void)
{
    rgb888_t bg = s_palette[s_bg & 0xF];
    /* Fill the full physical framebuffer (FB_W × FB_H) with bg colour.
     * Pixels outside the active mode area are black by convention but
     * clearing the full buffer avoids stale pixels at mode boundaries. */
    uint8_t *p = s_fb[0];
    for (int i = 0; i < FB_W * FB_H; i++) {
        p[0] = bg.r; p[1] = bg.g; p[2] = bg.b;
        p += BYTES_PER_PIX;
    }
}

static void scroll_up(void)
{
    /* Scroll viewport region up one character line.
     * Stride is always FB_W (physical framebuffer pitch = 800 pixels). */
    int row_bytes  = FB_W * BYTES_PER_PIX;          /* physical row stride */
    int char_bytes = FONT_H * row_bytes;

    int vp_y0 = s_vp_top  * FONT_H;
    int vp_y1 = (s_vp_bottom + 1) * FONT_H;  /* exclusive */
    int vp_x0 = s_vp_left  * FONT_W;
    int vp_x1 = (s_vp_right + 1) * FONT_W;
    int vp_w  = vp_x1 - vp_x0;

    /* If viewport spans the full mode width we can memmove entire rows
     * (stride = FB_W so rows are contiguous in memory even if mode_w < FB_W,
     * but the extra pixels at the right are also moved — acceptable) */
    if (s_vp_left == 0 && s_vp_right == COLS - 1) {
        uint8_t *dst = s_fb[0] + vp_y0 * row_bytes;
        uint8_t *src = dst + char_bytes;
        int move_bytes = (vp_y1 - vp_y0 - FONT_H) * row_bytes;
        if (move_bytes > 0) memmove(dst, src, move_bytes);
        /* Clear bottom character line (full physical width) */
        rgb888_t bg = s_palette[s_bg & 0xF];
        uint8_t *bot = s_fb[0] + (vp_y1 - FONT_H) * row_bytes;
        for (int i = 0; i < FONT_H * FB_W; i++) {
            bot[0] = bg.r; bot[1] = bg.g; bot[2] = bg.b;
            bot += BYTES_PER_PIX;
        }
    } else {
        /* Partial-width viewport: copy row by row using physical stride */
        for (int y = vp_y0; y < vp_y1 - FONT_H; y++) {
            uint8_t *dst = s_fb[0] + (y * FB_W + vp_x0) * BYTES_PER_PIX;
            uint8_t *src = dst + char_bytes;
            memmove(dst, src, vp_w * BYTES_PER_PIX);
        }
        rgb888_t bg = s_palette[s_bg & 0xF];
        for (int y = vp_y1 - FONT_H; y < vp_y1; y++) {
            uint8_t *p = s_fb[0] + (y * FB_W + vp_x0) * BYTES_PER_PIX;
            for (int x = 0; x < vp_w; x++) {
                p[0] = bg.r; p[1] = bg.g; p[2] = bg.b;
                p += BYTES_PER_PIX;
            }
        }
    }
}

static void cursor_advance(void)
{
    s_col++;
    if (s_col > s_vp_right) {
        s_col = s_vp_left;
        s_row++;
        if (s_row > s_vp_bottom) {
            scroll_up();
            s_row = s_vp_bottom;
        }
    }
}

static void newline(void)
{
    s_row++;
    if (s_row > s_vp_bottom) {
        scroll_up();
        s_row = s_vp_bottom;
    }
}

/* ------------------------------------------------------------------ */
/* VDU command state machine                                            */
/* ------------------------------------------------------------------ */

typedef enum {
    VDU_STATE_NORMAL = 0,
    VDU_STATE_VDU17,        /* expecting colour byte */
    VDU_STATE_VDU18_1,      /* expecting mode byte */
    VDU_STATE_VDU18_2,      /* expecting colour byte */
    VDU_STATE_VDU19_1,      /* palette index */
    VDU_STATE_VDU19_2,      /* physical colour */
    VDU_STATE_VDU19_3,      /* red */
    VDU_STATE_VDU19_4,      /* green */
    VDU_STATE_VDU19_5,      /* blue */
    VDU_STATE_VDU22,        /* expecting mode number */
    VDU_STATE_VDU23_1,      /* expecting first byte of VDU23 command */
    VDU_STATE_VDU23_SKIP,   /* skip remaining bytes of unknown VDU23 */
    VDU_STATE_VDU25_1,      /* PLOT: kind */
    VDU_STATE_VDU25_2,      /* PLOT: x lo */
    VDU_STATE_VDU25_3,      /* PLOT: x hi */
    VDU_STATE_VDU25_4,      /* PLOT: y lo */
    VDU_STATE_VDU25_5,      /* PLOT: y hi */
    VDU_STATE_VDU28_1,      /* text viewport: left */
    VDU_STATE_VDU28_2,
    VDU_STATE_VDU28_3,
    VDU_STATE_VDU28_4,
    VDU_STATE_VDU29_1,      /* graphics origin: x lo */
    VDU_STATE_VDU29_2,
    VDU_STATE_VDU29_3,
    VDU_STATE_VDU29_4,
    VDU_STATE_VDU31_1,      /* cursor tab: col */
    VDU_STATE_VDU31_2,      /* cursor tab: row */
} vdu_state_t;

static vdu_state_t s_vdu_state = VDU_STATE_NORMAL;
static int         s_vdu_skip  = 0;     /* remaining bytes to skip in VDU23_SKIP */

/* Scratch registers for multi-byte commands */
static uint8_t s_arg[8];
static int     s_arg_idx = 0;

/* Process one VDU byte */
static void vdu_process(uint8_t c);

/* ------------------------------------------------------------------ */
/* Queues and tasks                                                     */
/* ------------------------------------------------------------------ */

#define VDU_QUEUE_LEN   4096
#define KEY_QUEUE_LEN   64

static QueueHandle_t s_vdu_queue = NULL;
static QueueHandle_t s_key_queue = NULL;   /* ASCII chars available for getch() */

static void vdp_render_task(void *arg)
{
    uint8_t byte;
    while (1) {
        if (xQueueReceive(s_vdu_queue, &byte, portMAX_DELAY) == pdTRUE) {
            if (s_fb_mutex) xSemaphoreTake(s_fb_mutex, portMAX_DELAY);
            vdu_process(byte);
            if (s_fb_mutex) xSemaphoreGive(s_fb_mutex);
        }
    }
}

/* ------------------------------------------------------------------ */
/* VDU command processor                                                */
/* ------------------------------------------------------------------ */

static void vdu_process(uint8_t c)
{
    switch (s_vdu_state) {

    case VDU_STATE_NORMAL:
        if (c >= 0x20 && c != 0x7F) {
            /* Printable: draw at cursor, advance */
            draw_char(s_col, s_row, c);
            cursor_advance();
            fb_flush();
            return;
        }
        switch (c) {
        case 0x04: /* VDU 4 — text at text cursor (already default) */ break;
        case 0x05: /* VDU 5 — text at graphics cursor (ignored) */     break;
        case 0x07: /* VDU 7 — BEL (ignored) */                         break;
        case 0x08: /* VDU 8 — backspace */
            if (s_col > s_vp_left) {
                s_col--;
            } else if (s_row > s_vp_top) {
                s_row--;
                s_col = s_vp_right;
            }
            break;
        case 0x09: /* VDU 9 — tab */
            s_col = ((s_col / 8) + 1) * 8;
            if (s_col > s_vp_right) { s_col = s_vp_left; newline(); }
            break;
        case 0x0A: /* VDU 10 — LF */
            newline();
            fb_flush();
            break;
        case 0x0C: /* VDU 12 — CLS */
            clear_screen();
             s_col = s_vp_left;
             s_row = s_vp_top;
             fb_flush();
             break;
        case 0x0D: /* VDU 13 — CR */
             s_col = s_vp_left;
             break;
        case 0x0E: /* VDU 14 — page mode on (ignored) */  break;
        case 0x0F: /* VDU 15 — page mode off (ignored) */ break;
        case 0x10: /* VDU 16 — CLG (clear graphics, treat as CLS here) */
            clear_screen();
            fb_flush();
            break;
        case 0x11: /* VDU 17 — COLOUR n */
            s_vdu_state = VDU_STATE_VDU17;
            break;
        case 0x12: /* VDU 18 — GCOL mode, colour */
            s_vdu_state = VDU_STATE_VDU18_1;
            break;
        case 0x13: /* VDU 19 — define colour */
            s_vdu_state = VDU_STATE_VDU19_1;
            s_arg_idx = 0;
            break;
        case 0x16: /* VDU 22 — MODE n */
            s_vdu_state = VDU_STATE_VDU22;
            break;
        case 0x17: /* VDU 23 — redefine char / system commands */
            s_vdu_state = VDU_STATE_VDU23_1;
            s_arg_idx = 0;
            break;
        case 0x19: /* VDU 25 — PLOT */
            s_vdu_state = VDU_STATE_VDU25_1;
            s_arg_idx = 0;
            break;
        case 0x1C: /* VDU 28 — text viewport */
            s_vdu_state = VDU_STATE_VDU28_1;
            s_arg_idx = 0;
            break;
        case 0x1D: /* VDU 29 — graphics origin */
            s_vdu_state = VDU_STATE_VDU29_1;
            s_arg_idx = 0;
            break;
        case 0x1E: /* VDU 30 — cursor home */
            s_col = s_vp_left;
            s_row = s_vp_top;
            break;
        case 0x1F: /* VDU 31 — cursor to x,y */
            s_vdu_state = VDU_STATE_VDU31_1;
            break;
        default:
            break;
        }
        break;

    /* ---- VDU 17: COLOUR n ---- */
    case VDU_STATE_VDU17:
        if (c < 128) {
            s_fg = c & 0xF;
        } else {
            s_bg = (c - 128) & 0xF;
        }
        s_vdu_state = VDU_STATE_NORMAL;
        break;

    /* ---- VDU 18: GCOL mode, colour (2 bytes) ---- */
    case VDU_STATE_VDU18_1:
        /* mode byte: ignore */
        s_vdu_state = VDU_STATE_VDU18_2;
        break;
    case VDU_STATE_VDU18_2:
        /* colour byte: treat as foreground colour */
        s_fg = c & 0xF;
        s_vdu_state = VDU_STATE_NORMAL;
        break;

    /* ---- VDU 19: define colour (5 bytes: index, physical, r, g, b) ---- */
    case VDU_STATE_VDU19_1: s_arg[0] = c; s_vdu_state = VDU_STATE_VDU19_2; break;
    case VDU_STATE_VDU19_2: s_arg[1] = c; s_vdu_state = VDU_STATE_VDU19_3; break;
    case VDU_STATE_VDU19_3: s_arg[2] = c; s_vdu_state = VDU_STATE_VDU19_4; break;
    case VDU_STATE_VDU19_4: s_arg[3] = c; s_vdu_state = VDU_STATE_VDU19_5; break;
    case VDU_STATE_VDU19_5: {
        /* index=arg[0], physical=arg[1], r=arg[2], g=arg[3], b=c */
        int idx = s_arg[0] & 0xF;
        if (s_arg[1] == 16) {
            /* physical=16: use explicit RGB */
            s_palette[idx].r = s_arg[2];
            s_palette[idx].g = s_arg[3];
            s_palette[idx].b = c;
        } else if (s_arg[1] < 64) {
            /* physical = FabGL 6-bit index */
            uint8_t pi = s_arg[1] & 0x3F;
            s_palette[idx].r = s_palette_lookup6[pi][0];
            s_palette[idx].g = s_palette_lookup6[pi][1];
            s_palette[idx].b = s_palette_lookup6[pi][2];
        }
        s_vdu_state = VDU_STATE_NORMAL;
        break;
    }

    /* ---- VDU 22: MODE n ---- */
    case VDU_STATE_VDU22: {
        /* Agon video modes (standard set):
         *   0  = 640×480  16-colour
         *   1  = 640×480   4-colour
         *   2  = 640×480   2-colour
         *   3  = 640×240  64-colour
         *   4  = 640×240  16-colour
         *   5  = 640×240   4-colour
         *   6  = 640×240   2-colour
         *   8  = 320×240  64-colour
         *   9  = 320×240  16-colour
         *  10  = 320×240   4-colour
         *  11  = 320×240   2-colour
         *  12  = 320×200  64-colour
         *  13  = 320×200  16-colour
         *  14  = 320×200   4-colour
         *  15  = 320×200   2-colour
         *  16  = 800×600  16-colour  ← maximum, fills full framebuffer
         *  17  = 800×600   4-colour
         *  18  = 800×600   2-colour
         * All modes are rendered in the 800×600 physical framebuffer.
         * Lower-resolution modes occupy a sub-region (top-left aligned).
         * Colour depth is noted but we always use the 16-colour palette. */
        /* Pixel dimensions for each Agon mode 0..18 */
        static const struct { int w; int h; } mode_dims[] = {
            {640,480},{640,480},{640,480},  /* 0-2  */
            {640,240},{640,240},{640,240},{640,240}, /* 3-6 */
            {640,480},                               /* 7   fallback */
            {320,240},{320,240},{320,240},{320,240}, /* 8-11 */
            {320,200},{320,200},{320,200},{320,200}, /* 12-15 */
            {800,600},{800,600},{800,600},           /* 16-18 */
        };
        int mode = (int)c;
        if (mode < 0 || mode > 18) mode = 0;
        s_mode_w = mode_dims[mode].w;
        s_mode_h = mode_dims[mode].h;
        palette_reset();
        s_fg = 15; s_bg = 0;
        s_vp_left = 0; s_vp_top = 0;
        s_vp_right = COLS - 1; s_vp_bottom = ROWS - 1;
        clear_screen();
        s_col = 0; s_row = 0;
        fb_flush();
        s_vdu_state = VDU_STATE_NORMAL;
        break;
    }

    /* ---- VDU 23: redefine char / system ---- */
    case VDU_STATE_VDU23_1:
        s_arg[0] = c;
        if (c == 0) {
            /* VDU 23,0,... — system command; variable length, just skip 8 more */
            s_vdu_skip = 8;
            s_vdu_state = VDU_STATE_VDU23_SKIP;
        } else {
            /* VDU 23,n — redefine char n: 8 bytes of bitmap follow */
            s_vdu_skip = 8;
            s_vdu_state = VDU_STATE_VDU23_SKIP; /* simplified: skip for now */
        }
        break;
    case VDU_STATE_VDU23_SKIP:
        s_vdu_skip--;
        if (s_vdu_skip == 0) {
            s_vdu_state = VDU_STATE_NORMAL;
        }
        break;

    /* ---- VDU 25: PLOT k, x16, y16 (5 bytes total) ---- */
    case VDU_STATE_VDU25_1: s_arg[0] = c; s_vdu_state = VDU_STATE_VDU25_2; break;
    case VDU_STATE_VDU25_2: s_arg[1] = c; s_vdu_state = VDU_STATE_VDU25_3; break;
    case VDU_STATE_VDU25_3: s_arg[2] = c; s_vdu_state = VDU_STATE_VDU25_4; break;
    case VDU_STATE_VDU25_4: s_arg[3] = c; s_vdu_state = VDU_STATE_VDU25_5; break;
    case VDU_STATE_VDU25_5:
        /* PLOT command — minimal: just plot a single pixel for PLOT 69 (absolute) */
        {
            int16_t x = (int16_t)(s_arg[1] | (s_arg[2] << 8));
            int16_t y = (int16_t)(s_arg[3] | (c << 8));
            /* Agon graphics origin is bottom-left; flip Y */
            int px = (int)x;
            int py = s_mode_h - 1 - (int)y;
            if (px >= 0 && px < s_mode_w && py >= 0 && py < s_mode_h) {
                fb_plot_pixel(px, py, s_palette[s_fg & 0xF]);
                fb_flush();
            }
        }
        s_vdu_state = VDU_STATE_NORMAL;
        break;

    /* ---- VDU 28: text viewport (4 bytes: left, bottom, right, top) ---- */
    case VDU_STATE_VDU28_1: s_arg[0] = c; s_vdu_state = VDU_STATE_VDU28_2; break;
    case VDU_STATE_VDU28_2: s_arg[1] = c; s_vdu_state = VDU_STATE_VDU28_3; break;
    case VDU_STATE_VDU28_3: s_arg[2] = c; s_vdu_state = VDU_STATE_VDU28_4; break;
    case VDU_STATE_VDU28_4:
        /* left=arg[0], bottom=arg[1], right=arg[2], top=c */
        s_vp_left   = s_arg[0]; if (s_vp_left   < 0)        s_vp_left   = 0;
        s_vp_right  = s_arg[2]; if (s_vp_right  >= COLS)    s_vp_right  = COLS - 1;
        s_vp_top    = c;        if (s_vp_top    < 0)        s_vp_top    = 0;
        s_vp_bottom = s_arg[1]; if (s_vp_bottom >= ROWS)    s_vp_bottom = ROWS - 1;
        s_col = s_vp_left;
        s_row = s_vp_top;
        s_vdu_state = VDU_STATE_NORMAL;
        break;

    /* ---- VDU 29: graphics origin (4 bytes, ignored — just consume) ---- */
    case VDU_STATE_VDU29_1: s_vdu_state = VDU_STATE_VDU29_2; break;
    case VDU_STATE_VDU29_2: s_vdu_state = VDU_STATE_VDU29_3; break;
    case VDU_STATE_VDU29_3: s_vdu_state = VDU_STATE_VDU29_4; break;
    case VDU_STATE_VDU29_4: s_vdu_state = VDU_STATE_NORMAL;  break;

    /* ---- VDU 31: cursor to col, row ---- */
    case VDU_STATE_VDU31_1:
        s_col = s_vp_left + c;
        if (s_col > s_vp_right) s_col = s_vp_right;
        s_vdu_state = VDU_STATE_VDU31_2;
        break;
    case VDU_STATE_VDU31_2:
        s_row = s_vp_top + c;
        if (s_row > s_vp_bottom) s_row = s_vp_bottom;
        s_vdu_state = VDU_STATE_NORMAL;
        break;

    default:
        s_vdu_state = VDU_STATE_NORMAL;
        break;
    }
}

/* ------------------------------------------------------------------ */
/* PS/2 Set 2 → ASCII decoder                                          */
/* ------------------------------------------------------------------ */

/*
 * PS/2 Set 2 make codes → ASCII (unshifted).
 * 0x00 = no mapping.  Array indexed by make code 0x00..0x7F.
 */
static const uint8_t s_ps2_unshifted[128] = {
/*00*/  0,    0,    0,    0,    0,    0,    0,    0,
/*08*/  0,    0,    0,    0,    0,    '\t', '`',  0,
/*10*/  0,    0,    0,    0,    0,    'q',  '1',  0,
/*18*/  0,    0,    'z',  's',  'a',  'w',  '2',  0,
/*20*/  0,    'c',  'x',  'd',  'e',  '4',  '3',  0,
/*28*/  0,    ' ',  'v',  'f',  't',  'r',  '5',  0,
/*30*/  0,    'n',  'b',  'h',  'g',  'y',  '6',  0,
/*38*/  0,    0,    'm',  'j',  'u',  '7',  '8',  0,
/*40*/  0,    ',',  'k',  'i',  'o',  '0',  '9',  0,
/*48*/  0,    '.',  '/',  'l',  ';',  'p',  '-',  0,
/*50*/  0,    0,    '\'', 0,    '[',  '=',  0,    0,
/*58*/  0,    0,    '\r', ']',  0,    '\\', 0,    0,
/*60*/  0,    0,    0,    0,    0,    0,    '\b', 0,
/*68*/  0,    '1',  0,    '4',  '7',  0,    0,    0,
/*70*/  '0',  '.',  '2',  '5',  '6',  '8',  27,   0,
/*78*/  0,    '+',  '3',  '-',  '*',  '9',  0,    0,
};

static const uint8_t s_ps2_shifted[128] = {
/*00*/  0,    0,    0,    0,    0,    0,    0,    0,
/*08*/  0,    0,    0,    0,    0,    '\t', '~',  0,
/*10*/  0,    0,    0,    0,    0,    'Q',  '!',  0,
/*18*/  0,    0,    'Z',  'S',  'A',  'W',  '@',  0,
/*20*/  0,    'C',  'X',  'D',  'E',  '$',  '#',  0,
/*28*/  0,    ' ',  'V',  'F',  'T',  'R',  '%',  0,
/*30*/  0,    'N',  'B',  'H',  'G',  'Y',  '^',  0,
/*38*/  0,    0,    'M',  'J',  'U',  '&',  '*',  0,
/*40*/  0,    '<',  'K',  'I',  'O',  ')',  '(',  0,
/*48*/  0,    '>',  '?',  'L',  ':',  'P',  '_',  0,
/*50*/  0,    0,    '"',  0,    '{',  '+',  0,    0,
/*58*/  0,    0,    '\r', '}',  0,    '|',  0,    0,
/*60*/  0,    0,    0,    0,    0,    0,    '\b', 0,
/*68*/  0,    '1',  0,    '4',  '7',  0,    0,    0,
/*70*/  '0',  '.',  '2',  '5',  '6',  '8',  27,   0,
/*78*/  0,    '+',  '3',  '-',  '*',  '9',  0,    0,
};

/* PS/2 special make codes */
#define PS2_LSHIFT  0x12
#define PS2_RSHIFT  0x59
#define PS2_LCTRL   0x14
#define PS2_LALT    0x11
#define PS2_CAPS    0x58
#define PS2_BREAK   0xF0
#define PS2_EXTEND  0xE0

typedef struct {
    bool shift;
    bool ctrl;
    bool alt;
    bool caps;
    bool extended;          /* last byte was 0xE0 */
    bool release;           /* last byte was 0xF0 */
    bool ext_release;       /* 0xE0 0xF0 pending */
} kbd_state_t;

static kbd_state_t s_kbd = {0};

static void kbd_process_scancode(uint8_t byte)
{
    if (byte == PS2_EXTEND) {
        s_kbd.extended = true;
        return;
    }
    if (byte == PS2_BREAK) {
        s_kbd.release = true;
        return;
    }

    bool release  = s_kbd.release;
    bool extended = s_kbd.extended;
    s_kbd.release  = false;
    s_kbd.extended = false;

    /* Handle modifier keys */
    if (!extended) {
        if (byte == PS2_LSHIFT || byte == PS2_RSHIFT) {
            s_kbd.shift = !release;
            return;
        }
        if (byte == PS2_LCTRL) {
            s_kbd.ctrl = !release;
            return;
        }
        if (byte == PS2_LALT) {
            s_kbd.alt = !release;
            return;
        }
        if (byte == PS2_CAPS && !release) {
            s_kbd.caps = !s_kbd.caps;
            return;
        }
    } else {
        /* Extended: right-ctrl = 0x14, right-alt = 0x11 */
        if (byte == 0x14) { s_kbd.ctrl = !release; return; }
        if (byte == 0x11) { s_kbd.alt  = !release; return; }
        /* Extended navigation keys on release: ignore */
        if (release) return;
        /* Map extended keys to ASCII escape sequences (simplified) */
        uint8_t ascii = 0;
        switch (byte) {
        case 0x75: ascii = 0xC1; break; /* up arrow    → Agon VK */
        case 0x72: ascii = 0xC2; break; /* down arrow  */
        case 0x74: ascii = 0xC3; break; /* right arrow */
        case 0x6B: ascii = 0xC4; break; /* left arrow  */
        case 0x6C: ascii = 0x01; break; /* Home → ^A   */
        case 0x69: ascii = 0x05; break; /* End  → ^E   */
        case 0x7D: ascii = 0x19; break; /* PgUp */
        case 0x7A: ascii = 0x18; break; /* PgDn */
        case 0x70: ascii = 0x7F; break; /* Insert */
        case 0x71: ascii = 0x08; break; /* Delete → BS */
        default:   return;
        }
        xQueueSend(s_key_queue, &ascii, 0);
        return;
    }

    if (release) return;

    /* Normal make code */
    if (byte >= 128) return;

    bool effective_shift = s_kbd.shift;
    /* Caps lock: invert shift for alphabetic keys */
    if (s_kbd.caps) {
        uint8_t base = s_ps2_unshifted[byte];
        if (base >= 'a' && base <= 'z') effective_shift = !effective_shift;
    }

    uint8_t ascii = effective_shift ? s_ps2_shifted[byte] : s_ps2_unshifted[byte];
    if (ascii == 0) return;

    /* Ctrl: produce control char */
    if (s_kbd.ctrl && ascii >= '@' && ascii <= '_') ascii &= 0x1F;
    if (s_kbd.ctrl && ascii >= 'a' && ascii <= 'z') ascii &= 0x1F;

    xQueueSend(s_key_queue, &ascii, 0);
}

/* ------------------------------------------------------------------ */
/* Public API                                                           */
/* ------------------------------------------------------------------ */

static bool s_initialized = false;

esp_err_t mos_vdp_internal_init(esp_lcd_panel_handle_t dpi_panel)
{
    if (s_initialized) return ESP_OK;

    s_panel = dpi_panel;

    /* Obtain framebuffer pointers from DPI panel (double-buffered).
     * Non-fatal: if the panel or get_frame_buffer fails, we continue
     * without a framebuffer.  The shell still runs (I/O via UART or TCP);
     * fb_flush() is a no-op when s_fb[0] == NULL. */
    if (s_panel) {
        void *fb0 = NULL, *fb1 = NULL;
        esp_err_t ret = esp_lcd_dpi_panel_get_frame_buffer(s_panel, 2, &fb0, &fb1);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "get_frame_buffer failed (0x%x) — HDMI output disabled", ret);
            s_panel = NULL;   /* disable fb_flush() */
        } else {
            s_fb[0] = (uint8_t *)fb0;
            s_fb[1] = (uint8_t *)fb1;
            ESP_LOGI(TAG, "Framebuffers: fb0=%p fb1=%p (%u bytes each)",
                     s_fb[0], s_fb[1], (unsigned)FB_SIZE);
        }
    } else {
        ESP_LOGW(TAG, "No DPI panel — HDMI output disabled");
    }

    /* Reset palette and screen state — default mode 0 = 640×480 */
    s_mode_w = 640;
    s_mode_h = 480;
    palette_reset();
    s_fg = 15; s_bg = 0;
    s_vp_left = 0; s_vp_top = 0;
    s_vp_right = COLS - 1; s_vp_bottom = ROWS - 1;
    s_col = 0; s_row = 0;
    s_vdu_state = VDU_STATE_NORMAL;
    memset(&s_kbd, 0, sizeof(s_kbd));

    /* Clear both framebuffers to black and flush dcache → PSRAM.
     * fb[0] is the render target; fb[1] exists because the panel was created
     * with num_fbs=2 (IDF requires it for DPI), but we never render into it.
     * Both are zeroed so the DPI DMA never emits stale PSRAM content even
     * if it temporarily points to fb[1] during init.
     * C2M + UNALIGNED: flush dirty dcache lines to PSRAM.
     * (INVALIDATE is not combined with UNALIGNED — IDF rejects that.) */
    for (int i = 0; i < 2; i++) {
        if (s_fb[i]) {
            memset(s_fb[i], 0, FB_SIZE);
            esp_cache_msync(s_fb[i], FB_SIZE,
                ESP_CACHE_MSYNC_FLAG_DIR_C2M |
                ESP_CACHE_MSYNC_FLAG_TYPE_DATA |
                ESP_CACHE_MSYNC_FLAG_UNALIGNED);
        }
    }

    /* Create queues */
    s_vdu_queue = xQueueCreate(VDU_QUEUE_LEN, sizeof(uint8_t));
    s_key_queue = xQueueCreate(KEY_QUEUE_LEN, sizeof(uint8_t));
    if (!s_vdu_queue || !s_key_queue) {
        ESP_LOGE(TAG, "Failed to create queues");
        return ESP_ERR_NO_MEM;
    }

    /* Mutex for framebuffer access */
    s_fb_mutex = xSemaphoreCreateMutex();
    if (!s_fb_mutex) {
        ESP_LOGE(TAG, "Failed to create framebuffer mutex");
        return ESP_ERR_NO_MEM;
    }

    /* Spawn render task on core 1 (core 0 = USB host) */
    BaseType_t ok = xTaskCreatePinnedToCore(
        vdp_render_task, "vdp_render",
        CONFIG_MOS_VDP_INTERNAL_TASK_STACK_KB * 1024,
        NULL, 5, NULL, 1);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create render task");
        return ESP_ERR_NO_MEM;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "Internal VDP ready — fb=%dx%d mode=%dx%d (%d cols×%d rows)",
             FB_W, FB_H, s_mode_w, s_mode_h, COLS, ROWS);
    return ESP_OK;
}

void mos_vdp_internal_putch(uint8_t c)
{
    if (!s_vdu_queue) return;
    /* Non-blocking: if queue is full, drop the byte (overflow) */
    xQueueSend(s_vdu_queue, &c, 0);
}

int mos_vdp_internal_getch(void)
{
    if (!s_key_queue) return -1;
    uint8_t c = 0;
    /* Use a short timeout so the caller can periodically re-evaluate routing.
     * Returns -1 on timeout (no key available yet) — caller should retry. */
    if (xQueueReceive(s_key_queue, &c, pdMS_TO_TICKS(100)) == pdTRUE) {
        return (int)c;
    }
    return -1;
}

bool mos_vdp_internal_kbhit(void)
{
    if (!s_key_queue) return false;
    return uxQueueMessagesWaiting(s_key_queue) > 0;
}

void mos_vdp_internal_send_scancode(uint8_t byte)
{
    kbd_process_scancode(byte);
}

bool mos_vdp_internal_connected(void)
{
    /* The internal VDP is always "connected" when compiled in — it is a
     * physical display, not a network client.  Output is silently dropped
     * if the framebuffer is not yet initialised, but the shell must always
     * route I/O here rather than falling back to the UART console. */
    return true;
}

void mos_vdp_internal_flush(void)
{
    /* No-op: rendering is driven by the render task */
}
