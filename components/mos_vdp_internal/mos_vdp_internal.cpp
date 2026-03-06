/*
 * mos_vdp_internal.cpp — Internal VDP: Agon VDU commands → PSRAM framebuffer → HDMI
 *
 * Renders Agon-compatible VDU commands to an RGB888 double-buffered framebuffer
 * in PSRAM, outputting through the ESP32-P4 DPI controller → LT8912B → HDMI.
 *
 * Supported:
 *   - Text rendering (cols×rows, 8×8 font, 16 colours)
 *   - UDG: VDU 23,n (n=32..255) — redefine character glyphs (stored in PSRAM)
 *   - VDU 4/5/6/7/8/9/10/11/12/13/14/15/16/17/18/19/22/23/24/25/26/28/29/30/31/127
 *   - VDU 23,0 sub-protocol: MOS handshake (0x80), cursor pos (0x82),
 *     mode info (0x86), cursor show/hide (23,1), cursor behaviour (23,16),
 *     UDG reset (0x91), GCOL dotted line (23,6), scroll (23,7)
 *   - GCOL paint modes (Set/OR/AND/XOR/Invert)
 *   - PLOT engine: point, line, filled rect, filled triangle, filled circle,
 *     circle outline, flood fill, copy/move rect
 *   - VDU 24: graphics viewport
 *   - VDU 26: reset viewports
 *   - PS/2 Set 2 keyboard → ASCII + modifier decoding
 *
 * Board: Olimex ESP32-P4-PC
 * Output: 1024×768@60Hz VESA over HDMI via LT8912B DSI bridge
 */

#include "mos_vdp_internal.h"

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_cache.h"

/* Switch the DPI panel's active framebuffer by writing cur_fb_index directly.
 *
 * Why not draw_bitmap: draw_bitmap fires on_color_trans_done from task context,
 * which reprograms the DW-GDMA channel while the ISR may be reading it →
 * Load access fault in dw_gdma_channel_default_isr (MTVAL=0x4, always).
 *
 * Layout of esp_lcd_dpi_panel_t (esp_lcd_panel_dpi.c, IDF 5.x):
 *   offset 0:  base (esp_lcd_panel_t) — 9 fn-ptrs + 1 void* = 10 × sizeof(void*)
 *   offset 10×ptr: bus (void*)
 *   offset 11×ptr: virtual_channel (uint8_t)
 *   offset 11×ptr+1: cur_fb_index  (uint8_t)  ← we write this
 *
 * A uint8_t store is atomic on RISC-V — no mutex needed.
 * The DW-GDMA ISR reads cur_fb_index at frame-start; we write it any time. */
static inline void dpi_set_cur_fb(esp_lcd_panel_handle_t panel, uint8_t idx)
{
    /* base has 10 pointer-sized members (9 fn ptrs + user_data), then bus ptr,
     * then virtual_channel (u8), then cur_fb_index (u8). */
    uintptr_t base = (uintptr_t)panel;
    /* Verified from boot dump (IDF 5.x, ESP32-P4):
     * offset 0..43:  base vtable (10 fn ptrs + user_data) = 11 × 4 bytes
     * offset 44..47: bus ptr (4 bytes)
     * offset 48:     virtual_channel (uint8_t)
     * offset 49:     cur_fb_index    (uint8_t)  ← we write this */
    size_t off = 11 * sizeof(void *) + sizeof(void *) + 1;  /* = 49 on RV32 */
    volatile uint8_t *p = (volatile uint8_t *)(base + off);
    *p = idx;
}
#include "esp_heap_caps.h"
#include "esp_timer.h"

#include "vdp_font.h"
#include "mos_audio_synth.h"
#include "mos_sysvars_block.h"
#include "mos_vdp_ttxt.h"

static const char *TAG = "vdp_int";

/* ------------------------------------------------------------------ */
/* Screen geometry                                                      */
/* ------------------------------------------------------------------ */
#define FB_W            800
#define FB_H            600
#define FONT_W          8
#define FONT_H          8
#define BYTES_PER_PIX   3                     /* RGB888 */
#define FB_SIZE         (FB_W * FB_H * BYTES_PER_PIX)  /* 2359296 (~2.25 MB) */

static int     s_mode_w      = 640;
static int     s_mode_h      = 480;
static uint8_t s_mode_num    = 0;    /* current Agon mode number (0-30, +0x80 for double-buf) */
static uint8_t s_mode_colours = 16; /* number of colours in current mode */
static bool    s_ttxt_mode   = false; /* true when mode 7 (teletext) is active */
#define COLS            (s_mode_w / FONT_W)
#define ROWS            (s_mode_h / FONT_H)

/* Live sysvar block — updated on every mode change and cursor move.
 * Returned by mos_vdp_internal_get_sysvars(); the router reads this
 * instead of its own stale static copy. Zero-initialised; fields are
 * filled in by sysvars_update() before any caller can read them. */
static t_mos_sysvars s_sysvars = {};

/* ------------------------------------------------------------------ */
/* Scale + centre state                                                 */
/* ------------------------------------------------------------------ */
static int s_scale = 1;
static int s_off_x = 0;
static int s_off_y = 0;

/* Agon logical coordinate space: always 1280 × 1024 OS units,
 * matching BBC Micro / console8 VDP convention.
 * logicalScaleX = 1280 / mode_w  (e.g. 2.0 for 640-wide modes)
 * logicalScaleY = 1024 / mode_h  (e.g. ~2.133 for 480-high modes)
 * Coords arriving in VDU 25 / VDU 24 / VDU 29 are in these OS units. */
#define LOGICAL_SCRW 1280
#define LOGICAL_SCRH 1024
static float s_logical_scale_x = 2.0f;
static float s_logical_scale_y = 2.0f;

/* Convert an Agon OS-unit coordinate to a screen pixel coordinate */
static inline int agon_os_to_px_x(int16_t os_x)
{
    return (int)((float)os_x / s_logical_scale_x);
}
static inline int agon_os_to_px_y(int16_t os_y)
{
    return (int)((float)os_y / s_logical_scale_y);
}

static void scale_update(void)
{
    /* Integer scale: largest factor where both dimensions fit inside the FB.
     * Remainder pixels become black border (letterbox/pillarbox). */
    int sx = FB_W / s_mode_w;
    int sy = FB_H / s_mode_h;
    s_scale = (sx < sy) ? sx : sy;
    if (s_scale < 1) s_scale = 1;
    /* Centre the scaled image; unoccupied border pixels are black. */
    s_off_x = (FB_W - s_mode_w * s_scale) / 2;
    s_off_y = (FB_H - s_mode_h * s_scale) / 2;
    /* Logical OS-unit scale factors (1280×1024 logical space → mode pixels) */
    s_logical_scale_x = (float)LOGICAL_SCRW / (float)s_mode_w;
    s_logical_scale_y = (float)LOGICAL_SCRH / (float)s_mode_h;
}

/* ------------------------------------------------------------------ */
/* Agon 16-colour palette                                               */
/* ------------------------------------------------------------------ */
static const uint8_t s_palette_lookup6[64][3] = {
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

static const uint8_t s_palette02[2] = {
    0x00, 0x3F,
};
static const uint8_t s_palette04[4] = {
    0x00, 0x30, 0x3C, 0x3F,
};
static const uint8_t s_palette08[8] = {
    0x00, 0x30, 0x0C, 0x3C, 0x03, 0x33, 0x0F, 0x3F,
};
static const uint8_t s_palette10[16] = {
    0x00, 0x20, 0x08, 0x28, 0x02, 0x22, 0x0A, 0x2A,   /* dark */
    0x15, 0x30, 0x0C, 0x3C, 0x03, 0x33, 0x0F, 0x3F,   /* bright */
};
static const uint8_t s_palette40[64] = {
    0x00, 0x20, 0x08, 0x28, 0x02, 0x22, 0x0A, 0x2A,   /* first 16 same as s_palette10 */
    0x15, 0x30, 0x0C, 0x3C, 0x03, 0x33, 0x0F, 0x3F,
    0x01, 0x04, 0x05, 0x06, 0x07, 0x09, 0x0B, 0x0D,   /* remaining 48 */
    0x0E, 0x10, 0x11, 0x12, 0x13, 0x14, 0x16, 0x17,
    0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F,
    0x21, 0x23, 0x24, 0x25, 0x26, 0x27, 0x29, 0x2B,
    0x2C, 0x2D, 0x2E, 0x2F, 0x31, 0x32, 0x34, 0x35,
    0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B, 0x3D, 0x3E,
};

typedef struct { uint8_t r, g, b; } rgb888_t;

static rgb888_t s_palette[64];

/* Map a colour index to a valid palette slot.
 * Agon hardware wraps colour indices modulo the number of colours in the
 * current mode — e.g. colour 15 in a 4-colour mode → slot 15%4 = 3 (white).
 * Using & 0x3F (bitmask) instead gives slot 15 which is black (unfilled). */
static inline int pal_idx(int c)
{
    int n = s_mode_colours;
    if (n <= 0) n = 2;
    return ((c % n) + n) % n;   /* handles negative c gracefully */
}

static void palette_reset(void)
{
    const uint8_t *pal;
    int n = s_mode_colours;
    if      (n >=  64) { pal = s_palette40; n = 64; }
    else if (n >=  16) { pal = s_palette10; n = 16; }
    else if (n >=   8) { pal = s_palette08; n =  8; }
    else if (n >=   4) { pal = s_palette04; n =  4; }
    else               { pal = s_palette02; n =  2; }
    for (int i = 0; i < n; i++) {
        uint8_t idx = pal[i];
        s_palette[i].r = s_palette_lookup6[idx][0];
        s_palette[i].g = s_palette_lookup6[idx][1];
        s_palette[i].b = s_palette_lookup6[idx][2];
    }
    /* Fill remaining slots with black */
    for (int i = n; i < 64; i++) {
        s_palette[i] = {0, 0, 0};
    }
}

/* ------------------------------------------------------------------ */
/* UDG (User Defined Graphics) — VDU 23,n,b0..b7  (n = 32..255)       */
/*                                                                      */
/* 224 character slots × 8 bytes = 1792 bytes in PSRAM.                */
/* Slot is NULL until first definition; draw_char falls back to font.   */
/* ------------------------------------------------------------------ */
#define UDG_COUNT   224     /* chars 32..255 */
static uint8_t (*s_udg)[8] = NULL;   /* [224][8], allocated in PSRAM */

static void udg_init(void)
{
    if (!s_udg) {
        s_udg = (uint8_t (*)[8])heap_caps_calloc(UDG_COUNT, 8, MALLOC_CAP_SPIRAM);
        /* NULL = fall back to ROM font for all chars — still correct */
    }
}

/* Bitmask: which UDGs have been defined (bit set = use s_udg, clear = use font) */
/* 224 bits = 28 bytes */
static uint8_t s_udg_defined[28] = {0};

static inline void udg_set_defined(int n)   /* n = char code 32..255 */
{
    int slot = n - 32;
    s_udg_defined[slot >> 3] |= (uint8_t)(1u << (slot & 7));
}

static inline bool udg_is_defined(int n)
{
    int slot = n - 32;
    return (s_udg_defined[slot >> 3] >> (slot & 7)) & 1;
}

static void udg_reset_all(void)
{
    memset(s_udg_defined, 0, sizeof(s_udg_defined));
    /* No need to clear the data — just clearing the defined bits is enough */
}

/* Graphics viewport — declared here so sprite engine can reference them */
static int s_gvp_x0 = 0, s_gvp_y0 = 0;
static int s_gvp_x1 = 639, s_gvp_y1 = 479;

/* ================================================================== */
/* Sprite / Bitmap engine  (VDU 23,27)                                */
/*                                                                     */
/* Bitmaps: up to 256 slots, RGBA8888, stored in PSRAM.               */
/* Sprites: up to 64 entries; each sprite holds ≤8 frame pointers     */
/*          into the bitmap table.  Render happens on every fb flush.  */
/* ================================================================== */

#define BITMAP_MAX      256
#define SPRITE_MAX       64
#define SPRITE_FRAMES    8    /* max frames per sprite */

/* RGBA pixel (8 bits per channel, alpha=255 → opaque) */
typedef struct { uint8_t r, g, b, a; } rgba_t;

typedef struct {
    uint8_t  *data;      /* PSRAM: w×h × 4 bytes RGBA8888, NULL = empty */
    uint16_t  w, h;
} bitmap_t;

typedef struct {
    uint8_t   frame_ids[SPRITE_FRAMES]; /* bitmap indices */
    uint8_t   num_frames;
    uint8_t   cur_frame;
    int16_t   x, y;           /* position in logical pixels */
    bool      visible;
    uint8_t   paint_mode;     /* 0=opaque, 1=transparent(alpha) */
} sprite_t;

static bitmap_t  s_bitmaps[BITMAP_MAX];
static sprite_t  s_sprites[SPRITE_MAX];
static uint8_t   s_num_active_sprites = 0; /* sprites 0..n-1 are on-stage */
static uint8_t   s_cur_bitmap  = 0;
static uint8_t   s_cur_sprite  = 0;

/* ------ bitmap helpers ------ */

static void bitmap_free(uint8_t id)
{
    if (s_bitmaps[id].data) {
        heap_caps_free(s_bitmaps[id].data);
        s_bitmaps[id].data = NULL;
    }
    s_bitmaps[id].w = 0;
    s_bitmaps[id].h = 0;
}

/* Allocate/reallocate bitmap data in PSRAM */
static bool bitmap_alloc(uint8_t id, uint16_t w, uint16_t h)
{
    bitmap_free(id);
    uint32_t bytes = (uint32_t)w * h * 4;
    if (bytes == 0) return false;
    s_bitmaps[id].data = (uint8_t *)heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM);
    if (!s_bitmaps[id].data) {
        ESP_LOGW(TAG, "bitmap_alloc: PSRAM alloc failed (%ux%u)", w, h);
        return false;
    }
    s_bitmaps[id].w = w;
    s_bitmaps[id].h = h;
    return true;
}

/* Blit bitmap to the active framebuffer at logical position (bx,by).
 * Clips to the current graphics viewport.
 * paint_mode 0 = opaque (ignore alpha), 1 = alpha-blend. */
static void bitmap_draw_to_fb(uint8_t *fb, uint8_t id, int bx, int by, uint8_t paint_mode)
{
    if (!s_bitmaps[id].data) return;
    uint16_t bw = s_bitmaps[id].w;
    uint16_t bh = s_bitmaps[id].h;
    const rgba_t *src = (const rgba_t *)s_bitmaps[id].data;

    for (int row = 0; row < (int)bh; row++) {
        int ly = by + row;
        if (ly < s_gvp_y0 || ly > s_gvp_y1) continue;
        int py = ly * s_scale + s_off_y;
        if (py < 0 || py >= FB_H) continue;

        for (int col = 0; col < (int)bw; col++) {
            int lx = bx + col;
            if (lx < s_gvp_x0 || lx > s_gvp_x1) continue;
            int px = lx * s_scale + s_off_x;
            if (px < 0 || px >= FB_W) continue;

            rgba_t p = src[row * bw + col];

            for (int sy = 0; sy < s_scale; sy++) {
                for (int sx = 0; sx < s_scale; sx++) {
                    int fpx = px + sx;
                    int fpy = py + sy;
                    if (fpx >= FB_W || fpy >= FB_H) continue;
                    uint8_t *dst = fb + (fpy * FB_W + fpx) * BYTES_PER_PIX;

                    if (paint_mode == 0 || p.a == 255) {
                        /* opaque */
                        dst[0] = p.r; dst[1] = p.g; dst[2] = p.b;
                    } else if (p.a > 0) {
                        /* alpha blend */
                        uint16_t a = p.a;
                        dst[0] = (uint8_t)((dst[0] * (255-a) + p.r * a) >> 8);
                        dst[1] = (uint8_t)((dst[1] * (255-a) + p.g * a) >> 8);
                        dst[2] = (uint8_t)((dst[2] * (255-a) + p.b * a) >> 8);
                    }
                    /* a==0 → fully transparent, skip */
                }
            }
        }
    }
}

/* Render all active sprites onto fb (called before display flip) */
static void __attribute__((unused)) sprites_render(uint8_t *fb)
{
    for (int i = 0; i < (int)s_num_active_sprites; i++) {
        sprite_t *sp = &s_sprites[i];
        if (!sp->visible || sp->num_frames == 0) continue;
        uint8_t bid = sp->frame_ids[sp->cur_frame];
        bitmap_draw_to_fb(fb, bid, sp->x, sp->y, sp->paint_mode);
    }
}

static void sprites_reset_all(void)
{
    for (int i = 0; i < SPRITE_MAX; i++) {
        s_sprites[i].num_frames  = 0;
        s_sprites[i].cur_frame   = 0;
        s_sprites[i].visible     = false;
        s_sprites[i].x = s_sprites[i].y = 0;
        s_sprites[i].paint_mode  = 1; /* default: alpha-blend */
    }
    s_num_active_sprites = 0;
    s_cur_sprite = 0;
}

static void bitmaps_reset_all(void)
{
    for (int i = 0; i < BITMAP_MAX; i++) bitmap_free(i);
    s_cur_bitmap = 0;
}

/* ------------------------------------------------------------------ */
/* Framebuffer state                                                    */
/* ------------------------------------------------------------------ */
static esp_lcd_panel_handle_t s_panel = NULL;
static uint8_t *s_fb[2] = { NULL, NULL };
static SemaphoreHandle_t s_fb_mutex = NULL;
/* Double-buffer support: s_draw_buf_idx is the back buffer (render target).
 * The DPI controller always reads the *other* buffer (front).
 * Starts at 0; flip_buffer() swaps after syncing the back buffer. */
static int s_draw_buf_idx = 0;
static inline uint8_t *fb_draw(void) { return s_fb[s_draw_buf_idx]; }

/* Dirty flag: set by any CPU write to the framebuffer.
 * The periodic flush timer reads this and does the cache_msync + draw_bitmap
 * at ~60 Hz, decoupling PLOT throughput from display refresh. */
static volatile bool s_fb_dirty = false;
static esp_timer_handle_t s_flush_timer = NULL;

static void fb_flush_timer_cb(void *arg)
{
    /* Just set the flag — the render task will do the actual draw_bitmap
     * from its safe task context, avoiding any DMA race conditions. */
    s_fb_dirty = true;

    /* Teletext flash: toggle every ~30 frames (~0.5 s at 60 Hz) */
    if (s_ttxt_mode) {
        static int s_flash_ctr = 0;
        static bool s_flash_phase = false;
        s_flash_ctr++;
        if (s_flash_ctr >= 30) {
            s_flash_ctr = 0;
            s_flash_phase = !s_flash_phase;
            ttxt_flash(s_flash_phase);
        }
    }
}

/* ------------------------------------------------------------------ */
/* GCOL paint mode                                                      */
/*  0 = Set (replace), 1 = OR, 2 = AND, 3 = XOR, 4 = Invert dest      */
/* ------------------------------------------------------------------ */
static int s_gcol_mode = 0;   /* foreground paint mode */

static inline rgb888_t apply_gcol_mode(int mode, rgb888_t dst, rgb888_t src)
{
    switch (mode & 7) {
    case 0: return src;                                      /* Set */
    case 1: return {(uint8_t)(dst.r|src.r),(uint8_t)(dst.g|src.g),(uint8_t)(dst.b|src.b)}; /* OR */
    case 2: return {(uint8_t)(dst.r&src.r),(uint8_t)(dst.g&src.g),(uint8_t)(dst.b&src.b)}; /* AND */
    case 3: return {(uint8_t)(dst.r^src.r),(uint8_t)(dst.g^src.g),(uint8_t)(dst.b^src.b)}; /* XOR */
    case 4: return {(uint8_t)~dst.r,(uint8_t)~dst.g,(uint8_t)~dst.b};   /* Invert */
    default: return dst;   /* 5=no-op, 6/7=AND-NOT/OR-NOT (simplified) */
    }
}

/* ------------------------------------------------------------------ */
/* Graphics state                                                       */
/* ------------------------------------------------------------------ */
static int s_gfx_x = 0, s_gfx_y = 0;   /* last graphics cursor (logical) */
static int s_gfx_x_prev  = 0, s_gfx_y_prev  = 0;  /* penultimate graphics position */
static int s_gfx_x_prev2 = 0, s_gfx_y_prev2 = 0;  /* pen-penultimate (for triangles) */

/* Graphics viewport in logical pixels (inclusive) — also used by sprite engine */
/* declared early (before sprite engine) to avoid forward-reference errors */

/* Graphics origin */
static int s_gfx_origin_x = 0, s_gfx_origin_y = 0;

/* Graphics foreground colour index */
static int s_gfx_fg = 15;
static int s_gfx_bg = 0;

/* ------------------------------------------------------------------ */
/* Cursor state                                                         */
/* ------------------------------------------------------------------ */
static bool s_cursor_visible = true;

/* ------------------------------------------------------------------ */
/* Low-level pixel plot (with GCOL mode applied)                       */
/* ------------------------------------------------------------------ */
static inline void fb_plot_pixel_raw(int phys_x, int phys_y, rgb888_t c)
{
    if ((unsigned)phys_x >= (unsigned)FB_W || (unsigned)phys_y >= (unsigned)FB_H) return;
    uint8_t *p = fb_draw() + (phys_y * FB_W + phys_x) * BYTES_PER_PIX;
    p[0] = c.b; p[1] = c.g; p[2] = c.r;   /* DPI FB is BGR888 in memory */
}

static inline rgb888_t fb_read_pixel_raw(int phys_x, int phys_y)
{
    if ((unsigned)phys_x >= (unsigned)FB_W || (unsigned)phys_y >= (unsigned)FB_H)
        return {0,0,0};
    uint8_t *p = fb_draw() + (phys_y * FB_W + phys_x) * BYTES_PER_PIX;
    return {p[2], p[1], p[0]};   /* DPI FB is BGR888 in memory */
}

/* Plot a logical pixel with scale, offset, gcol mode, and graphics viewport clip */
static void fb_plot_pixel(int x, int y, rgb888_t c, int gcol_mode = 0)
{
    /* Clip to graphics viewport */
    if (x < s_gvp_x0 || x > s_gvp_x1 || y < s_gvp_y0 || y > s_gvp_y1) return;
    if ((unsigned)x >= (unsigned)s_mode_w || (unsigned)y >= (unsigned)s_mode_h) return;

    int phys_x = s_off_x + x * s_scale;
    int phys_y = s_off_y + y * s_scale;
    for (int dy = 0; dy < s_scale; dy++) {
        for (int dx = 0; dx < s_scale; dx++) {
            int px = phys_x + dx, py = phys_y + dy;
            rgb888_t out = (gcol_mode == 0) ? c :
                           apply_gcol_mode(gcol_mode, fb_read_pixel_raw(px, py), c);
            fb_plot_pixel_raw(px, py, out);
        }
    }
}

/* Fill a horizontal span [x0..x1] at logical row y, fast path for gcol_mode==0.
 * For gcol_mode != 0 falls back to fb_plot_pixel per pixel.
 * All coordinates must already be clipped to the graphics viewport. */
static void fb_fill_hspan(int x0, int x1, int y, rgb888_t c, int gcol_mode)
{
    if (x0 > x1) return;
    if (gcol_mode != 0) {
        for (int x = x0; x <= x1; x++)
            fb_plot_pixel(x, y, c, gcol_mode);
        return;
    }
    /* Fast path: write scaled pixels directly using 32-bit stores.
     * RGB888 = 3 bytes/pixel; 4 pixels = 12 bytes = 3 uint32_t — so we build
     * a 12-byte repeating pattern and blast it with 32-bit writes for speed. */
    int phys_x0 = s_off_x + x0 * s_scale;
    int phys_y0 = s_off_y + y  * s_scale;
    int phys_w  = (x1 - x0 + 1) * s_scale;
    /* Bounds check */
    if ((unsigned)phys_x0 >= (unsigned)FB_W || (unsigned)phys_y0 >= (unsigned)FB_H) return;
    if (phys_x0 + phys_w > FB_W) phys_w = FB_W - phys_x0;

    /* Pre-build 12-byte pattern: RGBRGBRGBRGB (4 pixels) */
        uint8_t pat[12] = {                /* DPI FB is BGR888 in memory */
        c.b, c.g, c.r,  c.b, c.g, c.r,
        c.b, c.g, c.r,  c.b, c.g, c.r
    };
    uint32_t w0, w1, w2;
    memcpy(&w0, pat+0, 4);
    memcpy(&w1, pat+4, 4);
    memcpy(&w2, pat+8, 4);

    int span_bytes = phys_w * BYTES_PER_PIX;

    for (int dy = 0; dy < s_scale; dy++) {
        int py = phys_y0 + dy;
        if ((unsigned)py >= (unsigned)FB_H) break;
        uint8_t *row = fb_draw() + (py * FB_W + phys_x0) * BYTES_PER_PIX;
        uint8_t *p   = row;
        uint8_t *end = row + span_bytes;
        /* Align to 4 bytes */
        while (p < end && ((uintptr_t)p & 3)) { p[0]=c.b; p[1]=c.g; p[2]=c.r; p+=3; }
        /* 12-byte (3×uint32_t) main loop */
        uint32_t *q = (uint32_t *)p;
        uint8_t  *q8 = (uint8_t *)q;
        while (q8 + 12 <= end) {
            q[0] = w0; q[1] = w1; q[2] = w2;
            q += 3; q8 += 12;
        }
        /* Tail */
        p = q8;
        while (p + 3 <= end) { p[0]=c.b; p[1]=c.g; p[2]=c.r; p+=3; }
    }
}

/* Mark FB dirty — the periodic timer will push it to the display at ~60 Hz.
 * Use fb_flush_now() for operations where immediate visibility is needed
 * (text, CLS, mode change). */
static inline void fb_flush(void)
{
    s_fb_dirty = true;
}

static void fb_flush_now(void)
{
    if (!fb_draw()) return;
    s_fb_dirty = false;
    /* Update centisecond timer — BBC BASIC reads TIME from sysvar.time for
     * RND seed and other timing. Updated here at ~60 Hz. */
    s_sysvars.time = (uint32_t)(esp_timer_get_time() / 10000ULL);
    /* Sync draw buffer cache → PSRAM so the DPI DMA reads fresh pixels.
     * Then set cur_fb_index directly (atomic byte store, offset verified=49). */
    esp_cache_msync(fb_draw(), FB_SIZE, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
    if (s_panel) dpi_set_cur_fb(s_panel, (uint8_t)s_draw_buf_idx);
}

/* Flip double buffers: sync back → PSRAM, point DPI at it, swap indices.
 * Must be called from render task context only. */
static void fb_flip_now(void)
{
    if (!s_panel || !fb_draw()) return;
    s_fb_dirty = false;
    esp_cache_msync(fb_draw(), FB_SIZE, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
    dpi_set_cur_fb(s_panel, (uint8_t)s_draw_buf_idx);
    s_draw_buf_idx ^= 1;
}

/* ------------------------------------------------------------------ */
/* Text cursor state                                                    */
/* ------------------------------------------------------------------ */
static int  s_col = 0, s_row = 0;
static int  s_fg  = 15;
static int  s_bg  = 0;

/* Text viewport (character coordinates, inclusive) */
static int s_vp_left = 0, s_vp_top = 0;
static int s_vp_right = 79, s_vp_bottom = 59;

/* ------------------------------------------------------------------ */
/* Low-level character rendering                                        */
/* ------------------------------------------------------------------ */

static const uint8_t *get_glyph(uint8_t c)
{
    if (c >= 32 && udg_is_defined(c) && s_udg) {
        return s_udg[c - 32];
    }
    return vdp_font_glyph(c);
}

static void draw_char(int col, int row, uint8_t c)
{
    const uint8_t *glyph = get_glyph(c);
    int px = col * FONT_W;
    int py = row * FONT_H;
    rgb888_t fg = s_palette[pal_idx(s_fg)];
    rgb888_t bg = s_palette[pal_idx(s_bg)];

    for (int r = 0; r < FONT_H; r++) {
        uint8_t bits = glyph[r];
        for (int b = 0; b < FONT_W; b++) {
            /* Direct fb_plot_pixel without gcol clip for text */
            int lx = px + b, ly = py + r;
            if ((unsigned)lx < (unsigned)s_mode_w && (unsigned)ly < (unsigned)s_mode_h) {
                int phys_x = s_off_x + lx * s_scale;
                int phys_y = s_off_y + ly * s_scale;
                rgb888_t col_val = (bits & (0x80 >> b)) ? fg : bg;
                for (int dy = 0; dy < s_scale; dy++)
                    for (int dx = 0; dx < s_scale; dx++)
                        fb_plot_pixel_raw(phys_x+dx, phys_y+dy, col_val);
            }
        }
    }
}

static void erase_char(int col, int row)
{
    int px = col * FONT_W;
    int py = row * FONT_H;
    rgb888_t bg = s_palette[pal_idx(s_bg)];
    for (int r = 0; r < FONT_H; r++) {
        for (int b = 0; b < FONT_W; b++) {
            int lx = px + b, ly = py + r;
            if ((unsigned)lx < (unsigned)s_mode_w && (unsigned)ly < (unsigned)s_mode_h) {
                int phys_x = s_off_x + lx * s_scale;
                int phys_y = s_off_y + ly * s_scale;
                for (int dy = 0; dy < s_scale; dy++)
                    for (int dx = 0; dx < s_scale; dx++)
                        fb_plot_pixel_raw(phys_x+dx, phys_y+dy, bg);
            }
        }
    }
}

static void clear_screen(void)
{
    rgb888_t bg = s_palette[pal_idx(s_bg)];
    memset(fb_draw(), 0, FB_SIZE);
    int scaled_w = s_mode_w * s_scale;
    int scaled_h = s_mode_h * s_scale;
    for (int py = s_off_y; py < s_off_y + scaled_h; py++) {
        uint8_t *row = fb_draw() + (py * FB_W + s_off_x) * BYTES_PER_PIX;
        for (int px = 0; px < scaled_w; px++) {
            row[0] = bg.b; row[1] = bg.g; row[2] = bg.r;   /* BGR888 */
            row += BYTES_PER_PIX;
        }
    }
}

static void clear_graphics_viewport(int colour_idx)
{
    /* Clear the graphics viewport only, in the given colour */
    rgb888_t c = s_palette[pal_idx(colour_idx)];
    for (int y = s_gvp_y0; y <= s_gvp_y1; y++)
        fb_fill_hspan(s_gvp_x0, s_gvp_x1, y, c, 0);
}

static void scroll_up(void)
{
    int row_bytes   = FB_W * BYTES_PER_PIX;
    int char_h_phys = FONT_H * s_scale;
    int char_bytes  = char_h_phys * row_bytes;

    int vp_py0 = s_off_y + s_vp_top    * char_h_phys;
    int vp_py1 = s_off_y + (s_vp_bottom + 1) * char_h_phys;
    int vp_px0 = s_off_x + s_vp_left   * FONT_W * s_scale;
    int vp_px1 = s_off_x + (s_vp_right + 1) * FONT_W * s_scale;
    int vp_pw  = (vp_px1 - vp_px0) * BYTES_PER_PIX;

    rgb888_t bg = s_palette[pal_idx(s_bg)];

    if (s_vp_left == 0 && s_vp_right == COLS - 1) {
        int move_bytes = (vp_py1 - vp_py0 - char_h_phys) * row_bytes;
        if (move_bytes > 0) {
            uint8_t *dst = fb_draw() + vp_py0 * row_bytes;
            uint8_t *src = dst + char_bytes;
            memmove(dst, src, move_bytes);
        }
        uint8_t *bot = fb_draw() + (vp_py1 - char_h_phys) * row_bytes
                                 + vp_px0 * BYTES_PER_PIX;
        for (int pr = 0; pr < char_h_phys; pr++) {
            uint8_t *p = bot + pr * row_bytes;
            for (int pp = 0; pp < (vp_px1 - vp_px0); pp++) {
                p[0] = bg.b; p[1] = bg.g; p[2] = bg.r;   /* BGR888 */
                p += BYTES_PER_PIX;
            }
        }
    } else {
        for (int py = vp_py0; py < vp_py1 - char_h_phys; py++) {
            uint8_t *dst = fb_draw() + py * row_bytes + vp_px0 * BYTES_PER_PIX;
            uint8_t *src = dst + char_bytes;
            memmove(dst, src, vp_pw);
        }
        for (int py = vp_py1 - char_h_phys; py < vp_py1; py++) {
            uint8_t *p = fb_draw() + py * row_bytes + vp_px0 * BYTES_PER_PIX;
            for (int pp = 0; pp < (vp_px1 - vp_px0); pp++) {
                p[0] = bg.b; p[1] = bg.g; p[2] = bg.r;   /* BGR888 */
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
/* PLOT engine                                                          */
/* ------------------------------------------------------------------ */

/* Convert Agon logical coordinates (origin bottom-left) to screen Y
 * Agon: x right, y up → screen: x right, y down                     */
static inline int agon_to_screen_y(int agon_y)
{
    return s_mode_h - 1 - agon_y;
}

static void plot_pixel(int x, int y, rgb888_t c)
{
    fb_plot_pixel(x, agon_to_screen_y(y), c, s_gcol_mode);
}

static void plot_line(int x0, int y0, int x1, int y1, rgb888_t c)
{
    /* Bresenham's line algorithm in screen space */
    int sy0 = agon_to_screen_y(y0);
    int sy1 = agon_to_screen_y(y1);
    int dx = abs(x1 - x0), dy = abs(sy1 - sy0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (sy0 < sy1) ? 1 : -1;
    int err = dx - dy;
    while (1) {
        fb_plot_pixel(x0, sy0, c, s_gcol_mode);
        if (x0 == x1 && sy0 == sy1) break;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 <  dx) { err += dx; sy0 += sy; }
    }
}

static void plot_line_omit_last(int x0, int y0, int x1, int y1, rgb888_t c)
{
    int sy0 = agon_to_screen_y(y0);
    int sy1 = agon_to_screen_y(y1);
    int dx = abs(x1 - x0), dy = abs(sy1 - sy0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (sy0 < sy1) ? 1 : -1;
    int err = dx - dy;
    while (1) {
        if (x0 == x1 && sy0 == sy1) break;  /* omit last */
        fb_plot_pixel(x0, sy0, c, s_gcol_mode);
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 <  dx) { err += dx; sy0 += sy; }
    }
}

static void plot_filled_rect(int x0, int y0, int x1, int y1, rgb888_t c)
{
    /* All coords in logical (Agon) space, Y up → convert */
    int sy0 = agon_to_screen_y(y0);
    int sy1 = agon_to_screen_y(y1);
    if (sy0 > sy1) { int t = sy0; sy0 = sy1; sy1 = t; }
    if (x0 > x1)   { int t = x0;  x0  = x1;  x1  = t;  }
    /* Clip to graphics viewport */
    int cx0 = (x0 < s_gvp_x0) ? s_gvp_x0 : x0;
    int cx1 = (x1 > s_gvp_x1) ? s_gvp_x1 : x1;
    int cy0 = (sy0 < s_gvp_y0) ? s_gvp_y0 : sy0;
    int cy1 = (sy1 > s_gvp_y1) ? s_gvp_y1 : sy1;
    for (int y = cy0; y <= cy1; y++)
        fb_fill_hspan(cx0, cx1, y, c, s_gcol_mode);
}

static void plot_filled_triangle(int x0, int y0, int x1, int y1, int x2, int y2, rgb888_t c)
{
    /* Rasterise triangle in screen space */
    int sy0 = agon_to_screen_y(y0);
    int sy1 = agon_to_screen_y(y1);
    int sy2 = agon_to_screen_y(y2);

    /* Sort vertices by y (top to bottom) */
    if (sy0 > sy1) { int t; t=sy0;sy0=sy1;sy1=t; t=x0;x0=x1;x1=t; }
    if (sy0 > sy2) { int t; t=sy0;sy0=sy2;sy2=t; t=x0;x0=x2;x2=t; }
    if (sy1 > sy2) { int t; t=sy1;sy1=sy2;sy2=t; t=x1;x1=x2;x2=t; }

    /* Fill flat-bottom then flat-top halves */
    auto fill_flat = [&](int xa, int ya, int xb, int yb, int xc, int yc) {
        /* ya == yb (flat bottom/top), xc/yc is apex */
        if (ya == yc) return;
        int steps = abs(yc - ya);
        /* Clamp iteration range to the visible viewport to avoid iterating
         * millions of steps for off-screen geometry. */
        int i0 = 0, i1 = steps;
        int y_lo = (yc > ya) ? s_gvp_y0 : s_gvp_y0;   /* unused, kept for clarity */
        (void)y_lo;
        if (yc > ya) {
            /* y increases with i: y = ya + i */
            if (ya + i0 < s_gvp_y0) i0 = s_gvp_y0 - ya;
            if (ya + i1 > s_gvp_y1) i1 = s_gvp_y1 - ya;
        } else {
            /* y decreases with i: y = ya - i */
            if (ya - i0 > s_gvp_y1) i0 = ya - s_gvp_y1;
            if (ya - i1 < s_gvp_y0) i1 = ya - s_gvp_y0;
        }
        if (i0 > i1) return;  /* entirely off-screen */
        for (int i = i0; i <= i1; i++) {
            int y  = ya + (yc > ya ? i : -i);
            int lx = xa + (xc - xa) * i / steps;
            int rx = xb + (xc - xb) * i / steps;
            if (lx > rx) { int t = lx; lx = rx; rx = t; }
            /* Clip X span to viewport */
            if (lx < s_gvp_x0) lx = s_gvp_x0;
            if (rx > s_gvp_x1) rx = s_gvp_x1;
            fb_fill_hspan(lx, rx, y, c, s_gcol_mode);
        }
    };

    if (sy1 == sy2) {
        fill_flat(x1, sy1, x2, sy2, x0, sy0);
    } else if (sy0 == sy1) {
        fill_flat(x0, sy0, x1, sy1, x2, sy2);
    } else {
        /* Split into flat-bottom + flat-top */
        int x_mid = x0 + (x2 - x0) * (sy1 - sy0) / (sy2 - sy0);
        fill_flat(x1, sy1, x_mid, sy1, x0, sy0);
        fill_flat(x1, sy1, x_mid, sy1, x2, sy2);
    }
}

static void plot_circle_outline(int cx, int cy, int x2, int y2, rgb888_t c)
{
    /* cx,cy = centre; x2,y2 = point on circumference (all Agon logical) */
    int dx = x2 - cx, dy = y2 - cy;
    int r = (int)sqrtf((float)(dx*dx + dy*dy));
    int scx = cx, scy = agon_to_screen_y(cy);

    int xx = 0, yy = r, err2 = 1 - r;
    while (xx <= yy) {
        fb_plot_pixel(scx+xx, scy+yy, c, s_gcol_mode);
        fb_plot_pixel(scx-xx, scy+yy, c, s_gcol_mode);
        fb_plot_pixel(scx+xx, scy-yy, c, s_gcol_mode);
        fb_plot_pixel(scx-xx, scy-yy, c, s_gcol_mode);
        fb_plot_pixel(scx+yy, scy+xx, c, s_gcol_mode);
        fb_plot_pixel(scx-yy, scy+xx, c, s_gcol_mode);
        fb_plot_pixel(scx+yy, scy-xx, c, s_gcol_mode);
        fb_plot_pixel(scx-yy, scy-xx, c, s_gcol_mode);
        if (err2 < 0) {
            err2 += 2*xx + 3;
        } else {
            yy--; err2 += 2*(xx-yy) + 5;
        }
        xx++;
    }
}

static void plot_filled_circle(int cx, int cy, int x2, int y2, rgb888_t c)
{
    int dx = x2 - cx, dy = y2 - cy;
    int r = (int)sqrtf((float)(dx*dx + dy*dy));
    int scx = cx, scy = agon_to_screen_y(cy);

    int xx = 0, yy = r, err2 = 1 - r;
    while (xx <= yy) {
        int lx0 = (scx-yy < s_gvp_x0) ? s_gvp_x0 : scx-yy;
        int rx0 = (scx+yy > s_gvp_x1) ? s_gvp_x1 : scx+yy;
        int lx1 = (scx-xx < s_gvp_x0) ? s_gvp_x0 : scx-xx;
        int rx1 = (scx+xx > s_gvp_x1) ? s_gvp_x1 : scx+xx;
        if (scy+xx >= s_gvp_y0 && scy+xx <= s_gvp_y1) fb_fill_hspan(lx0, rx0, scy+xx, c, s_gcol_mode);
        if (scy-xx >= s_gvp_y0 && scy-xx <= s_gvp_y1) fb_fill_hspan(lx0, rx0, scy-xx, c, s_gcol_mode);
        if (scy+yy >= s_gvp_y0 && scy+yy <= s_gvp_y1) fb_fill_hspan(lx1, rx1, scy+yy, c, s_gcol_mode);
        if (scy-yy >= s_gvp_y0 && scy-yy <= s_gvp_y1) fb_fill_hspan(lx1, rx1, scy-yy, c, s_gcol_mode);
        if (err2 < 0) {
            err2 += 2*xx + 3;
        } else {
            yy--; err2 += 2*(xx-yy) + 5;
        }
        xx++;
    }
}

static void plot_flood_fill(int x, int y, rgb888_t target, rgb888_t fill)
{
    /* Simple scanline flood fill in screen space */
    int sy = agon_to_screen_y(y);
    if (x < s_gvp_x0 || x > s_gvp_x1 || sy < s_gvp_y0 || sy > s_gvp_y1) return;

    /* Check if target == fill (nothing to do) */
    if (target.r == fill.r && target.g == fill.g && target.b == fill.b) return;

    /* Stack-based flood fill */
    typedef struct { int16_t x, y; } pt_t;
    int stack_max = (s_gvp_x1-s_gvp_x0+1) * (s_gvp_y1-s_gvp_y0+1);
    pt_t *stack = (pt_t *)heap_caps_malloc(stack_max * sizeof(pt_t), MALLOC_CAP_SPIRAM);
    if (!stack) return;
    int top = 0;
    stack[top++] = {(int16_t)x, (int16_t)sy};
    while (top > 0 && top < stack_max - 4) {
        pt_t p = stack[--top];
        int px = p.x, py = p.y;
        if (px < s_gvp_x0 || px > s_gvp_x1 || py < s_gvp_y0 || py > s_gvp_y1) continue;
        rgb888_t cur = fb_read_pixel_raw(s_off_x + px*s_scale, s_off_y + py*s_scale);
        if (cur.r != target.r || cur.g != target.g || cur.b != target.b) continue;
        fb_plot_pixel(px, py, fill, 0);
        if (px+1 <= s_gvp_x1) stack[top++] = {(int16_t)(px+1), (int16_t)py};
        if (px-1 >= s_gvp_x0) stack[top++] = {(int16_t)(px-1), (int16_t)py};
        if (py+1 <= s_gvp_y1) stack[top++] = {(int16_t)px, (int16_t)(py+1)};
        if (py-1 >= s_gvp_y0) stack[top++] = {(int16_t)px, (int16_t)(py-1)};
    }
    free(stack);
}

static void copy_rect(int x0, int y0, int x1, int y1, int dest_x, int dest_y)
{
    /* Copy rectangle from (x0,y0)-(x1,y1) to dest in screen space */
    int sy0 = agon_to_screen_y(y0), sy1 = agon_to_screen_y(y1);
    if (sy0 > sy1) { int t = sy0; sy0 = sy1; sy1 = t; }
    if (x0 > x1)   { int t = x0;  x0  = x1;  x1  = t;  }
    int sdest_y = agon_to_screen_y(dest_y);
    int w = x1 - x0 + 1, h = sy1 - sy0 + 1;
    for (int dy = 0; dy < h; dy++) {
        for (int dx = 0; dx < w; dx++) {
            rgb888_t c = fb_read_pixel_raw(s_off_x + (x0+dx)*s_scale, s_off_y + (sy0+dy)*s_scale);
            fb_plot_pixel(dest_x + dx, sdest_y + dy, c, 0);
        }
    }
}

/* Execute a PLOT command.
 * cmd: the PLOT command byte (operation | mode bits)
 * x, y: new graphics position (Agon logical, signed)
 * Note: command byte bits:
 *   [7:3] = operation, [2] = 0→relative/1→absolute, [1:0] = draw mode (0=move,1=fg,2=inv,3=bg) */
static void do_plot(uint8_t cmd, int16_t x_raw, int16_t y_raw)
{
    /* VDU coords are in Agon OS units (1280×1024 logical space).
     * Convert to mode pixels using the logical scale factors. */
    int x_px = agon_os_to_px_x(x_raw);
    int y_px = agon_os_to_px_y(y_raw);


    /* Determine absolute coordinates.
     * BBC Micro PLOT command bits [2:0]:
     *   bit 2 = 0 → relative (add to last position)
     *   bit 2 = 1 → absolute (from graphics origin)
     * PLOT 69 (0x45): bit2 = 1 → absolute */
    int abs_x, abs_y;
    if (cmd & 0x04) {
        /* absolute */
        abs_x = x_px + s_gfx_origin_x;
        abs_y = y_px + s_gfx_origin_y;
    } else {
        /* relative */
        abs_x = s_gfx_x + x_px;
        abs_y = s_gfx_y + y_px;
    }

    /* Shift position history (two deep, needed for filled triangles):
     * prev2 ← prev ← current position, then set current = abs */
    int prev_x = s_gfx_x, prev_y = s_gfx_y;
    s_gfx_x_prev2 = s_gfx_x_prev; s_gfx_y_prev2 = s_gfx_y_prev;
    s_gfx_x_prev  = prev_x;       s_gfx_y_prev  = prev_y;
    s_gfx_x = abs_x; s_gfx_y = abs_y;

    /* Determine colour from the bottom 2 bits of the command byte.
     * Agon/BBC Micro spec: colourMode = cmd & 0x03
     *   0 → move only (no draw)
     *   1 → foreground colour
     *   2 → logical inverse (XOR)
     *   3 → background colour
     * PLOT 69 (0x45): 0x45 & 0x03 = 1 → foreground */
    rgb888_t c;
    int colour_mode = cmd & 0x03;
    if (colour_mode == 3) {
        c = s_palette[pal_idx(s_gfx_bg)];  /* background colour */
    } else {
        c = s_palette[pal_idx(s_gfx_fg)];  /* foreground colour (modes 1 and 2) */
    }

    uint8_t op = cmd & 0xF8;  /* upper 5 bits, mask low 3 */

    /* colour_mode 0 = move only (e.g. PLOT 4, PLOT 0x80) — update position
     * but don't draw anything.  Matches console8 behaviour. */
    if (colour_mode == 0) {
        fb_flush();
        return;
    }

    switch (op) {
    case 0x00:  /* Line — all points */
    case 0x08:
        plot_line_omit_last(prev_x, prev_y, abs_x, abs_y, c);
        if (op == 0x00) {
            fb_plot_pixel(abs_x, agon_to_screen_y(abs_y), c, s_gcol_mode);
        }
        break;

    case 0x10:  /* Dot-dash line (simplified: draw solid) */
    case 0x18:
    case 0x20:
    case 0x28:
    case 0x30:
    case 0x38:
        plot_line(prev_x, prev_y, abs_x, abs_y, c);
        break;

    case 0x40:  /* Plot point */
    case 0x48:
        plot_pixel(abs_x, abs_y, c);
        break;

    case 0x50:  /* Filled triangle (v0=prev-prev, v1=prev, v2=current) */
        plot_filled_triangle(s_gfx_x_prev2, s_gfx_y_prev2,
                             s_gfx_x_prev,  s_gfx_y_prev,
                             abs_x, abs_y, c);
        break;

    case 0x58:  /* Fill horizontal line to right until non-bg colour */
        {
            rgb888_t bg = s_palette[pal_idx(s_gfx_bg)];
            int sy = agon_to_screen_y(abs_y);
            for (int ix = abs_x; ix <= s_gvp_x1; ix++) {
                rgb888_t cur = fb_read_pixel_raw(s_off_x + ix*s_scale, s_off_y + sy*s_scale);
                if (cur.r == bg.r && cur.g == bg.g && cur.b == bg.b) break;
                fb_plot_pixel(ix, sy, c, s_gcol_mode);
            }
        }
        break;

    case 0x60:  /* Filled rectangle (prev to abs) */
        plot_filled_rect(prev_x, prev_y, abs_x, abs_y, c);
        break;

    case 0x68:  /* Fill left until fg colour */
        {
            rgb888_t fg = s_palette[pal_idx(s_gfx_fg)];
            int sy = agon_to_screen_y(abs_y);
            for (int ix = abs_x; ix >= s_gvp_x0; ix--) {
                rgb888_t cur = fb_read_pixel_raw(s_off_x + ix*s_scale, s_off_y + sy*s_scale);
                if (cur.r == fg.r && cur.g == fg.g && cur.b == fg.b) break;
                fb_plot_pixel(ix, sy, c, s_gcol_mode);
            }
        }
        break;

    case 0x70:  /* Filled parallelogram (v0=prev-prev, v1=prev, v2=current) */
        plot_filled_triangle(s_gfx_x_prev2, s_gfx_y_prev2, s_gfx_x_prev, s_gfx_y_prev, abs_x, abs_y, c);
        {
            int ox = abs_x - s_gfx_x_prev + s_gfx_x_prev2;
            int oy = abs_y - s_gfx_y_prev + s_gfx_y_prev2;
            plot_filled_triangle(s_gfx_x_prev2, s_gfx_y_prev2, abs_x, abs_y, ox, oy, c);
        }
        break;

    case 0x80:  /* Flood fill to background — fill from abs_x,abs_y */
    case 0x88:
        {
            /* Read the colour at the target pixel to use as fill target */
            rgb888_t target_col = fb_read_pixel_raw(
                s_off_x + abs_x*s_scale, s_off_y + agon_to_screen_y(abs_y)*s_scale);
            plot_flood_fill(abs_x, abs_y, target_col, c);
        }
        break;

    case 0x90:  /* Circle outline */
        plot_circle_outline(prev_x, prev_y, abs_x, abs_y, c);
        break;

    case 0x98:  /* Filled circle */
        plot_filled_circle(prev_x, prev_y, abs_x, abs_y, c);
        break;

    case 0xB8:  /* Copy/move rectangle (prev-prev,prev = src rect, abs = dest) */
        copy_rect(s_gfx_x_prev, s_gfx_y_prev, prev_x, prev_y, abs_x, abs_y);
        break;

    default:
        /* Other plot commands: just update position (already done) */
        break;
    }
    fb_flush();
}

/* ------------------------------------------------------------------ */
/* VDU 23,0 output packet helpers (back-channel to MOS via UART)       */
/*                                                                      */
/* Agon protocol: VDP sends packets back on the same UART that MOS     */
/* uses. Packets start with 0x23 (ASCII '#'?) followed by type byte.   */
/* Our mos_vdp_router feeds these back to MOS via the VDU rx channel.  */
/* ------------------------------------------------------------------ */

/* Callback for sending response bytes to MOS — set during init */
static void (*s_response_cb)(uint8_t) = NULL;

static void vdp_send_byte(uint8_t b)
{
    if (s_response_cb) s_response_cb(b);
}

/* Update the live sysvar block so mos_vdp_internal_get_sysvars() is current */
static void sysvars_update(void)
{
    s_sysvars.scrWidth   = (uint16_t)s_mode_w;
    s_sysvars.scrHeight  = (uint16_t)s_mode_h;
    s_sysvars.scrCols    = (uint8_t)COLS;
    s_sysvars.scrRows    = (uint8_t)ROWS;
    s_sysvars.scrColours = s_mode_colours;
    s_sysvars.scrMode    = s_mode_num;
    /* cursor position in character cells */
    s_sysvars.cursorX    = (uint8_t)s_col;
    s_sysvars.cursorY    = (uint8_t)s_row;
    /* centiseconds since boot — used by BBC BASIC TIME and RND seed */
    s_sysvars.time       = (uint32_t)(esp_timer_get_time() / 10000ULL);
}

/* Send cursor position packet (VDP_CURSOR 0x82) */
static void send_cursor_pos(void)
{
    sysvars_update();
    /* Packet: 0x23, 0x02, col_lo, col_hi, row_lo, row_hi, 0x00, 0x00 */
    vdp_send_byte(0x23);
    vdp_send_byte(0x02);
    vdp_send_byte((uint8_t)(s_col & 0xFF));
    vdp_send_byte((uint8_t)((s_col >> 8) & 0xFF));
    vdp_send_byte((uint8_t)(s_row & 0xFF));
    vdp_send_byte((uint8_t)((s_row >> 8) & 0xFF));
    vdp_send_byte(0); vdp_send_byte(0);
}

/* Send mode information packet (VDP_MODE 0x86) */
static void send_mode_information(void)
{
    sysvars_update();
    /* Packet: 0x23, 0x06, screenW_lo, screenW_hi, screenH_lo, screenH_hi,
     *         cols_lo, cols_hi, rows_lo, rows_hi, colours, mode */
    uint16_t sw = (uint16_t)s_mode_w;
    uint16_t sh = (uint16_t)s_mode_h;
    uint16_t cols = (uint16_t)COLS;
    uint16_t rows = (uint16_t)ROWS;
    vdp_send_byte(0x23);
    vdp_send_byte(0x06);
    vdp_send_byte(sw & 0xFF); vdp_send_byte((sw >> 8) & 0xFF);
    vdp_send_byte(sh & 0xFF); vdp_send_byte((sh >> 8) & 0xFF);
    vdp_send_byte(cols & 0xFF); vdp_send_byte((cols >> 8) & 0xFF);
    vdp_send_byte(rows & 0xFF); vdp_send_byte((rows >> 8) & 0xFF);
    vdp_send_byte(s_mode_colours);
    vdp_send_byte(s_mode_num);
}

/* Send general poll response (VDP_GP 0x80) — used for MOS ready handshake */
static void send_general_poll(uint8_t seq)
{
    vdp_send_byte(0x23);
    vdp_send_byte(0x00);
    vdp_send_byte(seq);
}

/* Send character at cursor position (VDP_SCRCHAR 0x83) */
static void send_scrchar(uint8_t x, uint8_t y)
{
    uint8_t ch = 0x20;
    if (s_ttxt_mode) {
        ch = ttxt_get_char((int)x * ttxt_cell_w(), (int)y * ttxt_cell_h());
    }
    vdp_send_byte(0x23);
    vdp_send_byte(0x03);
    vdp_send_byte(ch);
}

/* Send pixel colour at position (VDP_SCRPIXEL 0x84) */
static void send_scrpixel(int x, int y)
{
    int sy = agon_to_screen_y(y);
    rgb888_t c = {0,0,0};
    if ((unsigned)x < (unsigned)s_mode_w && (unsigned)sy < (unsigned)s_mode_h && s_fb[0]) {
        c = fb_read_pixel_raw(s_off_x + x*s_scale, s_off_y + sy*s_scale);
    }
    vdp_send_byte(0x23);
    vdp_send_byte(0x04);
    vdp_send_byte(c.r); vdp_send_byte(c.g); vdp_send_byte(c.b);
    vdp_send_byte(0);  /* palette index — not tracked */
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

    /* VDU 23 sub-states */
    VDU_STATE_VDU23_1,      /* expecting first byte of VDU23 command */
    VDU_STATE_VDU23_SKIP,   /* skip remaining bytes of unknown VDU23 */

    /* VDU 23,0 system sub-protocol */
    VDU_STATE_VDU23_0_CMD,  /* expecting sub-command byte */
    VDU_STATE_VDU23_0_GP,   /* VDP_GP: 1 byte (sequence number) */
    VDU_STATE_VDU23_0_SCRCHAR_1, /* VDP_SCRCHAR: x */
    VDU_STATE_VDU23_0_SCRCHAR_2, /* VDP_SCRCHAR: y */
    VDU_STATE_VDU23_0_SCRPIXEL_1, /* VDP_SCRPIXEL: xlo */
    VDU_STATE_VDU23_0_SCRPIXEL_2, /* y */
    VDU_STATE_VDU23_0_SCRPIXEL_3,
    VDU_STATE_VDU23_0_SCRPIXEL_4,
    VDU_STATE_VDU23_0_KEYCODE,   /* VDP_KEYCODE: 2 bytes */
    VDU_STATE_VDU23_0_KEYCODE_2,
    VDU_STATE_VDU23_0_KEYSTATE_1, /* VDP_KEYSTATE: 2 bytes */
    VDU_STATE_VDU23_0_KEYSTATE_2,
    VDU_STATE_VDU23_0_MOUSE,      /* VDP_MOUSE: variable, just consume 4 bytes */
    VDU_STATE_VDU23_0_SKIP,       /* skip N bytes of unhandled 23,0 subcmd */

    /* VDU 23,0,0x85 audio sub-protocol (variable length, accumulate) */
    VDU_STATE_VDU23_0_AUDIO_CH,   /* channel byte */
    VDU_STATE_VDU23_0_AUDIO_CMD,  /* command byte */
    VDU_STATE_VDU23_0_AUDIO_ARGS, /* variable arg bytes (see audio_args_needed()) */

    /* VDU 23,1 — cursor enable/disable */
    VDU_STATE_VDU23_1_CURSOR,

    /* VDU 23,6 — set dotted line pattern (8 bytes) */
    VDU_STATE_VDU23_6_SKIP,

    /* VDU 23,7 — scroll rectangle (3 bytes) */
    VDU_STATE_VDU23_7_1,
    VDU_STATE_VDU23_7_2,
    VDU_STATE_VDU23_7_3,

    /* VDU 23,16 — cursor behaviour (2 bytes) */
    VDU_STATE_VDU23_16_1,
    VDU_STATE_VDU23_16_2,

    /* VDU 23,n (UDG redefine, n=32..255) — collect 8 bitmap bytes */
    VDU_STATE_VDU23_UDG,

    /* VDU 23,27 — sprite/bitmap engine */
    VDU_STATE_VDU23_27_CMD,    /* sub-command byte */
    VDU_STATE_VDU23_27_0,      /* cmd 0: bitmap select (1 byte) */
    VDU_STATE_VDU23_27_1_W0,   /* cmd 1: create bitmap from stream — w lo */
    VDU_STATE_VDU23_27_1_W1,   /* w hi */
    VDU_STATE_VDU23_27_1_H0,   /* h lo */
    VDU_STATE_VDU23_27_1_H1,   /* h hi — then receive w*h*4 RGBA bytes */
    VDU_STATE_VDU23_27_1_DATA, /* streaming pixel data */
    VDU_STATE_VDU23_27_2_W0,   /* cmd 2: solid-colour bitmap — w lo */
    VDU_STATE_VDU23_27_2_W1,
    VDU_STATE_VDU23_27_2_H0,
    VDU_STATE_VDU23_27_2_H1,
    VDU_STATE_VDU23_27_2_COL0, /* 4-byte RGBA colour */
    VDU_STATE_VDU23_27_2_COL1,
    VDU_STATE_VDU23_27_2_COL2,
    VDU_STATE_VDU23_27_2_COL3,
    VDU_STATE_VDU23_27_3_X0,   /* cmd 3: draw bitmap — x lo */
    VDU_STATE_VDU23_27_3_X1,
    VDU_STATE_VDU23_27_3_Y0,
    VDU_STATE_VDU23_27_3_Y1,
    VDU_STATE_VDU23_27_4,      /* cmd 4: sprite select (1 byte) */
    VDU_STATE_VDU23_27_6,      /* cmd 6: add frame (1 byte bitmap id) */
    VDU_STATE_VDU23_27_7,      /* cmd 7: activate sprites (1 byte count) */
    VDU_STATE_VDU23_27_10,     /* cmd 10: set frame (1 byte) */
    VDU_STATE_VDU23_27_13_X0,  /* cmd 13: move sprite to x,y */
    VDU_STATE_VDU23_27_13_X1,
    VDU_STATE_VDU23_27_13_Y0,
    VDU_STATE_VDU23_27_13_Y1,
    VDU_STATE_VDU23_27_14_X0,  /* cmd 14: move sprite by dx,dy */
    VDU_STATE_VDU23_27_14_X1,
    VDU_STATE_VDU23_27_14_Y0,
    VDU_STATE_VDU23_27_14_Y1,
    VDU_STATE_VDU23_27_18,     /* cmd 18: set paint mode (1 byte) */
    VDU_STATE_VDU23_27_21,     /* cmd 21: replace frame (1 byte) */
    VDU_STATE_VDU23_27_SKIP,   /* consume remaining bytes of unknown cmd */

    /* VDU 24 — graphics viewport (8 bytes: x0lo,x0hi,y0lo,y0hi, x1lo,x1hi,y1lo,y1hi) */
    VDU_STATE_VDU24_1, VDU_STATE_VDU24_2, VDU_STATE_VDU24_3, VDU_STATE_VDU24_4,
    VDU_STATE_VDU24_5, VDU_STATE_VDU24_6, VDU_STATE_VDU24_7, VDU_STATE_VDU24_8,

    /* VDU 25 — PLOT */
    VDU_STATE_VDU25_1,
    VDU_STATE_VDU25_2,
    VDU_STATE_VDU25_3,
    VDU_STATE_VDU25_4,
    VDU_STATE_VDU25_5,

    /* VDU 28 — text viewport */
    VDU_STATE_VDU28_1, VDU_STATE_VDU28_2, VDU_STATE_VDU28_3, VDU_STATE_VDU28_4,

    /* VDU 29 — graphics origin (4 bytes) */
    VDU_STATE_VDU29_1, VDU_STATE_VDU29_2, VDU_STATE_VDU29_3, VDU_STATE_VDU29_4,

    /* VDU 31 — cursor tab */
    VDU_STATE_VDU31_1,
    VDU_STATE_VDU31_2,
} vdu_state_t;

static vdu_state_t s_vdu_state = VDU_STATE_NORMAL;
static int         s_vdu_skip  = 0;

/* Scratch registers for multi-byte commands */
static uint8_t s_arg[16];
static int     s_arg_idx = 0;

/* UDG being built */
static uint8_t s_udg_char = 0;  /* char code 32..255 */

/* Audio command accumulator (VDU 23,0,0x85)
 * Layout: [0]=channel [1]=cmd [2+]=args (max 16 bytes total)          */
#define AUDIO_BUF_MAX  16
static uint8_t s_audio_buf[AUDIO_BUF_MAX];
static int     s_audio_buf_idx = 0;
static int     s_audio_args_needed = 0;  /* how many arg bytes to collect */

/* Returns how many argument bytes (after channel+cmd) are needed.
 * Returns -1 for unknown/variable length commands (use safe max). */
static int audio_args_needed(uint8_t cmd)
{
    switch (cmd) {
    case AUDIO_CMD_PLAY:         return 5;   /* vol(1) + freq(2) + dur(2) */
    case AUDIO_CMD_STATUS:       return 0;
    case AUDIO_CMD_VOLUME:       return 1;
    case AUDIO_CMD_FREQUENCY:    return 2;
    case AUDIO_CMD_WAVEFORM:     return 1;   /* min; sample waveform needs +2 */
    case AUDIO_CMD_ENV_VOLUME:   return 8;   /* max for ADSR type */
    case AUDIO_CMD_ENV_FREQUENCY:return 4;   /* type(1)+control(1)+step(2) min */
    case AUDIO_CMD_ENABLE:       return 0;
    case AUDIO_CMD_DISABLE:      return 0;
    case AUDIO_CMD_RESET:        return 0;
    case AUDIO_CMD_SEEK:         return 3;
    case AUDIO_CMD_DURATION:     return 3;
    case AUDIO_CMD_SAMPLERATE:   return 2;
    case AUDIO_CMD_SET_PARAM:    return 2;   /* param(1) + value(1) min */
    default:                     return 0;
    }
}

/* Process one VDU byte */
static void vdu_process(uint8_t c);

/* ------------------------------------------------------------------ */
/* Queues and tasks                                                     */
/* ------------------------------------------------------------------ */

#define VDU_QUEUE_LEN   4096
#define KEY_QUEUE_LEN   64

static QueueHandle_t s_vdu_queue = NULL;
static QueueHandle_t s_key_queue = NULL;

static void vdp_render_task(void *arg)
{
    uint8_t byte;
    while (1) {
        if (xQueueReceive(s_vdu_queue, &byte, pdMS_TO_TICKS(16)) == pdTRUE) {
            if (s_fb_mutex) xSemaphoreTake(s_fb_mutex, portMAX_DELAY);
            vdu_process(byte);
            /* Drain the queue fully before flushing — avoids a draw_bitmap
             * call per byte when thousands of PLOTs are in flight. */
            while (xQueueReceive(s_vdu_queue, &byte, 0) == pdTRUE) {
                vdu_process(byte);
            }
            if (s_fb_mutex) xSemaphoreGive(s_fb_mutex);
        }
        /* Flush at ~60Hz from task context — safe for draw_bitmap / DMA */
        if (s_fb_dirty) {
            if (s_fb_mutex) xSemaphoreTake(s_fb_mutex, portMAX_DELAY);
            fb_flush_now();
            if (s_fb_mutex) xSemaphoreGive(s_fb_mutex);
        }
    }
}

/* ------------------------------------------------------------------ */
/* VDU command processor                                                */
/* ------------------------------------------------------------------ */

static void mode_set(uint8_t mode)
{
    /* Agon screen mode table (VDP 1.04+).
     * Indexed by base mode number (bits[6:0]); bit 7 = double-buffer flag.
     * Modes 19-30 are §/§§ extended modes; mode 7 is teletext (no raster). */
    static const struct { int w; int h; uint8_t colours; } mode_dims[] = {
        /* 0 */ {640, 480, 16},
        /* 1 */ {640, 480,  4},
        /* 2 */ {640, 480,  2},
        /* 3 */ {640, 240, 64},
        /* 4 */ {640, 240, 16},
        /* 5 */ {640, 240,  4},
        /* 6 */ {640, 240,  2},
        /* 7 */ {640, 480, 16},  /* teletext — no native raster, fall back to 640×480 */
        /* 8 */ {320, 240, 64},
        /* 9 */ {320, 240, 16},
        /*10 */ {320, 240,  4},
        /*11 */ {320, 240,  2},
        /*12 */ {320, 200, 64},
        /*13 */ {320, 200, 16},
        /*14 */ {320, 200,  4},
        /*15 */ {320, 200,  2},
        /*16 */ {800, 600,  4},
        /*17 */ {800, 600,  2},
        /*18 */ {1024,768,  2},
        /*19 */ {1024,768,  4},
        /*20 */ {512, 384, 64},
        /*21 */ {512, 384, 16},
        /*22 */ {512, 384,  4},
        /*23 */ {512, 384,  2},
        /*24 */ {640, 512, 16},
        /*25 */ {640, 512,  4},
        /*26 */ {640, 512,  2},
        /*27 */ {640, 256, 64},
        /*28 */ {640, 256, 16},
        /*29 */ {640, 256,  4},
        /*30 */ {640, 256,  2},
    };
    /* bit 7 = double-buffer; base mode is bits[6:0] */
    int m = (int)(mode & 0x7F);
    if (m >= (int)(sizeof(mode_dims)/sizeof(mode_dims[0]))) m = 0;

    /* Leave teletext mode if we are switching away from mode 7 */
    if (s_ttxt_mode) {
        s_ttxt_mode = false;
        ttxt_deinit();
    }

    s_mode_w       = mode_dims[m].w;
    s_mode_h       = mode_dims[m].h;
    s_mode_num     = mode;          /* preserve double-buffer bit for 0x86 response */
    s_mode_colours = mode_dims[m].colours;
    scale_update();
    palette_reset();
    /* Default fg=15 (Agon white) in all modes.  pal_idx() wraps it to the
     * correct white slot for the mode: 15%4=3, 15%8=7, 15%16=15, 15%64=15.
     * Previously s_mode_colours-1 was used which gave slot 63 = 0x3E (cyan)
     * in 64-colour modes instead of slot 15 = 0x3F (white). */
    s_fg = 15; s_bg = 0;
    s_gfx_fg = 15; s_gfx_bg = 0;
    s_gcol_mode = 0;
    s_vp_left = 0; s_vp_top = 0;
    s_vp_right = COLS - 1; s_vp_bottom = ROWS - 1;
    s_gvp_x0 = 0; s_gvp_y0 = 0;
    s_gvp_x1 = s_mode_w - 1; s_gvp_y1 = s_mode_h - 1;
    s_gfx_origin_x = 0; s_gfx_origin_y = 0;
    s_gfx_x = 0; s_gfx_y = 0;
    s_gfx_x_prev = 0; s_gfx_y_prev = 0;
    s_gfx_x_prev2 = 0; s_gfx_y_prev2 = 0;
    udg_reset_all();
    sprites_reset_all();
    bitmaps_reset_all();
    clear_screen();
    s_col = 0; s_row = 0;

    /* Mode 7: initialise teletext engine */
    if (m == 7) {
        int rc = ttxt_init(fb_draw(), FB_W, FB_H);
        if (rc == 0) {
            s_ttxt_mode = true;
            ESP_LOGI(TAG, "mode 7: teletext active");
        } else {
            ESP_LOGE(TAG, "mode 7: ttxt_init failed (%d), falling back to normal", rc);
        }
    }

    fb_flush_now();
    /* Notify MOS of mode change */
    send_mode_information();
}

static void vdu_process(uint8_t c)
{
    switch (s_vdu_state) {

    /* ================================================================
     * Normal / printable characters
     * ================================================================ */
    case VDU_STATE_NORMAL:
        if (c >= 0x20 && c != 0x7F) {
            if (s_ttxt_mode) {
                ttxt_draw_char(s_col * ttxt_cell_w(), s_row * ttxt_cell_h(), c);
                /* Advance teletext cursor */
                s_col++;
                if (s_col >= TTXT_COLS) {
                    s_col = 0;
                    s_row++;
                    if (s_row >= TTXT_ROWS) {
                        ttxt_scroll();
                        s_row = TTXT_ROWS - 1;
                    }
                }
            } else {
                draw_char(s_col, s_row, c);
                cursor_advance();
            }
            fb_flush_now();
            return;
        }
        /* In teletext mode, control codes 0x01-0x1F are also fed to the ttxt
         * engine (as teletext attribute codes), EXCEPT for VDU codes that have
         * meaning at the VDU layer (VDU 12=CLS, VDU 10=LF, VDU 13=CR, VDU 8=BS,
         * VDU 22=MODE, VDU 23=sys). */
        if (s_ttxt_mode && c >= 0x01 && c <= 0x1F &&
            c != 0x08 && c != 0x0A && c != 0x0C && c != 0x0D &&
            c != 0x16 && c != 0x17 && c != 0x1F) {
            ttxt_draw_char(s_col * ttxt_cell_w(), s_row * ttxt_cell_h(), c);
            s_col++;
            if (s_col >= TTXT_COLS) {
                s_col = 0;
                s_row++;
                if (s_row >= TTXT_ROWS) {
                    ttxt_scroll();
                    s_row = TTXT_ROWS - 1;
                }
            }
            fb_flush_now();
            return;
        }
        switch (c) {
        case 0x01: /* VDU 1 — send next char to printer (ignored) */     break;
        case 0x02: /* VDU 2 — printer on (ignored) */                    break;
        case 0x03: /* VDU 3 — printer off (ignored) */                   break;
        case 0x04: /* VDU 4 — text cursor mode */                        break;
        case 0x05: /* VDU 5 — graphics cursor mode (ignored) */          break;
        case 0x06: /* VDU 6 — resume VDU (ignored) */                    break;
        case 0x07: /* VDU 7 — BEL (ignored for now, audio TODO) */       break;
        case 0x08: /* VDU 8 — backspace */
            if (s_col > s_vp_left) {
                s_col--;
            } else if (s_row > s_vp_top) {
                s_row--;
                s_col = s_vp_right;
            }
            break;
        case 0x09: /* VDU 9 — cursor right */
            s_col++;
            if (s_col > s_vp_right) s_col = s_vp_right;
            break;
        case 0x0A: /* VDU 10 — LF */
            if (s_ttxt_mode) {
                s_row++;
                if (s_row >= TTXT_ROWS) {
                    ttxt_scroll();
                    s_row = TTXT_ROWS - 1;
                }
            } else {
                newline();
            }
            fb_flush_now();
            break;
        case 0x0B: /* VDU 11 — cursor up */
            if (s_row > s_vp_top) s_row--;
            break;
        case 0x0C: /* VDU 12 — CLS */
            if (s_ttxt_mode) {
                ttxt_cls();
                s_col = 0; s_row = 0;
            } else {
                clear_screen();
                s_col = s_vp_left;
                s_row = s_vp_top;
            }
            fb_flush_now();
            break;
        case 0x0D: /* VDU 13 — CR */
            s_col = s_vp_left;
            break;
        case 0x0E: /* VDU 14 — page mode on (ignored) */   break;
        case 0x0F: /* VDU 15 — page mode off (ignored) */  break;
        case 0x10: /* VDU 16 — CLG (clear graphics viewport in gbg colour) */
            clear_graphics_viewport(s_gfx_bg);
            fb_flush_now();
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
        case 0x14: /* VDU 20 — reset colours */
            palette_reset();
            s_fg = 15; s_bg = 0;
            s_gfx_fg = 15; s_gfx_bg = 0;
            break;
        case 0x15: /* VDU 21 — disable VDU (ignored) */    break;
        case 0x16: /* VDU 22 — MODE n */
            s_vdu_state = VDU_STATE_VDU22;
            break;
        case 0x17: /* VDU 23 — system / redefine char */
            s_vdu_state = VDU_STATE_VDU23_1;
            s_arg_idx = 0;
            break;
        case 0x18: /* VDU 24 — graphics viewport (8 bytes) */
            s_vdu_state = VDU_STATE_VDU24_1;
            s_arg_idx = 0;
            break;
        case 0x19: /* VDU 25 — PLOT (ignored in teletext mode) */
            if (!s_ttxt_mode) {
                s_vdu_state = VDU_STATE_VDU25_1;
                s_arg_idx = 0;
            }
            break;
        case 0x1A: /* VDU 26 — reset text and graphics viewports */
            s_vp_left = 0; s_vp_top = 0;
            s_vp_right = COLS - 1; s_vp_bottom = ROWS - 1;
            s_gvp_x0 = 0; s_gvp_y0 = 0;
            s_gvp_x1 = s_mode_w - 1; s_gvp_y1 = s_mode_h - 1;
            s_col = s_vp_left; s_row = s_vp_top;
            break;
        case 0x1B: /* VDU 27 — escape prefix (ignore the next byte as-is) */
            /* The next byte is passed literally — but we just eat it as a character */
            /* For simplicity, ignore — no printer routing */
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
        case 0x7F: /* DEL / destructive backspace */
            /* Erase character at cursor and move left */
            if (s_col > s_vp_left) {
                s_col--;
                erase_char(s_col, s_row);
                fb_flush_now();
            } else if (s_row > s_vp_top) {
                s_row--;
                s_col = s_vp_right;
                erase_char(s_col, s_row);
                fb_flush_now();
            }
            break;
        default:
            break;
        }
        break;

    /* ---- VDU 17: COLOUR n ---- */
    case VDU_STATE_VDU17:
        if (c < 128) {
            s_fg = pal_idx(c);
        } else {
            s_bg = pal_idx(c - 128);
        }
        s_vdu_state = VDU_STATE_NORMAL;
        break;

    /* ---- VDU 18: GCOL mode, colour (2 bytes) ---- */
    case VDU_STATE_VDU18_1:
        s_gcol_mode = c & 0x07;
        s_vdu_state = VDU_STATE_VDU18_2;
        break;
    case VDU_STATE_VDU18_2:
        if (c < 128) {
            s_gfx_fg = c & 0x3F;
        } else {
            s_gfx_bg = (c - 128) & 0x3F;
        }
        s_vdu_state = VDU_STATE_NORMAL;
        break;

    /* ---- VDU 19: define colour (5 bytes) ---- */
    case VDU_STATE_VDU19_1: s_arg[0] = c; s_vdu_state = VDU_STATE_VDU19_2; break;
    case VDU_STATE_VDU19_2: s_arg[1] = c; s_vdu_state = VDU_STATE_VDU19_3; break;
    case VDU_STATE_VDU19_3: s_arg[2] = c; s_vdu_state = VDU_STATE_VDU19_4; break;
    case VDU_STATE_VDU19_4: s_arg[3] = c; s_vdu_state = VDU_STATE_VDU19_5; break;
    case VDU_STATE_VDU19_5: {
        int idx = pal_idx(s_arg[0]);
        if (s_arg[1] == 16) {
            s_palette[idx].r = s_arg[2];
            s_palette[idx].g = s_arg[3];
            s_palette[idx].b = c;
        } else if (s_arg[1] < 64) {
            uint8_t pi = s_arg[1] & 0x3F;
            s_palette[idx].r = s_palette_lookup6[pi][0];
            s_palette[idx].g = s_palette_lookup6[pi][1];
            s_palette[idx].b = s_palette_lookup6[pi][2];
        }
        s_vdu_state = VDU_STATE_NORMAL;
        break;
    }

    /* ---- VDU 22: MODE n ---- */
    case VDU_STATE_VDU22:
        mode_set(c);
        s_vdu_state = VDU_STATE_NORMAL;
        break;

    /* ================================================================
     * VDU 23 — system commands and UDG redefine
     * ================================================================ */
    case VDU_STATE_VDU23_1:
        s_arg[0] = c;
        if (c == 0) {
            /* VDU 23,0,xx — system sub-protocol */
            s_vdu_state = VDU_STATE_VDU23_0_CMD;
        } else if (c == 1) {
            /* VDU 23,1,b — cursor enable/disable */
            s_vdu_state = VDU_STATE_VDU23_1_CURSOR;
        } else if (c == 6) {
            /* VDU 23,6,b0..b7 — set dotted line pattern */
            s_vdu_skip = 8;
            s_vdu_state = VDU_STATE_VDU23_6_SKIP;
        } else if (c == 7) {
            /* VDU 23,7,extent,direction,movement — scroll rectangle */
            s_vdu_state = VDU_STATE_VDU23_7_1;
            s_arg_idx = 0;
        } else if (c == 16) {
            /* VDU 23,16,setting,mask — cursor behaviour */
            s_vdu_state = VDU_STATE_VDU23_16_1;
        } else if (c == 23) {
            /* VDU 23,23,n — line thickness (skip 1 byte) */
            s_vdu_skip = 1;
            s_vdu_state = VDU_STATE_VDU23_SKIP;
        } else if (c == 27) {
            /* VDU 23,27 — sprite/bitmap engine */
            s_vdu_state = VDU_STATE_VDU23_27_CMD;
        } else if (c >= 32) {
            /* VDU 23,n — UDG redefine (n=32..255): collect 8 bytes */
            s_udg_char = c;
            s_arg_idx = 0;
            s_vdu_state = VDU_STATE_VDU23_UDG;
        } else {
            /* Unknown VDU 23 first byte — skip 8 more bytes */
            s_vdu_skip = 8;
            s_vdu_state = VDU_STATE_VDU23_SKIP;
        }
        break;

    case VDU_STATE_VDU23_SKIP:
        s_vdu_skip--;
        if (s_vdu_skip == 0) s_vdu_state = VDU_STATE_NORMAL;
        break;

    /* ---- VDU 23,0 sub-protocol ---- */
    case VDU_STATE_VDU23_0_CMD:
        s_arg[1] = c;
        switch (c) {
        case 0x80: /* VDP_GP — general poll (1 byte: sequence) */
            s_vdu_state = VDU_STATE_VDU23_0_GP;
            break;
        case 0x81: /* VDP_KEYCODE — keyboard layout (2 bytes) */
            s_vdu_skip = 2; s_vdu_state = VDU_STATE_VDU23_0_SKIP;
            break;
        case 0x82: /* VDP_CURSOR — return cursor position */
            send_cursor_pos();
            s_vdu_state = VDU_STATE_NORMAL;
            break;
        case 0x83: /* VDP_SCRCHAR — read character at x,y (2 bytes) */
            s_vdu_state = VDU_STATE_VDU23_0_SCRCHAR_1;
            break;
        case 0x84: /* VDP_SCRPIXEL — read pixel at x,y (4 bytes) */
            s_vdu_state = VDU_STATE_VDU23_0_SCRPIXEL_1;
            s_arg_idx = 0;
            break;
        case 0x85: /* VDP_AUDIO — audio sub-protocol */
            /* Next byte is the channel number */
            s_audio_buf_idx = 0;
            s_vdu_state = VDU_STATE_VDU23_0_AUDIO_CH;
            break;
        case 0x86: /* VDP_MODE — return mode info */
            send_mode_information();
            s_vdu_state = VDU_STATE_NORMAL;
            break;
        case 0x87: /* VDP_RTC — real-time clock (skip 6 bytes) */
            s_vdu_skip = 6; s_vdu_state = VDU_STATE_VDU23_0_SKIP;
            break;
        case 0x88: /* VDP_KEYSTATE — keyboard repeat + LED state (2 bytes) */
            s_vdu_state = VDU_STATE_VDU23_0_KEYSTATE_1;
            break;
        case 0x89: /* VDP_MOUSE — mouse control (4 bytes min) */
            s_vdu_skip = 4; s_vdu_state = VDU_STATE_VDU23_0_SKIP;
            break;
        case 0x8A: /* VDP_CURSOR_HSTART (2 bytes) */
        case 0x8B: /* VDP_CURSOR_HEND (2 bytes) */
        case 0x0A: /* VDP_CURSOR_VSTART (2 bytes) */
        case 0x0B: /* VDP_CURSOR_VEND (2 bytes) */
            s_vdu_skip = 2; s_vdu_state = VDU_STATE_VDU23_0_SKIP;
            break;
        case 0x8C: /* VDP_CURSOR_MOVE (4 bytes: dx lo/hi, dy lo/hi) */
            s_vdu_skip = 4; s_vdu_state = VDU_STATE_VDU23_0_SKIP;
            break;
        case 0x90: /* VDP_UDG — redefine char in display font (skip 9 bytes) */
            s_vdu_skip = 9; s_vdu_state = VDU_STATE_VDU23_0_SKIP;
            break;
        case 0x91: /* VDP_UDG_RESET — reset all UDGs */
            udg_reset_all();
            s_vdu_state = VDU_STATE_NORMAL;
            break;
        case 0x92: /* VDP_MAP_CHAR_TO_BITMAP (skip 5 bytes) */
            s_vdu_skip = 5; s_vdu_state = VDU_STATE_VDU23_0_SKIP;
            break;
        case 0x93: /* VDP_SCRCHAR_GRAPHICS (skip 4 bytes) */
            s_vdu_skip = 4; s_vdu_state = VDU_STATE_VDU23_0_SKIP;
            break;
        case 0x94: /* VDP_READ_COLOUR (skip 1 byte) */
            s_vdu_skip = 1; s_vdu_state = VDU_STATE_VDU23_0_SKIP;
            break;
        case 0x95: /* VDP_FONT (skip 4 bytes) */
            s_vdu_skip = 4; s_vdu_state = VDU_STATE_VDU23_0_SKIP;
            break;
        case 0x98: /* VDP_CONTROLKEYS (1 byte) */
            s_vdu_skip = 1; s_vdu_state = VDU_STATE_VDU23_0_SKIP;
            break;
        case 0x9A: /* VDP_TEMP_PAGED_MODE (1 byte) */
            s_vdu_skip = 1; s_vdu_state = VDU_STATE_VDU23_0_SKIP;
            break;
        case 0xA0: /* VDP_BUFFERED — skip 4 bytes minimal */
            s_vdu_skip = 4; s_vdu_state = VDU_STATE_VDU23_0_SKIP;
            break;
        case 0xC0: /* VDP_LOGICALCOORDS (1 byte) */
            s_vdu_skip = 1; s_vdu_state = VDU_STATE_VDU23_0_SKIP;
            break;
        case 0xC1: /* VDP_LEGACYMODES (1 byte) */
            s_vdu_skip = 1; s_vdu_state = VDU_STATE_VDU23_0_SKIP;
            break;
        case 0xC3: /* VDP_SWITCHBUFFER — flip double buffers */
            fb_flip_now();
            s_vdu_state = VDU_STATE_NORMAL;
            break;
        case 0xCA: /* VDP_FLUSH_DRAWING_QUEUE */
            fb_flush();
            s_vdu_state = VDU_STATE_NORMAL;
            break;
        case 0xFE: /* VDP_CONSOLEMODE (1 byte) */
            s_vdu_skip = 1; s_vdu_state = VDU_STATE_VDU23_0_SKIP;
            break;
        case 0xFF: /* VDP_TERMINALMODE */
            s_vdu_state = VDU_STATE_NORMAL;
            break;
        default:
            /* Unknown VDU 23,0 sub-command — skip 4 bytes conservatively */
            s_vdu_skip = 4; s_vdu_state = VDU_STATE_VDU23_0_SKIP;
            break;
        }
        break;

    case VDU_STATE_VDU23_0_GP:
        /* Sequence byte for general poll — respond */
        send_general_poll(c);
        s_vdu_state = VDU_STATE_NORMAL;
        break;

    case VDU_STATE_VDU23_0_SCRCHAR_1: s_arg[2] = c; s_vdu_state = VDU_STATE_VDU23_0_SCRCHAR_2; break;
    case VDU_STATE_VDU23_0_SCRCHAR_2:
        send_scrchar(s_arg[2], c);
        s_vdu_state = VDU_STATE_NORMAL;
        break;

    case VDU_STATE_VDU23_0_SCRPIXEL_1: s_arg[2] = c; s_vdu_state = VDU_STATE_VDU23_0_SCRPIXEL_2; break;
    case VDU_STATE_VDU23_0_SCRPIXEL_2: s_arg[3] = c; s_vdu_state = VDU_STATE_VDU23_0_SCRPIXEL_3; break;
    case VDU_STATE_VDU23_0_SCRPIXEL_3: s_arg[4] = c; s_vdu_state = VDU_STATE_VDU23_0_SCRPIXEL_4; break;
    case VDU_STATE_VDU23_0_SCRPIXEL_4: {
        int x = (int16_t)(s_arg[2] | (s_arg[3] << 8));
        int y = (int16_t)(s_arg[4] | (c << 8));
        send_scrpixel(x, y);
        s_vdu_state = VDU_STATE_NORMAL;
        break;
    }

    case VDU_STATE_VDU23_0_KEYSTATE_1: s_arg[2] = c; s_vdu_state = VDU_STATE_VDU23_0_KEYSTATE_2; break;
    case VDU_STATE_VDU23_0_KEYSTATE_2:
        /* Keyboard state — ignore for now */
        s_vdu_state = VDU_STATE_NORMAL;
        break;

    case VDU_STATE_VDU23_0_SKIP:
        s_vdu_skip--;
        if (s_vdu_skip == 0) s_vdu_state = VDU_STATE_NORMAL;
        break;

    /* ---- VDU 23,0,0x85 audio sub-protocol ---- */
    case VDU_STATE_VDU23_0_AUDIO_CH:
        s_audio_buf[0] = c;   /* channel */
        s_audio_buf_idx = 1;
        s_vdu_state = VDU_STATE_VDU23_0_AUDIO_CMD;
        break;

    case VDU_STATE_VDU23_0_AUDIO_CMD:
        s_audio_buf[1] = c;   /* command */
        s_audio_buf_idx = 2;
        s_audio_args_needed = audio_args_needed(c);
        if (s_audio_args_needed == 0) {
            /* No args — dispatch immediately */
            uint8_t status = mos_synth_vdu_audio(s_audio_buf, 2);
            vdp_send_byte(0x23);
            vdp_send_byte(0x05);   /* PACKET_AUDIO */
            vdp_send_byte(s_audio_buf[0]);
            vdp_send_byte(status);
            s_vdu_state = VDU_STATE_NORMAL;
        } else {
            s_vdu_state = VDU_STATE_VDU23_0_AUDIO_ARGS;
        }
        break;

    case VDU_STATE_VDU23_0_AUDIO_ARGS:
        if (s_audio_buf_idx < AUDIO_BUF_MAX) {
            s_audio_buf[s_audio_buf_idx++] = c;
        }
        if (s_audio_buf_idx >= 2 + s_audio_args_needed) {
            /* All args collected — dispatch */
            uint8_t status = mos_synth_vdu_audio(s_audio_buf, s_audio_buf_idx);
            vdp_send_byte(0x23);
            vdp_send_byte(0x05);   /* PACKET_AUDIO */
            vdp_send_byte(s_audio_buf[0]);
            vdp_send_byte(status);
            s_vdu_state = VDU_STATE_NORMAL;
        }
        break;

    /* ---- VDU 23,1 — cursor enable/disable ---- */
    case VDU_STATE_VDU23_1_CURSOR:
        s_cursor_visible = (c != 0);
        s_vdu_state = VDU_STATE_NORMAL;
        break;

    /* ---- VDU 23,6 — dotted line pattern (8 bytes, ignore) ---- */
    case VDU_STATE_VDU23_6_SKIP:
        s_vdu_skip--;
        if (s_vdu_skip == 0) s_vdu_state = VDU_STATE_NORMAL;
        break;

    /* ---- VDU 23,7 — scroll rectangle ---- */
    case VDU_STATE_VDU23_7_1: s_arg[1] = c; s_vdu_state = VDU_STATE_VDU23_7_2; break;
    case VDU_STATE_VDU23_7_2: s_arg[2] = c; s_vdu_state = VDU_STATE_VDU23_7_3; break;
    case VDU_STATE_VDU23_7_3:
        /* extent=arg[1], direction=arg[2], movement=c */
        /* Simplified: only support full-viewport scroll up (direction=0, extent=0) */
        if (s_arg[2] == 0) scroll_up();
        s_vdu_state = VDU_STATE_NORMAL;
        break;

    /* ---- VDU 23,16 — cursor behaviour (2 bytes, ignore) ---- */
    case VDU_STATE_VDU23_16_1: s_arg[1] = c; s_vdu_state = VDU_STATE_VDU23_16_2; break;
    case VDU_STATE_VDU23_16_2:
        /* setting=arg[1], mask=c — ignore for now */
        s_vdu_state = VDU_STATE_NORMAL;
        break;

    /* ---- VDU 23,n — UDG redefine (8 bytes) ---- */
    case VDU_STATE_VDU23_UDG:
        if (s_udg) {
            s_udg[s_udg_char - 32][s_arg_idx] = c;
        }
        s_arg_idx++;
        if (s_arg_idx >= 8) {
            if (s_udg) udg_set_defined(s_udg_char);
            s_vdu_state = VDU_STATE_NORMAL;
        }
        break;

    /* ================================================================
     * VDU 23,27 — Sprite / Bitmap engine
     * ================================================================ */

    /* Sub-command dispatcher */
    case VDU_STATE_VDU23_27_CMD:
        s_arg[0] = c;   /* save sub-command for multi-byte states */
        s_arg_idx = 0;
        switch (c) {
        case 0:    s_vdu_state = VDU_STATE_VDU23_27_0;      break; /* select bitmap */
        case 1:    s_vdu_state = VDU_STATE_VDU23_27_1_W0;   break; /* create bitmap from stream */
        case 2:    s_vdu_state = VDU_STATE_VDU23_27_2_W0;   break; /* create solid-colour bitmap */
        case 3:    s_vdu_state = VDU_STATE_VDU23_27_3_X0;   break; /* draw bitmap at x,y */
        case 4:    s_vdu_state = VDU_STATE_VDU23_27_4;      break; /* select sprite */
        case 5:    /* clear sprite frames (no params) */
            if (s_cur_sprite < SPRITE_MAX) {
                s_sprites[s_cur_sprite].num_frames = 0;
                s_sprites[s_cur_sprite].cur_frame  = 0;
                s_sprites[s_cur_sprite].visible    = false;
            }
            s_vdu_state = VDU_STATE_NORMAL;
            break;
        case 6:    s_vdu_state = VDU_STATE_VDU23_27_6;      break; /* add frame */
        case 7:    s_vdu_state = VDU_STATE_VDU23_27_7;      break; /* activate sprites */
        case 8:    /* next frame */
            if (s_cur_sprite < SPRITE_MAX) {
                sprite_t *sp = &s_sprites[s_cur_sprite];
                if (sp->num_frames > 0)
                    sp->cur_frame = (uint8_t)((sp->cur_frame + 1) % sp->num_frames);
            }
            s_vdu_state = VDU_STATE_NORMAL;
            break;
        case 9:    /* previous frame */
            if (s_cur_sprite < SPRITE_MAX) {
                sprite_t *sp = &s_sprites[s_cur_sprite];
                if (sp->num_frames > 0)
                    sp->cur_frame = (uint8_t)(sp->cur_frame ? sp->cur_frame - 1 : sp->num_frames - 1);
            }
            s_vdu_state = VDU_STATE_NORMAL;
            break;
        case 10:   s_vdu_state = VDU_STATE_VDU23_27_10;     break; /* set frame id */
        case 11:   /* show sprite */
            if (s_cur_sprite < SPRITE_MAX) s_sprites[s_cur_sprite].visible = true;
            s_vdu_state = VDU_STATE_NORMAL;
            break;
        case 12:   /* hide sprite */
            if (s_cur_sprite < SPRITE_MAX) s_sprites[s_cur_sprite].visible = false;
            s_vdu_state = VDU_STATE_NORMAL;
            break;
        case 13:   s_vdu_state = VDU_STATE_VDU23_27_13_X0;  break; /* move to x,y */
        case 14:   s_vdu_state = VDU_STATE_VDU23_27_14_X0;  break; /* move by dx,dy */
        case 15:   /* refresh — nothing to do (render happens each frame) */
            s_vdu_state = VDU_STATE_NORMAL;
            break;
        case 16:   /* reset sprites + bitmaps + CLS */
            sprites_reset_all();
            bitmaps_reset_all();
            s_vdu_state = VDU_STATE_NORMAL;
            break;
        case 17:   /* reset sprites only */
            sprites_reset_all();
            s_vdu_state = VDU_STATE_NORMAL;
            break;
        case 18:   s_vdu_state = VDU_STATE_VDU23_27_18;     break; /* set paint mode */
        case 19:   /* set hardware sprite — no-op (software only) */
        case 20:   /* set software sprite — no-op */
            s_vdu_state = VDU_STATE_NORMAL;
            break;
        case 21:   s_vdu_state = VDU_STATE_VDU23_27_21;     break; /* replace frame */
        case 0x20: /* select bitmap 16-bit — reuse s_arg, next byte is lo */
            s_vdu_state = VDU_STATE_VDU23_27_0;  /* 1-byte (lo); hi ignored */
            break;
        default:
            /* Unknown — skip 4 bytes (safe minimum for any 2-word args) */
            s_vdu_skip = 4;
            s_vdu_state = VDU_STATE_VDU23_27_SKIP;
            break;
        }
        break;

    /* cmd 0: select bitmap (1 byte) */
    case VDU_STATE_VDU23_27_0:
        s_cur_bitmap = c;
        ESP_LOGD(TAG, "spr: bitmap %d selected", s_cur_bitmap);
        s_vdu_state = VDU_STATE_NORMAL;
        break;

    /* cmd 1: create bitmap from RGBA stream — w lo,hi, h lo,hi, then w*h*4 bytes */
    case VDU_STATE_VDU23_27_1_W0: s_arg[1]=c; s_vdu_state = VDU_STATE_VDU23_27_1_W1; break;
    case VDU_STATE_VDU23_27_1_W1: s_arg[2]=c; s_vdu_state = VDU_STATE_VDU23_27_1_H0; break;
    case VDU_STATE_VDU23_27_1_H0: s_arg[3]=c; s_vdu_state = VDU_STATE_VDU23_27_1_H1; break;
    case VDU_STATE_VDU23_27_1_H1: {
        uint16_t bw = (uint16_t)(s_arg[1] | (s_arg[2] << 8));
        uint16_t bh = (uint16_t)(s_arg[3] | (c       << 8));
        if (bh == 0) {
            /* h=0: capture from screen — not yet implemented, skip */
            s_vdu_state = VDU_STATE_NORMAL;
        } else if (bitmap_alloc(s_cur_bitmap, bw, bh)) {
            s_arg_idx = 0;   /* byte counter within pixel data */
            /* store total bytes to receive in s_vdu_skip (repurposed) */
            s_vdu_skip = (int)((uint32_t)bw * bh * 4);
            s_vdu_state = VDU_STATE_VDU23_27_1_DATA;
            ESP_LOGD(TAG, "spr: receiving bitmap %d %ux%u (%d bytes)",
                     s_cur_bitmap, bw, bh, s_vdu_skip);
        } else {
            /* alloc failed — skip the data stream */
            s_vdu_skip = (int)((uint32_t)bw * bh * 4);
            s_vdu_state = VDU_STATE_VDU23_27_SKIP;
        }
        break;
    }
    case VDU_STATE_VDU23_27_1_DATA:
        s_bitmaps[s_cur_bitmap].data[s_arg_idx++] = c;
        s_vdu_skip--;
        if (s_vdu_skip == 0) {
            ESP_LOGD(TAG, "spr: bitmap %d received OK", s_cur_bitmap);
            s_vdu_state = VDU_STATE_NORMAL;
        }
        break;

    /* cmd 2: solid-colour bitmap — w lo,hi, h lo,hi, then 4 RGBA bytes */
    case VDU_STATE_VDU23_27_2_W0: s_arg[1]=c; s_vdu_state = VDU_STATE_VDU23_27_2_W1; break;
    case VDU_STATE_VDU23_27_2_W1: s_arg[2]=c; s_vdu_state = VDU_STATE_VDU23_27_2_H0; break;
    case VDU_STATE_VDU23_27_2_H0: s_arg[3]=c; s_vdu_state = VDU_STATE_VDU23_27_2_H1; break;
    case VDU_STATE_VDU23_27_2_H1: s_arg[4]=c; s_vdu_state = VDU_STATE_VDU23_27_2_COL0; break;
    case VDU_STATE_VDU23_27_2_COL0: s_arg[5]=c; s_vdu_state = VDU_STATE_VDU23_27_2_COL1; break;
    case VDU_STATE_VDU23_27_2_COL1: s_arg[6]=c; s_vdu_state = VDU_STATE_VDU23_27_2_COL2; break;
    case VDU_STATE_VDU23_27_2_COL2: s_arg[7]=c; s_vdu_state = VDU_STATE_VDU23_27_2_COL3; break;
    case VDU_STATE_VDU23_27_2_COL3: {
        uint16_t bw = (uint16_t)(s_arg[1] | (s_arg[2] << 8));
        uint16_t bh = (uint16_t)(s_arg[3] | (s_arg[4] << 8));
        rgba_t   col = { s_arg[5], s_arg[6], s_arg[7], c };
        if (bitmap_alloc(s_cur_bitmap, bw, bh)) {
            rgba_t *p = (rgba_t *)s_bitmaps[s_cur_bitmap].data;
            uint32_t n = (uint32_t)bw * bh;
            for (uint32_t i = 0; i < n; i++) p[i] = col;
            ESP_LOGD(TAG, "spr: solid bitmap %d %ux%u rgba(%d,%d,%d,%d)",
                     s_cur_bitmap, bw, bh, col.r, col.g, col.b, col.a);
        }
        s_vdu_state = VDU_STATE_NORMAL;
        break;
    }

    /* cmd 3: draw current bitmap at logical x,y (2+2 signed words) */
    case VDU_STATE_VDU23_27_3_X0: s_arg[1]=c; s_vdu_state = VDU_STATE_VDU23_27_3_X1; break;
    case VDU_STATE_VDU23_27_3_X1: s_arg[2]=c; s_vdu_state = VDU_STATE_VDU23_27_3_Y0; break;
    case VDU_STATE_VDU23_27_3_Y0: s_arg[3]=c; s_vdu_state = VDU_STATE_VDU23_27_3_Y1; break;
    case VDU_STATE_VDU23_27_3_Y1: {
        int bx = (int16_t)(s_arg[1] | (s_arg[2] << 8));
        int by = (int16_t)(s_arg[3] | (c        << 8));
        if (xSemaphoreTake(s_fb_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            uint8_t *fb = fb_draw();  /* draw to back buffer */
            if (fb) bitmap_draw_to_fb(fb, s_cur_bitmap, bx, by, 1);
            xSemaphoreGive(s_fb_mutex);
        }
        ESP_LOGD(TAG, "spr: draw bitmap %d at (%d,%d)", s_cur_bitmap, bx, by);
        s_vdu_state = VDU_STATE_NORMAL;
        break;
    }

    /* cmd 4: select sprite (1 byte) */
    case VDU_STATE_VDU23_27_4:
        s_cur_sprite = (c < SPRITE_MAX) ? c : 0;
        ESP_LOGD(TAG, "spr: sprite %d selected", s_cur_sprite);
        s_vdu_state = VDU_STATE_NORMAL;
        break;

    /* cmd 6: add bitmap as next frame of current sprite (1 byte bitmap id) */
    case VDU_STATE_VDU23_27_6: {
        sprite_t *sp = &s_sprites[s_cur_sprite < SPRITE_MAX ? s_cur_sprite : 0];
        if (sp->num_frames < SPRITE_FRAMES) {
            sp->frame_ids[sp->num_frames++] = c;
            ESP_LOGD(TAG, "spr: sprite %d add frame bitmap %d (total %d)",
                     s_cur_sprite, c, sp->num_frames);
        }
        s_vdu_state = VDU_STATE_NORMAL;
        break;
    }

    /* cmd 7: activate N sprites (1 byte) */
    case VDU_STATE_VDU23_27_7:
        s_num_active_sprites = (c <= SPRITE_MAX) ? c : SPRITE_MAX;
        ESP_LOGD(TAG, "spr: %d sprites activated", s_num_active_sprites);
        s_vdu_state = VDU_STATE_NORMAL;
        break;

    /* cmd 10: set frame id on current sprite (1 byte) */
    case VDU_STATE_VDU23_27_10: {
        sprite_t *sp = &s_sprites[s_cur_sprite < SPRITE_MAX ? s_cur_sprite : 0];
        if (c < sp->num_frames) sp->cur_frame = c;
        s_vdu_state = VDU_STATE_NORMAL;
        break;
    }

    /* cmd 13: move sprite to absolute position (2 signed words) */
    case VDU_STATE_VDU23_27_13_X0: s_arg[1]=c; s_vdu_state = VDU_STATE_VDU23_27_13_X1; break;
    case VDU_STATE_VDU23_27_13_X1: s_arg[2]=c; s_vdu_state = VDU_STATE_VDU23_27_13_Y0; break;
    case VDU_STATE_VDU23_27_13_Y0: s_arg[3]=c; s_vdu_state = VDU_STATE_VDU23_27_13_Y1; break;
    case VDU_STATE_VDU23_27_13_Y1: {
        sprite_t *sp = &s_sprites[s_cur_sprite < SPRITE_MAX ? s_cur_sprite : 0];
        sp->x = (int16_t)(s_arg[1] | (s_arg[2] << 8));
        sp->y = (int16_t)(s_arg[3] | (c        << 8));
        ESP_LOGD(TAG, "spr: sprite %d move to (%d,%d)", s_cur_sprite, sp->x, sp->y);
        s_vdu_state = VDU_STATE_NORMAL;
        break;
    }

    /* cmd 14: move sprite by relative offset (2 signed words) */
    case VDU_STATE_VDU23_27_14_X0: s_arg[1]=c; s_vdu_state = VDU_STATE_VDU23_27_14_X1; break;
    case VDU_STATE_VDU23_27_14_X1: s_arg[2]=c; s_vdu_state = VDU_STATE_VDU23_27_14_Y0; break;
    case VDU_STATE_VDU23_27_14_Y0: s_arg[3]=c; s_vdu_state = VDU_STATE_VDU23_27_14_Y1; break;
    case VDU_STATE_VDU23_27_14_Y1: {
        sprite_t *sp = &s_sprites[s_cur_sprite < SPRITE_MAX ? s_cur_sprite : 0];
        sp->x = (int16_t)(sp->x + (int16_t)(s_arg[1] | (s_arg[2] << 8)));
        sp->y = (int16_t)(sp->y + (int16_t)(s_arg[3] | (c        << 8)));
        ESP_LOGD(TAG, "spr: sprite %d move by to (%d,%d)", s_cur_sprite, sp->x, sp->y);
        s_vdu_state = VDU_STATE_NORMAL;
        break;
    }

    /* cmd 18: set paint mode (0=opaque, 1=alpha-blend) */
    case VDU_STATE_VDU23_27_18:
        if (s_cur_sprite < SPRITE_MAX) s_sprites[s_cur_sprite].paint_mode = c & 1;
        s_vdu_state = VDU_STATE_NORMAL;
        break;

    /* cmd 21: replace current frame with bitmap (1 byte) */
    case VDU_STATE_VDU23_27_21: {
        sprite_t *sp = &s_sprites[s_cur_sprite < SPRITE_MAX ? s_cur_sprite : 0];
        if (sp->num_frames > 0) sp->frame_ids[sp->cur_frame] = c;
        s_vdu_state = VDU_STATE_NORMAL;
        break;
    }

    /* catch-all skip for unimplemented sub-commands */
    case VDU_STATE_VDU23_27_SKIP:
        s_vdu_skip--;
        if (s_vdu_skip == 0) s_vdu_state = VDU_STATE_NORMAL;
        break;

    /* ================================================================
     * VDU 24: graphics viewport (8 bytes: x0lo,x0hi,y0lo,y0hi,x1lo,x1hi,y1lo,y1hi)
     * ================================================================ */
    case VDU_STATE_VDU24_1: s_arg[0] = c; s_vdu_state = VDU_STATE_VDU24_2; break;
    case VDU_STATE_VDU24_2: s_arg[1] = c; s_vdu_state = VDU_STATE_VDU24_3; break;
    case VDU_STATE_VDU24_3: s_arg[2] = c; s_vdu_state = VDU_STATE_VDU24_4; break;
    case VDU_STATE_VDU24_4: s_arg[3] = c; s_vdu_state = VDU_STATE_VDU24_5; break;
    case VDU_STATE_VDU24_5: s_arg[4] = c; s_vdu_state = VDU_STATE_VDU24_6; break;
    case VDU_STATE_VDU24_6: s_arg[5] = c; s_vdu_state = VDU_STATE_VDU24_7; break;
    case VDU_STATE_VDU24_7: s_arg[6] = c; s_vdu_state = VDU_STATE_VDU24_8; break;
    case VDU_STATE_VDU24_8: {
        /* VDU 24 coords are in Agon OS units (1280×1024) — convert to pixels */
        int16_t rx0 = (int16_t)(s_arg[0] | (s_arg[1] << 8));
        int16_t ry0 = (int16_t)(s_arg[2] | (s_arg[3] << 8));
        int16_t rx1 = (int16_t)(s_arg[4] | (s_arg[5] << 8));
        int16_t ry1 = (int16_t)(s_arg[6] | (c << 8));
        int16_t x0 = (int16_t)agon_os_to_px_x(rx0);
        int16_t y0 = (int16_t)agon_os_to_px_y(ry0);
        int16_t x1 = (int16_t)agon_os_to_px_x(rx1);
        int16_t y1 = (int16_t)agon_os_to_px_y(ry1);
        /* Agon coords: x0,y0 = bottom-left, x1,y1 = top-right (pixels)
         * Convert to screen space (y flipped) */
        int sx0 = (int)x0, sx1 = (int)x1;
        int sy0 = agon_to_screen_y((int)y1); /* top of screen = high agon y */
        int sy1 = agon_to_screen_y((int)y0); /* bottom of screen = low agon y */
        if (sx0 > sx1) { int t = sx0; sx0 = sx1; sx1 = t; }
        if (sy0 > sy1) { int t = sy0; sy0 = sy1; sy1 = t; }
        /* Clamp to mode */
        if (sx0 < 0) sx0 = 0;
        if (sx1 >= s_mode_w) sx1 = s_mode_w - 1;
        if (sy0 < 0) sy0 = 0;
        if (sy1 >= s_mode_h) sy1 = s_mode_h - 1;
        s_gvp_x0 = sx0; s_gvp_y0 = sy0;
        s_gvp_x1 = sx1; s_gvp_y1 = sy1;
        s_vdu_state = VDU_STATE_NORMAL;
        break;
    }

    /* ================================================================
     * VDU 25: PLOT k, x16, y16 (5 bytes total)
     * ================================================================ */
    case VDU_STATE_VDU25_1: s_arg[0] = c; s_vdu_state = VDU_STATE_VDU25_2; break;
    case VDU_STATE_VDU25_2: s_arg[1] = c; s_vdu_state = VDU_STATE_VDU25_3; break;
    case VDU_STATE_VDU25_3: s_arg[2] = c; s_vdu_state = VDU_STATE_VDU25_4; break;
    case VDU_STATE_VDU25_4: s_arg[3] = c; s_vdu_state = VDU_STATE_VDU25_5; break;
    case VDU_STATE_VDU25_5: {
        int16_t x = (int16_t)(s_arg[1] | (s_arg[2] << 8));
        int16_t y = (int16_t)(s_arg[3] | (c << 8));
        do_plot(s_arg[0], x, y);
        s_vdu_state = VDU_STATE_NORMAL;
        break;
    }

    /* ================================================================
     * VDU 28: text viewport (4 bytes: left, bottom, right, top)
     * ================================================================ */
    case VDU_STATE_VDU28_1: s_arg[0] = c; s_vdu_state = VDU_STATE_VDU28_2; break;
    case VDU_STATE_VDU28_2: s_arg[1] = c; s_vdu_state = VDU_STATE_VDU28_3; break;
    case VDU_STATE_VDU28_3: s_arg[2] = c; s_vdu_state = VDU_STATE_VDU28_4; break;
    case VDU_STATE_VDU28_4:
        s_vp_left   = s_arg[0]; if (s_vp_left   < 0)       s_vp_left   = 0;
        s_vp_right  = s_arg[2]; if (s_vp_right  >= COLS)   s_vp_right  = COLS - 1;
        s_vp_top    = c;        if (s_vp_top    < 0)       s_vp_top    = 0;
        s_vp_bottom = s_arg[1]; if (s_vp_bottom >= ROWS)   s_vp_bottom = ROWS - 1;
        s_col = s_vp_left;
        s_row = s_vp_top;
        s_vdu_state = VDU_STATE_NORMAL;
        break;

    /* ================================================================
     * VDU 29: graphics origin (4 bytes: xlo, xhi, ylo, yhi)
     * ================================================================ */
    case VDU_STATE_VDU29_1: s_arg[0] = c; s_vdu_state = VDU_STATE_VDU29_2; break;
    case VDU_STATE_VDU29_2: s_arg[1] = c; s_vdu_state = VDU_STATE_VDU29_3; break;
    case VDU_STATE_VDU29_3: s_arg[2] = c; s_vdu_state = VDU_STATE_VDU29_4; break;
    case VDU_STATE_VDU29_4:
        /* VDU 29 coords are in Agon OS units (1280×1024) — convert to pixels */
        s_gfx_origin_x = agon_os_to_px_x((int16_t)(s_arg[0] | (s_arg[1] << 8)));
        s_gfx_origin_y = agon_os_to_px_y((int16_t)(s_arg[2] | (c << 8)));
        s_vdu_state = VDU_STATE_NORMAL;
        break;

    /* ================================================================
     * VDU 31: cursor to col, row
     * ================================================================ */
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
    bool extended;
    bool release;
    bool ext_release;
} kbd_state_t;

static kbd_state_t s_kbd = {false, false, false, false, false, false, false};

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

    if (!extended) {
        if (byte == PS2_LSHIFT || byte == PS2_RSHIFT) {
            s_kbd.shift = !release; return;
        }
        if (byte == PS2_LCTRL) { s_kbd.ctrl = !release; return; }
        if (byte == PS2_LALT)  { s_kbd.alt  = !release; return; }
        if (byte == PS2_CAPS && !release) { s_kbd.caps = !s_kbd.caps; return; }
    } else {
        if (byte == 0x14) { s_kbd.ctrl = !release; return; }
        if (byte == 0x11) { s_kbd.alt  = !release; return; }
        if (release) return;
        uint8_t ascii = 0;
        switch (byte) {
        case 0x75: ascii = 0xC1; break; /* up arrow    */
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
    if (byte >= 128) return;

    bool effective_shift = s_kbd.shift;
    if (s_kbd.caps) {
        uint8_t base = s_ps2_unshifted[byte];
        if (base >= 'a' && base <= 'z') effective_shift = !effective_shift;
    }

    uint8_t ascii = effective_shift ? s_ps2_shifted[byte] : s_ps2_unshifted[byte];
    if (ascii == 0) return;

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

    if (s_panel) {
        void *fb0 = NULL, *fb1 = NULL;
        esp_err_t ret = esp_lcd_dpi_panel_get_frame_buffer(s_panel, 2, &fb0, &fb1);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "get_frame_buffer failed (0x%x) — HDMI output disabled", ret);
            s_panel = NULL;
        } else {
            s_fb[0] = (uint8_t *)fb0;
            s_fb[1] = (uint8_t *)fb1;
            ESP_LOGI(TAG, "Framebuffers: fb0=%p fb1=%p (%u bytes each)",
                     s_fb[0], s_fb[1], (unsigned)FB_SIZE);
            /* Verify dpi_set_cur_fb offset (49 on RV32 IDF5.x):
             * read cur_fb_index — must be 0 at init, num_fbs must be 2. */
            {
                volatile uint8_t *raw = (volatile uint8_t *)s_panel;
                uint8_t cur = raw[49], num = raw[50];
                if (cur == 0 && num == 2) {
                    ESP_LOGI(TAG, "dpi_set_cur_fb offset=49 verified (cur=%d num_fbs=%d)", cur, num);
                } else {
                    ESP_LOGE(TAG, "dpi_set_cur_fb offset MISMATCH: raw[49]=%d raw[50]=%d (expected 0,2)", cur, num);
                }
            }
        }
    } else {
        ESP_LOGW(TAG, "No DPI panel — HDMI output disabled");
    }

    /* Allocate UDG table in PSRAM */
    udg_init();

    /* Reset state — boot into mode 0 (640×480, 16 colours) */
    s_mode_w = 640; s_mode_h = 480;
    s_mode_num = 0; s_mode_colours = 16;
    scale_update();
    palette_reset();
    s_fg = 15; s_bg = 0;
    s_gfx_fg = 15; s_gfx_bg = 0;
    s_gcol_mode = 0;
    s_vp_left = 0; s_vp_top = 0;
    s_vp_right = COLS - 1; s_vp_bottom = ROWS - 1;
    s_gvp_x0 = 0; s_gvp_y0 = 0;
    s_gvp_x1 = s_mode_w - 1; s_gvp_y1 = s_mode_h - 1;
    s_gfx_origin_x = 0; s_gfx_origin_y = 0;
    s_gfx_x = 0; s_gfx_y = 0;
    s_gfx_x_prev = 0; s_gfx_y_prev = 0;
    s_gfx_x_prev2 = 0; s_gfx_y_prev2 = 0;
    s_col = 0; s_row = 0;
    s_vdu_state = VDU_STATE_NORMAL;
    s_cursor_visible = true;
    s_kbd = {false, false, false, false, false, false, false};
    udg_reset_all();

    /* Clear both framebuffers */
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

    s_fb_mutex = xSemaphoreCreateMutex();
    if (!s_fb_mutex) {
        ESP_LOGE(TAG, "Failed to create framebuffer mutex");
        return ESP_ERR_NO_MEM;
    }

    /* Periodic flush timer: push dirty framebuffer to display at ~60 Hz.
     * This decouples PLOT throughput from cache_msync cost — individual
     * PLOT commands just mark s_fb_dirty; the timer does the actual sync. */
    esp_timer_create_args_t timer_args = {
        .callback        = fb_flush_timer_cb,
        .arg             = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name            = "vdp_flush",
        .skip_unhandled_events = true,
    };
    esp_err_t terr = esp_timer_create(&timer_args, &s_flush_timer);
    if (terr == ESP_OK) {
        esp_timer_start_periodic(s_flush_timer, 16667); /* ~60 Hz */
    } else {
        ESP_LOGW(TAG, "Failed to create flush timer (0x%x) — falling back to per-PLOT flush", terr);
    }

    BaseType_t ok = xTaskCreatePinnedToCore(
        vdp_render_task, "vdp_render",
        CONFIG_MOS_VDP_INTERNAL_TASK_STACK_KB * 1024,
        NULL, 5, NULL, 1);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create render task");
        return ESP_ERR_NO_MEM;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "Internal VDP ready — fb=%dx%d mode=%dx%d scale=%d off=(%d,%d) (%d cols×%d rows)",
             FB_W, FB_H, s_mode_w, s_mode_h, s_scale, s_off_x, s_off_y, COLS, ROWS);
    return ESP_OK;
}

void mos_vdp_internal_set_response_cb(void (*cb)(uint8_t))
{
    s_response_cb = cb;
}

void mos_vdp_internal_putch(uint8_t c)
{
    if (!s_vdu_queue) return;
    /* Block until space is available — natural backpressure like TCP send().
     * user_prog slows down to the render rate instead of dropping VDU bytes.
     * IDLE WDT check is disabled so blocking here is safe. */
    xQueueSend(s_vdu_queue, &c, portMAX_DELAY);
}

int mos_vdp_internal_getch(void)
{
    if (!s_key_queue) return -1;
    uint8_t c = 0;
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
    return true;
}

void mos_vdp_internal_flush(void)
{
    /* No-op: rendering is driven by the render task */
}

t_mos_sysvars *mos_vdp_internal_get_sysvars(void)
{
    sysvars_update();
    return &s_sysvars;
}
