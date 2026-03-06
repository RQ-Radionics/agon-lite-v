/*
 * mos_vdp_ttxt.c — Teletext (mode 7) backend for ESP32-P4 VDP internal
 *
 * Implements Agon mode 7 character-cell rendering directly into the RGB888
 * framebuffer.  Ported from agon-vdp/video/agon_ttxt.h (Lennart Benschop,
 * 2023) to plain C with no fabgl dependency.
 *
 * Screen layout:  40 columns × 25 rows
 * Cell size:      16 × 19 px (canvas height < 500) or 16 × 20 px (>= 500)
 *
 * Font data: vdp_ttxt_font.h — 256 glyphs × 20 rows of uint16_t bitmaps.
 * Each row encodes 16 pixels: low byte = pixels 0-7, high byte = pixels 8-15.
 *
 * Process-line semantics are identical to the original:
 *   - Control codes in range 0x01-0x1F are processed left-to-right.
 *   - Most codes take effect in the NEXT cell (display uses previous state).
 *   - A few codes (0x09, 0x0C, 0x0D, 0x18, 0x1C, 0x1D, 0x1E) take immediate
 *     effect in the same cell (for background colour and hold-graphics).
 */

#include "mos_vdp_ttxt.h"
#include "vdp_ttxt_font.h"

#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "esp_heap_caps.h"
#include "esp_log.h"

static const char *TAG = "vdp_ttxt";

/* ------------------------------------------------------------------ */
/* Internal state                                                       */
/* ------------------------------------------------------------------ */

/* Runtime font buffer: 256 glyphs, each stored as cell_h rows × 2 bytes/row.
 * Three variants: normal, top-half of double-height, bottom-half. */
static uint8_t *s_font_norm   = NULL;  /* 256 × cell_h × 2 bytes */
static uint8_t *s_font_top    = NULL;
static uint8_t *s_font_bottom = NULL;

/* Screen character buffer: 40 × 25 raw bytes */
static uint8_t  s_scrbuf[TTXT_COLS * TTXT_ROWS];

/* Double-height status per row: 0=normal, 1=top, 2=bottom */
static int8_t   s_dh_status[TTXT_ROWS];

/* Framebuffer pointer and geometry */
static uint8_t *s_fb      = NULL;
static int      s_fb_w    = 0;
static int      s_fb_h    = 0;
static int      s_cell_h  = 20;    /* 19 or 20 pixels high */

/* Flash state */
static bool     s_flash_phase = false;

/* Cursor tracking for the optimised single-char path */
static int      s_last_row = -1;
static int      s_last_col = -1;

/* ------------------------------------------------------------------ */
/* Colour helpers                                                       */
/* ------------------------------------------------------------------ */

/* Convert a 6-bit teletext colour index to RGB888 (3 bytes).
 * Bit layout: RRGGBB (2 bits each, MSB = high intensity).
 * Full intensity = 0xFF, dim = 0x00. */
static inline void ttxt_colour_rgb(uint8_t col6, uint8_t *r, uint8_t *g, uint8_t *b)
{
    /* R: bits 5-4 → 0x00 or 0xFF (no half-brightness in classic teletext) */
    *r = (col6 & 0x30) ? 0xFF : 0x00;
    *g = (col6 & 0x0C) ? 0xFF : 0x00;
    *b = (col6 & 0x03) ? 0xFF : 0x00;
}

/* ------------------------------------------------------------------ */
/* Graphics semigraphics character generation                          */
/* ------------------------------------------------------------------ */

/*
 * One byte of a graphics block character row.
 * dot:   1 = the corresponding half-column pixel block is active
 * contig: true = fill full byte (0xFF), false = interior 0x7E / border 0x00
 * inner:  true = we are in the inner rows of a block (not outer edge row)
 */
static inline uint8_t grp_byte(int dot, bool contig, bool inner)
{
    if (!dot) return 0x00;
    if (contig) return 0xFF;
    return inner ? 0x7E : 0x00;
}

/*
 * Fill one graphics semigraphics character into s_font_norm at glyph slot dst.
 * pat:    6-bit block pattern (bits 0-5 = 6 half-cells: top-L, top-R,
 *                                mid-L, mid-R, bot-L, bot-R)
 * contig: contiguous (solid) or separated (gapped)
 */
static void ttxt_set_graph_char(uint8_t dst, uint8_t pat, bool contig)
{
    int b1, b2, b3;
    if (s_cell_h == 19) {
        b1 = 6; b2 = 7; b3 = 6;
    } else {
        b1 = 7; b2 = 6; b3 = 7;
    }

    uint8_t *p = s_font_norm + (int)dst * s_cell_h * 2;
    int row = 0;

#define WRITE_ROW(block_bit, _inner) do { \
    p[row*2+0] = grp_byte((pat >> ((block_bit)  )) & 1, contig, (_inner)); \
    p[row*2+1] = grp_byte((pat >> ((block_bit)+1)) & 1, contig, (_inner)); \
    row++;                                                                   \
} while(0)

    /* Top block */
    WRITE_ROW(0, false);
    for (int i = 1; i < b1 - 1; i++) WRITE_ROW(0, true);
    WRITE_ROW(0, false);

    /* Middle block */
    WRITE_ROW(2, false);
    for (int i = 1; i < b2 - 1; i++) WRITE_ROW(2, true);
    WRITE_ROW(2, false);

    /* Bottom block */
    WRITE_ROW(4, false);
    for (int i = 1; i < b3 - 1; i++) WRITE_ROW(4, true);
    WRITE_ROW(4, false);

#undef WRITE_ROW
}

/*
 * Copy glyph src from vdp_ttxtfont[] into s_font_norm slot dst.
 * vdp_ttxtfont has 20 rows per glyph; we copy min(cell_h, 20) rows.
 */
static void ttxt_set_font_char(uint8_t dst, uint8_t src)
{
    const uint16_t *srcrow = &vdp_ttxtfont[(int)src * 20];
    uint8_t *dstrow = s_font_norm + (int)dst * s_cell_h * 2;
    for (int i = 0; i < s_cell_h; i++) {
        uint16_t w = srcrow[i];
        dstrow[i * 2 + 0] = (uint8_t)(w & 0xFF);
        dstrow[i * 2 + 1] = (uint8_t)(w >> 8);
    }
}

/* ------------------------------------------------------------------ */
/* Double-height font generation                                        */
/* ------------------------------------------------------------------ */

/*
 * Build s_font_top from s_font_norm:
 *   Each row of the output selects every other row from the top half of norm,
 *   stretching the top 10 source rows across all cell_h output rows.
 */
static void ttxt_build_top(void)
{
    for (int i = 32; i < 256; i++) {
        uint8_t *src = s_font_norm   + i * s_cell_h * 2;
        uint8_t *dst = s_font_top    + i * s_cell_h * 2;
        for (int j = 0; j < s_cell_h; j++) {
            /* pick every 2nd row from src, but stay in top half */
            int srow = (j / 2 < s_cell_h / 2) ? (j / 2) * 2 : (s_cell_h / 2 - 1) * 2;
            if (j % 2 == 1) srow += 2;  /* odd rows advance by one */
            if (srow >= s_cell_h * 2) srow = (s_cell_h - 2) * 2;
            dst[j * 2 + 0] = src[srow + 0];
            dst[j * 2 + 1] = src[srow + 1];
        }
    }
}

/*
 * Build s_font_bottom from s_font_norm:
 *   Mirror of top but starting from the middle of the source glyph.
 */
static void ttxt_build_bottom(void)
{
    int start = (s_cell_h == 20) ? 20 : 18;  /* byte offset into source glyph */
    for (int i = 32; i < 256; i++) {
        uint8_t *src = s_font_norm   + i * s_cell_h * 2;
        uint8_t *dst = s_font_bottom + i * s_cell_h * 2;
        uint8_t *p   = src + start;
        for (int j = 0; j < s_cell_h; j++) {
            dst[j * 2 + 0] = p[0];
            dst[j * 2 + 1] = p[1];
            if (j % 2 == (s_cell_h == 20 ? 1 : 0)) p += 2;
            /* clamp to end of source */
            if (p >= src + s_cell_h * 2) p = src + s_cell_h * 2 - 2;
        }
    }
}

/* ------------------------------------------------------------------ */
/* Low-level cell rendering                                             */
/* ------------------------------------------------------------------ */

/*
 * Draw glyph glyph_idx (index into the runtime font buffer) at cell (col, row)
 * with foreground colour fg6 and background colour bg6 (both 6-bit).
 * font_data selects which variant to use (norm/top/bottom).
 */
static void ttxt_render_cell(int col, int row,
                              uint8_t glyph_idx,
                              uint8_t fg6, uint8_t bg6,
                              const uint8_t *font_data)
{
    if (!s_fb || !font_data) return;
    if (col < 0 || col >= TTXT_COLS || row < 0 || row >= TTXT_ROWS) return;

    uint8_t fg_r, fg_g, fg_b;
    uint8_t bg_r, bg_g, bg_b;
    ttxt_colour_rgb(fg6, &fg_r, &fg_g, &fg_b);
    ttxt_colour_rgb(bg6, &bg_r, &bg_g, &bg_b);

    const uint8_t *glyph = font_data + (int)glyph_idx * s_cell_h * 2;
    int px0 = col * TTXT_CELL_W;
    int py0 = row * s_cell_h;

    for (int y = 0; y < s_cell_h; y++) {
        uint8_t lo = glyph[y * 2 + 0];
        uint8_t hi = glyph[y * 2 + 1];
        uint16_t bits = (uint16_t)lo | ((uint16_t)hi << 8);
        int py = py0 + y;
        if (py >= s_fb_h) break;

        uint8_t *line = s_fb + (py * s_fb_w + px0) * 3;
        for (int x = 0; x < TTXT_CELL_W; x++) {
            if (bits & (1u << x)) {
                line[x * 3 + 0] = fg_r;
                line[x * 3 + 1] = fg_g;
                line[x * 3 + 2] = fg_b;
            } else {
                line[x * 3 + 0] = bg_r;
                line[x * 3 + 1] = bg_g;
                line[x * 3 + 2] = bg_b;
            }
        }
    }
}

/* ------------------------------------------------------------------ */
/* Character translation                                                */
/* ------------------------------------------------------------------ */

/*
 * Translate a printable character to its glyph index in the runtime font.
 * UK teletext substitutions:
 *   '#' → pound sign (0xA3 glyph), '_' → hash glyph, '`' → em-dash glyph
 * In graphics mode (flag set), chars 0x20-0x3F and 0x60-0x7F map to the
 * semigraphics block area (128+ in the runtime buffer).
 */
static uint8_t ttxt_translate(uint8_t c, uint8_t state_flags)
{
    /* UK teletext character substitutions */
    switch (c) {
        case '#': c = (uint8_t)'_';  break;
        case '_': c = (uint8_t)'`';  break;
        case '`': c = (uint8_t)'#';  break;
        default: break;
    }
    c &= 0x7F;
    /* In graphics mode, alphanumeric range maps to block graphics */
    if ((state_flags & TTXT_FLAG_GRAPH) && (c & 0x20)) {
        if (state_flags & TTXT_FLAG_SEPARATE)
            c = (uint8_t)(c + 128);
        else
            c = (uint8_t)(c + 96);
    }
    return c;
}

/* ------------------------------------------------------------------ */
/* Line processor                                                       */
/* ------------------------------------------------------------------ */

typedef enum {
    TTXT_OP_SCAN,     /* parse control codes up to col, set state */
    TTXT_OP_UPDATE,   /* redraw from col onward (control code changed) */
    TTXT_OP_FLASH,    /* redraw only flashing characters */
    TTXT_OP_REPAINT,  /* redraw all 40 columns unconditionally */
} ttxt_op_t;

/*
 * Process (and optionally redraw) one row of the teletext screen.
 * Implements the left-to-right teletext attribute model: most control codes
 * take effect in the cell AFTER they appear.
 */
static void ttxt_process_line(int row, int col, ttxt_op_t op)
{
    int maxcol;
    int8_t old_dh;
    bool redraw = (op == TTXT_OP_REPAINT);
    uint8_t held_graph = 0;
    const uint8_t *font_data;

    do {
        maxcol = (op == TTXT_OP_SCAN) ? col : TTXT_COLS;

        /* Select font variant based on double-height status */
        if (s_dh_status[row] == 2)
            font_data = s_font_bottom;
        else
            font_data = s_font_norm;

        uint8_t state = 0;
        if (s_dh_status[row] == 2)
            state = TTXT_FLAG_DHLOW;

        old_dh = s_dh_status[row];

        if (op == TTXT_OP_UPDATE && s_dh_status[row] == 1) {
            s_dh_status[row] = 0;
            if (row < TTXT_ROWS - 1) s_dh_status[row + 1] = 0;
        }

        uint8_t fg = TTXT_WHITE;
        uint8_t bg = TTXT_BLACK;
        held_graph = 0;

        for (int i = 0; i < maxcol; i++) {
            uint8_t c = s_scrbuf[row * TTXT_COLS + i];

            if (op == TTXT_OP_UPDATE && i == col)
                redraw = true;

            if (TTXT_IS_CONTROL(c)) {
                /* Immediate-effect codes (same cell) */
                switch (c & 0x7F) {
                case 0x09: /* Flash off */
                    state &= ~TTXT_FLAG_FLASH;
                    if (op == TTXT_OP_FLASH) redraw = false;
                    break;
                case 0x0C: /* Double height off */
                    state &= ~TTXT_FLAG_HEIGHT;
                    held_graph = 0;
                    font_data = s_font_norm;
                    break;
                case 0x0D: /* Double height on */
                    state |= TTXT_FLAG_HEIGHT;
                    held_graph = 0;
                    if (s_dh_status[row] == 0) {
                        s_dh_status[row] = 1;
                        if (row < TTXT_ROWS - 1)
                            s_dh_status[row + 1] = 2;
                        font_data = s_font_top;
                    } else if (s_dh_status[row] == 1) {
                        font_data = s_font_top;
                    } else {
                        font_data = s_font_bottom;
                    }
                    break;
                case 0x18: /* Conceal */
                    state |= TTXT_FLAG_CONCEAL;
                    break;
                case 0x1C: /* Black background */
                    bg = TTXT_BLACK;
                    break;
                case 0x1D: /* New background = current fg */
                    bg = fg;
                    break;
                case 0x1E: /* Hold graphics */
                    state |= TTXT_FLAG_HOLD;
                    break;
                default:
                    break;
                }

                /* Draw the control cell (space or held graphic) */
                if (redraw) {
                    uint8_t draw_c = ' ';
                    if (held_graph && (state & TTXT_FLAG_HOLD))
                        draw_c = held_graph;
                    /* Conceal / flash-hide */
                    if ((state & TTXT_FLAG_CONCEAL) ||
                        ((state & TTXT_FLAG_DHLOW) && !(state & TTXT_FLAG_HEIGHT)) ||
                        ((state & TTXT_FLAG_FLASH) && !s_flash_phase))
                        draw_c = ' ';
                    ttxt_render_cell(i, row, draw_c, fg, bg, font_data);
                }

                /* Next-cell effect codes */
                switch (c & 0x7F) {
                case 0x01: fg = TTXT_RED;     held_graph = 0; state &= ~(TTXT_FLAG_GRAPH|TTXT_FLAG_CONCEAL); break;
                case 0x02: fg = TTXT_GREEN;   held_graph = 0; state &= ~(TTXT_FLAG_GRAPH|TTXT_FLAG_CONCEAL); break;
                case 0x03: fg = TTXT_YELLOW;  held_graph = 0; state &= ~(TTXT_FLAG_GRAPH|TTXT_FLAG_CONCEAL); break;
                case 0x04: fg = TTXT_BLUE;    held_graph = 0; state &= ~(TTXT_FLAG_GRAPH|TTXT_FLAG_CONCEAL); break;
                case 0x05: fg = TTXT_MAGENTA; held_graph = 0; state &= ~(TTXT_FLAG_GRAPH|TTXT_FLAG_CONCEAL); break;
                case 0x06: fg = TTXT_CYAN;    held_graph = 0; state &= ~(TTXT_FLAG_GRAPH|TTXT_FLAG_CONCEAL); break;
                case 0x07: fg = TTXT_WHITE;   held_graph = 0; state &= ~(TTXT_FLAG_GRAPH|TTXT_FLAG_CONCEAL); break;
                case 0x08: /* Flash on */
                    state |= TTXT_FLAG_FLASH;
                    if (op == TTXT_OP_FLASH) redraw = true;
                    break;
                case 0x11: fg = TTXT_RED;     state |= TTXT_FLAG_GRAPH; state &= ~TTXT_FLAG_CONCEAL; break;
                case 0x12: fg = TTXT_GREEN;   state |= TTXT_FLAG_GRAPH; state &= ~TTXT_FLAG_CONCEAL; break;
                case 0x13: fg = TTXT_YELLOW;  state |= TTXT_FLAG_GRAPH; state &= ~TTXT_FLAG_CONCEAL; break;
                case 0x14: fg = TTXT_BLUE;    state |= TTXT_FLAG_GRAPH; state &= ~TTXT_FLAG_CONCEAL; break;
                case 0x15: fg = TTXT_MAGENTA; state |= TTXT_FLAG_GRAPH; state &= ~TTXT_FLAG_CONCEAL; break;
                case 0x16: fg = TTXT_CYAN;    state |= TTXT_FLAG_GRAPH; state &= ~TTXT_FLAG_CONCEAL; break;
                case 0x17: fg = TTXT_WHITE;   state |= TTXT_FLAG_GRAPH; state &= ~TTXT_FLAG_CONCEAL; break;
                case 0x19: state &= ~TTXT_FLAG_SEPARATE; break;
                case 0x1A: state |= TTXT_FLAG_SEPARATE;  break;
                case 0x1F: state &= ~TTXT_FLAG_HOLD;     break;
                default: break;
                }

            } else {
                /* Printable character */
                uint8_t glyph = ttxt_translate(c, state);
                if (glyph >= 128) held_graph = glyph; else held_graph = 0;

                if (redraw) {
                    uint8_t draw_c = glyph;
                    if ((state & TTXT_FLAG_CONCEAL) ||
                        ((state & TTXT_FLAG_DHLOW) && !(state & TTXT_FLAG_HEIGHT)) ||
                        ((state & TTXT_FLAG_FLASH) && !s_flash_phase))
                        draw_c = ' ';
                    ttxt_render_cell(i, row, draw_c, fg, bg, font_data);
                }
            }
        }

        row++;
        col = 0;
        /* If double-height status changed, also process the next row */
    } while (row < TTXT_ROWS && old_dh != s_dh_status[row - 1]);
}

/* ------------------------------------------------------------------ */
/* Public API                                                           */
/* ------------------------------------------------------------------ */

int ttxt_init(uint8_t *fb, int fb_w, int fb_h)
{
    s_fb   = fb;
    s_fb_w = fb_w;
    s_fb_h = fb_h;
    s_cell_h = (fb_h >= 500) ? 20 : 19;

    /* Allocate or reallocate font buffers in PSRAM */
    size_t font_sz = (size_t)256 * s_cell_h * 2;  /* bytes */

    if (!s_font_norm) {
        s_font_norm = (uint8_t *)heap_caps_malloc(font_sz, MALLOC_CAP_SPIRAM);
        if (!s_font_norm) { ESP_LOGE(TAG, "font_norm alloc fail"); return -1; }
    }
    if (!s_font_top) {
        s_font_top = (uint8_t *)heap_caps_malloc(font_sz, MALLOC_CAP_SPIRAM);
        if (!s_font_top) { ESP_LOGE(TAG, "font_top alloc fail"); return -1; }
    }
    if (!s_font_bottom) {
        s_font_bottom = (uint8_t *)heap_caps_malloc(font_sz, MALLOC_CAP_SPIRAM);
        if (!s_font_bottom) { ESP_LOGE(TAG, "font_bottom alloc fail"); return -1; }
    }

    /* Build normal font from ROM data */
    for (int i = 32; i < 127; i++)
        ttxt_set_font_char((uint8_t)i, (uint8_t)i);

    /* UK teletext character substitutions */
    ttxt_set_font_char('#',  0xA3); /* £ pound */
    ttxt_set_font_char('[',  0x8F); /* left arrow */
    ttxt_set_font_char('\\', 0xBD); /* ½ */
    ttxt_set_font_char(']',  0x90); /* right arrow */
    ttxt_set_font_char('^',  0x8D); /* up arrow */
    ttxt_set_font_char('_',  '#');  /* hash */
    ttxt_set_font_char('`',  0x97); /* em-dash */
    ttxt_set_font_char('{',  0xBC); /* ¼ */
    ttxt_set_font_char('|',  0x9D); /* double vertical bar */
    ttxt_set_font_char('}',  0xBE); /* ¾ */
    ttxt_set_font_char('~',  0xF7); /* ÷ */
    ttxt_set_font_char(127,  0x81); /* solid block */

    /* Generate semigraphics characters (slots 128-255) */
    for (int i = 0; i < 32; i++) {
        ttxt_set_graph_char((uint8_t)(128 + i), (uint8_t)i,        true);  /* contig 0x00-0x1F */
        ttxt_set_graph_char((uint8_t)(160 + i), (uint8_t)i,        false); /* sep   0x00-0x1F */
        ttxt_set_graph_char((uint8_t)(192 + i), (uint8_t)(32 + i), true);  /* contig 0x20-0x3F */
        ttxt_set_graph_char((uint8_t)(224 + i), (uint8_t)(32 + i), false); /* sep   0x20-0x3F */
    }

    /* Build double-height variants */
    ttxt_build_top();
    ttxt_build_bottom();

    /* Clear screen */
    memset(s_scrbuf,   ' ', sizeof(s_scrbuf));
    memset(s_dh_status, 0,  sizeof(s_dh_status));
    s_last_row = -1;
    s_last_col = -1;
    s_flash_phase = false;

    ESP_LOGI(TAG, "teletext init OK, cell %dx%d, font=%zu bytes×3", TTXT_CELL_W, s_cell_h, font_sz);
    return 0;
}

void ttxt_deinit(void)
{
    if (s_font_norm)   { heap_caps_free(s_font_norm);   s_font_norm   = NULL; }
    if (s_font_top)    { heap_caps_free(s_font_top);    s_font_top    = NULL; }
    if (s_font_bottom) { heap_caps_free(s_font_bottom); s_font_bottom = NULL; }
    s_fb = NULL;
}

void ttxt_draw_char(int px, int py, uint8_t c)
{
    int cx = px / TTXT_CELL_W;
    int cy = py / s_cell_h;
    if (cx < 0 || cx >= TTXT_COLS || cy < 0 || cy >= TTXT_ROWS) return;

    uint8_t old = s_scrbuf[cy * TTXT_COLS + cx];
    s_scrbuf[cy * TTXT_COLS + cx] = c;

    bool c_ctrl   = TTXT_IS_CONTROL(c);
    bool old_ctrl = TTXT_IS_CONTROL(old);
    bool next_ctrl = (cx < TTXT_COLS - 1) &&
                      TTXT_IS_CONTROL(s_scrbuf[cy * TTXT_COLS + cx + 1]);

    if (c_ctrl || old_ctrl || next_ctrl) {
        /* Changing a control code — must re-render from this column */
        ttxt_process_line(cy, cx, TTXT_OP_UPDATE);
        s_last_row = -1;
        return;
    }

    if (cx != s_last_col + 1 || cy != s_last_row) {
        /* Non-sequential write: re-scan control codes first */
        ttxt_process_line(cy, cx, TTXT_OP_SCAN);
    }
    s_last_col = cx;
    s_last_row = cy;

    /* Render just this one glyph using the state from the scan */
    /* (SCAN leaves state vars set; re-issue a minimal render via REPAINT
     *  for just this cell by calling process_line with UPDATE from cx) */
    ttxt_process_line(cy, cx, TTXT_OP_UPDATE);
    s_last_row = cy;
    s_last_col = cx;
}

uint8_t ttxt_get_char(int px, int py)
{
    int cx = px / TTXT_CELL_W;
    int cy = py / s_cell_h;
    if (cx < 0 || cx >= TTXT_COLS || cy < 0 || cy >= TTXT_ROWS)
        return 0;
    return s_scrbuf[cy * TTXT_COLS + cx];
}

void ttxt_scroll(void)
{
    /* Shift screen buffer up one row */
    memmove(s_scrbuf, s_scrbuf + TTXT_COLS,
            (TTXT_ROWS - 1) * TTXT_COLS);
    memset(s_scrbuf + (TTXT_ROWS - 1) * TTXT_COLS, ' ', TTXT_COLS);

    /* Shift double-height status up */
    memmove(s_dh_status, s_dh_status + 1, TTXT_ROWS - 1);
    s_dh_status[TTXT_ROWS - 1] = 0;

    s_last_row--;

    /* Scroll the framebuffer up by one cell height */
    if (s_fb) {
        int stride = s_fb_w * 3;
        int rows_to_copy = (s_fb_h - s_cell_h);
        memmove(s_fb, s_fb + s_cell_h * stride,
                (size_t)rows_to_copy * (size_t)stride);
        /* Clear the bottom cell row in the framebuffer */
        memset(s_fb + rows_to_copy * stride, 0,
               (size_t)s_cell_h * (size_t)stride);
    }

    /* Fix up double-height status at top if needed */
    if (s_dh_status[0] == 2) {
        s_dh_status[0] = 1;
        ttxt_process_line(0, 0, TTXT_OP_UPDATE);
        s_last_row = -1;
    }
}

void ttxt_cls(void)
{
    memset(s_scrbuf,   ' ', sizeof(s_scrbuf));
    memset(s_dh_status, 0,  sizeof(s_dh_status));
    s_last_row = -1;
    s_last_col = -1;

    /* Clear framebuffer (black) */
    if (s_fb) {
        int total_rows = TTXT_ROWS * s_cell_h;
        if (total_rows > s_fb_h) total_rows = s_fb_h;
        memset(s_fb, 0, (size_t)total_rows * (size_t)s_fb_w * 3);
    }
}

void ttxt_flash(bool phase)
{
    s_flash_phase = phase;

    /* Check if any row has a flash control code */
    for (int row = 0; row < TTXT_ROWS; row++) {
        for (int col = 0; col < TTXT_COLS; col++) {
            if ((s_scrbuf[row * TTXT_COLS + col] & 0x7F) == 0x08) {
                /* This row has a flash code — repaint it */
                ttxt_process_line(row, TTXT_COLS, TTXT_OP_FLASH);
                break;
            }
        }
    }
    /* Restore cursor scan state */
    if (s_last_row >= 0)
        ttxt_process_line(s_last_row, s_last_col, TTXT_OP_SCAN);
}

void ttxt_repaint(void)
{
    for (int row = 0; row < TTXT_ROWS; row++)
        ttxt_process_line(row, TTXT_COLS, TTXT_OP_REPAINT);
}

int ttxt_cell_w(void) { return TTXT_CELL_W; }
int ttxt_cell_h(void) { return s_cell_h; }
