/*
 * sprites.c - Pacman sprite bouncing demo for ESP32-MOS
 *
 * VDP Bitmap API (VDU 23,27,...):
 *   0, n           - select bitmap n (8-bit; internally buffer 64000+n)
 *   1, w; h; bytes - load RGBA8888 bitmap (4 bytes/pixel, R G B A order)
 *                    NOTE: any non-zero alpha = fully visible
 *   2, w; h; c1; c2; - create solid-colour bitmap (two colours, top/bottom)
 *   3, x; y;       - draw current bitmap at pixel coords
 *
 * VDP Sprite API:
 *   4, n           - select sprite n
 *   5              - clear frames of current sprite
 *   6, n           - add bitmap n (8-bit) as next frame
 *   7, n           - activate n sprites
 *   8              - next frame
 *   11             - show current sprite
 *   12             - hide current sprite
 *   13, x; y;      - move to pixel coords
 *   15             - update/refresh GPU sprites
 *   16             - reset all bitmaps+sprites
 *   17             - reset sprites only
 *
 * Build: make -C sdk
 * Run:   sprites
 */

#include <stdint.h>
#include "mos_api_table.h"

#define NUM_SPRITES  32
#define SPRITE_W     16
#define SPRITE_H     16

/* MODE 1: 512x384 pixel screen */
#define SCREEN_W     512
#define SCREEN_H     384

static t_mos_api *g_mos;

static inline void vdu(uint8_t b)  { g_mos->putch(b); }
static inline void vdu16(uint16_t v) {
    vdu((uint8_t)(v & 0xFF));
    vdu((uint8_t)(v >> 8));
}

/* ── Bitmap commands ─────────────────────────────────────────────────────── */

static void bmp_select(uint8_t n)
{
    vdu(23); vdu(27); vdu(0); vdu(n);
}

/* Send header for VDU 23,27,1 — pixel data must follow immediately */
static void bmp_begin_rgba(uint8_t n, uint16_t w, uint16_t h)
{
    bmp_select(n);
    vdu(23); vdu(27); vdu(1);
    vdu16(w); vdu16(h);
}

/* ── Sprite commands ─────────────────────────────────────────────────────── */

static void spr_select(uint8_t n)       { vdu(23); vdu(27); vdu(4); vdu(n); }
static void spr_clear_frames(void)      { vdu(23); vdu(27); vdu(5); }
static void spr_add_frame(uint8_t bmp)  { vdu(23); vdu(27); vdu(6); vdu(bmp); }
static void spr_activate(uint8_t n)     { vdu(23); vdu(27); vdu(7); vdu(n); }
static void spr_next_frame(void)        { vdu(23); vdu(27); vdu(8); }
static void spr_show(void)              { vdu(23); vdu(27); vdu(11); }
static void spr_move(uint16_t x, uint16_t y) {
    vdu(23); vdu(27); vdu(13); vdu16(x); vdu16(y);
}
static void spr_update(void)            { vdu(23); vdu(27); vdu(15); }
static void spr_reset_all(void)         { vdu(23); vdu(27); vdu(16); }

/* ── Load 16x16 RGB file as RGBA bitmap ─────────────────────────────────── */

static int load_bitmap_from_file(const char *path, uint8_t bmp_id)
{
    uint8_t fh = g_mos->fopen(path, "r");
    if (!fh) {
        g_mos->puts("  ERR: cannot open ");
        g_mos->puts(path);
        g_mos->puts("\r\n");
        return -1;
    }

    bmp_begin_rgba(bmp_id, SPRITE_W, SPRITE_H);

    /* Stream SPRITE_W*SPRITE_H pixels: 3 bytes RGB from file → 4 bytes RGBA to VDP */
    uint8_t rgb[3];
    for (int i = 0; i < SPRITE_W * SPRITE_H; i++) {
        size_t got = g_mos->fread(rgb, 1, 3, fh);
        if (got != 3) {
            /* premature EOF — pad with transparent black */
            vdu(0); vdu(0); vdu(0); vdu(0);
        } else {
            uint8_t r = rgb[0], g2 = rgb[1], b = rgb[2];
            /* black background → transparent (alpha=0) */
            uint8_t a = (r == 0 && g2 == 0 && b == 0) ? 0 : 255;
            vdu(r); vdu(g2); vdu(b); vdu(a);
        }
    }

    g_mos->fclose(fh);
    return 0;
}

/* ── Create a simple solid-colour 16x16 bitmap (no file needed) ─────────── */
/* colour: 0xRRGGBBAA */
static void make_solid_bitmap(uint8_t bmp_id, uint8_t r, uint8_t g2, uint8_t b)
{
    bmp_begin_rgba(bmp_id, SPRITE_W, SPRITE_H);
    for (int i = 0; i < SPRITE_W * SPRITE_H; i++) {
        vdu(r); vdu(g2); vdu(b); vdu(255);
    }
}

/* ── LFSR pseudo-random ──────────────────────────────────────────────────── */
static uint32_t s_rand = 0xDEADBEEF;
static uint16_t rand16(void) {
    s_rand ^= s_rand << 13;
    s_rand ^= s_rand >> 17;
    s_rand ^= s_rand << 5;
    return (uint16_t)(s_rand & 0xFFFF);
}

/* ── Sprite state ─────────────────────────────────────────────────────────  */
typedef struct { int16_t x, y; int8_t dx, dy; uint8_t frame; } Spr;
static Spr s_spr[NUM_SPRITES];

/* ── Entry point ─────────────────────────────────────────────────────────── */
__attribute__((section(".text.entry")))
int _start(int argc, char **argv, t_mos_api *mos)
{
    (void)argc; (void)argv;
    g_mos = mos;

    /* MODE 1: 512x384, 256 colours */
    vdu(22); vdu(1);
    mos->delay_ms(100);

    /* Full reset of bitmaps+sprites to clean state */
    spr_reset_all();
    mos->delay_ms(50);

    mos->puts("Loading bitmaps...\r\n");

    /* Try to load from files; fall back to solid colours if missing */
    int ok0 = load_bitmap_from_file("A:/pacman1.rgb", 0);
    int ok1 = load_bitmap_from_file("A:/pacman2.rgb", 1);
    if (ok0 < 0) { make_solid_bitmap(0, 255, 255,   0); } /* yellow */
    if (ok1 < 0) { make_solid_bitmap(1, 255, 165,   0); } /* orange */

    /* Give VDP time to fully process all pixel data before sprite setup.
     * At 115200 baud each byte = ~87µs; 2*16*16*4 = 2048 bytes ≈ 178ms.
     * Add generous margin. */
    mos->delay_ms(500);

    mos->puts("Setting up sprites...\r\n");

    /* Set up NUM_SPRITES sprites, each with 2 frames */
    for (int i = 0; i < NUM_SPRITES; i++) {
        spr_select((uint8_t)i);
        spr_clear_frames();
        spr_add_frame(0);   /* frame 0 = bitmap 0 (open) */
        spr_add_frame(1);   /* frame 1 = bitmap 1 (closed) */
    }

    /* Activate sprites */
    spr_activate(NUM_SPRITES);
    mos->delay_ms(50);

    /* Initialise positions + velocities */
    for (int i = 0; i < NUM_SPRITES; i++) {
        s_spr[i].x  = (int16_t)(rand16() % (SCREEN_W - SPRITE_W));
        s_spr[i].y  = (int16_t)(rand16() % (SCREEN_H - SPRITE_H));
        int8_t vx = (int8_t)((rand16() % 3) + 1);
        int8_t vy = (int8_t)((rand16() % 3) + 1);
        s_spr[i].dx = (rand16() & 1) ? vx : (int8_t)-vx;
        s_spr[i].dy = (rand16() & 1) ? vy : (int8_t)-vy;
        s_spr[i].frame = 0;
    }

    /* Place and show all sprites */
    for (int i = 0; i < NUM_SPRITES; i++) {
        spr_select((uint8_t)i);
        spr_move((uint16_t)s_spr[i].x, (uint16_t)s_spr[i].y);
        spr_show();
    }
    spr_update();

    mos->puts("Running - press a key to exit\r\n");

    uint32_t tick = 0;
    while (1) {
        if (mos->kbhit()) { mos->getkey(); break; }

        /* Move + bounce */
        for (int i = 0; i < NUM_SPRITES; i++) {
            Spr *sp = &s_spr[i];
            sp->x += sp->dx;  sp->y += sp->dy;
            if (sp->x < 0)                  { sp->x = 0;                  sp->dx = (int8_t)-sp->dx; }
            if (sp->x > SCREEN_W - SPRITE_W) { sp->x = SCREEN_W-SPRITE_W; sp->dx = (int8_t)-sp->dx; }
            if (sp->y < 0)                  { sp->y = 0;                  sp->dy = (int8_t)-sp->dy; }
            if (sp->y > SCREEN_H - SPRITE_H) { sp->y = SCREEN_H-SPRITE_H; sp->dy = (int8_t)-sp->dy; }
        }

        /* Alternate frames every 8 ticks (~133ms) */
        uint8_t want_frame = (uint8_t)((tick >> 3) & 1);

        for (int i = 0; i < NUM_SPRITES; i++) {
            Spr *sp = &s_spr[i];
            spr_select((uint8_t)i);
            spr_move((uint16_t)sp->x, (uint16_t)sp->y);
            if (sp->frame != want_frame) {
                spr_next_frame();
                sp->frame = want_frame;
            }
        }
        spr_update();

        mos->delay_ms(16);
        tick++;
    }

    spr_activate(0);
    spr_update();
    vdu(22); vdu(0);
    mos->puts("Done.\r\n");
    return 0;
}
