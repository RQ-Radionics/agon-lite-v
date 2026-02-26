/*
 * sprites.c - Pacman sprite bouncing demo for ESP32-MOS
 *
 * Loads two 16x16 RGB bitmaps from the FAT filesystem (pacman1.rgb,
 * pacman2.rgb), uploads them to the Agon VDP as bitmaps, creates 32
 * sprites that bounce around the screen, and animates them by
 * alternating between the two frames (mouth open/closed).
 *
 * VDP Sprite/Bitmap API used:
 *   VDU 23,27,0,n        - select bitmap n (buffer id = 64000 + n)
 *   VDU 23,27,1,w;h; ... - define bitmap pixels (RGBA order: R,G,B,A)
 *                          NOTE: VDP expects R,G,B,A per pixel (RGBA8888)
 *                          .rgb files are raw R,G,B (3 bytes/pixel)
 *                          so we send: r, g, b, 255 (full alpha) per pixel
 *   VDU 23,27,4,n        - select sprite n
 *   VDU 23,27,5          - clear frames of current sprite
 *   VDU 23,27,6,n        - add bitmap n as frame to current sprite
 *   VDU 23,27,7,n        - activate n sprites
 *   VDU 23,27,11         - show current sprite
 *   VDU 23,27,13,x;y;    - move current sprite to pixel coords (x,y)
 *   VDU 23,27,15         - update/refresh all sprites
 *   VDU 23,27,8          - next frame of current sprite
 *
 * Screen: VDP always has 1280x1024 virtual coords regardless of mode.
 * In MODE 1 (512x384 pixel display): pixel coords map 1:1 in hardware,
 * but the sprite position uses logical pixel coordinates.
 * We use MODE 1 (256 colours, 512x384) for this demo.
 * Sprite pixel coords are 0-based from top-left.
 *
 * Build: make -C sdk
 * Run:   sprites
 */

#include <stdint.h>
#include "mos_api_table.h"

#define NUM_SPRITES  32
#define SPRITE_W     16
#define SPRITE_H     16

/* Screen dimensions in pixels for MODE 1 (512x384) */
#define SCREEN_W     512
#define SCREEN_H     384

/* Bitmap IDs (0 = pacman open, 1 = pacman closed) */
#define BMP_OPEN     0
#define BMP_CLOSED   1

static t_mos_api *g_mos;

static inline void vdu(uint8_t b) { g_mos->putch(b); }

static inline void vdu16(uint16_t v)
{
    vdu((uint8_t)(v & 0xFF));
    vdu((uint8_t)(v >> 8));
}

/* ── VDP bitmap/sprite helpers ──────────────────────────────────────────── */

/* Select bitmap n for subsequent define or use */
static void vdp_select_bitmap(uint8_t n)
{
    vdu(23); vdu(27); vdu(0); vdu(n);
}

/* Begin defining a WxH bitmap (pixels follow immediately after) */
static void vdp_begin_bitmap(uint16_t w, uint16_t h)
{
    vdu(23); vdu(27); vdu(1);
    vdu16(w);
    vdu16(h);
}

/* Select sprite n as the current sprite for subsequent sprite commands */
static void vdp_select_sprite(uint8_t n)
{
    vdu(23); vdu(27); vdu(4); vdu(n);
}

/* Clear all frames from current sprite */
static void vdp_clear_frames(void)
{
    vdu(23); vdu(27); vdu(5);
}

/* Add bitmap n as next frame of current sprite */
static void vdp_add_frame(uint8_t bitmap_id)
{
    vdu(23); vdu(27); vdu(6); vdu(bitmap_id);
}

/* Activate n sprites (must call before show/move) */
static void vdp_activate_sprites(uint8_t n)
{
    vdu(23); vdu(27); vdu(7); vdu(n);
}

/* Show current sprite */
static void vdp_show_sprite(void)
{
    vdu(23); vdu(27); vdu(11);
}

/* Move current sprite to pixel position (x, y) */
static void vdp_move_sprite(uint16_t x, uint16_t y)
{
    vdu(23); vdu(27); vdu(13);
    vdu16(x);
    vdu16(y);
}

/* Advance current sprite to next frame */
static void vdp_next_frame(void)
{
    vdu(23); vdu(27); vdu(8);
}

/* Refresh/display all sprites on screen */
static void vdp_update_sprites(void)
{
    vdu(23); vdu(27); vdu(15);
}

/* ── Load RGB file → upload to VDP as bitmap ────────────────────────────── */

/*
 * Load a 16x16 RGB raw file from the FAT and upload it as VDP bitmap `bmp_id`.
 * .rgb format: 768 bytes, 3 bytes per pixel (R, G, B), top-row first.
 * VDP RGBA format: 4 bytes per pixel (R, G, B, A).
 */
static int load_bitmap(const char *path, uint8_t bmp_id)
{
    uint8_t fh = g_mos->fopen(path, "r");
    if (!fh) {
        g_mos->puts("Cannot open: ");
        g_mos->puts(path);
        g_mos->puts("\r\n");
        return -1;
    }

    vdp_select_bitmap(bmp_id);
    vdp_begin_bitmap(SPRITE_W, SPRITE_H);

    /* Stream pixels: read 3 bytes (RGB), send 4 bytes (RGBA) */
    for (int row = 0; row < SPRITE_H; row++) {
        /* Read one row of RGB pixels */
        uint8_t rgb[SPRITE_W * 3];
        size_t got = g_mos->fread(rgb, 1, SPRITE_W * 3, fh);
        if (got != (size_t)(SPRITE_W * 3)) break;

        /* Convert RGB → RGBA and send to VDP */
        for (int col = 0; col < SPRITE_W; col++) {
            uint8_t r = rgb[col * 3 + 0];
            uint8_t g = rgb[col * 3 + 1];
            uint8_t b = rgb[col * 3 + 2];
            /* Black pixels → transparent */
            uint8_t a = (r == 0 && g == 0 && b == 0) ? 0 : 255;
            vdu(r); vdu(g); vdu(b); vdu(a);
        }
    }

    g_mos->fclose(fh);
    return 0;
}

/* ── Simple pseudo-random LFSR ──────────────────────────────────────────── */
static uint32_t s_rand = 0xDEADBEEF;
static uint16_t rand16(void)
{
    s_rand ^= s_rand << 13;
    s_rand ^= s_rand >> 17;
    s_rand ^= s_rand << 5;
    return (uint16_t)(s_rand & 0xFFFF);
}

/* ── Sprite state ───────────────────────────────────────────────────────── */
typedef struct {
    int16_t x, y;      /* current pixel position (top-left of sprite) */
    int8_t  dx, dy;    /* velocity in pixels per frame */
    uint8_t frame;     /* current frame (0 = open, 1 = closed) */
} Sprite;

static Sprite s_sprites[NUM_SPRITES];

/* ── Busy-wait delay ────────────────────────────────────────────────────── */
static void wait_ms(uint32_t ms)
{
    volatile uint32_t n = ms * 60000u;
    while (n--) { /* spin */ }
}

/* ── Entry point ─────────────────────────────────────────────────────────── */
__attribute__((section(".text.entry")))
int _start(int argc, char **argv, t_mos_api *mos)
{
    (void)argc; (void)argv;
    g_mos = mos;

    /* MODE 1: 512x384, 256 colours */
    vdu(22); vdu(1);

    mos->puts("Loading sprites...\r\n");

    /* Load both bitmaps */
    if (load_bitmap("A:/pacman1.rgb", BMP_OPEN)   < 0) return 1;
    if (load_bitmap("A:/pacman2.rgb", BMP_CLOSED) < 0) return 1;

    /* Set up sprites: each gets both frames */
    for (int i = 0; i < NUM_SPRITES; i++) {
        vdp_select_sprite((uint8_t)i);
        vdp_clear_frames();
        vdp_add_frame(BMP_OPEN);    /* frame 0 */
        vdp_add_frame(BMP_CLOSED);  /* frame 1 */
    }

    /* Activate all sprites */
    vdp_activate_sprites(NUM_SPRITES);

    /* Initialise sprite positions and velocities */
    for (int i = 0; i < NUM_SPRITES; i++) {
        s_sprites[i].x  = (int16_t)(rand16() % (SCREEN_W - SPRITE_W));
        s_sprites[i].y  = (int16_t)(rand16() % (SCREEN_H - SPRITE_H));
        /* velocity: ±1..±3, never zero */
        int8_t vx = (int8_t)((rand16() % 3) + 1);
        int8_t vy = (int8_t)((rand16() % 3) + 1);
        s_sprites[i].dx = (rand16() & 1) ? vx : (int8_t)-vx;
        s_sprites[i].dy = (rand16() & 1) ? vy : (int8_t)-vy;
        s_sprites[i].frame = (uint8_t)(i & 1);
    }

    /* Show all sprites at initial positions */
    for (int i = 0; i < NUM_SPRITES; i++) {
        vdp_select_sprite((uint8_t)i);
        vdp_move_sprite((uint16_t)s_sprites[i].x, (uint16_t)s_sprites[i].y);
        vdp_show_sprite();
    }
    vdp_update_sprites();

    mos->puts("Running - press a key to exit\r\n");

    /* Main animation loop */
    uint32_t tick = 0;
    while (1) {
        /* Check for keypress to exit */
        if (mos->getkey()) break;

        /* Move sprites, bounce off walls */
        for (int i = 0; i < NUM_SPRITES; i++) {
            Sprite *sp = &s_sprites[i];
            sp->x += sp->dx;
            sp->y += sp->dy;

            if (sp->x < 0) {
                sp->x = 0;
                sp->dx = (int8_t)-sp->dx;
            } else if (sp->x > SCREEN_W - SPRITE_W) {
                sp->x = SCREEN_W - SPRITE_W;
                sp->dx = (int8_t)-sp->dx;
            }
            if (sp->y < 0) {
                sp->y = 0;
                sp->dy = (int8_t)-sp->dy;
            } else if (sp->y > SCREEN_H - SPRITE_H) {
                sp->y = SCREEN_H - SPRITE_H;
                sp->dy = (int8_t)-sp->dy;
            }
        }

        /* Alternate frames every 8 ticks (~133ms at 60fps target) */
        uint8_t show_frame = (uint8_t)((tick >> 3) & 1);

        /* Update all sprite positions and frames */
        for (int i = 0; i < NUM_SPRITES; i++) {
            Sprite *sp = &s_sprites[i];
            vdp_select_sprite((uint8_t)i);
            vdp_move_sprite((uint16_t)sp->x, (uint16_t)sp->y);
            /* Set correct frame: advance from current until we reach target */
            if (sp->frame != show_frame) {
                vdp_next_frame();
                sp->frame = show_frame;
            }
        }
        vdp_update_sprites();

        wait_ms(16); /* ~60fps */
        tick++;
    }

    /* Deactivate sprites and restore text mode */
    vdp_activate_sprites(0);
    vdp_update_sprites();
    vdu(22); vdu(0);   /* MODE 0 = default text mode */

    mos->puts("Done.\r\n");
    return 0;
}
