/*
 * cube3d.c - 3D rotating cube demo for ESP32-MOS
 *
 * Direct translation of BBC BASIC "3D Cube Example" by Dean Belfield.
 *
 * Features:
 *   - Wireframe or solid (back-face culled, filled triangles)
 *   - Double-buffered (MODE 136 = MODE 1 + double-buffer flag)
 *   - Perspective projection
 *   - Euler angle rotation (PHI, THETA, PSI)
 *
 * VDP commands used:
 *   VDU 22,136        MODE 136 = MODE 1 (512x384) with double buffering
 *   VDU 29, x;y;      Set graphics origin to screen centre (640,512)
 *   VDU 18,mode,col   GCOL mode,colour
 *   VDU 16            CLG (clear graphics area)
 *   VDU 25,4, x;y;    MOVE x,y
 *   VDU 25,5, x;y;    PLOT 5 = draw line (from last MOVE)
 *   VDU 25,85,x;y;    PLOT 85 = fill triangle to (x,y)
 *   VDU 23,0,195      Flip double buffer (VDP system command 195)
 *
 * Build: make -C sdk
 * Run:   cube3d        (wireframe)
 *        cube3d s      (solid filled)
 */

#include <stdint.h>
#include "mos_api_table.h"

/* ── float sin/cos via Taylor series ────────────────────────────────────── */
/* We cannot use newlib's sinf because we are freestanding.
 * libgcc provides __floatsisf etc but NOT sin/cos.
 * Simple Taylor series is accurate enough for rotation. */

/* Reduce angle to [-PI, PI] */
static float my_fmod_pi(float x) {
    /* bring into [0, 2PI) then shift */
    const float twopi = 6.28318530718f;
    const float pi    = 3.14159265359f;
    while (x >  pi) x -= twopi;
    while (x < -pi) x += twopi;
    return x;
}

static float my_sinf(float x) {
    x = my_fmod_pi(x);
    /* Taylor: x - x^3/6 + x^5/120 - x^7/5040 + x^9/362880 */
    float x2 = x * x;
    return x * (1.0f + x2 * (-1.0f/6.0f + x2 * (1.0f/120.0f + x2 * (-1.0f/5040.0f + x2 * (1.0f/362880.0f)))));
}

static float my_cosf(float x) {
    /* cos(x) = sin(x + PI/2) */
    return my_sinf(x + 1.57079632679f);
}

/* ── API shortcuts ───────────────────────────────────────────────────────── */
static t_mos_api *g_mos;
static inline void vdu(uint8_t b)    { g_mos->putch(b); }
static inline void vdu16(int16_t v)  { vdu((uint8_t)(v & 0xFF)); vdu((uint8_t)((v >> 8) & 0xFF)); }

static void gcol(uint8_t mode, uint8_t col) { vdu(18); vdu(mode); vdu(col); }
static void clg(void)                       { vdu(16); }
static void move(int16_t x, int16_t y)     { vdu(25); vdu(4);  vdu16(x); vdu16(y); }
static void plot_line(int16_t x, int16_t y){ vdu(25); vdu(5);  vdu16(x); vdu16(y); }
static void plot_fill(int16_t x, int16_t y){ vdu(25); vdu(85); vdu16(x); vdu16(y); }
static void flip_buffer(void)               { vdu(23); vdu(0);  vdu(195); }
static void set_origin(int16_t x, int16_t y){ vdu(29); vdu16(x); vdu16(y); }

/* ── Cube data ───────────────────────────────────────────────────────────── */
#define CN  8   /* number of vertices */
#define TS  6   /* number of faces */

/* Vertex coords (integer, ±20) */
static const int8_t s_vx[CN+1] = { 0, -20, 20,-20, 20,-20, 20,-20, 20 };
static const int8_t s_vy[CN+1] = { 0,  20, 20,-20,-20, 20, 20,-20,-20 };
static const int8_t s_vz[CN+1] = { 0,  20, 20, 20, 20,-20,-20,-20,-20 };

/* Face definitions: 4 vertex indices (1-based) + color */
static const uint8_t s_face[TS][5] = {
    { 1,2,4,3, 9 },
    { 7,8,6,5,10 },
    { 2,6,8,4,11 },
    { 3,7,5,1,12 },
    { 3,4,8,7,13 },
    { 1,5,6,2,14 },
};

/* Projected screen coords (after perspective) */
static int16_t s_ax[CN+1], s_ay[CN+1];

/* ── Projection ──────────────────────────────────────────────────────────── */
static void project(float phi, float theta, float psi, int xd, int yd)
{
    const float DEG = 3.14159265359f / 180.0f;
    const int SD = 1024;
    const int OD = 128;

    for (int i = 1; i <= CN; i++) {
        float xx = (float)s_vx[i];
        float yy = (float)s_vy[i];
        float zz = (float)s_vz[i];

        /* Rotate X around X axis (phi) */
        float y2 = yy * my_cosf(phi*DEG) - zz * my_sinf(phi*DEG);
        zz       = yy * my_sinf(phi*DEG) + zz * my_cosf(phi*DEG);
        yy = y2;

        /* Rotate around Y axis (theta) */
        float x2 = xx * my_cosf(theta*DEG) - zz * my_sinf(theta*DEG);
        zz        = xx * my_sinf(theta*DEG) + zz * my_cosf(theta*DEG);
        xx = x2;

        /* Rotate around Z axis (psi) */
        x2  = xx * my_cosf(psi*DEG) - yy * my_sinf(psi*DEG);
        yy  = xx * my_sinf(psi*DEG) + yy * my_cosf(psi*DEG);
        xx = x2;

        /* Translate */
        xx += (float)xd;
        yy += (float)yd;

        /* Perspective projection */
        float denom = (float)(OD) - zz;
        if (denom < 1.0f) denom = 1.0f;
        s_ax[i] = (int16_t)(xx * (float)SD / denom);
        s_ay[i] = (int16_t)(yy * (float)SD / denom);
    }
}

/* ── Draw frame ──────────────────────────────────────────────────────────── */
static void draw_frame(int filled)
{
    clg();

    for (int i = 0; i < TS; i++) {
        int c1 = s_face[i][0], c2 = s_face[i][1];
        int c3 = s_face[i][2], c4 = s_face[i][3];
        uint8_t col = s_face[i][4];

        int16_t x1 = s_ax[c1], x2 = s_ax[c2], x3 = s_ax[c3], x4 = s_ax[c4];
        int16_t y1 = s_ay[c1], y2 = s_ay[c2], y3 = s_ay[c3], y4 = s_ay[c4];

        /* Back-face cull: cross product Z > 0 → facing away */
        int32_t cross_z = (int32_t)x1 * (y2 - y3)
                        + (int32_t)x2 * (y3 - y1)
                        + (int32_t)x3 * (y1 - y2);
        if (cross_z > 0) continue;

        gcol(0, col);

        if (filled) {
            /* Two filled triangles per quad face */
            move(x1, y1); move(x2, y2); plot_fill(x3, y3);
            move(x3, y3); move(x4, y4); plot_fill(x1, y1);
        } else {
            /* Wireframe */
            move(x1, y1);
            plot_line(x2, y2);
            plot_line(x3, y3);
            plot_line(x4, y4);
            plot_line(x1, y1);
        }
    }

    flip_buffer();
}

/* ── Entry point ─────────────────────────────────────────────────────────── */
__attribute__((section(".text.entry")))
int _start(int argc, char **argv, t_mos_api *mos)
{
    (void)argc;
    g_mos = mos;

    /* Default: wireframe.  Pass "s" argument for solid. */
    int filled = 0;
    if (argc > 1 && (argv[1][0] == 's' || argv[1][0] == 'S')) filled = 1;

    /* MODE 136 = MODE 1 (512×384, 64 colours) with double buffering */
    vdu(22); vdu(136);
    mos->delay_ms(100);

    /* Set graphics origin to screen centre: VDU 29,640;512; */
    set_origin(640, 512);

    mos->puts(filled ? "3D Cube - SOLID\r\n" : "3D Cube - WIREFRAME\r\n");
    mos->puts("Press any key to exit\r\n");

    float phi = 0.0f, theta = 0.0f, psi = 0.0f;

    while (1) {
        if (mos->kbhit()) { mos->getkey(); break; }

        project(phi, theta, psi, 0, 0);
        draw_frame(filled);

        phi   += 4.0f;
        theta -= 1.0f;
        /* psi stays 0 as in BASIC original */
    }

    /* Restore normal mode */
    vdu(22); vdu(0);
    mos->puts("Done.\r\n");
    return 0;
}
