/*
 * mandelbrot.c - Mandelbrot set benchmark for ESP32-MOS
 *
 * Direct translation of BBC BASIC by P.Mainwaring, March 19th 1990.
 * Uses VDU commands sent via mos->putch() to drive the Agon VDP.
 *
 * BBC BASIC original logic:
 *   MODE 12  (320x256, 16 colours on Agon VDP)
 *   ITERS=32, COLS=64, SCALE=1
 *   XRANGE=-2.5, YRANGE=-1
 *   MAXX=1280, MAXY=1024
 *   XSTP=4, YSTP=5  (1280/320, 1024/200 — integer)
 *   4-pass interleave via A%[]={0,1,0,1}, B%[]={0,1,1,0}
 *   CR = XRANGE + K%*4/MAXX        (= -2.5 + K%/320)
 *   CI = YRANGE + J%*3/MAXX        (= -1   + J%*3/1280)
 *   colour C% = COLS-1 - IT*(COLS-1)/ITERS
 *   GCOL 0,C%  →  VDU 18,0,C%
 *   PLOT 69,K%,J%  →  VDU 25,69, K%lo,K%hi, J%lo,J%hi
 *
 * Build:  make -C sdk mandelbrot
 * Run:    mandelbrot   (from MOS shell)
 */

#include <stdint.h>
#include "mos_api_table.h"

/* ── VDU helpers ─────────────────────────────────────────────────────────── */

static t_mos_api *g_mos;

static inline void vdu(uint8_t b)
{
    g_mos->putch(b);
}

static void vdu_mode(uint8_t mode)
{
    vdu(22); vdu(mode);
}

/* GCOL 0, colour */
static void vdu_gcol(uint8_t colour)
{
    vdu(18); vdu(0); vdu(colour);
}

/* PLOT 69 (plot point) at VDU coordinates (x, y) — 16-bit little-endian */
static void vdu_plot_point(int x, int y)
{
    vdu(25); vdu(69);
    vdu((uint8_t)(x & 0xFF)); vdu((uint8_t)((x >> 8) & 0xFF));
    vdu((uint8_t)(y & 0xFF)); vdu((uint8_t)((y >> 8) & 0xFF));
}

/* ── Mandelbrot ──────────────────────────────────────────────────────────── */

__attribute__((section(".text.entry")))
int _start(int argc, char **argv, t_mos_api *mos)
{
    (void)argc; (void)argv;
    g_mos = mos;

    /* Constants matching the BBC BASIC original */
    const int   ITERS  = 32;
    const int   COLS   = 64;
    const int   XSTP   = 4;          /* 1280/320 */
    const int   YSTP   = 5;          /* 1024/200 */
    const int   MAXX   = 1280;
    const int   MAXY   = 1024;
    const float XRANGE = -2.5f;
    const float YRANGE = -1.0f;

    /* 4-pass interleave: A%[]={0,1,0,1}, B%[]={0,1,1,0} */
    const int A[4] = {0, 1, 0, 1};
    const int B[4] = {0, 1, 1, 0};

    vdu_mode(12);   /* 320x256, 16 colours */

    for (int i = 0; i < 4; i++) {
        for (int j = A[i]; j <= MAXY; j += YSTP) {
            for (int k = B[i]; k <= MAXX; k += XSTP) {

                float cr = XRANGE + (float)k * 4.0f / (float)MAXX;
                float ci = YRANGE + (float)j * 3.0f / (float)MAXX;

                float zr = 0.0f, zi = 0.0f;
                float zr2 = 0.0f, zi2 = 0.0f, zm = 0.0f;
                int   it = 0;

                do {
                    float z1 = zr2 - zi2 + cr;
                    float z2 = 2.0f * zr * zi + ci;
                    zr  = z1;
                    zi  = z2;
                    zr2 = zr * zr;
                    zi2 = zi * zi;
                    zm  = zr2 + zi2;
                    it++;
                } while (it < ITERS && zm < 4.0f);

                /* colour: 63 - it*63/32, clamped to 0..63 */
                int c = (COLS - 1) - it * (COLS - 1) / ITERS;
                if (c < 0) c = 0;

                vdu_gcol((uint8_t)c);
                vdu_plot_point(k, j);
            }
        }
    }

    /* Wait for keypress (GET equivalent) */
    mos->getkey();

    /* Restore MODE 1 */
    vdu_mode(1);

    return 0;
}
