/*
 * hatgraph.c - "Hat Graph" (sombrero surface) for ESP32-MOS
 *
 * Direct translation of BBC BASIC by unknown author.
 * Draws a Mexican-hat / sombrero 3D surface using isometric projection
 * and a Y-buffer for hidden-line removal.
 *
 * Algorithm:
 *   Surface: YY = 56*(sin(r*XF) + 0.4*sin(3*r*XF))   where r = sqrt(X²+ZT²)
 *   XF = 3π/2 / 144  maps r ∈ [0,144] → argument ∈ [0, 3π/2]
 *   Isometric: screen_x = XI + ZI + 160
 *              screen_y = 90 - YY + ZI
 *   Y-buffer RR[]: stores minimum screen_y per column (hidden-line removal).
 *   Z loop 64→-64 (back to front) so nearer points overwrite farther ones.
 *
 * VDU coordinate system (Agon VDP):
 *   Origin bottom-left, units = 1/4 pixel.
 *   PLOT 69, x*4, 882-y*4  maps pixel (x,y) to VDU space with Y-flip.
 *
 * Build:  make -C sdk   (auto-copies to data/)
 * Run:    hatgraph       (from MOS shell)
 */

#include <stdint.h>
#include "mos_api_table.h"

/* ── libm stubs ─────────────────────────────────────────────────────────────
 * We are freestanding (-nostdlib). Pull in software float/math via libgcc
 * (linked by Makefile). sin() and sqrt() are provided by the ROM / newlib
 * in the firmware — but we cannot call them directly from user space since
 * we have no shared library mechanism. Implement them here using libgcc's
 * float ops and a miniature cordic/Taylor series, OR just use the firmware's
 * ROM functions via their fixed ROM addresses.
 *
 * Simplest approach: provide our own sin/sqrt via libgcc soft-float.
 * We use double-precision via compiler built-ins — libgcc provides
 * __divsf3, __mulsf3, etc. for float; __divdf3 etc. for double.
 * For sin/sqrt we implement compact versions sufficient for this use case.
 * ────────────────────────────────────────────────────────────────────────── */

/* We need sqrt and sin. Use the ESP32 ROM functions via their known addresses.
 * These are stable across all ESP32-S3 firmware versions (ROM is fixed).
 *
 * Alternatively: implement in C using libgcc soft-double.
 * We go with Taylor/CORDIC approximations to keep the binary self-contained.
 */

/* ── Math helpers (double precision, no libm needed) ────────────────────── */

/* Square root via Newton-Raphson.
 * Initial estimate by bit-manipulation of the IEEE 754 double exponent
 * (halving the exponent gives a good first approximation for any magnitude),
 * then iterate until converged. */
static double d_sqrt(double x)
{
    if (x <= 0.0) return 0.0;
    /* Bit-twiddled initial estimate: reinterpret double bits, halve exponent.
     * This gives ~8 significant bits (enough for fast N-R convergence). */
    union { double d; uint64_t u; } v;
    v.d = x;
    v.u = (v.u >> 1) + (0x3FF0000000000000ULL >> 1);  /* halve exponent */
    double r = v.d;
    /* Newton-Raphson: each step doubles significant bits.
     * Start ~8 bits → 3 steps → 64 bits (full double precision). */
    r = (r + x / r) * 0.5;
    r = (r + x / r) * 0.5;
    r = (r + x / r) * 0.5;
    r = (r + x / r) * 0.5;  /* extra step for safety */
    return r;
}

/* sin(x) via range reduction + Taylor series */
static double d_sin(double x)
{
    /* Reduce to [-π, π] */
    /* π approximation */
    const double PI  = 3.14159265358979323846;
    const double PI2 = 6.28318530717958647692;

    /* Reduce modulo 2π */
    double q = x / PI2;
    /* floor(q) */
    double qf = (double)(int)q - (q < (double)(int)q ? 1.0 : 0.0);
    x = x - qf * PI2;
    /* Now x ∈ [0, 2π) — centre to [-π, π] */
    if (x > PI)  x -= PI2;

    /* Taylor series: sin(x) = x - x³/6 + x⁵/120 - x⁷/5040 + x⁹/362880 - x¹¹/39916800 */
    double x2 = x * x;
    return x * (1.0
        - x2 * (1.0/6.0
        + x2 * (-(1.0/120.0)
        + x2 * (1.0/5040.0
        + x2 * (-(1.0/362880.0)
        + x2 *   (1.0/39916800.0))))));
}

/* ── VDU helpers ─────────────────────────────────────────────────────────── */

static t_mos_api *g_mos;

static inline void vdu(uint8_t b)   { g_mos->putch(b); }

static void vdu_mode(uint8_t m)     { vdu(22); vdu(m); }
static void vdu_cls(void)           { vdu(12); }
static void vdu_gcol(uint8_t colour){ vdu(18); vdu(0); vdu(colour); }

static void vdu_plot_point(int x, int y)
{
    vdu(25); vdu(69);
    vdu((uint8_t)(x & 0xFF)); vdu((uint8_t)((x >> 8) & 0xFF));
    vdu((uint8_t)(y & 0xFF)); vdu((uint8_t)((y >> 8) & 0xFF));
}

/* ── Hat Graph ───────────────────────────────────────────────────────────── */

__attribute__((section(".text.entry")))
int _start(int argc, char **argv, t_mos_api *mos)
{
    (void)argc; (void)argv;
    g_mos = mos;

    vdu_mode(1);    /* MODE 1 — correct aspect ratio for this program */
    vdu_cls();
    vdu_gcol(15);   /* white */

    const double XP = 144.0;
    const double XR = 4.71238905;   /* 3π/2 */
    const double XF = XR / XP;

    /* Y-buffer: stores minimum screen-Y per pixel column (hidden-line removal) */
    double RR[321];
    for (int i = 0; i <= 320; i++) RR[i] = 193.0;

    for (int zi = 64; zi >= -64; zi -= 2) {
        double zt = zi * 2.25;
        double zs = zt * zt;
        double sq = 20736.0 - zs;
        if (sq < 0.0) sq = 0.0;           /* clamp floating-point rounding */
        int xl = (int)(d_sqrt(sq) + 0.5);

        for (double xi = (double)(-xl); xi <= (double)xl; xi += 0.5) {
            double xt = d_sqrt(xi * xi + zs) * XF;
            double yy = (d_sin(xt) + d_sin(xt * 3.0) * 0.4) * 56.0;

            int    x1 = (int)(xi + (double)zi + 160.0);
            double y1 = 90.0 - yy + (double)zi;

            if (x1 < 0 || x1 > 320) continue;
            if (RR[x1] <= y1) continue;    /* hidden-line: already drawn closer */
            RR[x1] = y1;

            int vdu_x = x1 * 4;
            int vdu_y = 882 - (int)(y1 * 4.0);  /* 850 - y1*4 + 32, Y-flipped */
            vdu_plot_point(vdu_x, vdu_y);
        }
    }

    /* Wait for keypress */
    mos->getkey();
    return 0;
}
