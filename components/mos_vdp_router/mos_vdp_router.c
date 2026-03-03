/*
 * mos_vdp_router.c — VDP I/O router
 *
 * Selects between the TCP VDP (mos_vdp) and the internal VDP
 * (mos_vdp_internal) at compile time via Kconfig.
 *
 * When CONFIG_MOS_VDP_INTERNAL_ENABLED=y, the internal VDP is used
 * as the primary backend.  The TCP VDP server is still started so
 * that agon-sdl can also connect over the network; when a TCP client
 * connects it takes priority over the internal display.
 *
 * Backend priority (when both are compiled in):
 *   1. TCP VDP — if a client is currently connected
 *   2. Internal VDP — always ready after mos_vdp_internal_init()
 *
 * For boards without CONFIG_MOS_VDP_INTERNAL_ENABLED, all calls are
 * forwarded 1:1 to mos_vdp_*.
 */

#include "mos_vdp_router.h"
#include "mos_vdp.h"
#include "mos_sysvars_block.h"

#ifdef CONFIG_MOS_VDP_INTERNAL_ENABLED
#include "mos_vdp_internal.h"

/* A minimal sysvar block for the internal VDP path.
 * Fields are updated by the keyboard driver (keyascii, keymods, vkeycode,
 * vkeydown, vkeycount) as the internal VDP decodes scancodes.
 * Screen geometry fields (scrWidth, scrHeight, scrCols, scrRows, scrMode)
 * are set once during init and remain constant.
 * All other fields default to 0. */
static t_mos_sysvars s_internal_sysvars = {
    .scrWidth   = 640,
    .scrHeight  = 480,
    .scrCols    = 80,
    .scrRows    = 60,
    .scrMode    = 16,   /* Agon mode 16 = 640×480 */
    .scrColours = 6,    /* log2(64) = 6 */
};

t_mos_sysvars *mos_vdp_router_get_sysvars(void)
{
    /* If a TCP VDP client is connected, use its live sysvar block
     * (it tracks cursor position, pixel reads, audio status, etc.) */
    if (mos_vdp_connected()) return mos_vdp_get_sysvars();
    return &s_internal_sysvars;
}

/* Helper: true if the internal VDP should handle I/O right now
 * (i.e., no TCP client is connected). */
static inline bool use_internal(void)
{
    return !mos_vdp_connected();
}

bool mos_vdp_router_connected(void)
{
    return mos_vdp_connected() || mos_vdp_internal_connected();
}

bool mos_vdp_router_disconnecting(void)
{
    /* Only the TCP VDP can disconnect; internal VDP never disconnects. */
    return mos_vdp_disconnecting();
}

void mos_vdp_router_putch(uint8_t c)
{
    if (use_internal())
        mos_vdp_internal_putch(c);
    else
        mos_vdp_putch(c);
}

int mos_vdp_router_getch(void)
{
    while (1) {
        if (use_internal()) {
            int c = mos_vdp_internal_getch();  /* returns -1 on 100ms timeout */
            if (c >= 0) return c;
            /* Timeout — loop and re-evaluate: a TCP client may have connected */
        } else {
            return mos_vdp_getch();
        }
    }
}

bool mos_vdp_router_kbhit(void)
{
    if (use_internal())
        return mos_vdp_internal_kbhit();
    return mos_vdp_kbhit();
}

void mos_vdp_router_flush(void)
{
    if (use_internal())
        mos_vdp_internal_flush();
    else
        mos_vdp_flush();
}

void mos_vdp_router_sync(void)
{
    /* Internal VDP: rendering is handled by the render task asynchronously.
     * A true sync would need a semaphore; for now this is a no-op because
     * the render task drains the queue faster than any MOS shell output. */
    if (!use_internal())
        mos_vdp_sync();
}

void mos_vdp_router_request_mode(void)
{
    /* Internal VDP: mode is always current in s_internal_sysvars. */
    if (!use_internal())
        mos_vdp_request_mode();
}

#else   /* TCP-only path — no internal VDP compiled in */

t_mos_sysvars *mos_vdp_router_get_sysvars(void)
{
    return mos_vdp_get_sysvars();
}

bool mos_vdp_router_connected(void)      { return mos_vdp_connected(); }
bool mos_vdp_router_disconnecting(void)  { return mos_vdp_disconnecting(); }
void mos_vdp_router_putch(uint8_t c)     { mos_vdp_putch(c); }
int  mos_vdp_router_getch(void)          { return mos_vdp_getch(); }
bool mos_vdp_router_kbhit(void)          { return mos_vdp_kbhit(); }
void mos_vdp_router_flush(void)          { mos_vdp_flush(); }
void mos_vdp_router_sync(void)           { mos_vdp_sync(); }
void mos_vdp_router_request_mode(void)   { mos_vdp_request_mode(); }

#endif  /* CONFIG_MOS_VDP_INTERNAL_ENABLED */
