/*
 * mos_vdp_router.h — VDP I/O router: selects between TCP VDP and internal VDP.
 *
 * Provides a unified interface matching mos_vdp.h so that mos_hal.c and
 * mos_api.c can be written once without #ifdefs.
 *
 * Backend selection (compile-time, via Kconfig):
 *
 *   CONFIG_MOS_VDP_INTERNAL_ENABLED=y  (Olimex ESP32-P4-PC with HDMI)
 *     → connected()  : returns mos_vdp_internal_connected() (always true after init)
 *     → putch()      : calls mos_vdp_internal_putch()
 *     → getch()      : calls mos_vdp_internal_getch()
 *     → kbhit()      : calls mos_vdp_internal_kbhit()
 *     → disconnecting(): always false (internal VDP never disconnects)
 *     → flush()      : no-op
 *     → sync()       : no-op (rendering is synchronous in render task)
 *     → request_mode(): no-op (mode info is always current)
 *     → get_sysvars(): returns internal sysvar block
 *
 *   CONFIG_MOS_VDP_INTERNAL_ENABLED not set (Waveshare / TCP-only mode)
 *     → all calls delegated to mos_vdp_*() directly
 *
 * When both are available (future: dual mode), TCP takes priority if a
 * client is connected, otherwise falls back to internal.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Forward declaration for sysvars (full definition in mos_api_table.h) */
typedef struct t_mos_sysvars_s t_mos_sysvars;

/**
 * Returns true if any VDP backend is currently active and ready to
 * accept VDU bytes.  Replaces mos_vdp_connected() in mos_hal.c.
 */
bool mos_vdp_router_connected(void);

/**
 * Returns true if the active VDP is in the process of disconnecting.
 * Always false for the internal VDP.
 */
bool mos_vdp_router_disconnecting(void);

/**
 * Send one VDU byte to the active VDP backend.
 */
void mos_vdp_router_putch(uint8_t c);

/**
 * Read one byte from the active VDP backend (blocking).
 * Returns the byte, or -1 if the connection was lost / VDP shut down.
 */
int  mos_vdp_router_getch(void);

/**
 * Returns true if a byte is available from the active VDP (non-blocking).
 */
bool mos_vdp_router_kbhit(void);

/**
 * Flush any pending output to the active VDP backend.
 * No-op for the internal VDP.
 */
void mos_vdp_router_flush(void);

/**
 * Synchronise with the VDP: wait until all queued VDU bytes have been
 * processed.  No-op for the internal VDP (rendering is asynchronous but
 * the queue drains automatically; adding a full sync would require a
 * semaphore — deferred for now).
 */
void mos_vdp_router_sync(void);

/**
 * Request current screen mode information from the VDP.
 * No-op for the internal VDP (mode info is always current in sysvars).
 */
void mos_vdp_router_request_mode(void);

/**
 * Return a pointer to the live system variable block.
 * For TCP VDP: returns mos_vdp_get_sysvars().
 * For internal VDP: returns the internal sysvar block.
 */
t_mos_sysvars *mos_vdp_router_get_sysvars(void);

/**
 * Read the colour of a pixel at logical coordinate (x, y).
 *
 * For the internal VDP, reads directly from the framebuffer (no round-trip).
 * For the TCP VDP, sends VDU 23,0,&84 and waits up to 200 ms for the reply.
 * Fills *r, *g, *b with the RGB888 colour and *index with the palette index.
 * On failure (timeout or no VDP), all values are set to 0.
 */
void mos_vdp_router_read_pixel(int x, int y,
                                uint8_t *r, uint8_t *g, uint8_t *b,
                                uint8_t *index);
