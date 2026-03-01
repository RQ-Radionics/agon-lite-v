/*
 * mos_vdp.h - VDP channel over TCP for ESP32-MOS
 *
 * Listens on MOS_VDP_TCP_PORT.  The first client that connects becomes
 * the VDP.  Only one VDP connection is active at a time.
 *
 * All functions are thread-safe.
 */

#ifndef MOS_VDP_H
#define MOS_VDP_H

#include <stdbool.h>
#include <stdint.h>

/** TCP port the VDP server listens on. */
#define MOS_VDP_TCP_PORT    2323

/**
 * Start the VDP TCP server task.
 * Must be called after WiFi is connected.
 * Returns 0 on success, -1 if the server socket cannot be created.
 */
int  mos_vdp_init(void);

/** Returns true if a VDP client is currently connected. */
bool mos_vdp_connected(void);

/**
 * Send one byte to the VDP.
 * Blocks briefly if the TX buffer is full; drops the byte if not connected.
 */
void mos_vdp_putch(uint8_t c);

/**
 * Receive one byte from the VDP (blocking).
 * Returns the byte value, or -1 if the connection was lost.
 */
int  mos_vdp_getch(void);

/**
 * Check if a byte is available from the VDP (non-blocking).
 */
bool mos_vdp_kbhit(void);

/**
 * Returns true if the VDP has just disconnected and the session should end.
 * Used by the loader to abort a running user program when the client drops.
 */
bool mos_vdp_disconnecting(void);

/**
 * Signal the VDP layer to abort any blocked/running user program.
 * After this call, mos_vdp_getch() returns -1 immediately and
 * mos_vdp_putch() / mos_puts() discard output silently.
 * The flag is cleared automatically when the next client connects.
 * Safe to call from any task — does NOT longjmp.
 */
void mos_vdp_abort(void);

/**
 * Flush the TX write buffer immediately.
 * mos_vdp_putch() accumulates bytes in a buffer and only calls send()
 * when the buffer is full.  Call mos_vdp_flush() to force a send of any
 * pending bytes (e.g. after a complete VDU command sequence, or before
 * blocking on input).
 */
void mos_vdp_flush(void);

/**
 * Synchronise with the VDP: send a General Poll (VDU 23,0,&80,n) and
 * block until the VDP echoes back the response packet.  This guarantees
 * that all previously queued VDU bytes have been processed by the VDP
 * before returning.  Equivalent to *FX 19 / WAIT_VBLANK on the Agon.
 * Times out after 200 ms if no VDP is connected.
 */
void mos_vdp_sync(void);

#endif /* MOS_VDP_H */
