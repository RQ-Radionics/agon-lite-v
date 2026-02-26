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

#endif /* MOS_VDP_H */
