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
 * Binary system variable block — mirrors the agon-mos sysvar layout.
 * Programs obtain a pointer via mos->sysvars() and read fields by offset.
 * Updated automatically by the VDP parser as packets arrive.
 *
 * Offsets match agon-mos globals.asm sysvar_* constants exactly so that
 * programs ported from eZ80 can use the same offset arithmetic.
 */
typedef struct __attribute__((packed)) {
    uint32_t time;           /* 0x00 +4: centiseconds since boot (~10 Hz timer) */
    uint8_t  vpd_pflags;     /* 0x04 +1: VDP protocol completion flags          */
    uint8_t  keyascii;       /* 0x05 +1: last ASCII keycode received             */
    uint8_t  keymods;        /* 0x06 +1: modifier keys (shift/ctrl/alt bitmask) */
    uint8_t  cursorX;        /* 0x07 +1: cursor column                          */
    uint8_t  cursorY;        /* 0x08 +1: cursor row                             */
    uint8_t  scrchar;        /* 0x09 +1: character at cursor (SCRCHAR packet)   */
    uint8_t  scrpixel[3];    /* 0x0A +3: pixel R,G,B (SCRPIXEL packet)         */
    uint8_t  audioChannel;   /* 0x0D +1: audio channel                         */
    uint8_t  audioSuccess;   /* 0x0E +1: audio note queued (0=no, 1=yes)       */
    uint16_t scrWidth;       /* 0x0F +2: screen width in pixels                */
    uint16_t scrHeight;      /* 0x11 +2: screen height in pixels               */
    uint8_t  scrCols;        /* 0x13 +1: screen columns in characters          */
    uint8_t  scrRows;        /* 0x14 +1: screen rows in characters             */
    uint8_t  scrColours;     /* 0x15 +1: number of colours                     */
    uint8_t  scrpixelIndex;  /* 0x16 +1: palette index of last pixel read      */
    uint8_t  vkeycode;       /* 0x17 +1: virtual key code                      */
    uint8_t  vkeydown;       /* 0x18 +1: virtual key state (0=up, 1=down)      */
    uint8_t  vkeycount;      /* 0x19 +1: incremented every key packet          */
    uint8_t  rtc[6];         /* 0x1A +6: packed RTC union (see vdp_time_t)     */
    uint16_t keydelay;       /* 0x20 +2: keyboard repeat delay                 */
    uint16_t keyrate;        /* 0x22 +2: keyboard repeat rate                  */
    uint8_t  keyled;         /* 0x24 +1: keyboard LED status                   */
} t_mos_sysvars;

/**
 * Return a pointer to the live system variable block.
 * Updated in-place as VDP packets arrive; valid for firmware lifetime.
 */
t_mos_sysvars *mos_vdp_get_sysvars(void);

/**
 * Screen information received from the VDP via PACKET_MODE (cmd=0x06).
 * Sent by the VDP proactively on connect/mode-change, and on request
 * via VDU 23,0,0x86.
 *
 * Payload byte order (console8 firmware):
 *   [0..1] width    pixels, uint16 LE
 *   [2..3] height   pixels, uint16 LE
 *   [4]    cols     characters wide
 *   [5]    rows     characters tall
 *   [6]    colours  colour depth (log2 of number of colours)
 *   [7]    mode     video mode number
 */
typedef struct {
    uint16_t width;
    uint16_t height;
    uint8_t  cols;
    uint8_t  rows;
    uint8_t  colours;
    uint8_t  mode;
    bool     valid;   /* false until the first PACKET_MODE is received */
} mos_vdp_screen_t;

/**
 * Return a pointer to the last screen information received from the VDP.
 * The pointer is valid for the lifetime of the application.
 * Check .valid before using the fields.
 */
const mos_vdp_screen_t *mos_vdp_get_screen(void);

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

/**
 * Request current screen information from the VDP: sends VDU 23,0,0x86
 * and blocks until the VDP responds with PACKET_MODE, updating the struct
 * returned by mos_vdp_get_screen().  Times out after 200 ms.
 *
 * Use this instead of vdp_sync() when you need up-to-date screen info
 * (e.g. after a MODE change), because vdp_sync() only waits for a GP
 * response and the MODE packet may arrive after it returns.
 */
void mos_vdp_request_mode(void);

#endif /* MOS_VDP_H */
