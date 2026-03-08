/*
 * mos_api_table.h - MOS jump table for user programs
 *
 * User programs running from PSRAM obtain the API via a pointer to
 * t_mos_api located at MOS_API_ADDR (a fixed address in DRAM/IRAM
 * that is always valid while MOS is running).
 *
 * Usage in user code:
 *
 *   #include "mos_api_table.h"
 *   #define MOS ((volatile t_mos_api *)MOS_API_ADDR)
 *
 *   MOS->putch('H');
 *   MOS->puts("Hello!\n");
 *
 * The address MOS_API_ADDR is guaranteed not to move between firmware
 * versions with the same major version number.
 */

#ifndef MOS_API_TABLE_H
#define MOS_API_TABLE_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "mos_sysvars_block.h"   /* t_mos_sysvars */

/* Magic value at offset 0 — lets user code verify the table is present */
#define MOS_API_MAGIC   0x4D4F5301UL   /* 'MOS\x01' */

/* Version of this struct layout */
#define MOS_API_VERSION 1

/*
 * Jump table struct.
 * Fields must NEVER be reordered or removed — only appended.
 * Increment MOS_API_VERSION when appending new fields.
 */
typedef struct {
    /* --- identity --- */
    uint32_t magic;         /* MOS_API_MAGIC */
    uint32_t version;       /* MOS_API_VERSION */

    /* --- console I/O --- */
    uint8_t  (*getkey)(void);
    void     (*putch)(uint8_t c);
    void     (*puts)(const char *s);
    int      (*editline)(char *buf, int len);

    /* --- bulk file load/save --- */
    /* mos_load: open file, read entire contents into *addr, set *bytes_read.
     * Returns 0 on success, -1 on error. */
    int      (*load)(const char *path, void *addr, size_t *bytes_read);
    /* mos_save: write len bytes from addr to file (created/truncated).
     * Returns 0 on success, -1 on error. */
    int      (*save)(const char *path, const void *addr, size_t len);

    /* --- file I/O --- */
    uint8_t  (*fopen)(const char *path, const char *mode);
    int      (*fclose)(uint8_t fh);
    int      (*fgetc)(uint8_t fh);
    int      (*fputc)(uint8_t c, uint8_t fh);
    int      (*feof)(uint8_t fh);
    int      (*flseek)(uint8_t fh, long offset, int whence);
    size_t   (*fread)(void *buf, size_t sz, size_t n, uint8_t fh);
    size_t   (*fwrite)(const void *buf, size_t sz, size_t n, uint8_t fh);
    long     (*ftell)(uint8_t fh);

    /* --- directory / path --- */
    int      (*dir)(const char *path);
    int      (*cd)(const char *path);
    int      (*mkdir)(const char *path);
    int      (*rename)(const char *src, const char *dst);
    int      (*copy)(const char *src, const char *dst);
    int      (*del)(const char *path);

    /* --- system variables --- */
    int      (*setvariable)(const char *name, const char *value);
    int      (*getvariable)(const char *name, char *buf, size_t len);
    /* Return pointer to the live binary sysvar block (agon-mos compatible).
     * Fields are updated in-place as VDP packets arrive.
     * Cast to t_mos_sysvars* or index by sysvar_* byte offsets. */
    t_mos_sysvars *(*sysvars)(void);

    /* --- OSCLI --- */
    int      (*oscli)(const char *cmd);

    /* --- RTC --- */
    int      (*getrtc)(char *buf, size_t len);
    int      (*setrtc)(int yr, int mo, int da, int ho, int mi, int se);

    /* --- heap (user programs must use these, not newlib malloc directly) --- */
    void    *(*malloc)(size_t size);
    void     (*free)(void *ptr);

    /* --- misc --- */
    const char *(*mos_version)(void);
    const char *(*mos_variant)(void);
    const char *(*geterror)(int err);

    /* --- timing --- */
    void     (*delay_ms)(uint32_t ms);   /* yield-friendly delay (vTaskDelay) */
    uint32_t (*get_ticks_ms)(void);      /* milliseconds since boot */

    /* --- non-blocking input check --- */
    int      (*kbhit)(void);             /* non-zero if a key is available */

    /* --- I2C master --- */
    void     (*i2c_open)(uint8_t frequency);
    void     (*i2c_close)(void);
    uint8_t  (*i2c_write)(uint8_t addr, uint8_t size, const char *buf);
    uint8_t  (*i2c_read)(uint8_t addr, uint8_t size, char *buf);

    /* --- secondary UART (agon-mos 15h-18h) --- */
    /* uopen: open UART1 at given baud rate. Returns 0 on success, -1 on error. */
    int      (*uopen)(uint32_t baud);
    /* uclose: close UART1 and free its resources. */
    void     (*uclose)(void);
    /* ugetc: read one byte (blocking). Returns 0-255 or -1 if not open. */
    int      (*ugetc)(void);
    /* uputc: write one byte. Returns 0 on success, -1 if not open. */
    int      (*uputc)(uint8_t c);

    /* --- VDP synchronisation --- */
    /* Send a General Poll to the VDP and block until the response arrives.
     * This ensures all previously queued VDU bytes have been processed by
     * the VDP before BASIC continues — equivalent to *FX 19 on the Agon. */
    void     (*vdp_sync)(void);

    /* Request up-to-date screen info from the VDP: sends VDU 23,0,0x86 and
     * blocks until the VDP responds with PACKET_MODE.  After this call,
     * getvariable("VDP$Mode") etc. return current values.  Times out 200 ms. */
    void     (*vdp_request_mode)(void);

    /* --- program exit --- */
    /* Terminate the current user program and return to the MOS shell.
     * Implemented by the loader via longjmp inside the user_task frame.
     * User programs should call this instead of exit() / _exit(). */
    void     (*exit)(int status);

    /* --- pixel read (POINT) --- */
    /* Read the colour of a pixel at logical OS-unit coordinate (x, y).
     * Converts OS units to mode pixels and queries the active VDP backend.
     * For internal VDP: reads directly from framebuffer (no round-trip).
     * For TCP VDP: sends VDU 23,0,&84 and waits up to 200 ms.
     * Fills *r, *g, *b with RGB888 colour and *index with palette index.
     * On failure, all outputs are set to 0.
     * Returns scrpixelIndex (palette index), or -1 on failure/no VDP. */
    int      (*read_pixel)(int x, int y,
                           uint8_t *r, uint8_t *g, uint8_t *b,
                           uint8_t *index);

} t_mos_api;

#endif /* MOS_API_TABLE_H */
