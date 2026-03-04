/*
 * mos_api.h - MOS API for ESP32-MOS
 *
 * On the original Agon (eZ80), applications called MOS functions via RST 8
 * with a function number in register C.  On ESP32 there is no such mechanism.
 * Instead, this header exposes the same set of operations as ordinary C
 * functions that can be called directly.
 *
 * ABI notes:
 *  - All path strings must be null-terminated.
 *  - File handles are standard C FILE pointers cast to uint32_t for API
 *    compatibility; use the MOS_FH_* cast helpers.
 *  - Error codes use the FR_* / MOS_* values from mos_types.h.
 */

#ifndef MOS_API_H
#define MOS_API_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdio.h>
#include "mos_types.h"
#include "mos_api_table.h"

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------
 * Console I/O
 * ------------------------------------------------------------------------- */

/** Read one character from the console (blocking). */
uint8_t mos_api_getkey(void);

/** Write one character to the console. */
void    mos_api_putch(uint8_t c);

/** Write a null-terminated string to the console. */
void    mos_api_puts(const char *s);

/** Read a line from the console into buf (up to len-1 chars). */
int     mos_api_editline(char *buf, int len);

/* -------------------------------------------------------------------------
 * Filesystem — file operations
 * Mirrors original mos_FOPEN, mos_FCLOSE, mos_FGETC, mos_FPUTC,
 * mos_FEOF, mos_FLSEEK, mos_FREAD, mos_FWRITE, mos_GETFIL
 * ------------------------------------------------------------------------- */

/** Open a file. mode: 'r', 'w', 'a', 'r+', 'w+'. Returns handle or 0. */
uint8_t mos_api_fopen(const char *path, const char *mode);

/** Close a file handle. Returns 0 on success. */
int     mos_api_fclose(uint8_t fh);

/** Read one byte from a file. Returns byte or -1 on EOF/error. */
int     mos_api_fgetc(uint8_t fh);

/** Write one byte to a file. Returns 0 on success, -1 on error. */
int     mos_api_fputc(uint8_t c, uint8_t fh);

/** Returns non-zero if file handle is at end of file. */
int     mos_api_feof(uint8_t fh);

/** Seek within file. whence: SEEK_SET/SEEK_CUR/SEEK_END. */
int     mos_api_flseek(uint8_t fh, long offset, int whence);

/** Read up to count bytes from fh into buf. Returns bytes read. */
size_t  mos_api_fread(void *buf, size_t size, size_t count, uint8_t fh);

/** Write count elements of size bytes from buf to fh. Returns elements written. */
size_t  mos_api_fwrite(const void *buf, size_t size, size_t count, uint8_t fh);

/** Return current file position. */
long    mos_api_ftell(uint8_t fh);

/* -------------------------------------------------------------------------
 * Filesystem — directory / path operations
 * Mirrors original mos_DIR, mos_CD, mos_MKDIR, mos_REN,
 * mos_COPY, mos_DEL, mos_GETERROR
 * ------------------------------------------------------------------------- */

/** List directory path to console. Returns FR_OK or error. */
int  mos_api_dir(const char *path);

/** Change current directory. Returns FR_OK or error. */
int  mos_api_cd(const char *path);

/** Make a directory. Returns FR_OK or error. */
int  mos_api_mkdir(const char *path);

/** Rename / move a file. Returns FR_OK or error. */
int  mos_api_rename(const char *src, const char *dst);

/** Copy a file. Returns FR_OK or error. */
int  mos_api_copy(const char *src, const char *dst);

/** Delete a file. Returns FR_OK or error. */
int  mos_api_del(const char *path);

/** Return human-readable string for an error code. */
const char *mos_api_geterror(int error);

/* -------------------------------------------------------------------------
 * System variables
 * Mirrors original mos_SETVARIABLE / mos_GETVARIABLE
 * ------------------------------------------------------------------------- */

/** Set a system variable (string value). Returns FR_OK or error. */
int  mos_api_setvariable(const char *name, const char *value);

/** Get a system variable value into buf (up to buf_len-1 chars). */
int  mos_api_getvariable(const char *name, char *buf, size_t buf_len);

/* -------------------------------------------------------------------------
 * OSCLI — execute a MOS command string
 * Mirrors original mos_OSCLI
 * ------------------------------------------------------------------------- */

/** Execute a MOS command string. Returns FR_OK or error. */
int  mos_api_oscli(const char *cmd);

/* -------------------------------------------------------------------------
 * RTC / Clock
 * Mirrors original mos_GETRTC / mos_SETRTC
 * ------------------------------------------------------------------------- */

/** Get current time as "Weekday,DD Mon YYYY.HH:MM:SS" into buf. */
int  mos_api_getrtc(char *buf, size_t buf_len);

/** Set RTC from a struct tm. Returns 0 on success. */
int  mos_api_setrtc(int year, int month, int day,
                    int hour, int minute, int second);

/* -------------------------------------------------------------------------
 * Misc
 * ------------------------------------------------------------------------- */

/** Return MOS version string (e.g. "3.0.1"). */
const char *mos_api_version(void);

/** Return MOS variant string (e.g. "ESP32-S3"). */
const char *mos_api_variant(void);

/**
 * Initialise the jump table.
 * Must be called once from app_main before launching any user programs.
 */
void mos_api_table_init(void);

/**
 * Return pointer to the MOS API jump table.
 * The loader passes this pointer as the third argument to the user entry point.
 */
t_mos_api *mos_api_table_get(void);

/**
 * Register the exit() implementation from mos_loader.
 * Must be called after mos_api_table_init() and before launching any user
 * program.  Avoids a circular component dependency between mos_api and
 * mos_loader.
 *
 * @param fn  Function that terminates the running user program via longjmp.
 *            Pass NULL to clear (e.g. during teardown).
 */
void mos_api_set_exit_fn(void (*fn)(int status));

/**
 * Flash I/O proxy task lifecycle (ESP32-P4 only).
 *
 * On ESP32-P4 the user task stack lives in PSRAM. The flash driver asserts
 * that the current task stack is in internal DRAM before disabling the cache.
 * To satisfy this, all FAT/flash I/O is routed through a small proxy task
 * whose stack is in internal DRAM.
 *
 * mos_flash_io_start() — spawn the proxy task. Call before launching a user
 *                         program (from mos_loader, which has a DRAM stack).
 * mos_flash_io_stop()  — signal the proxy task to exit and wait for it.
 *                         Call after the user program returns.
 *
 * On ESP32-S3 these are no-ops (user task stack is already in DRAM).
 */
void mos_flash_io_start(void);
void mos_flash_io_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* MOS_API_H */
