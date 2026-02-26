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

} t_mos_api;

#endif /* MOS_API_TABLE_H */
