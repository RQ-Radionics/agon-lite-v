/*
 * mos_api.c - MOS API implementation for ESP32-MOS
 *
 * Replaces the RST 8 dispatch table of the original Agon eZ80 MOS.
 * All operations delegate to the underlying POSIX VFS / mos_* components.
 */

#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <sys/dirent.h>
#include <errno.h>
#include "mos_api.h"
#include "mos_hal.h"
#include "mos_fs.h"
#include "mos_file.h"
#include "mos_sysvars.h"
#include "mos_shell.h"
#include "mos_editor.h"
#include "mos_version.h"
#include "mos_types.h"
#include "esp_log.h"

static const char *TAG = "mos_api";

/* -------------------------------------------------------------------------
 * File handle table
 * Handle 0 is invalid (reserved).  Handles 1..MOS_MAX_OPEN_FILES map to
 * FILE* entries in s_fh[].
 * ------------------------------------------------------------------------- */
static FILE *s_fh[MOS_MAX_OPEN_FILES + 1];  /* index 0 unused */

static uint8_t alloc_fh(FILE *f)
{
    for (int i = 1; i <= MOS_MAX_OPEN_FILES; i++) {
        if (s_fh[i] == NULL) {
            s_fh[i] = f;
            return (uint8_t)i;
        }
    }
    return 0; /* no free handle */
}

static FILE *get_fh(uint8_t fh)
{
    if (fh == 0 || fh > MOS_MAX_OPEN_FILES) return NULL;
    return s_fh[fh];
}

/* Error table (mirrors mos_shell.c s_errors) */
static const char *s_errors[] = {
    "OK",
    "Error accessing storage",
    "Internal error",
    "Storage failure",
    "Could not find file",
    "Could not find path",
    "Invalid path name",
    "Access denied or directory full",
    "Access denied",
    "Invalid file/directory object",
    "Storage is write protected",
    "Logical drive number is invalid",
    "Volume has no work area",
    "No valid FAT volume",
    "Error occurred during mkfs",
    "Volume timeout",
    "Volume locked",
    "LFN working buffer could not be allocated",
    "Too many open files",
    "Invalid parameter",
    "Invalid command",
    "Invalid executable",
    "Out of memory",
    "Not implemented",
    "Load overlaps system area",
    "Bad string",
    "Too deep",
};
#define ERRORS_COUNT (sizeof(s_errors)/sizeof(char *))

/* =========================================================================
 * Console I/O
 * ========================================================================= */

uint8_t mos_api_getkey(void)
{
    return (uint8_t)mos_getch();
}

void mos_api_putch(uint8_t c)
{
    mos_putch((char)c);
}

void mos_api_puts(const char *s)
{
    const uint8_t *b = (const uint8_t *)s;
    ESP_LOGI(TAG, "mos_api_puts ptr=%p bytes=[%02x %02x %02x %02x %02x]",
             s, b ? b[0] : 0, b ? b[1] : 0, b ? b[2] : 0,
             b ? b[3] : 0, b ? b[4] : 0);
    mos_puts(s);
}

int mos_api_editline(char *buf, int len)
{
    return mos_editor_readline("", buf, (size_t)len);
}

/* =========================================================================
 * File operations
 * ========================================================================= */

uint8_t mos_api_fopen(const char *path, const char *mode)
{
    char resolved[MOS_PATH_MAX];
    mos_fs_resolve(path, resolved, sizeof(resolved));
    FILE *f = fopen(resolved, mode);
    if (!f) return 0;
    uint8_t fh = alloc_fh(f);
    if (fh == 0) {
        fclose(f);
        return 0;
    }
    return fh;
}

int mos_api_fclose(uint8_t fh)
{
    FILE *f = get_fh(fh);
    if (!f) return -1;
    int r = fclose(f);
    s_fh[fh] = NULL;
    return r;
}

int mos_api_fgetc(uint8_t fh)
{
    FILE *f = get_fh(fh);
    if (!f) return -1;
    return fgetc(f);
}

int mos_api_fputc(uint8_t c, uint8_t fh)
{
    FILE *f = get_fh(fh);
    if (!f) return -1;
    return (fputc(c, f) == EOF) ? -1 : 0;
}

int mos_api_feof(uint8_t fh)
{
    FILE *f = get_fh(fh);
    if (!f) return 1;
    return feof(f);
}

int mos_api_flseek(uint8_t fh, long offset, int whence)
{
    FILE *f = get_fh(fh);
    if (!f) return -1;
    return fseek(f, offset, whence);
}

size_t mos_api_fread(void *buf, size_t size, size_t count, uint8_t fh)
{
    FILE *f = get_fh(fh);
    if (!f) return 0;
    return fread(buf, size, count, f);
}

size_t mos_api_fwrite(const void *buf, size_t size, size_t count, uint8_t fh)
{
    FILE *f = get_fh(fh);
    if (!f) return 0;
    return fwrite(buf, size, count, f);
}

long mos_api_ftell(uint8_t fh)
{
    FILE *f = get_fh(fh);
    if (!f) return -1;
    return ftell(f);
}

/* =========================================================================
 * Directory / path operations
 * ========================================================================= */

int mos_api_dir(const char *path)
{
    char resolved[MOS_PATH_MAX];
    mos_fs_resolve(path ? path : ".", resolved, sizeof(resolved));

    DIR *d = opendir(resolved);
    if (!d) return FR_NO_PATH;

    mos_printf("\r\nDirectory of %s\r\n\r\n", resolved);
    struct dirent *entry;
    while ((entry = readdir(d)) != NULL) {
        if (entry->d_name[0] == '.') continue;
        char full[MOS_PATH_MAX + 256];
        snprintf(full, sizeof(full), "%s/%s", resolved, entry->d_name);
        struct stat st;
        bool is_dir = false;
        long size   = 0;
        if (stat(full, &st) == 0) {
            is_dir = S_ISDIR(st.st_mode);
            size   = (long)st.st_size;
        }
        if (is_dir) {
            mos_printf("  %-32s  <DIR>\r\n", entry->d_name);
        } else {
            mos_printf("  %-32s  %ld\r\n", entry->d_name, size);
        }
    }
    closedir(d);
    mos_printf("\r\n");
    return FR_OK;
}

int mos_api_cd(const char *path)
{
    if (!path || !*path) {
        mos_printf("%s\r\n", mos_fs_getcwd());
        return FR_OK;
    }
    char resolved[MOS_PATH_MAX];
    mos_fs_resolve(path, resolved, sizeof(resolved));
    if (!mos_isDirectory(resolved)) return FR_NO_PATH;
    return mos_fs_chdir(resolved);
}

int mos_api_mkdir(const char *path)
{
    char resolved[MOS_PATH_MAX];
    mos_fs_resolve(path, resolved, sizeof(resolved));
    return (mkdir(resolved, 0755) == 0) ? FR_OK : FR_INT_ERR;
}

int mos_api_rename(const char *src, const char *dst)
{
    char rsrc[MOS_PATH_MAX], rdst[MOS_PATH_MAX];
    mos_fs_resolve(src, rsrc, sizeof(rsrc));
    mos_fs_resolve(dst, rdst, sizeof(rdst));
    return (rename(rsrc, rdst) == 0) ? FR_OK : FR_INT_ERR;
}

int mos_api_copy(const char *src, const char *dst)
{
    char rsrc[MOS_PATH_MAX], rdst[MOS_PATH_MAX];
    mos_fs_resolve(src, rsrc, sizeof(rsrc));
    mos_fs_resolve(dst, rdst, sizeof(rdst));
    return mos_copyFile(rsrc, rdst);
}

int mos_api_del(const char *path)
{
    char resolved[MOS_PATH_MAX];
    mos_fs_resolve(path, resolved, sizeof(resolved));
    return (remove(resolved) == 0) ? FR_OK : FR_NO_FILE;
}

const char *mos_api_geterror(int error)
{
    if (error >= 0 && error < (int)ERRORS_COUNT) {
        return s_errors[error];
    }
    return "Unknown error";
}

/* =========================================================================
 * System variables
 * ========================================================================= */

int mos_api_setvariable(const char *name, const char *value)
{
    return createOrUpdateSystemVariable((char *)name, MOS_VAR_LITERAL, (void *)value);
}

int mos_api_getvariable(const char *name, char *buf, size_t buf_len)
{
    char *val = expandVariableToken((char *)name);
    if (!val) return FR_INVALID_NAME;
    strncpy(buf, val, buf_len - 1);
    buf[buf_len - 1] = '\0';
    free(val);
    return FR_OK;
}

/* =========================================================================
 * OSCLI
 * ========================================================================= */

int mos_api_oscli(const char *cmd)
{
    return mos_shell_exec(cmd);
}

/* =========================================================================
 * RTC / Clock
 * ========================================================================= */

int mos_api_getrtc(char *buf, size_t buf_len)
{
    time_t now = time(NULL);
    struct tm *t = localtime(&now);
    if (!t) return -1;
    strftime(buf, buf_len, "%a,%d %b %Y.%H:%M:%S", t);
    return 0;
}

int mos_api_setrtc(int year, int month, int day,
                   int hour, int minute, int second)
{
    struct tm t = {0};
    t.tm_year = year  - 1900;
    t.tm_mon  = month - 1;
    t.tm_mday = day;
    t.tm_hour = hour;
    t.tm_min  = minute;
    t.tm_sec  = second;
    time_t epoch = mktime(&t);
    if (epoch == (time_t)-1) return -1;
    struct timeval tv = { .tv_sec = epoch, .tv_usec = 0 };
    return settimeofday(&tv, NULL);
}

/* =========================================================================
 * Version info
 * ========================================================================= */

const char *mos_api_version(void)
{
    return VERSION_STRING;
}

const char *mos_api_variant(void)
{
    return VERSION_VARIANT;
}

/* =========================================================================
 * Jump table — allocated once, pointer passed to user programs as arg
 * ========================================================================= */

static t_mos_api s_mos_api_table;

t_mos_api *mos_api_table_get(void)
{
    return &s_mos_api_table;
}

void mos_api_table_init(void)
{
    t_mos_api *t = &s_mos_api_table;

    t->magic      = MOS_API_MAGIC;
    t->version    = MOS_API_VERSION;

    t->getkey     = mos_api_getkey;
    t->putch      = mos_api_putch;
    t->puts       = mos_api_puts;
    t->editline   = mos_api_editline;

    t->fopen      = mos_api_fopen;
    t->fclose     = mos_api_fclose;
    t->fgetc      = mos_api_fgetc;
    t->fputc      = mos_api_fputc;
    t->feof       = mos_api_feof;
    t->flseek     = mos_api_flseek;
    t->fread      = mos_api_fread;
    t->fwrite     = mos_api_fwrite;
    t->ftell      = mos_api_ftell;

    t->dir        = mos_api_dir;
    t->cd         = mos_api_cd;
    t->mkdir      = mos_api_mkdir;
    t->rename     = mos_api_rename;
    t->copy       = mos_api_copy;
    t->del        = mos_api_del;

    t->setvariable = mos_api_setvariable;
    t->getvariable = mos_api_getvariable;

    t->oscli      = mos_api_oscli;

    t->getrtc     = mos_api_getrtc;
    t->setrtc     = mos_api_setrtc;

    t->malloc     = malloc;
    t->free       = free;

    t->mos_version = mos_api_version;
    t->mos_variant = mos_api_variant;
    t->geterror   = mos_api_geterror;
}
