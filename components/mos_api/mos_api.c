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
#include "mos_vdp.h"
#include "mos_vdp_router.h"
#include "mos_fs.h"
#include "mos_file.h"
#include "mos_sysvars.h"
#include "mos_shell.h"
#include "mos_editor.h"
#include "mos_version.h"
#include "mos_types.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "mos_i2c.h"
#include "mos_uart.h"

/* Allocate from PSRAM for user programs (BBC BASIC needs large heaps) */
static void *mos_malloc_spiram(size_t size)
{
    return heap_caps_malloc(size, MALLOC_CAP_SPIRAM);
}

static void mos_free_spiram(void *ptr)
{
    heap_caps_free(ptr);
}

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
 * Bulk load / save
 * ========================================================================= */

int mos_api_load(const char *path, void *addr, size_t *bytes_read)
{
    if (!path || !addr || !bytes_read) return -1;
    char resolved[MOS_PATH_MAX];
    mos_fs_resolve(path, resolved, sizeof(resolved));
    FILE *f = fopen(resolved, "rb");
    if (!f) return -1;
    /* Read in chunks so we don't need a size query first */
    size_t total = 0;
    uint8_t *dst = (uint8_t *)addr;
    size_t n;
    while ((n = fread(dst + total, 1, 4096, f)) > 0)
        total += n;
    int err = ferror(f);
    fclose(f);
    if (err) return -1;
    *bytes_read = total;
    return 0;
}

int mos_api_save(const char *path, const void *addr, size_t len)
{
    if (!path || !addr) return -1;
    char resolved[MOS_PATH_MAX];
    mos_fs_resolve(path, resolved, sizeof(resolved));
    FILE *f = fopen(resolved, "wb");
    if (!f) return -1;
    size_t written = fwrite(addr, 1, len, f);
    int err = ferror(f);
    fclose(f);
    if (err || written != len) return -1;
    return 0;
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

/* =========================================================================
 * Timing
 * ========================================================================= */

void mos_api_delay_ms(uint32_t ms)
{
    if (ms == 0) return;
    vTaskDelay(pdMS_TO_TICKS(ms));
}

uint32_t mos_api_get_ticks_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

int mos_api_kbhit(void)
{
    return mos_kbhit() ? 1 : 0;
}

/* =========================================================================
 * I2C master
 * ========================================================================= */

static void mos_api_i2c_open(uint8_t frequency)
{
    mos_I2C_OPEN(frequency);
}

static void mos_api_i2c_close(void)
{
    mos_I2C_CLOSE();
}

static uint8_t mos_api_i2c_write(uint8_t addr, uint8_t size, const char *buf)
{
    return mos_I2C_WRITE(addr, size, buf);
}

static uint8_t mos_api_i2c_read(uint8_t addr, uint8_t size, char *buf)
{
    return mos_I2C_READ(addr, size, buf);
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

/* ── VDP screen sysvars — read callbacks for MOS_VAR_CODE variables ─── */

#define VDP_SYSVAR_READ(name, field, fmt)                           \
static int vdp_read_##name(char *buf, int *len) {                   \
    const mos_vdp_screen_t *s = mos_vdp_get_screen();              \
    if (!s->valid) return -1;                                       \
    int n = snprintf(buf, (size_t)*len, fmt, s->field);            \
    if (n < 0 || n >= *len) return -1;                             \
    *len = n;                                                       \
    return 0;                                                       \
}

VDP_SYSVAR_READ(mode,    mode,    "%u")
VDP_SYSVAR_READ(cols,    cols,    "%u")
VDP_SYSVAR_READ(rows,    rows,    "%u")
VDP_SYSVAR_READ(colours, colours, "%u")
VDP_SYSVAR_READ(width,   width,   "%u")
VDP_SYSVAR_READ(height,  height,  "%u")

static t_mosCodeSystemVariable s_vdp_cv_mode    = { vdp_read_mode,    NULL };
static t_mosCodeSystemVariable s_vdp_cv_cols    = { vdp_read_cols,    NULL };
static t_mosCodeSystemVariable s_vdp_cv_rows    = { vdp_read_rows,    NULL };
static t_mosCodeSystemVariable s_vdp_cv_colours = { vdp_read_colours, NULL };
static t_mosCodeSystemVariable s_vdp_cv_width   = { vdp_read_width,   NULL };
static t_mosCodeSystemVariable s_vdp_cv_height  = { vdp_read_height,  NULL };

static void register_vdp_sysvars(void)
{
    createOrUpdateSystemVariable("VDP$Mode",    MOS_VAR_CODE, &s_vdp_cv_mode);
    createOrUpdateSystemVariable("VDP$Cols",    MOS_VAR_CODE, &s_vdp_cv_cols);
    createOrUpdateSystemVariable("VDP$Rows",    MOS_VAR_CODE, &s_vdp_cv_rows);
    createOrUpdateSystemVariable("VDP$Colours", MOS_VAR_CODE, &s_vdp_cv_colours);
    createOrUpdateSystemVariable("VDP$Width",   MOS_VAR_CODE, &s_vdp_cv_width);
    createOrUpdateSystemVariable("VDP$Height",  MOS_VAR_CODE, &s_vdp_cv_height);
}

/* Logical OS-unit dimensions (BBC Micro / Agon standard) */
#define LOGICAL_SCRW 1280
#define LOGICAL_SCRH 1024

static int mos_api_read_pixel(int x, int y,
                               uint8_t *r, uint8_t *g, uint8_t *b,
                               uint8_t *index)
{
    /* Convert OS units to mode pixels using current screen dimensions */
    t_mos_sysvars *sv = mos_vdp_router_get_sysvars();
    int pw = (sv && sv->scrWidth)  ? (int)((long)x * sv->scrWidth  / LOGICAL_SCRW) : x;
    int ph = (sv && sv->scrHeight) ? (int)((long)y * sv->scrHeight / LOGICAL_SCRH) : y;
    mos_vdp_router_read_pixel(pw, ph, r, g, b, index);
    return (int)*index;
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

    t->load       = mos_api_load;
    t->save       = mos_api_save;

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
    t->sysvars     = mos_vdp_router_get_sysvars;

    t->oscli      = mos_api_oscli;

    t->getrtc     = mos_api_getrtc;
    t->setrtc     = mos_api_setrtc;

    t->malloc     = mos_malloc_spiram;
    t->free       = mos_free_spiram;

    t->mos_version = mos_api_version;
    t->mos_variant = mos_api_variant;
    t->geterror   = mos_api_geterror;

    t->delay_ms     = mos_api_delay_ms;
    t->get_ticks_ms = mos_api_get_ticks_ms;

    t->kbhit        = mos_api_kbhit;

    t->i2c_open     = mos_api_i2c_open;
    t->i2c_close    = mos_api_i2c_close;
    t->i2c_write    = mos_api_i2c_write;
    t->i2c_read     = mos_api_i2c_read;

    t->uopen    = mos_uopen;
    t->uclose   = mos_uclose;
    t->ugetc    = mos_ugetc;
    t->uputc    = mos_uputc;

    t->vdp_sync         = mos_vdp_router_sync;
    t->vdp_request_mode = mos_vdp_router_request_mode;

    t->exit         = NULL;  /* registered later via mos_api_set_exit_fn() */

    t->read_pixel   = mos_api_read_pixel;

    register_vdp_sysvars();
}

void mos_api_set_exit_fn(void (*fn)(int status))
{
    s_mos_api_table.exit = fn;
}
