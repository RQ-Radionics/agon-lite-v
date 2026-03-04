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
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/idf_additions.h"
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

/* Forward declarations needed by the flash I/O proxy below */
static FILE  *s_fh[MOS_MAX_OPEN_FILES + 1];  /* defined fully after proxy */
static uint8_t alloc_fh(FILE *f);
static FILE   *get_fh(uint8_t fh);

/* =========================================================================
 * Flash I/O proxy task (ESP32-P4 only)
 *
 * On ESP32-P4 the user program task runs with a PSRAM stack.  The IDF flash
 * driver asserts (cache_utils.c:127) that the calling task's stack is in
 * internal DRAM before disabling the cache for SPI1 flash access.  This is
 * a hard assert — no Kconfig knob disables it on P4.
 *
 * Solution: a dedicated "flash_io_task" with an internal-DRAM stack services
 * all FAT/VFS calls on behalf of the user task.  The user task sends a
 * typed request to a FreeRTOS queue, blocks on a per-request semaphore, and
 * the proxy task executes the call and signals completion.
 *
 * On ESP32-S3 (shared flash/PSRAM bus, DRAM stack) these are no-ops.
 * ========================================================================= */

#if CONFIG_IDF_TARGET_ESP32P4

typedef enum {
    FIO_FOPEN,
    FIO_FCLOSE,
    FIO_FGETC,
    FIO_FPUTC,
    FIO_FEOF,
    FIO_FLSEEK,
    FIO_FREAD,
    FIO_FWRITE,
    FIO_FTELL,
    FIO_LOAD,
    FIO_SAVE,
    FIO_DIR,
    FIO_CD,
    FIO_MKDIR,
    FIO_RENAME,
    FIO_COPY,
    FIO_DEL,
    FIO_STOP,   /* sentinel: proxy task should exit */
} flash_io_op_t;

typedef struct {
    flash_io_op_t   op;
    SemaphoreHandle_t done;  /* caller blocks on this; proxy gives it */

    /* Arguments — only the relevant fields are used per op */
    const char     *path;
    const char     *path2;   /* rename dst, save addr as path2 (reused) */
    const char     *mode;
    uint8_t         fh;
    uint8_t         c;
    void           *buf;
    const void     *cbuf;
    size_t          size;
    size_t          count;
    long            offset;
    int             whence;

    /* Return values */
    union {
        uint8_t  u8;
        int      i;
        size_t   sz;
        long     l;
    } ret;
} flash_io_req_t;

/* Single-element queue: only one flash I/O in flight at a time
 * (user_task blocks until the proxy responds). */
static QueueHandle_t  s_fio_queue;
static TaskHandle_t   s_fio_task;

/* ---- helpers that run inside flash_io_task (DRAM stack) ---- */

static void fio_do(flash_io_req_t *r)
{
    switch (r->op) {
    case FIO_FOPEN: {
        char resolved[MOS_PATH_MAX];
        mos_fs_resolve(r->path, resolved, sizeof(resolved));
        FILE *f = fopen(resolved, r->mode);
        if (!f) { r->ret.u8 = 0; break; }
        uint8_t fh = alloc_fh(f);
        if (fh == 0) { fclose(f); }
        r->ret.u8 = fh;
        break;
    }
    case FIO_FCLOSE: {
        FILE *f = get_fh(r->fh);
        if (!f) { r->ret.i = -1; break; }
        r->ret.i = fclose(f);
        s_fh[r->fh] = NULL;
        break;
    }
    case FIO_FGETC: {
        FILE *f = get_fh(r->fh);
        r->ret.i = f ? fgetc(f) : -1;
        break;
    }
    case FIO_FPUTC: {
        FILE *f = get_fh(r->fh);
        r->ret.i = f ? ((fputc(r->c, f) == EOF) ? -1 : 0) : -1;
        break;
    }
    case FIO_FEOF: {
        FILE *f = get_fh(r->fh);
        r->ret.i = f ? feof(f) : 1;
        break;
    }
    case FIO_FLSEEK: {
        FILE *f = get_fh(r->fh);
        r->ret.i = f ? fseek(f, r->offset, r->whence) : -1;
        break;
    }
    case FIO_FREAD: {
        FILE *f = get_fh(r->fh);
        r->ret.sz = f ? fread(r->buf, r->size, r->count, f) : 0;
        break;
    }
    case FIO_FWRITE: {
        FILE *f = get_fh(r->fh);
        r->ret.sz = f ? fwrite(r->cbuf, r->size, r->count, f) : 0;
        break;
    }
    case FIO_FTELL: {
        FILE *f = get_fh(r->fh);
        r->ret.l = f ? ftell(f) : -1;
        break;
    }
    case FIO_LOAD: {
        char resolved[MOS_PATH_MAX];
        mos_fs_resolve(r->path, resolved, sizeof(resolved));
        FILE *f = fopen(resolved, "rb");
        if (!f) { r->ret.i = -1; break; }
        size_t total = 0;
        uint8_t *dst = (uint8_t *)r->buf;
        size_t n;
        while ((n = fread(dst + total, 1, 4096, f)) > 0)
            total += n;
        int err = ferror(f);
        fclose(f);
        if (err) { r->ret.i = -1; break; }
        *(size_t *)r->path2 = total;   /* reuse path2 as bytes_read output */
        r->ret.i = 0;
        break;
    }
    case FIO_SAVE: {
        char resolved[MOS_PATH_MAX];
        mos_fs_resolve(r->path, resolved, sizeof(resolved));
        FILE *f = fopen(resolved, "wb");
        if (!f) { r->ret.i = -1; break; }
        size_t written = fwrite(r->cbuf, 1, r->size, f);
        int err = ferror(f);
        fclose(f);
        r->ret.i = (err || written != r->size) ? -1 : 0;
        break;
    }
    case FIO_DIR: {
        char resolved[MOS_PATH_MAX];
        mos_fs_resolve(r->path ? r->path : ".", resolved, sizeof(resolved));
        DIR *d = opendir(resolved);
        if (!d) { r->ret.i = FR_NO_PATH; break; }
        mos_printf("\r\nDirectory of %s\r\n\r\n", resolved);
        struct dirent *entry;
        while ((entry = readdir(d)) != NULL) {
            if (entry->d_name[0] == '.') continue;
            char full[MOS_PATH_MAX + 256];
            snprintf(full, sizeof(full), "%s/%s", resolved, entry->d_name);
            struct stat st;
            bool is_dir = false; long sz = 0;
            if (stat(full, &st) == 0) { is_dir = S_ISDIR(st.st_mode); sz = (long)st.st_size; }
            if (is_dir) mos_printf("  %-32s  <DIR>\r\n", entry->d_name);
            else        mos_printf("  %-32s  %ld\r\n",   entry->d_name, sz);
        }
        closedir(d);
        mos_printf("\r\n");
        r->ret.i = FR_OK;
        break;
    }
    case FIO_CD: {
        if (!r->path || !*r->path) { mos_printf("%s\r\n", mos_fs_getcwd()); r->ret.i = FR_OK; break; }
        char resolved[MOS_PATH_MAX];
        mos_fs_resolve(r->path, resolved, sizeof(resolved));
        if (!mos_isDirectory(resolved)) { r->ret.i = FR_NO_PATH; break; }
        r->ret.i = mos_fs_chdir(resolved);
        break;
    }
    case FIO_MKDIR: {
        char resolved[MOS_PATH_MAX];
        mos_fs_resolve(r->path, resolved, sizeof(resolved));
        r->ret.i = (mkdir(resolved, 0755) == 0) ? FR_OK : FR_INT_ERR;
        break;
    }
    case FIO_RENAME: {
        char rsrc[MOS_PATH_MAX], rdst[MOS_PATH_MAX];
        mos_fs_resolve(r->path,  rsrc, sizeof(rsrc));
        mos_fs_resolve(r->path2, rdst, sizeof(rdst));
        r->ret.i = (rename(rsrc, rdst) == 0) ? FR_OK : FR_INT_ERR;
        break;
    }
    case FIO_COPY: {
        char rsrc[MOS_PATH_MAX], rdst[MOS_PATH_MAX];
        mos_fs_resolve(r->path,  rsrc, sizeof(rsrc));
        mos_fs_resolve(r->path2, rdst, sizeof(rdst));
        r->ret.i = mos_copyFile(rsrc, rdst);
        break;
    }
    case FIO_DEL: {
        char resolved[MOS_PATH_MAX];
        mos_fs_resolve(r->path, resolved, sizeof(resolved));
        r->ret.i = (remove(resolved) == 0) ? FR_OK : FR_NO_FILE;
        break;
    }
    default:
        r->ret.i = -1;
        break;
    }
}

static void flash_io_task(void *arg)
{
    (void)arg;
    flash_io_req_t *r;
    while (xQueueReceive(s_fio_queue, &r, portMAX_DELAY) == pdTRUE) {
        if (r->op == FIO_STOP) {
            xSemaphoreGive(r->done);
            break;
        }
        fio_do(r);
        xSemaphoreGive(r->done);
    }
    vTaskDelete(NULL);
}

/* Dispatch a request to the proxy and block until it completes */
static void fio_call(flash_io_req_t *r)
{
    xQueueSend(s_fio_queue, &r, portMAX_DELAY);
    xSemaphoreTake(r->done, portMAX_DELAY);
}

void mos_flash_io_start(void)
{
    s_fio_queue = xQueueCreate(1, sizeof(flash_io_req_t *));
    configASSERT(s_fio_queue);
    /* 8 KB DRAM stack — enough for FAT + VFS stack frames */
    BaseType_t r = xTaskCreateWithCaps(
        flash_io_task, "flash_io", 8192 / sizeof(StackType_t),
        NULL, tskIDLE_PRIORITY + 6, &s_fio_task,
        MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    configASSERT(r == pdPASS);
}

void mos_flash_io_stop(void)
{
    /* Allocate stop token on our (DRAM) stack — we're called from mos_loader
     * which runs on the mos_main_task (PSRAM stack).  We need a semaphore. */
    SemaphoreHandle_t done = xSemaphoreCreateBinary();
    configASSERT(done);
    flash_io_req_t stop = { .op = FIO_STOP, .done = done };
    flash_io_req_t *p = &stop;
    xQueueSend(s_fio_queue, &p, portMAX_DELAY);
    xSemaphoreTake(done, portMAX_DELAY);
    vSemaphoreDelete(done);
    vQueueDelete(s_fio_queue);
    s_fio_queue = NULL;
    s_fio_task  = NULL;
}

/* ---- Proxy-aware wrappers (replace the direct POSIX calls on P4) ---- */
/* Each wrapper allocates the request on the user_task's PSRAM stack (fine —
 * only the flash_io_task's DRAM stack is active during the actual I/O). */

#define FIO_REQ(op_)    flash_io_req_t _r = {.op=(op_), .done=xSemaphoreCreateBinary()}; \
                        flash_io_req_t *_rp = &_r; configASSERT(_r.done)
#define FIO_CALL()      fio_call(_rp); vSemaphoreDelete(_r.done)

static uint8_t p4_fopen(const char *path, const char *mode)
{
    FIO_REQ(FIO_FOPEN); _r.path = path; _r.mode = mode;
    FIO_CALL(); return _r.ret.u8;
}
static int p4_fclose(uint8_t fh)
{
    FIO_REQ(FIO_FCLOSE); _r.fh = fh;
    FIO_CALL(); return _r.ret.i;
}
static int p4_fgetc(uint8_t fh)
{
    FIO_REQ(FIO_FGETC); _r.fh = fh;
    FIO_CALL(); return _r.ret.i;
}
static int p4_fputc(uint8_t c, uint8_t fh)
{
    FIO_REQ(FIO_FPUTC); _r.fh = fh; _r.c = c;
    FIO_CALL(); return _r.ret.i;
}
static int p4_feof(uint8_t fh)
{
    FIO_REQ(FIO_FEOF); _r.fh = fh;
    FIO_CALL(); return _r.ret.i;
}
static int p4_flseek(uint8_t fh, long offset, int whence)
{
    FIO_REQ(FIO_FLSEEK); _r.fh = fh; _r.offset = offset; _r.whence = whence;
    FIO_CALL(); return _r.ret.i;
}
static size_t p4_fread(void *buf, size_t size, size_t count, uint8_t fh)
{
    FIO_REQ(FIO_FREAD); _r.fh = fh; _r.buf = buf; _r.size = size; _r.count = count;
    FIO_CALL(); return _r.ret.sz;
}
static size_t p4_fwrite(const void *buf, size_t size, size_t count, uint8_t fh)
{
    FIO_REQ(FIO_FWRITE); _r.fh = fh; _r.cbuf = buf; _r.size = size; _r.count = count;
    FIO_CALL(); return _r.ret.sz;
}
static long p4_ftell(uint8_t fh)
{
    FIO_REQ(FIO_FTELL); _r.fh = fh;
    FIO_CALL(); return _r.ret.l;
}
static int p4_load(const char *path, void *addr, size_t *bytes_read)
{
    FIO_REQ(FIO_LOAD); _r.path = path; _r.buf = addr; _r.path2 = (const char *)bytes_read;
    FIO_CALL(); return _r.ret.i;
}
static int p4_save(const char *path, const void *addr, size_t len)
{
    FIO_REQ(FIO_SAVE); _r.path = path; _r.cbuf = addr; _r.size = len;
    FIO_CALL(); return _r.ret.i;
}
static int p4_dir(const char *path)
{
    FIO_REQ(FIO_DIR); _r.path = path;
    FIO_CALL(); return _r.ret.i;
}
static int p4_cd(const char *path)
{
    FIO_REQ(FIO_CD); _r.path = path;
    FIO_CALL(); return _r.ret.i;
}
static int p4_mkdir(const char *path)
{
    FIO_REQ(FIO_MKDIR); _r.path = path;
    FIO_CALL(); return _r.ret.i;
}
static int p4_rename(const char *src, const char *dst)
{
    FIO_REQ(FIO_RENAME); _r.path = src; _r.path2 = dst;
    FIO_CALL(); return _r.ret.i;
}
static int p4_copy(const char *src, const char *dst)
{
    FIO_REQ(FIO_COPY); _r.path = src; _r.path2 = dst;
    FIO_CALL(); return _r.ret.i;
}
static int p4_del(const char *path)
{
    FIO_REQ(FIO_DEL); _r.path = path;
    FIO_CALL(); return _r.ret.i;
}

#else /* !CONFIG_IDF_TARGET_ESP32P4 — no-ops */

void mos_flash_io_start(void) {}
void mos_flash_io_stop(void)  {}

#endif /* CONFIG_IDF_TARGET_ESP32P4 */

/* -------------------------------------------------------------------------
 * File handle table
 * Handle 0 is invalid (reserved).  Handles 1..MOS_MAX_OPEN_FILES map to
 * FILE* entries in s_fh[].
 * s_fh[] is declared above (forward declaration before flash I/O proxy).
 * ------------------------------------------------------------------------- */

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

void mos_api_table_init(void)
{
    t_mos_api *t = &s_mos_api_table;

    t->magic      = MOS_API_MAGIC;
    t->version    = MOS_API_VERSION;

    t->getkey     = mos_api_getkey;
    t->putch      = mos_api_putch;
    t->puts       = mos_api_puts;
    t->editline   = mos_api_editline;

    /* On ESP32-P4 the user task has a PSRAM stack. The IDF flash driver asserts
     * the calling task stack is in DRAM before disabling the cache. Use the
     * proxy wrappers that dispatch to flash_io_task (DRAM stack) instead. */
#if CONFIG_IDF_TARGET_ESP32P4
    t->load       = p4_load;
    t->save       = p4_save;
    t->fopen      = p4_fopen;
    t->fclose     = p4_fclose;
    t->fgetc      = p4_fgetc;
    t->fputc      = p4_fputc;
    t->feof       = p4_feof;
    t->flseek     = p4_flseek;
    t->fread      = p4_fread;
    t->fwrite     = p4_fwrite;
    t->ftell      = p4_ftell;
    t->dir        = p4_dir;
    t->cd         = p4_cd;
    t->mkdir      = p4_mkdir;
    t->rename     = p4_rename;
    t->copy       = p4_copy;
    t->del        = p4_del;
#else
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
#endif

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

    register_vdp_sysvars();
}

void mos_api_set_exit_fn(void (*fn)(int status))
{
    s_mos_api_table.exit = fn;
}
