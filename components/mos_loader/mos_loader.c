/*
 * mos_loader.c - User program loader for ESP32-MOS
 *
 * Loads a flat binary from FAT into a fixed PSRAM exec arena and
 * calls its entry point: int _start(int argc, char **argv, t_mos_api *mos)
 *
 * Supports ESP32-S3 (Xtensa LX7) and ESP32-P4 (RISC-V RV32IMAFC).
 * The binary MUST be compiled with MOS_EXEC_BASE == address of s_exec_arena
 * (read from esp32-mos.map after first build).
 *
 * Constraints:
 *   - PSRAM exec arena aligned to 64 bytes (P4 cache line; safe for S3 too)
 *   - M2C icache invalidation range must be cache-line aligned (64B P4, 32B S3)
 *   - Only one user program can run at a time (single arena)
 */

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <sys/stat.h>
#include "esp_log.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_cache.h"
#include "soc/ext_mem_defs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/idf_additions.h"
#include <setjmp.h>

#include "mos_loader.h"
#include "mos_fs.h"
#include "mos_api.h"
#include "mos_api_table.h"
#include "mos_vdp.h"
#include "mos_vdp_router.h"

static const char *TAG = "mos_loader";

/* Fixed PSRAM exec arena — 512 KB.
 * EXT_RAM_BSS_ATTR places it in .ext_ram.bss → PSRAM.
 *
 * ESP32-S3: data window 0x3Cxxxxxx, instruction window 0x42xxxxxx (separate).
 *           Cache line = 32 bytes.
 * ESP32-P4: unified address space at 0x48xxxxxx (DBUS == IBUS).
 *           Cache line = 64 bytes — must align arena AND sync sizes to 64B.
 *
 * MOS_EXEC_BASE in sdk/Makefile MUST equal the exec alias of s_exec_arena.
 * (check build/esp32-mos.map after first build for s_exec_arena address) */
#define MOS_EXEC_SIZE   (512 * 1024)

#if CONFIG_IDF_TARGET_ESP32P4
#  define MOS_CACHE_LINE  64u
#else
#  define MOS_CACHE_LINE  32u
#endif

static EXT_RAM_BSS_ATTR __attribute__((aligned(64))) uint8_t s_exec_arena[MOS_EXEC_SIZE];

/* Convert a PSRAM data-bus vaddr to its instruction-bus alias.
 * ESP32-S3: DBUS 0x3C000000 → IBUS 0x42000000 (separate windows).
 * ESP32-P4: unified address space — no conversion needed. */
static inline void *dbus_to_ibus(const void *dbus_ptr)
{
#if CONFIG_IDF_TARGET_ESP32P4
    return (void *)dbus_ptr;
#else
    uint32_t offset = (uint32_t)dbus_ptr - SOC_MMU_DBUS_VADDR_BASE;
    return (void *)(SOC_MMU_IBUS_VADDR_BASE + offset);
#endif
}

/* Entry point prototype for user programs */
typedef int (*mos_entry_t)(int argc, char **argv, t_mos_api *mos);

/* Stack size for user program task.
 *
 * ESP32-S3: stack MUST be in internal DRAM — flash driver disables the shared
 * SPI bus (flash+PSRAM share one controller on S3), making PSRAM-backed stacks
 * inaccessible during flash reads → assert in esp_task_stack_is_sane_cache_disabled.
 *
 * ESP32-P4: flash and PSRAM use INDEPENDENT SPI controllers
 * (CONFIG_SOC_MEMSPI_FLASH_PSRAM_INDEPENDENT=y), so disabling the flash cache
 * does NOT affect PSRAM access. PSRAM stacks are safe on P4.
 * Internal DRAM is only ~108KB free after IDF init — 128KB does not fit.
 * Use PSRAM (SPIRAM) for the task stack on P4; keep DRAM on S3.
 *
 * BBC BASIC bbeval/bbexec recurse deeply; 128KB is sufficient. */
#define USER_TASK_STACK_KB   128
#define USER_TASK_STACK_SIZE (USER_TASK_STACK_KB * 1024)

#if CONFIG_IDF_TARGET_ESP32P4
#  define USER_TASK_STACK_CAPS  (MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)
#else
/* ESP32-S3 and others: must use internal DRAM */
#  define USER_TASK_STACK_CAPS  (MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)
#endif

typedef struct {
    mos_entry_t     entry;
    int             argc;
    char          **argv;
    t_mos_api      *mos;
    int             retval;
    SemaphoreHandle_t done;
    jmp_buf         exit_jmp;   /* _exit() longjmps here — stays inside user_task */
    int             exit_status;
} user_task_args_t;

/* Global pointer so mos_exit() can reach the jmp_buf inside user_task.
 * Only one user program runs at a time so a single global is safe. */
static user_task_args_t *s_user_task_args = NULL;

/* Called via mos->exit(status) from user programs.
 * longjmps back into user_task (which is still on the FreeRTOS stack),
 * so vTaskDelete runs normally afterwards.
 * Exposed via mos_loader.h so mos_api.c can register it in the API table. */
void mos_loader_exit_fn(int status)
{
    if (s_user_task_args) {
        s_user_task_args->exit_status = status;
        longjmp(s_user_task_args->exit_jmp, 1);
    }
    /* Fallback if called outside a user task — just spin */
    while (1) { vTaskDelay(1); }
}

static void user_task(void *pvArg)
{
    user_task_args_t *a = (user_task_args_t *)pvArg;
    s_user_task_args = a;

    /* setjmp here so mos->exit() can longjmp back into this frame,
     * which is still inside FreeRTOS's task wrapper — safe to call
     * xSemaphoreGive + vTaskDelete afterwards. */
    if (setjmp(a->exit_jmp) == 0)
        a->retval = a->entry(a->argc, a->argv, a->mos);
    else
        a->retval = a->exit_status;

    s_user_task_args = NULL;
    xSemaphoreGive(a->done);
    vTaskDelete(NULL);
}

int mos_loader_exec(const char *path, int argc, char **argv)
{
    /* 1. Resolve path */
    char resolved[512];
    if (mos_fs_resolve(path, resolved, sizeof(resolved)) != 0) {
        ESP_LOGE(TAG, "Cannot resolve path: %s", path);
        return -1;
    }

    /* 2. Get file size */
    struct stat st;
    if (stat(resolved, &st) != 0) {
        ESP_LOGE(TAG, "Cannot stat '%s': %s", resolved, strerror(errno));
        return -1;
    }
    size_t size = (size_t)st.st_size;
    if (size == 0) {
        ESP_LOGE(TAG, "Empty file: %s", resolved);
        return -1;
    }
    if (size > MOS_EXEC_SIZE) {
        ESP_LOGE(TAG, "'%s' too large: %u > %u", resolved, (unsigned)size, MOS_EXEC_SIZE);
        return -1;
    }

    ESP_LOGD(TAG, "Exec arena @ %p", s_exec_arena);
    ESP_LOGI(TAG, "Loading '%s' (%u bytes)", resolved, (unsigned)size);

    /* 3. Read binary into fixed arena */
    FILE *f = fopen(resolved, "rb");
    if (!f) {
        ESP_LOGE(TAG, "Cannot open '%s': %s", resolved, strerror(errno));
        return -1;
    }
    size_t got = fread(s_exec_arena, 1, size, f);
    fclose(f);
    if (got != size) {
        ESP_LOGE(TAG, "Short read: got %u of %u bytes", (unsigned)got, (unsigned)size);
        return -1;
    }

    /* 3b. Validate magic: byte[0] must be the ISA's jump opcode.
     *     ESP32-S3 (Xtensa): 0x06 (density 'j' opcode)
     *     ESP32-P4 (RISC-V): 0x6F (JAL opcode) */
#if CONFIG_IDF_TARGET_ESP32P4
    if (s_exec_arena[0] != 0x6F) {
        ESP_LOGE(TAG, "'%s': not a valid ESP32-MOS binary (got %02x %02x %02x, "
                 "expected RISC-V JAL opcode 0x6F). "
                 "This may be an Xtensa or eZ80 binary — it cannot run on RISC-V.",
                 resolved,
                 s_exec_arena[0], s_exec_arena[1], s_exec_arena[2]);
        return -1;
    }
#else
    if (s_exec_arena[0] != 0x06) {
        ESP_LOGE(TAG, "'%s': not a valid ESP32-MOS binary (got %02x %02x %02x, "
                 "expected Xtensa 'j' opcode 0x06). "
                 "This may be an eZ80/Agon binary — it cannot run on Xtensa.",
                 resolved,
                 s_exec_arena[0], s_exec_arena[1], s_exec_arena[2]);
        return -1;
    }
#endif

    /* 4. Sync caches so the CPU sees the freshly-loaded binary.
     *
     * fread() writes through the CPU (FAT/WL driver uses memcpy), so the
     * data lands in the dcache as dirty lines — NOT yet in physical PSRAM.
     *
     * Step A: C2M writeback — flush dirty dcache lines to physical PSRAM.
     *         UNALIGNED flag allowed for C2M; covers any size.
     *
     * Step B: M2C invalidate icache — drop stale icache lines so the CPU
     *         fetches fresh code from PSRAM.
     *         ESP32-S3: IBUS = DBUS + 0x6000000. ESP32-P4: same address.
     *         CRITICAL: M2C does NOT accept UNALIGNED — base and size MUST
     *         be aligned to MOS_CACHE_LINE (64B on P4, 32B on S3). */

    /* Step A: C2M — UNALIGNED allowed */
    esp_err_t err = esp_cache_msync(s_exec_arena, size,
                                    ESP_CACHE_MSYNC_FLAG_DIR_C2M |
                                    ESP_CACHE_MSYNC_FLAG_INVALIDATE |
                                    ESP_CACHE_MSYNC_FLAG_TYPE_DATA |
                                    ESP_CACHE_MSYNC_FLAG_UNALIGNED);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_cache_msync (C2M data) failed: %s", esp_err_to_name(err));
        return -1;
    }

    /* Step B: M2C — align base down, end up, to cache-line boundary */
    void    *ibus_arena = dbus_to_ibus(s_exec_arena);
    uint32_t m2c_base   = (uint32_t)ibus_arena & ~(MOS_CACHE_LINE - 1u);
    uint32_t m2c_end    = ((uint32_t)ibus_arena + size + MOS_CACHE_LINE - 1u)
                          & ~(MOS_CACHE_LINE - 1u);
    err = esp_cache_msync((void *)m2c_base, (size_t)(m2c_end - m2c_base),
                          ESP_CACHE_MSYNC_FLAG_DIR_M2C |
                          ESP_CACHE_MSYNC_FLAG_INVALIDATE |
                          ESP_CACHE_MSYNC_FLAG_TYPE_INST);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_cache_msync (M2C inst) failed: %s", esp_err_to_name(err));
        return -1;
    }

    /* 5. Launch user program in a dedicated FreeRTOS task with a large stack
     * allocated from PSRAM.  This avoids overflowing app_main's stack when
     * running deep-recursing interpreters (e.g. BBC BASIC bbeval/bbexec).
     * app_main blocks on a semaphore until the user program returns.
     *
     * On ESP32-P4: flash_io_task was started once at boot in mos_main_task
     * and runs for the entire session — no need to start/stop it here. */

    void *ibus_entry = dbus_to_ibus(s_exec_arena);
    mos_entry_t entry_fn = (mos_entry_t)ibus_entry;
    ESP_LOGI(TAG, "Running '%s'", resolved);

    user_task_args_t args = {
        .entry  = entry_fn,
        .argc   = argc,
        .argv   = argv,
        .mos    = mos_api_table_get(),
        .retval = -1,
        .done   = xSemaphoreCreateBinary(),
    };
    if (!args.done) {
        ESP_LOGE(TAG, "Failed to create semaphore");
        return -1;
    }

    /* Stack allocation: see USER_TASK_STACK_CAPS above for rationale.
     * On ESP32-P4 this is PSRAM (independent bus from flash, but IDF flash
     * driver still asserts DRAM stack — use flash_io proxy for all I/O).
     * On ESP32-S3 this is internal DRAM (shared bus, must avoid PSRAM).
     *
     * xTaskCreateWithCaps allocates TCB + stack internally using the given
     * heap caps, and FreeRTOS owns their lifetime.  We call vTaskDeleteWithCaps
     * after the semaphore to let FreeRTOS free them — never free them ourselves.
     * This avoids the crash where free(tcb) races with prvCheckTasksWaitingTermination
     * in the idle task still dereferencing the TCB after vTaskDelete(NULL). */
    TaskHandle_t task = NULL;
    BaseType_t created = xTaskCreateWithCaps(
        user_task, "user_prog",
        USER_TASK_STACK_SIZE / sizeof(StackType_t),
        &args,
        tskIDLE_PRIORITY + 5,
        &task,
        USER_TASK_STACK_CAPS);
    if (created != pdPASS || !task) {
        ESP_LOGE(TAG, "xTaskCreateWithCaps failed (%u KB %s stack)", USER_TASK_STACK_KB,
                 (USER_TASK_STACK_CAPS & MALLOC_CAP_SPIRAM) ? "PSRAM" : "internal");
        vSemaphoreDelete(args.done);
        return -1;
    }

    /* Block until user program exits.
     * Poll every 200 ms so we can detect VDP disconnection.
     * On disconnect, call mos_vdp_abort() which makes getch() return -1
     * and putch() discard output — the user program will exit naturally
     * on its next I/O call.  We then wait up to 2 s for it to finish.
     * IMPORTANT: never longjmp from this task into user_task's stack. */
    bool aborted = false;
    while (xSemaphoreTake(args.done, pdMS_TO_TICKS(200)) != pdTRUE) {
        if (!aborted && mos_vdp_disconnecting()) {
            ESP_LOGW(TAG, "VDP disconnected — aborting user program");
            mos_vdp_abort();
            aborted = true;
        }
    }
    vSemaphoreDelete(args.done);
    vTaskDeleteWithCaps(task);

    /* Drain the VDP render queue before returning — the user program may have
     * queued VDU bytes via mos->putch() that vdp_render_task hasn't processed
     * yet. Without this, the display never updates if the program exits quickly
     * (e.g. bootlogo) or if the caller immediately launches another program. */
    mos_vdp_router_flush();

    int ret = args.retval;
    ESP_LOGI(TAG, "'%s' exited %d", resolved, ret);
    return ret;
}

/* Return the base address of the exec arena (for shell info / diagnostics) */
uintptr_t mos_loader_exec_base(void)
{
    return (uintptr_t)s_exec_arena;
}
