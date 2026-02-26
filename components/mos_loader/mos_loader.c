/*
 * mos_loader.c - User program loader for ESP32-MOS
 *
 * Loads a flat Xtensa binary from FAT into a fixed PSRAM exec arena and
 * calls its entry point: int _start(int argc, char **argv, t_mos_api *mos)
 *
 * The binary MUST be compiled with MOS_EXEC_BASE == address of s_exec_arena
 * (read from esp32-mos.map after first build).
 *
 * Constraints:
 *   - PSRAM exec requires 32-byte cache-line alignment (ESP32-S3)
 *   - Size must also be rounded up to 32 bytes for esp_cache_msync()
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

#include "mos_loader.h"
#include "mos_fs.h"
#include "mos_api.h"
#include "mos_api_table.h"

static const char *TAG = "mos_loader";

/* Fixed PSRAM exec arena — 64 KB, 32-byte aligned (cache line).
 * EXT_RAM_BSS_ATTR places it in .ext_ram.bss → PSRAM data window (0x3Cxxxxxx).
 * The same physical pages are also accessible via the instruction window
 * (0x42xxxxxx) which the CPU uses to fetch code.
 *
 * MOS_EXEC_BASE in sdk/Makefile MUST equal the IBUS alias of s_exec_arena:
 *   ibus_addr = (dbus_addr - SOC_MMU_DBUS_VADDR_BASE) + SOC_MMU_IBUS_VADDR_BASE
 * (check build/esp32-mos.map after first build for s_exec_arena dbus addr) */
#define MOS_EXEC_SIZE   (512 * 1024)
static EXT_RAM_BSS_ATTR __attribute__((aligned(32))) uint8_t s_exec_arena[MOS_EXEC_SIZE];

/* Convert a PSRAM data-bus vaddr to its instruction-bus alias */
static inline void *dbus_to_ibus(const void *dbus_ptr)
{
    uint32_t offset = (uint32_t)dbus_ptr - SOC_MMU_DBUS_VADDR_BASE;
    return (void *)(SOC_MMU_IBUS_VADDR_BASE + offset);
}

/* Entry point prototype for user programs */
typedef int (*mos_entry_t)(int argc, char **argv, t_mos_api *mos);

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

    /* 3b. Validate magic: byte[0] must be 0x06 (Xtensa density 'j' opcode).
     *     eZ80/Agon binaries start differently — reject early. */
    if (s_exec_arena[0] != 0x06) {
        ESP_LOGE(TAG, "'%s': not a valid ESP32-MOS binary (got %02x %02x %02x, "
                 "expected Xtensa 'j' opcode 0x06). "
                 "This may be an eZ80/Agon binary — it cannot run on Xtensa.",
                 resolved,
                 s_exec_arena[0], s_exec_arena[1], s_exec_arena[2]);
        return -1;
    }

    /* 4. Sync caches so the CPU sees the freshly-loaded binary.
     *
     * fread() writes through the CPU (FAT/WL driver uses memcpy), so the
     * data lands in the dcache as dirty lines — NOT yet in physical PSRAM.
     *
     * Step A: C2M writeback — flush dirty dcache lines to physical PSRAM.
     *         With INVALIDATE, also drop those lines from dcache so future
     *         firmware reads from DBUS (0x3Cxxxxxx) reload from PSRAM.
     *
     * Step B: M2C invalidate icache — drop stale icache lines for the IBUS
     *         window (0x42xxxxxx) so the CPU fetches fresh code from PSRAM.
     *         The IBUS address is the DBUS address + 0x6000000.
     *
     * Size rounded up to cache-line boundary (32 bytes). */
    size_t sync_size = (size + 31) & ~31u;

    /* Step A: write back dcache → PSRAM, then invalidate dcache lines */
    esp_err_t err = esp_cache_msync(s_exec_arena, sync_size,
                                    ESP_CACHE_MSYNC_FLAG_DIR_C2M |
                                    ESP_CACHE_MSYNC_FLAG_INVALIDATE |
                                    ESP_CACHE_MSYNC_FLAG_TYPE_DATA |
                                    ESP_CACHE_MSYNC_FLAG_UNALIGNED);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_cache_msync (C2M data) failed: %s", esp_err_to_name(err));
        return -1;
    }

    /* Step B: invalidate icache for IBUS window so instruction fetches reload */
    void *ibus_arena = dbus_to_ibus(s_exec_arena);
    err = esp_cache_msync(ibus_arena, sync_size,
                          ESP_CACHE_MSYNC_FLAG_DIR_M2C |
                          ESP_CACHE_MSYNC_FLAG_INVALIDATE |
                          ESP_CACHE_MSYNC_FLAG_TYPE_INST);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_cache_msync (M2C inst) failed: %s", esp_err_to_name(err));
        return -1;
    }

    /* 5. Jump via instruction-bus alias (0x42xxxxxx).
     * s_exec_arena is in the data window (0x3Cxxxxxx); the same physical
     * PSRAM pages are also mapped at the instruction window (0x42xxxxxx).
     * The CPU can only fetch code from the IBUS range. */
    void *ibus_entry = dbus_to_ibus(s_exec_arena);
    mos_entry_t entry = (mos_entry_t)ibus_entry;
    ESP_LOGI(TAG, "Running '%s'", resolved);

    int ret = entry(argc, argv, mos_api_table_get());

    ESP_LOGI(TAG, "'%s' exited %d", resolved, ret);
    return ret;
}

/* Return the base address of the exec arena (for shell info / diagnostics) */
uintptr_t mos_loader_exec_base(void)
{
    return (uintptr_t)s_exec_arena;
}
