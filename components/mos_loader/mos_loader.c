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

#include "mos_loader.h"
#include "mos_fs.h"
#include "mos_api.h"
#include "mos_api_table.h"

static const char *TAG = "mos_loader";

/* Fixed PSRAM exec arena — 64 KB, 32-byte aligned (cache line).
 * EXT_RAM_BSS_ATTR places it in .ext_ram.bss → PSRAM at a stable address.
 * MOS_EXEC_BASE in sdk/Makefile MUST match the address the linker assigns
 * to s_exec_arena (check build/esp32-mos.map after first build). */
#define MOS_EXEC_SIZE   (64 * 1024)
static EXT_RAM_BSS_ATTR __attribute__((aligned(32))) uint8_t s_exec_arena[MOS_EXEC_SIZE];

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

    ESP_LOGI(TAG, "Exec arena @ %p (MOS_EXEC_BASE must match)", s_exec_arena);
    ESP_LOGI(TAG, "Loading '%s' (%u bytes) into PSRAM", resolved, (unsigned)size);

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

    /* 4. Flush dcache → PSRAM, then invalidate icache so CPU fetches fresh.
     * Size must be rounded up to cache-line (32 bytes). */
    size_t sync_size = (size + 31) & ~31u;
    esp_err_t err = esp_cache_msync(s_exec_arena, sync_size,
                                    ESP_CACHE_MSYNC_FLAG_DIR_C2M |
                                    ESP_CACHE_MSYNC_FLAG_INVALIDATE |
                                    ESP_CACHE_MSYNC_FLAG_UNALIGNED);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_cache_msync failed: %s", esp_err_to_name(err));
        return -1;
    }

    /* 5. Jump to entry point */
    mos_entry_t entry = (mos_entry_t)(void *)s_exec_arena;
    ESP_LOGI(TAG, "Jumping to entry @ %p", entry);

    int ret = entry(argc, argv, mos_api_table_get());

    ESP_LOGI(TAG, "Program returned %d", ret);
    return ret;
}

/* Return the base address of the exec arena (for shell info / diagnostics) */
uintptr_t mos_loader_exec_base(void)
{
    return (uintptr_t)s_exec_arena;
}
