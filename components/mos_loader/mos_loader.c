/*
 * mos_loader.c - User program loader for ESP32-MOS
 *
 * Loads a flat Xtensa binary from FAT into PSRAM and calls its entry point.
 * The entry point receives (argc, argv, mos_api*) as arguments.
 */

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <sys/stat.h>
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_cache.h"

#include "mos_loader.h"
#include "mos_fs.h"
#include "mos_api.h"
#include "mos_api_table.h"

static const char *TAG = "mos_loader";

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
    ESP_LOGI(TAG, "Loading '%s' (%u bytes) into PSRAM", resolved, (unsigned)size);

    /* 3. Allocate PSRAM */
    void *mem = heap_caps_malloc(size, MALLOC_CAP_SPIRAM);
    if (!mem) {
        ESP_LOGE(TAG, "PSRAM alloc failed (%u bytes) — is PSRAM enabled?", (unsigned)size);
        return -1;
    }

    /* 4. Read binary */
    FILE *f = fopen(resolved, "rb");
    if (!f) {
        ESP_LOGE(TAG, "Cannot open '%s': %s", resolved, strerror(errno));
        heap_caps_free(mem);
        return -1;
    }
    size_t got = fread(mem, 1, size, f);
    fclose(f);
    if (got != size) {
        ESP_LOGE(TAG, "Short read: got %u of %u bytes", (unsigned)got, (unsigned)size);
        heap_caps_free(mem);
        return -1;
    }

    /* 5. Flush icache so the CPU sees fresh data in PSRAM */
    esp_cache_msync(mem, size, ESP_CACHE_MSYNC_FLAG_DIR_C2M |
                                ESP_CACHE_MSYNC_FLAG_INVALIDATE);

    /* 6. Call entry point: int _start(int argc, char **argv, t_mos_api *mos) */
    mos_entry_t entry = (mos_entry_t)mem;
    ESP_LOGI(TAG, "Jumping to entry @ %p", entry);

    int ret = entry(argc, argv, mos_api_table_get());

    ESP_LOGI(TAG, "Program returned %d", ret);

    /* 7. Free PSRAM */
    heap_caps_free(mem);
    return ret;
}
