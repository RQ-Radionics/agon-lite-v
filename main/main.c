/*
 * main.c - ESP32-MOS entry point
 *
 * Replaces the eZ80 main.c from agon-mos.
 * No VDP sync, no RST vectors, no ASM startup.
 * The ESP-IDF FreeRTOS runtime handles all low-level initialisation.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "mos_hal.h"
#include "mos_fs.h"
#include "mos_sysvars.h"
#include "mos_shell.h"

static const char *TAG = "esp32-mos";

void app_main(void)
{
    /* 1. Console */
    mos_hal_console_init();

    /* 2. Mount flash FAT (always first) */
    if (mos_fs_mount_flash() != 0) {
        ESP_LOGE(TAG, "Fatal: cannot mount flash FAT");
        /* Continue anyway — maybe SD is available */
    }

    /* 3. Mount SD card (optional — no card is not fatal) */
    if (mos_fs_mount_sd() != 0) {
        ESP_LOGW(TAG, "SD card not available");
    }

    /* 4. Shell init (variables, history) */
    mos_shell_init();

    /* 5. Welcome banner */
    mos_printf("\n\n");
    mos_printf("ESP32-MOS v" VERSION_STRING " " VERSION_VARIANT " \"" VERSION_SUBTITLE "\"\n");
    mos_printf("Flash: %s  SD: %s\n",
               mos_fs_flash_mounted() ? "OK" : "FAIL",
               mos_fs_sd_mounted()    ? "OK" : "N/A");
    mos_printf("Type HELP for commands.\n\n");

    /* 6. Run autoexec if present */
    char autoexec[64];
    snprintf(autoexec, sizeof(autoexec), "%s/autoexec.obey", MOS_FLASH_MOUNT);
    FILE *f = fopen(autoexec, "r");
    if (f) {
        fclose(f);
        ESP_LOGI(TAG, "Running %s", autoexec);
        mos_shell_exec("EXEC " MOS_FLASH_MOUNT "/autoexec.obey");
    }

    /* 7. Interactive shell loop (never returns) */
    mos_shell_run();
}
