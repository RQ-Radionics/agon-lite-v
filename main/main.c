/*
 * main.c - ESP32-MOS entry point
 *
 * Boot sequence:
 *   1. Console (UART0)
 *   2. Flash FAT mount
 *   3. SD card mount (optional)
 *   4. WiFi connect
 *   5. SNTP time sync (non-fatal if it times out)
 *   6. VDP TCP server start
 *   7. Shell init + welcome banner
 *   8. Autoexec
 *   9. Interactive shell loop (never returns)
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "mos_hal.h"
#include "mos_fs.h"
#include "mos_sysvars.h"
#include "mos_shell.h"
#include "mos_wifi.h"
#include "mos_sntp.h"
#include "mos_vdp.h"

static const char *TAG = "esp32-mos";

void app_main(void)
{
    /* 0. NVS — must be initialised before WiFi (and before FAT on some builds).
     *    Handle the two cases that require an erase: partition full or version
     *    mismatch after a firmware update. */
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES ||
        nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS needs erase (err 0x%x), erasing…", nvs_err);
        nvs_flash_erase();
        nvs_err = nvs_flash_init();
    }
    if (nvs_err != ESP_OK) {
        ESP_LOGE(TAG, "NVS init failed: 0x%x — WiFi will not work", nvs_err);
    }

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

    /* 4. WiFi — wait up to 15 s for IP */
    mos_printf("Connecting to WiFi…\n");
    int wifi_ok = mos_wifi_init(15000);
    if (wifi_ok == 0) {
        mos_printf("WiFi OK  IP: %s\n", mos_wifi_ip());
    } else {
        mos_printf("WiFi FAIL — network features unavailable\n");
    }

    /* 5. SNTP — only if WiFi is up; wait up to 10 s */
    if (wifi_ok == 0) {
        if (mos_sntp_init(NULL, 10000) == 0) {
            mos_printf("Clock synced via NTP\n");
        } else {
            mos_printf("NTP sync timeout — clock not set\n");
        }
    }

    /* 6. VDP TCP server */
    if (wifi_ok == 0) {
        if (mos_vdp_init() == 0) {
            mos_printf("VDP server listening on port %d\n", MOS_VDP_TCP_PORT);
        } else {
            mos_printf("VDP server failed to start\n");
        }
    }

    /* 7. Shell init (variables, history) */
    mos_shell_init();

    /* 8. Welcome banner */
    mos_printf("\n");
    mos_printf("ESP32-MOS v" VERSION_STRING " " VERSION_VARIANT " \"" VERSION_SUBTITLE "\"\n");
    mos_printf("Flash: %s  SD: %s  WiFi: %s  VDP: port %d\n",
               mos_fs_flash_mounted() ? "OK"   : "FAIL",
               mos_fs_sd_mounted()    ? "OK"   : "N/A",
               mos_wifi_is_connected()? "OK"   : "FAIL",
               MOS_VDP_TCP_PORT);
    mos_printf("Type HELP for commands.\n\n");

    /* 9. Run autoexec if present */
    char autoexec[64];
    snprintf(autoexec, sizeof(autoexec), "%s/autoexec.obey", MOS_FLASH_MOUNT);
    FILE *f = fopen(autoexec, "r");
    if (f) {
        fclose(f);
        ESP_LOGI(TAG, "Running %s", autoexec);
        mos_shell_exec("EXEC " MOS_FLASH_MOUNT "/autoexec.obey");
    }

    /* 10. Interactive shell loop (never returns) */
    mos_shell_run();
}
