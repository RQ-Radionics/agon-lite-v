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
#include "mos_api.h"
#include "mos_wifi.h"
#include "mos_sntp.h"
#include "mos_vdp.h"

static const char *TAG = "esp32-mos";

/*
 * VDU colour helpers — verified against agon-vdp/video/vdu.h
 *
 * VDU 17 (0x11) + n:
 *   n  0..63  = set text foreground (0=black 1=red 2=green 3=yellow
 *               4=blue 5=magenta 6=cyan 7=white 8..15=bright versions)
 *   n 128..191 = set text background (128=black, 129=red, ... 135=white)
 *
 * VDU 12 (0x0C) = CLS (clear screen)
 * VDU 30 (0x1E) = cursor home (top-left)
 */
#define VDU_FG(n)    "\x11" "\x0" #n   /* won't work for hex — use raw strings */

/* Foreground colours (n < 64) */
#define COL_RED      "\x11\x01"
#define COL_GREEN    "\x11\x02"
#define COL_YELLOW   "\x11\x03"
#define COL_CYAN     "\x11\x06"
#define COL_WHITE    "\x11\x07"
#define COL_BBLACK   "\x11\x08"
#define COL_BYELLOW  "\x11\x0b"
#define COL_BCYAN    "\x11\x0e"
#define COL_BWHITE   "\x11\x0f"

/* Background colours (n >= 128) */
#define BG_BLACK     "\x11\x80"
#define BG_BLUE      "\x11\x84"
#define BG_CYAN      "\x11\x86"

#define VDU_CLS      "\x0c"            /* VDU 12 — clear screen             */
#define VDU_HOME     "\x1e"            /* VDU 30 — cursor home              */

static void print_banner(void)
{
    /* All status values: green=OK, red=FAIL/error, white=N/A neutral */
    const char *flash_s = mos_fs_flash_mounted() ? COL_GREEN "OK"   COL_BWHITE
                                                  : COL_RED   "FAIL" COL_BWHITE;
    const char *sd_s    = mos_fs_sd_mounted()    ? COL_GREEN "OK"   COL_BWHITE
                                                  : COL_WHITE "N/A"  COL_BWHITE;
    const char *wifi_s  = mos_wifi_is_connected()? COL_GREEN "OK"   COL_BWHITE
                                                  : COL_RED   "FAIL" COL_BWHITE;

    /* Reset colours, clear screen, cursor home */
    mos_puts(BG_BLACK COL_BWHITE VDU_CLS VDU_HOME);

    /* Border = 34 visible chars: "  ********************************"          */
    /* Line 2:  "  *  ESP32-MOS v3.0.1           *"  (11 spaces padding)       */
    /* Line 3:  "  *  ESP32-S3  \"Arthur\"         *"  (9 spaces padding)      */
    mos_puts(COL_BCYAN
             "  ********************************\r\n"
             "  *  " COL_BYELLOW "ESP32-MOS v" VERSION_STRING
             "           " COL_BCYAN "*\r\n"
             "  *  " COL_BWHITE  VERSION_VARIANT "  \"" VERSION_SUBTITLE "\""
             "          " COL_BCYAN "*\r\n"
             "  ********************************" COL_BWHITE "\r\n"
             "\r\n");

    mos_printf("  Flash: %s   SD: %s   WiFi: %s\r\n", flash_s, sd_s, wifi_s);
    mos_printf("  VDP port: " COL_GREEN "%d" COL_BWHITE "\r\n", MOS_VDP_TCP_PORT);
    mos_printf("\r\n");
    mos_printf(COL_YELLOW "  Type HELP for commands." COL_BWHITE "\r\n");
    mos_printf("\r\n");
}

void app_main(void)
{
    /* 0. NVS - must be initialised before WiFi */
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES ||
        nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_err = nvs_flash_init();
    }
    if (nvs_err != ESP_OK) {
        ESP_LOGE(TAG, "NVS init failed: 0x%x", nvs_err);
    }

    /* 1. Console */
    mos_hal_console_init();

    /* 2. Mount flash FAT */
    if (mos_fs_mount_flash() != 0) {
        ESP_LOGE(TAG, "Fatal: cannot mount flash FAT");
    }

    /* 3. Mount SD card (optional) */
    if (mos_fs_mount_sd() != 0) {
        ESP_LOGW(TAG, "SD card not available");
    }

    /* 4. WiFi - wait up to 15 s for IP */
    mos_printf("Connecting to WiFi...\r\n");
    int wifi_ok = mos_wifi_init(15000);
    if (wifi_ok == 0) {
        mos_printf("WiFi OK  IP: %s\r\n", mos_wifi_ip());
    } else {
        mos_printf("WiFi FAIL - network features unavailable\r\n");
    }

    /* 5. SNTP - only if WiFi is up; wait up to 10 s */
    if (wifi_ok == 0) {
        if (mos_sntp_init(NULL, 10000) == 0) {
            mos_printf("Clock synced via NTP\r\n");
        } else {
            mos_printf("NTP sync timeout - clock not set\r\n");
        }
    }

    /* 6. VDP TCP server */
    bool vdp_ok = false;
    if (wifi_ok == 0) {
        if (mos_vdp_init() == 0) {
            vdp_ok = true;
            mos_printf("VDP server on port %d - connect to start shell\r\n",
                       MOS_VDP_TCP_PORT);
        } else {
            mos_printf("VDP server failed to start\r\n");
        }
    }

    /* 7. MOS API jump table */
    mos_api_table_init();

    /* 8. Shell init */
    mos_shell_init();

    /* 9. Session loop - each VDP connection is one session */
    while (1) {
        if (vdp_ok) {
            ESP_LOGI(TAG, "Waiting for VDP client on port %d...", MOS_VDP_TCP_PORT);
            while (!mos_vdp_connected()) {
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            /* Give the VDP a moment to finish handshake before sending VDU codes */
            vTaskDelay(pdMS_TO_TICKS(200));
            ESP_LOGI(TAG, "VDP client connected - starting shell session");
        }

        /* Welcome banner (includes CLS + colour reset) */
        print_banner();

        /* Run autoexec if present */
        char autoexec[64];
        snprintf(autoexec, sizeof(autoexec), "%s/autoexec.bat", MOS_FLASH_MOUNT);
        FILE *f = fopen(autoexec, "r");
        if (f) {
            fclose(f);
            ESP_LOGI(TAG, "Running %s", autoexec);
            mos_shell_exec("EXEC " MOS_FLASH_MOUNT "/autoexec.bat");
        }

        /* Interactive shell - returns when VDP disconnects */
        mos_shell_run();

        if (!vdp_ok) break;  /* safety */

        ESP_LOGI(TAG, "VDP client disconnected - waiting for next connection");
    }
}
