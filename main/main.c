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
#include "mos_loader.h"
#include "mos_net.h"
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
/* VDU 22 = set screen mode: 2 bytes (0x16, mode_number).
 * VDU 22,0 = 640x480 16-colour mode (defaultPalette10: indices 0..15).
 * The VDP client (agon-sdl) starts in Mode 1 (4 colours), which maps
 * colour indices mod 4, breaking all our cyan/yellow selections.
 * Cannot embed 0x00 in a C string literal (string terminates), so the
 * mode switch is sent via explicit mos_putch calls in print_banner().     */

static void print_banner(void)
{
    /* All status values: green=OK, red=FAIL/error, white=N/A neutral */
    const char *flash_s = mos_fs_flash_mounted() ? COL_GREEN "OK"   COL_BWHITE
                                                  : COL_RED   "FAIL" COL_BWHITE;
    const char *sd_s    = mos_fs_sd_mounted()    ? COL_GREEN "OK"   COL_BWHITE
                                                  : COL_WHITE "N/A"  COL_BWHITE;
    const char *wifi_s  = mos_net_is_connected() ? COL_GREEN "OK"   COL_BWHITE
                                                 : COL_RED   "FAIL" COL_BWHITE;

    /* Switch to 16-colour mode (VDU 22, 0).
     * Cannot embed 0x00 in a string literal so send the two bytes manually.
     * VDP starts in Mode 1 (4 colours); colour indices are taken mod 4,
     * which remaps cyan→yellow and breaks all our colour selections.
     * Mode 0 = 640×480 16-colour; defaultPalette10 maps indices 0–15 as
     * standard CGA: 0=black 1=darkred … 8=darkgray 9=brightred … 14=cyan 15=white */
    mos_putch(0x16); mos_putch(0x00);   /* VDU 22, 0 — set mode 0           */
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

/* ------------------------------------------------------------------ */
/* Main logic — runs in a task with a large PSRAM stack so that        */
/* esp_netif / Ethernet init / vfprintf don't overflow the tiny        */
/* app_main stack on RISC-V (each frame is much larger than Xtensa).  */
/* ------------------------------------------------------------------ */
#define MOS_MAIN_STACK_KB   96

static void mos_main_task(void *arg)
{
    /* Steps 0-2 (NVS + flash FAT) were done in app_main with LP DRAM stack.
     * Any SPI flash access (esp_partition, nvs_flash_init, vfs_fat_spiflash)
     * requires the calling task's stack to be in LP DRAM (passes
     * esp_task_stack_is_sane_cache_disabled). PSRAM stacks fail that check.
     * app_main passes the flash-mount result as (intptr_t)arg. */

    /* 1. Console */
    mos_hal_console_init();

    /* 2. (Done in app_main) — report flash result */
    if ((intptr_t)arg != 0) {
        ESP_LOGE(TAG, "Flash FAT mount failed in app_main");
    }

    /* 3. Mount SD card (optional — SD uses SDMMC, not SPI flash cache path) */
    if (mos_fs_mount_sd() != 0) {
        ESP_LOGW(TAG, "SD card not available");
    }

    /* 4. Network — bring up IP connectivity (WiFi, Ethernet, or none).
     * Backend is selected at compile time via CONFIG_MOS_NET_BACKEND.
     *   WiFi:     retry indefinitely (VDP over TCP needs network).
     *   Ethernet: single attempt with timeout; continue without network
     *             if no cable is plugged in — shell still works locally.
     *   None:     skip entirely. */
    int net_ok = -1;
#if defined(CONFIG_MOS_NET_WIFI)
    {
        uint32_t delay_ms = 15000;
        while (net_ok != 0) {
            mos_printf("Connecting to WiFi...\r\n");
            net_ok = mos_net_init(delay_ms);
            if (net_ok == 0) {
                mos_printf("WiFi OK  IP: %s\r\n", mos_net_ip());
            } else {
                mos_printf("WiFi FAIL - retrying in 30s\r\n");
                delay_ms = 30000;
                vTaskDelay(pdMS_TO_TICKS(5000));
            }
        }
    }
#elif defined(CONFIG_MOS_NET_ETHERNET)
    mos_printf("Starting Ethernet (IP101, RMII)...\r\n");
    net_ok = mos_net_init(CONFIG_MOS_ETH_LINK_TIMEOUT_MS);
    if (net_ok == 0) {
        mos_printf("Ethernet OK  IP: %s\r\n", mos_net_ip());
    } else {
        mos_printf("Ethernet: no link (cable unplugged?) — continuing without network\r\n");
    }
#else
    mos_printf("Network disabled\r\n");
#endif

    /* 5. SNTP — only if we have a network */
#if !defined(CONFIG_MOS_NET_NONE)
    if (net_ok == 0) {
        if (mos_sntp_init(NULL, 10000) == 0) {
            mos_printf("Clock synced via NTP\r\n");
        } else {
            mos_printf("NTP sync timeout - clock not set\r\n");
        }
    }
#endif

    /* 6. VDP TCP server — requires network; skip if not available */
    bool vdp_ok = false;
    if (net_ok == 0) {
        while (!vdp_ok) {
            if (mos_vdp_init() == 0) {
                vdp_ok = true;
                mos_printf("VDP server on port %d - connect to start shell\r\n",
                           MOS_VDP_TCP_PORT);
            } else {
                mos_printf("VDP server failed - retrying in 5s\r\n");
                vTaskDelay(pdMS_TO_TICKS(5000));
            }
        }
    }

    /* 7. MOS API jump table */
    mos_api_table_init();
    mos_api_set_exit_fn(mos_loader_exit_fn);

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

        /* Session ended (VDP disconnected or EOF).
         * Reset shell state so the next session starts with a clean slate:
         * user variables freed, history cleared, cwd back to flash root. */
        mos_shell_reset();
        ESP_LOGI(TAG, "Session reset — %s",
                 vdp_ok ? "waiting for next VDP connection" : "restarting shell");
    }
}

/* ------------------------------------------------------------------ */
/* app_main — runs on the FreeRTOS main task (stack in LP DRAM).       */
/*                                                                      */
/* SPI flash operations (nvs_flash_init, esp_vfs_fat_spiflash_mount)   */
/* assert that the calling task's stack is in LP DRAM via              */
/* esp_task_stack_is_sane_cache_disabled().  PSRAM stacks fail this.   */
/* So we do all flash-touching init HERE, then spawn mos_main_task     */
/* with a large PSRAM stack for the rest (Ethernet, shell, etc.).      */
/* ------------------------------------------------------------------ */
void app_main(void)
{
    /* 0. NVS — must be done before WiFi and before spawning PSRAM task */
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES ||
        nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_err = nvs_flash_init();
    }
    if (nvs_err != ESP_OK) {
        ESP_LOGE(TAG, "NVS init failed: 0x%x", nvs_err);
    }

    /* 1. Mount flash FAT — uses esp_partition / spi_flash (needs DRAM stack) */
    int flash_ok = mos_fs_mount_flash();
    if (flash_ok != 0) {
        ESP_LOGE(TAG, "Flash FAT mount failed");
    }

    /* 2. Spawn mos_main_task with a large PSRAM stack.
     *    TCB must be in internal RAM (LP DRAM); stack can be PSRAM since
     *    no further SPI flash cache ops occur from that task context.    */
    StaticTask_t *tcb   = heap_caps_malloc(sizeof(StaticTask_t),
                                           MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    StackType_t  *stack = heap_caps_malloc(MOS_MAIN_STACK_KB * 1024,
                                           MALLOC_CAP_SPIRAM);

    if (!tcb || !stack) {
        ESP_LOGE(TAG, "PSRAM stack alloc failed — falling back to xTaskCreate");
        xTaskCreate(mos_main_task, "mos_main", MOS_MAIN_STACK_KB * 1024,
                    (void *)(intptr_t)flash_ok, 5, NULL);
    } else {
        xTaskCreateStatic(mos_main_task, "mos_main",
                          MOS_MAIN_STACK_KB * 1024,
                          (void *)(intptr_t)flash_ok,
                          5, stack, tcb);
    }
}
