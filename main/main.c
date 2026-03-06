/*
 * main.c - ESP32-MOS entry point
 *
 * Boot sequence:
 *   1. NVS init (app_main, DRAM stack)
 *   2. Spawn mos_main_task (PSRAM stack)
 *   3. Console init
 *   4. HDMI / audio / keyboard init
 *   5. SD card mount (required)
 *   6. Network init (optional)
 *   7. SNTP time sync (non-fatal)
 *   8. VDP TCP server start
 *   9. Shell init + welcome banner + autoexec
 *  10. Interactive shell loop (never returns)
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
#include "mos_vdp_router.h"

#ifdef CONFIG_LT8912B_ENABLED
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_lt8912b.h"
#include "esp_ldo_regulator.h"
#include "esp_cache.h"
#include "driver/i2c_master.h"
#endif

#ifdef CONFIG_MOS_AUDIO_ENABLED
#include "mos_audio.h"
#endif

#ifdef CONFIG_MOS_KBD_ENABLED
#include "mos_kbd.h"
#endif

#ifdef CONFIG_MOS_VDP_INTERNAL_ENABLED
#include "mos_vdp_internal.h"
#include "mos_audio_synth.h"
#endif
#include "mos_ftp.h"

static const char *TAG = "esp32-mos";

/* ------------------------------------------------------------------ */
/* Keyboard scancode stub (fallback when internal VDP is disabled)     */
/* ------------------------------------------------------------------ */
#if defined(CONFIG_MOS_KBD_ENABLED) && !defined(CONFIG_MOS_VDP_INTERNAL_ENABLED)
static void mos_kbd_scancode_stub(uint8_t byte)
{
    ESP_LOGD(TAG, "KBD scancode: 0x%02X", byte);
}
#endif

/* ------------------------------------------------------------------ */
/* HDMI output via LT8912B (Olimex ESP32-P4-PC only)                   */
/*                                                                      */
/* Returns the DPI panel handle so callers can access the framebuffer  */
/* via esp_lcd_dpi_panel_get_frame_buffer().  Returns NULL on failure. */
/*                                                                      */
/* The shared I2C bus (ES8311 + LT8912B live on the same bus) is kept  */
/* in s_shared_i2c_bus so that mos_audio can call                      */
/* mos_audio_init_with_bus(s_shared_i2c_bus) after HDMI init.          */
/* ------------------------------------------------------------------ */
#ifdef CONFIG_LT8912B_ENABLED
static i2c_master_bus_handle_t s_shared_i2c_bus = NULL;

static esp_lcd_panel_handle_t hdmi_init(void)
{
    /* 0. Power on MIPI DSI PHY via internal LDO regulator (LDO_VO3 → 2500 mV).
     *    The DSI PHY requires VDD_MIPI_DPHY = 2.5V to start up.  Without this
     *    the PHY PLL never locks and esp_lcd_new_dsi_bus() hangs indefinitely.
     *    The LDO handle is kept alive for the lifetime of the application. */
    {
        esp_ldo_channel_handle_t ldo_dphy = NULL;
        esp_ldo_channel_config_t ldo_cfg = {
            .chan_id    = 3,      /* LDO_VO3 → VDD_MIPI_DPHY on ESP32-P4 devkits */
            .voltage_mv = 2500,
        };
        esp_err_t ldo_ret = esp_ldo_acquire_channel(&ldo_cfg, &ldo_dphy);
        if (ldo_ret != ESP_OK) {
            ESP_LOGW(TAG, "HDMI: LDO3 acquire failed (0x%x) — PHY power may come from external supply", ldo_ret);
        } else {
            ESP_LOGI(TAG, "HDMI: MIPI DPHY powered via LDO3 @ 2500mV");
        }
    }

    /* 1. Create MIPI DSI bus (2-lane, 350 Mbps, XTAL PLL source) */
    ESP_LOGI(TAG, "HDMI: step 1 — DSI bus init");
    esp_lcd_dsi_bus_handle_t dsi_bus = NULL;
    esp_lcd_dsi_bus_config_t bus_cfg = {
        .bus_id             = 0,
        .num_data_lanes     = CONFIG_LT8912B_DSI_LANES,
        .phy_clk_src        = MIPI_DSI_PHY_CLK_SRC_DEFAULT,
        .lane_bit_rate_mbps = CONFIG_LT8912B_DSI_LANE_MBPS,
    };
    if (esp_lcd_new_dsi_bus(&bus_cfg, &dsi_bus) != ESP_OK) {
        ESP_LOGE(TAG, "HDMI: DSI bus init failed");
        return NULL;
    }
    ESP_LOGI(TAG, "HDMI: step 1 OK");

    /* 2. Patch: disable EoTP — LT8912B cannot handle it */
    ESP_LOGI(TAG, "HDMI: step 2 — EoTP patch");
    if (esp_lcd_lt8912b_patch_dsi_eotp(dsi_bus) != ESP_OK) {
        ESP_LOGE(TAG, "HDMI: EoTP patch failed");
        esp_lcd_del_dsi_bus(dsi_bus);
        return NULL;
    }
    ESP_LOGI(TAG, "HDMI: step 2 OK");

    /* 3. Create the shared I2C bus (ES8311 audio codec and LT8912B both live
     *    on this bus — GPIO7/SDA, GPIO8/SCL, 400 kHz, I2C port 1).
     *    Creating it here, before any driver touches I2C, guarantees a clean
     *    FSM state.  Both esp_lcd_lt8912b_init_with_bus() and
     *    mos_audio_init_with_bus() add their devices to this same handle
     *    without fighting over bus ownership. */
    ESP_LOGI(TAG, "HDMI: step 3 — shared I2C bus + LT8912B init");
    {
        i2c_master_bus_config_t bus_cfg_i2c = {
            .i2c_port            = CONFIG_LT8912B_I2C_PORT,
            .sda_io_num          = CONFIG_LT8912B_SDA_GPIO,
            .scl_io_num          = CONFIG_LT8912B_SCL_GPIO,
            .clk_source          = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt   = 7,
            .flags.enable_internal_pullup = true,
        };
        if (i2c_new_master_bus(&bus_cfg_i2c, &s_shared_i2c_bus) != ESP_OK) {
            ESP_LOGE(TAG, "HDMI: shared I2C bus creation failed");
            esp_lcd_del_dsi_bus(dsi_bus);
            return NULL;
        }

        /* Scan the bus to verify devices are present (diagnostic only) */
        ESP_LOGI(TAG, "HDMI: I2C%d scan (SDA=%d SCL=%d):",
                 CONFIG_LT8912B_I2C_PORT,
                 CONFIG_LT8912B_SDA_GPIO, CONFIG_LT8912B_SCL_GPIO);
        for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
            if (i2c_master_probe(s_shared_i2c_bus, addr, 20) == ESP_OK) {
                ESP_LOGI(TAG, "  I2C device found at 0x%02X", addr);
            }
        }
    }

    /* Initialize LT8912B using the shared bus handle (does NOT own the bus) */
    esp_lcd_lt8912b_config_t lt_cfg = {
        .hpd_gpio  = CONFIG_LT8912B_HPD_GPIO,
        .hdmi_mode = CONFIG_LT8912B_HDMI_MODE,
        /* i2c_port / sda_gpio / scl_gpio ignored by init_with_bus */
    };
    if (esp_lcd_lt8912b_init_with_bus(s_shared_i2c_bus, &lt_cfg) != ESP_OK) {
        ESP_LOGE(TAG, "HDMI: LT8912B I2C init failed");
        i2c_del_master_bus(s_shared_i2c_bus);
        s_shared_i2c_bus = NULL;
        esp_lcd_del_dsi_bus(dsi_bus);
        return NULL;
    }
    ESP_LOGI(TAG, "HDMI: step 3 OK");

    /* 4. Create DPI panel — feeds pixel data from ESP32-P4 to LT8912B DSI input.
     * 800x600 @ 40 MHz pixel clock, RGB888 (24-bit).
     *    VESA 800x600@60Hz timings:
     *      htotal=1056 (hfp=40 hs=128 hbp=88), vtotal=628 (vfp=1 vs=4 vbp=23)
     *      h_polarity=positive, v_polarity=positive
     *    num_fbs=2: double-buffered so mos_vdp_internal can render to back buffer
     *               while the DMA controller reads the front buffer.
     *    disable_lp=1: stay in HS mode during blanking (required for video mode). */
    esp_lcd_panel_handle_t panel = NULL;
    esp_lcd_dpi_panel_config_t dpi_cfg = {
        .dpi_clk_src         = MIPI_DSI_DPI_CLK_SRC_DEFAULT,
        .dpi_clock_freq_mhz  = 40,          /* 40 MHz for 800x600@60Hz */
        .pixel_format        = LCD_COLOR_PIXEL_FORMAT_RGB888,
        .num_fbs             = 2,           /* double-buffered for tear-free rendering */
        .video_timing = {
            .h_size          = 800,
            .v_size          = 600,
            .hsync_pulse_width = 128,
            .hsync_back_porch  = 88,
            .hsync_front_porch = 40,
            .vsync_pulse_width = 4,
            .vsync_back_porch  = 23,
            .vsync_front_porch = 1,
        },
        .flags.disable_lp    = 1,
    };
    ESP_LOGI(TAG, "HDMI: step 4 — DPI panel create");
    if (esp_lcd_new_panel_dpi(dsi_bus, &dpi_cfg, &panel) != ESP_OK) {
        ESP_LOGE(TAG, "HDMI: DPI panel init failed");
        esp_lcd_lt8912b_deinit();
        esp_lcd_del_dsi_bus(dsi_bus);
        return NULL;
    }
    ESP_LOGI(TAG, "HDMI: step 4 OK");

    /* 5. Clear framebuffers to black BEFORE enabling the panel, so the DPI
     *    DMA never reads uninitialised PSRAM content and the screen starts black.
     *    The IDF allocates the framebuffers with calloc (heap_caps_calloc) so they
     *    start as zero in the cache, but the DMA reads physical PSRAM which may
     *    still contain stale data.  Flush cache → PSRAM before panel enable. */
    ESP_LOGI(TAG, "HDMI: step 5 — clear framebuffers + panel enable");
    {
        void *fb0 = NULL, *fb1 = NULL;
        if (esp_lcd_dpi_panel_get_frame_buffer(panel, 2, &fb0, &fb1) == ESP_OK) {
            size_t fb_size = 800 * 600 * 3;   /* RGB888 */
            /* Paint fb0 red (R=0xFF G=0x00 B=0x00) as a DMA sanity check.
             * If the screen shows red before the shell paints anything,
             * the DPI→LT8912B→HDMI pipeline is working correctly.
             * Remove this once display output is confirmed. */
            if (fb0) {
                memset(fb0, 0, fb_size);
                esp_cache_msync(fb0, fb_size,
                    ESP_CACHE_MSYNC_FLAG_DIR_C2M |
                    ESP_CACHE_MSYNC_FLAG_TYPE_DATA |
                    ESP_CACHE_MSYNC_FLAG_UNALIGNED);
            }
            if (fb1) {
                memset(fb1, 0, fb_size);
                esp_cache_msync(fb1, fb_size,
                    ESP_CACHE_MSYNC_FLAG_DIR_C2M |
                    ESP_CACHE_MSYNC_FLAG_TYPE_DATA |
                    ESP_CACHE_MSYNC_FLAG_UNALIGNED);
            }
        }
    }
    esp_lcd_panel_init(panel);
    esp_lcd_panel_disp_on_off(panel, true);
    ESP_LOGI(TAG, "HDMI: step 5 OK");

    ESP_LOGI(TAG, "HDMI: 800x600@60Hz ready%s",
             esp_lcd_lt8912b_is_connected() ? " (cable connected)" : " (no cable)");
    return panel;
}
#endif /* CONFIG_LT8912B_ENABLED */

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
    /* All status values: green=OK, red=FAIL/error */
    const char *sd_s    = mos_fs_sd_mounted()    ? COL_GREEN "OK"   COL_BWHITE
                                                  : COL_RED   "FAIL" COL_BWHITE;
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

    mos_printf("  SD: %s   WiFi: %s\r\n", sd_s, wifi_s);
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
/* 192 KB: mos_main_task runs esp_vfs_usb_serial_jtag_use_driver() +
 * Ethernet init (esp_netif_init, esp_eth_driver_install) which together
 * consume ~80-100 KB on RISC-V due to large ABI call frames and deep
 * newlib/lwIP stacks.  96 KB was confirmed insufficient (SP underflowed
 * by 36 bytes at mos_hal_console_init() entry).                         */
#define MOS_MAIN_STACK_KB   192

static void mos_main_task(void *arg)
{
    /* NVS was inited in app_main (requires LP DRAM stack).
     * SD card mount is done here — SDMMC does NOT go through the SPI flash
     * cache path and has no stack-location restriction.  PSRAM stacks are fine.
     * app_main passes 0 always (arg unused). */

    /* 1. Console */
    mos_hal_console_init();

    /* Brief delay so USB-JTAG CDC has time to connect before any potential
     * crash — otherwise the panic backtrace is lost before it can be sent. */
    vTaskDelay(pdMS_TO_TICKS(2000));

    /* 1b. HDMI output (Olimex ESP32-P4-PC only).
     *     Must come BEFORE audio: hdmi_init() creates the shared I2C bus and
     *     exercises it with LT8912B transactions, leaving the bus in a known
     *     good state. The production test does the same — I2C is initialised
     *     inside pt_display_init() long before audio_loopback_test runs. */
#ifdef CONFIG_LT8912B_ENABLED
    esp_lcd_panel_handle_t dpi_panel = hdmi_init();
#else
    esp_lcd_panel_handle_t dpi_panel = NULL;
#endif

    /* 1b2. Internal VDP — framebuffer renderer over HDMI (Olimex only). */
#ifdef CONFIG_MOS_VDP_INTERNAL_ENABLED
    {
        esp_err_t vdp_ret = mos_vdp_internal_init(dpi_panel);
        if (vdp_ret != ESP_OK) {
            ESP_LOGE(TAG, "Internal VDP init failed (0x%x %s) — no HDMI output",
                     vdp_ret, esp_err_to_name(vdp_ret));
        } else {
            ESP_LOGI(TAG, "Internal VDP init OK");
        }
    }
#endif

    /* 1c. Audio codec (ES8311) — after HDMI so the shared I2C bus is already
     *     exercised by LT8912B transactions (matches production test order). */
#ifdef CONFIG_MOS_AUDIO_ENABLED
#  ifdef CONFIG_LT8912B_ENABLED
    if (mos_audio_init_with_bus(s_shared_i2c_bus) != ESP_OK) {
#  else
    if (mos_audio_init() != ESP_OK) {
#  endif
        ESP_LOGW(TAG, "Audio init failed — continuing without audio");
    }
#endif

    /* 1c2. Audio synthesizer — Agon 3-channel VDP audio engine.
     *      Must be called after mos_audio_init() so the hardware is ready.
     *      The synth task will sleep gracefully if mos_audio is unavailable. */
#ifdef CONFIG_MOS_VDP_INTERNAL_ENABLED
    {
        esp_err_t synth_ret = mos_synth_init();
        if (synth_ret != ESP_OK) {
            ESP_LOGW(TAG, "Audio synth init failed (0x%x) — VDP audio disabled", synth_ret);
        }
    }
#endif

    /* 1d. USB HID keyboard (Olimex ESP32-P4-PC only) — PHY1 GPIO26/27, hub GPIO21.
     *     Produces PS/2 Set 2 scancodes delivered via callback.
     *     When the internal VDP is enabled, scancodes go directly to it.
     *     Non-fatal: if keyboard init fails, rest of system continues normally. */
#ifdef CONFIG_MOS_KBD_ENABLED
#  ifdef CONFIG_MOS_VDP_INTERNAL_ENABLED
    if (mos_kbd_init(mos_vdp_internal_send_scancode) != ESP_OK) {
#  else
    if (mos_kbd_init(mos_kbd_scancode_stub) != ESP_OK) {
#  endif
        ESP_LOGW(TAG, "Keyboard init failed — continuing without keyboard");
    }
#endif

    /* 2. Mount SD card — only storage volume.
     *    SDMMC uses its own bus; no SPI flash cache interference.
     *    Log a warning but continue — shell can still run (will show FAIL in banner). */
    if (mos_fs_mount_sd() != 0) {
        ESP_LOGW(TAG, "SD card not available — file operations will fail");
    }

    /* 4. Network — bring up IP connectivity (WiFi, Ethernet, or none).
     *
     * When the internal VDP is enabled (CONFIG_MOS_VDP_INTERNAL_ENABLED),
     * network is OPTIONAL: the shell starts on the HDMI display regardless.
     * Network failure is logged and skipped; the TCP VDP server is still
     * started so agon-sdl can connect later if a cable is plugged in.
     *
     * When TCP-only mode is used, network is REQUIRED: we loop until up.
     */
    int net_ok = -1;
#if defined(CONFIG_MOS_NET_WIFI)
    {
#ifdef CONFIG_MOS_VDP_INTERNAL_ENABLED
        /* With internal VDP: single attempt, non-blocking on failure */
        mos_printf("Connecting to WiFi...\r\n");
        net_ok = mos_net_init(15000);
        if (net_ok == 0) {
            mos_printf("WiFi OK  IP: %s\r\n", mos_net_ip());
        } else {
            mos_printf("WiFi FAIL - continuing without network\r\n");
        }
#else
        /* TCP-only: retry indefinitely */
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
#endif
    }
#elif defined(CONFIG_MOS_NET_ETHERNET)
#ifdef CONFIG_MOS_VDP_INTERNAL_ENABLED
    /* With internal VDP: single attempt, non-blocking on failure */
    mos_printf("Starting Ethernet (IP101, RMII)...\r\n");
    net_ok = mos_net_init(CONFIG_MOS_ETH_LINK_TIMEOUT_MS);
    if (net_ok == 0) {
        mos_printf("Ethernet OK  IP: %s\r\n", mos_net_ip());
    } else {
        mos_printf("Ethernet: no link — continuing without network\r\n");
    }
#else
    /* TCP-only: retry until link is up */
    while (net_ok != 0) {
        mos_printf("Starting Ethernet (IP101, RMII)...\r\n");
        net_ok = mos_net_init(CONFIG_MOS_ETH_LINK_TIMEOUT_MS);
        if (net_ok == 0) {
            mos_printf("Ethernet OK  IP: %s\r\n", mos_net_ip());
        } else {
            mos_printf("Ethernet: no link — retrying in 10s (plug cable?)\r\n");
            vTaskDelay(pdMS_TO_TICKS(10000));
        }
    }
#endif
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

    /* 6. FTP server — start if network is available */
#if !defined(CONFIG_MOS_NET_NONE)
    if (net_ok == 0) {
        if (mos_ftp_init() == 0) {
            mos_printf("FTP server on port 21 (root: /sdcard)\r\n");
        }
    }
#endif

    /* 7. VDP TCP server.
     * socket()+bind(INADDR_ANY)+listen() work even without an IP address,
     * so mos_vdp_init() is always called.  With no network, accept() will
     * never return a client and mos_vdp_connected() stays false — correct.
     * Without internal VDP, the loop below retries on failure (shouldn't
     * happen, but belt-and-suspenders). */
#ifdef CONFIG_MOS_VDP_INTERNAL_ENABLED
    if (mos_vdp_init() == 0) {
        if (net_ok == 0) {
            mos_printf("VDP server on port %d\r\n", MOS_VDP_TCP_PORT);
        }
    }
#else
    while (mos_vdp_init() != 0) {
        mos_printf("VDP server failed - retrying in 5s\r\n");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    mos_printf("VDP server on port %d - connect to start shell\r\n", MOS_VDP_TCP_PORT);
#endif

    /* 7. MOS API jump table */
    mos_api_table_init();
    mos_api_set_exit_fn(mos_loader_exit_fn);

    /* 8. Shell init */
    mos_shell_init();

    /* 9. Session loop - each VDP connection is one session.
     * With the internal VDP enabled, the router is immediately connected
     * after mos_vdp_internal_init() — no TCP client is required to start.
     * With TCP-only mode, we wait for a client as before. */
    bool first_session = true;

    while (1) {
#ifdef CONFIG_MOS_VDP_INTERNAL_ENABLED
        /* With internal VDP: shell runs immediately, always.
         * On first boot: show banner + run autoexec.
         * After a TCP client disconnects: skip banner/autoexec — just
         * resume the shell prompt on the internal display. */
        if (first_session) {
            ESP_LOGI(TAG, "Starting shell session (tcp=%d internal=%d)",
                     (int)mos_vdp_connected(),
                     (int)mos_vdp_internal_connected());
            print_banner();
            char autoexec[64];
            snprintf(autoexec, sizeof(autoexec), "%s/autoexec.bat", MOS_FLASH_MOUNT);
            FILE *f = fopen(autoexec, "r");
            if (f) {
                fclose(f);
                ESP_LOGI(TAG, "Running %s", autoexec);
                mos_shell_exec("EXEC " MOS_SD_MOUNT "/autoexec.bat");
            }
            first_session = false;
        } else {
            /* Returning from TCP session — clear screen, show prompt */
            ESP_LOGI(TAG, "TCP session ended — resuming internal VDP shell");
            mos_shell_reset();
            print_banner();
        }
#else
        ESP_LOGI(TAG, "Waiting for VDP client on port %d...", MOS_VDP_TCP_PORT);
        while (!mos_vdp_router_connected()) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        /* Give the VDP a moment to finish handshake before sending VDU codes */
        vTaskDelay(pdMS_TO_TICKS(200));
        ESP_LOGI(TAG, "VDP client connected - starting shell session");

        /* Welcome banner (includes CLS + colour reset) */
        print_banner();

        /* Run autoexec if present */
        {
            char autoexec[64];
            snprintf(autoexec, sizeof(autoexec), "%s/autoexec.bat", MOS_FLASH_MOUNT);
            FILE *f = fopen(autoexec, "r");
            if (f) {
                fclose(f);
                ESP_LOGI(TAG, "Running %s", autoexec);
                mos_shell_exec("EXEC " MOS_SD_MOUNT "/autoexec.bat");
            }
        }
#endif

        /* Interactive shell - returns when TCP VDP disconnects.
         * With internal VDP, mos_shell_run() never returns (getch blocks
         * indefinitely until a TCP client connects and later disconnects). */
        mos_shell_run();

        /* Session ended (TCP disconnect) — reset shell state */
        mos_shell_reset();
        ESP_LOGI(TAG, "Session ended — resetting");
    }
}

/* ------------------------------------------------------------------ */
/* app_main — runs on the FreeRTOS main task (stack in LP DRAM).       */
/*                                                                      */
/* nvs_flash_init() uses SPI flash and requires the calling task's     */
/* stack to be in LP DRAM (esp_task_stack_is_sane_cache_disabled).     */
/* SD card (SDMMC) has no such restriction — it's mounted later in     */
/* mos_main_task with a PSRAM stack.                                   */
/* ------------------------------------------------------------------ */
void app_main(void)
{
    /* 0. NVS — must be done before WiFi; requires LP DRAM stack */
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES ||
        nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_err = nvs_flash_init();
    }
    if (nvs_err != ESP_OK) {
        ESP_LOGE(TAG, "NVS init failed: 0x%x", nvs_err);
    }

    /* 1. Spawn mos_main_task with a large PSRAM stack.
     *    CONFIG_SPIRAM_ALLOW_STACK_EXTERNAL_MEMORY=y lets xTaskCreate allocate
     *    task stacks from PSRAM automatically when the requested size exceeds
     *    CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL (16KB). At 192KB xTaskCreate will
     *    use pvPortMallocCaps with MALLOC_CAP_SPIRAM internally — no manual
     *    xTaskCreateStatic needed, and IDF's xTaskCreate correctly uses bytes.
     */
    xTaskCreate(mos_main_task, "mos_main", MOS_MAIN_STACK_KB * 1024,
                NULL, 5, NULL);
}
