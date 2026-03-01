/*
 * mos_fs.c - Filesystem implementation for ESP32-MOS
 */

#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "wear_levelling.h"
#include "sdmmc_cmd.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "driver/sdmmc_host.h"
#include "sd_pwr_ctrl_by_on_chip_ldo.h"
#include "mos_fs.h"

static const char *TAG = "mos_fs";

/* --- State --- */
static bool s_flash_mounted = false;
static bool s_sd_mounted    = false;
static char s_current_drive = MOS_DRIVE_DEFAULT;
static char s_cwd[512]      = MOS_FLASH_MOUNT;

/* Wear-levelling handle for flash FAT */
static wl_handle_t s_wl_handle = WL_INVALID_HANDLE;

/* SD card handle */
static sdmmc_card_t *s_sd_card = NULL;

/* ------------------------------------------------------------------ */
/* Flash (internal)                                                     */
/* ------------------------------------------------------------------ */

int mos_fs_mount_flash(void)
{
    if (s_flash_mounted) {
        return 0;
    }

    /* Mount with wear levelling (read-write).
     * flash_data.sh uses wl_wrap.py to inject valid WL state/config blocks
     * into the FAT image before flashing, so the WL driver finds them and
     * does NOT reformat the partition.
     * format_if_mount_failed = false: if blocks are missing/corrupt, fail
     * loudly instead of silently erasing our files.
     */
    esp_vfs_fat_mount_config_t mount_cfg = {
        .format_if_mount_failed = false,
        .max_files              = 8,
        .allocation_unit_size   = 512,
    };

    esp_err_t err = esp_vfs_fat_spiflash_mount_rw_wl(
        MOS_FLASH_MOUNT,
        "storage",          /* partition label in partitions.csv */
        &mount_cfg,
        &s_wl_handle
    );

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Flash mount failed: %s", esp_err_to_name(err));
        return -1;
    }

    s_flash_mounted = true;
    ESP_LOGI(TAG, "Flash FAT mounted (rw) at %s", MOS_FLASH_MOUNT);
    return 0;
}

/* ------------------------------------------------------------------ */
/* SD card (external)                                                   */
/* ------------------------------------------------------------------ */

#if CONFIG_IDF_TARGET_ESP32P4
/*
 * ESP32-P4: SDMMC native interface (Slot 0, 4-bit, IOMUX-fixed GPIOs).
 *
 * Waveshare ESP32-P4-WIFI6 pin mapping (same as Espressif P4-Function-EV-Board):
 *   CLK = GPIO43   CMD = GPIO44
 *   D0  = GPIO39   D1  = GPIO40   D2  = GPIO41   D3  = GPIO42
 *
 * The SDMMC IO pins on ESP32-P4 are powered by internal LDO_VO4.
 * Without enabling it, card init times out immediately.
 *
 * Slot 1 (GPIOs 18-19, 14-17) is used by the esp-hosted C6 WiFi SDIO link
 * and must NOT be touched here.
 */
#define MOS_SD_LDO_CHAN   4   /* LDO_VO4 powers the SDMMC IO pins */

int mos_fs_mount_sd(void)
{
    if (s_sd_mounted) {
        return 0;
    }

    esp_vfs_fat_sdmmc_mount_config_t mount_cfg = {
        .format_if_mount_failed = false,
        .max_files              = 8,
        .allocation_unit_size   = 512,
    };

    /* Enable LDO_VO4 to power SDMMC IO pins */
    sd_pwr_ctrl_ldo_config_t ldo_cfg = { .ldo_chan_id = MOS_SD_LDO_CHAN };
    sd_pwr_ctrl_handle_t pwr_handle  = NULL;
    esp_err_t err = sd_pwr_ctrl_new_on_chip_ldo(&ldo_cfg, &pwr_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SD LDO init failed: %s", esp_err_to_name(err));
        return -1;
    }

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.slot         = SDMMC_HOST_SLOT_0;
    host.max_freq_khz = SDMMC_FREQ_DEFAULT;  /* 20 MHz — safe for all cards */
    host.pwr_ctrl_handle = pwr_handle;

    sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
    slot.width = 4;
    slot.clk   = 43;
    slot.cmd   = 44;
    slot.d0    = 39;
    slot.d1    = 40;
    slot.d2    = 41;
    slot.d3    = 42;
    slot.cd    = SDMMC_SLOT_NO_CD;   /* no Card Detect pin on Waveshare */
    slot.wp    = SDMMC_SLOT_NO_WP;
    slot.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    err = esp_vfs_fat_sdmmc_mount(MOS_SD_MOUNT, &host, &slot, &mount_cfg, &s_sd_card);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "SD mount failed: %s (no card?)", esp_err_to_name(err));
        sd_pwr_ctrl_del_on_chip_ldo(pwr_handle);
        return -1;
    }

    s_sd_mounted = true;
    ESP_LOGI(TAG, "SD FAT mounted at %s", MOS_SD_MOUNT);
    sdmmc_card_print_info(stdout, s_sd_card);
    return 0;
}

int mos_fs_remount_sd(void)
{
    if (s_sd_mounted) {
        esp_vfs_fat_sdcard_unmount(MOS_SD_MOUNT, s_sd_card);
        s_sd_mounted = false;
        s_sd_card    = NULL;
    }
    return mos_fs_mount_sd();
}

#else  /* ESP32-S3 and others: SPI interface */

/*
 * ESP32-S3 DevKit:
 *   MOSI=11, MISO=13, CLK=12, CS=10 (SPI2; GPIO 19/20 are USB D-/D+)
 */
#ifndef MOS_SD_PIN_MOSI
  #define MOS_SD_PIN_MOSI  11
#endif
#ifndef MOS_SD_PIN_MISO
  #define MOS_SD_PIN_MISO  13
#endif
#ifndef MOS_SD_PIN_CLK
  #define MOS_SD_PIN_CLK   12
#endif
#ifndef MOS_SD_PIN_CS
  #define MOS_SD_PIN_CS    10
#endif
#define MOS_SD_SPI_HOST  SPI2_HOST

int mos_fs_mount_sd(void)
{
    if (s_sd_mounted) {
        return 0;
    }

    esp_vfs_fat_sdmmc_mount_config_t mount_cfg = {
        .format_if_mount_failed = false,
        .max_files              = 8,
        .allocation_unit_size   = 512,
    };

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = MOS_SD_SPI_HOST;

    spi_bus_config_t bus_cfg = {
        .mosi_io_num   = MOS_SD_PIN_MOSI,
        .miso_io_num   = MOS_SD_PIN_MISO,
        .sclk_io_num   = MOS_SD_PIN_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    esp_err_t err = spi_bus_initialize(MOS_SD_SPI_HOST, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(err));
        return -1;
    }

    sdspi_device_config_t slot_cfg = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_cfg.gpio_cs   = MOS_SD_PIN_CS;
    slot_cfg.host_id   = MOS_SD_SPI_HOST;

    err = esp_vfs_fat_sdspi_mount(MOS_SD_MOUNT, &host, &slot_cfg, &mount_cfg, &s_sd_card);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "SD mount failed: %s (no card?)", esp_err_to_name(err));
        return -1;
    }

    s_sd_mounted = true;
    ESP_LOGI(TAG, "SD FAT mounted at %s", MOS_SD_MOUNT);
    sdmmc_card_print_info(stdout, s_sd_card);
    return 0;
}

int mos_fs_remount_sd(void)
{
    if (s_sd_mounted) {
        esp_vfs_fat_sdcard_unmount(MOS_SD_MOUNT, s_sd_card);
        spi_bus_free(MOS_SD_SPI_HOST);
        s_sd_mounted = false;
        s_sd_card    = NULL;
    }
    return mos_fs_mount_sd();
}

#endif /* CONFIG_IDF_TARGET_ESP32P4 */

/* ------------------------------------------------------------------ */
/* Status                                                               */
/* ------------------------------------------------------------------ */

bool mos_fs_flash_mounted(void) { return s_flash_mounted; }
bool mos_fs_sd_mounted(void)    { return s_sd_mounted; }
char mos_fs_getdrive(void)      { return s_current_drive; }
void mos_fs_setdrive(char d)    { s_current_drive = d; }
const char *mos_fs_getcwd(void) { return s_cwd; }

/* ------------------------------------------------------------------ */
/* Path resolution                                                      */
/* ------------------------------------------------------------------ */

/* Strip trailing "/." and trailing "/" (except root) from a resolved path */
static void path_normalise(char *p)
{
    size_t len = strlen(p);
    /* Remove trailing "/." */
    while (len >= 2 && p[len-2] == '/' && p[len-1] == '.') {
        p[len-2] = '\0';
        len -= 2;
    }
    /* Remove trailing "/" unless it IS the root */
    while (len > 1 && p[len-1] == '/') {
        p[--len] = '\0';
    }
    /* If we stripped everything, use mount root */
    if (len == 0) {
        strcpy(p, "/");
    }
}

int mos_fs_resolve(const char *path, char *out_buf, size_t out_size)
{
    if (!path || !out_buf || out_size == 0) return -1;

    const char *mount = (s_current_drive == MOS_DRIVE_SD)
                        ? MOS_SD_MOUNT : MOS_FLASH_MOUNT;

    /* Drive-prefixed: "A:..." or "B:..." */
    if (path[0] != '\0' && path[1] == ':') {
        char drive = path[0];
        const char *rest = path + 2;
        if (drive == MOS_DRIVE_FLASH || drive == 'a') {
            mount = MOS_FLASH_MOUNT;
        } else if (drive == MOS_DRIVE_SD || drive == 'b') {
            mount = MOS_SD_MOUNT;
        }
        /* Skip leading slash if present */
        if (*rest == '/') rest++;
        if (*rest == '\0') {
            snprintf(out_buf, out_size, "%s", mount);
        } else {
            snprintf(out_buf, out_size, "%s/%s", mount, rest);
        }
        path_normalise(out_buf);
        return 0;
    }

    /* Absolute VFS path */
    if (path[0] == '/') {
        snprintf(out_buf, out_size, "%s", path);
        path_normalise(out_buf);
        return 0;
    }

    /* "." means current directory */
    if (strcmp(path, ".") == 0) {
        snprintf(out_buf, out_size, "%s", s_cwd);
        path_normalise(out_buf);
        return 0;
    }

    /* Relative path: append to cwd */
    if (strcmp(s_cwd, "/") == 0 || strcmp(s_cwd, "") == 0) {
        snprintf(out_buf, out_size, "%s/%s", mount, path);
    } else {
        snprintf(out_buf, out_size, "%s/%s", s_cwd, path);
    }
    path_normalise(out_buf);
    return 0;
}

int mos_fs_chdir(const char *path)
{
    char resolved[512];
    if (mos_fs_resolve(path, resolved, sizeof(resolved)) != 0) return -1;

    /* Update current drive if path starts with a known mount */
    if (strncmp(resolved, MOS_FLASH_MOUNT, strlen(MOS_FLASH_MOUNT)) == 0) {
        s_current_drive = MOS_DRIVE_FLASH;
    } else if (strncmp(resolved, MOS_SD_MOUNT, strlen(MOS_SD_MOUNT)) == 0) {
        s_current_drive = MOS_DRIVE_SD;
    }

    snprintf(s_cwd, sizeof(s_cwd), "%s", resolved);
    return 0;
}
