/*
 * mos_fs.h - Filesystem abstraction for ESP32-MOS
 *
 * SD card is the sole user storage volume.  There is no internal flash FAT
 * partition — the IDF flash driver asserts that the calling task's stack is
 * in LP DRAM, which is incompatible with the PSRAM stacks used by mos_main
 * and user_task on ESP32-P4.  SDMMC does not have this restriction.
 *
 * VFS mount point:
 *   SD card  -> /sdcard
 *
 * Shell drive prefix:
 *   A: or /sdcard/ -> SD card (only drive)
 */

#ifndef MOS_FS_H
#define MOS_FS_H

#include <stddef.h>
#include <stdbool.h>

/* /flash is kept as an alias for /sdcard so that legacy code that builds
 * paths with MOS_FLASH_MOUNT still resolves to the SD card. */
#define MOS_FLASH_MOUNT     "/sdcard"
#define MOS_SD_MOUNT        "/sdcard"
#define MOS_DRIVE_FLASH     'A'
#define MOS_DRIVE_SD        'A'   /* same drive — only SD card exists */
#define MOS_DRIVE_DEFAULT   MOS_DRIVE_SD

int         mos_fs_mount_sd(void);
int         mos_fs_remount_sd(void);
bool        mos_fs_sd_mounted(void);
int         mos_fs_resolve(const char *path, char *out_buf, size_t out_size);
const char *mos_fs_getcwd(void);
int         mos_fs_chdir(const char *path);
char        mos_fs_getdrive(void);
void        mos_fs_setdrive(char drive);

/* Kept for source compatibility — always returns false now */
static inline bool mos_fs_flash_mounted(void) { return false; }

/* Kept for source compatibility — SD mount is the only storage */
static inline int mos_fs_mount_flash(void) { return 0; }

#endif /* MOS_FS_H */
