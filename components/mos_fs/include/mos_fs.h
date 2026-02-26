/*
 * mos_fs.h - Filesystem abstraction for ESP32-MOS
 *
 * Manages two FAT volumes:
 *   MOS_DRIVE_FLASH  - internal SPI flash, 4 MB FAT partition ("storage")
 *   MOS_DRIVE_SD     - external microSD card via SPI
 *
 * VFS mount points:
 *   Flash -> /flash
 *   SD    -> /sdcard
 *
 * Shell drive prefixes:
 *   A: or /flash/  -> flash
 *   B: or /sdcard/ -> SD card
 */

#ifndef MOS_FS_H
#define MOS_FS_H

#include <stddef.h>
#include <stdbool.h>

#define MOS_FLASH_MOUNT     "/flash"
#define MOS_SD_MOUNT        "/sdcard"
#define MOS_DRIVE_FLASH     'A'
#define MOS_DRIVE_SD        'B'
#define MOS_DRIVE_DEFAULT   MOS_DRIVE_FLASH

int         mos_fs_mount_flash(void);
int         mos_fs_mount_sd(void);
int         mos_fs_remount_sd(void);
bool        mos_fs_flash_mounted(void);
bool        mos_fs_sd_mounted(void);
int         mos_fs_resolve(const char *path, char *out_buf, size_t out_size);
const char *mos_fs_getcwd(void);
int         mos_fs_chdir(const char *path);
char        mos_fs_getdrive(void);
void        mos_fs_setdrive(char drive);

#endif /* MOS_FS_H */
