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

/**
 * Expand a path that may contain wildcard characters (* and ?) into a list
 * of matching absolute paths.
 *
 * @param pattern   Path with optional wildcards in the filename part
 *                  (e.g. "/sdcard/*.bin" or "/sdcard/foo/test?.*").
 *                  Wildcards in directory components are not supported.
 * @param results   Caller-supplied array of char pointers to fill.
 *                  Each entry points into a single heap block — free only
 *                  results[0] when done (if count > 0).
 * @param max       Maximum number of entries in results[].
 * @return          Number of matches found (0 if none, -1 on error).
 */
int mos_fs_glob(const char *pattern, char **results, int max);

/**
 * Read the first non-blank, non-comment line from a text file into buf.
 * Lines beginning with '#' (in any column position) are skipped.
 * Lines longer than buf_size-1 are truncated safely.
 * Trailing newline/CR are stripped.
 * @return  0 on success, -1 if file not found or no data line found.
 */
int         mos_fs_readline(const char *path, char *buf, size_t buf_size);

/* Kept for source compatibility — always returns false now */
static inline bool mos_fs_flash_mounted(void) { return false; }

/* Kept for source compatibility — SD mount is the only storage */
static inline int mos_fs_mount_flash(void) { return 0; }

#endif /* MOS_FS_H */
