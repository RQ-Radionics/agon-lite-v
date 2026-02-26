/*
 * mos_file.h - File path resolution for ESP32-MOS
 * Ported from agon-mos src/mos_file.h
 *
 * Original used FatFS (f_stat, f_open...) and umm_malloc.
 * ESP32 port uses POSIX VFS (stat, fopen, opendir...) and standard malloc.
 * Prefix resolution (A:, B:) is handled via mos_fs.
 */

#ifndef MOS_FILE_H
#define MOS_FILE_H

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include "mos_strings.h"

#define MOS_PATH_MAX    512

/* Flags for resolvePath / getResolvedPath (same bit positions as original) */
#define RESOLVE_OMIT_EXPAND         0x40
#define RESOLVE_MATCH_ALL_ATTRIBS   0x80
/* Attribute flags (mirrors FatFS AM_* values) */
#define MOS_AM_RDO  0x01
#define MOS_AM_HID  0x02
#define MOS_AM_SYS  0x04
#define MOS_AM_DIR  0x10
#define MOS_AM_ARC  0x20

/*
 * Check if path is a directory.
 * path must already be resolved (absolute, no prefix).
 */
bool mos_isDirectory(const char *path);

/*
 * Return pointer to end of prefix (the ':' character), or NULL.
 * Equivalent to original getFilepathPrefixEnd().
 */
char *mos_getFilepathPrefixEnd(char *filepath);

/*
 * Return pointer to leafname (last component after ':' or '/').
 * Returns pointer to end-of-string if path ends with special components.
 * Equivalent to original getFilepathLeafname().
 */
char *mos_getFilepathLeafname(char *filepath);

/*
 * Resolve a path: expand drive prefix (A:, B:) and combine with cwd.
 * Returns 0 on success, <0 on error.
 * out must be at least MOS_PATH_MAX bytes.
 */
int mos_resolvePath(const char *in, char *out, size_t out_size);

/*
 * Get the directory portion of a path (without leafname).
 * Writes result into out; returns 0 on success.
 */
int mos_getDirectoryForPath(const char *path, char *out, size_t out_size);

/*
 * Resolve a path, allocating a buffer for the result.
 * Caller must free *resolvedPath with free().
 * Returns 0 on success, <0 on error.
 */
int mos_getResolvedPath(const char *source, char **resolvedPath, uint8_t flags);

/*
 * Check if a file lives inside the moslets directory (/flash/mos/).
 */
bool mos_isMoslet(const char *filepath);

/*
 * Copy a file from src to dst.
 * Returns 0 on success, -1 on error.
 */
int mos_copyFile(const char *src, const char *dst);

#endif /* MOS_FILE_H */
