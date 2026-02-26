/*
 * mos_file.c - File path resolution for ESP32-MOS
 * Ported from agon-mos src/mos_file.c
 * Original authors: AGON MOS contributors
 *
 * Changes from original:
 *  - FatFS (f_stat, f_open, f_chdir...) replaced by POSIX VFS (stat, fopen, chdir...)
 *  - umm_malloc replaced by standard malloc/free
 *  - Drive prefix variables (A$Path, B$Path) replaced by mos_fs mount points
 *  - expandMacro/expandVariableToken not available; simple prefix substitution used
 *  - Control-char-terminated strings replaced by normal null-terminated strings
 */

#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <sys/stat.h>
#include "mos_file.h"
#include "mos_fs.h"
#include "mos_strings.h"

/* Moslets directory inside the flash volume */
#define MOS_MOSLET_PATH  MOS_FLASH_MOUNT "/mos/"

/* -------------------------------------------------------------------------
 * isDirectory
 * Check if a resolved (absolute) path is a directory.
 * Equivalent to original isDirectory().
 * -------------------------------------------------------------------------*/
bool mos_isDirectory(const char *path)
{
    if (!path) return false;

    /* Handle special cases: empty, ".", "..", "/" and similar */
    if (path[0] == '\0' ||
        strcmp(path, ".") == 0 || strcmp(path, "..") == 0 ||
        strcmp(path, "/") == 0 ||
        strcmp(path, "./") == 0 || strcmp(path, "../") == 0) {
        return true;
    }

    struct stat st;
    if (stat(path, &st) != 0) return false;
    return S_ISDIR(st.st_mode);
}

/* -------------------------------------------------------------------------
 * getFilepathPrefixEnd
 * Returns pointer to the ':' in a path, or NULL if there is none.
 * Equivalent to original getFilepathPrefixEnd().
 * -------------------------------------------------------------------------*/
char *mos_getFilepathPrefixEnd(char *filepath)
{
    return strchr(filepath, ':');
}

/* -------------------------------------------------------------------------
 * getFilepathLeafname
 * Returns pointer to leafname (last component after ':' or '/').
 * If leafname is "." or "..", returns pointer past end of string.
 * Equivalent to original getFilepathLeafname().
 * -------------------------------------------------------------------------*/
char *mos_getFilepathLeafname(char *filepath)
{
    if (!filepath) return NULL;

    /* Scan backwards to find last ':' or '/' */
    char *leafname = filepath + strlen(filepath);
    while (leafname > filepath) {
        if (leafname[-1] == ':' || leafname[-1] == '/') {
            break;
        }
        leafname--;
    }
    /* Return end-of-string for "." and ".." to avoid treating them as leafnames */
    if ((leafname[0] == '.') &&
        (leafname[1] == '\0' || (leafname[1] == '.' && leafname[2] == '\0'))) {
        return filepath + strlen(filepath);
    }
    return leafname;
}

/* -------------------------------------------------------------------------
 * mos_resolvePath
 * Resolve a path, expanding drive prefix (A:, B:) and combining with cwd.
 * Equivalent to parts of original resolvePath() that handle path expansion.
 *
 * Rules:
 *  - "A:<rest>" → MOS_FLASH_MOUNT/<rest>
 *  - "B:<rest>" → MOS_SD_MOUNT/<rest>
 *  - Absolute paths starting with '/' are used as-is
 *  - Relative paths are combined with cwd
 *
 * Returns 0 on success, -1 on error.
 * -------------------------------------------------------------------------*/
int mos_resolvePath(const char *in, char *out, size_t out_size)
{
    if (!in || !out || out_size == 0) return -1;

    /* Check for drive prefix: single letter followed by ':' */
    if (in[1] == ':' && isalpha((unsigned char)in[0])) {
        char drive = (char)toupper((unsigned char)in[0]);
        const char *rest = in + 2;
        const char *mount;

        if (drive == MOS_DRIVE_FLASH) {
            mount = MOS_FLASH_MOUNT;
        } else if (drive == MOS_DRIVE_SD) {
            mount = MOS_SD_MOUNT;
        } else {
            /* Unknown drive: treat like flash */
            mount = MOS_FLASH_MOUNT;
        }

        if (*rest == '\0') {
            /* "A:" alone → mount point */
            snprintf(out, out_size, "%s", mount);
        } else if (*rest == '/' || *rest == '\\') {
            snprintf(out, out_size, "%s%s", mount, rest);
        } else {
            snprintf(out, out_size, "%s/%s", mount, rest);
        }
        /* Normalise backslashes */
        for (char *p = out; *p; p++) {
            if (*p == '\\') *p = '/';
        }
        return 0;
    }

    /* Absolute path: use as-is */
    if (in[0] == '/') {
        snprintf(out, out_size, "%s", in);
        return 0;
    }

    /* Relative path: combine with cwd */
    const char *cwd = mos_fs_getcwd();
    if (!cwd || cwd[0] == '\0') {
        snprintf(out, out_size, "%s", in);
    } else if (cwd[strlen(cwd) - 1] == '/') {
        snprintf(out, out_size, "%s%s", cwd, in);
    } else {
        snprintf(out, out_size, "%s/%s", cwd, in);
    }
    /* Normalise backslashes */
    for (char *p = out; *p; p++) {
        if (*p == '\\') *p = '/';
    }
    return 0;
}

/* -------------------------------------------------------------------------
 * mos_getDirectoryForPath
 * Get the directory portion of a path (without leafname).
 * Equivalent to original getDirectoryForPath() for the no-prefix case.
 * -------------------------------------------------------------------------*/
int mos_getDirectoryForPath(const char *path, char *out, size_t out_size)
{
    if (!path || !out || out_size == 0) return -1;

    char resolved[MOS_PATH_MAX];
    if (mos_resolvePath(path, resolved, sizeof(resolved)) != 0) return -1;

    char *last_slash = strrchr(resolved, '/');
    if (!last_slash) {
        /* No slash: directory is cwd */
        snprintf(out, out_size, "%s", mos_fs_getcwd());
    } else {
        size_t len = (size_t)(last_slash - resolved);
        if (len == 0) len = 1; /* Root "/" */
        snprintf(out, out_size, "%.*s", (int)len, resolved);
    }
    return 0;
}

/* -------------------------------------------------------------------------
 * mos_getResolvedPath
 * Resolve a path and allocate a buffer for the result.
 * Caller is responsible for free()ing *resolvedPath.
 * Equivalent to original getResolvedPath().
 * -------------------------------------------------------------------------*/
int mos_getResolvedPath(const char *source, char **resolvedPath, uint8_t flags)
{
    (void)flags; /* flags reserved for future use */

    if (!source || !resolvedPath) return -1;

    char *buf = malloc(MOS_PATH_MAX);
    if (!buf) return -1;

    if (mos_resolvePath(source, buf, MOS_PATH_MAX) != 0) {
        free(buf);
        return -1;
    }

    *resolvedPath = buf;
    return 0;
}

/* -------------------------------------------------------------------------
 * mos_isMoslet
 * Check if a file is a moslet (lives in the moslets directory).
 * Equivalent to original isMoslet().
 * -------------------------------------------------------------------------*/
bool mos_isMoslet(const char *filepath)
{
    if (!filepath) return false;

    /* Make a mutable copy to use mos_getFilepathLeafname */
    char *copy = strdup(filepath);
    if (!copy) return false;

    /* Temporarily terminate at leafname to match directory exactly */
    char *leaf = mos_getFilepathLeafname(copy);
    char leafChar = *leaf;
    *leaf = '\0';

    bool result = (pmatch(MOS_MOSLET_PATH, copy,
                          MATCH_BEGINS_WITH | MATCH_CASE_INSENSITIVE |
                          MATCH_DISABLE_HASH | MATCH_DISABLE_STAR) == 0);

    *leaf = leafChar;
    free(copy);
    return result;
}

/* -------------------------------------------------------------------------
 * mos_copyFile
 * Copy a file from src to dst using POSIX stdio.
 * Equivalent to original copyFile().
 * -------------------------------------------------------------------------*/
int mos_copyFile(const char *src, const char *dst)
{
    FILE *fsrc = fopen(src, "rb");
    if (!fsrc) return -1;

    FILE *fdst = fopen(dst, "wb");
    if (!fdst) {
        fclose(fsrc);
        return -1;
    }

    uint8_t buf[1024];
    size_t n;
    int result = 0;

    while ((n = fread(buf, 1, sizeof(buf), fsrc)) > 0) {
        if (fwrite(buf, 1, n, fdst) != n) {
            result = -1;
            break;
        }
    }
    if (ferror(fsrc)) result = -1;

    fclose(fdst);
    fclose(fsrc);
    return result;
}
