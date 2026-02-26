/*
 * mos_shell.c - MOS shell for ESP32-MOS
 * Ported from agon-mos src/mos.c
 *
 * Changes from original:
 *  - FatFS (f_open, f_read, f_unlink, f_rename, f_mkdir...) → POSIX VFS
 *  - umm_malloc/umm_free → malloc/free
 *  - eZ80-specific commands (JMP, LOAD, SAVE, RUN, VDU, VDP, exec16/exec24) omitted or stubbed
 *  - RTC → ESP-IDF time functions
 *  - putch → mos_putch
 *  - mos_EDITLINE → mos_editor_readline
 *  - DIR implemented with opendir/readdir
 */

#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <sys/stat.h>
#include <sys/dirent.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>
#include <errno.h>
#include "mos_shell.h"
#include "mos_hal.h"
#include "mos_fs.h"
#include "mos_file.h"
#include "mos_sysvars.h"
#include "mos_strings.h"
#include "mos_editor.h"
#include "mos_types.h"
#include "esp_heap_caps.h"
#include "mos_loader.h"

/* Shell-specific MATCH flags for command lookup */
#define SHELL_MATCH_CMD  (MATCH_CASE_INSENSITIVE | MATCH_BEGINS_WITH | \
                          MATCH_DISABLE_HASH | MATCH_DISABLE_STAR)

/* -------------------------------------------------------------------------
 * Command table entry
 * ------------------------------------------------------------------------- */
typedef struct {
    const char  *name;
    int        (*func)(char *ptr);
    bool         expandArgs;
    const char  *args;
    const char  *help;
} t_mosCommand;

/* -------------------------------------------------------------------------
 * Forward declarations
 * ------------------------------------------------------------------------- */
int mos_exec(char *buffer);
int mos_execAlias(char *token, char *args, char *saveToken);

static int cmd_DIR(char *ptr);
static int cmd_CD(char *ptr);
static int cmd_CLS(char *ptr);
static int cmd_COPY(char *ptr);
static int cmd_CREDITS(char *ptr);
static int cmd_DEL(char *ptr);
static int cmd_DO(char *ptr);
static int cmd_ECHO(char *ptr);
static int cmd_EXEC(char *ptr);
static int cmd_HELP(char *ptr);
static int cmd_HOTKEY(char *ptr);
static int cmd_IF(char *ptr);
static int cmd_IFTHERE(char *ptr);
static int cmd_MEM(char *ptr);
static int cmd_MKDIR(char *ptr);
static int cmd_MOUNT(char *ptr);
static int cmd_OBEY(char *ptr);
static int cmd_PRINTF(char *ptr);
static int cmd_REN(char *ptr);
static int cmd_SET(char *ptr);
static int cmd_SETEVAL(char *ptr);
static int cmd_SETMACRO(char *ptr);
static int cmd_MODE(char *ptr);
static int cmd_SHOW(char *ptr);
static int cmd_TIME(char *ptr);
static int cmd_TRY(char *ptr);
static int cmd_TYPE(char *ptr);
static int cmd_UNSET(char *ptr);
static int cmd_RUN(char *ptr);

/* -------------------------------------------------------------------------
 * Command table (order matters for abbreviation matching)
 * ------------------------------------------------------------------------- */
static t_mosCommand s_commands[] = {
    { ".",          cmd_DIR,        true,  "[path]",               "List directory" },
    { "Cat",        cmd_DIR,        true,  "[path]",               "List directory" },
    { "CD",         cmd_CD,         true,  "<path>",               "Change directory" },
    { "CDir",       cmd_CD,         true,  "<path>",               "Change directory" },
    { "CLS",        cmd_CLS,        false, NULL,                   "Clear screen" },
    { "Copy",       cmd_COPY,       true,  "<src> <dst>",          "Copy file" },
    { "CP",         cmd_COPY,       true,  "<src> <dst>",          "Copy file" },
    { "Credits",    cmd_CREDITS,    false, NULL,                   "Show credits" },
    { "Delete",     cmd_DEL,        true,  "<file>",               "Delete file" },
    { "Dir",        cmd_DIR,        true,  "[path]",               "List directory" },
    { "Do",         cmd_DO,         false, "<cmd>",                "Execute a command (full path allowed)" },
    { "Echo",       cmd_ECHO,       false, "<text>",               "Print text (GSTrans applied)" },
    { "Erase",      cmd_DEL,        true,  "<file>",               "Delete file" },
    { "Exec",       cmd_EXEC,       true,  "<file>",               "Execute batch file" },
    { "Help",       cmd_HELP,       false, "[cmd]",                "Show help" },
    { "Hotkey",     cmd_HOTKEY,     false, "<n> [cmd]",            "Set function key hotkey" },
    { "If",         cmd_IF,         false, "<cond> THEN <cmd>",    "Conditional command" },
    { "IfThere",    cmd_IFTHERE,    false, "<path> THEN <cmd>",    "If file exists" },
    { "LS",         cmd_DIR,        true,  "[path]",               "List directory" },
    { "Mem",        cmd_MEM,        false, NULL,                   "Show memory info" },
    { "MkDir",      cmd_MKDIR,      true,  "<path>",               "Make directory" },
    { "Mode",       cmd_MODE,       false, "<n>",                  "Set screen mode (VDU 22,n)" },
    { "Mount",      cmd_MOUNT,      false, NULL,                   "Remount SD card" },
    { "Move",       cmd_REN,        true,  "<src> <dst>",          "Rename/move file" },
    { "MV",         cmd_REN,        true,  "<src> <dst>",          "Rename/move file" },
    { "Obey",       cmd_OBEY,       false, "[-v] <file> [args]",   "Execute script file" },
    { "PrintF",     cmd_PRINTF,     false, "<fmt>",                "Print with escape codes" },
    { "Rename",     cmd_REN,        true,  "<src> <dst>",          "Rename/move file" },
    { "RM",         cmd_DEL,        true,  "<file>",               "Delete file" },
    { "Set",        cmd_SET,        false, "<var> <val>",          "Set system variable" },
    { "SetEval",    cmd_SETEVAL,    false, "<var> <expr>",         "Set variable from expression" },
    { "SetMacro",   cmd_SETMACRO,   false, "<var> <macro>",        "Set macro variable" },
    { "Show",       cmd_SHOW,       false, "[pattern]",            "Show system variables" },
    { "Time",       cmd_TIME,       true,  "[yr mo da ho mi se]",  "Get/set time" },
    { "Try",        cmd_TRY,        false, "<cmd>",                "Run command, ignore failure" },
    { "Type",       cmd_TYPE,       true,  "<file>",               "Display file contents" },
    { "Unset",      cmd_UNSET,      false, "<var>",                "Delete system variable" },
    { "Run",        cmd_RUN,        true,  "<file> [args]",         "Load and run a binary from filesystem" },
    { NULL, NULL, false, NULL, NULL }
};

#define COMMANDS_COUNT  (sizeof(s_commands)/sizeof(t_mosCommand) - 1)

/* Error strings matching FR_* codes */
static const char *s_errors[] = {
    "OK",
    "Error accessing storage",
    "Internal error",
    "Storage failure",
    "Could not find file",
    "Could not find path",
    "Invalid path name",
    "Access denied or directory full",
    "Access denied",
    "Invalid file/directory object",
    "Storage is write protected",
    "Logical drive number is invalid",
    "Volume has no work area",
    "No valid FAT volume",
    "Error occurred during mkfs",
    "Volume timeout",
    "Volume locked",
    "LFN working buffer could not be allocated",
    "Too many open files",
    "Invalid parameter",
    /* MOS-specific (index 20+) */
    "Invalid command",
    "Invalid executable",
    "Out of memory",
    "Not implemented",
    "Load overlaps system area",
    "Bad string",
    "Too deep",
};
#define ERRORS_COUNT (sizeof(s_errors)/sizeof(char *))

/* Shell state */
static int s_depth = 0;     /* command nesting depth (prevent infinite loops) */

/* -------------------------------------------------------------------------
 * Helpers
 * ------------------------------------------------------------------------- */

static void mos_error(int error)
{
    if (error >= 0 && error < (int)ERRORS_COUNT) {
        mos_printf("\r\n%s\r\n", s_errors[error]);
    }
}

/* Trim leading/trailing whitespace (and optionally leading asterisks).
 * Modifies string in-place; returns new start pointer. */
static char *mos_trim(char *s, bool removeLeadingAsterisks, bool removeTrailingSpaces)
{
    char *ptr;
    if (!s) return NULL;
    if (!*s) return s;

    while (isspace((unsigned char)*s) ||
           (removeLeadingAsterisks && *s == '*')) {
        s++;
    }
    ptr = s + strlen(s) - 1;
    while (ptr > s && isspace((unsigned char)*ptr) &&
           (removeTrailingSpaces || *ptr != ' ')) {
        ptr--;
    }
    ptr[1] = '\0';
    return s;
}

/* Lookup command by name (case-insensitive, abbreviation allowed) */
static t_mosCommand *mos_getCommand(char *name)
{
    for (int i = 0; s_commands[i].name; i++) {
        if (pmatch(name, s_commands[i].name, SHELL_MATCH_CMD) == 0) {
            return &s_commands[i];
        }
    }
    return NULL;
}

/* Print a string showing pipe-escaped control chars */
static void printEscapedString(char *value)
{
    while (*value) {
        unsigned char c = (unsigned char)*value;
        if (c < 0x20) {
            mos_putch('|');
            mos_putch((char)(c + 0x40));
        } else if (c == 0x7F) {
            mos_putch('|');
            mos_putch('?');
        } else if (c == '|') {
            mos_putch('|');
            mos_putch('|');
        } else {
            mos_putch((char)c);
        }
        value++;
    }
}

/* -------------------------------------------------------------------------
 * mos_exec — Execute a command line
 * ------------------------------------------------------------------------- */
int mos_exec(char *buffer)
{
    char *ptr;
    int   result = 0;

    if (s_depth > 10) {
        return MOS_TOO_DEEP;
    }

    ptr = mos_trim(buffer, true, false);
    if (!ptr || *ptr == '#' || *ptr == '\0' ||
        (*ptr == '|' && *(ptr + 1) == ' ')) {
        return FR_OK;
    }

    /* Extract command token */
    char *commandPtr = NULL;
    char *argsPtr    = ptr;
    int   result2    = extractString(ptr, &argsPtr, " .", &commandPtr,
                                     EXTRACT_FLAG_OMIT_LEADSKIP |
                                     EXTRACT_FLAG_INCLUDE_QUOTES);
    if (result2 == FR_INVALID_PARAMETER && commandPtr && *commandPtr == '.') {
        result2 = FR_OK;
    }
    if (result2 != FR_OK) {
        return result2;
    }
    if (argsPtr && *argsPtr == '.') {
        argsPtr++;
    }

    int cmdLen = (int)(argsPtr - commandPtr);
    /* Strip surrounding quotes */
    if (*commandPtr == '"' && cmdLen >= 2 && commandPtr[cmdLen - 1] == '"') {
        commandPtr++;
        cmdLen -= 2;
    }
    argsPtr = mos_trim(argsPtr, false, false);

    bool skipAlias = false;
    if (*commandPtr == '%') {
        /* Leading % bypasses alias expansion */
        commandPtr++;
        cmdLen--;
        skipAlias = true;
    }

    if (!skipAlias) {
        /* Check for Alias$<cmd> */
        char *aliasToken = malloc((size_t)(cmdLen + 7));
        if (!aliasToken) return MOS_OUT_OF_MEMORY;
        snprintf(aliasToken, (size_t)(cmdLen + 7), "Alias$%.*s", cmdLen, commandPtr);
        /* Trailing '.' in alias token → wildcard */
        size_t alen = strlen(aliasToken);
        if (cmdLen > 1 && aliasToken[alen - 1] == '.') {
            aliasToken[alen - 1] = '*';
        }
        result = mos_execAlias(aliasToken, argsPtr, NULL);
        free(aliasToken);
        if (result != MOS_NOT_IMPLEMENTED) {
            return result;
        }
    }

    /* Build a null-terminated command name */
    char *command = malloc((size_t)(cmdLen + 1));
    if (!command) return MOS_OUT_OF_MEMORY;
    snprintf(command, (size_t)(cmdLen + 1), "%.*s", cmdLen, commandPtr);

    /* Drive change: "A:" or "B:" (case-insensitive, no args) */
    if (cmdLen == 2 && command[1] == ':') {
        char drive = command[0];
        char resolved[MOS_PATH_MAX];
        int r;
        if (drive == MOS_DRIVE_FLASH || drive == 'a') {
            if (!mos_fs_flash_mounted()) {
                mos_printf("Drive A: not available\r\n");
                free(command); return FR_NOT_READY;
            }
            mos_fs_resolve("A:", resolved, sizeof(resolved));
            r = mos_fs_chdir(resolved);
            free(command); return r;
        } else if (drive == MOS_DRIVE_SD || drive == 'b') {
            if (!mos_fs_sd_mounted()) {
                mos_printf("Drive B: not available\r\n");
                free(command); return FR_NOT_READY;
            }
            mos_fs_resolve("B:", resolved, sizeof(resolved));
            r = mos_fs_chdir(resolved);
            free(command); return r;
        }
    }

    t_mosCommand *cmd = mos_getCommand(command);
    free(command);

    if (cmd && cmd->func) {
        char *args = NULL;
        if (cmd->expandArgs) {
            args = expandMacro(argsPtr);
        }
        result = cmd->func(args ? args : argsPtr);
        free(args);
        return result;
    }

    /* Not a built-in — try to find <command>.bbc or <command>.bin */
    {
        char name[64];
        char resolved[MOS_PATH_MAX];
        char interp[MOS_PATH_MAX];
        snprintf(name, sizeof(name), "%.*s", cmdLen, commandPtr);

        const char *search[] = { mos_fs_getcwd(), MOS_FLASH_MOUNT, NULL };

        /* 1. BBC BASIC script: <name>.bbc — run via bbcbasic.bin */
        char bbcname[68];
        snprintf(bbcname, sizeof(bbcname), "%s.bbc", name);
        bool found_bbc = false;
        for (int si = 0; search[si] && !found_bbc; si++) {
            snprintf(resolved, sizeof(resolved), "%s/%s", search[si], bbcname);
            FILE *probe = fopen(resolved, "rb");
            if (probe) { fclose(probe); found_bbc = true; }
        }
        if (found_bbc) {
            /* Find bbcbasic.bin in cwd or /flash */
            bool found_interp = false;
            for (int si = 0; search[si] && !found_interp; si++) {
                snprintf(interp, sizeof(interp), "%s/bbcbasic.bin", search[si]);
                FILE *probe = fopen(interp, "rb");
                if (probe) { fclose(probe); found_interp = true; }
            }
            if (!found_interp) {
                mos_printf("bbcbasic.bin not found\r\n");
                return FR_NO_FILE;
            }
            char *argv_buf[3] = { "bbcbasic.bin", resolved, NULL };
            int ret = mos_loader_exec(interp, 2, argv_buf);
            if (ret < 0) {
                mos_printf("Failed to run '%s'\r\n", bbcname);
                return FR_INT_ERR;
            }
            return FR_OK;
        }

        /* 2. Native binary: <name>.bin */
        char binname[68];
        snprintf(binname, sizeof(binname), "%s.bin", name);
        bool found_bin = false;
        for (int si = 0; search[si] && !found_bin; si++) {
            snprintf(resolved, sizeof(resolved), "%s/%s", search[si], binname);
            FILE *probe = fopen(resolved, "rb");
            if (probe) { fclose(probe); found_bin = true; }
        }
        if (found_bin) {
            char *argv_buf[17];
            int   argc = 0;
            argv_buf[argc++] = binname;
            char *ap = argsPtr;
            char *arg;
            while (argc < 16) {
                if (extractString(ap, &ap, NULL, &arg,
                                  EXTRACT_FLAG_AUTO_TERMINATE) != FR_OK) break;
                argv_buf[argc++] = arg;
            }
            argv_buf[argc] = NULL;
            int ret = mos_loader_exec(resolved, argc, argv_buf);
            if (ret < 0) {
                mos_printf("Failed to run '%s'\r\n", binname);
                return FR_INT_ERR;
            }
            return FR_OK;
        }

        mos_printf("Unknown command: %.*s (type HELP for list)\r\n", cmdLen, commandPtr);
        return MOS_INVALID_COMMAND;
    }
}

/* -------------------------------------------------------------------------
 * mos_execAlias — Expand and run an alias variable
 * ------------------------------------------------------------------------- */
int mos_execAlias(char *token, char *args, char *saveToken)
{
    char *alias = expandVariableToken(token);
    if (!alias) {
        return MOS_NOT_IMPLEMENTED;
    }

    char *expandedAlias = substituteArguments(alias, args ? args : "", 0);
    free(alias);
    if (!expandedAlias) {
        return FR_INT_ERR;
    }

    if (saveToken) {
        createOrUpdateSystemVariable(saveToken, MOS_VAR_STRING, expandedAlias);
    }

    s_depth++;
    int result = FR_OK;
    char *commandEnd = expandedAlias;
    while (*commandEnd != '\0') {
        char *command = NULL;
        result = extractString(commandEnd, &commandEnd, "\r",
                               &command,
                               EXTRACT_FLAG_OMIT_LEADSKIP |
                               EXTRACT_FLAG_INCLUDE_QUOTES |
                               EXTRACT_FLAG_AUTO_TERMINATE);
        if (result != FR_OK) break;
        result = mos_exec(command);
        if (result != FR_OK) break;
    }
    free(expandedAlias);
    s_depth--;

    return result;
}

/* -------------------------------------------------------------------------
 * CMD: DIR / LS / Cat / .
 * ------------------------------------------------------------------------- */

/* DIR flags */
#define MOS_DIR_LONG_LISTING    0x01
#define MOS_DIR_SHOW_HIDDEN     0x02
#define MOS_DIR_SHOW_SYSTEM     0x04
#define MOS_DIR_HIDE_VOLUME_INFO 0x08

static int do_DIR(const char *path_in, uint8_t flags)
{
    char resolved[MOS_PATH_MAX];
    mos_fs_resolve(path_in, resolved, sizeof(resolved));

    DIR *d = opendir(resolved);
    if (!d) {
        mos_printf("DIR: cannot open '%s': %s\r\n", resolved, strerror(errno));
        return FR_NO_PATH;
    }

    if (!(flags & MOS_DIR_HIDE_VOLUME_INFO)) {
        mos_printf("\r\nDirectory of %s\r\n\r\n", resolved);
    }

    struct dirent *entry;
    while ((entry = readdir(d)) != NULL) {
        if (entry->d_name[0] == '.' && !(flags & MOS_DIR_SHOW_HIDDEN)) continue;

        char full[MOS_PATH_MAX + 256];
        snprintf(full, sizeof(full), "%s/%s", resolved, entry->d_name);
        struct stat st;
        bool got_stat = (stat(full, &st) == 0);

        bool is_dir  = got_stat && S_ISDIR(st.st_mode);

        if (flags & MOS_DIR_LONG_LISTING) {
            if (got_stat) {
                char timebuf[20] = "                   ";
                struct tm *t = localtime(&st.st_mtime);
                if (t) strftime(timebuf, sizeof(timebuf), "%Y-%m-%d %H:%M:%S", t);
                mos_printf("  %s  %-8s  %s\r\n",
                           timebuf,
                           is_dir ? "<DIR>" : "",
                           entry->d_name);
                if (!is_dir) {
                    mos_printf("  %*s  %ld bytes\r\n", 19, "", (long)st.st_size);
                }
            } else {
                mos_printf("  %-19s  %-8s  %s\r\n", "?", is_dir ? "<DIR>" : "", entry->d_name);
            }
        } else {
            if (is_dir) {
                mos_printf("  %-32s  <DIR>\r\n", entry->d_name);
            } else {
                mos_printf("  %-32s  %ld\r\n",
                           entry->d_name,
                           got_stat ? (long)st.st_size : -1L);
            }
        }
    }
    closedir(d);
    mos_printf("\r\n");
    return FR_OK;
}

static int cmd_DIR(char *ptr)
{
    uint8_t flags = 0;
    char *path;
    int   result;

    for (;;) {
        result = extractString(ptr, &ptr, NULL, &path, EXTRACT_FLAG_AUTO_TERMINATE);
        if (result == FR_INVALID_PARAMETER) {
            path = ".";
            break;
        } else if (result != FR_OK) {
            return result;
        }
        if (path[0] == '-') {
            path++;
            while (*path) {
                switch (*path) {
                    case 'l': flags |= MOS_DIR_LONG_LISTING;    break;
                    case 'a': flags |= MOS_DIR_SHOW_HIDDEN;     break;
                    case 's': flags |= MOS_DIR_SHOW_SYSTEM;     break;
                    case 'v': flags |= MOS_DIR_HIDE_VOLUME_INFO;break;
                    default:  mos_printf("Invalid flag: %c\r\n", *path); break;
                }
                path++;
            }
        } else {
            break;
        }
    }
    return do_DIR(path, flags);
}

/* -------------------------------------------------------------------------
 * CMD: CD
 * ------------------------------------------------------------------------- */
static int cmd_CD(char *ptr)
{
    char *path;
    int result = extractString(ptr, &ptr, NULL, &path, EXTRACT_FLAG_AUTO_TERMINATE);

    if (result != FR_OK) {
        /* No arg: print cwd */
        mos_printf("%s\r\n", mos_fs_getcwd());
        return FR_OK;
    }

    char resolved[MOS_PATH_MAX];
    mos_fs_resolve(path, resolved, sizeof(resolved));

    if (!mos_isDirectory(resolved)) {
        mos_printf("CD: '%s' is not a directory\r\n", resolved);
        return FR_NO_PATH;
    }
    return mos_fs_chdir(resolved);
}

/* -------------------------------------------------------------------------
 * CMD: CLS
 * ------------------------------------------------------------------------- */
static int cmd_CLS(char *ptr)
{
    (void)ptr;
    mos_putch(12);   /* VDU 12 — clear screen (BBC Micro / Agon VDP) */
    return FR_OK;
}

/* -------------------------------------------------------------------------
 * CMD: MODE
 * ------------------------------------------------------------------------- */
static int cmd_MODE(char *ptr)
{
    while (*ptr == ' ') ptr++;
    if (!*ptr) {
        mos_puts("Usage: MODE <n>\r\n");
        return FR_OK;
    }
    int n = atoi(ptr);
    if (n < 0 || n > 255) {
        mos_puts("Mode out of range\r\n");
        return FR_INVALID_PARAMETER;
    }
    mos_putch(22);              /* VDU 22 — MODE */
    mos_putch((uint8_t)n);
    return FR_OK;
}

/* -------------------------------------------------------------------------
 * CMD: COPY / CP
 * ------------------------------------------------------------------------- */
static int cmd_COPY(char *ptr)
{
    char *src, *dst;
    int result = extractString(ptr, &ptr, NULL, &src, EXTRACT_FLAG_AUTO_TERMINATE);
    if (result == FR_OK) result = extractString(ptr, &ptr, NULL, &dst, EXTRACT_FLAG_AUTO_TERMINATE);
    if (result != FR_OK) return result;

    char rsrc[MOS_PATH_MAX], rdst[MOS_PATH_MAX];
    mos_fs_resolve(src, rsrc, sizeof(rsrc));
    mos_fs_resolve(dst, rdst, sizeof(rdst));

    /* If dst is a directory, copy into it with same filename */
    if (mos_isDirectory(rdst)) {
        char *leaf = mos_getFilepathLeafname(rsrc);
        char newdst[MOS_PATH_MAX * 2];
        snprintf(newdst, sizeof(newdst), "%s/%s", rdst, leaf);
        return mos_copyFile(rsrc, newdst);
    }
    return mos_copyFile(rsrc, rdst);
}

/* -------------------------------------------------------------------------
 * CMD: CREDITS
 * ------------------------------------------------------------------------- */
static int cmd_CREDITS(char *ptr)
{
    (void)ptr;
    mos_printf("ESP32-MOS\r\n");
    mos_printf("Ported from Agon MOS by Dean Belfield and contributors\r\n");
    mos_printf("ESP-IDF (c) Espressif Systems\r\n");
    mos_printf("FatFS R0.14b (c) 2021 ChaN\r\n");
    mos_printf("\r\n");
    return FR_OK;
}

/* -------------------------------------------------------------------------
 * CMD: DEL / DELETE / ERASE / RM
 * ------------------------------------------------------------------------- */
static int cmd_DEL(char *ptr)
{
    char   *filename;
    bool    force   = false;
    bool    verbose = false;
    int     result;

    result = extractString(ptr, &ptr, NULL, &filename, EXTRACT_FLAG_AUTO_TERMINATE);
    if (result == FR_OK && strcasecmp(filename, "-f") == 0) {
        force  = true;
        result = extractString(ptr, &ptr, NULL, &filename, EXTRACT_FLAG_AUTO_TERMINATE);
    }
    if (result != FR_OK) return result;

    /* Verbose if pattern has wildcards */
    verbose = (mos_strcspn(filename, "*?:") != strlen(filename));
    if (!force) force = !verbose;

    char resolved[MOS_PATH_MAX];
    mos_fs_resolve(filename, resolved, sizeof(resolved));

    if (!force) {
        char verify[8];
        mos_printf("Delete %s? (Yes/No) ", resolved);
        mos_editor_readline("", verify, sizeof(verify));
        mos_printf("\r\n");
        if (strcasecmp(verify, "Yes") != 0 && strcasecmp(verify, "Y") != 0) {
            mos_printf("Cancelled.\r\n");
            return FR_OK;
        }
    }
    if (verbose) {
        mos_printf("Deleting %s\r\n", resolved);
    }

    if (remove(resolved) != 0) {
        mos_printf("DEL: cannot delete '%s': %s\r\n", resolved, strerror(errno));
        return FR_NO_FILE;
    }
    return FR_OK;
}

/* -------------------------------------------------------------------------
 * CMD: DO
 * ------------------------------------------------------------------------- */
static int cmd_DO(char *ptr)
{
    return mos_exec(ptr);
}

/* -------------------------------------------------------------------------
 * CMD: ECHO
 * ------------------------------------------------------------------------- */
static int cmd_ECHO(char *ptr)
{
    t_mosTransInfo *transInfo;
    char read_char;
    int result = gsInit(ptr, &transInfo,
                        GSTRANS_FLAG_NO_DOUBLEQUOTE | GSTRANS_FLAG_NO_TRACE);
    if (result != FR_OK) return result;

    while (transInfo != NULL) {
        result = gsRead(&transInfo, &read_char);
        if (result != FR_OK) {
            if (transInfo) gsDispose(&transInfo);
            return result;
        }
        if (transInfo == NULL) break;
        mos_putch(read_char);
    }
    mos_printf("\r\n");
    return FR_OK;
}

/* -------------------------------------------------------------------------
 * CMD: EXEC  — run a batch file of MOS commands
 * ------------------------------------------------------------------------- */
static int exec_file(const char *filepath, char *args, bool verbose)
{
    char resolved[MOS_PATH_MAX];
    mos_fs_resolve(filepath, resolved, sizeof(resolved));

    FILE *f = fopen(resolved, "r");
    if (!f) {
        mos_printf("EXEC: cannot open '%s': %s\r\n", resolved, strerror(errno));
        return FR_NO_FILE;
    }

    /* Set Obey$Dir to directory of script */
    char dir[MOS_PATH_MAX];
    mos_getDirectoryForPath(resolved, dir, sizeof(dir));
    createOrUpdateSystemVariable("Obey$Dir", MOS_VAR_STRING, dir);

    char *buf = malloc(256);
    if (!buf) { fclose(f); return MOS_OUT_OF_MEMORY; }

    s_depth++;
    int result = FR_OK;
    int lineno = 0;

    while (fgets(buf, 256, f)) {
        lineno++;
        /* Strip trailing newline */
        size_t len = strlen(buf);
        while (len > 0 && (buf[len-1] == '\r' || buf[len-1] == '\n')) {
            buf[--len] = '\0';
        }

        char *substituted = substituteArguments(buf, args ? args : "", 1);
        if (!substituted) { result = MOS_OUT_OF_MEMORY; break; }

        if (verbose) {
            mos_printf("Exec: %s\r\n", substituted);
        }
        result = mos_exec(substituted);
        free(substituted);
        if (result != FR_OK) {
            mos_printf("\r\nError at %s line %d\r\n", resolved, lineno);
            break;
        }
    }

    s_depth--;
    fclose(f);
    free(buf);
    return result;
}

static int cmd_EXEC(char *ptr)
{
    char *filename;
    int result = extractString(ptr, &ptr, NULL, &filename, EXTRACT_FLAG_AUTO_TERMINATE);
    if (result != FR_OK) return result;
    return exec_file(filename, ptr, false);
}

static int cmd_OBEY(char *ptr)
{
    char *filename;
    bool  verbose = false;
    int result = extractString(ptr, &ptr, NULL, &filename, EXTRACT_FLAG_AUTO_TERMINATE);
    if (result == FR_OK && strcasecmp(filename, "-v") == 0) {
        verbose = true;
        result = extractString(ptr, &ptr, NULL, &filename, EXTRACT_FLAG_AUTO_TERMINATE);
    }
    if (result != FR_OK) return result;
    return exec_file(filename, ptr, verbose);
}

/* -------------------------------------------------------------------------
 * CMD: HELP
 * ------------------------------------------------------------------------- */
static void printCommandInfo(t_mosCommand *cmd, bool full)
{
    if (!cmd->help) return;
    mos_printf("%s", cmd->name);
    if (cmd->args) mos_printf(" %s", cmd->args);

    /* Find and print aliases */
    int aliases = 0;
    for (int i = 0; s_commands[i].name; i++) {
        if (s_commands[i].func == cmd->func && s_commands[i].name != cmd->name) {
            aliases++;
        }
    }
    if (aliases > 0) {
        mos_printf(" (Aliases: ");
        for (int i = 0; s_commands[i].name; i++) {
            if (s_commands[i].func == cmd->func && s_commands[i].name != cmd->name) {
                mos_printf("%s", s_commands[i].name);
                if (aliases-- > 1) mos_printf(", ");
            }
        }
        mos_printf(")");
    }
    mos_printf("\r\n");
    if (full) {
        mos_printf("  %s\r\n", cmd->help);
    }
}

static int cmd_HELP(char *ptr)
{
    char *cmdname;
    int result = extractString(ptr, &ptr, NULL, &cmdname, EXTRACT_FLAG_AUTO_TERMINATE);
    bool hasCmd = (result == FR_OK);

    if (result == FR_INVALID_PARAMETER) {
        cmdname = "help";
    } else if (result != FR_OK) {
        return result;
    }

    if (strcasecmp(cmdname, "all") == 0) {
        for (int i = 0; s_commands[i].name; i++) {
            printCommandInfo(&s_commands[i], false);
        }
        return FR_OK;
    }

    do {
        bool found = false;
        for (int i = 0; s_commands[i].name; i++) {
            if (pmatch(cmdname, s_commands[i].name, SHELL_MATCH_CMD) == 0) {
                found = true;
                printCommandInfo(&s_commands[i], true);
                if (!hasCmd) {
                    mos_printf("\r\nAvailable commands:\r\n");
                    for (int j = 1; s_commands[j].name; j++) {
                        if (!s_commands[j].help) continue;
                        mos_printf("  %s\r\n", s_commands[j].name);
                    }
                }
            }
        }
        if (!found) {
            mos_printf("Command not found: %s\r\n", cmdname);
        }
        result = extractString(ptr, &ptr, NULL, &cmdname, EXTRACT_FLAG_AUTO_TERMINATE);
    } while (result == FR_OK);

    return (result == FR_INVALID_PARAMETER) ? FR_OK : result;
}

/* -------------------------------------------------------------------------
 * CMD: HOTKEY
 * ------------------------------------------------------------------------- */
static int cmd_HOTKEY(char *ptr)
{
    int fn_number = 0;
    char label[16];

    if (!extractNumber(ptr, &ptr, NULL, &fn_number, 0)) {
        /* No number: list current hotkeys */
        mos_printf("Hotkey assignments:\r\n\r\n");
        for (int k = 1; k <= 12; k++) {
            t_mosSystemVariable *hotkeyVar = NULL;
            snprintf(label, sizeof(label), "Hotkey$%d", k);
            mos_printf("F%d:%s ", k, k < 10 ? " " : "");
            if (getSystemVariable(label, &hotkeyVar) == 0) {
                printEscapedString((char *)hotkeyVar->value);
            } else {
                mos_printf("N/A");
            }
            mos_printf("\r\n");
        }
        mos_printf("\r\n");
        return FR_OK;
    }

    ptr = mos_trim(ptr, false, false);

    if (fn_number < 1 || fn_number > 12) return FR_INVALID_PARAMETER;
    snprintf(label, sizeof(label), "Hotkey$%d", fn_number);

    if (!ptr || strlen(ptr) < 1) {
        t_mosSystemVariable *hotkeyVar = NULL;
        if (getSystemVariable(label, &hotkeyVar) == 0) {
            removeSystemVariable(hotkeyVar);
            mos_printf("F%d cleared.\r\n", fn_number);
        } else {
            mos_printf("F%d already clear.\r\n", fn_number);
        }
        return FR_OK;
    }

    /* Strip surrounding quotes */
    if (ptr[0] == '"' && ptr[strlen(ptr) - 1] == '"') {
        ptr[strlen(ptr) - 1] = '\0';
        ptr++;
    }

    if (strlen(ptr) > 242) return MOS_BAD_STRING;
    strcat(ptr, "|M");

    char *hotkeyString = expandMacro(ptr);
    if (!hotkeyString) return FR_INT_ERR;
    int result = createOrUpdateSystemVariable(label, MOS_VAR_STRING, hotkeyString);
    free(hotkeyString);
    return result;
}

/* -------------------------------------------------------------------------
 * CMD: IF <cond> THEN <cmd> [ELSE <cmd>]
 * ------------------------------------------------------------------------- */
static int cmd_IF(char *ptr)
{
    char *then_part  = stristr(ptr, " THEN ");
    char *else_part  = NULL;
    int   result     = FR_OK;

    if (!then_part) return FR_INVALID_PARAMETER;
    *then_part = '\0';
    then_part += 6;

    else_part = stristr(then_part, " ELSE ");
    if (else_part) {
        *else_part = '\0';
        else_part += 6;
    }

    t_mosEvalResult *eval = evaluateExpression(ptr);
    if (!eval) return FR_INT_ERR;

    bool cond = false;
    if (eval->status == FR_OK) {
        if (eval->type == MOS_VAR_STRING) {
            cond = (strlen((char *)eval->result) > 0);
            free(eval->result);
        } else if (eval->type == MOS_VAR_NUMBER) {
            cond = (eval->result != NULL);
        }
    } else if (eval->status != FR_INVALID_PARAMETER) {
        result = eval->status;
    }
    free(eval);

    if (result == FR_OK) {
        if (cond) {
            result = mos_exec(then_part);
        } else if (else_part) {
            result = mos_exec(else_part);
        }
    }
    return result;
}

/* -------------------------------------------------------------------------
 * CMD: IFTHERE <path> THEN <cmd> [ELSE <cmd>]
 * ------------------------------------------------------------------------- */
static int cmd_IFTHERE(char *ptr)
{
    char *then_part = stristr(ptr, " THEN ");
    char *else_part = NULL;

    if (!then_part) return FR_INVALID_PARAMETER;
    *then_part = '\0';
    then_part += 6;

    else_part = stristr(then_part, " ELSE ");
    if (else_part) {
        *else_part = '\0';
        else_part += 6;
    }

    ptr = mos_trim(ptr, false, true);
    char *filepath = expandMacro(ptr);
    if (!filepath) return FR_INVALID_PARAMETER;

    /* Strip surrounding quotes */
    char *p = filepath;
    if (p[0] == '"' && p[strlen(p) - 1] == '"') {
        p[strlen(p) - 1] = '\0';
        p++;
    }

    bool exists = false;
    if (strlen(p) > 0) {
        char resolved[MOS_PATH_MAX];
        mos_fs_resolve(p, resolved, sizeof(resolved));
        struct stat st;
        exists = (stat(resolved, &st) == 0);
    }
    free(filepath);

    if (exists) {
        return mos_exec(then_part);
    } else if (else_part) {
        return mos_exec(else_part);
    }
    return FR_OK;
}

/* -------------------------------------------------------------------------
 * CMD: MEM — show heap / flash info
 * ------------------------------------------------------------------------- */
static int cmd_MEM(char *ptr)
{
    (void)ptr;
    size_t free_heap  = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
    size_t largest    = heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);
    mos_printf("Free heap:    %u bytes\r\n", (unsigned)free_heap);
    mos_printf("Largest block: %u bytes\r\n", (unsigned)largest);
    mos_printf("Flash mount:  %s\r\n", MOS_FLASH_MOUNT);
    mos_printf("SD mount:     %s\r\n", MOS_SD_MOUNT);
    mos_printf("\r\n");
    return FR_OK;
}

/* -------------------------------------------------------------------------
 * CMD: MKDIR
 * ------------------------------------------------------------------------- */
static int cmd_MKDIR(char *ptr)
{
    char *path;
    int result = extractString(ptr, &ptr, NULL, &path, EXTRACT_FLAG_AUTO_TERMINATE);
    if (result != FR_OK) return result;

    char resolved[MOS_PATH_MAX];
    mos_fs_resolve(path, resolved, sizeof(resolved));

    if (mkdir(resolved, 0755) != 0) {
        mos_printf("MKDIR: cannot create '%s': %s\r\n", resolved, strerror(errno));
        return FR_INT_ERR;
    }
    return FR_OK;
}

/* -------------------------------------------------------------------------
 * CMD: MOUNT
 * ------------------------------------------------------------------------- */
static int cmd_MOUNT(char *ptr)
{
    (void)ptr;
    mos_printf("Remounting SD card...\r\n");
    int r = mos_fs_remount_sd();
    mos_printf(r == 0 ? "SD mounted OK\r\n" : "SD mount failed\r\n");
    return r;
}

/* -------------------------------------------------------------------------
 * CMD: PRINTF
 * ------------------------------------------------------------------------- */
static int cmd_PRINTF(char *ptr)
{
    const char *p = ptr;
    while (*p) {
        if (*p == '\\') {
            p++;
            switch (*p) {
                case '\\': mos_putch('\\'); p++; break;
                case 'r':  mos_putch('\r'); p++; break;
                case 'n':  mos_putch('\n'); p++; break;
                case 'f':  mos_putch(12);  p++; break;
                case 't':  mos_putch('\t'); p++; break;
                case 'x': {
                    p++;
                    int c = 0;
                    if (isxdigit((unsigned char)*p)) {
                        c = (isdigit((unsigned char)*p) ?
                             (*p - '0') : (toupper((unsigned char)*p) - 55));
                        p++;
                        if (isxdigit((unsigned char)*p)) {
                            c = c * 16 + (isdigit((unsigned char)*p) ?
                                          (*p - '0') :
                                          (toupper((unsigned char)*p) - 55));
                            p++;
                        }
                    }
                    mos_putch((char)c);
                    break;
                }
                default:
                    if (*p) p++;
                    break;
            }
        } else {
            mos_putch(*p++);
        }
    }
    return FR_OK;
}

/* -------------------------------------------------------------------------
 * CMD: REN / RENAME / MOVE / MV
 * ------------------------------------------------------------------------- */
static int cmd_REN(char *ptr)
{
    char *src, *dst;
    int result = extractString(ptr, &ptr, NULL, &src, EXTRACT_FLAG_AUTO_TERMINATE);
    if (result == FR_OK) result = extractString(ptr, &ptr, NULL, &dst, EXTRACT_FLAG_AUTO_TERMINATE);
    if (result != FR_OK) return result;

    char rsrc[MOS_PATH_MAX], rdst[MOS_PATH_MAX];
    mos_fs_resolve(src, rsrc, sizeof(rsrc));
    mos_fs_resolve(dst, rdst, sizeof(rdst));

    if (mos_isDirectory(rdst)) {
        char *leaf = mos_getFilepathLeafname(rsrc);
        char newdst[MOS_PATH_MAX * 2];
        snprintf(newdst, sizeof(newdst), "%s/%s", rdst, leaf);
        if (rename(rsrc, newdst) != 0) goto ren_err;
    } else {
        if (rename(rsrc, rdst) != 0) goto ren_err;
    }
    return FR_OK;

ren_err:
    mos_printf("REN: cannot rename '%s': %s\r\n", rsrc, strerror(errno));
    return FR_INT_ERR;
}

/* -------------------------------------------------------------------------
 * CMD: SET
 * ------------------------------------------------------------------------- */
static int cmd_SET(char *ptr)
{
    char *token;
    int result = extractString(ptr, &ptr, NULL, &token, EXTRACT_FLAG_AUTO_TERMINATE);
    if (result != FR_OK) return result;

    mos_trim(ptr, false, false);
    if (!ptr || *ptr == '\0') return FR_INVALID_PARAMETER;

    char *newValue = expandMacro(ptr);
    if (!newValue) return FR_INT_ERR;
    result = createOrUpdateSystemVariable(token, MOS_VAR_STRING, newValue);
    free(newValue);
    return result;
}

/* -------------------------------------------------------------------------
 * CMD: SETEVAL
 * ------------------------------------------------------------------------- */
static int cmd_SETEVAL(char *ptr)
{
    char *token;
    int result = extractString(ptr, &ptr, NULL, &token, EXTRACT_FLAG_AUTO_TERMINATE);
    if (result != FR_OK) return result;

    while (isspace((unsigned char)*ptr)) ptr++;
    if (!*ptr) return FR_INVALID_PARAMETER;

    t_mosEvalResult *eval = evaluateExpression(ptr);
    if (!eval) return FR_INT_ERR;

    if (eval->status != FR_OK) {
        result = eval->status;
    } else {
        result = createOrUpdateSystemVariable(token, eval->type, eval->result);
        if (eval->type == MOS_VAR_STRING) free(eval->result);
    }
    free(eval);
    return result;
}

/* -------------------------------------------------------------------------
 * CMD: SETMACRO
 * ------------------------------------------------------------------------- */
static int cmd_SETMACRO(char *ptr)
{
    char *token;
    int result = extractString(ptr, &ptr, NULL, &token, EXTRACT_FLAG_AUTO_TERMINATE);
    if (result != FR_OK) return result;

    while (isspace((unsigned char)*ptr)) ptr++;
    if (!*ptr) return FR_INVALID_PARAMETER;

    return createOrUpdateSystemVariable(token, MOS_VAR_MACRO, ptr);
}

/* -------------------------------------------------------------------------
 * CMD: SHOW
 * ------------------------------------------------------------------------- */
static int cmd_SHOW(char *ptr)
{
    t_mosSystemVariable *var = NULL;
    char *token;
    int search = extractString(ptr, &ptr, NULL, &token, EXTRACT_FLAG_AUTO_TERMINATE);
    if (search != FR_OK) token = "*";

    while (getSystemVariable(token, &var) == 0) {
        mos_printf("%s", var->label);
        switch (var->type) {
            case MOS_VAR_MACRO:
                mos_printf("(Macro) : ");
                printEscapedString((char *)var->value);
                mos_printf("\r\n");
                break;
            case MOS_VAR_NUMBER:
                mos_printf("(Number) : %d\r\n", (int)(intptr_t)var->value);
                break;
            case MOS_VAR_CODE: {
                char *val = expandVariable(var, true);
                if (!val) {
                    mos_printf(" : Error fetching code variable\r\n");
                } else {
                    mos_printf(" : %s\r\n", val);
                    free(val);
                }
                break;
            }
            default:
                mos_printf(" : ");
                printEscapedString((char *)var->value);
                mos_printf("\r\n");
                break;
        }
    }
    return FR_OK;
}

/* -------------------------------------------------------------------------
 * CMD: TIME
 * ------------------------------------------------------------------------- */
static int cmd_TIME(char *ptr)
{
    int yr, mo, da, ho, mi, se;

    if (ptr && strlen(ptr) > 0) {
        if (!extractNumber(ptr, &ptr, NULL, &yr, EXTRACT_NUM_DECIMAL_ONLY | EXTRACT_NUM_POSITIVE_ONLY) ||
            !extractNumber(ptr, &ptr, NULL, &mo, EXTRACT_NUM_DECIMAL_ONLY | EXTRACT_NUM_POSITIVE_ONLY) ||
            !extractNumber(ptr, &ptr, NULL, &da, EXTRACT_NUM_DECIMAL_ONLY | EXTRACT_NUM_POSITIVE_ONLY) ||
            !extractNumber(ptr, &ptr, NULL, &ho, EXTRACT_NUM_DECIMAL_ONLY | EXTRACT_NUM_POSITIVE_ONLY) ||
            !extractNumber(ptr, &ptr, NULL, &mi, EXTRACT_NUM_DECIMAL_ONLY | EXTRACT_NUM_POSITIVE_ONLY) ||
            !extractNumber(ptr, &ptr, NULL, &se, EXTRACT_NUM_DECIMAL_ONLY | EXTRACT_NUM_POSITIVE_ONLY)) {
            return FR_INVALID_PARAMETER;
        }
        struct tm t = {0};
        t.tm_year = yr - 1900;
        t.tm_mon  = mo - 1;
        t.tm_mday = da;
        t.tm_hour = ho;
        t.tm_min  = mi;
        t.tm_sec  = se;
        time_t epoch = mktime(&t);
        struct timeval tv = { .tv_sec = epoch, .tv_usec = 0 };
        settimeofday(&tv, NULL);
    }

    time_t now = time(NULL);
    struct tm *t = localtime(&now);
    char buf[64];
    strftime(buf, sizeof(buf), "%a,%d %b %Y.%H:%M:%S", t);
    mos_printf("%s\r\n", buf);
    return FR_OK;
}

/* -------------------------------------------------------------------------
 * CMD: TRY
 * ------------------------------------------------------------------------- */
static int cmd_TRY(char *ptr)
{
    int result = mos_exec(ptr);
    createOrUpdateSystemVariable("Try$ReturnCode", MOS_VAR_NUMBER,
                                 (void *)(intptr_t)result);
    if (result > 0 && result < (int)ERRORS_COUNT) {
        createOrUpdateSystemVariable("Try$Error", MOS_VAR_STRING,
                                     (void *)s_errors[result]);
    }
    return FR_OK;
}

/* -------------------------------------------------------------------------
 * CMD: TYPE
 * ------------------------------------------------------------------------- */
static int cmd_TYPE(char *ptr)
{
    char *filename;
    int result = extractString(ptr, &ptr, NULL, &filename, EXTRACT_FLAG_AUTO_TERMINATE);
    if (result != FR_OK) return result;

    char resolved[MOS_PATH_MAX];
    mos_fs_resolve(filename, resolved, sizeof(resolved));

    FILE *f = fopen(resolved, "r");
    if (!f) {
        mos_printf("TYPE: cannot open '%s': %s\r\n", resolved, strerror(errno));
        return FR_NO_FILE;
    }
    char buf[128];
    while (fgets(buf, sizeof(buf), f)) {
        mos_puts(buf);
    }
    fclose(f);
    return FR_OK;
}

/* -------------------------------------------------------------------------
 * CMD: UNSET
 * ------------------------------------------------------------------------- */
static int cmd_UNSET(char *ptr)
{
    char *token;
    t_mosSystemVariable *var = NULL;
    int result = extractString(ptr, &ptr, NULL, &token, EXTRACT_FLAG_AUTO_TERMINATE);
    if (result != FR_OK) return result;

    while (getSystemVariable(token, &var) == 0) {
        if (var->type != MOS_VAR_CODE) {
            removeSystemVariable(var);
            var = NULL;
        }
    }
    return FR_OK;
}

/* -------------------------------------------------------------------------
 * CMD: RUN — load and execute a flat binary from filesystem
 * ------------------------------------------------------------------------- */
static int cmd_RUN(char *ptr)
{
    char *filename;
    int result = extractString(ptr, &ptr, NULL, &filename, EXTRACT_FLAG_AUTO_TERMINATE);
    if (result != FR_OK) {
        mos_printf("Usage: RUN <file> [args]\r\n");
        return result;
    }

    /* Build argv: argv[0] = filename, rest from remaining ptr */
    #define RUN_MAX_ARGS 16
    char *argv[RUN_MAX_ARGS];
    int   argc = 0;
    argv[argc++] = filename;

    /* Tokenise remaining arguments */
    char *arg;
    while (argc < RUN_MAX_ARGS - 1) {
        if (extractString(ptr, &ptr, NULL, &arg, EXTRACT_FLAG_AUTO_TERMINATE) != FR_OK) break;
        argv[argc++] = arg;
    }
    argv[argc] = NULL;

    int ret = mos_loader_exec(filename, argc, argv);
    if (ret < 0) {
        mos_printf("RUN: failed to load '%s'\r\n", filename);
        return FR_INT_ERR;
    }
    if (ret != 0) {
        mos_printf("RUN: program exited with code %d\r\n", ret);
    }
    return FR_OK;
}

/* -------------------------------------------------------------------------
 * mos_shell_init
 * ------------------------------------------------------------------------- */
void mos_shell_init(void)
{
    mos_sysvars_init();
    mos_editor_init();
}

/* -------------------------------------------------------------------------
 * mos_shell_exec  — public entry for single command execution
 * ------------------------------------------------------------------------- */
int mos_shell_exec(const char *line)
{
    if (!line || !*line) return 0;
    /* mos_exec modifies the string — make a mutable copy */
    char *buf = strdup(line);
    if (!buf) return MOS_OUT_OF_MEMORY;
    int r = mos_exec(buf);
    free(buf);
    return r;
}

/* -------------------------------------------------------------------------
 * mos_shell_run  — interactive shell loop
 * ------------------------------------------------------------------------- */
void mos_shell_run(void)
{
    char line[MOS_EDITOR_MAX_LINE];

    while (1) {
        /* Build prompt: "<drive>:<cwd>* " */
        char *prompt_var = expandVariableToken("CLI$Prompt");
        char full_prompt[160];
        snprintf(full_prompt, sizeof(full_prompt), "%c%s%s ",
                 mos_fs_getdrive(),
                 mos_fs_getcwd(),
                 prompt_var ? prompt_var : ".*");
        free(prompt_var);

        int r = mos_editor_readline(full_prompt, line, sizeof(line));
        if (r < 0) return;  /* VDP disconnected (or EOF) — end this session */
        if (!*line) continue;

        int result = mos_exec(line);
        if (result != FR_OK && result != MOS_INVALID_COMMAND) {
            mos_error(result);
        }
    }
}
