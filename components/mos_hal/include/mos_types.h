/*
 * mos_types.h - Portable types for ESP32-MOS
 *
 * Replaces the eZ80-specific defines.h and Zilog types.
 * All code should include this instead of the original defines.h.
 */

#ifndef MOS_TYPES_H
#define MOS_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* MOS-specific return codes (extend FatFS FRESULT which ends at ~19) */
typedef enum {
    MOS_OK                  = 0,
    MOS_INVALID_COMMAND     = 20,   /* Command could not be understood */
    MOS_INVALID_EXECUTABLE  = 21,   /* Executable format not recognised */
    MOS_OUT_OF_MEMORY       = 22,   /* Generic out of memory error */
    MOS_NOT_IMPLEMENTED     = 23,   /* API call not implemented */
    MOS_BAD_STRING          = 24,   /* Bad or incomplete string */
    MOS_TOO_DEEP            = 25,   /* Too many nested commands */
} MOSRESULT;

/*
 * FatFS-compatible result codes used throughout the port.
 * The original agon-mos uses FatFS FRESULT values; we map them to plain
 * integer constants so the port compiles without linking FatFS.
 */
#define FR_OK               0
#define FR_INT_ERR          2
#define FR_NOT_READY        3
#define FR_NO_FILE          4
#define FR_NO_PATH          5
#define FR_INVALID_NAME     6
#define FR_INVALID_PARAMETER 19

/* Maximum simultaneously open files */
#define MOS_MAX_OPEN_FILES      8

/* Debug flag (0 = production) */
#define DEBUG                   0

#endif /* MOS_TYPES_H */
