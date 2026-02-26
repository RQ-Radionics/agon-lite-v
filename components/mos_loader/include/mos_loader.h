/*
 * mos_loader.h - User program loader for ESP32-MOS
 *
 * Loads a flat binary from the FAT filesystem into PSRAM and executes it.
 *
 * Binary format (flat, position-dependent):
 *   - Compiled with xtensa-esp32s3-elf-gcc, -nostdlib, no startup files
 *   - Linked with MOS_EXEC_BASE as the text start address
 *   - Entry point symbol: _start
 *   - Entry point prototype:
 *       int _start(int argc, char **argv, t_mos_api *mos);
 *
 * The loader:
 *   1. Resolves the path via mos_fs
 *   2. Allocates PSRAM with heap_caps_malloc(MALLOC_CAP_SPIRAM)
 *   3. Reads the binary into PSRAM
 *   4. Flushes instruction cache
 *   5. Calls entry(argc, argv, mos_api_table_get())
 *   6. Frees PSRAM after the program returns
 */

#ifndef MOS_LOADER_H
#define MOS_LOADER_H

#include <stdint.h>

/* Load address in PSRAM where user programs are placed.
 * Must be within the mapped PSRAM window on ESP32-S3.
 * Adjust if your board has PSRAM at a different base. */
#define MOS_EXEC_BASE   0x3C000000UL

/**
 * Load and run a flat binary from the filesystem.
 *
 * @param path  Path to the binary file (e.g. "A:/helloworld.bin")
 * @param argc  Argument count to pass to the program
 * @param argv  Argument vector to pass to the program
 * @return      Exit code returned by the program, or negative on load error.
 */
int mos_loader_exec(const char *path, int argc, char **argv);

#endif /* MOS_LOADER_H */
