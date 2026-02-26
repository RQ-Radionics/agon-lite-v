/*
 * mos_hal.h - Hardware Abstraction Layer for ESP32-MOS
 *
 * Replaces uart.c/h, timer.c/h from the eZ80 port.
 * Console I/O is via ESP-IDF VFS (stdin/stdout).
 */

#ifndef MOS_HAL_H
#define MOS_HAL_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "mos_types.h"
#include "mos_version.h"

/* --- Console I/O --- */

/**
 * Initialise the console UART (called once from app_main).
 * Sets up line endings, disables buffering on stdin.
 */
void mos_hal_console_init(void);

/**
 * Write a single character to the console.
 */
void mos_putch(char c);

/**
 * Read a single character from the console (blocking).
 * Returns the character read.
 */
int mos_getch(void);

/**
 * Check if a character is available on the console (non-blocking).
 * Returns true if a character is waiting.
 */
bool mos_kbhit(void);

/**
 * Write a null-terminated string to the console.
 */
void mos_puts(const char *s);

/**
 * printf-style output to the console.
 */
int mos_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

#endif /* MOS_HAL_H */
