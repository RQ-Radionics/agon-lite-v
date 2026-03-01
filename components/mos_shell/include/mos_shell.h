/*
 * mos_shell.h - MOS shell for ESP32-MOS
 * Ported from agon-mos src/mos.h
 */

#ifndef MOS_SHELL_H
#define MOS_SHELL_H

/**
 * Initialise the shell (variables, history, etc.)
 */
void mos_shell_init(void);

/**
 * Reset shell session state: clear system variables (except CODE vars),
 * reset command history, reset cwd to flash root, reset nesting depth.
 * Called when the VDP client disconnects so the next session starts clean.
 */
void mos_shell_reset(void);

/**
 * Execute a single command line string.
 * Returns 0 on success, non-zero on error.
 */
int mos_shell_exec(const char *line);

/**
 * Run the interactive shell loop (never returns).
 */
void mos_shell_run(void);

#endif /* MOS_SHELL_H */
