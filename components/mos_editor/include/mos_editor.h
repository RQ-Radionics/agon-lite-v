/*
 * mos_editor.h - Interactive line editor for ESP32-MOS
 * Ported from agon-mos src/mos_editor.h
 */

#ifndef MOS_EDITOR_H
#define MOS_EDITOR_H

#include <stddef.h>

#define MOS_EDITOR_MAX_LINE     256
#define MOS_EDITOR_HISTORY_LEN  16

void mos_editor_init(void);
int  mos_editor_readline(const char *prompt, char *buf, size_t buf_size);

/** Clear the command history (called on session reset). */
void mos_editor_reset(void);

#endif /* MOS_EDITOR_H */
