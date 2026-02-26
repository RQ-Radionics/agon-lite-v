/*
 * mos_editor.c - Interactive line editor for ESP32-MOS
 *
 * Implements a readline-style editor with:
 *   - Character-by-character input via mos_getch()
 *   - Cursor movement (left/right arrows)
 *   - History ring buffer (up/down arrows)
 *   - Delete/backspace
 *   - VT100/ANSI escape sequences for cursor control
 *
 * Ported from agon-mos src/mos_editor.c
 */

#include <string.h>
#include <stdio.h>
#include "mos_editor.h"
#include "mos_hal.h"

/* --- History ring buffer --- */
static char s_history[MOS_EDITOR_HISTORY_LEN][MOS_EDITOR_MAX_LINE];
static int  s_hist_count = 0;   /* number of valid entries */
static int  s_hist_head  = 0;   /* index of next write slot */

static void history_add(const char *line)
{
    if (!line || !*line) return;
    /* Don't duplicate last entry */
    int prev = (s_hist_head - 1 + MOS_EDITOR_HISTORY_LEN) % MOS_EDITOR_HISTORY_LEN;
    if (s_hist_count > 0 && strcmp(s_history[prev], line) == 0) return;

    strncpy(s_history[s_hist_head], line, MOS_EDITOR_MAX_LINE - 1);
    s_history[s_hist_head][MOS_EDITOR_MAX_LINE - 1] = '\0';
    s_hist_head = (s_hist_head + 1) % MOS_EDITOR_HISTORY_LEN;
    if (s_hist_count < MOS_EDITOR_HISTORY_LEN) s_hist_count++;
}

static const char *history_get(int offset)
{
    /* offset=1 → last entry, offset=2 → second-to-last, etc. */
    if (offset < 1 || offset > s_hist_count) return NULL;
    int idx = (s_hist_head - offset + MOS_EDITOR_HISTORY_LEN) % MOS_EDITOR_HISTORY_LEN;
    return s_history[idx];
}

/* --- VT100 helpers --- */
static void cursor_left(int n)  { if (n > 0) mos_printf("\033[%dD", n); }
static void cursor_right(int n) { if (n > 0) mos_printf("\033[%dC", n); }
static void erase_to_eol(void)  { mos_puts("\033[K"); }

/* --- Public API --- */

void mos_editor_init(void)
{
    memset(s_history, 0, sizeof(s_history));
    s_hist_count = 0;
    s_hist_head  = 0;
}

int mos_editor_readline(const char *prompt, char *buf, size_t buf_size)
{
    if (!buf || buf_size < 2) return -1;

    if (prompt) mos_puts(prompt);

    char line[MOS_EDITOR_MAX_LINE];
    int  len    = 0;    /* current length */
    int  cursor = 0;    /* cursor position within line */
    int  hist_offset = 0;  /* 0 = editing current, 1+ = browsing history */
    char saved[MOS_EDITOR_MAX_LINE] = "";  /* saved current line while browsing */

    memset(line, 0, sizeof(line));

    while (1) {
        int c = mos_getch();
        if (c < 0) { buf[0] = '\0'; return -1; }

        if (c == '\r' || c == '\n') {
            /* Enter */
            mos_puts("\r\n");
            line[len] = '\0';
            if (len > 0) history_add(line);
            size_t copy = (len < (int)buf_size - 1) ? (size_t)len : buf_size - 1;
            memcpy(buf, line, copy);
            buf[copy] = '\0';
            return (int)copy;
        }

        if (c == 0x7F || c == '\b') {
            /* Backspace / DEL */
            if (cursor > 0) {
                memmove(&line[cursor - 1], &line[cursor], len - cursor);
                cursor--;
                len--;
                line[len] = '\0';
                /* Redraw from cursor */
                cursor_left(1);
                mos_puts(&line[cursor]);
                mos_putch(' ');
                cursor_left(len - cursor + 1);
            }
            continue;
        }

        if (c == 0x1B) {
            /* Escape sequence */
            int c2 = mos_getch();
            if (c2 == '[') {
                int c3 = mos_getch();
                switch (c3) {
                    case 'A': /* Up arrow — history back */
                        if (hist_offset == 0) {
                            strncpy(saved, line, MOS_EDITOR_MAX_LINE - 1);
                        }
                        if (hist_offset < s_hist_count) {
                            hist_offset++;
                            const char *h = history_get(hist_offset);
                            if (h) {
                                /* Erase current line and replace */
                                cursor_left(cursor);
                                erase_to_eol();
                                strncpy(line, h, MOS_EDITOR_MAX_LINE - 1);
                                len = cursor = (int)strlen(line);
                                mos_puts(line);
                            }
                        }
                        break;
                    case 'B': /* Down arrow — history forward */
                        if (hist_offset > 0) {
                            hist_offset--;
                            cursor_left(cursor);
                            erase_to_eol();
                            if (hist_offset == 0) {
                                strncpy(line, saved, MOS_EDITOR_MAX_LINE - 1);
                            } else {
                                const char *h = history_get(hist_offset);
                                if (h) strncpy(line, h, MOS_EDITOR_MAX_LINE - 1);
                            }
                            len = cursor = (int)strlen(line);
                            mos_puts(line);
                        }
                        break;
                    case 'C': /* Right arrow */
                        if (cursor < len) {
                            cursor_right(1);
                            cursor++;
                        }
                        break;
                    case 'D': /* Left arrow */
                        if (cursor > 0) {
                            cursor_left(1);
                            cursor--;
                        }
                        break;
                    case '3': { /* DEL key: ESC [ 3 ~ */
                        int c4 = mos_getch();
                        if (c4 == '~' && cursor < len) {
                            memmove(&line[cursor], &line[cursor + 1], len - cursor - 1);
                            len--;
                            line[len] = '\0';
                            mos_puts(&line[cursor]);
                            mos_putch(' ');
                            cursor_left(len - cursor + 1);
                        }
                        break;
                    }
                    case 'H': /* Home */
                        cursor_left(cursor);
                        cursor = 0;
                        break;
                    case 'F': /* End */
                        cursor_right(len - cursor);
                        cursor = len;
                        break;
                    default:
                        break;
                }
            }
            continue;
        }

        if (c == 0x01) { /* Ctrl-A: Home */
            cursor_left(cursor);
            cursor = 0;
            continue;
        }
        if (c == 0x05) { /* Ctrl-E: End */
            cursor_right(len - cursor);
            cursor = len;
            continue;
        }
        if (c == 0x0B) { /* Ctrl-K: kill to end of line */
            erase_to_eol();
            len = cursor;
            line[len] = '\0';
            continue;
        }
        if (c == 0x15) { /* Ctrl-U: kill whole line */
            cursor_left(cursor);
            erase_to_eol();
            len = cursor = 0;
            line[0] = '\0';
            continue;
        }

        /* Printable character */
        if (c >= 0x20 && c < 0x7F && len < (int)buf_size - 2) {
            hist_offset = 0; /* stop browsing history on edit */
            memmove(&line[cursor + 1], &line[cursor], len - cursor);
            line[cursor] = (char)c;
            cursor++;
            len++;
            line[len] = '\0';
            mos_putch((char)c);
            if (cursor < len) {
                /* Redraw tail and reposition cursor */
                mos_puts(&line[cursor]);
                cursor_left(len - cursor);
            }
        }
    }
}
