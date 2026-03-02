/*
 * mos_sysvars_block.h - Binary system variable block (agon-mos compatible)
 *
 * Standalone header with no ESP-IDF or component dependencies.
 * Included by both mos_api_table.h (firmware + SDK) and mos_vdp.h (firmware).
 *
 * The struct layout mirrors agon-mos globals.asm sysvar_* offsets exactly,
 * verified at compile-time by _Static_assert in mos_vdp.c.
 */

#ifndef MOS_SYSVARS_BLOCK_H
#define MOS_SYSVARS_BLOCK_H

#include <stdint.h>

/**
 * Binary system variable block — mirrors the agon-mos sysvar layout.
 * Obtain a pointer via mos->sysvars(); fields are updated live by the VDP parser.
 *
 * Offsets match agon-mos globals.asm sysvar_* constants:
 *   sysvar_time         0x00  sysvar_vpd_pflags  0x04  sysvar_keyascii    0x05
 *   sysvar_keymods      0x06  sysvar_cursorX     0x07  sysvar_cursorY     0x08
 *   sysvar_scrchar      0x09  sysvar_scrpixel    0x0A  sysvar_audioChannel 0x0D
 *   sysvar_audioSuccess 0x0E  sysvar_scrWidth    0x0F  sysvar_scrHeight   0x11
 *   sysvar_scrCols      0x13  sysvar_scrRows     0x14  sysvar_scrColours  0x15
 *   sysvar_scrpixelIndex 0x16 sysvar_vkeycode    0x17  sysvar_vkeydown    0x18
 *   sysvar_vkeycount    0x19  sysvar_rtc         0x1A  sysvar_keydelay    0x20
 *   sysvar_keyrate      0x22  sysvar_keyled      0x24
 */
typedef struct __attribute__((packed)) t_mos_sysvars_s {
    uint32_t time;           /* 0x00 +4: centiseconds since boot (~10 Hz timer) */
    uint8_t  vpd_pflags;     /* 0x04 +1: VDP protocol completion flags          */
    uint8_t  keyascii;       /* 0x05 +1: last ASCII keycode received             */
    uint8_t  keymods;        /* 0x06 +1: modifier keys (shift/ctrl/alt bitmask) */
    uint8_t  cursorX;        /* 0x07 +1: cursor column                          */
    uint8_t  cursorY;        /* 0x08 +1: cursor row                             */
    uint8_t  scrchar;        /* 0x09 +1: character at cursor (SCRCHAR packet)   */
    uint8_t  scrpixel[3];    /* 0x0A +3: pixel R,G,B (SCRPIXEL packet)         */
    uint8_t  audioChannel;   /* 0x0D +1: audio channel                         */
    uint8_t  audioSuccess;   /* 0x0E +1: audio note queued (0=no, 1=yes)       */
    uint16_t scrWidth;       /* 0x0F +2: screen width in pixels                */
    uint16_t scrHeight;      /* 0x11 +2: screen height in pixels               */
    uint8_t  scrCols;        /* 0x13 +1: screen columns in characters          */
    uint8_t  scrRows;        /* 0x14 +1: screen rows in characters             */
    uint8_t  scrColours;     /* 0x15 +1: number of colours                     */
    uint8_t  scrpixelIndex;  /* 0x16 +1: palette index of last pixel read      */
    uint8_t  vkeycode;       /* 0x17 +1: virtual key code                      */
    uint8_t  vkeydown;       /* 0x18 +1: virtual key state (0=up, 1=down)      */
    uint8_t  vkeycount;      /* 0x19 +1: incremented every key packet          */
    uint8_t  rtc[6];         /* 0x1A +6: packed RTC union (vdp_time_t)         */
    uint16_t keydelay;       /* 0x20 +2: keyboard repeat delay                 */
    uint16_t keyrate;        /* 0x22 +2: keyboard repeat rate                  */
    uint8_t  keyled;         /* 0x24 +1: keyboard LED status                   */
} t_mos_sysvars;

#endif /* MOS_SYSVARS_BLOCK_H */
