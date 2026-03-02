/*
 * vdp_font.h — Agon BBC Micro 8x8 font
 *
 * 224 characters, ASCII 0x20..0xFF, 8 bytes per character.
 * Bit 7 of each byte = leftmost pixel, bit 0 = rightmost.
 */
#pragma once
#include <stdint.h>

/* 224 glyphs × 8 bytes, covering ASCII 0x20 (space) through 0xFF */
extern const uint8_t vdp_font_8x8[224 * 8];

/* Returns pointer to 8-byte glyph for ASCII code c (0x20..0xFF).
 * Returns pointer to space glyph for out-of-range codes. */
static inline const uint8_t *vdp_font_glyph(uint8_t c)
{
    if (c < 0x20) c = 0x20;
    return &vdp_font_8x8[(c - 0x20) * 8];
}
