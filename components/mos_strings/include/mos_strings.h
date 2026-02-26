/*
 * mos_strings.h - String utilities for ESP32-MOS
 * Ported from agon-mos src/strings.h
 */

#ifndef MOS_STRINGS_H
#define MOS_STRINGS_H

#include <stdint.h>
#include <stddef.h>

/* pmatch flags */
#define MATCH_CASE_INSENSITIVE  0x01
#define MATCH_DISABLE_STAR      0x02
#define MATCH_DISABLE_HASH      0x04
#define MATCH_DOT_AS_STAR       0x08
#define MATCH_BEGINS_WITH       0x10
#define MATCH_UP_TO_SPACE       0x20

/* Convenience combos */
#define MATCH_WILDCARDS         0x00    /* default: star+hash enabled */
#define MATCH_COMMANDS          (MATCH_CASE_INSENSITIVE)

/**
 * Pattern match (recursive, supports * glob, # single-char wildcard,
 * dot-as-star, begins-with, case-insensitive, up-to-space).
 * Returns 0 on match, non-zero otherwise.
 */
int pmatch(const char *pattern, const char *string, uint8_t flags);

/** Case-insensitive strstr */
char *stristr(const char *str, const char *substr);

/** strnlen (missing in some toolchains) */
size_t mos_strnlen(const char *s, size_t maxlen);

/** strdup using malloc */
char *mos_strdup(const char *s);

/** strndup using malloc */
char *mos_strndup(const char *s, size_t n);

/** strcspn wrapper */
size_t mos_strcspn(const char *s, const char *reject);

/** strspn wrapper */
size_t mos_strspn(const char *s, const char *accept);

#endif /* MOS_STRINGS_H */
