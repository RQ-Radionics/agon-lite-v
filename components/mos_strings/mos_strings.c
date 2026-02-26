/*
 * mos_strings.c - String utilities for ESP32-MOS
 * Ported from agon-mos src/strings.c
 * Original authors: Leigh Brown, HeathenUK, and others
 *
 * Note: strcasecmp and strncasecmp are provided by ESP-IDF libc,
 * strdup/strndup are available in newlib, so only the MOS-specific
 * helpers and pmatch are needed here.
 */

#include <stdbool.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include "mos_strings.h"

/*
 * Case-insensitive substring search.
 * Returns pointer to first occurrence of substr in str, or NULL.
 */
char *stristr(const char *str, const char *substr)
{
    int c = tolower((unsigned char)*substr);
    if (c == '\0') {
        return (char *)str;
    }
    for (; *str; str++) {
        if (tolower((unsigned char)*str) == c) {
            int i;
            for (i = 0;;) {
                if (substr[++i] == '\0') {
                    return (char *)str;
                }
                if (tolower((unsigned char)str[i]) != tolower((unsigned char)substr[i])) {
                    break;
                }
            }
        }
    }
    return NULL;
}

/*
 * strnlen — count at most maxlen characters.
 * (Available in newlib but provided for portability.)
 */
size_t mos_strnlen(const char *s, size_t maxlen)
{
    size_t len = 0;
    while (len < maxlen && s[len] != '\0') {
        len++;
    }
    return len;
}

/*
 * strdup using standard malloc.
 * (newlib provides strdup, but we expose mos_strdup for callers that
 *  previously used umm_malloc-backed strdup.)
 */
char *mos_strdup(const char *s)
{
    char *d = malloc(strlen(s) + 1);
    if (d != NULL) {
        strcpy(d, s);
    }
    return d;
}

/*
 * strcspn — span of characters NOT in reject.
 */
size_t mos_strcspn(const char *s, const char *reject)
{
    const char *p = s;
    while (*p != '\0') {
        if (strchr(reject, *p) != NULL) {
            break;
        }
        p++;
    }
    return p - s;
}

/*
 * strspn — span of characters in accept.
 */
size_t mos_strspn(const char *s, const char *accept)
{
    const char *p = s;
    while (*p != '\0') {
        if (strchr(accept, *p) == NULL) {
            break;
        }
        p++;
    }
    return p - s;
}

/*
 * strndup using standard malloc.
 */
char *mos_strndup(const char *s, size_t n)
{
    size_t len = mos_strnlen(s, n);
    char *d = malloc(len + 1);
    if (d != NULL) {
        strncpy(d, s, len);
        d[len] = '\0';
    }
    return d;
}

/*
 * pmatch — wildcard pattern matching.
 *
 * Flags (see mos_strings.h):
 *   MATCH_CASE_INSENSITIVE  — ignore case
 *   MATCH_DISABLE_STAR      — treat '*' as literal
 *   MATCH_DISABLE_HASH      — treat '#' as literal
 *   MATCH_DOT_AS_STAR       — '.' at end of pattern = one-or-more chars
 *   MATCH_BEGINS_WITH       — match if string starts with pattern
 *   MATCH_UP_TO_SPACE       — stop comparing pattern at first space
 *
 * Returns 0 on match, non-zero otherwise (like strcmp).
 */
int pmatch(const char *pattern, const char *string, uint8_t flags)
{
    bool caseInsensitive = (flags & MATCH_CASE_INSENSITIVE) != 0;
    bool disableStar     = (flags & MATCH_DISABLE_STAR)     != 0;
    bool disableHash     = (flags & MATCH_DISABLE_HASH)     != 0;
    bool dotAsStar       = (flags & MATCH_DOT_AS_STAR)      != 0;
    bool beginsWith      = (flags & MATCH_BEGINS_WITH)      != 0;
    bool upToSpace       = (flags & MATCH_UP_TO_SPACE)      != 0;

    if (*pattern == '\0' ||
        (upToSpace && *pattern == ' ') ||
        (beginsWith && dotAsStar && *pattern == '.')) {
        /* Pattern exhausted: success if beginsWith, or string also exhausted */
        return (beginsWith || *string == '\0') ? 0 : -1;
    } else if (*pattern == '.' && dotAsStar && *(pattern + 1) == '\0') {
        /* Dot-as-star at end: matches one-or-more chars */
        return (*string == '\0') ? -1 : 0;
    } else if (*pattern == '*' && !disableStar) {
        /* '*' wildcard: skip it and try matching rest, or consume one char */
        if (pmatch(pattern + 1, string, flags) == 0 ||
            (*string && pmatch(pattern, string + 1, flags) == 0)) {
            return 0;
        }
        return -1;
    } else {
        /* Literal character or '#' single-char wildcard */
        char patternChar = caseInsensitive ? (char)tolower((unsigned char)*pattern) : *pattern;
        char stringChar  = caseInsensitive ? (char)tolower((unsigned char)*string)  : *string;

        if ((*pattern == '#' && !disableHash) || patternChar == stringChar) {
            return pmatch(pattern + 1, string + 1, flags);
        }

        return (int)(unsigned char)stringChar - (int)(unsigned char)patternChar;
    }
}
