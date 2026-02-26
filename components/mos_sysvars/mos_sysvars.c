/*
 * mos_sysvars.c - System variables for ESP32-MOS
 * Ported from agon-mos src/mos_sysvars.c
 *
 * Changes from original:
 *  - umm_malloc/umm_free replaced by malloc/free
 *  - BYTE replaced by uint8_t
 *  - FR_* codes defined in mos_types.h
 *  - mos_file.h dependency (for cwd) replaced by mos_fs.h
 *  - isspace() from <ctype.h>
 */

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "mos_sysvars.h"
#include "mos_strings.h"

/* Global linked list of system variables (sorted by label) */
t_mosSystemVariable  *mosSystemVariables = NULL;
/* Tracked (outermost) GSTrans chain */
t_mosTransInfo       *trackedTransInfo   = NULL;

/* =========================================================================
 * mos_sysvars_init
 * Register built-in default variables.
 * ========================================================================= */
void mos_sysvars_init(void)
{
    createOrUpdateSystemVariable("CLI$Prompt",    MOS_VAR_LITERAL, (void *)"*");
    createOrUpdateSystemVariable("CLI$AutoPaged", MOS_VAR_LITERAL, (void *)"0");
    createOrUpdateSystemVariable("Moslet$Path",   MOS_VAR_LITERAL, (void *)"/flash/mos/");
}

/* =========================================================================
 * getSystemVariable
 * Search for a variable matching pattern (case-insensitive, up to space).
 *
 * On entry:
 *   *var == NULL  → start from beginning of list
 *   *var != NULL  → start from the node AFTER *var
 *
 * On exit:
 *   *var → variable found (or last node before where it would be inserted)
 *   returns 0 on exact match, <0 if not found / partial match
 * ========================================================================= */
int getSystemVariable(char *pattern, t_mosSystemVariable **var)
{
    t_mosSystemVariable *current;
    int matchResult = -1;

    if (*var == NULL) {
        current = mosSystemVariables;
    } else {
        current = (*var)->next;
    }
    *var = NULL;

    while (current != NULL) {
        matchResult = pmatch(pattern, current->label,
                             MATCH_CASE_INSENSITIVE | MATCH_UP_TO_SPACE);
        if (matchResult <= 0) {
            *var = current;
        }
        if (matchResult == 0) {
            break;
        }
        if (matchResult > 0) {
            /* List is sorted; no point going further */
            break;
        }
        current = current->next;
    }

    return (*var != NULL) ? matchResult : -1;
}

/* =========================================================================
 * createSystemVariable
 * Allocate and initialise a new system variable node.
 * ========================================================================= */
t_mosSystemVariable *createSystemVariable(char *label, MOSVARTYPE type, void *value)
{
    t_mosSystemVariable *newVar = malloc(sizeof(t_mosSystemVariable));
    char *newLabel = mos_strdup(label);

    if (newVar == NULL || newLabel == NULL) {
        free(newVar);
        free(newLabel);
        return NULL;
    }

    newVar->label = newLabel;
    newVar->type  = type;
    newVar->next  = NULL;

    if (type == MOS_VAR_MACRO || type == MOS_VAR_STRING) {
        newVar->value = mos_strdup((char *)value);
    } else {
        newVar->value = value;
    }

    return newVar;
}

/* =========================================================================
 * insertSystemVariable
 * Insert var into the list after `before` (or at head if before == NULL).
 * ========================================================================= */
void insertSystemVariable(t_mosSystemVariable *var, t_mosSystemVariable *before)
{
    if (before == NULL) {
        var->next            = mosSystemVariables;
        mosSystemVariables   = var;
    } else {
        var->next    = before->next;
        before->next = var;
    }
}

/* =========================================================================
 * createOrUpdateSystemVariable
 * ========================================================================= */
int createOrUpdateSystemVariable(char *label, MOSVARTYPE type, void *value)
{
    t_mosSystemVariable *var = NULL;
    int result = getSystemVariable(label, &var);

    if (result == 0) {
        return updateSystemVariable(var, type, value);
    } else {
        t_mosSystemVariable *newVar = createSystemVariable(label, type, value);
        if (newVar != NULL) {
            insertSystemVariable(newVar, var);
            return FR_OK;
        }
    }
    return FR_INT_ERR;
}

/* =========================================================================
 * updateSystemVariable
 * ========================================================================= */
int updateSystemVariable(t_mosSystemVariable *var, MOSVARTYPE type, void *value)
{
    MOSVARTYPE oldType = var->type;

    if (oldType == MOS_VAR_CODE) {
        /* Delegate to setter function, if available */
        if (((t_mosCodeSystemVariable *)var->value)->write != NULL) {
            return ((t_mosCodeSystemVariable *)var->value)->write((char *)value);
        }
        /* Read-only: silently ignore */
        return FR_OK;
    }

    var->type = type;
    if (type == MOS_VAR_MACRO || type == MOS_VAR_STRING) {
        char *newValue = mos_strdup((char *)value);
        if (newValue == NULL) {
            return MOS_OUT_OF_MEMORY;
        }
        if (oldType == MOS_VAR_MACRO || oldType == MOS_VAR_STRING) {
            free(var->value);
        }
        var->value = newValue;
    } else {
        var->value = value;
    }
    return FR_OK;
}

/* =========================================================================
 * removeSystemVariable
 * ========================================================================= */
void removeSystemVariable(t_mosSystemVariable *var)
{
    t_mosSystemVariable *parent = findParentSystemVariable(var);
    if (parent == NULL) {
        mosSystemVariables = var->next;
    } else {
        parent->next = var->next;
    }
    free(var->label);
    if (var->type == MOS_VAR_MACRO || var->type == MOS_VAR_STRING) {
        free(var->value);
    }
    free(var);
}

/* =========================================================================
 * findParentSystemVariable
 * ========================================================================= */
t_mosSystemVariable *findParentSystemVariable(t_mosSystemVariable *var)
{
    t_mosSystemVariable *current = mosSystemVariables;
    t_mosSystemVariable *parent  = NULL;

    while (current != NULL) {
        if (current == var) {
            return parent;
        }
        parent  = current;
        current = current->next;
    }
    return NULL;
}

/* =========================================================================
 * setVarVal  — OS-level Set Variable
 * ========================================================================= */
int setVarVal(char *name, void *value, char **actualName, uint8_t *type)
{
    t_mosSystemVariable *var = *actualName ? mosSystemVariables : NULL;
    int result;
    bool transInput = (*type == MOS_VAR_STRING);
    bool freeValue  = transInput;

    if (*type > MOS_VAR_LITERAL && *type != 255) {
        return FR_INVALID_PARAMETER;
    }

    if (*type == MOS_VAR_EXPANDED) {
        t_mosEvalResult *evalResult = evaluateExpression((char *)value);
        if (evalResult == NULL) {
            return FR_INT_ERR;
        }
        if (evalResult->status != FR_OK) {
            result = evalResult->status;
            free(evalResult);
            return result;
        }
        *type  = evalResult->type;
        value  = evalResult->result;
        if (*type == MOS_VAR_STRING) {
            freeValue = true;
        }
        free(evalResult);
    }

    /* When actualName is set, skip forward to the node with that label */
    while (var && var->label != *actualName) {
        var = var->next;
    }
    result = getSystemVariable(name, &var);

    /* type == 255 means delete */
    if (*type == 255) {
        if (result == 0) {
            if (var->type != MOS_VAR_CODE) {
                removeSystemVariable(var);
            }
            return FR_OK;
        }
        return FR_INVALID_NAME;
    }

    if (transInput) {
        value = expandMacro((char *)value);
        if (value == NULL) {
            return FR_INT_ERR;
        }
    }
    if (*type == MOS_VAR_LITERAL) {
        *type = MOS_VAR_STRING;
    }

    if (result == -1) {
        /* Create new */
        t_mosSystemVariable *newVar = createSystemVariable(name, *type, value);
        if (newVar == NULL) {
            if (freeValue) free(value);
            return FR_INT_ERR;
        }
        insertSystemVariable(newVar, var);
        var    = newVar;
        result = FR_OK;
    } else {
        result = updateSystemVariable(var, *type, value);
    }

    *type       = var->type;
    *actualName = var->label;
    if (freeValue) {
        free(value);
    }
    return result;
}

/* =========================================================================
 * readVarVal  — OS-level Read Variable
 * ========================================================================= */
int readVarVal(char *namePattern, void *value, char **actualName, int *length, uint8_t *typeFlag)
{
    t_mosSystemVariable *var = mosSystemVariables;
    int result;
    int bufferLen = *length;
    bool expand   = (*typeFlag == 3);

    *length = 0;

    while (var && var->label != *actualName) {
        var = var->next;
    }
    result = getSystemVariable(namePattern, &var);

    if (result == -1) {
        return FR_INVALID_NAME;
    }

    *actualName = var->label;
    *typeFlag   = (uint8_t)var->type;
    if (var->type == MOS_VAR_CODE) {
        expand = true;
    }

    result = FR_OK;
    if (expand) {
        char *expanded = expandVariable(var, true);
        if (expanded == NULL) {
            return FR_INT_ERR;
        }
        *length = (int)strlen(expanded);
        if (value != NULL) {
            strncpy((char *)value, expanded, (size_t)bufferLen);
            if (bufferLen < *length) {
                result = MOS_OUT_OF_MEMORY;
            }
        }
        free(expanded);
    } else {
        if (var->type == MOS_VAR_MACRO || var->type == MOS_VAR_STRING) {
            *length = (int)strlen((char *)var->value);
            if (value != NULL) {
                strncpy((char *)value, (char *)var->value, (size_t)bufferLen);
                if (bufferLen < *length) {
                    result = MOS_OUT_OF_MEMORY;
                }
            }
        } else {
            /* Numbers are stored as raw int in value pointer */
            *length = 3;
            if (value != NULL) {
                *(int *)value = (int)(intptr_t)var->value;
            }
        }
    }
    return result;
}

/* =========================================================================
 * gsInit  — Initialise a GSTrans state object
 * ========================================================================= */
int gsInit(void *source, t_mosTransInfo **transInfoPtr, uint8_t flags)
{
    t_mosTransInfo *transInfo;

    if (transInfoPtr == NULL) {
        return FR_INVALID_PARAMETER;
    }

    transInfo = malloc(sizeof(t_mosTransInfo));
    if (transInfo == NULL) {
        return MOS_OUT_OF_MEMORY;
    }

    snprintf(transInfo->active, sizeof(transInfo->active), "TInfo");
    transInfo->source    = (char *)source;
    transInfo->parent    = NULL;
    transInfo->extraData = NULL;
    transInfo->type      = MOS_VAR_MACRO;
    transInfo->flags     = flags;

    if (!(flags & GSTRANS_FLAG_NO_DOUBLEQUOTE) && *((char *)source) == '"') {
        /* Skip opening double-quote */
        transInfo->source++;
    } else {
        transInfo->flags |= GSTRANS_FLAG_NO_DOUBLEQUOTE;
    }

    if (!(flags & GSTRANS_FLAG_NO_TRACE)) {
        if (trackedTransInfo != NULL) {
            gsDispose(&trackedTransInfo);
        }
        trackedTransInfo = transInfo;
    }

    *transInfoPtr = transInfo;
    return FR_OK;
}

/* =========================================================================
 * gsRead  — Read next transformed character from a GSTrans chain
 * ========================================================================= */
int gsRead(t_mosTransInfo **transInfo, char *read)
{
    t_mosTransInfo *current = *transInfo;
    int result = FR_OK;

    if (read == NULL) {
        return FR_INVALID_PARAMETER;
    }
    if (current == NULL) {
        return FR_OK;
    }
    if (strcmp(current->active, "TInfo") != 0) {
        return FR_INT_ERR;
    }

    *read = '\0';

    if (!current->parent &&
        (current->flags & GSTRANS_FLAG_TERMINATE_SPACE) &&
        isspace((unsigned char)*current->source)) {
        gsPop(transInfo);
        return FR_OK;
    }

    switch (current->type) {
        case MOS_VAR_LITERAL:
        case MOS_VAR_STRING:
        case MOS_VAR_CODE: {
            *read = *current->source++;
            if (*read == '\0') {
                gsPop(transInfo);
                result = gsRead(transInfo, read);
            }
            break;
        }

        case MOS_VAR_NUMBER: {
            /* source holds the remaining value; extraData holds the current divisor */
            if ((int)(intptr_t)current->extraData == 0) {
                *read = '\0';
                gsPop(transInfo);
                result = gsRead(transInfo, read);
                break;
            }
            *read = (char)('0' + ((int)(intptr_t)current->source /
                                   (int)(intptr_t)current->extraData));
            current->source   = (char *)(intptr_t)
                                 ((int)(intptr_t)current->source %
                                  (int)(intptr_t)current->extraData);
            current->extraData = (void *)(intptr_t)
                                 ((int)(intptr_t)current->extraData / 10);
            break;
        }

        case MOS_VAR_MACRO: {
            *read = *current->source++;
            switch (*read) {
                case '"': {
                    if (current->flags & GSTRANS_FLAG_NO_DOUBLEQUOTE) {
                        break;
                    }
                    *read = '\0';
                    gsDispose(transInfo);
                    result = gsRead(transInfo, read);
                    break;
                }
                case '\0': {
                    gsPop(transInfo);
                    result = gsRead(transInfo, read);
                    break;
                }
                case '|': {
                    if (current->flags & GSTRANS_FLAG_NO_PIPE) {
                        break;
                    }
                    if (*current->source == '\0') {
                        *read = '\0';
                        return MOS_BAD_STRING;
                    } else if (*current->source == '?') {
                        *read = 0x7F;
                        current->source++;
                    } else if (*current->source == '!') {
                        current->source++;
                        if (*current->source == '\0') {
                            *read = '\0';
                            return MOS_BAD_STRING;
                        }
                        *read = *current->source | (char)0x80;
                        current->source++;
                    } else if (*current->source == '|') {
                        *read = '|';
                        current->source++;
                    } else if (*current->source >= 0x40 && *current->source < 0x7F) {
                        *read = *current->source & 0x1F;
                        current->source++;
                    } else {
                        *read = *current->source;
                        current->source++;
                    }
                    break;
                }
                case '<': {
                    char *end = current->source;
                    if (*end == ' ') {
                        break; /* Not a variable ref; pass '<' through */
                    }
                    while (*end && *end != '>') {
                        end++;
                    }
                    if (*end == '>' && end > current->source) {
                        int number = 0;
                        if (!extractNumber(current->source, &end, ">", &number, 0)) {
                            /* Try as variable name */
                            t_mosSystemVariable *var = NULL;
                            *end = '\0';
                            if (getSystemVariable(current->source, &var) == 0) {
                                t_mosTransInfo *newTransInfo = malloc(sizeof(t_mosTransInfo));
                                *end = '>';
                                if (newTransInfo == NULL) {
                                    gsDispose(transInfo);
                                    result = MOS_OUT_OF_MEMORY;
                                } else {
                                    snprintf(newTransInfo->active,
                                             sizeof(newTransInfo->active), "TInfo");
                                    newTransInfo->source    = (char *)var->value;
                                    newTransInfo->parent    = current;
                                    newTransInfo->extraData = NULL;
                                    newTransInfo->type      = var->type;
                                    newTransInfo->flags     = current->flags;
                                    *transInfo = newTransInfo;
                                    if (!(current->flags & GSTRANS_FLAG_NO_TRACE)) {
                                        trackedTransInfo = newTransInfo;
                                    }
                                    current->source = end + 1;

                                    switch (var->type) {
                                        case MOS_VAR_NUMBER: {
                                            int num = (int)(intptr_t)var->value;
                                            newTransInfo->extraData = (void *)(intptr_t)1;
                                            if (num < 0) {
                                                newTransInfo->source =
                                                    (char *)(intptr_t)(-num);
                                                *read = '-';
                                            } else {
                                                newTransInfo->source =
                                                    (char *)(intptr_t)num;
                                            }
                                            while (((int)(intptr_t)newTransInfo->extraData * 10) <=
                                                    (int)(intptr_t)newTransInfo->source) {
                                                newTransInfo->extraData =
                                                    (void *)(intptr_t)
                                                    ((int)(intptr_t)newTransInfo->extraData * 10);
                                            }
                                            *transInfo = newTransInfo;
                                            if (num >= 0) {
                                                result = gsRead(transInfo, read);
                                            }
                                            break;
                                        }
                                        case MOS_VAR_CODE: {
                                            char *newValue = expandVariable(var, false);
                                            if (!newValue) {
                                                *read = '\0';
                                                gsDispose(transInfo);
                                                return MOS_BAD_STRING;
                                            }
                                            newTransInfo->source    = newValue;
                                            newTransInfo->extraData = newValue;
                                            *transInfo = newTransInfo;
                                            result = gsRead(transInfo, read);
                                            break;
                                        }
                                        default:
                                            result = gsRead(transInfo, read);
                                    }
                                }
                            } else {
                                /* Variable not found: skip past '>' and continue */
                                *end = '>';
                                current->source = end + 1;
                                return gsRead(transInfo, read);
                            }
                        } else {
                            *read = (char)(number & 0xFF);
                        }
                        *end = '>';
                        current->source = end + 1;
                    }
                    break;
                }
                default:
                    break;
            }
            break;
        }

        case MOS_VAR_EXPANDED:
        default:
            break;
    }

    return result;
}

/* =========================================================================
 * gsTrans  — Transform a source string using GSTrans rules
 * ========================================================================= */
int gsTrans(char *source, char *dest, int destLen, int *read, uint8_t flags)
{
    t_mosTransInfo *transInfo;
    int result;
    int remaining = destLen;
    char c;

    if (source == NULL || source == dest || read == NULL) {
        return FR_INVALID_PARAMETER;
    }
    if (dest == NULL) {
        remaining = 0;
    }
    *read = 0;

    result = gsInit(source, &transInfo, flags | GSTRANS_FLAG_NO_TRACE);
    if (result != FR_OK) {
        return result;
    }

    while (transInfo != NULL) {
        result = gsRead(&transInfo, &c);
        if (result != FR_OK) {
            return result;
        }
        if (transInfo == NULL) {
            break;
        }
        (*read)++;
        if (remaining > 0) {
            *dest++ = c;
            remaining--;
        }
    }

    return FR_OK;
}

/* =========================================================================
 * gsDispose  — Free the entire GSTrans chain
 * ========================================================================= */
void gsDispose(t_mosTransInfo **transInfoPtr)
{
    t_mosTransInfo *transInfo = *transInfoPtr;

    if (transInfo == NULL) {
        return;
    }
    if (!(transInfo->flags & GSTRANS_FLAG_NO_TRACE)) {
        trackedTransInfo = NULL;
    }
    while (transInfo != NULL) {
        t_mosTransInfo *next = transInfo->parent;
        snprintf(transInfo->active, sizeof(transInfo->active), "DEAD");
        free(transInfo);
        transInfo = next;
    }
    *transInfoPtr = NULL;
}

/* =========================================================================
 * gsPop  — Pop one level off a GSTrans chain
 * ========================================================================= */
void gsPop(t_mosTransInfo **transInfo)
{
    t_mosTransInfo *current = *transInfo;
    if (!current) {
        return;
    }
    if (current->type == MOS_VAR_CODE) {
        /* Free cached string stored in extraData */
        free(current->extraData);
    }
    *transInfo = current->parent;
    snprintf(current->active, sizeof(current->active), "DEAD");
    bool traced = !(current->flags & GSTRANS_FLAG_NO_TRACE);
    free(current);
    if (traced) {
        trackedTransInfo = *transInfo;
    }
}

/* =========================================================================
 * extractNumber
 * Parse a number (decimal, &hex, 0x hex, base_N) from source.
 * Returns true if successful.
 * ========================================================================= */
uint8_t extractNumber(char *source, char **end, char *divider, int *number, uint8_t flags)
{
    int   base     = 10;
    char *endptr   = NULL;
    char *parseEnd = NULL;
    char *start    = source;
    char  lastChar = '\0';
    int   value    = 0;

    if (divider == NULL) {
        divider = " ";
    }

    start += mos_strspn(start, divider);

    if (end == NULL || *end == NULL || *end == source) {
        endptr = start + mos_strcspn(start, divider);
    } else {
        endptr = *end;
    }
    lastChar  = *endptr;
    *endptr   = '\0';

    if (strlen(start) == 0) {
        *endptr = lastChar;
        return false;
    }

    if (flags & EXTRACT_NUM_H_SUFFIX_HEX) {
        if (*(endptr - 1) == 'h') {
            base = 16;
            endptr--;
        }
    }
    if (*start == '&') {
        base = 16;
        start++;
    } else if (*start == '0' &&
               (*(start + 1) == 'x' || *(start + 1) == 'X')) {
        base = 16;
        start += 2;
    } else if (base != 16) {
        char *underscore = strchr(start, '_');
        if (underscore != NULL && underscore > start) {
            char *baseEnd;
            *underscore = '\0';
            base = (int)strtol(start, &baseEnd, 10);
            if (baseEnd != underscore) {
                base = -1;
            }
            start       = underscore + 1;
            *underscore = '_';
        }
    }

    if ((flags & EXTRACT_NUM_DECIMAL_ONLY) && base != 10) {
        *endptr = lastChar;
        return false;
    }

    if (base > 1 && base <= 36) {
        value = (int)strtol(start, &parseEnd, base);
    } else {
        *endptr = lastChar;
        return false;
    }

    *endptr = lastChar;

    if ((parseEnd < endptr) ||
        ((flags & EXTRACT_NUM_POSITIVE_ONLY) && value < 0)) {
        if (end != NULL && *end == source) {
            *end = parseEnd;
        }
        return false;
    }

    if (end != NULL) {
        *end = parseEnd;
    }
    *number = value;
    return true;
}

/* =========================================================================
 * extractString
 * Extract a token from source, honouring quotes and dividers.
 * ========================================================================= */
int extractString(char *source, char **end, char *divider, char **result, uint8_t flags)
{
    char *start        = source;
    char *endptr       = NULL;
    bool  findEndQuotes = false;

    if (result == NULL) {
        return FR_INVALID_PARAMETER;
    }
    if (divider == NULL) {
        divider = " ";
    }

    if (!(flags & EXTRACT_FLAG_OMIT_LEADSKIP)) {
        start = start + mos_strspn(start, divider);
    }

    if (!(flags & EXTRACT_FLAG_NO_DOUBLEQUOTE) && *start == '"') {
        if (!(flags & EXTRACT_FLAG_INCLUDE_QUOTES)) {
            start++;
        }
        findEndQuotes = true;
    }

    if (findEndQuotes) {
        endptr = start;
        if (flags & EXTRACT_FLAG_INCLUDE_QUOTES) {
            endptr++;
        }
        while (*endptr != '\0') {
            if (*endptr == '"') {
                if (*(endptr + 1) == '"') {
                    endptr++;
                } else if (*(endptr - 1) != '\\') {
                    break;
                }
            }
            endptr++;
        }
        if (*endptr == '\0') {
            return MOS_BAD_STRING;
        }
        if (*(endptr + 1) != '\0' && strchr(divider, *(endptr + 1)) == NULL) {
            return MOS_BAD_STRING;
        }
        if (flags & EXTRACT_FLAG_INCLUDE_QUOTES) {
            endptr++;
        }
    } else {
        endptr = start + mos_strcspn(start, divider);
    }

    if (*endptr != '\0' && (flags & EXTRACT_FLAG_AUTO_TERMINATE)) {
        *endptr = '\0';
        endptr++;
    }

    *result = start;
    if (end != NULL) {
        *end = endptr;
    }

    if (start == endptr) {
        return FR_INVALID_PARAMETER;
    }

    return FR_OK;
}

/* =========================================================================
 * escapeString
 * Escape control characters and '|' using RISC OS pipe-notation.
 * ========================================================================= */
int escapeString(char *source, char *dest, int *length)
{
    char *start  = source;
    int   result = FR_OK;
    int   destLen = 1;

    if (source == NULL) {
        return FR_INVALID_PARAMETER;
    }

    while (*start != '\0') {
        if (*start < 32 || *start == 127 || *start == '|') ++destLen;
        destLen++;
        start++;
    }

    if (dest != NULL) {
        int remaining = *length;
        start = source;

        if (remaining <= 0) {
            return FR_INVALID_PARAMETER;
        }

        while (*start) {
            if (*start < 32 || *start == 127 || *start == '|') {
                if (remaining <= 2) {
                    result   = MOS_OUT_OF_MEMORY;
                    *dest++  = '\0';
                    break;
                }
                *dest++ = '|';
                if (*start == 0x7F) {
                    *dest++ = '?';
                } else if (*start == '|') {
                    *dest++ = '|';
                } else {
                    *dest++ = (char)(*start + 64);
                }
                remaining -= 2;
            } else {
                *dest++ = *start;
                remaining--;
                if (remaining == 1) {
                    *dest = '\0';
                    break;
                }
            }
            start++;
        }
        if (start < source + strlen(source) - 1) {
            result = MOS_OUT_OF_MEMORY;
        }
    }

    *length = destLen;
    return result;
}

/* =========================================================================
 * expandMacro
 * Fully expand a GSTrans macro string. Caller must free() the result.
 * ========================================================================= */
char *expandMacro(char *source)
{
    char *dest;
    int   read;
    int   result;

    result = gsTrans(source, NULL, 0, &read, GSTRANS_FLAG_NO_DOUBLEQUOTE);
    if (result != FR_OK) {
        return NULL;
    }
    dest = malloc((size_t)(read + 1));
    if (dest == NULL) {
        return NULL;
    }
    result = gsTrans(source, dest, read, &read, GSTRANS_FLAG_NO_DOUBLEQUOTE);
    if (result != FR_OK) {
        free(dest);
        return NULL;
    }
    dest[read] = '\0';
    return dest;
}

/* =========================================================================
 * expandVariable
 * Expand a variable to a string. Caller must free() the result.
 * ========================================================================= */
char *expandVariable(t_mosSystemVariable *var, bool showWriteOnly)
{
    if (var->type == MOS_VAR_MACRO) {
        return expandMacro((char *)var->value);
    }
    if (var->type == MOS_VAR_STRING) {
        return mos_strdup((char *)var->value);
    }
    if (var->type == MOS_VAR_NUMBER) {
        int   number  = (int)(intptr_t)var->value;
        int   digits  = 1;
        int   divisor = (number < 0) ? -1 : 1;
        char *value;

        while (divisor * 10 <= number) {
            divisor *= 10;
            digits++;
        }
        if (number < 0) digits++;

        value = malloc((size_t)(digits + 1));
        if (value == NULL) return NULL;
        snprintf(value, (size_t)(digits + 1), "%d", number);
        return value;
    }
    if (var->type == MOS_VAR_CODE) {
        int   len      = 0;
        char *newValue = NULL;

        if (((t_mosCodeSystemVariable *)var->value)->read == NULL) {
            if (showWriteOnly) {
                return mos_strdup("(write only)");
            }
            return NULL;
        }
        if (((t_mosCodeSystemVariable *)var->value)->read(NULL, &len) != FR_OK) {
            return NULL;
        }
        newValue = malloc((size_t)len);
        if (newValue == NULL) return NULL;
        if (((t_mosCodeSystemVariable *)var->value)->read(newValue, &len) != FR_OK) {
            free(newValue);
            return NULL;
        }
        return newValue;
    }
    return NULL;
}

/* =========================================================================
 * expandVariableToken
 * Look up a variable by name and expand it. Caller must free() the result.
 * ========================================================================= */
char *expandVariableToken(char *token)
{
    t_mosSystemVariable *var = NULL;
    if (getSystemVariable(token, &var) != 0) {
        return NULL;
    }
    return expandVariable(var, false);
}

/* =========================================================================
 * evaluateExpression
 * Evaluate source as a number or variable. Caller must free() result.
 * ========================================================================= */
t_mosEvalResult *evaluateExpression(char *source)
{
    t_mosSystemVariable *var    = NULL;
    int                  number = 0;
    t_mosEvalResult     *result = malloc(sizeof(t_mosEvalResult));

    if (result == NULL) {
        return NULL;
    }
    result->result = NULL;
    result->status = FR_OK;

    /* Try as a number first */
    if (extractNumber(source, NULL, NULL, &number, 0)) {
        result->type   = MOS_VAR_NUMBER;
        result->result = (void *)(intptr_t)number;
        return result;
    }

    /* Try as a variable */
    if (getSystemVariable(source, &var) != 0) {
        result->status = FR_INVALID_PARAMETER;
        return result;
    }

    if (var->type == MOS_VAR_NUMBER) {
        result->type   = MOS_VAR_NUMBER;
        result->result = var->value;
    } else {
        result->result = expandVariable(var, false);
        if (result->result == NULL) {
            result->status = FR_INT_ERR;
        }
        result->type = MOS_VAR_STRING;
    }

    return result;
}

/* =========================================================================
 * getArgument
 * Return the Nth whitespace-separated argument from source.
 * ========================================================================= */
char *getArgument(char *source, int argNo, char **end)
{
    char *result   = source;
    char *divider  = " ";
    int   argCount = 0;
    char *scanFrom = result;

    if (end != NULL) {
        *end = NULL;
    }

    while (argCount <= argNo) {
        if (extractString(scanFrom, &scanFrom, divider, &result,
                          EXTRACT_FLAG_INCLUDE_QUOTES)) {
            return NULL;
        }
        argCount++;
    }

    if (end != NULL) {
        *end = scanFrom;
    }
    return result;
}

/* =========================================================================
 * substituteArgs
 * Substitute %0..%9, %s, %*N into tmpl from args.
 * Returns total length including null terminator.
 * ========================================================================= */
int substituteArgs(char *tmpl, char *args, char *dest, int length, uint8_t flags)
{
    char *end         = tmpl + strlen(tmpl);
    char *argument;
    int   argNo;
    int   argLen      = 0;
    int   maxArg      = -1;
    int   size        = 0;
    int   destRemaining = length;
    bool  copying     = (dest != NULL);
    bool  omitRest    = (flags & 1) != 0;

    while (tmpl < end) {
        if (*tmpl == '%') {
            argument = NULL;
            tmpl++;
            if (*tmpl == 's') {
                tmpl++;
                argument = args;
                argLen   = (int)strlen(args);
                maxArg   = 99;
            } else if (*tmpl == '*' && *(tmpl + 1) >= '0' && *(tmpl + 1) <= '9') {
                tmpl++;
                argNo    = *tmpl - '0';
                tmpl++;
                argument = getArgument(args, argNo, NULL);
                if (argument != NULL) {
                    argLen = (int)strlen(argument);
                }
                maxArg = 99;
            } else if (*tmpl >= '0' && *tmpl <= '9') {
                char *argEnd = NULL;
                argNo    = *tmpl - '0';
                tmpl++;
                argument = getArgument(args, argNo, &argEnd);
                if (argument != NULL) {
                    argLen = (int)(argEnd - argument);
                }
                if (argNo > maxArg) {
                    maxArg = argNo;
                }
            } else {
                size++;
                if (copying) {
                    *dest++ = '%';
                    destRemaining--;
                }
                if (*tmpl == '%') {
                    tmpl++;
                }
            }

            if (argument != NULL) {
                size += argLen;
                if (copying) {
                    strncpy(dest, argument,
                            (size_t)(argLen < destRemaining ? argLen : destRemaining));
                    dest          += argLen;
                    destRemaining -= argLen;
                }
            }
        } else {
            size++;
            if (copying) {
                *dest++ = *tmpl;
                destRemaining--;
            }
            tmpl++;
        }
        if (copying && destRemaining <= 0) {
            copying = false;
        }
    }

    /* Append any remaining arguments not referenced by template */
    if (!omitRest && maxArg < 99) {
        argument = getArgument(args, maxArg + 1, NULL);
        if (argument != NULL) {
            argLen = (int)strlen(argument);
            if (argLen > 0) {
                size += argLen + 1;
                if (copying) {
                    *dest++ = ' ';
                    destRemaining--;
                    strncpy(dest, argument, (size_t)destRemaining);
                }
                copying = false;
            }
        }
    }

    if (copying) {
        *dest = '\0';
    }

    return ++size;
}

/* =========================================================================
 * substituteArguments
 * Allocate and return substituteArgs result. Caller must free().
 * ========================================================================= */
char *substituteArguments(char *source, char *args, uint8_t flags)
{
    int   size = substituteArgs(source, args, NULL, 0, flags);
    char *dest;

    if (size == 0) {
        return NULL;
    }
    dest = malloc((size_t)(size + 1));
    if (dest == NULL) {
        return NULL;
    }
    substituteArgs(source, args, dest, size, flags);
    dest[size] = '\0';
    return dest;
}

/* =========================================================================
 * mos_listVars  — Print all variables matching pattern to console
 * ========================================================================= */
void mos_listVars(const char *pattern)
{
    t_mosSystemVariable *var = mosSystemVariables;

    while (var != NULL) {
        if (!pattern || !*pattern ||
            stristr(var->label, pattern) != NULL) {
            const char *type_str;
            switch (var->type) {
                case MOS_VAR_NUMBER:   type_str = "Number"; break;
                case MOS_VAR_MACRO:    type_str = "Macro";  break;
                case MOS_VAR_CODE:     type_str = "Code";   break;
                case MOS_VAR_LITERAL:  type_str = "Literal";break;
                default:               type_str = "String"; break;
            }
            char *expanded = expandVariable(var, true);
            printf("%-32s = %-32s [%s]\n",
                   var->label,
                   expanded ? expanded : "(null)",
                   type_str);
            free(expanded);
        }
        var = var->next;
    }
}
