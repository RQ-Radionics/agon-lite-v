/*
 * mos_sysvars.h - System variables for ESP32-MOS
 * Ported from agon-mos src/mos_sysvars.h
 *
 * Changes from original:
 *  - BYTE replaced by uint8_t
 *  - umm_malloc replaced by malloc/free
 *  - ff.h types removed (not needed on ESP32 POSIX VFS)
 *  - FR_OK/FR_* error codes replaced by MOS result codes
 */

#ifndef MOS_SYSVARS_H
#define MOS_SYSVARS_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "mos_types.h"
#include "mos_strings.h"

/* -----------------------------------------------------------------------
 * Flags for extractNumber()
 * ----------------------------------------------------------------------- */
#define EXTRACT_NUM_DECIMAL_ONLY    (1 << 0)
#define EXTRACT_NUM_POSITIVE_ONLY   (1 << 1)
#define EXTRACT_NUM_H_SUFFIX_HEX    (1 << 2)

/* -----------------------------------------------------------------------
 * Flags for extractString()
 * ----------------------------------------------------------------------- */
#define EXTRACT_FLAG_AUTO_TERMINATE (1 << 0)
#define EXTRACT_FLAG_OMIT_LEADSKIP  (1 << 1)
#define EXTRACT_FLAG_NO_DOUBLEQUOTE (1 << 2)
#define EXTRACT_FLAG_INCLUDE_QUOTES (1 << 3)

/* -----------------------------------------------------------------------
 * Flags for gsInit() / gsTrans()
 * ----------------------------------------------------------------------- */
#define GSTRANS_FLAG_TERMINATE_SPACE    (1 << 0)
#define GSTRANS_FLAG_NO_PIPE            (1 << 1)
#define GSTRANS_FLAG_NO_DOUBLEQUOTE     (1 << 2)
#define GSTRANS_FLAG_NO_TRACE           (1 << 7)

/* -----------------------------------------------------------------------
 * MOS system variable types (identical enum values to original)
 * ----------------------------------------------------------------------- */
typedef enum {
    MOS_VAR_STRING   = 0,   /* String, GSTrans'd before storing */
    MOS_VAR_NUMBER   = 1,   /* Integer */
    MOS_VAR_MACRO    = 2,   /* String, GSTrans'd each time used */
    MOS_VAR_EXPANDED = 3,   /* Expression evaluated before storing */
    MOS_VAR_LITERAL  = 4,   /* Literal string, no GSTrans */
    MOS_VAR_CODE     = 16,  /* Code variable: read/write function pointers */
} MOSVARTYPE;

/* -----------------------------------------------------------------------
 * System variable node (singly linked list, sorted by label)
 * ----------------------------------------------------------------------- */
typedef struct t_mosSystemVariable {
    char                        *label;
    MOSVARTYPE                   type;
    void                        *value;
    struct t_mosSystemVariable  *next;
} t_mosSystemVariable;

/* -----------------------------------------------------------------------
 * Code variable: variable backed by read/write function pointers
 * ----------------------------------------------------------------------- */
typedef struct {
    int (*read)(char *buffer, int *length);
    int (*write)(char *buffer);
} t_mosCodeSystemVariable;

/* -----------------------------------------------------------------------
 * GSTrans state object
 * ----------------------------------------------------------------------- */
typedef struct t_mosTransInfo {
    char                    active[6];  /* "TInfo" when valid, "DEAD" after free */
    char                   *source;     /* Current position in source string */
    struct t_mosTransInfo  *parent;     /* Parent trans object */
    void                   *extraData;  /* Per-type scratch data */
    MOSVARTYPE               type;      /* Type of current variable */
    uint8_t                  flags;     /* GSTRANS_FLAG_* */
} t_mosTransInfo;

/* -----------------------------------------------------------------------
 * Expression evaluation result
 * ----------------------------------------------------------------------- */
typedef struct {
    void       *result;
    MOSVARTYPE  type;
    int         status;
} t_mosEvalResult;

/* -----------------------------------------------------------------------
 * Global variable list head
 * ----------------------------------------------------------------------- */
extern t_mosSystemVariable *mosSystemVariables;

/* -----------------------------------------------------------------------
 * Initialise the system variable subsystem and register defaults
 * ----------------------------------------------------------------------- */
void mos_sysvars_init(void);

/* -----------------------------------------------------------------------
 * Core variable management
 * ----------------------------------------------------------------------- */
int                    getSystemVariable(char *pattern, t_mosSystemVariable **var);
t_mosSystemVariable   *createSystemVariable(char *label, MOSVARTYPE type, void *value);
void                   insertSystemVariable(t_mosSystemVariable *var, t_mosSystemVariable *before);
int                    createOrUpdateSystemVariable(char *label, MOSVARTYPE type, void *value);
int                    updateSystemVariable(t_mosSystemVariable *var, MOSVARTYPE type, void *value);
void                   removeSystemVariable(t_mosSystemVariable *var);
t_mosSystemVariable   *findParentSystemVariable(t_mosSystemVariable *var);

/* -----------------------------------------------------------------------
 * OS-level variable API (MOS API equivalents)
 * ----------------------------------------------------------------------- */
int setVarVal(char *name, void *value, char **actualName, uint8_t *type);
int readVarVal(char *namePattern, void *value, char **actualName, int *length, uint8_t *typeFlag);

/* -----------------------------------------------------------------------
 * GSTrans string transformation
 * ----------------------------------------------------------------------- */
int  gsInit(void *source, t_mosTransInfo **transInfoPtr, uint8_t flags);
int  gsRead(t_mosTransInfo **transInfo, char *read);
int  gsTrans(char *source, char *dest, int destLen, int *read, uint8_t flags);
void gsDispose(t_mosTransInfo **transInfoPtr);
void gsPop(t_mosTransInfo **transInfo);

/* -----------------------------------------------------------------------
 * String / number extraction utilities
 * ----------------------------------------------------------------------- */
uint8_t extractNumber(char *source, char **end, char *divider, int *number, uint8_t flags);
int     extractString(char *source, char **end, char *divider, char **result, uint8_t flags);
int     escapeString(char *source, char *dest, int *length);

/* -----------------------------------------------------------------------
 * Macro / variable expansion
 * ----------------------------------------------------------------------- */
char             *expandMacro(char *source);
char             *expandVariable(t_mosSystemVariable *var, bool showWriteOnly);
char             *expandVariableToken(char *token);
t_mosEvalResult  *evaluateExpression(char *source);

/* -----------------------------------------------------------------------
 * Argument substitution
 * ----------------------------------------------------------------------- */
char *getArgument(char *source, int argNo, char **end);
int   substituteArgs(char *tmpl, char *args, char *dest, int length, uint8_t flags);
char *substituteArguments(char *source, char *args, uint8_t flags);

/* -----------------------------------------------------------------------
 * Convenience wrappers (simple string get/set, for shell use)
 * ----------------------------------------------------------------------- */
void        mos_listVars(const char *pattern);

#endif /* MOS_SYSVARS_H */
