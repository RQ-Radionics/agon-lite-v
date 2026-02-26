/*
 * mos_version.h - Version information for ESP32-MOS
 * Mirrors agon-mos src/version.h adapted for ESP32.
 */

#ifndef MOS_VERSION_H
#define MOS_VERSION_H

#define VERSION_MAJOR       3
#define VERSION_MINOR       0
#define VERSION_PATCH       1

#define VERSION_VARIANT     "ESP32-S3"
#define VERSION_SUBTITLE    "Arthur"

/* Stringified version: "3.0.1" */
#define _MOS_STR(x)         #x
#define _MOS_XSTR(x)        _MOS_STR(x)
#define VERSION_STRING      _MOS_XSTR(VERSION_MAJOR) "." \
                            _MOS_XSTR(VERSION_MINOR) "." \
                            _MOS_XSTR(VERSION_PATCH)

#endif /* MOS_VERSION_H */
