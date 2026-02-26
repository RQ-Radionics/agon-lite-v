/*
 * mos_sntp.h - SNTP time synchronisation for ESP32-MOS
 */

#ifndef MOS_SNTP_H
#define MOS_SNTP_H

#include <stdbool.h>

/**
 * Start SNTP client and wait for the first successful sync.
 *
 * @param tz_posix  POSIX timezone string (e.g. "CET-1CEST,M3.5.0,M10.5.0/3").
 *                  Pass NULL or "" for UTC.
 * @param max_wait_ms  Maximum time to wait for sync (ms). 0 = indefinite.
 * @return 0 on success, -1 on timeout.
 */
int  mos_sntp_init(const char *tz_posix, uint32_t max_wait_ms);

/** Returns true once the clock has been synchronised at least once. */
bool mos_sntp_is_synced(void);

#endif /* MOS_SNTP_H */
