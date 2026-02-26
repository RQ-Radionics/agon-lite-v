/*
 * mos_wifi.h - WiFi connection manager for ESP32-MOS
 */

#ifndef MOS_WIFI_H
#define MOS_WIFI_H

#include <stdbool.h>

/**
 * Initialise WiFi in STA mode and connect using credentials from
 * wifi_credentials.h.  Blocks until connected or max_wait_ms elapses.
 *
 * @param max_wait_ms  Maximum time to wait for IP address (ms).
 *                     Pass 0 to wait indefinitely.
 * @return  0 on success (IP obtained), -1 on timeout/error.
 */
int  mos_wifi_init(uint32_t max_wait_ms);

/** Returns true if WiFi is connected and an IP address has been assigned. */
bool mos_wifi_is_connected(void);

/** Returns the assigned IP address as a dotted-decimal string, or NULL. */
const char *mos_wifi_ip(void);

#endif /* MOS_WIFI_H */
