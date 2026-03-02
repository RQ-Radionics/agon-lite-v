/*
 * mos_net.h — Network abstraction layer for ESP32-MOS
 *
 * Provides a single interface for IP connectivity regardless of whether
 * the underlying transport is WiFi (via C6 coprocessor) or Ethernet (RMII).
 *
 * Backend is selected at compile time via Kconfig (MOS_NET_BACKEND).
 */

#ifndef MOS_NET_H
#define MOS_NET_H

#include <stdbool.h>
#include <stdint.h>
#include "sdkconfig.h"

/**
 * Bring up the network interface and wait for an IP address.
 *
 * @param max_wait_ms  Maximum time to wait (ms). 0 = wait indefinitely.
 * @return  0 on success (IP obtained), -1 on failure/timeout.
 */
int  mos_net_init(uint32_t max_wait_ms);

/** Returns true if the interface is up and an IP address is assigned. */
bool mos_net_is_connected(void);

/** Returns the assigned IP as a dotted-decimal string, or NULL. */
const char *mos_net_ip(void);

#endif /* MOS_NET_H */
