/*
 * mos_ftp.h — Minimal FTP server for ESP32-MOS
 *
 * Exposes /sdcard as the FTP root over an existing lwIP/esp_netif stack.
 * Single-session, passive-mode (PASV) only.
 *
 * While a client session is active the VDP internal render task and 60 Hz
 * timer are paused to avoid SD/PSRAM contention.
 *
 * Usage:
 *   After mos_net_init() returns 0:
 *     mos_ftp_init();   // starts listener task, returns immediately
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Start the FTP server listener task.
 * Must be called after mos_net_init() succeeds.
 * Returns 0 on success, -1 if task creation fails.
 */
int mos_ftp_init(void);

#ifdef __cplusplus
}
#endif
