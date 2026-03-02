/*
 * mos_uart.h - Secondary UART driver for ESP32-MOS
 *
 * Implements the agon-mos UART API:
 *   mos_uopen  (15h): open UART with given baud rate
 *   mos_uclose (16h): close UART
 *   mos_ugetc  (17h): read byte from UART (blocking)
 *   mos_uputc  (18h): write byte to UART
 *
 * Uses UART_NUM_1 (secondary port).  UART0 is reserved for the debug console.
 *
 * Default GPIO pins:
 *   ESP32-P4 (Waveshare ESP32-P4-WIFI6): TX = GPIO 38, RX = GPIO 37
 *   ESP32-S3:                             TX = GPIO 17, RX = GPIO 18
 *
 * Override before calling mos_uopen() via mos_uart_set_pins().
 */

#ifndef MOS_UART_H
#define MOS_UART_H

#include <stdint.h>
#ifdef ESP_PLATFORM
#  include "sdkconfig.h"
#endif

/* ── Default GPIO pins ─────────────────────────────────────────────────── */
#ifndef MOS_UART_TX_GPIO
#  if CONFIG_IDF_TARGET_ESP32P4
#    define MOS_UART_TX_GPIO  38   /* Waveshare ESP32-P4-WIFI6 header pin */
#  else
#    define MOS_UART_TX_GPIO  17   /* ESP32-S3 DevKit */
#  endif
#endif
#ifndef MOS_UART_RX_GPIO
#  if CONFIG_IDF_TARGET_ESP32P4
#    define MOS_UART_RX_GPIO  37   /* Waveshare ESP32-P4-WIFI6 header pin */
#  else
#    define MOS_UART_RX_GPIO  18   /* ESP32-S3 DevKit */
#  endif
#endif

/**
 * Override TX/RX GPIO pins.  Call before mos_uopen().
 */
void mos_uart_set_pins(int tx_gpio, int rx_gpio);

/**
 * Open the secondary UART at the given baud rate.
 * @param baud  baud rate (e.g. 9600, 115200, 230400)
 * @return 0 on success, -1 on error
 */
int  mos_uopen(uint32_t baud);

/**
 * Close the secondary UART and release its resources.
 */
void mos_uclose(void);

/**
 * Read one byte from the secondary UART (blocking).
 * @return byte value 0-255, or -1 if UART not open
 */
int  mos_ugetc(void);

/**
 * Write one byte to the secondary UART.
 * @param c  byte to send
 * @return 0 on success, -1 if UART not open
 */
int  mos_uputc(uint8_t c);

#endif /* MOS_UART_H */
