/*
 * mos_i2c.h - I2C driver for ESP32-MOS
 *
 * Replaces the eZ80-specific i2c.c (hardware ISR, PPD register) with the
 * ESP-IDF i2c_master driver.  The public interface matches the original
 * agon-mos API:
 *
 *   mos_I2C_OPEN(frequency)          — initialise bus
 *   mos_I2C_CLOSE()                  — release bus
 *   mos_I2C_WRITE(addr, size, buf)   — master write
 *   mos_I2C_READ(addr, size, buf)    — master read
 *
 * Frequency constants (same numeric values as original i2c.h):
 *   MOS_I2C_SPEED_57600  (0x01) →  57.6 kHz (standard mode)
 *   MOS_I2C_SPEED_115200 (0x02) → 115.2 kHz (fast mode)
 *   MOS_I2C_SPEED_230400 (0x03) → 230.4 kHz (fast mode)
 *
 * Return codes (same values as original RET_* defines):
 *   MOS_I2C_OK          (0x00) — success
 *   MOS_I2C_NORESPONSE  (0x01) — device did not ACK (not on bus)
 *   MOS_I2C_DATA_NACK   (0x02) — data phase NACK
 *   MOS_I2C_ARB_LOST    (0x04) — arbitration lost / timeout
 *   MOS_I2C_BUS_ERROR   (0x08) — bus error
 *
 * GPIO pins — configure once in sdkconfig or pass explicit values to
 * mos_i2c_init_pins() before calling mos_I2C_OPEN().  Defaults:
 *   ESP32-P4 (Waveshare ESP32-P4-WIFI6): SDA = GPIO 7, SCL = GPIO 8
 *   ESP32-S3 DevKit:                      SDA = GPIO 8, SCL = GPIO 9
 */

#ifndef MOS_I2C_H
#define MOS_I2C_H

#include <stdint.h>

/* ── Speed constants (compatible with original agon-mos i2c.h) ────────── */
#define MOS_I2C_SPEED_57600   0x01   /*  57.6 kHz — standard mode  */
#define MOS_I2C_SPEED_115200  0x02   /* 115.2 kHz — fast mode      */
#define MOS_I2C_SPEED_230400  0x03   /* 230.4 kHz — fast mode      */

/* ── Return codes (compatible with original RET_* defines) ────────────── */
#define MOS_I2C_OK          0x00
#define MOS_I2C_NORESPONSE  0x01
#define MOS_I2C_DATA_NACK   0x02
#define MOS_I2C_ARB_LOST    0x04
#define MOS_I2C_BUS_ERROR   0x08

/* ── Default GPIO pins ─────────────────────────────────────────────────── */
#ifndef MOS_I2C_SDA_GPIO
  #if CONFIG_IDF_TARGET_ESP32P4
    #define MOS_I2C_SDA_GPIO  7   /* Waveshare ESP32-P4-WIFI6 (40-pin header) */
  #else
    #define MOS_I2C_SDA_GPIO  8   /* ESP32-S3 DevKit */
  #endif
#endif
#ifndef MOS_I2C_SCL_GPIO
  #if CONFIG_IDF_TARGET_ESP32P4
    #define MOS_I2C_SCL_GPIO  8   /* Waveshare ESP32-P4-WIFI6 (40-pin header) */
  #else
    #define MOS_I2C_SCL_GPIO  9   /* ESP32-S3 DevKit */
  #endif
#endif

/* ── Optional: override GPIO pins before calling mos_I2C_OPEN() ────────── */
void    mos_i2c_set_pins(int sda_gpio, int scl_gpio);

/* ── Public API (mirrors agon-mos i2c.h) ──────────────────────────────── */

/**
 * Open / initialise the I2C bus.
 * @param frequency  MOS_I2C_SPEED_* constant (or 0 → default 57.6 kHz)
 */
void    mos_I2C_OPEN(uint8_t frequency);

/**
 * Close / release the I2C bus.
 */
void    mos_I2C_CLOSE(void);

/**
 * Write bytes to an I2C slave.
 * @param i2c_address  7-bit slave address
 * @param size         number of bytes to send (max 32)
 * @param buffer       data to send
 * @return  MOS_I2C_OK or error code
 */
uint8_t mos_I2C_WRITE(uint8_t i2c_address, uint8_t size, const char *buffer);

/**
 * Read bytes from an I2C slave.
 * @param i2c_address  7-bit slave address
 * @param size         number of bytes to read (max 32)
 * @param buffer       destination buffer (must be at least size bytes)
 * @return  MOS_I2C_OK or error code
 */
uint8_t mos_I2C_READ(uint8_t i2c_address, uint8_t size, char *buffer);

#endif /* MOS_I2C_H */
