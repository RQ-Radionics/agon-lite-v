/*
 * i2c_scan.c - I2C bus scanner for ESP32-MOS
 *
 * Scans all 7-bit I2C addresses (0x08..0x77) and prints which ones
 * respond.  Uses the mos->i2c_* API exposed in mos_api_table.h.
 *
 * Usage from the MOS shell:
 *   i2c_scan          (uses default SDA=8, SCL=9)
 *
 * Compile:
 *   cd sdk && make i2c_scan
 */

#include "mos_api_table.h"

/* Simple output helpers using putch to avoid any libc dependency */
static void put_hex_nibble(t_mos_api *mos, uint8_t n)
{
    n &= 0xF;
    mos->putch(n < 10 ? (uint8_t)('0' + n) : (uint8_t)('A' + n - 10));
}

static void put_hex2(t_mos_api *mos, uint8_t v)
{
    put_hex_nibble(mos, v >> 4);
    put_hex_nibble(mos, v & 0xF);
}

static void put_uint(t_mos_api *mos, uint32_t v)
{
    /* Recursive decimal: no array needed */
    if (v >= 10) put_uint(mos, v / 10);
    mos->putch((uint8_t)('0' + (v % 10)));
}

__attribute__((section(".text.entry")))
int _start(int argc, char **argv, t_mos_api *mos)
{
    (void)argc; (void)argv;

    mos->puts("\r\nI2C Bus Scanner\r\n");
    mos->puts("Scanning addresses 0x08 - 0x77...\r\n\r\n");

    /* MOS_I2C_SPEED_57600 = 0x01 (57.6 kHz, safe for slow devices) */
    mos->i2c_open(0x01);

    int found = 0;

    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        /* Probe: try to read 1 byte.  A NACK means no device at this address. */
        char dummy;
        uint8_t rc = mos->i2c_read(addr, 1, &dummy);

        if (rc == 0x00) { /* MOS_I2C_OK */
            mos->puts("  Found device at 0x");
            put_hex2(mos, addr);
            mos->puts("\r\n");
            found++;
        }
    }

    mos->i2c_close();

    mos->puts("\r\nScan complete. Devices found: ");
    put_uint(mos, (uint32_t)found);
    mos->puts("\r\n");

    return 0;
}
