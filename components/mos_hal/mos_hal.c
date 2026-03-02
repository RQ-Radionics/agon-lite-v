/*
 * mos_hal.c - Hardware Abstraction Layer for ESP32-MOS
 *
 * Console I/O routing:
 *   - If a VDP TCP client is connected: I/O goes through mos_vdp_putch/getch/kbhit.
 *   - Otherwise: I/O goes through the IDF-configured console (UART0 or USB-JTAG CDC),
 *     using libc stdio so the same code works on all boards without #ifdefs.
 *
 * The switch is transparent to the shell — it just calls mos_putch/mos_getch.
 */

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/uart_vfs.h"
#include "driver/uart.h"
#include "mos_hal.h"
#include "mos_vdp.h"

static const char *TAG = "mos_hal";

/* Static printf buffer — keeps vsnprintf off the task stack.
 * Protected by a mutex so mos_printf is safe from multiple tasks. */
#define MOS_PRINTF_BUF  512
static char             s_printf_buf[MOS_PRINTF_BUF];
static SemaphoreHandle_t s_printf_mux = NULL;

void mos_hal_console_init(void)
{
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    /* USB-JTAG CDC console (e.g. Olimex ESP32-P4-PC).
     * IDF configures the USB-JTAG VFS automatically when
     * CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y — do NOT call
     * esp_vfs_usb_serial_jtag_use_driver() here; that would
     * re-install the driver and conflict with the already-running
     * IDF logging infrastructure, breaking stdin/stdout. */
    ESP_LOGI(TAG, "Console USB-JTAG/CDC (IDF-managed)");
#else
    /* UART console (default for most boards) */
    uart_config_t cfg = {
        .baud_rate  = CONFIG_ESP_CONSOLE_UART_BAUDRATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(CONFIG_ESP_CONSOLE_UART_NUM, &cfg);
    uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM, 1024, 0, 0, NULL, 0);
    uart_vfs_dev_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);
    ESP_LOGI(TAG, "Console UART%d ready at %d baud",
             CONFIG_ESP_CONSOLE_UART_NUM, CONFIG_ESP_CONSOLE_UART_BAUDRATE);
#endif

    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stdin,  NULL, _IONBF, 0);

    s_printf_mux = xSemaphoreCreateMutex();
}

/* --- Public API: routes to VDP or stdio console --- */

void mos_putch(char c)
{
    if (mos_vdp_connected()) {
        mos_vdp_putch((uint8_t)c);
    } else {
        putchar(c);
    }
}

int mos_getch(void)
{
    if (mos_vdp_connected()) {
        /* -1 means VDP disconnected — propagate it, do NOT fall back to console.
         * The shell / editor will see -1 and return, allowing the session loop
         * in main.c to wait for the next VDP connection. */
        return mos_vdp_getch();
    }
    /* Both USB-JTAG and UART go through the IDF VFS — getchar() works for both. */
    return getchar();
}

bool mos_kbhit(void)
{
    if (mos_vdp_connected()) {
        return mos_vdp_kbhit();
    }
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    /* USB-JTAG: stdin is non-blocking when IDF manages the driver.
     * Use select() timeout=0 would be ideal, but IDF USB-JTAG VFS does not
     * support select.  Poll with a 0-tick read via the raw driver is gone;
     * instead we simply report false (no key) — the shell loop will call
     * mos_getch() which blocks until a key arrives.  This is acceptable
     * for interactive use; autoexec.bat doesn't use kbhit. */
    return false;
#else
    size_t available = 0;
    uart_get_buffered_data_len(CONFIG_ESP_CONSOLE_UART_NUM, &available);
    return available > 0;
#endif
}

void mos_puts(const char *s)
{
    if (!s) return;
    if (mos_vdp_connected()) {
        while (*s) mos_vdp_putch((uint8_t)*s++);
    } else {
        fputs(s, stdout);
    }
}

int mos_printf(const char *fmt, ...)
{
    if (s_printf_mux) xSemaphoreTake(s_printf_mux, portMAX_DELAY);
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(s_printf_buf, MOS_PRINTF_BUF, fmt, args);
    va_end(args);
    if (n > 0) {
        int len = (n < MOS_PRINTF_BUF) ? n : MOS_PRINTF_BUF - 1;
        if (mos_vdp_connected()) {
            for (int i = 0; i < len; i++) mos_vdp_putch((uint8_t)s_printf_buf[i]);
        } else {
            fwrite(s_printf_buf, 1, len, stdout);
        }
    }
    if (s_printf_mux) xSemaphoreGive(s_printf_mux);
    return n;
}
