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
#include "esp_vfs_dev.h"
#include "esp_vfs_usb_serial_jtag.h"
#include "driver/usb_serial_jtag.h"
#include "driver/uart_vfs.h"
#include "driver/uart.h"
#include "mos_hal.h"
#include "mos_vdp.h"

static const char *TAG = "mos_hal";

#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
/* One-byte pushback buffer for kbhit() + getch() coordination on USB-JTAG */
static int  s_jtag_pushed  = -1;
#endif

/* Static printf buffer — keeps vsnprintf off the task stack.
 * Protected by a mutex so mos_printf is safe from multiple tasks. */
#define MOS_PRINTF_BUF  512
static char             s_printf_buf[MOS_PRINTF_BUF];
static SemaphoreHandle_t s_printf_mux = NULL;

void mos_hal_console_init(void)
{
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    /* USB-JTAG CDC console (e.g. Olimex ESP32-P4-PC) */
    esp_vfs_usb_serial_jtag_use_driver();
    ESP_LOGI(TAG, "Console USB-JTAG/CDC ready");
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
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    /* Return byte saved by kbhit(), or block-read a fresh one. */
    if (s_jtag_pushed >= 0) {
        int c = s_jtag_pushed;
        s_jtag_pushed = -1;
        return c;
    }
    uint8_t b;
    while (usb_serial_jtag_read_bytes(&b, 1, portMAX_DELAY) < 1) {}
    return (int)b;
#else
    return getchar();
#endif
}

bool mos_kbhit(void)
{
    if (mos_vdp_connected()) {
        return mos_vdp_kbhit();
    }
    /* Non-blocking check: peek at the underlying driver's RX buffer. */
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    if (s_jtag_pushed >= 0) return true;
    uint8_t peek;
    if (usb_serial_jtag_read_bytes(&peek, 1, 0) > 0) {
        s_jtag_pushed = peek;
        return true;
    }
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
