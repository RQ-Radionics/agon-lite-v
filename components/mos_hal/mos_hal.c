/*
 * mos_hal.c - Hardware Abstraction Layer for ESP32-MOS
 *
 * Console I/O routing:
 *   - If a VDP TCP client is connected: I/O goes through mos_vdp_putch/getch/kbhit.
 *   - Otherwise: I/O goes through UART0 at 115200 baud.
 *
 * The switch is transparent to the shell — it just calls mos_putch/mos_getch.
 */

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "mos_hal.h"
#include "mos_vdp.h"

static const char *TAG = "mos_hal";

#define CONSOLE_UART_NUM    UART_NUM_0
#define CONSOLE_BAUD_RATE   115200
#define CONSOLE_RX_BUF_SZ   1024
#define CONSOLE_TX_BUF_SZ   0       /* tx synchronous */

void mos_hal_console_init(void)
{
    uart_config_t cfg = {
        .baud_rate  = CONSOLE_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(CONSOLE_UART_NUM, &cfg);
    uart_driver_install(CONSOLE_UART_NUM, CONSOLE_RX_BUF_SZ,
                        CONSOLE_TX_BUF_SZ, 0, NULL, 0);

    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stdin,  NULL, _IONBF, 0);

    ESP_LOGI(TAG, "Console UART%d ready at %d baud",
             CONSOLE_UART_NUM, CONSOLE_BAUD_RATE);
}

/* --- UART primitives (always available) --- */

static void uart_putch(char c)
{
    uart_write_bytes(CONSOLE_UART_NUM, &c, 1);
}

static int uart_getch(void)
{
    uint8_t byte;
    while (uart_read_bytes(CONSOLE_UART_NUM, &byte, 1, portMAX_DELAY) < 1) {
        vTaskDelay(1);
    }
    return (int)byte;
}

static bool uart_kbhit(void)
{
    size_t available = 0;
    uart_get_buffered_data_len(CONSOLE_UART_NUM, &available);
    return available > 0;
}

/* --- Public API: routes to VDP or UART --- */

void mos_putch(char c)
{
    if (mos_vdp_connected()) {
        mos_vdp_putch((uint8_t)c);
    } else {
        uart_putch(c);
    }
}

int mos_getch(void)
{
    if (mos_vdp_connected()) {
        int r = mos_vdp_getch();
        /* If VDP disconnected mid-read, fall back to UART */
        if (r < 0) return uart_getch();
        return r;
    }
    return uart_getch();
}

bool mos_kbhit(void)
{
    if (mos_vdp_connected()) {
        return mos_vdp_kbhit();
    }
    return uart_kbhit();
}

void mos_puts(const char *s)
{
    if (!s) return;
    if (mos_vdp_connected()) {
        while (*s) mos_vdp_putch((uint8_t)*s++);
    } else {
        uart_write_bytes(CONSOLE_UART_NUM, s, strlen(s));
    }
}

int mos_printf(const char *fmt, ...)
{
    char buf[512];
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    if (n <= 0) return n;
    int len = (n < (int)sizeof(buf)) ? n : (int)sizeof(buf) - 1;
    if (mos_vdp_connected()) {
        for (int i = 0; i < len; i++) mos_vdp_putch((uint8_t)buf[i]);
    } else {
        uart_write_bytes(CONSOLE_UART_NUM, buf, len);
    }
    return n;
}
