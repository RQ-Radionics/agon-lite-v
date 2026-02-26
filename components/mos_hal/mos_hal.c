/*
 * mos_hal.c - Hardware Abstraction Layer for ESP32-MOS
 *
 * Console I/O using ESP-IDF UART driver directly for non-blocking kbhit().
 * UART0 is used as the interactive console at 115200 baud.
 */

#include <stdio.h>
#include <stdarg.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "mos_hal.h"

static const char *TAG = "mos_hal";

#define CONSOLE_UART_NUM    UART_NUM_0
#define CONSOLE_BAUD_RATE   115200
#define CONSOLE_RX_BUF_SZ   1024
#define CONSOLE_TX_BUF_SZ   0       /* tx synchronous */

void mos_hal_console_init(void)
{
    /* Install UART driver so we can use uart_read_bytes for non-blocking I/O */
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

    /* Disable buffering on the C stdio layer too */
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stdin,  NULL, _IONBF, 0);

    ESP_LOGI(TAG, "Console UART%d ready at %d baud",
             CONSOLE_UART_NUM, CONSOLE_BAUD_RATE);
}

void mos_putch(char c)
{
    uart_write_bytes(CONSOLE_UART_NUM, &c, 1);
}

int mos_getch(void)
{
    uint8_t byte;
    /* Block until a byte arrives */
    while (uart_read_bytes(CONSOLE_UART_NUM, &byte, 1, portMAX_DELAY) < 1) {
        vTaskDelay(1);
    }
    return (int)byte;
}

bool mos_kbhit(void)
{
    size_t available = 0;
    uart_get_buffered_data_len(CONSOLE_UART_NUM, &available);
    return available > 0;
}

void mos_puts(const char *s)
{
    if (!s) return;
    uart_write_bytes(CONSOLE_UART_NUM, s, strlen(s));
}

int mos_printf(const char *fmt, ...)
{
    char buf[512];
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    if (n > 0) {
        uart_write_bytes(CONSOLE_UART_NUM, buf,
                         (n < (int)sizeof(buf)) ? n : (int)sizeof(buf) - 1);
    }
    return n;
}
