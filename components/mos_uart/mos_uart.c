/*
 * mos_uart.c - Secondary UART driver for ESP32-MOS
 *
 * Wraps the ESP-IDF uart driver on UART_NUM_1.
 * UART_NUM_0 is reserved for the debug/VDP console (mos_hal.c).
 */

#include <string.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "mos_uart.h"

static const char *TAG = "mos_uart";

#define MOS_UART_PORT       UART_NUM_1
#define MOS_UART_RX_BUF_SZ  256
#define MOS_UART_TX_BUF_SZ  0       /* synchronous TX */

static int  s_tx_gpio   = MOS_UART_TX_GPIO;
static int  s_rx_gpio   = MOS_UART_RX_GPIO;
static bool s_open      = false;

void mos_uart_set_pins(int tx_gpio, int rx_gpio)
{
    s_tx_gpio = tx_gpio;
    s_rx_gpio = rx_gpio;
}

int mos_uopen(uint32_t baud)
{
    if (s_open) {
        /* already open — reconfigure baud rate without full reinstall */
        uart_set_baudrate(MOS_UART_PORT, baud);
        return 0;
    }

    uart_config_t cfg = {
        .baud_rate  = (int)baud,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    };

    esp_err_t err = uart_param_config(MOS_UART_PORT, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed: %s", esp_err_to_name(err));
        return -1;
    }

    err = uart_set_pin(MOS_UART_PORT, s_tx_gpio, s_rx_gpio,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed: %s", esp_err_to_name(err));
        return -1;
    }

    err = uart_driver_install(MOS_UART_PORT, MOS_UART_RX_BUF_SZ,
                              MOS_UART_TX_BUF_SZ, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(err));
        return -1;
    }

    s_open = true;
    ESP_LOGI(TAG, "UART1 open: TX=GPIO%d RX=GPIO%d %lu baud",
             s_tx_gpio, s_rx_gpio, (unsigned long)baud);
    return 0;
}

void mos_uclose(void)
{
    if (!s_open) return;
    uart_driver_delete(MOS_UART_PORT);
    s_open = false;
    ESP_LOGI(TAG, "UART1 closed");
}

int mos_ugetc(void)
{
    if (!s_open) return -1;
    uint8_t byte;
    int n = uart_read_bytes(MOS_UART_PORT, &byte, 1, portMAX_DELAY);
    return (n == 1) ? (int)byte : -1;
}

int mos_uputc(uint8_t c)
{
    if (!s_open) return -1;
    int n = uart_write_bytes(MOS_UART_PORT, (const char *)&c, 1);
    return (n == 1) ? 0 : -1;
}
