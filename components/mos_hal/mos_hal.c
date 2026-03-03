/*
 * mos_hal.c - Hardware Abstraction Layer for ESP32-MOS
 *
 * Console I/O routing:
 *   - If any VDP backend is connected: I/O goes through mos_vdp_router_*.
 *     The router selects between TCP VDP (mos_vdp) and internal VDP
 *     (mos_vdp_internal) depending on compile-time config and runtime state.
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
#include "mos_vdp_router.h"

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

/* --- Routing helpers --- */

/*
 * use_vdp(): true when I/O should go through the VDP router.
 *
 * When CONFIG_MOS_VDP_INTERNAL_ENABLED is set, the internal VDP is the
 * primary console — always route through the router regardless of whether
 * the HDMI panel initialised successfully.  The router handles the case
 * where the internal VDP has no framebuffer (output is silently dropped
 * on HDMI but the shell still runs; a TCP client can still connect).
 *
 * Without the internal VDP, only route through VDP when a TCP client is
 * actually connected (or disconnecting, to deliver the -1 sentinel).
 */
static inline bool use_vdp(void)
{
#ifdef CONFIG_MOS_VDP_INTERNAL_ENABLED
    return true;   /* always — internal VDP is the console */
#else
    return mos_vdp_router_connected() || mos_vdp_router_disconnecting();
#endif
}

/* use_vdp_output(): same as use_vdp() but excludes the disconnecting state
 * (we don't want to send output to a dead socket). */
static inline bool use_vdp_output(void)
{
#ifdef CONFIG_MOS_VDP_INTERNAL_ENABLED
    return true;
#else
    return mos_vdp_router_connected();
#endif
}

/* --- Public API: routes to VDP or stdio console --- */

void mos_putch(char c)
{
    if (use_vdp_output()) {
        mos_vdp_router_putch((uint8_t)c);
    } else {
        putchar(c);
    }
}

int mos_getch(void)
{
    if (use_vdp()) {
        return mos_vdp_router_getch();
    }
    /* Both USB-JTAG and UART go through the IDF VFS — getchar() works for both. */
    return getchar();
}

bool mos_kbhit(void)
{
    if (use_vdp_output()) {
        return mos_vdp_router_kbhit();
    }
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
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
    if (use_vdp_output()) {
        while (*s) mos_vdp_router_putch((uint8_t)*s++);
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
        if (use_vdp_output()) {
            for (int i = 0; i < len; i++) mos_vdp_router_putch((uint8_t)s_printf_buf[i]);
        } else {
            fwrite(s_printf_buf, 1, len, stdout);
        }
    }
    if (s_printf_mux) xSemaphoreGive(s_printf_mux);
    return n;
}
