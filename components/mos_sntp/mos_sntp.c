/*
 * mos_sntp.c - SNTP time synchronisation for ESP32-MOS
 *
 * Uses the esp_sntp component (lwIP SNTP client).
 * Waits for the first successful sync before returning.
 */

#include <stdint.h>
#include <string.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_sntp.h"
#include "esp_log.h"

#include "mos_sntp.h"

static const char *TAG = "mos_sntp";

#define SNTP_SERVER     "pool.ntp.org"
#define SNTP_POLL_MS    100
#define SNTP_RETRY_MAX  100     /* 100 * 100 ms = 10 s per retry cycle */

static volatile bool s_synced = false;

static void sntp_sync_cb(struct timeval *tv)
{
    (void)tv;
    s_synced = true;
    char buf[32];
    time_t now = time(NULL);
    struct tm *t = localtime(&now);
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", t);
    ESP_LOGI(TAG, "Clock synced: %s", buf);
}

int mos_sntp_init(const char *tz_posix, uint32_t max_wait_ms)
{
    /* Set timezone */
    const char *tz = (tz_posix && tz_posix[0]) ? tz_posix : "UTC0";
    setenv("TZ", tz, 1);
    tzset();
    ESP_LOGI(TAG, "Timezone: %s", tz);

    /* Configure SNTP */
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, SNTP_SERVER);
    sntp_set_time_sync_notification_cb(sntp_sync_cb);
    esp_sntp_init();

    ESP_LOGI(TAG, "SNTP started, server: %s", SNTP_SERVER);

    /* Wait for sync */
    uint32_t elapsed = 0;
    while (!s_synced) {
        vTaskDelay(pdMS_TO_TICKS(SNTP_POLL_MS));
        elapsed += SNTP_POLL_MS;
        if (max_wait_ms > 0 && elapsed >= max_wait_ms) {
            ESP_LOGW(TAG, "SNTP sync timeout after %u ms", elapsed);
            return -1;
        }
    }
    return 0;
}

bool mos_sntp_is_synced(void)
{
    return s_synced;
}
