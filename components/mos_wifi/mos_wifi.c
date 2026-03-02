/*
 * mos_wifi.c - WiFi connection manager for ESP32-MOS
 *
 * Connects to the AP defined in wifi_credentials.h using the ESP-IDF
 * WiFi + netif stack.  Uses a FreeRTOS EventGroup to signal when an IP
 * address has been assigned (or when the connection permanently fails).
 *
 * When CONFIG_MOS_WIFI_ENABLED=n this entire file compiles to nothing;
 * the inline stubs in mos_wifi.h satisfy all callers.
 */

#include "sdkconfig.h"
#include "mos_wifi.h"

#if CONFIG_MOS_WIFI_ENABLED

#include <string.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "lwip/ip4_addr.h"

#include "wifi_credentials.h"

static const char *TAG = "mos_wifi";

/* EventGroup bits */
#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1

#define WIFI_MAX_RETRIES    5

static EventGroupHandle_t s_wifi_eg  = NULL;
static bool               s_inited   = false;
static int                s_retries  = 0;
static char               s_ip[16]   = {0};  /* "xxx.xxx.xxx.xxx\0" */

/* ------------------------------------------------------------------ */
/* Event handler                                                        */
/* ------------------------------------------------------------------ */

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    if (base == WIFI_EVENT) {
        switch (id) {
            case WIFI_EVENT_STA_START:
                ESP_LOGI(TAG, "STA started, connecting to '%s'…", WIFI_SSID);
                esp_wifi_connect();
                break;

            case WIFI_EVENT_STA_DISCONNECTED: {
                wifi_event_sta_disconnected_t *ev =
                    (wifi_event_sta_disconnected_t *)data;
                ESP_LOGW(TAG, "Disconnected (reason %d)", ev->reason);
                s_ip[0] = '\0';
                if (s_retries < WIFI_MAX_RETRIES) {
                    s_retries++;
                    ESP_LOGI(TAG, "Retry %d/%d…", s_retries, WIFI_MAX_RETRIES);
                    esp_wifi_connect();
                } else {
                    ESP_LOGE(TAG, "Max retries reached, giving up");
                    xEventGroupSetBits(s_wifi_eg, WIFI_FAIL_BIT);
                }
                break;
            }

            default:
                break;
        }

    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
        snprintf(s_ip, sizeof(s_ip), IPSTR, IP2STR(&ev->ip_info.ip));
        ESP_LOGI(TAG, "Got IP: %s", s_ip);
        s_retries = 0;
        xEventGroupSetBits(s_wifi_eg, WIFI_CONNECTED_BIT);
    }
}

/* ------------------------------------------------------------------ */
/* Public API                                                           */
/* ------------------------------------------------------------------ */

int mos_wifi_init(uint32_t max_wait_ms)
{
    if (s_inited) {
        return mos_wifi_is_connected() ? 0 : -1;
    }

    s_wifi_eg = xEventGroupCreate();
    if (!s_wifi_eg) {
        ESP_LOGE(TAG, "EventGroup alloc failed");
        return -1;
    }

    /* netif + default event loop (safe to call multiple times) */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    /* Register event handlers */
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL, NULL));

    /* Configure STA */
    wifi_config_t wifi_cfg = {0};
    strncpy((char *)wifi_cfg.sta.ssid,     WIFI_SSID,     sizeof(wifi_cfg.sta.ssid)     - 1);
    strncpy((char *)wifi_cfg.sta.password, WIFI_PASSWORD, sizeof(wifi_cfg.sta.password) - 1);
    wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    s_inited = true;
    ESP_LOGI(TAG, "WiFi init done, waiting for IP…");

    /* Wait for connected or fail */
    TickType_t ticks = (max_wait_ms == 0)
                       ? portMAX_DELAY
                       : pdMS_TO_TICKS(max_wait_ms);

    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_eg,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE,   /* don't clear on exit */
        pdFALSE,   /* wait for any bit */
        ticks);

    if (bits & WIFI_CONNECTED_BIT) {
        return 0;
    }
    ESP_LOGW(TAG, "WiFi not connected after init");
    return -1;
}

bool mos_wifi_is_connected(void)
{
    if (!s_wifi_eg) return false;
    return (xEventGroupGetBits(s_wifi_eg) & WIFI_CONNECTED_BIT) != 0;
}

const char *mos_wifi_ip(void)
{
    return (s_ip[0] != '\0') ? s_ip : NULL;
}

#endif /* CONFIG_MOS_WIFI_ENABLED */
