/*
 * mos_wifi.c - WiFi connection manager for ESP32-MOS
 *
 * Credentials are read at boot from /sdcard/wifi.cfg (simple key=value format):
 *   ssid=MyNetwork
 *   password=MySecret
 *   tz=CET-1CEST,M3.5.0,M10.5.0/3   (optional — POSIX TZ string, default UTC0)
 *
 * If the file is absent or unreadable, falls back to wifi_credentials.h.
 * The 'wifi' shell command connects with new credentials and saves them.
 */

#include "sdkconfig.h"
#include "mos_wifi.h"

#if CONFIG_MOS_WIFI_ENABLED

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "lwip/ip4_addr.h"

#include "mos_fs.h"
#include "wifi_credentials.h"

static const char *TAG = "mos_wifi";

#define WIFI_CFG_PATH       MOS_SD_MOUNT "/wifi.cfg"
#define WIFI_MAX_RETRIES    5

/* EventGroup bits */
#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1

static EventGroupHandle_t s_wifi_eg  = NULL;
static bool               s_inited   = false;
static int                s_retries  = 0;
static char               s_ip[16]   = {0};
static char               s_tz[64]   = {0};   /* POSIX TZ string from wifi.cfg */

/* ------------------------------------------------------------------ */
/* Internal: read wifi.cfg from SD                                      */
/* ------------------------------------------------------------------ */

static void load_cfg(char *ssid, size_t ssid_sz, char *pass, size_t pass_sz)
{
    /* Start with compiled-in fallback */
    strncpy(ssid, WIFI_SSID,     ssid_sz - 1);  ssid[ssid_sz - 1] = '\0';
    strncpy(pass, WIFI_PASSWORD, pass_sz - 1);  pass[pass_sz - 1] = '\0';
    s_tz[0] = '\0';

    FILE *f = fopen(WIFI_CFG_PATH, "r");
    if (!f) {
        ESP_LOGI(TAG, "No wifi.cfg on SD — using compiled-in credentials");
        return;
    }

    char line[128];
    bool got_ssid = false, got_pass = false;
    while (fgets(line, sizeof(line), f)) {
        /* Strip trailing newline/CR */
        size_t len = strlen(line);
        while (len > 0 && (line[len-1] == '\n' || line[len-1] == '\r'))
            line[--len] = '\0';

        if (strncmp(line, "ssid=", 5) == 0) {
            strncpy(ssid, line + 5, ssid_sz - 1);
            ssid[ssid_sz - 1] = '\0';
            got_ssid = true;
        } else if (strncmp(line, "password=", 9) == 0) {
            strncpy(pass, line + 9, pass_sz - 1);
            pass[pass_sz - 1] = '\0';
            got_pass = true;
        } else if (strncmp(line, "tz=", 3) == 0) {
            strncpy(s_tz, line + 3, sizeof(s_tz) - 1);
            s_tz[sizeof(s_tz) - 1] = '\0';
            ESP_LOGI(TAG, "Timezone from wifi.cfg: '%s'", s_tz);
        }
    }
    fclose(f);

    if (got_ssid || got_pass) {
        ESP_LOGI(TAG, "Loaded WiFi config from SD: ssid='%s'", ssid);
    } else {
        ESP_LOGW(TAG, "wifi.cfg found but no ssid=/password= lines — using fallback");
    }
}

/* ------------------------------------------------------------------ */
/* Event handler                                                        */
/* ------------------------------------------------------------------ */

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    if (base == WIFI_EVENT) {
        switch (id) {
            case WIFI_EVENT_STA_START:
                ESP_LOGI(TAG, "STA started, connecting…");
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
                    ESP_LOGE(TAG, "Max retries reached");
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
/* Internal: apply credentials and (re)connect                          */
/* ------------------------------------------------------------------ */

static int do_connect(const char *ssid, const char *password, uint32_t max_wait_ms)
{
    /* Reset retry counter and event bits for a fresh attempt */
    s_retries = 0;
    xEventGroupClearBits(s_wifi_eg, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);
    s_ip[0] = '\0';

    wifi_config_t wifi_cfg = {0};
    strncpy((char *)wifi_cfg.sta.ssid,     ssid,     sizeof(wifi_cfg.sta.ssid)     - 1);
    strncpy((char *)wifi_cfg.sta.password, password, sizeof(wifi_cfg.sta.password) - 1);
    wifi_cfg.sta.threshold.authmode = (password[0] == '\0')
                                      ? WIFI_AUTH_OPEN
                                      : WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_disconnect());
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_connect());

    TickType_t ticks = (max_wait_ms == 0) ? portMAX_DELAY
                                          : pdMS_TO_TICKS(max_wait_ms);
    EventBits_t bits = xEventGroupWaitBits(s_wifi_eg,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE, pdFALSE, ticks);
    if (bits & WIFI_CONNECTED_BIT) return 0;
    ESP_LOGW(TAG, "WiFi not connected (ssid='%s')", ssid);
    return -1;
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

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL, NULL));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    s_inited = true;

    /* Load credentials: SD file or fallback */
    char ssid[33] = {0}, pass[64] = {0};
    load_cfg(ssid, sizeof(ssid), pass, sizeof(pass));

    ESP_LOGI(TAG, "Connecting to '%s'…", ssid);
    return do_connect(ssid, pass, max_wait_ms);
}

int mos_wifi_connect(const char *ssid, const char *password, uint32_t max_wait_ms)
{
    if (!s_inited) {
        /* First call: initialise stack then connect with given credentials */
        s_wifi_eg = xEventGroupCreate();
        if (!s_wifi_eg) return -1;

        ESP_ERROR_CHECK(esp_netif_init());
        ESP_ERROR_CHECK(esp_event_loop_create_default());
        esp_netif_create_default_wifi_sta();

        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));

        ESP_ERROR_CHECK(esp_event_handler_instance_register(
            WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL, NULL));
        ESP_ERROR_CHECK(esp_event_handler_instance_register(
            IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL, NULL));

        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

        /* Configure before start so STA_START triggers connect immediately */
        wifi_config_t wifi_cfg = {0};
        strncpy((char *)wifi_cfg.sta.ssid,     ssid,     sizeof(wifi_cfg.sta.ssid)     - 1);
        strncpy((char *)wifi_cfg.sta.password, password, sizeof(wifi_cfg.sta.password) - 1);
        wifi_cfg.sta.threshold.authmode = (password[0] == '\0')
                                          ? WIFI_AUTH_OPEN
                                          : WIFI_AUTH_WPA2_PSK;
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
        ESP_ERROR_CHECK(esp_wifi_start());
        s_inited = true;

        TickType_t ticks = (max_wait_ms == 0) ? portMAX_DELAY
                                              : pdMS_TO_TICKS(max_wait_ms);
        EventBits_t bits = xEventGroupWaitBits(s_wifi_eg,
                                               WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                               pdFALSE, pdFALSE, ticks);
        if (bits & WIFI_CONNECTED_BIT) return 0;
        return -1;
    }

    /* Already initialised: reconfigure and reconnect */
    return do_connect(ssid, password, max_wait_ms);
}

int mos_wifi_save_config(const char *ssid, const char *password)
{
    FILE *f = fopen(WIFI_CFG_PATH, "w");
    if (!f) {
        ESP_LOGE(TAG, "Cannot write %s", WIFI_CFG_PATH);
        return -1;
    }
    fprintf(f, "ssid=%s\npassword=%s\n", ssid, password);
    fclose(f);
    ESP_LOGI(TAG, "WiFi config saved to %s", WIFI_CFG_PATH);
    return 0;
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

const char *mos_wifi_get_tz(void)
{
    return (s_tz[0] != '\0') ? s_tz : NULL;
}

#endif /* CONFIG_MOS_WIFI_ENABLED */
