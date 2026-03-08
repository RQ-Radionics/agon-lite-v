/*
 * mos_net.c — Network abstraction layer for ESP32-MOS
 *
 * Dispatches to the backend selected by Kconfig:
 *   MOS_NET_WIFI     → mos_wifi (WiFi via C6 coprocessor over SDIO)
 *   MOS_NET_ETHERNET → EMAC + IP101GRR PHY (RMII, external clock)
 *   MOS_NET_NONE     → stubs, no-op
 */

#include "sdkconfig.h"
#include "mos_net.h"
#include "esp_log.h"

static const char *TAG = "mos_net";

/* ================================================================== */
/* Backend: WiFi                                                        */
/* ================================================================== */

#if defined(CONFIG_MOS_NET_WIFI)

#include "mos_wifi.h"

int mos_net_init(uint32_t max_wait_ms)
{
    return mos_wifi_init(max_wait_ms);
}

bool mos_net_is_connected(void)
{
    return mos_wifi_is_connected();
}

const char *mos_net_ip(void)
{
    return mos_wifi_ip();
}

const char *mos_net_get_tz(void)
{
    return mos_wifi_get_tz();
}

/* ================================================================== */
/* Backend: Ethernet (RMII, IP101GRR)                                  */
/* ================================================================== */

#elif defined(CONFIG_MOS_NET_ETHERNET)

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_eth.h"
#include "esp_eth_mac.h"
#include "esp_eth_phy.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "lwip/ip4_addr.h"

#define ETH_CONNECTED_BIT  BIT0
#define ETH_FAIL_BIT       BIT1

static EventGroupHandle_t s_eth_eg      = NULL;
static bool               s_infra_up   = false;  /* netif + event loop created */
static bool               s_driver_up  = false;  /* eth driver installed & started */
static esp_netif_t       *s_eth_netif  = NULL;
static char               s_ip[16]     = {0};

static void eth_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    if (base == ETH_EVENT) {
        switch (id) {
            case ETHERNET_EVENT_CONNECTED:
                ESP_LOGI(TAG, "Ethernet link up");
                break;
            case ETHERNET_EVENT_DISCONNECTED:
                ESP_LOGW(TAG, "Ethernet link down");
                s_ip[0] = '\0';
                xEventGroupSetBits(s_eth_eg, ETH_FAIL_BIT);
                break;
            case ETHERNET_EVENT_START:
                ESP_LOGI(TAG, "Ethernet started");
                break;
            case ETHERNET_EVENT_STOP:
                ESP_LOGI(TAG, "Ethernet stopped");
                break;
            default:
                break;
        }
    } else if (base == IP_EVENT && id == IP_EVENT_ETH_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
        snprintf(s_ip, sizeof(s_ip), IPSTR, IP2STR(&ev->ip_info.ip));
        ESP_LOGI(TAG, "Ethernet got IP: %s", s_ip);
        xEventGroupSetBits(s_eth_eg, ETH_CONNECTED_BIT);
    }
}

int mos_net_init(uint32_t max_wait_ms)
{
    /* Step 1: one-time infrastructure (event loop, netif, event handlers).
     * These cannot be called twice — guard with s_infra_up. */
    if (!s_infra_up) {
        s_eth_eg = xEventGroupCreate();
        if (!s_eth_eg) {
            ESP_LOGE(TAG, "EventGroup alloc failed");
            return -1;
        }
        ESP_ERROR_CHECK(esp_netif_init());
        ESP_ERROR_CHECK(esp_event_loop_create_default());

        esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
        s_eth_netif = esp_netif_new(&netif_cfg);
        if (!s_eth_netif) {
            ESP_LOGE(TAG, "Failed to create Ethernet netif");
            return -1;
        }
        ESP_ERROR_CHECK(esp_event_handler_register(
            ETH_EVENT, ESP_EVENT_ANY_ID, eth_event_handler, NULL));
        ESP_ERROR_CHECK(esp_event_handler_register(
            IP_EVENT, IP_EVENT_ETH_GOT_IP, eth_event_handler, NULL));

        s_infra_up = true;

        /* Step 2: MAC + PHY + driver — also one-time, but separated so infra
         * is committed before we attempt the driver (which may fail on bad
         * GPIO config without corrupting the event loop state). */
        eth_esp32_emac_config_t emac_cfg = ETH_ESP32_EMAC_DEFAULT_CONFIG();
        emac_cfg.smi_gpio.mdc_num  = CONFIG_MOS_ETH_MDC_GPIO;
        emac_cfg.smi_gpio.mdio_num = CONFIG_MOS_ETH_MDIO_GPIO;
        emac_cfg.clock_config.rmii.clock_mode = EMAC_CLK_EXT_IN;
        emac_cfg.clock_config.rmii.clock_gpio = CONFIG_MOS_ETH_RMII_CLK_GPIO;
        ESP_LOGI(TAG, "EMAC: MDC=GPIO%d MDIO=GPIO%d RMII_CLK=GPIO%d",
                 CONFIG_MOS_ETH_MDC_GPIO, CONFIG_MOS_ETH_MDIO_GPIO,
                 CONFIG_MOS_ETH_RMII_CLK_GPIO);

        eth_mac_config_t mac_cfg = ETH_MAC_DEFAULT_CONFIG();
        esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&emac_cfg, &mac_cfg);
        if (!mac) {
            ESP_LOGE(TAG, "Failed to create EMAC");
            return -1;
        }

        /* Use AUTO address so the driver scans all 32 addresses.
         * If the correct address is known, set CONFIG_MOS_ETH_PHY_ADDR.  */
        eth_phy_config_t phy_cfg = ETH_PHY_DEFAULT_CONFIG();
        phy_cfg.phy_addr       = ESP_ETH_PHY_ADDR_AUTO;
        phy_cfg.reset_gpio_num = CONFIG_MOS_ETH_PHY_RST_GPIO;
        ESP_LOGI(TAG, "PHY: addr=AUTO reset_gpio=%d", CONFIG_MOS_ETH_PHY_RST_GPIO);

        esp_eth_phy_t *phy = esp_eth_phy_new_ip101(&phy_cfg);
        if (!phy) {
            ESP_LOGE(TAG, "Failed to create IP101 PHY object");
            return -1;
        }

        esp_eth_config_t eth_cfg = ETH_DEFAULT_CONFIG(mac, phy);
        esp_eth_handle_t eth_handle = NULL;
        esp_err_t err = esp_eth_driver_install(&eth_cfg, &eth_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Ethernet driver install failed: %s", esp_err_to_name(err));
            return -1;
        }

        esp_eth_netif_glue_handle_t glue = esp_eth_new_netif_glue(eth_handle);
        ESP_ERROR_CHECK(esp_netif_attach(s_eth_netif, glue));
        ESP_ERROR_CHECK(esp_eth_start(eth_handle));
        s_driver_up = true;
        ESP_LOGI(TAG, "Ethernet driver started");
    }

    if (!s_driver_up) {
        /* infra is up but driver failed — cannot retry driver without restart */
        ESP_LOGE(TAG, "Ethernet driver not available");
        return -1;
    }

    /* Clear FAIL bit so we get a fresh wait each call */
    xEventGroupClearBits(s_eth_eg, ETH_FAIL_BIT);

    ESP_LOGI(TAG, "Waiting for link+IP (timeout %u ms)…", max_wait_ms);
    TickType_t ticks = (max_wait_ms == 0)
                       ? portMAX_DELAY
                       : pdMS_TO_TICKS(max_wait_ms);

    EventBits_t bits = xEventGroupWaitBits(
        s_eth_eg,
        ETH_CONNECTED_BIT | ETH_FAIL_BIT,
        pdFALSE, pdFALSE, ticks);

    if (bits & ETH_CONNECTED_BIT) {
        return 0;
    }

    ESP_LOGW(TAG, "Ethernet: no IP yet (cable unplugged?)");
    return -1;
}

bool mos_net_is_connected(void)
{
    if (!s_eth_eg) return false;
    return (xEventGroupGetBits(s_eth_eg) & ETH_CONNECTED_BIT) != 0;
}

const char *mos_net_ip(void)
{
    return (s_ip[0] != '\0') ? s_ip : NULL;
}

const char *mos_net_get_tz(void)
{
    return NULL;   /* Ethernet backend: no TZ config */
}

/* ================================================================== */
/* Backend: None                                                        */
/* ================================================================== */

#else /* MOS_NET_NONE */

int  mos_net_init(uint32_t max_wait_ms) { (void)max_wait_ms; return -1; }
bool mos_net_is_connected(void)         { return false; }
const char *mos_net_ip(void)            { return NULL; }
const char *mos_net_get_tz(void)        { return NULL; }

#endif
