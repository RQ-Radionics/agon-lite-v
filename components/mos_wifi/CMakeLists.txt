idf_component_register(
    SRCS "mos_wifi.c"
    INCLUDE_DIRS "include"
    # mos_wifi.h only needs sdkconfig.h + stdbool (implicit); no IDF public deps.
    REQUIRES ""
    # esp_wifi / esp_netif / esp_event are only referenced inside mos_wifi.c,
    # and only when CONFIG_MOS_WIFI_ENABLED=y (i.e. CONFIG_MOS_NET_WIFI=y).
    # When the Ethernet backend is selected the #if guard omits all that code,
    # so the linker never pulls in the SDIO/esp-hosted driver.
    PRIV_REQUIRES esp_wifi esp_netif esp_event mos_fs
)
