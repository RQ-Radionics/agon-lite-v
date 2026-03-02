idf_component_register(
    SRCS "mos_net.c"
    INCLUDE_DIRS "include"
    # Public: only what mos_net.h exposes (nothing from IDF — just stdint/stdbool)
    REQUIRES ""
    # Private: what mos_net.c needs internally, varies by backend
    PRIV_REQUIRES esp_netif esp_event esp_eth mos_wifi
)
