idf_component_register(
    SRCS "mos_sntp.c"
    INCLUDE_DIRS "include"
    REQUIRES lwip esp_netif
)
