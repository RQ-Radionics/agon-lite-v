idf_component_register(
    SRCS "mos_vdp.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "../mos_api/include"
    REQUIRES lwip esp_netif
)
