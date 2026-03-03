idf_component_register(
    SRCS
        "mos_vdp_internal.cpp"
        "vdp_font.c"
    INCLUDE_DIRS "include"
    REQUIRES
        esp_lcd
        log
        freertos
        driver
    PRIV_REQUIRES
        esp_timer
        heap
        esp_mm
)
