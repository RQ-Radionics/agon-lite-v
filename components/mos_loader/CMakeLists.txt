idf_component_register(
    SRCS "mos_loader.c"
    INCLUDE_DIRS "include"
    REQUIRES mos_fs mos_api mos_vdp heap esp_mm freertos
)
