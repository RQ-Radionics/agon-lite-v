idf_component_register(
    SRCS "mos_fs.c"
    INCLUDE_DIRS "include"
    REQUIRES fatfs sdmmc driver esp_driver_sdmmc
)
