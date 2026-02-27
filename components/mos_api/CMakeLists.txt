idf_component_register(
    SRCS "mos_api.c"
    INCLUDE_DIRS "include"
    REQUIRES mos_hal mos_fs mos_file mos_sysvars mos_shell mos_strings esp_timer freertos heap mos_i2c
)
