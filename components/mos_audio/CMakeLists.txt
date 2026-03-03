idf_component_register(
    SRCS "mos_audio.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_i2s esp_driver_i2c esp_driver_gpio log esp_common
    PRIV_REQUIRES espressif__esp_codec_dev
)
