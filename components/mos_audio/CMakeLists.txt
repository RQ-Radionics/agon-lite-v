if(CONFIG_MOS_AUDIO_ENABLED)
    idf_component_register(
        SRCS "mos_audio.c"
        INCLUDE_DIRS "include"
        REQUIRES esp_driver_i2s esp_driver_i2c esp_driver_gpio log esp_common
        PRIV_REQUIRES espressif__esp_codec_dev
    )
else()
    # Stub: no sources, but deps declared so IDF header scanner (which ignores
    # #ifdef guards) does not complain about includes in the disabled .c files.
    idf_component_register(
        INCLUDE_DIRS "include"
        REQUIRES esp_driver_i2s esp_driver_i2c esp_driver_gpio log esp_common
        PRIV_REQUIRES espressif__esp_codec_dev
    )
endif()
