if(CONFIG_MOS_AUDIO_ENABLED)
    idf_component_register(
        SRCS "mos_audio.c" "es8311_local.c"
        INCLUDE_DIRS "."          # for es8311_local.h
                     "include"    # for mos_audio.h
        REQUIRES esp_driver_i2s esp_driver_i2c esp_driver_gpio log esp_common
    )
else()
    # Stub: no sources, but deps declared so IDF header scanner (which ignores
    # #ifdef guards) does not complain about includes in the disabled .c files.
    idf_component_register(
        INCLUDE_DIRS "."
                     "include"
        REQUIRES esp_driver_i2s esp_driver_i2c esp_driver_gpio log esp_common
    )
endif()
