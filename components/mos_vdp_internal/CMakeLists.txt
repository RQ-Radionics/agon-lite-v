if(CONFIG_MOS_VDP_INTERNAL_ENABLED)
    idf_component_register(
        SRCS
            "mos_vdp_internal.cpp"
            "vdp_font.c"
            "mos_audio_synth.c"
            "mos_vdp_ttxt.c"
        INCLUDE_DIRS "include" "."
        REQUIRES
            esp_lcd
            log
            freertos
            driver
            mos_api
        PRIV_REQUIRES
            esp_timer
            heap
            esp_mm
            mos_audio
    )
    # Link libm for sqrtf() used in PLOT circle/arc engine and sine LUT
    target_link_libraries(${COMPONENT_LIB} PRIVATE m)
else()
    # Stub registration: no sources compiled, but include dirs and deps declared
    # so that IDF's header scanner is satisfied (it doesn't honour #ifdef guards).
    idf_component_register(
        INCLUDE_DIRS "include"
        REQUIRES esp_lcd mos_api mos_audio log freertos
    )
endif()
