idf_component_register(
    SRCS
        "mos_vdp_internal.cpp"
        "vdp_font.c"
        "mos_audio_synth.c"
    INCLUDE_DIRS "include" "."
    REQUIRES
        esp_lcd
        log
        freertos
        driver
        mos_audio
    PRIV_REQUIRES
        esp_timer
        heap
        esp_mm
)

# Link libm for sqrtf() used in PLOT circle/arc engine and sine LUT
target_link_libraries(${COMPONENT_LIB} PRIVATE m)
