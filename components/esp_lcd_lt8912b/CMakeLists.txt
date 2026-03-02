idf_component_register(
    SRCS "esp_lcd_lt8912b.c" "esp_lcd_lt8912b_eotp.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_lcd driver esp_hw_support esp_timer log hal
)

# Include the private DSI header directory so we can access mipi_dsi_priv.h
# (needed for the EoTP patch that disables it after esp_lcd_new_dsi_bus).
# This is an IDF-internal path; if IDF restructures, the build will fail
# at the #include, making the breakage obvious and easy to fix.
target_include_directories(${COMPONENT_LIB} PRIVATE
    $ENV{IDF_PATH}/components/esp_lcd/dsi
)
