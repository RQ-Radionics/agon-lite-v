idf_component_register(
    SRCS "mos_hal.c"
    INCLUDE_DIRS "include"
    REQUIRES mos_vdp
    PRIV_REQUIRES driver esp_driver_uart esp_driver_usb_serial_jtag vfs
)
