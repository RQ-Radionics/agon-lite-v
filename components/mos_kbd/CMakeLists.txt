idf_component_register(
    SRCS "mos_kbd.c"
    INCLUDE_DIRS "include"
    REQUIRES
        driver
        log
        freertos
        usb
    PRIV_REQUIRES
        espressif__usb_host_hid
)
