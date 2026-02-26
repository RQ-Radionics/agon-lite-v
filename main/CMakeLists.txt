idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        nvs_flash
        mos_hal
        mos_fs
        mos_strings
        mos_file
        mos_sysvars
        mos_editor
        mos_shell
        mos_wifi
        mos_sntp
        mos_vdp
)
