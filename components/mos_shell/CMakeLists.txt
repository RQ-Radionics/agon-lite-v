idf_component_register(
    SRCS "mos_shell.c"
    INCLUDE_DIRS "include"
    REQUIRES mos_hal mos_fs mos_strings mos_file mos_sysvars mos_editor fatfs heap mos_loader
)
