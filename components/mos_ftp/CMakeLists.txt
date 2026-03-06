idf_component_register(
    SRCS "mos_ftp.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES mos_net mos_fs freertos
)
