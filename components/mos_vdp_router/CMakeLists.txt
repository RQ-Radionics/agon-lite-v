idf_component_register(
    SRCS "mos_vdp_router.c"
    INCLUDE_DIRS "include"
    REQUIRES
        mos_vdp
        mos_vdp_internal
    PRIV_INCLUDE_DIRS
        "../mos_api/include"
)
