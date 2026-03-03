if(CONFIG_MOS_VDP_INTERNAL_ENABLED)
    set(ROUTER_EXTRA_REQUIRES mos_vdp_internal)
else()
    set(ROUTER_EXTRA_REQUIRES "")
endif()

idf_component_register(
    SRCS "mos_vdp_router.c"
    INCLUDE_DIRS "include"
    REQUIRES
        mos_vdp
        ${ROUTER_EXTRA_REQUIRES}
    PRIV_INCLUDE_DIRS
        "../mos_api/include"
)
