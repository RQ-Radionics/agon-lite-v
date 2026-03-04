# USB component override for esp32-mos (Olimex ESP32-P4-PC)
#
# This component overrides two files from the IDF USB stack:
#
#   hub.c  — Removes the overly-conservative TT speed check (IDF-10023).
#             The FE1.1s hub has integrated TT hardware that handles LS/FS
#             devices transparently (USB 2.0 spec §11.23).
#
#   enum.c — On CHECK_SHORT_DEV_DESC failure, fall through to SECOND_RESET
#             instead of cancelling. LS/FS devices behind a HS hub with TT
#             may need the second reset to respond correctly.
#
# All other source files are taken directly from $IDF_PATH/components/usb/.

idf_build_get_property(target IDF_TARGET)

if(${target} STREQUAL "linux")
    return()
endif()

set(IDF_USB $ENV{IDF_PATH}/components/usb)

set(srcs)
set(priv_requires esp_driver_gpio esp_mm)

if(${target} STREQUAL "esp32p4")
    list(APPEND priv_requires esp_psram)
endif()

if(CONFIG_SOC_USB_OTG_SUPPORTED)
    list(APPEND srcs
        "${CMAKE_CURRENT_SOURCE_DIR}/hcd_dwc.c"       # patched: SPLIT transactions (HCSPLT)
        "${CMAKE_CURRENT_SOURCE_DIR}/enum.c"          # patched: CHECK_SHORT_DEV_DESC fallback
        "${CMAKE_CURRENT_SOURCE_DIR}/hub.c"           # patched: TT speed check removed
        "${IDF_USB}/usb_helpers.c"
        "${IDF_USB}/usb_host.c"
        "${IDF_USB}/usb_private.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/usbh.c"          # patched: passes hub_addr/hub_port to HCD
        "${IDF_USB}/usb_phy.c"
    )
endif()

if(CONFIG_USB_HOST_HUBS_SUPPORTED)
    list(APPEND srcs
        "${IDF_USB}/ext_hub.c"
        "${IDF_USB}/ext_port.c"
    )
endif()

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "${IDF_USB}/include"
                       PRIV_INCLUDE_DIRS
                           "${CMAKE_CURRENT_SOURCE_DIR}"   # for hcd_split_ext.h
                           "${IDF_USB}/private_include"
                       PRIV_REQUIRES ${priv_requires}
                       KCONFIG "${IDF_USB}/Kconfig"
                       )

if(CONFIG_COMPILER_STATIC_ANALYZER AND CMAKE_C_COMPILER_ID STREQUAL "GNU")
    set_property(SOURCE ${IDF_USB}/usb_host.c PROPERTY COMPILE_FLAGS -fno-analyzer)
endif()
