/*
 * SPDX-FileCopyrightText: 2026 esp32-mos project
 * SPDX-License-Identifier: Apache-2.0
 *
 * hcd_split_ext.h — extension to the HCD API for SPLIT transaction support.
 *
 * This header is internal to the esp32-mos USB component override and is NOT
 * part of the IDF public API.  It declares hcd_pipe_set_hub_info(), which must
 * be called after hcd_pipe_alloc() whenever a FS/LS device sits behind a HS hub
 * so that _buffer_exec() can programme the DWC HCSPLT register correctly.
 */

#pragma once

#include <stdint.h>
#include "hcd.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Set the upstream HS-hub address and port for a pipe.
 *
 * Must be called immediately after hcd_pipe_alloc() for any pipe belonging to
 * a FS or LS device that is connected through a HS hub.  The information is
 * used to programme the DWC HCSPLT register before every channel activation so
 * the hardware generates the correct SSPLIT / CSPLIT token sequence.
 *
 * @param pipe_hdl  Pipe handle returned by hcd_pipe_alloc()
 * @param hub_addr  USB device address of the upstream HS hub (1-127)
 * @param hub_port  Port number on that hub (1-based)
 */
void hcd_pipe_set_hub_info(hcd_pipe_handle_t pipe_hdl, uint8_t hub_addr, uint8_t hub_port);

#ifdef __cplusplus
}
#endif
