#!/usr/bin/env python3
"""
wl_wrap.py — Inject ESP-IDF Wear-Levelling state/config blocks into a raw FAT image.

The ESP-IDF wear_levelling driver stores three metadata sectors at the END of the
partition (just before the data area shrinks by these sectors):

    [ FAT data ... ] [ state1 ] [ state2 ] [ cfg ]

fatfsgen.py generates a plain FAT image and optionally adds a WL *wrapper* for the
data area, but does NOT write valid state/config blocks that the C WL driver expects.
As a result, the C driver finds invalid CRCs and calls initSections(), reformatting
the partition and wiping the files.

This script reads a raw FAT image (from fatfsgen.py WITHOUT --wl_mode), calculates
the correct WL layout, writes valid state1/state2/cfg blocks at the right offsets,
and produces a new image that the C driver accepts without reformatting.

Parameters match ESP-IDF defaults (wear_levelling.cpp):
  WL_DEFAULT_UPDATERATE      = 16
  WL_DEFAULT_TEMP_BUFF_SIZE  = 32
  WL_DEFAULT_WRITE_SIZE      = 16   (wl_pos_update_record_size)
  WL_DEFAULT_START_ADDR      = 0
  WL_CURRENT_VERSION         = 2
  WL_CFG_CRC_CONST           = 0xFFFFFFFF
  flash_sector_size           = 512  (CONFIG_WL_SECTOR_SIZE_512)
  wl_page_size               = flash_sector_size (= erase_size for SPI flash)

Usage:
  python3 wl_wrap.py <input.bin> <output.bin> <partition_start_addr> <partition_size>

Example:
  python3 wl_wrap.py fat_raw.bin storage.bin 0x210000 $((4*1024*1024))
"""

import sys
import struct
import binascii
import os
import random

# ── WL constants (must match ESP-IDF) ────────────────────────────────────────
WL_CURRENT_VERSION         = 2
WL_DEFAULT_UPDATERATE      = 16
WL_DEFAULT_TEMP_BUFF_SIZE  = 32
WL_DEFAULT_WRITE_SIZE      = 16   # wl_pos_update_record_size
WL_DEFAULT_START_ADDR      = 0
WL_CFG_CRC_CONST           = 0xFFFFFFFF

# ── Struct layouts (little-endian, same as C structs with ALIGNED_(16/32)) ───
#
# wl_config_t  (9 uint32 + 1 size_t each side — on 32-bit Xtensa, size_t = uint32):
#   [0]  wl_partition_start_addr   uint32
#   [1]  wl_partition_size         uint32
#   [2]  wl_page_size              uint32
#   [3]  flash_sector_size         uint32
#   [4]  wl_update_rate            uint32
#   [5]  wl_pos_update_record_size uint32
#   [6]  version                   uint32
#   [7]  wl_temp_buff_size         uint32   (size_t = 4 bytes on ESP32)
#   [8]  crc32                     uint32
#   Total = 36 bytes → padded to 48 bytes (multiple of 16)
#
# wl_state_t  (ALIGNED_(32)):
#   [0]  wl_dummy_sec_pos              uint32
#   [1]  wl_part_max_sec_pos           uint32
#   [2]  wl_dummy_sec_move_count       uint32
#   [3]  wl_sec_erase_cycle_count      uint32
#   [4]  wl_max_sec_erase_cycle_count  uint32
#   [5]  wl_block_size                 uint32
#   [6]  version                       uint32
#   [7]  wl_device_id                  uint32
#   [8..14] reserved[7]                uint32 × 7
#   [15] crc32                         uint32
#   Total = 16 × 4 = 64 bytes  (multiple of 16 ✓)

CFG_STRUCT_FMT   = '<9I'        # 9 × uint32 = 36 bytes
CFG_STRUCT_SIZE  = struct.calcsize(CFG_STRUCT_FMT)   # 36
CFG_PADDED_SIZE  = ((CFG_STRUCT_SIZE + 15) // 16) * 16  # 48 (multiple of 16)

STATE_STRUCT_FMT  = '<16I'      # 16 × uint32 = 64 bytes
STATE_STRUCT_SIZE = struct.calcsize(STATE_STRUCT_FMT)  # 64

WL_STATE_CRC_LEN_V2_OFFSET = 15 * 4   # offsetof(wl_state_t, crc32) = 60


def crc32_le(init_crc, data):
    """Replicate esp_rom_crc32_le / crc32::crc32_le (standard Ethernet CRC32)."""
    crc = binascii.crc32(data, init_crc ^ 0xFFFFFFFF) ^ 0xFFFFFFFF
    return crc & 0xFFFFFFFF


def make_cfg(partition_start, partition_size, flash_sector_size):
    page_size = flash_sector_size   # wl_page_size = erase_size = sector_size for perf mode

    fields = (
        partition_start,             # wl_partition_start_addr
        partition_size,              # wl_partition_size
        page_size,                   # wl_page_size
        flash_sector_size,           # flash_sector_size
        WL_DEFAULT_UPDATERATE,       # wl_update_rate
        WL_DEFAULT_WRITE_SIZE,       # wl_pos_update_record_size
        WL_CURRENT_VERSION,          # version
        WL_DEFAULT_TEMP_BUFF_SIZE,   # wl_temp_buff_size
        0,                           # crc32 placeholder
    )
    raw = struct.pack(CFG_STRUCT_FMT, *fields)
    crc = crc32_le(WL_CFG_CRC_CONST, raw[: CFG_STRUCT_SIZE - 4])  # all fields except crc32
    raw = raw[:-4] + struct.pack('<I', crc)
    # Pad to multiple of 16
    raw = raw + b'\xff' * (CFG_PADDED_SIZE - len(raw))
    return raw


def make_state(flash_size, state_size, cfg):
    page_size         = cfg[2]   # wl_page_size
    wl_update_rate    = cfg[4]
    wl_pos_record_sz  = cfg[5]
    version           = cfg[6]

    wl_part_max_sec_pos           = 1 + flash_size // page_size
    wl_max_sec_erase_cycle_count  = wl_update_rate   # since wl_update_rate != 0

    device_id = random.randint(0, 0xFFFFFFFF)

    fields = [
        0,                              # wl_dummy_sec_pos
        wl_part_max_sec_pos,            # wl_part_max_sec_pos
        0,                              # wl_dummy_sec_move_count
        0,                              # wl_sec_erase_cycle_count
        wl_max_sec_erase_cycle_count,   # wl_max_sec_erase_cycle_count
        page_size,                      # wl_block_size
        version,                        # version
        device_id,                      # wl_device_id
        0, 0, 0, 0, 0, 0, 0,           # reserved[7]
        0,                              # crc32 placeholder
    ]
    raw_for_crc = struct.pack(STATE_STRUCT_FMT, *fields)
    crc = crc32_le(WL_CFG_CRC_CONST, raw_for_crc[:WL_STATE_CRC_LEN_V2_OFFSET])
    fields[15] = crc
    raw = struct.pack(STATE_STRUCT_FMT, *fields)
    return raw


def main():
    if len(sys.argv) != 5:
        print(f"Usage: {sys.argv[0]} <input.bin> <output.bin> <partition_start_hex> <partition_size>")
        sys.exit(1)

    input_file  = sys.argv[1]
    output_file = sys.argv[2]
    part_start  = int(sys.argv[3], 0)
    part_size   = int(sys.argv[4], 0)

    flash_sector_size = 512   # CONFIG_WL_SECTOR_SIZE_512

    with open(input_file, 'rb') as f:
        data = bytearray(f.read())

    if len(data) != part_size:
        print(f"ERROR: input size {len(data)} != partition_size {part_size}")
        sys.exit(1)

    # ── Layout calculation (mirrors WL_Flash::config()) ──────────────────────
    # cfg_size = ceil(sizeof(wl_config_t) / sector) * sector
    cfg_size = ((CFG_PADDED_SIZE + flash_sector_size - 1) // flash_sector_size) * flash_sector_size

    # state_size = sector_size, unless (sizeof(wl_state_t) + N*pos_record_sz) > sector_size
    #   N = partition_size / sector_size
    n_sectors   = part_size // flash_sector_size
    state_needed = STATE_STRUCT_SIZE + n_sectors * WL_DEFAULT_WRITE_SIZE
    state_size = flash_sector_size
    if state_needed > state_size:
        state_size = ((state_needed + flash_sector_size - 1) // flash_sector_size) * flash_sector_size

    # Addresses are relative to partition start
    addr_cfg    = part_start + part_size - cfg_size
    addr_state2 = part_start + part_size - state_size * 1 - cfg_size
    addr_state1 = part_start + part_size - state_size * 2 - cfg_size

    # flash_size = usable data area (excluding WL overhead + dummy sector)
    flash_size = ((part_size - state_size * 2 - cfg_size) // flash_sector_size - 1) * flash_sector_size

    print(f"WL layout:")
    print(f"  partition:   start=0x{part_start:08x}  size=0x{part_size:08x}")
    print(f"  flash_size:  0x{flash_size:08x}  ({flash_size/1024:.1f} KB usable)")
    print(f"  state_size:  0x{state_size:08x}  (each, ×2)")
    print(f"  cfg_size:    0x{cfg_size:08x}")
    print(f"  addr_state1: 0x{addr_state1:08x}  (rel: 0x{addr_state1-part_start:08x})")
    print(f"  addr_state2: 0x{addr_state2:08x}  (rel: 0x{addr_state2-part_start:08x})")
    print(f"  addr_cfg:    0x{addr_cfg:08x}  (rel: 0x{addr_cfg-part_start:08x})")

    # ── Build cfg struct ──────────────────────────────────────────────────────
    cfg_raw = make_cfg(part_start, part_size, flash_sector_size)
    cfg_fields = struct.unpack(CFG_STRUCT_FMT, cfg_raw[:CFG_STRUCT_SIZE])

    # ── Build state structs ───────────────────────────────────────────────────
    state_raw = make_state(flash_size, state_size, cfg_fields)

    # ── Verify that FAT data doesn't overwrite our WL blocks ─────────────────
    rel_state1 = addr_state1 - part_start
    fat_last_nonff = -1
    for i in range(rel_state1 - 1, -1, -1):
        if data[i] != 0xFF:
            fat_last_nonff = i
            break
    if fat_last_nonff >= rel_state1:
        print(f"WARNING: FAT data extends to 0x{fat_last_nonff:08x}, overlapping WL area at 0x{rel_state1:08x}")
    else:
        print(f"  FAT data ends at 0x{fat_last_nonff:08x}, WL area starts at 0x{rel_state1:08x} ✓")

    # ── Write WL blocks into image ────────────────────────────────────────────
    def write_block(offset_in_part, src, size):
        data[offset_in_part : offset_in_part + size] = b'\xff' * size
        data[offset_in_part : offset_in_part + len(src)] = src

    write_block(rel_state1,              state_raw, state_size)
    write_block(addr_state2 - part_start, state_raw, state_size)
    write_block(addr_cfg    - part_start, cfg_raw,   cfg_size)

    with open(output_file, 'wb') as f:
        f.write(data)

    print(f"  Written: {output_file} ({len(data)} bytes)")
    print(f"  cfg crc32:   0x{cfg_fields[8]:08x}")
    state_fields = struct.unpack(STATE_STRUCT_FMT, state_raw)
    print(f"  state crc32: 0x{state_fields[15]:08x}")
    print("Done.")


if __name__ == '__main__':
    main()
