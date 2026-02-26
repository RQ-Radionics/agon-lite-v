#!/usr/bin/env python3
"""
wl_wrap.py — Inject ESP-IDF Wear-Levelling state/config blocks into a raw FAT image.

ARCHITECTURE (WL_Ext_Perf, CONFIG_WL_SECTOR_SIZE=512):
  Two sizes are in play:
    flash_erase_size = 4096  — physical SPI flash erase granularity (partition->erase_size)
    fat_sector_size  =  512  — logical FAT sector (CONFIG_WL_SECTOR_SIZE)

  WL_Flash::config() receives:
    wl_page_size      = flash_erase_size = 4096   ← controls layout/calcAddr
    flash_sector_size = flash_erase_size = 4096   ← same
    fat_sector_size   = fat_sector_size  =  512   ← only used by WL_Ext_Perf for r/w

  Layout at the END of the partition:
    [ FAT data ... ][ state1 (state_size) ][ state2 (state_size) ][ cfg (cfg_size) ]

  Sizes:
    cfg_size   = ceil(sizeof(wl_config_t)=48 / flash_erase_size) * flash_erase_size  = 4096
    state_size = max(flash_erase_size,
                     ceil((sizeof(wl_state_t) + N*pos_record_sz) / flash_erase_size)
                     * flash_erase_size)
               where N = partition_size / flash_erase_size

  Address mapping (calcAddr, wl_page_size=4096):
    physical = (flash_size - move_count*4096 + logical) % flash_size
    if physical >= dummy_sec_pos*4096: physical += 4096
    ⟹ with pos=1, move=0: logical 0 → physical 0  (FAT BPB is found at offset 0)

Usage:
  python3 wl_wrap.py <input.bin> <output.bin> <partition_start_hex> <partition_size>

Example:
  python3 wl_wrap.py fat_raw.bin storage.bin 0x210000 $((4*1024*1024))
"""

import sys
import struct
import binascii
import random

# ── WL constants ─────────────────────────────────────────────────────────────
WL_CURRENT_VERSION         = 2
WL_DEFAULT_UPDATERATE      = 16
WL_DEFAULT_TEMP_BUFF_SIZE  = 32
WL_DEFAULT_WRITE_SIZE      = 16   # wl_pos_update_record_size
WL_DEFAULT_START_ADDR      = 0
WL_CFG_CRC_CONST           = 0xFFFFFFFF

FLASH_ERASE_SIZE = 4096   # partition->erase_size on ESP32-S3 SPI flash
FAT_SECTOR_SIZE  = 512    # CONFIG_WL_SECTOR_SIZE

# ── Struct formats ────────────────────────────────────────────────────────────
# wl_config_t: 9 × uint32 = 36 bytes, padded to 48 (multiple of 16)
CFG_STRUCT_FMT   = '<9I'
CFG_STRUCT_SIZE  = struct.calcsize(CFG_STRUCT_FMT)   # 36
CFG_PADDED_SIZE  = ((CFG_STRUCT_SIZE + 15) // 16) * 16  # 48

# wl_state_t: 16 × uint32 = 64 bytes (multiple of 16 ✓)
STATE_STRUCT_FMT  = '<16I'
STATE_STRUCT_SIZE = struct.calcsize(STATE_STRUCT_FMT)  # 64

# WL_STATE_CRC_LEN_V2 = offsetof(wl_state_t, crc32) = 15 × 4 = 60
WL_STATE_CRC_LEN_V2 = 60


def crc32_le(init_crc, data):
    """esp_rom_crc32_le — standard Ethernet CRC32."""
    return binascii.crc32(data, init_crc ^ 0xFFFFFFFF) ^ 0xFFFFFFFF & 0xFFFFFFFF


def compute_layout(part_start, part_size):
    """Return (cfg_size, state_size, rel_state1, rel_state2, rel_cfg, flash_size)."""
    page_size = FLASH_ERASE_SIZE  # wl_page_size in WL_Flash

    # cfg_size: ceil(sizeof padded cfg / erase_size) * erase_size
    cfg_size = ((CFG_PADDED_SIZE + FLASH_ERASE_SIZE - 1) // FLASH_ERASE_SIZE) * FLASH_ERASE_SIZE

    # state_size: must hold wl_state_t + one pos-update record per flash sector
    n_flash_sectors = part_size // FLASH_ERASE_SIZE
    state_needed    = STATE_STRUCT_SIZE + n_flash_sectors * WL_DEFAULT_WRITE_SIZE
    state_size      = FLASH_ERASE_SIZE
    if state_needed > state_size:
        state_size = ((state_needed + FLASH_ERASE_SIZE - 1) // FLASH_ERASE_SIZE) * FLASH_ERASE_SIZE

    # Absolute addresses (as computed in WL_Flash::config)
    addr_cfg    = part_start + part_size - cfg_size
    addr_state2 = part_start + part_size - state_size * 1 - cfg_size
    addr_state1 = part_start + part_size - state_size * 2 - cfg_size

    # Usable flash_size (excludes WL overhead + 1 dummy page)
    flash_size = ((part_size - state_size * 2 - cfg_size) // page_size - 1) * page_size

    rel_cfg    = addr_cfg    - part_start
    rel_state2 = addr_state2 - part_start
    rel_state1 = addr_state1 - part_start

    return cfg_size, state_size, rel_state1, rel_state2, rel_cfg, flash_size


def make_cfg(part_start, part_size, flash_size):
    page_size = FLASH_ERASE_SIZE
    fields = (
        0,                           # wl_partition_start_addr — always 0 (WL_DEFAULT_START_ADDR)
                                     # The driver uses esp_partition_read() with offsets relative
                                     # to the partition start, so this field must be 0, NOT the
                                     # absolute flash address of the partition.
        part_size,                   # wl_partition_size
        page_size,                   # wl_page_size  ← MUST be flash erase size (4096)
        FLASH_ERASE_SIZE,            # flash_sector_size
        WL_DEFAULT_UPDATERATE,       # wl_update_rate
        WL_DEFAULT_WRITE_SIZE,       # wl_pos_update_record_size
        WL_CURRENT_VERSION,          # version
        WL_DEFAULT_TEMP_BUFF_SIZE,   # wl_temp_buff_size
        0,                           # crc32 placeholder
    )
    raw = struct.pack(CFG_STRUCT_FMT, *fields)
    crc = crc32_le(WL_CFG_CRC_CONST, raw[:CFG_STRUCT_SIZE - 4]) & 0xFFFFFFFF
    raw = raw[:-4] + struct.pack('<I', crc)
    raw = raw + b'\xff' * (CFG_PADDED_SIZE - len(raw))
    return raw


def make_state(flash_size):
    page_size = FLASH_ERASE_SIZE
    # wl_part_max_sec_pos = 1 + flash_size / page_size
    wl_part_max_sec_pos = 1 + flash_size // page_size

    # wl_dummy_sec_pos = 0.
    # recoverPos() scans the pos_update records (all 0xFF in a fresh image) and
    # always lands at position=0 because OkBuffSet() returns false on 0xFF bytes.
    # So pos is always 0 at runtime regardless of what we store — don't fight it.
    # Instead the FAT image is shifted by one page (4096 bytes) in wl_wrap main(),
    # so the BPB sits at physical offset 4096 = calcAddr(logical 0) with pos=0.
    wl_dummy_sec_pos = 0

    device_id = random.randint(0, 0xFFFFFFFF)

    fields = [
        wl_dummy_sec_pos,            # wl_dummy_sec_pos
        wl_part_max_sec_pos,         # wl_part_max_sec_pos
        0,                           # wl_dummy_sec_move_count
        0,                           # wl_sec_erase_cycle_count
        WL_DEFAULT_UPDATERATE,       # wl_max_sec_erase_cycle_count
        page_size,                   # wl_block_size
        WL_CURRENT_VERSION,          # version
        device_id,                   # wl_device_id
        0, 0, 0, 0, 0, 0, 0,        # reserved[7]
        0,                           # crc32 placeholder
    ]
    raw = struct.pack(STATE_STRUCT_FMT, *fields)
    crc = crc32_le(WL_CFG_CRC_CONST, raw[:WL_STATE_CRC_LEN_V2]) & 0xFFFFFFFF
    fields[15] = crc
    return struct.pack(STATE_STRUCT_FMT, *fields)


def main():
    if len(sys.argv) != 5:
        print(f"Usage: {sys.argv[0]} <input.bin> <output.bin> <partition_start_hex> <partition_size>")
        sys.exit(1)

    input_file  = sys.argv[1]
    output_file = sys.argv[2]
    part_start  = int(sys.argv[3], 0)
    part_size   = int(sys.argv[4], 0)

    with open(input_file, 'rb') as f:
        fat_raw = bytearray(f.read())

    if len(fat_raw) != part_size:
        print(f"ERROR: input size {len(fat_raw)} != partition_size {part_size}")
        sys.exit(1)

    cfg_size, state_size, rel_s1, rel_s2, rel_cfg, flash_size = compute_layout(part_start, part_size)

    # recoverPos() always sets wl_dummy_sec_pos=0 on a fresh image (all pos_update
    # records are 0xFF → OkBuffSet returns false at i=0 → position=0).
    # With pos=0: calcAddr(logical=0) = 0 + page = 4096 (physical).
    # So the FAT BPB must be at physical offset FLASH_ERASE_SIZE (4096), not 0.
    # We shift the entire FAT image forward by one page: physical[0..4095] = dummy (0xFF),
    # physical[4096..] = FAT data. The last page of FAT data is lost — but flash_size
    # already excludes one dummy page, so the usable FAT area fits exactly.
    DUMMY = FLASH_ERASE_SIZE
    data = bytearray(b'\xff' * part_size)
    data[DUMMY : DUMMY + len(fat_raw) - DUMMY] = fat_raw[:-DUMMY]
    print(f"  FAT shifted by 0x{DUMMY:x} bytes (dummy page at offset 0)")

    print(f"WL layout (flash_erase_size={FLASH_ERASE_SIZE}, fat_sector={FAT_SECTOR_SIZE}):")
    print(f"  partition:   start=0x{part_start:08x}  size=0x{part_size:08x}")
    print(f"  flash_size:  0x{flash_size:08x}  ({flash_size//1024} KB usable)")
    print(f"  state_size:  0x{state_size:08x}  (x2)")
    print(f"  cfg_size:    0x{cfg_size:08x}")
    print(f"  rel_state1:  0x{rel_s1:08x}")
    print(f"  rel_state2:  0x{rel_s2:08x}")
    print(f"  rel_cfg:     0x{rel_cfg:08x}")

    # Verify FAT data doesn't reach WL area
    fat_end = -1
    for i in range(rel_s1 - 1, -1, -1):
        if data[i] != 0xFF:
            fat_end = i
            break
    if fat_end >= rel_s1:
        print(f"WARNING: FAT data at 0x{fat_end:08x} overlaps WL area at 0x{rel_s1:08x}!")
    else:
        print(f"  FAT data ends at 0x{fat_end:08x}, WL area at 0x{rel_s1:08x} ✓")

    cfg_raw   = make_cfg(part_start, part_size, flash_size)
    state_raw = make_state(flash_size)

    # Verify calcAddr(0) points to FLASH_ERASE_SIZE (where BPB now lives)
    state = struct.unpack(STATE_STRUCT_FMT, state_raw)
    pos   = state[0]   # 0 — recoverPos always lands here
    page  = FLASH_ERASE_SIZE
    r     = (flash_size + 0) % flash_size
    if r >= pos * page:
        r += page
    want = FLASH_ERASE_SIZE
    print(f"  calcAddr(0): pos={pos} → physical 0x{r:x} ({'OK' if r == want else 'WRONG — want 0x%x' % want})")

    def write_block(rel_off, src, size):
        data[rel_off : rel_off + size] = b'\xff' * size
        data[rel_off : rel_off + len(src)] = src

    write_block(rel_s1,  state_raw, state_size)
    write_block(rel_s2,  state_raw, state_size)
    write_block(rel_cfg, cfg_raw,   cfg_size)

    with open(output_file, 'wb') as f:
        f.write(data)

    cfg_f   = struct.unpack(CFG_STRUCT_FMT, cfg_raw[:CFG_STRUCT_SIZE])
    print(f"  cfg crc32:   0x{cfg_f[8]:08x}")
    print(f"  state crc32: 0x{state[15]:08x}")
    print(f"  Written: {output_file} ({len(data)} bytes)")
    print("Done.")


if __name__ == '__main__':
    main()
