# Agent Instructions

This project uses **bd** (beads) for issue tracking. Run `bd onboard` to get started.

## Quick Reference

```bash
bd ready              # Find available work
bd show <id>          # View issue details
bd update <id> --status in_progress  # Claim work
bd close <id>         # Complete work
bd sync               # Sync with git
```

## Landing the Plane (Session Completion)

**When ending a work session**, you MUST complete ALL steps below. Work is NOT complete until `git push` succeeds.

**MANDATORY WORKFLOW:**

1. **File issues for remaining work** - Create issues for anything that needs follow-up
2. **Run quality gates** (if code changed) - Tests, linters, builds
3. **Update issue status** - Close finished work, update in-progress items
4. **PUSH TO REMOTE** - This is MANDATORY:
   ```bash
   git pull --rebase
   bd sync
   git push
   git status  # MUST show "up to date with origin"
   ```
5. **Clean up** - Clear stashes, prune remote branches
6. **Verify** - All changes committed AND pushed
7. **Hand off** - Provide context for next session

**CRITICAL RULES:**
- Work is NOT complete until `git push` succeeds
- NEVER stop before pushing - that leaves work stranded locally
- NEVER say "ready to push when you are" - YOU must push
- If push fails, resolve and retry until it succeeds

## SDK Workflow — CRITICAL

After building a user program, **always copy to `data/` before flashing storage**:
```bash
make -C sdk          # auto-copies .bin to data/ via Makefile rule
./flash_data.sh /dev/cu.usbmodem2401
```
The `make` rule now does this automatically. If copying manually: `cp sdk/foo.bin data/foo.bin`.

## Cache Sync — CRITICAL (commit cca199d)

When loading a binary into `s_exec_arena` (PSRAM DBUS, `0x3Cxxxxxx`):

`fread()` goes through the FAT driver (CPU/memcpy path) → data is in **dcache as dirty lines**, NOT yet in physical PSRAM.

**Correct two-step sync:**
```c
// Step A: flush dirty dcache → physical PSRAM, then invalidate dcache
esp_cache_msync(s_exec_arena, sync_size,
    C2M | INVALIDATE | TYPE_DATA | UNALIGNED);

// Step B: invalidate icache for IBUS window (0x42xxxxxx) — use IBUS alias addr
// M2C does NOT accept UNALIGNED flag
esp_cache_msync(dbus_to_ibus(s_exec_arena), sync_size,
    M2C | INVALIDATE | TYPE_INST);
```

Wrong approaches tried:
- Pure `M2C` for data: discards dirty dcache without writeback → binary lost
- `C2M` only (no icache): code fetch works but dcache refills stale on next run
- `M2C` with `UNALIGNED`: rejected by ESP-IDF (`ESP_ERR_INVALID_ARG`)

## Linker Script (mos_user.ld) — CRITICAL (commit cca199d)

The `> DBUS` region directive **resets VMA to the region origin** (`MOS_DBUS_BASE = 0x3C0E0000`) regardless of the current location counter. To place rodata at the correct DBUS alias of `_ibus_end`, use **explicit VMA form**:

```ld
_dbus_data_start = _ibus_end - BUS_OFFSET;   /* DBUS alias of where code ends */

.rodata _dbus_data_start : AT(_ibus_end) {   /* VMA explicit, LMA contiguous */
    *(.rodata*)
}
```

Do NOT use `> DBUS` for data sections — it overrides the explicit VMA.

## BBC BASIC Memory — CRITICAL

### Heap allocation (commit e02b038)
`mos->malloc` now calls `heap_caps_malloc(size, MALLOC_CAP_SPIRAM)` so user
programs draw from the 8 MB PSRAM, not internal DRAM (~300 KB).
`bbccon_esp32.h` sets `DEFAULT_RAM = 2 MB`, `MAXIMUM_RAM = 6 MB`.

### Heap Init — CRITICAL (commit 8e9164c)

`malloc()` on ESP32/FreeRTOS does **NOT** zero memory. BBC BASIC's `clear()` calls
`gettop()` on `progRAM` to find the end of the (empty) program. If `progRAM` has
garbage bytes, `gettop()` returns NULL → prints "Bad program" → calls `error(256)`
which longjmps back to `basic()`, **skipping** the `pfree = lomem + 4*fastvars`
assignment. `pfree` stays 0. Next variable access (`putvar`) dereferences `pfree`
(= 0) → StoreProhibited at 0x00000000.

**Fix in `bbcbasic/src/bbccon_esp32.c`:**
```c
/* Zero the heap so progRAM starts as a valid empty BBC BASIC program */
memset(userRAM, 0, (size_t) ram_size);
```
Must be done right after `malloc()`, before setting up workspace pointers.

## sdkconfig Isolation — CRITICAL (commit 9ef186b)

**Problem**: IDF auto-loads `sdkconfig.defaults.<target>` by name convention.
Naming the Waveshare config `sdkconfig.defaults.esp32p4` caused IDF to pull it
in for ALL esp32p4 builds (including Olimex), enabling the C6 WiFi coprocessor
(esp_hosted over SDIO slot 1) even though Olimex has no C6.

**Fix**: Rename to `sdkconfig.defaults.waveshare-p4wifi` (no longer matches
the auto-load convention). Both build scripts explicitly set `SDKCONFIG_DEFAULTS`
and wipe `sdkconfig` + `build/` before each build.

**Residual**: The `espressif__esp_hosted` managed component still compiles for
both boards (it's in `idf_component.yml` with `if: "target in [esp32p4]"`).
It defaults to enabled but selects `ESP_HOSTED_P4_DEV_BOARD_NONE` → GPIO reset
pin = -1 → no SDIO probing at boot on the Olimex. Functionally safe.

## Build Scripts

| Script | Board | Config files loaded |
|---|---|---|
| `build_p4.sh` | Waveshare ESP32-P4-WIFI6 | `sdkconfig.defaults` + `sdkconfig.defaults.waveshare-p4wifi` |
| `build_olimex_p4pc.sh` | Olimex ESP32-P4-PC | `sdkconfig.defaults` + `sdkconfig.defaults.olimex-p4pc` |
| `build_s3.sh` | Waveshare ESP32-S3 | `sdkconfig.defaults` |

Both P4 scripts wipe `sdkconfig` and `build/` before running `idf.py set-target`.

## Current State (session end 2026-03-02)

### Fully working ✅ (Waveshare ESP32-P4-WIFI6)
- VDP handshake, color banner, mode 16
- FAT mount with write (`wl_wrap.py`)
- PSRAM enabled
- Shell: external commands without `RUN` or `.bin` extension
- Loader: fixed arena, two-step cache sync, IBUS jump, Xtensa magic byte check
- SDK: `mos_vector.S` trampoline, correct IBUS/DBUS split linker script, little-endian
- **`helloworld` executes, prints to VDP, writes file — full API chain confirmed**
- **`bbcbasic` heap zero fix applied** — no more StoreProhibited crash

### Fully working ✅ (Olimex ESP32-P4-PC — compile-verified, not yet hardware-tested)
- `build_olimex_p4pc.sh` builds cleanly (0 errors)
- `app_main` launches `mos_main_task` with 96KB PSRAM stack (stack overflow fix)
- `mos_net` abstraction: Ethernet backend with IP101GRR PHY via RMII
- `mos_hal` console: USB-JTAG/CDC (USB-C on the Olimex board)
- Correct GPIO pins from schematic Rev B

### Next areas to work on
- **Hardware test the Olimex build** — flash and confirm boot, Ethernet DHCP,
  USB-JTAG console, shell prompt
- Confirm BBC BASIC works interactively on VDP (type PRINT PI, A=1, etc.)
- More shell commands (TYPE, DEL, RENAME, COPY, CD, MKDIR)
- XMODEM receive for uploading binaries over serial
- More user SDK examples
- I2C driver (issue esp32-mos-h4x)
- Sprites: investigate why sprites don't appear despite correct VDP sequence

