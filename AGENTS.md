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

## ESP32P4 Startup Stack — CRITICAL (commit cb0d77a)

`CONFIG_ESP_MAIN_TASK_STACK_SIZE` must be **8192**, NOT 65536.

**Why**: On ESP32-P4 rev<3 (`CONFIG_ESP32P4_SELECTS_REV_LESS_V3=y`) the internal
heap region available to `pvPortMalloc` before FreeRTOS starts is only ~111KB:

```
heap LOW: _heap_start_low (0x4ff1f360) → APP_USABLE_DIRAM_END (0x4ff3afc0)
APP_USABLE_DIRAM_END = SOC_ROM_STACK_START(0x4ff3cfc0) - SOC_ROM_STACK_SIZE(0x2000)
```

With ~15KB consumed by IDF init_fn handlers (idle tasks, timer task, etc.) before
`esp_startup_start_app()` runs, the TLSF allocator cannot find a contiguous 65KB
block → `pvPortMalloc(65536)` returns NULL → `xTaskCreatePinnedToCore` returns
`pdFAIL` → `assert(res == pdTRUE)` fires at `app_startup.c:86`.

`app_main()` only does NVS init, flash mount, and spawns `mos_main_task` with a
192KB PSRAM stack. It needs ≤8KB for itself. With 8KB, `pvPortMalloc(8200)` has
ample room in the 96KB remaining after init allocations.

**Never raise this back to 65536** — it will crash on every cold boot.

## Current State (session end 2026-03-04, session 5)

### Fully working ✅ (Waveshare ESP32-P4-WIFI6)
- VDP handshake, color banner, mode 16
- FAT mount with write (`wl_wrap.py`)
- PSRAM enabled
- Shell: external commands without `RUN` or `.bin` extension
- Loader: fixed arena, two-step cache sync, IBUS jump, Xtensa magic byte check
- SDK: `mos_vector.S` trampoline, correct IBUS/DBUS split linker script, little-endian
- **`helloworld` executes, prints to VDP, writes file — full API chain confirmed**
- **`bbcbasic` heap zero fix applied** — no more StoreProhibited crash

### Fully working ✅ (Olimex ESP32-P4-PC — boots on hardware)
- `build_olimex_p4pc.sh` builds cleanly (0 errors)
- **Boots correctly** after startup crash fix (commit cb0d77a)
- `app_main` launches `mos_main_task` with 192KB PSRAM stack
- `mos_net` abstraction: Ethernet backend with IP101GRR PHY via RMII
- `mos_hal` console: USB-JTAG/CDC (USB-C on the Olimex board)
- Correct GPIO pins from schematic Rev B
- **`esp_lcd_lt8912b` driver**: MIPI DSI → HDMI bridge (esp32-mos-4p4, closed)
  - `components/esp_lcd_lt8912b/` — full register init sequence (upstream Linux kernel)
  - Fixed 640×480@60Hz HDMI output (all 18 Agon video modes scaled to this)
  - EoTP patch in separate compilation unit (avoids TAG macro conflict with IDF)
  - HPD detect via GPIO15 + I2C register 0x48:0xC1 bit7
- **`mos_audio` driver**: ES8311 codec I2S+I2C 16384 Hz stereo (esp32-mos-sj7, closed ✅ confirmed working on hardware)
  - `components/mos_audio/` — `es8311_local.c/h` direct driver (new I2C master API), I2S0 APLL
  - GPIO6 = CODEC_PWR_DIS# (drive LOW to enable), GPIO53 = PA enable (active HIGH)
  - White noise fix: `es8311_adc_power_down()` after init (REG0E=0x32, REG15=0, REG17=0)
  - I2S configured STEREO; `mos_audio_write()` expands mono samples to L+R internally
  - `esp_codec_dev` managed component removed entirely — not needed
- **`mos_kbd` driver**: USB HID keyboard via FE1.1s hub (esp32-mos-7x7, closed)
  - `components/mos_kbd/` — full HID boot-protocol keyboard, PS/2 Set 2 scancodes
  - `espressif/usb_host_hid ^1.1.0` managed component (auto-downloaded)
  - USB PHY1 (OTG11 FS) `peripheral_map=BIT1`, GPIO26/27, HUB_RST# GPIO21
  - HID device callback runs inline in HID background task (core 0) — no queue
  - Scancode stub in `main.c` (logs at DEBUG); will be replaced by internal VDP
  - `CONFIG_USB_HOST_HUBS_SUPPORTED=y`, `CONFIG_USB_HOST_HUB_MULTI_LEVEL=y`
- **`mos_vdp_internal`**: UDG, GCOL modes, PLOT engine, VDU 23,0 protocol (esp32-mos-29j)
  - UDG glifos VDU 23,32-255 en PSRAM
  - GCOL paint modes (AND/OR/XOR/Invert) + CLG
  - PLOT completo: líneas, rectángulos, círculos, triángulos, flood fill
  - VDU 23,0 sub-protocol: handshake (0x80), cursor pos (0x82), mode info (0x86)
  - VDU 23,0,0x85 audio protocol + sintetizador 3 canales (square/sine/triangle/sawtooth/noise)
  - synth_mixer_task stack 8192 bytes (sinf() soft-float requiere >4096)
  - **PLOT coordinate system corregido** (commit bb608b3, sesión 6):
    - Escala 1280×1024 OS units (no >>2): X/2.0, Y/2.133 para MODE 1 640×480
    - Relative/absolute invertido: bit2=1 → absoluto (BBC Micro spec)
    - Selección color: cmd&0x03 (no cmd&0x40); PLOT 69 colourMode=1 → fg
    - Move-only (colourMode=0) no dibuja (commit d974969)
  - **Flush desacoplado del PLOT** (commits 2710bcf→735d281, sesión 6):
    - Cada PLOT solo marca s_fb_dirty=true; esp_timer a 60Hz también lo marca
    - render task drena la cola VDU completa en inner loop, luego llama
      draw_bitmap desde contexto de task (seguro para DW-GDMA)
    - cache_msync directo desde timer/ISR corrompía LLIs del DW-GDMA
      (Load access fault en dw_gdma_channel_default_isr)

### mos_kbd CMakeLists.txt note (learned this session)
- `espressif__usb_host_hid` must be in `PRIV_REQUIRES` (not `REQUIRES`) because the
  headers are only needed internally. IDF 5.x enforces this at configure time.

### Design decision recorded: Agon video modes and HDMI output
- Agon has 18 standard modes (plus extended/double-buffered variants)
- Pixel clocks range from 12.5 MHz (320×200) to 65 MHz (1024×768)
- **HDMI output is FIXED at 1024×768@60Hz** for all Agon modes (session 8)
- Modes with lower resolution rendered at native size, centered in 1024×768 FB
- **DO NOT use XIP_FROM_PSRAM** — tested, causes blue screen / worse flicker

### SD-only storage — CRITICAL fix (commit d0f0073, session 5)

The `flash_io_task` proxy (commits 1aebb3f–21259db) was reverted. Root cause:
`esp_task_stack_is_sane_cache_disabled()` is a hard assert in IDF that fires when
a PSRAM-stack task touches the SPI flash FAT driver. No Kconfig disables it on P4.

**Fix**: Remove the internal flash FAT partition entirely. SD card (SDMMC) has no
such restriction — it uses its own bus with no cache coherency dance.

- `partitions.csv`: `storage` (4M FAT) removed, `factory` app grown to 6M
- `MOS_FLASH_MOUNT` is now an alias for `MOS_SD_MOUNT` (`/sdcard`) for source compat
- `A:` and `B:` both resolve to `/sdcard` (only one physical drive exists)
- `app_main()` now only does NVS init; SD is mounted in `mos_main_task` (PSRAM stack)
- `wear_levelling` / `esp_partition` removed from `mos_fs` CMakeLists

**Consequence for Waveshare board**: The Waveshare build also no longer has a flash
FAT partition. All files must be on SD card. `flash_data.sh` no longer needs to
flash the storage partition (only the firmware binary).

### LT8912B HDMI driver — CRITICAL learnings (session 8, commit e663df9)

**Root cause of "check video cable"**: reg `0x33` is dual-use:
- Written `0x0C` as Tx Analog drive strength during init sequence
- Must be written `0x0E` at the END of init to enable HDMI output
- Without `0x0E`, TMDS output stays disabled permanently

**Root cause of periodic sync loss / blue screen**: `hs=8` (133 ns) too narrow
for LT8912B PLL to maintain lock. Must use VESA-proportioned hsync width.
Final working timings (pclk=60 MHz, PLL_F240M/4 exact):
- `htotal=1241 vtotal=806` → fps=59.985 Hz
- `hfp=14 hs=83 hbp=120  vfp=3 vs=6 vbp=29`
- DDS: `0x93/0x3E/0x29` (Espressif official esp_lcd_lt8912b component)
- Settle: `0x08`
- LVDS PLL reset pulse (`0x02=0xF7/0xFF`) must be FIRST before clock enable
- DDS reset (`0x05=0xFB/0xFF`) must follow MIPI RX reset (`0x03`)

**DO NOT use XIP_FROM_PSRAM** — causes blue screen flicker, worse than without.

### Next areas to work on
- **`mos_vdp_internal`** — PLOT engine verificado y funcionando (hatgraph, mandelbrot, cube3d)
  - Siguiente: buffered commands VDU 23,0,0xA0 (issue esp32-mos-29j.8)
- **`mos_vdp` dual router** — TCP external / internal VDP (esp32-mos-edq)
- I2C driver (issue esp32-mos-h4x)
- More shell commands (TYPE, DEL, RENAME, COPY, CD, MKDIR)
- USB SPLIT transactions para FS/LS devices via HS hub (esp32-mos-8pe)

### Audio ES8311 — FIX CONFIRMED (commit b482adc, session 7)
**Root cause**: `esp_codec_dev` calls `es8311_start()` unconditionally which writes
ADC_REG17=0xBF and ADC_REG15=0x40, activating the ADC with floating AINL/AINR → white noise.

**Fix**: Replaced `esp_codec_dev` with `es8311_local.c/h` (direct ES8311 driver using new
I2C master API). After `es8311_init()`, call `es8311_adc_power_down()`:
- REG0E=0x32 → power down ADC PGA + modulator (DAC bias kept)
- REG15=0x00, REG17=0x00 → ADC path fully silenced

I2S changed to STEREO (required by ES8311 LRCK framing); `mos_audio_write()` handles
mono→stereo expansion. `esp_codec_dev` removed from `idf_component.yml`.

