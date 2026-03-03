# ESP32-MOS

Port of the [Agon MOS](https://github.com/AgonConsole8/agon-mos) shell from the eZ80 CPU to **ESP32** with **ESP-IDF v5.5**.

**Scenario A**: The ESP32 replaces the eZ80 entirely as the main CPU.  
No VDP co-processor, no Z80 emulator, no RST syscalls. The MOS shell runs natively on Xtensa LX7.

---

## Supported Hardware

### Olimex ESP32-P4-PC (primary target)

| Component | Details |
|-----------|---------|
| SoC | ESP32-P4NRW32 (dual Xtensa LX9 @ 400 MHz, 32 MB PSRAM) |
| Display | HDMI 800×600@60Hz via LT8912B (MIPI DSI → HDMI bridge) |
| Audio | ES8311 codec — I2S1, I2C1 — 48 kHz stereo 16-bit |
| Keyboard | USB HID via FE1.1s hub (USB PHY1 / OTG11 FS) |
| Storage | microSD (SDMMC host) |
| Network | 100 Mbit Ethernet — IP101GRR PHY via RMII |
| Console | USB-JTAG/CDC (USB-C) |

Build:
```bash
bash build_olimex_p4pc.sh
```

Flash:
```bash
idf.py -p /dev/cu.usbmodem2401 flash monitor
```

### Waveshare ESP32-P4-WIFI6

| Component | Details |
|-----------|---------|
| SoC | ESP32-P4 (dual Xtensa LX9 @ 400 MHz, 8 MB PSRAM) |
| Network | Wi-Fi 6 / BLE5 via ESP32-C6 coprocessor (esp_hosted, SDIO) |
| Console | UART0 (USB-Serial adapter) |

Build:
```bash
bash build_p4.sh
```

### Waveshare ESP32-S3 (original target)

| Component | Details |
|-----------|---------|
| SoC | ESP32-S3 (dual Xtensa LX7 @ 240 MHz, up to 8 MB PSRAM) |
| Console | UART0 (TX=GPIO1, RX=GPIO3) — 115200 8N1 |
| Storage | microSD via SPI2 |

Build:
```bash
source ~/esp/esp-idf/export.sh
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/cu.usbserial-0001 flash monitor
```

#### microSD wiring (ESP32-S3 SPI)

| Signal | GPIO | Notes |
|--------|------|-------|
| MOSI   | 11   | SPI2 default |
| MISO   | 13   | SPI2 default |
| CLK    | 12   | SPI2 default |
| CS     | 10   | Chip select |

> **Warning**: GPIO 19 and 20 on ESP32-S3 are USB D−/D+ — do not use for SPI.

---

## System status

| Feature | Waveshare S3 | Waveshare P4 | Olimex P4-PC |
|---------|:---:|:---:|:---:|
| MOS shell | ✅ | ✅ | ✅ |
| Flash FAT (A:) | ✅ | ✅ | ✅ |
| SD card (B:) | ✅ | ✅ | ✅ |
| PSRAM heap | ✅ | ✅ | ✅ |
| External binary loader | ✅ | ✅ | — |
| Network / NTP | — | Wi-Fi | Ethernet |
| HDMI output | — | — | ✅ 800×600 |
| Audio (ES8311) | — | — | ✅ 48 kHz |
| USB HID keyboard | — | — | ✅ |
| Internal VDP | — | — | ✅ (640×480, 80×60) |

---

## Project structure

```
esp32-mos/
├── main/
│   ├── main.c              — app_main: board init → VDP → shell loop
│   └── idf_component.yml   — managed component dependencies
├── components/
│   ├── esp_lcd_lt8912b/    — MIPI DSI → HDMI bridge driver (LT8912B)
│   ├── mos_api/            — C-native MOS API (replaces RST 8 syscalls)
│   ├── mos_audio/          — ES8311 audio codec (I2S1 + I2C1, esp_codec_dev)
│   ├── mos_editor/         — VT100 line editor with history & Ctrl shortcuts
│   ├── mos_file/           — POSIX VFS helpers (resolvePath, copyFile)
│   ├── mos_fs/             — Flash FAT + SD mount / path resolution
│   ├── mos_hal/            — Console HAL (UART or USB-JTAG/CDC)
│   ├── mos_i2c/            — Shared I2C bus (port 1 on Olimex)
│   ├── mos_kbd/            — USB HID keyboard (HID boot protocol, PS/2 Set 2)
│   ├── mos_loader/         — PSRAM binary loader (two-step cache sync)
│   ├── mos_net/            — Network abstraction (Wi-Fi or Ethernet backend)
│   ├── mos_shell/          — Interactive shell and all built-in commands
│   ├── mos_sntp/           — SNTP time sync
│   ├── mos_strings/        — String utilities (stristr, pmatch, mos_strdup)
│   ├── mos_sysvars/        — System variables, GSTrans, expression eval
│   ├── mos_uart/           — UART driver
│   ├── mos_vdp/            — VDP TCP router (external VDP over network)
│   ├── mos_vdp_internal/   — Internal VDP (HDMI framebuffer, 640×480)
│   ├── mos_vdp_router/     — Dual VDP router: TCP external / internal
│   └── mos_wifi/           — Wi-Fi driver (ESP32-S3 / P4 + C6)
├── sdk/                    — User-program SDK (flat Xtensa binaries, PSRAM)
├── docs/
│   └── olimex-esp32-p4-pc-hardware.md
├── build_olimex_p4pc.sh    — Olimex P4-PC build script
├── build_p4.sh             — Waveshare P4 build script
├── build_s3.sh             — Waveshare S3 build script
├── sdkconfig.defaults                  — Common config
├── sdkconfig.defaults.waveshare-p4wifi — Waveshare P4 overrides
└── sdkconfig.defaults.olimex-p4pc     — Olimex P4-PC overrides
```

---

## Shell commands

| Command | Description |
|---------|-------------|
| `HELP` / `HELP ALL` | List commands |
| `DIR [path]` | List directory (`-l` long, `-a` hidden) |
| `CD <path>` | Change directory |
| `TYPE <file>` | Display file |
| `COPY <src> <dst>` | Copy file |
| `DEL <file>` | Delete file |
| `MKDIR <dir>` | Create directory |
| `ECHO <text>` | Print text (GSTrans applied) |
| `SET <var> <val>` | Set system variable |
| `SHOW [pattern]` | Show system variables |
| `IF <cond> THEN <cmd>` | Conditional |
| `IFTHERE <path> THEN <cmd>` | If file exists |
| `EXEC <file>` | Run batch file |
| `OBEY [-v] <file>` | Run script |
| `TIME [yr mo da ho mi se]` | Get/set clock |
| `MOUNT` | Remount SD card |
| `MEM` | Show heap / mount info |
| `CLS` | Clear screen |
| `HELLOWORLD` | Write test file to flash |

### Drive letters

| Prefix | Volume |
|--------|--------|
| `A:` | Internal flash FAT (`/flash`) |
| `B:` | SD card FAT (`/sdcard`) |

### Autoexec

Place `/flash/autoexec.obey` on the flash partition to run commands at startup.

---

## User program SDK

External programs run as flat Xtensa binaries loaded into PSRAM.  
See [`sdk/README.md`](sdk/README.md) for the full API and build instructions.

```bash
cd sdk/
make helloworld          # builds helloworld.bin → auto-copied to data/
./flash_data.sh /dev/cu.usbmodem2401   # flash to board storage
```

From the shell:
```
RUN A:/helloworld.bin
```

---

## Filesystem layout

```
partitions.csv:
  nvs       (24 KB)   — NVS key-value store
  phy_init  (4 KB)    — RF calibration data
  factory   (1 MB)    — Application firmware
  storage   (4 MB)    — FAT partition (A:)
```

---

## Differences from Agon MOS

| Feature | Agon MOS (eZ80) | ESP32-MOS |
|---------|----------------|-----------|
| CPU | eZ80 @ 18.432 MHz | Xtensa LX7/LX9 @ 240–400 MHz (dual-core) |
| Filesystem | FatFS direct | ESP-IDF VFS + wear-levelling |
| Memory | 512 KB on-chip RAM | up to 32 MB PSRAM |
| Syscalls | RST 8 (MOSAPI) | C function calls via `mos_api.h` |
| VDP | Separate eZ80 co-processor | Internal HDMI framebuffer (Olimex P4-PC) |
| SD interface | eZ80 SPI | SDMMC host or SPI |
| RTC | External (Agon) | `settimeofday` / SNTP |
| Network | None | Ethernet (Olimex) or Wi-Fi (Waveshare P4/S3) |
| eZ80-only commands | JMP, LOAD, SAVE, RUN, VDU, VDP | Omitted |

---

## License

Agon MOS is © Dean Belfield and contributors, licensed under the MIT License.  
ESP32 port additions are also MIT licensed.
