# ESP32-MOS

Port of the [Agon MOS](https://github.com/AgonConsole8/agon-mos) shell from the eZ80 CPU to the **ESP32-S3** with **ESP-IDF v5.5**.

**Scenario A**: The ESP32-S3 replaces the eZ80 entirely as the main CPU.  
No VDP, no Z80 emulator, no RST syscalls. The MOS shell runs natively on Xtensa.

---

## Hardware

### Minimum

| Component | Notes |
|-----------|-------|
| ESP32-S3 DevKit | Any board with ≥8 MB flash |
| USB-Serial | UART0 is the interactive console (TX=GPIO1, RX=GPIO3) |

### Optional: microSD card (SPI)

Default wiring — override via `sdkconfig` or `#define` before including `mos_fs.c`:

| Signal | GPIO | Notes |
|--------|------|-------|
| MOSI   | 11   | SPI2 default on ESP32-S3 |
| MISO   | 13   | SPI2 default on ESP32-S3 |
| CLK    | 12   | SPI2 default on ESP32-S3 |
| CS     | 10   | Chip select |

> **Warning**: GPIO 19 and 20 on ESP32-S3 are USB D−/D+ and **must not** be used for SPI.

To override pins, add to `sdkconfig.defaults` before the first build:

```
CONFIG_MOS_SD_PIN_MOSI=35
CONFIG_MOS_SD_PIN_MISO=36
CONFIG_MOS_SD_PIN_CLK=37
CONFIG_MOS_SD_PIN_CS=34
```

or define them in your top-level `CMakeLists.txt`:

```cmake
add_compile_definitions(
    MOS_SD_PIN_MOSI=35
    MOS_SD_PIN_MISO=36
    MOS_SD_PIN_CLK=37
    MOS_SD_PIN_CS=34
)
```

### Console (UART0)

| Signal | GPIO |
|--------|------|
| TX     | 1    |
| RX     | 3    |

Baud rate: **115200 8N1**

---

## Build

### Prerequisites

- ESP-IDF v5.5 installed at `/Users/rampa/esp/esp-idf/` (or wherever you installed it)
- Xtensa toolchain installed via `idf_tools.py install`

### Steps

```bash
# 1. Activate IDF environment
source ~/esp/esp-idf/export.sh

# 2. Configure target (only needed once, or after deleting sdkconfig)
idf.py set-target esp32s3

# 3. Build
idf.py build
```

The output binary is `build/esp32-mos.bin` (~374 KB; 64% of the 1 MB app partition is free).

### Flash

```bash
idf.py -p /dev/ttyUSB0 flash
# or on macOS:
idf.py -p /dev/cu.usbserial-0001 flash
```

### Monitor

```bash
idf.py -p /dev/ttyUSB0 monitor
# Ctrl-] to exit
```

---

## First Boot

On first boot the flash FAT partition is **auto-formatted** (this takes ~2 s). After that:

```
ESP32-MOS 3.0.1 ESP32-S3 "Arthur"
Flash: OK  SD: N/A
Type HELP for commands.

A/flash.* 
```

The prompt shows `<drive><cwd>.*`.

### Quick command reference

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

## Filesystem layout

```
partitions.csv:
  nvs       (24 KB)   — NVS key-value store
  phy_init  (4 KB)    — RF calibration data
  factory   (1 MB)    — Application firmware
  storage   (4 MB)    — FAT partition (A:)
```

---

## Project structure

```
esp32-mos/
├── main/main.c                     — app_main: init → autoexec → shell loop
├── components/
│   ├── mos_hal/                    — UART console driver
│   ├── mos_fs/                     — Flash FAT + SD SPI mount / path resolution
│   ├── mos_strings/                — String utilities (stristr, pmatch, mos_strdup)
│   ├── mos_file/                   — POSIX VFS helpers (resolvePath, copyFile)
│   ├── mos_sysvars/                — System variables, GSTrans, expression eval
│   ├── mos_editor/                 — VT100 line editor with history & Ctrl shortcuts
│   ├── mos_shell/                  — Interactive shell and all built-in commands
│   └── mos_api/                    — C-native MOS API (replaces RST 8 syscalls)
├── partitions.csv
├── sdkconfig.defaults
└── CMakeLists.txt
```

---

## Differences from Agon MOS

| Feature | Agon MOS (eZ80) | ESP32-MOS |
|---------|----------------|-----------|
| CPU | eZ80 @ 18.432 MHz | Xtensa LX7 @ 240 MHz (dual-core) |
| Filesystem | FatFS direct | ESP-IDF VFS + wear-levelling |
| Memory | 512 KB on-chip RAM | 512 KB SRAM + up to 8 MB PSRAM |
| Syscalls | RST 8 (MOSAPI) | C function calls via `mos_api.h` |
| VDP | Separate co-processor | Not applicable (Scenario A) |
| SD interface | eZ80 SPI | ESP32-S3 SPI (SPI2_HOST) |
| RTC | External (Agon) | `settimeofday` / SNTP (optional) |
| eZ80-only commands | JMP, LOAD, SAVE, RUN, VDU, VDP | Omitted |

---

## License

Agon MOS is © Dean Belfield and contributors, licensed under the MIT License.  
ESP32 port additions are also MIT licensed.
