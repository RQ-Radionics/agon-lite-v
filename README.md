# ESP32-MOS

Port of the [Agon MOS](https://github.com/AgonConsole8/agon-mos) shell and [BBC BASIC](https://github.com/AgonConsole8/agon-bbc-basic) from the eZ80 CPU to **ESP32-P4** with **ESP-IDF v5.5**, running on the [Olimex ESP32-P4-PC](https://www.olimex.com/Products/IoT/ESP32-P4/) — a remarkable open-hardware board with HDMI, Ethernet, audio, USB, and 32 MB PSRAM.

The ESP32-P4 replaces the eZ80 as the main CPU. No Z80 emulator, no co-processor. MOS and BBC BASIC run natively on the dual-core RISC-V @ 400 MHz.

---

## Credits

- **[Agon Platform / Console8](https://github.com/AgonConsole8)** — Dean Belfield, Jeroen Venema, and all contributors to agon-mos, agon-vdp, and the wider Agon community. This project would not exist without their work.
- **[BBC BASIC for Agon](https://github.com/AgonConsole8/agon-bbc-basic)** — R.T. Russell (original BBC BASIC V), Dean Belfield (Agon port). A masterpiece of elegant software that still runs circles around most modern interpreters.
- **[Olimex](https://www.olimex.com)** — for designing and manufacturing the open-hardware ESP32-P4-PC board, publishing full schematics, and making this kind of project possible.
- **[Espressif Systems](https://www.espressif.com)** — for ESP-IDF, the esp-bsp component library, and the LT8912B HDMI driver.

---

## Supported Hardware

### Olimex ESP32-P4-PC (primary target)

| Component | Details |
|-----------|---------|
| SoC | ESP32-P4NRW32 (dual RISC-V @ 400 MHz, 32 MB PSRAM) |
| Display | HDMI 1024×768@60Hz via LT8912B (MIPI DSI → HDMI bridge) |
| Audio | ES8311 codec — I2S0, I2C1 — 16384 Hz stereo |
| Keyboard | USB HID via FE1.1s hub (USB PHY1 / OTG11) |
| Storage | microSD card (SDMMC host, slot 0) |
| Network | 100 Mbit Ethernet — IP101GRR PHY via RMII |
| Console | USB-JTAG/CDC (USB-C connector) |

### Waveshare ESP32-P4-WIFI6 (secondary target)

| Component | Details |
|-----------|---------|
| SoC | ESP32-P4 (dual RISC-V @ 400 MHz, 8 MB PSRAM) |
| Network | Wi-Fi 6 / BLE5 via ESP32-C6 coprocessor |
| Console | USB-JTAG/CDC |

---

## Flashing — no development environment required

You can flash pre-built firmware directly from your browser using the **online flasher** at [https://rq-radionics.github.io/agonV-online-flasher/](https://rq-radionics.github.io/agonV-online-flasher/):

1. Download the latest release `.bin` files from the [Releases](../../releases) page
2. Connect the board via USB-C
3. Open the web flasher, select the COM port, and flash each `.bin` at the address listed in the release notes

To build from source, you need ESP-IDF v5.5:

```bash
# Olimex ESP32-P4-PC
bash build_olimex_p4pc.sh
idf.py -p /dev/cu.usbmodem* flash

# Waveshare ESP32-P4-WIFI6
bash build_p4.sh
idf.py -p /dev/cu.usbmodem* flash
```

---

## Storage — SD card only

All user files live on the **microSD card**. There is no internal flash FAT partition.

- Insert a FAT32-formatted microSD card before booting
- Drive `A:` and `B:` both map to the SD card root
- Place `autoexec.obey` in the SD root to run commands at startup

### Updating files over the network

The board runs an FTP server when a network connection is available. You can transfer BBC BASIC programs, binaries, and config files directly — no need to remove the SD card:

```
ftp <board-ip-address>
```

Username: `mos` / Password: `mos`

---

## Wi-Fi and NTP

Create a file `wifi.cfg` in the **root of the SD card**:

```
SSID=YourNetworkName
PASSWORD=YourPassword
```

The board reads this at boot, connects to Wi-Fi (Waveshare) or uses Ethernet (Olimex), and syncs the RTC automatically via NTP. The current time is available via the `TIME` command.

---

## Keyboard

The USB HID keyboard driver currently works with:

- ✅ **High Speed (HS) keyboards** — any keyboard that operates at USB 2.0 High Speed (480 Mbps), connected directly or via the onboard hub
- ✅ **2.4 GHz wireless USB receivers** — most wireless keyboard dongles work correctly
- ❌ **Standard wired USB keyboards** — most cheap wired keyboards are Low Speed (1.5 Mbps) or Full Speed (12 Mbps) and require SPLIT transactions through the hub's Transaction Translator (TT), which are not yet implemented in ESP-IDF's USB host stack

**In practice**: a gaming keyboard, or any keyboard marked "USB 2.0 High Speed", will work reliably. Most wireless keyboards with a USB dongle also work. If your keyboard does not respond, try a different model.

See [GitHub issue #1](../../issues/1) for the technical background and the plan to add LS/FS support.

---

## System status

| Feature | Waveshare P4 | Olimex P4-PC |
|---------|:---:|:---:|
| MOS shell | ✅ | ✅ |
| SD card storage | ✅ | ✅ |
| PSRAM heap | ✅ | ✅ |
| External binary loader | ✅ | ✅ |
| FTP server | ✅ | ✅ |
| Network / NTP / RTC | Wi-Fi | Ethernet |
| HDMI output | — | ✅ 1024×768@60Hz |
| Audio (ES8311) | — | ✅ 16384 Hz |
| USB HID keyboard | — | ✅ (HS / wireless) |
| BBC BASIC | ✅ | ✅ |
| Internal VDP | — | ✅ (all Agon modes) |
| Mode 7 (Teletext) | — | ✅ |

---

## Project structure

```
esp32-mos/
├── main/
│   ├── main.c              — board init → VDP → shell loop
│   └── idf_component.yml   — managed component dependencies
├── components/
│   ├── esp_lcd_lt8912b/    — MIPI DSI → HDMI bridge driver (LT8912B)
│   ├── mos_api/            — C-native MOS API
│   ├── mos_audio/          — ES8311 audio codec (direct driver)
│   ├── mos_editor/         — VT100 line editor with history & Ctrl shortcuts
│   ├── mos_file/           — POSIX VFS helpers
│   ├── mos_fs/             — SD mount / path resolution
│   ├── mos_hal/            — Console HAL (USB-JTAG/CDC or UART)
│   ├── mos_kbd/            — USB HID keyboard (HID boot protocol, PS/2 Set 2)
│   ├── mos_loader/         — PSRAM binary loader (two-step cache sync)
│   ├── mos_net/            — Network abstraction (Wi-Fi or Ethernet)
│   ├── mos_shell/          — Interactive shell and built-in commands
│   ├── mos_sntp/           — SNTP/NTP time sync
│   ├── mos_strings/        — String utilities
│   ├── mos_sysvars/        — System variables, GSTrans, expression eval
│   ├── mos_vdp_internal/   — Internal VDP (HDMI framebuffer, all Agon modes)
│   └── mos_vdp_router/     — Dual VDP router: TCP external / internal
├── sdk/                    — User-program SDK (flat RISC-V binaries, PSRAM)
├── docs/
│   └── olimex-esp32-p4-pc-hardware.md
├── build_olimex_p4pc.sh    — Olimex P4-PC build script
├── build_p4.sh             — Waveshare P4 build script
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

---

## User program SDK

External programs run as flat RISC-V binaries loaded into PSRAM.  
See [`sdk/README.md`](sdk/README.md) for the full API and build instructions.

```bash
cd sdk/
make helloworld          # builds helloworld.bin → auto-copied to data/

cd ..
./mk_sd_dist.sh          # merges data/ into sdcard/ and sdcard.zip
# then copy sdcard/ contents to your FAT32 microSD card
```

From the shell:
```
helloworld
```

---

## Differences from Agon MOS

| Feature | Agon MOS (eZ80) | ESP32-MOS |
|---------|----------------|-----------|
| CPU | eZ80 @ 18.432 MHz | RISC-V @ 400 MHz (dual-core) |
| Filesystem | FatFS direct | ESP-IDF VFS |
| Memory | 512 KB on-chip RAM | 32 MB PSRAM |
| Syscalls | RST 8 (MOSAPI) | C function calls via `mos_api.h` |
| VDP | Separate eZ80 co-processor | Internal HDMI framebuffer |
| Storage | Flash + SD | SD card only |
| RTC | External | NTP-synced via Wi-Fi or Ethernet |
| Network | None | Ethernet (Olimex) or Wi-Fi (Waveshare) |

---

## License

[Agon MOS](https://github.com/AgonConsole8/agon-mos) is © Dean Belfield and contributors, MIT License.  
[BBC BASIC for Agon](https://github.com/AgonConsole8/agon-bbc-basic) is © R.T. Russell and Dean Belfield, MIT License.  
ESP32 port additions are also MIT licensed.
