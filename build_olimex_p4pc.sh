#!/usr/bin/env bash
# build_olimex_p4pc.sh — Build for Olimex ESP32-P4-PC (RISC-V RV32IMAFC)
#
# Uses ONLY sdkconfig.defaults (common) + sdkconfig.defaults.olimex-p4pc.
# Does NOT include sdkconfig.defaults.esp32p4 (which enables the esp-hosted
# C6 WiFi coprocessor and would trigger SDIO probing on a board without one).
set -e

cd "$(dirname "$0")"

echo "=== Olimex ESP32-P4-PC build ==="
# CRITICAL: wipe any cached sdkconfig from a previous (Waveshare) build.
# IDF merges sdkconfig.defaults into an existing sdkconfig, so stale entries
# (e.g. CONFIG_ESP_HOSTED for the C6 coprocessor) survive if not deleted.
rm -f sdkconfig
rm -rf build

export SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.defaults.olimex-p4pc"
idf.py set-target esp32p4
idf.py build

echo ""
echo "=== SDK user programs (RISC-V) ==="
cd sdk
make clean
make TARGET=esp32p4

echo ""
echo "Done. Flash with:"
echo "  idf.py -p /dev/cu.usbmodem* flash"
echo "  ./flash_data.sh /dev/cu.usbmodem* olimex-p4pc"
