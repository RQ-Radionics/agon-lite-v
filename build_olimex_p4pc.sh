#!/usr/bin/env bash
# build_olimex_p4pc.sh — Build for Olimex ESP32-P4-PC (RISC-V RV32IMAFC)
#
# Uses ONLY sdkconfig.defaults (common) + sdkconfig.defaults.olimex-p4pc.
# Does NOT include sdkconfig.defaults.esp32p4 (which enables the esp-hosted
# C6 WiFi coprocessor and would trigger SDIO probing on a board without one).
set -e

cd "$(dirname "$0")"

echo "=== Olimex ESP32-P4-PC build ==="
# CRITICAL: wipe any cached sdkconfig from a previous build.
# IDF merges sdkconfig.defaults into an existing sdkconfig, so stale entries
# survive if not deleted.
#
# NOTE: idf.py set-target regenerates sdkconfig from Kconfig defaults BEFORE
# applying SDKCONFIG_DEFAULTS — so keys like ESP_MAIN_TASK_STACK_SIZE get the
# Kconfig default (65536) rather than our override (8192).  We therefore wipe
# sdkconfig a SECOND TIME after set-target so that idf.py build starts clean
# and applies SDKCONFIG_DEFAULTS from scratch.
rm -f sdkconfig
rm -rf build

export SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.defaults.olimex-p4pc"
idf.py -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.defaults.olimex-p4pc" set-target esp32p4
# Wipe again: set-target wrote Kconfig defaults; we want our overrides to win.
rm -f sdkconfig
idf.py -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.defaults.olimex-p4pc" build

echo ""
echo "=== SDK user programs (RISC-V) ==="
cd sdk
make clean
make TARGET=esp32p4

echo ""
echo "Done. Flash with:"
echo "  idf.py -p /dev/cu.usbmodem* flash"
echo "  ./flash_data.sh /dev/cu.usbmodem* olimex-p4pc"
