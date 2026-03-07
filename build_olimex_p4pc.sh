#!/usr/bin/env bash
# build_olimex_p4pc.sh — Build for Olimex ESP32-P4-PC (RISC-V RV32IMAFC)
#
# Uses ONLY sdkconfig.defaults (common) + sdkconfig.defaults.olimex-p4pc.
# Does NOT include sdkconfig.defaults.esp32p4 (which enables the esp-hosted
# C6 WiFi coprocessor and would trigger SDIO probing on a board without one).
set -e

cd "$(dirname "$0")"

echo "=== Olimex ESP32-P4-PC build ==="
# Wipe cached sdkconfig and build/ to ensure a clean start.
# SDKCONFIG_DEFAULTS overrides win because idf.py applies them after the
# Kconfig defaults during the configure phase of `build`.
rm -f sdkconfig
rm -rf build

export SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.defaults.olimex-p4pc"
idf.py -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.defaults.olimex-p4pc" \
    set-target esp32p4
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
