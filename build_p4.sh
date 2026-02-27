#!/usr/bin/env bash
# build_p4.sh — Clean build for ESP32-P4 (RISC-V RV32IMAFC)
set -e

cd "$(dirname "$0")"

echo "=== ESP32-P4 build ==="
idf.py set-target esp32p4
idf.py build

echo ""
echo "=== SDK user programs (RISC-V) ==="
cd sdk
make clean
make TARGET=esp32p4

echo ""
echo "Done. Flash with:"
echo "  idf.py -p /dev/cu.usbserial-0001 flash"
echo "  ./flash_data.sh /dev/cu.usbserial-0001 esp32p4"
