#!/usr/bin/env bash
# build_s3.sh — Clean build for ESP32-S3 (Xtensa LX7)
set -e

cd "$(dirname "$0")"

echo "=== ESP32-S3 build ==="
idf.py set-target esp32s3
idf.py build

echo ""
echo "=== SDK user programs (Xtensa) ==="
cd sdk
make clean
make TARGET=esp32s3

echo ""
echo "Done. Flash with:"
echo "  idf.py -p /dev/cu.usbserial-0001 flash"
echo "  ./flash_data.sh /dev/cu.usbserial-0001 esp32s3"
