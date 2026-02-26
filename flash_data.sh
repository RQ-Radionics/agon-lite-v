#!/usr/bin/env bash
# flash_data.sh — Build a FAT image from data/ and flash it to the storage partition
# Usage: ./flash_data.sh [PORT]
#   PORT defaults to /dev/cu.usbserial-0001 or set via $ESPPORT
#
# Requires ESP-IDF environment active: source /Users/rampa/esp/esp-idf/export.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
DATA_DIR="${SCRIPT_DIR}/data"
IMAGE="${SCRIPT_DIR}/build/storage.bin"

# Partition storage: offset 0x210000, size 4MB
PARTITION_OFFSET=0x210000
PARTITION_SIZE=$((4 * 1024 * 1024))

PORT="${1:-${ESPPORT:-/dev/cu.usbserial-0001}}"

# ---- sanity checks ----
if [ ! -d "${DATA_DIR}" ]; then
    echo "ERROR: data/ directory not found at ${DATA_DIR}"
    exit 1
fi

if ! command -v fatfsgen.py &>/dev/null; then
    echo "ERROR: fatfsgen.py not found. Activate IDF environment first:"
    echo "  source /Users/rampa/esp/esp-idf/export.sh"
    exit 1
fi

# ---- build FAT image ----
echo "Building FAT image from ${DATA_DIR} ..."
fatfsgen.py \
    --output_file "${IMAGE}" \
    --partition_size ${PARTITION_SIZE} \
    --long_name_support \
    "${DATA_DIR}"

echo "Image: ${IMAGE} ($(wc -c < "${IMAGE}") bytes)"

# ---- flash ----
echo "Flashing to ${PORT} at offset ${PARTITION_OFFSET} ..."
esptool.py \
    --chip esp32s3 \
    --port "${PORT}" \
    --baud 921600 \
    write_flash \
    ${PARTITION_OFFSET} "${IMAGE}"

echo "Done."
