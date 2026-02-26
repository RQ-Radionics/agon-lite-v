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

IDF_ROOT="${IDF_PATH:-/Users/rampa/esp/esp-idf}"
FATFSGEN="${IDF_ROOT}/components/fatfs/fatfsgen.py"

# Use IDF's own Python (has 'construct' and other required modules)
IDF_PYTHON="${IDF_PYTHON_ENV_PATH:-}"
if [ -z "${IDF_PYTHON}" ]; then
    # Try to find it under ~/.espressif/python_env
    IDF_PYTHON=$(ls -d ~/.espressif/python_env/idf5.5_py*_env 2>/dev/null | tail -1)
fi
PYTHON="${IDF_PYTHON}/bin/python"
if [ ! -x "${PYTHON}" ]; then
    PYTHON="python3"
fi

PORT="${1:-${ESPPORT:-/dev/cu.usbserial-0001}}"

# ---- sanity checks ----
if [ ! -d "${DATA_DIR}" ]; then
    echo "ERROR: data/ directory not found at ${DATA_DIR}"
    exit 1
fi

if [ ! -f "${FATFSGEN}" ]; then
    echo "ERROR: fatfsgen.py not found at ${FATFSGEN}"
    echo "  Set IDF_PATH or edit this script"
    exit 1
fi

if ! command -v esptool.py &>/dev/null; then
    echo "ERROR: esptool.py not found. Activate IDF environment first:"
    echo "  source /Users/rampa/esp/esp-idf/export.sh"
    exit 1
fi

mkdir -p "${SCRIPT_DIR}/build"

# ---- build FAT image ----
# --wl_mode perf: must match esp_vfs_fat_spiflash_mount_rw_wl() which uses wear levelling
echo "Building FAT image from ${DATA_DIR} ..."
"${PYTHON}" "${FATFSGEN}" \
    --output_file "${IMAGE}" \
    --partition_size ${PARTITION_SIZE} \
    --sector_size 512 \
    --long_name_support \
    --wl_mode perf \
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
