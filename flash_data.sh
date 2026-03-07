#!/usr/bin/env bash
# flash_data.sh — Build a FAT image from data/ and flash it to the storage partition
# Usage: ./flash_data.sh [PORT] [BOARD]
#   PORT  defaults to /dev/cu.usbserial-0001 or set via $ESPPORT
#   BOARD defaults to esp32p4 (also accepts olimex-p4pc)
#
# The FAT image is built from:
#   data/common/       — shared files (.bas, .bat, .rgb, etc.)
#   data/<BOARD>/      — target-specific binaries (.bin)
#
# Requires ESP-IDF environment active: source /Users/rampa/esp/esp-idf/export.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

PORT="${1:-${ESPPORT:-/dev/cu.usbserial-0001}}"
BOARD="${2:-esp32p4}"

# Map board names to esptool chip identifiers
case "${BOARD}" in
    olimex-p4pc)    CHIP="esp32p4" ;;
    *)              CHIP="${BOARD}" ;;
esac

DATA_COMMON="${SCRIPT_DIR}/data/common"
DATA_TARGET="${SCRIPT_DIR}/data/${BOARD}"
BUILD_DIR="${SCRIPT_DIR}/build"
STAGING="${BUILD_DIR}/data_staging"
FAT_RAW="${BUILD_DIR}/fat_raw.bin"
IMAGE="${BUILD_DIR}/storage.bin"
WL_WRAP="${SCRIPT_DIR}/wl_wrap.py"

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

# ---- sanity checks ----
if [ ! -d "${DATA_COMMON}" ]; then
    echo "ERROR: data/common/ directory not found at ${DATA_COMMON}"
    exit 1
fi

if [ ! -d "${DATA_TARGET}" ]; then
    echo "ERROR: data/${BOARD}/ directory not found at ${DATA_TARGET}"
    echo "  Build SDK binaries first: cd sdk && make TARGET=${CHIP}"
    exit 1
fi

if [ ! -f "${FATFSGEN}" ]; then
    echo "ERROR: fatfsgen.py not found at ${FATFSGEN}"
    echo "  Set IDF_PATH or edit this script"
    exit 1
fi

if [ ! -f "${WL_WRAP}" ]; then
    echo "ERROR: wl_wrap.py not found at ${WL_WRAP}"
    exit 1
fi

if ! command -v esptool.py &>/dev/null; then
    echo "ERROR: esptool.py not found. Activate IDF environment first:"
    echo "  source /Users/rampa/esp/esp-idf/export.sh"
    exit 1
fi

mkdir -p "${BUILD_DIR}"

# ---- step 0: merge common + target into staging directory ----
echo "Staging data for ${BOARD} (chip: ${CHIP}) ..."
rm -rf "${STAGING}"
mkdir -p "${STAGING}"
cp "${DATA_COMMON}"/* "${STAGING}/" 2>/dev/null || true
cp "${DATA_TARGET}"/* "${STAGING}/" 2>/dev/null || true
echo "  common: $(ls "${DATA_COMMON}" 2>/dev/null | wc -l | tr -d ' ') files"
  echo "  ${BOARD}: $(ls "${DATA_TARGET}" 2>/dev/null | wc -l | tr -d ' ') files"

# ---- step 1: build raw FAT image (no --wl_mode) ----
# fatfsgen.py --wl_mode does NOT generate the WL state/config blocks that the
# C wear_levelling driver expects. We generate a plain FAT image and then inject
# the correct WL blocks with wl_wrap.py.
echo "Building raw FAT image ..."
"${PYTHON}" "${FATFSGEN}" \
    --output_file "${FAT_RAW}" \
    --partition_size ${PARTITION_SIZE} \
    --sector_size 512 \
    --long_name_support \
    "${STAGING}"

# ---- step 2: inject WL state/config blocks ----
echo "Injecting wear-levelling blocks ..."
python3 "${WL_WRAP}" "${FAT_RAW}" "${IMAGE}" ${PARTITION_OFFSET} ${PARTITION_SIZE}

echo "Image: ${IMAGE} ($(wc -c < "${IMAGE}") bytes)"

# ---- cleanup staging ----
rm -rf "${STAGING}"

# ---- flash ----
echo "Flashing to ${PORT} at offset ${PARTITION_OFFSET} ..."
esptool.py \
    --chip "${CHIP}" \
    --port "${PORT}" \
    --baud 921600 \
    write_flash \
    ${PARTITION_OFFSET} "${IMAGE}"

echo "Done."
