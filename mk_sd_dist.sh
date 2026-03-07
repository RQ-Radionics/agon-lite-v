#!/usr/bin/env bash
# mk_sd_dist.sh — Build the SD card distribution directory and zip archive
#
# Usage: ./mk_sd_dist.sh [BOARD]
#   BOARD  defaults to esp32p4 (also: olimex-p4pc)
#
# Merges data/common/ + data/esp32p4/ + data/<BOARD>/ (if different) into
# a staging directory 'sdcard/', then zips it as 'sdcard.zip'.
#
# The sdcard/ directory can be copied directly to a FAT32 microSD card.
# The sdcard.zip is ready for use as a GitHub release artifact.
#
# No IDF environment needed — just bash + zip.

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BOARD="${1:-esp32p4}"

DATA_COMMON="${SCRIPT_DIR}/data/common"
DATA_P4="${SCRIPT_DIR}/data/esp32p4"
DATA_BOARD="${SCRIPT_DIR}/data/${BOARD}"
OUT_DIR="${SCRIPT_DIR}/sdcard"
ZIP_FILE="${SCRIPT_DIR}/sdcard.zip"

# ---- sanity checks ----
if [ ! -d "${DATA_COMMON}" ]; then
    echo "ERROR: data/common/ not found"
    exit 1
fi

if [ ! -d "${DATA_P4}" ]; then
    echo "ERROR: data/esp32p4/ not found — build SDK binaries first: cd sdk && make"
    exit 1
fi

# ---- build staging directory ----
echo "=== Building SD card distribution for ${BOARD} ==="

rm -rf "${OUT_DIR}"
mkdir -p "${OUT_DIR}"

# Layer 1: common files (.bas, .bbc, wifi.cfg, tests/, etc.)
echo "  Copying data/common/ ..."
cp -r "${DATA_COMMON}/." "${OUT_DIR}/"

# Layer 2: esp32p4 binaries (bin/, demos/, games/, *.bin)
echo "  Copying data/esp32p4/ ..."
cp -r "${DATA_P4}/." "${OUT_DIR}/"

# Layer 3: board-specific overrides (only if different from esp32p4)
if [ -d "${DATA_BOARD}" ] && [ "${DATA_BOARD}" != "${DATA_P4}" ]; then
    echo "  Applying ${BOARD} overrides from data/${BOARD}/ ..."
    cp -r "${DATA_BOARD}/." "${OUT_DIR}/"
fi

# ---- summary ----
NFILES=$(find "${OUT_DIR}" -type f | wc -l | tr -d ' ')
echo "  Total: ${NFILES} files → sdcard/"

# ---- zip ----
echo "  Creating sdcard.zip ..."
rm -f "${ZIP_FILE}"
(cd "${SCRIPT_DIR}" && zip -r -q sdcard.zip sdcard/)
ZIPSIZE=$(wc -c < "${ZIP_FILE}" | tr -d ' ')
echo "  sdcard.zip: ${ZIPSIZE} bytes"

echo ""
echo "Done."
echo "  Copy sdcard/ contents to a FAT32 microSD card, or"
echo "  use sdcard.zip as a GitHub release artifact."
