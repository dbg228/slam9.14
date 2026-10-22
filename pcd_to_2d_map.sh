#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

## mapping.sh / FAST-LIO overwrites this export on every successful save.
# Do not pick the newest *.pcd recursively: debug/filter outputs are newer
# than the original scan, but are not complete maps.
PCD_FILE="${1:-${SCRIPT_DIR}/robot_data/maps/data/scans.pcd}"
OUTPUT_PREFIX="${2:-robot_data/maps/map}"

if [ ! -s "${PCD_FILE}" ]; then
    echo "[pcd_to_2d_map] PCD not found: ${PCD_FILE}" >&2
    exit 1
fi

PCD_FILE="$(realpath -- "${PCD_FILE}")"
echo "[pcd_to_2d_map] input:  ${PCD_FILE}"
stat --printf='[pcd_to_2d_map] PCD saved: %y | size: %s bytes\n' -- "${PCD_FILE}"
echo "[pcd_to_2d_map] output: ${OUTPUT_PREFIX}.pgm / ${OUTPUT_PREFIX}.yaml"

./make_2d_map_from_pcd.py \
    --pcd "${PCD_FILE}" \
    --output-prefix "${OUTPUT_PREFIX}" \
    --z-min 0.05 \
    --z-max 0.35 \
    --enable-voxel-filter true \
    --voxel-leaf-size 0.03 \
    --enable-radius-filter true \
    --radius-search 0.10 \
    --radius-min-neighbors 2 \
    --map-resolution 0.05 \
    --min-points-per-cell 2 \
    --enable-vertical-filter true \
    --z-bin-size 0.05 \
    --min-z-bins-per-cell 2 \
    --min-z-span-per-cell 0.08 \
    --vertical-keep-dense-cells 6 \
    --enable-multi-height-evidence true \
    --height-layers=-0.15:0.05,0.05:0.35,0.35:0.60 \
    --multi-height-min-layers-per-cell 2 \
    --multi-height-min-points-per-layer 1 \
    --enable-line-bridge true \
    --line-vote-threshold 28 \
    --line-min-length-m 1.5 \
    --line-max-gap-m 0.40 \
    --line-min-support-ratio 0.65 \
    --enable-component-filter true \
    --connectivity 8 \
    --min-component-size 8 \
    --enable-morphological-close true \
    --closing-radius-cells 1 \
    --inflate-cells 0

echo "[pcd_to_2d_map] done:"
ls -lh --time-style=long-iso "${OUTPUT_PREFIX}.pgm" "${OUTPUT_PREFIX}.yaml"
