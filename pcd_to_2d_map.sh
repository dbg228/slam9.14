#!/usr/bin/env bash
set -euo pipefail

cd /home/zkam/test_nav

PCD_FILE="${1:-robot_data/maps/data/scans.pcd}"
OUTPUT_PREFIX="${2:-robot_data/maps/map}"

if [ ! -f "${PCD_FILE}" ]; then
    echo "[pcd_to_2d_map] PCD not found: ${PCD_FILE}" >&2
    exit 1
fi

echo "[pcd_to_2d_map] input:  ${PCD_FILE}"
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
    --enable-component-filter true \
    --connectivity 8 \
    --min-component-size 8 \
    --inflate-cells 0

echo "[pcd_to_2d_map] done:"
ls -lh --time-style=long-iso "${OUTPUT_PREFIX}.pgm" "${OUTPUT_PREFIX}.yaml"
