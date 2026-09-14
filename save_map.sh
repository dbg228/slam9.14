#!/usr/bin/env bash
set -euo pipefail

TEST_NAV_WS="/home/zkam/test_nav"
FASTLIO_WS="/home/zkam/fastlio_ws"
MAP_FILE="${FASTLIO_MAP_FILE:-/home/zkam/test_nav/robot_data/maps/data/scans.pcd}"

cd "${TEST_NAV_WS}"
set +u
source /opt/ros/humble/setup.bash
source "${TEST_NAV_WS}/install/setup.bash"
source "${FASTLIO_WS}/install/setup.bash"
set -u

echo "[save_map] Waiting for /map_save service..."
timeout 10s bash -lc 'until ros2 service list | grep -qx /map_save; do sleep 0.2; done'

echo "[save_map] Requesting FAST-LIO map save..."
timeout 60s ros2 service call /map_save std_srvs/srv/Trigger "{}"

if [ -f "${MAP_FILE}" ]; then
    echo "[save_map] Saved map file:"
    ls -lh --time-style=long-iso "${MAP_FILE}"
else
    echo "[save_map] Save service returned, but ${MAP_FILE} does not exist." >&2
    exit 1
fi
