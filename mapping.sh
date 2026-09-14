#!/usr/bin/env bash
set -euo pipefail

TEST_NAV_WS="/home/zkam/test_nav"
FASTLIO_WS="/home/zkam/fastlio_ws"
CONFIG_FILE="${FASTLIO_CONFIG_FILE:-mid360.yaml}"
USE_RVIZ="${FASTLIO_RVIZ:-true}"
ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/test_nav_ros_logs}"
MAP_FILE="${FASTLIO_MAP_FILE:-/home/zkam/test_nav/robot_data/maps/data/scans.pcd}"
export ROS_LOG_DIR

FAST_LIO_PID=0
CLEANED_UP=0

backup_existing_map() {
    if [ ! -f "${MAP_FILE}" ]; then
        return 0
    fi

    local backup_file
    backup_file="${MAP_FILE}.bak_$(date +%Y%m%d_%H%M%S)"
    cp -p -- "${MAP_FILE}" "${backup_file}"
    echo "[mapping] Backed up existing PCD before overwrite: ${backup_file}"
}

request_map_save() {
    echo "[mapping] Requesting FAST-LIO map save..."

    if ! timeout 10s bash -lc 'until ros2 service list | grep -qx /map_save; do sleep 0.2; done'; then
        echo "[mapping] /map_save service is not available; map was NOT saved."
        return 1
    fi

    # map_save overwrites MAP_FILE. Keep the PCD that matches the currently
    # deployed 2-D map so localization can be restored if a short mapping run
    # produces a different FAST-LIO coordinate origin.
    backup_existing_map

    if timeout 60s ros2 service call /map_save std_srvs/srv/Trigger "{}"; then
        if [ -f "${MAP_FILE}" ]; then
            echo "[mapping] Saved map file:"
            ls -lh --time-style=long-iso "${MAP_FILE}"
        else
            echo "[mapping] Save service returned, but ${MAP_FILE} does not exist."
            return 1
        fi
    else
        echo "[mapping] Map save call failed."
        return 1
    fi
}

cleanup() {
    if [ "${CLEANED_UP}" -ne 0 ]; then
        return
    fi
    CLEANED_UP=1

    if [ "${FAST_LIO_PID}" -ne 0 ] && kill -0 "${FAST_LIO_PID}" 2>/dev/null; then
        request_map_save || true
        kill -INT "-${FAST_LIO_PID}" 2>/dev/null || kill -INT "${FAST_LIO_PID}" 2>/dev/null || true
        wait "${FAST_LIO_PID}" 2>/dev/null || true
    fi
}

trap cleanup INT TERM HUP EXIT

mkdir -p "${ROS_LOG_DIR}"

cd "${TEST_NAV_WS}"
set +u
source /opt/ros/humble/setup.bash
source "${TEST_NAV_WS}/install/setup.bash"
source "${FASTLIO_WS}/install/setup.bash"
set -u

# Run the launch process in a separate process group.  Otherwise Ctrl-C from
# the terminal reaches fastlio_mapping before this wrapper can call /map_save.
setsid ros2 launch fast_lio mapping.launch.py \
    config_file:="${CONFIG_FILE}" \
    rviz:="${USE_RVIZ}" &

FAST_LIO_PID=$!
wait "${FAST_LIO_PID}"
