#!/usr/bin/env bash
# FAST-LIO-only trajectory-baseline launcher.
#
# This is deliberately separate from mapping.sh and localization.sh.  It
# starts only FAST-LIO plus the two local TF/odometry adapters needed by the
# motion-capture evaluation tools:
#
#   /tf_lio_raw: camera_init -> body        (FAST-LIO, isolated raw TF)
#   /tf:         odom -> base_link          (lio_tf_to_base_tf.py)
#   /odom:       odom -> base_link odometry (tf_to_odom_bridge.py)
#
# It does NOT start static_map_localizer_node, Nav2, a map->odom broadcaster,
# RViz, or /map_save.  PCD writing is disabled at the command line even when
# the selected FAST-LIO YAML enables it.

# ROS setup scripts inspect optional AMENT_* variables, so do not enable
# nounset until after they have been sourced.
set -eo pipefail

TEST_NAV_WS="${TEST_NAV_WS:-/home/zkam/test_nav}"
FASTLIO_WS="${FASTLIO_WS:-/home/zkam/fastlio_ws}"
FASTLIO_CONFIG="${FASTLIO_CONFIG:-${FASTLIO_WS}/src/FAST_LIO_ROS2/config/mid360.yaml}"
ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/test_nav_ros_logs}"

RAW_TF_TOPIC="${FASTLIO_RAW_TF_TOPIC:-/tf_lio_raw}"
ODOM_TOPIC="${FASTLIO_ODOM_TOPIC:-/odom}"

FASTLIO_PID=0
LIO_TF_PID=0
TF_ODOM_PID=0
CLEANED_UP=0

# Share localization.sh's lock: FAST-LIO-only evaluation must never run next
# to the static-map/NDT chain, because both would publish odom->base_link.
exec 9>/tmp/test_nav_static_localization.lock
if ! flock -n 9; then
    echo "[fastlio-baseline] ERROR: static localization or another baseline is already running." >&2
    echo "[fastlio-baseline] Stop it first; do not run two odom->base_link publishers." >&2
    exit 1
fi

cleanup() {
    if [ "${CLEANED_UP}" -ne 0 ]; then
        return
    fi
    CLEANED_UP=1

    # Every child has its own process group.  Signal only those groups; never
    # use pkill, and never request /map_save from this evaluation launcher.
    local pid
    for pid in "${FASTLIO_PID}" "${TF_ODOM_PID}" "${LIO_TF_PID}"; do
        if [ "${pid}" -ne 0 ] && kill -0 "${pid}" 2>/dev/null; then
            kill -INT -- "-${pid}" 2>/dev/null || kill -INT "${pid}" 2>/dev/null || true
        fi
    done
    for pid in "${FASTLIO_PID}" "${TF_ODOM_PID}" "${LIO_TF_PID}"; do
        if [ "${pid}" -ne 0 ]; then
            wait "${pid}" 2>/dev/null || true
        fi
    done
    echo "[fastlio-baseline] FAST-LIO-only baseline stopped; no map was saved."
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM HUP

if [ ! -d "${TEST_NAV_WS}" ]; then
    echo "[fastlio-baseline] ERROR: test_nav workspace does not exist: ${TEST_NAV_WS}" >&2
    exit 1
fi
if [ ! -r "${FASTLIO_CONFIG}" ]; then
    echo "[fastlio-baseline] ERROR: FAST-LIO config is not readable: ${FASTLIO_CONFIG}" >&2
    exit 1
fi
if [ ! -r "${TEST_NAV_WS}/lio_tf_to_base_tf.py" ] || \
   [ ! -r "${TEST_NAV_WS}/tf_to_odom_bridge.py" ]; then
    echo "[fastlio-baseline] ERROR: required local TF bridge script is missing." >&2
    exit 1
fi

# A ROS 2 parameter YAML must have a node selector and ros__parameters.  This
# is only a cheap preflight check; rclcpp remains the authoritative parser.
if ! grep -qE '^[[:space:]]*/\*\*:[[:space:]]*$' "${FASTLIO_CONFIG}" || \
   ! grep -qE '^[[:space:]]*ros__parameters:[[:space:]]*$' "${FASTLIO_CONFIG}"; then
    echo "[fastlio-baseline] ERROR: not a recognizable ROS 2 parameter YAML: ${FASTLIO_CONFIG}" >&2
    exit 1
fi

mkdir -p "${ROS_LOG_DIR}"
export ROS_LOG_DIR

cd "${TEST_NAV_WS}"
source /opt/ros/humble/setup.bash
source "${FASTLIO_WS}/install/setup.bash"
source "${TEST_NAV_WS}/install/setup.bash"
set -u

if ! command -v ros2 >/dev/null; then
    echo "[fastlio-baseline] ERROR: ros2 is unavailable after sourcing the workspaces." >&2
    exit 1
fi
if ! ros2 pkg executables fast_lio 2>/dev/null | awk '$2 == "fastlio_mapping" { found=1 } END { exit !found }'; then
    echo "[fastlio-baseline] ERROR: fast_lio/fastlio_mapping is unavailable." >&2
    exit 1
fi

# Refuse obvious active conflicts but never stop someone else's nodes.  The
# shared lock covers localization.sh; this graph check also covers Nav2 or a
# manually launched static-map chain.
ACTIVE_NODES="$(timeout 3s ros2 node list 2>/dev/null || true)"
CONFLICT_NODES=(
    /static_map_localizer
    /static_map_tf_broadcaster
    /map_to_odom_bridge
    /controller_server
    /planner_server
    /bt_navigator
    /lifecycle_manager_navigation
    /lifecycle_manager_localization
)
for node in "${CONFLICT_NODES[@]}"; do
    if printf '%s\n' "${ACTIVE_NODES}" | grep -Fxq "${node}"; then
        echo "[fastlio-baseline] ERROR: conflicting ROS node is already active: ${node}" >&2
        echo "[fastlio-baseline] Stop NDT/Nav2/global-localization first; it was not changed." >&2
        exit 1
    fi
done

echo "[fastlio-baseline] FAST-LIO config: ${FASTLIO_CONFIG}"
echo "[fastlio-baseline] Raw FAST-LIO TF: ${RAW_TF_TOPIC} (camera_init -> body)"
echo "[fastlio-baseline] Published local chain: /tf odom -> base_link; ${ODOM_TOPIC}"
echo "[fastlio-baseline] Safety: NDT/Nav2/map->odom/map_save are not started; pcd_save.pcd_save_en=false is forced."

# Isolate FAST-LIO's raw TF before its chassis-frame correction.  The command
# line parameter intentionally overrides a true value in mid360.yaml, so this
# run cannot write a PCD even if the default mapping YAML requests it.
setsid python3 "${TEST_NAV_WS}/lio_tf_to_base_tf.py" \
    --ros-args \
    -p raw_tf_topic:="${RAW_TF_TOPIC}" \
    -p raw_parent_frame:=camera_init \
    -p raw_child_frame:=body \
    -p odom_frame:=odom \
    -p base_frame:=base_link \
    -p sensor_x:=0.37 -p sensor_y:=0.28 -p sensor_z:=0.15 \
    -p sensor_yaw:=0.8229227423153264 &
LIO_TF_PID=$!

setsid python3 "${TEST_NAV_WS}/tf_to_odom_bridge.py" \
    --ros-args \
    -p odom_frame:=odom \
    -p base_frame:=base_link \
    -p odom_topic:="${ODOM_TOPIC}" \
    -p publish_rate:=50.0 &
TF_ODOM_PID=$!

setsid ros2 run fast_lio fastlio_mapping \
    --ros-args \
    --params-file "${FASTLIO_CONFIG}" \
    -p pcd_save.pcd_save_en:=false \
    -r /tf:="${RAW_TF_TOPIC}" &
FASTLIO_PID=$!

wait "${FASTLIO_PID}"
