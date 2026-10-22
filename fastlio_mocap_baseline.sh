#!/usr/bin/env bash
# FAST-LIO-only trajectory baseline for passive Tracker0 comparison.
#
# This deliberately does NOT start static_map_localizer_node, Nav2, a
# map->odom broadcaster, or map saving.  It leaves the fixed PCD untouched.
# Start the Livox driver separately before running this wrapper.
set -eo pipefail

PROJECT_DIR="/home/zkam/test_nav"
FASTLIO_WS="/home/zkam/fastlio_ws"
FASTLIO_CONFIG="${FASTLIO_CONFIG:-${FASTLIO_WS}/src/FAST_LIO_ROS2/config/mid360.yaml}"

if [[ ! -f "${FASTLIO_CONFIG}" ]]; then
  echo "[fastlio-mocap-baseline] ERROR: FAST-LIO config is missing: ${FASTLIO_CONFIG}" >&2
  exit 2
fi

# Prevent accidentally launching a second copy of this deliberately isolated
# baseline.  This lock cannot detect a separately started localization stack,
# so the operator must stop localization.sh/mapping.sh first.
exec 9>/tmp/test_nav_fastlio_mocap_baseline.lock
if ! flock -n 9; then
  echo '[fastlio-mocap-baseline] ERROR: another baseline wrapper is already running.' >&2
  exit 1
fi

cd "${PROJECT_DIR}"
source /opt/ros/humble/setup.bash
source "${FASTLIO_WS}/install/setup.bash"
source "${PROJECT_DIR}/install/setup.bash"

LIO_BRIDGE_PID=0
ODOM_BRIDGE_PID=0
FASTLIO_PID=0

cleanup() {
  trap - EXIT INT TERM
  for pid in "${FASTLIO_PID}" "${ODOM_BRIDGE_PID}" "${LIO_BRIDGE_PID}"; do
    if [[ "${pid}" -ne 0 ]]; then
      # Each child starts in its own process group. ros2 run forks the real
      # binary; terminating only its wrapper used to leave FAST-LIO running.
      kill -TERM -- "-${pid}" 2>/dev/null || true
    fi
  done
  for pid in "${FASTLIO_PID}" "${ODOM_BRIDGE_PID}" "${LIO_BRIDGE_PID}"; do
    if [[ "${pid}" -ne 0 ]]; then
      # Do not leave an unresponsive native mapping process behind when its
      # launcher exits. Targets are only this wrapper's own process groups.
      for attempt in {1..20}; do
        if ! kill -0 -- "-${pid}" 2>/dev/null; then break; fi
        sleep 0.1
      done
      if kill -0 -- "-${pid}" 2>/dev/null; then
        kill -KILL -- "-${pid}" 2>/dev/null || true
      fi
      wait "${pid}" 2>/dev/null || true
    fi
  done
  echo '[fastlio-mocap-baseline] FAST-LIO-only baseline stopped; no map was saved.'
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

echo '[fastlio-mocap-baseline] FAST-LIO only: NDT, Nav2, map->odom, and map saving are disabled.'
echo "[fastlio-mocap-baseline] Config: ${FASTLIO_CONFIG}"
echo '[fastlio-mocap-baseline] Raw TF: /tf_lio_raw (camera_init->body); corrected chassis TF: /tf (odom->base_link).'
echo '[fastlio-mocap-baseline] Stop localization.sh and mapping.sh before continuing; do not run a second FAST-LIO chain.'

# FAST-LIO emits camera_init->body.  Keep it out of /tf and publish only the
# explicitly configured chassis-centre odom->base_link bridge on /tf.
setsid python3 "${PROJECT_DIR}/lio_tf_to_base_tf.py" --ros-args \
  -p raw_tf_topic:=/tf_lio_raw \
  -p raw_parent_frame:=camera_init \
  -p raw_child_frame:=body \
  -p odom_frame:=odom \
  -p base_frame:=base_link \
  -p sensor_x:=0.37 -p sensor_y:=0.28 -p sensor_z:=0.15 \
  -p sensor_yaw:=0.8229227423153264 &
LIO_BRIDGE_PID=$!

setsid python3 "${PROJECT_DIR}/tf_to_odom_bridge.py" --ros-args \
  -p odom_frame:=odom -p base_frame:=base_link \
  -p odom_topic:=/odom -p publish_rate:=50.0 &
ODOM_BRIDGE_PID=$!

# Override the YAML's map-saving default.  Do not use mapping.sh here: that
# wrapper requests /map_save on exit and is intended for building maps.
FASTLIO_EXECUTABLE="$(ros2 pkg prefix fast_lio)/lib/fast_lio/fastlio_mapping"
setsid "${FASTLIO_EXECUTABLE}" --ros-args \
  --params-file "${FASTLIO_CONFIG}" \
  -p pcd_save.pcd_save_en:=false \
  -r /tf:=/tf_lio_raw &
FASTLIO_PID=$!

wait "${FASTLIO_PID}"
