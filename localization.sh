#!/usr/bin/env bash
# Static-map localization stack.
# FAST-LIO supplies continuous local odometry; static_map_localizer_node loads
# the immutable PCD and publishes map -> odom after a RViz 2D pose seed and
# guarded NDT registration. This script never writes the PCD map.
# ROS 的 setup.bash 会读取一些可选的 AMENT_* 变量；在它完成前不能启用
# `set -u`，否则未定义的可选变量会让 shell 提前退出。
set -eo pipefail

# Exactly one FAST-LIO/static-localization chain may own /tf, /Odometry and
# map->odom. A second copy races the first one and makes Nav2 localization
# non-deterministic.
exec 9>/tmp/test_nav_static_localization.lock
if ! flock -n 9; then
  echo "[localization] ERROR: static localization is already running; do not start a second copy." >&2
  exit 1
fi

FASTLIO_PID=0
TF_ODOM_PID=0
LIO_TF_PID=0

# `map.pgm` is generated from this same FAST-LIO export.  Do not replace it
# with the legacy 3lou_scans.pcd: that file is from another session and does
# not share the map coordinate system.
STATIC_MAP_PCD="${STATIC_MAP_PCD:-/home/zkam/test_nav/robot_data/maps/data/scans.pcd}"
FASTLIO_CONFIG="${FASTLIO_CONFIG:-/home/zkam/fastlio_ws/src/FAST_LIO_ROS2/config/mid360.yaml}"

cleanup() {
  [ "${FASTLIO_PID}" -ne 0 ] && kill "${FASTLIO_PID}" 2>/dev/null || true
  [ "${TF_ODOM_PID}" -ne 0 ] && kill "${TF_ODOM_PID}" 2>/dev/null || true
  [ "${LIO_TF_PID}" -ne 0 ] && kill "${LIO_TF_PID}" 2>/dev/null || true
  echo -e "\n[localization] Static localization stopped."
}
trap cleanup EXIT INT TERM

cd /home/zkam/test_nav
source /opt/ros/humble/setup.bash
source /home/zkam/fastlio_ws/install/setup.bash
source /home/zkam/test_nav/install/setup.bash
set -u

if [ ! -f "${STATIC_MAP_PCD}" ]; then
  echo "[localization] ERROR: static map does not exist: ${STATIC_MAP_PCD}" >&2
  exit 1
fi
if [ ! -f "${FASTLIO_CONFIG}" ]; then
  echo "[localization] ERROR: FAST-LIO config does not exist: ${FASTLIO_CONFIG}" >&2
  exit 1
fi

echo "[localization] Static PCD: ${STATIC_MAP_PCD}"
echo "[localization] FAST-LIO local odometry config: ${FASTLIO_CONFIG}"
echo "[localization] Use RViz 2D Pose Estimate to seed fixed-map NDT matching."
echo "[localization] The PCD is read-only in this mode."

# Convert FAST-LIO camera_init->body into the chassis odom->base_link TF.
python3 /home/zkam/test_nav/lio_tf_to_base_tf.py \
  --ros-args \
  -p raw_tf_topic:=/tf_lio_raw \
  -p raw_parent_frame:=camera_init \
  -p raw_child_frame:=body \
  -p odom_frame:=odom \
  -p base_frame:=base_link \
  -p sensor_x:=0.37 -p sensor_y:=0.28 -p sensor_z:=0.15 \
  -p sensor_yaw:=0.8229227423153264 &
LIO_TF_PID=$!

python3 /home/zkam/test_nav/tf_to_odom_bridge.py \
  --ros-args \
  -p odom_frame:=odom \
  -p base_frame:=base_link \
  -p odom_topic:=/odom \
  -p publish_rate:=50.0 &
TF_ODOM_PID=$!

# Starts FAST-LIO with PCD saving disabled plus source-controlled static NDT.
ros2 launch static_map_localization static_fastlio_localization.launch.py \
  fastlio_config:="${FASTLIO_CONFIG}" \
  map_pcd_path:="${STATIC_MAP_PCD}" &
FASTLIO_PID=$!

wait "${FASTLIO_PID}"
