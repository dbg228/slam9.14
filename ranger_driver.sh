#!/usr/bin/env bash
set -euo pipefail

RANGER_WS="/home/zkam/ranger_ws"
CAN_IF="${RANGER_CAN_IF:-can0}"
PUBLISH_ODOM_TF="${RANGER_PUBLISH_ODOM_TF:-true}"
ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ranger_ros_logs}"

mkdir -p "${ROS_LOG_DIR}"

set +u
source /opt/ros/humble/setup.bash
source "${RANGER_WS}/install/setup.bash"
set -u

if ! ip link show "${CAN_IF}" >/dev/null 2>&1; then
    echo "CAN interface ${CAN_IF} not found."
    echo "Run first: ./ranger_can_setup.sh"
    exit 1
fi

ros2 launch ranger_bringup ranger_mini_v2.launch.py \
    port_name:="${CAN_IF}" \
    robot_model:=ranger_mini_v2 \
    publish_odom_tf:="${PUBLISH_ODOM_TF}"
