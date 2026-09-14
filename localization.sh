#!/bin/bash

INIT_PID=0
RVIZ_PID=0
ODOM_TF_PID=0
TF_ODOM_PID=0
INIT_GATE_PID=0
MAP_ODOM_PID=0
LIO_TF_PID=0
RVIZ_CONFIG="/home/zkam/test_nav/robot_xj_project.rviz"
START_RVIZ="${LOCALIZATION_RVIZ:-false}"
START_ODOM_TF_BRIDGE="${LOCALIZATION_ODOM_TF_BRIDGE:-false}"
START_TF_ODOM_BRIDGE="${LOCALIZATION_TF_ODOM_BRIDGE:-true}"
# RViz 2D Pose Estimate is an operator-provided coarse pose.  A 2 m / 69 deg
# acceptance gate was too tight and rejected legitimate NDT refinements before
# they could reach lio_localization.  Keep the gate, but make its practical
# limits configurable and suitable for normal indoor manual initialization.
MAX_INIT_SHIFT_M="${LOCALIZATION_MAX_INIT_SHIFT_M:-5.0}"
MAX_INIT_YAW_RAD="${LOCALIZATION_MAX_INIT_YAW_RAD:-3.14159265359}"

# 清理后台进程
cleanup() {
    [ $INIT_PID -ne 0 ] && kill $INIT_PID 2>/dev/null || true
    [ $RVIZ_PID -ne 0 ] && kill $RVIZ_PID 2>/dev/null || true
    [ $ODOM_TF_PID -ne 0 ] && kill $ODOM_TF_PID 2>/dev/null || true
    [ $TF_ODOM_PID -ne 0 ] && kill $TF_ODOM_PID 2>/dev/null || true
    [ $INIT_GATE_PID -ne 0 ] && kill $INIT_GATE_PID 2>/dev/null || true
    [ $MAP_ODOM_PID -ne 0 ] && kill $MAP_ODOM_PID 2>/dev/null || true
    [ $LIO_TF_PID -ne 0 ] && kill $LIO_TF_PID 2>/dev/null || true
    pkill -f "localization init.launch.py" 2>/dev/null || true
    echo -e "\n进程已清理,脚本退出"
    exit 0
}

# 捕获Ctrl+C信号
trap cleanup SIGINT

cd /home/zkam/test_nav
source /opt/ros/humble/setup.bash
source /home/zkam/test_nav/install/setup.bash

echo "[localization] Starting guarded_initialization_gate.py"
python3 /home/zkam/test_nav/guarded_initialization_gate.py \
    --ros-args \
    -p user_initialpose_topic:=/initialpose_user \
    -p checked_initialpose_topic:=/initialpose_checked \
    -p candidate_topic:=/lidar_init_pose_candidate \
    -p accepted_topic:=/lidar_init_pose \
    -p max_candidate_translation_m:="${MAX_INIT_SHIFT_M}" \
    -p max_candidate_yaw_rad:="${MAX_INIT_YAW_RAD}" \
    -p max_abs_z_m:=3.0 &
INIT_GATE_PID=$!

if [ "${START_RVIZ}" = "true" ]; then
    if pgrep -af "rviz2.*robot_xj_project.rviz" >/dev/null 2>&1; then
        echo "[localization] RViz already running: ${RVIZ_CONFIG}"
    else
        echo "[localization] Starting RViz: ${RVIZ_CONFIG}"
        rviz2 -d "${RVIZ_CONFIG}" &
        RVIZ_PID=$!
    fi
fi

ros2 launch localization init.launch.py rviz:=false &
INIT_PID=$!
sleep 2

if [ "${START_ODOM_TF_BRIDGE}" = "true" ]; then
    echo "[localization] Starting optional odom_to_tf_bridge.py"
    python3 /home/zkam/test_nav/odom_to_tf_bridge.py \
        --ros-args \
        -p odom_topic:=/Odometry \
        -p parent_frame:=map \
        -p child_frame:=base_link \
        -p stamp_offset:=0.0 \
        -p publish_rate:=50.0 &
    ODOM_TF_PID=$!
else
    echo "[localization] odom_to_tf_bridge.py disabled; localization publishes TF directly"
fi

if [ "${START_TF_ODOM_BRIDGE}" = "true" ]; then
    echo "[localization] Starting tf_to_odom_bridge.py: TF odom->base_link -> /odom"
    python3 /home/zkam/test_nav/tf_to_odom_bridge.py \
        --ros-args \
        -p odom_frame:=odom \
        -p base_frame:=base_link \
        -p odom_topic:=/odom \
        -p publish_rate:=50.0 &
    TF_ODOM_PID=$!
else
    echo "[localization] tf_to_odom_bridge.py disabled"
fi

echo "[localization] Starting lio_tf_to_base_tf.py: /tf_lio_raw odom->sensor -> /tf odom->base_link"
python3 /home/zkam/test_nav/lio_tf_to_base_tf.py \
    --ros-args \
    -p raw_tf_topic:=/tf_lio_raw \
    -p raw_parent_frame:=odom \
    -p raw_child_frame:=base_link \
    -p odom_frame:=odom \
    -p base_frame:=base_link \
    -p sensor_x:=0.40 \
    -p sensor_y:=0.30 \
    -p sensor_z:=0.15 \
    -p sensor_yaw:=0.78539816340 &
LIO_TF_PID=$!

echo "[localization] Starting map_to_odom_bridge.py: /Odometry sensor pose + TF odom->base_link -> TF map->odom"
python3 /home/zkam/test_nav/map_to_odom_bridge.py \
    --ros-args \
    -p global_odom_topic:=/Odometry \
    -p map_frame:=map \
    -p odom_frame:=odom \
    -p base_frame:=base_link \
    -p global_pose_child_frame:=base_link \
    -p sensor_x:=0.40 \
    -p sensor_y:=0.30 \
    -p sensor_z:=0.15 \
    -p sensor_yaw:=0.78539816340 \
    -p publish_rate:=50.0 \
    -p max_global_xy_norm:=100.0 \
    -p max_abs_z:=5.0 &
MAP_ODOM_PID=$!

cd /home/zkam/test_nav
source /opt/ros/humble/setup.bash
source /home/zkam/test_nav/install/setup.bash
ros2 launch localization localization.launch.py rviz:=false

cleanup
