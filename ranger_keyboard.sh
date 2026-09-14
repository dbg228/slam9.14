#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/humble/setup.bash
source /home/zkam/ranger_ws/install/setup.bash
set -u

ros2 run teleop_twist_keyboard teleop_twist_keyboard
