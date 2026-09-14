#!/bin/bash

cd /home/zkam/test_nav
source /opt/ros/humble/setup.bash
source install/setup.bash

# RobotModel / TF is published by OmniBot_ws:
#   ros2 launch omnibot_control robot_control.launch.py
#
# Do NOT start another robot_state_publisher here.  Duplicate
# robot_state_publisher nodes publish competing base_link->livox_frame TF,
# which makes RViz model direction drift and can make Nav2 costmaps drop
# scans.  This script only provides the /livox/lidar -> /scan fallback used
# by Nav2 obstacle layers.
python3 /home/zkam/test_nav/livox_to_scan_fallback.py
