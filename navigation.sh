#!/bin/bash

cd /home/zkam/test_nav
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_navigation robot_navigation.launch.py \
    map:=/home/zkam/test_nav/robot_data/maps/map.yaml \
    params_file:=/home/zkam/test_nav/robot_data/config/navigation.yaml \
    use_sim_time:=false \
    use_composition:=False
