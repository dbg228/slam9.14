#!/bin/bash
# Start the unified RViz with this workspace overlaid, so custom RViz panels
# (including Inspection Route) are discoverable through the ament index.
set -e

cd /home/zkam/test_nav
source /opt/ros/humble/setup.bash
source /home/zkam/test_nav/install/setup.bash
exec rviz2 -d /home/zkam/test_nav/robot_xj_project.rviz
