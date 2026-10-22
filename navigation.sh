#!/bin/bash

cd /home/zkam/test_nav
source /opt/ros/humble/setup.bash
source install/setup.bash

# Only one Nav2 stack may own /map, /follow_path and cmd_vel_nav.
exec 9>/tmp/test_nav_navigation.lock
if ! flock -n 9; then
  echo "[navigation] ERROR: Nav2 is already running; do not start a second copy." >&2
  exit 1
fi

# Do not launch Nav2 against a half-started localization stack. Without the
# continuous odom -> base_link transform, Nav2's costmaps wait in activation
# and a later FollowPath goal looks accepted but can never produce cmd_vel.
NAV_TF_WAIT_SEC="${NAV_TF_WAIT_SEC:-15}"
if ! python3 - "${NAV_TF_WAIT_SEC}" <<'PY'
import sys
import time

import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

timeout = float(sys.argv[1])
rclpy.init()
node = rclpy.create_node('navigation_tf_preflight')
buffer = Buffer()
listener = TransformListener(buffer, node, spin_thread=False)
deadline = time.monotonic() + timeout
ready = False
while rclpy.ok() and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
    if buffer.can_transform('odom', 'base_link', Time(), timeout=Duration(seconds=0.0)):
        ready = True
        break
node.destroy_node()
rclpy.shutdown()
if not ready:
    print('[navigation] ERROR: localization TF odom -> base_link is unavailable.', file=sys.stderr)
    sys.exit(1)
PY
then
  echo "[navigation] Start ./localization.sh first; wait until it reports /odom, then retry." >&2
  exit 1
fi

# Fixed-PCD localization owns map -> odom.  Launch Nav2 navigation and the
# static map server only; do not start AMCL in parallel.
NAV_RUNTIME_PARAMS="$(mktemp /tmp/test_nav_navigation_ordered_XXXXXX.yaml)"
if ! python3 - robot_data/config/navigation.yaml "${NAV_RUNTIME_PARAMS}" <<'PY'
import copy
import sys
from pathlib import Path
import yaml

document=yaml.safe_load(Path(sys.argv[1]).read_text())
params=document['controller_server']['ros__parameters']
checker='inspection_route_goal_checker'
if checker not in params['goal_checker_plugins']:
    params['goal_checker_plugins'].append(checker)
settings=copy.deepcopy(params['goal_checker'])
settings.update(plugin='inspection_geometry_tools::OrderedRouteGoalChecker',route_progress_tolerance=.35)
params[checker]=settings
Path(sys.argv[2]).write_text(yaml.safe_dump(document,sort_keys=False))
PY
then
  rm -f -- "${NAV_RUNTIME_PARAMS}"
  exit 1
fi
trap 'rm -f -- "${NAV_RUNTIME_PARAMS}"' EXIT
ros2 launch static_map_localization static_nav2.launch.py \
    map:=/home/zkam/test_nav/robot_data/maps/map.yaml \
    params_file:="${NAV_RUNTIME_PARAMS}" \
    use_sim_time:=false \
    use_composition:=False
