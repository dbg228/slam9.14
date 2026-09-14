# Inspection geometry tools

First-stage ROS 2 package for map alignment and interactive inspection-area
selection.  Every output uses `frame_id=map` by default.

## Build and start

```bash
cd /home/zkam/test_nav
source /opt/ros/humble/setup.bash
colcon build --packages-select inspection_geometry_tools --symlink-install
source install/setup.bash
ros2 launch inspection_geometry_tools inspection_geometry.launch.py
```

The input map is `/map`. By default, `/aligned_map` is an **unchanged-coordinate
copy** of `/map`, so the manual polygon, offset path, localization, and Nav2
all use exactly the same physical `map` coordinates. The main wall direction
is still detected and shown with the aligned 10 cm RViz grid. This is the safe
mode for real robot navigation.

`rotate_occupancy_grid:=true` is retained only for offline visualization. It
resamples/crops the map and must not be used with the existing Nav2 setup,
because Nav2 still plans on the original `/map` coordinate system.
`/alignment_grid` is an RViz `MarkerArray`: its fine cells are **0.10 m x
0.10 m** and its heavier lines occur every metre. The node also publishes
`/theta_map` and `/theta_correction` as `std_msgs/msg/Float64` in radians.

In the RViz window, select **Publish Point** and click the map. The clicked
coordinate is used directly as the polygon vertex; no obstacle/edge snapping
or coordinate correction is performed. The polygon editor adds vertices in
click order.

Close, undo, or clear the current editor state with:

```bash
ros2 service call /inspection_polygon/close std_srvs/srv/Trigger '{}'
ros2 service call /inspection_polygon/undo std_srvs/srv/Trigger '{}'
ros2 service call /inspection_polygon/reset std_srvs/srv/Trigger '{}'
```

In normal RViz use, no terminal service call is needed: after placing at least
three vertices, click P1 (within `close_radius`, default 0.25 m) again. The
editor closes the loop, publishes `/inspection_polygon`, and generates the
offset path automatically.

`undo` removes the most recent click, its snapped feature, and its polygon
vertex together. `reset` clears all of them.

After a successful close, `/inspection_polygon` publishes a
`geometry_msgs/msg/PolygonStamped`; `/inspection_markers` renders labels,
edges, and the closed loop.

`polygon_offset_node` subscribes to the closed polygon and uses a geometric
buffer, not proportional scaling. Its default `offset_distance:=1.00` creates
an outward 1 m inspection loop. It publishes `/inspection_offset_polygon`, a
smoothly sampled `/inspection_path` (`nav_msgs/msg/Path`), and
`/inspection_path_markers`. Each Path pose contains a tangent yaw and RViz
shows a purple direction arrow every metre. `path_spacing` defaults to 0.10 m.

## Nav2 MPPI FollowPath execution

`inspection_path_executor_node` sends the chosen inspection-path segment
directly to Nav2's `/follow_path` action. This uses the configured MPPI
controller and does not invoke the global planner to replace the inspection
route.

RViz is preconfigured with separate **Mark Route START**, **Mark Route
MIDDLE**, and **Mark Route END** Publish Point tools. Select the named tool,
then click the purple route; no Topic-property editing is required. Their
topics are:

```text
/inspection_route_start
/inspection_route_middle   # optional
/inspection_route_end
```

Each click is projected to its nearest sampled route pose and shown as a
START/MIDDLE/END marker. START includes an arrow showing the purple path's
tangent direction. Execute the route with:

```bash
ros2 service call /inspection_route/follow std_srvs/srv/Trigger '{}'
```

The service first sends Nav2 `NavigateToPose` to START using that tangent yaw.
After Nav2 confirms arrival, it automatically sends the selected segment to
`FollowPath`; do not send a separate ordinary Nav2 Goal to START.

The selected purple segment uses the `InspectionStrict` MPPI controller profile.
It deliberately omits MPPI's obstacle `CostCritic` and strongly follows the
published path orientation. Normal `NavigateToPose` motion to START continues
to use the ordinary obstacle-avoiding controller. Before starting a strict
inspection segment, the operator must verify that the full robot footprint
along the purple line is clear.

The unified `robot_xj_project.rviz` also has an **Inspection Route** panel.
After selecting START and END, use **保存 START/END** to persist the selection
in `robot_data/inspection_route_selection.txt`; **载入已保存** restores it after
RViz or the route node restarts. The panel shows the straight-line distance to
START and the sampled purple-line distance from START to END. During execution
it switches to the actual Nav2 feedback: remaining distance to START, then
remaining distance to END while `FollowPath` is running.

Cancel the active MPPI path controller action with:

```bash
ros2 service call /inspection_route/cancel std_srvs/srv/Trigger '{}'
```

## Parameters

`map_alignment_node`: `alignment_method` (currently `pca`),
`occupied_threshold`, `min_line_length`, `grid_resolution` (default `0.10`),
`map_padding` (default `1.00` m), `rotate_occupancy_grid` (default `false`; keep
false for Nav2),
`map_canvas_scale` (default `2.00`), `frame_id`.

`feature_snap_node`: `frame_id` only. It is a compatibility forwarding node:
it republishes `/clicked_point` directly to `/snapped_feature_point` without
any feature snapping.

`polygon_offset_node`: `offset_distance` (default `1.00` m), `path_spacing`
(default `0.10` m), `direction_marker_spacing` (default `1.00` m),
`corner_points`, `reverse_path`, `frame_id`.

`inspection_path_executor_node`: `follow_path_action` (default
`/follow_path`), `controller_id`, `goal_checker_id`, `frame_id`.

`polygon_editor_node`: `close_radius` (default `0.25` m), `frame_id`.

The alignment estimator uses local boundary PCA and a fourth-angle vote, so
perpendicular walls are treated as one coordinate frame rather than cancelling
each other. The RViz grid is aligned to that feature direction, while points
are forwarded unchanged to the polygon editor in the original Nav2 `map`
coordinates.
