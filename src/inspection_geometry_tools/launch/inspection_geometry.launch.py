import fcntl
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_LOCK_FD = None


def _acquire_single_instance_lock():
    """Fail before starting any node when an inspection launch already exists."""
    global _LOCK_FD
    if _LOCK_FD is not None:
        return
    _LOCK_FD = os.open('/tmp/test_nav_inspection_geometry.lock', os.O_CREAT | os.O_RDWR, 0o644)
    try:
        fcntl.flock(_LOCK_FD, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        os.close(_LOCK_FD)
        _LOCK_FD = None
        raise RuntimeError(
            'inspection_geometry_tools is already running; stop the existing '
            'inspection launch instead of starting a second copy.') from exc


def generate_launch_description():
    _acquire_single_instance_lock()
    return LaunchDescription([
        DeclareLaunchArgument('map_topic', default_value='/map'),
        DeclareLaunchArgument('aligned_map_topic', default_value='/aligned_map'),
        DeclareLaunchArgument('frame_id', default_value='map'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('occupied_threshold', default_value='50'),
        DeclareLaunchArgument('grid_resolution', default_value='0.10'),
        DeclareLaunchArgument('map_padding', default_value='1.00'),
        DeclareLaunchArgument('map_canvas_scale', default_value='2.00'),
        DeclareLaunchArgument('offset_distance', default_value='0.9'),
        DeclareLaunchArgument('path_spacing', default_value='0.10'),
        DeclareLaunchArgument('reverse_path', default_value='false'),
        DeclareLaunchArgument('close_radius', default_value='0.25'),
        DeclareLaunchArgument('experiment_label', default_value='mppi'),
        DeclareLaunchArgument('tracking_base_frame', default_value='base_link'),
        DeclareLaunchArgument('controller_id', default_value='InspectionStrict'),
        DeclareLaunchArgument('goal_checker_id', default_value='inspection_route_goal_checker'),
        DeclareLaunchArgument(
            'experiment_output_dir',
            default_value='/home/zkam/test_nav/robot_data/experiments'),
        Node(
            package='inspection_geometry_tools',
            executable='map_alignment_node',
            name='map_alignment_node',
            output='screen',
            remappings=[('/map', LaunchConfiguration('map_topic')),
                        ('/aligned_map', LaunchConfiguration('aligned_map_topic'))],
            parameters=[{
                'frame_id': LaunchConfiguration('frame_id'),
                'occupied_threshold': LaunchConfiguration('occupied_threshold'),
                'grid_resolution': LaunchConfiguration('grid_resolution'),
                'map_padding': LaunchConfiguration('map_padding'),
                'map_canvas_scale': LaunchConfiguration('map_canvas_scale'),
            }],
        ),
        Node(
            package='inspection_geometry_tools',
            executable='feature_snap_node',
            name='feature_snap_node',
            output='screen',
            parameters=[{'frame_id': LaunchConfiguration('frame_id')}],
        ),
        Node(
            package='inspection_geometry_tools',
            executable='polygon_editor_node',
            name='polygon_editor_node',
            output='screen',
            parameters=[{
                'frame_id': LaunchConfiguration('frame_id'),
                'close_radius': LaunchConfiguration('close_radius'),
            }],
        ),
        Node(
            package='inspection_geometry_tools',
            executable='polygon_offset_node',
            name='polygon_offset_node',
            output='screen',
            parameters=[{
                'frame_id': LaunchConfiguration('frame_id'),
                'offset_distance': LaunchConfiguration('offset_distance'),
                'path_spacing': LaunchConfiguration('path_spacing'),
                'reverse_path': LaunchConfiguration('reverse_path'),
            }],
        ),
        Node(
            package='inspection_geometry_tools',
            executable='inspection_path_executor_node',
            name='inspection_path_executor_node',
            output='screen',
            parameters=[{
                'frame_id': LaunchConfiguration('frame_id'),
                'experiment_label': LaunchConfiguration('experiment_label'),
                'experiment_output_dir': LaunchConfiguration('experiment_output_dir'),
                'tracking_base_frame': LaunchConfiguration('tracking_base_frame'),
                # Do not rely on Nav2 Humble's empty-string defaults here.
                'controller_id': LaunchConfiguration('controller_id'),
                'goal_checker_id': LaunchConfiguration('goal_checker_id'),
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='inspection_geometry_rviz',
            arguments=['-d',
                       '/home/zkam/test_nav/install/inspection_geometry_tools/share/'
                       'inspection_geometry_tools/rviz/inspection_geometry.rviz'],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ])
