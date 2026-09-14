from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
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
            parameters=[{'frame_id': LaunchConfiguration('frame_id')}],
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
