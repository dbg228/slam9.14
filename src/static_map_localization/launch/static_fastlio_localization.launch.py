import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('static_map_localization')
    default_fastlio_config = '/home/zkam/fastlio_ws/src/FAST_LIO_ROS2/config/mid360.yaml'
    default_map = '/home/zkam/test_nav/robot_data/maps/data/scans.pcd'

    fastlio_config = LaunchConfiguration('fastlio_config')
    map_pcd_path = LaunchConfiguration('map_pcd_path')

    return LaunchDescription([
        DeclareLaunchArgument('fastlio_config', default_value=default_fastlio_config),
        DeclareLaunchArgument('map_pcd_path', default_value=default_map),
        # FAST-LIO remains the continuous local odometry source.  Its raw TF
        # is isolated so the bridge can publish the chassis odom->base_link.
        Node(
            package='fast_lio', executable='fastlio_mapping', name='fastlio_static_odom',
            parameters=[fastlio_config, {'pcd_save.pcd_save_en': False}],
            remappings=[('/tf', '/tf_lio_raw')], output='screen'),
        Node(
            package='static_map_localization', executable='static_map_localizer_node',
            name='static_map_localizer',
            parameters=[{'map_pcd_path': map_pcd_path}], output='screen'),
        Node(
            package='static_map_localization', executable='map_to_odom_tf_broadcaster.py',
            name='static_map_tf_broadcaster', output='screen'),
    ])
