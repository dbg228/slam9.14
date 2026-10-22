"""Nav2 navigation stack for the fixed-PCD localization mode.

The static_map_localizer is the sole owner of map->odom.  This launch starts
only map_server and Nav2 navigation servers; it deliberately does not include
Nav2's localization_launch.py / AMCL.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    nav2_bringup = get_package_share_directory('nav2_bringup')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value='/home/zkam/test_nav/robot_data/maps/map.yaml'),
        DeclareLaunchArgument(
            'params_file', default_value='/home/zkam/test_nav/robot_data/config/navigation.yaml'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='nav2_map_server', executable='map_server', name='map_server',
            output='screen', parameters=[{
                'yaml_filename': map_yaml,
                'use_sim_time': use_sim_time,
            }]),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_map_server', output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server'],
            }]),
        GroupAction(actions=[
            # Override the ORIGINAL output name, since ROS remapping is not
            # chained: cmd_vel_smoothed -> cmd_vel -> /nav/cmd_vel won't work.
            # Keep controller output and smoother input on cmd_vel_nav.
            SetRemap(
                src='velocity_smoother:cmd_vel_smoothed', dst='/nav/cmd_vel'),
            # Recovery motions use the same smoothing and chassis input chain.
            SetRemap(src='behavior_server:cmd_vel', dst='/cmd_vel_nav'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [nav2_bringup, '/launch/navigation_launch.py']),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': params_file,
                    'use_composition': 'False',
                }.items()),
        ]),
    ])
