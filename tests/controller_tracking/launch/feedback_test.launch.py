"""Experiment stack only. Starting this launch never arms the chassis relay."""
import json
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def build(context):
    session=Path(LaunchConfiguration('session').perform(context)).resolve()
    for f in ('config.yaml','nav2.yaml','frame.json','identity.json'):
        if not (session/f).is_file():raise RuntimeError('Incomplete frozen session: '+str(session/f))
    remap=[('/tf','/tracking/tf'),('/tf_static','/tracking/tf_static')]
    return [
        Node(package='controller_tracking',executable='tracking_feedback',arguments=['--session',str(session)],output='screen'),
        Node(package='controller_tracking',executable='tracking_gate',arguments=['--session',str(session)],output='screen'),
        Node(package='nav2_controller',executable='controller_server',name='controller_server',namespace='tracking',
             parameters=[str(session/'nav2.yaml')],remappings=remap+[('cmd_vel','/tracking/controller_cmd'),('speed_limit','/tracking/speed_limit')],output='screen'),
        Node(package='nav2_velocity_smoother',executable='velocity_smoother',name='velocity_smoother',namespace='tracking',
             parameters=[str(session/'nav2.yaml')],remappings=[('cmd_vel','/tracking/controller_cmd'),('cmd_vel_smoothed','/tracking/smoothed_cmd')],output='screen'),
        Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager',namespace='tracking',
             parameters=[{'autostart':True,'node_names':['controller_server','velocity_smoother']}],output='screen')]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('session',description='Absolute path to an immutable freeze output'),OpaqueFunction(function=build)])
