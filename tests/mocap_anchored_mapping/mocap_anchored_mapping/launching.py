"""Native maps and mocap-only initial localization; original launch files unchanged."""
import fcntl
import copy
import json
import os
from pathlib import Path
import time
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, EmitEvent, GroupAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node,SetRemap
from ament_index_python.packages import get_package_share_directory
from .cli import launch_guard,require_fixture_domain,_locks,graph_check
from .products import check_grid
from .isolation import native_prefix


def description(mode):
    return LaunchDescription([DeclareLaunchArgument('run',description='Absolute native-map run directory'),
        OpaqueFunction(function=lambda context:actions(Path(LaunchConfiguration('run').perform(context)),mode))])


def native_output_overrides(mode):
    # Localization needs native odometry, not repeated cloud/path rendering.
    # Keep its single executor available for measurement processing. Mapping
    # outputs and every estimator/input parameter remain unchanged.
    return [] if mode=='mapping' else [{
        'pcd_save.pcd_save_en':False,'publish.map_en':False,
        'publish.scan_publish_en':False,'publish.scan_bodyframe_pub_en':False,
        'publish.path_en':False}]


def navigation_status_ready(state,run,now_ns,maximum_age_s):
    if not state:return False
    stamp=state.get('native_stamp_ns',0)
    return bool(state.get('run_path')==str(run) and state.get('frozen') and
        state.get('fault') is None and not state.get('active_checkpoint') and
        stamp>0 and 0<=(now_ns-stamp)*1e-9<=maximum_age_s)


def ordered_navigation_parameters(document):
    """Add inspection completion checks without changing the frozen snapshot."""
    out=copy.deepcopy(document)
    params=out['controller_server']['ros__parameters']
    checker='inspection_route_goal_checker'
    if checker not in params['goal_checker_plugins']:
        params['goal_checker_plugins'].append(checker)
    settings=copy.deepcopy(params['goal_checker'])
    settings.update(plugin='inspection_geometry_tools::OrderedRouteGoalChecker',route_progress_tolerance=.35)
    params[checker]=settings
    return out


def navigation(run):
    cfg=check_grid(run);require_fixture_domain(cfg);domain=int(os.environ.get('ROS_DOMAIN_ID','0'))
    from .checkpoints import require_camera_world
    require_camera_world(cfg)
    lock=open('/tmp/test_nav_navigation.lock' if domain==0 else f'/tmp/test_nav_navigation_domain_{domain}.lock','a+')
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:lock.close();raise ValueError('Nav2 navigation is already running')
    _locks.append(lock);graph=graph_check('navigation',seconds=4.)
    if not {'native_base_bridge','anchored_map_server'}.issubset(graph['nodes']) or 'controller_server' in graph['nodes']:
        raise ValueError('Start this map mocap-initialized localization first; stop existing Nav2')
    import rclpy
    from std_msgs.msg import String
    from rclpy.qos import QoSProfile,DurabilityPolicy
    rclpy.init(args=[]);node=rclpy.create_node('anchored_navigation_run_check');status=[]
    node.create_subscription(String,'/anchored_localization/status',lambda m:status.append(json.loads(m.data)),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    # DDS may deliver the retained status before its live updates. Wait for a
    # qualifying update; retaining the original age limit rejects stale poses.
    until=time.monotonic()+5
    fresh=False
    while time.monotonic()<until:
        rclpy.spin_once(node,timeout_sec=.05)
        fresh=navigation_status_ready(status[-1] if status else None,run,
            node.get_clock().now().nanoseconds,cfg['initialization']['maximum_age_s'])
        if fresh:break
    node.destroy_node();rclpy.shutdown()
    if not fresh:
        raise ValueError('This map requires healthy frozen mocap initialization and fresh local LIO before navigation')
    # Frozen estimator/controller settings remain intact. The additional
    # checker is selected only by inspection routes, not ordinary Nav2 goals.
    import yaml
    from .core import checked_session
    capture=Path(status[-1]['capture_dir'])
    checked_session(run,capture)
    runtime=capture/'navigation_ordered_route.yaml'
    runtime.write_text(yaml.safe_dump(ordered_navigation_parameters(yaml.safe_load((run/'nav2.yaml').read_text())),sort_keys=False))
    return [GroupAction(actions=[SetRemap(src='velocity_smoother:cmd_vel_smoothed',dst='/nav/cmd_vel'),
        SetRemap(src='behavior_server:cmd_vel',dst='/cmd_vel_nav'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(get_package_share_directory('nav2_bringup'))/'launch/navigation_launch.py')),
            launch_arguments={'params_file':str(runtime),'use_sim_time':'false','use_composition':'False'}.items())])]


def actions(run,mode):
    run=run.resolve()
    if mode=='navigation':return navigation(run)
    cfg,_=launch_guard(run,mode);prefix='/anchored_mapping' if mode=='mapping' else '/anchored_localization'
    capture=run/'initialization' if mode=='mapping' else run/'localizations'/str(time.time_ns())
    if mode!='mapping':capture.mkdir(parents=True,exist_ok=False)
    remaps=[('/Odometry',prefix+'/native_odom'),('/tf',prefix+'/native_tf'),('/cloud_registered',prefix+'/cloud_F'),
        ('/cloud_registered_body',prefix+'/cloud_I'),('/cloud_effected',prefix+'/effected_F'),('/Laser_map',prefix+'/map_F'),
        ('/path',prefix+'/path_F'),('/map_save',prefix+'/save_native')]
    fast=Node(package='fast_lio',executable='fastlio_mapping',name='anchored_fastlio',output='screen',
        parameters=[str(run/'fastlio.yaml'),*native_output_overrides(mode)],remappings=remaps,
        prefix=native_prefix(run,mode,cfg,capture_root=capture))
    bridge=Node(package='mocap_anchored_mapping',executable='initial_anchor',name='initial_anchor' if mode=='mapping' else 'native_base_bridge',
        arguments=['--run',str(run),'--mode',mode,'--capture-dir',str(capture)],output='screen')
    processes=[fast,bridge]
    if mode in ('localization','checkpoints'):
        processes += [ExecuteProcess(cmd=['python3',str(Path(cfg['project_root'])/'tf_to_odom_bridge.py')],output='screen'),
            Node(package='nav2_map_server',executable='map_server',name='anchored_map_server',output='screen',parameters=[{'yaml_filename':str(run/'grid/map.yaml'),'use_sim_time':False}]),
            Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='anchored_map_lifecycle',output='screen',parameters=[{'autostart':True,'node_names':['anchored_map_server'],'use_sim_time':False}])]
    handlers=[RegisterEventHandler(OnProcessExit(target_action=p,on_exit=[EmitEvent(event=Shutdown(reason='Native-map core exited; no automatic restart or reanchoring'))])) for p in processes]
    return processes+handlers
