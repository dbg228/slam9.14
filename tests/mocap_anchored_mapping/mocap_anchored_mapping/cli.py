"""Explicit lifecycle commands. Never publishes a velocity or action goal."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import time
from .core import checked, checked_session, load, prepare, save, sha, update_products
from .products import check_grid, inspect, project, read_pcd


def graph_check(mode, seconds=1.5):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from tf2_msgs.msg import TFMessage
    own=not rclpy.ok()
    if own:rclpy.init(args=[])
    node=Node('anchored_workflow_preflight'); seen=[]
    def receive(msg):
        seen.extend((t.header.frame_id,t.child_frame_id) for t in msg.transforms if t.child_frame_id in ('map','odom','base_link','camera_init','body'))
    node.create_subscription(TFMessage,'/tf',receive,100)
    node.create_subscription(TFMessage,'/tf_static',receive,QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    until=time.monotonic()+seconds
    while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.05)
    names=sorted(set(n for n,_ in node.get_node_names_and_namespaces()))
    owners=[n for n in names if n in ('laserMapping','fastlio_mapping','fastlio_static_odom','static_map_localizer',
        'static_map_tf_broadcaster','lio_tf_to_base_tf','tf_to_odom_bridge','controller_server','map_server',
        'anchored_fastlio','initial_anchor','native_base_bridge','anchored_new_map_ndt','anchored_new_map_tf','anchored_map_server','anchored_map_lifecycle')]
    result=dict(mode=mode,nodes=names,conflicting_nodes=owners,observed_global_pose_tf=sorted(set(seen)),
        passed=not owners and not seen,rule='Stop old mapping/localization/navigation before switching; sensors/chassis/mocap may remain running')
    node.destroy_node()
    if own:rclpy.shutdown()
    return result


_locks=[]
def require_fixture_domain(cfg):
    if str(cfg['data_kind']).startswith('software_fixture') and not (os.environ.get('ROS_LOCALHOST_ONLY')=='1' and 180<=int(os.environ.get('ROS_DOMAIN_ID','0'))<=220):
        raise ValueError('Synthetic map launch is restricted to isolated localhost test domains 180..220')


def launch_guard(run,mode):
    """Shared by launch and CLI, so direct ros2 launch cannot skip preflight."""
    run=Path(run).resolve(); checked(run,mode!='mapping')
    from ament_index_python.packages import get_package_prefix
    cfg,_=checked(run,mode!='mapping')
    require_fixture_domain(cfg)
    from .checkpoints import require_camera_world
    require_camera_world(cfg)
    resolved_binary=Path(get_package_prefix('fast_lio'))/'lib/fast_lio/fastlio_mapping'
    if resolved_binary.resolve()!=Path(cfg['fastlio_binary']).resolve():
        raise ValueError('ROS overlay resolves a different FAST-LIO binary than the audited native source/config')
    if mode=='mapping':
        if (run/'initialization/raw.jsonl').exists() or (run/'raw/native_F.pcd').exists():
            raise ValueError('Run already started; use a new run (F resets when FAST-LIO restarts)')
    else:check_grid(run)
    lock=open(f'/tmp/test_nav_anchored_map_domain_{os.environ.get("ROS_DOMAIN_ID","0")}.lock','a+')
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:lock.close();raise ValueError('An anchored mapping/localization stack is already running')
    report=graph_check(mode)
    if not report['passed']:
        lock.close();raise ValueError(json.dumps(report,ensure_ascii=False))
    _locks.append(lock);return checked(run,mode!='mapping')


def correct_checkpoint(run,session,checkpoint):
    """Request a stationary correction in this exact running session; no motion."""
    import rclpy
    from std_srvs.srv import Trigger
    from std_msgs.msg import String
    from rclpy.qos import QoSProfile,DurabilityPolicy
    from .checkpoints import require_camera_world,service_name
    run=Path(run).resolve();session=Path(session).resolve()
    cfg,_=checked(run,True);require_camera_world(cfg)
    if not checked_session(run,session).get('checkpoint_corrections'):
        raise ValueError('Group 2 is initialization-only. Start the checkpoints localization mode for group 3.')
    rclpy.init(args=[]);node=rclpy.create_node('anchored_checkpoint_client')
    statuses=[]
    node.create_subscription(String,'/anchored_localization/status',lambda msg:statuses.append(json.loads(msg.data)),
        QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    try:
        client=node.create_client(Trigger,service_name(session,checkpoint))
        if not client.wait_for_service(timeout_sec=5):raise ValueError('No checkpoint service for this session')
        future=client.call_async(Trigger.Request());rclpy.spin_until_future_complete(node,future,timeout_sec=5)
        if not future.done() or future.result() is None:raise ValueError('Request outcome unknown; inspect live status before retrying')
        result=future.result()
        if not result.success:raise ValueError(result.message)
        accepted=json.loads(result.message);capture=Path(accepted['capture_dir']).resolve()
        if accepted['session']!=str(session) or capture.parent!=session/'checkpoints':raise ValueError('Wrong checkpoint session response')
        until=time.monotonic()+cfg['initialization']['timeout_s']+2
        while time.monotonic()<until:
            if (capture/'summary.lock.json').exists():
                value=load(capture/'summary.json')
                if sha(capture/'summary.json')!=load(capture/'summary.lock.json')['sha256']:raise ValueError('Checkpoint report integrity failed')
                if any(state.get('capture_dir')==str(session) and
                       state.get('last_checkpoint',{} ) and state['last_checkpoint'].get('capture_dir')==str(capture) and
                       state['last_checkpoint'].get('outcome')=='applied' for state in statuses):
                    return dict(outcome='applied',capture_dir=str(capture),report=value,
                        note='Correction acknowledged by this live localization session; no motion commands sent')
            if (capture/'failure.json').exists():raise ValueError(json.dumps(load(capture/'failure.json')))
            rclpy.spin_once(node,timeout_sec=.05)
        raise ValueError('No completed checkpoint report. Inspect live status; do not assume correction was applied.')
    finally:node.destroy_node();rclpy.shutdown()


def save_native(run):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from rcl_interfaces.srv import GetParameters
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    run=Path(run).resolve();cfg,_=checked(run,True);target=run/'raw/native_F.pcd'
    if target.exists() or (run/'raw/native_receipt.json').exists():raise ValueError('Native map already saved; no overwrite')
    rclpy.init(args=[]);node=Node('anchored_map_save_client');status=[]
    node.create_subscription(String,'/anchored_mapping/status',lambda m:status.append(json.loads(m.data)),
        QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def call(client,request):
        if not client.wait_for_service(timeout_sec=5):raise ValueError('Service unavailable:'+client.srv_name)
        future=client.call_async(request);rclpy.spin_until_future_complete(node,future,timeout_sec=40)
        if not future.done() or future.result() is None:raise ValueError('Service timeout; do not retry an uncertain save blindly')
        return future.result()
    try:
        client=node.create_client(GetParameters,'/anchored_fastlio/get_parameters')
        response=call(client,GetParameters.Request(names=['map_file_path','pcd_save.pcd_save_en']))
        path,enabled=response.values
        if Path(path.string_value).resolve()!=target or not enabled.bool_value:raise ValueError('Live FAST-LIO save parameters point outside this run')
        until=time.monotonic()+2
        while time.monotonic()<until and not status:rclpy.spin_once(node,timeout_sec=.05)
        if not status or status[-1]['run_path']!=str(run) or not status[-1]['frozen'] or status[-1]['fault']:
            raise ValueError('No healthy live frozen anchor for this run')
        result=call(node.create_client(Trigger,'/anchored_mapping/save_native'),Trigger.Request())
        if not result.success:raise ValueError('FAST-LIO save failed:'+result.message)
        if not target.is_file() or target.stat().st_size==0:raise ValueError('Service succeeded but no PCD written')
        header,data=read_pcd(target,cfg)
        if not len(data):raise ValueError('Empty native map')
        receipt=dict(run_id=run.name,data_kind=cfg['data_kind'],frame='map=native_F0',transform_count=0,path=str(target),sha256=sha(target),bytes=target.stat().st_size,
            points=len(data),service='/anchored_mapping/save_native',service_message=result.message,
            native_parameters_verified=True,calibration_sha256=sha(run/'calibration.yaml'),
            source='native FAST-LIO map_publish_callback accumulated world scans; no NDT, no loop-optimized merge')
        save(run/'raw/native_receipt.json',receipt);update_products(run);return receipt
    finally:node.destroy_node();rclpy.shutdown()


def main():
    parser=argparse.ArgumentParser(description=__doc__);sub=parser.add_subparsers(dest='command',required=True)
    p=sub.add_parser('prepare');p.add_argument('--run',required=True);p.add_argument('--site',required=True)
    for name in ('check','save','inspect','project','products'):
        p=sub.add_parser(name);p.add_argument('--run',required=True)
    p=sub.add_parser('validate');p.add_argument('--run',required=True)
    p.add_argument('--context',choices=['mapping','localization'],default='mapping');p.add_argument('--session')
    p=sub.add_parser('preflight');p.add_argument('--mode',choices=['mapping','localization'],default='mapping')
    p=sub.add_parser('correct');p.add_argument('--run',required=True);p.add_argument('--session',required=True)
    p.add_argument('--checkpoint',choices=['middle','end'],required=True)
    args=parser.parse_args()
    try:
        if args.command=='prepare':result=prepare(args.run,args.site)
        elif args.command=='preflight':
            result=graph_check(args.mode)
            print(json.dumps(result,indent=2,ensure_ascii=False));return 0 if result['passed'] else 2
        elif args.command=='check':
            cfg,ext=checked(args.run,(Path(args.run)/'calibration.yaml').exists())
            result=dict(inputs_unchanged=True,run_id=cfg['run_id'],M_axes_confirmed=cfg['M_from_mocap_world']['axes_confirmed'],
                clock_offset_verified=cfg['clock_offset_verified'],camera_latency_s=cfg['camera_latency_s'],
                map_basis=cfg['map_basis'],ndt_used=False,PCD_conversion_required=False,products=update_products(args.run))
        elif args.command=='validate':
            from .online import run_node
            return run_node(args.run,'validate',context=args.context,session=args.session)
        elif args.command=='correct':result=correct_checkpoint(args.run,args.session,args.checkpoint)
        else:result=dict(save=save_native,inspect=inspect,project=project,products=update_products)[args.command](args.run)
        print(json.dumps(result,ensure_ascii=False,indent=2));return 0
    except (ValueError,FileNotFoundError,FileExistsError,KeyError) as e:
        parser.exit(2,f'{args.command}: {e}\n')


if __name__=='__main__':raise SystemExit(main())
