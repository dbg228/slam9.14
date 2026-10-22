"""Prepare, record and supervise one 2 m trial. Default is read-only preflight."""
import argparse
import hashlib
from dataclasses import asdict
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from control_msgs.msg import DynamicJointState
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy

from .core import MotionConfig, load_mount, base_pose, pose_dict
from .chassis_evidence import current_chassis_evidence
from .qos import latest_value_qos


class Supervisor(Node):
    def __init__(self):
        super().__init__('localization_ab_supervisor')
        self.mocap=[]
        self.state=None
        self.state_receive=None
        self.guard_failure=''
        self.homing={}
        self.homing_receive=None
        self.local_pose=None
        self.local_receive=None
        self.ndt_ready=False
        self.ndt_diagnostic_receive=None
        self.permit=self.create_publisher(Bool,'/localization_ab/recorder_ready',latest_value_qos())
        self.create_subscription(PoseStamped,'/mocap/Tracker0/pose',self.receive_mocap,qos_profile_sensor_data)
        self.create_subscription(String,'/localization_ab/state',self.receive_state,latest_value_qos(10))
        self.create_subscription(String,'/localization_ab/guard_status',self.receive_guard,latest_value_qos(10))
        self.create_subscription(DynamicJointState,'/dynamic_joint_states',self.receive_joints,qos_profile_sensor_data)
        self.create_subscription(TFMessage,'/tf',self.receive_tf,50)
        self.create_subscription(Bool,'/static_localization/ready',self.receive_ready,
                                 QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        from static_map_localization.msg import NdtMatchDiagnosticDetail
        self.create_subscription(NdtMatchDiagnosticDetail,'/static_localization/ndt_match_diagnostic_detail',
                                 self.receive_ndt,20)

    def receive_ready(self,msg):self.ndt_ready=msg.data

    def receive_ndt(self,msg):self.ndt_diagnostic_receive=time.monotonic()

    def receive_tf(self,msg):
        for tf in msg.transforms:
            if tf.header.frame_id=='odom' and tf.child_frame_id=='base_link':
                self.local_pose=tf
                self.local_receive=time.monotonic()

    def receive_joints(self,msg):
        for name,values in zip(msg.joint_names,msg.interface_values):
            if 'homing_done' in values.interface_names:
                self.homing[name]=values.values[values.interface_names.index('homing_done')]
        self.homing_receive=time.monotonic()

    def receive_mocap(self,msg):
        self.mocap.append((time.monotonic(),msg))
        self.mocap=self.mocap[-200:]

    def receive_state(self,msg):
        self.state=json.loads(msg.data)
        self.state_receive=time.monotonic()

    def receive_guard(self,msg):
        value=json.loads(msg.data)
        if value.get('latched_failure'):self.guard_failure=value['latched_failure']

    def wait(self,duration):
        end=time.monotonic()+duration
        while time.monotonic()<end:rclpy.spin_once(self,timeout_sec=.05)

    def watchdog(self):
        client=self.create_client(GetParameters,'/arbitrator_controller/get_parameters')
        if not client.wait_for_service(timeout_sec=2.):return None
        req=GetParameters.Request();req.names=['external_watchdog_timeout']
        future=client.call_async(req)
        end=time.monotonic()+3
        while not future.done() and time.monotonic()<end:rclpy.spin_once(self,timeout_sec=.05)
        if not future.done() or not future.result().values:return None
        return future.result().values[0].double_value


def await_initial_local_tf(node):
    started=time.monotonic()
    node.wait(3.)
    # Discovery/acquisition is not the running liveness watchdog. A freshly
    # created DDS participant can take >3 s to discover an existing TF writer.
    # Remain disarmed for at most seven additional seconds; freshness checks
    # below and all armed watchdog deadlines stay exactly unchanged.
    discovery_end=time.monotonic()+7.
    while (node.local_receive is None or time.monotonic()-node.local_receive>.3) and time.monotonic()<discovery_end:
        node.wait(.1)
    return time.monotonic()-started


def preflight(node,args):
    acquisition_wait=await_initial_local_tf(node)
    nodes=node.get_node_names()
    checks={'initial_tf_acquisition_wait_s':acquisition_wait}
    checks['nav2_absent']=not any(n in nodes for n in ('controller_server','velocity_smoother','behavior_server'))
    checks['mocap_messages_received']=len(node.mocap)
    checks['mocap_fresh']=bool(node.mocap and time.monotonic()-node.mocap[-1][0]<.2)
    if node.mocap:
        m=node.mocap[-1][1]
        checks['mocap_frame']=m.header.frame_id
        checks['mocap_header_age_s']=(node.get_clock().now().nanoseconds-(m.header.stamp.sec*10**9+m.header.stamp.nanosec))/1e9
        checks['mocap_base_pose']=base_pose(pose_dict(m.header.stamp,m.pose),load_mount(args.mount))
    checks['fastlio_present']='laser_mapping' in nodes or 'fastlio_static_odom' in nodes
    checks['fastlio_node_count']=sum(n in ('laser_mapping','fastlio_static_odom') for n in nodes)
    checks['cmd_publishers']=[p.node_name for p in node.get_publishers_info_by_topic('/nav/cmd_vel')]
    checks['mode_publishers']=[p.node_name for p in node.get_publishers_info_by_topic('/teleop/mode')]
    checks['cmd_subscribers']=[p.node_name for p in node.get_subscriptions_info_by_topic('/nav/cmd_vel')]
    checks['external_watchdog_timeout_s']=node.watchdog()
    checks['wheel_homing_done']=node.homing
    checks['all_wheels_homed']=bool(node.homing_receive and time.monotonic()-node.homing_receive<.3 and
                                   all(math.isfinite(node.homing.get(f'wheel_{i}_turn',math.nan)) and
                                       node.homing.get(f'wheel_{i}_turn',0.)>.5 for i in range(1,5)))
    checks['chassis_driver_evidence']=current_chassis_evidence()
    checks['chassis_states_fresh']=bool(node.homing_receive and time.monotonic()-node.homing_receive<.3)
    checks['chassis_homing_verified']=checks['chassis_states_fresh'] and (
        checks['all_wheels_homed'] or checks['chassis_driver_evidence']['ready'])
    checks['local_pose_finite_fresh']=False
    checks['local_pose_receive_age_s']=time.monotonic()-node.local_receive if node.local_receive else None
    if node.local_pose:
        p=node.local_pose.transform.translation
        q=node.local_pose.transform.rotation
        stamp=node.local_pose.header.stamp
        age=(node.get_clock().now().nanoseconds-(stamp.sec*10**9+stamp.nanosec))/1e9
        checks['local_pose_header_age_s']=age
        checks['local_xy_norm_m']=math.hypot(p.x,p.y)
        checks['local_pose_finite_fresh']=bool(node.local_receive and time.monotonic()-node.local_receive<.3 and
            -.05<=age<=.5 and all(math.isfinite(v) for v in (p.x,p.y,p.z,q.x,q.y,q.z,q.w)) and
            math.hypot(q.x,q.y,q.z,q.w)>.5 and checks['local_xy_norm_m']<100.)
    checks['ndt_present']='static_map_localizer' in nodes
    checks['ndt_initialized_ready']=node.ndt_ready
    checks['ndt_diagnostic_fresh']=bool(node.ndt_diagnostic_receive and
                                      time.monotonic()-node.ndt_diagnostic_receive<6.)
    # A DDS discovery failure must not hide a second local FAST-LIO process.
    fastlio_pids=[]
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():continue
        try:
            executable=(entry/'cmdline').read_bytes().split(b'\0')[0]
        except OSError:continue
        if executable.endswith(b'/fast_lio/fastlio_mapping'):fastlio_pids.append(int(entry.name))
    checks['local_fastlio_processes']=fastlio_pids
    checks['free_disk_bytes']=shutil.disk_usage(args.project).free
    reasons=[]
    if not checks['nav2_absent']:reasons.append('Nav2 is running')
    if not checks['mocap_fresh']:reasons.append('No fresh mocap pose')
    if checks.get('mocap_frame')!='mocap_world':reasons.append('mocap frame is not mocap_world')
    if not -.05<=checks.get('mocap_header_age_s',math.inf)<=.25:reasons.append('mocap timestamp age invalid')
    if not checks['fastlio_present']:reasons.append('FAST-LIO absent')
    if checks['fastlio_node_count']!=1:reasons.append('Exactly one FAST-LIO node is required')
    if len(checks['local_fastlio_processes'])!=1:reasons.append('Exactly one local native FAST-LIO process required')
    if 'arbitrator_controller' not in checks['cmd_subscribers']:reasons.append('Chassis cmd subscriber absent')
    if checks['cmd_publishers']:reasons.append('Other chassis velocity publisher exists')
    if checks['mode_publishers']:reasons.append('Other mode publisher exists; teleop must be stopped for autonomous test')
    wd=checks['external_watchdog_timeout_s']
    if wd is None or not 0.<wd<=.5:reasons.append('Chassis watchdog must be configured <=0.5 s before controller configure')
    if not checks['chassis_homing_verified']:reasons.append('No completed current driver homing/velocity-restore evidence')
    if not checks['local_pose_finite_fresh']:reasons.append('Local odometry is missing, stale, invalid or diverged (>100m)')
    if checks['ndt_present'] and args.group!='B':reasons.append('NDT must be stopped for A/C trials')
    if args.group=='B' and not checks['ndt_present']:reasons.append('Start NDT and independently initialize map pose for B')
    if args.group=='B' and not checks['ndt_initialized_ready']:reasons.append('NDT has not accepted initialization')
    if args.group=='B' and not checks['ndt_diagnostic_fresh']:reasons.append('NDT diagnostic stream missing or stale')
    if checks['free_disk_bytes']<1024**3:reasons.append('Less than 1 GiB free storage')
    checks['ready_to_arm']=not reasons
    checks['blocking_reasons']=reasons
    return checks


class Recorder:
    def __init__(self,output,topics):
        self.ready=threading.Event()
        self.error=''
        self.log_path=output/'recorder.log'
        self.process=subprocess.Popen(['ros2','bag','record','-o',str(output/'bag'),*topics],
                                      stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,
                                      start_new_session=True,env={**os.environ,'RCUTILS_COLORIZED_OUTPUT':'0'})
        self.thread=threading.Thread(target=self.read,daemon=True);self.thread.start()

    def read(self):
        with self.log_path.open('w',encoding='utf8') as log:
            for line in self.process.stdout:
                log.write(line);log.flush()
                if 'All requested topics are subscribed' in line:self.ready.set()
                if '[ERROR]' in line or '[FATAL]' in line:self.error=line.strip()

    def close(self):
        if self.process.poll() is None:
            os.killpg(self.process.pid,signal.SIGINT)
            try:self.process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                # Keep original data; report recorder trouble, do not erase bag.
                raise RuntimeError('Recorder did not finish writing within 15 seconds')
        self.thread.join(timeout=1)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--group',choices=list('ABC'),required=True)
    parser.add_argument('--direction',choices=('forward','reverse'),default='forward')
    parser.add_argument('--project',type=Path,default=Path('/home/zkam/test_nav'))
    parser.add_argument('--mount',default='/home/zkam/test_nav/robot_data/config/mocap_tracker0.yaml')
    parser.add_argument('--output',type=Path)
    parser.add_argument('--execute',action='store_true',help='Actually arm and drive; default only inspects live state')
    parser.add_argument('--route-clear',action='store_true',help='Onsite operator has confirmed 2.5 m clear mocap-covered lane and reachable physical stop')
    parser.add_argument('--mocap-offset',type=float,default=0.)
    parser.add_argument('--timing-verified',action='store_true')
    args=parser.parse_args()
    load_mount(args.mount)
    if not math.isfinite(args.mocap_offset):parser.error('mocap offset must be finite')
    output=args.output or args.project/'robot_data/experiments/localization_ab'/(
        datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+args.group)
    if output.exists():parser.error('output directory already exists; original trials are never overwritten')
    output.mkdir(parents=True)
    rclpy.init();node=Supervisor()
    children=[];recorder=None;logs=[]
    manifest=dict(schema_version=1,group=args.group,execute_requested=args.execute,
                  onsite_route_clear_confirmed=args.route_clear,
                  direction=args.direction,
                  protocol=asdict(MotionConfig(direction=1 if args.direction=='forward' else -1)),mocap_offset_s=args.mocap_offset,
                  timing_verified=args.timing_verified,mount=load_mount(args.mount),
                  independent_reference=args.group!='C',status='preflight',
                  control=f'constant {0.15 if args.direction=="forward" else -0.15:+.2f} m/s body x; zero yaw; common mocap displacement >=2 m stop',
                  endpoint_note='2 m is zero-command trigger; braking overshoot is measured separately')
    manifest['common_controller_sha256']=hashlib.sha256((Path(__file__).parent/'core.py').read_bytes()).hexdigest()
    manifest['control_transport']={
        'private_realtime_topics':'BEST_EFFORT / KEEP_LAST; watchdog limits unchanged',
        'chassis_and_events':'original reliable delivery',
        'nodes_sha256':hashlib.sha256((Path(__file__).parent/'nodes.py').read_bytes()).hexdigest(),
        'qos_sha256':hashlib.sha256((Path(__file__).parent/'qos.py').read_bytes()).hexdigest()}
    def save():
        (output/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    def start(module,arguments):
        stream=(output/(module+'.log')).open('w',encoding='utf8');logs.append(stream)
        p=subprocess.Popen([sys.executable,'-u','-m','localization_ab_test.'+module,*arguments],
                           stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        children.append(p);return p
    # Console entry points for nodes share one module, select via a tiny -c call.
    def start_node(function,arguments):
        stream=(output/(function+'.log')).open('w',encoding='utf8');logs.append(stream)
        code='from localization_ab_test.nodes import '+function+'; '+function+'()'
        p=subprocess.Popen([sys.executable,'-u','-c',code,*arguments],stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        children.append(p);return p
    try:
        manifest['preflight']=preflight(node,args);save()
        print(json.dumps(manifest['preflight'],ensure_ascii=False,indent=2),flush=True)
        if not args.execute:
            manifest['status']='read_only_preflight';save();return
        if not args.route_clear:raise RuntimeError('Onsite clear route and physical stop confirmation required (--route-clear)')
        if not manifest['preflight']['ready_to_arm']:raise RuntimeError('Preflight blocked: '+'; '.join(manifest['preflight']['blocking_reasons']))
        source_args=['--groups',args.group,'--mount',args.mount,'--mocap-offset',str(args.mocap_offset)]
        if args.timing_verified:source_args.append('--timing-verified')
        start('sources',source_args)
        guard=start_node('guard_main',['--arm','--direction',args.direction])
        gate=start_node('motion_main',['--group',args.group,'--mount',args.mount,'--direction',args.direction])
        topics=['/mocap/Tracker0/pose','/vrpn_mocap/Tracker0/pose','/mocap/Tracker0/diagnostics',
                '/tf','/tf_lio_raw','/tf_static','/odom','/cloud_registered',
                '/nav/cmd_vel','/teleop/mode','/reference_velocity',
                '/localization_ab/estimate/'+args.group,'/localization_ab/request',
                '/localization_ab/state','/localization_ab/events','/localization_ab/guard_status',
                '/localization_ab/recorder_ready']
        if args.group=='C':topics.append('/localization_ab/correction')
        if args.group=='B':topics.extend(['/static_localization/ndt_match_diagnostic_detail',
                                         '/static_localization/map_to_odom','/static_localization/ready'])
        recorder=Recorder(output,topics)
        end=time.monotonic()+10
        while not recorder.ready.is_set() and time.monotonic()<end:
            if recorder.process.poll() is not None:raise RuntimeError('Recorder exited before ready')
            rclpy.spin_once(node,timeout_sec=.05)
        if not recorder.ready.is_set():raise RuntimeError('Required topics not all subscribed within 10 seconds')
        manifest['status']='running';manifest['recorded_topics']=topics;save()
        end=time.monotonic()+50
        while time.monotonic()<end:
            node.permit.publish(Bool(data=True))
            until=time.monotonic()+.08
            while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.01)
            if recorder.process.poll() is not None:raise RuntimeError('Recorder died during trial')
            if recorder.error:raise RuntimeError('Recorder error: '+recorder.error)
            if guard.poll() is not None:raise RuntimeError('Independent velocity guard died')
            if node.guard_failure:raise RuntimeError('Velocity guard aborted: '+node.guard_failure)
            if node.state and node.state['state'] in ('DONE','ABORTED'):
                manifest['final_state']=node.state
                manifest['status']='completed' if node.state['state']=='DONE' else 'aborted'
                break
            if gate.poll() is not None and (not node.state or node.state['state'] not in ('DONE','ABORTED')):
                raise RuntimeError('Motion process exited unexpectedly')
            if node.state_receive and time.monotonic()-node.state_receive>.3:raise RuntimeError('Motion heartbeat lost')
        else:raise RuntimeError('Supervisor hard timeout')
    except (Exception,KeyboardInterrupt) as error:
        manifest['status']='aborted';manifest['error']=str(error) or 'interrupted'
        print('ABORT: '+manifest['error'],file=sys.stderr,flush=True)
    finally:
        if args.execute and children:
            # False permit makes the separate relay latch STOP immediately.
            for _ in range(5):
                node.permit.publish(Bool(data=False));node.wait(.05)
            # Keep recording a 5-second zero-command tail on failure as well.
            if manifest['status']!='completed' and recorder and recorder.process.poll() is None:node.wait(5.)
        for p in reversed(children):
            if p.poll() is None:
                os.killpg(p.pid,signal.SIGINT)
                try:p.wait(timeout=3)
                except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM)
        if recorder:
            try:recorder.close()
            except Exception as error:manifest['recorder_error']=str(error)
        save()
        for stream in logs:stream.close()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    print('Trial manifest: '+str(output/'manifest.json'),flush=True)
    if manifest['status']=='aborted':raise SystemExit(2)


if __name__=='__main__':main()
