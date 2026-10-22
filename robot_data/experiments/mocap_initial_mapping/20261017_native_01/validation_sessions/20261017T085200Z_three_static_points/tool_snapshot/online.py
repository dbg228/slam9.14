"""Static mocap calibration / restart initialization. No motion or NDT inputs."""
import argparse
from collections import deque
import json
from pathlib import Path
import signal
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist, TransformStamped
from tf2_msgs.msg import TFMessage
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from rosidl_runtime_py.convert import message_to_ordereddict
from controller_tracking.feedback import PairedMocap, ns
from controller_tracking.world_capture import check_raw_normalized, clean, read_pose
from .core import (checked, estimate, static_window, inverse, initial_map_odom, validation_errors,
                   checked_session, load, matrix, mocap_base, native_base, record, save, sha, update_products)


def pose_matrix(p):
    return matrix([p.position.x,p.position.y,p.position.z],[p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w])


def tf_message(h, stamp, parent, child):
    msg=TransformStamped();msg.header.stamp=stamp;msg.header.frame_id=parent;msg.child_frame_id=child
    r=record(h,parent,child);t=msg.transform.translation;q=msg.transform.rotation
    t.x,t.y,t.z=r['translation_m'];q.x,q.y,q.z,q.w=r['quaternion_xyzw'];return msg


class NativeBridge(Node):
    def __init__(self,run,mode,capture_dir=None,context='mapping',session=None):
        super().__init__('calibration_validation' if mode=='validate' else ('initial_anchor' if mode=='mapping' else 'native_base_bridge'))
        self.run=Path(run).resolve();self.mode=mode;self.phase=context if mode=='validate' else mode
        self.cfg,self.ext=checked(self.run,mode!='mapping')
        from .cli import require_fixture_domain
        require_fixture_domain(self.cfg)
        if self.get_parameter('use_sim_time').value:raise ValueError('This acquisition profile requires the live ROS clock')
        self.prefix='/anchored_mapping' if self.phase=='mapping' else '/anchored_localization'
        if mode=='mapping':
            self.capture=self.run/'initialization'
            if (self.run/'calibration.yaml').exists() or (self.capture/'raw.jsonl').exists():raise ValueError('Native mapping cannot resume after restart; prepare a new run')
        else:
            parent=self.run/('validation' if mode=='validate' else 'localizations')
            self.capture=Path(capture_dir).resolve() if capture_dir else parent/str(time.time_ns())
            if self.capture.parent!=parent:raise ValueError('Capture directory must be an immediate child of '+str(parent))
            self.capture.mkdir(parents=True,exist_ok=True)
        self.calibration=load(self.run/'calibration.yaml') if mode!='mapping' else None
        self.fixed_map_odom=np.eye(4);self.session=Path(session).resolve() if session else None
        if mode=='validate' and self.phase=='localization':
            if self.session is None:raise ValueError('Validation of a restart needs --session from the live status capture_dir')
            self.fixed_map_odom=np.asarray(checked_session(self.run,self.session)['T_map_odom']['matrix'])
        self.h=None;self.fault=None;self.done=False;self.last_native=0;self.start=time.monotonic();self.live_ok=mode!='validate'
        self.graph_verified=False
        self.last_command=None;self.command_at=0.;self.pairs=[];self.pending=deque();self.mc=deque(maxlen=1000)
        self.raw=deque(maxlen=2000);self.normalized=deque(maxlen=2000)
        self.journal=(self.capture/'raw.jsonl').open('x',buffering=1)
        # A validation process has no TF or base-state publishers.
        self.dynamic=TransformBroadcaster(self) if mode!='validate' else None
        self.static=StaticTransformBroadcaster(self) if mode!='validate' else None
        self.base=self.create_publisher(PoseStamped,self.prefix+'/base_pose_F',50) if mode!='validate' else None
        status_topic=self.prefix+('/validation_status' if mode=='validate' else '/status')
        self.status=self.create_publisher(String,status_topic,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Odometry,self.prefix+'/native_odom',self.native,qos_profile_sensor_data)
        self.create_subscription(TFMessage,'/tf',self.check_tf,100)
        self.create_subscription(TFMessage,'/tf_static',self.check_tf,QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(.5,self.audit_publishers)
        if mode=='validate':
            self.create_subscription(String,self.prefix+'/status',self.live_status,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.mount=load(self.run/'mocap.yaml');normal=self.mount['normalizer']
        pcfg=dict(topics=dict(mocap=normal['output_topic'],validity=normal['diagnostics_topic']),
                  timing=dict(max_age_s=self.cfg['initialization']['maximum_age_s'],validity_wait_s=.08))
        self.join=PairedMocap(self,pcfg,self.valid_mocap,self.invalid)
        self.create_subscription(PoseStamped,normal['input_topic'],self.raw_mocap,qos_profile_sensor_data)
        self.create_subscription(PoseStamped,normal['output_topic'],lambda m:self.log('normalized',m),qos_profile_sensor_data)
        self.create_subscription(DiagnosticArray,normal['diagnostics_topic'],lambda m:self.log('validity',m),50)
        self.create_subscription(Twist,'/reference_velocity',self.command,qos_profile_sensor_data)
        self.create_timer(.01,self.tick);self.publish_status('waiting_for_static_synchronized_data')

    def live_status(self,msg):
        state=json.loads(msg.data)
        if state['run_path']!=str(self.run):self.fail('live_stack_belongs_to_different_map');return
        if self.phase=='localization' and state['capture_dir']!=str(self.session):self.fail('live_stack_belongs_to_different_restart');return
        self.live_ok=state['frozen'] and state['fault'] is None

    def log(self,kind,msg):
        if not self.journal.closed:self.journal.write(json.dumps(dict(kind=kind,receive_ros_ns=self.get_clock().now().nanoseconds,
            message=clean(message_to_ordereddict(msg))),allow_nan=False)+'\n')

    def event(self,reason):
        if not self.journal.closed:self.journal.write(json.dumps(dict(kind='event',reason=reason,receive_ros_ns=self.get_clock().now().nanoseconds))+'\n')

    def publish_status(self,reason):
        self.status.publish(String(data=json.dumps(dict(run_id=self.run.name,run_path=str(self.run),capture_dir=str(self.capture),
            mode=self.mode,context=self.phase,map_basis=self.cfg['map_basis'],frozen=self.h is not None,
            fault=self.fault,reason=reason,pairs=len(self.pairs),native_stamp_ns=self.last_native,
            source_mode='mocap_initial_then_lio',motion_commands_published=False,ndt_used=False))))

    def check_tf(self,msg):
        if self.fault:return
        critical=[t for t in msg.transforms if t.child_frame_id in ('base_link','camera_init','odom','map','body')]
        edges={('map','camera_init'),('camera_init','base_link')} if self.phase=='mapping' else {('map','odom'),('odom','camera_init'),('odom','base_link')}
        if any((t.header.frame_id,t.child_frame_id) not in edges for t in critical):self.fail('unexpected_global_pose_TF_edge')

    def audit_publishers(self):
        if self.fault or self.done:return
        self.graph_verified=False
        native=self.get_publishers_info_by_topic(self.prefix+'/native_odom')
        tf_publishers={topic:self.get_publishers_info_by_topic(topic) for topic in ('/tf','/tf_static')}
        unresolved=[p for p in [*native,*(p for pubs in tf_publishers.values() for p in pubs)]
                    if p.node_name=='_NODE_NAME_UNKNOWN_' or p.node_namespace=='_NODE_NAMESPACE_UNKNOWN_']
        if unresolved:
            # DDS endpoints can appear before their node identities during discovery.
            # Collect no pairs until every identity resolves; persistently unknown is a fault.
            if time.monotonic()-self.start<3.0:return
            self.fail('publisher_identity_unresolved_after_discovery');return
        if len(native)>1 or any(p.node_name!='anchored_fastlio' or p.node_namespace!='/' for p in native):self.fail('competing_or_wrong_native_publisher');return
        allowed={'robot_state_publisher','initial_anchor' if self.phase=='mapping' else 'native_base_bridge'}
        for topic,publishers in tf_publishers.items():
            counts={}
            for p in publishers:
                key=(p.node_namespace,p.node_name);counts[key]=counts.get(key,0)+1
                if p.node_name not in allowed or p.node_namespace!='/':self.fail('foreign_global_TF_publisher:'+str(key));return
            if any(c>1 for c in counts.values()):self.fail('duplicate_TF_publisher_name');return
        self.graph_verified=bool(native)

    def invalid(self,reason):
        if self.h is not None or self.fault or self.done:return
        self.event(reason);self.pairs=[];self.pending.clear();self.mc.clear();self.publish_status(reason)

    def raw_mocap(self,msg):
        if self.h is not None or self.done:return
        self.log('raw_vrpn_native_units',msg)
        try:self.raw.append(read_pose(msg.pose,ns(msg.header.stamp),self.get_clock().now().nanoseconds,msg.header.frame_id))
        except ValueError:self.invalid('invalid_raw_mocap')

    def valid_mocap(self,msg):
        if self.h is not None or self.fault or self.done:return
        try:
            if msg.header.frame_id!=self.mount['mocap']['world_frame']:raise ValueError('wrong_mocap_world')
            p=read_pose(msg.pose,ns(msg.header.stamp),self.get_clock().now().nanoseconds,msg.header.frame_id)
            self.normalized.append(p);self.mc.append((ns(msg.header.stamp),mocap_base(pose_matrix(msg.pose),self.cfg,self.ext)))
        except ValueError as e:self.invalid(str(e))

    def command(self,msg):
        self.log('observed_final_command',msg);self.command_at=time.monotonic()
        self.last_command=np.array([msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z])
        if not np.isfinite(self.last_command).all() or np.max(abs(self.last_command))>self.cfg['initialization']['maximum_command_abs']:self.invalid('nonzero_or_nonfinite_command')

    def fail(self,reason):
        self.fault=reason;self.event(reason);self.publish_status(reason)
        if not (self.capture/'failure.json').exists():save(self.capture/'failure.json',dict(reason=reason,frozen=self.h is not None,raw_preserved=True))
        if self.mode=='mapping':update_products(self.run)

    def native(self,msg):
        if self.fault or self.done:return
        stamp=ns(msg.header.stamp)
        if msg.header.frame_id!=self.cfg['native_world_frame'] or msg.child_frame_id!=self.cfg['native_body_frame']:self.fail('wrong_native_reference_frames');return
        if stamp<=self.last_native:self.fail('native_time_reset_or_duplicate_no_reanchoring');return
        age=(self.get_clock().now().nanoseconds-stamp)*1e-9
        if not 0<=age<=self.cfg['initialization']['maximum_age_s']:
            self.invalid('stale_native');return
        try:h=native_base(pose_matrix(msg.pose.pose),self.cfg,self.ext)
        except ValueError:self.fail('invalid_native_pose');return
        self.last_native=stamp
        if self.h is None:
            self.log('native_fastlio_odometry',msg)
            if self.graph_verified:self.pending.append((stamp,h,msg.header.stamp))
        else:self.publish_base(h,msg.header.stamp)

    def publish_base(self,h,stamp):
        if self.mode=='validate':return
        parent='camera_init' if self.mode=='mapping' else 'odom'
        tf=tf_message(h,stamp,parent,'base_link');self.dynamic.sendTransform(tf)
        p=PoseStamped();p.header=tf.header;p.pose.position.x,p.pose.position.y,p.pose.position.z=h[:3,3].tolist();p.pose.orientation=tf.transform.rotation
        self.base.publish(p);self.publish_status('calibrated_native_mapping' if self.mode=='mapping' else 'mocap_initialized_local_odometry')

    def tick(self):
        if self.fault:raise RuntimeError(self.fault)
        if self.h is not None or self.done:return
        limits=self.cfg['initialization']
        if time.monotonic()-self.start>limits['timeout_s']:self.fail('initialization_timeout_no_fallback');return
        if not self.live_ok or not self.graph_verified:return
        if self.last_command is None or time.monotonic()-self.command_at>limits['maximum_command_age_s']:
            if self.pairs or self.pending:self.invalid('no_fresh_zero_chassis_command')
            return
        if not np.isfinite(self.last_command).all() or np.max(abs(self.last_command))>limits['maximum_command_abs']:return
        offset=round(self.cfg['clock_offset_mocap_minus_lio_s']*1e9);now=self.get_clock().now().nanoseconds
        while self.pending and self.mc:
            stamp,h,header=self.pending[0];target=stamp+offset
            if self.mc[-1][0]<target and (now-stamp)*1e-9<.08:return
            self.pending.popleft();ms,mh=min(self.mc,key=lambda p:abs(p[0]-target))
            if abs(ms-target)*1e-9>limits['maximum_pair_skew_s']:self.invalid('no_synchronous_mocap');return
            if self.pairs and (stamp-self.pairs[-1]['stamp_ns'])*1e-9>limits['maximum_pair_gap_s']:self.invalid('long_native_pair_gap');return
            self.pairs.append(dict(stamp_ns=stamp,mocap_stamp_ns=ms,skew_s=(ms-target)*1e-9,T_M_B=mh.tolist(),T_F_B=h.tolist()))
            try:
                result=static_window(self.pairs,self.cfg) if self.mode=='validate' else estimate(self.pairs,self.cfg)
            except ValueError as e:
                if str(e) not in ('insufficient_pairs','need_three_seconds_of_measurement_time'):self.invalid(str(e));return
                continue
            first=self.pairs[0]['mocap_stamp_ns'];last=self.pairs[-1]['mocap_stamp_ns']
            units=check_raw_normalized([p for p in self.raw if first<=p['stamp_ns']<=last],[p for p in self.normalized if first<=p['stamp_ns']<=last],self.mount['mocap']['position_scale_to_m'])
            if units['coverage']<limits['minimum_raw_normalized_coverage'] or units['mismatched_stamp_ns']:self.invalid('raw_units_or_timestamp_check_failed');return
            self.freeze(result,units)
            if self.mode!='validate':self.publish_base(h,header)
            return

    def freeze(self,result,units):
        checked(self.run,self.mode!='mapping');self.event('held_out_validation_only' if self.mode=='validate' else 'freeze_once');self.journal.close()
        save(self.capture/'pairs.json',self.pairs)
        provenance=dict(run_id=self.run.name,data_kind=self.cfg['data_kind'],inputs_sha256=sha(self.run/'inputs.lock.json'),
            raw_sha256=sha(self.capture/'raw.jsonl'),pairs_sha256=sha(self.capture/'pairs.json'),raw_normalized_check=units,
            clock=dict(mocap_source=self.mount['mocap']['timestamp_source'],fastlio_source='lidar_end_time',
                offset_s=self.cfg['clock_offset_mocap_minus_lio_s'],offset_verified=self.cfg['clock_offset_verified'],
                camera_latency_s=self.cfg['camera_latency_s'],scope='static timestamp pairing, not exposure synchronization certification'),
            reference_point='base_link',ndt_used=False,motion_commands_published=False)
        if self.mode=='validate':
            output=validation_errors(self.pairs,self.calibration,self.fixed_map_odom)
            output.update(provenance,static_spread=result,calibration_sha256=sha(self.run/'calibration.yaml'),context=self.phase)
            save(self.capture/'summary.json',output);self.done=True;self.publish_status('held_out_validation_saved_no_refit');return
        native=np.asarray(result.pop('T_mocap_native')['matrix']);result.pop('T_native_mocap')
        if self.mode=='mapping':
            result.update(provenance,map_basis='native_fastlio_mapping_world',T_mocap_map=record(native,'mocap_world','map'),
                T_map_mocap=record(inverse(native),'map','mocap_world'),T_map_F0=record(np.eye(4),'map','F0'),
                map_origin_in_mocap_m=native[:3,3].tolist(),mocap_origin_in_map_m=inverse(native)[:3,3].tolist(),
                PCD_coordinates_changed=False,extrinsics_sha256=sha(self.run/'extrinsics.yaml'))
            save(self.run/'calibration.yaml',result,True);save(self.run/'calibration.lock.json',dict(sha256=sha(self.run/'calibration.yaml')))
            self.h=native;self.static.sendTransform(tf_message(np.eye(4),self.get_clock().now().to_msg(),'map','camera_init'));update_products(self.run)
        else:
            self.h=initial_map_odom(self.calibration,native)
            result.update(provenance,T_map_odom=record(self.h,'map','odom'),T_mocap_odom=record(native,'mocap_world','odom'),
                calibration_sha256=sha(self.run/'calibration.yaml'),initialization_only=True,continuous_mocap_correction=False)
            save(self.capture/'initialization.yaml',result,True);save(self.capture/'initialization.lock.json',dict(sha256=sha(self.capture/'initialization.yaml')))
            stamp=self.get_clock().now().to_msg();self.static.sendTransform([tf_message(self.h,stamp,'map','odom'),tf_message(np.eye(4),stamp,'odom','camera_init')])
        self.publish_status('frozen');self.get_logger().info('Static initialization saved. No NDT or continuous mocap correction. Manual operation only.')


def run_node(run,mode,capture_dir=None,context='mapping',session=None,ros=None):
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=ros or [],signal_handler_options=SignalHandlerOptions.NO);node=None;code=0
    try:
        node=NativeBridge(run,mode,capture_dir,context,session)
        while rclpy.ok() and not node.done:rclpy.spin_once(node,timeout_sec=.1)
    except KeyboardInterrupt:
        if node and node.h is None and not node.done:node.fail('operator_interrupt_before_initialization')
    except RuntimeError as e:
        code=2
        if node:node.get_logger().error(str(e))
        else:raise
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN)
        if node:
            if not node.journal.closed:node.journal.close()
            node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    return code


def main():
    p=argparse.ArgumentParser();p.add_argument('--run',required=True);p.add_argument('--mode',choices=['mapping','localization'],default='mapping');p.add_argument('--capture-dir')
    args,ros=p.parse_known_args();return run_node(args.run,args.mode,capture_dir=args.capture_dir,ros=ros)
