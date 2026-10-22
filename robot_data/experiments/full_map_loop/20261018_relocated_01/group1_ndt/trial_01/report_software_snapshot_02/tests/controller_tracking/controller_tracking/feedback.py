"""Paired mocap validity -> fixed test frame pose, consistent speed and private TF."""
import argparse
import json
import math
import time
import signal
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from .configuration import check_session
from .state import base_in_level, to_test, VelocityFit


def ns(stamp): return stamp.sec*10**9+stamp.nanosec


def set_stamp(header, nanoseconds, frame):
    header.frame_id=frame
    header.stamp.sec=int(nanoseconds)//10**9;header.stamp.nanosec=int(nanoseconds)%10**9


def xy_yaw(msg, p):
    msg.position.x=float(p[0]);msg.position.y=float(p[1])
    msg.orientation.z=math.sin(float(p[2])/2);msg.orientation.w=math.cos(float(p[2])/2)


def converted(msg,cfg,mount,anchor=None):
    p,q=msg.pose.position,msg.pose.orientation
    if msg.header.frame_id!='mocap_world':raise ValueError('unexpected_mocap_frame')
    pose,z=base_in_level([p.x,p.y,p.z],[q.x,q.y,q.z,q.w],mount['rigid_body_to_base_link'],
                         cfg['calibration']['world_level_rotation_xyzw'])
    return to_test(pose,anchor) if anchor is not None else pose


class PairedMocap:
    """Integer-header association shared by freeze and live state production."""
    def __init__(self,node,cfg,on_valid,on_invalid):
        self.node=node;self.cfg=cfg;self.valid=on_valid;self.invalid=on_invalid
        self.poses={};self.status={};self.last=0
        node.create_subscription(PoseStamped,cfg['topics']['mocap'],self.pose,qos_profile_sensor_data)
        node.create_subscription(DiagnosticArray,cfg['topics']['validity'],self.diagnostic,50)
        node.create_timer(.01,self.expire)

    def pose(self,msg):
        stamp=ns(msg.header.stamp)
        if stamp<=self.last:self.invalid('duplicate_or_backward_pose');return
        self.poses[stamp]=(msg,time.monotonic());self.join(stamp)

    def diagnostic(self,msg):
        stamp=ns(msg.header.stamp)
        statuses=[s for s in msg.status if s.name=='mocap_tracker0_normalizer']
        valid=bool(statuses) and all(s.level in (0,b'\x00') and dict((v.key,v.value) for v in s.values).get('valid')=='true' for s in statuses)
        if not valid:
            self.invalid('normalizer_invalid:'+(';'.join(s.message for s in statuses) or 'missing_status'))
        if stamp>0:self.status[stamp]=(valid,time.monotonic());self.join(stamp)

    def join(self,stamp):
        if stamp not in self.poses or stamp not in self.status:return
        msg,_=self.poses.pop(stamp);valid,_=self.status.pop(stamp)
        if not valid:return
        age=(self.node.get_clock().now().nanoseconds-stamp)*1e-9
        if stamp<=self.last or not 0<=age<=self.cfg['timing']['max_age_s']:
            self.invalid('out_of_order_or_stale_measurement');return
        self.last=stamp;self.valid(msg)

    def expire(self):
        now=time.monotonic();limit=self.cfg['timing']['validity_wait_s']
        for mapping in (self.poses,self.status):
            for stamp,(_,arrived) in list(mapping.items()):
                if now-arrived>limit:
                    del mapping[stamp];self.invalid('unpaired_pose_or_validity')


class Feedback(Node):
    def __init__(self,session):
        super().__init__('mocap_feedback',namespace='tracking')
        self.cfg,self.mount,self.frame=check_session(session)
        t=self.cfg['topics'];self.fit=VelocityFit(self.cfg['timing']['velocity_window_s'],self.cfg['timing']['max_gap_s'])
        self.publisher=self.create_publisher(Odometry,t['state'],50)
        self.truth=self.create_publisher(PoseStamped,t['base_pose'],50)
        self.status=self.create_publisher(String,t['state_status'],50)
        self.tf=self.create_publisher(TFMessage,'/tracking/tf',50)
        self.static=self.create_publisher(TFMessage,'/tracking/tf_static',QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(TFMessage,'/tf_static',self.static_tf,QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.static_cache={};self.last_receive=0.;self.last_source=0;self.last_fit=0;self.reason='waiting_for_mocap';self.valid=False
        self.count=0;self.invalid_count=0;self.compute_ms=None;self.delay=None
        self.paired=PairedMocap(self,self.cfg,self.receive,self.invalidate)
        self.create_timer(.02,self.tick)

    def static_tf(self,msg):
        # Relay robot mounting TF only; never import a localization/base parent.
        forbidden={'base_link','map','odom',self.cfg['frame']}
        for tf in msg.transforms:
            if tf.child_frame_id not in forbidden:self.static_cache[tf.child_frame_id]=tf
        if self.static_cache:self.static.publish(TFMessage(transforms=list(self.static_cache.values())))

    def invalidate(self,reason):
        self.valid=False;self.reason=reason;self.invalid_count+=1;self.fit.reset()
        self.publish_status()

    def receive(self,msg):
        start=time.monotonic();stamp=ns(msg.header.stamp)
        try:
            pose=converted(msg,self.cfg,self.mount,self.frame['anchor_level_xyyaw'])
            truth=PoseStamped();set_stamp(truth.header,stamp,self.cfg['frame']);xy_yaw(truth.pose,pose)
            self.truth.publish(truth)
            fitted=self.fit.add(stamp*1e-9,pose)
            self.last_source=stamp;self.last_receive=time.monotonic();self.count+=1
            if fitted is None:
                self.valid=False;self.reason='velocity_window_warmup';self.publish_status();return
            center,p,v,delay=fitted
            odom=Odometry();set_stamp(odom.header,round(center*1e9),self.cfg['frame']);odom.child_frame_id=self.cfg['base_frame']
            xy_yaw(odom.pose.pose,p);odom.twist.twist.linear.x=float(v[0]);odom.twist.twist.linear.y=float(v[1]);odom.twist.twist.angular.z=float(v[2])
            # Covariance is not calibrated. Diagonal -1 explicitly marks unknown in this experiment.
            for i in (0,7,14,21,28,35):odom.pose.covariance[i]=-1.;odom.twist.covariance[i]=-1.
            self.publisher.publish(odom)
            tf=TransformStamped();tf.header=odom.header;tf.child_frame_id=self.cfg['base_frame']
            tf.transform.translation.x=float(p[0]);tf.transform.translation.y=float(p[1]);tf.transform.rotation=odom.pose.pose.orientation
            self.tf.publish(TFMessage(transforms=[tf]))
            self.last_fit=ns(odom.header.stamp);self.delay=delay;self.valid=True;self.reason='valid'
            self.compute_ms=(time.monotonic()-start)*1000;self.publish_status()
        except ValueError as e:self.invalidate(str(e))

    def tick(self):
        age=(self.get_clock().now().nanoseconds-self.last_source)*1e-9
        if not self.last_receive or time.monotonic()-self.last_receive>self.cfg['timing']['max_age_s'] or not 0<=age<=self.cfg['timing']['max_age_s']:
            if self.valid or self.reason!='mocap_stale':self.invalidate('mocap_stale')
        self.publish_status()

    def publish_status(self):
        self.status.publish(String(data=json.dumps(dict(valid=self.valid,reason=self.reason,
            ros_ns=self.get_clock().now().nanoseconds,source_ns=self.last_source,fit_ns=self.last_fit,
            fit_delay_s=self.delay,compute_ms=self.compute_ms,valid_samples=self.count,invalid_events=self.invalid_count,
            clock_source=self.cfg['timing']['source'],camera_latency_s=self.cfg['timing']['camera_latency_s']))))


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--session',required=True);args,ros=parser.parse_known_args()
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=ros,signal_handler_options=SignalHandlerOptions.NO);node=Feedback(Path(args.session))
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
