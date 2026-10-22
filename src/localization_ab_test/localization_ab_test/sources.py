"""Three isolated estimate branches. Mocap global correction is only in C."""
import argparse
from bisect import bisect_right
from collections import deque
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Bool,String
from tf2_msgs.msg import TFMessage

from ndt_match_evaluation_core import yaw
from fastlio_mocap_trajectory_evaluation_core import interpolate_pose_strict
from .core import base_pose, load_mount, pose_dict, compose, inverse, planar


def stamp_ns(s):return s.sec*10**9+s.nanosec


def from_tf(tf):
    t,q=tf.transform.translation,tf.transform.rotation
    return dict(stamp_ns=stamp_ns(tf.header.stamp),valid=True,x=t.x,y=t.y,z=t.z,
                qx=q.x,qy=q.y,qz=q.z,qw=q.w)


class PoseSources(Node):
    def __init__(self,args):
        super().__init__('localization_ab_pose_sources')
        self.args=args
        self.mount=load_mount(args.mount)
        self.history=deque(maxlen=500)
        self.correction=self.ndt_correction=None
        self.correction_time=self.ndt_receive=None
        self.pending=deque(maxlen=200)
        self.last_lio_stamp=self.last_mocap_stamp=0
        self.ndt_ready=False
        self.pubs={g:self.create_publisher(PoseStamped,f'/localization_ab/estimate/{g}',20) for g in args.groups}
        self.diagnostic=self.create_publisher(String,'/localization_ab/correction',100)
        self.create_subscription(TFMessage,'/tf',self.tf,100)
        # A/B never create a mocap subscription in the estimate source.
        if 'C' in args.groups:
            self.create_subscription(PoseStamped,'/mocap/Tracker0/pose',self.mocap,qos_profile_sensor_data)
        if 'B' in args.groups:
            qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(TransformStamped,'/static_localization/map_to_odom',self.ndt,qos)
            self.create_subscription(Bool,'/static_localization/ready',self.ready,qos)

    def ready(self,msg):self.ndt_ready=msg.data

    def ndt(self,msg):
        if msg.header.frame_id!='map' or msg.child_frame_id!='odom':return
        self.ndt_correction=planar(from_tf(msg))
        self.ndt_receive=time.monotonic()

    def mocap(self,msg):
        if msg.header.frame_id!='mocap_world':return
        p=base_pose(pose_dict(msg.header.stamp,msg.pose),self.mount)
        if p['stamp_ns']<=self.last_mocap_stamp:return
        self.last_mocap_stamp=p['stamp_ns']
        self.pending.append(p)
        self.update_correction()

    def update_correction(self):
        if not self.history:return
        while self.pending:
            p=self.pending[0]
            # Positive offset means mocap header is later for same motion.
            target=p['stamp_ns']-round(self.args.mocap_offset*1e9)
            if target>self.history[-1]['stamp_ns']:break
            self.pending.popleft()
            local,reason=interpolate_pose_strict(list(self.history),target,.15)
            if local is None:continue
            # T_mocap_odom = T_mocap_base * inverse(T_odom_base), at one time.
            self.correction=compose(planar(p),inverse(planar(local)))
            self.correction_time=time.monotonic()
            self.diagnostic.publish(String(data=json.dumps(dict(
                mocap_stamp_ns=p['stamp_ns'],lio_target_stamp_ns=target,
                correction=list(self.correction),mocap_offset_s=self.args.mocap_offset,
                timing_verified=self.args.timing_verified,reason='mocap_global_pose_correction',
                feedback_scope='C output only; FAST-LIO and NDT states unchanged'))))

    def publish(self,g,pose,stamp,frame):
        import math
        msg=PoseStamped();msg.header.stamp.sec=stamp//10**9;msg.header.stamp.nanosec=stamp%10**9
        msg.header.frame_id=frame
        msg.pose.position.x=float(pose[0]);msg.pose.position.y=float(pose[1])
        msg.pose.orientation.z=math.sin(pose[2]/2);msg.pose.orientation.w=math.cos(pose[2]/2)
        self.pubs[g].publish(msg)

    def tf(self,msg):
        for tf in msg.transforms:
            if tf.header.frame_id!='odom' or tf.child_frame_id!='base_link':continue
            p=from_tf(tf);stamp=p['stamp_ns']
            if stamp<=self.last_lio_stamp:continue
            self.last_lio_stamp=stamp
            self.history.append(p)
            local=planar(p)
            if 'A' in self.pubs:self.publish('A',local,stamp,'odom')
            if 'B' in self.pubs and self.ndt_correction and self.ndt_ready:
                self.publish('B',compose(self.ndt_correction,local),stamp,'map')
            if 'C' in self.pubs:
                self.update_correction()
                if self.correction and time.monotonic()-self.correction_time<=.25:
                    self.publish('C',compose(self.correction,local),stamp,'mocap_world')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--groups',default='A',help='A, B, C or ABC for simultaneous independent outputs')
    parser.add_argument('--mount',required=True)
    parser.add_argument('--mocap-offset',type=float,default=0.)
    parser.add_argument('--timing-verified',action='store_true')
    args=parser.parse_args()
    if not args.groups or any(g not in 'ABC' for g in args.groups):parser.error('groups must be a subset of ABC')
    rclpy.init();node=PoseSources(args)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
