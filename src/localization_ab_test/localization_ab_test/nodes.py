"""Controller and separate fail-closed velocity relay. Guard owns chassis topic."""
import argparse
import json
import math
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from std_msgs.msg import Bool, String

from .core import MotionConfig, MotionGate, base_pose, load_mount, pose_dict
from .qos import latest_value_qos


PREFIX='/localization_ab'


class MotionNode(Node):
    def __init__(self,args):
        super().__init__('localization_ab_motion')
        self.group=args.group
        self.mount=load_mount(args.mount)
        self.direction=1 if args.direction=='forward' else -1
        self.gate=MotionGate(time.monotonic(),MotionConfig(direction=self.direction))
        self.command=self.create_publisher(TwistStamped,PREFIX+'/request',latest_value_qos())
        self.status=self.create_publisher(String,PREFIX+'/state',latest_value_qos(10))
        self.events=self.create_publisher(String,PREFIX+'/events',100)
        self.permit_time=self.estimate_time=None
        self.sent_events=0
        self.create_subscription(PoseStamped,'/mocap/Tracker0/pose',self.mocap,qos_profile_sensor_data)
        self.create_subscription(DiagnosticArray,'/mocap/Tracker0/diagnostics',self.diagnostics,10)
        self.create_subscription(Bool,PREFIX+'/recorder_ready',self.permit,latest_value_qos())
        self.create_subscription(PoseStamped,PREFIX+f'/estimate/{args.group}',self.estimate,10)
        self.create_subscription(String,PREFIX+'/guard_status',self.guard_status,latest_value_qos(10))
        self.timer=self.create_timer(.02,self.tick)

    def estimate(self,msg):
        if msg.header.stamp.sec>0:
            self.estimate_time=time.monotonic()

    def permit(self,msg):
        if msg.data:self.permit_time=time.monotonic()
        else:self.gate.stop(time.monotonic(),'recorder_not_ready')

    def diagnostics(self,msg):
        for status in msg.status:
            if any(k.key=='valid' and k.value!='true' for k in status.values):
                self.gate.stop(time.monotonic(),'mocap_diagnostic_invalid')

    def guard_status(self,msg):
        if json.loads(msg.data).get('latched_failure'):
            self.gate.stop(time.monotonic(),'velocity_guard_failure')

    def mocap(self,msg):
        now=time.monotonic()
        if self.gate.state=='ACQUIRE' and (self.permit_time is None or now-self.permit_time>.3
                                          or self.estimate_time is None):
            return
        age=(self.get_clock().now().nanoseconds-(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec))/1e9
        if msg.header.frame_id!='mocap_world' or not -.05 <= age <= .25:
            self.gate.stop(now,'mocap_frame_or_header_age_invalid')
        else:
            try:
                self.gate.ingest(base_pose(pose_dict(msg.header.stamp,msg.pose),self.mount),now)
            except (ValueError,KeyError):self.gate.stop(now,'invalid_mocap_geometry')
        if self.gate.state=='POST_STOP':self.publish_command(0.)

    def publish_command(self,speed):
        msg=TwistStamped()
        msg.header.stamp=self.get_clock().now().to_msg()
        msg.header.frame_id='base_link'
        msg.twist.linear.x=speed
        self.command.publish(msg)

    def tick(self):
        now=time.monotonic()
        if self.gate.state!='ACQUIRE':
            if self.permit_time is None or now-self.permit_time>.3:
                self.gate.stop(now,'recorder_heartbeat_lost')
            if self.estimate_time is None or now-self.estimate_time>.5:
                self.gate.stop(now,'localization_pose_stale')
        speed=self.gate.tick(now)
        self.publish_command(speed)
        while self.sent_events<len(self.gate.events):
            event=dict(self.gate.events[self.sent_events],group=self.group,
                       ros_stamp_ns=self.get_clock().now().nanoseconds,event_id=self.sent_events)
            self.events.publish(String(data=json.dumps(event)))
            self.sent_events+=1
        self.status.publish(String(data=json.dumps(dict(
            group=self.group,direction=self.direction,state=self.gate.state,reason=self.gate.reason,
            displacement_m=self.gate.distance,path_length_m=self.gate.path_length,
            mocap_stamp_ns=self.gate.last_stamp,ros_stamp_ns=self.get_clock().now().nanoseconds,
            request_linear_x=speed,request_angular_z=0.,duplicates=self.gate.duplicates))))

    def stop(self):
        self.publish_command(0.)


class VelocityGuard(Node):
    """Own the chassis topic in a separate process and latch any loss to STOP.

    A total computer/DDS failure still needs a short chassis watchdog. The
    runner checks the configured watchdog and refuses to arm if it is >0.5 s.
    """
    def __init__(self,args):
        super().__init__('localization_ab_velocity_guard')
        self.armed=args.arm
        self.allowed_speed=.15 if args.direction=='forward' else -.15
        self.latest=self.last_stamp=self.mocap_stamp=0
        self.speed=0.
        self.permit_time=self.mocap_time=None
        self.failure=''
        self.velocity=self.create_publisher(Twist,'/nav/cmd_vel',1)
        self.mode=self.create_publisher(String,'/teleop/mode',1)
        self.status=self.create_publisher(String,PREFIX+'/guard_status',latest_value_qos(10))
        self.create_subscription(TwistStamped,PREFIX+'/request',self.request,latest_value_qos())
        self.create_subscription(PoseStamped,'/mocap/Tracker0/pose',self.mocap,qos_profile_sensor_data)
        self.create_subscription(Bool,PREFIX+'/recorder_ready',self.permit,latest_value_qos())
        self.create_subscription(DiagnosticArray,'/mocap/Tracker0/diagnostics',self.diagnostic,10)
        self.create_timer(.02,self.tick)
        self.create_timer(.25,self.check_ownership)

    def fail(self,reason):
        # Keep the first cause; cleanup's false permit must not overwrite it.
        if not self.failure:
            self.get_logger().error(reason)
            self.failure=reason
        self.speed=0.
        self.stop()

    def diagnostic(self,msg):
        if any(k.key=='valid' and k.value!='true' for s in msg.status for k in s.values):
            self.fail('invalid_mocap_diagnostic')

    def permit(self,msg):
        if msg.data:self.permit_time=time.monotonic()
        else:self.fail('recorder_not_ready')

    def mocap(self,msg):
        stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        if stamp>self.mocap_stamp:
            self.mocap_stamp=stamp
            age=(self.get_clock().now().nanoseconds-stamp)/1e9
            if msg.header.frame_id=='mocap_world' and -.05<=age<=.25:
                self.mocap_time=time.monotonic()

    def request(self,msg):
        stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        age=(self.get_clock().now().nanoseconds-stamp)/1e9
        v=msg.twist
        values=(v.linear.x,v.linear.y,v.linear.z,v.angular.x,v.angular.y,v.angular.z)
        if not all(math.isfinite(x) for x in values) or v.linear.x not in (0.,self.allowed_speed) or any(x!=0. for x in values[1:]):
            self.fail('command_outside_fixed_protocol')
            return
        if stamp<=self.last_stamp or not -.05<=age<=.20 or msg.header.frame_id!='base_link':
            self.fail('old_or_invalid_request')
            return
        self.latest=time.monotonic()
        self.last_stamp=stamp
        self.speed=v.linear.x
        if not self.speed:self.stop()

    def check_ownership(self):
        if not self.armed:return
        for topic in ('/nav/cmd_vel','/teleop/mode'):
            others=[p.node_name for p in self.get_publishers_info_by_topic(topic)
                    if p.node_name!=self.get_name()]
            if others:self.fail('competing_publisher:'+topic+':'+','.join(others))

    def stop(self):
        if self.armed:
            self.velocity.publish(Twist())
            self.mode.publish(String(data='STOP'))

    def tick(self):
        now=time.monotonic()
        valid=(self.latest and now-self.latest<=.20 and self.permit_time is not None
               and now-self.permit_time<=.30 and self.mocap_time is not None and now-self.mocap_time<=.20)
        if self.speed and not valid:self.fail('request_recorder_or_mocap_stale')
        moving=bool(self.armed and valid and self.speed and not self.failure)
        if moving:
            msg=Twist();msg.linear.x=self.speed
            self.mode.publish(String(data='EXTERNAL'))
            self.velocity.publish(msg)
        else:self.stop()
        self.status.publish(String(data=json.dumps(dict(
            armed=self.armed,moving=moving,latched_failure=self.failure,
            ros_stamp_ns=self.get_clock().now().nanoseconds))))


def _spin(kind):
    parser=argparse.ArgumentParser()
    parser.add_argument('--direction',choices=('forward','reverse'),default='forward')
    if kind=='motion':
        parser.add_argument('--group',choices=list('ABC'),required=True)
        parser.add_argument('--mount',required=True)
    else:parser.add_argument('--arm',action='store_true')
    args=parser.parse_args()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    quit_requested=False
    def quit_handler(*_):
        nonlocal quit_requested
        quit_requested=True
    signal.signal(signal.SIGINT,quit_handler)
    signal.signal(signal.SIGTERM,quit_handler)
    node=MotionNode(args) if kind=='motion' else VelocityGuard(args)
    try:
        while rclpy.ok() and not quit_requested:
            rclpy.spin_once(node,timeout_sec=.02)
            if kind=='motion' and node.gate.state in ('DONE','ABORTED'):break
    finally:
        # Let independent guard observe zero commands before ROS teardown.
        for _ in range(10):
            node.stop()
            rclpy.spin_once(node,timeout_sec=.02)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


def motion_main():_spin('motion')
def guard_main():_spin('guard')
