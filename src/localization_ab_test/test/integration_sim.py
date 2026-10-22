"""Run without hardware: ROS_DOMAIN_ID=174 python3 test/integration_sim.py.

Exercises the real motion node + independent guard using a simulated base.
The explicit isolated domain check prevents publishing to the live robot.
"""
import argparse
import json
import os
import time
from types import SimpleNamespace

if os.environ.get('ROS_DOMAIN_ID')!='174':
    raise SystemExit('This simulation only runs in isolated ROS_DOMAIN_ID=174')

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import PoseStamped,Twist
from std_msgs.msg import Bool,String
from diagnostic_msgs.msg import DiagnosticArray,DiagnosticStatus,KeyValue
from localization_ab_test.nodes import MotionNode,VelocityGuard
from localization_ab_test.qos import latest_value_qos


class VirtualBase(Node):
    def __init__(self,fault):
        super().__init__('test_virtual_chassis')
        self.x=self.speed=0.
        self.last=time.monotonic()
        self.started=None;self.drop=False;self.fault=fault
        self.state={};self.commands=[];self.mode='STOP'
        self.pose=self.create_publisher(PoseStamped,'/mocap/Tracker0/pose',20)
        self.estimate=self.create_publisher(PoseStamped,'/localization_ab/estimate/A',20)
        self.permit=self.create_publisher(Bool,'/localization_ab/recorder_ready',latest_value_qos())
        self.diag=self.create_publisher(DiagnosticArray,'/mocap/Tracker0/diagnostics',10)
        self.create_subscription(Twist,'/nav/cmd_vel',self.command,10)
        self.create_subscription(String,'/teleop/mode',lambda msg:setattr(self,'mode',msg.data),10)
        self.create_subscription(String,'/localization_ab/state',self.status,latest_value_qos(10))
        self.create_timer(.01,self.tick)

    def command(self,msg):
        self.commands.append((time.monotonic(),msg.linear.x,msg.angular.z))
        self.speed=msg.linear.x if self.mode=='EXTERNAL' else 0.
        if msg.linear.x and self.started is None:self.started=time.monotonic()

    def status(self,msg):self.state=json.loads(msg.data)

    def tick(self):
        now=time.monotonic();self.x+=self.speed*(now-self.last);self.last=now
        fault_due=self.started is not None and now-self.started>.4
        if not (self.fault=='permit' and fault_due):
            self.permit.publish(Bool(data=True))
        msg=PoseStamped();msg.header.stamp=self.get_clock().now().to_msg();msg.header.frame_id='mocap_world'
        msg.pose.position.x=self.x;msg.pose.position.z=.18;msg.pose.orientation.w=1.
        self.drop=bool(self.fault=='mocap' and fault_due)
        if not self.drop:
            self.pose.publish(msg)
            diag=DiagnosticArray();diag.header=msg.header
            diag.status=[DiagnosticStatus(name='test',values=[KeyValue(key='valid',value='true')])]
            self.diag.publish(diag)
        msg.header.frame_id='odom';msg.pose.position.z=0.
        self.estimate.publish(msg)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--fault',choices=['none','mocap','request','permit'],default='none')
    parser.add_argument('--direction',choices=('forward','reverse'),default='forward')
    parser.add_argument('--mount',default='/home/zkam/test_nav/robot_data/config/mocap_tracker0.yaml')
    args=parser.parse_args()
    rclpy.init()
    base=VirtualBase(args.fault)
    motion=MotionNode(SimpleNamespace(group='A',mount=args.mount,direction=args.direction))
    guard=VelocityGuard(SimpleNamespace(arm=True,direction=args.direction))
    sign=1 if args.direction=='forward' else -1
    executor=SingleThreadedExecutor()
    for node in (base,motion,guard):executor.add_node(node)
    end=time.monotonic()+32
    motion_removed=False
    try:
        while time.monotonic()<end:
            executor.spin_once(timeout_sec=.002)
            if args.fault=='request' and base.started and time.monotonic()-base.started>.4 and not motion_removed:
                executor.remove_node(motion)
                motion_removed=True
            if args.fault=='request' and guard.failure and base.commands[-1][1]==0.:break
            if base.state.get('state') in ('DONE','ABORTED'):break
        if args.fault=='none':
            assert base.state.get('state')=='DONE',base.state
            assert base.state['reason']=='target_reached',base.state
            assert 2.<=sign*base.x<2.02,base.x
        elif args.fault in ('mocap','permit'):
            assert base.state.get('state')=='ABORTED',base.state
            allowed=('mocap_stale','velocity_guard_failure') if args.fault=='mocap' else ('recorder_heartbeat_lost','velocity_guard_failure')
            assert base.state['reason'] in allowed,base.state
            assert abs(base.x)<.2,base.x
        else:
            assert guard.failure=='request_recorder_or_mocap_stale',guard.failure
            assert abs(base.x)<.2,base.x
        assert any(sign*v>0. for _,v,_ in base.commands)
        assert all(v in (0.,sign*.15) and w==0. for _,v,w in base.commands)
        assert base.commands[-1][1]==0.
        print(json.dumps(dict(result='PASS',fault=args.fault,physical_distance_m=base.x,
                              final_state=base.state,guard_failure=guard.failure),indent=2))
    finally:
        guard.stop()
        for node in (motion,guard,base):node.destroy_node()
        executor.shutdown();rclpy.shutdown()


if __name__=='__main__':main()
