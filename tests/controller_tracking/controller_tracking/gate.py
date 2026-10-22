"""Normally-disarmed experiment relay; existing arbitrator and chassis stay unchanged."""
import argparse
import json
import time
import math
import signal
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Empty
from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from .configuration import check_session


class GateLogic:
    """Faults latch until an explicit new arm; no automatic feedback-source fallback."""
    def __init__(self,command_timeout=.2,lease_timeout=.3,age=.15):
        self.command_timeout=command_timeout;self.lease_timeout=lease_timeout;self.age=age
        self.armed=False;self.fault=None;self.started=0.;self.lease=0.;self.state=0.;self.command=0.
        self.state_valid=False;self.mode='';self.mode_received=0.;self.v=[0.,0.,0.]

    def arm(self,now):
        if not self.state_valid or now-self.state>self.age or self.mode!='EXTERNAL' or now-self.mode_received>.5:
            raise ValueError('need_fresh_valid_mocap_and_EXTERNAL')
        if max(abs(v) for v in self.v)>.03:raise ValueError('robot_not_stationary')
        self.armed=True;self.fault=None;self.started=now;self.lease=now;self.command=0.

    def evaluate(self,now):
        if not self.armed:return 'disarmed'
        if self.fault:return self.fault
        if not self.state_valid or now-self.state>self.age:self.fault='mocap_invalid_or_stale'
        elif self.mode!='EXTERNAL' or now-self.mode_received>.5:self.fault='operator_mode_or_mode_stale'
        elif now-self.lease>self.lease_timeout:self.fault='execution_lease_expired'
        elif now-self.started>self.command_timeout and now-self.command>self.command_timeout:self.fault='smoothed_command_timeout'
        return self.fault or 'armed'


class Gate(Node):
    def __init__(self,session):
        super().__init__('command_gate',namespace='tracking');self.cfg,_,_=check_session(session)
        ex=self.cfg['execution'];self.logic=GateLogic(ex['command_timeout_s'],ex['lease_timeout_s'],self.cfg['timing']['max_age_s'])
        t=self.cfg['topics'];self.command=Twist();self.release_at=None;self.previous='disarmed'
        self.last_graph_check=0.
        self.output=self.create_publisher(Twist,t['gate_cmd'],10)
        self.status=self.create_publisher(String,'/tracking/gate_status',50)
        self.create_subscription(String,t['state_status'],self.state_status,50)
        self.create_subscription(Odometry,t['state'],self.velocity,50)
        self.create_subscription(String,t['mode'],self.mode,10)
        self.create_subscription(Twist,t['smoothed_cmd'],self.cmd,10)
        self.create_subscription(Empty,'/tracking/execution_lease',self.lease,10)
        self.create_service(SetBool,'/tracking/arm',self.arm)
        self.create_timer(.02,self.tick)

    def state_status(self,msg):
        data=json.loads(msg.data);self.logic.state_valid=data['valid'];self.logic.state=time.monotonic()
        # A brief invalid event between timer ticks must still stop a running test.
        if self.logic.armed and not data['valid']:self.logic.fault='mocap_invalid:'+data['reason']

    def velocity(self,msg):
        v=msg.twist.twist;self.logic.v=[v.linear.x,v.linear.y,v.angular.z]

    def mode(self,msg):self.logic.mode=msg.data;self.logic.mode_received=time.monotonic()
    def lease(self,msg):self.logic.lease=time.monotonic()

    def cmd(self,msg):
        if not all(math.isfinite(v) for v in (msg.linear.x,msg.linear.y,msg.angular.z)):
            if self.logic.armed:self.logic.fault='nonfinite_command'
            return
        self.command=msg;self.logic.command=time.monotonic()
        if self.logic.evaluate(time.monotonic())=='armed':self.output.publish(msg)

    def arm(self,req,res):
        if not req.data:
            if self.logic.armed:
                self.logic.fault='explicit_disarm';self.release_at=time.monotonic()+1.
                self.output.publish(Twist())
            res.success=True;res.message='zero relay for 1s then disarmed';return res
        try:
            others=[p.node_name for p in self.get_publishers_info_by_topic('/nav/cmd_vel') if p.node_name!='command_gate' or p.node_namespace!='/tracking']
            if others:raise ValueError('competing_chassis_publishers:'+str(others))
            self.logic.arm(time.monotonic());self.release_at=None
            res.success=True;res.message='armed_by_explicit_manual_execute'
        except ValueError as e:res.success=False;res.message=str(e)
        return res

    def tick(self):
        now=time.monotonic()
        if self.logic.armed and now-self.last_graph_check>.2:
            self.last_graph_check=now
            for topic,node in [('/nav/cmd_vel','command_gate'),('/tracking/state','mocap_feedback'),('/tracking/tf','mocap_feedback')]:
                publishers=self.get_publishers_info_by_topic(topic)
                if len(publishers)!=1 or publishers[0].node_name!=node or publishers[0].node_namespace!='/tracking':
                    self.logic.fault='publisher_changed:'+topic
        if self.release_at and now>=self.release_at:self.logic.armed=False;self.release_at=None
        reason=self.logic.evaluate(now)
        if self.logic.armed and reason!='armed':self.output.publish(Twist())
        self.status.publish(String(data=json.dumps(dict(ros_ns=self.get_clock().now().nanoseconds,
            armed=self.logic.armed,reason=reason,changed=reason!=self.previous))))
        self.previous=reason


def main():
    p=argparse.ArgumentParser();p.add_argument('--session',required=True);args,ros=p.parse_known_args()
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=ros,signal_handler_options=SignalHandlerOptions.NO);node=Gate(args.session)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN)
        if node.logic.armed:
            for _ in range(5):node.output.publish(Twist());rclpy.spin_once(node,timeout_sec=.01)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
