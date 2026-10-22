#!/usr/bin/env python3
"""Isolated ROS relay test; refuses the real robot domain."""
import os
if os.environ.get('ROS_LOCALHOST_ONLY')!='1' or not 180<=int(os.environ.get('ROS_DOMAIN_ID','0'))<=220:
    raise SystemExit('Requires isolated localhost ROS domain 180..220')
import json
from pathlib import Path
import tempfile
import time
from auto_2m_trial import OperatorGuard
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile,DurabilityPolicy
from geometry_msgs.msg import Twist,TwistStamped,PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String,Bool
from localization_ab_test.qos import latest_value_qos


def main():
    rclpy.init();executor=SingleThreadedExecutor()
    with tempfile.TemporaryDirectory(prefix='auto2m_guard_fixture_') as folder:
        p=Path(folder);meta={'data_kind':'software_fixture','run':'fixture_run','localization_session':'fixture_restart'}
        (p/'manifest.json').write_text(json.dumps(meta))
        guard=OperatorGuard(p);driver=rclpy.create_node('two_meter_fixture');teleop=rclpy.create_node('teleop_node');native=rclpy.create_node('anchored_fastlio')
        for n in [guard,driver,teleop,native]:executor.add_node(n)
        request=driver.create_publisher(TwistStamped,'/localization_ab/request',latest_value_qos())
        permit=driver.create_publisher(Bool,'/localization_ab/recorder_ready',latest_value_qos())
        mocap=driver.create_publisher(PoseStamped,'/mocap/Tracker0/pose',10)
        mode=teleop.create_publisher(String,'/teleop/mode',10)
        odom=native.create_publisher(Odometry,'/anchored_localization/native_odom',10)
        status=native.create_publisher(String,'/anchored_localization/status',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        observed=[];driver.create_subscription(Twist,'/nav/cmd_vel',lambda m:observed.append((time.monotonic(),m.linear.x,m.linear.y,m.angular.z)),10)
        start=time.monotonic();takeover=None
        while time.monotonic()-start<8:
            now=time.monotonic();stamp=driver.get_clock().now().to_msg()
            velocity=.15 if now-start>3.5 else 0.
            if any(abs(v[1]-.15)<1e-9 for v in observed) and takeover is None:takeover=now
            manual=takeover is not None and now-takeover<.6
            mode.publish(String(data='MANUAL' if manual else 'EXTERNAL'))
            req=TwistStamped();req.header.stamp=stamp;req.header.frame_id='base_link';req.twist.linear.x=velocity;request.publish(req)
            permit.publish(Bool(data=True))
            pose=PoseStamped();pose.header.stamp=stamp;pose.header.frame_id='mocap_world';pose.pose.orientation.w=1.;mocap.publish(pose)
            m=Odometry();m.header.stamp=stamp;m.header.frame_id='camera_init';m.child_frame_id='body';m.pose.pose.orientation.w=1.;odom.publish(m)
            status.publish(String(data=json.dumps(dict(run_path='fixture_run',capture_dir='fixture_restart',frozen=True,fault=None))))
            until=time.monotonic()+.015
            while time.monotonic()<until:executor.spin_once(timeout_sec=.002)
            if takeover is not None and now-takeover>1.2:break
        assert takeover is not None,guard.failure
        assert guard.failure.startswith('operator_took_control:MANUAL'),guard.failure
        assert observed[-1][1:]==(0.,0.,0.)
        assert all(v[1:]==(0.,0.,0.) for v in observed if v[0]>takeover+.1)
        assert [x.node_name for x in driver.get_publishers_info_by_topic('/teleop/mode')]==['teleop_node']
        result={'data_kind':'software_fixture','domain':os.environ['ROS_DOMAIN_ID'],'passed':True,'positive_request_forwarded':True,
                'manual_takeover_stopped':True,'no_automatic_resume':True,'no_mode_publisher_created':True,'guard_failure_expected':guard.failure}
        target=Path(__file__).resolve().parents[1]/'.validation/auto2m_guard_result.json';target.parent.mkdir(exist_ok=True);target.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
        for n in [guard,driver,teleop,native]:executor.remove_node(n);n.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':main()
