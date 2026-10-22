#!/usr/bin/env python3
"""Isolated fake Nav2 checks pause cancellation, exact suffix and one report."""
import os
from pathlib import Path
import subprocess
import tempfile
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PointStamped, PoseStamped, TransformStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose, FollowPath
from nav_msgs.msg import Odometry, Path as RosPath
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster


def main():
    assert int(os.environ.get('ROS_DOMAIN_ID','0'))!=0, 'Use an isolated ROS domain'
    rclpy.init();node=rclpy.create_node('pause_resume_fixture');group=ReentrantCallbackGroup()
    executor=MultiThreadedExecutor(num_threads=4);executor.add_node(node)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    state={'x':-.2,'goals':[],'finish':False,'actual':None,'reference':None}
    def pose(x,y=0.):
        p=PoseStamped();p.header.frame_id='map';p.pose.position.x=float(x);p.pose.position.y=float(y);p.pose.orientation.w=1.;return p
    def lifecycle(req,res):res.current_state.id=3;res.current_state.label='active';return res
    node.create_service(GetState,'/controller_server/get_state',lifecycle,callback_group=group)
    def plan(handle):
        result=ComputePathToPose.Result();result.path.header.frame_id='map';result.path.poses=[pose(state['x']),handle.request.goal];handle.succeed();return result
    plan_server=ActionServer(node,ComputePathToPose,'/compute_path_to_pose',execute_callback=plan,callback_group=group)
    def follow(handle):
        state['goals'].append(handle.request.path)
        while rclpy.ok():
            if handle.is_cancel_requested:handle.canceled();return FollowPath.Result()
            if state['finish']:handle.succeed();return FollowPath.Result()
            feedback=FollowPath.Feedback();feedback.distance_to_goal=6.;handle.publish_feedback(feedback);time.sleep(.03)
    follow_server=ActionServer(node,FollowPath,'/follow_path',execute_callback=follow,
        cancel_callback=lambda goal:CancelResponse.ACCEPT,callback_group=group)
    tf=TransformBroadcaster(node);odom=node.create_publisher(Odometry,'/fixture_odom',10)
    def tick():
        t=TransformStamped();t.header.frame_id='map';t.child_frame_id='base_link';t.header.stamp=node.get_clock().now().to_msg();t.transform.translation.x=state['x'];t.transform.rotation.w=1.;tf.sendTransform(t)
        m=Odometry();m.header.stamp=t.header.stamp;odom.publish(m)
    node.create_timer(.04,tick,callback_group=group)
    retained=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    path_pub=node.create_publisher(RosPath,'/inspection_path',retained)
    clicks={name:node.create_publisher(PointStamped,'/inspection_route_'+name,10) for name in ['start','middle','end']}
    node.create_subscription(RosPath,'/inspection_route/actual_path',lambda m:state.update(actual=m),retained)
    node.create_subscription(RosPath,'/inspection_route/reference_path',lambda m:state.update(reference=m),retained)
    def wait(check,reason,seconds=8):
        until=time.monotonic()+seconds
        while time.monotonic()<until:
            if check():return
            time.sleep(.03)
        raise AssertionError(reason)
    def call(name):
        c=node.create_client(Trigger,'/inspection_route/'+name);assert c.wait_for_service(timeout_sec=5)
        f=c.call_async(Trigger.Request());wait(f.done,name+' service');r=f.result();assert r.success,(name,r.message);return r
    def xy(path):return [(p.pose.position.x,p.pose.position.y) for p in path.poses]
    with tempfile.TemporaryDirectory(prefix='inspection_pause_test_') as temp:
        binary=Path(__file__).resolve().parents[3]/'install/inspection_geometry_tools/lib/inspection_geometry_tools/inspection_path_executor_node'
        child=subprocess.Popen([str(binary),'--ros-args','-p','tracking_odom_topic:=/fixture_odom','-p','experiment_output_dir:='+temp],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
        try:
            path=RosPath();path.header.frame_id='map'
            path.poses=[pose(i/10) for i in range(21)]+[pose(2,i/10) for i in range(1,21)]+[pose(2-i/10,2) for i in range(1,21)]+[pose(0,2-i/10) for i in range(1,21)]
            path_pub.publish(path);time.sleep(1.5)
            for name,x,y in [('start',0,0),('middle',2,2),('end',0,0)]:
                msg=PointStamped();msg.header.frame_id='map';msg.point.x=float(x);msg.point.y=float(y);clicks[name].publish(msg)
            time.sleep(1.2);call('follow');wait(lambda:len(state['goals'])==1,'first action')
            original=xy(state['goals'][0]);time.sleep(.25)
            for i in range(9):state['x']=-.2+.1*i;time.sleep(.12)
            time.sleep(.25)
            call('pause');time.sleep(.7)
            assert not list(Path(temp).glob('*')), 'pause must not export a terminal report'
            count=len(state['actual'].poses);state['x']=.7;time.sleep(.3)
            assert len(state['actual'].poses)>count, 'paused measurements must continue'
            call('resume');wait(lambda:len(state['goals'])==2,'resumed action')
            remainder=xy(state['goals'][1]);assert 2<len(remainder)<len(original)
            assert any(original[i:]==remainder for i in range(1,len(original))), 'resume must be an exact original suffix'
            assert xy(state['reference'])==original, 'full evaluation reference must remain unchanged'
            state['finish']=True;wait(lambda:bool(list(Path(temp).glob('*/trajectory.svg'))),'final report')
            assert len(list(Path(temp).glob('*/trajectory.svg')))==1
            print('PASS: cancellation before resume; exact remaining path; continuous actual path; one full-reference report')
        finally:
            state['finish']=True
            child.terminate()
            try:child.wait(timeout=5)
            except subprocess.TimeoutExpired:child.kill();child.wait()
    follow_server.destroy();plan_server.destroy();executor.shutdown();node.destroy_node();rclpy.shutdown();thread.join(timeout=2)


if __name__=='__main__':main()
