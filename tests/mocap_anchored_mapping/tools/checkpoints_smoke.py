#!/usr/bin/env python3
"""Synthetic group-3 integration in an isolated localhost ROS domain only."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import PoseStamped,Twist
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray,DiagnosticStatus,KeyValue
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from mocap_anchored_mapping.core import prepare,checked,load,save,sha,inverse,checked_session

PKG=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(PKG/'test'))
from test_mapping import wall_pcd,transform,fixture_calibration


def fill(p,h,scale=1.):
    p.position.x,p.position.y,p.position.z=(h[:3,3]*scale).tolist()
    p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=Rotation.from_matrix(h[:3,:3]).as_quat().tolist()


def main():
    if os.environ.get('ROS_LOCALHOST_ONLY')!='1' or not 180<=int(os.environ.get('ROS_DOMAIN_ID','0'))<=220:
        raise ValueError('Requires isolated localhost domain 180..220')
    directory=Path(sys.argv[1]).resolve();directory.mkdir(parents=True,exist_ok=False)
    cfg=load(PKG/'config/site.yaml');cfg['data_kind']='software_fixture_checkpoint_only'
    cfg['initialization']['timeout_s']=8.
    save(directory/'site.yaml',cfg,True)
    run=directory/'software_fixture';prepare(run,directory/'site.yaml');cfg,ext=checked(run)
    world=transform([4,-3,.8],[.04,-.03,1.2]);odom=transform([.6,-.4,.1],[.01,-.02,.7])
    body=transform([.3,.8,.1],[.01,.02,-.3]);bias=np.eye(4)
    wall_pcd(run/'raw/native_F.pcd');fixture_calibration(run,world)
    calibration_sha=sha(run/'calibration.yaml');pcd_sha=sha(run/'raw/native_F.pcd')
    session=run/'localizations/checkpoint_session'
    rclpy.init(args=[]);node=Node('anchored_fastlio')
    raw=node.create_publisher(PoseStamped,'/vrpn_mocap/Tracker0/pose',50)
    normal=node.create_publisher(PoseStamped,'/mocap/Tracker0/pose',50)
    valid=node.create_publisher(DiagnosticArray,'/mocap/Tracker0/diagnostics',50)
    command=node.create_publisher(Twist,'/reference_velocity',50)
    native=node.create_publisher(Odometry,'/anchored_localization/native_odom',50)
    statuses=[];dynamic=[];static=[];children=[];logs=[];send_mocap=True;count=0;command_speed=0.
    node.create_subscription(String,'/anchored_localization/status',lambda msg:statuses.append(json.loads(msg.data)),
        QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(TFMessage,'/tf',lambda msg:dynamic.extend(msg.transforms),100)
    node.create_subscription(TFMessage,'/tf_static',lambda msg:static.extend(msg.transforms),
        QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def sample():
        nonlocal count
        count+=1;stamp=node.get_clock().now().to_msg()
        cmd=Twist();cmd.linear.x=command_speed;command.publish(cmd)
        if send_mocap:
            tracker=world@bias@odom@body@inverse(np.asarray(ext['T_Tracker_B']['matrix']))
            for pub,scale in [(raw,1000.),(normal,1.)]:
                p=PoseStamped();p.header.stamp=stamp;p.header.frame_id='mocap_world';fill(p.pose,tracker,scale);pub.publish(p)
            d=DiagnosticArray();d.header=p.header
            s=DiagnosticStatus();s.name='mocap_tracker0_normalizer';s.level=b'\x00';s.message='synthetic_valid'
            s.values=[KeyValue(key='valid',value='true')];d.status=[s];valid.publish(d)
        if count%10==0:
            p=Odometry();p.header.stamp=stamp;p.header.frame_id='camera_init';p.child_frame_id='body'
            fill(p.pose.pose,body@np.asarray(ext['T_B_I']['matrix']));native.publish(p)
    node.create_timer(.01,sample)
    def until(condition,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end and not condition():rclpy.spin_once(node,timeout_sec=.01)
        return condition()
    def spawn(label,args):
        log=(directory/(label+'.log')).open('x');logs.append(log)
        p=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT);children.append(p);return p
    def request(label,name):
        return spawn(label,[sys.executable,'-m','mocap_anchored_mapping.cli','correct','--run',str(run),'--session',str(session),'--checkpoint',name])
    try:
        bridge=spawn('bridge',[sys.executable,'-c','from mocap_anchored_mapping.online import main; raise SystemExit(main())','--run',str(run),'--mode','checkpoints','--capture-dir',str(session)])
        assert until(lambda:(session/'initialization.lock.json').exists() or bridge.poll() is not None,16)
        assert bridge.poll() is None,'bridge.log'
        initial=checked_session(run,session)
        assert initial['checkpoint_corrections'] and not initial['initialization_only']
        np.testing.assert_allclose(initial['T_map_odom']['matrix'],odom,atol=1e-10)
        initial_sha=sha(session/'initialization.yaml')
        rejected=request('end_before_middle','end')
        assert until(lambda:rejected.poll() is not None,6) and rejected.returncode!=0
        assert 'Expected checkpoint middle' in (directory/'end_before_middle.log').read_text()
        send_mocap=False;before=len(dynamic)
        assert until(lambda:len(dynamic)>before+8,3) and bridge.poll() is None
        send_mocap=True;bias=transform([.03,-.04,0],[0,0,0])
        command_speed=.1;until(lambda:False,.4)
        rejected=request('moving_rejected','middle')
        assert until(lambda:rejected.poll() is not None,6) and rejected.returncode!=0
        assert 'fresh zero chassis command' in (directory/'moving_rejected.log').read_text()
        command_speed=0.;until(lambda:False,.4)
        middle=request('middle','middle')
        assert until(lambda:middle.poll() is not None,14) and middle.returncode==0,'middle.log'
        assert until(lambda:statuses and statuses[-1]['map_odom_version']==1,2)
        middle_report=load(Path(statuses[-1]['last_checkpoint']['capture_dir'])/'summary.json')
        assert abs(middle_report['before']['distance_XY_m']['rmse']-.05)<1e-9
        assert middle_report['after_same_window_consistency']['distance_XY_m']['rmse']<1e-9
        np.testing.assert_allclose(statuses[-1]['T_map_odom']['matrix'],bias@odom,atol=1e-10)
        # A later window must use the active correction, never the initial map->odom.
        bias=transform([.042,-.024,0],[0,0,0])
        heldout=spawn('heldout',[sys.executable,'-m','mocap_anchored_mapping.cli','validate','--run',str(run),
            '--context','localization','--session',str(session)])
        assert until(lambda:heldout.poll() is not None,14) and heldout.returncode==0,'heldout.log'
        heldout_summary=load(next((run/'validation').glob('*/summary.json')))
        assert abs(heldout_summary['distance_XY_m']['rmse']-.02)<1e-9
        assert heldout_summary['map_odom_version']==1 and heldout_summary['held_out_from_checkpoint_fit']
        send_mocap=False;before=len(dynamic)
        assert until(lambda:len(dynamic)>before+8,3)
        retained=np.asarray(statuses[-1]['T_map_odom']['matrix'])
        failed=request('end_without_mocap','end')
        assert until(lambda:failed.poll() is not None,14) and failed.returncode!=0
        np.testing.assert_allclose(statuses[-1]['T_map_odom']['matrix'],retained,atol=1e-10)
        assert bridge.poll() is None and statuses[-1]['completed_checkpoints']==['middle']
        send_mocap=True;bias=transform([-.06,.08,0],[0,0,0]);until(lambda:False,.4)
        end=request('end_retry','end')
        assert until(lambda:end.poll() is not None,14) and end.returncode==0,'end_retry.log'
        assert until(lambda:statuses and statuses[-1]['map_odom_version']==2,2)
        report=load(Path(statuses[-1]['last_checkpoint']['capture_dir'])/'summary.json')
        assert abs(report['before']['distance_XY_m']['rmse']-.15)<1e-9
        assert statuses[-1]['completed_checkpoints']==['middle','end']
        assert not any(t.header.frame_id=='map' and t.child_frame_id=='odom' for t in static)
        assert any(t.header.frame_id=='map' and t.child_frame_id=='odom' for t in dynamic)
        assert sha(run/'calibration.yaml')==calibration_sha and sha(run/'raw/native_F.pcd')==pcd_sha
        assert sha(session/'initialization.yaml')==initial_sha
        result=dict(passed=True,data_kind='software_fixture',domain=os.environ['ROS_DOMAIN_ID'],
            checkpoint_order_enforced=True,moving_request_rejected=True,
            correction_held_outside_mocap=True,timeout_kept_previous_transform=True,
            retry_after_failed_capture=True,map_odom_dynamic_only=True,
            middle_before_XY_m=.05,end_before_XY_m=.15,
            heldout_current_correction_known_error_m=.02,
            old_calibration_PCD_and_initialization_unchanged=True,no_real_robot_commands=True)
        save(directory/'result.json',result);print(json.dumps(result))
    finally:
        for child in reversed(children):
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
                try:child.wait(timeout=4)
                except subprocess.TimeoutExpired:child.terminate();child.wait(timeout=4)
        for log in logs:log.close()
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
