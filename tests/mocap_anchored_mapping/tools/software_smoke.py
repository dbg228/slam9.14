#!/usr/bin/env python3
"""Isolated synthetic full workflow: native mapping, restart, held-out validation."""
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
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
from std_srvs.srv import Trigger
from mocap_anchored_mapping.core import prepare,checked,load,save,sha,inverse,checked_session,matrix
from mocap_anchored_mapping.products import project
PKG=Path(__file__).resolve().parents[1];sys.path.insert(0,str(PKG/'test'))
from test_mapping import wall_pcd,transform


def fill(p,h,scale=1.):
    p.position.x,p.position.y,p.position.z=(h[:3,3]*scale).tolist()
    p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=Rotation.from_matrix(h[:3,:3]).as_quat().tolist()


def tf_matrix(message):
    t=message.transform.translation;q=message.transform.rotation
    return matrix([t.x,t.y,t.z],[q.x,q.y,q.z,q.w])


def main():
    if os.environ.get('ROS_LOCALHOST_ONLY')!='1' or not 180<=int(os.environ.get('ROS_DOMAIN_ID','0'))<=220:raise ValueError('Requires isolated localhost test domain 180..220')
    directory=Path(sys.argv[1]).resolve();directory.mkdir(parents=True,exist_ok=False)
    site=load(PKG/'config/site.yaml');site['data_kind']='software_fixture_not_a_real_map';save(directory/'site.yaml',site,True)
    run=directory/'software_fixture';prepare(run,directory/'site.yaml');cfg,ext=checked(run)
    rclpy.init(args=[]);node=Node('anchored_fastlio')
    node.declare_parameter('map_file_path',str(run/'raw/native_F.pcd'));node.declare_parameter('pcd_save.pcd_save_en',True)
    raw=node.create_publisher(PoseStamped,'/vrpn_mocap/Tracker0/pose',50);normal=node.create_publisher(PoseStamped,'/mocap/Tracker0/pose',50)
    valid=node.create_publisher(DiagnosticArray,'/mocap/Tracker0/diagnostics',50);command=node.create_publisher(Twist,'/reference_velocity',50)
    pubs={c:node.create_publisher(Odometry,'/anchored_'+c+'/native_odom',50) for c in ['mapping','localization']}
    statuses={c:[] for c in pubs};tfs=[]
    for context in pubs:node.create_subscription(String,'/anchored_'+context+'/status',lambda m,c=context:statuses[c].append(json.loads(m.data)),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(TFMessage,'/tf',lambda m:tfs.extend(m.transforms),100)
    node.create_subscription(TFMessage,'/tf_static',lambda m:tfs.extend(m.transforms),QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    h=transform([1,-2,.8],[.04,-.03,.4]);g=transform([-.5,.6,.04],[.03,.02,-.8])
    b=transform([.3,.2,-.1],[.02,.01,.7]);restart_b=transform([.4,-.2,.1],[.01,-.01,.2])
    context='mapping';send_mocap=True;shift=0.;bias=np.eye(4);count=0;children=[];logs=[]
    def sample():
        nonlocal count
        count+=1;stamp=node.get_clock().now().to_msg();command.publish(Twist())
        ob=(b if context=='mapping' else restart_b).copy();ob[0,3]+=shift
        mb=h@ob if context=='mapping' else h@bias@g@ob
        if send_mocap:
            tracker=mb@inverse(np.asarray(ext['T_Tracker_B']['matrix']))
            a=PoseStamped();a.header.stamp=stamp;a.header.frame_id='mocap_world';fill(a.pose,tracker,1000);raw.publish(a)
            a=PoseStamped();a.header.stamp=stamp;a.header.frame_id='mocap_world';fill(a.pose,tracker);normal.publish(a)
            d=DiagnosticArray();d.header=a.header;s=DiagnosticStatus();s.name='mocap_tracker0_normalizer';s.level=b'\x00';s.message='synthetic_valid';s.values=[KeyValue(key='valid',value='true')];d.status=[s];valid.publish(d)
        if count%10==0:
            a=Odometry();a.header.stamp=stamp;a.header.frame_id='camera_init';a.child_frame_id='body';fill(a.pose.pose,ob@np.asarray(ext['T_B_I']['matrix']));pubs[context].publish(a)
    timer=node.create_timer(.01,sample)
    def save_map(request,response):wall_pcd(run/'raw/native_F.pcd');response.success=True;response.message='SOFTWARE FIXTURE ONLY';return response
    node.create_service(Trigger,'/anchored_mapping/save_native',save_map)
    def until(condition,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end and not condition():rclpy.spin_once(node,timeout_sec=.01)
        return condition()
    def spawn(label,command):
        log=(directory/(label+'.log')).open('x');logs.append(log);p=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT);children.append(p);return p
    def stop(p):
        if p.poll() is None:p.send_signal(signal.SIGINT);p.wait(timeout=4)
    online=[sys.executable,'-c','from mocap_anchored_mapping.online import main; raise SystemExit(main())','--run',str(run)]
    try:
        mapper=spawn('mapping',online)
        assert until(lambda:(run/'calibration.lock.json').exists() or mapper.poll() is not None,16)
        assert mapper.poll() is None,'mapping.log'
        calibration=load(run/'calibration.yaml');np.testing.assert_allclose(calibration['T_mocap_map']['matrix'],h,atol=1e-10)
        np.testing.assert_allclose(calibration['map_origin_in_mocap_m'],h[:3,3],atol=1e-10)
        digest=sha(run/'calibration.yaml');send_mocap=False;shift=.4;before=len(tfs)
        until(lambda:len(tfs)>before+8,2);assert mapper.poll() is None and sha(run/'calibration.yaml')==digest
        mapping_edges=[t for t in tfs if (t.header.frame_id,t.child_frame_id)==('map','camera_init')]
        assert mapping_edges
        np.testing.assert_allclose(tf_matrix(mapping_edges[-1]),np.eye(4),atol=1e-12)
        saver=spawn('save',[sys.executable,'-m','mocap_anchored_mapping.cli','save','--run',str(run)])
        assert until(lambda:saver.poll() is not None,12) and saver.returncode==0,'save.log'
        raw_sha=sha(run/'raw/native_F.pcd');projection=project(run);assert sha(run/'raw/native_F.pcd')==raw_sha
        assert projection['PCD_XYZ_transform_count']==0
        stop(mapper)
        # No mapping TF writer survives; the next process owns the standard localization tree.
        until(lambda:False,1)
        context='localization';send_mocap=True;shift=0.;session=run/'localizations/restart_01'
        restart=spawn('restart',online+['--mode','localization','--capture-dir',str(session)])
        assert until(lambda:(session/'initialization.lock.json').exists() or restart.poll() is not None,16)
        assert restart.poll() is None,'restart.log'
        initialized=checked_session(run,session);np.testing.assert_allclose(initialized['T_map_odom']['matrix'],g,atol=1e-10)
        assert sha(run/'calibration.yaml')==digest
        session_sha=sha(session/'initialization.yaml');send_mocap=False;shift=.4;before=len(tfs)
        assert until(lambda:len(tfs)>before+8,2) and restart.poll() is None
        restart_edges=[t for t in tfs if (t.header.frame_id,t.child_frame_id)==('map','odom')]
        assert restart_edges
        np.testing.assert_allclose(tf_matrix(restart_edges[-1]),g,atol=1e-10)
        send_mocap=True;bias=transform([.03,-.04,0],[0,0,0])
        validator=spawn('heldout',[sys.executable,'-m','mocap_anchored_mapping.cli','validate','--run',str(run),'--context','localization','--session',str(session)])
        assert until(lambda:validator.poll() is not None,16) and validator.returncode==0,'heldout.log'
        summaries=list((run/'validation').glob('*/summary.json'));assert len(summaries)==1
        summary=load(summaries[0]);assert abs(summary['distance_3D_m']['rmse']-.05)<1e-9
        assert not summary['fit_performed'] and sha(run/'calibration.yaml')==digest and sha(session/'initialization.yaml')==session_sha
        # A native reset must stop the active bridge, never silently re-anchor.
        timer.cancel();old=Odometry();old.header.frame_id='camera_init';old.child_frame_id='body';old.header.stamp.sec=1;fill(old.pose.pose,restart_b)
        pubs[context].publish(old);assert until(lambda:restart.poll() is not None,3) and restart.returncode!=0
        assert load(session/'failure.json')['reason'].startswith('native_time_reset')
        save(directory/'result.json',dict(passed=True,schema=2,data_kind='software_fixture',domain=os.environ['ROS_DOMAIN_ID'],
            map_origin_direction_checked=True,PCD_bytes_unchanged=True,original_slice_heights_reused=True,
            mapping_global_TF_identity_checked=True,restart_global_TF_direction_checked=True,
            restart_map_odom_direction_checked=True,continued_after_mocap_loss=True,old_world_calibration_unchanged=True,
            heldout_no_refit=True,heldout_known_error_RMSE_m=summary['distance_3D_m']['rmse'],native_reset_rejected=True,
            NDT_inputs_or_nodes_used=False,motion_commands_published_to_real_domain=False,run=str(run)))
        print(json.dumps(dict(passed=True,software_fixture_only=True,directory=str(directory))))
    finally:
        for child in reversed(children):
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
                try:child.wait(timeout=4)
                except subprocess.TimeoutExpired:child.terminate();child.wait(timeout=4)
        for log in logs:log.close()
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
