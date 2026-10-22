#!/usr/bin/env python3
"""Isolated ROS integration exercise, never the hardware ROS domain.

Requires an explicit unused ROS_DOMAIN_ID >= 100. Generates software_fixture data.
No chassis launch, no CAN access, no normal-domain discovery or messages.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import yaml


def fixture():
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped,Twist
    from std_msgs.msg import String
    from std_srvs.srv import SetBool
    from diagnostic_msgs.msg import DiagnosticArray,DiagnosticStatus,KeyValue
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float32MultiArray
    import math
    rclpy.init();node=Node('software_fixture');x=0.;y=0.;yaw=0.;cmd=Twist();drop=False;last=time.monotonic()
    raw=node.create_publisher(PoseStamped,'/vrpn_mocap/Tracker0/pose',20)
    norm=node.create_publisher(PoseStamped,'/mocap/Tracker0/pose',20)
    diag=node.create_publisher(DiagnosticArray,'/mocap/Tracker0/diagnostics',20)
    mode=node.create_publisher(String,'/teleop/mode',10);final=node.create_publisher(Twist,'/reference_velocity',20)
    joints=node.create_publisher(JointState,'/joint_states',20)
    wheel=node.create_publisher(Float32MultiArray,'/omni_velocity_controller/diagnostics/joint_vel',20)
    def command(msg):
        nonlocal cmd
        cmd=msg
    def dropout(req,res):
        nonlocal drop
        drop=req.data;res.success=True;res.message='software fixture dropout';return res
    def reset(req,res):
        nonlocal x,y,yaw,cmd
        x=y=yaw=0.;cmd=Twist();res.success=True;return res
    node.create_subscription(Twist,'/nav/cmd_vel',command,20)
    node.create_service(SetBool,'/software_fixture/dropout',dropout)
    node.create_service(SetBool,'/software_fixture/reset',reset)
    def tick():
        nonlocal x,y,yaw,last
        now=time.monotonic();dt=now-last;last=now
        x+=dt*cmd.linear.x*math.cos(yaw);y+=dt*cmd.linear.x*math.sin(yaw);yaw+=dt*cmd.angular.z
        mode.publish(String(data='EXTERNAL'));final.publish(cmd)
        if not drop:
            p=PoseStamped();p.header.stamp=node.get_clock().now().to_msg();p.header.frame_id='mocap_world'
            p.pose.position.x=x;p.pose.position.y=y;p.pose.position.z=.18
            p.pose.orientation.z=math.sin(yaw/2);p.pose.orientation.w=math.cos(yaw/2);norm.publish(p)
            d=DiagnosticArray();d.header=p.header
            d.status=[DiagnosticStatus(name='mocap_tracker0_normalizer',level=DiagnosticStatus.OK,message='software_valid',values=[KeyValue(key='valid',value='true')])];diag.publish(d)
            p.pose.position.x*=1000;p.pose.position.y*=1000;p.pose.position.z*=1000;raw.publish(p)
        state=JointState();state.header.stamp=node.get_clock().now().to_msg()
        state.name=[f'wheel_{i}_{kind}' for kind in ('turn','roll') for i in range(1,5)]
        state.position=[0.]*4+[x]*4;state.velocity=[0.]*4+[cmd.linear.x]*4;joints.publish(state)
        wheel.publish(Float32MultiArray(data=state.velocity))
    node.create_timer(.01,tick)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()


def run(args):
    from controller_tracking.configuration import load
    root=Path(__file__).resolve().parents[3];out=Path(args.output).resolve();out.mkdir(parents=True,exist_ok=False)
    cfg=load(root/'tests/controller_tracking/config/experiment.yaml')
    cfg['calibration'].update(id='SOFTWARE_FIXTURE_ONLY',level_verified=True,world_level_rotation_xyzw=[0.,0.,0.,1.])
    cfg['site'].update(stable_mocap_bounds=[-10,10,-10,10],clear_floor_bounds=[-10,10,-10,10],marker_visibility_verified=True,stopping_margin_verified=True,physical_stop_verified=True)
    cfg['geometry']['length']=.6;cfg['execution']['timeout_s']=12.
    (out/'fixture_config.yaml').write_text(yaml.safe_dump(cfg));children=[];logs=[]
    def start(cmd,name):
        f=(out/(name+'.log')).open('w');logs.append(f)
        p=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);children.append(p);return p
    cli=[sys.executable,'-m','controller_tracking.cli']
    def call(cmd,name,timeout=30):
        p=start(cmd,name)
        if p.wait(timeout=timeout):raise RuntimeError(name+' failed; see '+str(out/(name+'.log')))
    def stop(p):
        if p.poll() is None:
            os.killpg(p.pid,signal.SIGINT)
            try:p.wait(timeout=15)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
    try:
        import rclpy
        from rclpy.node import Node
        rclpy.init();node=Node('isolated_smoke_guard');time.sleep(.7)
        others=[n for n,ns in node.get_node_names_and_namespaces() if n!='isolated_smoke_guard']
        node.destroy_node();rclpy.shutdown()
        if others:raise RuntimeError('Test domain is not empty: '+str(others))
        start([sys.executable,__file__,'--domain',str(args.domain),'--fixture'],'fixture')
        time.sleep(1.)
        call([*cli,'freeze','--config',str(out/'fixture_config.yaml'),'--output',str(out/'session')],'freeze')
        start(['ros2','launch','controller_tracking','feedback_test.launch.py','session:='+str(out/'session')],'stack')
        time.sleep(5.)
        call([*cli,'check','--session',str(out/'session'),'--output',str(out/'check.json')],'check')
        for number,inject in [(1,False),(2,True)]:
            round=out/f'round_{number:02d}'
            recorder=start([*cli,'record','--session',str(out/'session'),'--kind','line','--run',str(round)],f'record_{number}')
            time.sleep(3.)
            executor=start([*cli,'execute','--run',str(round)],f'execute_{number}')
            if inject:
                deadline=time.monotonic()+15
                while time.monotonic()<deadline:
                    events=round/'events.jsonl'
                    if events.exists() and 'goal_accepted' in events.read_text():break
                    if executor.poll() is not None:raise RuntimeError('executor ended before dropout injection')
                    time.sleep(.05)
                time.sleep(.2)
                call(['ros2','service','call','/software_fixture/dropout','std_srvs/srv/SetBool','{data: true}'],'inject_gap')
            if executor.wait(timeout=25):raise RuntimeError('execute process error')
            stop(recorder)
            meta=json.loads((round/'recording.json').read_text());meta['source']='software_fixture';(round/'recording.json').write_text(json.dumps(meta,indent=2))
            call([*cli,'analyze','--run',str(round)],f'analysis_{number}')
            if not inject:
                # Reset is simulated, not a vehicle-motion command.
                call(['ros2','service','call','/software_fixture/reset','std_srvs/srv/SetBool','{data: true}'],'reset')
                time.sleep(.5)
        executions=[json.loads((out/f'round_{i:02d}/execution.json').read_text()) for i in (1,2)]
        assert executions[0]['success'],executions[0]
        assert not executions[1]['success'] and ('mocap' in executions[1]['reason'] or 'gate:' in executions[1]['reason']),executions[1]
        assert all(e['stop_check']['zero'] for e in executions)
        (out/'result.json').write_text(json.dumps(dict(source='software_fixture',domain=args.domain,passed=True,executions=executions),indent=2))
        print('ISOLATED SOFTWARE CHECK PASSED:',out)
    finally:
        for p in reversed(children):stop(p)
        for f in logs:f.close()


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--domain',type=int,required=True);parser.add_argument('--fixture',action='store_true');parser.add_argument('--output')
    args=parser.parse_args()
    if not 100<=args.domain<=232:parser.error('explicit isolated domain 100..232 required')
    if not args.fixture and args.domain==int(os.environ.get('ROS_DOMAIN_ID','0')):parser.error('refuse current ROS domain')
    if args.fixture and os.environ.get('TRACKING_SOFTWARE_TEST_DOMAIN')!=str(args.domain):parser.error('fixture requires isolated parent guard')
    os.environ['ROS_DOMAIN_ID']=str(args.domain)
    os.environ['TRACKING_SOFTWARE_TEST_DOMAIN']=str(args.domain)
    if args.fixture:fixture()
    elif not args.output:parser.error('--output required')
    else:run(args)
