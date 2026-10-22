#!/usr/bin/env python3
"""Pause the original executor, use its original correction CLI, then resume."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from action_msgs.msg import GoalStatusArray
from std_srvs.srv import Trigger
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial',type=Path,required=True)
    parser.add_argument('--already-paused',action='store_true',help='Recover an original route already paused without applying any correction')
    args=parser.parse_args();data=args.trial.resolve()/'data'
    meta=json.loads((data/'manifest.json').read_text());run=Path(meta['run'])
    script=Path(__file__).resolve().parents[2]/'mapping_modes.sh'
    rclpy.init();node=rclpy.create_node('group3_middle_checkpoint')
    state={};report={'outcome':'waiting_for_middle','original_controller_used':True,'original_correction_used':True}
    retained=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    latest=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(String,'/teleop/mode',lambda m:state.update(mode=m.data),latest)
    node.create_subscription(Twist,'/reference_velocity',lambda m:state.update(command=max(abs(m.linear.x),abs(m.linear.y),abs(m.angular.z))),latest)
    node.create_subscription(String,'/anchored_localization/status',lambda m:state.update(localization=json.loads(m.data)),retained)
    def goal(m):
        if m.status_list:
            g=max(m.status_list,key=lambda g:g.goal_info.stamp.sec*10**9+g.goal_info.stamp.nanosec)
            state['goal_status']=g.status
    node.create_subscription(GoalStatusArray,'/follow_path/_action/status',goal,retained)
    def mocap(m):
        q=m.pose.orientation
        state.update(mocap_ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec,z_up=1-2*(q.x*q.x+q.y*q.y))
    node.create_subscription(PoseStamped,'/mocap/Tracker0/pose',mocap,latest)
    clients={name:node.create_client(Trigger,'/inspection_route/'+name) for name in ('pause','resume','cancel')}
    def save():
        target=data/'middle_correction.json';temp=target.with_suffix('.tmp')
        temp.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');temp.replace(target)
    def spin():rclpy.spin_once(node,timeout_sec=.01);time.sleep(.002)
    def wait(condition,seconds,reason):
        until=time.monotonic()+seconds
        while time.monotonic()<until:
            spin()
            if condition():return
        raise RuntimeError(reason)
    def call(name):
        client=clients[name]
        if not client.wait_for_service(timeout_sec=3):raise RuntimeError('missing_'+name+'_service')
        future=client.call_async(Trigger.Request());rclpy.spin_until_future_complete(node,future,timeout_sec=5)
        if not future.done() or future.exception() or not future.result().success:
            raise RuntimeError(name+'_rejected:'+str(future.result() if future.done() else 'timeout'))
        return future.result().message
    def command(action,extra,timeout):
        child=subprocess.Popen([str(script),'anchored',action,'--run',str(run),*extra],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True)
        until=time.monotonic()+timeout
        while child.poll() is None and time.monotonic()<until:spin()
        if child.poll() is None:
            os.killpg(child.pid,signal.SIGTERM)
            try:child.wait(timeout=3)
            except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
            raise RuntimeError(action+'_timeout')
        if child.returncode:raise RuntimeError(action+'_failed_'+str(child.returncode))
    def validate(label):
        before=set((run/'validation').iterdir())
        command('validate',['--context','localization','--session',meta['localization_session']],25)
        captures=sorted(set((run/'validation').iterdir())-before)
        valid=[p for p in captures if (p/'summary.json').exists()]
        if len(valid)!=1:raise RuntimeError('expected_one_'+label+'_validation')
        report[label]=str(valid[0]);save()
        return json.loads((valid[0]/'summary.json').read_text())
    paused=False
    try:
        if args.already_paused:
            def recovered_static_reference():
                now=node.get_clock().now().nanoseconds
                return (state.get('goal_status')==5 and state.get('mode')=='EXTERNAL' and
                        state.get('command',1)<1e-5 and state.get('z_up',-1)>math.cos(math.radians(45)) and
                        0<=(now-state.get('mocap_ns',0))*1e-9<.20 and
                        0<=(now-state.get('localization',{}).get('native_stamp_ns',0))*1e-9<meta.get('maximum_native_age_s',.25))
            wait(recovered_static_reference,30,'paused_reference_not_recovered')
            request=dict(pose_ns=state['mocap_ns'],receive_ros_ns=node.get_clock().now().nanoseconds,
                         trigger_source='fresh_mocap_at_already_paused_original_route')
            (data/'middle_pause_request.json').write_text(json.dumps(request)+'\n')
        wait(lambda:(data/'middle_pause_request.json').exists() or (data/'navigation_terminal.json').exists(),800,'middle_not_reached')
        if not (data/'middle_pause_request.json').exists():return
        request=json.loads((data/'middle_pause_request.json').read_text())
        if (node.get_clock().now().nanoseconds-request['pose_ns'])*1e-9>.25:raise RuntimeError('stale_middle_request')
        report.update(outcome='pausing',request=request,pause_ros_ns=node.get_clock().now().nanoseconds);save()
        report['pause_response']='Original route already paused; verified canceled goal and fresh zero command' if args.already_paused else call('pause');paused=True
        wait(lambda:state.get('goal_status')==5 and state.get('command',1)<1e-5,10,'navigation_did_not_pause')
        try:before=validate('before_validation')
        except RuntimeError as exc:
            if not meta.get('continue_despite_native_delay'):raise
            report['before_validation_missing_reason']=str(exc);before=None;save()
        if (before and before['orientation_error_deg']['rmse']>45) or state.get('z_up',-1)<math.cos(math.radians(45)):
            raise RuntimeError('middle_reference_not_upright')
        report['outcome']='correcting';save()
        command('correct',['--session',meta['localization_session'],'--checkpoint','middle'],50)
        wait(lambda:state.get('localization',{}).get('last_checkpoint',{}).get('outcome')=='applied' and state['localization'].get('map_odom_version')==1,5,'correction_not_acknowledged')
        report.update(correction_capture=state['localization']['last_checkpoint']['capture_dir'],map_odom_version=1,correction_applied_ros_ns=node.get_clock().now().nanoseconds)
        save()
        try:validate('after_validation')
        except RuntimeError as exc:report['after_validation_missing_reason']=str(exc);save()
        if state.get('mode')!='EXTERNAL' or state.get('command',1)>1e-5:raise RuntimeError('need_stationary_external_before_resume')
        report['resume_response']=call('resume');paused=False
        wait(lambda:state.get('goal_status') in (1,2),8,'resume_goal_not_accepted')
        report.update(outcome='applied_and_resumed',resume_ros_ns=node.get_clock().now().nanoseconds);save()
    except BaseException as exc:
        report.update(outcome='failed',reason=str(exc),paused=paused,ros_ns=node.get_clock().now().nanoseconds);save()
        print(json.dumps(report,ensure_ascii=False),flush=True)
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
