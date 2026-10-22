#!/usr/bin/env python3
"""Passive trial progress and original endpoint validation; never command motion."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import time
import math
import yaml

import rclpy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data

def ns(stamp): return stamp.sec * 10**9 + stamp.nanosec

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial',type=Path,required=True)
    parser.add_argument('--middle-static',action='store_true')
    parser.add_argument('--middle-correction',action='store_true')
    args=parser.parse_args();data=args.trial.resolve()/'data'
    meta=json.loads((data/'manifest.json').read_text());run=Path(meta['run'])
    session=Path(meta['localization_session']);begin=None;validation=None;before=set();recorded=False
    rclpy.init();node=rclpy.create_node('group2_trial_passive_observer')
    state={'route':'','mode':None,'terminal':None};subs=[];origin=None
    if args.middle_correction:
        initial=yaml.safe_load((session/'initialization.yaml').read_text())
        origin=initial['static_spread']['T_M_B']['mean_xyz_m'][:2]
        if (data/'middle_correction.json').exists():
            previous=json.loads((data/'middle_correction.json').read_text())
            state['middle_requested']=bool(previous.get('correction_capture'))
    planned_target=meta.get('middle_hold_target_map_xy')
    fixed_map_odom=yaml.safe_load((session/'initialization.yaml').read_text())['T_map_odom']['matrix'] if planned_target else None
    (data/'observer_pid.json').write_text(json.dumps({'pid':os.getpid(),'script':str(Path(__file__).resolve())})+'\n')
    def action(msg):
        if begin is None or not msg.status_list:return
        latest=max(msg.status_list,key=lambda s:ns(s.goal_info.stamp))
        if ns(latest.goal_info.stamp)<begin:return
        state.setdefault('first_goal_stamp_ns',ns(latest.goal_info.stamp))
        state['action_status']=int(latest.status)
        if args.middle_correction and latest.status==5 and ns(latest.goal_info.stamp)==state['first_goal_stamp_ns'] and (data/'middle_pause_request.json').exists():
            state['middle_goal_canceled_for_correction']=True
            return
        if latest.status in (4,5,6) and state['terminal'] is None:
            state['terminal']={'status':int(latest.status),'receive_ros_ns':node.get_clock().now().nanoseconds}
            (data/'navigation_terminal.json').write_text(json.dumps(state['terminal'],indent=2)+'\n')
            print('NAVIGATION_TERMINAL',json.dumps(state['terminal']),flush=True)
    def command(msg):
        now=node.get_clock().now().nanoseconds
        if max(abs(msg.linear.x),abs(msg.linear.y),abs(msg.angular.z))>1e-5:state['last_motion_ns']=now
        elif state['terminal'] and 'first_zero_after_terminal_ns' not in state:state['first_zero_after_terminal_ns']=now
    retained=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    latest=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
    subs.append(node.create_subscription(Odometry,'/anchored_localization/native_odom',lambda m:state.update(native_stamp_ns=ns(m.header.stamp)),qos_profile_sensor_data))
    def mocap(msg):
        nonlocal origin
        stamp=ns(msg.header.stamp);state['last_mocap_ns']=stamp
        p=msg.pose.position;q=msg.pose.orientation
        if origin is None:origin=(p.x,p.y)
        upright=1-2*(q.x*q.x+q.y*q.y)>math.cos(math.radians(45))
        state['mocap_upright']=upright
        if (args.middle_static or args.middle_correction) and begin and not state.get('middle_requested') and not state['terminal'] and state['mode']=='EXTERNAL' and (stamp-begin)*1e-9>30 and math.hypot(p.x-origin[0],p.y-origin[1])>2 and upright and 0<=(node.get_clock().now().nanoseconds-stamp)*1e-9<.20:
            (data/'middle_pause_request.json').write_text(json.dumps(dict(pose_ns=stamp,receive_ros_ns=node.get_clock().now().nanoseconds))+'\n');state['middle_requested']=True
    subs.append(node.create_subscription(PoseStamped,'/mocap/Tracker0/pose',mocap,qos_profile_sensor_data))
    def planned_middle(msg):
        stamp=ns(msg.header.stamp)
        if not begin or state.get('middle_requested') or state['terminal'] or state['mode']!='EXTERNAL' or (stamp-begin)*1e-9<30:return
        if not 0<=(node.get_clock().now().nanoseconds-stamp)*1e-9<.20:return
        if args.middle_correction and (not state.get('mocap_upright') or not 0<=(node.get_clock().now().nanoseconds-state.get('last_mocap_ns',0))*1e-9<.20):return
        p=msg.pose.position;xyz=[p.x,p.y,p.z]
        xy=[sum(fixed_map_odom[i][j]*xyz[j] for j in range(3))+fixed_map_odom[i][3] for i in (0,1)]
        if math.hypot(xy[0]-planned_target[0],xy[1]-planned_target[1])<=meta.get('middle_hold_radius_m',.17):
            (data/'middle_pause_request.json').write_text(json.dumps(dict(pose_ns=stamp,receive_ros_ns=node.get_clock().now().nanoseconds,trigger_source='planned_coverage_location_native_base_pose',map_xy=xy))+'\n');state['middle_requested']=True
    if (args.middle_static or args.middle_correction) and planned_target:subs.append(node.create_subscription(PoseStamped,'/anchored_localization/base_pose_F',planned_middle,qos_profile_sensor_data))
    subs.append(node.create_subscription(String,'/inspection_route/status',lambda m:state.update(route=m.data),retained))
    subs.append(node.create_subscription(String,'/teleop/mode',lambda m:state.update(mode=m.data),latest))
    subs.append(node.create_subscription(GoalStatusArray,'/follow_path/_action/status',action,retained))
    subs.append(node.create_subscription(Twist,'/reference_velocity',command,latest))
    start=time.monotonic();next_update=start
    while time.monotonic()-start<1200:
        rclpy.spin_once(node,timeout_sec=.05);now=node.get_clock().now().nanoseconds
        if begin is None and (data/'departure.json').exists():begin=json.loads((data/'departure.json').read_text())['request_ros_ns']
        if args.middle_correction and state['terminal'] is None and (data/'middle_correction.json').exists():
            correction=json.loads((data/'middle_correction.json').read_text())
            if correction.get('outcome')=='failed' and state.get('middle_goal_canceled_for_correction'):
                state['terminal']={'status':5,'receive_ros_ns':now,'reason':'middle_correction_or_resume_failed'}
                (data/'navigation_terminal.json').write_text(json.dumps(state['terminal'],indent=2)+'\n')
        if time.monotonic()>=next_update:
            report=dict(elapsed_s=round((now-begin)*1e-9,1) if begin else None,mode=state['mode'],route=state['route'],terminal=state['terminal'],native_age_s=(now-state['native_stamp_ns'])*1e-9 if state.get('native_stamp_ns') else None,mocap_age_s=(now-state['last_mocap_ns'])*1e-9 if state.get('last_mocap_ns') else None)
            (data/'live_state.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');print(json.dumps(report,ensure_ascii=False),flush=True);next_update=time.monotonic()+10
        if state['terminal']:
            if state['terminal']['status']!=4:
                result=dict(exit_code=None,captures=[],trigger_policy='No endpoint measurement for an aborted or canceled route',reason='route_did_not_succeed')
                (data/'endpoint_validation.json').write_text(json.dumps(result,indent=2)+'\n')
                break
            age=(now-state['terminal']['receive_ros_ns'])*1e-9
            if age>=4 and validation is None:
                before=set((run/'validation').glob('*'))
                validation=subprocess.Popen([str(Path(__file__).resolve().parents[2]/'mapping_modes.sh'),'anchored','validate','--run',str(run),'--context','localization','--session',str(session)],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
            if validation is not None and validation.poll() is not None and not recorded:
                captures=set((run/'validation').glob('*'))-before
                result=dict(exit_code=validation.returncode,captures=[str(p.resolve()) for p in sorted(captures)],trigger_policy='Original static validation requested at navigation terminal +4 s; no refit')
                (data/'endpoint_validation.json').write_text(json.dumps(result,indent=2)+'\n');print('ENDPOINT_VALIDATION',json.dumps(result),flush=True);recorded=True
            if age>=25 and recorded:break
        # The bag records the complete streams. This monitor needs only their
        # latest state; avoid competing with LIO on high-rate command topics.
        time.sleep(.002)
    (data/'observer_final.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n')
    node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
