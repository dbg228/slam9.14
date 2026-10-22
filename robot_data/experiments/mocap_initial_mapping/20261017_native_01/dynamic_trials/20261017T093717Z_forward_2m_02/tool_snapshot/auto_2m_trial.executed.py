#!/usr/bin/env python3
"""Reuse the historical 2 m protocol with frozen map feedback and operator mode control."""
import argparse
from bisect import bisect_left
from collections import deque
from dataclasses import asdict
import csv
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[3]
sys.path[:0]=[str(ROOT),str(ROOT/'src/localization_ab_test'),str(ROOT/'tests/controller_tracking'),str(ROOT/'tests/mocap_anchored_mapping')]
os.environ['PYTHONPATH']=os.pathsep.join(sys.path[:4]+[os.environ.get('PYTHONPATH','')])
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import PoseStamped,Twist,TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String,Bool
from diagnostic_msgs.msg import DiagnosticArray
from rcl_interfaces.srv import GetParameters
from rosidl_runtime_py.convert import message_to_ordereddict
from controller_tracking.common import stats,wrap
from controller_tracking.world_capture import clean
from mocap_anchored_mapping.core import checked,checked_session,load,sha,native_base,mocap_base,record
from mocap_anchored_mapping.online import pose_matrix
from localization_ab_test.core import MotionConfig
from localization_ab_test.nodes import VelocityGuard
from localization_ab_test.qos import latest_value_qos
from localization_ab_test.run import Recorder
from localization_ab_test.chassis_evidence import hardware_readiness


def relay_ready(now,received,mode,graph_ok,source_ok,failure):
    limits={'request':.20,'permit':.30,'mocap':.20,'native':.30,'status':.50,'mode':.50}
    return (not failure and graph_ok and source_ok and mode=='EXTERNAL'
            and all(k in received and 0<=now-received[k]<=age for k,age in limits.items()))


class OperatorGuard(VelocityGuard):
    """Retain historical request validation but never publish or override teleop mode."""
    def __init__(self,trial):
        Node.__init__(self,'anchored_2m_velocity_guard')
        self.meta=load(Path(trial)/'manifest.json');self.armed=True;self.allowed_speed=.15
        self.latest=self.last_stamp=self.mocap_stamp=0;self.speed=0.;self.failure=''
        self.permit_time=self.mocap_time=None;self.mode_value='';self.received={}
        self.source_ok=False;self.graph_ok=False;self.ever_enabled=False;self.created=time.monotonic()
        self.velocity=self.create_publisher(Twist,'/nav/cmd_vel',1)
        self.status=self.create_publisher(String,'/localization_ab/guard_status',latest_value_qos(10))
        self.create_subscription(TwistStamped,'/localization_ab/request',self.request,latest_value_qos())
        self.create_subscription(Bool,'/localization_ab/recorder_ready',self.permit,latest_value_qos())
        self.create_subscription(PoseStamped,'/mocap/Tracker0/pose',self.mocap,qos_profile_sensor_data)
        self.create_subscription(DiagnosticArray,'/mocap/Tracker0/diagnostics',self.diagnostic,10)
        self.create_subscription(String,'/teleop/mode',self.operator_mode,10)
        self.create_subscription(Odometry,'/anchored_localization/native_odom',self.native,qos_profile_sensor_data)
        self.create_subscription(String,'/anchored_localization/status',self.source_status,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(.02,self.tick);self.create_timer(.2,self.check_ownership)

    def operator_mode(self,msg):
        self.mode_value=msg.data;self.received['mode']=time.monotonic()
        if self.ever_enabled and msg.data!='EXTERNAL':self.fail('operator_took_control:'+msg.data)

    def native(self,msg):
        age=(self.get_clock().now().nanoseconds-msg.header.stamp.sec*10**9-msg.header.stamp.nanosec)*1e-9
        if msg.header.frame_id=='camera_init' and msg.child_frame_id=='body' and 0<=age<=.25:
            self.received['native']=time.monotonic()

    def source_status(self,msg):
        s=json.loads(msg.data);self.received['status']=time.monotonic()
        self.source_ok=(s['run_path']==self.meta['run'] and s['capture_dir']==self.meta['localization_session'] and s['frozen'] and not s['fault'])
        if self.ever_enabled and not self.source_ok:self.fail('localization_identity_or_frozen_state_changed')

    def check_ownership(self):
        expected={'/nav/cmd_vel':self.get_name(),'/teleop/mode':'teleop_node','/anchored_localization/native_odom':'anchored_fastlio'}
        self.graph_ok=False
        for topic,name in expected.items():
            pubs=self.get_publishers_info_by_topic(topic)
            if len(pubs)!=1 or pubs[0].node_name!=name or pubs[0].node_namespace!='/':
                if time.monotonic()-self.created>3:self.fail('publisher_ownership:'+topic)
                return
        self.graph_ok=True

    def stop(self):
        if self.armed:self.velocity.publish(Twist())

    def tick(self):
        now=time.monotonic()
        for key,value in [('request',self.latest),('permit',self.permit_time),('mocap',self.mocap_time)]:
            if value is not None and value>0:self.received[key]=value
        valid=relay_ready(now,self.received,self.mode_value,self.graph_ok,self.source_ok,self.failure)
        if valid:self.ever_enabled=True
        elif self.ever_enabled or self.speed:self.fail('request_recorder_mocap_localization_or_mode_not_ready')
        if valid and self.speed:
            msg=Twist();msg.linear.x=self.speed;self.velocity.publish(msg)
        else:self.stop()
        self.status.publish(String(data=json.dumps(dict(armed=True,moving=bool(valid and self.speed),
            ready=valid,latched_failure=self.failure,mode=self.mode_value,ros_stamp_ns=self.get_clock().now().nanoseconds))))


class Monitor(Node):
    def __init__(self,trial,execute):
        super().__init__('anchored_2m_supervisor');self.trial=Path(trial);self.meta=load(self.trial/'manifest.json')
        self.run=Path(self.meta['run']);self.cfg,self.ext=checked(self.run,True)
        self.seed=checked_session(self.run,Path(self.meta['localization_session']))
        self.map_odom=np.asarray(self.seed['T_map_odom']['matrix']);self.cal=load(self.run/'calibration.yaml')
        self.mode='';self.state=None;self.guard=None;self.times={};self.source=None;self.final=[]
        self.mc=[];self.estimates=[];self.events=[];self.base_history=deque(maxlen=100)
        self.journal=(self.trial/('raw.jsonl' if execute else 'preflight_'+str(time.time_ns())+'.jsonl')).open('x',buffering=1)
        self.permit=self.create_publisher(Bool,'/localization_ab/recorder_ready',latest_value_qos())
        self.estimate=self.create_publisher(PoseStamped,'/localization_ab/estimate/A',20)
        for key,typ,topic,qos in [('mocap',PoseStamped,'/mocap/Tracker0/pose',qos_profile_sensor_data),
            ('raw_mocap',PoseStamped,'/vrpn_mocap/Tracker0/pose',qos_profile_sensor_data),
            ('validity',DiagnosticArray,'/mocap/Tracker0/diagnostics',50),
            ('native',Odometry,'/anchored_localization/native_odom',qos_profile_sensor_data),
            ('mode',String,'/teleop/mode',10),('final',Twist,'/reference_velocity',qos_profile_sensor_data),
            ('state',String,'/localization_ab/state',latest_value_qos(10)),
            ('guard',String,'/localization_ab/guard_status',latest_value_qos(10)),
            ('events',String,'/localization_ab/events',100),
            ('status',String,'/anchored_localization/status',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))]:
            self.create_subscription(typ,topic,lambda msg,k=key:self.receive(k,msg),qos)
        self.valid_stamps=set()

    def receive(self,key,msg):
        now=time.monotonic();self.times[key]=now
        self.journal.write(json.dumps(dict(kind=key,receive_ros_ns=self.get_clock().now().nanoseconds,
            message=clean(message_to_ordereddict(msg))),allow_nan=False)+'\n')
        if key=='mode':self.mode=msg.data
        elif key=='state':self.state=json.loads(msg.data)
        elif key=='guard':self.guard=json.loads(msg.data)
        elif key=='events':self.events.append(json.loads(msg.data))
        elif key=='status':self.source=json.loads(msg.data)
        elif key=='final':self.final=(self.final+[[now,msg.linear.x,msg.linear.y,msg.angular.z]])[-100:]
        elif key=='validity':
            valid=bool(msg.status) and all(s.level in (0,b'\x00') and dict((v.key,v.value) for v in s.values).get('valid')=='true' for s in msg.status)
            if valid:self.valid_stamps.add(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec)
        elif key in ('mocap','native'):
            stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
            if key=='mocap':
                if msg.header.frame_id!='mocap_world':return
                h=mocap_base(pose_matrix(msg.pose),self.cfg,self.ext);self.mc.append((stamp,h))
            else:
                age=(self.get_clock().now().nanoseconds-stamp)*1e-9
                if msg.header.frame_id!='camera_init' or msg.child_frame_id!='body' or not 0<=age<=.25:return
                h=self.map_odom@native_base(pose_matrix(msg.pose.pose),self.cfg,self.ext)
                self.estimates.append((stamp,h));self.base_history.append((now,h))
                p=PoseStamped();p.header=msg.header;p.header.frame_id='map';p.pose.position.x,p.pose.position.y,p.pose.position.z=h[:3,3].tolist()
                q=record(h,'map','base_link')['quaternion_xyzw'];p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z,p.pose.orientation.w=q
                self.estimate.publish(p)

    def inspect(self):
        end=time.monotonic()+4
        while time.monotonic()<end:rclpy.spin_once(self,timeout_sec=.02)
        now=time.monotonic();errors=[]
        for key,limit in [('mode',.5),('native',.3),('mocap',.2),('validity',.2),('status',.5),('final',.3)]:
            if key not in self.times or now-self.times[key]>limit:errors.append('missing_or_stale:'+key)
        if self.mode!='EXTERNAL':errors.append('operator_mode_not_EXTERNAL:'+self.mode)
        if not self.source or self.source['run_path']!=str(self.run) or self.source['capture_dir']!=self.meta['localization_session'] or not self.source['frozen'] or self.source['fault']:errors.append('wrong_or_unhealthy_frozen_localization')
        if self.get_publishers_info_by_topic('/nav/cmd_vel'):errors.append('another_velocity_publisher')
        if any(n in ('controller_server','velocity_smoother','static_map_localizer') for n,_ in self.get_node_names_and_namespaces()):errors.append('competing_navigation_or_NDT')
        for topic,name in [('/teleop/mode','teleop_node'),('/anchored_localization/native_odom','anchored_fastlio')]:
            pubs=self.get_publishers_info_by_topic(topic)
            if len(pubs)!=1 or pubs[0].node_name!=name or pubs[0].node_namespace!='/':errors.append('wrong_publisher:'+topic)
        if not self.final or any(max(abs(x) for x in r[1:])>1e-4 for r in self.final[-20:]):errors.append('chassis_not_stationary')
        history=[h[:3,3] for t,h in self.base_history if now-t<1]
        if len(history)<5 or np.max(np.linalg.norm(np.asarray(history)-np.mean(history,axis=0),axis=1))>.02:errors.append('native_pose_not_static')
        client=self.create_client(GetParameters,'/arbitrator_controller/get_parameters')
        if not client.wait_for_service(timeout_sec=2):errors.append('watchdog_service_missing');watchdog=None
        else:
            future=client.call_async(GetParameters.Request(names=['external_watchdog_timeout']));rclpy.spin_until_future_complete(self,future,timeout_sec=3)
            watchdog=future.result().values[0].double_value if future.done() and future.result() else None
            if watchdog is None or not 0<watchdog<=.5:errors.append('watchdog_not_short')
        prep=load(self.trial/'watchdog_preparation.json')
        if prep.get('status')!='prepared' or prep.get('watchdog_s')!=watchdog:errors.append('no_successful_watchdog_reconfigure_evidence')
        hardware=prep.get('hardware_readiness',{})
        if not hardware.get('ready'):errors.append('hardware_restore_evidence_missing')
        for name,entry in self.meta['protected_files'].items():
            if sha(self.run/name)!=entry['sha256']:errors.append('protected_file_changed:'+name)
        return dict(ready=not errors,errors=errors,mode=self.mode,watchdog_s=watchdog,source=self.source,
                    mocap_samples=len(self.mc),native_samples=len(self.estimates),hardware=hardware)


def analyze(node):
    valid=sorted(((t,h) for t,h in node.mc if t in node.valid_stamps),key=lambda item:item[0]);stamps=[x[0] for x in valid]
    starts=[e for e in node.events if e['event']=='motion_start'];stops=[e for e in node.events if e['event']=='stop_command']
    start=starts[0]['ros_stamp_ns'] if starts else None;stop=stops[0]['ros_stamp_ns'] if stops else None
    rows=[];map_mc=np.asarray(node.cal['T_map_mocap']['matrix'])
    for stamp,h in node.estimates:
        phase='pre_motion' if start is None or stamp<start else ('motion' if stop is None or stamp<stop else 'post_stop')
        row=dict(stamp_ns=stamp,time_s=(stamp-(start or node.estimates[0][0]))*1e-9,phase=phase,reference_valid=False,
                 reference_stamp_ns='',match_skew_s='',estimate_x_m=float(h[0,3]),estimate_y_m=float(h[1,3]),
                 reference_x_m='',reference_y_m='',error_x_m='',error_y_m='',error_xy_m='',yaw_error_deg='')
        if stamps:
            i=bisect_left(stamps,stamp);candidates=[j for j in (i-1,i) if 0<=j<len(stamps)];j=min(candidates,key=lambda j:abs(stamps[j]-stamp))
            if abs(stamps[j]-stamp)<=25_000_000:
                truth=map_mc@valid[j][1];d=h[:2,3]-truth[:2,3];dyaw=wrap(math.atan2(h[1,0],h[0,0])-math.atan2(truth[1,0],truth[0,0]))
                row.update(reference_valid=True,reference_stamp_ns=stamps[j],match_skew_s=(stamps[j]-stamp)*1e-9,
                    reference_x_m=float(truth[0,3]),reference_y_m=float(truth[1,3]),error_x_m=float(d[0]),error_y_m=float(d[1]),
                    error_xy_m=float(np.linalg.norm(d)),yaw_error_deg=float(np.degrees(dyaw)))
        rows.append(row)
    if rows:
        with (node.trial/'samples.csv').open('w',newline='') as stream:
            w=csv.DictWriter(stream,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    measured=[r for r in rows if r['phase']!='pre_motion'];matched=[r for r in measured if r['reference_valid']]
    result=dict(data_kind='real_2m_dynamic_localization_test',state=node.state,
        guard_during_trial=node.meta.get('guard_at_trial_end'),guard_after_intentional_disarm=node.guard,events=node.events,
        motion_started=start is not None,reference_coverage=len(matched)/len(measured) if measured else None,
        XY_error_m=stats([r['error_xy_m'] for r in matched]),yaw_error_deg=stats([r['yaw_error_deg'] for r in matched]),
        phases={phase:dict(total=sum(r['phase']==phase for r in rows),XY_error_m=stats([r['error_xy_m'] for r in rows if r['phase']==phase and r['reference_valid']])) for phase in ['pre_motion','motion','post_stop']},
        timing_verified=False,matching='nearest within 25ms; no interpolation, extrapolation, zero-fill or alignment fit',
        calibration_sha256=sha(node.run/'calibration.yaml'),initialization_sha256=sha(Path(node.meta['localization_session'])/'initialization.yaml'))
    if start and stop:
        tail=[r for r in matched if 2<=(r['stamp_ns']-stop)*1e-9<=5]
        result['post_stop_2_to_5s_XY_error_m']=stats([r['error_xy_m'] for r in tail])
        if starts and valid:
            p=starts[0]['start_base_pose'];tail_mc=[h[:2,3] for t,h in valid if 2<=(t-stop)*1e-9<=5]
            if tail_mc:result['stopped_displacement_m']=float(np.linalg.norm(np.mean(tail_mc,axis=0)-np.array([p['x'],p['y']])))
    (node.trial/'results.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    if matched:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig,axes=plt.subplots(1,2,figsize=(12,4.5))
        axes[0].plot([r['reference_x_m'] for r in matched],[r['reference_y_m'] for r in matched],label='Mocap')
        axes[0].plot([r['estimate_x_m'] for r in measured],[r['estimate_y_m'] for r in measured],label='FAST-LIO',alpha=.8)
        axes[0].set(xlabel='map X (m)',ylabel='map Y (m)');axes[0].axis('equal');axes[0].legend()
        axes[1].plot([r['time_s'] for r in matched],[r['error_xy_m']*1000 for r in matched],'.-',markersize=2)
        if stop:axes[1].axvline((stop-start)*1e-9,color='tab:orange',linestyle='--',label='Stop command');axes[1].legend()
        axes[1].set(xlabel='Time from motion start (s)',ylabel='XY error (mm)')
        for ax in axes:ax.grid(alpha=.3)
        fig.suptitle('Frozen mocap initialization; no refit; timing unverified');fig.tight_layout();fig.savefig(node.trial/'errors_xy.png',dpi=180);plt.close(fig)
    text=['# 本轮自动前进 2 m 定位实验','',f"状态：{node.meta['stage']}；真实传感器数据；固定速度请求 0.15m/s。",'',
          '复用旧 MotionGate 的动捕 base_link 位移停止条件；保留手柄 STOP/MANUAL 接管，独立进程发布底盘命令，底盘看门狗临时配置为0.25s。',
          '位移2m是发零触发距离，实际停车可能有过冲。位姿采用本次固定世界标定及已冻结的map→odom，不重新拟合，不用NDT。',
          '动捕单位米；两边参考点base_link；原始消息见raw.jsonl与bag/，CSV缺失参考留空。曝光延迟/时差未独立核实，动态指标为初步结果。','',
          '```json',json.dumps(result,ensure_ascii=False,indent=2),'```']
    (node.trial/'report.md').write_text('\n'.join(text)+'\n');return result


def run(trial,execute):
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=Monitor(trial,execute);children=[];recorder=None;streams=[]
    def save(): (node.trial/'manifest.json').write_text(json.dumps(node.meta,ensure_ascii=False,indent=2)+'\n')
    def start(argv,name):
        stream=(node.trial/name).open('x');streams.append(stream)
        p=subprocess.Popen(argv,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True);children.append(p);return p
    try:
        audit=node.inspect();(node.trial/('execution_preflight.json' if execute else 'readiness.json')).write_text(json.dumps(audit,ensure_ascii=False,indent=2)+'\n')
        print(json.dumps(audit,ensure_ascii=False,indent=2),flush=True)
        if not execute:return
        if not node.meta['route_clear_confirmed'] or not audit['ready']:raise RuntimeError('Preflight blocked: '+str(audit['errors']))
        node.meta.update(stage='recording_before_motion',legacy_protocol=asdict(MotionConfig()),legacy_motion_core_sha256=sha(ROOT/'src/localization_ab_test/localization_ab_test/core.py'),tool_sha256=sha(Path(__file__)))
        save()
        guard=start([sys.executable,'-u',str(Path(__file__).resolve()),'--trial',str(node.trial),'--execute','--guard'],'guard.log')
        motion=start([sys.executable,'-u','-c','from localization_ab_test.nodes import motion_main; motion_main()',
            '--group','A','--direction','forward','--mount',str(node.run/'mocap.yaml')],'motion.log')
        topics=['/vrpn_mocap/Tracker0/pose','/mocap/Tracker0/pose','/mocap/Tracker0/diagnostics','/anchored_localization/native_odom',
                '/anchored_localization/status','/localization_ab/estimate/A','/localization_ab/request','/localization_ab/state',
                '/localization_ab/events','/localization_ab/guard_status','/localization_ab/recorder_ready','/nav/cmd_vel','/teleop/mode','/reference_velocity','/tf','/tf_static']
        recorder=Recorder(node.trial,topics);deadline=time.monotonic()+10
        while not recorder.ready.is_set() and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.02)
            if any(p.poll() is not None for p in children) or recorder.process.poll() is not None:raise RuntimeError('Child exited before recorder ready')
        if not recorder.ready.is_set():raise RuntimeError('Recorder subscriptions not ready')
        node.meta['stage']='running';save();deadline=time.monotonic()+50
        while time.monotonic()<deadline:
            node.permit.publish(Bool(data=True));until=time.monotonic()+.06
            while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.01)
            if recorder.process.poll() is not None or recorder.error:raise RuntimeError('Recorder failed')
            if guard.poll() is not None:raise RuntimeError('Independent guard exited')
            if node.guard and node.guard['latched_failure']:raise RuntimeError('Guard: '+node.guard['latched_failure'])
            if node.state and node.state['state'] in ('DONE','ABORTED'):
                node.meta['stage']='completed' if node.state['state']=='DONE' else 'aborted';break
            if motion.poll() is not None:raise RuntimeError('Motion process exited')
        else:raise RuntimeError('Supervisor timeout')
    except (Exception,KeyboardInterrupt) as e:
        node.meta.update(stage='aborted',error=str(e) or 'operator_interrupt');print(str(e),file=sys.stderr,flush=True)
    finally:
        if execute:node.meta['guard_at_trial_end']=dict(node.guard) if node.guard else None
        if execute and children:
            end=time.monotonic()+(1 if node.meta['stage']=='completed' else 5)
            while time.monotonic()<end:
                node.permit.publish(Bool(data=False));rclpy.spin_once(node,timeout_sec=.03)
        for p in reversed(children):
            if p.poll() is None:
                os.killpg(p.pid,signal.SIGINT)
                try:p.wait(timeout=5)
                except subprocess.TimeoutExpired:raise RuntimeError('Child failed to stop; guard must remain until verified')
        if recorder:recorder.close()
        if execute:
            end=time.monotonic()+.4
            while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
            node.meta['protected_files_unchanged']=all(sha(node.run/name)==v['sha256'] for name,v in node.meta['protected_files'].items())
            node.meta['final_command_zero']=bool(node.final and time.monotonic()-node.final[-1][0]<.3 and max(abs(v) for v in node.final[-1][1:])<1e-4)
            save();result=analyze(node);print(json.dumps(result,ensure_ascii=False,indent=2),flush=True)
        for stream in streams:stream.close()
        node.journal.close();node.destroy_node();rclpy.shutdown()


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--trial',required=True)
    parser.add_argument('--execute',action='store_true');parser.add_argument('--guard',action='store_true',help=argparse.SUPPRESS)
    args=parser.parse_args()
    if args.guard:
        if not args.execute:parser.error('Guard requires explicit execute')
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=OperatorGuard(args.trial)
        try:rclpy.spin(node)
        except KeyboardInterrupt:pass
        finally:
            for _ in range(10):node.stop();rclpy.spin_once(node,timeout_sec=.01)
            node.destroy_node();rclpy.shutdown()
    else:run(args.trial,args.execute)


if __name__=='__main__':main()
