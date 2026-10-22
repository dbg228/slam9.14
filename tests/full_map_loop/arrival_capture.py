#!/usr/bin/env python3
"""Reconstruct a fixed first-arrival static window from a closed real bag."""
import argparse
import bisect
from collections import Counter
import copy
import hashlib
import json
from pathlib import Path
import sqlite3
import sys

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tests/controller_tracking'))
from controller_tracking import world_calibration as wc
from controller_tracking import world_capture as capture


def messages(bag,topic,begin=0,end=2**63-1):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    for path in sorted(Path(bag).glob('*.db3')):
        with sqlite3.connect(f'file:{path.resolve()}?mode=ro',uri=True) as db:
            row=db.execute('SELECT id,type FROM topics WHERE name=?',(topic,)).fetchone()
            if row is None:continue
            cls=get_message(row[1])
            for received,data in db.execute('SELECT timestamp,data FROM messages WHERE topic_id=? AND timestamp>=? AND timestamp<=? ORDER BY timestamp',(row[0],begin,end)):
                yield received,deserialize_message(data,cls)


def command_values(m):
    return [m.linear.x,m.linear.y,m.linear.z,m.angular.x,m.angular.y,m.angular.z]


def motion_interval(trial,bag):
    departure=wc.load(trial/'formal_departure_request.json')['request_ros_ns']
    terminal=wc.load(trial/'route_monitor_latest.json')
    if terminal['goal_status']!=4:raise ValueError('This arrival extraction requires recorded navigation success')
    ended=terminal['ros_now_ns'];first=None;last=None;zero=None
    for received,m in messages(bag,'/reference_velocity',departure,ended+3*10**9):
        if max(abs(v) for v in command_values(m))>1e-4:
            if first is None:first=received
            last=received;zero=None
        elif last is not None and zero is None:zero=received
    if first is None or zero is None:raise ValueError('No bounded motion interval with a recorded stop')
    return dict(start_ns=first,end_ns=zero+10**9,first_command_ns=first,last_nonzero_command_ns=last,
        zero_command_ns=zero,navigation_terminal_ns=ended,command_threshold=1e-4,settling_margin_s=1.,
        definition='First nonzero final chassis command through first following zero plus one second; original ROS receive times; distance itself uses continuous odom source stamps')


def extract(trial,calibration_session,output,window_index=0):
    from rosidl_runtime_py.convert import message_to_ordereddict
    trial=Path(trial).resolve();bag=trial/'formal_bag_01';source=Path(calibration_session).resolve();output=Path(output).resolve()
    if not (bag/'metadata.yaml').exists():raise ValueError('Close the recorder before extracting the canonical arrival window')
    if output.exists():raise ValueError('Use a new output; all extraction attempts are retained')
    cal=wc.check_calibration(source);inputs=wc.frozen_inputs(source);cfg=copy.deepcopy(inputs['config'])
    interval=motion_interval(trial,bag)
    if window_index<0:raise ValueError('Window index must be nonnegative')
    earliest=interval['zero_command_ns']+round((cfg['limits']['warmup_s']+window_index*cfg['limits']['seconds'])*1e9)
    candidates=list(messages(bag,cfg['topics']['ndt'],earliest,earliest+40*10**9))
    candidates.sort(key=lambda r:capture.stamp_ns(r[1].cloud_stamp))
    anchor=next((capture.stamp_ns(m.cloud_stamp) for _,m in candidates if capture.stamp_ns(m.cloud_stamp)>=earliest),None)
    if anchor is None:raise ValueError('No NDT measurement after the independently observed arrival stop')
    begin=anchor-500_000_000;end=begin+round(cfg['limits']['seconds']*1e9)
    output.mkdir();(output/'endpoint').mkdir()
    manifest={'source_bag':str(bag),'files':{str(p.resolve()):{'bytes':p.stat().st_size,'sha256':wc.sha(p)} for p in bag.glob('*.db3')}}
    wc.save(output/'source_bag_manifest.json',manifest)
    cfg.update(evaluation_only=True,static_history_pairing=True,fixed_calibration_session=str(source),
        data_kind='real_measurement_from_recorded_bag',
        extraction_policy='First NDT source stamp after recorded final command stop plus original warmup; fixed five-second source-time window centered by cadence, never selected by residual or fitness')
    cfg['map_reference_files']=[*cfg['map_reference_files'],str(source/'calibration.yaml'),str(source/'calibration.lock.json'),str(source/'frozen_inputs.json'),str(output/'source_bag_manifest.json')]
    wc.save(output/'session.yaml',cfg,True);wc.save(output/'body_extrinsic.yaml',inputs['body_extrinsic'],True);wc.freeze_inputs(output)
    journal=[];raw=[];normalized=[];statuses={};invalid=Counter();counts=Counter();commands=[];ndt=[]
    def log(topic,received,m):
        counts[topic]+=1;journal.append(dict(receive_ros_ns=received,topic=topic,
            selection='source_measurement_time_window' if hasattr(m,'header') else 'receive_time_window',
            message=capture.clean(message_to_ordereddict(m))))
    for topic,key in [(cfg['normalizer']['input_topic'],'raw'),(cfg['normalizer']['output_topic'],'normalized'),(cfg['normalizer']['diagnostics_topic'],'status')]:
        for received,m in messages(bag,topic,begin-2*10**9,end+2*10**9):
            stamp=capture.stamp_ns(m.header.stamp)
            if not begin<=stamp<=end:continue
            log(topic,received,m)
            if key=='status':
                relevant=[s for s in m.status if s.name=='mocap_tracker0_normalizer']
                valid=bool(relevant) and all(s.level in (0,b'\x00') and {v.key:v.value for v in s.values}.get('valid')=='true' for s in relevant)
                statuses[stamp]=valid
                if not valid:invalid['normalizer_invalid']+=1
            else:
                pose=capture.read_pose(m.pose,stamp,received,m.header.frame_id)
                (raw if key=='raw' else normalized).append(pose)
    normalized.sort(key=lambda p:p['stamp_ns']);valid_poses=[]
    for p in normalized:
        if statuses.get(p['stamp_ns']) is True:valid_poses.append(p)
        else:invalid['missing_or_invalid_normalizer_status']+=1
    ready=sorted((r,bool(m.data)) for r,m in messages(bag,cfg['topics']['map_ready']))
    ready_stamps=[p[0] for p in ready]
    for received,m in candidates:
        stamp=capture.stamp_ns(m.cloud_stamp)
        if not begin<=stamp<=end:continue
        log(cfg['topics']['ndt'],received,m);index=bisect.bisect_right(ready_stamps,received)-1
        p=capture.read_pose(getattr(m,cfg['ndt_pose_field']),stamp,received,m.header.frame_id)
        ndt.append(dict(stamp_ns=stamp,receive_ros_ns=received,frame=m.header.frame_id,
            valid=bool(getattr(m,cfg['ndt_pose_field']+'_valid')),sync_valid=m.cloud_to_odom_sync_valid,
            cloud_odom_skew_s=m.cloud_to_odom_skew_sec,pose=p,match_id=int(m.match_id),accepted=bool(m.accepted),
            stage=m.stage,map_ready=ready[index][1] if index>=0 else False))
    for received,m in messages(bag,cfg['topics']['command'],begin,end):
        log(cfg['topics']['command'],received,m);commands.append(command_values(m))
    result=capture.summarize(cfg,inputs['body_extrinsic']['rigid_body_to_base_link'],valid_poses,ndt,commands,sum(invalid.values()),evaluation_only=True)
    units=capture.check_raw_normalized(raw,valid_poses,cfg['mocap']['position_scale_to_m'])
    if units['coverage']<cfg['limits']['minimum_raw_normalized_coverage'] or units['mismatched_stamp_ns']:
        result['quality_passed']=False;result['failures'].append('raw_normalized_units_or_coverage_mismatch')
    for topic in (cfg['normalizer']['input_topic'],cfg['normalizer']['output_topic'],cfg['normalizer']['diagnostics_topic']):
        if not counts[topic]:result['quality_passed']=False;result['failures'].append('missing_stream:'+topic)
    raw_path=output/'endpoint/raw.jsonl'
    with raw_path.open('x') as f:
        for record in sorted(journal,key=lambda r:r['receive_ros_ns']):f.write(json.dumps(record,allow_nan=False)+'\n')
    result.update(group='endpoint',evaluation_only=True,transform_refitted=False,no_motion_published=True,
        inputs_sha256=wc.sha(output/'frozen_inputs.json'),raw_sha256=wc.sha(raw_path),
        calibration_sha256_at_capture=wc.sha(source/'calibration.yaml'),data_kind=cfg['data_kind'],
        raw_normalized_check=units,invalid_events=dict(invalid),observed_messages_in_window=dict(counts),
        ndt_age_policy='same_timestamp_valid_static_history',source_age_limit_s=cfg['limits']['maximum_source_age_s'],
        ndt_source_age_limit_s=None,source_window_ns=[begin,end],motion_interval=interval,
        time_from_navigation_terminal_to_window_start_s=(begin-interval['navigation_terminal_ns'])*1e-9,
        extraction_policy=cfg['extraction_policy'],source_bag_manifest='source_bag_manifest.json',
        time_window_index=window_index,
        publisher_identity_note='Bag messages do not encode publisher identity; live trial source audits retained separately')
    wc.save(output/'endpoint/summary.json',result);wc.save(output/'motion_interval.json',interval)
    return dict(output=str(output),quality_passed=result['quality_passed'],failures=result['failures'],
        pair_count=result['pair_count'],raw_coverage=units['coverage'],source_window_ns=[begin,end],motion_interval=interval)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--trial',required=True);p.add_argument('--calibration-session',required=True);p.add_argument('--output',required=True);p.add_argument('--window-index',type=int,default=0);a=p.parse_args()
    result=extract(a.trial,a.calibration_session,a.output,a.window_index);print(json.dumps(result,indent=2));return 0 if result['quality_passed'] else 2


if __name__=='__main__':raise SystemExit(main())
