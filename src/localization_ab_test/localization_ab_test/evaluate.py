"""Offline scoring with explicit timing provenance and one start alignment."""
import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

from fastlio_mocap_trajectory_evaluation_core import EvaluationOptions,evaluate_trajectory,scalar_stats
from ndt_match_evaluation_core import wrap,yaw
from .core import base_pose,load_mount,pose_dict


def read_bag(path,group,mount):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path),storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
    available={i.name:i.type for i in reader.get_all_topics_and_types()}
    pose_topic=f'/localization_ab/estimate/{group}'
    selected=(pose_topic,'/mocap/Tracker0/pose','/localization_ab/events','/mocap/Tracker0/diagnostics')
    missing=[x for x in selected if x not in available]
    if missing:raise ValueError('Required topics missing: '+','.join(missing))
    types={t:get_message(available[t]) for t in selected}
    estimates=[];mocap=[];events=[];diagnostics=[];frames=set()
    while reader.has_next():
        t,data,record_stamp=reader.read_next()
        if t not in types:continue
        msg=deserialize_message(data,types[t])
        if t==pose_topic:
            frames.add(msg.header.frame_id)
            estimates.append(pose_dict(msg.header.stamp,msg.pose))
        elif t=='/mocap/Tracker0/pose':
            if msg.header.frame_id!='mocap_world':raise ValueError('Unexpected mocap frame')
            mocap.append(base_pose(pose_dict(msg.header.stamp,msg.pose),mount))
        elif t=='/localization_ab/events':events.append(json.loads(msg.data))
        else:
            for s in msg.status:
                fields={v.key:v.value for v in s.values}
                diagnostics.append(dict(stamp_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,
                                        record_stamp_ns=record_stamp,valid=fields.get('valid')=='true',reason=s.message))
    expected={'A':'odom','B':'map','C':'mocap_world'}[group]
    if frames!={expected}:raise ValueError(f'Estimate frames {frames} do not match group frame {expected}')
    for d in diagnostics:
        if not d['valid'] and d['stamp_ns']>0:
            # Keep an invalid boundary even if normalizer omitted the pose.
            stamps={p['stamp_ns'] for p in mocap}
            if d['stamp_ns'] not in stamps:
                mocap.append(dict(stamp_ns=d['stamp_ns'],valid=False,x=math.nan,y=math.nan,z=math.nan,
                                  qx=0.,qy=0.,qz=0.,qw=1.))
            else:
                for p in mocap:
                    if p['stamp_ns']==d['stamp_ns']:p['valid']=False
    return estimates,mocap,events,diagnostics


def score(estimates,mocap,events,group,offset,timing_verified=False):
    starts=[e for e in events if e['event']=='motion_start']
    stops=[e for e in events if e['event']=='stop_command']
    if len(starts)!=1 or len(stops)!=1:raise ValueError('One unambiguous start and stop event required')
    start,stop=starts[0],stops[0]
    motion_start=int(start['ros_stamp_ns']);stop_time=int(stop['ros_stamp_ns'])
    if stop_time<=motion_start:raise ValueError('Invalid event order')
    estimates=sorted(estimates,key=lambda p:p['stamp_ns'])
    if not estimates:raise ValueError('No estimate poses')
    alignment_start=(motion_start-estimates[0]['stamp_ns'])/1e9-3.5
    if alignment_start<0:raise ValueError('Need >=3.5 seconds of recorded stationary poses before motion')
    options=EvaluationOptions(time_offset_seconds=offset,alignment_start_seconds=alignment_start,
                              alignment_window_seconds=3.,alignment_min_samples=15,
                              max_interpolation_gap_seconds=.10,alignment_max_static_motion_m=.03)
    records,base_summary,_=evaluate_trajectory(estimates,mocap,options)
    end=stop_time+5*10**9
    rows=[]
    start_pose=start['start_base_pose']
    previous=None;previous_estimate=None;path=0.
    for r in records:
        stamp=r['lio_stamp_ns']
        if not motion_start<=stamp<=end:continue
        error=r['error'];reference=r['mocap_base_pose'];estimate=r['lio_pose']
        row=dict(stamp_ns=stamp,time_s=(stamp-motion_start)/1e9,
                 phase='moving' if stamp<stop_time else 'post_stop',valid=error is not None,
                 missing_reason=r['match_reason'],true_displacement_m=None,true_path_distance_m=None,
                 ex_m=None,ey_m=None,e_xy_m=None,yaw_error_deg=None,
                 estimate_x_m=estimate['x'],estimate_y_m=estimate['y'],estimate_yaw_rad=yaw(estimate),
                 reference_x_m=None,reference_y_m=None,pose_jump_m=None,yaw_jump_deg=None,error_jump_m=None)
        if previous_estimate and 0.<(stamp-previous_estimate['stamp_ns'])/1e9<=.25:
            row['pose_jump_m']=math.hypot(estimate['x']-previous_estimate['x'],estimate['y']-previous_estimate['y'])
            row['yaw_jump_deg']=abs(math.degrees(wrap(yaw(estimate)-yaw(previous_estimate))))
        previous_estimate=estimate
        if error:
            row.update(ex_m=error['ex_m'],ey_m=error['ey_m'],e_xy_m=error['e_xy_m'],
                       yaw_error_deg=math.degrees(error['yaw_rad']),
                       true_displacement_m=math.hypot(reference['x']-start_pose['x'],reference['y']-start_pose['y']),
                       reference_x_m=r['aligned_mocap_pose']['x'],reference_y_m=r['aligned_mocap_pose']['y'])
            if previous and (stamp-previous[0]['lio_stamp_ns'])/1e9<=.25:
                prev=previous[0];pest=prev['lio_pose'];pref=prev['mocap_base_pose']
                path+=math.hypot(reference['x']-pref['x'],reference['y']-pref['y'])
                row.update(pose_jump_m=math.hypot(estimate['x']-pest['x'],estimate['y']-pest['y']),
                           yaw_jump_deg=abs(math.degrees(wrap(yaw(estimate)-yaw(pest)))),
                           error_jump_m=math.hypot(error['ex_m']-prev['error']['ex_m'],error['ey_m']-prev['error']['ey_m']))
            row['true_path_distance_m']=path
            previous=(r,row)
        else:previous=None
        rows.append(row)
    valid=[r for r in rows if r['valid']]
    moving=[r for r in valid if r['phase']=='moving']
    endpoint=[r for r in valid if stop_time+2*10**9<=r['stamp_ns']<=end]
    def metrics(data):
        return dict(samples=len(data),xy_m=scalar_stats([r['e_xy_m'] for r in data]),
                    yaw_deg=scalar_stats([r['yaw_error_deg'] for r in data]))
    endpoint_summary=dict(samples=len(endpoint),xy_error_m=None,yaw_error_deg=None,
                          physical_displacement_m=None,overshoot_m=None)
    if len(endpoint)>=10:
        ex=float(np.mean([r['ex_m'] for r in endpoint]));ey=float(np.mean([r['ey_m'] for r in endpoint]))
        angles=np.radians([r['yaw_error_deg'] for r in endpoint])
        physical=float(np.mean([r['true_displacement_m'] for r in endpoint]))
        xy=np.array([[r['reference_x_m'],r['reference_y_m']] for r in endpoint])
        spread=float(np.max(np.linalg.norm(xy-xy.mean(axis=0),axis=1)))
        endpoint_summary.update(xy_error_m=math.hypot(ex,ey),mean_ex_m=ex,mean_ey_m=ey,
                                yaw_error_deg=math.degrees(math.atan2(np.sin(angles).sum(),np.cos(angles).sum())),
                                physical_displacement_m=physical,overshoot_m=physical-2.,
                                reference_max_radius_m=spread,reference_stationary=spread<.005)
    jumps=[r['pose_jump_m'] for r in rows if r['pose_jump_m'] is not None]
    ejumps=[r['error_jump_m'] for r in valid if r['error_jump_m'] is not None]
    yjumps=[r['yaw_jump_deg'] for r in rows if r['yaw_jump_deg'] is not None]
    summary=dict(group=group,reference_independent=group!='C',timing_verified=timing_verified,
                 result_label=('relative localization error' if group!='C' else 'mocap consistency; reference participates in correction'),
                 dynamic_accuracy_status='timing_verified' if timing_verified else 'provisional_time_offset_unverified',
                 time_offset_s=offset,alignment=base_summary['alignment'],
                 all_motion_and_5s_stop=metrics(valid),moving_only=metrics(moving),endpoint=endpoint_summary,
                 requested_samples=len(rows),valid_samples=len(valid),coverage=len(valid)/len(rows) if rows else 0.,
                 max_adjacent_pose_translation_m=max(jumps) if jumps else None,
                 max_adjacent_pose_yaw_deg=max(yjumps) if yjumps else None,
                 max_adjacent_error_jump_m=max(ejumps) if ejumps else None,
                 stop_reason=stop['reason'],trigger_displacement_m=stop['displacement_m'],
                 complete_5s_tail=bool(rows and rows[-1]['stamp_ns']>=end-200_000_000),
                 definitions=dict(scoring='native estimate measurement stamps; no resampling repeated low-frequency NDT as raw matches',
                                  estimate='A: local base pose; B: held NDT map->odom composed with local base; C: held mocap correction composed with local base',
                                  alignment='one fixed SE(2) from 3s stationary window ending .5s before motion; window excluded; no trajectory refit or scale',
                                  endpoint='norm of mean ex/ey during 2 to 5 seconds after stop command; includes braking overshoot',
                                  pose_jump='maximum consecutive estimate displacement at <=.25s over entire scoring interval, even when reference is missing; includes real motion',
                                  error_jump='maximum consecutive change of estimate-reference error, compensating actual reference movement',
                                  distance_axis='mocap planar base origin displacement from start; sampled path distance also provided'))
    return rows,summary


def write_report(output,rows,summary):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    output.mkdir(parents=True,exist_ok=False)
    with (output/'frame_errors.csv').open('w',newline='',encoding='utf8') as stream:
        w=csv.DictWriter(stream,fieldnames=list(rows[0]) if rows else [])
        w.writeheader();w.writerows(rows)
    (output/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    valid=[r for r in rows if r['valid']]
    if valid:
        t=[r['time_s'] for r in valid];xy=[r['e_xy_m']*1000 for r in valid];ya=[r['yaw_error_deg'] for r in valid]
        fig,axs=plt.subplots(3,1,figsize=(9,9),constrained_layout=True)
        axs[0].plot([r['reference_x_m'] for r in valid],[r['reference_y_m'] for r in valid],label='mocap base')
        axs[0].plot([r['estimate_x_m'] for r in valid],[r['estimate_y_m'] for r in valid],label='estimate')
        axs[0].set(xlabel='x (m)',ylabel='y (m)');axs[0].axis('equal');axs[0].legend()
        axs[1].plot([r['time_s'] for r in rows],[r['e_xy_m']*1000 if r['valid'] else math.nan for r in rows])
        axs[1].set(xlabel='time (s)',ylabel='XY error (mm)')
        axs[2].plot([r['time_s'] for r in rows],[r['yaw_error_deg'] if r['valid'] else math.nan for r in rows])
        axs[2].set(xlabel='time (s)',ylabel='yaw error (deg)')
        fig.suptitle(summary['group']+' | '+summary['dynamic_accuracy_status']+' | '+summary['result_label'])
        for ax in axs:ax.grid(alpha=.25)
        for ext in ('png','svg'):fig.savefig(output/('trajectory_errors.'+ext),dpi=180)
        plt.close(fig)
        fig,axs=plt.subplots(2,1,figsize=(8,7),constrained_layout=True)
        d=[r['true_displacement_m'] for r in valid]
        axs[0].scatter(d,xy,s=8);axs[0].set(ylabel='XY error (mm)')
        axs[1].scatter(d,ya,s=8);axs[1].set(xlabel='true mocap displacement from start (m)',ylabel='yaw error (deg)')
        # Fixed 5 cm bins create readable curves without fitting the trajectory.
        moving=[r for r in valid if r['phase']=='moving']
        centers=[];mean_xy=[];mean_yaw=[]
        for left in np.arange(0.,2.051,.05):
            points=[r for r in moving if left<=r['true_displacement_m']<left+.05]
            centers.append(left+.025)
            mean_xy.append(float(np.mean([r['e_xy_m'] for r in points]))*1000 if points else math.nan)
            angles=np.radians([r['yaw_error_deg'] for r in points])
            mean_yaw.append(math.degrees(math.atan2(np.sin(angles).sum(),np.cos(angles).sum())) if points else math.nan)
        axs[0].plot(centers,mean_xy,label='5 cm bin mean (moving)',color='tab:red')
        axs[1].plot(centers,mean_yaw,label='5 cm circular bin mean (moving)',color='tab:red')
        for ax in axs:ax.legend()
        for ax in axs:ax.grid(alpha=.25)
        for ext in ('png','svg'):fig.savefig(output/('errors_by_true_distance.'+ext),dpi=180)
        plt.close(fig)
    m=summary['all_motion_and_5s_stop'];x=m['xy_m'];y=m['yaw_deg']
    def fmt(value,scale=1.):return 'unknown' if value is None else f'{value*scale:.3f}'
    report=[f"# Group {summary['group']} — {summary['result_label']}",'',
            f"Trial status: **{summary.get('trial_status','synthetic')}**; stop reason: **{summary['stop_reason']}**.",
            ('Failed trial: exclude from completed 2 m comparisons. Metrics below describe only evaluable samples.'
             if summary.get('trial_status')=='aborted' else ''),'',
            f"Timing status: **{summary['dynamic_accuracy_status']}**; offset {summary['time_offset_s']:.6f} s.",'',
            '| Metric | Value |','|---|---|',
            f"| XY RMSE / P95 / max (mm) | {fmt(x['rmse'],1000)} / {fmt(x['p95_absolute'],1000)} / {fmt(x['max_absolute'],1000)} |",
            f"| Yaw RMSE (deg) | {fmt(y['rmse'])} |",
            f"| Endpoint error (mm) | {fmt(summary['endpoint']['xy_error_m'],1000)} |",
            f"| Max adjacent pose / error jump (mm) | {fmt(summary['max_adjacent_pose_translation_m'],1000)} / {fmt(summary['max_adjacent_error_jump_m'],1000)} |",
            f"| Reference coverage | {summary['coverage']*100:.1f}% |",
            f"| Stop reason | {summary['stop_reason']} |",'',
            'C uses the same mocap stream for correction and comparison: its result measures consistency, not independent localization accuracy.',
            'All metrics exclude the initial alignment window and cover motion plus the recorded 5-second stop tail.',
            'When timing is unverified, dynamic RMSE is provisional. Endpoint uses the 2–5 s stationary tail; inspect reference_stationary in summary.json.',
            'The maximum raw pose jump contains real motion; maximum error jump also provided to distinguish correction discontinuities.',
            'See summary.json for missing data, endpoint physical overshoot, and full definitions.','']
    (output/'report.md').write_text('\n'.join(report),encoding='utf8')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial',type=Path,required=True)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    manifest=json.loads((args.trial/'manifest.json').read_text())
    group=manifest['group']
    estimates,mocap,events,diagnostics=read_bag(args.trial/'bag',group,manifest['mount'])
    rows,summary=score(estimates,mocap,events,group,manifest['mocap_offset_s'],manifest['timing_verified'])
    summary['trial_status']=manifest['status']
    summary['direction']=manifest.get('direction','forward')
    summary['diagnostics_invalid_count']=sum(not d['valid'] for d in diagnostics)
    write_report(args.output or args.trial/'report',rows,summary)
    print(json.dumps(summary,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
