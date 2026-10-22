#!/usr/bin/env python3
"""Real group-2 trial images from recorded data; no pose fitting or CSV export."""
import argparse
from collections import Counter,defaultdict
import json
import math
from pathlib import Path
import shutil
import sqlite3
import sys
import zipfile

import numpy as np
from scipy.spatial.transform import Rotation
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tests/controller_tracking'))
sys.path.insert(0,str(ROOT/'tests/mocap_anchored_mapping'))
import tracking_report
from ndt_endpoint_report import distance_series
from mocap_anchored_mapping.core import load as load_yaml_document,matrix,mocap_base,native_base,checked_session

def load(path):
    path=Path(path)
    return json.loads(path.read_text(encoding='utf-8')) if path.suffix=='.json' else load_yaml_document(path)

def ns(stamp):return stamp.sec*10**9+stamp.nanosec
def pose_h(p):return matrix([p.position.x,p.position.y,p.position.z],[p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w])
def rmse(values):
    a=np.asarray(values,float);return float(np.sqrt(np.mean(a*a))) if len(a) else None
def line(ax,t,y,label=None,maximum_gap_s=.5,**kw):
    t=np.asarray(t,float);y=np.asarray(y,float)
    if not len(t):return
    gaps=np.flatnonzero(np.diff(t)>maximum_gap_s)+1
    ax.plot(np.insert(t,gaps,np.nan),np.insert(y,gaps,np.nan),label=label,**kw)

def recording_files(bags):
    for bag in bags:
        info=load(bag/'metadata.yaml')['rosbag2_bagfile_information']
        for relative in info['relative_file_paths']:
            path=bag/relative
            if path.suffix=='.db3':yield path

def export(trial):
    trial=trial.resolve();data=trial/'data';figures=trial/'figures';figures.mkdir(exist_ok=True)
    meta=load(data/'manifest.json');begin=load(data/'departure.json')['request_ros_ns']
    bags=[data/name for name in meta.get('recording_directories',['bag'])]
    run=Path(meta['run']);cfg=load(run/'config.yaml');extrinsics=load(run/'extrinsics.yaml')
    cal=load(run/'calibration.yaml');session=checked_session(run,Path(meta['localization_session']))
    map_odom=np.asarray(session['T_map_odom']['matrix']);map_mc=np.asarray(cal['T_map_mocap']['matrix'])
    correction=load(data/'middle_correction.json') if (data/'middle_correction.json').exists() else {}
    correction_report=None;transform_schedule=[(-1,map_odom)]
    if correction.get('correction_capture'):
        capture=Path(correction['correction_capture']);correction_report=load(capture/'summary.json')
        from controller_tracking.world_calibration import sha
        if sha(capture/'summary.json')!=load(capture/'summary.lock.json')['sha256']:raise ValueError('Correction report integrity failed')
        transform_schedule.append((correction_report['before']['samples'][-1]['stamp_ns'],np.asarray(correction_report['T_map_odom_after']['matrix'])))
    def transform_at(stamp):
        return next(h for t,h in reversed(transform_schedule) if stamp>=t)
    applied_ns=correction.get('correction_applied_ros_ns')
    terminal=load(data/'navigation_terminal.json');end=terminal['receive_ros_ns'];duration=(end-begin)*1e-9
    assert terminal['status'] in (4,5,6)
    title=f"{meta.get('figure_title','Group 2 test')}, offset {meta['offset_distance_m']:.2f} m"
    result=dict(experiment_display_name=meta['experiment_display_name'],offset_distance_m=meta['offset_distance_m'],navigation_status=terminal['status'],full_route_succeeded=terminal['status']==4,duration_s=duration,correction_performed=correction_report is not None)
    def save(fig,name):
        for ext in ('png','svg'):fig.savefig(figures/(name+'.'+ext),dpi=170)
        plt.close(fig)
    trackout=data/'tracking_analysis'
    if not trackout.exists():tracking_report.export(tracking_report.read_paths(bags),trackout,Path(meta['run'])/'grid/map.yaml',export_csv=False)
    for name in ('trajectory_tracking','tracking_errors','overall_tracking_error'):
        for ext in ('png','svg'):shutil.copy2(trackout/(name+'.'+ext),figures/(name+'.'+ext))
    track=load(trackout/'summary.json');result.update(reference_length_m=track['reference_length_m'],tracking_rmse_mm=track['full_follow_path']['path_distance_m']['rmse']*1000,tracking_samples=track['full_follow_path']['samples'])
    pause_attempt=data/'middle_pause_attempt_01'/'middle_correction.json'
    if correction.get('resume_ros_ns') and pause_attempt.exists():
        pause_ns=load(pause_attempt)['pause_ros_ns'];resume_ns=correction['resume_ros_ns']
        active=[r for r in track['rows'] if not pause_ns<=r['stamp_ns']<resume_ns]
        result.update(active_route_tracking_rmse_mm=rmse([r['path_distance_m'] for r in active])*1000,
            active_route_tracking_samples=len(active),excluded_middle_pause_s=(resume_ns-pause_ns)*1e-9,
            tracking_pause_policy='All recorded samples retained. Active-route metric additionally excludes the explicitly paused middle interval; no alignment or pose refit.')
        fig,axes=plt.subplots(3,1,figsize=(12,8),sharex=True,constrained_layout=True)
        pauses=(resume_ns-pause_ns)*1e-9
        tt=[(r['stamp_ns']-begin)*1e-9-(pauses if r['stamp_ns']>=resume_ns else 0.) for r in active]
        gap=next((i for i,r in enumerate(active) if r['stamp_ns']>=resume_ns),len(active))
        for ax,key,scale,label in zip(axes,['path_distance_m','lateral_m','heading_error_deg'],[1000,1000,1],['Path distance [mm]','Lateral error [mm]','Heading error [deg]']):
            values=[r[key]*scale for r in active]
            for lo,hi in ((0,gap),(gap,len(active))):line(ax,tt[lo:hi],values[lo:hi])
            ax.axvline((pause_ns-begin)*1e-9,color='green',linestyle='--',label='Middle pause removed');ax.set_ylabel(label);ax.grid(alpha=.3);ax.legend()
        axes[-1].set_xlabel('Time with middle stationary pause removed [s]')
        fig.suptitle(title+f": active-route tracking RMSE {result['active_route_tracking_rmse_mm']:.2f} mm; recorded map poses")
        save(fig,'active_route_tracking_error')
    base=[];native=[];mocap=[];diagnostics=defaultdict(list);feedback=[];targets=[];commands=[];last_plot={};types={}
    feedback_topic=meta.get('recorded_wheel_feedback_topic','/joint_states')
    target_topic=meta.get('recorded_wheel_command_topic','/omni_velocity_controller/diagnostics/joint_vel')
    command_topic=meta.get('recorded_final_command_topic','/reference_velocity')
    topics={'/anchored_localization/base_pose_F','/anchored_localization/native_odom','/mocap/Tracker0/pose','/mocap/Tracker0/diagnostics',feedback_topic,target_topic,command_topic}
    for file in recording_files(bags):
        with sqlite3.connect(f'file:{file}?mode=ro&immutable=1',uri=True) as db:
            info={tid:(name,typ) for tid,name,typ in db.execute('SELECT id,name,type FROM topics') if name in topics}
            if not info:continue
            for name,typ in info.values():
                if typ not in types:types[typ]=get_message(typ)
            marks=','.join('?' for _ in info)
            query=f'SELECT topic_id,timestamp,data FROM messages WHERE topic_id IN ({marks}) AND timestamp>=? AND timestamp<=? ORDER BY timestamp'
            for tid,received,blob in db.execute(query,(*info,begin-1_000_000_000,end+27_000_000_000)):
                name,typ=info[tid]
                if name in (feedback_topic,target_topic):
                    if not begin<=received<=end+500_000_000 or received-last_plot.get(name,0)<20_000_000:continue
                    last_plot[name]=received
                msg=deserialize_message(blob,types[typ])
                if name=='/anchored_localization/base_pose_F':
                    if msg.header.frame_id!='odom':raise ValueError('Expected continuous native base pose in odom')
                    base.append((ns(msg.header.stamp),received,msg.pose))
                elif name=='/anchored_localization/native_odom':
                    if msg.header.frame_id!='camera_init' or msg.child_frame_id!='body':raise ValueError('Unexpected native odometry frames')
                    h=native_base(pose_h(msg.pose.pose),cfg,extrinsics)
                    native.append((ns(msg.header.stamp),h[:3,3]))
                elif name=='/mocap/Tracker0/pose':mocap.append((ns(msg.header.stamp),received,msg))
                elif name=='/mocap/Tracker0/diagnostics':
                    statuses=[s for s in msg.status if s.name=='mocap_tracker0_normalizer']
                    valid=bool(statuses) and all(s.level in (0,b'\x00') and {v.key:v.value for v in s.values}.get('valid')=='true' for s in statuses)
                    diagnostics[ns(msg.header.stamp)].append((received,valid))
                elif name==command_topic:commands.append((received,max(abs(msg.linear.x),abs(msg.linear.y),abs(msg.angular.z))))
                elif name==feedback_topic:
                    idx={n:i for i,n in enumerate(msg.name)};names=[f'wheel_{i}_roll' for i in range(1,5)]
                    if all(n in idx for n in names) and len(msg.velocity)==len(msg.name):feedback.append([(received-begin)*1e-9,*[msg.velocity[idx[n]] for n in names]])
                elif len(msg.data)>=8:targets.append([(received-begin)*1e-9,*msg.data[4:8]])
        print('Read',file.parent.name+'/'+file.name,flush=True)
    base=sorted({s:(s,r,p) for s,r,p in base}.values());commands.sort()
    moving=[s for s,value in commands if begin<=s<=end+2_000_000_000 and value>1e-5]
    zero=next((s for s,value in commands if moving and s>moving[-1] and value<=1e-5),None)
    distance_start=moving[0] if moving else begin;distance_end=zero+1_000_000_000 if zero else end
    distance_points=sorted({s:(s,float(p[0]),float(p[1])) for s,p in native}.values()) if native else [(s,p.position.x,p.position.y) for s,r,p in base]
    distance=distance_series([dict(stamp_ns=s,x=x,y=y) for s,x,y in distance_points],distance_start,distance_end)
    distance['source']='Recorded native IMU odometry converted to base_link with the frozen original T_B_I; continuous odom source stamps, no map corrections' if native else 'Recorded /anchored_localization/base_pose_F in continuous native odom; same base_link reference'
    selected=[(s,x,y) for s,x,y in distance_points if distance_start<=s<=distance_end]
    sparse=[]
    for row in selected:
        if not sparse or row[0]-sparse[-1][0]>=1_000_000_000:sparse.append(row)
    if sparse and selected and sparse[-1][0]!=selected[-1][0]:sparse.append(selected[-1])
    sensitivity=distance_series([dict(stamp_ns=s,x=x,y=y) for s,x,y in sparse],distance_start,distance_end,maximum_gap_s=1.5)
    result.update(lio_estimated_total_m=distance['estimated_total_m'],lio_observed_segments_m=distance['observed_segments_distance_m'],lio_1s_sample_polyline_m=sensitivity['observed_segments_distance_m'],lio_distance_complete=distance['complete'],lio_gap_count=len(distance['gaps']),distance_interpretation='Continuous LIO estimate, not independently measured physical travel. Motion-boundary window; gaps >0.5 s are not bridged. 1 s actual-sample decimation is a sensitivity check, not truth.')
    result['map_trajectory_polyline_length_m']=track['map_coordinate_polyline_length_m']
    result['recording_interruption']=meta.get('recording_interruption')
    if correction.get('resume_ros_ns') and pause_attempt.exists():
        active_samples=[dict(stamp_ns=s,x=x,y=y) for s,x,y in distance_points if not pause_ns<=s<resume_ns]
        active_distance=distance_series(active_samples,distance_start,distance_end)
        unexpected_gaps=[g for g in active_distance['gaps'] if not (g['from_ns']<=pause_ns and g['to_ns']>=resume_ns)]
        result.update(lio_active_route_observed_m=active_distance['observed_segments_distance_m'],
            lio_active_route_unexpected_gap_count=len(unexpected_gaps),
            lio_active_route_scope='Native base_link estimated polyline during active route phases. Explicitly stationary middle pause excluded. Remaining source gaps >0.5 s not bridged; not independently measured physical distance.')
        fig,ax=plt.subplots(figsize=(11,5),constrained_layout=True)
        for lo,hi in ((distance_start,pause_ns),(resume_ns,distance_end)):
            rr=[r for r in active_distance['rows'] if lo<=r['stamp_ns']<=hi]
            line(ax,[(r['stamp_ns']-distance_start)*1e-9-((resume_ns-pause_ns)*1e-9 if r['stamp_ns']>=resume_ns else 0.) for r in rr],[r['observed_distance_m'] for r in rr])
        ax.axvline((pause_ns-distance_start)*1e-9,color='green',linestyle='--',label='Middle stationary pause removed')
        ax.axhline(result['reference_length_m'],color='gray',linestyle=':',label='Reference length')
        ax.set(xlabel='Active-route time; middle stationary pause excluded [s]',ylabel='Observed LIO travel estimate [m]',title=f"Active-route LIO observed segments: {result['lio_active_route_observed_m']:.2f} m; source gaps not bridged")
        ax.grid(alpha=.3);ax.legend();save(fig,'active_route_distance')
    if meta.get('continue_despite_native_delay'):
        result['experimental_limitations']=meta.get('experimental_limitations',[])
        result['continuation_speed_limit_mps']=meta.get('continuation_speed_limit_mps')
        result['maximum_native_age_s']=meta.get('maximum_native_age_s')
        result['same_speed_controlled_comparison']=False
    (data/'distance_summary.json').write_text(json.dumps(dict(raw=distance,one_second_sensitivity=sensitivity),indent=2)+'\n')
    fig,ax=plt.subplots(figsize=(11,5),constrained_layout=True)
    line(ax,[r['time_s'] for r in distance['rows']],[r['observed_distance_m'] for r in distance['rows']],'Raw LIO observed segments')
    line(ax,[r['time_s'] for r in sensitivity['rows']],[r['observed_distance_m'] for r in sensitivity['rows']],'1 s actual-sample sensitivity',maximum_gap_s=1.5)
    if meta.get('recording_interruption'):
        rr=track['rows'];xy=np.array([[r['x_m'],r['y_m']] for r in rr]);length=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(xy,axis=0),axis=1))]
        line(ax,[(r['stamp_ns']-distance_start)*1e-9 for r in rr],length,'Retained map trajectory polyline (polling stamps)')
        ax.text(.02,.96,f"Map trajectory polyline: {result['map_trajectory_polyline_length_m']:.2f} m\nNative LIO total unavailable: recording gap",
            transform=ax.transAxes,va='top',bbox=dict(facecolor='white',alpha=.8,edgecolor='gray'))
    ax.axhline(result['reference_length_m'],linestyle='--',color='gray',label='Reference length');ax.set(xlabel='Time from first motion command [s]',ylabel='Estimated distance [m]',title=title+': LIO estimated travel'+('' if distance['complete'] else ' (incomplete coverage)'));ax.grid(alpha=.3);ax.legend();save(fig,'distance_curve')
    fig,axes=plt.subplots(4,2,figsize=(13,10),sharex=True,constrained_layout=True)
    for i in range(4):
        for col,(label,rows) in enumerate([('Target [rad/s]',targets),('Feedback [reported interface units]',feedback)]):
            ax=axes[i,col]
            if rows:
                a=np.array(rows);line(ax,a[:,0],a[:,i+1],label)
            ax.set_ylabel(f'Wheel {i+1} rate');ax.grid(alpha=.3);ax.legend(fontsize=8)
    for ax in axes[-1]:ax.set_xlabel('Time from departure [s]')
    fig.suptitle(title+': rolling-wheel rates; physical feedback scale unverified',fontsize=13);save(fig,'wheel_speed_curves')
    result.update(wheel_feedback_plot_samples=len(feedback),wheel_target_plot_samples=len(targets),wheel_physical_scale_verified=False)
    initial=load(Path(meta['initial_validation'])/'summary.json');endpoint=None
    if (data/'endpoint_validation.json').exists():
        e=load(data/'endpoint_validation.json')
        for cap in e.get('captures',[]):
            if e.get('exit_code')==0 and (Path(cap)/'summary.json').exists():endpoint=load(Path(cap)/'summary.json');break
    fig,axes=plt.subplots(2,2,figsize=(12,7),constrained_layout=True)
    for col,(label,doc) in enumerate([('Initial',initial),('Endpoint',endpoint)]):
        if doc is None:
            axes[0,col].text(.5,.5,'No qualifying static measurement',ha='center',transform=axes[0,col].transAxes);continue
        rows=doc['samples'];tt=[(r['stamp_ns']-rows[0]['stamp_ns'])*1e-9 for r in rows]
        for j,name in [(0,'X'),(1,'Y')]:axes[0,col].plot(tt,[r['XYZ_error_m'][j]*1000 for r in rows],label=name)
        axes[0,col].plot(tt,[r['distance_XY_m']*1000 for r in rows],label='XY norm');axes[0,col].legend();axes[1,col].plot(tt,[r['orientation_error_deg'] for r in rows])
        result[label.lower()+'_xy_rmse_mm']=doc['distance_XY_m']['rmse']*1000
        result[label.lower()+'_position_3d_rmse_mm']=doc['distance_3D_m']['rmse']*1000
        result[label.lower()+'_orientation_rmse_deg']=doc['orientation_error_deg']['rmse']
        # A stable but inverted rigid-body solution can pass a static-spread
        # check. Preserve the measured XY number, flag its reference quality.
        suspect=doc['orientation_error_deg']['rmse']>90
        result[label.lower()+'_reference_suspect']=suspect
        result[label.lower()+'_valid_for_localization_accuracy']=not suspect
        result[label.lower()+'_static_samples']=len(rows);axes[0,col].set_title(f"{label}: XY RMSE {result[label.lower()+'_xy_rmse_mm']:.2f} mm")
        if suspect:
            axes[0,col].set_title(f'{label}: REFERENCE SUSPECT; XY value not validated',color='crimson')
            axes[1,col].text(.02,.94,'Near-inverted 3D orientation; inspect raw Tracker0 pose',transform=axes[1,col].transAxes,va='top',color='crimson',fontsize=9)
    for ax in axes[0]:ax.set_ylabel('Position error [mm]')
    for ax in axes[1]:ax.set(xlabel='Time within static window [s]',ylabel='3D orientation error [deg]')
    for ax in axes.flat:ax.grid(alpha=.3)
    axes[0,0].text(.02,.02,'Same starting placement as mocap initialization',transform=axes[0,0].transAxes,fontsize=8,
        bbox=dict(facecolor='white',alpha=.8,edgecolor='none'))
    fig.suptitle(title+': original static validation, no refit',fontsize=13);save(fig,'initial_endpoint_errors');result['endpoint_static_available']=endpoint is not None
    mount=load(run/'mocap.yaml');truth=[];invalid=Counter()
    for stamp,received,msg in sorted(mocap):
        matches=[r for r,valid in diagnostics.get(stamp,[]) if valid and abs(r-received)<=80_000_000 and 0<=max(r,received)-stamp<=250_000_000]
        if not matches:invalid['unpaired_invalid_or_stale_mocap']+=1;continue
        if msg.header.frame_id!=mount['mocap']['world_frame']:invalid['wrong_mocap_frame']+=1;continue
        if truth and stamp<=truth[-1][0]:invalid['duplicate_or_backward_mocap']+=1;continue
        truth.append((stamp,map_mc@mocap_base(pose_h(msg.pose),cfg,extrinsics)))
    truth_times=np.array([s for s,h in truth],dtype=np.int64);rows=[]
    for stamp,received,pose in base:
        if stamp<begin or not len(truth_times):continue
        j=int(np.searchsorted(truth_times,stamp));indices=[i for i in [j-1,j] if 0<=i<len(truth_times)];i=min(indices,key=lambda i:abs(int(truth_times[i])-stamp))
        if abs(int(truth_times[i])-stamp)>25_000_000:continue
        if applied_ns and transform_schedule[1][0]<=received<=applied_ns+250_000_000:
            invalid['correction_application_boundary_omitted']+=1;continue
        current=transform_schedule[1][1] if applied_ns and received>applied_ns else map_odom
        estimate=current@pose_h(pose);h=truth[i][1];delta=estimate[:3,3]-h[:3,3];yaw=lambda a:math.atan2(a[1,0],a[0,0])
        rows.append(dict(stamp_ns=stamp,mocap_stamp_ns=int(truth_times[i]),time_s=(stamp-begin)*1e-9,xy_error_m=float(np.linalg.norm(delta[:2])),error_xyz_m=delta.tolist(),yaw_error_deg=math.degrees(math.remainder(yaw(estimate)-yaw(h),2*math.pi)),orientation_error_deg=float(np.degrees(Rotation.from_matrix(h[:3,:3].T@estimate[:3,:3]).magnitude())),estimated_map_xyz_m=estimate[:3,3].tolist(),mocap_map_xyz_m=h[:3,3].tolist()))
    # Original validators are independent recorders. Retain their measured
    # static pairs if the general bag recorder missed that window.
    captures=[Path(meta['initial_validation'])]
    for filename in ('middle_static_capture.json','endpoint_validation.json'):
        if (data/filename).exists():
            captures.extend(Path(p) for p in load(data/filename).get('captures',[]))
    for key in ('before_validation','after_validation'):
        if correction.get(key):captures.append(Path(correction[key]))
    seen={r['stamp_ns'] for r in rows};static_pair_additions=0
    for capture in captures:
        if not (capture/'summary.json').exists() or not (capture/'pairs.json').exists():continue
        validation=load(capture/'summary.json')
        for pair in load(capture/'pairs.json'):
            stamp=pair['stamp_ns']
            if stamp<begin or stamp in seen:continue
            current=np.asarray(validation['T_map_odom_used']['matrix']) if validation.get('T_map_odom_used') else transform_at(stamp)
            estimate=current@np.asarray(pair['T_F_B']);h=map_mc@np.asarray(pair['T_M_B']);delta=estimate[:3,3]-h[:3,3]
            yaw=lambda a:math.atan2(a[1,0],a[0,0])
            rows.append(dict(stamp_ns=stamp,mocap_stamp_ns=pair['mocap_stamp_ns'],time_s=(stamp-begin)*1e-9,
                xy_error_m=float(np.linalg.norm(delta[:2])),error_xyz_m=delta.tolist(),
                yaw_error_deg=math.degrees(math.remainder(yaw(estimate)-yaw(h),2*math.pi)),
                orientation_error_deg=float(np.degrees(Rotation.from_matrix(h[:3,:3].T@estimate[:3,:3]).magnitude())),
                estimated_map_xyz_m=estimate[:3,3].tolist(),mocap_map_xyz_m=h[:3,3].tolist(),
                source='original_static_validation_pairs',validation_capture=str(capture)))
            seen.add(stamp);static_pair_additions+=1
    rows.sort(key=lambda r:r['stamp_ns'])
    result['localization_static_pairs_added_from_original_validators']=static_pair_additions
    (data/'localization_samples.json').write_text(json.dumps(dict(fixed_transform=correction_report is None,correction_performed=correction_report is not None,invalid_reasons=dict(invalid),samples=rows),indent=2)+'\n')
    fig,axes=plt.subplots(3,1,figsize=(12,9),sharex=True,constrained_layout=True)
    if rows:
        tt=[r['time_s'] for r in rows]
        for j,label in [(0,'X'),(1,'Y')]:line(axes[0],tt,[r['error_xyz_m'][j]*1000 for r in rows],label)
        line(axes[1],tt,[r['xy_error_m']*1000 for r in rows],'XY norm');line(axes[2],tt,[r['yaw_error_deg'] for r in rows],'Yaw')
    for ax,label in zip(axes,['Signed position [mm]','XY error [mm]','Yaw error [deg]']):ax.set_ylabel(label);ax.grid(alpha=.3);ax.axvline(duration,color='gray',linestyle='--',label='Navigation terminal');ax.legend()
    if correction_report:
        for ax in axes:ax.axvline((applied_ns-begin)*1e-9,color='green',linestyle='--',label='Middle mocap correction acknowledged');ax.legend()
    axes[-1].set_xlabel('Time from departure [s]');fig.suptitle(title+': mocap-visible localization error; gaps retained',fontsize=13);save(fig,'localization_error_curve')
    fig,axes=plt.subplots(3,1,figsize=(12,9),sharex=True,constrained_layout=True)
    if rows:
        tt=[r['time_s'] for r in rows]
        line(axes[0],tt,[r['error_xyz_m'][2]*1000 for r in rows],'Z error')
        line(axes[1],tt,[r['orientation_error_deg'] for r in rows],'Full 3D orientation difference')
        line(axes[2],tt,[r['xy_error_m']*1000 for r in rows],'Raw XY difference')
    for ax,label in zip(axes,['Z error [mm]','3D orientation error [deg]','XY difference [mm]']):
        ax.set_ylabel(label);ax.grid(alpha=.3);ax.axvline(duration,color='gray',linestyle='--',label='Navigation terminal');ax.legend()
    axes[-1].set_xlabel('Time from departure [s]')
    quality_title='near-inverted endpoint pose requires review' if result.get('endpoint_reference_suspect') else '3D position and orientation comparison'
    fig.suptitle(title+': reference quality; '+quality_title,fontsize=13)
    save(fig,'localization_reference_quality')
    segments=[];segment=[]
    for row in rows:
        if segment and row['time_s']-segment[-1]['time_s']>.5:segments.append(segment);segment=[]
        segment.append(row)
    if segment:segments.append(segment)
    windows=[dict(start_s=s[0]['time_s'],end_s=s[-1]['time_s'],samples=len(s),xy_rmse_mm=rmse([r['xy_error_m'] for r in s])*1000) for s in segments]
    middle=[w for w in windows if w['start_s']>10 and w['end_s']<duration-10]
    result.update(mocap_valid_pairs=len(rows),mocap_visibility_windows=windows,middle_coverage_windows=middle,dynamic_error_note='Exact source-header pose/validity pairing; recorded receiver age <=0.25 s and validity arrival separation <=0.08 s; nearest native pose <=25 ms, no interpolation. Camera exposure latency and device clock offset unverified.',initial_error_scope='Subsequent static samples after mocap initialization at the same starting placement; not spatial holdout absolute accuracy.')
    middle_static=None
    if correction.get('before_validation'):
        middle_static=load(Path(correction['before_validation'])/'summary.json')
    if (data/'middle_static_capture.json').exists():
        capture=load(data/'middle_static_capture.json')
        if capture.get('outcome')=='static_measurement_saved_limits_restored':
            for path in capture.get('captures',[]):
                if (Path(path)/'summary.json').exists():middle_static=load(Path(path)/'summary.json');break
    result['middle_static_available']=middle_static is not None
    if middle_static:
        result.update(middle_static_xy_rmse_mm=middle_static['distance_XY_m']['rmse']*1000,middle_static_orientation_rmse_deg=middle_static['orientation_error_deg']['rmse'],middle_static_samples=len(middle_static['samples']),middle_static_reference_suspect=middle_static['orientation_error_deg']['rmse']>90)
    middle_after=load(Path(correction['after_validation'])/'summary.json') if correction.get('after_validation') else None
    if correction_report:
        result.update(middle_correction=correction,middle_before_xy_rmse_mm=middle_static['distance_XY_m']['rmse']*1000 if middle_static else None,middle_fit_before_xy_rmse_mm=correction_report['before']['distance_XY_m']['rmse']*1000,middle_fit_consistency_xy_rmse_mm=correction_report['after_same_window_consistency']['distance_XY_m']['rmse']*1000,
            middle_correction_interpretation='Original static correction changes map->odom once. Fit-window residual is not independent accuracy. Subsequent static validation is separate in time; body/world calibration uncertainty remains.')
    if middle_after:result['middle_after_xy_rmse_mm']=middle_after['distance_XY_m']['rmse']*1000
    fig,ax=plt.subplots(figsize=(11,4),constrained_layout=True)
    if middle_static:
        rr=middle_static['samples'];tt=[(r['stamp_ns']-begin)*1e-9 for r in rr]
        line(ax,tt,[r['distance_XY_m']*1000 for r in rr],'Static XY difference')
        ax.legend();ax.set_title(title+f": midroute STATIC, XY RMSE {result['middle_static_xy_rmse_mm']:.2f} mm")
        if middle_after:
            rr=middle_after['samples'];line(ax,[(r['stamp_ns']-begin)*1e-9 for r in rr],[r['distance_XY_m']*1000 for r in rr],'After correction, separate static samples');ax.legend()
            ax.set_title(title+f": middle before {result['middle_static_xy_rmse_mm']:.2f} / after {result['middle_after_xy_rmse_mm']:.2f} mm")
    elif middle:
        for i,w in enumerate(middle,1):
            rr=[r for r in rows if w['start_s']<=r['time_s']<=w['end_s']];line(ax,[r['time_s'] for r in rr],[r['xy_error_m']*1000 for r in rr],f'Visibility window {i}')
        ax.legend()
    else:ax.text(.5,.5,'MIDROUTE MOCAP MISSING\nNo valid Tracker0 samples in the midroute coverage window.\nMissing data is not zero error.',ha='center',va='center',transform=ax.transAxes,fontsize=13)
    if not middle_static:ax.set_title(title+': midroute mocap visibility comparison; no static measurement')
    ax.set(xlabel='Time from departure [s]',ylabel='XY error [mm]');ax.grid(alpha=.3);save(fig,'middle_localization_error')
    if (data/'observer_final.json').exists():
        after=load(data/'observer_final.json');z=after.get('first_zero_after_terminal_ns')
        if z:
            lo=z+4_000_000_000;hi=z+19_000_000_000;rr=[r for r in rows if lo<=r['stamp_ns']<=hi];gaps=np.diff([r['stamp_ns'] for r in rr])*1e-9
            complete=bool(rr and rr[0]['stamp_ns']-lo<=250_000_000 and hi-rr[-1]['stamp_ns']<=250_000_000 and (not len(gaps) or max(gaps)<=.5))
            suspect=bool(rr and any(r['orientation_error_deg']>90 for r in rr))
            result['endpoint_fixed_15s']=dict(samples=len(rr),complete=complete,reference_suspect=suspect,valid_for_localization_accuracy=complete and not suspect,xy_rmse_mm=rmse([r['xy_error_m'] for r in rr])*1000 if rr else None)
            fig,ax=plt.subplots(figsize=(11,4),constrained_layout=True)
            if rr:line(ax,[(r['stamp_ns']-lo)*1e-9 for r in rr],[r['xy_error_m']*1000 for r in rr],'XY norm')
            ax.set(xlim=(0,15),xlabel='Time in fixed endpoint window [s]',ylabel='Raw XY difference [mm]',title='Endpoint +4 to +19 s: '+('REFERENCE SUSPECT; XY value not validated' if suspect else ('complete' if complete else 'partial / missing data')));ax.grid(alpha=.3);save(fig,'endpoint_fixed_window')
    if result.get('endpoint_reference_suspect'):
        result['measurement_quality_status']='endpoint_mocap_reference_suspect_requires_retest'
        result['endpoint_quality_note']='Raw numerical XY residual retained for audit only. Static endpoint 3D orientation is near inverted; do not use this XY value as a validated localization accuracy result. No orientation repair or transform refit performed.'
    if correction_report:
        fig,axes=plt.subplots(1,2,figsize=(13,6),constrained_layout=True)
        measurements=[('Initial',result.get('initial_xy_rmse_mm')),
                      ('Middle before',result.get('middle_static_xy_rmse_mm')),
                      ('Middle after\nseparate samples',result.get('middle_after_xy_rmse_mm')),
                      ('Endpoint',result.get('endpoint_xy_rmse_mm'))]
        valid=[(label,value) for label,value in measurements if value is not None]
        bars=axes[0].bar([x[0] for x in valid],[x[1] for x in valid],color=['#4267ac','#e59a42','#49a078','#855ca0'][:len(valid)])
        for bar,(_,value) in zip(bars,valid):axes[0].text(bar.get_x()+bar.get_width()/2,bar.get_height(),f'{value:.2f}',ha='center',va='bottom')
        axes[0].set(ylabel='Static XY RMSE [mm]',title='Measured windows; shared world/body calibration');axes[0].margins(y=.18);axes[0].grid(axis='y',alpha=.25)
        axes[1].axis('off')
        note=[f"Original route completed: {result['full_route_succeeded']}",
              f"Reference length: {result['reference_length_m']:.2f} m",
              f"Tracking RMSE (all recorded samples): {result['tracking_rmse_mm']:.2f} mm",
              'Middle correction: applied once; no endpoint correction',
              'After-correction window is separate from correction fit',
              'Unobserved mocap intervals remain gaps']
        if meta.get('continue_despite_native_delay'):
            if result.get('active_route_tracking_rmse_mm') is not None:
                note.append(f"Tracking RMSE (middle pause excluded): {result['active_route_tracking_rmse_mm']:.2f} mm")
            if result.get('lio_active_route_observed_m') is not None:
                note.append(f"Active-route LIO observed estimate: {result['lio_active_route_observed_m']:.2f} m")
            note.extend(['','Continuation conditions:',
                         'Long stationary pause before correction',
                         'Native pose age allowance: 3.0 s; source stamps retained',
                         'After middle: speed <=0.10 m/s, yaw rate <=0.15 rad/s',
                         'Speed differs from prior groups; not a controlled comparison',
                         'Stationary recording split; full LIO distance has missing data'])
        axes[1].text(0,1,'\n'.join(note),transform=axes[1].transAxes,va='top',fontsize=10,linespacing=1.6)
        fig.suptitle(title+': experiment overview');save(fig,'00_experiment_overview')
    (data/'results_summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    package=meta.get('figure_package_name','group2_test_result_figures.zip')
    if Path(package).name!=package:raise ValueError('Expected a ZIP basename')
    with zipfile.ZipFile(trial/package,'w',zipfile.ZIP_DEFLATED) as archive:
        for p in sorted(figures.rglob('*')):
            if p.is_file() and p.suffix in ('.png','.svg') and not p.name.startswith('offset_'):archive.write(p,str(p.relative_to(figures)))
    return result

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--trial',type=Path,required=True)
    print(json.dumps(export(parser.parse_args().trial),ensure_ascii=False,indent=2))
