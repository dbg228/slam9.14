#!/usr/bin/env python3
"""Passive original-NDT endpoint captures and distance/error figures. No goals."""
import argparse
import copy
import csv
import json
import math
from pathlib import Path
import sqlite3
import sys

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tests/controller_tracking'))
import numpy as np
from controller_tracking import world_calibration as wc
from controller_tracking.common import stats


def score_pairs(pairs,transform):
    """Use a previously frozen transform; never fit initial or endpoint data."""
    h=np.asarray(transform,float);rows=[]
    if h.shape!=(4,4) or not np.isfinite(h).all():raise ValueError('Invalid frozen transform')
    if not pairs:raise ValueError('No paired NDT and mocap measurements')
    first=pairs[0]['ndt_stamp_ns']
    for pair in pairs:
        truth=h[:3,:3]@np.asarray(pair['mocap_xyz_m'])+h[:3,3]
        estimate=np.asarray(pair['map_xyz_m']);e=estimate-truth
        true_r=h[:3,:3]@np.asarray(pair['mocap_rotation']);est_r=np.asarray(pair['map_rotation'])
        angle=math.atan2(est_r[1,0],est_r[0,0])-math.atan2(true_r[1,0],true_r[0,0])
        yaw=math.degrees(math.atan2(math.sin(angle),math.cos(angle)))
        rows.append(dict(stamp_ns=pair['ndt_stamp_ns'],time_s=(pair['ndt_stamp_ns']-first)*1e-9,
            ex_m=float(e[0]),ey_m=float(e[1]),ez_m=float(e[2]),xy_m=float(np.linalg.norm(e[:2])),
            distance_3d_m=float(np.linalg.norm(e)),yaw_deg=yaw,match_id=pair.get('match_id'),
            ndt_receive_age_s=pair.get('ndt_receive_age_s')))
    return dict(samples=len(rows),rows=rows,xy_m=stats([r['xy_m'] for r in rows]),
        distance_3d_m=stats([r['distance_3d_m'] for r in rows]),yaw_deg=stats([r['yaw_deg'] for r in rows]),
        mean_error_xyz_m=np.mean([[r['ex_m'],r['ey_m'],r['ez_m']] for r in rows],axis=0).tolist(),
        transform_refitted=False,error_sign='NDT map estimate minus transformed mocap base_link')


def distance_series(samples,start_ns,end_ns,maximum_gap_s=.5):
    """Integrate continuous odom->base_link; map/NDT corrections never enter."""
    if end_ns<=start_ns:raise ValueError('Distance interval must end after departure')
    if not math.isfinite(maximum_gap_s) or maximum_gap_s<=0:raise ValueError('Invalid maximum distance sample gap')
    selected=[p for p in samples if start_ns<=p['stamp_ns']<=end_ns]
    if any(not math.isfinite(p[key]) for p in selected for key in ('x','y')):
        raise ValueError('Nonfinite odometry samples; distance cannot be reported')
    selected.sort(key=lambda p:p['stamp_ns'])
    unique=[];duplicates=0
    for p in selected:
        if unique and p['stamp_ns']==unique[-1]['stamp_ns']:
            duplicates+=1;continue
        unique.append(p)
    rows=[];distance=0.;gaps=[];largest_step=0.
    for i,p in enumerate(unique):
        if i:
            previous=unique[i-1];dt=(p['stamp_ns']-previous['stamp_ns'])*1e-9
            step=math.hypot(p['x']-previous['x'],p['y']-previous['y']);largest_step=max(largest_step,step)
            if dt>maximum_gap_s:gaps.append(dict(from_ns=previous['stamp_ns'],to_ns=p['stamp_ns'],seconds=dt))
            else:distance+=step
        rows.append(dict(stamp_ns=p['stamp_ns'],time_s=(p['stamp_ns']-start_ns)*1e-9,observed_distance_m=distance))
    complete=bool(len(unique)>1 and not gaps and
        (unique[0]['stamp_ns']-start_ns)*1e-9<=maximum_gap_s and
        (end_ns-unique[-1]['stamp_ns'])*1e-9<=maximum_gap_s)
    return dict(source='continuous odom->base_link from recorded /tf',
        label='LIO estimated accumulated planar travel; not mocap ground-truth distance',
        samples=len(unique),rows=rows,estimated_total_m=distance if complete else None,
        observed_segments_distance_m=distance,complete=complete,gaps=gaps,duplicate_stamps=duplicates,
        largest_step_m=largest_step,start_ns=start_ns,end_ns=end_ns,
        initial_to_final_displacement_m=math.hypot(unique[-1]['x']-unique[0]['x'],unique[-1]['y']-unique[0]['y']) if len(unique)>1 else None,
        caveat='Unfiltered odometry integration includes pose noise. Long missing intervals are not bridged; global NDT correction jumps are excluded by choosing odom coordinates.')


SENSOR_MOUNT=(.37,.28,.8229227423153264)


def raw_tf_base_xy(t,mount=SENSOR_MOUNT):
    """Same measured planar lever arm as localization.sh/lio_tf_to_base_tf.py."""
    q=t.transform.rotation;p=t.transform.translation
    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))-mount[2]
    return p.x-(math.cos(yaw)*mount[0]-math.sin(yaw)*mount[1]),p.y-(math.sin(yaw)*mount[0]+math.cos(yaw)*mount[1])


def read_distance(bags,start_ns,end_ns):
    from rclpy.serialization import deserialize_message
    from tf2_msgs.msg import TFMessage
    samples=[];native=[]
    for bag in bags:
        for path in sorted(Path(bag).glob('*.db3')):
            with sqlite3.connect(f'file:{path.resolve()}?mode=ro',uri=True) as db:
                for topic_id,name in db.execute('SELECT id,name FROM topics WHERE name IN (?,?)',('/tf','/tf_lio_raw')).fetchall():
                    cursor=db.execute('SELECT data FROM messages WHERE topic_id=? AND timestamp>=? AND timestamp<=? ORDER BY timestamp',
                        (topic_id,start_ns,end_ns+30*10**9))
                    for data, in cursor:
                        msg=deserialize_message(data,TFMessage)
                        for t in msg.transforms:
                            stamp=t.header.stamp.sec*10**9+t.header.stamp.nanosec
                            if not start_ns<=stamp<=end_ns:continue
                            frames=(t.header.frame_id,t.child_frame_id)
                            if name=='/tf' and frames==('odom','base_link'):
                                samples.append(dict(stamp_ns=stamp,x=t.transform.translation.x,y=t.transform.translation.y))
                            elif name=='/tf_lio_raw' and frames==('camera_init','body'):
                                x,y=raw_tf_base_xy(t);native.append(dict(stamp_ns=stamp,x=x,y=y))
    primary=distance_series(samples,start_ns,end_ns)
    if not native:return primary
    by_stamp={r['stamp_ns']:r for r in samples}
    differences=[math.hypot(r['x']-by_stamp[r['stamp_ns']]['x'],r['y']-by_stamp[r['stamp_ns']]['y'])
        for r in native if r['stamp_ns'] in by_stamp]
    if not differences or max(differences)>1e-5:
        raise ValueError('Native TF body mount cannot be verified against simultaneous recorded base TF')
    result=distance_series(samples+native,start_ns,end_ns)
    result.update(source='Recorded continuous odom->base_link plus camera_init->body converted by existing measured sensor mount',
        raw_sensor_mount_xy_yaw=list(SENSOR_MOUNT),native_samples=len(native),
        mount_overlap_samples=len(differences),maximum_mount_overlap_difference_m=max(differences),
        recorded_base_tf_only=dict(samples=primary['samples'],complete=primary['complete'],gaps=len(primary['gaps'])),
        native_recovery='Recover same-stamp missing base TF from recorded native TF, using original bridge geometry; no interpolation, fitting, map corrections, or retimestamping')
    return result


def capture(calibration_session,output,phase,maximum_ndt_age_s=None,static_history_pairing=False):
    """A separate capture session can only evaluate, not modify registration."""
    from controller_tracking.world_capture import capture_static
    source=Path(calibration_session).resolve();calibration=wc.check_calibration(source)
    inputs=wc.frozen_inputs(source);cfg=copy.deepcopy(inputs['config'])
    if maximum_ndt_age_s is not None:
        if not math.isfinite(maximum_ndt_age_s) or maximum_ndt_age_s<=0:raise ValueError('Invalid static NDT age bound')
        cfg['limits']['maximum_ndt_source_age_s']=maximum_ndt_age_s
        cfg['static_pairing_note']='NDT processing age bound explicitly set before capture; original source stamps, valid static mocap history, skew and motion checks retained; no control freshness or estimator parameters changed.'
    if static_history_pairing:
        cfg['static_history_pairing']=True;cfg['retain_mocap_warmup']=True
        cfg['limits']['warmup_s']=max(10.,cfg['limits']['warmup_s'])
        cfg['static_pairing_note']='Evaluation only: delayed NDT outputs must match real valid mocap samples within retained stationary history. Record receive age without changing stamps. Missing contemporaneous reference, motion, invalid mocap or synchronization still fails; not a live control freshness claim.'
    if cfg['map_source']!='ndt':raise ValueError('This original-NDT capture needs NDT map observations')
    current=wc.load(ROOT/'robot_data/config/mocap_world_session.yaml')['camera_world_id']
    if cfg.get('camera_world_id')!=current:raise ValueError('Camera world changed; establish a new registration')
    output=Path(output).resolve()
    if output.exists():raise ValueError('Preserve existing captures; use a new output directory')
    output.mkdir(parents=True)
    cfg.update(evaluation_only=True,fixed_calibration_session=str(source))
    cfg['map_reference_files']=[*cfg['map_reference_files'],str(source/'calibration.yaml'),str(source/'calibration.lock.json'),str(source/'frozen_inputs.json')]
    wc.save(output/'session.yaml',cfg,True);wc.save(output/'body_extrinsic.yaml',inputs['body_extrinsic'],True)
    wc.freeze_inputs(output)
    result=capture_static(output,phase)
    return dict(quality_passed=result['quality_passed'],failures=result['failures'],
        summary=str(output/phase/'summary.json'),pair_count=result['pair_count'],
        calibration_sha256=wc.sha(source/'calibration.yaml'),transform_refitted=False)


def load_evaluation(path,calibration_session):
    path=Path(path).resolve();document=wc.get_group(path.parent.parent,path.parent.name)
    if not document.get('evaluation_only') or document['calibration_sha256_at_capture']!=wc.sha(Path(calibration_session)/'calibration.yaml'):
        raise ValueError('Endpoint/initial capture did not use this frozen calibration')
    return document


def location_coverage(document,calibration_session):
    """Separate fresh evaluation data from a spatially unseen placement."""
    source=Path(calibration_session)
    center=np.mean([p['mocap_xyz_m'] for p in document['matched_pairs']],axis=0)
    distances={group:float(np.linalg.norm(center-np.asarray(wc.get_group(source,group)['mocap_xyz_m'])))
        for group in wc.TRAIN}
    closest=min(distances,key=distances.get)
    threshold=wc.frozen_inputs(source)['config']['limits']['holdout_minimum_separation_m']
    return dict(nearest_training_group=closest,separation_m=distances[closest],
        spatially_held_out=distances[closest]>=threshold,separation_threshold_m=threshold,
        evaluation_data_used_for_fitting=False)


def figures(output,initial,endpoint,distance):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    phases=([('Initial',initial)] if initial else [])+([('Endpoint',endpoint)] if endpoint else [])
    fig,axes=plt.subplots(2,len(phases),figsize=(6*len(phases),7),squeeze=False,sharey='row',constrained_layout=True)
    for col,(label,data) in enumerate(phases):
        t=[r['time_s'] for r in data['rows']]
        for key,name in [('ex_m','X'),('ey_m','Y'),('xy_m','XY norm')]:
            axes[0,col].plot(t,[r[key]*1000 for r in data['rows']],'-o',label=name)
        axes[0,col].set(title=label+' position error',ylabel='Error [mm]');axes[0,col].legend()
        axes[1,col].plot(t,[r['yaw_deg'] for r in data['rows']],'-o')
        axes[1,col].set(title=label+' heading error',ylabel='Yaw error [deg]',xlabel='Time within measurement window [s]')
        for ax in axes[:,col]:ax.grid(alpha=.3)
    fig.suptitle('Original NDT; frozen registration; measured samples only')
    for ext in ('png','svg','pdf'):fig.savefig(output/('initial_endpoint_errors.'+ext),dpi=180)
    plt.close(fig)
    if distance:
        fig,ax=plt.subplots(figsize=(10,4),constrained_layout=True)
        # Split lines at gaps instead of drawing apparent motion across missing data.
        segments=[[]]
        for row in distance['rows']:
            if segments[-1] and row['stamp_ns']-segments[-1][-1]['stamp_ns']>500000000:segments.append([])
            segments[-1].append(row)
        for segment in segments:
            ax.plot([r['time_s'] for r in segment],[r['observed_distance_m'] for r in segment])
        ax.set(title='Accumulated odometry travel estimate',xlabel='Time from departure [s]',ylabel='Estimated travel [m]');ax.grid(alpha=.3)
        for ext in ('png','svg','pdf'):fig.savefig(output/('distance_curve.'+ext),dpi=180)
        plt.close(fig)


def report(args):
    source=Path(args.calibration_session).resolve();calibration=wc.check_calibration(source)
    transform=calibration['map_from_mocap']['homogeneous_matrix']
    if not args.initial and not args.endpoint:raise ValueError('Provide at least one valid initial or endpoint capture')
    initial=None;endpoint=None;distance=None;coverage={}
    if args.initial:
        first=load_evaluation(args.initial,source)
        if first['group']!='initial':raise ValueError('Initial capture phase mismatch')
        initial=score_pairs(first['matched_pairs'],transform)
        initial['estimator_instability_warnings']=first.get('estimator_instability_warnings',[])
        initial['ndt_age_policy']=first.get('ndt_age_policy','bounded_receive_age')
        coverage['initial']=location_coverage(first,source)
    if args.endpoint:
        last=load_evaluation(args.endpoint,source)
        if last['group']!='endpoint':raise ValueError('Endpoint capture phase mismatch')
        endpoint=score_pairs(last['matched_pairs'],transform)
        endpoint['estimator_instability_warnings']=last.get('estimator_instability_warnings',[])
        endpoint['ndt_age_policy']=last.get('ndt_age_policy','bounded_receive_age')
        coverage['endpoint']=location_coverage(last,source)
        start=args.motion_start_ns if args.motion_start_ns is not None else (initial['rows'][-1]['stamp_ns'] if initial else None)
        end=args.motion_end_ns or endpoint['rows'][0]['stamp_ns']
        if start is not None and end<=start:raise ValueError('Endpoint precedes departure')
        if args.bags:
            if start is None:raise ValueError('Missing initial capture: provide the independently recorded --motion-start-ns for distance')
            distance=read_distance(args.bags,start,end)
    output=Path(args.output).resolve();output.mkdir(parents=True,exist_ok=False)
    result=dict(status='endpoint_completed' if endpoint else 'initial_only_endpoint_pending',
        reference='current map/mocap fixed registration; NDT participated in registration',
        independent_absolute_accuracy_certified=False,calibration_sha256=wc.sha(source/'calibration.yaml'),
        transform_refitted=False,initial=initial,endpoint=endpoint,distance=distance,location_coverage=coverage,
        initial_capture=str(Path(args.initial).resolve()) if args.initial else None,
        initial_missing_reason=getattr(args,'initial_missing_reason',None) if not initial else None,
        endpoint_capture=str(Path(args.endpoint).resolve()) if args.endpoint else None)
    wc.save(output/'summary.json',result);figures(output,initial,endpoint,distance)
    for name,data in [('initial_error_samples',initial),('endpoint_error_samples',endpoint),('distance_samples',distance)]:
        if data and data['rows']:
            with (output/(name+'.csv')).open('x',newline='') as f:
                writer=csv.DictWriter(f,fieldnames=list(data['rows'][0]));writer.writeheader();writer.writerows(data['rows'])
    lines=['# 原 NDT 全图测试结果','',
        '先用三个停车位置建立新固定坐标关系；随后初始与终点评价数据不参与拟合。NDT参与坐标标定，结果是该固定关系下的一致性误差，不认证独立绝对精度。','',
        '| 窗口 | 样本 | XY RMSE mm | XY最大 mm | 航向RMSE ° |','|---|---:|---:|---:|---:|']
    for name,data in [('初始',initial),('终点',endpoint)]:
        if data:lines.append(f"|{name}|{data['samples']}|{data['xy_m']['rmse']*1000:.3f}|{data['xy_m']['max_abs']*1000:.3f}|{data['yaw_deg']['rmse']:.4f}|")
        else:lines.append(f'|{name}|缺测或待采集|—|—|—|')
    if not initial:lines+=['','初始定位误差缺测：'+(result['initial_missing_reason'] or '无通过质量检查的本轮初始动捕窗口')+'。不使用历史窗口替代。']
    if endpoint and endpoint.get('ndt_age_policy')=='same_timestamp_valid_static_history':
        lines+=['','终点使用同一测量时刻的有效静止动捕历史配对，保留迟到 NDT 输出的实际接收延迟。所有时间戳不变；仍检查真值静止、流有效性、时间配对及同步。不据此声称实时控制新鲜度达标。']
    for name,data in [('初始',initial),('终点',endpoint)]:
        if data and data.get('estimator_instability_warnings'):
            lines+=['',name+'窗口中 NDT 估计有静止波动；已保留全部时间配对样本用于误差统计，未按估计误差大小筛选窗口。']
    for phase,name in [('initial','初始'),('endpoint','终点')]:
        placement=coverage.get(phase)
        if placement and not placement['spatially_held_out']:
            lines+=['',f"{name}位置距标定点 {placement['nearest_training_group']} 仅 {placement['separation_m']:.3f} m：这是冻结后另采的时间留样，位置不独立于标定点。"]
    if distance:
        value=distance['estimated_total_m']
        lines+=['',f'累计里程计估计路程：{value:.3f} m。' if value is not None else f"里程数据不完整；已观测连续段累计 {distance['observed_segments_distance_m']:.3f} m，不作为整圈总距离。",'该距离从连续 odom→base_link 积分，不是起终点直线距离，也不是动捕实测全程里程。','', '![累计里程曲线](distance_curve.png)']
    lines+=['','![初始和终点误差曲线](initial_endpoint_errors.png)','',
        '曲线只连接真实配对的NDT测量样本；不补造高频数据。PNG、SVG、PDF与逐样本CSV均已导出。多帧仅为单次停车窗口的描述统计。']
    (output/'report.md').write_text('\n'.join(lines)+'\n')
    return dict(output=str(output),status=result['status'],initial_xy_rmse_m=initial['xy_m']['rmse'] if initial else None,
        endpoint_xy_rmse_m=endpoint['xy_m']['rmse'] if endpoint else None,
        estimated_travel_m=distance['estimated_total_m'] if distance else None)


def main():
    parser=argparse.ArgumentParser(description=__doc__);sub=parser.add_subparsers(dest='command',required=True)
    p=sub.add_parser('capture');p.add_argument('--calibration-session',required=True);p.add_argument('--output',required=True)
    p.add_argument('--phase',choices=['initial','endpoint'],required=True)
    p.add_argument('--maximum-ndt-age-s',type=float,help='Static pairing only; does not change mocap age checks or ROS control parameters')
    p.add_argument('--static-history-pairing',action='store_true',help='Evaluate delayed estimates only when same-time valid stationary reference history exists')
    p=sub.add_parser('report');p.add_argument('--calibration-session',required=True);p.add_argument('--initial')
    p.add_argument('--initial-missing-reason')
    p.add_argument('--endpoint');p.add_argument('--bags',nargs='+');p.add_argument('--output',required=True)
    p.add_argument('--motion-start-ns',type=int);p.add_argument('--motion-end-ns',type=int)
    args=parser.parse_args()
    try:
        result=capture(args.calibration_session,args.output,args.phase,args.maximum_ndt_age_s,args.static_history_pairing) if args.command=='capture' else report(args)
        print(json.dumps(result,ensure_ascii=False,indent=2));return 0 if result.get('quality_passed',True) else 2
    except (ValueError,OSError,KeyError) as exc:parser.exit(2,str(exc)+'\n')


if __name__=='__main__':raise SystemExit(main())
