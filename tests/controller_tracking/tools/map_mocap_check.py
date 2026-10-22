#!/usr/bin/env python3
"""Read-only map/mocap static captures and frozen-transform holdout checks.

No commands, initial poses, parameters or TF are published. Existing project
geometry/alignment functions are reused; normal localization remains unchanged.
"""
import argparse
from collections import Counter,deque
from dataclasses import asdict
import json
import math
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT))
sys.path.insert(0,str(ROOT/'tests/controller_tracking'))
import numpy as np
import yaml
from fastlio_mocap_trajectory_evaluation_core import (
    Se2Alignment, estimate_initial_static_alignment, interpolate_pose_strict, pose_error, _static_spread)
from ndt_match_evaluation_core import rigid_body_to_base_pose,yaw
from controller_tracking.common import digest,stats,write_json
from controller_tracking.state import rotation


def pose_dict(message,stamp_ns):
    p,q=message.position,message.orientation
    return dict(stamp_ns=stamp_ns,valid=True,x=p.x,y=p.y,z=p.z,qx=q.x,qy=q.y,qz=q.z,qw=q.w)


def capture(args):
    if not all(math.isfinite(v) for v in [args.seconds,args.warmup,args.timeout,args.mocap_offset_s]) or args.seconds<=0 or args.warmup<0 or args.timeout<=args.seconds+args.warmup:
        raise ValueError('Invalid capture timing arguments')
    import rclpy
    from rclpy.node import Node
    from rclpy.duration import Duration
    from rclpy.time import Time
    from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
    from tf2_ros import Buffer,TransformException
    from tf2_msgs.msg import TFMessage
    from std_msgs.msg import Bool,String
    from geometry_msgs.msg import PoseStamped,Twist
    from controller_tracking.feedback import PairedMocap,ns
    mount_document=yaml.safe_load(Path(args.mount).read_text());mount=mount_document['rigid_body_to_base_link']
    if mount.get('measured') is not True:raise ValueError('Measured Tracker0->base_link mount required')
    cfg=dict(topics=dict(mocap='/mocap/Tracker0/pose',validity='/mocap/Tracker0/diagnostics'),
             timing=dict(max_age_s=.25,validity_wait_s=.10))
    output=Path(args.output)
    if output.exists():raise ValueError('Capture already exists; do not overwrite observations')
    output.parent.mkdir(parents=True,exist_ok=True)
    rclpy.init();node=Node('map_mocap_static_capture');buffer=Buffer(cache_time=Duration(seconds=20))
    local=deque(maxlen=2000);mocap_history=deque(maxlen=2000);pending=deque();samples=[];raw=[];errors=Counter();ready=[False];mode=[None];final=[]
    def tf_callback(msg):
        for tf in msg.transforms:
            if (tf.header.frame_id,tf.child_frame_id) not in [('map','odom'),('odom','base_link')]:continue
            buffer.set_transform(tf,'map_mocap_readonly')
            if tf.child_frame_id=='base_link':
                t=ns(tf.header.stamp);p,q=tf.transform.translation,tf.transform.rotation
                if local and t<=local[-1]['stamp_ns']:continue
                source=dict(stamp_ns=t,valid=True,x=p.x,y=p.y,z=p.z,qx=q.x,qy=q.y,qz=q.z,qw=q.w)
                local.append(source);pending.append((source,time.monotonic()))
    def good(msg):
        t=ns(msg.header.stamp);p=pose_dict(msg.pose,t)
        p['frame']=msg.header.frame_id;raw.append(p)
        if p['frame']!='mocap_world':errors['unexpected_mocap_frame']+=1;return
        try:
            base=rigid_body_to_base_pose(p,mount['translation_m'],mount['quaternion_xyzw'])
            mocap_history.append(base)
        except ValueError as error:errors[str(error)]+=1
    def invalid(reason):
        errors[reason]+=1;mocap_history.clear()
    paired=PairedMocap(node,cfg,good,invalid)
    node.create_subscription(TFMessage,'/tf',tf_callback,qos_profile_sensor_data)
    node.create_subscription(Bool,'/static_localization/ready',lambda m:ready.__setitem__(0,m.data),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(String,'/teleop/mode',lambda m:mode.__setitem__(0,m.data),10)
    node.create_subscription(Twist,'/reference_velocity',lambda m:final.append([m.linear.x,m.linear.y,m.angular.z]),qos_profile_sensor_data)
    started=time.monotonic();deadline=started+args.timeout;observed_start=None
    try:
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.005)
            while pending:
                local_sample,arrived=pending[0];target=local_sample['stamp_ns'];stamp=target+round(args.mocap_offset_s*1e9)
                if not mocap_history or mocap_history[-1]['stamp_ns']<stamp:
                    if time.monotonic()-arrived<.25:break
                    pending.popleft();errors['no_fresh_mocap_at_local_time']+=1;continue
                pending.popleft()
                if not ready[0]:errors['map_not_initialized']+=1;continue
                base,reason=interpolate_pose_strict(list(mocap_history),stamp,.10)
                if base is None:errors[reason]+=1;continue
                try:
                    tf=buffer.lookup_transform('map','base_link',Time(nanoseconds=target))
                except TransformException:errors['no_synchronous_map_tf']+=1;continue
                t,q=tf.transform.translation,tf.transform.rotation
                mapped=dict(stamp_ns=target,valid=True,x=t.x,y=t.y,z=t.z,qx=q.x,qy=q.y,qz=q.z,qw=q.w)
                processed_ns=node.get_clock().now().nanoseconds
                age=(processed_ns-target)*1e-9
                if not 0<=age<=.25:errors['stale_local_tf']+=1;continue
                if time.monotonic()-started<args.warmup:continue
                samples.append(dict(mocap_base=base,map_base=mapped,local_tf_age_s=age,processed_ros_ns=processed_ns))
                if observed_start is None:observed_start=stamp
            if samples and (samples[-1]['mocap_base']['stamp_ns']-observed_start)*1e-9>=args.seconds:break
    except KeyboardInterrupt:errors['operator_interrupt']+=1
    finally:
        report=dict(source='real_static_capture',calibration_id=args.calibration_id,mount=mount,mount_sha256=digest(mount),
                    mocap_source_clock=mount_document['mocap']['timestamp_source'],camera_timing_verified=False,
                    mocap_offset_s=args.mocap_offset_s,offset_is_user_supplied_not_estimated=True,
                    matching='map TF queried at exact odom->base_link measurement stamps; mocap interpolated only within adjacent valid <=0.10s brackets',
                    source_age_definition='processing ROS time minus this matched local measurement stamp; not latest TF header',
                    target='map',source_frame='mocap_world',reference_point='base_link',
                    no_motion_command_published=True,map_ready=ready[0],mode=mode[0],
                    samples=samples,normalized_tracker_samples=raw,rejections=dict(errors),
                    body_tilt_deg=stats([math.degrees(math.acos(float(np.clip(rotation([s['mocap_base'][k] for k in ('qx','qy','qz','qw')])[2,2],-1,1)))) for s in samples]),
                    final_command_max_abs=float(np.max(abs(np.asarray(final)))) if final else None,
                    completed=bool(samples and (samples[-1]['mocap_base']['stamp_ns']-samples[0]['mocap_base']['stamp_ns'])*1e-9>=args.seconds))
        write_json(output,report);node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    return report


def valid_pairs(document):
    if not document['completed'] or len(document['samples'])<30:raise ValueError('Insufficient synchronized capture')
    pairs=[(s['mocap_base'],s['map_base']) for s in document['samples']]
    stamps=[a['stamp_ns'] for a,b in pairs]
    if any(b<=a or (b-a)*1e-9>.25 for a,b in zip(stamps,stamps[1:])):
        raise ValueError('Capture includes duplicate/backward/long-gap timestamps; recapture a continuous static window')
    if document.get('final_command_max_abs') is None or document['final_command_max_abs']>1e-4:
        raise ValueError('Need observed zero final command during static capture')
    for poses in zip(*pairs):
        spread=_static_spread(list(poses))
        if spread['max_xy_radius_m']>.03 or spread['max_yaw_deviation_rad']>math.radians(2):
            raise ValueError('Capture was not static in one or both sources')
    if any(math.degrees(math.acos(float(np.clip(rotation([a[k] for k in ('qx','qy','qz','qw')])[2,2],-1,1))))>10 for a,b in pairs):
        raise ValueError('Mocap base tilt exceeds planar-test threshold; verify ground plane before SE2 alignment')
    return pairs


def align(capture_path,output):
    document=json.loads(Path(capture_path).read_text());pairs=valid_pairs(document)
    transform,spread=estimate_initial_static_alignment(pairs,.03,math.radians(2))
    c,s=math.cos(transform.yaw_rad),math.sin(transform.yaw_rad)
    inverse=Se2Alignment(-transform.yaw_rad,-c*transform.tx_m-s*transform.ty_m,s*transform.tx_m-c*transform.ty_m)
    report=dict(status='candidate_start_aligned_relative_not_independent_map_calibration',
                capture=str(Path(capture_path).resolve()),capture_sha256=digest(document),mount_sha256=document['mount_sha256'],
                calibration_id=document['calibration_id'],map_from_mocap=asdict(transform),mocap_from_map=asdict(inverse),
                static_spread=spread,source_frame='mocap_world',target_frame='map',reference_point='base_link',
                ground_and_scale_independently_verified=False,camera_timing_verified=False,
                note='Frozen once from this static window; map localization bias at start is absorbed; no TF is broadcast')
    if Path(output).exists():raise ValueError('Candidate already exists; preserve the frozen transform')
    write_json(output,report);return report


def verify(transform_path,captures,output):
    frozen=json.loads(Path(transform_path).read_text());transform=Se2Alignment(**frozen['map_from_mocap']);results=[]
    for path in captures:
        document=json.loads(Path(path).read_text())
        if digest(document)==frozen['capture_sha256']:raise ValueError('Training capture cannot be an independent holdout')
        if document['mount_sha256']!=frozen['mount_sha256'] or document['calibration_id']!=frozen['calibration_id']:
            raise ValueError('Calibration or mount changed; do not mix sessions')
        pairs=valid_pairs(document)
        # No re-alignment: validate the ONE frozen transform against every held-out sample.
        errors=[pose_error(b,transform.apply(a)) for a,b in pairs]
        rows=np.asarray([[e['ex_m'],e['ey_m'],e['e_xy_m'],e['yaw_rad']] for e in errors])
        results.append(dict(capture=str(Path(path).resolve()),samples=len(rows),xy_m=stats(rows[:,2]),
                            yaw_rad=stats(rows[:,3]),map_center=[float(np.mean([b[k] for a,b in pairs])) for k in ('x','y')],
                            mocap_center=[float(np.mean([a[k] for a,b in pairs])) for k in ('x','y')]))
    centers=np.asarray([r['mocap_center'] for r in results]);rank=int(np.linalg.matrix_rank(centers-centers.mean(axis=0),tol=.05)) if len(centers)>1 else 0
    result=dict(source='held_out_static_capture',transform_sha256=digest(frozen),transform_refit=False,
                capture_results=results,heldout_position_spread_rank=rank,
                sufficient_noncollinear_coverage=bool(len(results)>=3 and rank==2),
                limitation='Map observations come from FAST-LIO/NDT, not independently surveyed landmarks; metrics cannot prove absolute calibration or compensate scale/tilt')
    if Path(output).exists():raise ValueError('Verification output exists')
    write_json(output,result);return result


def report_session(session,finalize=False):
    """Regenerate derived holdout reports, retaining every failed raw capture."""
    import hashlib
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    session=Path(session);candidate_path=session/'candidate.json'
    frozen=json.loads(candidate_path.read_text());transform=Se2Alignment(**frozen['map_from_mocap'])
    directory=session/'spatial_report';directory.mkdir(exist_ok=True)
    records=[];usable_centers=[];observed_centers=[];error_arrays=[];origin=None
    fig,axes=plt.subplots(2,1,figsize=(10,7));overlay,ax=plt.subplots(figsize=(7,6))
    train=json.loads(Path(frozen['capture']).read_text())['samples']
    training_center=np.mean([[s['mocap_base']['x'],s['mocap_base']['y']] for s in train],axis=0)
    mapped_training_center=np.mean([[s['map_base']['x'],s['map_base']['y']] for s in train],axis=0)
    ax.scatter(*mapped_training_center,marker='*',s=120,label='training location')
    for capture_path in sorted(session.glob('point0[2-9]_attempt*.json')):
        document=json.loads(capture_path.read_text());quality=True;reason=None
        try:
            if document['mount_sha256']!=frozen['mount_sha256'] or document['calibration_id']!=frozen['calibration_id']:
                raise ValueError('Calibration or mount changed')
            valid_pairs(document)
        except ValueError as error:quality=False;reason=str(error)
        pairs=[(s['mocap_base'],s['map_base']) for s in document['samples']]
        item=dict(capture=capture_path.name,static_quality_passed=quality,quality_failure=reason,
                  capture_complete=document['completed'],rejections=document['rejections'],samples=len(pairs),
                  residuals_are_diagnostic_only=not quality)
        if pairs:
            errors=[pose_error(b,transform.apply(a)) for a,b in pairs]
            e=np.array([[r['ex_m'],r['ey_m'],r['e_xy_m'],r['yaw_rad']] for r in errors])
            mc=np.mean([[a['x'],a['y']] for a,b in pairs],axis=0)
            mapped=np.array([[b['x'],b['y']] for a,b in pairs]);predicted=np.array([[transform.apply(a)['x'],transform.apply(a)['y']] for a,b in pairs])
            static={key:_static_spread([p[i] for p in pairs]) for key,i in [('mocap',0),('map',1)]}
            item.update(xy_m=stats(e[:,2]),yaw_rad=stats(e[:,3]),static_spread=static,
                        mocap_center=mc.tolist(),distance_from_training_m=float(np.linalg.norm(mc-training_center)),
                        max_matched_sample_gap_s=max((b['stamp_ns']-a['stamp_ns'])*1e-9 for (a,_),(b,_) in zip(pairs,pairs[1:])) if len(pairs)>1 else None,
                        source_age_definition=document.get('source_age_definition','legacy: age of latest local TF in receive buffer; not a per-sample latency measurement'))
            observed_centers.append(mc)
            t=np.array([a['stamp_ns'] for a,b in pairs],dtype=np.float64)*1e-9
            if origin is None:origin=t[0]
            # Break figures at gaps too; a connecting line would imply support
            # across an interval that was excluded by the synchronization check.
            pt=[];pe=[]
            for i,stamp in enumerate(t):
                if i and stamp-t[i-1]>.25:pt.append(float('nan'));pe.append([float('nan')]*4)
                pt.append(stamp);pe.append(e[i])
            pt=np.asarray(pt);pe=np.asarray(pe)
            axes[0].plot(pt-origin,pe[:,2]*1000,label=capture_path.stem+(' (quality failed)' if not quality else ''))
            axes[1].plot(pt-origin,np.rad2deg(pe[:,3]),label=capture_path.stem)
            ax.scatter(*mapped.mean(axis=0),marker='o',label=capture_path.stem+' map')
            ax.scatter(*predicted.mean(axis=0),marker='x',label=capture_path.stem+' fixed mocap->map')
            ax.plot([mapped[:,0].mean(),predicted[:,0].mean()],[mapped[:,1].mean(),predicted[:,1].mean()],alpha=.6)
            if quality:usable_centers.append(mc);error_arrays.append(e)
        else:item.update(xy_m=stats([]),yaw_rad=stats([]))
        records.append(item)
    centers=np.asarray(usable_centers)
    rank=int(np.linalg.matrix_rank(centers-centers.mean(axis=0),tol=.05)) if len(centers)>1 else 0
    observed=np.asarray(observed_centers)
    observed_rank=int(np.linalg.matrix_rank(observed-observed.mean(axis=0),tol=.05)) if len(observed)>1 else 0
    for a in axes:
        a.grid()
        if records:a.legend(fontsize=8)
    axes[0].set_ylabel('fixed-transform XY residual [mm]');axes[1].set_ylabel('wrapped yaw residual [deg]');axes[1].set_xlabel('time from first holdout [s]')
    fig.tight_layout();fig.savefig(directory/'residuals.png',dpi=130);plt.close(fig)
    ax.set_aspect('equal');ax.grid();ax.legend(fontsize=7);ax.set_xlabel('map X [m]');ax.set_ylabel('map Y [m]');overlay.tight_layout();overlay.savefig(directory/'point_overlay.png',dpi=130);plt.close(overlay)
    combined=np.vstack(error_arrays) if error_arrays else np.empty((0,4))
    enough=bool(len(usable_centers)>=3 and rank==2)
    status=('completed_geometry_checks_without_absolute_certification' if enough else 'completed_with_insufficient_qualified_coverage') if finalize else 'spatial_checks_in_progress'
    result=dict(status=status,acquisition_complete=finalize,transform_refit=False,transform_content_sha256=digest(frozen),
                transform_file_sha256=hashlib.sha256(candidate_path.read_bytes()).hexdigest(),
                attempts=records,attempt_count=len(records),quality_passed_count=len(usable_centers),
                sufficient_noncollinear_coverage=enough,heldout_rank=rank,all_observed_position_rank=observed_rank,
                quality_passed_pooled_descriptive_xy_m=stats(combined[:,2]),
                precision_acceptance_threshold=None,absolute_calibration_validated=False,
                caveat='Residuals include FAST-LIO/NDT map estimate errors; no independent surveyed map truth. Pooled samples are descriptive, not independent repeats.')
    write_json(directory/'summary.json',result)
    lines=['# 冻结坐标变换的空间验证',f'采集已结束：{finalize}。全过程使用candidate.json，不重新对齐验证点；原始失败尝试保留。',
           '|记录|样本|离训练点 m|静止质量|XY RMSE mm|P95 mm|最大 mm|航向RMSE °|',
           '|---|---:|---:|---|---:|---:|---:|---:|']
    for r in records:
        def value(x,scale=1):return f'{x*scale:.3f}' if x is not None else '缺失'
        lines.append('|'+ '|'.join([r['capture'],str(r['samples']),value(r.get('distance_from_training_m')),
            '通过' if r['static_quality_passed'] else '未通过',value(r['xy_m']['rmse'],1000),value(r['xy_m']['p95_abs'],1000),
            value(r['xy_m']['max_abs'],1000),value(r['yaw_rad']['rmse'],180/math.pi)])+'|')
    lines.extend(['',f'已有 {len(usable_centers)} 个通过质量检查的验证记录；非共线覆盖充分：{result["sufficient_noncollinear_coverage"]}。',
        f'所有实际观测点的空间秩为{observed_rank}，仅合格验证点的空间秩为{rank}；空间分布与连续数据质量分别判断。',
        '质量未通过的行仅作诊断，不纳入可用验证点；没有给定高精度验收阈值，因此不自动宣布高精度通过。',
        'point_overlay.png是map估计与固定变换后动捕的点位叠加，连线只表示两种点位之差；residuals.png记录各次采样残差，不连接采样间的移动过程。',
        '残差包含地图定位的变化，不能单独归因于外参；没有独立测量的地图对应点，不能认证绝对地图标定精度。相机曝光时间/动态延迟、地面与尺度仍未独立核实。'])
    (directory/'report.md').write_text('\n\n'.join(lines)+'\n')
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__);sub=parser.add_subparsers(dest='command',required=True)
    p=sub.add_parser('capture');p.add_argument('--output',required=True);p.add_argument('--mount',default=str(ROOT/'robot_data/config/mocap_tracker0.yaml'));p.add_argument('--seconds',type=float,default=5.);p.add_argument('--warmup',type=float,default=3.);p.add_argument('--timeout',type=float,default=20.);p.add_argument('--mocap-offset-s',type=float,default=0.);p.add_argument('--calibration-id')
    p=sub.add_parser('align');p.add_argument('--capture',required=True);p.add_argument('--output',required=True)
    p=sub.add_parser('verify');p.add_argument('--transform',required=True);p.add_argument('--captures',nargs='+',required=True);p.add_argument('--output',required=True)
    p=sub.add_parser('report');p.add_argument('--session',required=True);p.add_argument('--finalize',action='store_true')
    args=parser.parse_args()
    try:
        if args.command=='capture':result=capture(args);print(json.dumps({k:result[k] for k in ('completed','map_ready','rejections')},ensure_ascii=False))
        elif args.command=='align':print(json.dumps(align(args.capture,args.output),ensure_ascii=False,indent=2))
        elif args.command=='verify':print(json.dumps(verify(args.transform,args.captures,args.output),ensure_ascii=False,indent=2))
        else:
            result=report_session(args.session,args.finalize)
            print(json.dumps({k:result[k] for k in ('attempt_count','quality_passed_count','sufficient_noncollinear_coverage')},ensure_ascii=False))
    except (ValueError,OSError) as error:parser.exit(2,str(error)+'\n')


if __name__=='__main__':main()
