#!/usr/bin/env python3
"""Export map-estimated robot versus recorded inspection paths; never fit poses."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import sqlite3
import sys

import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tests/controller_tracking'))
from controller_tracking.common import stats,wrap


def project(samples,reference):
    """Exact nearest polyline segment; signed lateral positive to its left."""
    reference=np.asarray(reference,float)
    samples=np.asarray(samples,float)
    if reference.ndim!=2 or reference.shape[1]!=3 or len(reference)<2:
        raise ValueError('Reference must contain at least two x/y/yaw poses')
    if samples.ndim!=2 or samples.shape[1]!=3 or not np.isfinite(samples).all() or not np.isfinite(reference).all():
        raise ValueError('Nonfinite or malformed trajectory')
    delta=np.diff(reference[:,:2],axis=0);length=np.linalg.norm(delta,axis=1)
    valid=length>1e-8
    if not valid.any():raise ValueError('Reference contains no nonzero segments')
    indices=np.flatnonzero(valid);a=reference[:-1,:2][valid];d=delta[valid];sizes=length[valid]
    along=np.r_[0.,np.cumsum(length)]
    rows=[]
    for p in samples:
        fractions=np.clip(np.sum((p[:2]-a)*d,axis=1)/sizes**2,0,1)
        feet=a+fractions[:,None]*d;distances=np.linalg.norm(p[:2]-feet,axis=1)
        j=int(np.argmin(distances));i=int(indices[j]);f=float(fractions[j]);error=p[:2]-feet[j]
        heading=reference[i,2]+f*wrap(reference[i+1,2]-reference[i,2])
        rows.append(dict(segment_index=i,reference_s_m=float(along[i]+f*length[i]),
            projection_x_m=float(feet[j,0]),projection_y_m=float(feet[j,1]),
            path_distance_m=float(distances[j]),lateral_m=float((d[j,0]*error[1]-d[j,1]*error[0])/sizes[j]),
            heading_error_deg=float(math.degrees(wrap(p[2]-heading)))))
    return rows,float(length.sum())


def path_rows(message,actual=False):
    if message.header.frame_id!='map':raise ValueError('Expected paths in original map coordinates')
    result=[];last=None
    for p in message.poses:
        if p.header.frame_id not in ('','map'):raise ValueError('Mixed path frames')
        q=p.pose.orientation;norm=math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
        if not math.isfinite(norm) or abs(norm-1)>1e-3:raise ValueError('Invalid path quaternion')
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        xyz=[p.pose.position.x,p.pose.position.y,yaw]
        if not np.isfinite(xyz).all():raise ValueError('Nonfinite path coordinates')
        stamp=p.header.stamp.sec*10**9+p.header.stamp.nanosec
        if actual and last is not None and stamp<=last:raise ValueError('Nonincreasing actual trajectory timestamps')
        result.append(dict(stamp_ns=stamp,x_m=xyz[0],y_m=xyz[1],yaw_rad=yaw));last=stamp
    return result


def read_paths(bags,execution_index=None):
    """Use one recorded FollowPath execution, never concatenate retries."""
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Path as RosPath
    paths=[p for bag in bags for p in sorted(Path(bag).glob('*.db3'))]
    references={};purple=[]
    for path in paths:
        with sqlite3.connect(f'file:{path.resolve()}?mode=ro',uri=True) as db:
            for topic_id,name in db.execute('SELECT id,name FROM topics'):
                if name not in ('/inspection_route/reference_path','/inspection_path'):continue
                for received,data in db.execute('SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp',(topic_id,)):
                    if name.endswith('/reference_path'):
                        key=hashlib.sha256(data).hexdigest()
                        if key not in references:references[key]=(received,data)
                    else:purple.append((received,data))
    events=sorted(references.values(),key=lambda r:r[0])
    if not events:raise ValueError('No executed reference path recorded yet')
    if execution_index is None:
        if len(events)!=1:raise ValueError(f'{len(events)} executions found; choose --execution-index explicitly')
        execution_index=0
    if not 0<=execution_index<len(events):raise ValueError('Execution index out of range')
    started,data=events[execution_index];end=events[execution_index+1][0] if execution_index+1<len(events) else 2**63-1
    reference=path_rows(deserialize_message(data,RosPath));latest=None
    for path in paths:
        with sqlite3.connect(f'file:{path.resolve()}?mode=ro',uri=True) as db:
            topic=db.execute('SELECT id FROM topics WHERE name=?',('/inspection_route/actual_path',)).fetchone()
            if topic:
                row=db.execute('SELECT timestamp,data FROM messages WHERE topic_id=? AND timestamp>=? AND timestamp<? ORDER BY timestamp DESC LIMIT 1',
                    (topic[0],started,end)).fetchone()
                if row and (latest is None or row[0]>latest[0]):latest=row
    if latest is None:raise ValueError('No actual map trajectory recorded for this execution')
    actual=path_rows(deserialize_message(latest[1],RosPath),actual=True)
    if len(actual)<2:raise ValueError('Too few actual trajectory samples')
    candidates=[r for r in purple if r[0]<=started]
    inspection=path_rows(deserialize_message(max(candidates,key=lambda r:r[0])[1],RosPath)) if candidates else None
    return dict(reference=reference,actual=actual,inspection=inspection,execution_index=execution_index,
        recorded_executions=len(events),reference_receive_ns=started,last_actual_receive_ns=latest[0])


def metrics(rows):
    return dict(samples=len(rows),path_distance_m=stats([r['path_distance_m'] for r in rows]),
        lateral_m=stats([r['lateral_m'] for r in rows]),
        heading_error_deg=stats([r['heading_error_deg'] for r in rows]))


def export(data,output,map_yaml=None,export_csv=True):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.transforms import Affine2D
    actual=data['actual'];ref=data['reference']
    poses=lambda rows:np.array([[r['x_m'],r['y_m'],r['yaw_rad']] for r in rows])
    errors,length=project(poses(actual),poses(ref))
    rows=[dict(time_s=(p['stamp_ns']-actual[0]['stamp_ns'])*1e-9,**p,**e) for p,e in zip(actual,errors)]
    inspection=data.get('inspection');purple_errors=None
    if inspection:
        purple_errors,_=project(poses(actual),poses(inspection))
        for row,p in zip(rows,purple_errors):row['inspection_path_distance_m']=p['path_distance_m']
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    if export_csv:
        with (output/'tracking_samples.csv').open('x',newline='') as f:
            writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    gaps=[dict(from_ns=a['stamp_ns'],to_ns=b['stamp_ns'],seconds=(b['stamp_ns']-a['stamp_ns'])*1e-9)
        for a,b in zip(actual,actual[1:]) if b['stamp_ns']-a['stamp_ns']>500000000]
    result=dict(frame='map',source='recorded /inspection_route/actual_path versus /inspection_route/reference_path',
        interpretation='Map-estimated tracking deviation; not independent mocap localization accuracy',
        timestamp_meaning='inspection node polling time; latest map->base_link TF source time is not retained in this Path',
        transform_fitted=False,alignment_applied=False,reference_length_m=length,
        execution_index=data['execution_index'],recorded_executions=data['recorded_executions'],
        full_follow_path=metrics(rows),inspection_path=metrics(purple_errors) if purple_errors else None,
        gaps=gaps,duration_s=rows[-1]['time_s'],
        first_actual_stamp_ns=actual[0]['stamp_ns'],last_actual_stamp_ns=actual[-1]['stamp_ns'],
        map_coordinate_polyline_length_m=float(np.linalg.norm(np.diff(poses(actual)[:,:2],axis=0),axis=1).sum()),
        distance_note='Map trajectory length includes global localization corrections and noise; use continuous LIO odom distance in endpoint report for travel.',
        projection_note='Nearest finite polyline segment; left-positive geometric lateral; heading interpolates recorded path yaw; ties use earliest segment.',
        rows=rows)
    (output/'summary.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    (output/'reference_paths.json').write_text(json.dumps(dict(reference=ref,inspection=inspection),indent=2)+'\n')
    def save(fig,name):
        for ext in ('png','svg','pdf'):fig.savefig(output/f'{name}.{ext}',dpi=180)
        plt.close(fig)
    def background(ax):
        if not map_yaml:return
        spec=yaml.safe_load(Path(map_yaml).read_text());origin=spec['origin'];resolution=spec['resolution']
        grid=plt.imread(str(Path(map_yaml).resolve().parent/spec['image']))
        height,width=grid.shape[:2]
        transform=Affine2D().rotate(origin[2]).translate(origin[0],origin[1])+ax.transData
        ax.imshow(np.flipud(grid),origin='lower',cmap='gray',vmin=0,vmax=255,
            extent=[0,width*resolution,0,height*resolution],transform=transform,alpha=.45)
    def route_view(ax):
        coordinates=np.vstack([poses(ref)[:,:2],poses(actual)[:,:2]]+([poses(inspection)[:,:2]] if inspection else []))
        ax.set_xlim(float(coordinates[:,0].min())-2,float(coordinates[:,0].max())+2)
        ax.set_ylim(float(coordinates[:,1].min())-2,float(coordinates[:,1].max())+2)
    t=np.array([r['time_s'] for r in rows]);distance=np.array([r['path_distance_m'] for r in rows])
    # Break plot lines at measured gaps; do not interpolate absent data.
    break_indices=[i for i in range(1,len(rows)) if rows[i]['stamp_ns']-rows[i-1]['stamp_ns']>500000000]
    def broken(values):
        times=t.copy();values=np.asarray(values,float).copy()
        return np.insert(times,break_indices,np.nan),np.insert(values,break_indices,np.nan)
    fig,ax=plt.subplots(figsize=(10,8),constrained_layout=True);background(ax)
    if inspection:ax.plot(poses(inspection)[:,0],poses(inspection)[:,1],color='gray',alpha=.5,label='Inspection purple path')
    ax.plot(poses(ref)[:,0],poses(ref)[:,1],color='purple',label='Executed fixed reference')
    xy=poses(actual)[:,:2];ax.plot(np.insert(xy[:,0],break_indices,np.nan),np.insert(xy[:,1],break_indices,np.nan),color='tab:red',label='Map-estimated robot')
    ax.scatter(xy[0,0],xy[0,1],color='green',marker='o',label='Robot start')
    ax.scatter(xy[-1,0],xy[-1,1],color='black',marker='x',label='Robot last sample')
    ax.set(xlabel='map X [m]',ylabel='map Y [m]',title='Full-route tracking');ax.set_aspect('equal');ax.legend();ax.grid(alpha=.2)
    route_view(ax)
    save(fig,'trajectory_tracking')
    fig,axes=plt.subplots(3,1,figsize=(12,9),sharex=True,constrained_layout=True)
    axes[0].plot(*broken(distance),label='Executed fixed reference')
    if purple_errors:axes[0].plot(*broken([r['path_distance_m'] for r in purple_errors]),'--',alpha=.7,label='Inspection purple path')
    axes[0].set_ylabel('Nearest-path distance [m]');axes[0].legend()
    axes[1].plot(*broken([r['lateral_m'] for r in rows]));axes[1].set_ylabel('Signed lateral [m]')
    axes[2].plot(*broken([r['heading_error_deg'] for r in rows]));axes[2].set(ylabel='Heading error [deg]',xlabel='Time from first tracking sample [s]')
    for ax in axes:ax.grid(alpha=.3)
    fig.suptitle('Map-estimated full-route tracking errors');save(fig,'tracking_errors')
    fig,ax=plt.subplots(figsize=(10,8),constrained_layout=True);background(ax)
    ax.plot(poses(ref)[:,0],poses(ref)[:,1],color='purple',alpha=.5,label='Fixed reference')
    points=ax.scatter(xy[:,0],xy[:,1],c=distance,s=10,cmap='viridis');fig.colorbar(points,ax=ax,label='Nearest-path distance [m]')
    ax.set(xlabel='map X [m]',ylabel='map Y [m]',title='Full-route tracking error in map');ax.set_aspect('equal');ax.legend();ax.grid(alpha=.2)
    route_view(ax)
    save(fig,'overall_tracking_error')
    m=result['full_follow_path']
    (output/'report.md').write_text('\n'.join(['# 整圈地图轨迹跟踪误差','',
        '使用原 map 中小车 base_link 估计轨迹与实际下发的固定 FollowPath 比较；不拟合或平移轨迹。该项是跟踪偏差，动捕定位误差见同轮初始/终点报告。','',
        f"参考路径长 {length:.3f} m；{len(rows)} 个实际样本；窗口 {result['duration_s']:.2f} s。",
        f"到路径距离 RMSE {m['path_distance_m']['rmse']:.4f} m，P95 {m['path_distance_m']['p95_abs']:.4f} m，最大 {m['path_distance_m']['max_abs']:.4f} m。",
        f"航向误差 RMSE {m['heading_error_deg']['rmse']:.3f}°。",'',
        '紫线偏差另保留；完整 FollowPath 含接入路径。时间戳为巡检节点采样时刻，未保存对应 TF 源时刻，不用于动态动捕同步。',
        '地图轨迹折线长度含定位修正跳变，实际行驶距离以终点报告的连续 LIO odom 积分为准。',
        f'大于 0.5 秒的采样缺口：{len(gaps)}；图中不跨缺口连线。','',
        '![整圈轨迹](trajectory_tracking.png)','','![整圈跟踪误差](tracking_errors.png)','','![地图整体误差](overall_tracking_error.png)','',
        '图形均导出 PNG/SVG/PDF；'+('tracking_samples.csv 保留逐样本数据。' if export_csv else '本次禁用 CSV 导出。')])+'\n')
    return dict(output=str(output),samples=len(rows),reference_length_m=length,tracking_rmse_m=m['path_distance_m']['rmse'])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bags',nargs='+',required=True);parser.add_argument('--output',required=True)
    parser.add_argument('--map-yaml');parser.add_argument('--execution-index',type=int)
    parser.add_argument('--no-csv',action='store_true')
    args=parser.parse_args()
    try:print(json.dumps(export(read_paths(args.bags,args.execution_index),args.output,args.map_yaml,export_csv=not args.no_csv),indent=2))
    except (ValueError,OSError) as e:parser.exit(2,str(e)+'\n')


if __name__=='__main__':main()
