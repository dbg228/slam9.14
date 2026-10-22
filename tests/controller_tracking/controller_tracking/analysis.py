"""One offline evaluator for all controllers; segment projection, no trajectory fitting."""
import json
import math
from pathlib import Path
from collections import defaultdict,Counter
import numpy as np
import yaml
from .common import interpolate, project, stats, wrap, write_json, digest


def header_s(msg):
    s=msg['header']['stamp'];return s['sec']+s['nanosec']*1e-9


def yaw(q):return math.atan2(2*(q['w']*q['z']+q['x']*q['y']),1-2*(q['y']**2+q['z']**2))


def pose_row(msg):
    p=msg['pose'];p=p.get('pose',p)
    return [header_s(msg),p['position']['x'],p['position']['y'],yaw(p['orientation'])]


def twist(v):return [v['linear']['x'],v['linear']['y'],v['angular']['z']]


def read_bag(path,selected):
    """Stream only required types; reduce high-rate wheel samples for plots, retain counts."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from rosidl_runtime_py.convert import message_to_ordereddict
    metadata=yaml.safe_load((Path(path)/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path),storage_id=metadata['storage_identifier']),rosbag2_py.ConverterOptions('',''))
    types={t.name:t.type for t in reader.get_all_topics_and_types()}
    decoders={t:get_message(typ) for t,typ in types.items() if t in selected};last={};counts=Counter()
    while reader.has_next():
        topic,data,received=reader.read_next();counts[topic]+=1
        if topic not in decoders:continue
        # Raw bag stays full rate. Analysis figures use <=100Hz wheel/command sampling.
        if topic in ('/joint_states','/reference_velocity') and received-last.get(topic,0)<10_000_000:continue
        last[topic]=received
        yield dict(topic=topic,receive_s=received*1e-9,msg=message_to_ordereddict(deserialize_message(data,decoders[topic])))


def monotonic(rows):
    good=[];rejected=0
    for row in rows:
        if good and row[0]<=good[-1][0]:rejected+=1;continue
        good.append(row)
    return np.asarray(good,float).reshape(-1,4),rejected


def match_columns(rows,times,width,gap):
    """Scalar/vector interpolation for wheel feedback, invalid across large gaps."""
    out=np.full((len(times),width),np.nan)
    if not rows:return out
    a=np.asarray(rows,float)
    for i,t in enumerate(times):
        j=np.searchsorted(a[:,0],t)
        if j<len(a) and abs(a[j,0]-t)<1e-7:out[i]=a[j,1:];continue
        if j==0 or j==len(a) or not 0<a[j,0]-a[j-1,0]<=gap:continue
        f=(t-a[j-1,0])/(a[j,0]-a[j-1,0]);out[i]=a[j-1,1:]+f*(a[j,1:]-a[j-1,1:])
    return out


def analyze_records(records,path,cfg,execution,out,source='real_bag',production=None):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    out=Path(out);out.mkdir(parents=True,exist_ok=False)
    by=defaultdict(list)
    for record in records:by[record['topic']].append(record)
    receive=[r['receive_s'] for rr in by.values() for r in rr]
    if not receive:raise ValueError('Empty recording')
    started=execution.get('start_ros_ns');ended=execution.get('end_ros_ns')
    t0=started*1e-9 if started else min(receive);t1=ended*1e-9 if ended else max(receive)
    if t1<=t0:t1=t0+.01
    grid=np.arange(t0,t1+.0001,.01);gap=cfg['timing']['max_gap_s'];topics=cfg['topics'];warnings=[]
    raw,reject=monotonic([pose_row(r['msg']) for r in by[topics['base_pose']]])
    # Invalid diagnostics split interpolation support even when gaps are short.
    invalid=[]
    for r in by[topics['validity']]:
        m=r['msg'];s=header_s(m)
        def level(value):return ord(value) if isinstance(value,str) else int.from_bytes(value,'little') if isinstance(value,bytes) else value
        if s>0 and any(level(st['level'])!=0 for st in m['status']):invalid.append(s)
    truth=interpolate(raw,grid,gap)
    for bad in invalid:
        j=np.searchsorted(raw[:,0],bad) if len(raw) else 0
        lo=raw[j-1,0] if j else bad;hi=raw[j,0] if j<len(raw) else bad
        truth[(grid>=lo)&(grid<=hi)]=np.nan
    errors=project(truth,path);valid=np.isfinite(errors[:,1]);s=errors[:,0]
    phase_index=np.searchsorted(path['local']['boundaries_s'],s,side='right')
    names=['line'] if path['kind']=='line' else ['lead','entry','arc','exit','tail']
    phases=np.array([names[min(int(i),len(names)-1)] for i in phase_index]);phases[~valid]='missing'
    states=[]
    for r in by[topics['state']]:
        m=r['msg'];states.append([header_s(m),*twist(m['twist']['twist'])])
    speed=match_columns(states,grid,3,gap);speed[~valid]=np.nan
    input_pose,rejected_input=monotonic([pose_row(r['msg']) for r in by['/tracking/input_pose']])
    # This comparison describes controller input versus source, not independent localization accuracy.
    used=interpolate(input_pose,grid,gap)
    state_error=used-truth;state_error[:,2]=wrap(state_error[:,2])
    summary=dict(source=source,path_kind=path['kind'],path_sha256=path['points_sha256'],
        config_sha256=path['config_sha256'],frame_sha256=path.get('frame_sha256'),
        execution=execution,window_s=[t0,t1],motion_goal_sent=started is not None,
        reference_data_coverage=float(np.mean(valid)),
        reference_data_coverage_definition='regular-grid support from normalized-valid, mounted mocap; no camera-native optical tracking confidence is available',
        valid_truth_samples=int(valid.sum()),grid_samples=len(grid),
        rejected_truth_timestamps=reject,rejected_controller_input_timestamps=rejected_input,
        lateral_m=stats(errors[:,1]),heading_rad=stats(errors[:,2]),speed_mps=stats(speed[:,0]),
        angular_speed_rad_s=stats(speed[:,2]),controller_input_vs_source_xy_m=stats(np.linalg.norm(state_error[:,:2],axis=1)),
        phases={},turn_response={},limits={},wheel={},warnings=warnings)
    timing_sources={}
    for key in ('raw_mocap','mocap','base_pose','state'):
        stamps=[header_s(r['msg']) for r in by[topics[key]] if t0<=header_s(r['msg'])<=t1]
        dt=np.diff(stamps)
        timing_sources[key]=dict(samples=len(stamps),nonincreasing_count=int(sum(dt<=0)),
            long_gap_count=int(sum(dt>gap)),max_gap_s=float(max(dt)) if len(dt) else None,
            estimated_missing_at_expected_rate=int(sum(np.maximum(0,np.rint(dt*cfg['timing']['expected_hz'])-1))) if len(dt) else 0)
    summary['sampling']=timing_sources
    summary['controller_pose_coverage']=float(np.mean(np.all(np.isfinite(used),axis=1)))
    reasons=Counter();fit_delays=[]
    for r in by[topics['state_status']]:
        if not t0<=r['receive_s']<=t1:continue
        e=json.loads(r['msg']['data']);reasons[e['reason']]+=1
        if e.get('fit_delay_s') is not None:fit_delays.append(e['fit_delay_s'])
    summary['feedback_status_sample_counts']=dict(reasons)
    summary['fit_delay_s']=stats(fit_delays)
    for name in names:
        mask=phases==name
        summary['phases'][name]=dict(samples=int(mask.sum()),lateral_m=stats(errors[mask,1]),
            heading_rad=stats(errors[mask,2]),speed_mps=stats(speed[mask,0]),
            first_s=float(grid[mask][0]-t0) if mask.any() else None,
            last_s=float(grid[mask][-1]-t0) if mask.any() else None)
    def axes_plot(name,rows,ylabel):
        fig,axes=plt.subplots(len(rows),1,figsize=(10,max(3,2.7*len(rows))),squeeze=False)
        for ax,(label,series) in zip(axes[:,0],rows):
            for title,t,y in series:
                if len(t):ax.plot(np.asarray(t)-t0,y,label=title)
            for phase in ('entry','arc','exit'):
                p=summary['phases'].get(phase,{})
                if p.get('first_s') is not None:ax.axvspan(p['first_s'],p['last_s'],alpha=.08,label=phase)
            ax.set_ylabel(label);ax.grid();handles,_=ax.get_legend_handles_labels()
            if handles:ax.legend(fontsize=8)
        axes[-1,0].set_xlabel('time from goal / recording start [s]');fig.tight_layout();fig.savefig(out/name,dpi=130);plt.close(fig)
    fig,ax=plt.subplots(figsize=(7,6));ref=np.array(path['points'])
    ax.plot(ref[:,0],ref[:,1],label='fixed reference');ax.plot(truth[:,0],truth[:,1],label='mocap base_link')
    ax.plot(used[:,0],used[:,1],alpha=.6,label='actual MPPI input pose');ax.axis('equal');ax.grid();ax.legend();ax.set_xlabel('tracking_world X [m]');ax.set_ylabel('Y [m]');fig.tight_layout();fig.savefig(out/'trajectory.png',dpi=130);plt.close(fig)
    axes_plot('errors.png',[('lateral [m]',[('segment error',grid,errors[:,1])]),('heading [rad]',[('wrapped heading error',grid,errors[:,2])])],None)
    axes_plot('speed.png',[('vx [m/s]',[('mocap estimate',grid,speed[:,0])]),('vy [m/s]',[('mocap estimate',grid,speed[:,1])]),('wz [rad/s]',[('mocap estimate',grid,speed[:,2])])],None)
    commands={}
    for name in ('controller_cmd','smoothed_cmd','gate_cmd','final_cmd'):
        commands[name]=[[r['receive_s'],*twist(r['msg'])] for r in by[topics[name]] if t0<=r['receive_s']<=t1]
    commands['actual_MPPI_input_speed']=[[header_s(r['msg']),*twist(r['msg']['twist'])] for r in by['/tracking/input_speed']]
    plotrows=[]
    for col,label in ((1,'vx [m/s]'),(3,'wz [rad/s]')):
        series=[('mocap feedback',grid,speed[:,col-1])]
        for name,rr in commands.items():
            if rr:
                a=np.array(rr);series.append((name,a[:,0],a[:,col]))
        plotrows.append((label,series))
    axes_plot('commands.png',plotrows,None)
    # Output cadence is not compute duration. Both are measured separately.
    timing=[]
    for r in by['/tracking/compute_timing']:
        try:
            e=json.loads(r['msg']['data'])
            if t0<=e['call_ros_ns']*1e-9<=t1:timing.append(e)
        except (ValueError,KeyError):warnings.append('bad compute timing message')
    ct=np.array([e['call_ros_ns']*1e-9 for e in timing]);duration=np.array([e['compute_steady_ns']*1e-6 for e in timing])
    if production is None:
        production=yaml.safe_load(Path(cfg['production_nav']).read_text()) if Path(cfg['production_nav']).exists() else {}
    frequency=production.get('controller_server',{}).get('ros__parameters',{}).get('controller_frequency',20.)
    summary['compute_ms']=stats(duration);summary['control_period_s']=stats(np.diff(ct))
    summary['compute_over_period_count']=int(sum(duration>1000/frequency));summary['compute_exception_count']=sum(not e['ok'] for e in timing)
    summary['controller_timeout_log_evidence']=[dict(time_s=r['receive_s']-t0,message=r['msg'].get('msg',''))
        for r in by['/rosout'] if t0<=r['receive_s']<=t1 and 'controller_server' in r['msg'].get('name','')
        and any(word in r['msg'].get('msg','').lower() for word in ('missed','timeout','timed out','failed'))]
    axes_plot('timing.png',[('MPPI compute [ms]',[('steady clock, delegate only',ct,duration)]),('call interval [s]',[('ROS invocation clock',ct[1:],np.diff(ct))])],None)
    joints=[];positions=[]
    for r in by[topics['joints']]:
        m=r['msg'];indices={n:i for i,n in enumerate(m['name'])};names8=[f'wheel_{i}_{kind}' for kind in ('turn','roll') for i in range(1,5)]
        if all(n in indices for n in names8) and len(m['velocity'])==len(m['name']):
            joints.append([header_s(m),*[m['velocity'][indices[n]] for n in names8]])
        if all(n in indices for n in names8) and len(m['position'])==len(m['name']):
            positions.append([header_s(m),*[m['position'][indices[n]] for n in names8]])
    cmd=[[r['receive_s'],*r['msg']['data'][:8]] for r in by[topics['wheel_cmd']] if len(r['msg']['data'])>=8]
    # Match only bounded time brackets, never forward-fill stale wheel feedback.
    matched=match_columns(joints,[r[0] for r in cmd],8,gap)
    numeric=np.asarray([r[1:] for r in cmd],float).reshape(-1,8)-matched
    verified=cfg['wheel_feedback']['velocity_units_verified']
    summary['wheel']['velocity_units_verified']=verified
    summary['wheel']['actual_target_steering_angle_available']=False
    summary['wheel']['numeric_interface_rate_error']={f'{kind}_{i+1}':stats(numeric[:,i+offset]) for kind,offset in [('turn',0),('roll',4)] for i in range(4)}
    summary['wheel']['physical_rate_error']=summary['wheel']['numeric_interface_rate_error'] if verified else None
    summary['wheel']['target_angle_error']=None
    wheelplots=[]
    for i in range(8):
        wheelplots.append((f'{"turn" if i<4 else "roll"} {i%4+1} [rad/s]',
            [(title,np.array(rr)[:,0],np.array(rr)[:,i+1]) for title,rr in [('real interface target',cmd),('reported feedback (units unverified)' if not verified else 'feedback',joints)] if rr]))
    axes_plot('wheel_rates.png',wheelplots,None)
    angles=[]
    angle_diag=[[r['receive_s'],*r['msg']['data'][:8]] for r in by[topics['wheel_angle']] if len(r['msg']['data'])>=8]
    for i in range(4):
        series=[('actual encoder angle',np.array(positions)[:,0],np.array(positions)[:,i+1])] if positions else []
        if angle_diag:
            a=np.array(angle_diag);series.extend([('raw actual diagnostic',a[:,0],a[:,i+1]),('filtered actual diagnostic',a[:,0],a[:,i+5])])
        angles.append((f'turn {i+1} angle [rad]',series))
    axes_plot('wheel_angles.png',angles,None)
    # Saturation is unknown unless an actual physical threshold was supplied.
    for kind,offset in [('turn',0),('roll',4)]:
        limit=cfg['wheel_feedback'].get(kind+'_rate_limit_rad_s')
        summary['limits'][kind+'_at_limit_fraction']=float(np.mean(abs(np.asarray(cmd)[:,offset+1:offset+5])>=.99*limit)) if limit and cmd else None
    rawcmd=np.array(commands['controller_cmd']).reshape(-1,4)
    smoother=production.get('velocity_smoother',{}).get('ros__parameters',{})
    wzlimit=smoother.get('max_velocity',[None,None,None])[2]
    summary['limits']['raw_command_over_smoother_wz_limit_fraction']=float(np.mean(abs(rawcmd[:,3])>wzlimit)) if len(rawcmd) and wzlimit else None
    target=cfg['speed_requirement']['constant_target_mps']
    summary['speed_requirement']=cfg['speed_requirement']
    summary['speed_target_error_mps']=stats(speed[:,0]-target) if target is not None else None
    summary['turn_response']['definition']='First entry/exit by frozen arc-length boundaries; onset abs(wz)>=0.05 rad/s, never internal-controller activation evidence'
    entry=summary['phases'].get('entry',{}).get('first_s');exit_s=summary['phases'].get('tail',{}).get('first_s')
    if entry is not None:
        mask=(grid>=t0+entry)&np.isfinite(speed[:,2])&(abs(speed[:,2])>=.05)
        summary['turn_response']['entry_to_wz_threshold_s']=float(grid[mask][0]-t0-entry) if mask.any() else None
        for command_key in ('controller_cmd','smoothed_cmd','final_cmd'):
            rr=[r for r in commands[command_key] if r[0]>=t0+entry and abs(r[3])>=.05]
            summary['turn_response'][command_key+'_onset_after_entry_s']=rr[0][0]-t0-entry if rr else None
    if exit_s is not None:
        mask=(grid>=t0+exit_s)&valid
        summary['turn_response']['after_exit_heading_rad']=stats(errors[mask,2])
        summary['turn_response']['after_exit_lateral_m']=stats(errors[mask,1])
    if not timing:warnings.append('No measured MPPI compute telemetry; command intervals cannot replace compute duration')
    if not verified:warnings.append('Wheel 0x606C velocity units unverified; numeric rate errors are not established physical execution errors')
    warnings.extend(['No real target steering angle is published by current velocity-mode chassis',
        'Twist/Float32MultiArray have no source header: bag receive times cannot isolate CAN/network latency',
        'JointState headers are broadcaster times; per-wheel motor sample/arrival times are unavailable',
        'Normalizer validity checks finite pose/quaternion, frame, timestamp and staleness; camera-native tracking confidence/occlusion status is unavailable',
        'Mocap receiver clock is not exposure time; camera latency remains unverified' if not cfg['timing']['camera_latency_verified'] else 'Camera latency config recorded; no trajectory-based correction applied'])
    for key in ('raw_mocap','mocap','validity','state','base_pose','controller_cmd','final_cmd','wheel_cmd','joints'):
        if not by[topics[key]]:warnings.append('missing '+key+': '+topics[key])
    audit=[]
    for topic,rr in sorted(by.items()):
        if not rr:continue
        audit.append(dict(topic=topic,analysis_samples=len(rr),first_receive_s=rr[0]['receive_s'],last_receive_s=rr[-1]['receive_s'],
            frames=sorted({r['msg'].get('header',{}).get('frame_id','') for r in rr})))
    write_json(out/'topic_audit.json',audit);write_json(out/'summary.json',summary)
    evidence=[]
    if valid.any():
        i=int(np.nanargmax(abs(errors[:,1])));evidence.append(f"最大横向误差发生在 {grid[i]-t0:.3f}s，阶段 {phases[i]}，值 {errors[i,1]:.4f}m；见 errors.png。")
    else:evidence.append('本轮没有可用动捕真值；轨迹精度、阶段响应和真实速度结果均缺失。')
    if len(duration):
        i=int(np.argmax(duration));evidence.append(f"最长 MPPI 调用 {duration[i]:.3f}ms，发生在 {ct[i]-t0:.3f}s；见 timing.png。")
    report=['# 控制器实验报告',f'数据来源：{source}；目标发送：{started is not None}；结果：{execution.get("reason","未执行/未记录结果")}。',
        f'动捕参考数据覆盖率：{summary["reference_data_coverage"]:.2%}。全程统计与预定义阶段均见 summary.json；缺失值保留 null。',
        *evidence,'比较用状态来源相同。输入位姿差是反馈链检查，不能当作独立定位误差。',
        '图表：trajectory.png、errors.png、speed.png、commands.png、wheel_rates.png、wheel_angles.png、timing.png。',
        '仅凭这些统计不认定单一根因；结合误差、命令与轮组曲线的同一时间段判断。',
        '限制：',*['- '+w for w in warnings]]
    (out/'report.md').write_text('\n\n'.join(report)+'\n')
    return summary


def analyze(run,output=None):
    run=Path(run);cfg=yaml.safe_load((run/'session/config.yaml').read_text())
    # Always use frozen formal parameters, even if production config changes later.
    frozen=yaml.safe_load((run/'session/nav2.yaml').read_text())['tracking']
    path=json.loads((run/'path.json').read_text());execution=json.loads((run/'execution.json').read_text()) if (run/'execution.json').exists() else {}
    if digest(path['points'])!=path['points_sha256']:raise ValueError('Path integrity check failed')
    selected=set(cfg['topics'].values())|{'/tracking/input_pose','/tracking/input_speed','/tracking/compute_timing','/tracking/events','/tracking/gate_status','/tracking/received_plan','/rosout'}
    # Analysis uses the immutable parameter snapshot via a temporary in-memory override.
    source=json.loads((run/'recording.json').read_text()).get('source','real_bag') if (run/'recording.json').exists() else 'real_bag'
    result=analyze_records(read_bag(run/'bag',selected),path,cfg,execution,output or run/'analysis',source=source,production=frozen)
    identity=json.loads((run/'session/identity.json').read_text())
    result['frozen_identity']=identity
    write_json(Path(output or run/'analysis')/'summary.json',result)
    return result


def compare(runs,output):
    rows=[]
    for run in runs:
        run=Path(run)
        summary=json.loads((run/'analysis/summary.json').read_text())
        rows.append(dict(run=str(run),success=summary['execution'].get('success',False),
            reason=summary['execution'].get('reason','missing_execution_result'),
            source=summary['source'],coverage=summary['reference_data_coverage'],
            lateral_m=summary['lateral_m'],heading_rad=summary['heading_rad'],speed_mps=summary['speed_mps'],
            path_sha256=summary['path_sha256'],config_sha256=summary['config_sha256'],frame_sha256=summary.get('frame_sha256'),frozen_identity=summary.get('frozen_identity')))
    identities={(r['path_sha256'],r['config_sha256'],r['frame_sha256'],digest(r['frozen_identity'])) for r in rows}
    out=Path(output);out.mkdir(parents=True,exist_ok=False)
    result=dict(runs=rows,total=len(rows),success_count=sum(r['success'] for r in rows),
        success_rate=sum(r['success'] for r in rows)/len(rows) if rows else None,
        comparable_same_feedback_route_configuration=len(identities)==1,
        warning='All supplied attempts retained, including failures; do not omit unsuccessful round folders')
    write_json(out/'comparison.json',result)
    (out/'report.md').write_text(f'# 重复测试汇总\n\n成功 {result["success_count"]}/{len(rows)}；失败实验保留在分母。\n\n同一状态/路线/配置：{result["comparable_same_feedback_route_configuration"]}。详细统计见 comparison.json。\n')
    return result


def batch_runs(batch,kind=None):
    """Discover every recorded attempt of a batch, including failed preflights."""
    batch=Path(batch).resolve();found=[]
    for meta in sorted(batch.parent.glob('*/recording.json')):
        identity=json.loads(meta.read_text())
        if Path(identity['session']).resolve()!=batch:continue
        if kind and identity['path_kind']!=kind:continue
        if not (meta.parent/'recording_result.json').exists():
            raise ValueError('Stop recording before comparison: '+str(meta.parent))
        if not (meta.parent/'analysis/summary.json').exists():
            raise ValueError('Analyze every failed/successful attempt first: '+str(meta.parent))
        found.append(meta.parent)
    if not found:raise ValueError('No recorded attempts found for this batch/kind')
    return found
