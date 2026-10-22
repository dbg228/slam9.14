#!/usr/bin/env python3
"""Read archived messages offline; never initialize ROS or publish motion."""
import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def load(path):
    return json.loads(path.read_text())


def mean_curve(curves,bin_s=.5,max_gap_s=.20):
    """Equal trial weights in fixed time bins; no interpolation or missing zeros.

    Each trial needs >=3 matched samples and no >max_gap_s gap including bin
    edges. The primary mean is absent unless every selected trial contributes.
    """
    last=max((float(s['time_s']) for c in curves for s in c),default=0.)
    output=[]
    for left in np.arange(0,last,bin_s):
        xy=[];heading=[]
        for samples in curves:
            valid=sorted((s for s in samples if left<=float(s['time_s'])<left+bin_s
                and s['reference_valid'] and s['error_xy_m']!='' and s['yaw_error_deg']!=''),key=lambda s:float(s['time_s']))
            times=[left,*[float(s['time_s']) for s in valid],left+bin_s]
            if len(valid)<3 or max(np.diff(times))>max_gap_s+1e-9:continue
            xy.append(float(np.mean([float(s['error_xy_m'])*1000 for s in valid])))
            heading.append(float(np.mean([abs(float(s['yaw_error_deg'])) for s in valid])))
        full=len(xy)==len(curves) and bool(curves)
        output.append(dict(time_s=float(left+bin_s/2),support=len(xy),expected=len(curves),
            xy_mean_mm=float(np.mean(xy)) if full else None,
            xy_between_trial_std_mm=float(np.std(xy,ddof=1)) if full and len(xy)>1 else None,
            yaw_abs_mean_deg=float(np.mean(heading)) if full else None,
            yaw_abs_between_trial_std_deg=float(np.std(heading,ddof=1)) if full and len(xy)>1 else None))
    return output


def main():
    from auto_2m_trial import replay
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,required=True,help='Existing dynamic_trials directory')
    args=parser.parse_args();root=args.root.resolve()
    status=load(root/'forward_summary.json');rows=[];curves=[];starts=[]
    for number,name in enumerate(status['completed_forward_trials'],1):
        trial=Path(name).resolve()
        if trial.parent!=root:raise ValueError('Completed trials must be immediate children of root')
        meta=load(trial/'manifest.json');result=load(trial/'results.json')
        replayed,samples=replay(trial,include_samples=True)
        if abs(replayed['XY_error_m']['rmse']-result['XY_error_m']['rmse'])>1e-10:
            raise ValueError('Archived statistic and raw replay disagree: '+name)
        if not (meta['direction']=='forward' and meta['stage']=='completed'
                and result['state']['state']=='DONE' and result['state']['reason']=='target_reached'):
            raise ValueError('An incomplete trial was listed as completed: '+name)
        event=next(e for e in result['events'] if e['event']=='motion_start')
        start=event['start_base_pose'];starts.append(start)
        first=starts[0];yaw=lambda p:2*np.arctan2(p['qz'],p['qw'])
        dyaw=np.arctan2(np.sin(yaw(start)-yaw(first)),np.cos(yaw(start)-yaw(first)))
        row=dict(round=number,trial=str(trial),stage=meta['stage'],localization_session=meta['localization_session'],
            xy_mean_mm=result['XY_error_m']['mean']*1000,
            xy_rmse_mm=result['XY_error_m']['rmse']*1000,xy_p95_mm=result['XY_error_m']['p95_abs']*1000,
            xy_max_mm=result['XY_error_m']['max_abs']*1000,yaw_rmse_deg=result['yaw_error_deg']['rmse'],
            reference_pair_coverage=result['reference_coverage'],motion_duration_s=result['motion_duration_s'],
            stop_trigger_m=result['state']['displacement_m'],stopped_displacement_m=result['stopped_displacement_m'],
            request_mps=meta['command_mps'],mean_displacement_speed_mps=result['state']['displacement_m']/result['motion_duration_s'],
            start_xy_distance_from_round1_m=float(np.hypot(start['x']-first['x'],start['y']-first['y'])),
            start_yaw_difference_from_round1_deg=float(np.rad2deg(dyaw)))
        rows.append(row)
        curves.append([s for s in samples if s['phase'] in ('motion','post_stop')])

    attempts=[]
    for path in sorted(root.rglob('manifest.json')):
        meta=load(path)
        if meta.get('data_kind')!='real_2m_dynamic_localization_test' or meta.get('direction')!='forward':continue
        result=load(path.with_name('results.json')) if path.with_name('results.json').exists() else {}
        attempts.append(dict(trial=str(path.parent),stage=meta.get('stage'),
            reason=(result.get('state') or {}).get('reason',meta.get('error','unknown')),
            motion_started=result.get('motion_started'),displacement_m=(result.get('state') or {}).get('displacement_m')))
    success=sum(a['stage']=='completed' for a in attempts)
    if status.get('export_csv',False):
        with (root/'forward_summary.csv').open('w') as stream:
            writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    aggregate={}
    for key in ('xy_mean_mm','xy_rmse_mm','xy_p95_mm','xy_max_mm','yaw_rmse_deg','stopped_displacement_m'):
        values=[r[key] for r in rows]
        aggregate[key]=dict(mean=float(np.mean(values)),sample_std=float(np.std(values,ddof=1)) if len(values)>1 else None)
    status.update(completed_forward_count=len(rows),stage='completed' if len(rows)>=status['requested_forward_count'] else 'incomplete',
        forward_attempts=attempts,forward_success_rate=success/len(attempts) if attempts else None,
        aggregation='Equal weight per completed trial; sample standard deviation across trials. Different initialization/start poses documented, not an identical-route repeatability claim.',
        across_completed_trials=aggregate,per_trial=rows,
        average_curve=dict(method='0.5s time bins from motion_start; equal trial weights; >=3 valid pairs per trial/bin; max gap incl. edges <=0.2s; no interpolation; full support required',samples=mean_curve(curves)))
    (root/'forward_summary.json').write_text(json.dumps(status,ensure_ascii=False,indent=2)+'\n')

    fig,axes=plt.subplots(3,1,figsize=(9,9),constrained_layout=True)
    for row,samples in zip(rows,curves):
        t=np.array([float(s['time_s']) for s in samples])
        for ax,key,factor in [(axes[0],'error_xy_m',1000),(axes[1],'yaw_error_deg',1)]:
            y=np.array([float(s[key])*factor if s[key]!='' else np.nan for s in samples])
            # Insert a gap even if dropped source frames leave no CSV row.
            tx=[];yy=[]
            for i,(x,v) in enumerate(zip(t,y)):
                if i and x-t[i-1]>.2:tx.append(np.nan);yy.append(np.nan)
                tx.append(x);yy.append(v)
            ax.plot(tx,yy,linewidth=1.2,label=f"Trial {row['round']}")
    axes[0].set(ylabel='XY error (mm)',title='2 m forward localization trials')
    axes[1].set(ylabel='Yaw error (deg)',xlabel='Time from motion start (s)')
    x=np.arange(len(rows));axes[2].bar(x-.2,[r['xy_rmse_mm'] for r in rows],.4,label='RMSE')
    axes[2].bar(x+.2,[r['xy_p95_mm'] for r in rows],.4,label='P95')
    axes[2].set(xticks=x,xticklabels=[f"Trial {r['round']}" for r in rows],ylabel='XY error (mm)')
    for ax in axes:ax.grid(axis='y',alpha=.25);ax.legend(loc='best')
    fig.savefig(root/'forward_comparison.png',dpi=180);plt.close(fig)

    averaged=status['average_curve']['samples'];times=[s['time_s'] for s in averaged]
    fig,axes=plt.subplots(3,1,figsize=(9,8),constrained_layout=True,gridspec_kw={'height_ratios':[3,3,1]})
    for ax,mean_key,std_key,label in [(axes[0],'xy_mean_mm','xy_between_trial_std_mm','Mean XY error (mm)'),
            (axes[1],'yaw_abs_mean_deg','yaw_abs_between_trial_std_deg','Mean absolute yaw error (deg)')]:
        means=np.array([s[mean_key] if s[mean_key] is not None else np.nan for s in averaged])
        deviations=np.array([s[std_key] if s[std_key] is not None else np.nan for s in averaged])
        ax.plot(times,means,color='tab:blue',label=f'Mean of {len(rows)} trials')
        ax.fill_between(times,np.maximum(0,means-deviations),means+deviations,color='tab:blue',alpha=.2,label='Between-trial SD')
        ax.set(ylabel=label);ax.legend();ax.grid(alpha=.25)
    axes[0].set_title(f'{len(rows)} trials: equal-weight mean error (0.5 s bins)')
    axes[2].step(times,[s['support'] for s in averaged],where='mid',color='tab:gray')
    axes[2].set(xlabel='Time from motion start (s)',ylabel='Trials',ylim=(-.1,len(rows)+.5),yticks=list(range(len(rows)+1)))
    axes[2].grid(alpha=.25);fig.savefig(root/'forward_mean_error.png',dpi=180);plt.close(fig)

    lines=['# 前进 2 米定位实验汇总','',f"已完成 {len(rows)}/{status['requested_forward_count']} 组；自动运动进程已退出，后续复位由人工完成。",'',
        '|组|平均 XY 误差 (mm)|XY RMSE (mm)|P95 (mm)|最大 (mm)|航向 RMSE (°)|参考配对覆盖率|停车位移 (m)|',
        '|---|---:|---:|---:|---:|---:|---:|---:|']
    for r in rows:
        lines.append(f"|{r['round']}|{r['xy_mean_mm']:.2f}|{r['xy_rmse_mm']:.2f}|{r['xy_p95_mm']:.2f}|{r['xy_max_mm']:.2f}|{r['yaw_rmse_deg']:.3f}|{r['reference_pair_coverage']:.2%}|{r['stopped_displacement_m']:.4f}|")
    lines+=['',f"各轮 XY RMSE 的等权均值 ± 样本标准差：{aggregate['xy_rmse_mm']['mean']:.2f} ± {aggregate['xy_rmse_mm']['sample_std']:.2f} mm；仅为这批记录的描述统计，不将每帧作为独立实验。",'',
        f"各轮平均 XY 距离误差的等权平均：{aggregate['xy_mean_mm']['mean']:.2f} mm。平均距离误差与 RMSE 分开报告，不将 X/Y 正负误差抵消后当作距离误差。",'',
        '![逐轮误差对比](forward_comparison.png)','', '![平均误差](forward_mean_error.png)','',
        '平均曲线：按运动开始计时，以0.5秒固定分箱计算每轮箱内平均，再按轮等权平均。每轮每箱至少3个有效配对且含箱边界的最大间隔≤0.2秒；只有全部完成轮均有有效数据时才画均值，缺失不插值、不填零。阴影为轮间样本标准差，不是置信区间；下方记录实际支持轮数。全程统计仍使用完整预定义窗口，不因分箱筛选而改变。','',
        '统计从运动开始至发零后5秒，前置静止和退出清理阶段另存。最近时间配对≤25ms，无插值、填零或事后轨迹拟合；参考配对覆盖率是有效配对的原生定位帧比例，并非光学曝光覆盖率。',
        '', '两边均已换算至 base_link，使用同一冻结 map↔mocap 标定；不使用 NDT。这是动捕初始锚定后 LIO 的定位误差，不是对地图原点或刚体外参精度的独立验证。动捕曝光延迟与传感器时差未经独立核实，因此动态精度是本次配置下的初步测量。',
        '', '第2轮前静止重启局部 LIO，并关闭累计整图显示发布；第2轮起沿用同一初始化，后续以各轮 manifest 为准。第1轮初始化不同。人工复位也有位置、朝向差异，下表直接报告，不能宣称完全相同路线的严格重复性或据此认定算法改善。',
        '', '|组|距第1轮起点 (cm)|相对第1轮航向 (°)|位移/运动时间 (m/s)|请求速度 (m/s)|', '|---|---:|---:|---:|---:|']
    for r in rows:lines.append(f"|{r['round']}|{100*r['start_xy_distance_from_round1_m']:.2f}|{r['start_yaw_difference_from_round1_deg']:.2f}|{r['mean_displacement_speed_mps']:.3f}|{r['request_mps']:.2f}|")
    lines+=['', '位移/运动时间为平均前进速度，不是恒速保证；停车距离包含惯性过冲。', '',
        f"共保存 {len(attempts)} 次前进尝试，其中 {success} 次完成，完成率 {status['forward_success_rate']:.1%}；包括静止阶段启动失败，不含自动返回段。失败未删除、不计入完成轮的误差均值。", '', '|尝试目录|结果|原因|停止触发位移 (m)|','|---|---|---|---:|']
    for a in attempts:lines.append(f"|[{Path(a['trial']).name}]({Path(a['trial']).name}/report.md)|{a['stage']}|{a['reason']}|{a['displacement_m'] if a['displacement_m'] is not None else '未知'}|")
    lines+=['', '此前3次自动返回失败记录同样保留，但不作为前进试验。原监督程序退出检查标记false均保持原样；第2轮独立复核481条零命令，第4轮872条，第5轮605条，均无自动速度发布者。见各轮 post_stop_verification.json；正式结束时guard状态及进程退出码见manifest.json。', '',
        '每轮目录包含 bag/、raw.jsonl、errors_xy.png、results.json、report.md、manifest.json 和冻结输入。CSV已按用户要求删除，新轮禁用CSV导出；曲线从原始消息在内存中重算，汇总数值保存于forward_summary.json。', '']
    for r in rows:lines.append(f"- 第{r['round']}组：[报告]({Path(r['trial']).name}/report.md)、[误差图]({Path(r['trial']).name}/errors_xy.png)")
    (root/'FORWARD_STATUS.md').write_text('\n'.join(lines)+'\n')
    print(json.dumps(dict(completed=len(rows),attempts=len(attempts),aggregate=aggregate,report=str(root/'FORWARD_STATUS.md')),ensure_ascii=False,indent=2))


if __name__=='__main__':main()
