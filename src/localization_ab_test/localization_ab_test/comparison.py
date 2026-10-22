"""Offline A/B and optional C report comparison. Never drives the robot."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np


def load_trial(path,group,index):
    if group not in ('A','B','C') or index not in (1,2,3):
        raise ValueError('Expected group A/B/C and trial 1..3')
    manifest=json.loads((path/'manifest.json').read_text())
    s=json.loads((path/'report/summary.json').read_text())
    if manifest['group']!=group or manifest['status']!='completed':
        raise ValueError(f'Not a completed {group} trial: {path}')
    direction=manifest.get('direction','forward')
    expected=('forward','reverse','forward')[index-1]
    if direction not in ('forward','reverse'):raise ValueError('Unknown direction')
    if group in ('A','B') and direction!=expected:
        raise ValueError('Direction ordering differs between A/B')
    if group=='C' and s.get('reference_independent') is not False:
        raise ValueError('C report must explicitly identify its non-independent reference')
    x=s['all_motion_and_5s_stop']['xy_m']
    row=dict(group=group,trial=index,direction=direction,xy_rmse_mm=x['rmse']*1000,
             p95_mm=x['p95_absolute']*1000,max_mm=x['max_absolute']*1000,
             yaw_rmse_deg=s['all_motion_and_5s_stop']['yaw_deg']['rmse'],
             endpoint_mm=s['endpoint']['xy_error_m']*1000,
             max_pose_jump_mm=s['max_adjacent_pose_translation_m']*1000,
             max_error_jump_mm=s['max_adjacent_error_jump_m']*1000,
             reference_coverage=s['coverage'],timing_verified=s['timing_verified'],
             reference_independent=group!='C',
             metric_label='same-mocap consistency' if group=='C' else 'start-aligned relative localization error',
             directory=str(path.resolve()))
    with (path/'report/frame_errors.csv').open() as f:frames=list(csv.DictReader(f))
    return row,frames


def summarize_rows(rows):
    groups={}
    for g in dict.fromkeys(r['group'] for r in rows):
        selected=[r for r in rows if r['group']==g];v=[r['xy_rmse_mm'] for r in selected]
        groups[g]=dict(trials=len(selected),directions=[r['direction'] for r in selected],
                       reference_independent=g!='C',metric_label=selected[0]['metric_label'],
                       mean_per_trial_xy_rmse_mm=float(np.mean(v)),
                       sample_std_per_trial_xy_rmse_mm=float(np.std(v,ddof=1)),
                       max_observed_error_mm=max(r['max_mm'] for r in selected))
    return groups


def plot_metrics(rows,output,stem):
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    available={f.name for f in font_manager.fontManager.ttflist}
    for family in ('Noto Sans CJK SC','Noto Sans CJK JP','Noto Sans CJK TC'):
        if family in available:
            plt.rcParams['font.family']=family
            break
    plt.rcParams['axes.unicode_minus']=False
    fig,axs=plt.subplots(2,3,figsize=(15,8.5),constrained_layout=True)
    colors={'A':'#3274a1','B':'#d65f39','C':'#36966b'}
    labels=[f"{r['group']}{r['trial']}" for r in rows]
    metrics=[('xy_rmse_mm','平面位置 RMSE','mm'),('p95_mm','平面位置 P95','mm'),
             ('max_mm','平面位置最大差值','mm'),('yaw_rmse_deg','航向 RMSE','°'),
             ('endpoint_mm','停稳终点位姿差','mm'),('max_error_jump_mm','最大相邻误差跳变','mm')]
    for ax,(key,title,unit) in zip(axs.flat,metrics):
        values=[r[key] for r in rows]
        bars=ax.bar(labels,values,color=[colors[r['group']] for r in rows],width=.72,zorder=3)
        ax.bar_label(bars,labels=[f'{v:.3f}' if abs(v)<1 else f'{v:.2f}' for v in values],
                     padding=3,fontsize=9)
        ax.set(title=title,ylabel=unit);ax.margins(y=.2)
        ax.grid(axis='y',alpha=.2,zorder=0)
        ax.spines[['top','right']].set_visible(False)
    title='A：FAST-LIO    B：FAST-LIO + NDT'
    if any(r['group']=='C' for r in rows):title+='    C：FAST-LIO + 动捕校正'
    fig.suptitle(title,fontsize=16)
    for ext in ('png','svg','pdf'):fig.savefig(output/f'{stem}_metrics.{ext}',dpi=200)
    plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--a',nargs=3,type=Path,required=True)
    parser.add_argument('--b',nargs=3,type=Path,required=True)
    parser.add_argument('--c',nargs=3,type=Path,help='Optional three C trials; actual directions retained in tables')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():parser.error('Existing comparison will not be overwritten')
    inputs=[('A',args.a),('B',args.b)]+([('C',args.c)] if args.c else [])
    loaded=[load_trial(p,g,i) for g,paths in inputs for i,p in enumerate(paths,1)]
    args.output.mkdir(parents=True)
    rows=[r for r,_ in loaded]
    with (args.output/'comparison.csv').open('w',newline='',encoding='utf8') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    groups=summarize_rows(rows)
    (args.output/'comparison.json').write_text(json.dumps(dict(trials=rows,groups=groups,
        definition='Mean/sample SD across three per-trial RMSEs, not pooled frame statistics. Mixed directions; timing unverified.'),ensure_ascii=False,indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axs=plt.subplots(2,2,figsize=(12,8),constrained_layout=True)
    colors=['#1f77b4','#4895c7','#83b4d8','#c0392b','#df6b40','#ec9c5b',
            '#187b4e','#36966b','#70b88e']
    labels=[]
    for j,(r,frames) in enumerate(loaded):
        label=f"{r['group']}{r['trial']}";labels.append(label)
        valid=[x for x in frames if x['valid']=='True' and x['phase']=='moving']
        d=[float(x['true_displacement_m']) for x in valid]
        axs[0,0].plot(d,[float(x['e_xy_m'])*1000 for x in valid],label=label,color=colors[j],linewidth=1.)
        axs[0,1].plot(d,[float(x['yaw_error_deg']) for x in valid],label=label,color=colors[j],linewidth=1.)
    axs[0,0].set(xlabel='Mocap displacement (m)',ylabel='XY deviation (mm)',title='XY deviation over distance')
    axs[0,1].set(xlabel='Mocap displacement (m)',ylabel='Yaw difference (deg)',title='Yaw difference over distance')
    for ax,key,title in [(axs[1,0],'xy_rmse_mm','Motion + 5 s stop: XY RMSE'),(axs[1,1],'endpoint_mm','Stationary endpoint error')]:
        bars=ax.bar(labels,[r[key] for r in rows],color=colors)
        ax.bar_label(bars,fmt='%.2f',padding=3)
        ax.set(ylabel='mm',title=title);ax.margins(y=.18)
    for ax in axs[0]:ax.legend(fontsize=8,ncol=3)
    for ax in axs.flat:ax.grid(alpha=.2)
    fig.suptitle('A: FAST-LIO | B: FAST-LIO + NDT'+(' | C: FAST-LIO + Mocap' if args.c else ''))
    stem='abc' if args.c else 'ab'
    for ext in ('png','svg','pdf'):fig.savefig(args.output/f'{stem}_comparison.{ext}',dpi=200)
    plt.close(fig)
    plot_metrics(rows,args.output,stem)
    lines=[f'# {"A/B/C" if args.c else "A/B"} 各三次两米试验对比','',
           'A=FAST-LIO，B=FAST-LIO+固定地图 NDT；A/B 动捕仅用于距离触发与离线参考。',
           '47.15° 安装角固定，单次只用起始静止窗口做一次 SE(2) 对齐，不拟合尺度或测试轨迹。','',
           '| 试次 | 方向 | XY RMSE mm | P95 mm | Max mm | yaw RMSE ° | 终点差 mm |',
           '|---|---|---:|---:|---:|---:|---:|']
    for r in rows:
        lines.append(f"| {r['group']}{r['trial']} | {r['direction']} | {r['xy_rmse_mm']:.3f} | {r['p95_mm']:.3f} | {r['max_mm']:.3f} | {r['yaw_rmse_deg']:.4f} | {r['endpoint_mm']:.3f} |")
    for g,s in groups.items():
        lines.extend(['',f"{g}：三次 RMSE 均值 {s['mean_per_trial_xy_rmse_mm']:.3f} mm，样本标准差 {s['sample_std_per_trial_xy_rmse_mm']:.3f} mm。"])
    lines.extend(['','当前短路线样本中，B 的相对误差大于 A，不能推广为所有环境下 NDT 都更差。',
                  '各组均有正反向，只有三个试次；不能将所有帧混合冒充独立重复或宣称统计显著性。',
                  '动捕使用 Ubuntu 接收时间，时间偏移尚未独立核实；动态结果不是已认证绝对精度。',
                  'A/B 之间底盘和 FAST-LIO 曾重启，实际起点、朝向也不完全相同；不是严格同一轨迹的配对试验。',
                  'B 第二次曾在发车前被阻止，未发生运动；现场重新确认后在新目录继续，原阻止记录保留。',
                  '原始 bag 和单次逐帧数据保留在 comparison.csv 的 directory 所指目录。'])
    if args.c:
        lines.extend(['','C=FAST-LIO+动捕全局校正，参考参与校正，数值为同源动捕一致性，不是独立绝对定位精度。',
                      'A/B 顺序为前/倒/前；C 为倒/前/倒，不能作为三个配对同向重复或据此给出绝对精度提升倍数。',
                      'C 批次前因旧 FAST-LIO 滞后，按原参数重启；批内三趟未重启。',
                      '终点位姿差不是相对 2.000 m 的停车过冲；最大相邻误差跳变已扣除真实运动。'])
    lines.extend(['',f'![统计比较]({stem}_metrics.png)','',f'![距离曲线]({stem}_comparison.png)'])
    (args.output/'report.md').write_text('\n'.join(lines)+'\n',encoding='utf8')
    print(json.dumps(groups,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
