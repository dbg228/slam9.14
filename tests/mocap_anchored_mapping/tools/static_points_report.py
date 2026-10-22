#!/usr/bin/env python3
"""Report archived static captures using frozen transforms; never fits or runs ROS."""
import argparse
import csv
import json
import math
from pathlib import Path
import sys

PKG = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(PKG), str(PKG.parent / 'controller_tracking')]
import numpy as np
from controller_tracking.common import stats, wrap
from mocap_anchored_mapping.core import load, sha, validation_errors


def write_csv(path, rows):
    with path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def plot_xy_only(batch):
    """Additional planar figure from archived independent samples; original outputs stay intact."""
    batch=Path(batch).resolve()
    with (batch/'samples.csv').open(newline='') as stream:
        rows=[r for r in csv.DictReader(stream) if r['role']=='independent_validation']
    if not rows:raise ValueError('No independent validation samples')
    for r in rows:
        expected=math.hypot(float(r['error_x_m']),float(r['error_y_m']))
        if not math.isclose(expected,float(r['error_xy_m']),rel_tol=1e-10,abs_tol=1e-12):
            raise ValueError('Archived XY error inconsistent with signed X/Y components')
    labels=list(dict.fromkeys(r['label'] for r in rows))
    groups=[[r for r in rows if r['label']==label] for label in labels]
    metrics=[stats([float(r['error_xy_m']) for r in group]) for group in groups]
    overall=stats([float(r['error_xy_m']) for r in rows])
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    x=np.arange(len(labels));fig,axes=plt.subplots(2,1,figsize=(9,7),sharex=True)
    for offset,axis,color in [(-.18,'x','tab:blue'),(.18,'y','tab:orange')]:
        values=[np.mean([float(r['error_'+axis+'_m']) for r in group])*1000 for group in groups]
        bars=axes[0].bar(x+offset,values,width=.34,label='Mean '+axis.upper(),color=color)
        axes[0].bar_label(bars,fmt='%.2f',padding=3,fontsize=9)
    axes[0].axhline(0,color='black',linewidth=.8)
    axes[0].set(ylabel='Signed error (mm)',title='Planar localization error: LIO minus mocap (base_link)')
    axes[0].margins(y=.25);axes[0].legend(ncol=2);axes[0].grid(axis='y',alpha=.3)
    for offset,key,label,color in [(-.25,'rmse','XY RMSE','tab:blue'),(0.,'p95_abs','XY P95','tab:green'),(.25,'max_abs','XY maximum','tab:orange')]:
        bars=axes[1].bar(x+offset,[s[key]*1000 for s in metrics],width=.23,label=label,color=color)
        axes[1].bar_label(bars,fmt='%.2f',padding=3,fontsize=9)
    axes[1].set(ylabel='Planar distance error (mm)',xticks=x,
                xticklabels=[label+'\n(n='+str(len(group))+')' for label,group in zip(labels,groups)])
    axes[1].margins(y=.3);axes[1].legend(ncol=3,loc='upper left');axes[1].grid(axis='y',alpha=.3)
    fig.text(.5,.045,f"All {len(rows)} pairs: XY RMSE {overall['rmse']*1000:.2f} mm | P95 {overall['p95_abs']*1000:.2f} mm | Max {overall['max_abs']*1000:.2f} mm",ha='center',fontsize=11)
    fig.text(.5,.015,'Fixed transforms; initialization/repeat samples excluded; Z omitted.',ha='center',fontsize=9,color='dimgray')
    fig.tight_layout(rect=(0,.085,1,1));target=batch/'errors_xy.png';fig.savefig(target,dpi=180)
    fig.savefig(target.with_suffix('.pdf'));fig.savefig(target.with_suffix('.svg'));plt.close(fig)
    return dict(figure=str(target),pdf=str(target.with_suffix('.pdf')),svg=str(target.with_suffix('.svg')),
                samples=len(rows),positions=len(labels),XY_error_m=overall,Z_included=False)



def report(batch):
    batch = Path(batch).resolve()
    manifest = load(batch / 'session.json')
    run = Path(manifest['run'])
    for name, expected in manifest['protected_inputs'].items():
        if sha(run / name) != expected['sha256']:
            raise ValueError('Protected input changed: ' + name)
    calibration = load(batch / 'frozen_inputs/calibration.yaml')
    seed = load(batch / 'frozen_inputs/initialization.yaml')
    if sha(batch / 'frozen_inputs/calibration.yaml') != sha(run / 'calibration.yaml'):
        raise ValueError('Archived and live calibration differ')
    if sha(batch / 'frozen_inputs/initialization.yaml') != sha(Path(manifest['localization_session']) / 'initialization.yaml'):
        raise ValueError('Restart initialization changed')
    map_odom = np.asarray(seed['T_map_odom']['matrix'])
    map_mocap = np.asarray(calibration['T_map_mocap']['matrix'])
    sample_rows, point_rows, eligible_errors = [], [], []
    for capture in manifest.get('captures', []):
        path = (batch / capture['path']).resolve()
        if not path.is_relative_to(batch):
            raise ValueError('Capture must be archived inside this batch')
        summary = load(path / 'summary.json')
        pairs = load(path / 'pairs.json')
        if summary['fit_performed'] or summary['transform_updated']:
            raise ValueError('Validation performed a fit')
        if summary['calibration_sha256'] != sha(run / 'calibration.yaml'):
            raise ValueError('Capture used another calibration')
        if summary['raw_sha256'] != sha(path / 'raw.jsonl') or summary['pairs_sha256'] != sha(path / 'pairs.json'):
            raise ValueError('Capture hash mismatch')
        replay = validation_errors(pairs, calibration, map_odom)
        np.testing.assert_allclose([r['XYZ_error_m'] for r in replay['samples']],
                                   [r['XYZ_error_m'] for r in summary['samples']], atol=1e-12, rtol=0)
        rows = []
        for pair, error in zip(pairs, replay['samples']):
            truth = map_mocap @ np.asarray(pair['T_M_B'])
            estimated = map_odom @ np.asarray(pair['T_F_B'])
            yaw_error = float(wrap(math.atan2(estimated[1, 0], estimated[0, 0]) - math.atan2(truth[1, 0], truth[0, 0])))
            row = dict(label=capture['label'], role=capture['role'], stamp_ns=pair['stamp_ns'],
                       mocap_stamp_ns=pair['mocap_stamp_ns'], skew_s=pair['skew_s'])
            for axis, i in zip('xyz', range(3)):
                row['mocap_map_' + axis + '_m'] = float(truth[i, 3])
                row['lio_map_' + axis + '_m'] = float(estimated[i, 3])
                row['error_' + axis + '_m'] = error['XYZ_error_m'][i]
            row.update(error_xy_m=error['distance_XY_m'], error_3d_m=error['distance_3D_m'],
                       yaw_error_deg=math.degrees(yaw_error), orientation_error_deg=error['orientation_error_deg'])
            rows.append(row)
        point = dict(label=capture['label'], role=capture['role'], pairs=len(rows),
                     duration_s=(pairs[-1]['stamp_ns']-pairs[0]['stamp_ns'])*1e-9,
                     raw_normalized_coverage=summary['raw_normalized_check']['coverage'])
        mocap_mean = np.mean([np.asarray(p['T_M_B'])[:3, 3] for p in pairs], axis=0)
        init_mean = np.asarray(seed['static_spread']['T_M_B']['mean_xyz_m'])
        point['distance_from_restart_initialization_m'] = float(np.linalg.norm(mocap_mean-init_mean))
        for i, a in enumerate('xyz'):
            point['mocap_world_mean_'+a+'_m'] = float(mocap_mean[i])
        for key in rows[0]:
            if key.startswith(('mocap_map_', 'lio_map_', 'error_')) and key.endswith('_m'):
                point['mean_' + key] = float(np.mean([r[key] for r in rows]))
        for key in ['error_xy_m', 'error_3d_m', 'yaw_error_deg', 'orientation_error_deg']:
            for metric, value in stats([r[key] for r in rows]).items():
                if metric != 'n':
                    point[key + '_' + metric] = value
        sample_rows.extend(rows)
        point_rows.append(point)
        if capture['role'] == 'independent_validation':
            eligible_errors.extend(rows)
    if sample_rows:
        write_csv(batch / 'samples.csv', sample_rows)
        write_csv(batch / 'point_summary.csv', point_rows)
    independent = [r for r in point_rows if r['role'] == 'independent_validation']
    result = dict(data_kind=manifest['data_kind'], planned_points=manifest['planned_validation_points'],
                  completed_independent_points=len(independent), complete=len(independent)==manifest['planned_validation_points'],
                  initialization_samples_excluded=True, fit_performed=False, transform_updated=False,
                  independent_world_calibration_validation=False,
                  scope='LIO pose after mocap restart initialization, compared to subsequent independent static mocap samples; not PCD global relocalization or independent world-transform calibration',
                  reference_point='base_link', failures=manifest.get('failed_attempts', []),
                  calibration_sha256=sha(run / 'calibration.yaml'),
                  initialization_sha256=sha(batch / 'frozen_inputs/initialization.yaml'),
                  capture_attempts=len(point_rows)+len(manifest.get('failed_attempts', [])),
                  successful_windows=len(point_rows),
                  excluded_windows=len(point_rows)-len(independent),
                  pooled_sample_statistics={k:stats([r[k] for r in eligible_errors]) for k in
                                           ['error_xy_m', 'error_3d_m', 'yaw_error_deg', 'orientation_error_deg']})
    if independent:
        mean_bias = np.array([[r['mean_error_'+a+'_m'] for a in 'xyz'] for r in independent])
        result['equal_weight_position_mean_3d_error_m'] = stats(np.linalg.norm(mean_bias, axis=1))
        positions = np.array([[r['mocap_world_mean_'+a+'_m'] for a in 'xyz'] for r in independent])
        result['pairwise_position_distances_m'] = [dict(a=independent[i]['label'], b=independent[j]['label'],
            distance_m=float(np.linalg.norm(positions[i]-positions[j]))) for i in range(len(positions)) for j in range(i+1,len(positions))]
        if len(positions)==3:
            v=positions[1,:2]-positions[0,:2];w=positions[2,:2]-positions[0,:2]
            result['third_point_distance_to_first_two_line_XY_m'] = float(abs(v[0]*w[1]-v[1]*w[0])/np.linalg.norm(v)) if np.linalg.norm(v)>1e-9 else None
    (batch / 'results.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    lines = ['# 三个静止位置独立验证', '',
             f"数据类别：{manifest['data_kind']}；独立位置完成 {len(independent)}/{manifest['planned_validation_points']}。初始化与原位复测不计入独立精度统计。", '',
             '固定 map↔mocap 与本次 map→odom 全程冻结；两边均为 base_link，误差为 LIO−动捕。无 NDT、无重拟合、无插值或填零。', '',
             '|位置|用途|样本|XY RMSE (mm)|3D RMSE (mm)|航向 RMSE (deg)|', '|---|---|---:|---:|---:|---:|']
    for r in point_rows:
        lines.append(f"|{r['label']}|{r['role']}|{r['pairs']}|{r['error_xy_m_rmse']*1000:.3f}|{r['error_3d_m_rmse']*1000:.3f}|{r['yaw_error_deg_rmse']:.4f}|")
    lines += ['', '独立位置的全部有效配对样本汇总（各样本等权）：', '',
              '|指标|样本数|RMSE|P95绝对值|最大绝对值|', '|---|---:|---:|---:|---:|']
    for key, label, scale in [('error_xy_m','XY (mm)',1000),('error_3d_m','3D (mm)',1000),('yaw_error_deg','航向 (deg)',1),('orientation_error_deg','三维姿态角 (deg)',1)]:
        s=result['pooled_sample_statistics'][key]
        if s['n']:lines.append(f"|{label}|{s['n']}|{s['rmse']*scale:.4f}|{s['p95_abs']*scale:.4f}|{s['max_abs']*scale:.4f}|")
    lines += ['', '逐点平均带符号偏差（LIO−动捕）：', '', '|位置|ΔX (mm)|ΔY (mm)|ΔZ (mm)|', '|---|---:|---:|---:|']
    for r in independent:
        lines.append(f"|{r['label']}|{r['mean_error_x_m']*1000:.3f}|{r['mean_error_y_m']*1000:.3f}|{r['mean_error_z_m']*1000:.3f}|")
    lines += ['', '逐样本坐标、带符号 XYZ/航向误差见 samples.csv；各位置统计见 point_summary.csv；原始消息在 points/ 与 excluded/。', '',
              'coverage 是该静止窗口原始/米制动捕同时间戳证据匹配比例，不代表整段移动过程覆盖率。未采集连续运动过程，不据此评估动态精度。', '',
              '精度相对于当前动捕基准，包含本次 map→odom 初始化、安装外参和 LIO 的综合影响；不能单凭本实验判定误差来源。动捕曝光延迟/独立时差尚未标定。', '',
              '因为本次重启初值与验证参考都使用同一份 map↔mocap 标定，本结果不能独立验证这份世界标定的准确度；若要验证原始建图锚定，需要原建图实例保持运行，或使用独立地图参照。', '',
              '本入口是动捕初始化后的 FAST-LIO 局部里程计；二维地图仅供显示，没有 PCD 匹配或 NDT 全局纠偏，不能宣称验证了历史地图重定位精度。', '',
              '每次采集使用首个满足预先设定时长、静止、同步、有效性要求的完整窗口；未按误差大小筛选最好片段。', '',
              f"失败采集尝试保留 {len(manifest.get('failed_attempts', []))} 次。"]
    (batch / 'report.md').write_text('\n'.join(lines)+'\n')
    if point_rows:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(7, 6))
        for r in point_rows:
            x,y = r['mean_mocap_map_x_m'],r['mean_mocap_map_y_m']
            ex,ey = r['mean_lio_map_x_m'],r['mean_lio_map_y_m']
            ax.plot([x,ex],[y,ey],color='gray')
            ax.plot(x,y,'o',color='tab:blue');ax.plot(ex,ey,'x',color='tab:orange')
            label=r['label'] if r['role']=='independent_validation' else 'init repeat (excluded)'
            ax.annotate(label,(x,y),xytext=(5,5),textcoords='offset points')
        ax.plot([],[],'o',color='tab:blue',label='Mocap reference');ax.plot([],[],'x',color='tab:orange',label='LIO estimate')
        ax.set(xlabel='map X (m)',ylabel='map Y (m)',title='Static positions (base_link); no refit')
        ax.set_aspect('equal',adjustable='datalim');ax.grid();ax.legend();fig.tight_layout();fig.savefig(batch/'positions.png',dpi=160);plt.close(fig)
        fig,axes=plt.subplots(2,1,figsize=(8,6),sharex=True)
        for a in 'xyz':axes[0].plot([r['mean_error_'+a+'_m']*1000 for r in point_rows],'o-',label=a.upper())
        axes[0].set(ylabel='Mean signed error (mm)');axes[0].legend();axes[0].grid()
        axes[1].plot([r['error_xy_m_rmse']*1000 for r in point_rows],'o-',label='XY RMSE')
        axes[1].plot([r['error_3d_m_rmse']*1000 for r in point_rows],'o-',label='3D RMSE')
        axes[1].set(ylabel='RMSE (mm)',xticks=range(len(point_rows)),xticklabels=[r['label'] if r['role']=='independent_validation' else 'init repeat\n(excluded)' for r in point_rows]);axes[1].legend();axes[1].grid()
        fig.tight_layout();fig.savefig(batch/'errors.png',dpi=160);plt.close(fig)
    return result


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--batch',required=True)
    parser.add_argument('--xy-only',action='store_true',help='Only add errors_xy.png from independent samples; preserve existing 3D results')
    args=parser.parse_args()
    print(json.dumps(plot_xy_only(args.batch) if args.xy_only else report(args.batch),ensure_ascii=False,indent=2))
