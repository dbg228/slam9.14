#!/usr/bin/env python3
"""Plot and summarize an already frozen calibration and completed validation.

This command never fits a transform or selects the lowest-error capture.
An explicitly supplied verification session may share the frozen inputs and
training files by relative symlink when a failed holdout was retained.
"""
import argparse
import datetime
import json
import math
import os
from pathlib import Path
import sys
from zoneinfo import ZoneInfo

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
from controller_tracking import world_calibration as wc


def report(session, verification_session=None):
    s = Path(session).resolve(); vdir = Path(verification_session or session).resolve()
    c = wc.check_calibration(s); wc.check_calibration(vdir)
    v = wc.load(vdir/'validation.json'); before = wc.sha(s/'calibration.yaml')
    if v['calibration_sha256'] != before or wc.sha(vdir/'calibration.yaml') != before:
        raise ValueError('Verification belongs to a different frozen transform')
    if v['holdout_hashes'] != wc.group_digest(vdir, wc.HOLDOUT):
        raise ValueError('Held-out raw data changed after verification')
    observations = [wc.get_group(s, g) for g in wc.TRAIN]+[wc.get_group(vdir, wc.HOLDOUT)]
    points = c['errors']['points']+v['errors']['points']
    attempts = []
    config = wc.load(s/'session.yaml')
    old = config.get('previous_failed_capture_session')
    candidates = ([Path(old)/'calib_01/summary.json'] if old else [])
    candidates += [s/g/'summary.json' for g in wc.GROUPS]
    if vdir != s: candidates.append(vdir/'test_04/summary.json')
    for path in candidates:
        if path.exists():
            d = wc.load(path)
            attempts.append(dict(path=str(path), group=d['group'], quality_passed=d['quality_passed'],
                failures=d['failures'], valid_mocap_samples=d['valid_mocap_samples'],
                pair_count=d['pair_count'], maximum_mocap_gap_s=d['maximum_mocap_gap_s'],
                raw_normalized_coverage=d['raw_normalized_check']['coverage']))
    r = np.asarray(c['map_from_mocap']['rotation_matrix'])
    tilt = math.degrees(math.acos(float(np.clip(r[2, 2], -1, 1))))
    result = dict(data_kind=c['data_kind'], completed_at=datetime.datetime.now(ZoneInfo('Asia/Shanghai')).isoformat(),
        session=str(s), verification_session=str(vdir), physical_placements=4,
        calibration_points=3, validation_points=1, attempts=attempts,
        qualified_capture_windows=sum(a['quality_passed'] for a in attempts),
        calibration_errors=c['errors'], validation_errors=v['errors'],
        map_from_mocap=c['map_from_mocap'], mocap_from_map=c['mocap_from_map'], origins=c['origins'],
        fitted_z_axis_tilt_deg=tilt, fitted_tilt_is_not_independent_ground_measurement=True,
        numeric_acceptance_passed=v['numeric_acceptance_passed'], protocol_passed=v['protocol_passed'],
        calibration_sha256=before, transform_refit=False, tf_published=False,
        absolute_accuracy_certified=False)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    figures = s/'figures'; figures.mkdir(exist_ok=True)
    measured = np.array([p['map_xyz_m'] for p in points])
    predicted = np.array([p['predicted_map_xyz_m'] for p in points])
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))
    labels = ['C1', 'C2', 'C3', 'V4']
    for ax, (i, j) in zip(axes, [(0, 1), (0, 2)]):
        ax.scatter(measured[:, i], measured[:, j], c='black', marker='+', s=85, label='Map estimate (NDT)')
        ax.scatter(predicted[:3, i], predicted[:3, j], facecolors='none', edgecolors='tab:blue', s=65, label='Transformed mocap: train')
        ax.scatter(predicted[3:, i], predicted[3:, j], facecolors='none', edgecolors='tab:orange', s=85, label='Transformed mocap: held out')
        for k, label in enumerate(labels):
            ax.plot([measured[k, i], predicted[k, i]], [measured[k, j], predicted[k, j]], c='tab:red', lw=1)
            ax.annotate(label, measured[k, [i, j]], xytext=(4, 5), textcoords='offset points')
        ax.set_xlabel('Map X (m)'); ax.set_ylabel('Map '+('Y' if j==1 else 'Z')+' (m)'); ax.grid(alpha=.3)
    axes[0].set_aspect('equal', adjustable='datalim'); axes[0].legend(fontsize=8)
    fig.suptitle('Four static placements; connectors show residuals, not a driven trajectory')
    fig.tight_layout(); fig.savefig(figures/'point_overlay.png', dpi=150); plt.close(fig)
    errors = np.array([p['error_xyz_m'] for p in points])*1000
    x = np.arange(4); fig, ax = plt.subplots(figsize=(9, 4.5))
    ax.axvspan(2.5, 3.5, color='tab:orange', alpha=.12, label='Held-out placement')
    for i, name in enumerate(['X', 'Y', 'Z']): ax.bar(x+(i-1)*.22, errors[:, i], width=.22, label='Signed '+name)
    ax.plot(x, [p['error_3d_m']*1000 for p in points], 'kD--', label='3D distance')
    ax.plot(x, [p['error_xy_m']*1000 for p in points], 's--', c='tab:purple', label='XY distance')
    ax.axhline(0, color='black', lw=.7); ax.set_xticks(x, ['C1 (fit)', 'C2 (fit)', 'C3 (fit)', 'V4 (validation)'])
    ax.set_ylabel('Transformed mocap minus map (mm)'); ax.grid(axis='y', alpha=.3)
    ax.legend(fontsize=8, ncol=3); fig.tight_layout(); fig.savefig(figures/'point_errors.png', dpi=150); plt.close(fig)
    verification_prefix = os.path.relpath(vdir, s)
    protocol_text = '通过' if v['protocol_passed'] else '未通过'
    lines = ['# 固定 map ↔ mocap_world：三点标定与独立一点验证', '',
        '**四个位置已采集，独立布点协议'+protocol_text+'。只用前三点求解；第四点未参与拟合，变换未修改、未广播。**', '',
        '数据类型：'+('实车测量' if c['data_kind']=='real_measurement' else '**合成软件数据，非实车结果**')+'。', '',
        '本报告使用同一个 base_link 原点，地图坐标来自 NDT/FAST-LIO，不是独立测量地图真值。', '',
        '|数据|位置数|三维 RMSE mm|三维最大 mm|XY RMSE mm|XY最大 mm|', '|---|---:|---:|---:|---:|---:|']
    for label, e, n in [('标定拟合', c['errors'], 3), ('独立验证', v['errors'], 1)]:
        values = [e['distance_3d_m']['rmse'], e['distance_3d_m']['max_abs'], e['distance_xy_m']['rmse'], e['distance_xy_m']['max_abs']]
        lines.append(f'|{label}|{n}|'+ '|'.join(f'{a*1000:.3f}' for a in values)+'|')
    lines += ['', '|位置|ΔX mm|ΔY mm|ΔZ mm|三维 mm|XY mm|', '|---|---:|---:|---:|---:|---:|']
    for p in points:
        values = [*p['error_xyz_m'], p['error_3d_m'], p['error_xy_m']]
        lines.append('|'+p['group']+'|'+ '|'.join(f'{a*1000:.3f}' for a in values)+'|')
    lines += ['', '误差符号为固定变换后的动捕坐标减地图估计坐标。独立验证只有一个位置，因此其距离 RMSE 与最大值相同；没有把三帧 NDT 当成三次独立实验。', '',
        '![静止点位叠加](figures/point_overlay.png)', '', '![各位置误差](figures/point_errors.png)', '',
        '|位置|有效动捕|NDT配对|跨度 s|动捕最大静止半径 mm|地图最大静止半径 mm|', '|---|---:|---:|---:|---:|---:|']
    for group, o in zip(wc.GROUPS, observations):
        lines.append(f"|{group}|{o['valid_mocap_samples']}|{o['pair_count']}|{o['mocap_span_s']:.3f}|{o['mocap_static']['maximum_radius_m']*1000:.3f}|{o['map_static']['maximum_radius_m']*1000:.3f}|")
    lines += ['', f"静止半径是各自数据散布，不是绝对误差。双方均值使用相同 NDT 时刻的动捕配对；配置窗口为{config['limits']['seconds']:.1f}秒，完整原始流与无效信息仍保存。", '',
        '|采集记录|数据质量|动捕数|NDT配对|说明|', '|---|---|---:|---:|---|']
    for a in attempts:
        path = Path(a['path'])
        label = str(path.relative_to(s)) if path.is_relative_to(s) else '前序会话/'+path.parent.name
        link = '['+label+']('+os.path.relpath(path, s)+')'
        lines.append(f"|{link}|{'通过' if a['quality_passed'] else '失败保留'}|{a['valid_mocap_samples']}|{a['pair_count']}|{'; '.join(a['failures']) or '按预设门槛通过'}|")
    lines += ['', f"共四个物理位置、{len(attempts)}段采集窗口，{result['qualified_capture_windows']}段合格；失败段均保留、未用于精度统计。", '']
    if vdir != s:
        lines += ['第4点使用显式指定的补采会话，不按残差自动选择窗口。三点变换未再拟合；共享文件使用相对软链接，避免复制原始数据。补采约定和同位置检查见 '+verification_prefix+'/retry_protocol.json 与 same_placement_check.json。', '']
    lines += [
        '动捕原点在 map 中 XYZ（m）：'+str(c['origins']['mocap_origin_in_map_m']), '',
        '地图原点在 mocap_world 中 XYZ（m）：'+str(c['origins']['map_origin_in_mocap_m']), '',
        f'冻结标定 SHA256：`{before}`。验证前后哈希一致，pre_validation_integrity.json 证明冻结时尚无第4点数据。', '',
        f'三维拟合的Z轴倾角约 {tilt:.3f}°；它来自这三个位置的配准，不能当作独立地面倾斜测量。', '',
        '验证误差明显大于拟合残差，且 Z 分项较大。误差包含 NDT、动捕、安装外参和时间配对影响；单个验证位置不能唯一分离根因。固定偏差可能被三点配准吸收，后续比较方法必须沿用同一变换，不能分别重标定后比较。', '',
        '数值验收：'+('未预先指定门槛，保持 null，只报告数值。' if v['numeric_acceptance_passed'] is None else ('通过预设门槛。' if v['numeric_acceptance_passed'] else '未通过预设门槛。'))+'本次没有认证全场、动态同步或独立地图绝对精度。', '',
        '启动与恢复证据、软件验证状态见会话状态与全工程验证记录。软件检查不代替实车精度检查。', '',
        '安装外参单独保存于 body_extrinsic.yaml；calibration_id 是会话审计标签，未提供厂家编号则仍为 null。时间使用Ubuntu接收时间，曝光延迟与跨源偏移未独立标定，原始VRPN无厂家光学置信度。没有跨缺口插值、填零、拟合尺度或输出CSV。', '',
        '[标定YAML](calibration.yaml) · [标定分项报告](calibration_report.md) · [独立验证报告]('+verification_prefix+'/validation_report.md) · [验证JSON]('+verification_prefix+'/validation.json)', '',
        '本报告只读已有数据与固定变换，不启动或停止节点，不自动驱动车子或修改正式导航TF。']
    (s/'report.md').write_text('\n'.join(lines)+'\n', encoding='utf-8')
    if wc.sha(s/'calibration.yaml') != before: raise RuntimeError('Calibration changed during reporting')
    (s/'report_summary.json').write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__); p.add_argument('--session', required=True)
    p.add_argument('--verification-session'); args = p.parse_args()
    result = report(args.session, args.verification_session)
    print('Report: '+str(Path(result['session'])/'report.md'))


if __name__ == '__main__': main()
