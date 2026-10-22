#!/usr/bin/env python3
"""Decompose residual static NDT errors from an NDT-vs-mocap evaluation report.

This is an offline reader for a report produced by ndt_match_evaluation.py.  It
does not start ROS, publish TF, alter NDT, or command the robot.  In particular,
it refuses reports that do not explicitly record an offline Tracker rigid-body
to base_link transform: comparing a Tracker origin directly to NDT base_link
would make the result geometrically ambiguous.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from ndt_match_evaluation_core import wrap, yaw


def bool_value(value):
    return str(value).strip().lower() in ('1', 'true', 'yes')


def finite_float(value, field, row):
    try:
        number = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f'match {row.get("match_id", "?")}: invalid {field}') from error
    if not math.isfinite(number):
        raise ValueError(f'match {row.get("match_id", "?")}: non-finite {field}')
    return number


def pose_from_csv(row, prefix):
    return {
        'x': finite_float(row[prefix+'_x'], prefix+'_x', row),
        'y': finite_float(row[prefix+'_y'], prefix+'_y', row),
        'z': finite_float(row[prefix+'_z'], prefix+'_z', row),
        'qx': finite_float(row[prefix+'_qx'], prefix+'_qx', row),
        'qy': finite_float(row[prefix+'_qy'], prefix+'_qy', row),
        'qz': finite_float(row[prefix+'_qz'], prefix+'_qz', row),
        'qw': finite_float(row[prefix+'_qw'], prefix+'_qw', row),
    }


def read_report(report_dir, pose_kind):
    report_dir = Path(report_dir)
    summary_path, matches_path = report_dir/'summary.json', report_dir/'matches.csv'
    if not summary_path.is_file() or not matches_path.is_file():
        raise ValueError('Need summary.json and matches.csv from ndt_match_evaluation.py')
    summary = json.loads(summary_path.read_text(encoding='utf-8'))
    if summary.get('mocap_reference_pose') != 'base_link pose derived offline from recorded Tracker rigid-body pose':
        raise ValueError('Report does not prove that Tracker rigid-body poses were transformed to base_link offline')
    transform = summary.get('tracker_to_base_applied')
    if not isinstance(transform, dict):
        raise ValueError('Report has no explicit Tracker0->base_link transform')
    with matches_path.open(newline='', encoding='utf-8') as stream:
        source_rows = list(csv.DictReader(stream))
    if not source_rows:
        raise ValueError('matches.csv is empty')
    all_stamps = [finite_float(row.get('stamp_ns'), 'stamp_ns', row) for row in source_rows]
    first_stamp = min(all_stamps)
    records, rejected = [], {'alignment_or_warmup': 0, 'invalid_reference': 0, 'invalid_pose': 0}
    for row in source_rows:
        if bool_value(row.get('in_alignment_window')):
            rejected['alignment_or_warmup'] += 1
            continue
        if not bool_value(row.get('reference_valid')):
            rejected['invalid_reference'] += 1
            continue
        if not bool_value(row.get(pose_kind+'_valid')):
            rejected['invalid_pose'] += 1
            continue
        try:
            reference, ndt = pose_from_csv(row, 'reference'), pose_from_csv(row, pose_kind)
        except ValueError:
            rejected['invalid_pose'] += 1
            continue
        stamp = int(finite_float(row.get('stamp_ns'), 'stamp_ns', row))
        ex, ey = ndt['x']-reference['x'], ndt['y']-reference['y']
        yaw_error = wrap(yaw(ndt)-yaw(reference))
        records.append({
            'match_id': int(finite_float(row.get('match_id'), 'match_id', row)),
            'stamp_ns': stamp, 'time_from_first_ndt_s': (stamp-first_stamp)/1e9,
            'ndt': ndt, 'reference': reference,
            'ex_m': ex, 'ey_m': ey, 'e_xy_m': math.hypot(ex, ey), 'yaw_error_rad': yaw_error,
            'fitness': finite_float(row.get('fitness'), 'fitness', row) if bool_value(row.get('fitness_valid')) else None,
            'correction_translation_m': finite_float(row.get('update_translation_m'), 'update_translation_m', row),
            'correction_yaw_rad': finite_float(row.get('update_yaw_rad'), 'update_yaw_rad', row),
            'converged': bool_value(row.get('converged')),
            'accepted': bool_value(row.get('accepted')),
            'iterations': int(finite_float(row.get('iterations'), 'iterations', row)),
            'input_points': int(finite_float(row.get('input_points'), 'input_points', row)),
            'source_points': int(finite_float(row.get('source_points'), 'source_points', row)),
            'target_raw_points': int(finite_float(row.get('target_raw_points'), 'target_raw_points', row)),
            'target_points': int(finite_float(row.get('target_points'), 'target_points', row)),
        })
    records.sort(key=lambda item: (item['stamp_ns'], item['match_id']))
    if len(records) < 3:
        raise ValueError('Need at least three evaluable post-alignment records')
    return summary, transform, records, rejected


def sample_stats(values):
    values = np.asarray(values, dtype=float)
    return {'mean': float(np.mean(values)), 'std_sample': float(np.std(values, ddof=1)) if len(values) > 1 else 0.,
            'rmse': float(math.sqrt(float(np.mean(values**2)))),
            'p95_absolute': float(np.percentile(np.abs(values), 95)),
            'max_absolute': float(np.max(np.abs(values)))}


def vector_stats(vectors):
    values = np.asarray(vectors, dtype=float)
    norms = np.linalg.norm(values, axis=1)
    return {'samples': int(len(values)), 'mean_vector_m': [float(value) for value in np.mean(values, axis=0)],
            'rmse_m': float(math.sqrt(float(np.mean(norms**2)))),
            'p95_m': float(np.percentile(norms, 95)), 'max_m': float(np.max(norms))}


def scalar_motion_stats(values, scale=1.0):
    values = np.asarray(values, dtype=float) * scale
    return {'samples': int(len(values)), 'rmse': float(math.sqrt(float(np.mean(values**2)))),
            'p95_absolute': float(np.percentile(np.abs(values), 95)),
            'max_absolute': float(np.max(np.abs(values)))}


def pearson_or_none(left, right):
    left, right = np.asarray(left, dtype=float), np.asarray(right, dtype=float)
    if len(left) < 3 or np.std(left) < 1e-12 or np.std(right) < 1e-12:
        return None
    return float(np.corrcoef(left, right)[0, 1])


def relative_motion(records, max_gap_seconds):
    pairs, skipped = [], 0
    for previous, current in zip(records, records[1:]):
        dt = (current['stamp_ns']-previous['stamp_ns'])/1e9
        if dt <= 0 or dt > max_gap_seconds:
            skipped += 1
            continue
        ndt_delta = np.array([current['ndt']['x']-previous['ndt']['x'], current['ndt']['y']-previous['ndt']['y']])
        ref_delta = np.array([current['reference']['x']-previous['reference']['x'],
                              current['reference']['y']-previous['reference']['y']])
        ndt_yaw = wrap(yaw(current['ndt'])-yaw(previous['ndt']))
        ref_yaw = wrap(yaw(current['reference'])-yaw(previous['reference']))
        pairs.append({'match_id_from': previous['match_id'], 'match_id_to': current['match_id'], 'dt_s': dt,
                      'ndt_delta_xy_m': ndt_delta, 'reference_delta_xy_m': ref_delta,
                      'delta_error_xy_m': ndt_delta-ref_delta,
                      'ndt_delta_yaw_rad': ndt_yaw, 'reference_delta_yaw_rad': ref_yaw,
                      'delta_error_yaw_rad': wrap(ndt_yaw-ref_yaw)})
    if not pairs:
        raise ValueError('No adjacent NDT records remain after gap filtering')
    ndt_xy = [item['ndt_delta_xy_m'] for item in pairs]
    ref_xy = [item['reference_delta_xy_m'] for item in pairs]
    err_xy = [item['delta_error_xy_m'] for item in pairs]
    return pairs, {'pairs': len(pairs), 'skipped_long_or_nonpositive_gap_pairs': skipped,
                   'dt_seconds': {'median': float(np.median([item['dt_s'] for item in pairs])),
                                  'max': float(max(item['dt_s'] for item in pairs))},
                   'ndt_delta_position': vector_stats(ndt_xy),
                   'mocap_delta_position': vector_stats(ref_xy),
                   'relative_delta_position_error': vector_stats(err_xy),
                   'ndt_delta_yaw_deg': scalar_motion_stats([item['ndt_delta_yaw_rad'] for item in pairs], 180/math.pi),
                   'mocap_delta_yaw_deg': scalar_motion_stats([item['reference_delta_yaw_rad'] for item in pairs], 180/math.pi),
                   'relative_delta_yaw_error_deg': scalar_motion_stats([item['delta_error_yaw_rad'] for item in pairs], 180/math.pi)}


def analyze(records, outlier_threshold_m, max_gap_seconds):
    ex, ey = np.array([item['ex_m'] for item in records]), np.array([item['ey_m'] for item in records])
    error_vectors = np.column_stack((ex, ey))
    center = np.mean(error_vectors, axis=0)
    residual = error_vectors-center
    xy = np.linalg.norm(error_vectors, axis=1)
    outliers = [item for item in records if item['e_xy_m'] >= outlier_threshold_m]
    motion_pairs, motion = relative_motion(records, max_gap_seconds)
    finite_fitness = [item for item in records if item['fitness'] is not None]
    comparison_fields = ('fitness', 'correction_translation_m', 'correction_yaw_rad', 'iterations',
                         'input_points', 'source_points', 'target_points')
    outlier_comparison = {}
    regular = [item for item in records if item['e_xy_m'] < outlier_threshold_m]
    for field in comparison_fields:
        all_values = [item[field] for item in records if item[field] is not None]
        high_values = [item[field] for item in outliers if item[field] is not None]
        normal_values = [item[field] for item in regular if item[field] is not None]
        outlier_comparison[field] = {
            'all_mean': float(np.mean(all_values)) if all_values else None,
            'outlier_mean': float(np.mean(high_values)) if high_values else None,
            'non_outlier_mean': float(np.mean(normal_values)) if normal_values else None,
        }
    return {
        'evaluable_frames': len(records),
        'error_components_m': {'ex': sample_stats(ex), 'ey': sample_stats(ey)},
        'xy_error_m': {'mean': float(np.mean(xy)), 'rmse': float(math.sqrt(float(np.mean(xy**2)))),
                        'p95': float(np.percentile(xy, 95)), 'max': float(np.max(xy))},
        'yaw_error_deg': sample_stats([item['yaw_error_rad'] * 180/math.pi for item in records]),
        'residual_center_m': [float(center[0]), float(center[1])],
        'residual_center_norm_m': float(np.linalg.norm(center)),
        'center_removed_random_xy_rms_m': float(math.sqrt(float(np.mean(np.sum(residual**2, axis=1))))),
        'center_squared_fraction_of_xy_rmse_squared': float(np.dot(center, center)/np.mean(xy**2)),
        'fitness_error_pearson_r': pearson_or_none([item['fitness'] for item in finite_fitness],
                                                    [item['e_xy_m'] for item in finite_fitness]),
        'correction_error_pearson_r': pearson_or_none([item['correction_translation_m'] for item in records], xy),
        'outlier_threshold_m': outlier_threshold_m,
        'outlier_count': len(outliers),
        'outlier_match_ids': [item['match_id'] for item in outliers],
        'outlier_field_means': outlier_comparison,
        'point_cloud_overlap_ratio': 'unavailable: current NdtMatchDiagnostic records source/target point counts but no overlap-ratio metric',
        'relative_motion': motion,
    }, outliers, motion_pairs


def public_record(record):
    result = {key: value for key, value in record.items() if key not in ('ndt', 'reference')}
    for prefix in ('ndt', 'reference'):
        for key, value in record[prefix].items():
            result[prefix+'_'+key] = value
    result['yaw_error_deg'] = result.pop('yaw_error_rad') * 180/math.pi
    result['correction_yaw_deg'] = result.pop('correction_yaw_rad') * 180/math.pi
    return result


def write_csv(path, rows):
    public = [public_record(row) for row in rows]
    fields = sorted({key for row in public for key in row})
    with Path(path).open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader(); writer.writerows(public)


def save_plots(directory, records, summary, outliers, motion_pairs):
    directory = Path(directory)
    ex, ey = np.array([item['ex_m'] for item in records])*1000, np.array([item['ey_m'] for item in records])*1000
    center = np.array(summary['residual_center_m'])*1000
    fig, axis = plt.subplots(figsize=(7, 6), constrained_layout=True)
    axis.scatter(ex, ey, color='tab:blue', s=28, label='Post-alignment NDT frames')
    if outliers:
        axis.scatter([item['ex_m']*1000 for item in outliers], [item['ey_m']*1000 for item in outliers],
                     color='tab:red', marker='x', s=55, label=f"XY error ≥ {summary['outlier_threshold_m']*1000:.0f} mm")
    axis.scatter([0], [0], color='black', marker='+', s=110, label='Zero error')
    axis.scatter([center[0]], [center[1]], color='tab:orange', marker='D', s=45, label='Residual center')
    axis.axhline(0, color='0.55', lw=.8); axis.axvline(0, color='0.55', lw=.8)
    axis.set(xlabel='ex = x_NDT − x_mocap (mm)', ylabel='ey = y_NDT − y_mocap (mm)',
             title='Static XY error scatter (common start alignment)', aspect='equal')
    axis.grid(alpha=.3); axis.legend(loc='best')
    for extension in ('png', 'svg'):
        fig.savefig(directory/('xy_error_scatter.'+extension), dpi=180)
    plt.close(fig)

    time = np.asarray([item['time_from_first_ndt_s'] for item in records])
    fig, axes = plt.subplots(5, 1, figsize=(11, 12), sharex=True, constrained_layout=True)
    axes[0].plot(time, [item['e_xy_m']*1000 for item in records], '.-', color='tab:blue', label='XY error')
    axes[0].axhline(summary['outlier_threshold_m']*1000, color='tab:red', ls='--', lw=.9, label='Outlier threshold')
    axes[0].set_ylabel('XY error (mm)'); axes[0].legend()
    axes[1].plot(time, [item['yaw_error_rad']*180/math.pi for item in records], '.-', color='tab:purple')
    axes[1].set_ylabel('Yaw error (deg)')
    fitness = [math.nan if item['fitness'] is None else item['fitness'] for item in records]
    axes[2].plot(time, fitness, '.-', color='tab:green')
    axes[2].set_ylabel('Fitness\n(lower better)')
    axes[3].plot(time, [item['correction_translation_m']*1000 for item in records], '.-', color='tab:orange')
    axes[3].set_ylabel('Correction\ntranslation (mm)')
    axes[4].step(time, [1 if item['converged'] else 0 for item in records], where='mid', color='tab:green', label='Converged')
    twin = axes[4].twinx()
    twin.plot(time, [item['iterations'] for item in records], 'o-', ms=3, color='tab:gray', label='Iterations')
    axes[4].set(ylim=(-.1, 1.1), ylabel='Converged', xlabel='Seconds from first NDT record')
    twin.set_ylabel('Iterations')
    for axis in axes:
        axis.grid(alpha=.3)
    fig.suptitle('Static NDT error, score, correction, and convergence at cloud timestamps')
    for extension in ('png', 'svg'):
        fig.savefig(directory/('static_error_timeseries.'+extension), dpi=180)
    plt.close(fig)

    time_motion = np.cumsum([0.] + [item['dt_s'] for item in motion_pairs])[1:]
    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True, constrained_layout=True)
    axes[0].plot(time_motion, [np.linalg.norm(item['ndt_delta_xy_m'])*1000 for item in motion_pairs], '.-', label='NDT increment')
    axes[0].plot(time_motion, [np.linalg.norm(item['reference_delta_xy_m'])*1000 for item in motion_pairs], '.-', label='Mocap increment')
    axes[0].plot(time_motion, [np.linalg.norm(item['delta_error_xy_m'])*1000 for item in motion_pairs], '.-', label='Increment error')
    axes[0].set_ylabel('Adjacent Δposition (mm)'); axes[0].legend(); axes[0].grid(alpha=.3)
    axes[1].plot(time_motion, [item['ndt_delta_yaw_rad']*180/math.pi for item in motion_pairs], '.-', label='NDT increment')
    axes[1].plot(time_motion, [item['reference_delta_yaw_rad']*180/math.pi for item in motion_pairs], '.-', label='Mocap increment')
    axes[1].plot(time_motion, [item['delta_error_yaw_rad']*180/math.pi for item in motion_pairs], '.-', label='Increment error')
    axes[1].set(xlabel='Elapsed adjacent-pair time (s)', ylabel='Adjacent Δyaw (deg)'); axes[1].legend(); axes[1].grid(alpha=.3)
    fig.suptitle('Static adjacent-frame relative motion')
    for extension in ('png', 'svg'):
        fig.savefig(directory/('relative_motion.'+extension), dpi=180)
    plt.close(fig)


def write_markdown(path, report_dir, pose_kind, transform, excluded, summary, outliers):
    ex, ey = summary['error_components_m']['ex'], summary['error_components_m']['ey']
    xy, motion = summary['xy_error_m'], summary['relative_motion']
    center_mm = [value*1000 for value in summary['residual_center_m']]
    lines = [
        '# 静止 NDT 误差分解', '',
        f'- 输入报告：`{Path(report_dir).resolve()}`',
        f'- NDT pose：`{pose_kind}`；动捕参考：Tracker0 刚体经报告中显式的 Tracker0→base_link 外参离线转换。',
        f'- 外参平移 m：`{transform.get("translation_m")}`；四元数 xyzw：`{transform.get("quaternion_xyzw")}`。',
        '- 起始对齐模式会吸收常数 `mocap_world→map` 偏差及常数平面安装偏差；本报告只能分解**对齐后的残余中心偏移与时变散布**，不能声称测得绝对外参或绝对地图偏差。',
        f'- 排除的起始等待/对齐记录：{excluded["alignment_or_warmup"]}；无效参考：{excluded["invalid_reference"]}；无效 NDT pose：{excluded["invalid_pose"]}。', '',
        '## 逐帧误差统计', '',
        '| 指标 | 数值 |', '| --- | ---: |',
        f'| 帧数 | {summary["evaluable_frames"]} |',
        f'| mean(ex) / std(ex) | {ex["mean"]*1000:.3f} / {ex["std_sample"]*1000:.3f} mm |',
        f'| mean(ey) / std(ey) | {ey["mean"]*1000:.3f} / {ey["std_sample"]*1000:.3f} mm |',
        f'| XY RMSE / P95 / max | {xy["rmse"]*1000:.3f} / {xy["p95"]*1000:.3f} / {xy["max"]*1000:.3f} mm |',
        f'| 残余中心 (ex, ey) | ({center_mm[0]:.3f}, {center_mm[1]:.3f}) mm |',
        f'| 残余中心模长 | {summary["residual_center_norm_m"]*1000:.3f} mm |',
        f'| 去中心随机 XY RMS | {summary["center_removed_random_xy_rms_m"]*1000:.3f} mm |',
        f'| 中心项占 XY RMSE² | {summary["center_squared_fraction_of_xy_rmse_squared"]*100:.2f}% |',
        f'| yaw RMSE / P95 / max | {summary["yaw_error_deg"]["rmse"]:.4f} / {summary["yaw_error_deg"]["p95_absolute"]:.4f} / {summary["yaw_error_deg"]["max_absolute"]:.4f} deg |', '',
        '## 相邻帧重复匹配稳定性', '',
        f'- 相邻帧对：{motion["pairs"]}，中位时间间隔 {motion["dt_seconds"]["median"]:.3f} s，因长间隔跳过 {motion["skipped_long_or_nonpositive_gap_pairs"]} 对。',
        f'- NDT Δposition RMSE：{motion["ndt_delta_position"]["rmse_m"]*1000:.3f} mm；相对动捕 Δposition 误差 RMSE：{motion["relative_delta_position_error"]["rmse_m"]*1000:.3f} mm。',
        f'- NDT Δyaw RMSE：{motion["ndt_delta_yaw_deg"]["rmse"]:.4f} deg；相对动捕 Δyaw 误差 RMSE：{motion["relative_delta_yaw_error_deg"]["rmse"]:.4f} deg。', '',
        '## 异常帧与可获得诊断量', '',
        f'- 阈值：XY error ≥ {summary["outlier_threshold_m"]*1000:.1f} mm；异常帧数：{summary["outlier_count"]}；match_id：{summary["outlier_match_ids"]}。',
        f'- Pearson r(error, fitness)：{summary["fitness_error_pearson_r"]}；r(error, correction magnitude)：{summary["correction_error_pearson_r"]}。这些只是小样本相关性，不代表因果。',
        f'- 点云重叠率：{summary["point_cloud_overlap_ratio"]}',
        '- 请查看 `outlier_frames.csv` 中每个异常帧的 fitness、校正量、航向误差、迭代次数和点数；没有记录的指标不会被臆造。', '',
        '## 解释边界', '',
        '若散点的残余中心远小于去中心随机 RMS，则这段对齐后的误差主要表现为时变定位/里程计/NDT链路散布，而不是残余常数偏置。但起始对齐已经消除了常数坐标系偏差，故不能用该结论否定未独立标定的 `mocap_world→map` 或 Tracker0→base_link 外参。要量化那些绝对偏差，需要独立的 map←mocap_world 标定。'
    ]
    Path(path).write_text('\n'.join(lines)+'\n', encoding='utf-8')


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report-dir', type=Path, required=True, help='Successful ndt_match_evaluation.py report directory')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--pose-kind', choices=('raw', 'final'), default='final')
    parser.add_argument('--outlier-threshold-m', type=float, default=.04)
    parser.add_argument('--max-adjacent-gap-seconds', type=float, default=5.)
    return parser.parse_args()


def main():
    args = parse_args()
    if args.output.exists():
        raise SystemExit(f'Output already exists: {args.output}')
    if not math.isfinite(args.outlier_threshold_m) or args.outlier_threshold_m <= 0 or not math.isfinite(args.max_adjacent_gap_seconds) or args.max_adjacent_gap_seconds <= 0:
        raise SystemExit('Outlier threshold and maximum adjacent gap must be finite and positive')
    report, transform, records, excluded = read_report(args.report_dir, args.pose_kind)
    summary, outliers, motion_pairs = analyze(records, args.outlier_threshold_m, args.max_adjacent_gap_seconds)
    summary.update({
        'source_report': str(args.report_dir.resolve()), 'pose_kind': args.pose_kind,
        'coordinate_chain': 'NDT map->base_link compared with Tracker0->base_link transformed pose, then common map<-mocap start alignment',
        'alignment_mode': report.get('alignment_mode'), 'start_alignment': report.get('start_alignment'),
        'tracker_to_base_applied': transform, 'excluded_records': excluded,
    })
    args.output.mkdir(parents=True)
    write_csv(args.output/'static_frame_errors.csv', records)
    write_csv(args.output/'outlier_frames.csv', outliers)
    (args.output/'static_error_summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    write_markdown(args.output/'static_error_report.md', args.report_dir, args.pose_kind, transform, excluded, summary, outliers)
    save_plots(args.output, records, summary, outliers, motion_pairs)
    print(f'Wrote {args.output}; frames={len(records)}, outliers={len(outliers)}')


if __name__ == '__main__':
    main()
