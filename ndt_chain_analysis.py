#!/usr/bin/env python3
"""Separate FAST-LIO initial, raw NDT, and final localization errors.

Input is one report directory created by ``ndt_match_evaluation.py``.  It
never starts ROS or changes a localization parameter.  The script only uses
poses evaluated at each NDT input-cloud timestamp.  In particular, it refuses
to treat the legacy callback-time ``initial_map_to_base`` field from a V1 bag
as a time-matched FAST-LIO prediction.
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


POSE_KEYS = ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')
STAGES = ('initial', 'raw', 'final')
STAGE_LABELS = {
    'initial': 'FAST-LIO initial prediction',
    'raw': 'NDT output',
    'final': 'Published final state at cloud time',
}
COLORS = {'initial': 'tab:red', 'raw': 'tab:orange', 'final': 'tab:blue'}


def bool_value(value):
    return str(value).strip().lower() in ('1', 'true', 'yes')


def as_float(value):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return math.nan
    return result


def pose_from_csv(row, prefix):
    values = {key: as_float(row.get(prefix+'_'+key)) for key in POSE_KEYS}
    return values if all(math.isfinite(value) for value in values.values()) else None


def pose_error(reference, estimate):
    ex, ey = estimate['x']-reference['x'], estimate['y']-reference['y']
    return {'ex_m': ex, 'ey_m': ey, 'e_xy_m': math.hypot(ex, ey),
            'yaw_error_rad': wrap(yaw(estimate)-yaw(reference))}


def load_report(report_dir):
    report_dir = Path(report_dir)
    summary_path, matches_path = report_dir/'summary.json', report_dir/'matches.csv'
    if not summary_path.is_file() or not matches_path.is_file():
        raise ValueError('Need summary.json and matches.csv from ndt_match_evaluation.py')
    summary = json.loads(summary_path.read_text(encoding='utf-8'))
    if summary.get('mocap_reference_pose') != 'base_link pose derived offline from recorded Tracker rigid-body pose':
        raise ValueError('Report does not prove that Tracker rigid-body poses were transformed to base_link offline')
    with matches_path.open(newline='', encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError('matches.csv is empty')
    rows.sort(key=lambda row: (int(as_float(row.get('stamp_ns'))), int(as_float(row.get('match_id')))))
    return summary, rows


def make_records(rows, only_relative_baseline=False):
    records, unavailable = [], {stage: 0 for stage in STAGES}
    for row in rows:
        if only_relative_baseline and not bool_value(row.get('is_relative_baseline')):
            continue
        if bool_value(row.get('in_alignment_window')) or not bool_value(row.get('reference_valid')):
            continue
        reference = pose_from_csv(row, 'reference')
        if reference is None:
            continue
        record = {
            'match_id': int(as_float(row.get('match_id'))),
            'stamp_ns': int(as_float(row.get('stamp_ns'))),
            'reference': reference,
            'accepted': bool_value(row.get('accepted')),
            'converged': bool_value(row.get('converged')),
            'iterations': int(as_float(row.get('iterations'))) if math.isfinite(as_float(row.get('iterations'))) else None,
            'fitness': as_float(row.get('fitness')) if bool_value(row.get('fitness_valid')) else None,
            'correction_translation_m': as_float(row.get('update_translation_m')),
            'correction_yaw_rad': as_float(row.get('update_yaw_rad')),
            'init_to_ndt_translation_m': as_float(row.get('init_to_ndt_translation_m')),
            'init_to_ndt_yaw_rad': as_float(row.get('init_to_ndt_yaw_rad')),
            'nearest_neighbor_inlier_ratio': as_float(row.get('nearest_neighbor_inlier_ratio')),
            'nearest_neighbor_residual_rms_m': as_float(row.get('nearest_neighbor_residual_rms_m')),
            'source_points': as_float(row.get('source_points')),
            'target_points': as_float(row.get('target_points')),
            'diagnostic_detail_available': bool_value(row.get('diagnostic_detail_available')),
            'stage': row.get('stage', ''),
        }
        for stage in STAGES:
            valid = bool_value(row.get(stage+'_valid'))
            pose = pose_from_csv(row, stage) if valid else None
            record[stage] = pose
            record[stage+'_error'] = pose_error(reference, pose) if pose is not None else None
            if pose is None:
                unavailable[stage] += 1
        records.append(record)
    if not records:
        raise ValueError('No post-alignment rows with a valid mocap reference')
    return records, unavailable


def scalar_stats(values):
    values = np.asarray(values, dtype=float)
    return {'mean': float(np.mean(values)), 'std_sample': float(np.std(values, ddof=1)) if len(values) > 1 else 0.0,
            'rmse': float(math.sqrt(float(np.mean(values**2)))),
            'p95_absolute': float(np.percentile(np.abs(values), 95)), 'max_absolute': float(np.max(np.abs(values)))}


def stage_metrics(records, stage):
    usable = [record for record in records if record[stage+'_error'] is not None]
    if not usable:
        return {'samples': 0, 'metrics_available': False,
                'reason': 'time-matched '+STAGE_LABELS[stage]+' is not recorded in this report'}
    ex = [record[stage+'_error']['ex_m'] for record in usable]
    ey = [record[stage+'_error']['ey_m'] for record in usable]
    xy = [record[stage+'_error']['e_xy_m'] for record in usable]
    yaw_deg = [record[stage+'_error']['yaw_error_rad']*180/math.pi for record in usable]
    return {'samples': len(usable), 'metrics_available': True,
            'ex_m': scalar_stats(ex), 'ey_m': scalar_stats(ey),
            'xy_m': {'rmse': float(math.sqrt(float(np.mean(np.square(xy))))),
                     'p95': float(np.percentile(xy, 95)), 'max': float(np.max(xy)), 'mean': float(np.mean(xy))},
            'yaw_deg': scalar_stats(yaw_deg),
            'weighting': 'one pose per NDT input-cloud timestamp'}


def relative_metrics(records, stage, max_gap_seconds):
    pairs, skipped = [], 0
    usable = [record for record in records if record[stage] is not None]
    for left, right in zip(usable, usable[1:]):
        dt = (right['stamp_ns']-left['stamp_ns'])/1e9
        if dt <= 0 or dt > max_gap_seconds:
            skipped += 1
            continue
        estimate_delta = np.array([right[stage]['x']-left[stage]['x'], right[stage]['y']-left[stage]['y']])
        reference_delta = np.array([right['reference']['x']-left['reference']['x'],
                                    right['reference']['y']-left['reference']['y']])
        estimate_yaw = wrap(yaw(right[stage])-yaw(left[stage]))
        reference_yaw = wrap(yaw(right['reference'])-yaw(left['reference']))
        pairs.append({'from_match_id': left['match_id'], 'to_match_id': right['match_id'], 'dt_s': dt,
                      'estimate_delta_xy_m': estimate_delta, 'reference_delta_xy_m': reference_delta,
                      'delta_error_xy_m': estimate_delta-reference_delta,
                      'estimate_delta_yaw_rad': estimate_yaw, 'reference_delta_yaw_rad': reference_yaw,
                      'delta_error_yaw_rad': wrap(estimate_yaw-reference_yaw)})
    if not pairs:
        return [], {'pairs': 0, 'metrics_available': False,
                    'reason': 'fewer than two time-adjacent '+STAGE_LABELS[stage]+' poses'}
    def vector(values):
        norms = np.linalg.norm(np.asarray(values), axis=1)
        return {'rmse_m': float(math.sqrt(float(np.mean(norms**2)))), 'p95_m': float(np.percentile(norms, 95)),
                'max_m': float(np.max(norms))}
    return pairs, {
        'pairs': len(pairs), 'skipped_long_or_nonpositive_gap_pairs': skipped,
        'max_gap_seconds': max_gap_seconds,
        'estimate_delta_position': vector([pair['estimate_delta_xy_m'] for pair in pairs]),
        'reference_delta_position': vector([pair['reference_delta_xy_m'] for pair in pairs]),
        'relative_delta_position_error': vector([pair['delta_error_xy_m'] for pair in pairs]),
        'relative_delta_yaw_error_deg': scalar_stats([pair['delta_error_yaw_rad']*180/math.pi for pair in pairs]),
    }


def ndt_effect(records):
    paired = [record for record in records if record['initial_error'] is not None and record['raw_error'] is not None]
    if not paired:
        return {'samples': 0, 'available': False,
                'reason': 'T_init at cloud time absent; V1 callback-time initial pose is intentionally not substituted'}
    changes = np.asarray([record['raw_error']['e_xy_m']-record['initial_error']['e_xy_m'] for record in paired])
    return {'samples': len(paired), 'available': True,
            'raw_minus_initial_xy_error_m': scalar_stats(changes),
            'raw_improves_xy_error_frames': int(np.count_nonzero(changes < 0.0)),
            'raw_worsens_xy_error_frames': int(np.count_nonzero(changes > 0.0)),
            'interpretation': 'negative raw-minus-initial means NDT reduced that frame\'s XY error'}


def public_record(record):
    output = {key: value for key, value in record.items() if key not in (*STAGES, 'reference',
              *(stage+'_error' for stage in STAGES))}
    for prefix in ('reference', *STAGES):
        pose = record.get(prefix)
        for key in POSE_KEYS:
            output[prefix+'_'+key] = '' if pose is None else pose[key]
    for stage in STAGES:
        error = record[stage+'_error']
        for key in ('ex_m', 'ey_m', 'e_xy_m', 'yaw_error_rad'):
            output[stage+'_'+key] = '' if error is None else error[key]
    return output


def write_csv(path, records):
    rows = [public_record(record) for record in records]
    fields = sorted({key for row in rows for key in row})
    with Path(path).open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader(); writer.writerows(rows)


def context_rows(records, requested_ids, half_window):
    selection = []
    index = {record['match_id']: position for position, record in enumerate(records)}
    missing = [item for item in requested_ids if item not in index]
    for match_id in requested_ids:
        if match_id not in index:
            continue
        center = index[match_id]
        for position in range(max(0, center-half_window), min(len(records), center+half_window+1)):
            row = public_record(records[position])
            row['requested_outlier_match_id'] = match_id
            row['offset_from_requested_match'] = position-center
            selection.append(row)
    fields = sorted({key for row in selection for key in row})
    return selection, fields, missing


def write_context(path, records, requested_ids, half_window):
    rows, fields, missing = context_rows(records, requested_ids, half_window)
    with Path(path).open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader(); writer.writerows(rows)
    return missing, len(rows)


def plots(output, records, motion):
    first_stamp = min(record['stamp_ns'] for record in records)
    time = [(record['stamp_ns']-first_stamp)/1e9 for record in records]
    for extension in ('png', 'svg'):
        fig, axes = plt.subplots(4, 1, figsize=(11, 12), sharex=True, constrained_layout=True)
        for stage in STAGES:
            selected = [(t, record) for t, record in zip(time, records) if record[stage+'_error'] is not None]
            if not selected:
                continue
            axes[0].plot([item[0] for item in selected], [item[1][stage+'_error']['e_xy_m']*1000 for item in selected],
                         '.-', color=COLORS[stage], label=STAGE_LABELS[stage])
            axes[1].plot([item[0] for item in selected], [item[1][stage+'_error']['yaw_error_rad']*180/math.pi for item in selected],
                         '.-', color=COLORS[stage], label=STAGE_LABELS[stage])
        axes[0].set_ylabel('XY error (mm)'); axes[1].set_ylabel('Yaw error (deg)')
        axes[2].plot(time, [math.nan if record['fitness'] is None else record['fitness'] for record in records],
                     '.-', color='tab:green', label='PCL fitness (lower better)')
        axes[2].set_ylabel('Fitness')
        axes[3].plot(time, [record['correction_translation_m']*1000 for record in records], '.-',
                     color='tab:orange', label='map→odom correction magnitude')
        axes[3].step(time, [record['iterations'] if record['iterations'] is not None else math.nan for record in records],
                     where='mid', color='tab:gray', label='PCL outer iterations')
        axes[3].set_ylabel('Correction mm / iterations'); axes[3].set_xlabel('Seconds from first NDT record')
        for axis in axes:
            axis.grid(alpha=.3)
            if axis.lines:
                axis.legend(loc='best')
        fig.suptitle('Localization chain error at NDT input-cloud timestamps')
        fig.savefig(Path(output)/('chain_timeseries.'+extension), dpi=180); plt.close(fig)

        fig, axes = plt.subplots(2, 1, figsize=(10, 7), constrained_layout=True)
        for stage, pairs in motion.items():
            if not pairs:
                continue
            motion_time = np.cumsum([pair['dt_s'] for pair in pairs])
            axes[0].plot(motion_time, [np.linalg.norm(pair['delta_error_xy_m'])*1000 for pair in pairs],
                         '.-', color=COLORS[stage], label=STAGE_LABELS[stage])
            axes[1].plot(motion_time, [pair['delta_error_yaw_rad']*180/math.pi for pair in pairs],
                         '.-', color=COLORS[stage], label=STAGE_LABELS[stage])
        axes[0].set_ylabel('Relative delta XY error (mm)')
        axes[1].set_ylabel('Relative delta yaw error (deg)'); axes[1].set_xlabel('Cumulative adjacent-pair time (s)')
        for axis in axes:
            axis.grid(alpha=.3)
            if axis.lines:
                axis.legend(loc='best')
        fig.suptitle('Adjacent-frame relative motion error')
        fig.savefig(Path(output)/('chain_relative_motion.'+extension), dpi=180); plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report-dir', required=True, type=Path)
    parser.add_argument('--output', type=Path, help='Defaults to REPORT_DIR/chain_analysis')
    parser.add_argument('--case-match-ids', default='225,226,241')
    parser.add_argument('--case-half-window', type=int, default=4)
    parser.add_argument('--max-adjacent-gap-seconds', type=float, default=5.0)
    args = parser.parse_args()
    if args.case_half_window < 0 or args.max_adjacent_gap_seconds <= 0:
        raise SystemExit('case-half-window must be nonnegative and max-adjacent-gap-seconds must be positive')
    output = args.output or args.report_dir/'chain_analysis'
    if output.exists():
        raise SystemExit('Output already exists: '+str(output))
    case_ids = [int(value) for value in args.case_match_ids.split(',') if value.strip()]
    summary, rows = load_report(args.report_dir)
    alignment_mode = summary.get('alignment_mode', 'unknown')
    alignment_labels = {
        'start': 'start-window aligned relative error',
        'first_init': 'first-T_init relative error (first NDT correction retained)',
        'first_raw_ndt': 'first-raw-NDT relative error (raw baseline is zero by construction)',
    }
    alignment_label = alignment_labels.get(alignment_mode, 'relative error in the input report alignment frame')
    cold_first_only = alignment_mode == 'first_init'
    records, unavailable = make_records(rows, only_relative_baseline=cold_first_only)
    metrics = {stage: stage_metrics(records, stage) for stage in STAGES}
    motion_pairs, motion_metrics = {}, {}
    for stage in STAGES:
        motion_pairs[stage], motion_metrics[stage] = relative_metrics(records, stage, args.max_adjacent_gap_seconds)
    result = {
        'input_report': str(args.report_dir.resolve()),
        'input_report_schema_version': summary.get('schema_version'),
        'input_alignment_mode': alignment_mode,
        'input_alignment_interpretation': alignment_label,
        'analysis_scope': ('first true NDT registration only' if cold_first_only
                           else 'all report rows that have a valid reference and are outside any alignment window'),
        'records_after_alignment_and_valid_reference': len(records),
        'unavailable_stage_records': unavailable,
        'relative_error_in_input_alignment_frame': metrics,
        'adjacent_frame_relative_motion': motion_metrics,
        'ndt_effect_raw_vs_initial': ndt_effect(records),
        'important_time_semantics': (
            'All relative metrics use only the input-cloud timestamp poses and the fixed alignment from the input report. '
            'T_init is never reconstructed from the V1 callback-time legacy field.'),
    }
    output.mkdir(parents=True)
    write_csv(output/'chain_frame_errors.csv', records)
    missing, context_count = write_context(output/'case_context.csv', records, case_ids, args.case_half_window)
    result['case_context'] = {'requested_match_ids': case_ids, 'missing_match_ids': missing,
                              'half_window_frames': args.case_half_window, 'rows_written': context_count,
                              'csv': 'case_context.csv'}
    (output/'chain_summary.json').write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    report = [
        '# Localization-chain analysis', '',
        'All errors are against the same base_link mocap reference and the same fixed alignment from the input report.',
        'The comparison timestamp is the NDT input-cloud timestamp, not matching completion or pose-publication time.', '',
        '## '+alignment_label, '```json', json.dumps(metrics, ensure_ascii=False, indent=2), '```', '',
        '## Adjacent-frame relative motion', '```json', json.dumps(motion_metrics, ensure_ascii=False, indent=2), '```', '',
        '## NDT effect from initial prediction to raw result', '```json', json.dumps(result['ndt_effect_raw_vs_initial'], ensure_ascii=False, indent=2), '```', '',
        f'Case context for {case_ids}: `case_context.csv` (±{args.case_half_window} records).',
    ]
    (output/'chain_summary.md').write_text('\n'.join(report)+'\n', encoding='utf-8')
    plots(output, records, motion_pairs)
    print('Wrote '+str(output))


if __name__ == '__main__':
    main()
