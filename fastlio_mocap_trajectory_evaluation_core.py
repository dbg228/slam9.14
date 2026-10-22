"""Pure geometry, timing and statistics for FAST-LIO--mocap trajectory tests.

This module deliberately has no ROS imports.  It never estimates a time
offset: callers must supply one.  Poses use ``x, y, z, qx, qy, qz, qw`` and
integer ``stamp_ns`` fields.  The fixed planar alignment is estimated once
from a caller-selected stationary start window and is never re-fit later.
"""
from __future__ import annotations

from bisect import bisect_left
from dataclasses import asdict, dataclass
import math

import numpy as np

from ndt_match_evaluation_core import normalized_quaternion, slerp, wrap, yaw, yaw_quaternion


POSE_KEYS = ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')


def finite_pose(pose: dict | None) -> bool:
    """Return true only for a finite, non-zero quaternion pose."""
    try:
        if pose is None or not all(math.isfinite(float(pose[key])) for key in POSE_KEYS):
            return False
        normalized_quaternion(pose)
        return True
    except (KeyError, TypeError, ValueError):
        return False


def _copy_pose(pose: dict) -> dict:
    return {key: pose[key] for key in ('stamp_ns', 'valid', *POSE_KEYS) if key in pose}


def interpolate_pose_strict(samples: list[dict], stamp_ns: int,
                            max_gap_seconds: float) -> tuple[dict | None, str | None]:
    """Interpolate a pose only across adjacent valid, short-gap samples.

    Unlike a convenience interpolation routine that filters invalid samples,
    this preserves invalid endpoints as a hard boundary.  Thus an invalid
    mocap measurement cannot be silently bridged by two older/newer samples.
    """
    if stamp_ns <= 0:
        return None, 'invalid_target_timestamp'
    if not math.isfinite(max_gap_seconds) or max_gap_seconds <= 0:
        raise ValueError('max_gap_seconds must be finite and positive')
    ordered = sorted(samples, key=lambda item: int(item.get('stamp_ns', 0)))
    if len(ordered) < 2:
        return None, 'insufficient_mocap_samples'
    stamps = [int(item.get('stamp_ns', 0)) for item in ordered]
    if any(right <= left for left, right in zip(stamps, stamps[1:])):
        return None, 'nonmonotonic_or_duplicate_mocap_timestamps'
    index = bisect_left(stamps, int(stamp_ns))
    if index < len(ordered) and stamps[index] == stamp_ns:
        sample = ordered[index]
        if sample.get('valid', True) and finite_pose(sample):
            return _copy_pose(sample), None
        return None, 'invalid_mocap_sample_at_target'
    if index == 0 or index == len(ordered):
        return None, 'mocap_timestamp_outside_recording'
    left, right = ordered[index-1], ordered[index]
    if not left.get('valid', True) or not right.get('valid', True) or not finite_pose(left) or not finite_pose(right):
        return None, 'invalid_mocap_interpolation_endpoint'
    left_stamp, right_stamp = stamps[index-1], stamps[index]
    gap_s = (right_stamp-left_stamp) / 1e9
    if gap_s > max_gap_seconds:
        return None, 'mocap_interpolation_gap_too_large'
    ratio = (stamp_ns-left_stamp) / (right_stamp-left_stamp)
    qx, qy, qz, qw = slerp(left, right, ratio)
    return {
        'stamp_ns': int(stamp_ns), 'valid': True,
        'x': float(left['x']) + ratio*(float(right['x'])-float(left['x'])),
        'y': float(left['y']) + ratio*(float(right['y'])-float(left['y'])),
        'z': float(left['z']) + ratio*(float(right['z'])-float(left['z'])),
        'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw,
    }, None


@dataclass(frozen=True)
class Se2Alignment:
    """The fixed transform ``T_odom_mocap_world`` in the horizontal plane."""
    yaw_rad: float
    tx_m: float
    ty_m: float
    method: str = 'explicit_initial_static_window'

    def apply(self, pose: dict) -> dict:
        c, s = math.cos(self.yaw_rad), math.sin(self.yaw_rad)
        qx, qy, qz, qw = yaw_quaternion(wrap(yaw(pose)+self.yaw_rad))
        # SE(2) alignment purposefully has no vertical calibration term.  z
        # is retained for audit only and never used in planar accuracy metrics.
        return {
            'stamp_ns': int(pose['stamp_ns']), 'valid': bool(pose.get('valid', True)),
            'x': c*float(pose['x']) - s*float(pose['y']) + self.tx_m,
            'y': s*float(pose['x']) + c*float(pose['y']) + self.ty_m,
            'z': float(pose['z']), 'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw,
        }


def _static_spread(poses: list[dict]) -> dict:
    if not poses:
        return {'samples': 0, 'max_xy_radius_m': None, 'max_yaw_deviation_rad': None}
    xy = np.asarray([[p['x'], p['y']] for p in poses], dtype=float)
    center = np.mean(xy, axis=0)
    radii = np.linalg.norm(xy-center, axis=1)
    angles = np.asarray([yaw(p) for p in poses], dtype=float)
    mean_yaw = math.atan2(float(np.sin(angles).sum()), float(np.cos(angles).sum()))
    deviations = np.asarray([wrap(angle-mean_yaw) for angle in angles], dtype=float)
    return {
        'samples': len(poses), 'center_x_m': float(center[0]), 'center_y_m': float(center[1]),
        'mean_yaw_rad': mean_yaw, 'max_xy_radius_m': float(np.max(radii)),
        'max_yaw_deviation_rad': float(np.max(np.abs(deviations))),
    }


def estimate_initial_static_alignment(pairs: list[tuple[dict, dict]],
                                      max_static_motion_m: float,
                                      max_static_yaw_rad: float) -> tuple[Se2Alignment, dict]:
    """Estimate exactly one odom<-mocap SE(2) transform from static pairs.

    Pairs are ``(mocap_base_link, fastlio_odom_base_link)`` at matched times.
    Both source and target must satisfy caller-declared static tolerances;
    fitting a moving interval is rejected instead of disguising drift.
    """
    if len(pairs) < 3:
        raise ValueError('alignment_needs_at_least_three_time_matched_poses')
    if not math.isfinite(max_static_motion_m) or max_static_motion_m < 0:
        raise ValueError('max_static_motion_m must be finite and nonnegative')
    if not math.isfinite(max_static_yaw_rad) or max_static_yaw_rad < 0:
        raise ValueError('max_static_yaw_rad must be finite and nonnegative')
    mocap, lio = zip(*pairs)
    if not all(finite_pose(pose) for pose in (*mocap, *lio)):
        raise ValueError('invalid_alignment_pose')
    mocap_spread, lio_spread = _static_spread(list(mocap)), _static_spread(list(lio))
    for label, spread in (('mocap', mocap_spread), ('fastlio', lio_spread)):
        if spread['max_xy_radius_m'] > max_static_motion_m:
            raise ValueError(f'{label}_alignment_window_is_not_static_position')
        if spread['max_yaw_deviation_rad'] > max_static_yaw_rad:
            raise ValueError(f'{label}_alignment_window_is_not_static_yaw')
    deltas = [wrap(yaw(target)-yaw(source)) for source, target in pairs]
    theta = math.atan2(sum(math.sin(value) for value in deltas), sum(math.cos(value) for value in deltas))
    c, s = math.cos(theta), math.sin(theta)
    translations = [(target['x']-c*source['x']+s*source['y'],
                     target['y']-s*source['x']-c*source['y']) for source, target in pairs]
    alignment = Se2Alignment(theta, float(np.mean([item[0] for item in translations])),
                             float(np.mean([item[1] for item in translations])))
    return alignment, {'mocap': mocap_spread, 'fastlio': lio_spread}


def pose_error(reference: dict, estimate: dict) -> dict:
    """``estimate-reference`` error in the aligned odom plane."""
    ex, ey = float(estimate['x'])-float(reference['x']), float(estimate['y'])-float(reference['y'])
    return {'ex_m': ex, 'ey_m': ey, 'e_xy_m': math.hypot(ex, ey),
            'yaw_rad': wrap(yaw(estimate)-yaw(reference))}


def scalar_stats(values: list[float]) -> dict:
    values = np.asarray([value for value in values if value is not None and math.isfinite(float(value))], dtype=float)
    if not len(values):
        return {'samples': 0, 'mean': None, 'std': None, 'rmse': None,
                'p95_absolute': None, 'max_absolute': None}
    return {'samples': int(len(values)), 'mean': float(np.mean(values)),
            'std': float(np.std(values, ddof=1)) if len(values) > 1 else 0.0,
            'rmse': float(np.sqrt(np.mean(values**2))),
            'p95_absolute': float(np.percentile(np.abs(values), 95)),
            'max_absolute': float(np.max(np.abs(values)))}


@dataclass(frozen=True)
class EvaluationOptions:
    time_offset_seconds: float
    alignment_start_seconds: float
    alignment_window_seconds: float
    alignment_min_samples: int = 20
    max_interpolation_gap_seconds: float = .10
    max_adjacent_gap_seconds: float = .25
    alignment_max_static_motion_m: float = .02
    alignment_max_static_yaw_deg: float = 2.


def _validate_options(options: EvaluationOptions) -> None:
    finite = (options.time_offset_seconds, options.alignment_start_seconds,
              options.alignment_window_seconds, options.max_interpolation_gap_seconds,
              options.max_adjacent_gap_seconds, options.alignment_max_static_motion_m,
              options.alignment_max_static_yaw_deg)
    if not all(math.isfinite(value) for value in finite):
        raise ValueError('all evaluation options must be finite')
    if options.alignment_start_seconds < 0 or options.alignment_window_seconds <= 0:
        raise ValueError('alignment window start must be nonnegative and duration positive')
    if options.alignment_min_samples < 3 or options.max_interpolation_gap_seconds <= 0 or options.max_adjacent_gap_seconds <= 0:
        raise ValueError('alignment_min_samples >=3 and gap thresholds must be positive')
    if options.alignment_max_static_motion_m < 0 or options.alignment_max_static_yaw_deg < 0:
        raise ValueError('static tolerances must be nonnegative')


def _deduplicate_lio(samples: list[dict]) -> tuple[list[dict], dict]:
    """Refuse ambiguous TF samples, permit byte-for-byte equivalent repeats."""
    ordered = sorted(samples, key=lambda item: int(item.get('stamp_ns', 0)))
    result, exact_duplicates = [], 0
    for sample in ordered:
        if int(sample.get('stamp_ns', 0)) <= 0 or not sample.get('valid', True) or not finite_pose(sample):
            continue
        if result and int(sample['stamp_ns']) == int(result[-1]['stamp_ns']):
            old = result[-1]
            error = pose_error(old, sample)
            if error['e_xy_m'] > 1e-9 or abs(error['yaw_rad']) > 1e-9 or abs(float(old['z'])-float(sample['z'])) > 1e-9:
                raise ValueError('conflicting_odom_to_base_tf_at_same_timestamp')
            exact_duplicates += 1
            continue
        result.append(_copy_pose(sample))
    return result, {'input_samples': len(samples), 'usable_unique_samples': len(result),
                    'exact_duplicate_samples_ignored': exact_duplicates,
                    'invalid_or_zero_timestamp_samples_ignored': len(samples)-len(result)-exact_duplicates}


def _phase(stamp_ns: int, alignment_start_ns: int, alignment_end_ns: int) -> str:
    if stamp_ns < alignment_start_ns:
        return 'pre_alignment_window'
    if stamp_ns <= alignment_end_ns:
        return 'alignment_window'
    return 'formal'


def _error_summary(records: list[dict]) -> dict:
    usable = [record['error'] for record in records if record.get('phase') == 'formal' and record.get('error')]
    return {
        'samples': len(usable),
        'ex_m': scalar_stats([row['ex_m'] for row in usable]),
        'ey_m': scalar_stats([row['ey_m'] for row in usable]),
        'e_xy_m': scalar_stats([row['e_xy_m'] for row in usable]),
        'yaw_deg': scalar_stats([math.degrees(row['yaw_rad']) for row in usable]),
        'definition': 'FAST-LIO odom->base_link minus aligned mocap_world->Tracker0->base_link; scoring excludes the explicit alignment window.',
    }


def _adjacent_metrics(records: list[dict], max_gap_seconds: float) -> tuple[dict, list[dict]]:
    formal = [record for record in records if record.get('phase') == 'formal' and record.get('error')]
    values, rows, skipped = [], [], 0
    for first, second in zip(formal, formal[1:]):
        dt = (int(second['lio_stamp_ns'])-int(first['lio_stamp_ns'])) / 1e9
        if not 0 < dt <= max_gap_seconds:
            skipped += 1
            continue
        lio_a, lio_b = first['lio_pose'], second['lio_pose']
        ref_a, ref_b = first['aligned_mocap_pose'], second['aligned_mocap_pose']
        dlio = np.asarray([lio_b['x']-lio_a['x'], lio_b['y']-lio_a['y']], dtype=float)
        dref = np.asarray([ref_b['x']-ref_a['x'], ref_b['y']-ref_a['y']], dtype=float)
        difference = dlio-dref
        yaw_error = wrap(wrap(yaw(lio_b)-yaw(lio_a))-wrap(yaw(ref_b)-yaw(ref_a)))
        row = {'from_lio_stamp_ns': int(first['lio_stamp_ns']), 'to_lio_stamp_ns': int(second['lio_stamp_ns']),
               'dt_s': dt, 'lio_delta_xy_m': float(np.linalg.norm(dlio)),
               'mocap_delta_xy_m': float(np.linalg.norm(dref)),
               'delta_ex_m': float(difference[0]), 'delta_ey_m': float(difference[1]),
               'delta_e_xy_m': float(np.linalg.norm(difference)),
               'delta_yaw_rad': yaw_error}
        rows.append(row)
        values.append(row)
    return {
        'pairs': len(values), 'skipped_pairs_due_to_gap_or_order': skipped,
        'delta_ex_m': scalar_stats([row['delta_ex_m'] for row in values]),
        'delta_ey_m': scalar_stats([row['delta_ey_m'] for row in values]),
        'delta_e_xy_m': scalar_stats([row['delta_e_xy_m'] for row in values]),
        'delta_yaw_deg': scalar_stats([math.degrees(row['delta_yaw_rad']) for row in values]),
        'definition': 'Difference between consecutive global-frame planar displacements, not an absolute-position metric.',
    }, rows


def _trajectory_summary(records: list[dict], max_gap_seconds: float) -> dict:
    formal = [record for record in records if record.get('phase') == 'formal' and record.get('error')]
    if len(formal) < 2:
        return {'samples': len(formal), 'available': False}
    lio_length = mocap_length = 0.
    usable_segments = 0
    for first, second in zip(formal, formal[1:]):
        dt = (int(second['lio_stamp_ns'])-int(first['lio_stamp_ns'])) / 1e9
        if not 0 < dt <= max_gap_seconds:
            continue
        a, b = first['lio_pose'], second['lio_pose']
        ma, mb = first['aligned_mocap_pose'], second['aligned_mocap_pose']
        lio_length += math.hypot(b['x']-a['x'], b['y']-a['y'])
        mocap_length += math.hypot(mb['x']-ma['x'], mb['y']-ma['y'])
        usable_segments += 1
    first, last = formal[0], formal[-1]
    return {
        'samples': len(formal), 'available': True, 'usable_adjacent_segments': usable_segments,
        'fastlio_planar_path_length_m': lio_length, 'aligned_mocap_planar_path_length_m': mocap_length,
        'fastlio_end_to_end_displacement_m': math.hypot(last['lio_pose']['x']-first['lio_pose']['x'], last['lio_pose']['y']-first['lio_pose']['y']),
        'aligned_mocap_end_to_end_displacement_m': math.hypot(last['aligned_mocap_pose']['x']-first['aligned_mocap_pose']['x'], last['aligned_mocap_pose']['y']-first['aligned_mocap_pose']['y']),
        'terminal_error': last['error'],
        'definition': 'Path lengths sum only adjacent valid matched samples within max_adjacent_gap_seconds; terminal error is the final formal matched sample.',
    }


def evaluate_trajectory(lio_samples: list[dict], mocap_base_samples: list[dict],
                        options: EvaluationOptions) -> tuple[list[dict], dict, list[dict]]:
    """Evaluate FAST-LIO and mocap paths without any online side effects."""
    _validate_options(options)
    lio, lio_input = _deduplicate_lio(lio_samples)
    if len(lio) < 3:
        raise ValueError('need_at_least_three_valid_odom_to_base_link_tf_samples')
    mocap = sorted(mocap_base_samples, key=lambda item: int(item.get('stamp_ns', 0)))
    if len(mocap) < 2:
        raise ValueError('need_at_least_two_mocap_samples')
    origin_ns = int(lio[0]['stamp_ns'])
    alignment_start_ns = origin_ns + round(options.alignment_start_seconds*1e9)
    alignment_end_ns = alignment_start_ns + round(options.alignment_window_seconds*1e9)
    offset_ns = round(options.time_offset_seconds*1e9)
    records = []
    for sample in lio:
        target = int(sample['stamp_ns']) + offset_ns
        matched, reason = interpolate_pose_strict(mocap, target, options.max_interpolation_gap_seconds)
        record = {
            'lio_stamp_ns': int(sample['stamp_ns']), 'mocap_target_stamp_ns': target,
            'time_offset_seconds': options.time_offset_seconds,
            'phase': _phase(int(sample['stamp_ns']), alignment_start_ns, alignment_end_ns),
            'lio_pose': sample, 'mocap_base_pose': matched, 'match_reason': reason,
            'aligned_mocap_pose': None, 'error': None,
        }
        records.append(record)
    alignment_records = [record for record in records
                         if record['phase'] == 'alignment_window' and record['mocap_base_pose'] is not None]
    if len(alignment_records) < options.alignment_min_samples:
        raise ValueError(f'alignment_has_{len(alignment_records)}_matched_samples_need_{options.alignment_min_samples}')
    alignment, static_spread = estimate_initial_static_alignment(
        [(record['mocap_base_pose'], record['lio_pose']) for record in alignment_records],
        options.alignment_max_static_motion_m, math.radians(options.alignment_max_static_yaw_deg))
    match_failures: dict[str, int] = {}
    for record in records:
        if record['mocap_base_pose'] is None:
            reason = record['match_reason'] or 'unknown_mocap_match_failure'
            match_failures[reason] = match_failures.get(reason, 0)+1
            continue
        aligned = alignment.apply(record['mocap_base_pose'])
        record['aligned_mocap_pose'] = aligned
        record['error'] = pose_error(aligned, record['lio_pose'])
    adjacent, adjacent_rows = _adjacent_metrics(records, options.max_adjacent_gap_seconds)
    formal = [record for record in records if record['phase'] == 'formal']
    formal_matched = [record for record in formal if record['error']]
    summary = {
        'schema_version': 1,
        'method': {
            'coordinate_chain': 'mocap_world->Tracker0 then explicit T_Tracker0_base_link; one fixed planar T_odom_mocap_world; compared with recorded odom->base_link',
            'time_matching': 'For each FAST-LIO TF time t, mocap is interpolated at t + time_offset_seconds. Positive offset means the corresponding mocap header timestamp is later.',
            'time_offset_was_user_provided': True,
            'time_offset_auto_fit_performed': False,
            'alignment_scope': 'One fixed SE(2) alignment from the explicit initial static window only; it is not refit on the formal trajectory and no scale is fitted.',
        },
        'options': asdict(options),
        'time': {'lio_first_stamp_ns': origin_ns, 'alignment_start_stamp_ns': alignment_start_ns,
                 'alignment_end_stamp_ns': alignment_end_ns},
        'input': {'fastlio_tf': lio_input, 'mocap_base_input_samples': len(mocap)},
        'matching': {'lio_samples': len(records), 'matched_samples': sum(r['error'] is not None for r in records),
                     'overall_coverage': sum(r['error'] is not None for r in records)/len(records),
                     'formal_samples': len(formal), 'formal_matched_samples': len(formal_matched),
                     'formal_coverage': len(formal_matched)/len(formal) if formal else None,
                     'failures': match_failures},
        'alignment': {**asdict(alignment), 'samples': len(alignment_records),
                      'static_spread': static_spread,
                      'excluded_from_metrics': True},
        'error_metrics_formal': _error_summary(records),
        'adjacent_motion_error_formal': adjacent,
        'trajectory_formal': _trajectory_summary(records, options.max_adjacent_gap_seconds),
        'limitations': [
            'This is start-aligned relative tracking error. The fixed initial odom<->mocap offset is intentionally not evaluated.',
            'time_offset_seconds is supplied by the operator and is never optimized from this trajectory.',
            'Mocap header timestamps must be understood before dynamic absolute-time conclusions; this evaluator only applies the stated offset.',
            'SE(2) alignment evaluates planar x/y/yaw only. It does not claim a calibrated vertical/world-frame transform.',
            'A matched mocap sample is a reference observation, not an online feedback input; this offline tool does not modify FAST-LIO, NDT, TF, or navigation.',
        ],
    }
    return records, summary, adjacent_rows

