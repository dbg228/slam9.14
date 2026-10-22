#!/usr/bin/env python3
"""Offline static analysis after the first accepted NDT and a fixed settling count.

All initialization/settling records are retained. No ROS nodes are started and
no localization settings are changed. Mocap alignment uses ONLY the excluded
settling window, shared by initial/raw/final poses. Large updates are events,
not automatically ground-truth errors.
"""
from __future__ import annotations

import argparse
from bisect import bisect_left
from collections import Counter
from dataclasses import asdict, dataclass
import csv
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from ndt_match_evaluation import (
    apply_tracker_to_base, json_clean, load_json_input, load_rosbag,
    tracker_to_base_from_config,
)
from ndt_match_evaluation_core import (
    Se2Alignment, interpolate_pose, normalized_quaternion, pose_error,
    rigid_body_to_base_pose, rotate_vector, wrap, yaw, yaw_quaternion,
)

STAGES = ('initial', 'raw', 'final', 'map_odom')
PASSIVE = {'waiting_for_initialization', 'no_input_cloud', 'source_below_minimum',
           'submap_below_minimum', 'no_odom_pose_at_cloud_time'}
REGISTRATION = {'accepted', 'ndt_exception', 'ndt_not_converged', 'invalid_fitness',
                'fitness_above_limit', 'correction_translation_above_limit',
                'correction_yaw_above_limit'}
POSE_KEYS = ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')


@dataclass
class Options:
    settling_matches: int = 3
    max_interpolation_gap_seconds: float = .10
    max_adjacent_gap_seconds: float = 5.
    time_offset_seconds: float = 0.
    static_position_tolerance_m: float = .01
    static_yaw_tolerance_deg: float = 1.


def valid_pose(p):
    try:
        return p is not None and all(math.isfinite(float(p[k])) for k in POSE_KEYS) and np.linalg.norm(normalized_quaternion(p)) > 0
    except (KeyError, TypeError, ValueError):
        return False


def compose(a, b):
    return rigid_body_to_base_pose(a, [b[k] for k in ('x', 'y', 'z')], normalized_quaternion(b))


def inverse(p):
    x, y, z, w = normalized_quaternion(p)
    q = (-x, -y, -z, w)
    t = rotate_vector(q, [-p[k] for k in ('x', 'y', 'z')])
    return dict(zip(POSE_KEYS, (*t, *q)))


def relative(a, b):
    return compose(inverse(a), b)


def magnitude(p):
    return math.hypot(p['x'], p['y'])


def vector(p):
    return np.array([p[k] for k in ('x', 'y', 'z')], dtype=float)


def stats(values):
    values = np.array([v for v in values if v is not None and math.isfinite(v)], dtype=float)
    if not len(values):
        return {'samples': 0, 'rmse': None, 'p95_absolute': None, 'max_absolute': None, 'mean': None}
    return {'samples': len(values), 'mean': float(values.mean()), 'rmse': float(np.sqrt(np.mean(values**2))),
            'p95_absolute': float(np.percentile(np.abs(values), 95)), 'max_absolute': float(np.max(np.abs(values)))}


def phases(diagnostics, settling_matches):
    if settling_matches < 2:
        raise ValueError('At least two excluded settling registrations are required for the independent alignment window')
    # Match IDs are assigned in callback order, including invalid cloud stamps.
    records = sorted(diagnostics, key=lambda d: int(d['match_id']))
    ids = [int(d['match_id']) for d in records]
    if len(ids) != len(set(ids)):
        raise ValueError('Duplicate/reset match_id: evaluate one node session per bag')
    unknown = set(d['stage'] for d in records) - PASSIVE - REGISTRATION
    if unknown:
        raise ValueError('Unknown diagnostic stages: '+str(sorted(unknown)))
    first = next((i for i, d in enumerate(records)
                  if d['stage'] in REGISTRATION and d.get('accepted') and d.get('converged')), None)
    if first is None:
        raise ValueError('No successful accepted NDT initialization in this recording')
    # Prevent an arbitrary mid-run bag from being called initialization.
    if not any(d['stage'] == 'waiting_for_initialization' for d in records[:first]):
        raise ValueError('Initialization not observed: record waiting_for_initialization before the first accepted NDT')
    if any(d['stage'] == 'waiting_for_initialization' for d in records[first+1:]):
        raise ValueError('A second initialization/session appears in this bag; split it first')
    result, skipped = [], 0
    for i, d in enumerate(records):
        if i < first:
            phase = 'pre_initialization'
        elif i == first:
            phase = 'initialization_complete'
        elif d['stage'] not in REGISTRATION:
            phase = 'passive_after_initialization'
        elif skipped < settling_matches:
            phase = 'settling'
            skipped += 1
        else:
            phase = 'formal'
        result.append({**d, 'phase': phase})
    if skipped < settling_matches or not any(d['phase'] == 'formal' for d in result):
        raise ValueError('Not enough registrations after initialization and the fixed settling period')
    return result


def state_poses(d):
    a = d.get('initial_map_odom_pose')
    b = d.get('raw_map_odom_pose')
    a = a if valid_pose(a) else None
    b = b if valid_pose(b) else None
    # Exact node semantics: rejection keeps A; acceptance installs B.
    return a, b, b if d.get('accepted') else a


def center_pose(poses):
    theta = math.atan2(sum(math.sin(yaw(p)) for p in poses), sum(math.cos(yaw(p)) for p in poses))
    return dict(zip(POSE_KEYS, (*np.mean([vector(p) for p in poses], axis=0), *yaw_quaternion(theta))))


def pose_spread(poses):
    if not poses:
        return {'samples': 0}
    c = center_pose(poses)
    errors = [pose_error(c, p) for p in poses]
    return {'samples': len(poses), 'xy_about_centroid_m': stats([e['xy_m'] for e in errors]),
            'yaw_about_circular_mean_deg': stats([math.degrees(e['yaw_rad']) for e in errors]),
            'xyz_peak_to_peak_m': np.ptp([vector(p) for p in poses], axis=0).tolist(),
            'definition': 'Descriptive centered scatter only; not a separate reference alignment or absolute accuracy'}


def match_reference(samples, stamp, max_gap):
    """Use adjacent samples without interpolating across explicitly invalid data."""
    if stamp <= 0:
        return None, 'invalid_target_timestamp'
    ordered = sorted(samples, key=lambda p: int(p['stamp_ns']))
    times = [int(p['stamp_ns']) for p in ordered]
    index = bisect_left(times, stamp)
    if index < len(times) and times[index] == stamp:
        p = ordered[index]
        if p.get('valid', True) and valid_pose(p):
            return dict(p), None
        return None, 'invalid_mocap_sample'
    if index == 0 or index == len(times):
        return None, 'mocap_timestamp_outside_recording'
    pair = ordered[index-1:index+1]
    if any(not p.get('valid', True) or not valid_pose(p) for p in pair):
        return None, 'invalid_mocap_interpolation_endpoint'
    return interpolate_pose(pair, stamp, max_gap)


def establish_alignment(records, mocap, options):
    pairs = []
    for d in records:
        if d['phase'] != 'settling':
            continue
        stamp = int(d['stamp_ns']) + round(options.time_offset_seconds*1e9)
        reference, reason = match_reference(mocap, stamp, options.max_interpolation_gap_seconds)
        p = d.get('final_pose')
        if reference is None or not valid_pose(reference) or not d.get('final_valid') or not valid_pose(p):
            raise ValueError(f"Settling match {d['match_id']} lacks valid synchronized final/base mocap: {reason}; do not borrow formal samples")
        pairs.append((reference, p))
    angles = [wrap(yaw(p)-yaw(r)) for r, p in pairs]
    theta = math.atan2(sum(math.sin(a) for a in angles), sum(math.cos(a) for a in angles))
    c, s = math.cos(theta), math.sin(theta)
    xy = [[p['x']-c*r['x']+s*r['y'], p['y']-s*r['x']-c*r['y']] for r, p in pairs]
    tx, ty = np.mean(xy, axis=0)
    return Se2Alignment(theta, float(tx), float(ty), 0., 'excluded_settling_window'), pairs


def stage_summary(records, stage):
    usable = [r for r in records if r.get(stage+'_error') is not None]
    return {'samples': len(usable),
            'xy_m': stats([r[stage+'_error']['xy_m'] for r in usable]),
            'yaw_deg': stats([math.degrees(r[stage+'_error']['yaw_rad']) for r in usable]),
            'ex_m': stats([r[stage+'_error']['x_m'] for r in usable]),
            'ey_m': stats([r[stage+'_error']['y_m'] for r in usable])}


def adjacent_summary(records, stage, max_gap):
    xy, heading, reference_xy, ref_heading, residual_xy, residual_yaw = [], [], [], [], [], []
    skipped = 0
    for a, b in zip(records, records[1:]):
        dt = (int(b['stamp_ns'])-int(a['stamp_ns']))/1e9
        pa, pb = a.get(stage+'_pose'), b.get(stage+'_pose')
        if (not 0 < dt <= max_gap or not valid_pose(pa) or not valid_pose(pb) or
                (stage != 'map_odom' and (not a.get(stage+'_valid') or not b.get(stage+'_valid')))):
            skipped += 1
            continue  # Do not bridge an invalid intermediate registration.
        delta = vector(pb)-vector(pa)
        dyaw = wrap(yaw(pb)-yaw(pa))
        xy.append(float(np.linalg.norm(delta[:2])))
        heading.append(math.degrees(dyaw))
        ra, rb = a.get('reference_pose'), b.get('reference_pose')
        if stage != 'map_odom' and valid_pose(ra) and valid_pose(rb):
            dr = vector(rb)-vector(ra)
            dryaw = wrap(yaw(rb)-yaw(ra))
            reference_xy.append(float(np.linalg.norm(dr[:2])))
            ref_heading.append(math.degrees(dryaw))
            residual_xy.append(float(np.linalg.norm((delta-dr)[:2])))
            residual_yaw.append(math.degrees(wrap(dyaw-dryaw)))
    return {'delta_xy_m': stats(xy), 'delta_yaw_deg': stats(heading), 'skipped_pairs': skipped,
            'reference_delta_xy_m': stats(reference_xy), 'reference_delta_yaw_deg': stats(ref_heading),
            'delta_xy_error_m': stats(residual_xy), 'delta_yaw_error_deg': stats(residual_yaw)}


def effect_summary(records):
    pairs = [r for r in records if r.get('delta_e_m') is not None]
    improving = [r for r in pairs if r['delta_e_m'] < -1e-9]
    worsening = [r for r in pairs if r['delta_e_m'] > 1e-9]
    worst = max(worsening, key=lambda r: r['delta_e_m'], default=None)
    largest = max(worsening, key=lambda r: r.get('base_candidate_update_xy_m') or 0., default=None)
    return {'paired_frames': len(pairs), 'improved_count': len(improving), 'worsened_count': len(worsening),
            'neutral_count': len(pairs)-len(improving)-len(worsening),
            'improved_ratio': len(improving)/len(pairs) if pairs else None,
            'worsened_ratio': len(worsening)/len(pairs) if pairs else None,
            'mean_improvement_m': float(np.mean([-r['delta_e_m'] for r in improving])) if improving else None,
            'mean_worsening_m': float(np.mean([r['delta_e_m'] for r in worsening])) if worsening else None,
            'delta_e_m': stats([r['delta_e_m'] for r in pairs]),
            'max_worsening_match_id': worst['match_id'] if worst else None,
            'max_worsening_m': worst['delta_e_m'] if worst else None,
            'largest_base_update_among_worsening_frames_m': largest['base_candidate_update_xy_m'] if largest else None,
            'largest_base_update_among_worsening_frames_match_id': largest['match_id'] if largest else None,
            'interpretation': 'e_raw - e_initial against the shared excluded-settling alignment; improvement/worsening is conditional on that reference, not absolute truth'}


def update_summary(records, key):
    usable = [r for r in records if r.get(key) is not None]
    return {'magnitude_m': stats([r[key] for r in usable]),
            'thresholds': {str(mm): {'count': sum(r[key] > mm/1000 for r in usable),
                                    'denominator': len(usable),
                                    'ratio': sum(r[key] > mm/1000 for r in usable)/len(usable) if usable else None,
                                    'match_ids': [r['match_id'] for r in usable if r[key] > mm/1000]}
                           for mm in (10, 20, 30)},
            'unknown_count': len(records)-len(usable), 'comparison': 'strictly greater than threshold'}


def propagation(records, max_gap):
    output = []
    for a, b in zip(records, records[1:]):
        dt = (int(b['stamp_ns'])-int(a['stamp_ns']))/1e9
        required = (a.get('initial_map_odom_pose'), a.get('map_odom_pose'),
                    b.get('initial_map_odom_pose'), a.get('initial_pose'), b.get('initial_pose'))
        if not 0 < dt <= max_gap or not all(valid_pose(p) for p in required):
            continue
        pre, post, next_pre, initial, next_initial = required
        next_odom = compose(inverse(next_pre), next_initial)
        predicted = compose(post, next_odom)
        without_update = compose(pre, next_odom)
        ndt_part = vector(predicted)-vector(without_update)
        lio_part = vector(without_update)-vector(initial)
        observed = vector(next_initial)-vector(initial)
        mismatch = relative(post, next_pre)
        q = normalized_quaternion(mismatch)
        row = {'from_match_id': a['match_id'], 'to_match_id': b['match_id'], 'dt_s': dt,
               'accepted': bool(a.get('accepted')), 'phase': a['phase'],
               'map_state_inheritance_position_difference_m': float(np.linalg.norm(vector(mismatch))),
               'map_state_inheritance_rotation_difference_deg': math.degrees(2*math.acos(min(1., abs(float(q[3]))))),
               'closure_xy_m': float(np.linalg.norm((observed-ndt_part-lio_part)[:2])),
               'ndt_contribution_to_next_init_xy_m': float(np.linalg.norm(ndt_part[:2])),
               'lio_contribution_to_next_init_xy_m': float(np.linalg.norm(lio_part[:2])),
               'observed_next_init_delta_xy_m': float(np.linalg.norm(observed[:2])),
               'current_delta_e_m': a.get('delta_e_m'),
               'next_initial_relative_error_m': b['initial_error']['xy_m'] if b.get('initial_error') else None}
        for label, v in [('ndt', ndt_part), ('lio', lio_part), ('observed', observed)]:
            row.update({label+'_delta_'+axis+'_m': float(v[i]) for i, axis in enumerate(('x', 'y', 'z'))})
        output.append(row)
    return output


def event_persistence(records):
    """Cumulative state displacement after an event, including later updates."""
    output = []
    for i, event in enumerate(records):
        if (event.get('map_applied_update_xy_m') or 0.) <= .03:
            continue
        pre, post = event.get('initial_map_odom_pose'), event.get('map_odom_pose')
        if not valid_pose(pre) or not valid_pose(post):
            continue
        for future in records[i+1:]:
            odom = future.get('odom_base_reconstructed_pose')
            initial = future.get('initial_pose')
            if not valid_pose(odom) or not valid_pose(initial):
                continue
            before = vector(compose(pre, odom))
            first_change = (vector(compose(post, odom))-before)[:2]
            net_change = (vector(initial)-before)[:2]
            length2 = float(np.dot(first_change, first_change))
            output.append({'event_match_id': event['match_id'], 'future_match_id': future['match_id'],
                           'cumulative_init_displacement_from_pre_event_m': float(np.linalg.norm(net_change)),
                           'projection_on_event_update_fraction': float(np.dot(net_change, first_change)/length2) if length2 > 1e-18 else None,
                           'includes_intervening_ndt_updates': True})
    return output


def analyze(diagnostics, mocap_base, options=Options()):
    records = phases(diagnostics, options.settling_matches)
    alignment, alignment_pairs = establish_alignment(records, mocap_base, options)
    settling_states = [state_poses(d)[2] for d in records if d['phase'] == 'settling']
    if not all(valid_pose(p) for p in settling_states):
        raise ValueError('Settling map->odom states missing; re-read the original bag with the updated parser')
    map_center = center_pose(settling_states)
    seen_stamps = set()
    for d in records:
        a, b, final_state = state_poses(d)
        d['map_odom_pose'] = final_state
        d['map_odom_reference_pose'] = map_center
        d['map_odom_error'] = pose_error(map_center, final_state) if valid_pose(final_state) else None
        stamp = int(d['stamp_ns'])
        ref, reason = match_reference(mocap_base, stamp+round(options.time_offset_seconds*1e9), options.max_interpolation_gap_seconds)
        d['reference_pose'] = alignment.apply(ref) if valid_pose(ref) else None
        d['reference_reason'] = '' if valid_pose(ref) else (reason or 'invalid_mocap_pose')
        d['duplicate_measurement'] = stamp in seen_stamps and d['phase'] == 'formal'
        if d['stage'] in REGISTRATION:
            seen_stamps.add(stamp)
        for stage in ('initial', 'raw', 'final'):
            p = d.get(stage+'_pose')
            d[stage+'_error'] = (pose_error(d['reference_pose'], p)
                                if d.get(stage+'_valid') and valid_pose(p) and d['reference_pose'] else None)
        d['delta_e_m'] = (d['raw_error']['xy_m']-d['initial_error']['xy_m']
                          if d['raw_error'] is not None and d['initial_error'] is not None else None)
        d['base_candidate_update_xy_m'] = (float(np.linalg.norm((vector(d['raw_pose'])-vector(d['initial_pose']))[:2]))
                                          if d.get('initial_valid') and d.get('raw_valid') and valid_pose(d.get('initial_pose')) and valid_pose(d.get('raw_pose')) else None)
        d['map_candidate_update_xy_m'] = magnitude(relative(a, b)) if valid_pose(a) and valid_pose(b) else None
        d['map_applied_update_xy_m'] = magnitude(relative(a, final_state)) if valid_pose(a) and valid_pose(final_state) else None
        d['map_applied_update_yaw_rad'] = yaw(relative(a, final_state)) if valid_pose(a) and valid_pose(final_state) else None
        d['odom_base_reconstructed_pose'] = (compose(inverse(a), d['initial_pose'])
                                              if valid_pose(a) and valid_pose(d.get('initial_pose')) else None)
    formal = [d for d in records if d['phase'] == 'formal' and not d['duplicate_measurement']]
    if not formal:
        raise ValueError('No new input-cloud measurements remain in the formal phase')
    accepted = [d for d in formal if d.get('accepted')]
    initial = next(d for d in records if d['phase'] == 'initialization_complete')
    lo, hi = min(d['stamp_ns'] for d in formal), max(d['stamp_ns'] for d in formal)
    mocap_static = pose_spread([p for p in mocap_base if lo+round(options.time_offset_seconds*1e9) <= p['stamp_ns'] <= hi+round(options.time_offset_seconds*1e9) and p.get('valid', True) and valid_pose(p)])
    settling_reference_spread = pose_spread([p[0] for p in alignment_pairs])
    settling_pose_spread = pose_spread([p[1] for p in alignment_pairs])
    warnings = []
    if settling_pose_spread['xy_about_centroid_m']['max_absolute'] > options.static_position_tolerance_m:
        warnings.append('被排除的稳定期 NDT 位姿仍明显变化；固定跳过次数不保证已稳定收敛。相对 RMSE 和 Δe 依赖该变化中的对齐窗口，正式段按预定规则完整保留。')
    for label, spread in [('settling mocap', settling_reference_spread), ('formal mocap', mocap_static)]:
        if not spread['samples']:
            warnings.append(label+' unavailable; static condition unverified')
        elif (spread['xy_about_centroid_m']['max_absolute'] > options.static_position_tolerance_m or
              spread['yaw_about_circular_mean_deg']['max_absolute'] > options.static_yaw_tolerance_deg):
            warnings.append(label+' exceeds static tolerance; do not interpret as a stationary experiment')
    transitions = propagation(formal, options.max_adjacent_gap_seconds)
    discontinuities = [t for t in transitions if t['map_state_inheritance_position_difference_m'] > 1e-5 or
                       t['map_state_inheritance_rotation_difference_deg'] > .01]
    if discontinuities:
        warnings.append('存在前次最终 map→odom 与下次初始状态不连续的记录；需要排查额外人工初值或其他状态写入。')
    summary = {
        'schema_version': 1, 'options': asdict(options), 'warnings': warnings,
        'phase_counts': dict(Counter(d['phase'] for d in records)),
        'initialization': {'match_id': initial['match_id'], 'base_correction_m': initial['base_candidate_update_xy_m'],
                           'interpretation': 'Correction from a manual coarse seed; excluded from stability accuracy metrics. Its absolute correctness is not determined by this experiment.'},
        'settling_match_ids': [d['match_id'] for d in records if d['phase'] == 'settling'],
        'formal_match_ids': [d['match_id'] for d in formal], 'formal_duration_seconds': (hi-lo)/1e9,
        'duplicate_measurements_excluded': sum(d['duplicate_measurement'] for d in records),
        'formal_reference_coverage': sum(d['reference_pose'] is not None for d in formal)/len(formal),
        'formal_reference_failures': dict(Counter(d['reference_reason'] for d in formal if d['reference_pose'] is None)),
        'formal_accepted': len(accepted), 'formal_rejected': len(formal)-len(accepted),
        'formal_unconverged': sum(not d.get('converged') for d in formal),
        'alignment': {**asdict(alignment), 'match_ids': [d['match_id'] for d in records if d['phase'] == 'settling'],
                      'scope': 'One fixed planar transform from excluded settling pairs only. Shared by T_init/T_ndt/T_final. No scale fit or formal-trajectory refit.'},
        'map_odom_reference': {'pose': map_center, 'definition': 'Variation about excluded settling map->odom state center; not a mocap ground-truth error'},
        'final_pose_definition': ('Localizer-internal final base_link state at the cloud timestamp, '
                                  'not the publication-time pose. The odom TF bridge planarizes raw sensor roll/pitch, '
                                  'whereas the internal localizer uses full 3D extrinsics; do not call them exactly identical.'),
        'static_verification': {'mocap_formal': mocap_static, 'mocap_settling': settling_reference_spread,
                                'ndt_final_settling': settling_pose_spread,
                                'lio_reconstructed_formal': pose_spread([d['odom_base_reconstructed_pose'] for d in formal if valid_pose(d['odom_base_reconstructed_pose'])]),
                                'lio_note': 'O = inverse(pre-NDT map->odom) * T_init, using the same recorded cloud-time odometry; algebraic reconstruction, not an independent sensor.'},
        'all_formal': {stage: stage_summary(formal, stage) for stage in STAGES},
        'accepted_formal': {stage: stage_summary(accepted, stage) for stage in STAGES},
        'adjacent_frames': {stage: adjacent_summary(formal, stage, options.max_adjacent_gap_seconds) for stage in STAGES},
        'ndt_net_effect_all': effect_summary(formal), 'ndt_net_effect_accepted': effect_summary(accepted),
        'updates': {key: update_summary(formal, key) for key in
                    ('map_candidate_update_xy_m', 'map_applied_update_xy_m', 'base_candidate_update_xy_m')},
        'propagation': {'pairs': len(transitions),
                        'state_discontinuity_match_ids': [t['to_match_id'] for t in discontinuities],
                        'map_state_inheritance_position_difference_m': stats([r['map_state_inheritance_position_difference_m'] for r in transitions]),
                        'ndt_contribution_to_next_init_xy_m': stats([r['ndt_contribution_to_next_init_xy_m'] for r in transitions]),
                        'lio_contribution_to_next_init_xy_m': stats([r['lio_contribution_to_next_init_xy_m'] for r in transitions]),
                        'closure_xy_m': stats([r['closure_xy_m'] for r in transitions]),
                        'definition': 'T_init_next = M_post_current * O_next when state is inherited. Contributions use full SE(3); report vector components because scalar norms cannot be added.'},
        'formal_ndt_align_duration_s': stats([d.get('ndt_align_duration_s') for d in formal]),
        'large_update_persistence': event_persistence(formal),
        'ndt_configuration_observed': {key: sorted({d[key] for d in formal if d.get(key) is not None}) for key in
                                       ('configured_ndt_max_iterations', 'configured_ndt_transformation_epsilon',
                                        'configured_ndt_step_size', 'configured_ndt_resolution')},
        'interpretation': 'Relative stability after initialization. Large NDT updates are not necessarily incorrect. Net improvement is conditional on the fixed settling alignment; no absolute accuracy or guaranteed local-minimum diagnosis.'}
    return records, summary, transitions


def write_csv(path, rows):
    flat = []
    def expand(prefix, value, row):
        if isinstance(value, dict):
            for key, child in value.items():
                expand(prefix+'_'+key if prefix else key, child, row)
        elif isinstance(value, (list, tuple)):
            row[prefix] = json.dumps(json_clean(value))
        else:
            row[prefix] = json_clean(value)
    for value in rows:
        row = {}
        expand('', value, row)
        flat.append(row)
    with Path(path).open('w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=sorted({k for r in flat for k in r}))
        writer.writeheader()
        writer.writerows(flat)


def event_context(records, half_window=3):
    formal = [d for d in records if d['phase'] == 'formal']
    events = [d for d in formal if (d.get('map_applied_update_xy_m') or 0) > .03]
    rows = []
    for e in events:
        index = next(i for i, r in enumerate(records) if r['match_id'] == e['match_id'])
        for i in range(max(0, index-half_window), min(len(records), index+half_window+1)):
            rows.append({**records[i], 'event_match_id': e['match_id'], 'offset_from_event': i-index})
    return rows


def plots(output, records, transitions, max_gap):
    formal = [r for r in records if r['phase'] == 'formal' and not r['duplicate_measurement']]
    origin = formal[0]['stamp_ns']
    time = [(r['stamp_ns']-origin)/1e9 for r in formal]
    colors = dict(zip(STAGES, ('tab:red', 'tab:orange', 'tab:blue', 'tab:purple')))
    for ext in ('png', 'svg'):
        fig, axes = plt.subplots(5, 1, figsize=(12, 14), sharex=True, constrained_layout=True)
        for stage in ('initial', 'raw', 'final'):
            axes[0].plot(time, [1000*r[stage+'_error']['xy_m'] if r[stage+'_error'] else math.nan for r in formal], '.-', label=stage, color=colors[stage])
            axes[1].plot(time, [math.degrees(r[stage+'_error']['yaw_rad']) if r[stage+'_error'] else math.nan for r in formal], '.-', label=stage, color=colors[stage])
        axes[2].plot(time, [1000*r['map_applied_update_xy_m'] if r['map_applied_update_xy_m'] is not None else math.nan for r in formal], '.-', label='Applied map->odom update')
        largest = max((r for r in formal if r['map_applied_update_xy_m'] is not None), key=lambda r: r['map_applied_update_xy_m'], default=None)
        if largest:
            axes[2].annotate('match '+str(largest['match_id']),
                             ((largest['stamp_ns']-origin)/1e9, largest['map_applied_update_xy_m']*1000),
                             xytext=(12, -12), textcoords='offset points')
        for mm in (10, 20, 30):
            axes[2].axhline(mm, color='grey', alpha=.4, linestyle='--')
        axes[3].plot(time, [r['fitness'] if r.get('fitness_valid') else math.nan for r in formal], '.-', label='Fitness (lower better)')
        axes[4].plot(time, [r['iterations'] for r in formal], '.-', label='PCL iterations')
        axes[4].step(time, [int(r['accepted']) for r in formal], where='mid', label='Accepted')
        axes[4].step(time, [int(r['converged']) for r in formal], where='mid', linestyle=':', label='Converged')
        for ax, label in zip(axes, ('Relative XY (mm)', 'Relative yaw (deg)', 'Update (mm)', 'Fitness', 'Count / state')):
            ax.set_ylabel(label); ax.grid(alpha=.3); ax.legend(loc='best')
        axes[-1].set_xlabel('Seconds from formal stage start')
        fig.suptitle('Post-initialization static test; fixed excluded-settling alignment')
        fig.savefig(output/('post_init_timeseries.'+ext), dpi=170); plt.close(fig)
        fig, axes = plt.subplots(2, 2, figsize=(12, 9), constrained_layout=True)
        for stage in ('initial', 'raw', 'final'):
            selected = [r[stage+'_error'] for r in formal if r[stage+'_error']]
            axes[0, 0].scatter([e['x_m']*1000 for e in selected], [e['y_m']*1000 for e in selected], label=stage, s=22)
        axes[0, 0].set(xlabel='ex (mm)', ylabel='ey (mm)'); axes[0, 0].legend()
        effects = [1000*r['delta_e_m'] if r['delta_e_m'] is not None else math.nan for r in formal]
        axes[0, 1].bar(time, effects, color=['tab:red' if e > 0 else 'tab:green' for e in effects])
        axes[0, 1].set(xlabel='Time (s)', ylabel='e_ndt - e_init (mm)')
        for axis, stage in zip(axes[1], ('map_odom', 'final')):
            for key in ('x', 'y'):
                axis.plot(time, [1000*r[stage+'_error'][key+'_m'] if r[stage+'_error'] else math.nan for r in formal], '.-', label=key)
            axis.set(xlabel='Time (s)', ylabel=stage+' change from reference (mm)'); axis.legend()
        for ax in axes.flat:
            ax.grid(alpha=.3)
        fig.suptitle('Net effect is conditional on the shared alignment; large update != absolute error')
        fig.savefig(output/('update_effects.'+ext), dpi=170); plt.close(fig)
        fig, axes = plt.subplots(4, 1, figsize=(12, 12), constrained_layout=True)
        ids = [r['match_id'] for r in formal]
        axes[0].plot(ids, [1000*r['map_applied_update_xy_m'] if r['map_applied_update_xy_m'] is not None else math.nan for r in formal], '.-', label='Applied map->odom update')
        axes[0].set_yscale('symlog', linthresh=1.)
        for threshold in (10, 20, 30):
            axes[0].axhline(threshold, color='gray', linestyle='--', alpha=.4)
        for stage in STAGES:
            dx, dh = [], []
            for a, b in zip(formal, formal[1:]):
                pa, pb = a.get(stage+'_pose'), b.get(stage+'_pose')
                dt = (b['stamp_ns']-a['stamp_ns'])/1e9
                if (not 0 < dt <= max_gap or not valid_pose(pa) or not valid_pose(pb) or
                        (stage != 'map_odom' and (not a.get(stage+'_valid') or not b.get(stage+'_valid')))):
                    dx.append(math.nan); dh.append(math.nan)
                else:
                    dx.append(float(np.linalg.norm((vector(pb)-vector(pa))[:2]))*1000)
                    dh.append(math.degrees(wrap(yaw(pb)-yaw(pa))))
            axes[1].plot(ids[1:], dx, '.-', label=stage, color=colors[stage])
            axes[2].plot(ids[1:], dh, '.-', label=stage, color=colors[stage])
        axes[1].set_yscale('symlog', linthresh=1.)
        axes[3].plot([r['to_match_id'] for r in transitions], [r['ndt_contribution_to_next_init_xy_m']*1000 for r in transitions], '.-', label='Previous NDT contribution')
        axes[3].plot([r['to_match_id'] for r in transitions], [r['lio_contribution_to_next_init_xy_m']*1000 for r in transitions], '.-', label='FAST-LIO contribution')
        axes[3].set_yscale('symlog', linthresh=1.)
        for ax, label in zip(axes, ('Applied update mm', 'Adjacent XY mm', 'Adjacent yaw deg', 'Next T_init contribution mm')):
            ax.set(xlabel='Match ID', ylabel=label); ax.grid(alpha=.3); ax.legend(loc='best')
        fig.suptitle('Frame increments and inherited NDT update; symlog below/above 1 mm')
        fig.savefig(output/('stability_increments.'+ext), dpi=170); plt.close(fig)


def report_text(summary, records, transitions):
    s = summary
    def mm(v):
        return '未知' if v is None else f'{1000*v:.3f}'
    def deg(v):
        return '未知' if v is None else f'{v:.4f}'
    lines = ['# 初始化后静止 NDT 分析', '',
             f"初始化完成 match {s['initialization']['match_id']}；排除稳定期 {s['settling_match_ids']}；正式段 {s['formal_match_ids']}。",
             f"正式段 {len(s['formal_match_ids'])} 次匹配，跨度 {s['formal_duration_seconds']:.2f} s，接受 {s['formal_accepted']}、拒绝 {s['formal_rejected']}，动捕覆盖率 {s['formal_reference_coverage']:.1%}。", '',
             '首次相对人工粗初值的修正只描述初始化过程。该数据无法判定其绝对定位偏差，已排除其稳定性统计。',
             '只用排除的稳定期建立一次固定 SE(2) 动捕→map 变换，T_init/T_ndt/T_final 共用；正式数据不重新拟合。',
             'map→odom 是相对排除期状态中心的变化量，没有动捕 map→odom 真值。', '', '## 数据条件与限制', '']
    lines += ['- '+w for w in s['warnings']] or ['- 未触发所配置的静止阈值告警。']
    for label, key in [('动捕正式段', 'mocap_formal'), ('由诊断重建 FAST-LIO', 'lio_reconstructed_formal'), ('稳定期最终位姿', 'ndt_final_settling')]:
        v = s['static_verification'][key]
        lines.append(f"- {label}：去中心 XY RMS {mm(v.get('xy_about_centroid_m', {}).get('rmse'))} mm。")
    lines += ['', '## 正式阶段位姿变化', '',
              '| 状态 | XY RMSE mm | P95 mm | 最大 mm | 相邻 ΔXY RMS mm | 相邻 Δyaw RMS ° |',
              '|---|---:|---:|---:|---:|---:|']
    for stage in STAGES:
        a, b = s['all_formal'][stage]['xy_m'], s['adjacent_frames'][stage]
        lines.append(f"| {stage} | {mm(a['rmse'])} | {mm(a['p95_absolute'])} | {mm(a['max_absolute'])} | {mm(b['delta_xy_m']['rmse'])} | {deg(b['delta_yaw_deg']['rmse'])} |")
    lines += ['', '全部原始可评估结果与接受结果分别保存在 summary.json 的 all_formal / accepted_formal；不跨无效位姿或超限间隔计算增量，动捕失配时不计算参考增量误差。',
              '', '## 实际应用的 map→odom 更新', '', '| 阈值（严格大于） | 次数 / 有效分母 | 比例 |', '|---|---:|---:|']
    for threshold, v in s['updates']['map_applied_update_xy_m']['thresholds'].items():
        lines.append(f"| {threshold} mm | {v['count']} / {v['denominator']} | {v['ratio']:.1%} |" if v['ratio'] is not None else f'| {threshold} mm | 0 / 0 | 未知 |')
    e = s['ndt_net_effect_all']
    lines += ['', '## NDT 净效果（相对此固定参考）', '',
              (f"改善 {e['improved_count']} / {e['paired_frames']}（{e['improved_ratio']:.1%}）；恶化 {e['worsened_count']} / {e['paired_frames']}（{e['worsened_ratio']:.1%}）；不变 {e['neutral_count']}。" if e['paired_frames'] else '没有可配对计算净效果的帧。'),
              f"改善帧平均减少 {mm(e['mean_improvement_m'])} mm；恶化帧平均增加 {mm(e['mean_worsening_m'])} mm。",
              f"最大误差增加在 match {e['max_worsening_match_id']}，Δe = {mm(e['max_worsening_m'])} mm。",
              '这些改善/恶化依赖固定对齐参考；若排除期仍未稳定，不能把它们解释为绝对真值下的正确/错误更新。',
              '', '## 大更新与后继初值', '',
              '| match | 应用 map→odom mm | base 修正 mm | Δe mm | fitness | 迭代 | 下一帧继承状态差 mm |',
              '|---|---:|---:|---:|---:|---:|---:|']
    formal = [r for r in records if r['phase'] == 'formal' and not r['duplicate_measurement']]
    for r in formal:
        if (r['map_applied_update_xy_m'] or 0) <= .03:
            continue
        t = next((t for t in transitions if t['from_match_id'] == r['match_id']), {})
        score = f"{r['fitness']:.5f}" if r.get('fitness_valid') else '未知'
        lines.append(f"| {r['match_id']} | {mm(r['map_applied_update_xy_m'])} | {mm(r['base_candidate_update_xy_m'])} | {mm(r['delta_e_m'])} | {score} | {r.get('iterations')} | {mm(t.get('map_state_inheritance_position_difference_m'))} |")
    largest = max((r for r in formal if r['map_applied_update_xy_m'] is not None), key=lambda r: r['map_applied_update_xy_m'], default=None)
    if largest:
        followed = [r for r in s['large_update_persistence'] if r['event_match_id'] == largest['match_id']]
        if followed:
            last = followed[-1]
            lines += ['', f"最大更新 match {largest['match_id']} 之后，到 match {last['future_match_id']} 的初值仍相对该事件前的 map→odom 状态偏移 {mm(last['cumulative_init_displacement_from_pre_event_m'])} mm（同一未来 odom 位姿下比较）。",
                      '这描述事件后连同后续 NDT 更新的累计状态，不把多个更新的作用全部归因于单一帧；逐帧见 event_persistence.csv。']
    lines += ['', 'propagation.csv 使用完整三维变换验证 M_pre(k+1)=M_post(k)，并把下一帧 T_init 变化分为前次 NDT 更新与 FAST-LIO 变化的向量贡献。',
              'map→odom 的修正与 base_link 的修正作用于不同原点，旋转存在杠杆臂，二者平移数值可不同。',
              'event_context.csv 保留每次 >30 mm 应用更新前后各3帧（含初始化/稳定期的阶段标签），便于查看后续状态是否被再次校正。',
              '', '## 判读', '',
              f"人工粗初始化引起的大修正与后续静止变化已分段。固定跳过{s['options']['settling_matches']}次只是预先约定的统计规则，不保证算法已完成收敛；正式段中的持续收敛或跳变仍完整保留。",
              '较低 fitness、已收敛、已接受都不能单独证明定位正确；此报告可定位更新发生与传播的位置，不能仅凭诊断分辨局部最优、地图几何约束、外参或模型失配。',
              'T_final 指定位节点内部的云时刻最终 base_link 位姿。在线 odom→base_link 桥使用平面化姿态，内部计算使用全3D外参，两者不可直接声称严格相同。',
              '动捕使用接收端时间，仅按当前静止条件解释；独立绝对 map 标定和严格动态同步仍未建立。', '']
    audit = s.get('tf_chain_audit')
    if audit:
        lines += ['## TF 几何链独立核查', '',
                  f"{audit['matched_measurements']} / {audit['requested_measurements']} 帧可匹配。内部 T_final 与『本帧最终 map→odom × 云时刻实际 odom→base_link TF』的 XY 差 RMS {mm(audit['internal_vs_tf_chain_xy_difference_m']['rmse'])} mm，最大 {mm(audit['internal_vs_tf_chain_xy_difference_m']['max_absolute'])} mm。",
                  '该组合检查内部与TF外参实现的一致性，不声称NDT计算完成前的测量时刻已广播了此最终TF。',
                  '人工初值话题记录情况：'+json.dumps(audit['initialpose_topics'], ensure_ascii=False)+'。未录制的话题不能用于断言没有再次点击；另用前后 map→odom 状态连续性核查。', '']
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    src = parser.add_mutually_exclusive_group(required=True)
    src.add_argument('--bag', type=Path)
    src.add_argument('--input-json', type=Path, help='diagnostics plus raw Tracker mocap, including full map->odom states')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--tracker-to-base-config', type=Path, default=Path('robot_data/config/mocap_tracker0.yaml'))
    parser.add_argument('--ndt-topic', default='/static_localization/ndt_match_diagnostic_detail')
    parser.add_argument('--mocap-topic', default='/mocap/Tracker0/pose')
    parser.add_argument('--settling-matches', type=int, default=3)
    parser.add_argument('--max-interpolation-gap-seconds', type=float, default=.10)
    parser.add_argument('--max-adjacent-gap-seconds', type=float, default=5.)
    parser.add_argument('--time-offset-seconds', type=float, default=0.)
    args = parser.parse_args()
    if args.output.exists():
        raise SystemExit('Output already exists: '+str(args.output))
    if args.settling_matches < 2 or min(args.max_interpolation_gap_seconds, args.max_adjacent_gap_seconds) <= 0 or not all(math.isfinite(v) for v in (args.time_offset_seconds, args.max_interpolation_gap_seconds, args.max_adjacent_gap_seconds)):
        raise SystemExit('Need settling-matches >=2 and finite positive time gaps')
    diagnostics, raw_mocap = load_rosbag(args.bag, args.ndt_topic, args.mocap_topic) if args.bag else load_json_input(args.input_json)
    mount = tracker_to_base_from_config(args.tracker_to_base_config)
    mocap = apply_tracker_to_base(raw_mocap, mount)
    options = Options(args.settling_matches, args.max_interpolation_gap_seconds, args.max_adjacent_gap_seconds, args.time_offset_seconds)
    records, summary, transitions = analyze(diagnostics, mocap, options)
    if args.bag:
        from ndt_post_init_tf_audit import audit_tf_chain
        summary['tf_chain_audit'] = audit_tf_chain(args.bag, records)
    summary.update({'input_bag': str(args.bag.resolve()) if args.bag else None,
                    'tracker_to_base_applied': mount, 'ndt_topic': args.ndt_topic, 'mocap_topic': args.mocap_topic})
    args.output.mkdir(parents=True)
    (args.output/'summary.json').write_text(json.dumps(json_clean(summary), ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    (args.output/'input_data.json').write_text(json.dumps(json_clean({'diagnostics': diagnostics, 'mocap': raw_mocap}), ensure_ascii=False, allow_nan=False)+'\n', encoding='utf-8')
    write_csv(args.output/'frames.csv', records)
    write_csv(args.output/'propagation.csv', transitions)
    write_csv(args.output/'event_context.csv', event_context(records))
    write_csv(args.output/'event_persistence.csv', summary['large_update_persistence'])
    if 'tf_chain_audit' in summary:
        write_csv(args.output/'tf_chain_audit.csv', summary['tf_chain_audit']['measurements'])
    (args.output/'report.md').write_text(report_text(summary, records, transitions), encoding='utf-8')
    plots(args.output, records, transitions, options.max_adjacent_gap_seconds)
    print(f"Wrote {args.output}; initialization={summary['initialization']['match_id']}; settling={summary['settling_match_ids']}; formal={len(summary['formal_match_ids'])}")


if __name__ == '__main__':
    main()
