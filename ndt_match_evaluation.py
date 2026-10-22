#!/usr/bin/env python3
"""Offline NDT-vs-mocap evaluator.

This process never starts ROS nodes, broadcasts TF, sends goals, or changes
NDT/Nav2 parameters.  Its normal input is a rosbag2 containing the diagnostic
topic emitted by static_map_localizer_node and the raw compensated mocap pose.
"""
from __future__ import annotations

import argparse
import csv
from collections import Counter
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml

from ndt_match_evaluation_core import (
    Se2Alignment, error_metrics, estimate_start_alignment, interpolate_pose,
    pose_error, rigid_body_to_base_pose, wrap, yaw,
)


POSE_KEYS = ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def pose_dict(pose):
    return {'x': float(pose.position.x), 'y': float(pose.position.y), 'z': float(pose.position.z),
            'qx': float(pose.orientation.x), 'qy': float(pose.orientation.y),
            'qz': float(pose.orientation.z), 'qw': float(pose.orientation.w)}


def finite_pose(value):
    return value is not None and all(math.isfinite(float(value[key])) for key in POSE_KEYS)


def diagnostic_from_ros(message):
    """Convert V1 or versioned-detail diagnostic to a JSON-safe record.

    V1 bags remain deliberately supported.  They do *not* contain a pose at
    the cloud timestamp before NDT, so ``initial_valid`` is false rather than
    silently substituting the callback-time legacy pose.
    """
    base = getattr(message, 'base', message)
    raw_valid = bool(base.raw_ndt_map_to_base_at_cloud_valid)
    final_valid = bool(base.final_map_to_base_at_cloud_valid)
    initial_valid = bool(getattr(message, 'initial_map_to_base_at_cloud_valid', False))
    return {
        'diagnostic_detail_available': hasattr(message, 'base'),
        'match_id': int(base.match_id),
        'stamp_ns': stamp_ns(base.cloud_stamp),
        'completed_ns': stamp_ns(base.matching_completed_stamp),
        'started_ns': stamp_ns(base.matching_started_stamp),
        'duration_s': float(base.computation_duration_sec),
        'stage': base.stage, 'decision_reason': base.decision_reason,
        'accepted': bool(base.accepted), 'converged': bool(base.converged),
        # This is PCL getFinalNumIteration(), not a synthetic counter.
        'iterations': int(base.iteration_count),
        'fitness_valid': bool(base.fitness_score_valid), 'fitness': float(base.fitness_score),
        'fitness_definition': base.fitness_score_definition,
        'score_lower_is_better': bool(base.score_lower_is_better),
        'input_points': int(base.input_point_count), 'source_points': int(base.source_point_count),
        'target_raw_points': int(base.target_raw_point_count), 'target_points': int(base.target_point_count),
        'max_fitness': float(base.configured_max_fitness_score),
        'update_translation_m': float(base.correction_translation_m),
        'update_yaw_rad': float(base.correction_yaw_rad),
        'covariance_available': bool(base.covariance_available),
        'covariance_note': base.covariance_note,
        'cloud_to_odom_sync_valid': bool(base.cloud_to_odom_sync_valid),
        'cloud_to_odom_skew_s': float(base.cloud_to_odom_skew_sec),
        # Preserve the full SE(3) state as well as the base-link poses.  These
        # are independent of mocap alignment and allow update propagation to
        # be audited without confusing the odom origin with the chassis.
        'initial_map_odom_pose': (pose_dict(base.initial_map_to_odom)
                                  if base.initial_prediction_valid else None),
        'raw_map_odom_pose': (pose_dict(base.raw_ndt_map_to_odom)
                              if base.raw_ndt_result_valid else None),
        'initial_valid': initial_valid,
        'initial_pose': (pose_dict(message.initial_map_to_base_at_cloud) if initial_valid else None),
        'initial_pose_source': getattr(message, 'initial_pose_at_cloud_source',
                                       'unavailable_in_v1_diagnostic'),
        'raw_valid': raw_valid, 'raw_pose': pose_dict(base.raw_ndt_map_to_base_at_cloud) if raw_valid else None,
        'final_valid': final_valid, 'final_pose': pose_dict(base.final_map_to_base_at_cloud) if final_valid else None,
        'final_pose_source': base.final_pose_source,
        'final_publish_valid': bool(getattr(message, 'final_map_to_base_at_publish_valid', False)),
        'final_publish_pose': (pose_dict(message.final_map_to_base_at_publish)
                               if bool(getattr(message, 'final_map_to_base_at_publish_valid', False)) else None),
        'final_publish_stamp_ns': (stamp_ns(message.final_pose_publish_stamp)
                                   if hasattr(message, 'final_pose_publish_stamp') else 0),
        'configured_ndt_max_iterations': getattr(message, 'configured_ndt_max_iterations', None),
        'configured_ndt_transformation_epsilon': getattr(message, 'configured_ndt_transformation_epsilon', None),
        'configured_ndt_step_size': getattr(message, 'configured_ndt_step_size', None),
        'configured_ndt_resolution': getattr(message, 'configured_ndt_resolution', None),
        'ndt_align_duration_s': float(getattr(message, 'ndt_align_duration_sec', math.nan)),
        'init_to_ndt_correction_valid': bool(getattr(message, 'init_to_ndt_correction_valid', False)),
        'init_to_ndt_correction_pose': (pose_dict(message.initial_to_ndt_base_at_cloud)
                                        if bool(getattr(message, 'init_to_ndt_correction_valid', False)) else None),
        'init_to_ndt_translation_m': float(getattr(message, 'init_to_ndt_translation_m', math.nan)),
        'init_to_ndt_yaw_rad': float(getattr(message, 'init_to_ndt_yaw_rad', math.nan)),
        'map_odom_correction_valid': bool(getattr(message, 'map_odom_correction_valid', False)),
        'map_odom_correction_pose': (pose_dict(message.map_odom_correction)
                                     if bool(getattr(message, 'map_odom_correction_valid', False)) else None),
        'nearest_neighbor_metrics_available': bool(getattr(message, 'nearest_neighbor_metrics_available', False)),
        'nearest_neighbor_evaluated_source_count': getattr(message, 'nearest_neighbor_evaluated_source_count', None),
        'nearest_neighbor_inlier_count': getattr(message, 'nearest_neighbor_inlier_count', None),
        'nearest_neighbor_inlier_ratio': float(getattr(message, 'nearest_neighbor_inlier_ratio', math.nan)),
        'nearest_neighbor_distance_threshold_m': float(getattr(message, 'nearest_neighbor_distance_threshold_m', math.nan)),
        'nearest_neighbor_residual_rms_m': float(getattr(message, 'nearest_neighbor_residual_rms_m', math.nan)),
        'nearest_neighbor_metrics_definition': getattr(message, 'nearest_neighbor_metrics_definition',
                                                        'unavailable_in_v1_diagnostic'),
        'hessian_condition_available': bool(getattr(message, 'hessian_condition_available', False)),
        'hessian_condition_number': float(getattr(message, 'hessian_condition_number', math.nan)),
        'hessian_condition_note': getattr(message, 'hessian_condition_note', 'unavailable_in_v1_diagnostic'),
    }


def mocap_from_ros(message):
    return {'stamp_ns': stamp_ns(message.header.stamp), 'frame_id': message.header.frame_id,
            'valid': True, **pose_dict(message.pose)}


def load_rosbag(path, ndt_topic, mocap_topic):
    """Load only the two data topics; TF is retained in the bag for audit."""
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as error:
        raise RuntimeError('ROS 2 rosbag2_py/rclpy is required for --bag input; source ROS and install/setup.bash') from error
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    missing = [topic for topic in (ndt_topic, mocap_topic) if topic not in topic_types]
    if missing:
        raise RuntimeError('Required bag topics not found: '+', '.join(missing)+'; available: '+', '.join(sorted(topic_types)))
    message_types = {topic: get_message(topic_types[topic]) for topic in (ndt_topic, mocap_topic)}
    diagnostics, mocap = [], []
    while reader.has_next():
        topic, serialized, _ = reader.read_next()
        if topic not in message_types:
            continue
        message = deserialize_message(serialized, message_types[topic])
        if topic == ndt_topic:
            diagnostics.append(diagnostic_from_ros(message))
        else:
            mocap.append(mocap_from_ros(message))
    return diagnostics, mocap


def load_json_input(path):
    document = json.loads(Path(path).read_text(encoding='utf-8'))
    if not isinstance(document, dict) or not isinstance(document.get('diagnostics'), list) or not isinstance(document.get('mocap'), list):
        raise ValueError('JSON input must be an object with diagnostics[] and mocap[]')
    return document['diagnostics'], document['mocap']


def tracker_to_base_from_config(path):
    """Load an explicit fixed Tracker-rigid-body -> base_link transform.

    The passive receiver intentionally publishes Tracker0 rather than a pose
    mislabeled as base_link.  This loader makes the conversion auditable and
    confines it to offline evaluation.
    """
    document = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    try:
        transform = document['rigid_body_to_base_link']
        measured = transform['measured']
        translation = [float(value) for value in transform['translation_m']]
        quaternion = [float(value) for value in transform['quaternion_xyzw']]
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError('Invalid rigid_body_to_base_link in '+str(path)) from error
    if measured is not True or len(translation) != 3 or len(quaternion) != 4:
        raise ValueError('rigid_body_to_base_link must be explicitly measured with three translation and four quaternion values')
    if not all(math.isfinite(value) for value in (*translation, *quaternion)):
        raise ValueError('rigid_body_to_base_link contains non-finite values')
    if math.hypot(*quaternion) < 1e-9:
        raise ValueError('rigid_body_to_base_link quaternion is zero')
    return {'config_file': str(Path(path).resolve()), 'translation_m': translation,
            'quaternion_xyzw': quaternion, 'notes': str(transform.get('notes', ''))}


def apply_tracker_to_base(mocap, transform):
    """Return base_link poses while preserving source timestamps and validity."""
    return [rigid_body_to_base_pose(item, transform['translation_m'], transform['quaternion_xyzw']) for item in mocap]


def mapped_reference(raw, alignment):
    """Map a base-link mocap pose with the one fixed start-window alignment."""
    return alignment.apply(raw)


def start_alignment(diagnostics, mocap, args):
    candidates = []
    rejected = Counter()
    for diagnostic in sorted(diagnostics, key=lambda item: int(item.get('stamp_ns', 0))):
        stamp = int(diagnostic.get('stamp_ns', 0)) + round(args.time_offset_seconds * 1e9)
        if not diagnostic.get('final_valid') or not finite_pose(diagnostic.get('final_pose')):
            continue
        raw, reason = interpolate_pose(mocap, stamp, args.max_interpolation_gap_seconds)
        if raw is None:
            rejected['alignment_'+str(reason)] += 1
            continue
        candidates.append((stamp, raw, diagnostic['final_pose']))
    if not candidates:
        raise RuntimeError('No time-matched final map poses available for initial alignment')
    recording_start = candidates[0][0]
    warmup_seconds = getattr(args, 'alignment_start_delay_seconds', 0.0)
    alignment_start = recording_start + round(warmup_seconds * 1e9)
    window = [item for item in candidates if
              alignment_start <= item[0] <= alignment_start + round(args.alignment_window_seconds * 1e9)]
    if len(window) < args.alignment_min_samples:
        raise RuntimeError(
            f'Initial alignment has {len(window)} samples after {warmup_seconds:g} s warmup; '
            f'need {args.alignment_min_samples}')
    transform = estimate_start_alignment([(item[1], item[2]) for item in window], args.mocap_to_base_yaw_rad,
                                         args.alignment_max_static_motion_m)
    # Do not score the optional warmup before the alignment samples either:
    # its map<-mocap transform has not yet been established.  This is a fixed,
    # user-provided delay, never an automatically selected low-error segment.
    return (transform, recording_start, alignment_start,
            alignment_start + round(args.alignment_window_seconds*1e9), dict(rejected), len(window))


def first_raw_ndt_alignment(diagnostics, mocap, args):
    """Fix the relative origin at the first time-matched converged raw NDT pose.

    This deliberately makes the selected raw NDT output have zero error.  It
    is for analysing *changes after the first NDT result*, not absolute error
    or any map->odom discrepancy that existed before the bag began.
    """
    rejected = Counter()
    for diagnostic in sorted(diagnostics, key=lambda item: (int(item.get('stamp_ns', 0)), int(item.get('match_id', 0)))):
        if not diagnostic.get('converged'):
            rejected['baseline_not_converged'] += 1
            continue
        if not diagnostic.get('raw_valid') or not finite_pose(diagnostic.get('raw_pose')):
            rejected['baseline_raw_pose_unavailable'] += 1
            continue
        stamp = int(diagnostic.get('stamp_ns', 0))
        if stamp <= 0:
            rejected['baseline_missing_or_zero_cloud_stamp'] += 1
            continue
        raw, reason = interpolate_pose(mocap, stamp + round(args.time_offset_seconds*1e9),
                                       args.max_interpolation_gap_seconds)
        if raw is None:
            rejected['baseline_'+str(reason)] += 1
            continue
        ndt = diagnostic['raw_pose']
        theta = wrap(yaw(ndt) - yaw(raw) - args.mocap_to_base_yaw_rad)
        c, s = math.cos(theta), math.sin(theta)
        tx = float(ndt['x']) - c*float(raw['x']) + s*float(raw['y'])
        ty = float(ndt['y']) - s*float(raw['x']) - c*float(raw['y'])
        return (Se2Alignment(theta, tx, ty, args.mocap_to_base_yaw_rad, 'first_raw_ndt_baseline'),
                {'match_id': int(diagnostic.get('match_id', 0)), 'stamp_ns': stamp,
                 'stage': diagnostic.get('stage', ''), 'accepted': bool(diagnostic.get('accepted'))},
                dict(rejected))
    raise RuntimeError('No converged raw NDT pose can be time-matched to mocap for first-raw-NDT baseline')


def first_initial_alignment(diagnostics, mocap, args):
    """Fix a cold-start relative origin at the first true pre-NDT prediction.

    The intended collection protocol starts recording before the localizer and
    stops it immediately after this registration.  The first ``T_init`` then
    contains only the one manual initialization and FAST-LIO, never a prior
    accepted NDT map->odom update.  Unlike ``first_raw_ndt_alignment``, this
    does *not* erase the first NDT correction: raw/final errors at the
    baseline timestamp expose precisely that correction.
    """
    passive_stages = {'waiting_for_initialization', 'no_input_cloud', 'source_below_minimum',
                      'submap_below_minimum', 'no_odom_pose_at_cloud_time'}
    rejected = Counter()
    for diagnostic in sorted(diagnostics, key=lambda item: (int(item.get('stamp_ns', 0)), int(item.get('match_id', 0)))):
        if str(diagnostic.get('stage', '')) in passive_stages:
            rejected['baseline_not_a_registration'] += 1
            continue
        if not diagnostic.get('initial_valid') or not finite_pose(diagnostic.get('initial_pose')):
            raise RuntimeError('The first true NDT registration has no valid cloud-time T_init; do not substitute a later match')
        stamp = int(diagnostic.get('stamp_ns', 0))
        if stamp <= 0:
            raise RuntimeError('The first true NDT registration has no valid input-cloud timestamp')
        raw, reason = interpolate_pose(mocap, stamp + round(args.time_offset_seconds*1e9),
                                       args.max_interpolation_gap_seconds)
        if raw is None:
            raise RuntimeError('The first true NDT registration cannot be matched to mocap: '+str(reason)+
                               '; do not substitute a later match')
        initial = diagnostic['initial_pose']
        theta = wrap(yaw(initial) - yaw(raw) - args.mocap_to_base_yaw_rad)
        c, s = math.cos(theta), math.sin(theta)
        tx = float(initial['x']) - c*float(raw['x']) + s*float(raw['y'])
        ty = float(initial['y']) - s*float(raw['x']) - c*float(raw['y'])
        return (Se2Alignment(theta, tx, ty, args.mocap_to_base_yaw_rad, 'first_initial_prediction_baseline'),
                {'match_id': int(diagnostic.get('match_id', 0)), 'stamp_ns': stamp,
                 'stage': diagnostic.get('stage', ''), 'accepted': bool(diagnostic.get('accepted')),
                 'converged': bool(diagnostic.get('converged')),
                 'raw_pose_available': bool(diagnostic.get('raw_valid')),
                 'final_pose_available': bool(diagnostic.get('final_valid'))},
                dict(rejected))
    raise RuntimeError('No first NDT-registration T_init can be time-matched to mocap; require V2 detail diagnostics')


def evaluate(diagnostics, mocap, args):
    diagnostics = sorted(diagnostics, key=lambda item: (int(item.get('stamp_ns', 0)), int(item.get('match_id', 0))))
    mocap = sorted(mocap, key=lambda item: int(item.get('stamp_ns', 0)))
    counts = Counter()
    relative_baseline = None
    if args.alignment_mode == 'first_raw_ndt':
        alignment, relative_baseline, alignment_rejected = first_raw_ndt_alignment(diagnostics, mocap, args)
        alignment_exclusion_start, alignment_start, alignment_end, alignment_samples = None, None, None, 0
    elif args.alignment_mode == 'first_init':
        alignment, relative_baseline, alignment_rejected = first_initial_alignment(diagnostics, mocap, args)
        alignment_exclusion_start, alignment_start, alignment_end, alignment_samples = None, None, None, 0
    else:
        (alignment, alignment_exclusion_start, alignment_start, alignment_end,
         alignment_rejected, alignment_samples) = start_alignment(diagnostics, mocap, args)

    rows = []
    for diagnostic in diagnostics:
        counts['total_attempts'] += 1
        stamp = int(diagnostic.get('stamp_ns', 0))
        row = dict(diagnostic)
        row['reference_valid'] = False
        row['reference_reason'] = ''
        row['reference_pose'] = None
        row['initial_error'] = None
        row['raw_error'] = None
        row['final_error'] = None
        row['in_alignment_window'] = bool(
            alignment_exclusion_start is not None and alignment_exclusion_start <= stamp <= alignment_end)
        row['is_relative_baseline'] = bool(
            relative_baseline is not None and int(relative_baseline['match_id']) == int(row.get('match_id', -1)))
        if stamp <= 0:
            row['reference_reason'] = 'missing_or_zero_cloud_stamp'
            counts[row['reference_reason']] += 1
            rows.append(row)
            continue
        raw, reason = interpolate_pose(mocap, stamp + round(args.time_offset_seconds*1e9), args.max_interpolation_gap_seconds)
        if raw is None:
            row['reference_reason'] = reason
            counts[reason] += 1
            rows.append(row)
            continue
        try:
            reference = mapped_reference(raw, alignment)
        except (ValueError, KeyError, TypeError) as error:
            row['reference_reason'] = 'invalid_mocap_pose:'+str(error)
            counts['invalid_mocap_pose'] += 1
            rows.append(row)
            continue
        row['reference_valid'] = True
        row['reference_pose'] = reference
        counts['reference_valid'] += 1
        if diagnostic.get('initial_valid') and finite_pose(diagnostic.get('initial_pose')):
            row['initial_error'] = pose_error(reference, diagnostic['initial_pose'])
            counts['initial_fastlio_pose_evaluable'] += 1
        elif diagnostic.get('initial_valid'):
            counts['invalid_initial_fastlio_pose'] += 1
        if diagnostic.get('raw_valid') and finite_pose(diagnostic.get('raw_pose')):
            row['raw_error'] = pose_error(reference, diagnostic['raw_pose'])
            counts['raw_pose_evaluable'] += 1
        elif diagnostic.get('raw_valid'):
            counts['invalid_raw_pose'] += 1
        if diagnostic.get('final_valid') and finite_pose(diagnostic.get('final_pose')):
            row['final_error'] = pose_error(reference, diagnostic['final_pose'])
            counts['final_pose_evaluable'] += 1
        elif diagnostic.get('final_valid'):
            counts['invalid_final_pose'] += 1
        rows.append(row)

    if args.alignment_mode == 'first_init':
        # This mode answers one deliberately narrow question: what did the
        # first real NDT registration do to its own pre-NDT prediction?  Keep
        # every later diagnostic row in matches.csv for auditability, but do
        # not allow a second (already history-contaminated) registration to
        # enter the cold-start summary metrics.
        test_rows = [row for row in rows if row['is_relative_baseline']]
        analysis_scope = 'first true NDT registration only; later rows retained in matches.csv but excluded from metrics'
    else:
        test_rows = [row for row in rows if not row['in_alignment_window']]
        analysis_scope = ('all non-alignment-window rows' if args.alignment_mode == 'start'
                          else 'all rows relative to the first raw NDT baseline')
    initial_all = [row for row in test_rows if row['initial_error'] is not None]
    initial_accepted = [row for row in initial_all if row.get('accepted')]
    raw_all = [row for row in test_rows if row['raw_error'] is not None]
    raw_accepted = [row for row in raw_all if row.get('accepted')]
    final_all = [row for row in test_rows if row['final_error'] is not None]
    final_accepted = [row for row in final_all if row.get('accepted')]
    attempted_ndt = [row for row in rows if row.get('stage') not in ('waiting_for_initialization', 'no_input_cloud',
                                                                     'source_below_minimum', 'submap_below_minimum')]
    score_rows = [row for row in raw_all if row.get('fitness_valid') and math.isfinite(float(row.get('fitness', math.nan)))]
    durations = [float(row['duration_s']) for row in rows if math.isfinite(float(row.get('duration_s', math.nan)))]
    ndt_align_durations = [float(row['ndt_align_duration_s']) for row in rows
                           if math.isfinite(float(row.get('ndt_align_duration_s', math.nan)))]
    summary = {
        'schema_version': 2,
        'created_utc': datetime.now(timezone.utc).isoformat(),
        'alignment_mode': args.alignment_mode,
        'measurement_time': 'NDT input /cloud_registered header timestamp; never matching completion timestamp',
        'time_offset_seconds': args.time_offset_seconds,
        'mocap_interpolation_max_gap_seconds': args.max_interpolation_gap_seconds,
        'alignment_window_excluded_from_metrics': args.alignment_mode == 'start',
        'alignment_warmup_excluded_from_metrics': args.alignment_mode == 'start',
        'metrics_analysis_scope': analysis_scope,
        'metrics_scored_rows': len(test_rows),
        'counts': dict(counts),
        'total_attempts': len(rows),
        'attempted_ndt_registrations': len(attempted_ndt),
        'converged_attempts': sum(bool(row.get('converged')) for row in attempted_ndt),
        'accepted_attempts': sum(bool(row.get('accepted')) for row in rows),
        'failed_or_not_accepted_attempts': sum(not bool(row.get('accepted')) for row in attempted_ndt),
        'reference_coverage_all_attempts': len([row for row in rows if row['reference_valid']]) / len(rows) if rows else 0.,
        'reference_coverage_test_attempts': len([row for row in test_rows if row['reference_valid']]) / len(test_rows) if test_rows else 0.,
        'initial_fastlio_all_evaluable': error_metrics(initial_all, 'initial'),
        'initial_fastlio_accepted_evaluable': error_metrics(initial_accepted, 'initial'),
        'raw_ndt_all_evaluable': error_metrics(raw_all, 'raw'),
        'raw_ndt_accepted_evaluable': error_metrics(raw_accepted, 'raw'),
        'final_map_base_all_evaluable': error_metrics(final_all, 'final'),
        'final_map_base_accepted_evaluable': error_metrics(final_accepted, 'final'),
        'fitness_vs_error_samples': len(score_rows),
        'fitness_definition': (score_rows[0].get('fitness_definition') if score_rows else
                               'No finite NDT fitness score in evaluable records'),
        'score_lower_is_better': True,
        'covariance': 'No covariance is reported unless upstream diagnostics explicitly expose it; this node reports unavailable.',
        'initial_pose_time_semantics': (
            'Available only in NdtMatchDiagnosticDetail: T_map_odom before NDT multiplied by '
            'T_odom_base nearest the input-cloud timestamp. V1 diagnostics do not provide this pose.'),
        'final_pose_time_semantics': (
            'final_map_base is evaluated at input-cloud time for fair comparison. The separately recorded '
            'final_publish_pose has publication time and is not used for measurement-time error metrics.'),
    }
    if durations:
        summary['matching_duration_s'] = {'samples': len(durations), 'mean': float(np.mean(durations)),
                                          'p95': float(np.percentile(durations, 95)), 'max': float(np.max(durations))}
    else:
        summary['matching_duration_s'] = {'samples': 0}
    if ndt_align_durations:
        summary['ndt_align_duration_s'] = {
            'samples': len(ndt_align_durations), 'mean': float(np.mean(ndt_align_durations)),
            'p95': float(np.percentile(ndt_align_durations, 95)), 'max': float(np.max(ndt_align_durations)),
            'definition': 'PCL NormalDistributionsTransform::align() only; excludes post-alignment diagnostics'}
    else:
        summary['ndt_align_duration_s'] = {
            'samples': 0,
            'definition': 'Unavailable in V1 diagnostics; V2 detail records PCL align duration separately.'}
    if args.alignment_mode == 'first_raw_ndt':
        summary['first_raw_ndt_baseline'] = {
            **relative_baseline,
            'map_from_mocap_yaw_rad': alignment.yaw_rad,
            'translation_m': [alignment.tx_m, alignment.ty_m],
            'mocap_to_base_yaw_rad': alignment.mocap_to_base_yaw_rad,
            'rejected_before_baseline': alignment_rejected,
            'limitation': ('The selected raw NDT pose is zero by construction. Metrics quantify changes after '
                           'that first result and do not assess history before the baseline.'),
        }
    elif args.alignment_mode == 'first_init':
        summary['first_initial_prediction_baseline'] = {
            **relative_baseline,
            'map_from_mocap_yaw_rad': alignment.yaw_rad,
            'translation_m': [alignment.tx_m, alignment.ty_m],
            'mocap_to_base_yaw_rad': alignment.mocap_to_base_yaw_rad,
            'rejected_before_baseline': alignment_rejected,
            'limitation': ('The selected pre-NDT T_init is zero by construction. The raw/final value at '
                           'this same timestamp is retained, so it measures the first NDT correction relative '
                           'to T_init; this does not measure manual 2D-pose initialization bias.'),
        }
        first = test_rows[0]
        summary['cold_start_first_match'] = {
            'match_id': int(first['match_id']),
            'cloud_stamp_ns': int(first['stamp_ns']),
            'stage': first.get('stage', ''),
            'accepted': bool(first.get('accepted')),
            'converged': bool(first.get('converged')),
            'iterations': first.get('iterations'),
            'fitness': (first.get('fitness') if first.get('fitness_valid') else None),
            'fitness_definition': first.get('fitness_definition', ''),
            'initial_error_relative_to_T_init': first.get('initial_error'),
            'raw_ndt_error_relative_to_T_init': first.get('raw_error'),
            'final_error_relative_to_T_init': first.get('final_error'),
            'init_to_ndt_correction': {
                'available': bool(first.get('init_to_ndt_correction_valid')),
                'translation_m': first.get('init_to_ndt_translation_m'),
                'yaw_rad': first.get('init_to_ndt_yaw_rad'),
            },
            'map_odom_correction': {
                'available': bool(first.get('map_odom_correction_valid')),
                'translation_m': first.get('update_translation_m'),
                'yaw_rad': first.get('update_yaw_rad'),
            },
            'poses_at_input_cloud_timestamp': {
                'T_init': first.get('initial_pose'),
                'T_ndt': first.get('raw_pose'),
                'T_final': first.get('final_pose'),
            },
            'interpretation': ('T_init is the zero relative origin only for this one cold-start registration. '
                               'T_ndt/T_final values are retained; this is the NDT-induced change relative '
                               'to the pre-NDT initial prediction, not an absolute map-localization error.'),
        }
    else:
        summary['start_alignment'] = {'map_from_mocap_yaw_rad': alignment.yaw_rad,
                                      'translation_m': [alignment.tx_m, alignment.ty_m],
                                      'mocap_to_base_yaw_rad': alignment.mocap_to_base_yaw_rad,
                                      'start_stamp_ns': alignment_start, 'end_stamp_ns': alignment_end,
                                      'metrics_exclusion_start_stamp_ns': alignment_exclusion_start,
                                      'warmup_seconds': getattr(args, 'alignment_start_delay_seconds', 0.0),
                                      'samples': alignment_samples, 'rejected_during_alignment': alignment_rejected,
                                      'limitation': 'Initial fixed offset is absorbed; reports relative localization error after start alignment.'}
    return rows, summary


def compact_pose(pose, prefix, output):
    if pose is None:
        for key in POSE_KEYS:
            output[prefix+'_'+key] = ''
    else:
        for key in POSE_KEYS:
            output[prefix+'_'+key] = pose[key]


def write_csv(path, rows):
    flat = []
    for row in rows:
        item = {key: row.get(key, '') for key in ('match_id', 'stamp_ns', 'completed_ns', 'duration_s', 'stage',
                'decision_reason', 'accepted', 'converged', 'iterations', 'fitness_valid', 'fitness',
                'score_lower_is_better', 'input_points', 'source_points', 'target_raw_points', 'target_points',
                'max_fitness', 'update_translation_m', 'update_yaw_rad', 'cloud_to_odom_sync_valid',
                'cloud_to_odom_skew_s', 'diagnostic_detail_available', 'initial_valid', 'initial_pose_source',
                'raw_valid', 'final_valid', 'final_pose_source', 'final_publish_valid', 'final_publish_stamp_ns',
                'configured_ndt_max_iterations', 'configured_ndt_transformation_epsilon',
                'configured_ndt_step_size', 'configured_ndt_resolution',
                'ndt_align_duration_s',
                'init_to_ndt_correction_valid', 'init_to_ndt_translation_m', 'init_to_ndt_yaw_rad',
                'map_odom_correction_valid', 'nearest_neighbor_metrics_available',
                'nearest_neighbor_evaluated_source_count', 'nearest_neighbor_inlier_count',
                'nearest_neighbor_inlier_ratio', 'nearest_neighbor_distance_threshold_m',
                'nearest_neighbor_residual_rms_m', 'hessian_condition_available',
                'hessian_condition_number', 'hessian_condition_note', 'reference_valid',
                'reference_reason', 'in_alignment_window', 'is_relative_baseline')}
        compact_pose(row.get('reference_pose'), 'reference', item)
        compact_pose(row.get('initial_pose'), 'initial', item)
        compact_pose(row.get('raw_pose'), 'raw', item)
        compact_pose(row.get('final_pose'), 'final', item)
        compact_pose(row.get('final_publish_pose'), 'final_publish', item)
        compact_pose(row.get('init_to_ndt_correction_pose'), 'init_to_ndt_correction', item)
        compact_pose(row.get('map_odom_correction_pose'), 'map_odom_correction', item)
        for source, prefix in ((row.get('initial_error'), 'initial_error'), (row.get('raw_error'), 'raw_error'),
                               (row.get('final_error'), 'final_error')):
            for key in ('x_m', 'y_m', 'xy_m', 'yaw_rad'):
                item[prefix+'_'+key] = '' if source is None else source[key]
        flat.append(item)
    fields = sorted({key for item in flat for key in item})
    with Path(path).open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(flat)


def valid_plot_rows(rows, kind, summary):
    return [row for row in rows
            if not row['in_alignment_window']
            and (summary['alignment_mode'] != 'first_init' or row['is_relative_baseline'])
            and row.get(kind+'_error') is not None]


def save_plots(directory, rows, summary):
    directory = Path(directory)
    initial, raw, final = (valid_plot_rows(rows, 'initial', summary), valid_plot_rows(rows, 'raw', summary),
                           valid_plot_rows(rows, 'final', summary))
    for extension in ('png', 'svg'):
        fig, axis = plt.subplots(figsize=(8, 7), constrained_layout=True)
        reference = final or raw or initial
        if reference:
            axis.plot([row['reference_pose']['x'] for row in reference], [row['reference_pose']['y'] for row in reference],
                      '.-', ms=3, color='tab:green', label='Mocap reference')
        if raw:
            axis.plot([row['raw_pose']['x'] for row in raw], [row['raw_pose']['y'] for row in raw],
                      '.-', ms=3, color='tab:orange', label='Raw NDT')
        if initial:
            axis.plot([row['initial_pose']['x'] for row in initial], [row['initial_pose']['y'] for row in initial],
                      '.-', ms=3, color='tab:red', label='FAST-LIO initial prediction')
        if final:
            axis.plot([row['final_pose']['x'] for row in final], [row['final_pose']['y'] for row in final],
                      '.-', ms=3, color='tab:blue', label='Final map→base_link')
        axis.set(xlabel='Map X (m)', ylabel='Map Y (m)', title='NDT measurement-time trajectory comparison')
        axis.axis('equal'); axis.grid(alpha=.3)
        if reference or initial or raw or final:
            axis.legend()
        fig.savefig(directory/('trajectory.'+extension), dpi=180); plt.close(fig)

        fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True, constrained_layout=True)
        for items, name, color in ((initial, 'initial', 'tab:red'), (raw, 'raw', 'tab:orange'),
                                   (final, 'final', 'tab:blue')):
            if not items:
                continue
            times = [(row['stamp_ns']-min(item['stamp_ns'] for item in rows))/1e9 for row in items]
            for axis, key, scale, label in ((axes[0], 'x_m', 1000, 'X'), (axes[1], 'y_m', 1000, 'Y'),
                                             (axes[2], 'xy_m', 1000, 'XY'), (axes[3], 'yaw_rad', 180/math.pi, 'Yaw')):
                axis.plot(times, [row[name+'_error'][key]*scale for row in items], color=color, label=name)
                axis.set_ylabel(label + (' (mm)' if key != 'yaw_rad' else ' (deg)')); axis.grid(alpha=.3)
        axes[-1].set_xlabel('Seconds from first NDT record')
        for axis in axes:
            if axis.lines:
                axis.legend()
        fig.suptitle('Localization error at NDT input-cloud timestamps')
        fig.savefig(directory/('errors.'+extension), dpi=180); plt.close(fig)

        score = [row for row in raw if row.get('fitness_valid') and math.isfinite(float(row.get('fitness', math.nan)))]
        fig, axes = plt.subplots(2, 1, figsize=(10, 8), constrained_layout=True)
        if score:
            times = [(row['stamp_ns']-min(item['stamp_ns'] for item in rows))/1e9 for row in score]
            colors = ['tab:green' if row['accepted'] else 'tab:red' for row in score]
            axes[0].scatter(times, [row['fitness'] for row in score], c=colors, s=22)
            axes[0].set_ylabel('NDT fitness (lower better)')
            axes[1].scatter([row['fitness'] for row in score], [row['raw_error']['xy_m'] for row in score], c=colors, s=22)
            axes[1].set(xlabel='NDT fitness (lower better)', ylabel='Raw NDT XY error (m)')
        else:
            axes[0].text(.5, .5, 'No finite fitness/raw-error pairs', transform=axes[0].transAxes, ha='center')
        for axis in axes:
            axis.grid(alpha=.3)
        fig.suptitle('Green: accepted; red: rejected. Fitness is not absolute error.')
        fig.savefig(directory/('fitness_vs_error.'+extension), dpi=180); plt.close(fig)


def write_summary_text(path, summary):
    if summary['alignment_mode'] == 'first_raw_ndt':
        alignment_note = ('Metrics use the first time-matched converged raw NDT output as zero relative origin; '
                          'the baseline is included and history before it is not assessed.')
    elif summary['alignment_mode'] == 'first_init':
        alignment_note = ('Metrics use the first true pre-NDT T_init as zero relative origin; first raw/final '
                          'errors therefore show the first NDT update relative to that initial prediction.')
    else:
        alignment_note = ('Metrics exclude the configured start warmup and start-alignment window. '
                          'A missing reference, raw NDT result, or final pose is counted and never replaced by zero.')
    text = [f"Total scheduled attempts: {summary['total_attempts']}",
            f"NDT registrations entered: {summary['attempted_ndt_registrations']}",
            f"Metrics scored rows: {summary['metrics_scored_rows']} ({summary['metrics_analysis_scope']})",
            f"Converged registrations: {summary['converged_attempts']}",
            f"Accepted attempts: {summary['accepted_attempts']}",
            f"Reference coverage (all/test): {summary['reference_coverage_all_attempts']:.1%} / {summary['reference_coverage_test_attempts']:.1%}",
            'PCL ndt.align duration: '+json.dumps(summary['ndt_align_duration_s'], ensure_ascii=False),
            '', alignment_note, 'A missing reference, raw NDT result, or final pose is counted and never replaced by zero.',
            'Initial FAST-LIO, raw NDT, and final poses use the SAME mocap→map alignment transform.',
            'Matching completion time is not used as the measurement time.', '']
    for name in ('initial_fastlio_all_evaluable', 'initial_fastlio_accepted_evaluable',
                 'raw_ndt_all_evaluable', 'raw_ndt_accepted_evaluable',
                 'final_map_base_all_evaluable', 'final_map_base_accepted_evaluable'):
        text.append(name+': '+json.dumps(json_clean(summary[name]), ensure_ascii=False, allow_nan=False))
    if 'cold_start_first_match' in summary:
        text.extend(['', 'Cold-start first NDT registration:',
                     json.dumps(json_clean(summary['cold_start_first_match']), ensure_ascii=False, indent=2,
                                allow_nan=False)])
    text.extend(['', 'Rejection / unmatched counts:',
                 json.dumps(json_clean(summary['counts']), ensure_ascii=False, indent=2, allow_nan=False)])
    Path(path).write_text('\n'.join(text)+'\n', encoding='utf-8')


def json_clean(value):
    """JSON has no NaN token; preserve unavailable numeric diagnostics as null."""
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {key: json_clean(item) for key, item in value.items()}
    if isinstance(value, list):
        return [json_clean(item) for item in value]
    return value


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--bag', type=Path, help='rosbag2 directory')
    source.add_argument('--input-json', type=Path, help='normalized synthetic/offline JSON input')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--ndt-topic', default='/static_localization/ndt_match_diagnostic_detail',
                        help='V2 detail topic by default. Use /static_localization/ndt_match_diagnostic explicitly for old V1 bags, which lack cloud-time T_init.')
    parser.add_argument('--mocap-topic', default='/mocap/Tracker0/pose')
    parser.add_argument('--tracker-to-base-config', type=Path,
                        default=Path('robot_data/config/mocap_tracker0.yaml'),
                        help='Explicit Tracker0->base_link YAML applied offline to --mocap-topic; never publishes TF.')
    parser.add_argument('--alignment-mode', choices=('start', 'first_raw_ndt', 'first_init'), default='start',
                        help='start: static-window relative error; first_raw_ndt: re-zero at raw NDT; first_init: cold-start source test, zero the first pre-NDT T_init and retain the first NDT correction.')
    parser.add_argument('--mocap-to-base-yaw-rad', type=float, default=0.0,
                        help='Only for input already expressed at base_link. Do not combine a nonzero value with --tracker-to-base-config.')
    # The production localizer attempts NDT once per 2 s by default.  Keep a
    # 25 s default so the default minimum of ten time-matched diagnostics is
    # actually attainable without changing the online localization cadence.
    parser.add_argument('--alignment-window-seconds', type=float, default=25.)
    parser.add_argument('--alignment-start-delay-seconds', type=float, default=0.,
                        help='Fixed initial static warmup before the alignment window. It and the alignment window are excluded from metrics; never auto-selected.')
    parser.add_argument('--alignment-min-samples', type=int, default=10)
    parser.add_argument('--alignment-max-static-motion-m', type=float, default=.05)
    parser.add_argument('--max-interpolation-gap-seconds', type=float, default=.10)
    parser.add_argument('--time-offset-seconds', type=float, default=0.,
                        help='Only a separately measured fixed mocap timing offset; it is never trajectory-optimized.')
    return parser.parse_args()


def main():
    args = parse_args()
    if (args.alignment_window_seconds <= 0 or args.alignment_start_delay_seconds < 0 or
            args.alignment_min_samples < 3 or args.alignment_max_static_motion_m <= 0 or
            args.max_interpolation_gap_seconds <= 0):
        raise SystemExit('Alignment/interpolation limits must be positive; alignment-min-samples must be at least 3.')
    if not math.isfinite(args.time_offset_seconds) or not math.isfinite(args.alignment_start_delay_seconds):
        raise SystemExit('--time-offset-seconds and --alignment-start-delay-seconds must be finite.')
    tracker_to_base = tracker_to_base_from_config(args.tracker_to_base_config) if args.tracker_to_base_config else None
    if tracker_to_base is not None:
        if abs(args.mocap_to_base_yaw_rad) > 1e-12:
            raise SystemExit('Do not combine --tracker-to-base-config with a nonzero --mocap-to-base-yaw-rad; that would apply the rigid-body rotation twice.')
        # The full Tracker->base transform is applied before alignment, so
        # start-alignment must not apply a second yaw-only mount correction.
        args.mocap_to_base_yaw_rad = 0.0
    if not math.isfinite(args.mocap_to_base_yaw_rad):
        raise SystemExit('--mocap-to-base-yaw-rad must be finite.')
    diagnostics, raw_mocap = (load_rosbag(args.bag, args.ndt_topic, args.mocap_topic) if args.bag else load_json_input(args.input_json))
    mocap = apply_tracker_to_base(raw_mocap, tracker_to_base) if tracker_to_base is not None else raw_mocap
    if args.output.exists():
        raise SystemExit(f'Output already exists: {args.output}')
    try:
        rows, summary = evaluate(diagnostics, mocap, args)
    except (RuntimeError, ValueError) as error:
        raise SystemExit(f'Evaluation stopped without creating output: {error}') from error
    args.output.mkdir(parents=True)
    summary['mocap_topic'] = args.mocap_topic
    if tracker_to_base is not None:
        summary['mocap_reference_pose'] = 'base_link pose derived offline from recorded Tracker rigid-body pose'
        summary['tracker_to_base_applied'] = tracker_to_base
    else:
        summary['mocap_reference_pose'] = 'input pose used directly; caller is responsible for confirming it is already base_link'
    (args.output/'summary.json').write_text(json.dumps(json_clean(summary), ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    write_csv(args.output/'matches.csv', rows)
    (args.output/'input_diagnostics.json').write_text(json.dumps(json_clean(diagnostics), ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    (args.output/'input_mocap.json').write_text(json.dumps(json_clean(mocap), ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    if tracker_to_base is not None:
        (args.output/'input_mocap_tracker_raw.json').write_text(json.dumps(json_clean(raw_mocap), ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    save_plots(args.output, rows, summary)
    write_summary_text(args.output/'summary.txt', summary)
    print(f'Wrote {args.output}; attempts={summary["total_attempts"]}, reference coverage={summary["reference_coverage_all_attempts"]:.1%}')


if __name__ == '__main__':
    main()
