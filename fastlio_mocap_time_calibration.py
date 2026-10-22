#!/usr/bin/env python3
"""Estimate the fixed Tracker0--FAST-LIO timestamp offset from a calibration bag.

This is deliberately an *offline*, passive tool.  It never starts a ROS node,
does not publish TF, and does not change FAST-LIO, NDT, Nav2, the VRPN server,
or the XINGYING project.  It estimates a time offset only from an independently
recorded calibration motion: planar-speed and signed yaw-rate correlations.
It never spatially aligns trajectories or minimizes trajectory-position error.

Sign convention
---------------
For an exact FAST-LIO ``odom -> base_link`` transform stamped ``t``, a positive
reported offset ``d`` means that the matching motion-capture pose is evaluated
at header timestamp ``t + d``.  Thus a +20 ms result means the same motion
appears about 20 ms later in the motion-capture header timeline.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from collections import Counter
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable

import numpy as np
import yaml

from ndt_match_evaluation_core import rigid_body_to_base_pose, wrap, yaw


def stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def pose_from_pose_message(message) -> dict:
    """Convert PoseStamped to the small ROS-independent pose representation."""
    return {
        'stamp_ns': stamp_ns(message.header.stamp),
        'x': float(message.pose.position.x), 'y': float(message.pose.position.y),
        'z': float(message.pose.position.z),
        'qx': float(message.pose.orientation.x), 'qy': float(message.pose.orientation.y),
        'qz': float(message.pose.orientation.z), 'qw': float(message.pose.orientation.w),
        'valid': True,
    }


def pose_from_transform(transform) -> dict:
    return {
        'stamp_ns': stamp_ns(transform.header.stamp),
        'x': float(transform.transform.translation.x),
        'y': float(transform.transform.translation.y),
        'z': float(transform.transform.translation.z),
        'qx': float(transform.transform.rotation.x), 'qy': float(transform.transform.rotation.y),
        'qz': float(transform.transform.rotation.z), 'qw': float(transform.transform.rotation.w),
        'valid': True,
    }


def pose_from_odom(message) -> dict:
    pose = message.pose.pose
    return {
        'stamp_ns': stamp_ns(message.header.stamp),
        'x': float(pose.position.x), 'y': float(pose.position.y), 'z': float(pose.position.z),
        'qx': float(pose.orientation.x), 'qy': float(pose.orientation.y),
        'qz': float(pose.orientation.z), 'qw': float(pose.orientation.w), 'valid': True,
    }


def finite_pose(sample: dict) -> bool:
    try:
        values = [float(sample[key]) for key in ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')]
        return int(sample.get('stamp_ns', 0)) > 0 and all(math.isfinite(value) for value in values) and \
            math.sqrt(sum(value * value for value in values[3:])) > 1e-9
    except (KeyError, TypeError, ValueError):
        return False


def deduplicate_poses(samples: Iterable[dict]) -> tuple[list[dict], dict]:
    """Keep one pose per stamp; drop conflicting duplicate samples transparently."""
    ordered = sorted((dict(item) for item in samples if finite_pose(item)), key=lambda item: int(item['stamp_ns']))
    result: list[dict] = []
    exact_duplicates = conflicting_duplicates = 0
    for item in ordered:
        if result and int(item['stamp_ns']) == int(result[-1]['stamp_ns']):
            old = result[-1]
            same = all(abs(float(item[key]) - float(old[key])) <= 1e-10
                       for key in ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'))
            if same:
                exact_duplicates += 1
            else:
                conflicting_duplicates += 1
            continue
        result.append(item)
    return result, {
        'input_samples': len(samples), 'usable_unique_samples': len(result),
        'exact_duplicates_ignored': exact_duplicates,
        'conflicting_duplicates_ignored': conflicting_duplicates,
        'invalid_or_zero_stamp_ignored': len(samples) - len(ordered),
    }


def load_tracker_to_base(path: Path) -> dict:
    document = yaml.safe_load(path.read_text(encoding='utf-8'))
    try:
        raw = document['rigid_body_to_base_link']
        measured = raw['measured']
        translation = [float(value) for value in raw['translation_m']]
        quaternion = [float(value) for value in raw['quaternion_xyzw']]
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f'Invalid rigid_body_to_base_link in {path}') from error
    if measured is not True or len(translation) != 3 or len(quaternion) != 4:
        raise ValueError('rigid_body_to_base_link must be explicitly measured and contain 3-D translation + xyzw quaternion')
    if not all(math.isfinite(value) for value in (*translation, *quaternion)) or math.hypot(*quaternion) < 1e-9:
        raise ValueError('rigid_body_to_base_link has non-finite values or a zero quaternion')
    return {'config_file': str(path.resolve()), 'translation_m': translation,
            'quaternion_xyzw': quaternion, 'notes': str(raw.get('notes', ''))}


def read_rosbag(path: Path, mocap_topic: str, tf_topic: str, odom_topic: str,
                odom_frame: str, base_frame: str) -> tuple[list[dict], list[dict], dict]:
    """Read passive mocap and the exact corrected LIO transform from a rosbag2.

    ``/tf`` is preferred because its transform header preserves the LIO pose
    time.  ``/odom`` is only a fall-back for bags without that TF; its bridge
    publication timestamp is explicitly labelled in the result.
    """
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as error:
        raise RuntimeError('Source /opt/ros/humble/setup.bash and install/setup.bash before reading rosbag2') from error
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if mocap_topic not in topic_types:
        raise RuntimeError(f'Bag is missing {mocap_topic}; available: {", ".join(sorted(topic_types))}')
    subscribed = [mocap_topic]
    if tf_topic in topic_types:
        subscribed.append(tf_topic)
    if odom_topic in topic_types:
        subscribed.append(odom_topic)
    if not any(topic in subscribed for topic in (tf_topic, odom_topic)):
        raise RuntimeError(f'Bag has neither {tf_topic} nor {odom_topic}; cannot obtain FAST-LIO pose samples')
    kinds = {topic: get_message(topic_types[topic]) for topic in subscribed}
    mocap, tf_poses, odom_poses = [], [], []
    message_counts: Counter[str] = Counter()
    record_times: list[int] = []
    while reader.has_next():
        topic, serialized, record_stamp_ns = reader.read_next()
        if topic not in kinds:
            continue
        message_counts[topic] += 1
        record_times.append(int(record_stamp_ns))
        message = deserialize_message(serialized, kinds[topic])
        if topic == mocap_topic:
            mocap.append(pose_from_pose_message(message))
        elif topic == tf_topic:
            for transform in message.transforms:
                if transform.header.frame_id == odom_frame and transform.child_frame_id == base_frame:
                    tf_poses.append(pose_from_transform(transform))
        elif topic == odom_topic:
            if message.header.frame_id == odom_frame and message.child_frame_id == base_frame:
                odom_poses.append(pose_from_odom(message))
    mocap, mocap_audit = deduplicate_poses(mocap)
    tf_poses, tf_audit = deduplicate_poses(tf_poses)
    odom_poses, odom_audit = deduplicate_poses(odom_poses)
    if tf_poses:
        lio, source, lio_audit = tf_poses, 'tf_exact_transform_stamp', tf_audit
    elif odom_poses:
        lio, source, lio_audit = odom_poses, 'odom_bridge_header_stamp_fallback', odom_audit
    else:
        raise RuntimeError(f'No usable {odom_frame}->{base_frame} samples found in {tf_topic} or {odom_topic}')
    info = {
        'available_topics': topic_types,
        'bag_record_time_start_ns': min(record_times) if record_times else None,
        'bag_record_time_end_ns': max(record_times) if record_times else None,
        'read_message_counts': dict(message_counts),
        'fastlio_pose_source': source,
        'mocap_audit': mocap_audit,
        'tf_audit': tf_audit,
        'odom_audit': odom_audit,
    }
    return lio, mocap, info


def _time_median(t: np.ndarray, values: np.ndarray, radius_s: float) -> np.ndarray:
    """Variable-rate centred median; robust to isolated rigid-body jumps."""
    answer = np.empty(len(values), dtype=float)
    for index, now in enumerate(t):
        left = np.searchsorted(t, now - radius_s, side='left')
        right = np.searchsorted(t, now + radius_s, side='right')
        window = values[left:right]
        answer[index] = float(np.median(window[np.isfinite(window)])) if np.isfinite(window).any() else math.nan
    return answer


def _robust_local_slope(t: np.ndarray, values: np.ndarray, half_window_s: float,
                        residual_floor: float) -> np.ndarray:
    """Centred local least-squares derivative with one MAD residual rejection pass."""
    output = np.full(len(values), math.nan, dtype=float)
    for index, now in enumerate(t):
        left = np.searchsorted(t, now - half_window_s, side='left')
        right = np.searchsorted(t, now + half_window_s, side='right')
        tx = t[left:right] - now
        yy = values[left:right]
        finite = np.isfinite(tx) & np.isfinite(yy)
        if np.count_nonzero(finite) < 3:
            continue
        tx, yy = tx[finite], yy[finite]
        if float(np.ptp(tx)) <= 1e-9:
            continue
        slope, intercept = np.polyfit(tx, yy, 1)
        residual = yy - (slope * tx + intercept)
        center = float(np.median(residual))
        mad = float(np.median(np.abs(residual - center)))
        threshold = max(float(residual_floor), 4.0 * 1.4826 * mad)
        accepted = np.abs(residual - center) <= threshold
        if np.count_nonzero(accepted) >= 3 and float(np.ptp(tx[accepted])) > 1e-9:
            slope = float(np.polyfit(tx[accepted], yy[accepted], 1)[0])
        output[index] = float(slope)
    return output


def kinematic_signals(samples: list[dict], *, median_radius_seconds: float = 0.10,
                      derivative_half_window_seconds: float = 0.25) -> list[dict]:
    """Produce robust planar-speed and signed-yaw-rate samples from poses.

    No frame alignment is required: planar speed and yaw rate are invariant to
    an unknown constant planar world transform.  A time median protects the
    estimator against isolated VRPN/rigid-body jumps; it is not a substitute
    for valid mocap measurements, which remain visible in output diagnostics.
    """
    ordered, _ = deduplicate_poses(samples)
    if len(ordered) < 5:
        raise ValueError('need_at_least_five_valid_pose_samples_for_kinematics')
    stamps = np.asarray([int(item['stamp_ns']) for item in ordered], dtype=np.int64)
    if np.any(np.diff(stamps) <= 0):
        raise ValueError('nonmonotonic_pose_timestamps')
    t = (stamps - stamps[0]).astype(float) / 1e9
    x = np.asarray([item['x'] for item in ordered], dtype=float)
    y = np.asarray([item['y'] for item in ordered], dtype=float)
    heading = np.unwrap(np.asarray([yaw(item) for item in ordered], dtype=float))
    x_smooth = _time_median(t, x, median_radius_seconds)
    y_smooth = _time_median(t, y, median_radius_seconds)
    heading_smooth = _time_median(t, heading, median_radius_seconds)
    vx = _robust_local_slope(t, x_smooth, derivative_half_window_seconds, residual_floor=0.008)
    vy = _robust_local_slope(t, y_smooth, derivative_half_window_seconds, residual_floor=0.008)
    wz = _robust_local_slope(t, heading_smooth, derivative_half_window_seconds, residual_floor=0.030)
    return [
        {'stamp_ns': int(stamp), 'linear_speed_mps': float(math.hypot(dx, dy)),
         'yaw_rate_radps': float(dyaw), 'vx_mps': float(dx), 'vy_mps': float(dy)}
        for stamp, dx, dy, dyaw in zip(stamps, vx, vy, wz)
    ]


def _interpolate_scalar_strict(samples: list[dict], target_ns: np.ndarray, key: str,
                               max_gap_seconds: float) -> np.ndarray:
    """Linearly interpolate one signal without bridging a long or invalid gap."""
    stamps = np.asarray([int(item['stamp_ns']) for item in samples], dtype=np.int64)
    values = np.asarray([float(item[key]) for item in samples], dtype=float)
    output = np.full(len(target_ns), math.nan, dtype=float)
    positions = np.searchsorted(stamps, target_ns, side='left')
    exact = (positions < len(stamps)) & (stamps[np.minimum(positions, len(stamps)-1)] == target_ns)
    exact_indices = np.where(exact)[0]
    for idx in exact_indices:
        value = values[positions[idx]]
        if math.isfinite(float(value)):
            output[idx] = value
    other = np.where(~exact & (positions > 0) & (positions < len(stamps)))[0]
    max_gap_ns = round(max_gap_seconds * 1e9)
    for idx in other:
        left, right = positions[idx] - 1, positions[idx]
        gap = int(stamps[right] - stamps[left])
        if gap <= 0 or gap > max_gap_ns or not (math.isfinite(float(values[left])) and math.isfinite(float(values[right]))):
            continue
        ratio = (int(target_ns[idx]) - int(stamps[left])) / gap
        output[idx] = float(values[left] + ratio * (values[right] - values[left]))
    return output


def _correlation(a: np.ndarray, b: np.ndarray, threshold: float, minimum_samples: int) -> tuple[float | None, int]:
    valid = np.isfinite(a) & np.isfinite(b) & ((np.abs(a) >= threshold) | (np.abs(b) >= threshold))
    if int(np.count_nonzero(valid)) < minimum_samples:
        return None, int(np.count_nonzero(valid))
    aa, bb = a[valid], b[valid]
    if float(np.std(aa)) < 1e-10 or float(np.std(bb)) < 1e-10:
        return None, int(np.count_nonzero(valid))
    return float(np.corrcoef(aa, bb)[0, 1]), int(np.count_nonzero(valid))


@dataclass(frozen=True)
class OffsetSearchOptions:
    search_min_seconds: float = -0.30
    search_max_seconds: float = 0.30
    search_step_seconds: float = 0.001
    max_interpolation_gap_seconds: float = 0.10
    linear_activity_threshold_mps: float = 0.015
    yaw_activity_threshold_radps: float = 0.015
    minimum_active_samples: int = 20
    near_best_correlation_drop: float = 0.010


def _validate_search_options(options: OffsetSearchOptions) -> None:
    values = asdict(options)
    if not all(math.isfinite(float(value)) for value in values.values() if isinstance(value, (float, int))):
        raise ValueError('all search options must be finite')
    if options.search_min_seconds >= options.search_max_seconds or options.search_step_seconds <= 0:
        raise ValueError('search range must be ordered and search step must be positive')
    if options.max_interpolation_gap_seconds <= 0 or options.minimum_active_samples < 3:
        raise ValueError('interpolation gap must be positive and minimum_active_samples must be >= 3')
    if options.linear_activity_threshold_mps < 0 or options.yaw_activity_threshold_radps < 0 or options.near_best_correlation_drop < 0:
        raise ValueError('activity thresholds and near-best drop must be nonnegative')


def search_time_offset(lio_signals: list[dict], mocap_signals: list[dict],
                       options: OffsetSearchOptions) -> tuple[list[dict], dict]:
    """Search a fixed time lag using invariant kinematics, never positions."""
    _validate_search_options(options)
    if len(lio_signals) < 5 or len(mocap_signals) < 5:
        raise ValueError('need_at_least_five_lio_and_mocap_kinematic_samples')
    lio_stamps = np.asarray([int(item['stamp_ns']) for item in lio_signals], dtype=np.int64)
    lio_speed = np.asarray([float(item['linear_speed_mps']) for item in lio_signals], dtype=float)
    lio_yaw_rate = np.asarray([float(item['yaw_rate_radps']) for item in lio_signals], dtype=float)
    candidate_offsets = np.arange(options.search_min_seconds,
                                  options.search_max_seconds + options.search_step_seconds * .5,
                                  options.search_step_seconds)
    rows: list[dict] = []
    for offset in candidate_offsets:
        target = lio_stamps + int(round(float(offset) * 1e9))
        mocap_speed = _interpolate_scalar_strict(mocap_signals, target, 'linear_speed_mps',
                                                 options.max_interpolation_gap_seconds)
        mocap_yaw_rate = _interpolate_scalar_strict(mocap_signals, target, 'yaw_rate_radps',
                                                    options.max_interpolation_gap_seconds)
        speed_corr, speed_n = _correlation(lio_speed, mocap_speed,
                                           options.linear_activity_threshold_mps, options.minimum_active_samples)
        yaw_corr, yaw_n = _correlation(lio_yaw_rate, mocap_yaw_rate,
                                       options.yaw_activity_threshold_radps, options.minimum_active_samples)
        valid_correlations = [value for value in (speed_corr, yaw_corr) if value is not None and math.isfinite(value)]
        # Equal channel weight avoids the 0.4-m translation segment drowning
        # the independent left/right rotation evidence merely through samples.
        combined = float(np.mean(valid_correlations)) if valid_correlations else None
        rows.append({
            'offset_seconds': float(offset),
            'offset_milliseconds': float(offset * 1000.0),
            'linear_speed_correlation': speed_corr,
            'linear_speed_active_samples': speed_n,
            'signed_yaw_rate_correlation': yaw_corr,
            'signed_yaw_rate_active_samples': yaw_n,
            'combined_correlation': combined,
            'channels_used': len(valid_correlations),
        })
    usable = [row for row in rows if row['combined_correlation'] is not None]
    if not usable:
        raise ValueError('no_candidate_has_enough_valid_active_kinematic_samples')
    best = max(usable, key=lambda row: (row['combined_correlation'], -abs(row['offset_seconds'])))
    near = [row for row in usable if row['combined_correlation'] >= best['combined_correlation'] - options.near_best_correlation_drop]
    at_zero = min(usable, key=lambda row: abs(row['offset_seconds']))
    speed_usable = [row for row in usable if row['linear_speed_correlation'] is not None]
    yaw_usable = [row for row in usable if row['signed_yaw_rate_correlation'] is not None]
    speed_best = max(speed_usable, key=lambda row: (row['linear_speed_correlation'], -abs(row['offset_seconds']))) \
        if speed_usable else None
    yaw_best = max(yaw_usable, key=lambda row: (row['signed_yaw_rate_correlation'], -abs(row['offset_seconds']))) \
        if yaw_usable else None
    summary = {
        'sign_convention': ('For each FAST-LIO transform stamped t, positive offset d evaluates the corresponding '
                            'mocap kinematics at header timestamp t+d.'),
        'best_offset_seconds': best['offset_seconds'],
        'best_offset_milliseconds': best['offset_milliseconds'],
        'best_combined_correlation': best['combined_correlation'],
        'best_linear_speed_correlation': best['linear_speed_correlation'],
        'best_signed_yaw_rate_correlation': best['signed_yaw_rate_correlation'],
        'best_channels_used': best['channels_used'],
        'zero_offset_combined_correlation': at_zero['combined_correlation'],
        'correlation_gain_over_zero': (best['combined_correlation'] - at_zero['combined_correlation']
                                       if at_zero['combined_correlation'] is not None else None),
        'near_best_interval_seconds': [min(row['offset_seconds'] for row in near),
                                       max(row['offset_seconds'] for row in near)],
        'near_best_interval_milliseconds': [min(row['offset_milliseconds'] for row in near),
                                            max(row['offset_milliseconds'] for row in near)],
        'near_best_definition': (f'candidate combined correlation >= best - {options.near_best_correlation_drop:g}; '
                                 'this is a sensitivity interval, not a statistical confidence interval'),
        'linear_speed_peak_offset_milliseconds': (speed_best['offset_milliseconds'] if speed_best else None),
        'linear_speed_peak_correlation': (speed_best['linear_speed_correlation'] if speed_best else None),
        'signed_yaw_rate_peak_offset_milliseconds': (yaw_best['offset_milliseconds'] if yaw_best else None),
        'signed_yaw_rate_peak_correlation': (yaw_best['signed_yaw_rate_correlation'] if yaw_best else None),
        'channel_peak_separation_milliseconds': (
            abs(speed_best['offset_milliseconds'] - yaw_best['offset_milliseconds'])
            if speed_best and yaw_best else None),
        'usable_candidates': len(usable),
        'total_candidates': len(rows),
    }
    return rows, summary


def _range_summary(samples: list[dict]) -> dict:
    if not samples:
        return {'samples': 0, 'start_stamp_ns': None, 'end_stamp_ns': None, 'duration_seconds': None}
    first, last = int(samples[0]['stamp_ns']), int(samples[-1]['stamp_ns'])
    return {'samples': len(samples), 'start_stamp_ns': first, 'end_stamp_ns': last,
            'duration_seconds': (last - first) / 1e9}


def assess_calibration_quality(lio: list[dict], mocap: list[dict], result: dict) -> dict:
    """Make limitations machine-readable instead of silently blessing a bad bag."""
    lio_range, mocap_range = _range_summary(lio), _range_summary(mocap)
    overlap_start = max(lio_range['start_stamp_ns'], mocap_range['start_stamp_ns'])
    overlap_end = min(lio_range['end_stamp_ns'], mocap_range['end_stamp_ns'])
    overlap_s = max(0.0, (overlap_end - overlap_start) / 1e9)
    lio_duration = max(lio_range['duration_seconds'], 1e-9)
    coverage = overlap_s / lio_duration
    width_ms = result['near_best_interval_milliseconds'][1] - result['near_best_interval_milliseconds'][0]
    reasons = []
    if mocap_range['end_stamp_ns'] + 1_000_000_000 < lio_range['end_stamp_ns']:
        reasons.append('mocap_ended_before_fastlio_recording')
    if coverage < .95:
        reasons.append('mocap_covers_less_than_95_percent_of_fastlio_duration')
    if width_ms > 30.0:
        reasons.append('near_best_offset_interval_wider_than_30_ms')
    separation = result.get('channel_peak_separation_milliseconds')
    if separation is not None and separation > 30.0:
        reasons.append('linear_and_yaw_channel_peaks_differ_by_more_than_30_ms')
    return {
        'overlap_start_stamp_ns': overlap_start, 'overlap_end_stamp_ns': overlap_end,
        'overlap_seconds': overlap_s, 'mocap_coverage_of_fastlio_duration': coverage,
        'near_best_interval_width_milliseconds': width_ms,
        'quality_status': ('provisional_not_approved_for_millimetre_dynamic_accuracy'
                           if reasons else 'usable_fixed_offset_for_subsequent_offline_matching'),
        'quality_reasons': reasons,
        'quality_rule_note': ('A missing tail, a >30 ms near-best width, or >30 ms disagreement between independent '
                              'speed/yaw peaks is treated as insufficient for a millimetre-grade dynamic claim.'),
    }


def _write_csv(path: Path, rows: list[dict]) -> None:
    keys = list(rows[0]) if rows else []
    with path.open('w', newline='', encoding='utf-8') as handle:
        writer = csv.DictWriter(handle, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


def _write_kinematic_csv(path: Path, lio_signals: list[dict], mocap_signals: list[dict], offset_seconds: float,
                         max_gap_seconds: float) -> None:
    stamps = np.asarray([int(item['stamp_ns']) for item in lio_signals], dtype=np.int64)
    target = stamps + int(round(offset_seconds * 1e9))
    m_speed = _interpolate_scalar_strict(mocap_signals, target, 'linear_speed_mps', max_gap_seconds)
    m_wz = _interpolate_scalar_strict(mocap_signals, target, 'yaw_rate_radps', max_gap_seconds)
    start = int(stamps[0])
    rows = []
    for item, target_ns, speed, wz in zip(lio_signals, target, m_speed, m_wz):
        rows.append({
            'lio_stamp_ns': int(item['stamp_ns']),
            'seconds_from_lio_start': (int(item['stamp_ns']) - start) / 1e9,
            'mocap_target_stamp_ns': int(target_ns),
            'lio_linear_speed_mps': item['linear_speed_mps'],
            'mocap_linear_speed_mps': None if not math.isfinite(float(speed)) else float(speed),
            'lio_yaw_rate_radps': item['yaw_rate_radps'],
            'mocap_yaw_rate_radps': None if not math.isfinite(float(wz)) else float(wz),
        })
    _write_csv(path, rows)


def _write_plots(output: Path, candidates: list[dict], lio_signals: list[dict], mocap_signals: list[dict],
                 result: dict, options: OffsetSearchOptions) -> list[str]:
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        return []
    plots: list[str] = []
    offsets = np.asarray([row['offset_milliseconds'] for row in candidates], dtype=float)
    combined = np.asarray([math.nan if row['combined_correlation'] is None else row['combined_correlation']
                           for row in candidates], dtype=float)
    speed = np.asarray([math.nan if row['linear_speed_correlation'] is None else row['linear_speed_correlation']
                        for row in candidates], dtype=float)
    yaw_rate = np.asarray([math.nan if row['signed_yaw_rate_correlation'] is None else row['signed_yaw_rate_correlation']
                           for row in candidates], dtype=float)
    figure, axis = plt.subplots(figsize=(9, 4.8), constrained_layout=True)
    axis.plot(offsets, combined, label='combined', linewidth=2.2)
    axis.plot(offsets, speed, label='planar speed', alpha=.8)
    axis.plot(offsets, yaw_rate, label='signed yaw rate', alpha=.8)
    axis.axvline(result['best_offset_milliseconds'], color='tab:red', linestyle='--',
                label=f"best {result['best_offset_milliseconds']:.1f} ms")
    axis.set(xlabel='mocap timestamp offset relative to FAST-LIO (ms)', ylabel='Pearson correlation',
             title='Independent time-offset calibration from kinematic signals')
    axis.grid(True, alpha=.25)
    axis.legend()
    for suffix in ('svg', 'png'):
        path = output / f'time_offset_correlation.{suffix}'
        figure.savefig(path, dpi=180 if suffix == 'png' else None)
        plots.append(path.name)
    plt.close(figure)

    lio_stamps = np.asarray([int(item['stamp_ns']) for item in lio_signals], dtype=np.int64)
    target = lio_stamps + int(round(result['best_offset_seconds'] * 1e9))
    m_speed = _interpolate_scalar_strict(mocap_signals, target, 'linear_speed_mps', options.max_interpolation_gap_seconds)
    m_wz = _interpolate_scalar_strict(mocap_signals, target, 'yaw_rate_radps', options.max_interpolation_gap_seconds)
    elapsed = (lio_stamps - lio_stamps[0]) / 1e9
    figure, axes = plt.subplots(2, 1, figsize=(10, 6.5), sharex=True, constrained_layout=True)
    axes[0].plot(elapsed, [item['linear_speed_mps'] for item in lio_signals], label='FAST-LIO', linewidth=1.8)
    axes[0].plot(elapsed, m_speed, label='mocap at t+best offset', alpha=.85)
    axes[0].set(ylabel='planar speed (m/s)', title='Kinematic signals at selected fixed offset')
    axes[1].plot(elapsed, [item['yaw_rate_radps'] for item in lio_signals], label='FAST-LIO', linewidth=1.8)
    axes[1].plot(elapsed, m_wz, label='mocap at t+best offset', alpha=.85)
    axes[1].set(xlabel='seconds from first FAST-LIO TF', ylabel='signed yaw rate (rad/s)')
    mocap_end_s = (int(mocap_signals[-1]['stamp_ns']) - int(lio_signals[0]['stamp_ns'])) / 1e9
    for axis in axes:
        if elapsed[0] <= mocap_end_s <= elapsed[-1]:
            axis.axvline(mocap_end_s, color='0.25', linestyle=':', alpha=.8,
                         label='last valid mocap sample')
        axis.grid(True, alpha=.25)
        axis.legend()
    for suffix in ('svg', 'png'):
        path = output / f'kinematic_signals_at_best_offset.{suffix}'
        figure.savefig(path, dpi=180 if suffix == 'png' else None)
        plots.append(path.name)
    plt.close(figure)
    return plots


def _markdown(summary: dict, output: Path) -> str:
    result = summary['offset_search']
    mocap = summary['input']['mocap_base_link']
    lio = summary['input']['fastlio']
    quality = summary['quality']
    quality_line = ('**本包仅可作为初步估计，不能用于毫米级动态误差结论。** 原因：`' +
                    '`、`'.join(quality['quality_reasons']) + '`。'
                    if quality['quality_reasons'] else
                    '**本包通过当前自动质量规则，可将固定偏移用于后续独立轨迹包的离线时间匹配。**')
    return '\n'.join([
        '# FAST-LIO — Tracker0 时间偏移标定报告', '',
        '该报告只使用独立时间标定动作包的平移速度与有符号航向角速度相关性。它没有空间对齐，'
        '没有最小化轨迹位置误差，也不修改任何在线节点。', '',
        '## 结果', '',
        f"- 建议固定偏移：**{result['best_offset_milliseconds']:.1f} ms**", 
        f"- 定义：{result['sign_convention']}",
        f"- 合并相关性：{result['best_combined_correlation']:.5f}；零偏移：{result['zero_offset_combined_correlation']:.5f}",
        f"- 相对零偏移增益：{result['correlation_gain_over_zero']:.5f}",
        f"- 近最优敏感区间：{result['near_best_interval_milliseconds'][0]:.1f} 至 "
        f"{result['near_best_interval_milliseconds'][1]:.1f} ms（非统计置信区间）", '',
        f"- 平移通道峰值：{result['linear_speed_peak_offset_milliseconds']:.1f} ms；"
        f"航向通道峰值：{result['signed_yaw_rate_peak_offset_milliseconds']:.1f} ms；"
        f"二者相差：{result['channel_peak_separation_milliseconds']:.1f} ms。", '',
        quality_line, '',
        '## 数据覆盖', '',
        f"- FAST-LIO 精确 TF：{lio['samples']} 条，持续 {lio['duration_seconds']:.3f} s；"
        f"来源：`{summary['input']['fastlio_pose_source']}`。",
        f"- 动捕（已应用显式 Tracker0→base_link 外参）：{mocap['samples']} 条，持续 "
        f"{mocap['duration_seconds']:.3f} s。后续无动捕样本不会被外推或重复。", '',
        f"- 有效共同时间段：{quality['overlap_seconds']:.3f} s，占 FAST-LIO 全段的 "
        f"{quality['mocap_coverage_of_fastlio_duration'] * 100.0:.1f}%。", '',
        '## 限制', '',
        '- 动捕 header 当前来自 Ubuntu 接收时钟；该数值是该采集链的经验固定偏移，不能声称为相机曝光时刻延迟。',
        '- 若近最优区间很宽、平移和转向通道给出相互矛盾的峰值，必须重录带连续有效动捕的标定包，不能把该值用于毫米级结论。',
        '- 该结果应只供后续独立两米包的时间匹配使用；不能用两米精度包反向重估此偏移。', '',
        '## 文件', '',
        '- `time_offset_summary.json`：机器可读完整审计与参数；',
        '- `time_offset_candidates.csv`：每个候选偏移的相关性；',
        '- `kinematic_signals.csv`：最佳偏移下的逐帧信号；',
        '- `time_offset_correlation.svg/png`、`kinematic_signals_at_best_offset.svg/png`：图。', '',
    ])


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True, help='rosbag2 directory for the dedicated timing calibration run')
    parser.add_argument('--output', type=Path, required=True, help='new or empty output directory')
    parser.add_argument('--mocap-topic', default='/mocap/Tracker0/pose')
    parser.add_argument('--tf-topic', default='/tf')
    parser.add_argument('--odom-topic', default='/odom')
    parser.add_argument('--odom-frame', default='odom')
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--tracker-to-base-config', type=Path, required=True)
    parser.add_argument('--search-min-seconds', type=float, default=-.30)
    parser.add_argument('--search-max-seconds', type=float, default=.30)
    parser.add_argument('--search-step-seconds', type=float, default=.001)
    parser.add_argument('--max-interpolation-gap-seconds', type=float, default=.10)
    parser.add_argument('--linear-activity-threshold-mps', type=float, default=.015)
    parser.add_argument('--yaw-activity-threshold-radps', type=float, default=.015)
    parser.add_argument('--minimum-active-samples', type=int, default=20)
    parser.add_argument('--near-best-correlation-drop', type=float, default=.010)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if not args.bag.is_dir():
        raise SystemExit(f'Bag directory does not exist: {args.bag}')
    if args.output.exists() and any(args.output.iterdir()):
        raise SystemExit(f'Output directory must be absent or empty: {args.output}')
    args.output.mkdir(parents=True, exist_ok=True)
    transform = load_tracker_to_base(args.tracker_to_base_config)
    lio, tracker, bag_info = read_rosbag(args.bag, args.mocap_topic, args.tf_topic, args.odom_topic,
                                         args.odom_frame, args.base_frame)
    mocap = [rigid_body_to_base_pose(item, transform['translation_m'], transform['quaternion_xyzw'])
             for item in tracker]
    lio_signals = kinematic_signals(lio)
    mocap_signals = kinematic_signals(mocap)
    options = OffsetSearchOptions(
        search_min_seconds=args.search_min_seconds, search_max_seconds=args.search_max_seconds,
        search_step_seconds=args.search_step_seconds,
        max_interpolation_gap_seconds=args.max_interpolation_gap_seconds,
        linear_activity_threshold_mps=args.linear_activity_threshold_mps,
        yaw_activity_threshold_radps=args.yaw_activity_threshold_radps,
        minimum_active_samples=args.minimum_active_samples,
        near_best_correlation_drop=args.near_best_correlation_drop,
    )
    candidates, result = search_time_offset(lio_signals, mocap_signals, options)
    quality = assess_calibration_quality(lio, mocap, result)
    _write_csv(args.output / 'time_offset_candidates.csv', candidates)
    _write_kinematic_csv(args.output / 'kinematic_signals.csv', lio_signals, mocap_signals,
                         result['best_offset_seconds'], options.max_interpolation_gap_seconds)
    summary = {
        'schema_version': 1,
        'purpose': ('Fixed time offset from independent calibration motion only; no position-error minimization or '
                    'spatial trajectory fitting is performed.'),
        'input': {
            'bag': str(args.bag.resolve()), 'mocap_topic': args.mocap_topic, 'tf_topic': args.tf_topic,
            'odom_topic': args.odom_topic, 'fastlio_pose_source': bag_info['fastlio_pose_source'],
            'fastlio': _range_summary(lio), 'tracker0_raw': _range_summary(tracker),
            'mocap_base_link': _range_summary(mocap), 'bag_audit': bag_info,
        },
        'tracker0_to_base_link': transform,
        'kinematic_estimator': {
            'median_radius_seconds': .10, 'derivative_half_window_seconds': .25,
            'signal_definition': 'planar speed magnitude and signed yaw rate; both invariant to a fixed planar world-frame alignment',
            'outlier_handling': 'centred time median plus robust local derivative; no missing sample is extrapolated',
        },
        'search_options': asdict(options),
        'offset_search': result,
        'quality': quality,
        'limitations': [
            'Mocap poses end at their last valid header timestamp. The tool does not repeat or extrapolate a stale pose.',
            'The selected offset is for offline matching of this receiver-clock dataset. It is not a verified camera exposure-time latency.',
            'This tool does not use position error, estimate a map/world transform, or control the robot.',
        ],
    }
    plots = _write_plots(args.output, candidates, lio_signals, mocap_signals, result, options)
    summary['output_files'] = ['time_offset_summary.json', 'time_offset_report.md', 'time_offset_candidates.csv',
                               'kinematic_signals.csv', *plots]
    (args.output / 'time_offset_summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    (args.output / 'time_offset_report.md').write_text(_markdown(summary, args.output), encoding='utf-8')
    print(f"Wrote {args.output}; best offset={result['best_offset_milliseconds']:.1f} ms; "
          f"combined correlation={result['best_combined_correlation']:.5f}")


if __name__ == '__main__':
    main()
