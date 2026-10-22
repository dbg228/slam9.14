#!/usr/bin/env python3
"""Create an auditable static-stability report from Tracker0 rosbag2 files.

This offline tool never starts a ROS node or changes the robot.  It reports
the *reference-system* noise floor only; it is not an NDT localization-error
evaluation.  Every bag stays a separate record even when its human label is
repeated, avoiding an accidental merge of distinct trials.
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


NORMALIZED_TOPIC = '/mocap/Tracker0/pose'
RAW_TOPIC = '/vrpn_mocap/Tracker0/pose'
DIAGNOSTICS_TOPIC = '/mocap/Tracker0/diagnostics'


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def pose_record(message):
    return {
        'stamp_ns': stamp_ns(message.header.stamp),
        'frame_id': message.header.frame_id,
        'x': float(message.pose.position.x), 'y': float(message.pose.position.y), 'z': float(message.pose.position.z),
        'qx': float(message.pose.orientation.x), 'qy': float(message.pose.orientation.y),
        'qz': float(message.pose.orientation.z), 'qw': float(message.pose.orientation.w),
    }


def uint8(value):
    return int.from_bytes(value, 'little') if isinstance(value, bytes) else int(value)


def diagnostic_record(message):
    statuses = []
    for status in message.status:
        statuses.append({
            'level': uint8(status.level), 'message': status.message,
            'values': {item.key: item.value for item in status.values},
        })
    return {'stamp_ns': stamp_ns(message.header.stamp), 'statuses': statuses}


def read_bag(path):
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as error:
        raise RuntimeError('Source ROS 2 and install/setup.bash before reading rosbag2') from error
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    available = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if NORMALIZED_TOPIC not in available:
        raise RuntimeError(f'{path}: missing required topic {NORMALIZED_TOPIC}')
    topics = [topic for topic in (NORMALIZED_TOPIC, RAW_TOPIC, DIAGNOSTICS_TOPIC) if topic in available]
    kinds = {topic: get_message(available[topic]) for topic in topics}
    normalized, raw, diagnostics = [], [], []
    while reader.has_next():
        topic, serialized, _ = reader.read_next()
        if topic not in kinds:
            continue
        message = deserialize_message(serialized, kinds[topic])
        if topic == NORMALIZED_TOPIC:
            normalized.append(pose_record(message))
        elif topic == RAW_TOPIC:
            raw.append(pose_record(message))
        else:
            diagnostics.append(diagnostic_record(message))
    return normalized, raw, diagnostics


def percentile(values, value):
    return float(np.percentile(np.asarray(values, dtype=float), value))


def analyze_samples(normalized, raw, diagnostics, raw_position_scale=0.001):
    """Pure statistics helper, kept ROS-free for synthetic unit tests."""
    normalized = sorted(normalized, key=lambda item: item['stamp_ns'])
    raw = sorted(raw, key=lambda item: item['stamp_ns'])
    if len(normalized) < 2:
        raise ValueError('Need at least two normalized pose samples')
    stamps = np.asarray([item['stamp_ns'] for item in normalized], dtype=np.int64)
    intervals = np.diff(stamps) / 1e9
    xyz = np.asarray([[item['x'], item['y'], item['z']] for item in normalized], dtype=float)
    center = np.mean(xyz, axis=0)
    radial_xy_m = np.linalg.norm(xyz[:, :2] - center[:2], axis=1)
    angles = np.asarray([yaw(item) for item in normalized], dtype=float)
    yaw_center = math.atan2(float(np.sum(np.sin(angles))), float(np.sum(np.cos(angles))))
    yaw_deviation = np.asarray([wrap(value-yaw_center) for value in angles], dtype=float)
    raw_by_stamp = {item['stamp_ns']: item for item in raw}
    conversion_residual_m = []
    for item in normalized:
        source = raw_by_stamp.get(item['stamp_ns'])
        if source is None:
            continue
        conversion_residual_m.append(math.sqrt(
            (source['x'] * raw_position_scale - item['x']) ** 2 +
            (source['y'] * raw_position_scale - item['y']) ** 2 +
            (source['z'] * raw_position_scale - item['z']) ** 2))
    status_count = valid_count = non_ok_count = 0
    messages = {}
    for diagnostic in diagnostics:
        for status in diagnostic['statuses']:
            status_count += 1
            non_ok_count += int(status['level'] != 0)
            valid_count += int(status['values'].get('valid') == 'true')
            messages[status['message']] = messages.get(status['message'], 0) + 1
    positive = intervals[intervals > 0]
    median_interval = float(np.median(positive)) if len(positive) else math.nan
    return {
        'samples_normalized': len(normalized), 'samples_raw': len(raw), 'diagnostic_statuses': status_count,
        'valid_diagnostic_statuses': valid_count, 'non_ok_diagnostic_statuses': non_ok_count,
        'diagnostic_messages': messages, 'frame_ids': sorted({item['frame_id'] for item in normalized}),
        'duration_s': float((stamps[-1] - stamps[0]) / 1e9),
        'frequency_hz': float((len(stamps)-1) / ((stamps[-1] - stamps[0]) / 1e9)),
        'nonpositive_timestamp_steps': int(np.sum(intervals <= 0)),
        'inter_message_dt_ms_median': median_interval * 1000.0,
        'inter_message_dt_ms_p95': percentile(intervals, 95) * 1000.0,
        'inter_message_dt_ms_max': float(np.max(intervals)) * 1000.0,
        'intervals_gt_2x_median': int(np.sum(intervals > 2.0 * median_interval)) if math.isfinite(median_interval) else 0,
        'mean_x_m': float(center[0]), 'mean_y_m': float(center[1]), 'mean_z_m': float(center[2]),
        'std_x_mm': float(np.std(xyz[:, 0], ddof=1) * 1000.0),
        'std_y_mm': float(np.std(xyz[:, 1], ddof=1) * 1000.0),
        'std_z_mm': float(np.std(xyz[:, 2], ddof=1) * 1000.0),
        'xy_rms_mm': float(math.sqrt(float(np.mean(radial_xy_m**2))) * 1000.0),
        'xy_p95_mm': percentile(radial_xy_m, 95) * 1000.0,
        'xy_max_mm': float(np.max(radial_xy_m)) * 1000.0,
        'yaw_mean_deg': math.degrees(yaw_center),
        'yaw_std_deg': math.degrees(float(np.std(yaw_deviation, ddof=1))),
        'yaw_p95_abs_deg': math.degrees(percentile(np.abs(yaw_deviation), 95)),
        'yaw_max_abs_deg': math.degrees(float(np.max(np.abs(yaw_deviation)))),
        'raw_to_normalized_exact_stamp_pairs': len(conversion_residual_m),
        'raw_to_normalized_coverage': len(conversion_residual_m) / len(raw) if raw else None,
        'raw_to_normalized_max_residual_m': float(max(conversion_residual_m)) if conversion_residual_m else None,
        '_xy_deviation_mm': (radial_xy_m * 1000.0).tolist(),
        '_yaw_deviation_deg': np.degrees(yaw_deviation).tolist(),
    }


def label_for_bag(path):
    name = Path(path).parent.name
    parts = name.split('_')
    return (parts[-1] if parts else name) + '\n' + (parts[0][-6:] if parts else '')


def public_record(record):
    return {key: value for key, value in record.items() if not key.startswith('_')}


def write_summary_csv(path, records):
    rows = [public_record(record) for record in records]
    fields = sorted({key for row in rows for key in row})
    with Path(path).open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def save_plot(directory, records):
    labels = [record['label'] for record in records]
    fig, axes = plt.subplots(1, 2, figsize=(13, 5.3), constrained_layout=True)
    axes[0].boxplot([record['_xy_deviation_mm'] for record in records], labels=labels, showfliers=False)
    axes[0].set(title='Static planar position deviation', ylabel='Distance from per-record mean (mm)')
    axes[1].boxplot([record['_yaw_deviation_deg'] for record in records], labels=labels, showfliers=False)
    axes[1].set(title='Static heading deviation', ylabel='Yaw deviation from circular mean (deg)')
    for axis in axes:
        axis.grid(axis='y', alpha=.3)
    fig.suptitle('Tracker0 static-reference stability; each rosbag remains an independent record')
    for extension in ('png', 'svg'):
        fig.savefig(Path(directory) / ('static_stability.'+extension), dpi=180)
    plt.close(fig)


def write_markdown(path, records):
    lines = [
        '# Tracker0 静止参考稳定性报告', '',
        '该报告只刻画动捕参考链路的静止噪声与消息完整性，不是 NDT 定位误差。',
        '重复标签按独立 rosbag 保留；不能据此假定它们是不同物理位置。', '',
        '| 记录 | 时长 s | Hz | XY RMS / P95 / max mm | yaw std / P95 / max deg | 规范化覆盖率 |',
        '| --- | ---: | ---: | ---: | ---: | ---: |',
    ]
    for row in records:
        coverage = row['raw_to_normalized_coverage']
        coverage_text = 'n/a' if coverage is None else f'{coverage:.3%}'
        lines.append(
            f"| {row['run_id']} | {row['duration_s']:.3f} | {row['frequency_hz']:.3f} | "
            f"{row['xy_rms_mm']:.3f} / {row['xy_p95_mm']:.3f} / {row['xy_max_mm']:.3f} | "
            f"{row['yaw_std_deg']:.4f} / {row['yaw_p95_abs_deg']:.4f} / {row['yaw_max_abs_deg']:.4f} | {coverage_text} |")
    lines.extend(['', '解释：消息间隔超过两倍中位数仅报告调度/传输不均匀性，不能在无相机原始帧率信息时直接称为丢帧。',
                  '动态 NDT 误差比较仍须按点云测量时间匹配，且不得用该静止噪声从定位误差中相减。'])
    Path(path).write_text('\n'.join(lines)+'\n', encoding='utf-8')


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bags', nargs='+', type=Path, required=True, help='rosbag2 directories')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--raw-position-scale', type=float, default=.001)
    return parser.parse_args()


def main():
    args = parse_args()
    if not math.isfinite(args.raw_position_scale) or args.raw_position_scale <= 0.0:
        raise SystemExit('--raw-position-scale must be finite and positive')
    if args.output.exists():
        raise SystemExit(f'Output already exists: {args.output}')
    args.output.mkdir(parents=True)
    records = []
    for bag in args.bags:
        normalized, raw, diagnostics = read_bag(bag)
        record = analyze_samples(normalized, raw, diagnostics, args.raw_position_scale)
        record['run_id'] = Path(bag).parent.name
        record['bag'] = str(Path(bag).resolve())
        record['label'] = label_for_bag(bag)
        records.append(record)
    write_summary_csv(args.output / 'static_mocap_summary.csv', records)
    (args.output / 'static_mocap_summary.json').write_text(
        json.dumps([public_record(record) for record in records], ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    write_markdown(args.output / 'static_mocap_report.md', records)
    save_plot(args.output, records)
    print(f'Wrote {args.output} for {len(records)} bags')


if __name__ == '__main__':
    main()
