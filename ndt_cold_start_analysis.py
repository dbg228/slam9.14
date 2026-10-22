#!/usr/bin/env python3
"""Summarize the first NDT registration from each deliberately cold-start bag.

The input bags must all have been collected at one unchanged physical parking
position after fully stopping and restarting the static localizer.  This tool
retains first-match failures and reports first-result pose scatter in map
coordinates for relative repeatability.
"""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from ndt_cold_start_core import first_accepted_registration, first_registration, summarize_cold_start_trials
from ndt_match_evaluation import load_rosbag
from ndt_match_evaluation_core import yaw


def clean_json(value):
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {key: clean_json(item) for key, item in value.items()}
    if isinstance(value, list):
        return [clean_json(item) for item in value]
    return value


def compact_pose(record, prefix, row):
    pose = record.get('raw_pose') if record else None
    for key in ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'):
        row[prefix+'_'+key] = '' if pose is None else pose.get(key, '')
    if pose is not None:
        try:
            row[prefix+'_yaw_deg'] = math.degrees(yaw(pose))
        except ValueError:
            row[prefix+'_yaw_deg'] = ''
    else:
        row[prefix+'_yaw_deg'] = ''


def flatten_record(bag, role, record):
    row = {'bag': str(Path(bag).resolve()), 'role': role}
    keys = ('match_id', 'stamp_ns', 'stage', 'decision_reason', 'accepted', 'converged', 'iterations',
            'fitness_valid', 'fitness', 'update_translation_m', 'update_yaw_rad',
            'init_to_ndt_translation_m', 'init_to_ndt_yaw_rad', 'raw_valid', 'initial_valid',
            'diagnostic_detail_available', 'configured_ndt_max_iterations',
            'configured_ndt_transformation_epsilon', 'configured_ndt_step_size',
            'configured_ndt_resolution', 'ndt_align_duration_s')
    for key in keys:
        row[key] = '' if record is None else record.get(key, '')
    compact_pose(record, 'raw', row)
    return row


def save_scatter(output, first_records):
    usable = [record for record in first_records
              if record and bool(record.get('raw_valid')) and isinstance(record.get('raw_pose'), dict)]
    for suffix in ('png', 'svg'):
        fig, axis = plt.subplots(figsize=(7, 6), constrained_layout=True)
        if usable:
            xy = np.asarray([[record['raw_pose']['x'], record['raw_pose']['y']] for record in usable], dtype=float)
            center = np.mean(xy, axis=0)
            axis.scatter(xy[:, 0], xy[:, 1], color='tab:blue', label='First raw NDT per restart')
            axis.scatter([center[0]], [center[1]], marker='x', s=100, color='tab:red', label='Centroid')
            for index, point in enumerate(xy, 1):
                axis.annotate(str(index), point, xytext=(4, 4), textcoords='offset points')
            axis.set(xlabel='Map X (m)', ylabel='Map Y (m)',
                     title='Cold-start first-match repeatability (not absolute truth)')
            axis.axis('equal'); axis.legend()
        else:
            axis.text(.5, .5, 'No first raw NDT outputs available', ha='center', va='center', transform=axis.transAxes)
        axis.grid(alpha=.3)
        fig.savefig(output/('cold_start_xy_scatter.'+suffix), dpi=180)
        plt.close(fig)


def write_summary_text(path, summary):
    repeatability = summary['first_registration_raw_pose_repeatability']
    lines = [
        'Cold-start first-NDT summary',
        'IMPORTANT: clean/cold start is guaranteed by the collection procedure, not inferred from a bag.',
        'IMPORTANT: pose scatter about its own centroid is a relative repeatability metric.',
        '',
        f"Trials supplied: {summary['trial_count']}",
        f"Trials with a real NDT registration: {summary['trials_with_registration']}",
        f"First-registration acceptance: {summary['first_registration_accepted_count']} / "
        f"{summary['trials_with_registration']} ({summary['first_registration_accepted_rate']:.1%})",
        f"First-registration convergence: {summary['first_registration_converged_count']} / "
        f"{summary['trials_with_registration']}",
        '', 'First raw-NDT pose repeatability:', json.dumps(repeatability, ensure_ascii=False, indent=2),
    ]
    Path(path).write_text('\n'.join(lines)+'\n', encoding='utf-8')


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, action='append', required=True,
                        help='One rosbag2 directory per full localizer restart; repeat this option.')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--ndt-topic', default='/static_localization/ndt_match_diagnostic_detail')
    parser.add_argument('--mocap-topic', default='/mocap/Tracker0/pose',
                        help='Required only to prove the bag contains the recorded reference stream.')
    return parser.parse_args()


def main():
    args = parse_args()
    if args.output.exists():
        raise SystemExit('Output already exists: '+str(args.output))
    trials, rows = [], []
    for bag in args.bag:
        diagnostics, _ = load_rosbag(bag, args.ndt_topic, args.mocap_topic)
        first = first_registration(diagnostics)
        accepted = first_accepted_registration(diagnostics)
        trials.append({'bag': str(bag), 'first_registration': first, 'first_accepted': accepted})
        rows.append(flatten_record(bag, 'first_registration', first))
        rows.append(flatten_record(bag, 'first_accepted', accepted))
    summary = summarize_cold_start_trials(trials)
    summary.update({
        'schema_version': 1,
        'created_utc': datetime.now(timezone.utc).isoformat(),
        'ndt_topic': args.ndt_topic,
        'mocap_topic_present_in_each_bag': args.mocap_topic,
        'metric_scope': 'relative cold-start repeatability only; no absolute-error evaluation is implemented',
        'collection_requirement': (
            'Each bag must begin before the localizer is started; each trial must fully stop the '
            'localizer before the next trial; all trials must use the same stationary physical position.'),
    })
    args.output.mkdir(parents=True)
    fields = sorted({key for row in rows for key in row})
    with (args.output/'first_matches.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader(); writer.writerows(rows)
    (args.output/'summary.json').write_text(json.dumps(clean_json(summary), ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    write_summary_text(args.output/'summary.txt', summary)
    save_scatter(args.output, [trial['first_registration'] for trial in trials])
    print(f'Wrote {args.output}; first registrations={summary["trials_with_registration"]}/'
          f'{summary["trial_count"]}, accepted={summary["first_registration_accepted_rate"]:.1%}')


if __name__ == '__main__':
    main()
