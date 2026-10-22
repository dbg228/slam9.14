#!/usr/bin/env python3
"""Audit saved Group 3 pairs under both recorded transforms; never refit."""
from pathlib import Path
import hashlib
import json
import math

import numpy as np
from scipy.spatial.transform import Rotation
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


OUT = Path(__file__).resolve().parent
ROOT = next(p for p in OUT.parents if (p / 'mapping_modes.sh').is_file())
TRIAL = ROOT / 'robot_data/experiments/full_map_loop/20261018_relocated_01/group3_mocap_correction/trial_01'


def read(path):
    text = Path(path).read_text()
    return json.loads(text) if Path(path).suffix == '.json' else yaml.safe_load(text)


def describe(path):
    path = Path(path).resolve()
    return dict(path=str(path), bytes=path.stat().st_size,
                sha256=hashlib.sha256(path.read_bytes()).hexdigest())


def main():
    manifest_path = TRIAL / 'data/manifest.json'
    correction_path = TRIAL / 'data/middle_correction.json'
    endpoint_path = TRIAL / 'data/endpoint_validation.json'
    manifest = read(manifest_path)
    correction = read(correction_path)
    run = Path(manifest['run'])
    calibration_path = run / 'calibration.yaml'
    checkpoint = Path(correction['correction_capture'])
    report_path = checkpoint / 'summary.json'
    report = read(report_path)
    h0 = np.array(report['T_map_odom_before']['matrix'], float)
    h1 = np.array(report['T_map_odom_after']['matrix'], float)
    c = np.array(read(calibration_path)['T_map_mocap']['matrix'], float)
    delta = h1 @ np.linalg.inv(h0)
    a, b = delta[:3, :3], delta[:3, 3]
    captures = {
        'initial': Path(manifest['initial_validation']),
        'middle_before': Path(correction['before_validation']),
        'middle_fit': checkpoint,
        'middle_after_same_station': Path(correction['after_validation']),
        'endpoint': Path(read(endpoint_path)['captures'][0]),
    }
    inputs = [manifest_path, correction_path, endpoint_path, calibration_path,
              run / 'calibration.lock.json', report_path, checkpoint / 'summary.lock.json']
    comparisons, means, stamps = {}, {}, {}
    for name, capture in captures.items():
        pair_path = capture / 'pairs.json'
        inputs += [pair_path, capture / 'summary.json']
        pairs = read(pair_path)
        f = np.array([p['T_F_B'] for p in pairs], float)
        m = np.array([p['T_M_B'] for p in pairs], float)
        truth = c @ m
        p0, p1 = h0 @ f, h1 @ f
        e0 = p0[:, :3, 3] - truth[:, :3, 3]
        e1 = p1[:, :3, 3] - truth[:, :3, 3]
        means[name] = dict(p0=p0[:, :3, 3].mean(0), p1=p1[:, :3, 3].mean(0),
                           truth=truth[:, :3, 3].mean(0), mocap=m[:, :3, 3].mean(0))
        stamps[name] = [p['stamp_ns'] for p in pairs]
        def score(error):
            mean = error.mean(0)
            return dict(xy_rmse_mm=float(np.sqrt(np.mean(np.sum(error[:, :2] ** 2, axis=1))) * 1000),
                        xyz_mean_error_mm=(mean * 1000).tolist(),
                        xy_mean_error_direction_deg=math.degrees(math.atan2(mean[1], mean[0])))
        comparisons[name] = dict(samples=len(pairs), source_first_ns=stamps[name][0],
                                source_last_ns=stamps[name][-1], H0=score(e0), H1=score(e1),
                                coordinate_change_xyz_mean_mm=((p1[:, :3, 3] - p0[:, :3, 3]).mean(0) * 1000).tolist())

    middle = means['middle_fit']
    arm = means['endpoint']['p0'] - middle['p0']
    middle_shift = middle['p1'] - middle['p0']
    rotational_effect = (a - np.eye(3)) @ arm
    endpoint_shift = means['endpoint']['p1'] - means['endpoint']['p0']
    assert np.allclose(middle_shift + rotational_effect, endpoint_shift, atol=1e-12)
    assert abs(comparisons['middle_before']['H0']['xy_rmse_mm'] - 218.36742223083675) < 1e-8
    assert abs(comparisons['middle_before']['H1']['xy_rmse_mm'] - 2.324580732148129) < 1e-8
    assert abs(comparisons['endpoint']['H0']['xy_rmse_mm'] - 27.384030971026966) < 1e-8
    assert abs(comparisons['endpoint']['H1']['xy_rmse_mm'] - 176.94507223104782) < 1e-8
    assert not set(stamps['middle_fit']).intersection(stamps['middle_after_same_station'])
    summary = dict(
        audit_scope='Group 3 saved-pose transform comparison, attached to the three-group review',
        data_kind='real_recorded_static_pose_pairs_offline_audit',
        transform_refitted=False, original_trial_files_modified=False,
        formula={
            'frames': 'F=T_F_B is native odom -> base_link; M=T_M_B is mocap world -> base_link; C=T_map_mocap',
            'errors': 'e_k(i) = translation(Hk @ F_i) - translation(C @ M_i), k in {0,1}',
            'xy_rmse': '1000 * sqrt(mean(e_x^2 + e_y^2)) [mm]',
            'relative_update': 'D = H1 @ inverse(H0) = [A,b]',
            'middle_pivot_decomposition': 'p1 = p0 + d_middle + (A-I) @ (p0-p_middle), where d_middle=(A-I)@p_middle+b',
            'direction': 'atan2(mean e_y, mean e_x), degrees in map XY',
        },
        transforms=dict(H0_map_odom=h0.tolist(), H1_map_odom=h1.tolist(),
                        D_H1_times_inverse_H0=delta.tolist(),
                        rotation_magnitude_deg=float(np.degrees(Rotation.from_matrix(a).magnitude())),
                        rotation_vector_deg=np.degrees(Rotation.from_matrix(a).as_rotvec()).tolist(),
                        euler_xyz_roll_pitch_yaw_deg=Rotation.from_matrix(a).as_euler('xyz', degrees=True).tolist(),
                        translation_at_map_origin_xyz_mm=(b * 1000).tolist(),
                        translation_at_map_origin_xy_mm=float(np.linalg.norm(b[:2]) * 1000),
                        translation_at_map_origin_3d_mm=float(np.linalg.norm(b) * 1000)),
        comparisons=comparisons,
        middle_pivot_endpoint_decomposition=dict(
            middle_mean_old_map_xyz_m=middle['p0'].tolist(),
            middle_coordinate_change_xyz_mm=(middle_shift * 1000).tolist(),
            endpoint_minus_middle_old_map_xyz_m=arm.tolist(),
            endpoint_distance_from_middle_m=float(np.linalg.norm(arm)),
            endpoint_rotation_term_xyz_mm=(rotational_effect * 1000).tolist(),
            endpoint_rotation_term_xy_mm=float(np.linalg.norm(rotational_effect[:2]) * 1000),
            endpoint_full_coordinate_change_xyz_mm=(endpoint_shift * 1000).tolist(),
            endpoint_full_coordinate_change_xy_mm=float(np.linalg.norm(endpoint_shift[:2]) * 1000)),
        heldout_after_scope=dict(
            fit_after_source_stamp_overlap=0,
            seconds_from_fit_end_to_after_start=(stamps['middle_after_same_station'][0] - stamps['middle_fit'][-1]) * 1e-9,
            mean_mocap_station_movement_xyz_mm=((means['middle_after_same_station']['mocap'] - middle['mocap']) * 1000).tolist(),
            mean_mocap_station_movement_3d_mm=float(np.linalg.norm(means['middle_after_same_station']['mocap'] - middle['mocap']) * 1000),
            interpretation='Different samples and a different time window at the same station. Demonstrates local temporal holdout residual; not spatial holdout absolute accuracy.'),
        reference_quality=dict(independent_accuracy_verified=report['independent_accuracy_verified'],
                               clock_offset_verified=report['clock_offset_verified'], camera_latency_s=report['camera_latency_s']),
        limitations=[
            'H0 endpoint comparison uses the same endpoint measurements recorded after actual navigation with H1. It is not an actual uncorrected re-run or an uncorrected control outcome.',
            'No new transform, trajectory alignment, calibration, or error model was fitted. Both H0 and H1 were already saved during the real experiment.',
            'This comparison proves the coordinate update changes the endpoint residual; it cannot alone distinguish native localization error, spatial calibration inconsistency, and rigid-body/extrinsic bias.',
            'The total 1.519 degree rotation is mostly pitch and roll. It must not be described as 1.519 degrees of planar yaw.',
            'Actual trial included approximately 1383 seconds of stationary pause, a 3-second native age allowance and 0.10 m/s continuation limit. It is not a same-speed controlled comparison with previous groups.',
            'Current mocap reference and cross-device exposure timing have not been independently certified as absolute ground truth.',
        ],
        input_files=[describe(p) for p in dict.fromkeys(inputs)],
    )
    (OUT / 'audit_summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False) + '\n')

    fig, ax = plt.subplots(figsize=(11.8, 7.6))
    xx = np.array([0., 1.])
    labels = ['Middle: same recorded BEFORE samples', 'Endpoint: same recorded endpoint samples']
    old = [comparisons['middle_before']['H0']['xy_rmse_mm'], comparisons['endpoint']['H0']['xy_rmse_mm']]
    new = [comparisons['middle_before']['H1']['xy_rmse_mm'], comparisons['endpoint']['H1']['xy_rmse_mm']]
    for offset, values, label, color in [(-.18, old, 'H0: saved original map-to-odom', '#4267ac'),
                                          (.18, new, 'H1: saved middle correction', '#db7a37')]:
        bars = ax.bar(xx + offset, values, width=.34, label=label, color=color)
        for bar, value in zip(bars, values):
            ax.text(bar.get_x() + bar.get_width() / 2, value + 3, f'{value:.3f}', ha='center', fontsize=11)
    ax.set_xticks(xx, labels)
    ax.set(ylabel='XY residual RMSE against the current mocap reference [mm]', ylim=(0, 255))
    ax.grid(axis='y', alpha=.25)
    ax.set_axisbelow(True)
    ax.legend(loc='upper right', fontsize=10)
    fig.suptitle('Group 3 audit: same recorded poses, offline transform comparison\nNot an uncorrected re-run; no transform refit', fontsize=15)
    fig.text(.04, .01,
             'Actual held-out AFTER window: 2.308 mm, same station, different samples (13.0 s after fit).\n'
             'Full update: roll -0.539°, pitch +1.414°, yaw -0.136°; 7.0 m rotation propagation gives only 15.803 mm XY.\n'
             'H0 endpoint value is an offline counterfactual for the actual H1-driven run. Absolute truth is not independently certified.',
             fontsize=9)
    fig.subplots_adjust(left=.09, right=.98, bottom=.23, top=.84)
    for suffix in ('png', 'svg'):
        fig.savefig(OUT / f'group3_saved_transform_comparison.{suffix}', dpi=180)
    plt.close(fig)
    print(json.dumps(dict(output=str(OUT), middle_H0_mm=old[0], middle_H1_mm=new[0],
                          endpoint_H0_mm=old[1], endpoint_H1_mm=new[1]), indent=2))


if __name__ == '__main__':
    main()
