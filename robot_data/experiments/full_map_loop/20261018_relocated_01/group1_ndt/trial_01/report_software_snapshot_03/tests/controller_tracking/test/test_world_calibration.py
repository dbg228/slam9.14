"""Known 3-D transforms, locked holdouts and passive capture data quality."""
import copy
import json
import math
from pathlib import Path

import numpy as np
import pytest
import yaml

from controller_tracking.state import rotation
from controller_tracking import world_calibration as wc
from controller_tracking.world_capture import summarize, check_raw_normalized

ROOT = Path(__file__).resolve().parents[3]
MOCAP_CONFIG = ROOT/'robot_data/config/mocap_tracker0.yaml'


def session_fixture(directory):
    wc.init_session(directory, MOCAP_CONFIG, 'manual')
    cfg = wc.load(directory/'session.yaml')
    cfg.update(data_kind='software_fixture', calibration_id='synthetic_camera',
               map_reference_id='synthetic_map', physical_reference_confirmed=True,
               map_method='known synthetic transform')
    (directory/'session.yaml').write_text(yaml.safe_dump(cfg))
    wc.freeze_inputs(directory)
    return cfg


def observation(directory, group, source, target, angle=0., fail=False):
    destination = directory/group; destination.mkdir()
    wc.save(destination/'raw.jsonl', dict(data_kind='software_fixture', point=source))
    wc.save(destination/'summary.json', dict(group=group, inputs_sha256=wc.sha(directory/'frozen_inputs.json'),
        raw_sha256=wc.sha(destination/'raw.jsonl'), quality_passed=not fail,
        mocap_xyz_m=list(source), map_xyz_m=None,
        mocap_base_rotation_matrix=rotation([0, 0, math.sin(angle/2), math.cos(angle/2)]).tolist(),
        calibration_sha256_at_capture=wc.sha(directory/'calibration.yaml') if group==wc.HOLDOUT else None))
    input_path = directory/f'{group}_input.yaml'
    wc.save(input_path, dict(group=group, frame='map', reference_point='base_link_origin',
        unit='metre', xyz_m=list(target), method='synthetic truth', independently_measured=True), True)
    wc.enter_map(directory, group, input_path)


def known_transform():
    r = rotation([.12, -.21, .31, .90])
    return r, np.array([2., -3., .4])


def test_full_3d_fit_direction_inverse_and_origins():
    r, t = known_transform()
    points = np.array([[0., 0., .2], [1., 0., .2], [.2, .8, .2]])
    fit_r, fit_t, checks = wc.fit_rigid(points, points @ r.T+t)
    assert np.allclose(fit_r, r, atol=1e-12)
    assert np.allclose(fit_t, t, atol=1e-12)
    assert abs(np.linalg.det(fit_r)-1.) < 1e-12
    assert np.allclose((points @ fit_r.T+fit_t-fit_t) @ fit_r, points)
    assert np.allclose((-r.T @ t) @ r.T+t, [0, 0, 0])
    assert checks['mocap']['minimum_altitude_m'] > .5


@pytest.mark.parametrize('points', [np.zeros((3, 3)), [[0, 0, 0], [1, 0, 0], [2, 0, 0]],
                                  [[0, 0, 0], [1, 0, 0], [1, .001, 0]]])
def test_degenerate_geometry_rejected(points):
    with pytest.raises(ValueError, match='collinear'):
        wc.fit_rigid(points, points)


def test_scale_is_not_fitted_and_bad_correspondence_retained():
    source = np.array([[0., 0., 0.], [1., 0., 0.], [0., 1., 0.]])
    target = 1.2*source
    r, t, checks = wc.fit_rigid(source, target)
    errors = wc.error_report(source, target, r, t, wc.TRAIN)
    assert errors['distance_3d_m']['rmse'] > .1
    assert max(checks['edge_length_difference_map_minus_mocap_m']) > .19
    assert np.allclose(r.T @ r, np.eye(3))


def test_end_to_end_holdout_error_survives_and_candidate_is_immutable(tmp_path):
    s = tmp_path/'session'; session_fixture(s)
    r, t = known_transform()
    source = np.array([[0., 0., .2], [1., 0., .2], [.2, .8, .2]])
    for group, p in zip(wc.TRAIN, source): observation(s, group, p.tolist(), (r @ p+t).tolist())
    result = wc.solve(s); frozen = (s/'calibration.yaml').read_bytes()
    assert result['training_point_count'] == 3 and not result['holdout_used']
    assert result['errors']['distance_3d_m']['rmse'] < 1e-12
    test = np.array([.6, .5, .2]); offset = np.array([.03, -.04, .12])
    observation(s, wc.HOLDOUT, test.tolist(), (r @ test+t-offset).tolist(), angle=.6)
    verified = wc.verify(s)
    assert np.allclose(verified['errors']['points'][0]['error_xyz_m'], offset)
    assert verified['errors']['distance_3d_m']['rmse'] == pytest.approx(.13)
    assert verified['errors']['distance_xy_m']['rmse'] == pytest.approx(.05)
    assert verified['protocol_passed'] and not verified['transform_refit']
    assert verified['numeric_acceptance_passed'] is None
    assert (s/'calibration.yaml').read_bytes() == frozen
    assert (s/'validation_report.md').exists()
    with pytest.raises(ValueError, match='Solve exactly once'): wc.solve(s)
    with pytest.raises(FileExistsError): wc.verify(s)


def test_cannot_solve_after_holdout_collection_or_with_failed_training(tmp_path):
    s = tmp_path/'session'; session_fixture(s)
    (s/wc.HOLDOUT).mkdir()
    with pytest.raises(ValueError, match='BEFORE'): wc.solve(s)
    (s/wc.HOLDOUT).rmdir()
    observation(s, wc.TRAIN[0], [0, 0, 0], [1, 1, 1], fail=True)
    with pytest.raises(ValueError, match='quality failed'): wc.solve(s)


def test_mount_or_input_changes_are_rejected(tmp_path):
    s = tmp_path/'session'; session_fixture(s)
    body = wc.load(s/'body_extrinsic.yaml')
    body['rigid_body_to_base_link']['translation_m'][2] += .01
    (s/'body_extrinsic.yaml').write_text(yaml.safe_dump(body))
    with pytest.raises(ValueError, match='changed'): wc.frozen_inputs(s)


def capture_fixture():
    cfg = dict(map_source='ndt', map_method='NDT synthetic', mocap_frame='mocap_world', map_frame='map',
        clock_offset_s=0., clock_offset_verified=False, camera_latency_s=None,
        ndt_pose_field='final_map_to_base_at_cloud', topics=dict(ndt='/synthetic_ndt'),
        limits=dict(seconds=4., minimum_mocap_samples=30, minimum_pairs=3,
            maximum_source_age_s=.5, maximum_mocap_gap_s=.15, maximum_ndt_gap_s=2.,
            maximum_mocap_radius_m=.01, maximum_map_radius_m=.05,
            maximum_orientation_deviation_deg=2., maximum_pair_skew_s=.05,
            maximum_cloud_odom_skew_s=.05, maximum_command_abs=1e-4))
    mount = dict(translation_m=[.1, .2, -.18], quaternion_xyzw=[0, 0, 0, 1])
    poses = [dict(stamp_ns=10**9+i*10_000_000, receive_ros_ns=10**9+i*10_000_000+1_000_000,
                  frame='mocap_world', xyz_m=[1., 2., .18], quaternion_xyzw=[0, 0, 1., 0]) for i in range(401)]
    ndt = [dict(stamp_ns=10**9+i*10_000_000, receive_ros_ns=10**9+i*10_000_000+2_000_000,
                match_id=i, valid=True, frame='map', sync_valid=True, cloud_odom_skew_s=.001,
                pose=dict(xyz_m=[.9, 1.8, 0.], quaternion_xyzw=[0, 0, 1., 0]),
                accepted=True, stage='accepted', map_ready=True) for i in (0, 100, 200, 300, 400)]
    return cfg, mount, poses, ndt


def test_capture_applies_rotated_mount_matches_same_times_and_wrap_safe():
    cfg, mount, poses, ndt = capture_fixture()
    # q and -q encode the same orientation, including the pi boundary.
    poses[200]['quaternion_xyzw'] = [0, 0, -1., 0]
    result = summarize(cfg, mount, poses, ndt, [[0]*6])
    assert result['quality_passed']
    assert np.allclose(result['mocap_xyz_m'], [.9, 1.8, 0.])
    assert result['pair_count'] == 5 and result['mocap_span_s'] == pytest.approx(4.)
    assert not result['interpolation_used']


def test_capture_dropouts_stale_nonzero_commands_and_nonfinite_are_not_hidden():
    cfg, mount, poses, ndt = capture_fixture()
    gap = poses[:100]+poses[150:]
    assert 'long_mocap_gap' in summarize(cfg, mount, gap, ndt, [[0]*6])['failures']
    stale = copy.deepcopy(ndt)
    for p in stale: p['receive_ros_ns'] += 10**9
    assert 'insufficient_ndt_mocap_pairs' in summarize(cfg, mount, poses, stale, [[0]*6])['failures']
    assert 'nonzero_final_command' in summarize(cfg, mount, poses, ndt, [[.1]*6])['failures']
    assert 'no_observed_final_command' in summarize(cfg, mount, poses, ndt, [])['failures']
    assert not summarize(cfg, mount, poses, ndt, [[0]*6], invalid_events=1)['quality_passed']
    poses[-1]['xyz_m'][0] = float('nan')
    result = summarize(cfg, mount, poses, ndt, [[0]*6])
    assert result['rejection_counts'] and result['valid_mocap_samples'] == 400


def test_ndt_drift_is_preserved_not_used_to_select_best_samples():
    cfg, mount, poses, ndt = capture_fixture()
    for i, p in enumerate(ndt): p['pose']['xyz_m'][0] += .01*i
    result = summarize(cfg, mount, poses, ndt, [[0]*6])
    assert result['quality_passed']
    assert result['map_xyz_m'][0] == pytest.approx(.92)
    assert result['pair_count'] == 5
    assert result['map_provenance']['independently_measured'] is False


def test_evaluation_retains_large_estimator_jitter_as_error_not_bad_truth():
    cfg,mount,poses,ndt=capture_fixture()
    for i,p in enumerate(ndt):p['pose']['xyz_m'][0]+=.2*i
    assert 'map_not_static' in summarize(cfg,mount,poses,ndt,[[0]*6])['failures']
    evaluated=summarize(cfg,mount,poses,ndt,[[0]*6],evaluation_only=True)
    assert evaluated['quality_passed']
    assert evaluated['pair_count']==len(ndt)
    assert 'map_not_static' in evaluated['estimator_instability_warnings']
    assert evaluated['map_xyz_m'][0]==pytest.approx(1.3)


def test_actual_raw_unit_and_timestamp_check():
    _, _, normalized, _ = capture_fixture()
    raw = copy.deepcopy(normalized)
    for p in raw: p['xyz_m'] = [v*1000 for v in p['xyz_m']]
    check = check_raw_normalized(raw, normalized, .001)
    assert check['coverage'] == 1. and not check['mismatched_stamp_ns']
    wrong = check_raw_normalized(raw, normalized, 1.)
    assert len(wrong['mismatched_stamp_ns']) == len(normalized)
    assert check_raw_normalized([], normalized, .001)['coverage'] == 0.


def test_delayed_static_ndt_pairs_need_real_preroll_not_retiming():
    cfg, mount, poses, template = capture_fixture()
    cfg['limits']['maximum_source_age_s'] = 2.
    records = []
    for i in (0, 150, 300):
        record = copy.deepcopy(template[0])
        record.update(stamp_ns=10**9 + i*10_000_000,
                      receive_ros_ns=10**9 + i*10_000_000 + 1_400_000_000,
                      match_id=i)
        records.append(record)
    result = summarize(cfg, mount, poses, records, [[0]*6])
    assert result['quality_passed'] and result['pair_count'] == 3
    assert [p['ndt_stamp_ns'] for p in result['matched_pairs']] == [r['stamp_ns'] for r in records]
    assert result['matched_pairs'][0]['mocap_stamp_ns'] == records[0]['stamp_ns']
    without_preroll = [p for p in poses if p['stamp_ns'] >= 2*10**9]
    missing = summarize(cfg, mount, without_preroll, records, [[0]*6])
    assert missing['rejection_counts']['no_nearby_valid_mocap'] == 1
    assert 'insufficient_ndt_mocap_pairs' in missing['failures']


def test_static_ndt_age_bound_does_not_relax_mocap_age_or_change_stamps():
    cfg,mount,poses,ndt=capture_fixture()
    for p in ndt:p['receive_ros_ns']=p['stamp_ns']+2_100_000_000
    assert not summarize(cfg,mount,poses,ndt,[[0]*6])['quality_passed']
    cfg['limits']['maximum_ndt_source_age_s']=3.
    result=summarize(cfg,mount,poses,ndt,[[0]*6],evaluation_only=True)
    assert result['quality_passed'] and result['pair_count']==len(ndt)
    assert [p['ndt_stamp_ns'] for p in result['matched_pairs']]==[p['stamp_ns'] for p in ndt]
    for p in poses:p['receive_ros_ns']+=10**9
    rejected=summarize(cfg,mount,poses,ndt,[[0]*6],evaluation_only=True)
    assert not rejected['quality_passed'] and 'stale_mocap' in rejected['rejection_counts']


def test_delayed_evaluation_requires_same_time_static_truth_not_receive_freshness():
    cfg,mount,poses,ndt=capture_fixture();cfg['static_history_pairing']=True
    for p in ndt:p['receive_ros_ns']=p['stamp_ns']+6_000_000_000
    assert not summarize(cfg,mount,poses,ndt,[[0]*6])['quality_passed']
    evaluated=summarize(cfg,mount,poses,ndt,[[0]*6],evaluation_only=True)
    assert evaluated['quality_passed'] and evaluated['pair_count']==5
    assert all(p['ndt_receive_age_s']==6. for p in evaluated['matched_pairs'])
    missing=summarize(cfg,mount,poses[100:],ndt,[[0]*6],evaluation_only=True)
    assert missing['rejection_counts']['no_nearby_valid_mocap']==1
    moved=copy.deepcopy(poses);moved[-1]['xyz_m'][0]+=.5
    assert 'mocap_not_static' in summarize(cfg,mount,moved,ndt,[[0]*6],evaluation_only=True)['failures']


def test_nonfinite_command_does_not_pass_as_stationary():
    cfg, mount, poses, ndt = capture_fixture()
    result = summarize(cfg, mount, poses, ndt, [[float('nan')]*6])
    assert not result['quality_passed'] and result['maximum_command_abs'] is None


@pytest.mark.parametrize('ready,stage', [(False, 'accepted'), (True, 'waiting_for_initialization')])
def test_valid_pose_flag_is_insufficient_without_initialized_map(ready, stage):
    cfg, mount, poses, ndt = capture_fixture()
    for record in ndt: record.update(map_ready=ready, stage=stage)
    result = summarize(cfg, mount, poses, ndt, [[0]*6])
    assert not result['quality_passed'] and result['pair_count']==0
    assert 'ndt_not_ready_during_capture' in result['failures']
