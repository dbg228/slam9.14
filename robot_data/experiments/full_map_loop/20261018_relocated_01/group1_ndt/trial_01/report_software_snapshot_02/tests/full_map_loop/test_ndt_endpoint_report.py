"""Known truth checks for frozen-frame errors and accumulated loop distance."""
import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest

SPEC = importlib.util.spec_from_file_location('ndt_endpoint_report', Path(__file__).with_name('ndt_endpoint_report.py'))
report = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(report)


def rz(degrees):
    a = math.radians(degrees)
    return np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])


def test_frozen_transform_preserves_known_initial_bias_and_wraps_heading():
    transform = np.eye(4)
    transform[:3, :3] = rz(30)
    transform[:3, 3] = [3, -2, .2]
    original = transform.copy()
    position = np.array([.4, -.8, .1])
    pair = dict(ndt_stamp_ns=10**9, mocap_xyz_m=position.tolist(),
        map_xyz_m=(transform[:3, :3] @ position + transform[:3, 3] + [.03, -.04, .12]).tolist(),
        mocap_rotation=rz(149).tolist(), map_rotation=rz(-179).tolist())
    result = report.score_pairs([pair], transform)
    assert result['xy_m']['rmse'] == pytest.approx(.05)
    assert result['distance_3d_m']['rmse'] == pytest.approx(.13)
    assert result['rows'][0]['yaw_deg'] == pytest.approx(2)
    assert not result['transform_refitted']
    assert np.array_equal(transform, original)


def test_loop_distance_accumulates_travel_despite_zero_displacement():
    samples = [dict(stamp_ns=i * 10**8, x=x, y=y)
        for i, (x, y) in enumerate([(0, 0), (1, 0), (1, 1), (0, 1), (0, 0)])]
    # Repeated TF delivery must not double-count a step.
    result = report.distance_series(samples + [samples[2]], 0, 4 * 10**8)
    assert result['complete']
    assert result['estimated_total_m'] == pytest.approx(4)
    assert result['initial_to_final_displacement_m'] == 0
    assert result['duplicate_stamps'] == 1


def test_missing_interval_is_not_reported_as_a_complete_route():
    samples = [dict(stamp_ns=t, x=x, y=0) for t, x in
        [(0, 0), (10**8, 1), (2 * 10**9, 100), (21 * 10**8, 101)]]
    result = report.distance_series(samples, 0, 21 * 10**8)
    assert result['estimated_total_m'] is None
    assert result['observed_segments_distance_m'] == pytest.approx(2)
    assert len(result['gaps']) == 1
    assert not report.distance_series(samples[1:], 0, 4 * 10**9)['complete']


def test_bad_distance_input_cannot_silently_produce_a_number():
    with pytest.raises(ValueError, match='Nonfinite'):
        report.distance_series([dict(stamp_ns=1, x=float('nan'), y=0)], 0, 2)
    with pytest.raises(ValueError, match='end after'):
        report.distance_series([], 2, 1)


def test_bag_reader_excludes_global_ndt_correction_jumps(tmp_path):
    # Real ROS message serialization in a tiny software-only SQLite bag.
    serialization = pytest.importorskip('rclpy.serialization')
    tf_messages = pytest.importorskip('tf2_msgs.msg')
    geometry = pytest.importorskip('geometry_msgs.msg')
    import sqlite3
    with sqlite3.connect(tmp_path / 'fixture.db3') as db:
        db.execute('CREATE TABLE topics (id INTEGER, name TEXT)')
        db.execute('CREATE TABLE messages (topic_id INTEGER, timestamp INTEGER, data BLOB)')
        db.execute("INSERT INTO topics VALUES (1, '/tf')")
        for i, x in enumerate([0., 1., 0.]):
            transforms = []
            for parent, child, position in [('odom', 'base_link', x), ('map', 'odom', 1000. * i)]:
                t = geometry.TransformStamped()
                t.header.stamp.sec = 1
                t.header.stamp.nanosec = i * 10**8
                t.header.frame_id = parent
                t.child_frame_id = child
                t.transform.translation.x = position
                t.transform.rotation.w = 1.
                transforms.append(t)
            message = tf_messages.TFMessage(transforms=transforms)
            db.execute('INSERT INTO messages VALUES (1, ?, ?)',
                (2 * 10**9 + i * 10**8, serialization.serialize_message(message)))
    result = report.read_distance([tmp_path], 10**9, 12 * 10**8)
    assert result['complete']
    assert result['estimated_total_m'] == pytest.approx(2)


def test_report_exports_real_sample_curves_with_frozen_registration(tmp_path):
    from argparse import Namespace
    import copy
    wc = report.wc
    source = tmp_path/'registration'
    wc.init_session(source, report.ROOT/'robot_data/config/mocap_tracker0.yaml', 'ndt')
    cfg = wc.load(source/'session.yaml')
    cfg.update(data_kind='software_fixture', calibration_id='fixture_world',
        map_reference_id='fixture_map', physical_reference_confirmed=True)
    (source/'session.yaml').unlink()
    wc.save(source/'session.yaml', cfg, True)
    wc.freeze_inputs(source)
    for group, p in zip(wc.TRAIN, [[0., 0., .1], [1., 0., .1], [0., 1., .1]]):
        (source/group).mkdir()
        wc.save(source/group/'raw.jsonl', {'data_kind': 'software_fixture'})
        wc.save(source/group/'summary.json', dict(group=group, quality_passed=True,
            inputs_sha256=wc.sha(source/'frozen_inputs.json'), raw_sha256=wc.sha(source/group/'raw.jsonl'),
            mocap_xyz_m=p, map_xyz_m=(np.array(p)+[2., -1., .2]).tolist(),
            map_provenance={'independently_measured': False}))
    wc.solve(source)
    fixed_bytes = (source/'calibration.yaml').read_bytes()
    captures = []
    for phase, first_stamp in [('initial', 10**9), ('endpoint', 20*10**9)]:
        s = tmp_path/phase
        s.mkdir()
        local_cfg = copy.deepcopy(cfg)
        local_cfg.update(evaluation_only=True, fixed_calibration_session=str(source))
        wc.save(s/'session.yaml', local_cfg, True)
        wc.save(s/'body_extrinsic.yaml', wc.load(source/'body_extrinsic.yaml'), True)
        wc.freeze_inputs(s)
        (s/phase).mkdir()
        wc.save(s/phase/'raw.jsonl', {'data_kind': 'software_fixture'})
        pairs = [dict(ndt_stamp_ns=first_stamp+i*10**9, mocap_xyz_m=[0., 0., .1],
            map_xyz_m=[2.03, -1.04, .3], mocap_rotation=np.eye(3).tolist(),
            map_rotation=np.eye(3).tolist(), match_id=i) for i in range(3)]
        summary = s/phase/'summary.json'
        wc.save(summary, dict(group=phase, quality_passed=True, evaluation_only=True,
            inputs_sha256=wc.sha(s/'frozen_inputs.json'), raw_sha256=wc.sha(s/phase/'raw.jsonl'),
            calibration_sha256_at_capture=wc.sha(source/'calibration.yaml'), matched_pairs=pairs))
        captures.append(summary)
    output = tmp_path/'report'
    result = report.report(Namespace(calibration_session=source, initial=captures[0],
        endpoint=captures[1], bags=[], motion_start_ns=None, motion_end_ns=None, output=output))
    assert result['initial_xy_rmse_m'] == pytest.approx(.05)
    assert result['endpoint_xy_rmse_m'] == pytest.approx(.05)
    assert (source/'calibration.yaml').read_bytes() == fixed_bytes
    assert all((output/f'initial_endpoint_errors.{ext}').stat().st_size > 1000 for ext in ('png', 'svg', 'pdf'))
    assert not wc.load(output/'summary.json')['independent_absolute_accuracy_certified']
    assert not wc.load(output/'summary.json')['location_coverage']['initial']['spatially_held_out']
    assert '位置不独立于标定点' in (output/'report.md').read_text()
    assert len((output/'initial_error_samples.csv').read_text().splitlines())==4
    assert len((output/'endpoint_error_samples.csv').read_text().splitlines())==4
