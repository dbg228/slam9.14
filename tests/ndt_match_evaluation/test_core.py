"""Synthetic checks for NDT timing, rigid offsets, and angle handling."""
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

from ndt_match_evaluation import (
    apply_tracker_to_base, evaluate, first_initial_alignment, first_raw_ndt_alignment, save_plots,
    tracker_to_base_from_config, write_csv, write_summary_text,
)
from ndt_match_evaluation_core import estimate_start_alignment, interpolate_pose, pose_error, rigid_body_to_base_pose, wrap, yaw_quaternion


def pose(stamp, x, y, heading):
    qx, qy, qz, qw = yaw_quaternion(heading)
    return {'stamp_ns': stamp, 'valid': True, 'x': x, 'y': y, 'z': 0., 'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw}


def heading(value):
    return math.atan2(2*(value['qw']*value['qz']+value['qx']*value['qy']), 1-2*(value['qy']**2+value['qz']**2))


def mapped(raw, theta, tx, ty, mount):
    c, s = math.cos(theta), math.sin(theta)
    return pose(raw['stamp_ns'], c*raw['x']-s*raw['y']+tx, s*raw['x']+c*raw['y']+ty, heading(raw)+theta+mount)


def diagnostic(index, final, raw=None, accepted=True, score=.2):
    return {'match_id': index, 'stamp_ns': final['stamp_ns'], 'completed_ns': final['stamp_ns']+100_000_000,
            'duration_s': .03, 'stage': 'accepted' if accepted else 'fitness_above_limit', 'decision_reason': 'synthetic',
            'accepted': accepted, 'converged': True, 'iterations': 8, 'fitness_valid': True, 'fitness': score,
            'fitness_definition': 'synthetic lower better', 'score_lower_is_better': True,
            'input_points': 1000, 'source_points': 300, 'target_raw_points': 500, 'target_points': 250,
            'max_fitness': 1.5, 'update_translation_m': .02, 'update_yaw_rad': .01,
            'cloud_to_odom_sync_valid': True, 'cloud_to_odom_skew_s': .01,
            'raw_valid': raw is not None, 'raw_pose': raw, 'final_valid': True, 'final_pose': final,
            'final_pose_source': 'accepted_this_match'}


class NdtEvaluationTests(unittest.TestCase):
    def test_interpolation_handles_wrap_and_rejects_long_gap(self):
        first, second = pose(1_000_000_000, 0., 0., math.radians(179)), pose(1_100_000_000, 1., 0., math.radians(-179))
        middle, error = interpolate_pose([first, second], 1_050_000_000, .2)
        self.assertIsNone(error)
        self.assertAlmostEqual(middle['x'], .5)
        self.assertLess(abs(abs(heading(middle))-math.pi), .02)
        missing, error = interpolate_pose([first, second], 1_050_000_000, .01)
        self.assertIsNone(missing)
        self.assertEqual(error, 'mocap_interpolation_gap_too_large')

    def test_known_transform_and_mount_offset(self):
        theta, tx, ty, mount = .7, 2.5, -1.2, -.4
        pairs = []
        for index in range(5):
            raw = pose(1_000_000_000+index*10_000_000, 1., -2., .3)
            pairs.append((raw, mapped(raw, theta, tx, ty, mount)))
        alignment = estimate_start_alignment(pairs, mount, .02)
        self.assertAlmostEqual(wrap(alignment.yaw_rad-theta), 0., places=7)
        self.assertAlmostEqual(alignment.tx_m, tx, places=7)
        self.assertAlmostEqual(alignment.ty_m, ty, places=7)
        truth = mapped(pose(2_000_000_000, 3., 1., math.radians(179)), theta, tx, ty, mount)
        self.assertLess(pose_error(truth, alignment.apply(pose(2_000_000_000, 3., 1., math.radians(179))))['xy_m'], 1e-9)

    def test_full_tracker_to_base_transform_rotates_the_lever_arm(self):
        tracker = pose(123, 1., 2., math.pi / 2.)
        base = rigid_body_to_base_pose(tracker, (.2, -.1, -.18), yaw_quaternion(-.4))
        # p_TB=(+0.2,-0.1,-0.18) rotates with the tracker: at +90 degrees it
        # becomes (+0.1,+0.2,-0.18) in the mocap world.
        self.assertEqual(base['stamp_ns'], 123)
        self.assertAlmostEqual(base['x'], 1.1, places=9)
        self.assertAlmostEqual(base['y'], 2.2, places=9)
        self.assertAlmostEqual(base['z'], -.18, places=9)
        self.assertAlmostEqual(wrap(heading(base) - (math.pi / 2. - .4)), 0., places=9)

    def test_tracker0_config_is_explicit_and_applied_offline(self):
        transform = tracker_to_base_from_config(Path('robot_data/config/mocap_tracker0.yaml'))
        self.assertEqual(transform['translation_m'], [0., 0., -.18])
        self.assertEqual(transform['quaternion_xyzw'], [0., 0., 0., 1.])
        converted = apply_tracker_to_base([pose(100, 2., 3., 0.)], transform)
        self.assertEqual(len(converted), 1)
        self.assertAlmostEqual(converted[0]['x'], 2.)
        self.assertAlmostEqual(converted[0]['y'], 3.)
        self.assertAlmostEqual(converted[0]['z'], -.18)

    def test_evaluation_keeps_rejected_raw_result(self):
        theta, tx, ty, mount = .2, 4., -3., -.3
        raw = [pose(1_000_000_000+i*100_000_000, 0., 0., .1) for i in range(3)]
        raw.extend([pose(2_000_000_000, 1., 0., .2), pose(2_100_000_000, 2., 0., .3)])
        final = [mapped(item, theta, tx, ty, mount) for item in raw]
        biased = dict(final[-1]); biased['x'] += .4
        diagnostics = [diagnostic(i, item, item) for i, item in enumerate(final[:-1])]
        diagnostics.append(diagnostic(4, final[-1], biased, accepted=False, score=2.))
        args = SimpleNamespace(alignment_mode='start', time_offset_seconds=0., max_interpolation_gap_seconds=.15,
                               alignment_window_seconds=.25, alignment_min_samples=3,
                               alignment_max_static_motion_m=.01, mocap_to_base_yaw_rad=mount, calibration_file='unused')
        _, summary = evaluate(diagnostics, raw, args)
        self.assertEqual(summary['raw_ndt_all_evaluable']['samples'], 2)
        self.assertEqual(summary['raw_ndt_accepted_evaluable']['samples'], 1)
        self.assertAlmostEqual(summary['raw_ndt_all_evaluable']['xy_m']['max_absolute'], .4)
        self.assertAlmostEqual(summary['final_map_base_all_evaluable']['xy_m']['max_absolute'], 0.)

    def test_explicit_alignment_warmup_is_excluded_and_not_auto_selected(self):
        theta, tx, ty, mount = .2, 4., -3., -.3
        raw = [pose(1_000_000_000 + index * 100_000_000, 0., 0., .1) for index in range(6)]
        final = [mapped(item, theta, tx, ty, mount) for item in raw]
        diagnostics = [diagnostic(index, item, item) for index, item in enumerate(final)]
        args = SimpleNamespace(alignment_mode='start', time_offset_seconds=0., max_interpolation_gap_seconds=.15,
                               alignment_start_delay_seconds=.2, alignment_window_seconds=.2,
                               alignment_min_samples=3, alignment_max_static_motion_m=.01,
                               mocap_to_base_yaw_rad=mount, calibration_file='unused')
        rows, summary = evaluate(diagnostics, raw, args)
        self.assertEqual([row['match_id'] for row in rows if row['in_alignment_window']], [0, 1, 2, 3, 4])
        self.assertEqual(summary['final_map_base_all_evaluable']['samples'], 1)
        self.assertEqual(summary['start_alignment']['warmup_seconds'], .2)

    def test_first_raw_ndt_baseline_zeros_first_raw_output_and_keeps_all_samples(self):
        theta, tx, ty, mount = -.4, 3., 2., .2
        mocap = [pose(1_000_000_000 + index*100_000_000, .1*index, 0., .3) for index in range(3)]
        mapped_poses = [mapped(item, theta, tx, ty, mount) for item in mocap]
        diagnostics = [diagnostic(index, item, item) for index, item in enumerate(mapped_poses)]
        args = SimpleNamespace(alignment_mode='first_raw_ndt', time_offset_seconds=0.,
                               max_interpolation_gap_seconds=.15, mocap_to_base_yaw_rad=mount)
        alignment, baseline, rejected = first_raw_ndt_alignment(diagnostics, mocap, args)
        self.assertEqual(baseline['match_id'], 0)
        self.assertEqual(rejected, {})
        self.assertLess(pose_error(mapped_poses[0], alignment.apply(mocap[0]))['xy_m'], 1e-9)
        rows, summary = evaluate(diagnostics, mocap, args)
        self.assertEqual(summary['alignment_mode'], 'first_raw_ndt')
        self.assertEqual(summary['first_raw_ndt_baseline']['match_id'], 0)
        self.assertFalse(any(row['in_alignment_window'] for row in rows))
        self.assertTrue(rows[0]['is_relative_baseline'])
        self.assertLess(rows[0]['raw_error']['xy_m'], 1e-9)

    def test_first_initial_baseline_retains_first_ndt_correction(self):
        theta, tx, ty, mount = .3, -2., 1., -.2
        mocap = [pose(1_000_000_000, 0., 0., .1), pose(1_100_000_000, 0., 0., .1)]
        initial = [mapped(item, theta, tx, ty, mount) for item in mocap]
        raw = [dict(item) for item in initial]
        raw[0]['x'] += .04
        raw[0]['y'] -= .03
        raw[1]['x'] += .04
        raw[1]['y'] -= .03
        diagnostics = [diagnostic(index, initial[index], raw[index]) for index in range(2)]
        for index, item in enumerate(diagnostics):
            item.update({'initial_valid': True, 'initial_pose': initial[index],
                         'initial_pose_source': 'synthetic_v2',
                         'init_to_ndt_correction_valid': True,
                         'init_to_ndt_translation_m': .05,
                         'init_to_ndt_yaw_rad': .01})
        args = SimpleNamespace(alignment_mode='first_init', time_offset_seconds=0.,
                               max_interpolation_gap_seconds=.15, mocap_to_base_yaw_rad=mount)
        alignment, baseline, _ = first_initial_alignment(diagnostics, mocap, args)
        self.assertEqual(baseline['match_id'], 0)
        self.assertLess(pose_error(initial[0], alignment.apply(mocap[0]))['xy_m'], 1e-9)
        rows, summary = evaluate(diagnostics, mocap, args)
        self.assertLess(rows[0]['initial_error']['xy_m'], 1e-9)
        self.assertAlmostEqual(rows[0]['raw_error']['x_m'], .04, places=9)
        self.assertAlmostEqual(rows[0]['raw_error']['y_m'], -.03, places=9)
        self.assertEqual(summary['first_initial_prediction_baseline']['match_id'], 0)
        self.assertEqual(summary['metrics_scored_rows'], 1)
        self.assertEqual(summary['raw_ndt_all_evaluable']['samples'], 1)
        first = summary['cold_start_first_match']
        self.assertAlmostEqual(first['raw_ndt_error_relative_to_T_init']['x_m'], .04, places=9)
        self.assertAlmostEqual(first['init_to_ndt_correction']['translation_m'], .05, places=9)

    def test_first_initial_refuses_to_substitute_a_later_match_when_first_has_no_mocap(self):
        mount = 0.
        first_initial = pose(1_000_000_000, 0., 0., 0.)
        second_initial = pose(1_100_000_000, 0., 0., 0.)
        diagnostics = [diagnostic(1, first_initial, first_initial), diagnostic(2, second_initial, second_initial)]
        for item, initial in zip(diagnostics, (first_initial, second_initial)):
            item.update({'initial_valid': True, 'initial_pose': initial,
                         'initial_pose_source': 'synthetic_v2'})
        # The first true registration precedes the motion-capture recording;
        # a valid reference for the second match must not silently replace it.
        mocap = [pose(1_100_000_000, 0., 0., 0.)]
        args = SimpleNamespace(alignment_mode='first_init', time_offset_seconds=0.,
                               max_interpolation_gap_seconds=.15, mocap_to_base_yaw_rad=mount)
        with self.assertRaisesRegex(RuntimeError, 'do not substitute a later match'):
            first_initial_alignment(diagnostics, mocap, args)

    def test_report_artifacts_are_written(self):
        theta, tx, ty, mount = .1, 1., 2., -.2
        raw = [pose(1_000_000_000+i*100_000_000, 0., 0., 0.) for i in range(3)]
        raw.extend([pose(2_000_000_000, 1., 0., .1), pose(2_100_000_000, 2., 0., .2)])
        final = [mapped(item, theta, tx, ty, mount) for item in raw]
        diagnostics = [diagnostic(index, item, item) for index, item in enumerate(final)]
        args = SimpleNamespace(alignment_mode='start', time_offset_seconds=0., max_interpolation_gap_seconds=.15,
                               alignment_window_seconds=.25, alignment_min_samples=3,
                               alignment_max_static_motion_m=.01, mocap_to_base_yaw_rad=mount, calibration_file='unused')
        rows, summary = evaluate(diagnostics, raw, args)
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            write_csv(output/'matches.csv', rows)
            save_plots(output, rows, summary)
            write_summary_text(output/'summary.txt', summary)
            for filename in ('matches.csv', 'summary.txt', 'trajectory.png', 'trajectory.svg',
                             'errors.png', 'errors.svg', 'fitness_vs_error.png', 'fitness_vs_error.svg'):
                self.assertGreater((output/filename).stat().st_size, 0, filename)


if __name__ == '__main__':
    unittest.main()
