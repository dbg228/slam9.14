"""Scientific invariants for the excluded-initialization static analysis.

These tests use known physical poses rather than replaying recorded metrics:
the shared reference, rejected proposals, gaps and SE(3) lever arms must remain
distinguishable even when all of them appear in one stationary experiment.
"""
import copy
import math
import unittest

import numpy as np

from ndt_match_evaluation_core import yaw, yaw_quaternion
from ndt_post_init_analysis import Options, analyze, compose, inverse, phases, propagation, relative


SECOND = 1_000_000_000


def pose(stamp=SECOND, x=0., y=0., z=0., heading=0., quaternion=None, valid=True):
    q = yaw_quaternion(heading) if quaternion is None else quaternion
    return dict(zip(('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'), (x, y, z, *q)),
                stamp_ns=stamp, valid=valid)


def diagnostic(match_id, initial=None, raw=None, *, stage='accepted', accepted=True,
               converged=True, stamp=None, pre=None, candidate=None):
    stamp = match_id*SECOND if stamp is None else stamp
    initial = pose(stamp) if initial is None else initial
    raw = pose(stamp) if raw is None else raw
    pre = pose(stamp) if pre is None else pre
    candidate = pose(stamp) if candidate is None else candidate
    return {'match_id': match_id, 'stamp_ns': stamp, 'stage': stage,
            'accepted': accepted, 'converged': converged,
            'initial_valid': True, 'initial_pose': initial,
            'raw_valid': True, 'raw_pose': raw,
            'final_valid': True, 'final_pose': raw if accepted else initial,
            'initial_map_odom_pose': pre, 'raw_map_odom_pose': candidate,
            'fitness_valid': True, 'fitness': .2, 'iterations': 2,
            'ndt_align_duration_s': .01}


def session(formal_count=3):
    diagnostics = [diagnostic(1, stage='waiting_for_initialization', accepted=False, converged=False),
                   diagnostic(2, stage='ndt_not_converged', accepted=False, converged=False)]
    # ID 3 completes initialization; 4, 5, 6 are the excluded settling window.
    diagnostics += [diagnostic(i) for i in range(3, 7+formal_count)]
    mocap = [pose(i*SECOND) for i in range(1, 7+formal_count)]
    return diagnostics, mocap


class PostInitializationTests(unittest.TestCase):
    def test_initialization_requires_both_success_and_acceptance_and_skip_counts_attempts(self):
        diagnostics = [
            diagnostic(1, stage='waiting_for_initialization', accepted=False, converged=False),
            diagnostic(2, stage='fitness_above_limit', accepted=False, converged=True),
            diagnostic(3, stage='ndt_not_converged', accepted=False, converged=False),
            diagnostic(4),
            diagnostic(5, stage='no_input_cloud', accepted=False, converged=False),
            diagnostic(6, stage='ndt_exception', accepted=False, converged=False),
            diagnostic(7, stage='fitness_above_limit', accepted=False),
            diagnostic(8), diagnostic(9)]
        result = phases(list(reversed(diagnostics)), 3)
        phases_by_id = {r['match_id']: r['phase'] for r in result}
        self.assertEqual(phases_by_id[2], 'pre_initialization')
        self.assertEqual(phases_by_id[3], 'pre_initialization')
        self.assertEqual(phases_by_id[4], 'initialization_complete')
        self.assertEqual(phases_by_id[5], 'passive_after_initialization')
        self.assertEqual([r['match_id'] for r in result if r['phase'] == 'settling'], [6, 7, 8])
        self.assertEqual(phases_by_id[9], 'formal')

    def test_midrun_recording_cannot_claim_to_observe_initialization(self):
        diagnostics, _ = session()
        with self.assertRaisesRegex(ValueError, 'Initialization not observed'):
            phases(diagnostics[2:], 3)
        # A reset inside one recording would mix two unrelated state histories.
        diagnostics.append(diagnostic(10, stage='waiting_for_initialization', accepted=False, converged=False))
        with self.assertRaisesRegex(ValueError, 'second initialization'):
            phases(diagnostics, 3)

    def test_one_settling_alignment_is_shared_and_formal_outlier_cannot_change_it(self):
        diagnostics, mocap = session()
        theta, tx, ty = .7, 3., -2.
        c, s = math.cos(theta), math.sin(theta)
        mocap = [pose(p['stamp_ns'], x=.5, y=-.2, heading=-.1) for p in mocap]
        known = lambda stamp, dx: pose(stamp, x=c*.5+s*.2+tx+dx,
                                       y=s*.5-c*.2+ty, heading=theta-.1)
        for d in diagnostics:
            stamp = d['stamp_ns']
            formal = d['match_id'] >= 7
            d.update(initial_pose=known(stamp, .02 if formal else 0.),
                     raw_pose=known(stamp, .05 if formal else 0.),
                     final_pose=known(stamp, .03 if formal else 0.))
        rows, summary, _ = analyze(diagnostics, mocap)
        self.assertAlmostEqual(summary['alignment']['yaw_rad'], theta)
        self.assertAlmostEqual(summary['alignment']['tx_m'], tx)
        self.assertAlmostEqual(summary['alignment']['ty_m'], ty)
        first = next(r for r in rows if r['phase'] == 'formal')
        for stage, expected in [('initial', .02), ('raw', .05), ('final', .03)]:
            self.assertAlmostEqual(first[stage+'_error']['xy_m'], expected)
        altered = copy.deepcopy(diagnostics)
        altered[-1]['final_pose']['x'] += 1.2
        _, second, _ = analyze(altered, mocap)
        self.assertEqual(summary['alignment'], second['alignment'])
        self.assertGreater(second['all_formal']['final']['xy_m']['max_absolute'], 1.)
        self.assertEqual(second['all_formal']['final']['samples'], 3)

    def test_rejected_proposal_retains_raw_error_but_applies_no_map_update(self):
        diagnostics, mocap = session()
        diagnostics[-3] = diagnostic(7, pose(7*SECOND, x=.02), pose(7*SECOND, x=.07),
                                     stage='fitness_above_limit', accepted=False,
                                     pre=pose(x=.02), candidate=pose(x=.07))
        rows, summary, _ = analyze(diagnostics, mocap)
        rejected = next(r for r in rows if r['match_id'] == 7)
        self.assertAlmostEqual(rejected['raw_error']['xy_m'], .07)
        self.assertAlmostEqual(rejected['final_error']['xy_m'], .02)
        self.assertAlmostEqual(rejected['delta_e_m'], .05)
        self.assertAlmostEqual(rejected['map_candidate_update_xy_m'], .05)
        self.assertAlmostEqual(rejected['map_applied_update_xy_m'], 0.)
        self.assertEqual(summary['all_formal']['raw']['samples'], 3)
        self.assertEqual(summary['accepted_formal']['raw']['samples'], 2)
        self.assertEqual(summary['updates']['map_candidate_update_xy_m']['thresholds']['30']['count'], 1)
        self.assertEqual(summary['updates']['map_applied_update_xy_m']['thresholds']['30']['count'], 0)

    def test_long_reference_gap_is_unknown_and_adjacent_errors_never_bridge_it(self):
        diagnostics, mocap = session()
        mocap = [p for p in mocap if p['stamp_ns'] != 8*SECOND]
        rows, summary, _ = analyze(diagnostics, mocap)
        gap = next(r for r in rows if r['match_id'] == 8)
        self.assertIsNone(gap['reference_pose'])
        self.assertIsNone(gap['raw_error'])
        self.assertIsNone(gap['delta_e_m'])
        self.assertEqual(gap['reference_reason'], 'mocap_interpolation_gap_too_large')
        self.assertAlmostEqual(summary['formal_reference_coverage'], 2/3)
        self.assertEqual(summary['all_formal']['raw']['samples'], 2)
        # Raw NDT increments remain measurable, but no consecutive *reference*
        # pair is valid; connecting frame 7 directly to frame 9 would be biased.
        self.assertEqual(summary['adjacent_frames']['raw']['delta_xy_m']['samples'], 2)
        self.assertEqual(summary['adjacent_frames']['raw']['delta_xy_error_m']['samples'], 0)

    def test_explicit_invalid_reference_and_pose_flags_do_not_become_zero_error(self):
        diagnostics, mocap = session()
        for p in mocap:
            if p['stamp_ns'] == 8*SECOND:
                p['valid'] = False
        mocap += [pose(8*SECOND-20_000_000), pose(8*SECOND+20_000_000)]
        mocap.sort(key=lambda p: p['stamp_ns'])
        diagnostics[-2]['raw_valid'] = False
        rows, summary, _ = analyze(diagnostics, mocap)
        invalid = next(r for r in rows if r['match_id'] == 8)
        self.assertIsNone(invalid['reference_pose'])
        self.assertTrue(invalid['reference_reason'])
        self.assertIsNone(invalid['base_candidate_update_xy_m'])
        self.assertIsNone(invalid['raw_error'])
        self.assertEqual(summary['adjacent_frames']['raw']['delta_xy_m']['samples'], 0)
        self.assertEqual(summary['adjacent_frames']['raw']['delta_xy_error_m']['samples'], 0)

    def test_net_effect_and_large_update_counts_use_all_predeclared_formal_frames(self):
        diagnostics, mocap = session(formal_count=4)
        pairs = [(.04, .005), (.005, .02), (.01, .035), (.02, .02)]
        for match_id, (initial, raw) in enumerate(pairs, 7):
            diagnostics[match_id-1] = diagnostic(
                match_id, pose(match_id*SECOND, x=initial), pose(match_id*SECOND, x=raw),
                pre=pose(x=initial), candidate=pose(x=raw))
        _, summary, _ = analyze(diagnostics, mocap)
        effect = summary['ndt_net_effect_all']
        self.assertEqual(effect['paired_frames'], 4)
        self.assertEqual(effect['improved_count'], 1)
        self.assertEqual(effect['worsened_count'], 2)
        self.assertEqual(effect['neutral_count'], 1)
        self.assertAlmostEqual(effect['improved_ratio'], .25)
        self.assertAlmostEqual(effect['worsened_ratio'], .5)
        self.assertAlmostEqual(effect['mean_improvement_m'], .035)
        self.assertAlmostEqual(effect['mean_worsening_m'], .02)
        self.assertEqual(effect['max_worsening_match_id'], 9)
        thresholds = summary['updates']['map_applied_update_xy_m']['thresholds']
        for threshold, expected in [('10', 3), ('20', 2), ('30', 1)]:
            self.assertEqual(thresholds[threshold]['count'], expected)
            self.assertEqual(thresholds[threshold]['denominator'], 4)
            self.assertAlmostEqual(thresholds[threshold]['ratio'], expected/4)

    def test_yaw_wrap_is_two_degrees_not_358(self):
        diagnostics, mocap = session(formal_count=2)
        for d in diagnostics:
            heading = math.radians(-179 if d['match_id'] == 8 else 179)
            for stage in ('initial', 'raw', 'final'):
                d[stage+'_pose'] = pose(d['stamp_ns'], heading=heading)
        rows, summary, _ = analyze(diagnostics, mocap)
        last = rows[-1]
        self.assertAlmostEqual(math.degrees(last['raw_error']['yaw_rad']), 2.)
        self.assertAlmostEqual(summary['adjacent_frames']['raw']['delta_yaw_deg']['rmse'], 2.)
        self.assertAlmostEqual(summary['all_formal']['raw']['yaw_deg']['max_absolute'], 2.)

    def test_same_cloud_measurement_is_excluded_even_across_settling_boundary(self):
        diagnostics, mocap = session(formal_count=4)
        diagnostics[6]['stamp_ns'] = diagnostics[5]['stamp_ns']
        diagnostics[-1]['stamp_ns'] = diagnostics[-2]['stamp_ns']
        rows, summary, _ = analyze(diagnostics, mocap)
        self.assertEqual(summary['formal_match_ids'], [8, 9])
        self.assertEqual(summary['duplicate_measurements_excluded'], 2)
        self.assertEqual(summary['all_formal']['raw']['samples'], 2)
        self.assertTrue(next(r for r in rows if r['match_id'] == 7)['duplicate_measurement'])
        self.assertTrue(next(r for r in rows if r['match_id'] == 10)['duplicate_measurement'])

    def test_full_3d_lever_arm_and_next_initial_decomposition_survive_odom_origin_change(self):
        # +90 degrees about Y maps local (2, 3, 1) to world (1, 3, -2).
        a = pose(x=4., y=5., z=6., quaternion=(0., math.sqrt(.5), 0., math.sqrt(.5)))
        b = pose(x=2., y=3., z=1., heading=.3)
        c = compose(a, b)
        np.testing.assert_allclose([c[k] for k in ('x', 'y', 'z')], [5., 8., 4.], atol=1e-12)
        recovered = relative(a, c)
        np.testing.assert_allclose([recovered[k] for k in ('x', 'y', 'z')], [2., 3., 1.], atol=1e-12)
        self.assertAlmostEqual(yaw(recovered), .3)
        identity = compose(inverse(a), a)
        np.testing.assert_allclose([identity[k] for k in ('x', 'y', 'z')], 0., atol=1e-12)

        pre = pose(x=3., y=-1., z=.4, quaternion=(0., math.sin(.15), 0., math.cos(.15)))
        update = pose(x=.03, y=-.01, z=.02, quaternion=(math.sin(.01), 0., 0., math.cos(.01)))
        post = compose(pre, update)
        odom = pose(x=2., y=.4, z=.8, heading=.3)
        next_odom = pose(x=2.001, y=.398, z=.799, heading=.301)
        initial, next_initial = compose(pre, odom), compose(post, next_odom)
        records = [dict(match_id=7, stamp_ns=7*SECOND, phase='formal', accepted=True,
                        initial_map_odom_pose=pre, map_odom_pose=post, initial_pose=initial),
                   dict(match_id=8, stamp_ns=8*SECOND, phase='formal', accepted=True,
                        initial_map_odom_pose=post, map_odom_pose=post, initial_pose=next_initial)]
        first = propagation(records, 5.)[0]
        self.assertLess(first['closure_xy_m'], 1e-12)
        self.assertLess(first['map_state_inheritance_position_difference_m'], 1e-12)
        self.assertGreater(first['ndt_contribution_to_next_init_xy_m'], .01)
        for axis in ('x', 'y', 'z'):
            self.assertAlmostEqual(first['observed_delta_'+axis+'_m'],
                                   first['ndt_delta_'+axis+'_m']+first['lio_delta_'+axis+'_m'])
        # Re-express the same physical motion in an odom frame whose origin is
        # far away and whose axes are rotated. Map-origin corrections may vary,
        # while actual base motion and its NDT/LIO contributions cannot.
        new_origin = pose(x=10., y=-8., z=3., heading=.6)
        moved = copy.deepcopy(records)
        for r in moved:
            r['initial_map_odom_pose'] = compose(r['initial_map_odom_pose'], new_origin)
            r['map_odom_pose'] = compose(r['map_odom_pose'], new_origin)
        second = propagation(moved, 5.)[0]
        self.assertLess(second['closure_xy_m'], 1e-12)
        for key in ('ndt_contribution_to_next_init_xy_m', 'lio_contribution_to_next_init_xy_m',
                    'observed_next_init_delta_xy_m'):
            self.assertAlmostEqual(first[key], second[key], places=11)


if __name__ == '__main__':
    unittest.main()
