"""Synthetic checks for cold-start first-match selection and scatter metrics."""
from __future__ import annotations

import math
import unittest

from ndt_cold_start_core import (
    first_accepted_registration, first_registration, raw_repeatability,
    summarize_cold_start_trials,
)
from ndt_match_evaluation_core import yaw_quaternion


def record(match_id, stamp, stage='accepted', accepted=True, x=1.0, y=2.0, heading=0.0):
    qx, qy, qz, qw = yaw_quaternion(heading)
    return {'match_id': match_id, 'stamp_ns': stamp, 'stage': stage, 'accepted': accepted,
            'converged': accepted, 'raw_valid': True,
        'raw_pose': {'x': x, 'y': y, 'z': 0., 'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw}}


class ColdStartAnalysisTests(unittest.TestCase):
    def test_first_real_registration_keeps_a_rejected_first_match(self):
        waiting = record(1, 1, stage='waiting_for_initialization', accepted=False)
        rejected = record(2, 2, stage='fitness_above_limit', accepted=False, x=3.)
        accepted = record(3, 3, accepted=True, x=4.)
        self.assertEqual(first_registration([waiting, rejected, accepted])['match_id'], 2)
        self.assertEqual(first_accepted_registration([waiting, rejected, accepted])['match_id'], 3)

    def test_repeatability_removes_common_bias_and_handles_yaw_wrap(self):
        records = [record(1, 1, x=10.0, y=-4.0, heading=math.radians(179)),
                   record(2, 2, x=10.01, y=-4.0, heading=math.radians(-179))]
        metrics = raw_repeatability(records)
        self.assertTrue(metrics['metrics_available'])
        self.assertAlmostEqual(metrics['centroid_map_xy_m'][0], 10.005, places=9)
        self.assertAlmostEqual(metrics['xy_scatter_rmse_m'], .005, places=9)
        self.assertLess(metrics['yaw_std_deg'], 2.0)

    def test_summary_counts_missing_and_rejected_trials(self):
        rejected = record(1, 1, accepted=False)
        accepted = record(1, 1, accepted=True)
        summary = summarize_cold_start_trials([
            {'first_registration': rejected}, {'first_registration': accepted}, {'first_registration': None},
        ])
        self.assertEqual(summary['trial_count'], 3)
        self.assertEqual(summary['trials_without_registration'], 1)
        self.assertEqual(summary['first_registration_accepted_count'], 1)
        self.assertAlmostEqual(summary['first_registration_accepted_rate'], .5)


if __name__ == '__main__':
    unittest.main()
