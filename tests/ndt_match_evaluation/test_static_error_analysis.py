"""Synthetic checks for post-alignment static-error decomposition."""
import math
import unittest

from ndt_match_evaluation_core import yaw_quaternion
from ndt_static_error_analysis import analyze


def pose(x, y, heading=0.):
    qx, qy, qz, qw = yaw_quaternion(heading)
    return {'x': x, 'y': y, 'z': 0., 'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw}


def record(index, error_x, error_y, correction=.01):
    reference = pose(0., 0.)
    return {
        'match_id': index, 'stamp_ns': 1_000_000_000 + index*2_000_000_000,
        'time_from_first_ndt_s': index*2., 'reference': reference,
        'ndt': pose(error_x, error_y), 'ex_m': error_x, 'ey_m': error_y,
        'e_xy_m': math.hypot(error_x, error_y), 'yaw_error_rad': 0.,
        'fitness': .1, 'correction_translation_m': correction,
        'correction_yaw_rad': 0., 'converged': True, 'accepted': True,
        'iterations': 1, 'input_points': 1000, 'source_points': 300,
        'target_raw_points': 500, 'target_points': 250,
    }


class StaticErrorAnalysisTests(unittest.TestCase):
    def test_separates_residual_center_outlier_and_relative_motion(self):
        records = [record(0, .01, 0.), record(1, .01, 0.), record(2, .01, 0.), record(3, .05, 0., .05)]
        summary, outliers, pairs = analyze(records, .04, 5.)
        self.assertEqual(summary['evaluable_frames'], 4)
        self.assertAlmostEqual(summary['residual_center_m'][0], .02)
        self.assertEqual(summary['outlier_match_ids'], [3])
        self.assertEqual(len(outliers), 1)
        self.assertEqual(summary['relative_motion']['pairs'], 3)
        self.assertAlmostEqual(summary['relative_motion']['mocap_delta_position']['rmse_m'], 0.)
        self.assertAlmostEqual(summary['relative_motion']['relative_delta_position_error']['max_m'], .04)
        self.assertEqual(len(pairs), 3)


if __name__ == '__main__':
    unittest.main()
