import math
import unittest

from mocap_static_evaluation import analyze_samples
from ndt_match_evaluation_core import yaw_quaternion


def sample(stamp_ns, x, y, yaw_rad):
    qx, qy, qz, qw = yaw_quaternion(yaw_rad)
    return {'stamp_ns': stamp_ns, 'frame_id': 'mocap_world', 'x': x, 'y': y, 'z': 0.,
            'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw}


class StaticMocapEvaluationTests(unittest.TestCase):
    def test_metrics_handle_yaw_wrap_and_exact_mm_to_m_conversion(self):
        normalized = [sample(1_000_000_000, 1.000, 2.000, math.radians(179.8)),
                      sample(1_010_000_000, 1.001, 2.000, math.radians(-179.8)),
                      sample(1_020_000_000, 0.999, 2.000, math.radians(180.0))]
        raw = []
        for item in normalized:
            source = dict(item)
            source['x'] *= 1000.; source['y'] *= 1000.; source['z'] *= 1000.
            raw.append(source)
        diagnostics = [{'statuses': [{'level': 0, 'message': 'valid', 'values': {'valid': 'true'}}]} for _ in normalized]
        result = analyze_samples(normalized, raw, diagnostics)
        self.assertAlmostEqual(result['frequency_hz'], 100.)
        self.assertEqual(result['nonpositive_timestamp_steps'], 0)
        self.assertAlmostEqual(result['raw_to_normalized_max_residual_m'], 0.)
        self.assertEqual(result['valid_diagnostic_statuses'], 3)
        self.assertLess(result['yaw_p95_abs_deg'], 1.)


if __name__ == '__main__':
    unittest.main()
