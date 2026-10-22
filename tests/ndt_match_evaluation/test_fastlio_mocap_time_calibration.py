"""Synthetic tests for the independent FAST-LIO--mocap timing estimator."""
from __future__ import annotations

import math
import unittest

import numpy as np

from fastlio_mocap_time_calibration import (
    OffsetSearchOptions,
    kinematic_signals,
    search_time_offset,
)


class TimeCalibrationTests(unittest.TestCase):
    def test_known_positive_offset_is_recovered_with_documented_sign(self):
        """mocap(t + d) == lio(t) must recover positive d."""
        known_offset = 0.037

        def model(t):
            # Non-periodic motion features avoid an ambiguous cross-correlation
            # peak while retaining separate translation and rotation evidence.
            speed = (0.004 + 0.15 * math.exp(-((t - 3.1) / .45) ** 2)
                     + 0.10 * math.exp(-((t - 8.2) / .65) ** 2)
                     + 0.06 * math.exp(-((t - 13.7) / .3) ** 2))
            yaw_rate = (0.22 * math.exp(-((t - 5.4) / .55) ** 2)
                        - 0.31 * math.exp(-((t - 11.5) / .7) ** 2)
                        + 0.12 * math.exp(-((t - 15.4) / .35) ** 2))
            return speed, yaw_rate

        lio_t = np.arange(0.0, 18.0, .05)
        mocap_t = np.arange(-.5, 18.5, .011)
        start_ns = 1_000_000_000
        lio = []
        for value in lio_t:
            speed, rate = model(value)
            lio.append({'stamp_ns': start_ns + round(value * 1e9),
                        'linear_speed_mps': speed, 'yaw_rate_radps': rate})
        # At timestamp tm, the mocap sample represents model(tm - known), so
        # matching an LIO sample at t requires tm=t+known.
        mocap = []
        for value in mocap_t:
            speed, rate = model(value - known_offset)
            mocap.append({'stamp_ns': start_ns + round(value * 1e9),
                          'linear_speed_mps': speed, 'yaw_rate_radps': rate})
        _, result = search_time_offset(
            lio, mocap,
            OffsetSearchOptions(search_min_seconds=-.10, search_max_seconds=.10,
                                search_step_seconds=.001, max_interpolation_gap_seconds=.05,
                                linear_activity_threshold_mps=.01,
                                yaw_activity_threshold_radps=.01,
                                minimum_active_samples=20),
        )
        self.assertAlmostEqual(result['best_offset_seconds'], known_offset, places=3)
        self.assertGreater(result['best_combined_correlation'], .999)

    def test_kinematic_signal_recovers_constant_translation_and_yaw_rate(self):
        samples = []
        # x=t, yaw=0.4t, uniformly sampled.  Values are a valid planar pose.
        for index in range(101):
            t = index * .05
            half = .5 * .4 * t
            samples.append({
                'stamp_ns': 10_000_000_000 + round(t * 1e9),
                'x': t, 'y': 0., 'z': 0.,
                'qx': 0., 'qy': 0., 'qz': math.sin(half), 'qw': math.cos(half),
                'valid': True,
            })
        signals = kinematic_signals(samples, median_radius_seconds=.05,
                                    derivative_half_window_seconds=.15)
        middle = signals[10:-10]
        # The calibration estimator intentionally includes a finite time
        # median and local robust fit, so this verifies physical fidelity,
        # rather than demanding an unrealistically exact finite-window slope.
        self.assertAlmostEqual(float(np.median([item['linear_speed_mps'] for item in middle])), 1.0, delta=.03)
        self.assertAlmostEqual(float(np.median([item['yaw_rate_radps'] for item in middle])), .4, delta=.02)

    def test_insufficient_active_motion_is_rejected(self):
        values = [{'stamp_ns': 1_000_000_000 + index * 100_000_000,
                   'linear_speed_mps': 0., 'yaw_rate_radps': 0.}
                  for index in range(30)]
        with self.assertRaisesRegex(ValueError, 'no_candidate_has_enough'):
            search_time_offset(values, values, OffsetSearchOptions(minimum_active_samples=5))


if __name__ == '__main__':
    unittest.main()
