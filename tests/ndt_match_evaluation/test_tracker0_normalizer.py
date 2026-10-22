"""Unit tests for the passive Tracker0 normalizer's geometry safeguards."""
import unittest

from geometry_msgs.msg import PoseStamped

from mocap_pose_normalizer import (
    normalize_pose,
    receive_clock_comparison_note,
    receive_clock_minus_header_ns,
)


class Tracker0NormalizerTests(unittest.TestCase):
    def test_scales_only_translation_and_normalizes_quaternion(self):
        source = PoseStamped()
        source.header.frame_id = 'mocap_world'
        source.header.stamp.sec = 12
        source.header.stamp.nanosec = 34
        source.pose.position.x = 1250.
        source.pose.position.y = -500.
        source.pose.position.z = 100.
        source.pose.orientation.z = 2.
        source.pose.orientation.w = 2.
        result = normalize_pose(source, .001, 'mocap_world')
        self.assertEqual(result.header.frame_id, 'mocap_world')
        self.assertEqual(result.header.stamp, source.header.stamp)
        self.assertAlmostEqual(result.pose.position.x, 1.25)
        self.assertAlmostEqual(result.pose.position.y, -.5)
        self.assertAlmostEqual(result.pose.position.z, .1)
        self.assertAlmostEqual(result.pose.orientation.z, 2**-.5)
        self.assertAlmostEqual(result.pose.orientation.w, 2**-.5)

    def test_rejects_zero_quaternion(self):
        source = PoseStamped()
        source.pose.orientation.w = 0.
        with self.assertRaisesRegex(ValueError, 'zero_quaternion'):
            normalize_pose(source, .001, 'mocap_world')

    def test_receive_clock_comparison_preserves_signed_delay(self):
        self.assertEqual(receive_clock_minus_header_ns(1_000_000_000, 1_023_500_000),
                         23_500_000)
        # A negative value is diagnostic evidence of clock offset; it must not
        # be silently clamped or interpreted as a valid nonnegative latency.
        self.assertEqual(receive_clock_minus_header_ns(1_100, 1_000), -100)
        self.assertIsNone(receive_clock_minus_header_ns(0, 1_000))
        self.assertIsNone(receive_clock_minus_header_ns(1_000, 0))

    def test_receive_clock_note_does_not_claim_camera_latency(self):
        local_note = receive_clock_comparison_note('ubuntu_receiver_clock')
        self.assertIn('not_camera_measurement_latency', local_note)
        remote_note = receive_clock_comparison_note('vrpn_server_clock_unverified')
        self.assertIn('possible_cross_host_clock_offset', remote_note)
        self.assertIn('not_a_verified_camera_measurement_latency', remote_note)


if __name__ == '__main__':
    unittest.main()
