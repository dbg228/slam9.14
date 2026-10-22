import math
import unittest
from types import SimpleNamespace as NS

from ndt_chain_analysis import make_records, ndt_effect, relative_metrics, stage_metrics
from ndt_match_evaluation import diagnostic_from_ros
from ndt_match_evaluation_core import yaw_quaternion


def pose(x, y, angle=0.0):
    qx, qy, qz, qw = yaw_quaternion(angle)
    return NS(position=NS(x=x, y=y, z=0.0), orientation=NS(x=qx, y=qy, z=qz, w=qw))


class ChainAnalysisTests(unittest.TestCase):
    def test_detail_message_preserves_cloud_time_initial_pose(self):
        stamp = NS(sec=42, nanosec=5)
        base = NS(
            match_id=7, cloud_stamp=stamp, matching_started_stamp=stamp,
            matching_completed_stamp=stamp, computation_duration_sec=.02,
            initial_prediction_valid=True, initial_map_to_odom=pose(.4, -.2),
            raw_ndt_result_valid=True, raw_ndt_map_to_odom=pose(.5, -.2),
            stage='accepted', decision_reason='accepted', accepted=True, converged=True, iteration_count=1,
            fitness_score_valid=True, fitness_score=.1, fitness_score_definition='test', score_lower_is_better=True,
            input_point_count=10, source_point_count=8, target_raw_point_count=12, target_point_count=9,
            configured_max_fitness_score=1.5, correction_translation_m=.03, correction_yaw_rad=.01,
            covariance_available=False, covariance_note='none', cloud_to_odom_sync_valid=True,
            cloud_to_odom_skew_sec=0.0, raw_ndt_map_to_base_at_cloud_valid=True,
            raw_ndt_map_to_base_at_cloud=pose(1.1, 2.0), final_map_to_base_at_cloud_valid=True,
            final_map_to_base_at_cloud=pose(1.1, 2.0), final_pose_source='accepted_this_match')
        detail = NS(
            base=base, initial_map_to_base_at_cloud_valid=True, initial_map_to_base_at_cloud=pose(1.0, 2.0),
            initial_pose_at_cloud_source='pre_ndt_map_to_odom_times_odom_to_base_at_cloud',
            final_map_to_base_at_publish_valid=False, configured_ndt_max_iterations=15,
            configured_ndt_transformation_epsilon=.01, configured_ndt_step_size=.25, configured_ndt_resolution=1.2,
            init_to_ndt_correction_valid=True, initial_to_ndt_base_at_cloud=pose(.1, 0.0),
            init_to_ndt_translation_m=.1, init_to_ndt_yaw_rad=0.0, map_odom_correction_valid=False,
            nearest_neighbor_metrics_available=False, nearest_neighbor_evaluated_source_count=0,
            nearest_neighbor_inlier_count=0, nearest_neighbor_inlier_ratio=math.nan,
            nearest_neighbor_distance_threshold_m=1.2, nearest_neighbor_residual_rms_m=math.nan,
            nearest_neighbor_metrics_definition='unavailable', hessian_condition_available=False,
            hessian_condition_number=math.nan, hessian_condition_note='unavailable')
        record = diagnostic_from_ros(detail)
        self.assertTrue(record['initial_valid'])
        self.assertEqual(record['initial_pose']['x'], 1.0)
        self.assertTrue(record['diagnostic_detail_available'])
        self.assertEqual(record['iterations'], 1)
        self.assertEqual(record['initial_map_odom_pose']['x'], .4)
        self.assertEqual(record['raw_map_odom_pose']['x'], .5)

    def test_chain_separates_initial_from_raw_and_handles_yaw_wrap(self):
        def row(match_id, stamp_ns, reference_x, initial_x, raw_x, yaw_angle):
            qx, qy, qz, qw = yaw_quaternion(yaw_angle)
            values = {
                'match_id': str(match_id), 'stamp_ns': str(stamp_ns), 'in_alignment_window': 'false',
                'reference_valid': 'true', 'accepted': 'true', 'converged': 'true', 'iterations': '1',
                'fitness_valid': 'true', 'fitness': '.1', 'update_translation_m': '.01', 'update_yaw_rad': '0',
                'diagnostic_detail_available': 'true', 'init_to_ndt_translation_m': '.01',
                'init_to_ndt_yaw_rad': '0', 'source_points': '10', 'target_points': '10', 'stage': 'accepted',
            }
            for prefix, x in (('reference', reference_x), ('initial', initial_x), ('raw', raw_x), ('final', raw_x)):
                values[prefix+'_valid'] = 'true'
                values.update({prefix+'_x': str(x), prefix+'_y': '0', prefix+'_z': '0',
                               prefix+'_qx': str(qx), prefix+'_qy': str(qy), prefix+'_qz': str(qz), prefix+'_qw': str(qw)})
            return values
        # Initial prediction has a 10 cm bias; NDT/final is exact.  The yaw
        # samples straddle pi, so wrapped incremental yaw remains small.
        rows = [row(1, 0, 0.0, .1, 0.0, math.pi-.01),
                row(2, 2_000_000_000, 1.0, 1.1, 1.0, -math.pi+.01)]
        records, unavailable = make_records(rows)
        self.assertEqual(unavailable['initial'], 0)
        self.assertAlmostEqual(stage_metrics(records, 'initial')['xy_m']['rmse'], .1)
        self.assertAlmostEqual(stage_metrics(records, 'raw')['xy_m']['rmse'], 0.0)
        effect = ndt_effect(records)
        self.assertTrue(effect['available'])
        self.assertEqual(effect['raw_improves_xy_error_frames'], 2)
        pairs, metrics = relative_metrics(records, 'raw', 3.0)
        self.assertEqual(len(pairs), 1)
        self.assertAlmostEqual(metrics['relative_delta_yaw_error_deg']['max_absolute'], 0.0, places=8)


if __name__ == '__main__':
    unittest.main()
