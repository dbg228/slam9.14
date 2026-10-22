"""Read-only audit of cloud-time diagnostic poses against the recorded TF geometry.

This audit combines the final map->odom STATE of each match with the recorded
odom->base_link TF nearest its cloud stamp. It does not claim the state had
already been broadcast at that measurement instant (NDT finishes later).
"""
from __future__ import annotations

from bisect import bisect_left
from collections import Counter
import math
from pathlib import Path
import sqlite3


def audit_tf_chain(bag_path, records, max_skew_seconds=.05):
    """Return a JSON-safe audit dictionary without modifying the bag or ROS state.

    ``records`` are post-init analysis records with ``phase`` and final
    ``map_odom_pose``. Only unique formal cloud measurements are compared.
    ROS imports are lazy; sqlite3 databases are opened with ``mode=ro``.
    """
    if not math.isfinite(max_skew_seconds) or max_skew_seconds < 0:
        raise ValueError('max_skew_seconds must be finite and non-negative')
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from ndt_post_init_analysis import compose, stats, valid_pose
    from ndt_match_evaluation_core import pose_error

    path = Path(bag_path).resolve()
    databases = sorted(path.glob('*.db3')) if path.is_dir() else [path]
    if not databases or any(not p.is_file() or p.suffix != '.db3' for p in databases):
        raise ValueError('TF audit requires an existing SQLite rosbag directory or .db3 file')
    seed_topics = ('/initialpose', '/initialpose_user', '/initialpose_checked')
    seed_counts = {topic: {'recorded': False, 'message_count': 0} for topic in seed_topics}
    tf_recorded, tf_message_count, invalid_tf, duplicates, conflicting_duplicates = False, 0, 0, 0, 0
    odom_samples = {}
    for database in databases:
        connection = sqlite3.connect(database.as_uri()+'?mode=ro', uri=True)
        try:
            topics = {name: (topic_id, kind) for topic_id, name, kind in
                      connection.execute('SELECT id, name, type FROM topics')}
            for topic in seed_topics:
                if topic in topics:
                    seed_counts[topic]['recorded'] = True
                    count = connection.execute('SELECT COUNT(*) FROM messages WHERE topic_id = ?',
                                               (topics[topic][0],)).fetchone()[0]
                    seed_counts[topic]['message_count'] += int(count)
            if '/tf' not in topics:
                continue
            tf_recorded = True
            topic_id, kind = topics['/tf']
            message_type = get_message(kind)
            for (serialized,) in connection.execute(
                    'SELECT data FROM messages WHERE topic_id = ? ORDER BY timestamp', (topic_id,)):
                tf_message_count += 1
                for tf in deserialize_message(serialized, message_type).transforms:
                    if tf.header.frame_id != 'odom' or tf.child_frame_id != 'base_link':
                        continue
                    stamp = int(tf.header.stamp.sec)*1_000_000_000 + int(tf.header.stamp.nanosec)
                    tr, q = tf.transform.translation, tf.transform.rotation
                    pose = dict(zip(('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'),
                                    (tr.x, tr.y, tr.z, q.x, q.y, q.z, q.w)))
                    if stamp <= 0 or not valid_pose(pose):
                        invalid_tf += 1
                        continue
                    if stamp in odom_samples:
                        duplicates += 1
                        old = odom_samples[stamp]
                        error = pose_error(old, pose)
                        if error['xy_m'] > 1e-9 or abs(error['yaw_rad']) > 1e-9 or abs(old['z']-pose['z']) > 1e-9:
                            conflicting_duplicates += 1
                    odom_samples[stamp] = pose
        finally:
            connection.close()

    times = sorted(odom_samples)
    rows, skipped, seen_stamps = [], Counter(), set()
    eligible = [r for r in records if r.get('phase') == 'formal' and not r.get('duplicate_measurement', False)]
    for record in eligible:
        stamp = int(record.get('stamp_ns', 0))
        row = {'match_id': int(record['match_id']), 'cloud_stamp_ns': stamp,
               'valid': False, 'reason': None}
        state, internal = record.get('map_odom_pose'), record.get('final_pose')
        if stamp <= 0:
            row['reason'] = 'invalid_cloud_timestamp'
        elif stamp in seen_stamps:
            row['reason'] = 'duplicate_cloud_timestamp'
        elif not record.get('final_valid') or not valid_pose(internal) or not valid_pose(state):
            row['reason'] = 'invalid_final_diagnostic_or_map_odom_state'
        elif not times:
            row['reason'] = 'no_recorded_odom_to_base_link_tf'
        else:
            index = bisect_left(times, stamp)
            choices = times[max(index-1, 0):min(index+1, len(times))]
            nearest = min(choices, key=lambda value: (abs(value-stamp), value))
            skew = (nearest-stamp)/1e9
            row.update({'odom_tf_stamp_ns': nearest, 'tf_minus_cloud_seconds': skew})
            if abs(skew) > max_skew_seconds:
                row['reason'] = 'odom_tf_outside_max_skew'
            else:
                composed = compose(state, odom_samples[nearest])
                difference = pose_error(internal, composed)
                row.update({'valid': True, 'tf_chain_pose_using_final_match_state': composed,
                            'xy_difference_m': difference['xy_m'],
                            'x_difference_m': difference['x_m'], 'y_difference_m': difference['y_m'],
                            'yaw_difference_deg': math.degrees(difference['yaw_rad'])})
        seen_stamps.add(stamp)
        if row['reason'] is not None:
            skipped[row['reason']] += 1
        rows.append(row)

    good = [r for r in rows if r['valid']]
    return {
        'available': bool(good), 'bag_path': str(path),
        'definition': 'Final NDT map->odom state composed with recorded odom->base_link at/near each input-cloud stamp; compared with the internal final_at_cloud pose.',
        'scope': 'Formal-phase unique NDT input-cloud measurements only',
        'max_skew_seconds': float(max_skew_seconds), 'requested_measurements': len(eligible),
        'matched_measurements': len(good), 'skipped_counts': dict(skipped),
        'tf_topic_recorded': tf_recorded, 'tf_topic_messages': tf_message_count,
        'unique_odom_to_base_tf_samples': len(times), 'invalid_odom_tf_samples': invalid_tf,
        'duplicate_odom_tf_timestamps': duplicates,
        'conflicting_duplicate_odom_tf_timestamps': conflicting_duplicates,
        'duplicate_timestamp_policy': 'Last recorded sample retained; conflicting planar transforms counted explicitly',
        'tf_minus_cloud_seconds': stats([r['tf_minus_cloud_seconds'] for r in good]),
        'internal_vs_tf_chain_xy_difference_m': stats([r['xy_difference_m'] for r in good]),
        'internal_vs_tf_chain_yaw_difference_deg': stats([r['yaw_difference_deg'] for r in good]),
        'initialpose_topics': seed_counts,
        'initialpose_audit_note': 'Topic absent from the bag means unavailable evidence, not proof that no seed was sent. Counts do not establish which node accepted each seed.',
        'geometry_implementation_note': 'Current localizer uses full 3D FAST-LIO sensor-to-base composition; lio_tf_to_base_tf.py emits yaw-only base rotation and planar XY extrinsic compensation. This audit quantifies the resulting difference without changing either implementation.',
        'limitations': [
            'This is an independent geometry audit, not mocap ground-truth accuracy.',
            'The final map->odom state is selected from that match; NDT completion and TF publication happen later. These are not actual map->base TF values broadcast at the earlier cloud timestamp.',
            'Nearest odom TF is allowed only within the configured skew; nonzero skew can contribute temporal differences.',
            'Recorded /tf does not identify the publishing node; conflicting publishers cannot be ruled out solely from message contents.',
        ],
        'measurements': rows,
    }
