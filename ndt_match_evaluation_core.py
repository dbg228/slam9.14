"""Pure geometry and timing helpers for fixed-map NDT evaluation.

This module deliberately has no ROS imports so its time matching and frame
logic can be tested with synthetic data.  All poses are dictionaries with
``x, y, z, qx, qy, qz, qw`` and timestamps are integer nanoseconds.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw(pose: dict) -> float:
    x, y, z, w = (float(pose[key]) for key in ('qx', 'qy', 'qz', 'qw'))
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError('invalid_quaternion')
    x, y, z, w = (value / norm for value in (x, y, z, w))
    return math.atan2(2.0 * (w*z + x*y), 1.0 - 2.0 * (y*y + z*z))


def yaw_quaternion(angle: float) -> tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(angle / 2.0), math.cos(angle / 2.0)


def normalized_quaternion(pose: dict) -> np.ndarray:
    q = np.asarray([pose[key] for key in ('qx', 'qy', 'qz', 'qw')], dtype=float)
    length = np.linalg.norm(q)
    if not np.isfinite(q).all() or length < 1e-9:
        raise ValueError('invalid_quaternion')
    return q / length


def quaternion_multiply(a, b) -> tuple[float, float, float, float]:
    """Hamilton product of two normalized-or-normalizable xyzw quaternions."""
    ax, ay, az, aw = (float(value) for value in a)
    bx, by, bz, bw = (float(value) for value in b)
    result = np.asarray((aw*bx + ax*bw + ay*bz - az*by,
                         aw*by - ax*bz + ay*bw + az*bx,
                         aw*bz + ax*by - ay*bx + az*bw,
                         aw*bw - ax*bx - ay*by - az*bz), dtype=float)
    length = float(np.linalg.norm(result))
    if not np.isfinite(result).all() or length < 1e-9:
        raise ValueError('invalid_quaternion_product')
    return tuple(float(value / length) for value in result)


def rotate_vector(quaternion_xyzw, vector_xyz) -> tuple[float, float, float]:
    """Rotate a 3-vector by an xyzw quaternion without assuming planar motion."""
    q = np.asarray(quaternion_xyzw, dtype=float)
    vector = np.asarray(vector_xyz, dtype=float)
    length = float(np.linalg.norm(q))
    if q.shape != (4,) or vector.shape != (3,) or not np.isfinite(q).all() or not np.isfinite(vector).all() or length < 1e-9:
        raise ValueError('invalid_rotation_input')
    x, y, z, w = q / length
    # Equivalent to q * (vector, 0) * inverse(q), expanded to avoid an
    # ambiguous quaternion-vector convention at the call site.
    cross = np.cross((x, y, z), vector)
    rotated = vector + 2.0 * (w * cross + np.cross((x, y, z), cross))
    return tuple(float(value) for value in rotated)


def rigid_body_to_base_pose(pose: dict, tracker_to_base_translation_m, tracker_to_base_quaternion_xyzw) -> dict:
    """Apply ``T_tracker_base`` to a world pose ``T_world_tracker``.

    ``tracker_to_base_translation_m`` is the base-link origin expressed in
    Tracker axes.  The result is the base-link pose in the same world frame.
    This is a full 3-D rigid transform, not a frame-id relabeling and not a
    planar approximation.
    """
    translation = np.asarray(tracker_to_base_translation_m, dtype=float)
    rotation = np.asarray(tracker_to_base_quaternion_xyzw, dtype=float)
    if translation.shape != (3,) or rotation.shape != (4,) or not np.isfinite(translation).all() or not np.isfinite(rotation).all():
        raise ValueError('invalid_tracker_to_base_transform')
    tracker_rotation = normalized_quaternion(pose)
    offset_world = rotate_vector(tracker_rotation, translation)
    output = dict(pose)
    output['x'] = float(pose['x']) + offset_world[0]
    output['y'] = float(pose['y']) + offset_world[1]
    output['z'] = float(pose.get('z', 0.)) + offset_world[2]
    output['qx'], output['qy'], output['qz'], output['qw'] = quaternion_multiply(tracker_rotation, rotation)
    output['valid'] = bool(pose.get('valid', True))
    return output


def slerp(a: dict, b: dict, ratio: float) -> tuple[float, float, float, float]:
    qa, qb = normalized_quaternion(a), normalized_quaternion(b)
    dot = float(np.dot(qa, qb))
    if dot < 0.0:
        qb, dot = -qb, -dot
    dot = max(-1.0, min(1.0, dot))
    if dot > 0.9995:
        value = qa + ratio * (qb - qa)
        value /= np.linalg.norm(value)
    else:
        theta = math.acos(dot)
        value = (math.sin((1.0-ratio)*theta) * qa + math.sin(ratio*theta) * qb) / math.sin(theta)
    return tuple(float(item) for item in value)


def interpolate_pose(samples: list[dict], stamp_ns: int, max_gap_seconds: float) -> tuple[dict | None, str | None]:
    """Interpolate only between adjacent valid samples within one short gap."""
    if stamp_ns <= 0:
        return None, 'invalid_target_timestamp'
    usable = [item for item in samples if item.get('valid', True) and int(item.get('stamp_ns', 0)) > 0]
    usable.sort(key=lambda item: int(item['stamp_ns']))
    if len(usable) < 2:
        return None, 'insufficient_mocap_samples'
    for left, right in zip(usable, usable[1:]):
        first, second = int(left['stamp_ns']), int(right['stamp_ns'])
        if stamp_ns == first:
            return dict(left), None
        if first < stamp_ns < second:
            gap = (second-first) / 1e9
            if gap > max_gap_seconds:
                return None, 'mocap_interpolation_gap_too_large'
            ratio = (stamp_ns-first) / (second-first)
            qx, qy, qz, qw = slerp(left, right, ratio)
            return {
                'stamp_ns': stamp_ns, 'valid': True,
                'x': float(left['x']) + ratio*(float(right['x'])-float(left['x'])),
                'y': float(left['y']) + ratio*(float(right['y'])-float(left['y'])),
                'z': float(left.get('z', 0.)) + ratio*(float(right.get('z', 0.))-float(left.get('z', 0.))),
                'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw,
            }, None
    if stamp_ns == int(usable[-1]['stamp_ns']):
        return dict(usable[-1]), None
    return None, 'mocap_timestamp_outside_recording'


@dataclass(frozen=True)
class Se2Alignment:
    yaw_rad: float
    tx_m: float
    ty_m: float
    mocap_to_base_yaw_rad: float
    method: str

    def apply(self, pose: dict) -> dict:
        c, s = math.cos(self.yaw_rad), math.sin(self.yaw_rad)
        x, y = float(pose['x']), float(pose['y'])
        qx, qy, qz, qw = yaw_quaternion(wrap(yaw(pose) + self.yaw_rad + self.mocap_to_base_yaw_rad))
        return {'x': c*x-s*y+self.tx_m, 'y': s*x+c*y+self.ty_m,
                'z': float(pose.get('z', 0.)), 'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw}


def estimate_start_alignment(pairs: list[tuple[dict, dict]], mocap_to_base_yaw_rad: float,
                             max_static_motion_m: float) -> Se2Alignment:
    """Estimate one fixed map<-mocap SE(2) transform from a static start window.

    ``pairs`` are (raw mocap base-center pose, final map base_link pose) at the
    same timestamp.  The separately measured tracker-to-base yaw is required;
    it is not silently assumed to be zero.
    """
    if len(pairs) < 3:
        raise ValueError('alignment_needs_at_least_three_time_matched_poses')
    for raw, mapped in pairs:
        for value in (*raw.values(), *mapped.values(), mocap_to_base_yaw_rad):
            if isinstance(value, (int, float)) and not math.isfinite(value):
                raise ValueError('nonfinite_alignment_input')
    raw_xy = np.asarray([[row[0]['x'], row[0]['y']] for row in pairs], dtype=float)
    map_xy = np.asarray([[row[1]['x'], row[1]['y']] for row in pairs], dtype=float)
    # This is intentionally a static alignment window.  If it is not static,
    # translation/yaw would conflate motion with a calibration transform.
    if max(np.linalg.norm(raw_xy-row, axis=1).max() for row in [raw_xy.mean(axis=0)]) > max_static_motion_m:
        raise ValueError('mocap_alignment_window_is_not_static')
    if max(np.linalg.norm(map_xy-row, axis=1).max() for row in [map_xy.mean(axis=0)]) > max_static_motion_m:
        raise ValueError('map_alignment_window_is_not_static')
    angles = [wrap(yaw(mapped) - yaw(raw) - mocap_to_base_yaw_rad) for raw, mapped in pairs]
    theta = math.atan2(sum(math.sin(value) for value in angles), sum(math.cos(value) for value in angles))
    c, s = math.cos(theta), math.sin(theta)
    translations = [(mapped['x'] - c*raw['x'] + s*raw['y'],
                     mapped['y'] - s*raw['x'] - c*raw['y']) for raw, mapped in pairs]
    return Se2Alignment(theta, float(np.mean([item[0] for item in translations])),
                        float(np.mean([item[1] for item in translations])),
                        mocap_to_base_yaw_rad, 'start_static_window')


def pose_error(reference: dict, estimate: dict) -> dict:
    dx, dy = float(estimate['x'])-float(reference['x']), float(estimate['y'])-float(reference['y'])
    return {'x_m': dx, 'y_m': dy, 'xy_m': math.hypot(dx, dy),
            'yaw_rad': wrap(yaw(estimate)-yaw(reference))}


def error_metrics(rows: list[dict], prefix: str) -> dict:
    usable = [row for row in rows if row.get(prefix+'_error') is not None]
    if not usable:
        return {'samples': 0, 'metrics_available': False}
    result = {'samples': len(usable), 'metrics_available': True,
              'weighting': 'one record per NDT input-cloud timestamp'}
    for name in ('x_m', 'y_m', 'xy_m', 'yaw_rad'):
        values = np.asarray([row[prefix+'_error'][name] for row in usable], dtype=float)
        output_name, scale = ('yaw_deg', 180.0/math.pi) if name == 'yaw_rad' else (name, 1.0)
        values *= scale
        result[output_name] = {'mean': float(np.mean(values)), 'rmse': float(np.sqrt(np.mean(values**2))),
                               'p95_absolute': float(np.percentile(np.abs(values), 95)),
                               'max_absolute': float(np.max(np.abs(values)))}
    return result
