"""Selection and repeatability statistics for clean, cold-start NDT trials.

This module intentionally has no ROS imports.  A *cold start* is a test
procedure claim (the localizer was completely stopped and restarted before a
trial), not something that can be inferred from a diagnostic message alone.
The helpers retain the first real registration, including a rejected one, so
that a first-match failure cannot disappear from a repeatability summary.
"""
from __future__ import annotations

import math

import numpy as np

from ndt_match_evaluation_core import wrap, yaw


# These diagnostic stages happen before PCL NDT is entered.  They must not be
# called a "first NDT match" merely because they carry an incrementing ID.
NON_REGISTRATION_STAGES = frozenset((
    'waiting_for_initialization', 'no_input_cloud', 'source_below_minimum',
    'submap_below_minimum', 'no_odom_pose_at_cloud_time',
))


def is_registration(record: dict) -> bool:
    """Whether this diagnostic represents an entered NDT registration.

    The node records explicit failure stages such as ``ndt_exception`` and
    ``not_converged``.  They remain registrations and deliberately stay in
    the first-match result.  Unknown future stages are also retained rather
    than silently filtered out.
    """
    return str(record.get('stage', '')).strip() not in NON_REGISTRATION_STAGES


def first_registration(records: list[dict]) -> dict | None:
    """Return the earliest actual registration by cloud time then match ID."""
    candidates = [record for record in records if is_registration(record)]
    if not candidates:
        return None
    return min(candidates, key=lambda item: (int(item.get('stamp_ns', 0)), int(item.get('match_id', 0))))


def first_accepted_registration(records: list[dict]) -> dict | None:
    """Return the earliest accepted NDT registration, if one was recorded."""
    candidates = [record for record in records
                  if is_registration(record) and bool(record.get('accepted'))]
    if not candidates:
        return None
    return min(candidates, key=lambda item: (int(item.get('stamp_ns', 0)), int(item.get('match_id', 0))))


def _finite_raw_pose(record: dict) -> dict | None:
    pose = record.get('raw_pose')
    keys = ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')
    if not bool(record.get('raw_valid')) or not isinstance(pose, dict):
        return None
    try:
        if not all(math.isfinite(float(pose[key])) for key in keys):
            return None
        yaw(pose)  # Validate the quaternion as well.
    except (KeyError, TypeError, ValueError):
        return None
    return pose


def raw_repeatability(records: list[dict]) -> dict:
    """Summarize raw-NDT scatter about its centroid, not absolute error.

    Every record must be the first registration of one cold-start trial at
    the *same physical parking position*.  A constant map/mocap transform
    does not affect this scatter.  It therefore quantifies first-match
    repeatability but intentionally cannot reveal a common absolute bias.
    """
    usable = [(record, _finite_raw_pose(record)) for record in records]
    usable = [(record, pose) for record, pose in usable if pose is not None]
    result = {
        'raw_pose_samples': len(usable),
        'metrics_available': False,
        'definition': ('Scatter of first raw NDT map->base_link poses about their own centroid. '
                       'This is cold-start repeatability, not absolute localization error.'),
    }
    if not usable:
        return result
    xy = np.asarray([[float(pose['x']), float(pose['y'])] for _, pose in usable], dtype=float)
    center = np.mean(xy, axis=0)
    residuals = xy-center
    radii = np.linalg.norm(residuals, axis=1)
    headings = np.asarray([yaw(pose) for _, pose in usable], dtype=float)
    mean_heading = math.atan2(float(np.sin(headings).sum()), float(np.cos(headings).sum()))
    heading_residuals = np.asarray([wrap(value-mean_heading) for value in headings], dtype=float)
    result.update({
        'metrics_available': True,
        'centroid_map_xy_m': [float(center[0]), float(center[1])],
        'mean_raw_yaw_deg': math.degrees(mean_heading),
        'x_std_m': float(np.std(xy[:, 0], ddof=0)),
        'y_std_m': float(np.std(xy[:, 1], ddof=0)),
        'xy_scatter_rmse_m': float(math.sqrt(float(np.mean(radii*radii)))),
        'xy_scatter_p95_m': float(np.percentile(radii, 95)),
        'xy_scatter_max_m': float(np.max(radii)),
        'yaw_std_deg': math.degrees(float(np.std(heading_residuals, ddof=0))),
        'yaw_p95_absolute_deg': math.degrees(float(np.percentile(np.abs(heading_residuals), 95))),
        'yaw_max_absolute_deg': math.degrees(float(np.max(np.abs(heading_residuals)))),
    })
    return result


def summarize_cold_start_trials(trials: list[dict]) -> dict:
    """Count first-registration outcomes and summarize accepted raw poses."""
    first = [trial['first_registration'] for trial in trials if trial.get('first_registration') is not None]
    accepted = [record for record in first if bool(record.get('accepted'))]
    return {
        'trial_count': len(trials),
        'trials_with_registration': len(first),
        'trials_without_registration': len(trials)-len(first),
        'first_registration_accepted_count': len(accepted),
        'first_registration_accepted_rate': len(accepted)/len(first) if first else 0.0,
        'first_registration_converged_count': sum(bool(record.get('converged')) for record in first),
        # All registration outcomes are retained in the CSV.  Raw-pose scatter
        # naturally requires an available raw output, irrespective of whether
        # a separate acceptance policy later rejected it.
        'first_registration_raw_pose_repeatability': raw_repeatability(first),
    }
