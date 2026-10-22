"""Offline checkpoint calculation and camera-world identity; no ROS imports."""
import hashlib
from pathlib import Path
import numpy as np
from .core import estimate, initial_map_odom, load, record, validation_errors


def require_camera_world(cfg):
    if str(cfg.get('data_kind','')).startswith('software_fixture'):
        return
    active=Path(cfg['project_root'])/'robot_data/config/mocap_world_session.yaml'
    if active.is_file():
        current=load(active)
        if cfg.get('camera_world_id')!=current['camera_world_id']:
            raise ValueError('Camera world changed: old map/mocap calibration is historical. Prepare a new mapping run or independently recalibrate this map to the current camera world.')


def service_name(session, checkpoint):
    if checkpoint not in ('middle','end'):
        raise ValueError('Checkpoint must be middle or end')
    identity=hashlib.sha256(str(Path(session).resolve()).encode()).hexdigest()[:16]
    return f'/anchored_localization/checkpoints/s{identity}/{checkpoint}'


def correction_report(pairs, cfg, calibration, previous):
    if any(np.asarray(pair['T_M_B'])[2,2] < np.cos(np.radians(45.)) for pair in pairs):
        raise ValueError('checkpoint_reference_not_upright')
    result=estimate(pairs,cfg)
    candidate=initial_map_odom(calibration,result['T_mocap_native']['matrix'])
    before=validation_errors(pairs,calibration,previous)
    after=validation_errors(pairs,calibration,candidate)
    from controller_tracking.common import stats
    for summary,transform in ((before,previous),(after,candidate)):
        errors=[]
        for pair in pairs:
            truth=np.asarray(calibration['T_map_mocap']['matrix'])@np.asarray(pair['T_M_B'])
            predicted=np.asarray(transform)@np.asarray(pair['T_F_B'])
            delta=np.arctan2(predicted[1,0],predicted[0,0])-np.arctan2(truth[1,0],truth[0,0])
            errors.append(float(np.degrees(np.arctan2(np.sin(delta),np.cos(delta)))))
        summary['yaw_error_deg']=stats(errors)
    return candidate,dict(
        T_map_odom_before=record(previous,'map','odom'),
        T_map_odom_after=record(candidate,'map','odom'),
        before=before,after_same_window_consistency=after,
        fit=result,reference_point='base_link',
        world_calibration_refitted=False,PCD_coordinates_changed=False,
        independent_accuracy_verified=False,
        evaluation_note='The after value uses correction-fitting samples and measures consistency. Report arrival error from before; use a separate held-out window for residual accuracy.')


def next_checkpoint(completed):
    if list(completed)==[]:return 'middle'
    if list(completed)==['middle']:return 'end'
    if list(completed)==['middle','end']:return None
    raise ValueError('Unexpected checkpoint history')
