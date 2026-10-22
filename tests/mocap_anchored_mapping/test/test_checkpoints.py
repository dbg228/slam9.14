"""Checkpoint frame direction, retained drift and relocated-camera rejection."""
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from mocap_anchored_mapping.checkpoints import correction_report,next_checkpoint,require_camera_world,service_name
from mocap_anchored_mapping.core import inverse,load,record,save
from test_mapping import SITE,transform,pairs


def test_moved_camera_new_calibration_and_checkpoint_drift():
    cfg=load(SITE)
    new_world=transform([4,-3,.8],[.04,-.03,1.2])
    old_world=transform([-1,2,.5],[.02,.01,-.6])
    odom=transform([.6,-.4,.1],[.01,-.02,.7])
    body=transform([.3,.8,.1],[.01,.02,-.3])
    drift=transform([.03,-.04,0],[0,0,0])
    observations=pairs(new_world@drift@odom,body)
    calibration=dict(T_map_mocap=record(inverse(new_world),'map','mocap_world'))
    candidate,report=correction_report(observations,cfg,calibration,odom)
    np.testing.assert_allclose(candidate,drift@odom,atol=1e-12)
    assert report['before']['distance_XY_m']['rmse']==pytest.approx(.05)
    assert report['after_same_window_consistency']['distance_XY_m']['rmse']<1e-12
    assert not report['independent_accuracy_verified'] and not report['world_calibration_refitted']
    # Old calibration must not accidentally give a plausible "zero error".
    wrong,_=correction_report(observations,cfg,dict(T_map_mocap=record(inverse(old_world),'map','mocap_world')),odom)
    assert np.linalg.norm(wrong-candidate)>1
    # The correction composes with later local motion without another mocap sample.
    later=transform([2,1,.1],[.02,-.01,.6])
    np.testing.assert_allclose(candidate@later,drift@odom@later,atol=1e-12)


def test_inverted_reference_cannot_be_applied_as_a_correction():
    cfg=load(SITE)
    observations=pairs(transform([0,0,0],[np.pi,0,0]),transform([0,0,0],[0,0,0]))
    with pytest.raises(ValueError,match='checkpoint_reference_not_upright'):
        correction_report(observations,cfg,dict(T_map_mocap=record(np.eye(4),'map','mocap_world')),np.eye(4))


def test_camera_relocation_rejects_old_run_but_preserves_it(tmp_path):
    active=tmp_path/'robot_data/config/mocap_world_session.yaml'
    save(active,dict(camera_world_id='relocated_shared_world'),True)
    cfg=dict(project_root=str(tmp_path),camera_world_id='previous_world')
    with pytest.raises(ValueError,match='Camera world changed'):require_camera_world(cfg)
    cfg['camera_world_id']='relocated_shared_world';require_camera_world(cfg)
    assert load(active)['camera_world_id']=='relocated_shared_world'


def test_order_and_session_specific_service_identity(tmp_path):
    assert next_checkpoint([])=='middle'
    assert next_checkpoint(['middle'])=='end'
    assert next_checkpoint(['middle','end']) is None
    with pytest.raises(ValueError):next_checkpoint(['end'])
    assert service_name(tmp_path/'first','middle')!=service_name(tmp_path/'second','middle')
    with pytest.raises(ValueError):service_name(tmp_path,'arbitrary')


def test_checkpoint_map_odom_dynamic_only():
    pytest.importorskip('rclpy')
    from mocap_anchored_mapping.checkpoint_online import CheckpointBridge
    captured=[]
    state=SimpleNamespace(static=SimpleNamespace(sendTransform=captured.append),
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(to_msg=lambda:None)))
    # Stamp must be a ROS Time message even for a software fixture.
    from builtin_interfaces.msg import Time
    state.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(to_msg=lambda:Time()))
    CheckpointBridge.publish_initial_transforms(state)
    assert [(t.header.frame_id,t.child_frame_id) for t in captured]==[('odom','camera_init')]


def test_active_or_undiscovered_navigation_blocks_correction():
    pytest.importorskip('rclpy')
    from mocap_anchored_mapping.checkpoint_online import CheckpointBridge
    topic='/follow_path/_action/status'
    state=SimpleNamespace(nav_goals={},get_publishers_info_by_topic=lambda name:[object()] if name==topic else [])
    assert not CheckpointBridge.navigation_idle(state)
    for status in (1,2,3):
        state.nav_goals={topic:[status]};assert not CheckpointBridge.navigation_idle(state)
    state.nav_goals={topic:[4,5,6]};assert CheckpointBridge.navigation_idle(state)


def test_delayed_native_warmup_keeps_new_mocap_history():
    pytest.importorskip('rclpy')
    from collections import deque
    import time
    from mocap_anchored_mapping.online import NativeBridge
    cfg=load(SITE)
    clock=SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=3_000_000_000))
    history=deque([(2_000_000_000,np.eye(4))])
    state=SimpleNamespace(fault=None,done=False,collecting=lambda:True,cfg=cfg,
        start=time.monotonic(),live_ok=True,graph_verified=True,
        last_command=np.zeros(6),command_at=time.monotonic(),
        pending=deque([(1_000_000_000,np.eye(4),None)]),mc=history,pairs=[],
        get_clock=lambda:clock)
    state.invalid=lambda reason:pytest.fail('A capture warmup must retain newer mocap: '+reason)
    NativeBridge.tick(state)
    assert not state.pending
    assert list(state.mc)==list(history) and len(state.mc)==1


def test_heldout_uses_current_checkpoint_and_rejects_version_change(tmp_path):
    pytest.importorskip('rclpy')
    import json
    from std_msgs.msg import String
    from mocap_anchored_mapping.online import NativeBridge
    session=tmp_path/'localizations/session'
    state=SimpleNamespace(run=tmp_path,phase='localization',session=session,checkpoint_validation=True,
        validation_version=None,live_ok=False,fault=None)
    state.fail=lambda reason:setattr(state,'fault',reason)
    h=transform([.03,-.04,0],[0,0,.2])
    message=dict(run_path=str(tmp_path),capture_dir=str(session),frozen=True,fault=None,
        active_checkpoint=None,map_odom_version=1,T_map_odom=record(h,'map','odom'))
    NativeBridge.live_status(state,String(data=json.dumps(message)))
    np.testing.assert_allclose(state.fixed_map_odom,h)
    assert state.live_ok and state.validation_version==1
    message['map_odom_version']=2
    NativeBridge.live_status(state,String(data=json.dumps(message)))
    assert state.fault=='checkpoint_transform_changed_during_heldout_validation'
