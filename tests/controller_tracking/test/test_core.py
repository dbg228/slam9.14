import copy
import json
import math
from pathlib import Path
import numpy as np
import pytest
from controller_tracking.configuration import load,nav_overlay,differences,snapshot,check_session
from controller_tracking.geometry import geometry,swept,site_check
from controller_tracking.state import base_in_level,to_test,freeze_anchor,VelocityFit
from controller_tracking.common import interpolate,project,wrap,digest,write_json
from controller_tracking.gate import GateLogic

ROOT=Path(__file__).resolve().parents[3]

@pytest.fixture
def cfg():return load(ROOT/'tests/controller_tracking/config/experiment.yaml')

def test_mount_full_rotation_then_ground_level():
    mount=dict(measured=True,translation_m=[1,0,-.18],quaternion_xyzw=[0,0,math.sin(math.pi/4),math.cos(math.pi/4)])
    p,z=base_in_level([2,3,.18],[0,0,math.sin(math.pi/4),math.cos(math.pi/4)],mount,[0,0,0,1])
    assert np.allclose(p[:2],[2,4]);assert abs(wrap(p[2]-math.pi))<1e-12;assert abs(z)<1e-12
    assert np.allclose(to_test(p,[2,4,math.pi]),[0,0,0],atol=1e-12)
    with pytest.raises(ValueError):base_in_level([0,0,0],[0,0,0,0],mount,[0,0,0,1])
    with pytest.raises(ValueError):base_in_level([0,0,0],[.7,0,0,.7],mount,[0,0,0,1])

def test_angle_interpolation_and_dropouts():
    rows=[[0,0,0,math.radians(179)],[.02,1,0,math.radians(-179)],[1,2,0,0]]
    out=interpolate(rows,[-.01,.01,.5,1.1],.05)
    assert abs(abs(out[1,2])-math.pi)<1e-12;assert np.isnan(out[[0,2,3]]).all()
    with pytest.raises(ValueError):interpolate([[0,0,0,0],[0,0,0,0]],[0],.1)

def test_velocity_consistency_and_gap_reset():
    fit=VelocityFit(.10,.03);last=None
    for t in np.arange(0,.201,.01):last=fit.add(t,[.3*t,0,math.radians(179)+.1*t])
    t,p,v,delay=last
    assert abs(delay-.05)<.006;assert np.allclose(p,[.3*t,0,wrap(math.radians(179)+.1*t)])
    assert abs(v[2]-.1)<1e-8;assert abs(np.linalg.norm(v[:2])-.3)<1e-7
    with pytest.raises(ValueError):fit.add(.5,[0,0,0])
    assert fit.rows==[]
    assert fit.add(.51,[0,0,0]) is None

def test_stationary_freeze_and_yaw_boundary():
    rows=[[t,1,2,math.radians(179 if i%2 else -179)] for i,t in enumerate(np.linspace(0,1,101))]
    anchor=freeze_anchor(rows,yaw_limit=.04);assert abs(abs(anchor[2])-math.pi)<.001
    rows[-1][1]+=.2
    with pytest.raises(ValueError):freeze_anchor(rows)

@pytest.mark.parametrize('kind',['line','left_arc','right_arc'])
def test_geometry_direction_tangent_and_curvature(cfg,kind):
    path=geometry(cfg,kind);p=np.array(path['points']);s=np.array(path['local']['s']);k=np.array(path['local']['curvature'])
    assert np.all(np.diff(s)>0);assert max(np.diff(s))<=cfg['geometry']['ds']+1e-9
    assert np.allclose(p[0],[0,0,0]);assert abs(k[0])<1e-12;assert abs(k[-1])<1e-12
    tangent=np.arctan2(np.diff(p[:,1]),np.diff(p[:,0]));mid=(p[1:,2]+p[:-1,2])/2
    assert max(abs(wrap(tangent-mid)))<.001
    if kind!='line':
        sign=1 if kind=='left_arc' else -1
        assert abs(p[-1,2]-sign*math.radians(cfg['geometry']['angle_deg']))<1e-6
        assert sign*p[-1,1]>0
        # All joins have continuous curvature; endpoint of central circle has no corner.
        assert max(abs(np.diff(k)))<.11

def test_segment_projection_not_nearest_discrete(cfg):
    path=geometry(cfg,'line');path['points']=[[0,0,math.pi-.01],[1,0,-math.pi+.01]];path['local']['s']=[0,1]
    result=project([[.5,.1,math.pi]],path)[0]
    assert abs(result[0]-.5)<1e-12;assert abs(result[3]-.1)<1e-12;assert abs(result[2])<1e-12

def test_sweep_parking_along_final_tangent(cfg):
    cfg['geometry']['angle_deg']=90
    path=geometry(cfg,'left_arc');h=swept(path,cfg)
    delta=h[-1].mean(axis=0)-h[-2].mean(axis=0)
    assert abs(delta[0])<1e-6;assert abs(delta[1]-.5)<1e-6
    assert not all(site_check(path,cfg).values())

def test_only_wiring_and_probe_differ(cfg,tmp_path):
    import yaml
    nav=yaml.safe_load(Path(cfg['production_nav']).read_text());overlay=nav_overlay(nav,cfg['frame'])['tracking']
    diff=differences({k:nav[k] for k in overlay},overlay)
    assert len(diff)==4
    assert overlay['controller_server']['ros__parameters']['FollowPath']['critics']==nav['controller_server']['ros__parameters']['FollowPath']['critics']
    assert overlay['controller_server']['ros__parameters']['FollowPath']['vx_max']==.3
    snapshot(cfg,tmp_path)
    frame={'config_sha256':digest(cfg)};write_json(tmp_path/'frame.json',frame)
    identity=json.loads((tmp_path/'identity.json').read_text());identity['frame_sha256']=digest(frame);write_json(tmp_path/'identity.json',identity)
    check_session(tmp_path)
    frame['changed']=True;write_json(tmp_path/'frame.json',frame)
    with pytest.raises(ValueError):check_session(tmp_path)

def test_gate_fault_latches_no_auto_resume():
    gate=GateLogic();gate.mode='EXTERNAL';gate.mode_received=1.;gate.state=1.;gate.state_valid=True
    gate.arm(1.);gate.command=1.05;gate.lease=1.05
    assert gate.evaluate(1.05)=='armed'
    gate.state_valid=False
    assert gate.evaluate(1.06)=='mocap_invalid_or_stale'
    gate.state_valid=True;gate.state=1.07
    assert gate.evaluate(1.07)=='mocap_invalid_or_stale'

def test_gate_lease_expiry_and_stationary_requirement():
    gate=GateLogic();gate.state_valid=True;gate.state=1;gate.mode='EXTERNAL';gate.mode_received=1;gate.v=[.1,0,0]
    with pytest.raises(ValueError):gate.arm(1)
    gate.v=[0,0,0];gate.arm(1);gate.state=1.4;gate.mode_received=1.4;gate.command=1.4
    assert gate.evaluate(1.4)=='execution_lease_expired'
