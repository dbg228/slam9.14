"""Transform direction and genuinely held-out residuals, with synthetic poses only."""
import importlib.util
import json
import math
from pathlib import Path
import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[3]
spec=importlib.util.spec_from_file_location('map_mocap_check',ROOT/'tests/controller_tracking/tools/map_mocap_check.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def fixture(path,x=0.,y=0.,error=0.):
    samples=[];transform=module.Se2Alignment(.7,2.,-1.)
    for i in range(101):
        source=dict(stamp_ns=10**9+i*10_000_000,valid=True,x=x,y=y,z=0.,qx=0.,qy=0.,qz=0.,qw=1.)
        target=transform.apply(source);target['x']+=error
        samples.append(dict(mocap_base=source,map_base=target))
    doc=dict(completed=True,samples=samples,mount_sha256='synthetic_same_mount',calibration_id='software_fixture',final_command_max_abs=0.)
    path.write_text(json.dumps(doc));return doc


def test_direction_and_inverse(tmp_path):
    capture=tmp_path/'start.json';fixture(capture)
    result=module.align(capture,tmp_path/'candidate.json')
    forward=module.Se2Alignment(**result['map_from_mocap']);inverse=module.Se2Alignment(**result['mocap_from_map'])
    assert abs(forward.yaw_rad-.7)<1e-12
    point=dict(stamp_ns=1,valid=True,x=-2.,y=.4,z=0.,qx=0.,qy=0.,qz=0.,qw=1.)
    recovered=inverse.apply(forward.apply(point))
    assert np.allclose([recovered['x'],recovered['y']],[-2.,.4],atol=1e-12)


def test_holdout_cannot_realign_away_error(tmp_path):
    start=tmp_path/'start.json';fixture(start);candidate=tmp_path/'candidate.json';module.align(start,candidate)
    with pytest.raises(ValueError,match='Training capture'):module.verify(candidate,[start],tmp_path/'bad.json')
    second=tmp_path/'second.json';fixture(second,x=1.,y=1.,error=.08)
    result=module.verify(candidate,[second],tmp_path/'verification.json')
    assert result['transform_refit'] is False
    assert abs(result['capture_results'][0]['xy_m']['rmse']-.08)<1e-12
    assert result['sufficient_noncollinear_coverage'] is False


def test_long_gap_and_moving_holdout_are_rejected(tmp_path):
    path=tmp_path/'sample.json';doc=fixture(path)
    doc['samples']=doc['samples'][:20]+doc['samples'][60:]
    with pytest.raises(ValueError,match='long-gap'):module.valid_pairs(doc)
    doc=fixture(path);doc['samples'][-1]['mocap_base']['x']+=.2
    with pytest.raises(ValueError,match='not static'):module.valid_pairs(doc)


def test_spatial_report_retains_failed_attempt_without_zero_fill_or_refit(tmp_path):
    start=tmp_path/'start.json';fixture(start)
    candidate=tmp_path/'candidate.json';module.align(start,candidate);before=candidate.read_bytes()
    fixture(tmp_path/'point02_attempt01.json',x=1.,y=1.,error=.08)
    failed=tmp_path/'point03_attempt01.json';doc=fixture(failed,x=0.,y=1.)
    doc['completed']=False;doc['samples']=[];doc['rejections']={'no_fresh_local_tf':10};failed.write_text(json.dumps(doc))
    good=tmp_path/'point02_attempt01.json';doc=json.loads(good.read_text());doc['rejections']={};good.write_text(json.dumps(doc))
    result=module.report_session(tmp_path)
    assert result['attempt_count']==2 and result['quality_passed_count']==1
    assert abs(result['quality_passed_pooled_descriptive_xy_m']['rmse']-.08)<1e-12
    assert result['attempts'][1]['xy_m']['rmse'] is None
    assert result['sufficient_noncollinear_coverage'] is False
    assert result['absolute_calibration_validated'] is False
    assert candidate.read_bytes()==before
