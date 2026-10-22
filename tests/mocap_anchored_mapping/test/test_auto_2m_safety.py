"""The 2m relay requires all independent live inputs and preserves operator override."""
from pathlib import Path
import importlib.util
import pytest

pytest.importorskip('rclpy')
spec=importlib.util.spec_from_file_location('auto_2m_trial',Path(__file__).resolve().parents[1]/'tools/auto_2m_trial.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def fresh():return {k:9.99 for k in ['request','permit','mocap','native','status','mode']}


def test_all_inputs_and_external_required():
    assert module.relay_ready(10.,fresh(),'EXTERNAL',True,True,'')
    assert not module.relay_ready(10.,fresh(),'MANUAL',True,True,'')
    assert not module.relay_ready(10.,fresh(),'STOP',True,True,'')
    assert not module.relay_ready(10.,fresh(),'EXTERNAL',False,True,'')
    assert not module.relay_ready(10.,fresh(),'EXTERNAL',True,False,'')
    assert not module.relay_ready(10.,fresh(),'EXTERNAL',True,True,'latched_failure')


def test_native_watchdog_matches_original_half_second_without_changing_other_limits():
    r=fresh();r['native']=9.6
    assert module.relay_ready(10.,r,'EXTERNAL',True,True,'')
    r['native']=9.499
    assert not module.relay_ready(10.,r,'EXTERNAL',True,True,'')
    assert module.RELAY_MAX_AGE_S=={'request':.20,'permit':.30,'mocap':.20,'native':.50,'status':.50,'mode':.50}


@pytest.mark.parametrize('key',list(fresh()))
def test_each_stale_missing_or_future_input_blocks_motion(key):
    for value in [9.,11.,None]:
        r=fresh()
        if value is None:del r[key]
        else:r[key]=value
        assert not module.relay_ready(10.,r,'EXTERNAL',True,True,'')


def test_analysis_excludes_cleanup_and_keeps_missing_reference_empty(tmp_path):
    import csv
    import json
    from types import SimpleNamespace
    import numpy as np
    def at(x,y):
        h=np.eye(4);h[:2,3]=[x,y];return h
    (tmp_path/'calibration.yaml').write_text('{}\n')
    (tmp_path/'initialization.yaml').write_text('{}\n')
    times=[.5,1.,2.,3.,7.,9.]
    estimates=[(int(t*1e9),at(.03,.04) if t in (1.,2.,7.) else at(2.,0.)) for t in times]
    reference=[(int(t*1e9),np.eye(4)) for t in times if t!=3.]
    node=SimpleNamespace(trial=tmp_path,run=tmp_path,
        meta={'stage':'software_fixture','data_kind':'software_fixture','localization_session':str(tmp_path)},
        cal={'T_map_mocap':{'matrix':np.eye(4).tolist()}},estimates=estimates,mc=reference,
        valid_stamps={t for t,h in reference},native_ages={},state={},guard={},
        events=[{'event':'motion_start','ros_stamp_ns':10**9,'start_base_pose':{'x':0.,'y':0.}},
                {'event':'stop_command','ros_stamp_ns':2*10**9}])
    result=module.analyze(node)
    assert result['reference_coverage']==pytest.approx(.75)
    assert result['XY_error_m']['n']==3 and result['XY_error_m']['rmse']==pytest.approx(.05)
    assert result['phases']['cleanup']['total']==1
    rows=list(csv.DictReader((tmp_path/'samples.csv').open()))
    missing=next(r for r in rows if r['stamp_ns']==str(3*10**9))
    assert missing['reference_valid']=='False' and missing['error_xy_m']==''
    (tmp_path/'samples.csv').unlink();node.meta['export_csv']=False
    second=module.analyze(node)
    assert not (tmp_path/'samples.csv').exists()
    assert second['XY_error_m']==result['XY_error_m']
    readonly,raw_rows=module.analyze(node,write_outputs=False)
    assert readonly['XY_error_m']==result['XY_error_m'] and len(raw_rows)==len(rows)


def test_startup_stale_zero_never_refreshes_watchdog_but_nonzero_and_running_zero_fail():
    from types import SimpleNamespace
    msg=module.TwistStamped();msg.header.frame_id='base_link';msg.header.stamp.sec=8
    state=SimpleNamespace(ever_enabled=False,last_stamp=9*10**9,latest=None,failure='',allowed_speed=-.15)
    state.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=10*10**9))
    state.get_logger=lambda:SimpleNamespace(warning=lambda text:None)
    state.fail=lambda reason:setattr(state,'failure',reason)
    module.OperatorGuard.request(state,msg)
    assert state.failure=='' and state.latest is None and state.last_stamp==9*10**9
    msg.twist.linear.x=-.15;module.OperatorGuard.request(state,msg)
    assert state.failure=='old_or_invalid_request' and state.latest is None
    state.failure='';state.ever_enabled=True;msg.twist.linear.x=0.
    module.OperatorGuard.request(state,msg)
    assert state.failure=='old_or_invalid_request' and state.latest is None


def test_valid_zero_after_motion_is_a_stop_not_a_guard_failure(monkeypatch):
    from types import SimpleNamespace
    stops=[];failures=[]
    state=SimpleNamespace(latest=9.99,permit_time=9.99,mocap_time=9.99,received=fresh(),
        mode_value='EXTERNAL',graph_ok=True,source_ok=True,failure='',speed=0.,ever_enabled=True)
    state.stop=lambda:stops.append(True);state.fail=failures.append
    state.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=10*10**9))
    state.status=SimpleNamespace(publish=lambda msg:None)
    monkeypatch.setattr(module.time,'monotonic',lambda:10.)
    module.OperatorGuard.tick(state)
    assert stops==[True] and not failures


def test_only_reverse_reset_can_use_remaining_distance():
    module.ResetConfig(direction=-1,distance_m=1.5).validate()
    for config in [module.ResetConfig(direction=1,distance_m=1.5),module.ResetConfig(direction=-1,distance_m=3.),module.MotionConfig(distance_m=1.5)]:
        with pytest.raises(ValueError):config.validate()
