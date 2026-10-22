import math
import numpy as np
import pytest

from localization_ab_test.core import MotionConfig,MotionGate,base_pose,compose,inverse
from localization_ab_test.evaluate import score,write_report


def p(t,x=0.,y=0.,a=0.):
    return dict(stamp_ns=1_000_000_000+round(t*1e9),valid=True,x=x,y=y,z=.18,
                qx=0.,qy=0.,qz=math.sin(a/2),qw=math.cos(a/2))


def settled(direction=1):
    gate=MotionGate(0.,MotionConfig(direction=direction))
    for i in range(501):
        t=i*.01;gate.ingest(p(t),t);gate.tick(t)
    assert gate.state=='MOVING'
    return gate


def test_fixed_speed_target_and_five_second_stop():
    gate=settled()
    for i in range(1,1341):
        t=5+i*.01;x=i*.0015
        gate.ingest(p(t,x=x),t)
        speed=gate.tick(t)
        if x<2:assert speed==.15
        else:
            assert speed==0.;break
    assert gate.reason=='target_reached'
    assert 2<=gate.distance<2.002
    stop=gate.stop_started
    assert gate.tick(stop+4.999)==0 and gate.state=='POST_STOP'
    assert gate.tick(stop+5.001)==0 and gate.state=='DONE'


def test_duplicate_pose_does_not_refresh_watchdog():
    gate=settled();last=gate.latest.copy()
    gate.ingest(last,5.19)
    assert gate.tick(5.21)==0
    assert gate.reason=='mocap_stale'


def test_jump_and_no_motion_abort_fail_closed():
    gate=settled();gate.ingest(p(5.01,x=.4),5.01)
    assert gate.reason=='mocap_pose_jump' and gate.tick(5.01)==0
    gate=settled()
    for i in range(1,302):
        t=5+i*.01;gate.ingest(p(t),t);gate.tick(t)
    assert gate.reason=='no_progress'


def test_distance_is_displacement_not_accumulated_jitter():
    gate=settled()
    for i in range(1,200):
        t=5+i*.01;gate.ingest(p(t,x=.001*math.sin(i)),t);gate.tick(t)
    assert gate.distance<.002 and gate.path_length>.10 and gate.state=='MOVING'


def test_mount_rotation_and_se2_correction():
    mount={'translation_m':[.2,0.,-.18],'quaternion_xyzw':[0.,0.,0.,1.]}
    result=base_pose(p(0,a=math.pi/2),mount)
    assert result['x']==pytest.approx(0.,abs=1e-12)
    assert result['y']==pytest.approx(.2)
    local=(.5,.2,3.13);global_base=(1.,2.,-3.13)
    corrected=compose(compose(global_base,inverse(local)),local)
    assert corrected==pytest.approx(global_base)


def test_protocol_cannot_accidentally_change_between_groups():
    for changes in [dict(speed_mps=.2),dict(distance_m=1.),dict(post_stop_s=2.)]:
        with pytest.raises(ValueError):MotionConfig(**changes).validate()


def synthetic():
    ref=[];estimate=[]
    yaw0=math.pi-.01
    transform=(.7,-.3,.4)
    for t in np.arange(0,25,.05):
        x=.15*min(max(0.,t-5),2/.15)
        ref.append(p(t,x=x,a=yaw0))
    for t in np.arange(0,25,.1):
        x=.15*min(max(0.,t-5),2/.15)
        # After the predeclared static alignment, inject +10 mm bias.
        bias=.01 if t>=5 else 0.
        v=compose(transform,(x+bias,0.,yaw0+(.02 if t>=5 else 0.)))
        estimate.append(p(t,x=v[0],y=v[1],a=v[2]))
    events=[dict(event='motion_start',ros_stamp_ns=p(5)['stamp_ns'],start_base_pose=p(5)),
            dict(event='stop_command',ros_stamp_ns=p(5+2/.15)['stamp_ns'],
                 reason='target_reached',displacement_m=2.)]
    return estimate,ref,events


def test_evaluation_known_bias_yaw_wrap_endpoint_and_c_label(tmp_path):
    est,ref,events=synthetic()
    rows,summary=score(est,ref,events,'A',0.,False)
    assert summary['all_motion_and_5s_stop']['xy_m']['rmse']==pytest.approx(.01,abs=1e-10)
    assert summary['all_motion_and_5s_stop']['yaw_deg']['rmse']==pytest.approx(math.degrees(.02))
    assert summary['endpoint']['xy_error_m']==pytest.approx(.01)
    assert summary['endpoint']['physical_displacement_m']==pytest.approx(2.)
    assert summary['coverage']==1.
    assert summary['dynamic_accuracy_status']=='provisional_time_offset_unverified'
    assert min(r['time_s'] for r in rows)>=0.
    _,c_summary=score(est,ref,events,'C',0.)
    assert c_summary['reference_independent'] is False
    write_report(tmp_path/'report',rows,summary)
    for name in ('frame_errors.csv','summary.json','report.md','trajectory_errors.png','errors_by_true_distance.svg'):
        assert (tmp_path/'report'/name).is_file()


def test_reference_dropout_remains_missing():
    est,ref,events=synthetic()
    ref=[r for r in ref if not 8.<(r['stamp_ns']-10**9)/1e9<9.]
    rows,summary=score(est,ref,events,'B',0.)
    assert summary['coverage']<1.
    assert any(not r['valid'] and r['e_xy_m'] is None for r in rows)
    assert any(r['missing_reason']=='mocap_interpolation_gap_too_large' for r in rows)


@pytest.mark.parametrize('direction',[-1,1])
def test_explicit_direction_and_same_stop_protections(direction):
    gate=settled(direction)
    for i in range(1,1341):
        t=5+i*.01;x=direction*i*.0015
        gate.ingest(p(t,x=x),t);speed=gate.tick(t)
        if abs(x)<2.:assert speed==direction*.15
        else:
            assert speed==0.;break
    assert gate.reason=='target_reached'
    assert gate.tick(gate.stop_started+5.001)==0. and gate.state=='DONE'
    gate=settled(direction)
    gate.ingest(p(5.01,x=-direction*.04),5.01)
    assert gate.tick(5.01)==0. and gate.reason=='wrong_motion_direction'


@pytest.mark.parametrize('direction',[0,2,-2,True,1.0,float('nan')])
def test_direction_validation(direction):
    with pytest.raises(ValueError):MotionConfig(direction=direction).validate()
