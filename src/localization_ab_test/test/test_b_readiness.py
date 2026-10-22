from collections import deque
from types import SimpleNamespace
import time
import pytest


def test_b_identity_correction_is_not_published_before_ndt_ready():
    pytest.importorskip('rclpy')
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage
    from localization_ab_test.sources import PoseSources
    tf=TransformStamped();tf.header.frame_id='odom';tf.child_frame_id='base_link'
    tf.header.stamp.sec=123;tf.transform.rotation.w=1.
    published=[]
    fake=SimpleNamespace(last_lio_stamp=0,history=deque(),pubs={'B':None},
                         ndt_correction=(0.,0.,0.),ndt_ready=False,
                         publish=lambda *args:published.append(args))
    PoseSources.tf(fake,TFMessage(transforms=[tf]))
    assert not published
    fake.ndt_ready=True;tf.header.stamp.nanosec=1
    PoseSources.tf(fake,TFMessage(transforms=[tf]))
    assert len(published)==1 and published[0][0]=='B'


@pytest.mark.parametrize('direction',[-1,1])
def test_guard_relays_selected_sign_only(direction):
    pytest.importorskip('rclpy')
    from geometry_msgs.msg import TwistStamped
    from localization_ab_test.nodes import VelocityGuard
    errors=[];velocity=[];mode=[];states=[]
    clock=SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=10_000_000_000))
    now=time.monotonic()
    fake=SimpleNamespace(allowed_speed=direction*.15,latest=0.,last_stamp=0,speed=0.,
                         fail=errors.append,stop=lambda:None,get_clock=lambda:clock,
                         permit_time=now,mocap_time=now,armed=True,failure='',
                         velocity=SimpleNamespace(publish=velocity.append),
                         mode=SimpleNamespace(publish=mode.append),
                         status=SimpleNamespace(publish=states.append))
    request=TwistStamped();request.header.stamp.sec=10;request.header.frame_id='base_link'
    request.twist.linear.x=direction*.15
    VelocityGuard.request(fake,request);VelocityGuard.tick(fake)
    assert not errors and velocity[-1].linear.x==direction*.15
    request.twist.linear.x=-direction*.15
    VelocityGuard.request(fake,request)
    assert errors==['command_outside_fixed_protocol']


def test_series_never_continues_on_failed_or_unfinished_trial():
    from localization_ab_test.series import completed,PLAN
    assert PLAN==('forward','reverse','forward')
    good={'status':'completed','final_state':{'reason':'target_reached'}}
    assert completed(0,good)
    assert not completed(2,good)
    assert not completed(0,{'status':'aborted','final_state':{'reason':'mocap_stale'}})


def test_only_unstarted_preflight_block_allows_explicit_continuation(tmp_path):
    import json
    from localization_ab_test.series import continuation_trials
    first=tmp_path/'01_forward';last=tmp_path/'02_reverse'
    first.mkdir();last.mkdir();(first/'report').mkdir()
    (first/'manifest.json').write_text(json.dumps({'status':'completed','final_state':{'reason':'target_reached'}}))
    (first/'report/summary.json').write_text('{}')
    (last/'manifest.json').write_text(json.dumps({'status':'aborted','error':'Preflight blocked: test'}))
    (tmp_path/'series_manifest.json').write_text(json.dumps(dict(group='B',status='aborted',trials=[
        dict(direction='forward',directory=str(first)),dict(direction='reverse',directory=str(last))])))
    prefix=continuation_trials(tmp_path,'B')
    assert len(prefix)==1 and prefix[0]['carried_forward_from_previous_series']
    stored=json.loads((tmp_path/'series_manifest.json').read_text())
    stored['trials'][0]['directory']='/not_an_existing_old_root/01_forward'
    stored['trials'][1]['directory']='/not_an_existing_old_root/02_reverse'
    (tmp_path/'series_manifest.json').write_text(json.dumps(stored))
    assert continuation_trials(tmp_path,'B')[0]['directory']==str(first.resolve())
    (last/'bag').mkdir()
    with pytest.raises(ValueError):continuation_trials(tmp_path,'B')


@pytest.mark.parametrize('arrival',[4.,None])
def test_discovery_wait_is_passive_bounded_and_not_a_stale_data_override(monkeypatch,arrival):
    from localization_ab_test import run
    now=[0.]
    monkeypatch.setattr(run,'time',SimpleNamespace(monotonic=lambda:now[0]))
    fake=SimpleNamespace(local_receive=None)
    def wait(dt):
        now[0]+=dt
        if arrival is not None and now[0]>=arrival:fake.local_receive=now[0]
    fake.wait=wait
    elapsed=run.await_initial_local_tf(fake)
    if arrival is None:
        assert 10.<=elapsed<10.2 and fake.local_receive is None
    else:
        assert 4.<=elapsed<4.2 and now[0]-fake.local_receive<=.3
