from collections import deque
from types import SimpleNamespace
import json
import math
import time
import pytest

from localization_ab_test.core import compose,inverse,planar


def pose(stamp,x,y=0.,a=0.):
    return dict(stamp_ns=stamp,valid=True,x=x,y=y,z=0.,qx=0.,qy=0.,qz=math.sin(a/2),qw=math.cos(a/2))


def test_c_correction_matches_known_transform_and_does_not_use_future_measurement():
    from localization_ab_test.sources import PoseSources
    transform=(1.,-2.,.3)
    ref=compose(transform,(-.0285,0.,0.))
    pending=deque([pose(1_190_000_000,ref[0],ref[1],ref[2]),pose(1_210_000_000,99.)])
    logs=[]
    fake=SimpleNamespace(history=deque([pose(1_100_000_000,-.015),pose(1_200_000_000,-.03)]),
                         pending=pending,args=SimpleNamespace(mocap_offset=0.,timing_verified=False),
                         correction=None,correction_time=None,diagnostic=SimpleNamespace(publish=logs.append))
    PoseSources.update_correction(fake)
    assert fake.correction==pytest.approx(transform)
    assert len(fake.pending)==1 and fake.pending[0]['stamp_ns']==1_210_000_000
    assert len(logs)==1
    detail=json.loads(logs[0].data)
    assert detail['lio_target_stamp_ns']==1_190_000_000
    assert detail['feedback_scope']=='C output only; FAST-LIO and NDT states unchanged'
    assert compose(fake.correction,(-.03,0.,0.))==pytest.approx(compose(transform,(-.03,0.,0.)))


def test_stale_c_correction_is_not_published():
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage
    from localization_ab_test.sources import PoseSources
    tf=TransformStamped();tf.header.frame_id='odom';tf.child_frame_id='base_link'
    tf.header.stamp.sec=2;tf.transform.rotation.w=1.
    published=[]
    fake=SimpleNamespace(last_lio_stamp=0,history=deque(),pubs={'C':None},
                         correction=(1.,2.,0.),correction_time=time.monotonic()-1.,
                         update_correction=lambda:None,publish=lambda *args:published.append(args))
    PoseSources.tf(fake,TFMessage(transforms=[tf]))
    assert not published
    fake.correction_time=time.monotonic();tf.header.stamp.nanosec=1
    PoseSources.tf(fake,TFMessage(transforms=[tf]))
    assert len(published)==1 and published[0][0]=='C' and published[0][-1]=='mocap_world'


def test_reverse_first_series_plan():
    from localization_ab_test.series import make_plan
    assert make_plan('reverse')==('reverse','forward','reverse')
    assert make_plan('forward')==('forward','reverse','forward')
    with pytest.raises(ValueError):make_plan('turn')
