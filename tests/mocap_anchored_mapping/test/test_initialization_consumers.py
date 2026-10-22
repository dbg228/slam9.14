"""Frozen initial-only localization releases acquisition without losing audits."""
from types import SimpleNamespace

from geometry_msgs.msg import Twist
from mocap_anchored_mapping import online
from mocap_anchored_mapping.online import NativeBridge


def bridge(initial_only=True,mode='localization'):
    destroyed=[]
    node=SimpleNamespace(mode=mode,initialization_subscriptions=['mocap','validity','command'],
        initialization_timers=['pair_expiry','capture_tick'],
        localization_metadata=lambda:{'initialization_only':initial_only},
        destroy_subscription=lambda value:destroyed.append(('subscription',value)),
        destroy_timer=lambda value:destroyed.append(('timer',value)))
    return node,destroyed


def test_initial_only_consumers_are_released_once():
    node,destroyed=bridge()
    NativeBridge.release_initialization_inputs(node)
    assert destroyed==[('subscription','mocap'),('subscription','validity'),('subscription','command'),
        ('timer','pair_expiry'),('timer','capture_tick')]
    NativeBridge.release_initialization_inputs(node)
    assert len(destroyed)==5


def test_checkpoints_retain_capture_inputs_for_later_corrections():
    node,destroyed=bridge(initial_only=False)
    NativeBridge.release_initialization_inputs(node)
    assert destroyed==[]
    assert node.initialization_subscriptions==['mocap','validity','command']


def test_command_journal_sampling_keeps_changes_and_motion_guard(monkeypatch):
    journal=[];invalid=[]
    times=iter([1.,1.001,1.002,1.003,1.06])
    monkeypatch.setattr(online.time,'monotonic',lambda:next(times))
    node=SimpleNamespace(command_log_at=0.,command_log_value=None,
        cfg={'initialization':{'maximum_command_abs':1e-5}},
        log=lambda kind,msg:journal.append((kind,msg.linear.x)),invalid=invalid.append)
    zero=Twist();moving=Twist();moving.linear.x=.1
    for msg in [zero,zero,moving,zero,zero]:NativeBridge.command(node,msg)
    assert [value for kind,value in journal]==[0.,.1,0.,0.]
    assert invalid==['nonzero_or_nonfinite_command']
    assert node.command_at==1.06
    assert not node.last_command.any()
