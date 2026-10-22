"""Private fresh-state transport must not queue reliable retransmissions."""
from types import SimpleNamespace
import pytest


def test_latest_transport_preserves_only_latest_and_is_best_effort():
    pytest.importorskip('rclpy')
    from rclpy.qos import ReliabilityPolicy, HistoryPolicy
    from localization_ab_test.qos import latest_value_qos
    profile=latest_value_qos()
    assert profile.depth==1
    assert profile.history==HistoryPolicy.KEEP_LAST
    assert profile.reliability==ReliabilityPolicy.BEST_EFFORT


def test_guard_failure_keeps_first_cause_and_always_stops():
    pytest.importorskip('rclpy')
    from localization_ab_test.nodes import VelocityGuard
    errors=[];stops=[]
    fake=SimpleNamespace(failure='',speed=.15,
                         get_logger=lambda:SimpleNamespace(error=errors.append),
                         stop=lambda:stops.append(True))
    VelocityGuard.fail(fake,'request_recorder_or_mocap_stale')
    VelocityGuard.fail(fake,'recorder_not_ready')
    assert fake.failure=='request_recorder_or_mocap_stale'
    assert fake.speed==0. and len(stops)==2 and len(errors)==1
