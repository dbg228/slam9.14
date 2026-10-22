"""Regression: a stale stream must emit a diagnostic without killing the receiver."""
from types import SimpleNamespace
import pytest


def test_ros_logger_accepts_stale_message_and_reports_only_once():
    pytest.importorskip('rclpy')
    from mocap_pose_normalizer import MocapPoseNormalizer
    messages=[];statuses=[]
    # ROS 2 logger accepts one message argument, unlike printf-style logging.
    logger=SimpleNamespace(error=lambda message:messages.append(message))
    fake=SimpleNamespace(last_receive_clock_ns=1_000_000_000,stale_after_seconds=.5,
                         stale_reported=False,output_frame='mocap_world',
                         get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=2_000_000_000)),
                         get_logger=lambda:logger,publish_status=lambda *args:statuses.append(args))
    MocapPoseNormalizer.check_stale(fake)
    MocapPoseNormalizer.check_stale(fake)
    assert fake.stale_reported is True
    assert len(messages)==len(statuses)==1
    assert statuses[0][1] is False
    assert '1.000' in messages[0]
