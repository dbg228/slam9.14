"""Software checks for selecting motion bounds without using localization errors."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace as S

import pytest

SPEC = importlib.util.spec_from_file_location('arrival_capture', Path(__file__).with_name('arrival_capture.py'))
arrival = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(arrival)


def command(x):
    return S(linear=S(x=x, y=0., z=0.), angular=S(x=0., y=0., z=0.))


def setup_trial(tmp_path, status=4):
    arrival.wc.save(tmp_path/'formal_departure_request.json', {'request_ros_ns': 10*10**9})
    arrival.wc.save(tmp_path/'route_monitor_latest.json', {'goal_status': status, 'ros_now_ns': 20*10**9})


def test_motion_bounds_exclude_initial_idle_and_keep_last_stop_after_pause(tmp_path, monkeypatch):
    setup_trial(tmp_path)
    stream = [(10, 0.), (11, .2), (12, 0.), (13, .1), (14, 0.), (15, .00005)]
    def messages(bag, topic, begin, end):
        assert topic == '/reference_velocity'
        assert (begin, end) == (10*10**9, 23*10**9)
        return iter((t*10**9, command(x)) for t, x in stream)
    monkeypatch.setattr(arrival, 'messages', messages)
    result = arrival.motion_interval(tmp_path, tmp_path/'bag')
    assert result['start_ns'] == 11*10**9
    assert result['last_nonzero_command_ns'] == 13*10**9
    assert result['zero_command_ns'] == 14*10**9
    assert result['end_ns'] == 15*10**9


@pytest.mark.parametrize('values', [[0., 0.], [.2, .1]])
def test_motion_bounds_require_both_motion_and_following_stop(tmp_path, monkeypatch, values):
    setup_trial(tmp_path)
    monkeypatch.setattr(arrival, 'messages', lambda *args: iter(
        ((11+i)*10**9, command(x)) for i, x in enumerate(values)))
    with pytest.raises(ValueError, match='bounded motion interval'):
        arrival.motion_interval(tmp_path, tmp_path/'bag')


def test_non_successful_navigation_cannot_be_presented_as_arrival(tmp_path):
    setup_trial(tmp_path, status=6)
    with pytest.raises(ValueError, match='navigation success'):
        arrival.motion_interval(tmp_path, tmp_path/'bag')
