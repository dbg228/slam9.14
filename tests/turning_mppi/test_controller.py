"""Small software regressions; these do not certify hardware or continuous safety.

Run: python3 -m pytest -q tests/turning_mppi/test_controller.py
"""
from dataclasses import replace
import math

import numpy as np
import pytest

from .controller import Config, GuidedMPPI, Path, clearance, smooth_step


def straight_path():
    x = np.linspace(0., 3., 31)
    return Path(np.c_[x, np.zeros_like(x), np.zeros_like(x)])


def small_config(**changes):
    values = {'samples': 8, 'steps': 12}
    values.update(changes)
    return replace(Config(), **values)


def test_projection_preserves_left_sign_and_finite_endpoints():
    path = straight_path()
    assert path.project(.45, .2) == pytest.approx((.45, .2, 0.))
    assert path.project(.45, -.2) == pytest.approx((.45, -.2, 0.))
    # The endpoint is finite. Lateral error alone omits longitudinal overshoot.
    s, lateral, _ = path.project(3.2, 0.)
    assert s == pytest.approx(path.length)
    assert lateral == pytest.approx(0.)
    foot = np.array(path.at(s)[:2])
    assert np.linalg.norm(np.array([3.2, 0.]) - foot) == pytest.approx(.2)
    assert path.project(-.3, .1)[0] == pytest.approx(0.)
    vertical = Path([[0, 0, math.pi / 2], [0, 1, math.pi / 2], [0, 2, math.pi / 2]])
    assert vertical.project(-.2, .5)[1] == pytest.approx(.2)


@pytest.mark.parametrize('points', [
    [[0, 0, 0], [1, 0, 0]],
    [[0, 0, 0], [0, 0, 0], [1, 0, 0]],
    [[0, 0, 0], [1, np.nan, 0], [2, 0, 0]],
    [[0, 0, 0], [1, 0, np.inf], [2, 0, 0]],
])
def test_path_rejects_unusable_geometry(points):
    with pytest.raises(ValueError):
        Path(points)


def test_clearance_handles_side_corner_penetration_and_rotation():
    cfg = Config()
    assert clearance(0., 0., 0., [(cfg.half_length + .13, 0., .10)], cfg) == pytest.approx(.03)
    assert clearance(0., 0., math.pi / 2,
                     [(cfg.half_width + .13, 0., .10)], cfg) == pytest.approx(.03)
    # A 3-4-5 corner triangle distinguishes rectangle distance from a circle proxy.
    corner = [(cfg.half_length + .3, cfg.half_width + .4, .1)]
    assert clearance(0., 0., 0., corner, cfg) == pytest.approx(.4)
    assert clearance(0., 0., 0., [(0., 0., .1)], cfg) == pytest.approx(-cfg.half_width - .1)
    angle = .71
    rotation = np.array([[math.cos(angle), -math.sin(angle)],
                         [math.sin(angle), math.cos(angle)]])
    translation = np.array([1.3, -.8])
    rotated_center = rotation @ np.array(corner[0][:2]) + translation
    assert clearance(*translation, angle, [(*rotated_center, .1)], cfg) == pytest.approx(.4)


def test_smoother_respects_rates_bounds_and_stops_without_overshoot():
    cfg = Config()
    current = np.zeros(2)
    for target in ([20., -20.], [0., 0.], [20., 20.], [0., 0.]):
        for _ in range(35):
            following = smooth_step(current, target, cfg)
            assert np.all(abs(following - current) <= np.array([cfg.accel_v, cfg.accel_w]) * cfg.dt + 1e-12)
            assert 0. <= following[0] <= cfg.vmax
            assert -cfg.wmax <= following[1] <= cfg.wmax
            current = following
    assert current == pytest.approx([0., 0.])


@pytest.mark.parametrize('component', range(5))
@pytest.mark.parametrize('bad', [np.nan, np.inf, -np.inf])
def test_nonfinite_state_is_rejected_before_optimization(component, bad):
    controller = GuidedMPPI(straight_path(), small_config())
    state = np.zeros(5)
    state[component] = bad
    with pytest.raises(ValueError):
        controller.command(state)


@pytest.mark.parametrize('argument,value', [
    ('state', [0., 0., 0.]),
    ('smooth', [np.nan, 0.]),
    ('smooth', [0., np.inf]),
    ('smooth', [0., 0., 0.]),
    ('pending', [(0., np.nan)]),
    ('pending', [(0., 0., 0.)]),
    ('obstacles', [(np.inf, 0., .1)]),
    ('obstacles', [(0., 0., np.nan)]),
    ('obstacles', [(0., 0., -.1)]),
    ('obstacles', [(0., 0.)]),
])
def test_invalid_external_inputs_are_rejected(argument, value):
    controller = GuidedMPPI(straight_path(), small_config())
    inputs = {'state': np.zeros(5), argument: value}
    with pytest.raises(ValueError):
        controller.command(**inputs)


@pytest.mark.parametrize('field,value', [
    ('steps', 0), ('samples', 4), ('dt', 0.), ('tau_v', 0.),
    ('tau_w', -.1), ('temperature', 0.), ('speed', 0.),
    ('vmax', 0.), ('wmax', 0.),
    ('dt', np.nan), ('tau_v', np.inf), ('temperature', np.nan),
])
def test_invalid_prediction_configuration_is_rejected(field, value):
    with pytest.raises(ValueError):
        GuidedMPPI(straight_path(), small_config(**{field: value}))


@pytest.mark.parametrize('mode', ['full', 'ideal', 'tracker'])
def test_returned_commands_remain_finite_and_bounded_across_repeated_calls(mode):
    cfg = small_config()
    controller = GuidedMPPI(straight_path(), cfg, seed=8, mode=mode)
    for state in ([0., .1, .2, 0., 0.], [.02, -.1, -.2, .1, -.1], [.04, 0., 0., .2, .1]):
        command = controller.command(state, smooth=(.1, .05), pending=[(.1, .05), (.1, .05)])
        assert command.shape == (2,)
        assert np.isfinite(command).all()
        assert 0. <= command[0] <= cfg.vmax
        assert abs(command[1]) <= cfg.wmax


def record_rollouts(controller, monkeypatch):
    records = []
    original = controller.rollout

    def observe(*args, **kwargs):
        result = original(*args, **kwargs)
        records.append((kwargs.get('guide_only', False), result))
        return result

    monkeypatch.setattr(controller, 'rollout', observe)
    return records


def test_blocked_rollouts_report_failure_and_validate_exact_zero_brake(monkeypatch):
    controller = GuidedMPPI(straight_path(), small_config(), seed=2)
    records = record_rollouts(controller, monkeypatch)
    command = controller.command([0., 0., .4, .15, .2], smooth=(.15, .2),
                                 pending=[(.15, .2), (.15, .2)], obstacles=[(0., 0., .1)])
    assert command == pytest.approx([0., 0.])
    brakes = [result for is_brake, result in records if is_brake]
    assert len(brakes) == 1
    _, safe, commands, distance = brakes[0]
    assert np.all(commands == 0.)  # Includes yaw: no hidden tracking turn while braking.
    assert not safe[0]
    assert controller.last_diagnostics['status'] == 'no_feasible_rollout'
    assert controller.last_diagnostics['feasible'] is False
    assert controller.last_diagnostics['min_predicted_clearance'] == pytest.approx(distance[0])


def test_tracker_can_stop_for_obstacle_without_claiming_route_completion(monkeypatch):
    cfg = small_config(steps=40)
    controller = GuidedMPPI(straight_path(), cfg, mode='tracker')
    records = record_rollouts(controller, monkeypatch)
    command = controller.command([0., .1, .2, 0., 0.], obstacles=[(.65, .1, .05)])
    assert not records[0][1][1][0]  # Forward tracking violates the footprint margin.
    brake = next(result for is_brake, result in records if is_brake)
    assert brake[1][0]
    assert np.all(brake[2] == 0.)
    assert command == pytest.approx([0., 0.])
    assert controller.last_diagnostics['status'] == 'brake'
    assert controller.last_diagnostics['feasible'] is True


def test_accepted_weighted_candidate_has_its_own_validated_rollout(monkeypatch):
    cfg = small_config(noise_v=0., noise_w=0.)
    controller = GuidedMPPI(straight_path(), cfg, mode='full')
    records = record_rollouts(controller, monkeypatch)
    command = controller.command(np.zeros(5))
    assert controller.last_diagnostics['status'] == 'mean'
    is_brake, (_, feasible, commands, distance) = records[-1]
    assert not is_brake and feasible.shape == (1,) and feasible[0]
    assert command == pytest.approx(commands[0, 0])
    assert controller.last_diagnostics['min_predicted_clearance'] == distance[0]


def test_unsafe_weighted_mean_falls_back_to_valid_candidate(monkeypatch):
    """A nonconvex feasible-set oracle models two safe sides of an obstacle.

    This tests selection safety separately from the geometric distance tests:
    a mean between feasible left/right choices must never inherit their status.
    """
    cfg = small_config(noise_v=0., noise_w=0.)
    controller = GuidedMPPI(straight_path(), cfg)
    evaluated = []

    def two_corridors(state, smooth, pending, residuals, obstacles, guide_only=False):
        count = len(residuals)
        turn = residuals[:, 0, 1]
        safe = abs(turn) > .1
        commands = np.zeros((count, cfg.steps, 2))
        commands[:, :, 0] = .1
        commands[:, :, 1] = turn[:, None]
        if guide_only:
            commands[:] = 0.
            safe[:] = True
        evaluated.append((guide_only, safe.copy()))
        return np.zeros(count), safe, commands, np.where(safe, .2, -.1)

    monkeypatch.setattr(controller, 'rollout', two_corridors)
    command = controller.command(np.zeros(5))
    assert evaluated[0][1].sum() == 2
    assert not evaluated[-1][0] and not evaluated[-1][1][0]
    assert controller.last_diagnostics['status'] == 'sample'
    assert abs(command[1]) > .1
    assert controller.last_diagnostics['min_predicted_clearance'] > 0.
