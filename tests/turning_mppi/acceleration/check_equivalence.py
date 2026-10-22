#!/usr/bin/env python3
"""Check C++ rollout against preserved NumPy equations and measure wall time."""
from dataclasses import replace
import json
from pathlib import Path
import sys
import time
from types import MethodType

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from controller import Config, GuidedMPPI
from benchmark import make_path
from acceleration.fast_rollout import prepare, rollout


def run():
    rng = np.random.default_rng(8117)
    original = getattr(GuidedMPPI, "_rollout_python", GuidedMPPI.rollout)
    if original is rollout or original.__module__.startswith("acceleration"):
        raise RuntimeError("Equivalence check needs the preserved original NumPy method")
    backend = prepare()
    errors = {"cost": 0., "commands": 0., "clearance": 0.}
    checked = 0
    fixtures = []
    for kind in ("straight", "left", "right", "s_bend"):
        for mode in ("full", "ideal", "tracker"):
            for delay_steps in (0, 2, 5):
                path = make_path(kind)
                cfg = replace(Config(), samples=41, steps=37)
                ctl = GuidedMPPI(path, cfg, mode=mode)
                ctl.s = float(rng.uniform(0, path.length))
                rx, ry, tangent, _, _ = path.at(ctl.s)
                state = np.array([rx + rng.normal(0, .04), ry + rng.normal(0, .04),
                                  tangent + rng.normal(0, .05), rng.uniform(0, .2), rng.normal(0, .1)])
                smooth = rng.uniform([0., -.3], [.2, .3])
                pending = rng.uniform([0., -.3], [.2, .3], (delay_steps, 2))
                residuals = rng.normal(size=(cfg.samples, cfg.steps, 2)) * [.05, .12]
                obstacles = [] if delay_steps == 0 else [(rx + .7, ry + .1, .13), (rx - .5, ry + .8, .15)]
                for guide_only in (False, True):
                    inputs = (state, smooth, pending, residuals, obstacles)
                    expected = original(ctl, *inputs, guide_only=guide_only)
                    actual = rollout(ctl, *inputs, guide_only=guide_only)
                    for index, name in ((0, "cost"), (2, "commands"), (3, "clearance")):
                        finite = np.isfinite(expected[index]) & np.isfinite(actual[index])
                        if finite.any():
                            errors[name] = max(errors[name], float(np.max(np.abs(expected[index][finite] - actual[index][finite]))))
                        np.testing.assert_allclose(actual[index], expected[index], rtol=1e-11, atol=1e-9)
                    np.testing.assert_array_equal(actual[1], expected[1])
                    checked += 1
                fixtures.append((ctl, inputs))
    # Production batch size and prediction horizon, not the tiny test batch.
    cfg = Config()
    ctl = GuidedMPPI(make_path("s_bend"), cfg)
    ctl.s = .9
    x, y, yaw, _, _ = ctl.path.at(ctl.s)
    inputs = (np.array([x, y, yaw + .02, .18, .02]), np.array([.18, .02]),
              np.array([[.18, .02], [.18, .02]]), rng.normal(size=(1000, 50, 2)) * [.05, .12],
              [(1.8, .47, .13)])
    times = {}
    for name, function, repetitions in (("numpy", original, 5), ("cpp", rollout, 15)):
        function(ctl, *inputs)
        elapsed = []
        for _ in range(repetitions):
            start = time.perf_counter(); function(ctl, *inputs)
            elapsed.append((time.perf_counter() - start) * 1000)
        times[name] = {"median_ms": float(np.median(elapsed)), "p95_ms": float(np.percentile(elapsed, 95)), "calls": repetitions}
    # This additionally checks importance weights, accepted weighted means,
    # sample selection and shifted warm starts, not just trajectory costs.
    slow = GuidedMPPI(make_path("left"), cfg, seed=92)
    fast = GuidedMPPI(make_path("left"), cfg, seed=92)
    slow.rollout = MethodType(original, slow)
    fast.rollout = MethodType(rollout, fast)
    command_error = 0.
    for step in range(16):
        progress = .7 + .08 * step
        x, y, yaw, _, _ = slow.path.at(progress)
        state = np.array([x + .01, y - .015, yaw + .03, .19, .15])
        command_inputs = (state, [.19, .15], [[.19, .15], [.19, .15]], [(2., .8, .13)])
        a, b = slow.command(*command_inputs), fast.command(*command_inputs)
        command_error = max(command_error, float(np.max(np.abs(a - b))))
        np.testing.assert_allclose(a, b, rtol=1e-11, atol=1e-9)
        assert slow.last_diagnostics["status"] == fast.last_diagnostics["status"]
    result = {"passed": True, "random_fixture_cases": checked, "absolute_max_error": errors,
              "backend": backend, "numpy_version": np.__version__,
              "warm_start_command_comparison_calls": 16, "maximum_command_error": command_error,
              "timing_1000_by_50": times, "compiler": "g++ -O3 -std=c++17 -shared -fPIC -ffp-contract=off",
              "evidence": "Numerical equivalence and isolated runtime; no ROS or hardware"}
    (HERE / "equivalence.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    run()
