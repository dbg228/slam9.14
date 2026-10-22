"""Validate the native adapter and seeded installed-library reproducibility."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from native_mppi import NativeMPPI


def commands(config, plugin, seed):
    x = np.linspace(0, 6, 301)
    path = np.c_[x, x * 0, x * 0]
    with NativeMPPI(config, plugin, seed=seed) as controller:
        try:
            controller.step(0, 0, 0, 0, 0)
        except RuntimeError as exc:
            assert "set_path" in str(exc)
        else:
            raise AssertionError("Missing plan must raise")
        controller.set_path(path)
        result = np.array([
            controller.step(i * .005, .005 * np.sin(i / 3), 0, .1, 0)
            for i in range(512)
        ])
        assert np.all(np.isfinite(result))
        vmax, wmax = (.2, .45) if plugin == "InspectionStrict" else (.3, .65)
        # Original MPPI's output filter can slightly overshoot internal limits;
        # do not silently clamp it. The downstream plant models the smoother.
        assert np.min(result[:, 0]) >= -.02
        assert np.max(result[:, 0]) <= vmax + .02
        assert np.max(np.abs(result[:, 1])) <= wmax + .02
        return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path,
                        default=Path(__file__).resolve().parents[3] / "robot_data/config/navigation.yaml")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = {"installed_package": "nav2_mppi_controller 1.1.20",
              "config_sha256": hashlib.sha256(args.config.read_bytes()).hexdigest(),
              "libraries": {}, "plugins": {}}
    for filename in ("libmppi_controller.so", "libmppi_critics.so"):
        report["libraries"][filename] = hashlib.sha256(
            (Path("/opt/ros/humble/lib") / filename).read_bytes()).hexdigest()
    for plugin in ("InspectionStrict", "FollowPath"):
        a = commands(args.config, plugin, 7)
        b = commands(args.config, plugin, 7)
        c = commands(args.config, plugin, 8)
        assert not np.array_equal(a[0], c[0]), "Different sampling seed had no initial effect"
        report["plugins"][plugin] = {
            "sequence_length": len(a),
            "observed_min_v": float(a[:, 0].min()),
            "observed_max_v": float(a[:, 0].max()),
            "observed_max_abs_w": float(np.abs(a[:, 1]).max()),
            "finite_and_sanity_envelope": True,
            "same_seed_exact_reproduction": bool(np.array_equal(a, b)),
            "same_seed_max_abs_difference": float(np.abs(a - b).max()),
            "different_seed_changes_initial_command": True,
            "seed_7_first_four_commands": a[:4].tolist()}
    result = json.dumps(report, indent=2) + "\n"
    if args.output:
        args.output.write_text(result)
    print(result, end="")


if __name__ == "__main__":
    main()
