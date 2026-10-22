#!/usr/bin/env python3
"""Passive curvature-conditioned diagnostic of the original FAST-LIO + NDT run.

This reads recorded map-estimated poses. It neither runs a controller nor supplies
independent ground truth. Geometry is never aligned, fitted, or moved. Curvature
is a spatial finite-window descriptor of the supplied polyline, not a claim that
its piecewise-linear vertices have a finite differential curvature.

Example (from repository root):
  python3 tests/turning_mppi/history.py --path \
    robot_data/experiments/full_map_loop/20261018_relocated_01/group1_ndt/trial_01/frozen_inspection_path.json \
    --samples robot_data/experiments/full_map_loop/20261018_relocated_01/group1_ndt/trial_01/tracking_report_final/tracking_samples.csv \
    --output tests/turning_mppi/results/history
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import shlex
import sys

import numpy as np


def wrap(value):
    return (np.asarray(value) + np.pi) % (2 * np.pi) - np.pi


def stats(values):
    values = np.asarray(values, dtype=float)
    if not len(values):
        return {"n": 0, "rmse": None, "p95_abs": None, "max_abs": None, "mean": None}
    return {"n": len(values), "rmse": float(np.sqrt(np.mean(values ** 2))),
            "p95_abs": float(np.percentile(np.abs(values), 95)),
            "max_abs": float(np.max(np.abs(values))), "mean": float(np.mean(values))}


def load_path(path):
    document = json.loads(Path(path).read_text())
    if document.get("frame") not in (None, "map"):
        raise ValueError("Historical samples require a reference in the original map frame")
    if "points_xyz_m" in document:
        xy = np.asarray(document["points_xyz_m"], dtype=float)[:, :2]
        kind = "frozen inspection purple path; differs from the executed connector/reference"
    elif "reference" in document:
        xy = np.asarray([[row["x_m"], row["y_m"]] for row in document["reference"]], dtype=float)
        kind = "recorded executed reference path"
    else:
        raise ValueError("Expected points_xyz_m or reference in path JSON")
    if xy.ndim != 2 or xy.shape[1] != 2 or len(xy) < 2 or not np.isfinite(xy).all():
        raise ValueError("Malformed or nonfinite reference geometry")
    delta = np.diff(xy, axis=0)
    length = np.linalg.norm(delta, axis=1)
    usable = length > 1e-9
    if not usable.any():
        raise ValueError("Reference has no nonzero segments")
    original_s = np.r_[0., np.cumsum(length)]
    return {"xy": xy, "a": xy[:-1][usable], "d": delta[usable],
            "length": length[usable], "s": original_s[:-1][usable],
            "indices": np.flatnonzero(usable), "total_length": float(length.sum()),
            "heading": np.arctan2(delta[usable, 1], delta[usable, 0]),
            "duplicate_segments": int((~usable).sum()), "kind": kind}


def read_samples(path, maximum_gap_s):
    valid = []
    invalid = []
    last_stamp = None
    with Path(path).open(newline="") as stream:
        reader = csv.DictReader(stream)
        required = {"stamp_ns", "x_m", "y_m", "yaw_rad"}
        if not required.issubset(set(reader.fieldnames or [])):
            raise ValueError("CSV is missing fields: " + ", ".join(sorted(required - set(reader.fieldnames or []))))
        for line, source in enumerate(reader, 2):
            try:
                stamp = int(source["stamp_ns"])
                pose = [float(source[name]) for name in ("x_m", "y_m", "yaw_rad")]
                if not np.isfinite(pose).all():
                    raise ValueError("nonfinite pose")
                if last_stamp is not None and stamp <= last_stamp:
                    raise ValueError("nonincreasing sample timestamp")
                valid.append({"source_csv_line": line, "stamp_ns": stamp,
                              "x_m": pose[0], "y_m": pose[1], "yaw_rad": pose[2]})
                last_stamp = stamp
            except (TypeError, ValueError, OverflowError) as error:
                invalid.append({"source_csv_line": line, "reason": str(error)})
    gaps = [{"from_ns": a["stamp_ns"], "to_ns": b["stamp_ns"],
             "seconds": (b["stamp_ns"] - a["stamp_ns"]) * 1e-9}
            for a, b in zip(valid, valid[1:])
            if (b["stamp_ns"] - a["stamp_ns"]) * 1e-9 > maximum_gap_s]
    return valid, invalid, gaps


def tangent_at(path, position):
    position = np.clip(position, 0., path["total_length"])
    index = np.searchsorted(path["s"], position, side="right") - 1
    return path["heading"][np.clip(index, 0, len(path["heading"]) - 1)]


def curvature_at(path, position, window_m):
    low = np.maximum(np.asarray(position) - window_m / 2, 0.)
    high = np.minimum(np.asarray(position) + window_m / 2, path["total_length"])
    return wrap(tangent_at(path, high) - tangent_at(path, low)) / np.maximum(high - low, 1e-9)


def project_samples(path, samples, window_m, derivative_window_m, bend_threshold, phase_threshold):
    rows = []
    if not samples:
        return rows
    first_stamp = samples[0]["stamp_ns"]
    for source in samples:
        p = np.array([source["x_m"], source["y_m"]])
        fraction = np.clip(np.sum((p - path["a"]) * path["d"], axis=1) / path["length"] ** 2, 0., 1.)
        feet = path["a"] + fraction[:, None] * path["d"]
        distance = np.linalg.norm(p - feet, axis=1)
        index = int(np.argmin(distance))
        error = p - feet[index]
        s = float(path["s"][index] + fraction[index] * path["length"][index])
        curvature = float(curvature_at(path, s, window_m))
        low = max(s - derivative_window_m / 2, 0.)
        high = min(s + derivative_window_m / 2, path["total_length"])
        derivative = float((abs(curvature_at(path, high, window_m)) - abs(curvature_at(path, low, window_m))) / max(high - low, 1e-9))
        bend = abs(curvature) >= bend_threshold
        phase = "line"
        if bend:
            phase = "entry" if derivative > phase_threshold else "exit" if derivative < -phase_threshold else "bend_interior"
        # A nearby, topologically distant segment can make nearest projection
        # ambiguous near crossings/start-end proximity. Do not silently filter it.
        candidate_s = path["s"] + fraction * path["length"]
        ambiguous = bool(np.any((np.abs(candidate_s - s) > 1.) & (distance <= distance[index] + .02)))
        rows.append({**source, "time_s": (source["stamp_ns"] - first_stamp) * 1e-9,
                     "segment_index": int(path["indices"][index]), "reference_s_m": s,
                     "projection_x_m": float(feet[index, 0]), "projection_y_m": float(feet[index, 1]),
                     "path_distance_m": float(distance[index]),
                     "lateral_m": float((path["d"][index, 0] * error[1] - path["d"][index, 1] * error[0]) / path["length"][index]),
                     "heading_error_deg": float(np.degrees(wrap(source["yaw_rad"] - path["heading"][index]))),
                     "curvature_proxy_per_m": curvature, "abs_curvature_gradient_per_m2": derivative,
                     "curvature_bin": "bend" if bend else "line", "phase": phase,
                     "projection_topology_ambiguous": ambiguous})
    return rows


def summarize(rows):
    return {"samples": len(rows), "path_distance_m": stats([r["path_distance_m"] for r in rows]),
            "lateral_m": stats([r["lateral_m"] for r in rows]),
            "heading_error_deg": stats([r["heading_error_deg"] for r in rows]),
            "projection_topology_ambiguous_samples": sum(r["projection_topology_ambiguous"] for r in rows)}


def plot(document, path, output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    rows = document["rows"]
    fig, axes = plt.subplots(2, 2, figsize=(13, 9), constrained_layout=True)
    ax = axes[0, 0]
    ax.plot(path["xy"][:, 0], path["xy"][:, 1], color="0.7", linewidth=2, label="Frozen reference")
    for label, color in [("line", "tab:blue"), ("bend", "tab:orange")]:
        part = [r for r in rows if r["curvature_bin"] == label]
        ax.scatter([r["x_m"] for r in part], [r["y_m"] for r in part], s=4, color=color, label=label)
    ax.set(xlabel="Original map X [m]", ylabel="Original map Y [m]", title="Recorded map-estimated poses")
    ax.set_aspect("equal"); ax.legend()
    ax = axes[0, 1]
    labels = ["line", "bend", "entry", "bend_interior", "exit"]
    summaries = [document["curvature_bins"].get(label, document["bend_phases"].get(label)) for label in labels]
    pos = np.arange(len(labels))
    rmse = [1000 * d["path_distance_m"]["rmse"] if d["samples"] else np.nan for d in summaries]
    p95 = [1000 * d["path_distance_m"]["p95_abs"] if d["samples"] else np.nan for d in summaries]
    ax.bar(pos - .19, rmse, .38, label="RMSE"); ax.bar(pos + .19, p95, .38, label="P95")
    ax.set_xticks(pos, [f"{label}\nn={d['samples']}" for label, d in zip(labels, summaries)], fontsize=8)
    ax.set(ylabel="Estimated path deviation [mm]", title="Sample weights; entry/exit phase assignments exploratory"); ax.legend()
    ax = axes[1, 0]
    for label, color in [("line", "tab:blue"), ("bend", "tab:orange")]:
        part = [r for r in rows if r["curvature_bin"] == label]
        ax.scatter([r["reference_s_m"] for r in part], [1000 * r["path_distance_m"] for r in part], s=4, color=color, label=label)
    ax.set(xlabel="Projected reference distance [m]", ylabel="Estimated path deviation [mm]", title="All valid recorded samples; no trajectory alignment")
    ax = axes[1, 1]
    ss = np.linspace(0., path["total_length"], max(2, math.ceil(path["total_length"] / .01)))
    kk = curvature_at(path, ss, document["classification"]["curvature_window_m"])
    ax.plot(ss, np.abs(kk), color="tab:purple")
    ax.axhline(document["classification"]["bend_threshold_per_m"], color="gray", linestyle="--", label="Bend threshold")
    ax.set(xlabel="Reference distance [m]", ylabel="|Finite-window curvature| [1/m]", title="Geometry descriptor; reference is not modified"); ax.legend()
    for ax in axes.flat:
        ax.grid(alpha=.2)
    fig.suptitle("Original FAST-LIO + NDT history: estimated tracking deviation, not independent ground truth")
    fig.savefig(output / "history.png", dpi=170)
    plt.close(fig)


def self_test():
    import tempfile
    with tempfile.TemporaryDirectory() as directory:
        directory = Path(directory)
        line = directory / "line.json"
        line.write_text(json.dumps({"frame": "map", "points_xyz_m": [[0, 0, 0], [1, 0, 0], [1, 0, 0], [2, 0, 0]]}))
        path = load_path(line)
        samples = [{"source_csv_line": 2, "stamp_ns": 100, "x_m": .5, "y_m": .2, "yaw_rad": 0.},
                   {"source_csv_line": 3, "stamp_ns": 200, "x_m": 2.2, "y_m": 0., "yaw_rad": 0.}]
        rows = project_samples(path, samples, .6, .2, .15, .25)
        assert path["duplicate_segments"] == 1
        assert abs(rows[0]["lateral_m"] - .2) < 1e-12
        assert abs(rows[1]["path_distance_m"] - .2) < 1e-12
        assert rows[1]["reference_s_m"] == 2.
        assert all(row["curvature_bin"] == "line" for row in rows)
        circle = directory / "circle.json"
        tt = np.linspace(0., np.pi / 2, 1001)
        circle.write_text(json.dumps({"points_xyz_m": np.c_[np.cos(tt), np.sin(tt), np.zeros_like(tt)].tolist()}))
        path = load_path(circle)
        assert abs(float(curvature_at(path, .7, .6)) - 1.) < .01
        source = directory / "samples.csv"
        source.write_text("stamp_ns,x_m,y_m,yaw_rad\n100,0,0,0\n200,nan,0,0\n99,0,0,0\n1000000100,1,0,0\n")
        valid, invalid, gaps = read_samples(source, .5)
        assert len(valid) == 2 and len(invalid) == 2 and len(gaps) == 1
        assert stats([])["rmse"] is None
    print("history self-test passed: projection/sign/endpoints, duplicate geometry, curvature, invalid rows and missing intervals")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--path", type=Path)
    parser.add_argument("--samples", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--curvature-window-m", type=float, default=.6)
    parser.add_argument("--derivative-window-m", type=float, default=.2)
    parser.add_argument("--bend-threshold-per-m", type=float, default=.15)
    parser.add_argument("--phase-threshold-per-m2", type=float, default=.25)
    parser.add_argument("--maximum-gap-s", type=float, default=.5)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    if not all((args.path, args.samples, args.output)):
        parser.error("--path, --samples and --output are required")
    if any(not math.isfinite(v) or v <= 0. for v in (args.curvature_window_m, args.derivative_window_m, args.bend_threshold_per_m, args.phase_threshold_per_m2, args.maximum_gap_s)):
        parser.error("Classification lengths/thresholds must be finite and positive")
    path = load_path(args.path)
    samples, invalid, gaps = read_samples(args.samples, args.maximum_gap_s)
    if not samples:
        raise ValueError("No valid historical samples")
    rows = project_samples(path, samples, args.curvature_window_m, args.derivative_window_m, args.bend_threshold_per_m, args.phase_threshold_per_m2)
    source = lambda p: {"path": str(p.resolve()), "sha256": hashlib.sha256(p.read_bytes()).hexdigest()}
    document = {"source": "recorded original FAST-LIO + NDT map-estimated tracking history",
                "independent_ground_truth": False, "controller_comparison": False,
                "interpretation": "Estimated deviation to the supplied reference, with localization error mixed in. This diagnoses history; it does not establish real turn accuracy or an improved controller's effect.",
                "reference_kind": path["kind"], "frame": "map", "alignment_applied": False, "transform_fitted": False,
                "sources": {"path": source(args.path), "samples": source(args.samples)},
                "reproduce_command": shlex.join([sys.executable, *sys.argv]),
                "classification": {"curvature_window_m": args.curvature_window_m,
                    "derivative_window_m": args.derivative_window_m, "bend_threshold_per_m": args.bend_threshold_per_m,
                    "phase_threshold_per_m2": args.phase_threshold_per_m2,
                    "definition": "kappa = wrapped change of polyline segment tangent across a fixed spatial window / covered length. Entry/exit = positive/negative gradient of abs(kappa) over the derivative window, within bend samples. Edge windows are shortened; the reference is not closed or smoothed.",
                    "phase_reliability": "Entry/exit assignments are exploratory: discrete polyline tangents can make d(abs(kappa))/ds oscillate. They are not a validated physical entry/exit detector and must not support a causal claim or controller-gain selection.",
                    "projection": "Exact nearest finite segment; ties choose the earliest reference segment. Left-positive lateral. Heading error uses that segment tangent.",
                    "statistics": "Valid recorded sample weights, not distance weights or independent repeated trials. Classification thresholds are descriptive choices, not identified actuator parameters."},
                "reference_length_m": path["total_length"], "reference_duplicate_segments": path["duplicate_segments"],
                "input_rows": len(samples) + len(invalid), "valid_rows": len(samples), "invalid_rows": len(invalid),
                "invalid_details": invalid, "observed_timestamp_gaps": gaps,
                "missing_sample_count": None, "missing_note": "The nominal polling rate is not a guaranteed acquisition schedule; missing sample count is unknown. Gaps are retained and never interpolated.",
                "overall": summarize(rows),
                "curvature_bins": {label: summarize([r for r in rows if r["curvature_bin"] == label]) for label in ("line", "bend")},
                "bend_phases": {label: summarize([r for r in rows if r["phase"] == label]) for label in ("entry", "bend_interior", "exit")},
                "rows": rows}
    # Fixed sensitivity sweep avoids presenting one arbitrary bend width as
    # conclusive. The samples and geometry stay identical in every descriptor.
    document["curvature_window_sensitivity"] = {}
    for window in (.4, .6, .8):
        alternate = project_samples(path, samples, window, args.derivative_window_m, args.bend_threshold_per_m, args.phase_threshold_per_m2)
        common_bends = [(primary, other) for primary, other in zip(rows, alternate)
                        if primary["curvature_bin"] == other["curvature_bin"] == "bend"]
        document["curvature_window_sensitivity"][str(window)] = {
            "curvature_bins": {label: summarize([r for r in alternate if r["curvature_bin"] == label]) for label in ("line", "bend")},
            "bend_phases": {label: summarize([r for r in alternate if r["phase"] == label]) for label in ("entry", "bend_interior", "exit")},
            "common_bend_samples": len(common_bends),
            "phase_agreement_with_primary_among_common_bends": (
                sum(primary["phase"] == other["phase"] for primary, other in common_bends) / len(common_bends)
                if common_bends else None)}
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "history.json").write_text(json.dumps(document, indent=2, allow_nan=False) + "\n")
    plot(document, path, args.output)
    print(json.dumps({"output": str(args.output), "valid_rows": len(rows), "invalid_rows": len(invalid),
                      "gaps": len(gaps), "overall": document["overall"],
                      "curvature_bins": document["curvature_bins"], "bend_phases": document["bend_phases"]}, indent=2))


if __name__ == "__main__":
    main()
