"""Auditable secondary analysis of the frozen, obstacle-free tracking trials.

No controller is re-run. Raw states are observed at t; the command on the same
row was issued at t-dt. Integrals use only observed adjacent sample intervals,
never an invented t=0 sample and never a bridge between disconnected phases.
The execution-chain reconstruction replays the SAME synthetic Plant: agreement
is a data-integrity check, not independent validation or system identification.
"""
from __future__ import annotations

import argparse
from collections import defaultdict
import csv
from dataclasses import fields
import hashlib
import itertools
import json
import math
from pathlib import Path as FilePath
import sys

import numpy as np

HERE = FilePath(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))
from controller import Config, Path
from benchmark import Plant

PHASES = ("all", "line", "turn", "entry", "apex", "exit")
META_FIELDS = ("trial_id", "dataset", "scenario", "condition", "method", "seed",
               "complete", "failure", "phase")
RAW_COLUMNS = ("t", "x", "y", "yaw", "v", "w", "cmd_v", "cmd_w", "s", "e",
               "e_yaw", "k", "compute_s", "clearance")


def sha256(path):
    return hashlib.sha256(FilePath(path).read_bytes()).hexdigest()


def finite_float(value):
    """Serialize an undefined statistic as an empty CSV cell / JSON null."""
    if isinstance(value, (np.bool_, bool)):
        return bool(value)
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (float, np.floating)):
        return float(value) if np.isfinite(value) else None
    return value


def write_csv(path, rows, fieldnames=None):
    path = FilePath(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if fieldnames is None:
        fieldnames = list(dict.fromkeys(k for row in rows for k in row))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, extrasaction="raise")
        writer.writeheader()
        for row in rows:
            writer.writerow({k: finite_float(v) for k, v in row.items()})


def moments(a):
    a = np.asarray(a, dtype=float)
    a = a[np.isfinite(a)]
    if not len(a):
        return {k: None for k in ("rmse", "mae", "signed_mean", "sd", "p95_abs", "max_abs")}
    return dict(rmse=float(np.sqrt(np.mean(a*a))), mae=float(np.mean(abs(a))),
                signed_mean=float(np.mean(a)), sd=float(np.std(a, ddof=1)) if len(a)>1 else None,
                p95_abs=float(np.percentile(abs(a), 95)), max_abs=float(np.max(abs(a))))


def good_intervals(t, mask, nominal_dt):
    """Only pairs of adjacent selected observations with no missing sample gap."""
    dt = np.diff(np.asarray(t, dtype=float))
    return np.asarray(mask[:-1]) & np.asarray(mask[1:]) & (dt>0) & (dt<=1.5*nominal_dt)


def integrate_observed(t, values, mask, nominal_dt):
    """Composite trapezoids; missing endpoints and phase boundaries are omitted."""
    t, values = np.asarray(t, dtype=float), np.asarray(values, dtype=float)
    valid = good_intervals(t, mask, nominal_dt)
    valid &= np.isfinite(values[:-1]) & np.isfinite(values[1:])
    if not valid.any():
        return None, 0., 0
    dt = np.diff(t)[valid]
    integral = np.sum(.5*(values[:-1][valid]+values[1:][valid])*dt)
    return float(integral), float(np.sum(dt)), int(np.sum(valid))


def distance_moments(values, coordinates, t, mask, nominal_dt):
    values, coordinates = np.asarray(values), np.asarray(coordinates)
    ds = np.linalg.norm(np.diff(coordinates, axis=0), axis=1)
    valid = good_intervals(t, mask, nominal_dt)
    valid &= np.isfinite(values[:-1]) & np.isfinite(values[1:]) & np.isfinite(ds)
    total = float(np.sum(ds[valid]))
    if total <= 0:
        return None, None, total
    rms = math.sqrt(float(np.sum(.5*(values[:-1][valid]**2+values[1:][valid]**2)*ds[valid]))/total)
    mae = float(np.sum(.5*(abs(values[:-1][valid])+abs(values[1:][valid]))*ds[valid]))/total
    return rms, mae, total


def derivative_series(values, t, nominal_dt):
    """Velocity-derived rate at interval midpoints; second rate at midpoint pairs.

    Arrays are stored on their right endpoint row for convenient CSV export;
    accompanying time arrays carry their actual derivative evaluation times.
    No filtering is applied. These sampled finite differences are NOT physical
    continuous-time peaks; their effective bandwidth is limited by 20 Hz data.
    """
    values, t = np.asarray(values, dtype=float), np.asarray(t, dtype=float)
    n = len(t)
    first = np.full(n, np.nan)
    first_time = np.full(n, np.nan)
    second = np.full(n, np.nan)
    second_time = np.full(n, np.nan)
    if n < 2:
        return first, first_time, second, second_time
    dt = np.diff(t)
    valid = (dt>0) & (dt<=1.5*nominal_dt) & np.isfinite(values[:-1]) & np.isfinite(values[1:])
    first[1:][valid] = np.diff(values)[valid]/dt[valid]
    first_time[1:] = .5*(t[:-1]+t[1:])
    if n > 2:
        mid_dt = np.diff(first_time[1:])
        valid_second = np.isfinite(first[1:-1]) & np.isfinite(first[2:]) & (mid_dt>0)
        second[2:][valid_second] = np.diff(first[1:])[valid_second]/mid_dt[valid_second]
        second_time[2:] = .5*(first_time[1:-1]+first_time[2:])
    return first, first_time, second, second_time


def geometric_phases(path, progress):
    k = np.interp(progress, path.s, path.k)
    dk = np.interp(progress, path.s, path.dk)
    turn = abs(k)>.15
    change = dk*np.sign(k)
    phase = np.full(len(progress), "line", dtype="U5")
    phase[turn & (change>.4)] = "entry"
    phase[turn & (change<-.4)] = "exit"
    phase[turn & (abs(change)<=.4)] = "apex"
    return phase, k, dk


def phase_mask(labels, phase):
    if phase == "all":
        return np.ones(len(labels), dtype=bool)
    if phase == "turn":
        return labels != "line"
    return labels == phase


def planar_derivatives(trace, nominal_dt):
    """Differentiate sampled world velocity; include turning/centripetal effects."""
    vx = trace[:,4]*np.cos(trace[:,3])
    vy = trace[:,4]*np.sin(trace[:,3])
    ax, at, jx, jt = derivative_series(vx, trace[:,0], nominal_dt)
    ay, _, jy, _ = derivative_series(vy, trace[:,0], nominal_dt)
    return dict(world_vx_mps=vx, world_vy_mps=vy,
                world_accel_x_mps2=ax, world_accel_y_mps2=ay,
                world_accel_norm_mps2=np.hypot(ax,ay), world_accel_t_s=at,
                world_jerk_x_mps3=jx, world_jerk_y_mps3=jy,
                world_jerk_norm_mps3=np.hypot(jx,jy), world_jerk_t_s=jt)


def lag_curve(t, reference, response_t, response, mask, dt=.05, max_lag=1.):
    """Normalized xcorr ref(t) vs response(t+lag); positive means response lags."""
    t, reference = np.asarray(t), np.asarray(reference)
    response_t, response = np.asarray(response_t), np.asarray(response)
    valid_response = np.isfinite(response_t) & np.isfinite(response)
    out = []
    # Interpolation is only meaningful for the expected uniformly sampled data.
    response_valid_t = response_t[valid_response]
    sampling_ok = len(response_valid_t)>1 and np.all(np.diff(response_valid_t)>0) and np.all(np.diff(response_valid_t)<=1.5*dt)
    lags = np.arange(-round(max_lag/dt), round(max_lag/dt)+1)*dt
    for lag in lags:
        if sampling_ok and np.sum(valid_response)>1:
            shifted = np.interp(t+lag, response_t[valid_response], response[valid_response], left=np.nan, right=np.nan)
            valid = mask & np.isfinite(reference) & np.isfinite(shifted)
            a, b = reference[valid], shifted[valid]
        else:
            a, b = np.array([]), np.array([])
        sa, sb = (float(np.std(a)), float(np.std(b))) if len(a) else (None, None)
        correlation = float(np.corrcoef(a, b)[0, 1]) if len(a)>2 and sa>1e-10 and sb>1e-10 else None
        out.append(dict(lag_s=float(lag), correlation=correlation, n_pairs=len(a),
                        ref_sd_radps=sa, response_sd_radps=sb))
    eligible = [r for r in out if r["correlation"] is not None and r["n_pairs"]>=max(3, .7*np.sum(mask))]
    if not eligible:
        return dict(lag_s=None, correlation=None, n_pairs=0, reliable=False,
                    status="insufficient_pairs_or_constant_signal", ref_sd_radps=None,
                    response_sd_radps=None, peak_width_95_s=None), out
    best = max(eligible, key=lambda r: (r["correlation"], -abs(r["lag_s"])))
    peak_index = out.index(best)
    lo = hi = peak_index
    while lo>0 and out[lo-1]["correlation"] is not None and out[lo-1]["correlation"]>=best["correlation"]-.05:
        lo -= 1
    while hi+1<len(out) and out[hi+1]["correlation"] is not None and out[hi+1]["correlation"]>=best["correlation"]-.05:
        hi += 1
    width = out[hi]["lag_s"]-out[lo]["lag_s"]
    reasons = []
    if best["n_pairs"]<30:
        reasons.append("fewer_than_30_pairs")
    if best["ref_sd_radps"]<.01 or best["response_sd_radps"]<.01:
        reasons.append("low_excitation")
    if best["correlation"]<.6:
        reasons.append("weak_correlation")
    if abs(best["lag_s"])>=max_lag-.5*dt:
        reasons.append("window_boundary")
    if width>.75:
        reasons.append("broad_peak")
    return {**best, "reliable":not reasons, "status":"ok" if not reasons else ";".join(reasons),
            "peak_width_95_s":float(width)}, out


def collect_expected(source, manifest):
    """Include expected failures even if a worker never wrote an NPZ."""
    args = manifest.get("arguments", {})
    expected = {f"{scenario}_{condition}_{method}_{seed}":dict(scenario=scenario, plant=condition, method=method, seed=seed)
                for scenario, condition, method, seed in itertools.product(
                    args.get("scenarios", []), args.get("plants", []),
                    args.get("methods", []), args.get("seeds", []))}
    for filename in source.glob("*.npz"):
        key = filename.stem
        if key not in expected:
            sidecar = filename.with_suffix(".json")
            if sidecar.exists():
                r = json.loads(sidecar.read_text())
                expected[key] = {k:r[k] for k in ("scenario", "plant", "method", "seed")}
    summary = source/"summary.json"
    if summary.exists():
        for r in json.loads(summary.read_text()):
            key = f"{r['scenario']}_{r['plant']}_{r['method']}_{r['seed']}"
            expected.setdefault(key, {})
            expected[key].update(r)
    return expected


def reconstruct(trace, path, cfg, result):
    params = result.get("true_plant", {})
    plant = Plant(cfg, tau_w=params.get("tau_w", .25), delay=params.get("delay", .1),
                  tau_v=.12, bias=params.get("yaw_bias", 0.))
    plant.state[:3] = path.points[0]
    states, smooth, delayed = [], [], []
    for row in trace:
        queued = list(plant.queue)
        plant.step(row[6:8])
        states.append(plant.state.copy())
        smooth.append(plant.smooth.copy())
        delayed.append(queued[0].copy() if queued else plant.smooth.copy())
    return np.asarray(states), np.asarray(smooth), np.asarray(delayed)


def numeric_trial_metrics(trace, labels, cfg, metadata, derivative_data):
    t = trace[:, 0]
    dt = cfg.dt
    planar = planar_derivatives(trace, dt)
    rows = []
    for phase in PHASES:
        mask = phase_mask(labels, phase)
        n = int(np.sum(mask))
        selected_t = t[mask]
        pairmask = good_intervals(t, mask, dt)
        firstmask = np.r_[False, pairmask]
        secondmask = np.zeros(len(t), dtype=bool)
        secondmask[2:] = pairmask[:-1] & pairmask[1:]
        row = {**metadata, "phase":phase, "n_samples":n, "n_intervals":int(np.sum(pairmask)),
               "observed_duration_s":float(np.sum(np.diff(t)[pairmask])),
               "first_observed_t_s":float(selected_t[0]) if n else None,
               "last_observed_t_s":float(selected_t[-1]) if n else None,
               "missing_initial_interval_s":float(t[0]) if len(t) else None,
               "sampling_gap_count":int(np.sum(np.diff(t)>1.5*dt)),
               "sampling_nonpositive_dt_count":int(np.sum(np.diff(t)<=0)),
               "sampling_dt_mean_s":float(np.mean(np.diff(t))) if len(t)>1 else None,
               "sampling_dt_max_s":float(np.max(np.diff(t))) if len(t)>1 else None,
               "phase_segment_count":int(np.sum(mask & ~np.r_[False, mask[:-1]]))}
        for name, col, unit in (("lateral", 9, "m"), ("yaw", 10, "rad")):
            for stat, val in moments(trace[mask, col]).items():
                row[f"{name}_{stat}_{unit}"] = val
            iae, covered, pairs = integrate_observed(t, abs(trace[:, col]), mask, dt)
            itae, _, _ = integrate_observed(t, t*abs(trace[:, col]), mask, dt)
            ise, _, _ = integrate_observed(t, trace[:, col]**2, mask, dt)
            row[f"{name}_iae_{unit}_s"] = iae
            row[f"{name}_itae_{unit}_s2"] = itae
            row[f"{name}_ise_{unit}2_s"] = ise
            row[f"{name}_time_rmse_{unit}"] = math.sqrt(ise/covered) if covered>0 else None
            rms, mae, dist = distance_moments(trace[:, col], trace[:, 1:3], t, mask, dt)
            row[f"{name}_distance_rmse_{unit}"] = rms
            row[f"{name}_distance_mae_{unit}"] = mae
            row["observed_distance_m"] = dist
            row[f"{name}_finite_sample_count"] = int(np.sum(np.isfinite(trace[mask, col])))
        row["speed_mean_mps"] = float(np.mean(trace[mask, 4])) if n else None
        row["speed_p05_mps"] = float(np.percentile(trace[mask, 4], 5)) if n else None
        row["speed_p95_mps"] = float(np.percentile(trace[mask, 4], 95)) if n else None
        for channel, col, base_unit, rate_unit, jerk_unit in (
            ("cmd_v", 6, "mps", "mps2", "mps3"),
            ("cmd_w", 7, "radps", "radps2", "radps3"),
            ("body_v", 4, "mps", "mps2", "mps3"),
            ("body_w", 5, "radps", "radps2", "radps3")):
            increments = np.diff(trace[:, col])[pairmask]
            row[f"{channel}_delta_rms_{base_unit}"] = float(np.sqrt(np.mean(increments**2))) if len(increments) else None
            row[f"{channel}_total_variation_{base_unit}"] = float(np.sum(abs(increments))) if len(increments) else None
            duration = row["observed_duration_s"]
            row[f"{channel}_tv_per_s_{rate_unit}"] = row[f"{channel}_total_variation_{base_unit}"]/duration if duration else None
            rate, _, jerk, _ = derivative_data[channel]
            rate_name = "rate" if channel.startswith("cmd") else "accel"
            for stat in ("rmse", "p95_abs", "max_abs"):
                key = "rms" if stat == "rmse" else stat
                row[f"{channel}_{rate_name}_{key}_{rate_unit}"] = moments(rate[firstmask])[stat]
                row[f"{channel}_jerk_{key}_{jerk_unit}"] = moments(jerk[secondmask])[stat]
        for channel, unit, valid in (("world_accel_norm","mps2",firstmask),("world_jerk_norm","mps3",secondmask)):
            values=planar[channel+"_"+unit][valid]
            summary=moments(values)
            row[channel+"_rms_"+unit]=summary["rmse"]
            row[channel+"_p95_"+unit]=summary["p95_abs"]
            row[channel+"_max_"+unit]=summary["max_abs"]
        compute = trace[mask, 12]*1000
        for quantile in (50, 95, 99):
            row[f"compute_p{quantile}_ms"] = float(np.percentile(compute, quantile)) if n else None
        row["compute_max_ms"] = float(np.max(compute)) if n else None
        row["deadline_misses"] = int(np.sum(compute>dt*1000)) if n else None
        row["deadline_miss_fraction"] = float(np.mean(compute>dt*1000)) if n else None
        rows.append(row)
    return rows


def group_descriptives(trials):
    numeric_keys = [k for k,v in next((r for r in trials if r.get("n_samples",0)>0), {}).items()
                    if k not in META_FIELDS and k!="seed" and isinstance(v, (int,float,np.number)) and not isinstance(v, (bool,np.bool_))]
    groups = defaultdict(list)
    for row in trials:
        groups[(row["dataset"], row["scenario"], row["condition"], row["method"], row["phase"])].append(row)
        if row["dataset"] == "tracking_final":
            groups[(row["dataset"], "fixed_three_scenarios", row["condition"], row["method"], row["phase"])].append(row)
    output = []
    for (dataset, scenario, condition, method, phase), members in sorted(groups.items()):
        for metric in numeric_keys:
            values = np.array([r[metric] for r in members if r.get(metric) is not None and np.isfinite(r[metric])], dtype=float)
            output.append(dict(dataset=dataset, scenario=scenario, condition=condition, method=method,
                               phase=phase, metric=metric, n_trials=len(members), n_valid=len(values),
                               n_complete=sum(bool(r["complete"]) for r in members),
                               n_seeds=len({r["seed"] for r in members}),
                               n_scenarios=len({r["scenario"] for r in members}),
                               mean=float(np.mean(values)) if len(values) else None,
                               sd=float(np.std(values, ddof=1)) if len(values)>1 else None,
                               median=float(np.median(values)) if len(values) else None,
                               minimum=float(np.min(values)) if len(values) else None,
                               maximum=float(np.max(values)) if len(values) else None,
                               inference="descriptive_fixed_scenarios" if scenario=="fixed_three_scenarios" else "descriptive_seed_repeats"))
    return output


PAIRED_METRICS = ("lateral_rmse_m", "lateral_p95_abs_m", "lateral_max_abs_m", "lateral_mae_m",
                  "yaw_rmse_rad", "yaw_p95_abs_rad", "speed_mean_mps",
                  "cmd_w_rate_rms_radps2", "cmd_w_total_variation_radps", "body_w_accel_rms_radps2",
                  "body_w_jerk_rms_radps3", "cmd_v_rate_rms_mps2", "body_v_accel_rms_mps2",
                  "body_v_jerk_rms_mps3", "world_accel_norm_rms_mps2", "world_jerk_norm_rms_mps3", "compute_p95_ms")


def exact_signflip_p(cluster_means):
    """Two-sided sign randomization diagnostic; assumptions are NOT established."""
    a = np.asarray(cluster_means, dtype=float)
    a = a[np.isfinite(a)]
    if not len(a):
        return None
    observed = abs(float(np.mean(a)))
    null = [abs(float(np.mean(a*np.asarray(signs)))) for signs in itertools.product((-1,1), repeat=len(a))]
    return sum(x>=observed-1e-14 for x in null)/len(null)


def paired_tables(trials):
    lookup = {(r["dataset"], r["scenario"], r["condition"], r["seed"], r["method"], r["phase"]):r for r in trials}
    pairs, grouped = [], defaultdict(list)
    for row in trials:
        if row["method"] != "full":
            continue
        native = lookup.get((row["dataset"], row["scenario"], row["condition"], row["seed"], "native", row["phase"]))
        if native is None:
            continue
        for metric in PAIRED_METRICS:
            a, b = row.get(metric), native.get(metric)
            valid = a is not None and b is not None and np.isfinite(a) and np.isfinite(b)
            paired = dict(dataset=row["dataset"], scenario=row["scenario"], condition=row["condition"],
                          seed=row["seed"], phase=row["phase"], metric=metric,
                          full_trial_id=row["trial_id"], native_trial_id=native["trial_id"],
                          both_complete=bool(row["complete"] and native["complete"]), valid=bool(valid),
                          full=a, native=b, delta_full_minus_native=a-b if valid else None,
                          reduction_pct=100*(b-a)/b if valid and b!=0 else None)
            pairs.append(paired)
            grouped[(row["dataset"], row["scenario"], row["condition"], row["phase"], metric)].append(paired)
            if row["dataset"] == "tracking_final":
                grouped[(row["dataset"], "fixed_three_scenarios", row["condition"], row["phase"], metric)].append(paired)
    summary = []
    for (dataset, scenario, condition, phase, metric), rows in sorted(grouped.items()):
        valid = [r for r in rows if r["valid"]]
        delta = np.array([r["delta_full_minus_native"] for r in valid])
        by_seed = defaultdict(list)
        for r in valid:
            by_seed[r["seed"]].append(r["delta_full_minus_native"])
        # The same noise seed is reused across scenarios. Flip whole seed blocks,
        # NEVER nine apparently independent scenario/seed pairs.
        expected_scenarios = len({r["scenario"] for r in rows})
        clusters = [float(np.mean(v)) for v in by_seed.values() if len(v)==expected_scenarios]
        ratio = (100*(np.mean([r["native"] for r in valid])-np.mean([r["full"] for r in valid]))/np.mean([r["native"] for r in valid])
                 if valid and np.mean([r["native"] for r in valid])!=0 else None)
        summary.append(dict(dataset=dataset, scenario=scenario, condition=condition, phase=phase, metric=metric,
                            n_expected_pairs=len(rows), n_valid_pairs=len(valid), n_seed_blocks=len(clusters),
                            n_scenarios=expected_scenarios, n_both_complete=sum(r["both_complete"] for r in rows),
                            native_mean=float(np.mean([r["native"] for r in valid])) if valid else None,
                            full_mean=float(np.mean([r["full"] for r in valid])) if valid else None,
                            delta_mean=float(np.mean(delta)) if valid else None,
                            delta_sd=float(np.std(delta, ddof=1)) if len(delta)>1 else None,
                            reduction_of_means_pct=ratio,
                            positive_reduction_pair_count=sum(r["reduction_pct"] is not None and r["reduction_pct"]>0 for r in valid),
                            seed_block_signflip_p=exact_signflip_p(clusters),
                            inferential_status="exploratory_randomization_diagnostic_not_confirmatory",
                            confidence_interval="not_reported_small_n_and_fixed_scenarios"))
    # One explicit multiplicity family: ALL reported exploratory signflip tests.
    ordered = sorted((r for r in summary if r["seed_block_signflip_p"] is not None), key=lambda r:r["seed_block_signflip_p"])
    previous = 0.
    for i, row in enumerate(ordered):
        previous = min(1., max(previous, (len(ordered)-i)*row["seed_block_signflip_p"]))
        row["signflip_p_holm_all_reported_tests"] = previous
        row["multiplicity_family_size"] = len(ordered)
    return pairs, summary


def metric_dictionary():
    return {
        "schema_version":1, "language":"English", "units":"SI; radians, not degrees",
        "evidence":"Synthetic closed-loop traces; FAST-LIO/NDT was not executed; no obstacles in this analysis.",
        "primary_endpoint":"Per-trial turn lateral_rmse_m, descriptive paired comparison of full versus native.",
        "timing":{"state":"Raw state and error are post-step at t; first observed state is t=0.05 s.",
                  "command":"cmd_v/cmd_w on row i were issued at t[i]-dt and held for the next control interval.",
                  "initial_missing":"No t=0 state sample is inserted. Time integrals omit [0, first_t]. Missing interval is reported.",
                  "derivative":"First differences are evaluated at interval midpoints; second differences at means of consecutive midpoints. Exported on the right endpoint row with explicit evaluation times."},
        "phases":{"rule":"Geometry only; curvature k=gradient(unwrapped path yaw, path arc length), dk=gradient(k,s), interpolated at measured true path progress.",
                  "line":"abs(k)<=0.15 m^-1", "turn":"abs(k)>0.15 m^-1",
                  "entry":"turn and sign(k)*dk>0.4 m^-2", "exit":"turn and sign(k)*dk<-0.4 m^-2",
                  "apex":"turn and abs(sign(k)*dk)<=0.4 m^-2",
                  "channel_alignment":"A phase row groups state endpoint samples at t and the command that produced that interval, issued at t-dt. Command phase bins therefore follow the resulting state interval, not an invented command-time state. Lag analysis separately uses the exact command issue clock.",
                  "caveat":"On the unsmoothed historical polyline, derivative-defined phases can alternate rapidly; these are geometric bins, not independently observed steering events."},
        "errors":{"rmse":"sqrt(mean(e_i^2)); sample weighted, one statistic per trial/phase.",
                  "mae":"mean(abs(e_i))", "signed_mean":"mean(e_i)",
                  "sd":"Sample standard deviation, ddof=1; undefined for fewer than 2 observations.",
                  "p95_abs":"95th percentile abs(e), NumPy linear quantile interpolation.", "max_abs":"max(abs(e_i))",
                  "iae":"sum of trapezoids of abs(e) dt, observed contiguous intervals only.",
                  "itae":"sum of trapezoids of t*abs(e) dt; t is mission time from actual experiment origin 0. Phase ITAE uses the same mission origin, not a reset clock.",
                  "ise":"sum of trapezoids of e^2 dt",
                  "time_rmse":"sqrt(ISE / covered observed phase duration)",
                  "distance_rmse":"sqrt(sum(0.5*(e_i^2+e_(i+1)^2)*ds_i)/sum(ds_i)); ds is Euclidean displacement between adjacent true x,y samples.",
                  "distance_mae":"Trapezoidal abs(e) weighted by the same true Euclidean displacement.",
                  "phase_integration":"Both adjacent endpoints must belong to the phase, and 0<dt<=1.5*nominal_dt. Never concatenate separated phase segments. Boundary fragments are omitted explicitly."},
        "smoothness":{"separation":"cmd channels describe requested commands; body channels describe simulated executed velocities after smoothing, delay and first-order dynamics.",
                      "delta_rms":"RMS of adjacent velocity increments; no inferred pre-first-command jump.",
                      "total_variation":"sum(abs(delta velocity)) across valid, contiguous selected intervals.",
                      "tv_per_s":"total variation / covered observed duration",
                      "rate_or_accel":"First finite difference delta velocity/delta time. cmd uses rate; body uses accel. RMS, P95 abs and max abs supplied.",
                      "planar_accel_and_jerk":"world_accel/jerk differentiate sampled world velocity v*[cos(yaw),sin(yaw)] and report vector norms; these include turning/centripetal effects. body_v acceleration/jerk describe longitudinal speed only; body_w describes rotation.",
                      "jerk":"Second finite difference of velocity using interval-midpoint timestamps; all three samples must be in phase. For command channels, this is second command derivative, not executed mechanical jerk.",
                      "bandwidth_caveat":"Unfiltered 20 Hz sampled traces, Nyquist 10 Hz; differentiation amplifies noise and P95/max depend on sample rate and any preprocessing. No claim about unobserved continuous-time maxima or hardware comfort.",
                      "normalization_caveat":"TV depends on duration. Compare both TV/time and equal-route trials; each method can have a different completion time."},
        "lag":{"reference":"reference target angular velocity = manifest.controller.speed * interpolated path curvature at state timestamp t. Also export body_v*k as a separate kinematic diagnostic.",
               "definition":"For each lag L in [-1,+1] s at 0.05 s steps, Pearson correlation(reference(t), response(t+L)); response uses its own physical timestamp and linear interpolation within observed support only. Per-lag demeaning and variance normalization.",
               "positive_lag":"Response trails the curvature reference. Negative lag indicates anticipatory association.",
               "eligibility":"At least 3 pairs and at least 70% of reference-phase samples must overlap. No interpolation across a detected sampling gap.",
               "reliable_heuristic":"At least 30 pairs; reference/response SD each >=0.01 rad/s; peak corr >=0.6; optimum not on window boundary; connected corr>=peak-0.05 width <=0.75 s. Thresholds are analysis diagnostics, not calibrated confidence levels.",
               "limitations":"Closed-loop association with a path-dependent reference; NOT causal transport-delay identification, NOT material/mechanical hysteresis and NOT proof of model validity. Low-excitation phases can be unidentifiable. Selected maxima are optimistic descriptive statistics."},
        "model_reproduction":{"definition":"Replay the same synthetic Plant with saved command sequence, saved path initial pose, manifest Config and saved true_plant parameters. Derived smoother, delayed command and body states are explicitly marked reproduced.",
                              "interpretation":"Reproduction error checks provenance/timing and data integrity only. It is NOT independent predictive validation, hardware parameter estimation or real FAST-LIO/NDT data."},
        "statistics":{"unit":"One completed or failed trial is the experimental unit. Never treat correlated time samples as independent replicates.",
                      "scenario_groups":"3 seeded repeats per scenario/condition/method; historical route has 1 repeat per method. Mean, SD, min and max are descriptive.",
                      "aggregate":"9 trials = 3 fixed scenarios x 3 reused noise seeds, not 9 IID route samples. Fixed-scenario pooled summaries are descriptive.",
                      "pairing":"full-native matched on dataset, scenario, condition and noise seed; report failures and undefined metrics without replacing them by zero.",
                      "failures":"Incomplete trials retain observed partial-horizon metrics with completion/failure/coverage fields. Such metrics do not establish successful route tracking and should not be compared as if every trial completed.",
                      "small_n":"No confidence intervals are reported: n=3 is too small for dependable coverage and scenarios are fixed. No population-wide significance claim.",
                      "signflip":"Optional exploratory two-sided exact signflip diagnostic, flipping entire seed blocks together across scenarios. At 3 blocks the smallest attainable two-sided p is 0.25. Symmetry/exchangeability is not established; this is not a confirmatory test.",
                      "multiplicity":"Holm adjustment over ALL signflip diagnostics in paired_summary.csv; family size is explicit. Native implementation UB/nonrepeatability threats are documented in the manuscript."},
        "missingness":"Empty CSV cells and JSON null mean undefined/missing. Infinity clearance means no obstacles, is exported blank with not_applicable_no_obstacles status; it is not a numeric measurement. Expected missing NPZ/failed trials remain in per-trial output.",
        "outputs":{"trial_metrics.csv":"Wide trial x phase metrics, including all failures and missing observations.",
                   "samples/*.csv":"All original raw time samples and explicitly named derived quantities; no decimation.",
                   "phase_lags.csv":"Lag diagnostics per trial x phase x response; unreliable cases retained.",
                   "lag_curves.csv":"Full normalized correlation scan over prescribed lag window.",
                   "model_parameters.csv":"Assumed synthetic true plant and controller prediction parameters for every trial.",
                   "path_profiles.csv":"All stored reference path points, arc lengths, curvature and curvature gradients, once per dataset/scenario; no smoothing or decimation.",
                   "grouped_metrics.csv":"Long descriptive group table with metric, mean, SD, minimum, maximum and valid trial count.",
                   "paired_differences.csv":"Matched full/native per-trial differences.",
                   "paired_summary.csv":"Matched group descriptive effect sizes and exploratory seed-block diagnostics.",
                   "provenance.json":"Input checksums, source checksums, inventory and missingness counts."}}


def analyze(sources, output):
    output = FilePath(output)
    data_dir, table_dir = output/"data", output/"tables"
    (data_dir/"samples").mkdir(parents=True, exist_ok=True)
    table_dir.mkdir(parents=True, exist_ok=True)
    trials, lags, curves, models, path_profiles = [], [], [], [], []
    seen_paths = {}
    provenance = dict(schema=1, evidence="synthetic_closed_loop_not_fastlio_ndt", sources=[],
                      analysis_sha256=sha256(__file__), numpy_version=np.__version__, python_version=sys.version,
                      reproduction_source_sha256={name:sha256(HERE/name) for name in ("controller.py","benchmark.py")},
                      selected_scope="All expected trials, all NPZ traces and all summary attempts in tracking_final and historical_route; no obstacles; other pilot/tuning directories are excluded and are not held-out confirmatory evidence.")
    for source in map(FilePath, sources):
        manifest_file = source/"manifest.json"
        manifest = json.loads(manifest_file.read_text())
        cfg_fields = {f.name for f in fields(Config)}
        cfg = Config(**{k:v for k,v in manifest["controller"].items() if k in cfg_fields})
        dataset = source.name
        source_record = dict(dataset=dataset, path=str(source.relative_to(HERE)) if source.is_relative_to(HERE) else str(source),
                             manifest_sha256=sha256(manifest_file), expected_trials=0, trials=[])
        expected = collect_expected(source, manifest)
        source_record["expected_trials"] = len(expected)
        for key, metadata in sorted(expected.items()):
            json_file, npz_file = source/(key+".json"), source/(key+".npz")
            result = json.loads(json_file.read_text()) if json_file.exists() else {**metadata, "complete":metadata.get("complete",False), "failure":metadata.get("failure","missing_trial_sidecar")}
            meta = dict(trial_id=key, dataset=dataset, scenario=metadata["scenario"], condition=metadata["plant"],
                        method=metadata["method"], seed=metadata["seed"], complete=bool(result.get("complete", False)),
                        failure=result.get("failure") or "")
            entry = dict(trial_id=key, json_sha256=sha256(json_file) if json_file.exists() else None,
                         npz_sha256=sha256(npz_file) if npz_file.exists() else None,
                         complete=meta["complete"], failure=meta["failure"])
            if not npz_file.exists():
                for phase in PHASES:
                    trials.append({**meta, "phase":phase, "n_samples":0, "data_status":"missing_npz"})
                entry["data_status"] = "missing_npz"
                source_record["trials"].append(entry)
                continue
            with np.load(npz_file, allow_pickle=False) as bundle:
                a = bundle["trace"].copy()
                columns = list(map(str, bundle["columns"]))
                path = Path(bundle["path"])
                obstacles = bundle["obstacles"]
            if columns != list(RAW_COLUMNS):
                raise ValueError(f"Unexpected trace schema in {npz_file}: {columns}")
            if len(obstacles):
                raise ValueError(f"Obstacle-containing trial is outside tracking-only scope: {npz_file}")
            path_key = (dataset, metadata["scenario"])
            if path_key not in seen_paths:
                seen_paths[path_key] = path.points.copy()
                for i, point in enumerate(path.points):
                    path_profiles.append(dict(dataset=dataset, scenario=metadata["scenario"], point_index=i,
                                              x_m=point[0], y_m=point[1], yaw_rad=point[2], arc_length_m=path.s[i],
                                              curvature_per_m=path.k[i], curvature_gradient_per_m2=path.dk[i]))
            elif not np.array_equal(seen_paths[path_key],path.points):
                raise ValueError(f"Inconsistent path points within dataset/scenario: {npz_file}")
            entry["raw_samples"] = len(a)
            entry["nonfinite_counts"] = {name:int(np.sum(~np.isfinite(a[:,i]))) for i,name in enumerate(columns)}
            entry["nonfinite_interpretation"] = {"clearance":"All infinity values are the no-obstacles sentinel, not missing collision measurements."}
            if not len(a):
                for phase in PHASES:
                    trials.append({**meta, "phase":phase, "n_samples":0, "data_status":"empty_trace"})
                source_record["trials"].append(entry)
                continue
            if not np.isfinite(a[:,:13]).all():
                # Keep raw/derived output for audit, but mark the affected trial.
                entry["data_status"] = "nonfinite_required_channels"
            else:
                entry["data_status"] = "ok"
            labels, k, dk = geometric_phases(path, a[:,8])
            t, command_t = a[:,0], a[:,0]-cfg.dt
            derivative_data = {name:derivative_series(a[:,col], command_t if name.startswith("cmd") else t, cfg.dt)
                               for name,col in (("cmd_v",6),("cmd_w",7),("body_v",4),("body_w",5))}
            entry["derived_missingness"]={name:{"first_derivative_undefined":int(np.sum(~np.isfinite(values[0]))),
                                                "second_derivative_undefined":int(np.sum(~np.isfinite(values[2])))}
                                         for name,values in derivative_data.items()}
            metrics = numeric_trial_metrics(a, labels, cfg, meta, derivative_data)
            for row in metrics:
                row["data_status"] = entry["data_status"]
            trials.extend(metrics)
            reproduced, smooth, delayed = reconstruct(a, path, cfg, result)
            error = reproduced-a[:,1:6]
            entry["plant_reproduction_max_abs_error_by_state"] = dict(zip(("x_m","y_m","yaw_rad","v_mps","w_radps"), np.max(abs(error),axis=0).tolist()))
            entry["plant_reproduction_interpretation"] = "Replay of the same simulator; integrity check only, not independent model validation."
            source_record["trials"].append(entry)
            params = result.get("true_plant", {})
            models.append({**{k:v for k,v in meta.items() if k not in ("complete","failure")},
                           "simulation_dt_s":cfg.dt, "plant_integrator_substep_s":cfg.dt/10,
                           "smoother_hz":30, "assumed_true_tau_v_s":.12,
                           "assumed_true_tau_w_s":params.get("tau_w",.25),
                           "assumed_true_delay_s":params.get("delay",.1),
                           "assumed_true_yaw_rate_bias_radps":params.get("yaw_bias",0.),
                           "controller_prediction_tau_v_s":cfg.tau_v, "controller_prediction_tau_w_s":cfg.tau_w,
                           "controller_prediction_delay_s":cfg.delay, "prediction_horizon_s":cfg.dt*cfg.steps,
                           "candidate_samples":cfg.samples, "smoother_accel_v_limit_mps2":cfg.accel_v,
                           "smoother_accel_w_limit_radps2":cfg.accel_w,
                           "plant_reproduction_max_abs_state_error":float(np.max(abs(error))),
                           "parameter_source":"Assumed synthetic plant; not hardware identified"})
            sample_rows = []
            planar = planar_derivatives(a, cfg.dt)
            ref_x, ref_y, ref_yaw, _, _ = path.at(a[:,8])
            for i, raw in enumerate(a):
                row = {"trial_id":key, "dataset":dataset, "scenario":meta["scenario"], "condition":meta["condition"],
                       "method":meta["method"], "seed":meta["seed"], "sample_index":i,
                       **dict(zip(columns, raw)), "command_issue_t_s":command_t[i], "geometry_phase":labels[i],
                       "ref_x_m":ref_x[i], "ref_y_m":ref_y[i], "ref_yaw_rad":ref_yaw[i],
                       "ref_k_per_m":k[i], "ref_dk_per_m2":dk[i],
                       "ref_w_target_v_radps":cfg.speed*k[i], "ref_w_body_v_radps":a[i,4]*k[i],
                       "clearance_status":"not_applicable_no_obstacles",
                       "reproduced_smoother_v_mps":smooth[i,0], "reproduced_smoother_w_radps":smooth[i,1],
                       "reproduced_delayed_v_mps":delayed[i,0], "reproduced_delayed_w_radps":delayed[i,1],
                       "reproduced_body_v_mps":reproduced[i,3], "reproduced_body_w_radps":reproduced[i,4],
                       "reproduction_error_body_v_mps":error[i,3], "reproduction_error_body_w_radps":error[i,4]}
                for channel, (rate, rate_t, jerk, jerk_t) in derivative_data.items():
                    row.update({channel+"_first_derivative":rate[i], channel+"_first_derivative_t_s":rate_t[i],
                                channel+"_second_derivative":jerk[i], channel+"_second_derivative_t_s":jerk_t[i]})
                row.update({name:values[i] for name,values in planar.items()})
                sample_rows.append(row)
            sample_file=data_dir/"samples"/(key+".csv")
            write_csv(sample_file, sample_rows)
            entry["sample_csv_sha256"]=sha256(sample_file)
            for phase in PHASES:
                mask = phase_mask(labels, phase)
                for name, rt, response in (("command_w",command_t,a[:,7]), ("body_w",t,a[:,5])):
                    peak, scan = lag_curve(t, cfg.speed*k, rt, response, mask, cfg.dt)
                    lag_meta = {**{k:v for k,v in meta.items() if k not in ("complete","failure")}, "phase":phase, "response":name}
                    lags.append({**lag_meta, **peak, "lag_window_min_s":-1., "lag_window_max_s":1., "lag_resolution_s":cfg.dt})
                    curves.extend({**lag_meta, **point} for point in scan)
        provenance["sources"].append(source_record)
    grouped = group_descriptives(trials)
    pairs, paired = paired_tables(trials)
    write_csv(data_dir/"trial_metrics.csv", trials)
    write_csv(data_dir/"phase_lags.csv", lags)
    write_csv(data_dir/"lag_curves.csv", curves)
    write_csv(data_dir/"model_parameters.csv", models)
    write_csv(data_dir/"path_profiles.csv", path_profiles)
    write_csv(table_dir/"grouped_metrics.csv", grouped)
    write_csv(table_dir/"paired_differences.csv", pairs)
    write_csv(table_dir/"paired_summary.csv", paired)
    provenance["totals"] = dict(expected_trials=sum(s["expected_trials"] for s in provenance["sources"]),
                                observed_trials=sum("raw_samples" in t for s in provenance["sources"] for t in s["trials"]),
                                raw_samples=sum(t.get("raw_samples",0) for s in provenance["sources"] for t in s["trials"]),
                                complete_trials=sum(t["complete"] for s in provenance["sources"] for t in s["trials"]),
                                failed_trials=sum(not t["complete"] for s in provenance["sources"] for t in s["trials"]),
                                trial_phase_rows=len(trials), lag_rows=len(lags))
    (data_dir/"metric_dictionary.json").write_text(json.dumps(metric_dictionary(), indent=2, allow_nan=False)+"\n")
    (data_dir/"provenance.json").write_text(json.dumps(provenance, indent=2, allow_nan=False)+"\n")
    print(json.dumps(provenance["totals"], indent=2))
    return provenance


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sources", nargs="+", type=FilePath, default=[HERE/"results/tracking_final", HERE/"results/historical_route"])
    parser.add_argument("--output", type=FilePath, default=HERE/"paper")
    args = parser.parse_args()
    analyze(args.sources, args.output)


if __name__ == "__main__":
    main()
