"""Supplemental, synthetic-only convergence and actuator-model experiments.

No ROS commands are published. All controller and plant settings are frozen to
the existing benchmark; only the documented initial condition is changed.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path as FilePath
import subprocess
import sys
import time

import numpy as np

HERE = FilePath(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from benchmark import Plant, ROOT, make_path
from controller import Config, GuidedMPPI, wrap

EVIDENCE = "synthetic_closed_loop_not_fastlio_ndt_not_hardware_identification"
TRACE_COLUMNS = [
    "t", "x", "y", "yaw", "v", "w", "cmd_v", "cmd_w", "s", "e",
    "e_yaw", "k", "compute_s", "clearance", "t_command_issue", "command_valid",
    "measured_x", "measured_y", "measured_yaw", "measured_v", "measured_w",
    "smooth_v", "smooth_w", "delayed_v", "delayed_w", "effective_samples",
    "diagnostics_available", "min_sample_cost", "selected_cost",
]
OPEN_COLUMNS = ["t", "cmd_v", "cmd_w", "smooth_v", "smooth_w", "delayed_v",
                "delayed_w", "v", "w"]


def dump_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def save_trace(destination, trace, columns, **arrays):
    np.savez_compressed(destination.with_suffix(".npz"), trace=trace,
                        columns=np.asarray(columns), **arrays)
    np.savetxt(destination.with_suffix(".csv"), trace, delimiter=",",
               header=",".join(columns), comments="", fmt="%.12g")


def plant_parameters(kind):
    return ({"tau_w": .25, "delay": .10, "tau_v": .12, "bias": 0.}
            if kind == "nominal" else
            {"tau_w": .38, "delay": .15, "tau_v": .12, "bias": .008})


def settling_metrics(times, lateral, heading, dwell=1., ey_limit=.01,
                     heading_limit=math.radians(2.)):
    """Sampled threshold residence, including endpoint samples at both ends.

    first_window: first in-band interval lasting dwell seconds, even if exited.
    last_exit: start of the final in-band interval, only if it lasts dwell.
    An unobserved event is right censored at the final observation time.
    These are empirical finite-horizon residence measures, not Lyapunov proofs.
    """
    t = np.asarray(times, dtype=float)
    inside = (abs(np.asarray(lateral)) <= ey_limit) & (abs(np.asarray(heading)) <= heading_limit)
    starts = np.flatnonzero(inside & np.r_[True, ~inside[:-1]])
    ends = np.flatnonzero(inside & np.r_[~inside[1:], True])
    sustained = [(int(a), int(b)) for a, b in zip(starts, ends)
                 if t[b] - t[a] >= dwell - 1e-9]
    first = float(t[sustained[0][0]]) if sustained else None
    final = next((float(t[a]) for a, b in sustained if b == len(t)-1), None)
    return {"joint_settling_first_window_s": first,
            "last_exit_settling_s": final,
            "first_window_censored": first is None,
            "last_exit_censored": final is None,
            "censor_time_s": float(t[-1]), "dwell_s": dwell,
            "lateral_threshold_m": ey_limit, "heading_threshold_rad": heading_limit,
            "final_joint_in_band": bool(inside[-1]),
            "fraction_joint_in_band": float(np.mean(inside)),
            "threshold_exit_count": int(np.sum(inside[:-1] & ~inside[1:]))}


class RolloutObserver:
    """Observe existing evaluations; no costs, actions or RNG draws are changed."""
    def __init__(self, controller):
        self.calls = []
        original = controller.rollout

        def observe(*args, **kwargs):
            value = original(*args, **kwargs)
            costs, safe = value[:2]
            is_brake = kwargs.get("guide_only", False)
            self.calls.append((is_brake, np.asarray(costs).copy(), np.asarray(safe).copy()))
            return value
        controller.rollout = observe

    def cost_pair(self, status):
        samples = self.calls[0]
        costs, safe = samples[1:]
        best = float(np.min(costs[safe])) if np.any(safe) else 0.
        selected = float(self.calls[-1][1][0]) if status == "mean" else best
        return best, selected


def execute_convergence(method, kind, seed, output):
    cfg = Config()
    path = make_path("left")
    plant = Plant(cfg, **plant_parameters(kind))
    plant.state[:3] = path.points[0] + [0., .10, math.radians(10.)]
    max_steps = int((path.length/cfg.speed*1.65+8)/cfg.dt)
    noise = np.random.default_rng(seed+1900).normal(size=(max_steps, 5)) * [.002,.002,.003,.002,.003]
    observer = None
    if method == "native":
        from native.native_mppi import NativeMPPI
        controller = NativeMPPI(str(ROOT / "robot_data/config/navigation.yaml"),
                                "InspectionStrict", seed=seed)
        controller.set_path(path.points)
    else:
        controller = GuidedMPPI(path, cfg, seed, mode=method)
        observer = RolloutObserver(controller)
    hint, e0, tangent = path.project(*plant.state[:2], 0.)
    # Initial command/diagnostic fields are unavailable (command_valid=0).
    rows = [[0., *plant.state, 0., 0., hint, e0,
             float(wrap(plant.state[2]-tangent)), float(np.interp(hint,path.s,path.k)),
             0., 0., 0., 0., *plant.state, 0., 0., 0., 0., 0., 0., 0., 0.]]
    complete, failure = False, None
    began_trial = time.perf_counter()
    statuses = []
    try:
        for step in range(max_steps):
            measured = plant.state + noise[step]
            issue_t = plant.time
            if observer:
                observer.calls.clear()
            began = time.perf_counter()
            if method == "native":
                command = controller.step(*map(float, measured))
            else:
                pending = list(plant.queue)[-round(cfg.delay/cfg.dt):]
                command = controller.command(measured, plant.smooth, pending, [])
            compute_s = time.perf_counter()-began
            # This is the exact queue element Plant.step pops after append.
            delayed = plant.queue[0].copy() if plant.delay_steps else None
            plant.step(command)
            if delayed is None:
                delayed = plant.smooth.copy()
            x,y,yaw,v,w = plant.state
            hint,e,tangent = path.project(x,y,hint)
            diag = getattr(controller,"last_diagnostics",{})
            status = diag.get("status","native")
            costs = observer.cost_pair(status) if observer else (0.,0.)
            rows.append([plant.time,*plant.state,*map(float,command),hint,e,
                         float(wrap(yaw-tangent)),float(np.interp(hint,path.s,path.k)),
                         compute_s,0.,issue_t,1.,*measured,*plant.smooth,*delayed,
                         float(diag.get("effective_samples",0.)),float(observer is not None),*costs])
            statuses.append(status)
            if hint >= path.length-.035 and np.hypot(x-path.points[-1,0],y-path.points[-1,1]) < .08:
                complete = True
                break
            if abs(e) > .8:
                failure = "tracking_divergence"
                break
    except Exception as exc:
        failure = f"controller_exception: {exc}"
    finally:
        if hasattr(controller,"close"):
            controller.close()
    if not complete and not failure:
        failure = "timeout"
    a = np.asarray(rows,dtype=float)
    metrics = settling_metrics(a[:,0],a[:,9],a[:,10])
    metrics.update({"method":method,"plant":kind,"seed":seed,"scenario":"left",
                    "evidence":EVIDENCE,"complete":complete,"failure":failure,
                    "initial_lateral_error_m":.10,"initial_heading_error_rad":math.radians(10.),
                    "time_s":float(plant.time),"steps":len(rows)-1,
                    "wall_time_s":time.perf_counter()-began_trial,
                    "lateral_rmse_m":float(np.sqrt(np.mean(a[:,9]**2))),
                    "lateral_max_abs_m":float(np.max(abs(a[:,9]))),
                    "heading_rmse_rad":float(np.sqrt(np.mean(a[:,10]**2))),
                    "compute_p95_ms":float(np.percentile(a[1:,12],95)*1000) if len(a)>1 else None,
                    "deadline_misses":int(np.sum(a[1:,12]>cfg.dt)),
                    "diagnostic_instrumentation":"rollout wrapper copies costs only; no RNG/control change",
                    "true_plant":plant_parameters(kind)})
    key = f"convergence_{kind}_{method}_{seed}"
    save_trace(output/key,a,TRACE_COLUMNS,path=path.points,status=np.asarray(["initial"]+statuses),
               noise=noise[:len(rows)-1])
    dump_json(output/(key+".json"),metrics)
    return metrics


def input_command(t, profile):
    if profile == "step":
        if t < 3.:
            return np.zeros(2)
        v = .15 if t < 38. else 0.
        w = .25 if 7. <= t < 15. else (-.25 if 23. <= t < 31. else 0.)
        return np.array([v,w])
    if t < 2.:
        return np.zeros(2)
    z = t-2.
    return np.array([.12+.03*np.sin(.31*z)+.015*np.sin(.83*z),
                     .12*np.sin(.51*z)+.08*np.sin(1.17*z)+.04*np.sin(2.39*z)])


def openloop_trace(kind, profile, output):
    cfg=Config()
    plant=Plant(cfg,**plant_parameters(kind))
    rows=[[0.]*len(OPEN_COLUMNS)]
    for _ in range(round(45./cfg.dt)):
        command=input_command(plant.time,profile)
        delayed=plant.queue[0].copy() if plant.delay_steps else None
        plant.step(command)
        if delayed is None:
            delayed=plant.smooth.copy()
        rows.append([plant.time,*command,*plant.smooth,*delayed,*plant.state[3:]])
    a=np.asarray(rows)
    save_trace(output/f"openloop_{kind}_{profile}",a,OPEN_COLUMNS)
    return a


def fit_channel(train, validation, channel, dt=.05):
    """Recover unity-gain first-order+integer-delay model from smooth input.

    End-of-interval samples obey y[k]=a*y[k-1]+(1-a)*(u[k-d]+bias).
    Candidate delays use equal fitting indices, and validation is on an entirely
    different input sequence. No noisy real sensor data is present here.
    """
    ui,yi=(3,7) if channel=="v" else (4,8)
    u,y=train[:,ui],train[:,yi]
    delays=range(9)
    k=np.arange(10,len(y))
    candidates=[]
    for d in delays:
        design=np.c_[u[k-d]-y[k-1],np.ones(len(k))]
        gain,offset=np.linalg.lstsq(design,y[k]-y[k-1],rcond=None)[0]
        a=1.-gain
        if not 0.<a<1. or abs(gain)<1e-12:
            continue
        residual=(y[k]-y[k-1])-design@np.array([gain,offset])
        candidates.append({"delay_steps":d,"delay_s":d*dt,
                           "tau_s":float(-dt/np.log(a)),"a":float(a),
                           "bias":float(offset/gain),
                           "train_one_step_rmse":float(np.sqrt(np.mean(residual**2))),
                           "design_condition_number":float(np.linalg.cond(design))})
    if not candidates:
        return {"fit_available":False,"reason":"no stable first-order candidate"},None
    fit=dict(min(candidates,key=lambda c:c["train_one_step_rmse"]))
    d,a,bias=fit["delay_steps"],fit["a"],fit["bias"]
    uv,yv=validation[:,ui],validation[:,yi]
    predicted=np.zeros(len(yv));predicted[0]=yv[0]
    one_step=[]
    for n in range(1,len(yv)):
        inp=uv[n-d] if n-d>=0 else 0.
        predicted[n]=a*predicted[n-1]+(1-a)*(inp+bias)
        one_step.append(a*yv[n-1]+(1-a)*(inp+bias))
    fit.update({"fit_available":True,"train_profile":"step",
                "validation_profile":"independent_multisine",
                "validation_recursive_rmse":float(np.sqrt(np.mean((predicted-yv)**2))),
                "validation_recursive_max_abs":float(np.max(abs(predicted-yv))),
                "validation_one_step_rmse":float(np.sqrt(np.mean((np.asarray(one_step)-yv[1:])**2))),
                "all_delay_candidates":candidates,
                "input":"30Hz smoother output sampled at 20Hz, before transport queue",
                "output":"true synthetic body velocity; no sensor measurement noise",
                "gain_constraint":1.,"model":"G(s)=exp(-D*s)/(tau*s+1), additive steady-state bias",
                "identification_scope":"synthetic parameter recovery and heldout-input consistency only"})
    return fit,predicted


def model_experiments(output):
    results={"evidence":EVIDENCE,"dt_s":.05,"profiles":{},"plants":{}}
    for kind in ["nominal","heldout"]:
        train=openloop_trace(kind,"step",output)
        validation=openloop_trace(kind,"multisine",output)
        fits={}
        predictions=[]
        for channel in ["v","w"]:
            fit,predicted=fit_channel(train,validation,channel)
            fits[channel]=fit
            predictions.append(predicted if predicted is not None else np.zeros(len(validation)))
        save_trace(output/f"model_prediction_{kind}",
                   np.column_stack([validation[:,0],validation[:,7:9],*predictions]),
                   ["t","v_true","w_true","v_fit_prediction","w_fit_prediction"])
        frequencies=np.geomspace(.02,2.5,100)
        frequency_rows=[]
        for f in frequencies:
            row=[f]
            for channel in ["v","w"]:
                fit=fits[channel]
                if fit["fit_available"]:
                    omega=2*np.pi*f
                    value=np.exp(-1j*omega*fit["delay_s"])/(1+1j*omega*fit["tau_s"])
                    row.extend([abs(value),20*np.log10(abs(value)),
                                (-omega*fit["delay_s"]-np.arctan(omega*fit["tau_s"]))*180/np.pi])
                else:
                    row.extend([0.,0.,0.])
            frequency_rows.append(row)
        save_trace(output/f"frequency_response_{kind}",np.asarray(frequency_rows),
                   ["frequency_hz","v_gain","v_gain_db","v_phase_deg",
                    "w_gain","w_gain_db","w_phase_deg"])
        results["plants"][kind]={"assumed_true_parameters":plant_parameters(kind),"fits":fits}
    results["profiles"]={
        "step":"45s; at3s v=0.15m/s, at38s v=0; w=+0.25rad/s at7-15s, -0.25 at23-31s, otherwise0",
        "multisine":"45s independent validation; first2s zero; then v=.12+.03sin(.31t)+.015sin(.83t); w=.12sin(.51t)+.08sin(1.17t)+.04sin(2.39t), t relative to2s",
    }
    results["limitations"]=[
        "No real FAST-LIO, NDT, CAN, motor or lidar identification",
        "Delay is restricted to an integer multiple of 50ms; cannot infer sub-sample delay",
        "Body velocities are noiseless synthetic states; exact recovery is expected from matching assumed plant",
        "Raw command->body response includes a nonlinear rate limiter; the stated transfer applies downstream of smoother",
        "Dynamic input-output loop area reflects lag and input frequency; no static mechanical hysteresis model is present",
    ]
    dump_json(output/"model_validation.json",results)
    return results


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--output",type=FilePath,default=HERE/"paper/supplement")
    ap.add_argument("--worker",action="store_true")
    ap.add_argument("--method",choices=["native","tracker","ideal","full"])
    ap.add_argument("--plant",choices=["nominal","heldout"])
    ap.add_argument("--seed",type=int,default=0)
    args=ap.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    from acceleration.fast_rollout import prepare
    backend=prepare()
    if args.worker:
        result=execute_convergence(args.method,args.plant,args.seed,args.output)
        print(json.dumps(result,allow_nan=False),flush=True)
        return
    dump_json(args.output/"manifest.json",{
        "schema":1,"evidence":EVIDENCE,"controller":asdict(Config()),"backend":backend,
        "initial_state":{"e_y_m":.10,"e_yaw_rad":math.radians(10.),"v_mps":0.,"w_radps":0.},
        "scenario":"left","methods":["native","tracker","ideal","full"],
        "plants":["nominal","heldout"],"seeds":[0,1,2],"retuning":False,
        "source_sha256":{name:hashlib.sha256((HERE/name).read_bytes()).hexdigest()
                          for name in ["paper_supplement.py","controller.py","benchmark.py"]},
        "production_navigation_sha256":hashlib.sha256((ROOT/"robot_data/config/navigation.yaml").read_bytes()).hexdigest(),
        "trace_columns":TRACE_COLUMNS,"openloop_columns":OPEN_COLUMNS,
        "sample_semantics":"t is post-step state timestamp; command issue time=t-dt; measured state is pre-step controller input; smooth/delayed correspond to interval ending at t; initial row command_valid=0",
        "timing_semantics":"20Hz simulated time; compute_s uses actual perf_counter wall timing including observer; results are hardware/load dependent",
        "native_diagnostics":"native internal rollout costs and ESS unavailable; diagnostics_available=0; numeric zero sentinels must be excluded",
        "no_obstacles":True,"clearance_column":"zero sentinel because no obstacles; do not interpret as collision",
        "convergence_scope":"finite-sample empirical residence; no theoretical stability or asymptotic convergence claim",
    })
    model_experiments(args.output)
    results=[]
    for kind in ["nominal","heldout"]:
        for seed in [0,1,2]:
            for method in ["native","tracker","ideal","full"]:
                key=f"convergence_{kind}_{method}_{seed}"
                cmd=[sys.executable,str(FilePath(__file__).resolve()),"--worker","--output",str(args.output.resolve()),
                     "--method",method,"--plant",kind,"--seed",str(seed)]
                subprocess.run(cmd,check=True,timeout=240,stdout=subprocess.DEVNULL)
                r=json.loads((args.output/(key+".json")).read_text())
                results.append(r)
                dump_json(args.output/"convergence_summary.json",results)
                print(json.dumps({k:r[k] for k in ["method","plant","seed","complete","joint_settling_first_window_s","last_exit_settling_s"]}),flush=True)


if __name__=="__main__":
    main()
