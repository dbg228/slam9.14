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


def output_error_prediction(smooth_input, tau, gain, bias, delay_steps, dt=.05):
    """Exact sampled first-order response, assuming the documented zero reset."""
    from scipy.signal import lfilter
    u=np.asarray(smooth_input)
    delayed=np.zeros(len(u))
    if delay_steps:
        delayed[delay_steps:]=u[:-delay_steps]
    else:
        delayed[:]=u
    a=np.exp(-dt/tau)
    return np.r_[0.,lfilter([1-a],[1.,-a],gain*delayed[1:]+bias)]


def fit_output_error(smooth_input, observed_output, sigma, dt=.05):
    """Output-error NLS avoids treating noisy previous output as exact input.

    Fit tau, gain and constant bias separately for each integer delay candidate.
    The initial physical state is known to be zero by the synthetic reset.
    Bounds are broad fixed modeling choices, not chosen using fitted truth.
    """
    from scipy.optimize import least_squares
    u=np.asarray(smooth_input); y=np.asarray(observed_output)
    candidates=[]
    for d in range(9):
        def residual(parameters):
            return (output_error_prediction(u,*parameters,d,dt)[1:]-y[1:])/sigma
        result=least_squares(residual,[.20,1.,0.],
                             bounds=([.03,.5,-.05],[2.,1.5,.05]),
                             ftol=1e-10,xtol=1e-10,gtol=1e-10,max_nfev=160,
                             x_scale="jac")
        candidates.append({"delay_steps":d,"delay_s":d*dt,
                           "tau_s":float(result.x[0]),"gain":float(result.x[1]),
                           "bias":float(result.x[2]),
                           "train_observed_rmse":float(np.sqrt(np.mean((result.fun*sigma)**2))),
                           "optimizer_success":bool(result.success),"optimizer_nfev":int(result.nfev),
                           "jacobian_rank":int(np.linalg.matrix_rank(result.jac)),
                           "jacobian_condition_number":float(np.linalg.cond(result.jac)),
                           "active_bound_count":int(np.count_nonzero(result.active_mask))})
    fit=dict(min(candidates,key=lambda c:c["train_observed_rmse"]))
    fit["delay_candidates"]=candidates
    fit["parameter_bounds"]={"tau_s":[.03,2.],"gain":[.5,1.5],"bias":[-.05,.05]}
    fit["known_initial_output"]=0.
    fit["noise_sigma"]=sigma
    return fit


def residual_statistics(residual):
    r=np.asarray(residual)
    centered=r-r.mean()
    power=float(np.dot(centered,centered))
    acf=[float(np.dot(centered[:-lag],centered[lag:])/power) if power else 0.
         for lag in range(1,21)]
    return {"mean":float(np.mean(r)),"sample_sd":float(np.std(r,ddof=1)),
            "rmse":float(np.sqrt(np.mean(r*r))),"max_abs":float(np.max(abs(r))),
            "autocorrelation_lags_1_to_20":acf,
            "durbin_watson":float(np.sum(np.diff(r)**2)/np.sum(r*r)) if np.sum(r*r) else None}


def noisy_model_experiments(output):
    """Independent synthetic observation-noise experiment, without retuning."""
    sigma=np.array([.002,.003])
    root={"evidence":EVIDENCE,"method":"recursive_output_error_nonlinear_least_squares_integer_delay_grid",
          "training_input":"step; exact logged smoother output",
          "validation_input":"independent multisine; exact logged smoother output",
          "training_noise_seed":303,"validation_noise_seed":304,
          "measurement_noise_sigma":{"v_mps":.002,"w_radps":.003},
          "noise_model":"independent zero-mean Gaussian output observation noise only",
          "initial_condition":"known zero body velocity from synthetic reset",
          "plants":{},"noise_monte_carlo_replicates":5,
          "limitations":["Synthetic observation-noise sensitivity, not real robot identification",
                         "Exact smoother input is assumed known; no input measurement error, colored noise or unmodeled slip",
                         "Plant structure matches fitted structure; independent inputs test prediction but do not establish external validity",
                         "Delay restricted to 50ms grid; no sub-sample delay claim",
                         "Five repeated noise draws share the same latent plant/input; sample SD is not a confidence interval or hardware variability",
                         "Noise amplitudes are assumed experiment settings, not estimated FAST-LIO/NDT sensor statistics"]}
    parameter_rows=[]
    for kind in ["nominal","heldout"]:
        train=np.load(output/f"openloop_{kind}_step.npz")["trace"]
        validation=np.load(output/f"openloop_{kind}_multisine.npz")["trace"]
        replicates=[]
        for repeat in range(5):
            train_seed=303+2*repeat; validation_seed=304+2*repeat
            train_obs=train[:,7:9]+np.random.default_rng(train_seed).normal(size=(len(train),2))*sigma
            val_obs=validation[:,7:9]+np.random.default_rng(validation_seed).normal(size=(len(validation),2))*sigma
            record={"training_noise_seed":train_seed,"validation_noise_seed":validation_seed,"channels":{}}
            train_predictions=[]; val_predictions=[]
            for j,channel in enumerate(["v","w"]):
                fit=fit_output_error(train[:,3+j],train_obs[:,j],sigma[j])
                parameters=[fit["tau_s"],fit["gain"],fit["bias"],fit["delay_steps"]]
                training_prediction=output_error_prediction(train[:,3+j],*parameters)
                prediction=output_error_prediction(validation[:,3+j],*parameters)
                fit.update({"output_units":"m/s" if channel=="v" else "rad/s",
                            "validation_observed_rmse":float(np.sqrt(np.mean((prediction[1:]-val_obs[1:,j])**2))),
                            "validation_latent_truth_rmse":float(np.sqrt(np.mean((prediction[1:]-validation[1:,7+j])**2))),
                            "training_latent_truth_rmse":float(np.sqrt(np.mean((training_prediction[1:]-train[1:,7+j])**2))),
                            "validation_observed_residual":residual_statistics(val_obs[1:,j]-prediction[1:]),
                            "residual_sign":"observed minus predicted; excludes known initial sample"})
                record["channels"][channel]=fit
                parameter_rows.append([0. if kind=="nominal" else 1.,repeat,train_seed,validation_seed,
                                       float(j),fit["delay_s"],fit["tau_s"],fit["gain"],fit["bias"],
                                       fit["train_observed_rmse"],fit["validation_observed_rmse"],
                                       fit["validation_latent_truth_rmse"]])
                train_predictions.append(training_prediction);val_predictions.append(prediction)
            if repeat==0:
                cols=["t","v_observed","w_observed","v_latent","w_latent","v_prediction","w_prediction",
                      "cmd_v","cmd_w","smooth_v","smooth_w"]
                save_trace(output/f"noisy_model_prediction_{kind}",
                           np.column_stack([validation[:,0],val_obs,validation[:,7:9],*val_predictions,
                                            validation[:,1:3],validation[:,3:5]]),cols)
                save_trace(output/f"noisy_model_training_{kind}",
                           np.column_stack([train[:,0],train_obs,train[:,7:9],*train_predictions,
                                            train[:,1:3],train[:,3:5]]),cols)
            replicates.append(record)
        sensitivity={}
        for channel in ["v","w"]:
            records=[r["channels"][channel] for r in replicates]
            sensitivity[channel]={"n_noise_draws":5,"delay_selection_counts":{
                str(d):sum(r["delay_steps"]==d for r in records) for d in range(9)}}
            for parameter in ["tau_s","gain","bias","validation_observed_rmse","validation_latent_truth_rmse"]:
                values=[r[parameter] for r in records]
                sensitivity[channel][parameter]={"mean":float(np.mean(values)),"sample_sd":float(np.std(values,ddof=1)),
                                                   "min":float(min(values)),"max":float(max(values))}
        root["plants"][kind]={"assumed_true_parameters":plant_parameters(kind),
                               "base_fit":replicates[0],"noise_replicates":replicates,
                               "noise_sensitivity":sensitivity}
    save_trace(output/"noisy_model_parameters",np.asarray(parameter_rows),
               ["plant_code_nominal0_heldout1","repeat","train_noise_seed","validation_noise_seed",
                "channel_code_v0_w1","delay_s","tau_s","gain","bias","train_observed_rmse",
                "validation_observed_rmse","validation_latent_truth_rmse"])
    root["source_sha256"]=hashlib.sha256(FilePath(__file__).read_bytes()).hexdigest()
    root["trace_sha256"]={p.name:hashlib.sha256(p.read_bytes()).hexdigest()
                           for p in output.glob("openloop_*.npz")}
    dump_json(output/"noisy_output_error_validation.json",root)
    dump_json(output/"noisy_model_manifest.json",{
        "source_sha256":root["source_sha256"],"evidence":EVIDENCE,
        "separate_extension":"Added after 24 frozen convergence trials; no changes to controller or convergence run settings",
        "convergence_source_archive":"convergence_protocol_source.py",
        "convergence_source_archive_sha256":hashlib.sha256((output/"convergence_protocol_source.py").read_bytes()).hexdigest()
                                            if (output/"convergence_protocol_source.py").exists() else None,
        "noise_sigma_v_mps":.002,"noise_sigma_w_radps":.003,
        "noise_seeds":[[303+2*j,304+2*j] for j in range(5)],
        "noise_inputs":"same latent step/multisine plant data; independent Gaussian observation draws",
    })
    return root


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--output",type=FilePath,default=HERE/"paper/supplement")
    ap.add_argument("--worker",action="store_true")
    ap.add_argument("--method",choices=["native","tracker","ideal","full"])
    ap.add_argument("--plant",choices=["nominal","heldout"])
    ap.add_argument("--seed",type=int,default=0)
    ap.add_argument("--noisy-model-only",action="store_true",
                    help="Extend existing openloop traces with independent noisy output-error fits")
    args=ap.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    if args.noisy_model_only:
        noisy_model_experiments(args.output)
        return
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
