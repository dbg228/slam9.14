"""Publication graphics from frozen simulation traces, never ROS commands.

Run: python3 tests/turning_mppi/paper_figures.py
Outputs vector PDF/SVG and 300 dpi PNG with an auditable bilingual index.
The script does not modify controllers, experiments, or input measurements.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/turning_mppi_paper_mpl")
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.ticker import FixedLocator, MaxNLocator
import numpy as np

HERE = Path(__file__).resolve().parent
METHODS = ("native", "tracker", "ideal", "full")
LABEL = {"native": "Native MPPI", "tracker": "Preview tracker",
         "ideal": "Ideal-dynamics ablation", "full": "Guided MPC"}
SHORT_LABEL = {"native": "Native", "tracker": "Tracker", "ideal": "Ideal", "full": "Guided"}
COLORS = {"native": "#0072B2", "tracker": "#009E73",
          "ideal": "#CC79A7", "full": "#D55E00"}
SCENARIOS = ("left", "right", "s_bend")
CONDITIONS = ("nominal", "heldout")
SCENE_LABEL = {"left": "Left turn", "right": "Right turn", "s_bend": "S bend"}
COND_LABEL = {"nominal": "Nominal plant", "heldout": "Held-out plant"}
MM = 1 / 25.4
REGISTRY: list[dict] = []


def style():
    plt.rcParams.update({
        "font.family": "DejaVu Sans", "font.size": 7.5,
        "axes.labelsize": 7.5, "axes.titlesize": 8,
        "xtick.labelsize": 6.8, "ytick.labelsize": 6.8,
        "legend.fontsize": 6.8, "lines.linewidth": 1.15,
        "axes.spines.top": False, "axes.spines.right": False,
        "axes.linewidth": .55, "xtick.major.width": .55,
        "ytick.major.width": .55, "grid.linewidth": .4,
        "grid.alpha": .22, "axes.axisbelow": True,
        "pdf.fonttype": 42, "ps.fonttype": 42,
        "svg.fonttype": "none", "savefig.facecolor": "white",
        "figure.facecolor": "white", "axes.unicode_minus": True,
    })


def load_trace(source: Path) -> dict:
    with np.load(source, allow_pickle=False) as z:
        trace = np.asarray(z["trace"], float)
        columns = z["columns"].tolist()
        data = {str(c): trace[:, i] for i, c in enumerate(columns)}
        if "path" in z:
            data["path"] = np.asarray(z["path"], float)
    data["source"] = str(source.relative_to(HERE))
    return data


def tracking_trials(directory: Path):
    rows = json.loads((directory / "summary.json").read_text())
    trials = []
    for row in rows:
        key = f"{row['scenario']}_{row['plant']}_{row['method']}_{row['seed']}"
        d = load_trace(directory / (key + ".npz"))
        d.update(meta=row, trial_id=key)
        trials.append(d)
    return trials


def choose(trials, scenario=None, condition=None, method=None, seed=None):
    return [d for d in trials
            if (scenario is None or d["meta"]["scenario"] == scenario)
            and (condition is None or d["meta"]["plant"] == condition)
            and (method is None or d["meta"]["method"] == method)
            and (seed is None or d["meta"]["seed"] == seed)]


def decorate(axes):
    for ax in np.asarray(axes, dtype=object).ravel():
        ax.grid(True)
        if not isinstance(ax.xaxis.get_major_locator(),FixedLocator):
            ax.xaxis.set_major_locator(MaxNLocator(5))
        ax.yaxis.set_major_locator(MaxNLocator(5))


def panel_letters(axes):
    for i, ax in enumerate(np.asarray(axes, dtype=object).ravel()):
        ax.text(.015, 1.035, f"({chr(97+i)})", transform=ax.transAxes,
                va="bottom", fontsize=8, weight="bold")


def method_legend(fig, y=.99):
    handles = [Line2D([], [], color=COLORS[m], label=LABEL[m]) for m in METHODS]
    fig.legend(handles=handles, loc="upper center", bbox_to_anchor=(.5, y),
               ncol=4, frameon=False, columnspacing=1.3, handlelength=2)


def save(fig, output, name, title, caption_en, caption_zh, sources,
         manipulations, sample_unit, width_mm=178, height_mm=None):
    paths = []
    if height_mm is None:
        height_mm = float(fig.get_figheight() / MM)
    for suffix in ("pdf", "svg", "png"):
        p = output / f"{name}.{suffix}"
        kwargs = {"dpi": 300} if suffix == "png" else {}
        metadata = {"Title": title, "Creator": "paper_figures.py; Matplotlib"}
        if suffix == "png":
            metadata["Description"] = caption_en
        fig.savefig(p, bbox_inches="tight", pad_inches=.035, metadata=metadata, **kwargs)
        paths.append(p.name)
    plt.close(fig)
    unique_sources = sorted(set(str(s) for s in sources))
    REGISTRY.append({"id": name, "title": title,
                     "caption_en": caption_en,
                     "files": paths, "sources": unique_sources,
                     "manipulations": manipulations, "sample_unit": sample_unit,
                     "nominal_width_mm": width_mm,
                     "nominal_height_mm": round(height_mm, 2),
                     "formats": "PDF/SVG vector; PNG 300 dpi",
                     "font": "DejaVu Sans; PDF embedded TrueType; SVG editable text",
                     "evidence": "synthetic closed-loop simulation, not FAST-LIO/NDT execution"})


def turn_spans(d):
    active = np.abs(d["k"]) > .15
    boundaries = np.diff(np.r_[False, active, False].astype(int))
    return [(d["s"][i], d["s"][min(j, len(active)-1)])
            for i, j in zip(np.flatnonzero(boundaries == 1),
                            np.flatnonzero(boundaries == -1))]


def representative(trials, output):
    """Selection rule is fixed seed zero, for every geometry and condition."""
    for scenario in SCENARIOS:
        for condition in CONDITIONS:
            dd = choose(trials, scenario, condition, seed=0)
            fig, ax = plt.subplots(2, 3, figsize=(178*MM, 111*MM))
            p = dd[0]["path"]
            ax[0, 0].plot(p[:, 0], p[:, 1], "--", color=".15", lw=.95)
            for d in dd:
                m = d["meta"]["method"]; c = COLORS[m]
                ax[0, 0].plot(d["x"], d["y"], color=c)
                ax[0, 1].plot(d["s"], 1000*d["e"], color=c)
                ax[0, 2].plot(d["s"], np.rad2deg(d["e_yaw"]), color=c)
                ax[1, 0].plot(d["t"], d["v"], color=c)
                ax[1, 0].plot(d["t"]-.05, d["cmd_v"], color=c, ls=":", alpha=.7, lw=.8)
                ax[1, 1].plot(d["t"], d["w"], color=c)
                ax[1, 1].plot(d["t"]-.05, d["cmd_w"], color=c, ls=":", alpha=.7, lw=.8)
            ref = dd[0]
            ax[1, 2].plot(ref["s"], ref["k"], color=".15")
            for a in (ax[0, 1], ax[0, 2], ax[1, 2]):
                for lo, hi in turn_spans(ref):
                    a.axvspan(lo, hi, color=".85", alpha=.3, lw=0)
                if a is not ax[1, 2]:
                    a.axhline(0, color=".5", lw=.5)
            ax[0, 0].set(xlabel="x (m)", ylabel="y (m)")
            ax[0, 0].set_aspect("equal", adjustable="datalim")
            ax[0, 1].set(xlabel="Path progress (m)", ylabel="Lateral error (mm)")
            ax[0, 2].set(xlabel="Path progress (m)", ylabel="Heading error (deg)")
            ax[1, 0].set(xlabel="Time (s)", ylabel="Linear speed (m/s)")
            ax[1, 1].set(xlabel="Time (s)", ylabel="Angular speed (rad/s)")
            ax[1, 2].set(xlabel="Path progress (m)", ylabel="Reference curvature (1/m)")
            ax[1, 0].text(.97, .04, "solid: body\ndotted: command", transform=ax[1, 0].transAxes,
                          ha="right", va="bottom", fontsize=6.2)
            decorate(ax); panel_letters(ax); method_legend(fig)
            fig.subplots_adjust(left=.09, right=.99, bottom=.11, top=.86, wspace=.48, hspace=.48)
            title = f"{SCENE_LABEL[scenario]}, {COND_LABEL[condition]}, seed 0"
            fig.text(.5, .905, title, ha="center", fontsize=8)
            save(fig, output, f"fig01_{scenario}_{condition}", title,
                 f"Representative {scenario.replace('_',' ')} tracking under the {condition} plant. "
                 "Seed 0 was selected in advance for every geometry/condition, not by performance. "
                 "(a) Planar path; (b,c) signed true-state lateral/heading errors versus projected progress; "
                 "(d,e) body (solid) and issued command (dotted) velocities; (f) local reference curvature. "
                 "Shading denotes |curvature|>0.15 1/m, not obstacle regions. Commands are plotted at t−0.05 s "
                 "because stored body states are recorded after each integration step. No smoothing/interpolation is applied.",
                 f"{SCENE_LABEL[scenario]}、{COND_LABEL[condition]}下预先固定种子0的代表曲线：平面轨迹、随投影里程变化的有符号横向/航向误差、线/角速度及参考曲率。"
                 "实线为车体速度，点线为指令；指令按实际发出时刻t−0.05 s绘制。灰带为|曲率|>0.15 m⁻¹。全部原始采样点连线，无滤波，非按最优结果选图。",
                 [d["source"] for d in dd],
                 "SI→mm/deg unit conversion; command timestamps shifted by −dt; connected raw samples only.",
                 "One closed-loop trial per method; seed 0; no error bars")


def trial_metric(d, metric):
    if metric == "turn_rmse_mm":
        return d["meta"]["turn"]["rmse_m"]*1000
    if metric == "turn_heading_deg":
        return np.rad2deg(d["meta"]["turn"]["heading_rmse_rad"])
    dt = .05
    if metric.startswith("cmd_w"):
        delta = np.diff(d["cmd_w"])
    elif metric.startswith("cmd_v"):
        delta = np.diff(d["cmd_v"])
    elif metric.startswith("body_w"):
        delta = np.diff(d["w"], n=2) / dt**2
    elif metric.startswith("body_v"):
        delta = np.diff(d["v"], n=2) / dt**2
    else:
        raise ValueError(metric)
    if "rate" in metric:
        delta = delta / dt
    return float(np.sqrt(np.mean(delta**2)))


def trial_dots(ax, trials, metric, condition, scenario=None):
    for i, m in enumerate(METHODS):
        dd = choose(trials, scenario, condition, m)
        dd.sort(key=lambda d: (d["meta"]["scenario"], d["meta"]["seed"]))
        values = np.array([trial_metric(d, metric) for d in dd])
        jitter = np.linspace(-.13, .13, len(values))
        ax.scatter(i+jitter, values, s=15, color=COLORS[m], alpha=.8, linewidths=.4,
                   edgecolors="white", zorder=3)
        ax.plot([i-.19, i+.19], [np.mean(values)]*2, color=".1", lw=1.4, zorder=4)
    ax.set_xticks(range(4))
    ax.set_xticklabels([SHORT_LABEL[m] for m in METHODS])
    ax.set_ylim(bottom=0)


def trial_distributions(trials, output):
    for metric, stem, ylabel, desc in (
        ("turn_rmse_mm", "fig02_turn_lateral_rmse", "Turn lateral RMSE (mm)", "lateral"),
        ("turn_heading_deg", "fig03_turn_heading_rmse", "Turn heading RMSE (deg)", "heading"),
    ):
        fig, ax = plt.subplots(2, 3, figsize=(178*MM, 100*MM), sharey="row")
        for row, condition in enumerate(CONDITIONS):
            for col, scenario in enumerate(SCENARIOS):
                a = ax[row, col]
                trial_dots(a, trials, metric, condition, scenario)
                a.set_title(f"{SCENE_LABEL[scenario]} · {condition}")
                if col == 0:
                    a.set_ylabel(ylabel)
            upper=max(trial_metric(d,metric) for d in choose(trials,condition=condition))*1.12
            ax[row,0].set_ylim(0,upper)
        decorate(ax); panel_letters(ax)
        fig.subplots_adjust(left=.09, right=.99, bottom=.1, top=.92, wspace=.25, hspace=.52)
        save(fig, output, stem, f"Trial-level turn {desc} errors",
             f"Turn {desc} RMSE for all 72 tracking trials. Each dot is one complete trial's turn-only RMSE; "
             "each geometry/condition/method has three actual seeds. Black bars denote arithmetic means, not confidence intervals. "
             "The same pre-generated state perturbations are paired by geometry/condition/seed. "
             "Samples within trajectories are not independent statistical replicates. All four controllers and both plants are retained.",
             f"全部72次跟踪实验的弯道{desc}均方根误差；每点为一轮试验，每组3个种子，黑短横线为均值而非置信区间。"
             "保留全部4种控制器及2种执行条件；同场景/条件/种子使用相同预生成状态扰动，轨迹中的时间点不视作独立重复。",
             [d["source"] for d in trials], "Turn mask |k|>0.15 1/m; one RMSE per trial; descriptive mean.",
             "72 trials; n=3 per geometry×condition×method")


def paired_turn_changes(trials,output):
    fig,ax=plt.subplots(2,3,figsize=(178*MM,100*MM),sharey="row")
    for row,condition in enumerate(CONDITIONS):
        for col,baseline in enumerate(("native","tracker","ideal")):
            a=ax[row,col]
            for i,scenario in enumerate(SCENARIOS):
                changes=[]
                for seed in (0,1,2):
                    full=choose(trials,scenario,condition,"full",seed)[0]
                    base=choose(trials,scenario,condition,baseline,seed)[0]
                    changes.append(trial_metric(full,"turn_rmse_mm")-trial_metric(base,"turn_rmse_mm"))
                a.scatter(i+np.linspace(-.12,.12,3),changes,color=COLORS[baseline],s=18,alpha=.85)
                a.plot([i-.19,i+.19],[np.mean(changes)]*2,color=".15",lw=1.3)
            a.axhline(0,color=".3",ls="--",lw=.7)
            a.set_xticks(range(3));a.set_xticklabels(["Left","Right","S bend"])
            a.set_title(f"{condition}: Guided − "+SHORT_LABEL[baseline],fontsize=7.3)
            if col==0:a.set_ylabel("Paired turn RMSE change (mm)")
    decorate(ax);panel_letters(ax)
    fig.subplots_adjust(left=.1,right=.99,bottom=.1,top=.92,wspace=.25,hspace=.55)
    save(fig,output,"fig02b_paired_turn_changes","Paired change in turn lateral RMSE",
         "Paired turn lateral RMSE changes: Guided MPC minus each comparison controller using matched geometry, plant, and seed. "
         "Each dot is one pair and each black bar the mean of three pairs within a geometry; negative values favor Guided MPC. "
         "All 54 paired comparisons (18 pairs against each comparator) are retained, including nominal cases where the tracker is more accurate. "
         "Pairing shares pre-generated state perturbations but does not make the native implementation bitwise deterministic. No inferential p-values or confidence intervals are shown.",
         "",[d["source"] for d in trials],"Subtract whole-trial turn RMSE values only after pairing geometry/condition/seed; mean of three pairs.",
         "54 matched comparisons; three pairs per geometry×condition×comparator")


def phase_figure(trials, output):
    phases = ("line", "entry", "apex", "exit")
    fig, ax = plt.subplots(2, 2, figsize=(178*MM, 104*MM))
    for row, condition in enumerate(CONDITIONS):
        for col, field in enumerate(("rmse_m", "heading_rmse_rad")):
            for j, m in enumerate(METHODS):
                dd = choose(trials, condition=condition, method=m)
                means, low, high = [], [], []
                for phase in phases:
                    v = np.array([d["meta"]["phases"][phase][field] for d in dd], float)
                    v *= 1000 if col == 0 else 180/np.pi
                    means.append(np.mean(v)); low.append(np.min(v)); high.append(np.max(v))
                x = np.arange(4)+(j-1.5)*.14
                a = ax[row, col]
                a.errorbar(x, means, yerr=[np.array(means)-low, np.array(high)-means],
                           fmt="o-", color=COLORS[m], ms=3, lw=.9, capsize=2, elinewidth=.6)
            ax[row, col].set_xticks(range(4)); ax[row, col].set_xticklabels(["Straight", "Entry", "Apex", "Exit"])
            ax[row, col].set_ylabel("Lateral RMSE (mm)" if col == 0 else "Heading RMSE (deg)")
            ax[row, col].set_title(COND_LABEL[condition]); ax[row, col].set_ylim(bottom=0)
    decorate(ax); panel_letters(ax); method_legend(fig)
    fig.subplots_adjust(left=.09, right=.99, bottom=.1, top=.86, wspace=.3, hspace=.5)
    save(fig, output, "fig04_phase_errors", "Tracking errors by turn phase",
         "Tracking error by geometric phase. Markers are the mean of nine trial-level RMSE values per method/plant "
         "(three geometries×three seeds); whiskers show the observed minimum–maximum, not uncertainty intervals. "
         "Turn: |κ|>0.15 1/m; entry/exit: sign(κ)dκ/ds >0.4 / <−0.4 1/m²; remaining turn samples form the apex. "
         "The finite path geometry and existing benchmark phase labels are retained.",
         "按几何分段的误差：每点为3种路径×3个种子的9轮RMSE均值，竖线为实际最小至最大范围，非置信区间。"
         "弯道判据|κ|>0.15 m⁻¹；入/出弯判据sign(κ)dκ/ds>0.4/<−0.4 m⁻²；其余弯道为弯中。",
         [d["source"] for d in trials], "Reuse benchmark geometric phase masks; per-trial RMSE then mean/min/max across trials.",
         "n=9 trial summaries per condition×method×phase; geometries pooled descriptively")


def smoothness(trials, output):
    for channel, unit, angular in (("w", "rad", True), ("v", "m", False)):
        metrics = ((f"cmd_{channel}_delta", f"Command Δ{channel} RMS ({unit}/s)"),
                   (f"cmd_{channel}_rate", f"Command rate RMS ({unit}/s²)"),
                   (f"body_{channel}_jerk", f"Body {'angular ' if angular else 'speed '}jerk RMS ({unit}/s³)"))
        fig, ax = plt.subplots(2, 3, figsize=(178*MM, 102*MM))
        for row, condition in enumerate(CONDITIONS):
            for col, (metric, ylabel) in enumerate(metrics):
                trial_dots(ax[row, col], trials, metric, condition)
                ax[row, col].set_ylabel(ylabel); ax[row, col].set_title(COND_LABEL[condition])
        decorate(ax); panel_letters(ax)
        fig.subplots_adjust(left=.1, right=.99, bottom=.1, top=.91, wspace=.49, hspace=.57)
        kind = "angular" if angular else "longitudinal"
        save(fig, output, f"fig05_{kind}_smoothness", f"{kind.capitalize()} command/body smoothness",
             f"{kind.capitalize()} smoothness for all tracking trials. Each dot is a whole-trial statistic; black bars are means "
             "of nine trials per method/plant. Columns distinguish raw command-increment RMS (Δu), command-rate RMS "
             "(Δu/Δt), and body-velocity second-difference RMS (Δ²v/Δt², a numerical jerk estimate). "
             "Δt=0.05 s; differences use recorded samples, no filtering and no artificial prepended zero; startup is included. "
             "Command increment has velocity units and is not acceleration. Body jerk is a simulated finite-difference diagnostic, not measured mechanical jerk.",
             f"全部跟踪试验的{'角' if angular else '线'}速度平滑性：依次给出指令增量RMS（速度单位）、指令变化率RMS、"
             "车体速度二阶差分RMS（数值jerk）。Δt=0.05 s，无滤波、不额外补零，包含启动段；每点一轮、每组9轮，黑横线为均值。"
             "数值jerk来自仿真车体速度，不能直接当作实测机械冲击。",
             [d["source"] for d in trials], "First/second finite differences at 20 Hz; whole-trial RMS; no smoothing/prepended values.",
             "n=9 whole-trial statistics per condition×method")


def turn_smoothness(data_directory, output):
    source=data_directory/"trial_metrics.csv"
    if not source.exists():return
    with source.open(newline="") as stream:
        rows=[r for r in csv.DictReader(stream) if r["dataset"]=="tracking_final" and r["phase"]=="turn"]
    if not rows:return
    for channel,unit in (("w","rad"),("v","m")):
        metrics=((f"cmd_{channel}_delta_rms_{'radps' if channel=='w' else 'mps'}",f"Turn command Δ{channel} RMS ({unit}/s)"),
                 (f"cmd_{channel}_rate_rms_{'radps2' if channel=='w' else 'mps2'}",f"Turn command rate RMS ({unit}/s²)"),
                 (f"body_{channel}_jerk_rms_{'radps3' if channel=='w' else 'mps3'}",f"Turn {'angular' if channel=='w' else 'speed'} jerk RMS ({unit}/s³)"))
        fig,ax=plt.subplots(2,3,figsize=(178*MM,102*MM))
        for row,condition in enumerate(CONDITIONS):
            for col,(metric,ylabel) in enumerate(metrics):
                a=ax[row,col]
                for i,m in enumerate(METHODS):
                    rr=sorted([r for r in rows if r["condition"]==condition and r["method"]==m],key=lambda r:(r["scenario"],int(r["seed"])))
                    values=np.array([float(r[metric]) for r in rr])
                    a.scatter(i+np.linspace(-.13,.13,len(values)),values,color=COLORS[m],s=15,alpha=.8,linewidths=.4,edgecolors="white")
                    a.plot([i-.19,i+.19],[np.mean(values)]*2,color=".1",lw=1.4)
                a.set_xticks(range(4));a.set_xticklabels([SHORT_LABEL[m] for m in METHODS])
                a.set(ylabel=ylabel,title=COND_LABEL[condition]);a.set_ylim(bottom=0)
        decorate(ax);panel_letters(ax)
        fig.subplots_adjust(left=.1,right=.99,bottom=.1,top=.91,wspace=.49,hspace=.57)
        kind="angular" if channel=="w" else "longitudinal"
        save(fig,output,f"fig05b_turn_{kind}_smoothness",f"Turn-only {kind} smoothness",
             f"Turn-only {kind} smoothness, complementary to the whole-trial smoothness plots. "
             "Each dot is one trial's statistic on samples with |κ|>0.15 1/m and each black bar the mean of nine trials per method/plant. "
             "Columns distinguish command increment, command rate, and numerical body jerk with different physical units. "
             "Finite differences are formed on the original regularly sampled trajectory before phase restriction, using only the valid "
             "within-phase intervals specified in the metric exporter, rather than differencing across disjoint turn segments. "
             "No filtering is applied. The accuracy gain must be read together with its command/body smoothness tradeoff.",
             "",[str(source.relative_to(HERE))],"Use exported turn-only finite-difference RMS metrics; descriptive mean; no smoothing.",
             "72 trial-level turn summaries; n=9 per method×condition")


def planar_smoothness(data_directory,output):
    source=data_directory/"trial_metrics.csv"
    if not source.exists():return
    with source.open(newline="") as stream:
        rows=[r for r in csv.DictReader(stream) if r["dataset"]=="tracking_final" and r["phase"]=="turn"]
    fields=(("world_accel_norm_rms_mps2","Planar acceleration norm RMS (m/s²)"),
            ("world_jerk_norm_rms_mps3","Planar jerk norm RMS (m/s³)"))
    if not rows or not all(f in rows[0] for f,_ in fields):return
    fig,ax=plt.subplots(2,2,figsize=(178*MM,101*MM))
    for row,condition in enumerate(CONDITIONS):
        for col,(field,ylabel) in enumerate(fields):
            a=ax[row,col]
            for i,m in enumerate(METHODS):
                rr=sorted([r for r in rows if r["condition"]==condition and r["method"]==m],key=lambda r:(r["scenario"],int(r["seed"])))
                values=np.array([float(r[field]) for r in rr])
                a.scatter(i+np.linspace(-.13,.13,len(values)),values,color=COLORS[m],s=15,alpha=.8)
                a.plot([i-.19,i+.19],[np.mean(values)]*2,color=".1",lw=1.4)
            a.set_xticks(range(4));a.set_xticklabels([SHORT_LABEL[m] for m in METHODS])
            a.set(ylabel=ylabel,title=COND_LABEL[condition]);a.set_ylim(bottom=0)
    decorate(ax);panel_letters(ax)
    fig.subplots_adjust(left=.11,right=.99,bottom=.11,top=.92,wspace=.35,hspace=.55)
    save(fig,output,"fig05c_turn_planar_smoothness","Turn-only planar motion smoothness",
         "Turn-only planar motion smoothness from the simulated world-frame velocity vector [v cos(yaw), v sin(yaw)]. "
         "The first and second chronological finite differences estimate planar acceleration and jerk, respectively; their Euclidean-norm "
         "RMS values include directional changes and centripetal effects omitted by a scalar-speed derivative. "
         "Only consecutive valid within-turn samples enter each derivative. Each dot is one trial and black bars show means over nine "
         "trials per method/plant. These are finite-difference simulation diagnostics, not measured ride-comfort or hardware load tests.",
         "",[str(source.relative_to(HERE))],"Use exported world-frame vector finite-difference norm RMS on valid within-turn samples; no filter.",
         "72 trial-level turn summaries; n=9 per method×condition")


def runtime(trials, output):
    fig, ax = plt.subplots(2, 2, figsize=(178*MM, 101*MM))
    for col, condition in enumerate(CONDITIONS):
        for i, m in enumerate(METHODS):
            dd = choose(trials, condition=condition, method=m)
            values = np.sort(np.concatenate([d["compute_s"]*1000 for d in dd]))
            ax[0, col].plot(values, np.arange(1, len(values)+1)/len(values), color=COLORS[m])
            p95 = [np.percentile(d["compute_s"]*1000, 95) for d in dd]
            ax[1, col].scatter(i+np.linspace(-.13,.13,len(p95)), p95, color=COLORS[m], s=15, alpha=.8)
            ax[1, col].plot([i-.19,i+.19], [np.mean(p95)]*2, color=".1", lw=1.4)
        ax[0, col].axvline(50, color=".2", ls="--", lw=.8)
        ax[0, col].set(xlabel="Controller-call time (ms)", ylabel="Empirical CDF", ylim=(0,1.02),
                       title=COND_LABEL[condition])
        ax[0, col].set_xlim(left=0)
        ax[1, col].axhline(50, color=".2", ls="--", lw=.8)
        ax[1, col].set_xticks(range(4)); ax[1, col].set_xticklabels([SHORT_LABEL[m] for m in METHODS])
        ax[1, col].set_ylabel("Trial call-time P95 (ms)"); ax[1,col].set_ylim(bottom=0)
    decorate(ax); panel_letters(ax); method_legend(fig)
    fig.subplots_adjust(left=.09,right=.99,bottom=.1,top=.85,wspace=.28,hspace=.54)
    save(fig, output, "fig06_runtime", "Measured offline computation time",
         "Offline computation time: upper panels show descriptive empirical CDFs of recorded controller calls pooled within each method/plant; "
         "lower panels preserve the independent trial unit with nine trial P95 values and their arithmetic mean. "
         "The dashed 50 ms line is the nominal 20 Hz controller period. Calls within one trial are dependent. "
         "Measured wall-clock cost was not fed back as additional plant delay; this is not a real-time scheduling guarantee.",
         "离线计算耗时：上排为同方法/条件全部调用的描述性经验分布，下排为每轮P95（每组9轮）及均值。"
         "虚线50 ms对应20 Hz周期。单轮内调用存在时间相关性；计算耗时未回灌到被控对象，不能证明实车实时调度满足性。",
         [d["source"] for d in trials], "Seconds→milliseconds; raw-call ECDF; per-trial P95; no runtime trimming.",
         "Raw calls for descriptive ECDF; n=9 trial P95 values per condition×method")


def corr_curve(t, reference, response_t, response, max_lag=1.):
    lags = np.arange(-round(max_lag/.05), round(max_lag/.05)+1)*.05
    values = []
    for lag in lags:
        target = t + lag
        valid = (target >= response_t[0]) & (target <= response_t[-1])
        a = reference[valid]
        b = np.interp(target[valid], response_t, response)
        if len(a) < 10 or np.std(a) < 1e-9 or np.std(b) < 1e-9:
            values.append(np.nan)
        else:
            values.append(float(np.corrcoef(a,b)[0,1]))
    return lags, np.asarray(values)


def lag_diagnostics(trials, output):
    for condition in CONDITIONS:
        dd = choose(trials, "s_bend", condition, seed=0)
        fig, ax = plt.subplots(2, 4, figsize=(178*MM, 106*MM))
        for col, m in enumerate(METHODS):
            d = next(d for d in dd if d["meta"]["method"] == m)
            t = d["t"]; ref = .2*d["k"]
            ax[0,col].plot(t, ref, color=".15", ls="--", lw=.9)
            ax[0,col].plot(t-.05, d["cmd_w"], color=COLORS[m], ls=":", lw=.9)
            ax[0,col].plot(t, d["w"], color=COLORS[m], lw=1.1)
            for response_t, response, linestyle in ((t-.05,d["cmd_w"],":"),(t,d["w"],"-")):
                lag, cc = corr_curve(t,ref,response_t,response)
                ax[1,col].plot(lag,cc,color=COLORS[m],ls=linestyle)
            ax[0,col].set_title({"native":"Native MPPI","tracker":"Preview tracker","ideal":"Ideal ablation","full":"Guided MPC"}[m], fontsize=7)
            ax[0,col].set_xlabel("Time (s)")
            ax[1,col].set_xlabel("Response lag (s)")
            ax[1,col].axvline(0,color=".5",lw=.5)
            ax[1,col].set_ylim(-1.02,1.02)
        ax[0,0].set_ylabel("Angular speed (rad/s)")
        ax[1,0].set_ylabel("Normalized correlation")
        handles = [Line2D([],[],color=".15",ls="--",label="Reference 0.20κ"),
                   Line2D([],[],color=".35",ls=":",label="Issued command"),
                   Line2D([],[],color=".35",ls="-",label="Body response")]
        fig.legend(handles=handles,ncol=3,loc="upper center",bbox_to_anchor=(.5,.995),frameon=False)
        decorate(ax); panel_letters(ax)
        fig.subplots_adjust(left=.09,right=.99,bottom=.11,top=.85,wspace=.43,hspace=.57)
        save(fig, output, f"fig07_lag_{condition}", f"Angular response/lag association, {condition}",
             f"Seed-0 S-bend angular-response diagnostic under the {condition} plant. Upper row compares target-speed reference "
             "0.20κ(s(t)), issued angular command, and simulated body response. Lower row gives overlap-normalized, demeaned "
             "Pearson correlations corr(reference(t), response(t+lag)) over −1 to +1 s on a 0.05 s grid. "
             "Command is timestamped at t−0.05 s; response interpolation is linear solely for lag alignment. "
             "Positive lag means response trails the reference. Correlation includes closed-loop coupling and path-progress dependence; "
             "it is an association diagnostic, not an identified physical delay or causal measurement.",
             f"{condition}条件、固定种子0的S弯角速度响应及相关迟滞诊断。参考为0.20κ(s(t))，点线为发出指令、实线为车体响应；"
             "下排为−1至+1 s范围内每0.05 s的重叠归一化去均值Pearson相关。正迟滞表示响应落后；仅对齐时线性插值。"
             "闭环耦合及里程进度会影响相关峰值，因此不能当作物理延迟辨识或因果证明。",
             [d["source"] for d in dd], "Command timestamp correction; target reference 0.20κ; per-lag overlap Pearson correlation; linear interpolation for alignment only.",
             "One preselected S-bend trial per method; dependent time samples; no confidence intervals")


def historical(directory, output):
    sources = sorted(directory.glob("*.npz"))
    if not sources:
        return
    dd = [load_trace(p) for p in sources]
    fig, ax = plt.subplots(2, 2, figsize=(178*MM, 106*MM))
    p = dd[0]["path"]
    ax[0,0].plot(p[:,0],p[:,1],color=".2",ls="--",lw=.8)
    for d in dd:
        m = "native" if "_native_" in d["source"] else "full"
        ax[0,0].plot(d["x"],d["y"],color=COLORS[m],label=LABEL[m])
        ax[0,1].plot(d["s"],d["e"]*1000,color=COLORS[m])
        ax[1,0].plot(d["s"],np.rad2deg(d["e_yaw"]),color=COLORS[m])
        ax[1,1].plot(d["t"],d["w"],color=COLORS[m])
    ax[0,0].set(xlabel="x (m)",ylabel="y (m)");ax[0,0].set_aspect("equal",adjustable="datalim")
    ax[0,0].legend(frameon=False)
    ax[0,1].set(xlabel="Path progress (m)",ylabel="Lateral error (mm)")
    ax[1,0].set(xlabel="Path progress (m)",ylabel="Heading error (deg)")
    ax[1,1].set(xlabel="Time (s)",ylabel="Body angular speed (rad/s)")
    decorate(ax);panel_letters(ax)
    fig.subplots_adjust(left=.09,right=.99,bottom=.1,top=.94,wspace=.3,hspace=.5)
    save(fig,output,"fig08_historical_route","Frozen historical reference route: synthetic replay",
         "Supplementary synthetic closed-loop replay of the frozen historical 47.7 m inspection reference. "
         "Only Native MPPI and Guided MPC were run, once each, in the nominal synthetic plant. "
         "The reference point locations were reused; FAST-LIO and NDT were not executed and these are not historical measured robot traces. "
         "This single-route check is not pooled into the 72-trial aggregate statistics. No smoothing is applied.",
         "冻结旧47.7 m巡检参考路径的补充闭环仿真：原MPPI与完整方法在名义模型下各1轮。"
         "仅复用旧路径点，未运行FAST-LIO/NDT，曲线并非历史实车轨迹；不并入72轮主实验统计，无滤波。",
         [d["source"] for d in dd],"SI→mm/deg; raw sample connection; reference geometry reused unchanged.",
         "One synthetic trial for each of two methods; one historical route")


def convergence(supplement, output):
    sources = sorted(supplement.glob("convergence_*.npz"))
    if not sources:
        return
    dd = []
    for source in sources:
        d = load_trace(source)
        condition, method, seed = source.stem.removeprefix("convergence_").split("_")
        d.update(condition=condition,method=method,seed=int(seed))
        dd.append(d)
    fig, ax = plt.subplots(2,2,figsize=(178*MM,107*MM))
    for row,condition in enumerate(CONDITIONS):
        for col,(field,scale,threshold,ylabel) in enumerate((("e",1000,10,"Absolute lateral error (mm)"),
                                                         ("e_yaw",180/np.pi,2,"Absolute heading error (deg)"))):
            for m in METHODS:
                ds=[d for d in dd if d["condition"]==condition and d["method"]==m]
                # All seeds are shown individually; no time samples treated as replicates.
                for d in ds:
                    ax[row,col].plot(d["t"],abs(d[field])*scale,color=COLORS[m],lw=.65,alpha=.56)
            ax[row,col].axhline(threshold,color=".2",ls="--",lw=.8)
            ax[row,col].set(xlabel="Time since initial offset (s)",ylabel=ylabel,title=COND_LABEL[condition])
            ax[row,col].set_xlim(left=0)
    decorate(ax);panel_letters(ax);method_legend(fig)
    fig.subplots_adjust(left=.1,right=.99,bottom=.11,top=.85,wspace=.3,hspace=.53)
    save(fig,output,"fig09_initial_offset_convergence","Recovery from initial tracking offsets",
         "Initial-offset recovery in the left-turn supplementary test (initial lateral offset 0.10 m and heading offset +10 deg). "
         "All three seed traces per method/plant are overlaid, without temporal smoothing or pointwise confidence bands. "
         "Dashed lines show the predeclared 10 mm lateral and 2 deg heading bands; convergence requires joint continuous residence "
         "in both bands for the declared hold duration, not merely one threshold crossing. This empirical finite-run recovery is not a stability theorem.",
         "直线路径初始偏差恢复试验：横向偏移0.10 m、航向+10°，每方法/条件的3个种子原始曲线全部叠加。"
         "虚线为预先声明的10 mm与2°容差带；收敛要求在两容差带内同时连续保持指定时长，不能只看首次越线；有限时仿真不等于稳定性证明。",
         [d["source"] for d in dd],"Absolute errors, SI→mm/deg; three raw seed traces, no envelope/CI or filter.",
         "24 supplementary closed-loop trials; three seeds per condition×method")
    summary_path=supplement/"convergence_summary.json"
    if not summary_path.exists():
        return
    rows=json.loads(summary_path.read_text())
    if isinstance(rows,dict):
        rows=rows.get("trials",rows.get("results",[]))
    if not rows:
        return
    # Field names are discovered explicitly, avoiding accidental conversion of non-settling runs into zero.
    if not any("joint_settling_first_window_s" in r for r in rows):
        return
    fig,axs=plt.subplots(2,2,figsize=(178*MM,100*MM),sharey=True)
    for row,condition in enumerate(CONDITIONS):
        for col,(field,title) in enumerate((("joint_settling_first_window_s","First 1 s joint-band window"),
                                          ("last_exit_settling_s","Last exit with ≥1 s observed dwell"))):
            a=axs[row,col]
            count_labels=[]
            for i,m in enumerate(METHODS):
                rr=[r for r in rows if r.get("plant",r.get("condition"))==condition and r.get("method")==m]
                nc=sum(r.get(field) is None for r in rr)
                count_labels.append(SHORT_LABEL[m]+f"\nc={nc}/{len(rr)}")
                for j,r in enumerate(rr):
                    value=r.get(field)
                    censored=value is None
                    if censored:
                        value=r["censor_time_s"]
                    a.scatter(i+(j-1)*.11,value,color=COLORS[m],marker="v" if censored else "o",s=24,
                              facecolors="none" if censored else COLORS[m],linewidths=.9)
            a.set_xticks(range(4));a.set_xticklabels(count_labels)
            a.set_title(f"{condition}: {title}",fontsize=7);a.set_ylim(bottom=0)
            if col==0:a.set_ylabel("Joint settling time (s)")
    # Shared axes must include every variable observation horizon, including censoring markers.
    upper=max(float(r["censor_time_s"]) for r in rows)*1.10
    axs[0,0].set_ylim(0,upper)
    decorate(axs);panel_letters(axs)
    fig.subplots_adjust(left=.09,right=.99,bottom=.11,top=.92,wspace=.25,hspace=.58)
    save(fig,output,"fig10_joint_settling","Joint-band settling and right censoring",
         "Joint-band settling times for every left-turn initial-offset recovery trial. Left: first continuous 1 s window within both "
         "10 mm lateral and 2 deg heading bands. Right: time after the last observed threshold exit, accepted only when the terminal "
         "in-band dwell is at least 1 s. Filled circles denote observed events; open downward triangles denote right-censored trials "
         "at their own observation horizon. Horizons vary because trials end at geometric route completion or failure. Censored trials "
         "are neither assigned zero nor excluded. Labels c=n/N report the censored count. Last-exit settling is retrospective over a finite run, not a guarantee of future residence.",
         "每次初始偏差试验同时进入10 mm/2°容差带并保持规定时长的收敛时间。实心圆为达标；空心倒三角为截至观测终点仍未达标的右删失，"
         "不能将其记为0或删除；保持时长定义见补充实验说明。",
         [str(summary_path.relative_to(HERE))],"Summary settling times; censored runs plotted at observation horizon with open triangles.",
         "All 24 recovery attempts, including censored runs",height_mm=100)


def model_responses(supplement,output):
    for kind in ("step","multisine"):
        sources=[supplement/f"openloop_{c}_{kind}.npz" for c in CONDITIONS]
        if not all(p.exists() for p in sources):
            continue
        dd=[load_trace(p) for p in sources]
        fig,ax=plt.subplots(2,2,figsize=(178*MM,101*MM))
        for row,(condition,d) in enumerate(zip(CONDITIONS,dd)):
            for field,label,color,ls in (("cmd_w","Command",".25",":"),
                                         ("smooth_w","30 Hz smoother","#0072B2","-"),
                                         ("delayed_w","Transport output","#009E73","--"),
                                         ("w","Body response","#D55E00","-")):
                ax[row,0].plot(d["t"],d[field],color=color,ls=ls,lw=1,label=label)
            ax[row,0].set(xlabel="Time (s)",ylabel="Angular speed (rad/s)",title=COND_LABEL[condition])
            ax[row,1].plot(d["cmd_w"],d["w"],color="#D55E00",lw=.8)
            # Arrows indicate time direction; they do not imply quasistatic hysteresis.
            for idx in np.linspace(2,len(d["t"])-3,5,dtype=int):
                ax[row,1].annotate("",xy=(d["cmd_w"][idx+1],d["w"][idx+1]),
                                   xytext=(d["cmd_w"][idx-1],d["w"][idx-1]),
                                   arrowprops={"arrowstyle":"->","color":"#D55E00","lw":.7})
            ax[row,1].set(xlabel="Command angular speed (rad/s)",ylabel="Body angular speed (rad/s)",
                          title="Dynamic input–output loop")
        ax[0,0].legend(frameon=False,fontsize=6.2,loc="best")
        decorate(ax);panel_letters(ax)
        fig.subplots_adjust(left=.09,right=.99,bottom=.11,top=.93,wspace=.32,hspace=.52)
        save(fig,output,f"fig11_openloop_{kind}",f"Synthetic execution-chain {kind} response",
             f"Open-loop {kind} response of the same synthetic execution chain. Left: command, 30 Hz rate-limited smoother output, "
             "discrete transport output, and first-order body response for nominal and held-out parameters. Right: time-parametrized "
             "command-versus-body loops with arrows showing temporal direction. A loop caused by dynamics is not evidence of material, "
             "frictional, or rate-independent hysteresis. Inputs/outputs are generated from the assumed model, not identified hardware data.",
             f"同一合成执行链的{kind}开环响应。左列为指令、30 Hz限加速度平滑器、离散传输输出及一阶车体响应；"
             "右列为按时间连线的指令—响应动态输入输出环，箭头表示时间方向。动态环不能证明材料/摩擦或率无关滞回；全部来自假设模型，非实车辨识。",
             [d["source"] for d in dd],"Raw samples connected; input–output time-parametrized loop; no filtering.",
             "One deterministic open-loop input per plant condition")


def frequency_response(data_directory, output):
    source=data_directory/"model_frequency_response.csv"
    if not source.exists():return
    with source.open(newline="") as stream:
        rows=list(csv.DictReader(stream))
    fig,ax=plt.subplots(2,2,figsize=(178*MM,100*MM),sharex="col")
    for col,axis in enumerate(("v","w")):
        for condition,color,ls in (("nominal","#0072B2","-"),("heldout","#D55E00","--")):
            rr=[r for r in rows if r["axis"]==axis and r["condition"]==condition]
            f=np.array([float(r["frequency_Hz"]) for r in rr])
            gain=np.array([float(r["magnitude_dB"]) for r in rr])
            phase=np.array([float(r["unwrapped_phase_deg"]) for r in rr])
            ax[0,col].semilogx(f,gain,color=color,ls=ls,label=COND_LABEL[condition])
            ax[1,col].semilogx(f,phase,color=color,ls=ls)
        ax[0,col].set_title("Linear channel" if axis=="v" else "Angular channel")
        ax[0,col].set_ylabel("Magnitude (dB)")
        ax[1,col].set(xlabel="Frequency (Hz)",ylabel="Unwrapped phase (deg)")
        ax[0,col].legend(frameon=False)
    for a in ax.ravel():
        a.grid(True,which="both")
        a.yaxis.set_major_locator(MaxNLocator(5))
    panel_letters(ax)
    fig.subplots_adjust(left=.1,right=.99,bottom=.11,top=.93,wspace=.34,hspace=.46)
    save(fig,output,"fig12_assumed_frequency_response","Assumed linear-channel frequency response",
         "Analytical frequency responses of the assumed transport-delay plus first-order velocity channels for nominal and held-out "
         "parameters. The 30 Hz nonlinear rate-limited smoother, feedback, sampling residuals, and command constraints are excluded. "
         "These curves characterize an assumed channel, not a measured robot frequency response or the closed-loop controller bandwidth.",
         "",[str(source.relative_to(HERE))],"Analytic frequency-response CSV; logarithmic frequency axis; unwrapped phase.",
         "Deterministic analytical model, no statistical replicates")


def prediction_validation(supplement,output):
    sources=[supplement/f"noisy_model_prediction_{c}.npz" for c in CONDITIONS]
    if not all(p.exists() for p in sources):return
    dd=[load_trace(p) for p in sources]
    # Supplement schema permits explicit noisy measurement names. Never invent a prediction channel.
    prediction_fields=(("v_fit_prediction","w_fit_prediction"),
                       ("v_prediction","w_prediction"))
    fields=next((pair for pair in prediction_fields if all(f in dd[0] for f in pair)),None)
    if fields is None:return
    fig,ax=plt.subplots(2,2,figsize=(178*MM,102*MM))
    for row,(condition,d) in enumerate(zip(CONDITIONS,dd)):
        for col,axis in enumerate(("v","w")):
            true_field=next((f for f in (f"{axis}_true",f"{axis}_latent",axis) if f in d),None)
            measurement=next((f for f in (f"{axis}_measured",f"{axis}_observed",f"{axis}_noisy",f"measured_{axis}") if f in d),None)
            if measurement:
                ax[row,col].plot(d["t"],d[measurement],color=".65",lw=.55,label="Noisy observation")
            if true_field:
                ax[row,col].plot(d["t"],d[true_field],color=".15",ls="--",lw=.9,label="Synthetic truth")
            ax[row,col].plot(d["t"],d[fields[col]],color="#D55E00",lw=1,label="Fitted-model prediction")
            ax[row,col].set(xlabel="Validation time (s)",ylabel="Linear speed (m/s)" if col==0 else "Angular speed (rad/s)",
                            title=COND_LABEL[condition])
    ax[0,0].legend(frameon=False,fontsize=6)
    decorate(ax);panel_letters(ax)
    fig.subplots_adjust(left=.09,right=.99,bottom=.11,top=.93,wspace=.3,hspace=.53)
    save(fig,output,"fig13_noisy_model_validation","Synthetic noisy-output model validation",
         "Independent multisine validation of velocity-channel models fitted to the synthetic step experiment with added observation noise. "
         "The observation, unobserved simulation truth, and recursive fitted-model prediction are shown separately. "
         "Training and validation noise are independent. This establishes a synthetic identification procedure, not identification of the actual robot; "
         "refer to model_validation.json for assumptions, estimated parameters, and error values. No display filtering is applied.",
         "",[d["source"] for d in dd],"Raw noisy measurements, synthetic truth, and recursive model predictions; no display filtering.",
         "One synthetic step fit and one independent multisine validation per plant")


def receding_diagnostics(supplement,output):
    sources=sorted(supplement.glob("convergence_*.npz"))
    dd=[]
    for source in sources:
        d=load_trace(source)
        if "diagnostics_available" not in d:continue
        condition,method,seed=source.stem.removeprefix("convergence_").split("_")
        if method not in ("ideal","full"):continue
        d.update(condition=condition,method=method,seed=int(seed));dd.append(d)
    if not dd:return
    fig,ax=plt.subplots(2,2,figsize=(178*MM,102*MM))
    for row,condition in enumerate(CONDITIONS):
        for d in dd:
            if d["condition"]!=condition:continue
            valid=d["diagnostics_available"]>.5
            if "command_valid" in d:valid &= d["command_valid"]>.5
            c=COLORS[d["method"]]
            ax[row,0].plot(d["t"][valid],d["effective_samples"][valid]/1000,color=c,lw=.6,alpha=.6)
            ax[row,1].plot(d["t"][valid],d["min_sample_cost"][valid],color=c,ls=":",lw=.6,alpha=.6)
            ax[row,1].plot(d["t"][valid],d["selected_cost"][valid],color=c,lw=.6,alpha=.6)
        ax[row,0].set(xlabel="Time (s)",ylabel="Effective sample fraction",ylim=(0,1.03),title=COND_LABEL[condition])
        ax[row,1].set(xlabel="Time (s)",ylabel="Receding-horizon cost (a.u.)",title=COND_LABEL[condition])
    handles=[Line2D([],[],color=COLORS[m],label=LABEL[m]) for m in ("ideal","full")]
    handles +=[Line2D([],[],color=".35",ls=":",label="Min. sampled cost"),
               Line2D([],[],color=".35",ls="-",label="Selected sequence cost")]
    fig.legend(handles=handles,ncol=2,loc="upper center",bbox_to_anchor=(.5,1.0),frameon=False)
    decorate(ax);panel_letters(ax)
    fig.subplots_adjust(left=.1,right=.99,bottom=.1,top=.82,wspace=.28,hspace=.55)
    save(fig,output,"fig14_receding_horizon_diagnostics","Sampling diagnostics during offset recovery",
         "Available sampling diagnostics for both residual methods during all initial-offset recovery runs (three seeds per plant/method). "
         "Left: effective sample size divided by the 1000 sampled sequences. Right: minimum sampled cost (dotted) and selected sequence cost (solid). "
         "Only observations with diagnostics_available=1 and command_valid=1 are plotted; zero placeholders for unavailable native/tracker diagnostics are excluded. "
         "The horizon and measured state change every call, so the cost sequence is a receding-horizon diagnostic, not optimizer convergence across iterations "
         "and not an inter-method optimality test. Costs are in arbitrary units and no temporal smoothing is used.",
         "",[d["source"] for d in dd],"Mask unavailable diagnostics/commands; ESS divided by 1000; raw receding-horizon costs, no filter.",
         "12 supplementary residual-controller runs; three seeds per plant/method")


def write_index(output):
    # Remove only these obsolete names emitted by an earlier version of this script.
    for obsolete in ("fig05_linear_smoothness","fig05b_turn_linear_smoothness"):
        for extension in ("pdf","svg","png"):
            path=output/f"{obsolete}.{extension}"
            if path.exists():path.unlink()
    (output/"figure_manifest.json").write_text(json.dumps(REGISTRY,ensure_ascii=False,indent=2)+"\n")
    lines=["# Publication figure index","",
           "All plots are derived from synthetic simulation traces. They are not FAST-LIO/NDT localization or real-robot validation.","",
           "PDF/SVG are vector graphics; PNG files are 300 dpi. Suggested double-column width: 178 mm. "
           "PDF embeds DejaVu Sans TrueType; SVG retains editable text. The PDF/SVG files should be preferred for manuscripts.",""]
    main_ids=["fig01_s_bend_heldout","fig02_turn_lateral_rmse","fig04_phase_errors",
              "fig05b_turn_angular_smoothness","fig05c_turn_planar_smoothness","fig06_runtime",
              "fig09_initial_offset_convergence","fig10_joint_settling","fig11_openloop_step",
              "fig13_noisy_model_validation"]
    available={r["id"] for r in REGISTRY}
    lines += ["## Suggested manuscript placement","",
              "The following selection covers distinct scientific questions; it is not a selection of favorable trials. "
              "The remaining figures and all six fixed-seed representative geometry/condition combinations should remain available as supplementary material.",""]
    for key in main_ids:
        if key in available:
            lines.append(f"- Main text or a combined multipanel figure: [{key}]({key}.pdf).")
    lines += ["", "Suggested supplementary material: paired changes, heading-error distributions, remaining representative trajectories, "
              "whole-trial/longitudinal smoothness, correlation diagnostics, historical-route replay, multisine loops, analytical Bode curves, "
              "and receding-horizon sampling diagnostics. Preserve the smoothness tradeoff when condensing figures.", ""]
    for item in REGISTRY:
        lines += [f"## {item['id']} — {item['title']}","",
                  " · ".join(f"[{Path(p).suffix[1:].upper()}]({p})" for p in item["files"]),"",
                  "**Caption.** "+item["caption_en"],"",
                  "**Sample unit:** "+item["sample_unit"],"",
                  "**Processing:** "+item["manipulations"],"",
                  "**Sources:** "+", ".join("`"+p+"`" for p in item["sources"]),""]
    (output/"INDEX.md").write_text("\n".join(lines)+"\n")
    manifest={"script_sha256":hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              "matplotlib_version":matplotlib.__version__,"numpy_version":np.__version__,
              "figure_count":len(REGISTRY),"files":{}}
    for p in sorted(output.iterdir()):
        if p.suffix in (".pdf",".svg",".png"):
            manifest["files"][p.name]=hashlib.sha256(p.read_bytes()).hexdigest()
    (output/"artifact_hashes.json").write_text(json.dumps(manifest,indent=2)+"\n")


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tracking",type=Path,default=HERE/"results/tracking_final")
    parser.add_argument("--historical",type=Path,default=HERE/"results/historical_route")
    parser.add_argument("--supplement",type=Path,default=HERE/"paper/supplement")
    parser.add_argument("--output",type=Path,default=HERE/"paper/figures")
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    style();trials=tracking_trials(args.tracking)
    representative(trials,args.output)
    trial_distributions(trials,args.output)
    paired_turn_changes(trials,args.output)
    phase_figure(trials,args.output)
    smoothness(trials,args.output)
    turn_smoothness(HERE/"paper/data",args.output)
    planar_smoothness(HERE/"paper/data",args.output)
    runtime(trials,args.output)
    lag_diagnostics(trials,args.output)
    historical(args.historical,args.output)
    convergence(args.supplement,args.output)
    model_responses(args.supplement,args.output)
    frequency_response(HERE/"paper/data",args.output)
    prediction_validation(args.supplement,args.output)
    receding_diagnostics(args.supplement,args.output)
    write_index(args.output)
    print(json.dumps({"figures":len(REGISTRY),"output":str(args.output)},ensure_ascii=False))


if __name__=="__main__":
    main()
