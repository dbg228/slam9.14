"""Welch PSD appendix from all frozen tracking trials; never re-runs control."""
from __future__ import annotations

import argparse
import csv
import hashlib
import itertools
import json
import os
from pathlib import Path
import sys

import numpy as np
from scipy import __version__ as scipy_version
from scipy.signal import welch

os.environ.setdefault("MPLCONFIGDIR", "/tmp/turning_mppi_spectrum_mpl")
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

HERE = Path(__file__).resolve().parent
METHODS = ("native", "tracker", "ideal", "full")
LABELS = dict(native="Native MPPI", tracker="Preview tracker", ideal="Ideal-dynamics ablation", full="Guided MPC")
COLORS = dict(native="#0072B2", tracker="#009E73", ideal="#CC79A7", full="#D55E00")
PARAMETERS = dict(fs=20., window="hann", nperseg=128, noverlap=64, nfft=256,
                  detrend="constant", return_onesided=True, scaling="density", average="mean")
CHANNELS = dict(cmd_w="rad/s", w="rad/s", cmd_v="m/s", v="m/s")


def digest(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def write_csv(path, rows):
    with Path(path).open("w", newline="", encoding="utf-8") as h:
        writer = csv.DictWriter(h, fieldnames=list(rows[0]))
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def spectrum(values):
    values = np.asarray(values, float)
    if len(values)<PARAMETERS["nperseg"] or not np.isfinite(values).all():
        raise ValueError("Welch requires at least 128 finite, consecutive samples; no samples are dropped.")
    frequency, density = welch(values, **PARAMETERS)
    # Explicit trapezoid convention, with an interpolated 2 Hz boundary.
    total = float(np.trapz(density, frequency))
    high_f = np.r_[2., frequency[frequency>2.]]
    high_p = np.interp(high_f, frequency, density)
    high = float(np.trapz(high_p, high_f))
    threshold = 64*np.finfo(float).eps*max(float(np.mean(values**2)), 1e-24)
    valid = np.isfinite(total) and total>threshold
    return frequency, density, dict(total_power=total, power_above_2hz=high,
                                    fraction_above_2hz=high/total if valid else None,
                                    denominator_threshold=threshold,
                                    power_status="valid" if valid else "zero_or_roundoff_power_fraction_undefined")


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--source",type=Path,default=HERE/"results/tracking_final")
    ap.add_argument("--output",type=Path,default=HERE/"paper/spectrum")
    args=ap.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    source_manifest=json.loads((args.source/"manifest.json").read_text())
    a=source_manifest["arguments"]
    expected=list(itertools.product(a["scenarios"],a["plants"],a["methods"],a["seeds"]))
    if len(expected)!=72:
        raise ValueError("This appendix is prescribed for the complete 72-trial frozen tracking matrix.")
    psd_rows, power_rows, inputs, grouped=[],[],[],{}
    for scenario,condition,method,seed in expected:
        trial_id=f"{scenario}_{condition}_{method}_{seed}"
        p=args.source/(trial_id+".npz")
        with np.load(p,allow_pickle=False) as z:
            trace=z["trace"]; columns=list(map(str,z["columns"]))
            if len(z["obstacles"]):
                raise ValueError("Obstacle traces are outside the appendix scope.")
        t=trace[:,columns.index("t")]
        if not np.allclose(np.diff(t),.05,rtol=0,atol=1e-8):
            raise ValueError(f"Irregular samples in {trial_id}; no interpolation/stitching is allowed.")
        meta=dict(trial_id=trial_id,scenario=scenario,condition=condition,method=method,seed=seed)
        inputs.append(dict(**meta,sha256=digest(p),n_samples=len(trace),first_state_t_s=float(t[0]),
                           last_state_t_s=float(t[-1]),observed_span_s=float(t[-1]-t[0])))
        for channel,unit in CHANNELS.items():
            frequency,density,power=spectrum(trace[:,columns.index(channel)])
            n_segments=1+(len(trace)-PARAMETERS["nperseg"])//(PARAMETERS["nperseg"]-PARAMETERS["noverlap"])
            power_rows.append(dict(**meta,channel=channel,input_unit=unit,power_unit=f"({unit})^2",
                                   n_samples=len(trace),n_welch_segments=n_segments,
                                   observed_span_s=float(t[-1]-t[0]),**power))
            psd_rows.extend(dict(**meta,channel=channel,frequency_hz=float(f),density=float(d),
                                 density_unit=f"({unit})^2/Hz") for f,d in zip(frequency,density))
            grouped.setdefault((condition,method,channel),[]).append(density)
    write_csv(args.output/"trial_psd.csv",psd_rows)
    write_csv(args.output/"trial_spectral_power.csv",power_rows)
    mean_rows=[]
    for (condition,method,channel),spectra in sorted(grouped.items()):
        for f,d in zip(frequency,np.mean(spectra,axis=0)):
            mean_rows.append(dict(condition=condition,method=method,channel=channel,n_trials=len(spectra),
                                  frequency_hz=float(f),mean_density=float(d),density_unit=f"({CHANNELS[channel]})^2/Hz"))
    write_csv(args.output/"mean_psd.csv",mean_rows)
    plt.rcParams.update({"font.family":"DejaVu Sans","font.size":8,"axes.titlesize":8,
                         "legend.fontsize":7,"pdf.fonttype":42,"ps.fonttype":42,"svg.fonttype":"none",
                         "axes.spines.top":False,"axes.spines.right":False})
    fig,axes=plt.subplots(2,2,figsize=(178/25.4,122/25.4),sharex=True)
    for row,channel in enumerate(("cmd_w","w")):
        for col,condition in enumerate(("nominal","heldout")):
            ax=axes[row,col]
            for method in METHODS:
                mean=np.mean(grouped[(condition,method,channel)],axis=0)
                # Mask zero PSD values. There is no hidden pseudocount/floor.
                ax.semilogy(frequency,np.ma.masked_less_equal(mean,0),color=COLORS[method],lw=1.1)
            ax.axvline(2.,color=".4",ls=":",lw=.8)
            ax.set_title(("Nominal plant" if condition=="nominal" else "Held-out plant")+
                         (": commanded yaw rate" if channel=="cmd_w" else ": body yaw rate"))
            ax.text(.01,1.04,f"({chr(97+row*2+col)})",transform=ax.transAxes,weight="bold")
            ax.set(xlim=(0,10),ylabel="PSD [(rad/s)²/Hz]")
            ax.grid(alpha=.2,which="both",lw=.4)
            if row==1:
                ax.set_xlabel("Frequency [Hz]")
    fig.legend([Line2D([],[],color=COLORS[m]) for m in METHODS],[LABELS[m] for m in METHODS],
               loc="upper center",ncol=4,frameon=False,bbox_to_anchor=(.5,1.005))
    fig.tight_layout(rect=(0,0,1,.95))
    base="Fig15_welch_spectra"
    for suffix in ("pdf","svg","png"):
        fig.savefig(args.output/(base+"."+suffix),dpi=300,bbox_inches="tight",pad_inches=.04)
    plt.close(fig)
    caption=("Figure 15. Unfiltered whole-trial angular-velocity spectra from all 72 frozen synthetic tracking trials. "
             "Each curve is the descriptive arithmetic mean of nine trial PSDs (three fixed path geometries × three reused noise seeds) "
             "for one method and plant condition; curves are not confidence intervals or nine IID route samples. "
             "Welch parameters: Fs=20 Hz, Hann window, 128 samples per segment, 64-sample overlap, 256-point FFT, "
             "constant detrending per segment, one-sided density scaling, and mean segment averaging. "
             "The dotted line marks 2 Hz. Whole continuous trials include startup and turning transients and have unequal durations; "
             "phases are never concatenated. Nonpositive PSD values are masked on the logarithmic axis without a pseudocount. "
             "The command clock is t−0.05 s and the body clock is t; this constant offset does not change magnitude PSD. "
             "Synthetic localization perturbations enter controller observations, not the saved true body-state trace directly. "
             "No FAST-LIO/NDT execution or real actuator identification is represented.")
    (args.output/"caption_en.txt").write_text(caption+"\n")
    method=dict(parameters=PARAMETERS,fft_bin_spacing_hz=20/256,
                window_unpadded_bin_spacing_hz=20/128,segment_duration_s=128/20,
                resolution_caveat="Zero-padding from 128 to 256 FFT points refines the displayed frequency grid; it does not increase physical resolving power. The 6.4 s Hann window governs spectral resolution.",
                raw_preprocessing="None before Welch; specified segment-wise constant detrending is part of Welch.",
                power_integral="Trapezoid integration over [0,10] Hz; numerator over [2,10] Hz with density linearly interpolated at 2 Hz. A single boundary has zero measure.",
                power_caveat="Windowed segment estimates and omission of an initial sample mean integrated PSD is not asserted to equal whole-trial raw variance exactly.",
                denominator="Fraction is null/empty if integrated power <= 64*float64_epsilon*max(mean(raw_signal^2),1e-24); PSD values themselves are retained unmodified.",
                uncertainty="No confidence bands. Fixed geometries and reused seeds; means are descriptive. Different trace lengths produce different segment counts and transient contributions.",
                units=CHANNELS,caption_en=caption,input_manifest_sha256=digest(args.source/"manifest.json"),
                analysis_sha256=digest(__file__),numpy_version=np.__version__,scipy_version=scipy_version,
                python_version=sys.version,inputs=inputs,
                outputs={p.name:digest(p) for p in sorted(args.output.iterdir()) if p.is_file() and p.name!="manifest.json"})
    (args.output/"manifest.json").write_text(json.dumps(method,indent=2,allow_nan=False)+"\n")
    print(json.dumps(dict(trials=len(inputs),psd_rows=len(psd_rows),power_rows=len(power_rows),mean_psd_rows=len(mean_rows))))


if __name__=="__main__":
    main()
