"""Generate English manuscript results and LaTeX tables from the saved data."""
import argparse
import csv
import json
from pathlib import Path
import textwrap
import numpy as np

HERE=Path(__file__).resolve().parent
METHODS=['native','tracker','ideal','full']
LABELS={'native':'Native MPPI','tracker':'Preview tracker','ideal':'Ideal-dynamics ablation','full':'Guided MPC'}


def read_csv(p):
    with p.open(newline='')as f:return list(csv.DictReader(f))


def number(x):return float(x) if x not in ('',None) else None


def tex_escape(x):
    return (str(x).replace('&',r'\&').replace('%',r'\%').replace('_',r'\_')
            .replace(' ± ',r' $\pm$ ').replace('°',r'$^\circ$')
            .replace('²',r'$^{2}$').replace('³',r'$^{3}$').replace('–','--').replace('>=',r'$\geq$'))


def tex_header(x):
    label,sep,unit=str(x).partition(' [')
    lines=textwrap.wrap(label,width=19,break_long_words=False)
    if sep:lines.append('['+unit)
    return r'\shortstack{' + r'\\'.join(tex_escape(v)for v in lines) + '}'


def table(folder,stem,headers,rows,caption):
    folder.mkdir(parents=True,exist_ok=True)
    with (folder/(stem+'.csv')).open('w',newline='')as f:
        w=csv.writer(f);w.writerow(headers);w.writerows(rows)
    md=[caption,'','|'+'|'.join(headers)+'|','|'+'|'.join(['---']*len(headers))+'|']
    md+=['|'+'|'.join(map(str,row))+'|'for row in rows]
    (folder/(stem+'.md')).write_text('\n'.join(md)+'\n')
    tex=[r'\begin{table*}[t]',r'\centering',r'\caption{'+tex_escape(caption)+'}',
         r'\label{tab:'+stem+'}',r'\resizebox{\textwidth}{!}{%',
         r'\begin{tabular}{'+'l'*min(2,len(headers))+'r'*max(0,len(headers)-2)+'}',r'\toprule',
         ' & '.join(map(tex_header,headers))+r' \\',r'\midrule']
    tex+=[' & '.join(map(tex_escape,row))+r' \\'for row in rows]
    tex +=[r'\bottomrule',r'\end{tabular}}',r'\end{table*}']
    (folder/(stem+'.tex')).write_text('\n'.join(tex)+'\n')
    return '\n'.join(md)


def build(base):
    grouped=read_csv(base/'tables/grouped_metrics.csv')
    trials=read_csv(base/'data/trial_metrics.csv')
    conv=json.loads((base/'supplement/convergence_summary.json').read_text())
    noisy=json.loads((base/'supplement/noisy_output_error_validation.json').read_text())
    def group(condition,method,metric,phase='turn',scenario='fixed_three_scenarios'):
        rr=[r for r in grouped if r['dataset']=='tracking_final' and r['scenario']==scenario and
            r['condition']==condition and r['method']==method and r['phase']==phase and r['metric']==metric]
        if len(rr)!=1:raise ValueError((condition,method,metric,len(rr)))
        return number(rr[0]['mean']),number(rr[0]['sd'])
    def cell(pair,scale=1,d=3):
        mean,sd=pair
        if mean is None:return 'NA'
        return f'{mean*scale:.{d}f}'+(f' ± {sd*scale:.{d}f}'if sd is not None else '')
    summaries={}
    main=[];smooth=[];phase_rows=[]
    for condition in ['nominal','heldout']:
        for method in METHODS:
            main.append([condition,LABELS[method],cell(group(condition,method,'lateral_rmse_m'),1000),
                         cell(group(condition,method,'lateral_p95_abs_m'),1000),
                         cell(group(condition,method,'yaw_rmse_rad'),180/np.pi),
                         cell(group(condition,method,'speed_mean_mps')),
                         cell(group(condition,method,'compute_p95_ms'),1,2)])
            smooth.append([condition,LABELS[method],
                cell(group(condition,method,'cmd_w_rate_rms_radps2')),
                cell(group(condition,method,'cmd_w_tv_per_s_radps2')),
                cell(group(condition,method,'body_w_accel_rms_radps2')),
                cell(group(condition,method,'body_w_jerk_rms_radps3')),
                cell(group(condition,method,'world_jerk_norm_rms_mps3'))])
            for phase in ['line','entry','apex','exit']:
                phase_rows.append([condition,LABELS[method],phase,
                    cell(group(condition,method,'lateral_rmse_m',phase),1000),
                    cell(group(condition,method,'lateral_p95_abs_m',phase),1000),
                    cell(group(condition,method,'yaw_rmse_rad',phase),180/np.pi)])
        summaries[condition]={}
        for metric in ['lateral_rmse_m','cmd_w_rate_rms_radps2','body_w_jerk_rms_radps3','world_jerk_norm_rms_mps3']:
            original=group(condition,'native',metric)[0]; proposed=group(condition,'full',metric)[0]
            summaries[condition][metric]={'native':original,'full':proposed,
                                          'full_minus_native_pct':100*(proposed/original-1)}
    table1=table(base/'tables','table01_tracking',
        ['Condition','Controller','Turn RMSE [mm]','Turn P95 [mm]','Yaw RMSE [deg]','Turn speed [m/s]','Turn compute P95 [ms]'],main,
        'Main tracking experiment. Values are mean ± sample SD of per-trial statistics across three fixed routes and three reused seeds (9 conditional observations per controller/condition), not confidence intervals or 9 independent route draws. All 72 routes completed.')
    table2=table(base/'tables','table02_smoothness',
        ['Condition','Controller','Command yaw-rate derivative RMS [rad/s²]','Command yaw TV/time [rad/s²]',
         'Body angular acceleration RMS [rad/s²]','Body angular jerk RMS [rad/s³]','Planar jerk RMS [m/s³]'],smooth,
        'Turn-only smoothness tradeoffs. Mean ± sample SD across the same nine fixed-scenario observations. Angular and planar jerk describe different physical quantities; command derivatives are not executed mechanical jerk.')
    table3=table(base/'tables','table03_phase_errors',
        ['Condition','Controller','Geometry phase','Lateral RMSE [mm]','Lateral P95 [mm]','Yaw RMSE [deg]'],phase_rows,
        'Phase-resolved tracking errors. Phases are fixed by reference curvature and its arc-length derivative; the nine-observation variability is descriptive.')
    recovery=[]; individual=[]
    for condition in ['nominal','heldout']:
        for method in METHODS:
            rr=[r for r in conv if r['plant']==condition and r['method']==method]
            def outcome(key):
                vals=[r[key]for r in rr if r[key]is not None]
                return (f'{len(vals)}/{len(rr)}',
                        f'{min(vals):.2f}–{max(vals):.2f}'if vals else 'not observed')
            nfirst,first=outcome('joint_settling_first_window_s');nlast,last=outcome('last_exit_settling_s')
            recovery.append([condition,LABELS[method],nfirst,first,nlast,last,
                             f"{min(r['censor_time_s']for r in rr):.2f}–{max(r['censor_time_s']for r in rr):.2f}"])
            for r in sorted(rr,key=lambda x:x['seed']):
                individual.append([condition,LABELS[method],r['seed'],
                    'not observed'if r['joint_settling_first_window_s']is None else f"{r['joint_settling_first_window_s']:.2f}",
                    'not observed'if r['last_exit_settling_s']is None else f"{r['last_exit_settling_s']:.2f}",
                    f"{r['censor_time_s']:.2f}",r['first_window_censored'],r['last_exit_censored']])
    table4=table(base/'tables','table04_recovery',
        ['Condition','Controller','First-window events','Observed first-window range [s]',
         'Last-exit events','Observed last-exit range [s]','Observation-end range [s]'],recovery,
        'Recovery from +0.10 m lateral and +10 deg heading error. Joint band: 10 mm and 2 deg, residence >=1 s. Event ranges include only observed events, with event counts retained explicitly; they are not uncensored population means. All 24 routes completed.')
    table(base/'tables','table05_recovery_individual',
          ['Condition','Controller','Seed','First-window [s]','Last-exit [s]','Observation end [s]','First censored','Last censored'],individual,
          'Individual recovery outcomes. Not observed means right-censored at observation end; no infinite or zero event times are substituted.')
    fitrows=[]
    for condition,p in noisy['plants'].items():
        for axis,fit in p['base_fit']['channels'].items():
            truth=p['assumed_true_parameters']['tau_'+axis]
            sd=p['noise_sensitivity'][axis]['tau_s']['sample_sd']
            unit='m/s'if axis=='v'else'rad/s'
            fitrows.append([condition,axis,f'{truth:.3f}',f"{fit['tau_s']:.6f}",f'{sd:.6f}',
                f"{fit['delay_s']:.2f}",f"{fit['gain']:.6f}",f"{fit['bias']:.6f}",
                f"{fit['validation_observed_rmse']:.6f}",f"{fit['validation_latent_truth_rmse']:.6f}",unit])
    table6=table(base/'tables','table06_noisy_identification',
        ['Condition','Axis','Assumed tau [s]','Estimated tau [s]','Tau SD, 5 noise draws [s]',
         'Selected D [s]','Gain','Bias','Validation observed RMSE','Validation latent RMSE','Output unit'],fitrows,
        'Synthetic noisy output-error identification: step training (seed 303), independent multisine validation (seed 304), observation SD 0.002 m/s and 0.003 rad/s. Time-constant SD spans five noise draws and is not a confidence interval. Known synthetic reset and exact smoother input are used.')
    spectral=read_csv(base/'spectrum/trial_spectral_power.csv')
    spectrum_rows=[]
    spectral_means={}
    for condition in ['nominal','heldout']:
        spectral_means[condition]={}
        for method in METHODS:
            pairs=[]
            for channel in ['cmd_w','w']:
                vals=[100*float(r['fraction_above_2hz'])for r in spectral if r['condition']==condition and
                      r['method']==method and r['channel']==channel and r['fraction_above_2hz']!='']
                assert len(vals)==9
                pairs.append((float(np.mean(vals)),float(np.std(vals,ddof=1))))
            spectral_means[condition][method]={'command_pct':pairs[0][0],'body_pct':pairs[1][0]}
            spectrum_rows.append([condition,LABELS[method],cell(pairs[0]),cell(pairs[1])])
    table7=table(base/'tables','table07_spectral_power',
        ['Condition','Controller','Command yaw power above 2 Hz [%]','Body yaw power above 2 Hz [%]'],spectrum_rows,
        'Whole-trial high-frequency angular power fraction. Mean ± sample SD of nine per-trial ratios; continuous trials include startup and turn transients. Welch: Fs=20 Hz, Hann128, overlap64, FFT256, segment-wise constant detrending. This is a different temporal support from the turn-only smoothness table.')
    # Supply human-readable rows as well as the supplement's numeric code table.
    numeric=read_csv(base/'supplement/noisy_model_parameters.csv')
    readable=[]
    for r in numeric:
        readable.append({'condition':['nominal','heldout'][int(float(r['plant_code_nominal0_heldout1']))],
            'axis':['v','w'][int(float(r['channel_code_v0_w1']))],
            **{k:v for k,v in r.items()if 'code_'not in k}})
    with (base/'tables/noisy_parameter_estimates.csv').open('w',newline='')as f:
        w=csv.DictWriter(f,fieldnames=list(readable[0]));w.writeheader();w.writerows(readable)
    def mm(condition,method):return group(condition,method,'lateral_rmse_m')[0]*1000
    nom=summaries['nominal'];held=summaries['heldout']
    doc=[
        '# Results and discussion','',
        '## Evidence and experiment inventory','',
        'The main experiment comprises 72 synthetic closed-loop trials, with 24 additional initial-error recovery trials and two historical-route simulations. All 98 closed-loop routes reached their geometric completion criterion. The dataset also contains independent-input open-loop model experiments and noisy-output identification examples. None of these experiments executes FAST-LIO/NDT or measures physical vehicle accuracy. The historical FAST-LIO/NDT record is contextual evidence only and is not merged with synthetic errors.','',
        '## Tracking accuracy and ablation','',table1,'',
        f"Guided MPC reduced mean turn lateral RMSE from {mm('nominal','native'):.3f} to {mm('nominal','full'):.3f} mm in the nominal condition ({-nom['lateral_rmse_m']['full_minus_native_pct']:.1f}% reduction), and from {mm('heldout','native'):.3f} to {mm('heldout','full'):.3f} mm under the stated model mismatch ({-held['lateral_rmse_m']['full_minus_native_pct']:.1f}% reduction). These are ratios of conditional mean per-trial errors, not population effect estimates. The true synthetic state is used for evaluation; the controller receives the perturbed state.",'',
        f"The preview tracker achieved the smallest nominal error ({mm('nominal','tracker'):.3f} mm), outperforming guided MPC in that setting. Under the held-out mismatch, its error increased to {mm('heldout','tracker'):.3f} mm, compared with {mm('heldout','full'):.3f} mm for guided MPC. The ideal-dynamics ablation reached {mm('heldout','ideal'):.3f} mm. This pattern supports the utility of execution-chain prediction and residual feedback in these cases, while showing that sampling is not uniformly beneficial. The ideal ablation jointly removes prediction dynamics and their associated compensation, so it does not isolate every mechanism individually.",'',
        'Mean turn speeds are close, but their time profiles are not forced to coincide. The results therefore do not establish an isolated yaw-controller advantage under a pointwise identical speed reference. All individual trials and phase-specific results remain available for that distinction.','',
        '![Preselected seed-0 S-bend, held-out condition](figures/fig01_s_bend_heldout.png)','',
        '*Representative seed 0 was fixed before result selection. The complete figure caption, data sources and all remaining geometry/condition combinations are retained in the figure index.*','',
        table3,'',
        '## Smoothness tradeoffs','',table2,'',
        f"The accuracy gain was accompanied by more angular command activity. Guided MPC increased turn command yaw-rate derivative RMS by {nom['cmd_w_rate_rms_radps2']['full_minus_native_pct']:.1f}% nominally and {held['cmd_w_rate_rms_radps2']['full_minus_native_pct']:.1f}% under mismatch relative to Native MPPI. Executed angular jerk RMS increased by {nom['body_w_jerk_rms_radps3']['full_minus_native_pct']:.1f}% and {held['body_w_jerk_rms_radps3']['full_minus_native_pct']:.1f}%, respectively. The present objective does not explicitly penalize command increments or jerk.",'',
        f"Conversely, planar world-frame jerk RMS decreased by {-nom['world_jerk_norm_rms_mps3']['full_minus_native_pct']:.1f}% and {-held['world_jerk_norm_rms_mps3']['full_minus_native_pct']:.1f}% in the two conditions. Planar jerk includes linear-speed variation and direction change; angular jerk is the second derivative of yaw rate. These metrics are not interchangeable. The appropriate conclusion is a channel-dependent tradeoff, rather than a claim that the controller is globally smoother. Physical wheel-angle or steering-motor smoothness is not measured.",'',
        '![Turn angular smoothness for all trials](figures/fig05b_turn_angular_smoothness.png)','',
        '*Every point is a trial-level statistic; horizontal black marks are descriptive means. See the companion planar-smoothness figure and Table 2 before making an overall smoothness statement.*','',
        '## Angular power spectra','',table7,'',
        f"The whole-trial spectral analysis is consistent with increased angular command activity: mean command power above 2 Hz was {spectral_means['nominal']['full']['command_pct']:.3f}% for guided MPC versus {spectral_means['nominal']['native']['command_pct']:.3f}% for Native MPPI nominally, and {spectral_means['heldout']['full']['command_pct']:.3f}% versus {spectral_means['heldout']['native']['command_pct']:.3f}% under mismatch. Body-response spectra are supplied separately. The 256-point FFT has a 0.078125 Hz grid spacing, but zero-padding a 128-sample window does not increase physical resolving power. No statistical spectral confidence bounds or hardware vibration conclusions are claimed.",'',
        '## Recovery and finite-horizon settling','',table4,'',
        'Guided MPC satisfied the final joint error-band residence criterion in all six recovery trials, with last-exit times of 5.10–5.35 s nominally and 5.00–5.35 s under mismatch. Native MPPI had no observed last-exit event in its six recovery trials before route observation ended. This is a censored finite-horizon result, not an infinite settling-time estimate. First-window and last-exit statistics are intentionally distinguished: an initial residence can be followed by a later exit.','',
        'Recovery experiments start with 100 mm lateral and 10 deg heading error. Their larger transient errors are not pooled into the on-route main experiment. All observed event times, censor indicators and observation endpoints are exported. The initial condition has one sign and magnitude; conclusions should not be generalized to an untested region of attraction.','',
        '![All initial-offset recovery traces](figures/fig09_initial_offset_convergence.png)','',
        '*All three seeds are drawn separately, ending at their actual observation endpoints. Dashed lines show the 10 mm and 2 deg thresholds; joint residence and censoring are evaluated separately in Table 4.*','',
        '## Response lag and model-data analysis','',
        'Closed-loop lag curves are supplied with peak correlation, overlap and peak width. Many whole-trial maxima are broad and fail the declared reliability diagnostics. They should not be summarized as precise identified delays or used to establish mechanical hysteresis. The 50 ms scan grid also limits apparent lag resolution. The open-loop experiments provide a separate, identifiable synthetic model exercise.','',table6,'',
        f"For the yaw channel, noisy output-error fitting recovered approximately {noisy['plants']['nominal']['base_fit']['channels']['w']['tau_s']:.6f} s and {noisy['plants']['heldout']['base_fit']['channels']['w']['tau_s']:.6f} s time constants from synthetic truths of 0.25 s and 0.38 s. The selected transport delays were 0.10 s and 0.15 s. Validation used a different multisine input and an independent output-noise realization. The approximately 0.003 rad/s observed validation error is close to the imposed yaw observation-noise SD; much smaller latent-truth errors are reported separately. These results establish recovery within the specified synthetic model family, not identification of the real chassis.",'',
        'Noiseless fits and replay checks are retained as implementation-consistency tests. They are not independent model-validation evidence. Dynamic input-output loops, analytical Bode curves and residuals are provided to expose response behavior, while explicitly excluding claims about rate-independent hysteresis, friction or backlash.','',
        '![Independent multisine validation of synthetic noisy model fits](figures/fig13_noisy_model_validation.png)','',
        '*Training and validation inputs/noise draws differ. Near-overlap with synthetic truth is conditional on the declared first-order model family and known reset/input, not evidence of physical vehicle-model accuracy.*','',
        '## Computation and sampling diagnostics','',
        'The main 72-trial cohort contained no measured 50 ms compute exceedance. Guided MPC per-trial P95 computation averaged approximately 24–26 ms for 1,000 trajectories and 50 steps. These timings are machine-specific and do not include computation-induced plant latency. Sampling costs and effective sample sizes from the recovery experiment are additional diagnostics. Since the state and objective change between receding-horizon cycles, their traces are not evidence of monotonic solver convergence.','',
        '## Statistical and implementation limitations','',
        'Each route/condition/controller cell has three seeded repeats. The nine-observation pooled summaries are conditional on three fixed routes and reuse the same three noise seeds. Per-seed traces and matched differences are supplied, but confidence intervals with dependable population coverage are not claimed. Exploratory seed-block sign-flip diagnostics have very coarse attainable p-values; multiplicity-adjusted results do not establish confirmatory significance.','',
        'The original Nav2 baseline also has unresolved implementation variability documented by long-sequence repeat tests. An independently reproduced upstream helper boundary defect does not prove a cause of complete-trial variability. Strong publication claims should include a verified patched/current baseline as a separately named experiment. The preview-tracker and ideal-dynamics ablations remain necessary mechanism controls.','',
        'The physical FAST-LIO/NDT closed loop, wheel-level actuation model and independent motion-capture validation have not been completed. The smoother is a multirate approximation, and controller-side conditioning state is available from simulation. The two synthetic execution settings are not a general uncertainty set. Consequently the supported claim is improved conditional synthetic tracking with faster observed recovery and a measured smoothness tradeoff.','',
        '## Data and figures','',
        'Full-resolution samples, geometry, parameters, lag scans, all per-trial metrics and provenance are in `data/`; publication tables and LaTeX floats are in `tables/`. `figures/INDEX.md` defines every figure, variability band, source and processing operation. `supplement/README.md` documents the recovery and model experiments. See `METHODS.md` for equations and `CLAIMS_AND_LIMITATIONS.md` for the claim-to-evidence map.','']
    (base/'RESULTS_AND_DISCUSSION.md').write_text('\n'.join(doc))
    (base/'tables/paper_numeric_summary.json').write_text(json.dumps(summaries,indent=2)+'\n')
    (base/'tables/appendix_tables.tex').write_text('% Requires booktabs and graphicx. Paths assume the paper directory as compilation root.\n'+
        '\n'.join(r'\input{tables/'+name+'.tex}'for name in ['table01_tracking','table02_smoothness','table03_phase_errors',
                 'table04_recovery','table05_recovery_individual','table06_noisy_identification','table07_spectral_power'])+'\n')
    (base/'MANUSCRIPT.md').write_text('# Execution-aware guided sampling MPC for fixed-route inspection: a reproducible simulation study\n\n'+
        (base/'METHODS.md').read_text()+'\n'+(base/'RESULTS_AND_DISCUSSION.md').read_text())
    print(base/'MANUSCRIPT.md')


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--paper',type=Path,default=HERE/'paper');build(ap.parse_args().paper)
