# Execution-aware guided sampling MPC for fixed-route inspection: a reproducible simulation study

# Experimental methods

## Study scope and controller variants

This study evaluates fixed-route inspection tracking in a synthetic closed loop. Obstacle costs are excluded from the inspection task. The original FAST-LIO/NDT configuration and historical reference route are retained as project assets, but neither FAST-LIO nor NDT is executed in these controller experiments. Synthetic pose perturbations must not be interpreted as measured localization errors.

Four controllers are compared. **Native MPPI** calls the installed Nav2 1.1.20 `MPPIController` through its original pluginlib factory, using the production `InspectionStrict` parameter subtree. **Preview tracker** uses the same geometric guidance and execution prediction as the proposed controller, without stochastic residual search. **Ideal-dynamics ablation** retains residual sampling and the objective but removes execution-chain dynamics and their associated preview/lag compensation. **Execution-aware guided sampling MPC**, abbreviated **guided MPC**, combines the tracker proposal with sampled residual optimization. The last method is MPPI-inspired; an exact importance-sampling derivation for its state-dependent proposal and selection rules is not claimed.

The baseline and guided methods use a 2.5 s prediction horizon, 50 prediction steps and 1,000 sampled trajectories per update. The tracker uses one deterministic proposal. All controllers are evaluated at a nominal 20 Hz control rate and share a synthetic downstream envelope of 0.20 m/s forward speed and 0.45 rad/s yaw rate. The original controller's raw output is preserved before downstream clipping. In particular, its native output filter can slightly exceed the optimizer's internal speed limit.

The Native MPPI adapter calls the controller plugin directly. Controller-server velocity deadbands, Nav2 progress/goal termination, live TF lookup and sensor-layer updates are not part of this adapter. Completion is instead evaluated by the same geometric criterion for every method. Thus, this is an original-plugin comparison rather than a complete reproduction of the physical navigation pipeline.

## Reference geometry and guidance

Let the reference route be parameterized by arc length \(s\), with position \(p_r(s)\), unwrapped tangent angle \(\psi_r(s)\), curvature \(\kappa(s)\), and curvature derivative \(\kappa'(s)\). Finite-segment projection within an ordered local search window defines progress. The signed lateral error is

\[
e_y=[-\sin\psi_r,\;\cos\psi_r]\,[p-p_r]^\top,
\qquad e_\psi=\operatorname{atan2}(\sin(\psi-\psi_r),\cos(\psi-\psi_r)).
\]

The reference yaw is interpolated from the stored path orientation; the reported lateral error is its normal component at the projected point. It is not interchangeable with Euclidean endpoint error. Command proposals use

\[
s_p=s+D_c\max(v,0.05),\qquad
u_\omega=\operatorname{clip}\{v_r\kappa(s_p)-k_y e_y-k_\psi e_\psi+
\tau_{\omega,c}v_r^2\kappa'(s_p)+r_\omega\},
\]

where \(k_y=1.5\), \(k_\psi=2.0\), \(D_c=0.10\) s and \(\tau_{\omega,c}=0.25\) s. The derivative feedforward approximates an inverse first-order yaw response under approximately constant reference speed; acceleration-dependent terms are omitted. The forward proposal is \(u_v=\operatorname{clip}(v_r+r_v,0,0.20)\). Its reference is the minimum of the cruise speed, the implemented endpoint braking profile, and the yaw-rate feasibility cap. The exact expressions are exported in `data/model_specification.json`.

Residual innovations have standard deviations 0.05 m/s and 0.12 rad/s. Temporal correlation is imposed by \(\epsilon_j=0.80\epsilon_{j-1}+0.60\eta_j\). Candidate sets include the deterministic tracker, the shifted previous residual sequence, left/right exploration and a braking proposal. Composite objective values define normalized weights \(w_i\propto\exp[-(J_i-J_{\min})/0.20]\). The weighted residual is re-evaluated and accepted only when its objective is no more than 0.05 above the best candidate; otherwise, the best candidate is selected. No monotonic objective decrease across successive control cycles follows from this rule.

For inspection, the stage objective implemented by the prototype is

\[
\ell=\Delta t\left[140(1+2\min(|\kappa|,2))e_y^2+10e_\psi^2+
12(v-v_r)^2+0.15r_v^2+0.05r_\omega^2\right],
\]

with terminal terms \(35e_y^2+3e_\psi^2-2(s_N-s_0)\). The implementation evaluates stage errors before each integration update and speed after the update; the terminal error reuses the final pre-update error. These details are retained for reproducibility. No command-increment or jerk penalty is present in this objective, so improved tracking does not imply improved smoothness.

## Synthetic execution chain and assumed models

The simulated execution chain contains independent speed/yaw clipping, a 30 Hz open-loop rate limiter, a transport queue and first-order body responses. Forward and yaw acceleration limits are 0.45 m/s² and 0.80 rad/s². Nominal time constants are \(\tau_v=0.12\) s and \(\tau_\omega=0.25\) s with transport delay \(D=0.10\) s. A held-out condition increases \(\tau_\omega\) to 0.38 s and \(D\) to 0.15 s and adds a 0.008 rad/s yaw bias. Controller assumptions are unchanged in this condition.

After the smoother, the idealized linear channels are

\[
G_v(s)=\frac{e^{-Ds}}{\tau_v s+1},\qquad
G_\omega(s)=\frac{e^{-Ds}}{\tau_\omega s+1}.
\]

These expressions exclude nonlinear clipping and rate limits; the bias is a separate input. They are declared simulation models, not identified vehicle transfer functions. Body velocities are integrated in 5 ms substeps using exponential first-order updates, followed by a midpoint-yaw planar pose update. The implemented multirate approximation collapses smoother events within a control interval into its final snapshot, which is then used throughout that body interval. It is not exact event-time simulation of the physical smoother.

Guided controllers receive measured body state together with the simulated smoother state and recent conditioned-command queue. These command-conditioning states can in principle be reconstructed from issued commands and known timing, but that reconstruction has not been validated on hardware. Ground-truth body state is perturbed before controller input by independent Gaussian noise with standard deviations 2 mm in each position coordinate, 0.003 rad in yaw, 0.002 m/s in forward speed and 0.003 rad/s in yaw rate. The same pre-generated noise sequence is reused for matched scenario/seed comparisons. Synthetic ground truth, rather than the perturbed observation, is used for evaluation.

`data/controller_model_parameters.csv` gives the fixed assumptions. `data/model_parameters.csv` records the model parameters associated with individual trials. `data/local_linearizations.json` contains a fixed-speed, delay-free linearization of the tracking guide only. Its eigenvalues are not a stability certificate for the constrained, delayed sampling controller.

## Main tracking protocol

The main experiment contains a smooth left 90° turn, its mirrored right turn and an S-shaped sequence of opposite turns. Turn radii are 0.80 m for individual turns and 0.75 m for the S route, with 0.30 m curvature-transition lengths. Each controller is evaluated under both execution conditions with three noise seeds. Thus, 72 trials form the main cohort. A 47.7 m frozen historical inspection route is evaluated separately, once per Native MPPI and guided MPC. This supplementary route experiment reuses path geometry but still uses synthetic state and actuation.

Geometric route completion requires progress within 35 mm of the route end and position within 80 mm of its endpoint. It is not a terminal parking-accuracy criterion. All attempts, including unsuccessful attempts, remain in the experiment inventory. The main route-tracking experiment begins on the reference route and therefore does not by itself measure recovery from a nonzero initial tracking error.

Geometry defines the analysis phases before inspecting controller errors: line segments satisfy \(|\kappa|\leq0.15\,\mathrm{m}^{-1}\); the remaining samples are turns. Entry/exit phases satisfy \(\operatorname{sign}(\kappa)\kappa'>0.4\,\mathrm{m}^{-2}\) or \(<-0.4\,\mathrm{m}^{-2}\), respectively; the intervening curved samples form the apex bin. These are geometric bins. Derivative-based bins on the unsmoothed historical polyline can alternate rapidly and should not be interpreted as independently observed steering events.

## Accuracy and integrated error

Per-trial metrics include RMSE, MAE, signed mean, sample SD, 95th-percentile absolute error, maximum absolute error, IAE, ISE and ITAE for both lateral and heading error. Integrals use trapezoids only on observed contiguous intervals. The original recordings start at 0.05 s; no artificial initial sample is inserted, and the missing initial interval is reported. Phase integrals require both adjacent samples to belong to the same phase; separated turns are never concatenated across gaps. Distance-weighted error metrics use true intersample Euclidean displacement, and mission-time ITAE retains the original time origin for every phase.

## Smoothness and computational cost

Requested-command and executed-body metrics are analyzed separately. For each velocity channel, the increment RMS and total variation are reported, together with total variation per observed second. First finite differences quantify command rate or body acceleration; second finite differences quantify command second derivative or body jerk. Angular quantities use radians throughout the machine-readable tables. Planar world-frame acceleration and jerk additionally account for the rotating velocity direction, including centripetal effects.

All derivatives refer to the sampled 20 Hz signals. They do not estimate unobserved continuous-time maxima. No curve smoothing or low-pass preprocessing is applied to improve a controller's visual appearance. Differences in experiment duration and endpoint behavior are controlled by reporting phase-specific and duration-normalized quantities. Wall-clock compute median, P95, P99, maximum and 50 ms deadline exceedances are retained. Compute time is measured but is not fed back into the synthetic plant as an additional actuation delay.

Whole-trial command and body power spectra use Welch estimation with 20 Hz sampling, a 128-sample Hann window, 64-sample overlap, a 256-point FFT, segment-wise constant detrending and one-sided density scaling. The FFT grid spacing is 0.078125 Hz; the unpadded window spacing is 0.15625 Hz, and zero-padding does not improve physical resolving power. The power fraction above 2 Hz is integrated over 2–10 Hz and divided by the 0–10 Hz integral. Negligible denominators are reported as undefined. Continuous whole trials include startup and turning transients; disjoint phase segments are not stitched together for spectral estimation.

## Error recovery and the meaning of convergence

A separate 24-trial experiment initializes the left-turn task with a +0.10 m lateral displacement and +10° heading error. It otherwise uses the same controllers, two execution conditions and three seeds without retuning. A genuine initial sample at \(t=0\) is recorded for this experiment.

The joint tolerance band is \(|e_y|\leq0.01\) m and \(|e_\psi|\leq2°\), with a minimum continuous sampled residence of 1 s. **First-window recovery** records the first residence interval meeting this duration, even if the trajectory later exits. **Last-exit settling** records the beginning of the final in-band interval only if it lasts at least 1 s before observation ends. Unobserved events are right-censored at the last observation, not assigned zero or omitted from the denominator. A completed route can therefore have a censored settling event.

These are empirical finite-horizon recovery measures. Neither these curves nor local guide eigenvalues prove asymptotic stability. The deployed prototype performs one sampled update per MPC cycle; changing receding-horizon objective values and effective sample sizes are diagnostic traces, not a fixed-objective solver-convergence experiment.

## Response lag and dynamic loops

Closed-loop response lag is described by normalized cross-correlation between a fixed-cruise-speed curvature reference \(\omega_r=v_{\mathrm{cruise}}\kappa(s(t))\) and commanded or executed yaw rate. Response clocks are respected: a command on a raw row is issued at \(t-0.05\) s, whereas its paired body state is observed at \(t\). Correlation scans cover −1 to +1 s with 50 ms spacing, within observed support only. Positive lag denotes a trailing response; negative lag denotes anticipatory association.

Peak correlation, overlap, excitation and peak width accompany every estimate. Broad, boundary-limited or weakly excited peaks are flagged as unreliable. Because both reference progress and control signals depend on the evolving closed loop, these lags are descriptive associations, not independently identified causal transport delays.

Open-loop step and multisine tests separately exercise the declared synthetic execution model. Their command, smoothed input, delayed input and body-response channels are retained. Model fitting and independent-input prediction are reported separately from exact replay checks. Noiseless recovery of parameters from data generated by the same equations is an implementation-consistency result; noisy output-error fits provide an additional synthetic identification example. Neither constitutes vehicle identification.

For noisy identification, transport delay is searched on integer lags \(d=0,\ldots,8\) at 50 ms spacing. At each candidate lag, recursive output-error nonlinear least squares fits \(\tau\), gain \(g\) and bias \(b\), using \(\hat y_k=a\hat y_{k-1}+(1-a)(g u_{k-d}+b)\), \(a=\exp(-0.05/\tau)\). The known zero reset initializes prediction, and the initial noisy observed sample is excluded from the fit. Bounds are \(\tau\in[0.03,2]\) s, \(g\in[0.5,1.5]\), and \(b\in[-0.05,0.05]\) in the corresponding velocity unit. Step-input training uses noise seed 303, and independent multisine validation uses seed 304; five noise-draw pairs quantify parameter sensitivity without asserting confidence-interval coverage.

Input-output loops caused by delay and first-order dynamics are described as **dynamic loops**. Their presence does not establish rate-independent mechanical hysteresis, steering backlash, dead zones or friction. Those mechanisms require additional physical experiments.

## Statistical reporting

The experimental unit is a trial. The three seeded repeats for each fixed scenario are reported with descriptive mean, SD, range and individual points. Pooled nine-trial summaries condition on the three chosen scenarios; they are not nine independent route draws. Reused seed sequences preserve pairing but also induce dependence across scenarios.

Matched differences compare controllers on the same scenario and seed. Exploratory sign-flip diagnostics, where supplied, flip entire seed blocks and apply Holm adjustment across all reported diagnostics. With three blocks, attainable two-sided p-values are coarse; these diagnostics do not support population-wide significance claims. Confidence intervals with dependable coverage are not claimed. Figure legends define every variability band and state its number of trials.

## Related work and contribution boundary

Sampling-based control and information-theoretic MPC have established foundations in [Williams et al.](https://arxiv.org/abs/1707.02342). Integrating tracking-controller capabilities into sampling optimization has precedent in [RMPPI](https://arxiv.org/abs/2102.09027). Command smoothness has been addressed through input lifting in [Smooth MPPI](https://arxiv.org/abs/2112.09988), and guided multimodal proposals have been studied in [SVG-MPPI](https://arxiv.org/abs/2309.11040). A recent [MPPI–PID preprint](https://arxiv.org/abs/2603.29499) optimizes PID gains online; the present prototype instead samples input residual sequences around fixed-gain guidance.

Accordingly, the proposed contribution should be assessed as a platform-motivated execution-chain model and curvature-preview proposal with controlled ablations, rather than as the first combination of a tracker and MPPI. No new optimality, importance-sampling or stability theorem is established here.

# Results and discussion

## Evidence and experiment inventory

The main experiment comprises 72 synthetic closed-loop trials, with 24 additional initial-error recovery trials and two historical-route simulations. All 98 closed-loop routes reached their geometric completion criterion. The dataset also contains independent-input open-loop model experiments and noisy-output identification examples. None of these experiments executes FAST-LIO/NDT or measures physical vehicle accuracy. The historical FAST-LIO/NDT record is contextual evidence only and is not merged with synthetic errors.

## Tracking accuracy and ablation

Main tracking experiment. Values are mean ± sample SD of per-trial statistics across three fixed routes and three reused seeds (9 conditional observations per controller/condition), not confidence intervals or 9 independent route draws. All 72 routes completed.

|Condition|Controller|Turn RMSE [mm]|Turn P95 [mm]|Yaw RMSE [deg]|Turn speed [m/s]|Turn compute P95 [ms]|
|---|---|---|---|---|---|---|
|nominal|Native MPPI|4.182 ± 2.100|6.019 ± 2.710|0.545 ± 0.062|0.199 ± 0.001|8.92 ± 0.86|
|nominal|Preview tracker|0.401 ± 0.122|0.688 ± 0.168|0.104 ± 0.018|0.200 ± 0.000|1.22 ± 0.10|
|nominal|Ideal-dynamics ablation|2.442 ± 0.316|4.943 ± 0.338|1.585 ± 0.333|0.200 ± 0.000|26.75 ± 3.49|
|nominal|Guided MPC|0.893 ± 0.127|1.594 ± 0.212|0.321 ± 0.041|0.200 ± 0.000|25.04 ± 3.27|
|heldout|Native MPPI|5.731 ± 1.211|8.342 ± 1.531|1.004 ± 0.073|0.198 ± 0.001|8.91 ± 0.88|
|heldout|Preview tracker|5.074 ± 2.312|8.154 ± 3.883|0.714 ± 0.045|0.200 ± 0.000|1.34 ± 0.17|
|heldout|Ideal-dynamics ablation|8.613 ± 1.641|15.341 ± 3.348|4.376 ± 0.650|0.199 ± 0.001|26.46 ± 3.65|
|heldout|Guided MPC|1.461 ± 0.125|2.516 ± 0.418|0.390 ± 0.071|0.200 ± 0.000|24.06 ± 1.95|

Guided MPC reduced mean turn lateral RMSE from 4.182 to 0.893 mm in the nominal condition (78.7% reduction), and from 5.731 to 1.461 mm under the stated model mismatch (74.5% reduction). These are ratios of conditional mean per-trial errors, not population effect estimates. The true synthetic state is used for evaluation; the controller receives the perturbed state.

The preview tracker achieved the smallest nominal error (0.401 mm), outperforming guided MPC in that setting. Under the held-out mismatch, its error increased to 5.074 mm, compared with 1.461 mm for guided MPC. The ideal-dynamics ablation reached 8.613 mm. This pattern supports the utility of execution-chain prediction and residual feedback in these cases, while showing that sampling is not uniformly beneficial. The ideal ablation jointly removes prediction dynamics and their associated compensation, so it does not isolate every mechanism individually.

Mean turn speeds are close, but their time profiles are not forced to coincide. The results therefore do not establish an isolated yaw-controller advantage under a pointwise identical speed reference. All individual trials and phase-specific results remain available for that distinction.

![Preselected seed-0 S-bend, held-out condition](figures/fig01_s_bend_heldout.png)

*Representative seed 0 was fixed before result selection. The complete figure caption, data sources and all remaining geometry/condition combinations are retained in the figure index.*

Phase-resolved tracking errors. Phases are fixed by reference curvature and its arc-length derivative; the nine-observation variability is descriptive.

|Condition|Controller|Geometry phase|Lateral RMSE [mm]|Lateral P95 [mm]|Yaw RMSE [deg]|
|---|---|---|---|---|---|
|nominal|Native MPPI|line|2.685 ± 0.859|4.289 ± 1.437|0.416 ± 0.108|
|nominal|Native MPPI|entry|2.612 ± 0.856|3.972 ± 1.392|0.943 ± 0.104|
|nominal|Native MPPI|apex|4.251 ± 2.259|5.782 ± 2.782|0.384 ± 0.070|
|nominal|Native MPPI|exit|4.855 ± 2.793|5.857 ± 3.137|0.550 ± 0.081|
|nominal|Preview tracker|line|0.220 ± 0.091|0.446 ± 0.184|0.061 ± 0.011|
|nominal|Preview tracker|entry|0.305 ± 0.119|0.508 ± 0.198|0.137 ± 0.043|
|nominal|Preview tracker|apex|0.430 ± 0.137|0.686 ± 0.164|0.078 ± 0.012|
|nominal|Preview tracker|exit|0.331 ± 0.147|0.536 ± 0.253|0.143 ± 0.039|
|nominal|Ideal-dynamics ablation|line|2.598 ± 0.450|7.158 ± 0.578|0.962 ± 0.239|
|nominal|Ideal-dynamics ablation|entry|2.413 ± 0.194|4.470 ± 0.311|1.373 ± 0.191|
|nominal|Ideal-dynamics ablation|apex|2.525 ± 0.408|5.085 ± 0.403|1.544 ± 0.390|
|nominal|Ideal-dynamics ablation|exit|2.009 ± 0.585|3.956 ± 0.899|1.889 ± 0.344|
|nominal|Guided MPC|line|0.687 ± 0.195|1.450 ± 0.411|0.170 ± 0.023|
|nominal|Guided MPC|entry|1.143 ± 0.208|1.664 ± 0.254|0.454 ± 0.112|
|nominal|Guided MPC|apex|0.865 ± 0.161|1.499 ± 0.232|0.212 ± 0.030|
|nominal|Guided MPC|exit|0.640 ± 0.144|1.134 ± 0.247|0.479 ± 0.078|
|heldout|Native MPPI|line|1.512 ± 0.355|2.854 ± 0.662|0.632 ± 0.117|
|heldout|Native MPPI|entry|2.473 ± 0.571|4.731 ± 1.334|1.711 ± 0.118|
|heldout|Native MPPI|apex|6.395 ± 1.286|8.507 ± 1.617|0.739 ± 0.053|
|heldout|Native MPPI|exit|5.081 ± 1.801|6.055 ± 2.182|0.960 ± 0.160|
|heldout|Preview tracker|line|3.756 ± 1.306|6.738 ± 2.694|0.476 ± 0.066|
|heldout|Preview tracker|entry|4.471 ± 1.655|6.784 ± 2.983|1.096 ± 0.076|
|heldout|Preview tracker|apex|5.282 ± 2.778|7.931 ± 4.464|0.534 ± 0.039|
|heldout|Preview tracker|exit|4.470 ± 1.145|6.027 ± 1.586|0.860 ± 0.074|
|heldout|Ideal-dynamics ablation|line|6.193 ± 4.483|13.637 ± 9.049|3.126 ± 2.180|
|heldout|Ideal-dynamics ablation|entry|4.293 ± 1.438|7.914 ± 2.754|2.494 ± 0.754|
|heldout|Ideal-dynamics ablation|apex|8.801 ± 1.135|15.212 ± 2.590|4.592 ± 0.588|
|heldout|Ideal-dynamics ablation|exit|10.507 ± 3.984|15.060 ± 5.818|4.684 ± 1.575|
|heldout|Guided MPC|line|1.784 ± 0.405|3.402 ± 1.099|0.286 ± 0.028|
|heldout|Guided MPC|entry|1.620 ± 0.327|2.078 ± 0.341|0.435 ± 0.167|
|heldout|Guided MPC|apex|1.440 ± 0.191|2.520 ± 0.502|0.396 ± 0.057|
|heldout|Guided MPC|exit|1.211 ± 0.595|1.668 ± 0.846|0.276 ± 0.092|

## Smoothness tradeoffs

Turn-only smoothness tradeoffs. Mean ± sample SD across the same nine fixed-scenario observations. Angular and planar jerk describe different physical quantities; command derivatives are not executed mechanical jerk.

|Condition|Controller|Command yaw-rate derivative RMS [rad/s²]|Command yaw TV/time [rad/s²]|Body angular acceleration RMS [rad/s²]|Body angular jerk RMS [rad/s³]|Planar jerk RMS [m/s³]|
|---|---|---|---|---|---|---|
|nominal|Native MPPI|0.192 ± 0.019|0.147 ± 0.017|0.126 ± 0.006|0.540 ± 0.095|0.187 ± 0.128|
|nominal|Preview tracker|0.262 ± 0.018|0.210 ± 0.014|0.127 ± 0.006|0.902 ± 0.065|0.027 ± 0.001|
|nominal|Ideal-dynamics ablation|0.530 ± 0.037|0.424 ± 0.032|0.246 ± 0.038|1.550 ± 0.085|0.051 ± 0.008|
|nominal|Guided MPC|0.473 ± 0.035|0.379 ± 0.027|0.134 ± 0.005|1.550 ± 0.087|0.028 ± 0.001|
|heldout|Native MPPI|0.216 ± 0.013|0.167 ± 0.011|0.130 ± 0.008|0.412 ± 0.043|0.164 ± 0.078|
|heldout|Preview tracker|0.275 ± 0.018|0.219 ± 0.014|0.130 ± 0.007|0.612 ± 0.043|0.028 ± 0.002|
|heldout|Ideal-dynamics ablation|0.705 ± 0.220|0.465 ± 0.106|0.328 ± 0.025|1.094 ± 0.071|0.258 ± 0.309|
|heldout|Guided MPC|0.475 ± 0.034|0.380 ± 0.027|0.128 ± 0.007|1.028 ± 0.055|0.028 ± 0.002|

The accuracy gain was accompanied by more angular command activity. Guided MPC increased turn command yaw-rate derivative RMS by 146.1% nominally and 120.0% under mismatch relative to Native MPPI. Executed angular jerk RMS increased by 186.9% and 149.7%, respectively. The present objective does not explicitly penalize command increments or jerk.

Conversely, planar world-frame jerk RMS decreased by 84.9% and 83.1% in the two conditions. Planar jerk includes linear-speed variation and direction change; angular jerk is the second derivative of yaw rate. These metrics are not interchangeable. The appropriate conclusion is a channel-dependent tradeoff, rather than a claim that the controller is globally smoother. Physical wheel-angle or steering-motor smoothness is not measured.

![Turn angular smoothness for all trials](figures/fig05b_turn_angular_smoothness.png)

*Every point is a trial-level statistic; horizontal black marks are descriptive means. See the companion planar-smoothness figure and Table 2 before making an overall smoothness statement.*

## Angular power spectra

Whole-trial high-frequency angular power fraction. Mean ± sample SD of nine per-trial ratios; continuous trials include startup and turn transients. Welch: Fs=20 Hz, Hann128, overlap64, FFT256, segment-wise constant detrending. This is a different temporal support from the turn-only smoothness table.

|Condition|Controller|Command yaw power above 2 Hz [%]|Body yaw power above 2 Hz [%]|
|---|---|---|---|
|nominal|Native MPPI|0.574 ± 0.387|0.024 ± 0.016|
|nominal|Preview tracker|0.793 ± 0.100|0.019 ± 0.003|
|nominal|Ideal-dynamics ablation|1.607 ± 0.345|0.052 ± 0.012|
|nominal|Guided MPC|4.717 ± 0.613|0.152 ± 0.028|
|heldout|Native MPPI|0.395 ± 0.235|0.009 ± 0.005|
|heldout|Preview tracker|0.562 ± 0.074|0.007 ± 0.001|
|heldout|Ideal-dynamics ablation|0.825 ± 0.279|0.011 ± 0.003|
|heldout|Guided MPC|3.542 ± 0.512|0.053 ± 0.010|

The whole-trial spectral analysis is consistent with increased angular command activity: mean command power above 2 Hz was 4.717% for guided MPC versus 0.574% for Native MPPI nominally, and 3.542% versus 0.395% under mismatch. Body-response spectra are supplied separately. The 256-point FFT has a 0.078125 Hz grid spacing, but zero-padding a 128-sample window does not increase physical resolving power. No statistical spectral confidence bounds or hardware vibration conclusions are claimed.

## Recovery and finite-horizon settling

Recovery from +0.10 m lateral and +10 deg heading error. Joint band: 10 mm and 2 deg, residence >=1 s. Event ranges include only observed events, with event counts retained explicitly; they are not uncensored population means. All 24 routes completed.

|Condition|Controller|First-window events|Observed first-window range [s]|Last-exit events|Observed last-exit range [s]|Observation-end range [s]|
|---|---|---|---|---|---|---|
|nominal|Native MPPI|2/3|17.25–17.30|0/3|not observed|22.00–23.50|
|nominal|Preview tracker|3/3|14.50–14.55|3/3|14.50–14.55|17.85–17.85|
|nominal|Ideal-dynamics ablation|3/3|16.85–18.75|3/3|16.85–18.75|19.45–22.65|
|nominal|Guided MPC|3/3|5.10–5.35|3/3|5.10–5.35|18.25–18.40|
|heldout|Native MPPI|0/3|not observed|0/3|not observed|22.05–24.20|
|heldout|Preview tracker|0/3|not observed|0/3|not observed|17.90–17.90|
|heldout|Ideal-dynamics ablation|0/3|not observed|0/3|not observed|24.95–26.20|
|heldout|Guided MPC|3/3|5.00–5.35|3/3|5.00–5.35|18.45–18.50|

Guided MPC satisfied the final joint error-band residence criterion in all six recovery trials, with last-exit times of 5.10–5.35 s nominally and 5.00–5.35 s under mismatch. Native MPPI had no observed last-exit event in its six recovery trials before route observation ended. This is a censored finite-horizon result, not an infinite settling-time estimate. First-window and last-exit statistics are intentionally distinguished: an initial residence can be followed by a later exit.

Recovery experiments start with 100 mm lateral and 10 deg heading error. Their larger transient errors are not pooled into the on-route main experiment. All observed event times, censor indicators and observation endpoints are exported. The initial condition has one sign and magnitude; conclusions should not be generalized to an untested region of attraction.

![All initial-offset recovery traces](figures/fig09_initial_offset_convergence.png)

*All three seeds are drawn separately, ending at their actual observation endpoints. Dashed lines show the 10 mm and 2 deg thresholds; joint residence and censoring are evaluated separately in Table 4.*

## Response lag and model-data analysis

Closed-loop lag curves are supplied with peak correlation, overlap and peak width. Many whole-trial maxima are broad and fail the declared reliability diagnostics. They should not be summarized as precise identified delays or used to establish mechanical hysteresis. The 50 ms scan grid also limits apparent lag resolution. The open-loop experiments provide a separate, identifiable synthetic model exercise.

Synthetic noisy output-error identification: step training (seed 303), independent multisine validation (seed 304), observation SD 0.002 m/s and 0.003 rad/s. Time-constant SD spans five noise draws and is not a confidence interval. Known synthetic reset and exact smoother input are used.

|Condition|Axis|Assumed tau [s]|Estimated tau [s]|Tau SD, 5 noise draws [s]|Selected D [s]|Gain|Bias|Validation observed RMSE|Validation latent RMSE|Output unit|
|---|---|---|---|---|---|---|---|---|---|---|
|nominal|v|0.120|0.119127|0.000417|0.10|1.000837|-0.000118|0.001975|0.000040|m/s|
|nominal|w|0.250|0.250363|0.001472|0.10|1.000972|-0.000072|0.002990|0.000114|rad/s|
|heldout|v|0.120|0.118563|0.000720|0.15|1.000811|-0.000115|0.001976|0.000049|m/s|
|heldout|w|0.380|0.380436|0.001825|0.15|1.000993|0.007927|0.002990|0.000111|rad/s|

For the yaw channel, noisy output-error fitting recovered approximately 0.250363 s and 0.380436 s time constants from synthetic truths of 0.25 s and 0.38 s. The selected transport delays were 0.10 s and 0.15 s. Validation used a different multisine input and an independent output-noise realization. The approximately 0.003 rad/s observed validation error is close to the imposed yaw observation-noise SD; much smaller latent-truth errors are reported separately. These results establish recovery within the specified synthetic model family, not identification of the real chassis.

Noiseless fits and replay checks are retained as implementation-consistency tests. They are not independent model-validation evidence. Dynamic input-output loops, analytical Bode curves and residuals are provided to expose response behavior, while explicitly excluding claims about rate-independent hysteresis, friction or backlash.

![Independent multisine validation of synthetic noisy model fits](figures/fig13_noisy_model_validation.png)

*Training and validation inputs/noise draws differ. Near-overlap with synthetic truth is conditional on the declared first-order model family and known reset/input, not evidence of physical vehicle-model accuracy.*

## Computation and sampling diagnostics

The main 72-trial cohort contained no measured 50 ms compute exceedance. Guided MPC per-trial P95 computation averaged approximately 24–26 ms for 1,000 trajectories and 50 steps. These timings are machine-specific and do not include computation-induced plant latency. Sampling costs and effective sample sizes from the recovery experiment are additional diagnostics. Since the state and objective change between receding-horizon cycles, their traces are not evidence of monotonic solver convergence.

## Statistical and implementation limitations

Each route/condition/controller cell has three seeded repeats. The nine-observation pooled summaries are conditional on three fixed routes and reuse the same three noise seeds. Per-seed traces and matched differences are supplied, but confidence intervals with dependable population coverage are not claimed. Exploratory seed-block sign-flip diagnostics have very coarse attainable p-values; multiplicity-adjusted results do not establish confirmatory significance.

The original Nav2 baseline also has unresolved implementation variability documented by long-sequence repeat tests. An independently reproduced upstream helper boundary defect does not prove a cause of complete-trial variability. Strong publication claims should include a verified patched/current baseline as a separately named experiment. The preview-tracker and ideal-dynamics ablations remain necessary mechanism controls.

The physical FAST-LIO/NDT closed loop, wheel-level actuation model and independent motion-capture validation have not been completed. The smoother is a multirate approximation, and controller-side conditioning state is available from simulation. The two synthetic execution settings are not a general uncertainty set. Consequently the supported claim is improved conditional synthetic tracking with faster observed recovery and a measured smoothness tradeoff.

## Data and figures

Full-resolution samples, geometry, parameters, lag scans, all per-trial metrics and provenance are in `data/`; publication tables and LaTeX floats are in `tables/`. `figures/INDEX.md` defines every figure, variability band, source and processing operation. `supplement/README.md` documents the recovery and model experiments. See `METHODS.md` for equations and `CLAIMS_AND_LIMITATIONS.md` for the claim-to-evidence map.
