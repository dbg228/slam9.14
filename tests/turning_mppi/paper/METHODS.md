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
