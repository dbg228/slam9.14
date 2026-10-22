These files are synthetic supplemental experiments for offline controller
development. No FAST-LIO/NDT process, real robot, lidar rendering, CAN recording,
or hardware parameter identification is represented by these traces.

`manifest.json` records the frozen controller configuration, original source
hashes and trace semantics for 24 initial-offset experiments. The left-turn path
and all controller/plant settings are inherited from the original benchmark.
Only the initial pose changes to lateral error +0.10 m and heading error +10 deg.
Each method/condition/seed runs in a new process; localization perturbations are
identical for methods with the same seed. Native MPPI uses the installed original
engine. See `../../native/README.md` for its reproducibility limitations.

`convergence_summary.json` gives trial-level completion, failure and empirical
threshold-residence measures. `joint_settling_first_window_s` is the beginning of
the first interval with |e_y| <= 0.01 m AND |e_yaw| <= 2 deg continuously for at
least 1 observed second. It allows later exits. `last_exit_settling_s` is the
beginning of the final such interval, and requires at least 1 second remaining
in the recorded trace. An unobserved event is null and right censored at that
trial's final time. Observation ends at geometric route completion or a recorded
failure/timeout, so censoring horizons vary. These measurements do not establish
theoretical stability, asymptotic convergence, or ultimate boundedness.

Every `convergence_*.npz` has `trace`, `columns`, reference `path`, controller
`status`, and the applied localization `noise` array. CSV columns match the NPZ.
The initial sample at t = 0 is included. State values in a row are post-step;
the row's measured state and command were used at `t_command_issue = t - 0.05 s`.
The smoother and delayed values are those applied during that interval. The
first row has `command_valid = 0`; its command/diagnostic zeros are unavailable
sentinels. Native internal sampling costs/ESS are unavailable and marked by
`diagnostics_available = 0`. The clearance field is an unused zero sentinel in
these experiments with no obstacles and must not be treated as collision data.

The cost observer copies returned rollout costs without new evaluations or RNG
draws. A regression verifies unchanged commands and residual sequences. The
reported `compute_s` includes that small observer overhead and uses actual wall
time, while the synthetic time step remains 0.05 s. These wall timings depend on
the host and concurrent load. Stepwise cost or ESS trends are diagnostics of a
moving-horizon sampler, not optimization convergence certificates.

`openloop_*_step.*` and `openloop_*_multisine.*` use independent prescribed input
profiles with exact plant execution-chain logs. `model_validation.json` fits a
unity-gain first-order-plus-integer-delay model from smoother output to true body
velocity. The fit uses the step profile; validation recursively predicts the
entire independent multisine trace. Delay candidates use equal training sample
indices. The noiseless near-machine-precision agreement is an implementation
consistency check on matching synthetic equations, not empirical identification
accuracy. The raw-command-to-output relation additionally includes a nonlinear
rate limiter, so it is not the stated linear transfer. Input-output loop shapes
represent dynamic lag; no static mechanical hysteresis model is implemented.

Reproduce the frozen initial-offset experiment and noiseless model experiment:

```bash
python3 tests/turning_mppi/paper_supplement.py --output /tmp/mppi_supplement
python3 -m pytest -q tests/turning_mppi/test_paper_supplement.py
```

`convergence_protocol_source.py` archives the exact supplemental script used for
the 24 convergence trials and matches its hash in `manifest.json`. The current
script subsequently gained a separate noisy-model experiment; its source hash
is recorded in `noisy_model_manifest.json`. Neither the controller nor the
convergence experimental configuration changed during that extension.

`noisy_output_error_validation.json` is an additional synthetic observation-noise
experiment. The true body-velocity signals receive independent zero-mean
Gaussian noise with sigma_v = 0.002 m/s and sigma_w = 0.003 rad/s. These amplitudes
are assumed settings, not measured localization statistics. The base training
and validation seeds are 303 and 304. Five independent paired draws use seeds
(303,304), (305,306), (307,308), (309,310), (311,312). They share the same latent
plant and prescribed input profiles and are not independent hardware trials.

For each candidate delay 0 through 8 samples, recursive output-error nonlinear
least squares estimates tau, gain, and steady-state bias from the training step
profile. The broad fixed bounds are tau in [0.03,2] seconds, gain in [0.5,1.5],
and bias in [-0.05,0.05] output units. The known zero initial physical state is
used. Fitting compares a recursively predicted state with observed output and
does not use noisy previous measurements as exact regressors. The selected
model predicts the independent multisine profile without refitting. RMSE
against noisy observations and against latent synthetic truth are reported
separately, together with residual ACF, Durbin-Watson ratio, Jacobian rank,
conditioning, active-bound count, and every delay candidate's training error.
Reported standard deviations across five noise draws quantify sensitivity to
this chosen noise model and are not confidence intervals or hardware variation.

`noisy_model_training_*.npz` and `noisy_model_prediction_*.npz` preserve the base
training and independent validation data. Columns are t, observed v/w, latent
v/w, recursively predicted v/w, raw commands, and exact smoother outputs.
`noisy_model_parameters.csv` has all 20 plant/channel/noise-draw parameter rows.

Reproduce only the noisy extension using existing open-loop traces:

```bash
python3 tests/turning_mppi/paper_supplement.py --noisy-model-only
```
