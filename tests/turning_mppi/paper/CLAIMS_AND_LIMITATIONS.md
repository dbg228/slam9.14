# Claim-to-evidence map

| Candidate statement | Evidence available | Permissible interpretation |
|---|---|---|
| Tracking error is reduced in the tested synthetic cases | 72 main traces; per-trial and per-phase tables | Descriptive, conditional on the specified routes, disturbance seeds and models |
| Performance is retained under model mismatch | One additional yaw time constant/delay/bias setting | Sensitivity to this stated mismatch, not general robust-control guarantees |
| Initial tracking errors recover faster | 24 offset-initialization trials with joint-band residence/censoring | Empirical error recovery, not asymptotic or Lyapunov convergence |
| Commands are smoother | Command increments/rates/TV and body jerk tables | Must be evaluated from data; do not infer it from reduced tracking error or smoother body plots |
| Response is less delayed | Correlation scans with peak-width/excitation diagnostics | Association with curvature reference; unreliable/broad peaks remain unidentified |
| The actuator model is identified | Assumed synthetic plant, open-loop fitting and independent-input prediction | Synthetic parameter recovery only; no physical calibration has been performed |
| The system has mechanical hysteresis | Synthetic dynamic input-output loops only | Unsupported: dynamic lag is not evidence of rate-independent mechanical hysteresis |
| The optimizer converges | One sampled update per receding-horizon cycle; cost/ESS diagnostics | No fixed-objective iterative-convergence experiment or proof |
| FAST-LIO/NDT improves with the new controller | Original configuration and old-record diagnostics | Unsupported: the new controller has not run in a physical FAST-LIO/NDT closed loop |
| This is the first tracker–MPPI combination | Related MPPI/tracker/gain-guidance literature exists | Unsupported; novelty must be stated more narrowly and compared with prior work |

## Baseline implementation validity

The Native MPPI baseline is the installed Nav2 1.1.20 library loaded through its original pluginlib factory. Its parameter subtree is preserved. Long-sequence checks nevertheless showed non-bitwise-repeatable commands after reseeding. A separate AddressSanitizer test demonstrates an out-of-range access in an installed path-distance helper; a causal link to the complete benchmark's variability has not been established. See `../native/README.md`, `../native/smoke_test_results.json`, and `../native/path_bounds_asan.txt`.

This version-specific baseline supports reporting the observed runs with their variability. A stronger publication claim should additionally use a verified patched or current baseline as a separately named experiment, and should retain the ideal-dynamics and tracker ablations. The existing unmodified baseline must not be silently replaced after its results have been reported.

## Additional limitations

The synthetic model has no individual steering-angle state, wheel slip, terrain contact, CAN timing, lidar/IMU simulation or correlated localization error model. The multirate smoother is an approximation. Guided controllers use available simulated command-conditioning state; a hardware observer is not validated. Compute latency does not affect simulated motion, and operating-system scheduling is not a real-time guarantee. The historical-route simulation has one repeat per method. The initial-error experiment uses one sign and magnitude of initial error. The fixed routes and three seeds do not establish broad generalization.

Favorable and unfavorable results must remain together. In particular, the present controller's objective penalizes residual magnitude but not command increments or jerk; tracking and smoothness can therefore trade off. The tables and discussion should report that tradeoff rather than label the controller uniformly superior.

## Reporting references

The [NeurIPS reporting checklist](https://neurips.cc/public/guides/PaperChecklist) is used as a general reproducibility reference for describing variability sources, computation and experimental scope. It is not presented as a requirement of a specific control journal. Normalized-correlation and spectral-analysis conventions can be checked against the official [SciPy correlation](https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.correlation_lags.html) and [Welch](https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.welch.html) documentation. The installed analysis version is recorded in the exported environment metadata.
