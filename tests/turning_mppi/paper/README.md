# English paper data and analysis package

The package supports an English-language **simulation study** of fixed-route inspection tracking. Obstacle avoidance is outside the current study. It does not establish new-controller performance in a physical FAST-LIO/NDT closed loop.

## Manuscript and results

- [Complete editable manuscript sections](MANUSCRIPT.md): equations, experimental methods, numerical results and discussion.
- [Methods](METHODS.md) and [results/discussion](RESULTS_AND_DISCUSSION.md) are also supplied separately.
- [Claim-to-evidence map](CLAIMS_AND_LIMITATIONS.md) states which conclusions the data support.
- [References](references.bib) contains BibTeX entries for the cited control literature, including a 2026 MPPI–PID preprint. The prototype does not claim the first tracker/MPPI combination.

The manuscript includes both favorable and unfavorable outcomes. Turn tracking improves, and the offset-recovery experiment observes a 5.00–5.35 s final joint-band residence time for guided MPC. Angular command activity and angular jerk increase, while planar world-frame jerk decreases. No blanket smoothness or theoretical-convergence claim is made.

## Data inventory

| Dataset | Size | Purpose |
|---|---:|---|
| Main synthetic route tracking | 72 trials | Four methods × three fixed routes × two execution conditions × three seeds |
| Historical-route synthetic replay | 2 trials | Same frozen 47.7 m path; one repeat per method |
| Initial-error synthetic recovery | 24 trials | +0.10 m lateral and +10 deg heading error; joint tolerance and censoring |
| Raw main/replay state samples | 41,589 rows | All samples retained in 74 CSV files |
| Trial/phase metric table | 444 rows | All/line/turn/entry/apex/exit |
| Correlation lag diagnostics | 888 rows | Command/body clocks, overlap, excitation and reliability flags |
| Step and multisine profiles | 4 latent profiles | Two assumed plants; independent validation input |
| Noisy model fits | 20 channel fits | Two plants × two channels × five output-noise draws |
| Welch spectral data | 37,152 rows | 72 trials × four channels × 129 FFT bins |

All 98 closed-loop routes completed the geometric criterion. Settling events that were not observed remain right-censored, even when the route completed.

## Figures and tables

[The core figure index](figures/INDEX.md) contains **26 figure groups** with English captions, source lists and processing notes. [The spectral appendix caption](spectrum/caption_en.txt) describes the additional [Figure 15](spectrum/Fig15_welch_spectra.pdf). In total, **27 figure groups / 81 graphics files** are provided as vector PDF, editable-text SVG and 300 dpi PNG. Use PDF/SVG for the paper. Data are not visually smoothed to improve appearance. Variability displays show actual trials or explicitly defined descriptive ranges, not unsupported confidence intervals.

Seven publication table groups are available in CSV, Markdown and LaTeX:

1. [Tracking accuracy and computation](tables/table01_tracking.md).
2. [Angular and planar smoothness](tables/table02_smoothness.md).
3. [Geometry-defined phase errors](tables/table03_phase_errors.md).
4. [Recovery event counts/ranges](tables/table04_recovery.md).
5. [All individual recovery events and censoring](tables/table05_recovery_individual.md).
6. [Noisy model identification and independent-input validation](tables/table06_noisy_identification.md).
7. [Whole-trial high-frequency angular power](tables/table07_spectral_power.md).

LaTeX floats and the combined [table include file](tables/appendix_tables.tex) require `booktabs` and `graphicx`, with the paper directory as the compilation root. A LaTeX engine was not available in the analysis environment, so these source tables have not been compiled here. Graphics files were independently checked for embedding, format and image resolution.

## Raw and derived data

- [Metric dictionary](data/metric_dictionary.json): definitions, units, time alignment, integration support, derivatives, statistics and missing values.
- [Trial metrics](data/trial_metrics.csv) and [all full-resolution sample CSVs](data/samples/).
- [Long grouped table](tables/grouped_metrics.csv), [paired differences](tables/paired_differences.csv) and [paired summaries](tables/paired_summary.csv).
- [Correlation peaks](data/phase_lags.csv) and [complete lag scans](data/lag_curves.csv). Broad or weak peaks must not be interpreted as identified physical delays.
- [Reference geometry](data/path_profiles.csv).
- [Model specification](data/model_specification.json), [fixed controller/model parameter table](data/controller_model_parameters.csv), [per-trial model settings](data/model_parameters.csv), [local guide linearizations](data/local_linearizations.json) and [assumed frequency responses](data/model_frequency_response.csv).
- [Recovery/model experiment documentation](supplement/README.md), [24-trial outcomes](supplement/convergence_summary.json), [noisy fitting diagnostics](supplement/noisy_output_error_validation.json) and [human-readable parameter estimates](tables/noisy_parameter_estimates.csv).
- [Spectral data and parameters](spectrum/manifest.json).

Original simulation NPZ/JSON inputs are retained in the sibling `../results/tracking_final/` and `../results/historical_route/` folders. Supplemental NPZ/CSV traces reside in `supplement/`.

## Reproduce the analysis

From the project root, run:

```bash
bash tests/turning_mppi/paper/reproduce_analysis.sh
```

This regenerates analysis from the saved traces; it does not drive the vehicle or rerun native controller trials. It overwrites generated figures/tables/manuscript results. `METHODS.md` is the editable methods source. The exact observed Python package versions are recorded in [requirements-analysis.txt](requirements-analysis.txt); the original environment used Python 3.10.12. Existing NPZ analysis needs NumPy, SciPy, Matplotlib and PyYAML, without ROS. Rerunning native controller simulations additionally needs the installed Nav2 1.1.20 development/runtime packages and the local native bridge build. C++ prediction compilation needs g++.

Supplemental simulations can be rerun separately using `paper_supplement.py` followed by its `--noisy-model-only` extension. Use a new output directory to retain the frozen experimental source/manifest. The original native plugin is not bitwise reproducible on long traces; exact new-run equality is not promised.

## Verification and reproducibility boundaries

[Provenance](data/provenance.json) retains input/source/derived-file hashes and all missingness counts. [Core graphic validation](figures/artifact_validation.json), [core graphic hashes](figures/artifact_hashes.json) and the spectral manifest audit exported assets. `VALIDATION.json` records the final package checks. The downloadable archive also carries a file-size/SHA-256 manifest.

The same-simulator replay matched every original trace; this verifies integrity, not independent predictive validity. Assumed/identified synthetic time constants, Gaussian noise and exact smoother inputs do not establish real chassis parameters. The native version's repeatability and independently reproduced helper boundary issue are disclosed in the manuscript and native documentation. A verified current/patched baseline, measured actuator identification and a real FAST-LIO/NDT closed loop remain necessary for stronger physical claims.
