# Publication figure index

All plots are derived from synthetic simulation traces. They are not FAST-LIO/NDT localization or real-robot validation.

PDF/SVG are vector graphics; PNG files are 300 dpi. Suggested double-column width: 178 mm. PDF embeds DejaVu Sans TrueType; SVG retains editable text. The PDF/SVG files should be preferred for manuscripts.

## Suggested manuscript placement

The following selection covers distinct scientific questions; it is not a selection of favorable trials. The remaining figures and all six fixed-seed representative geometry/condition combinations should remain available as supplementary material.

- Main text or a combined multipanel figure: [fig01_s_bend_heldout](fig01_s_bend_heldout.pdf).
- Main text or a combined multipanel figure: [fig02_turn_lateral_rmse](fig02_turn_lateral_rmse.pdf).
- Main text or a combined multipanel figure: [fig04_phase_errors](fig04_phase_errors.pdf).
- Main text or a combined multipanel figure: [fig05b_turn_angular_smoothness](fig05b_turn_angular_smoothness.pdf).
- Main text or a combined multipanel figure: [fig05c_turn_planar_smoothness](fig05c_turn_planar_smoothness.pdf).
- Main text or a combined multipanel figure: [fig06_runtime](fig06_runtime.pdf).
- Main text or a combined multipanel figure: [fig09_initial_offset_convergence](fig09_initial_offset_convergence.pdf).
- Main text or a combined multipanel figure: [fig10_joint_settling](fig10_joint_settling.pdf).
- Main text or a combined multipanel figure: [fig11_openloop_step](fig11_openloop_step.pdf).
- Main text or a combined multipanel figure: [fig13_noisy_model_validation](fig13_noisy_model_validation.pdf).

Suggested supplementary material: paired changes, heading-error distributions, remaining representative trajectories, whole-trial/longitudinal smoothness, correlation diagnostics, historical-route replay, multisine loops, analytical Bode curves, and receding-horizon sampling diagnostics. Preserve the smoothness tradeoff when condensing figures.

## fig01_left_nominal — Left turn, Nominal plant, seed 0

[PDF](fig01_left_nominal.pdf) · [SVG](fig01_left_nominal.svg) · [PNG](fig01_left_nominal.png)

**Caption.** Representative left tracking under the nominal plant. Seed 0 was selected in advance for every geometry/condition, not by performance. (a) Planar path; (b,c) signed true-state lateral/heading errors versus projected progress; (d,e) body (solid) and issued command (dotted) velocities; (f) local reference curvature. Shading denotes |curvature|>0.15 1/m, not obstacle regions. Commands are plotted at t−0.05 s because stored body states are recorded after each integration step. No smoothing/interpolation is applied.

**Sample unit:** One closed-loop trial per method; seed 0; no error bars

**Processing:** SI→mm/deg unit conversion; command timestamps shifted by −dt; connected raw samples only.

**Sources:** `results/tracking_final/left_nominal_full_0.npz`, `results/tracking_final/left_nominal_ideal_0.npz`, `results/tracking_final/left_nominal_native_0.npz`, `results/tracking_final/left_nominal_tracker_0.npz`

## fig01_left_heldout — Left turn, Held-out plant, seed 0

[PDF](fig01_left_heldout.pdf) · [SVG](fig01_left_heldout.svg) · [PNG](fig01_left_heldout.png)

**Caption.** Representative left tracking under the heldout plant. Seed 0 was selected in advance for every geometry/condition, not by performance. (a) Planar path; (b,c) signed true-state lateral/heading errors versus projected progress; (d,e) body (solid) and issued command (dotted) velocities; (f) local reference curvature. Shading denotes |curvature|>0.15 1/m, not obstacle regions. Commands are plotted at t−0.05 s because stored body states are recorded after each integration step. No smoothing/interpolation is applied.

**Sample unit:** One closed-loop trial per method; seed 0; no error bars

**Processing:** SI→mm/deg unit conversion; command timestamps shifted by −dt; connected raw samples only.

**Sources:** `results/tracking_final/left_heldout_full_0.npz`, `results/tracking_final/left_heldout_ideal_0.npz`, `results/tracking_final/left_heldout_native_0.npz`, `results/tracking_final/left_heldout_tracker_0.npz`

## fig01_right_nominal — Right turn, Nominal plant, seed 0

[PDF](fig01_right_nominal.pdf) · [SVG](fig01_right_nominal.svg) · [PNG](fig01_right_nominal.png)

**Caption.** Representative right tracking under the nominal plant. Seed 0 was selected in advance for every geometry/condition, not by performance. (a) Planar path; (b,c) signed true-state lateral/heading errors versus projected progress; (d,e) body (solid) and issued command (dotted) velocities; (f) local reference curvature. Shading denotes |curvature|>0.15 1/m, not obstacle regions. Commands are plotted at t−0.05 s because stored body states are recorded after each integration step. No smoothing/interpolation is applied.

**Sample unit:** One closed-loop trial per method; seed 0; no error bars

**Processing:** SI→mm/deg unit conversion; command timestamps shifted by −dt; connected raw samples only.

**Sources:** `results/tracking_final/right_nominal_full_0.npz`, `results/tracking_final/right_nominal_ideal_0.npz`, `results/tracking_final/right_nominal_native_0.npz`, `results/tracking_final/right_nominal_tracker_0.npz`

## fig01_right_heldout — Right turn, Held-out plant, seed 0

[PDF](fig01_right_heldout.pdf) · [SVG](fig01_right_heldout.svg) · [PNG](fig01_right_heldout.png)

**Caption.** Representative right tracking under the heldout plant. Seed 0 was selected in advance for every geometry/condition, not by performance. (a) Planar path; (b,c) signed true-state lateral/heading errors versus projected progress; (d,e) body (solid) and issued command (dotted) velocities; (f) local reference curvature. Shading denotes |curvature|>0.15 1/m, not obstacle regions. Commands are plotted at t−0.05 s because stored body states are recorded after each integration step. No smoothing/interpolation is applied.

**Sample unit:** One closed-loop trial per method; seed 0; no error bars

**Processing:** SI→mm/deg unit conversion; command timestamps shifted by −dt; connected raw samples only.

**Sources:** `results/tracking_final/right_heldout_full_0.npz`, `results/tracking_final/right_heldout_ideal_0.npz`, `results/tracking_final/right_heldout_native_0.npz`, `results/tracking_final/right_heldout_tracker_0.npz`

## fig01_s_bend_nominal — S bend, Nominal plant, seed 0

[PDF](fig01_s_bend_nominal.pdf) · [SVG](fig01_s_bend_nominal.svg) · [PNG](fig01_s_bend_nominal.png)

**Caption.** Representative s bend tracking under the nominal plant. Seed 0 was selected in advance for every geometry/condition, not by performance. (a) Planar path; (b,c) signed true-state lateral/heading errors versus projected progress; (d,e) body (solid) and issued command (dotted) velocities; (f) local reference curvature. Shading denotes |curvature|>0.15 1/m, not obstacle regions. Commands are plotted at t−0.05 s because stored body states are recorded after each integration step. No smoothing/interpolation is applied.

**Sample unit:** One closed-loop trial per method; seed 0; no error bars

**Processing:** SI→mm/deg unit conversion; command timestamps shifted by −dt; connected raw samples only.

**Sources:** `results/tracking_final/s_bend_nominal_full_0.npz`, `results/tracking_final/s_bend_nominal_ideal_0.npz`, `results/tracking_final/s_bend_nominal_native_0.npz`, `results/tracking_final/s_bend_nominal_tracker_0.npz`

## fig01_s_bend_heldout — S bend, Held-out plant, seed 0

[PDF](fig01_s_bend_heldout.pdf) · [SVG](fig01_s_bend_heldout.svg) · [PNG](fig01_s_bend_heldout.png)

**Caption.** Representative s bend tracking under the heldout plant. Seed 0 was selected in advance for every geometry/condition, not by performance. (a) Planar path; (b,c) signed true-state lateral/heading errors versus projected progress; (d,e) body (solid) and issued command (dotted) velocities; (f) local reference curvature. Shading denotes |curvature|>0.15 1/m, not obstacle regions. Commands are plotted at t−0.05 s because stored body states are recorded after each integration step. No smoothing/interpolation is applied.

**Sample unit:** One closed-loop trial per method; seed 0; no error bars

**Processing:** SI→mm/deg unit conversion; command timestamps shifted by −dt; connected raw samples only.

**Sources:** `results/tracking_final/s_bend_heldout_full_0.npz`, `results/tracking_final/s_bend_heldout_ideal_0.npz`, `results/tracking_final/s_bend_heldout_native_0.npz`, `results/tracking_final/s_bend_heldout_tracker_0.npz`

## fig02_turn_lateral_rmse — Trial-level turn lateral errors

[PDF](fig02_turn_lateral_rmse.pdf) · [SVG](fig02_turn_lateral_rmse.svg) · [PNG](fig02_turn_lateral_rmse.png)

**Caption.** Turn lateral RMSE for all 72 tracking trials. Each dot is one complete trial's turn-only RMSE; each geometry/condition/method has three actual seeds. Black bars denote arithmetic means, not confidence intervals. The same pre-generated state perturbations are paired by geometry/condition/seed. Samples within trajectories are not independent statistical replicates. All four controllers and both plants are retained.

**Sample unit:** 72 trials; n=3 per geometry×condition×method

**Processing:** Turn mask |k|>0.15 1/m; one RMSE per trial; descriptive mean.

**Sources:** `results/tracking_final/left_heldout_full_0.npz`, `results/tracking_final/left_heldout_full_1.npz`, `results/tracking_final/left_heldout_full_2.npz`, `results/tracking_final/left_heldout_ideal_0.npz`, `results/tracking_final/left_heldout_ideal_1.npz`, `results/tracking_final/left_heldout_ideal_2.npz`, `results/tracking_final/left_heldout_native_0.npz`, `results/tracking_final/left_heldout_native_1.npz`, `results/tracking_final/left_heldout_native_2.npz`, `results/tracking_final/left_heldout_tracker_0.npz`, `results/tracking_final/left_heldout_tracker_1.npz`, `results/tracking_final/left_heldout_tracker_2.npz`, `results/tracking_final/left_nominal_full_0.npz`, `results/tracking_final/left_nominal_full_1.npz`, `results/tracking_final/left_nominal_full_2.npz`, `results/tracking_final/left_nominal_ideal_0.npz`, `results/tracking_final/left_nominal_ideal_1.npz`, `results/tracking_final/left_nominal_ideal_2.npz`, `results/tracking_final/left_nominal_native_0.npz`, `results/tracking_final/left_nominal_native_1.npz`, `results/tracking_final/left_nominal_native_2.npz`, `results/tracking_final/left_nominal_tracker_0.npz`, `results/tracking_final/left_nominal_tracker_1.npz`, `results/tracking_final/left_nominal_tracker_2.npz`, `results/tracking_final/right_heldout_full_0.npz`, `results/tracking_final/right_heldout_full_1.npz`, `results/tracking_final/right_heldout_full_2.npz`, `results/tracking_final/right_heldout_ideal_0.npz`, `results/tracking_final/right_heldout_ideal_1.npz`, `results/tracking_final/right_heldout_ideal_2.npz`, `results/tracking_final/right_heldout_native_0.npz`, `results/tracking_final/right_heldout_native_1.npz`, `results/tracking_final/right_heldout_native_2.npz`, `results/tracking_final/right_heldout_tracker_0.npz`, `results/tracking_final/right_heldout_tracker_1.npz`, `results/tracking_final/right_heldout_tracker_2.npz`, `results/tracking_final/right_nominal_full_0.npz`, `results/tracking_final/right_nominal_full_1.npz`, `results/tracking_final/right_nominal_full_2.npz`, `results/tracking_final/right_nominal_ideal_0.npz`, `results/tracking_final/right_nominal_ideal_1.npz`, `results/tracking_final/right_nominal_ideal_2.npz`, `results/tracking_final/right_nominal_native_0.npz`, `results/tracking_final/right_nominal_native_1.npz`, `results/tracking_final/right_nominal_native_2.npz`, `results/tracking_final/right_nominal_tracker_0.npz`, `results/tracking_final/right_nominal_tracker_1.npz`, `results/tracking_final/right_nominal_tracker_2.npz`, `results/tracking_final/s_bend_heldout_full_0.npz`, `results/tracking_final/s_bend_heldout_full_1.npz`, `results/tracking_final/s_bend_heldout_full_2.npz`, `results/tracking_final/s_bend_heldout_ideal_0.npz`, `results/tracking_final/s_bend_heldout_ideal_1.npz`, `results/tracking_final/s_bend_heldout_ideal_2.npz`, `results/tracking_final/s_bend_heldout_native_0.npz`, `results/tracking_final/s_bend_heldout_native_1.npz`, `results/tracking_final/s_bend_heldout_native_2.npz`, `results/tracking_final/s_bend_heldout_tracker_0.npz`, `results/tracking_final/s_bend_heldout_tracker_1.npz`, `results/tracking_final/s_bend_heldout_tracker_2.npz`, `results/tracking_final/s_bend_nominal_full_0.npz`, `results/tracking_final/s_bend_nominal_full_1.npz`, `results/tracking_final/s_bend_nominal_full_2.npz`, `results/tracking_final/s_bend_nominal_ideal_0.npz`, `results/tracking_final/s_bend_nominal_ideal_1.npz`, `results/tracking_final/s_bend_nominal_ideal_2.npz`, `results/tracking_final/s_bend_nominal_native_0.npz`, `results/tracking_final/s_bend_nominal_native_1.npz`, `results/tracking_final/s_bend_nominal_native_2.npz`, `results/tracking_final/s_bend_nominal_tracker_0.npz`, `results/tracking_final/s_bend_nominal_tracker_1.npz`, `results/tracking_final/s_bend_nominal_tracker_2.npz`

## fig03_turn_heading_rmse — Trial-level turn heading errors

[PDF](fig03_turn_heading_rmse.pdf) · [SVG](fig03_turn_heading_rmse.svg) · [PNG](fig03_turn_heading_rmse.png)

**Caption.** Turn heading RMSE for all 72 tracking trials. Each dot is one complete trial's turn-only RMSE; each geometry/condition/method has three actual seeds. Black bars denote arithmetic means, not confidence intervals. The same pre-generated state perturbations are paired by geometry/condition/seed. Samples within trajectories are not independent statistical replicates. All four controllers and both plants are retained.

**Sample unit:** 72 trials; n=3 per geometry×condition×method

**Processing:** Turn mask |k|>0.15 1/m; one RMSE per trial; descriptive mean.

**Sources:** `results/tracking_final/left_heldout_full_0.npz`, `results/tracking_final/left_heldout_full_1.npz`, `results/tracking_final/left_heldout_full_2.npz`, `results/tracking_final/left_heldout_ideal_0.npz`, `results/tracking_final/left_heldout_ideal_1.npz`, `results/tracking_final/left_heldout_ideal_2.npz`, `results/tracking_final/left_heldout_native_0.npz`, `results/tracking_final/left_heldout_native_1.npz`, `results/tracking_final/left_heldout_native_2.npz`, `results/tracking_final/left_heldout_tracker_0.npz`, `results/tracking_final/left_heldout_tracker_1.npz`, `results/tracking_final/left_heldout_tracker_2.npz`, `results/tracking_final/left_nominal_full_0.npz`, `results/tracking_final/left_nominal_full_1.npz`, `results/tracking_final/left_nominal_full_2.npz`, `results/tracking_final/left_nominal_ideal_0.npz`, `results/tracking_final/left_nominal_ideal_1.npz`, `results/tracking_final/left_nominal_ideal_2.npz`, `results/tracking_final/left_nominal_native_0.npz`, `results/tracking_final/left_nominal_native_1.npz`, `results/tracking_final/left_nominal_native_2.npz`, `results/tracking_final/left_nominal_tracker_0.npz`, `results/tracking_final/left_nominal_tracker_1.npz`, `results/tracking_final/left_nominal_tracker_2.npz`, `results/tracking_final/right_heldout_full_0.npz`, `results/tracking_final/right_heldout_full_1.npz`, `results/tracking_final/right_heldout_full_2.npz`, `results/tracking_final/right_heldout_ideal_0.npz`, `results/tracking_final/right_heldout_ideal_1.npz`, `results/tracking_final/right_heldout_ideal_2.npz`, `results/tracking_final/right_heldout_native_0.npz`, `results/tracking_final/right_heldout_native_1.npz`, `results/tracking_final/right_heldout_native_2.npz`, `results/tracking_final/right_heldout_tracker_0.npz`, `results/tracking_final/right_heldout_tracker_1.npz`, `results/tracking_final/right_heldout_tracker_2.npz`, `results/tracking_final/right_nominal_full_0.npz`, `results/tracking_final/right_nominal_full_1.npz`, `results/tracking_final/right_nominal_full_2.npz`, `results/tracking_final/right_nominal_ideal_0.npz`, `results/tracking_final/right_nominal_ideal_1.npz`, `results/tracking_final/right_nominal_ideal_2.npz`, `results/tracking_final/right_nominal_native_0.npz`, `results/tracking_final/right_nominal_native_1.npz`, `results/tracking_final/right_nominal_native_2.npz`, `results/tracking_final/right_nominal_tracker_0.npz`, `results/tracking_final/right_nominal_tracker_1.npz`, `results/tracking_final/right_nominal_tracker_2.npz`, `results/tracking_final/s_bend_heldout_full_0.npz`, `results/tracking_final/s_bend_heldout_full_1.npz`, `results/tracking_final/s_bend_heldout_full_2.npz`, `results/tracking_final/s_bend_heldout_ideal_0.npz`, `results/tracking_final/s_bend_heldout_ideal_1.npz`, `results/tracking_final/s_bend_heldout_ideal_2.npz`, `results/tracking_final/s_bend_heldout_native_0.npz`, `results/tracking_final/s_bend_heldout_native_1.npz`, `results/tracking_final/s_bend_heldout_native_2.npz`, `results/tracking_final/s_bend_heldout_tracker_0.npz`, `results/tracking_final/s_bend_heldout_tracker_1.npz`, `results/tracking_final/s_bend_heldout_tracker_2.npz`, `results/tracking_final/s_bend_nominal_full_0.npz`, `results/tracking_final/s_bend_nominal_full_1.npz`, `results/tracking_final/s_bend_nominal_full_2.npz`, `results/tracking_final/s_bend_nominal_ideal_0.npz`, `results/tracking_final/s_bend_nominal_ideal_1.npz`, `results/tracking_final/s_bend_nominal_ideal_2.npz`, `results/tracking_final/s_bend_nominal_native_0.npz`, `results/tracking_final/s_bend_nominal_native_1.npz`, `results/tracking_final/s_bend_nominal_native_2.npz`, `results/tracking_final/s_bend_nominal_tracker_0.npz`, `results/tracking_final/s_bend_nominal_tracker_1.npz`, `results/tracking_final/s_bend_nominal_tracker_2.npz`

## fig02b_paired_turn_changes — Paired change in turn lateral RMSE

[PDF](fig02b_paired_turn_changes.pdf) · [SVG](fig02b_paired_turn_changes.svg) · [PNG](fig02b_paired_turn_changes.png)

**Caption.** Paired turn lateral RMSE changes: Guided MPC minus each comparison controller using matched geometry, plant, and seed. Each dot is one pair and each black bar the mean of three pairs within a geometry; negative values favor Guided MPC. All 54 paired comparisons (18 pairs against each comparator) are retained, including nominal cases where the tracker is more accurate. Pairing shares pre-generated state perturbations but does not make the native implementation bitwise deterministic. No inferential p-values or confidence intervals are shown.

**Sample unit:** 54 matched comparisons; three pairs per geometry×condition×comparator

**Processing:** Subtract whole-trial turn RMSE values only after pairing geometry/condition/seed; mean of three pairs.

**Sources:** `results/tracking_final/left_heldout_full_0.npz`, `results/tracking_final/left_heldout_full_1.npz`, `results/tracking_final/left_heldout_full_2.npz`, `results/tracking_final/left_heldout_ideal_0.npz`, `results/tracking_final/left_heldout_ideal_1.npz`, `results/tracking_final/left_heldout_ideal_2.npz`, `results/tracking_final/left_heldout_native_0.npz`, `results/tracking_final/left_heldout_native_1.npz`, `results/tracking_final/left_heldout_native_2.npz`, `results/tracking_final/left_heldout_tracker_0.npz`, `results/tracking_final/left_heldout_tracker_1.npz`, `results/tracking_final/left_heldout_tracker_2.npz`, `results/tracking_final/left_nominal_full_0.npz`, `results/tracking_final/left_nominal_full_1.npz`, `results/tracking_final/left_nominal_full_2.npz`, `results/tracking_final/left_nominal_ideal_0.npz`, `results/tracking_final/left_nominal_ideal_1.npz`, `results/tracking_final/left_nominal_ideal_2.npz`, `results/tracking_final/left_nominal_native_0.npz`, `results/tracking_final/left_nominal_native_1.npz`, `results/tracking_final/left_nominal_native_2.npz`, `results/tracking_final/left_nominal_tracker_0.npz`, `results/tracking_final/left_nominal_tracker_1.npz`, `results/tracking_final/left_nominal_tracker_2.npz`, `results/tracking_final/right_heldout_full_0.npz`, `results/tracking_final/right_heldout_full_1.npz`, `results/tracking_final/right_heldout_full_2.npz`, `results/tracking_final/right_heldout_ideal_0.npz`, `results/tracking_final/right_heldout_ideal_1.npz`, `results/tracking_final/right_heldout_ideal_2.npz`, `results/tracking_final/right_heldout_native_0.npz`, `results/tracking_final/right_heldout_native_1.npz`, `results/tracking_final/right_heldout_native_2.npz`, `results/tracking_final/right_heldout_tracker_0.npz`, `results/tracking_final/right_heldout_tracker_1.npz`, `results/tracking_final/right_heldout_tracker_2.npz`, `results/tracking_final/right_nominal_full_0.npz`, `results/tracking_final/right_nominal_full_1.npz`, `results/tracking_final/right_nominal_full_2.npz`, `results/tracking_final/right_nominal_ideal_0.npz`, `results/tracking_final/right_nominal_ideal_1.npz`, `results/tracking_final/right_nominal_ideal_2.npz`, `results/tracking_final/right_nominal_native_0.npz`, `results/tracking_final/right_nominal_native_1.npz`, `results/tracking_final/right_nominal_native_2.npz`, `results/tracking_final/right_nominal_tracker_0.npz`, `results/tracking_final/right_nominal_tracker_1.npz`, `results/tracking_final/right_nominal_tracker_2.npz`, `results/tracking_final/s_bend_heldout_full_0.npz`, `results/tracking_final/s_bend_heldout_full_1.npz`, `results/tracking_final/s_bend_heldout_full_2.npz`, `results/tracking_final/s_bend_heldout_ideal_0.npz`, `results/tracking_final/s_bend_heldout_ideal_1.npz`, `results/tracking_final/s_bend_heldout_ideal_2.npz`, `results/tracking_final/s_bend_heldout_native_0.npz`, `results/tracking_final/s_bend_heldout_native_1.npz`, `results/tracking_final/s_bend_heldout_native_2.npz`, `results/tracking_final/s_bend_heldout_tracker_0.npz`, `results/tracking_final/s_bend_heldout_tracker_1.npz`, `results/tracking_final/s_bend_heldout_tracker_2.npz`, `results/tracking_final/s_bend_nominal_full_0.npz`, `results/tracking_final/s_bend_nominal_full_1.npz`, `results/tracking_final/s_bend_nominal_full_2.npz`, `results/tracking_final/s_bend_nominal_ideal_0.npz`, `results/tracking_final/s_bend_nominal_ideal_1.npz`, `results/tracking_final/s_bend_nominal_ideal_2.npz`, `results/tracking_final/s_bend_nominal_native_0.npz`, `results/tracking_final/s_bend_nominal_native_1.npz`, `results/tracking_final/s_bend_nominal_native_2.npz`, `results/tracking_final/s_bend_nominal_tracker_0.npz`, `results/tracking_final/s_bend_nominal_tracker_1.npz`, `results/tracking_final/s_bend_nominal_tracker_2.npz`

## fig04_phase_errors — Tracking errors by turn phase

[PDF](fig04_phase_errors.pdf) · [SVG](fig04_phase_errors.svg) · [PNG](fig04_phase_errors.png)

**Caption.** Tracking error by geometric phase. Markers are the mean of nine trial-level RMSE values per method/plant (three geometries×three seeds); whiskers show the observed minimum–maximum, not uncertainty intervals. Turn: |κ|>0.15 1/m; entry/exit: sign(κ)dκ/ds >0.4 / <−0.4 1/m²; remaining turn samples form the apex. The finite path geometry and existing benchmark phase labels are retained.

**Sample unit:** n=9 trial summaries per condition×method×phase; geometries pooled descriptively

**Processing:** Reuse benchmark geometric phase masks; per-trial RMSE then mean/min/max across trials.

**Sources:** `results/tracking_final/left_heldout_full_0.npz`, `results/tracking_final/left_heldout_full_1.npz`, `results/tracking_final/left_heldout_full_2.npz`, `results/tracking_final/left_heldout_ideal_0.npz`, `results/tracking_final/left_heldout_ideal_1.npz`, `results/tracking_final/left_heldout_ideal_2.npz`, `results/tracking_final/left_heldout_native_0.npz`, `results/tracking_final/left_heldout_native_1.npz`, `results/tracking_final/left_heldout_native_2.npz`, `results/tracking_final/left_heldout_tracker_0.npz`, `results/tracking_final/left_heldout_tracker_1.npz`, `results/tracking_final/left_heldout_tracker_2.npz`, `results/tracking_final/left_nominal_full_0.npz`, `results/tracking_final/left_nominal_full_1.npz`, `results/tracking_final/left_nominal_full_2.npz`, `results/tracking_final/left_nominal_ideal_0.npz`, `results/tracking_final/left_nominal_ideal_1.npz`, `results/tracking_final/left_nominal_ideal_2.npz`, `results/tracking_final/left_nominal_native_0.npz`, `results/tracking_final/left_nominal_native_1.npz`, `results/tracking_final/left_nominal_native_2.npz`, `results/tracking_final/left_nominal_tracker_0.npz`, `results/tracking_final/left_nominal_tracker_1.npz`, `results/tracking_final/left_nominal_tracker_2.npz`, `results/tracking_final/right_heldout_full_0.npz`, `results/tracking_final/right_heldout_full_1.npz`, `results/tracking_final/right_heldout_full_2.npz`, `results/tracking_final/right_heldout_ideal_0.npz`, `results/tracking_final/right_heldout_ideal_1.npz`, `results/tracking_final/right_heldout_ideal_2.npz`, `results/tracking_final/right_heldout_native_0.npz`, `results/tracking_final/right_heldout_native_1.npz`, `results/tracking_final/right_heldout_native_2.npz`, `results/tracking_final/right_heldout_tracker_0.npz`, `results/tracking_final/right_heldout_tracker_1.npz`, `results/tracking_final/right_heldout_tracker_2.npz`, `results/tracking_final/right_nominal_full_0.npz`, `results/tracking_final/right_nominal_full_1.npz`, `results/tracking_final/right_nominal_full_2.npz`, `results/tracking_final/right_nominal_ideal_0.npz`, `results/tracking_final/right_nominal_ideal_1.npz`, `results/tracking_final/right_nominal_ideal_2.npz`, `results/tracking_final/right_nominal_native_0.npz`, `results/tracking_final/right_nominal_native_1.npz`, `results/tracking_final/right_nominal_native_2.npz`, `results/tracking_final/right_nominal_tracker_0.npz`, `results/tracking_final/right_nominal_tracker_1.npz`, `results/tracking_final/right_nominal_tracker_2.npz`, `results/tracking_final/s_bend_heldout_full_0.npz`, `results/tracking_final/s_bend_heldout_full_1.npz`, `results/tracking_final/s_bend_heldout_full_2.npz`, `results/tracking_final/s_bend_heldout_ideal_0.npz`, `results/tracking_final/s_bend_heldout_ideal_1.npz`, `results/tracking_final/s_bend_heldout_ideal_2.npz`, `results/tracking_final/s_bend_heldout_native_0.npz`, `results/tracking_final/s_bend_heldout_native_1.npz`, `results/tracking_final/s_bend_heldout_native_2.npz`, `results/tracking_final/s_bend_heldout_tracker_0.npz`, `results/tracking_final/s_bend_heldout_tracker_1.npz`, `results/tracking_final/s_bend_heldout_tracker_2.npz`, `results/tracking_final/s_bend_nominal_full_0.npz`, `results/tracking_final/s_bend_nominal_full_1.npz`, `results/tracking_final/s_bend_nominal_full_2.npz`, `results/tracking_final/s_bend_nominal_ideal_0.npz`, `results/tracking_final/s_bend_nominal_ideal_1.npz`, `results/tracking_final/s_bend_nominal_ideal_2.npz`, `results/tracking_final/s_bend_nominal_native_0.npz`, `results/tracking_final/s_bend_nominal_native_1.npz`, `results/tracking_final/s_bend_nominal_native_2.npz`, `results/tracking_final/s_bend_nominal_tracker_0.npz`, `results/tracking_final/s_bend_nominal_tracker_1.npz`, `results/tracking_final/s_bend_nominal_tracker_2.npz`

## fig05_angular_smoothness — Angular command/body smoothness

[PDF](fig05_angular_smoothness.pdf) · [SVG](fig05_angular_smoothness.svg) · [PNG](fig05_angular_smoothness.png)

**Caption.** Angular smoothness for all tracking trials. Each dot is a whole-trial statistic; black bars are means of nine trials per method/plant. Columns distinguish raw command-increment RMS (Δu), command-rate RMS (Δu/Δt), and body-velocity second-difference RMS (Δ²v/Δt², a numerical jerk estimate). Δt=0.05 s; differences use recorded samples, no filtering and no artificial prepended zero; startup is included. Command increment has velocity units and is not acceleration. Body jerk is a simulated finite-difference diagnostic, not measured mechanical jerk.

**Sample unit:** n=9 whole-trial statistics per condition×method

**Processing:** First/second finite differences at 20 Hz; whole-trial RMS; no smoothing/prepended values.

**Sources:** `results/tracking_final/left_heldout_full_0.npz`, `results/tracking_final/left_heldout_full_1.npz`, `results/tracking_final/left_heldout_full_2.npz`, `results/tracking_final/left_heldout_ideal_0.npz`, `results/tracking_final/left_heldout_ideal_1.npz`, `results/tracking_final/left_heldout_ideal_2.npz`, `results/tracking_final/left_heldout_native_0.npz`, `results/tracking_final/left_heldout_native_1.npz`, `results/tracking_final/left_heldout_native_2.npz`, `results/tracking_final/left_heldout_tracker_0.npz`, `results/tracking_final/left_heldout_tracker_1.npz`, `results/tracking_final/left_heldout_tracker_2.npz`, `results/tracking_final/left_nominal_full_0.npz`, `results/tracking_final/left_nominal_full_1.npz`, `results/tracking_final/left_nominal_full_2.npz`, `results/tracking_final/left_nominal_ideal_0.npz`, `results/tracking_final/left_nominal_ideal_1.npz`, `results/tracking_final/left_nominal_ideal_2.npz`, `results/tracking_final/left_nominal_native_0.npz`, `results/tracking_final/left_nominal_native_1.npz`, `results/tracking_final/left_nominal_native_2.npz`, `results/tracking_final/left_nominal_tracker_0.npz`, `results/tracking_final/left_nominal_tracker_1.npz`, `results/tracking_final/left_nominal_tracker_2.npz`, `results/tracking_final/right_heldout_full_0.npz`, `results/tracking_final/right_heldout_full_1.npz`, `results/tracking_final/right_heldout_full_2.npz`, `results/tracking_final/right_heldout_ideal_0.npz`, `results/tracking_final/right_heldout_ideal_1.npz`, `results/tracking_final/right_heldout_ideal_2.npz`, `results/tracking_final/right_heldout_native_0.npz`, `results/tracking_final/right_heldout_native_1.npz`, `results/tracking_final/right_heldout_native_2.npz`, `results/tracking_final/right_heldout_tracker_0.npz`, `results/tracking_final/right_heldout_tracker_1.npz`, `results/tracking_final/right_heldout_tracker_2.npz`, `results/tracking_final/right_nominal_full_0.npz`, `results/tracking_final/right_nominal_full_1.npz`, `results/tracking_final/right_nominal_full_2.npz`, `results/tracking_final/right_nominal_ideal_0.npz`, `results/tracking_final/right_nominal_ideal_1.npz`, `results/tracking_final/right_nominal_ideal_2.npz`, `results/tracking_final/right_nominal_native_0.npz`, `results/tracking_final/right_nominal_native_1.npz`, `results/tracking_final/right_nominal_native_2.npz`, `results/tracking_final/right_nominal_tracker_0.npz`, `results/tracking_final/right_nominal_tracker_1.npz`, `results/tracking_final/right_nominal_tracker_2.npz`, `results/tracking_final/s_bend_heldout_full_0.npz`, `results/tracking_final/s_bend_heldout_full_1.npz`, `results/tracking_final/s_bend_heldout_full_2.npz`, `results/tracking_final/s_bend_heldout_ideal_0.npz`, `results/tracking_final/s_bend_heldout_ideal_1.npz`, `results/tracking_final/s_bend_heldout_ideal_2.npz`, `results/tracking_final/s_bend_heldout_native_0.npz`, `results/tracking_final/s_bend_heldout_native_1.npz`, `results/tracking_final/s_bend_heldout_native_2.npz`, `results/tracking_final/s_bend_heldout_tracker_0.npz`, `results/tracking_final/s_bend_heldout_tracker_1.npz`, `results/tracking_final/s_bend_heldout_tracker_2.npz`, `results/tracking_final/s_bend_nominal_full_0.npz`, `results/tracking_final/s_bend_nominal_full_1.npz`, `results/tracking_final/s_bend_nominal_full_2.npz`, `results/tracking_final/s_bend_nominal_ideal_0.npz`, `results/tracking_final/s_bend_nominal_ideal_1.npz`, `results/tracking_final/s_bend_nominal_ideal_2.npz`, `results/tracking_final/s_bend_nominal_native_0.npz`, `results/tracking_final/s_bend_nominal_native_1.npz`, `results/tracking_final/s_bend_nominal_native_2.npz`, `results/tracking_final/s_bend_nominal_tracker_0.npz`, `results/tracking_final/s_bend_nominal_tracker_1.npz`, `results/tracking_final/s_bend_nominal_tracker_2.npz`

## fig05_longitudinal_smoothness — Longitudinal command/body smoothness

[PDF](fig05_longitudinal_smoothness.pdf) · [SVG](fig05_longitudinal_smoothness.svg) · [PNG](fig05_longitudinal_smoothness.png)

**Caption.** Longitudinal smoothness for all tracking trials. Each dot is a whole-trial statistic; black bars are means of nine trials per method/plant. Columns distinguish raw command-increment RMS (Δu), command-rate RMS (Δu/Δt), and body-velocity second-difference RMS (Δ²v/Δt², a numerical jerk estimate). Δt=0.05 s; differences use recorded samples, no filtering and no artificial prepended zero; startup is included. Command increment has velocity units and is not acceleration. Body jerk is a simulated finite-difference diagnostic, not measured mechanical jerk.

**Sample unit:** n=9 whole-trial statistics per condition×method

**Processing:** First/second finite differences at 20 Hz; whole-trial RMS; no smoothing/prepended values.

**Sources:** `results/tracking_final/left_heldout_full_0.npz`, `results/tracking_final/left_heldout_full_1.npz`, `results/tracking_final/left_heldout_full_2.npz`, `results/tracking_final/left_heldout_ideal_0.npz`, `results/tracking_final/left_heldout_ideal_1.npz`, `results/tracking_final/left_heldout_ideal_2.npz`, `results/tracking_final/left_heldout_native_0.npz`, `results/tracking_final/left_heldout_native_1.npz`, `results/tracking_final/left_heldout_native_2.npz`, `results/tracking_final/left_heldout_tracker_0.npz`, `results/tracking_final/left_heldout_tracker_1.npz`, `results/tracking_final/left_heldout_tracker_2.npz`, `results/tracking_final/left_nominal_full_0.npz`, `results/tracking_final/left_nominal_full_1.npz`, `results/tracking_final/left_nominal_full_2.npz`, `results/tracking_final/left_nominal_ideal_0.npz`, `results/tracking_final/left_nominal_ideal_1.npz`, `results/tracking_final/left_nominal_ideal_2.npz`, `results/tracking_final/left_nominal_native_0.npz`, `results/tracking_final/left_nominal_native_1.npz`, `results/tracking_final/left_nominal_native_2.npz`, `results/tracking_final/left_nominal_tracker_0.npz`, `results/tracking_final/left_nominal_tracker_1.npz`, `results/tracking_final/left_nominal_tracker_2.npz`, `results/tracking_final/right_heldout_full_0.npz`, `results/tracking_final/right_heldout_full_1.npz`, `results/tracking_final/right_heldout_full_2.npz`, `results/tracking_final/right_heldout_ideal_0.npz`, `results/tracking_final/right_heldout_ideal_1.npz`, `results/tracking_final/right_heldout_ideal_2.npz`, `results/tracking_final/right_heldout_native_0.npz`, `results/tracking_final/right_heldout_native_1.npz`, `results/tracking_final/right_heldout_native_2.npz`, `results/tracking_final/right_heldout_tracker_0.npz`, `results/tracking_final/right_heldout_tracker_1.npz`, `results/tracking_final/right_heldout_tracker_2.npz`, `results/tracking_final/right_nominal_full_0.npz`, `results/tracking_final/right_nominal_full_1.npz`, `results/tracking_final/right_nominal_full_2.npz`, `results/tracking_final/right_nominal_ideal_0.npz`, `results/tracking_final/right_nominal_ideal_1.npz`, `results/tracking_final/right_nominal_ideal_2.npz`, `results/tracking_final/right_nominal_native_0.npz`, `results/tracking_final/right_nominal_native_1.npz`, `results/tracking_final/right_nominal_native_2.npz`, `results/tracking_final/right_nominal_tracker_0.npz`, `results/tracking_final/right_nominal_tracker_1.npz`, `results/tracking_final/right_nominal_tracker_2.npz`, `results/tracking_final/s_bend_heldout_full_0.npz`, `results/tracking_final/s_bend_heldout_full_1.npz`, `results/tracking_final/s_bend_heldout_full_2.npz`, `results/tracking_final/s_bend_heldout_ideal_0.npz`, `results/tracking_final/s_bend_heldout_ideal_1.npz`, `results/tracking_final/s_bend_heldout_ideal_2.npz`, `results/tracking_final/s_bend_heldout_native_0.npz`, `results/tracking_final/s_bend_heldout_native_1.npz`, `results/tracking_final/s_bend_heldout_native_2.npz`, `results/tracking_final/s_bend_heldout_tracker_0.npz`, `results/tracking_final/s_bend_heldout_tracker_1.npz`, `results/tracking_final/s_bend_heldout_tracker_2.npz`, `results/tracking_final/s_bend_nominal_full_0.npz`, `results/tracking_final/s_bend_nominal_full_1.npz`, `results/tracking_final/s_bend_nominal_full_2.npz`, `results/tracking_final/s_bend_nominal_ideal_0.npz`, `results/tracking_final/s_bend_nominal_ideal_1.npz`, `results/tracking_final/s_bend_nominal_ideal_2.npz`, `results/tracking_final/s_bend_nominal_native_0.npz`, `results/tracking_final/s_bend_nominal_native_1.npz`, `results/tracking_final/s_bend_nominal_native_2.npz`, `results/tracking_final/s_bend_nominal_tracker_0.npz`, `results/tracking_final/s_bend_nominal_tracker_1.npz`, `results/tracking_final/s_bend_nominal_tracker_2.npz`

## fig05b_turn_angular_smoothness — Turn-only angular smoothness

[PDF](fig05b_turn_angular_smoothness.pdf) · [SVG](fig05b_turn_angular_smoothness.svg) · [PNG](fig05b_turn_angular_smoothness.png)

**Caption.** Turn-only angular smoothness, complementary to the whole-trial smoothness plots. Each dot is one trial's statistic on samples with |κ|>0.15 1/m and each black bar the mean of nine trials per method/plant. Columns distinguish command increment, command rate, and numerical body jerk with different physical units. Finite differences are formed on the original regularly sampled trajectory before phase restriction, using only the valid within-phase intervals specified in the metric exporter, rather than differencing across disjoint turn segments. No filtering is applied. The accuracy gain must be read together with its command/body smoothness tradeoff.

**Sample unit:** 72 trial-level turn summaries; n=9 per method×condition

**Processing:** Use exported turn-only finite-difference RMS metrics; descriptive mean; no smoothing.

**Sources:** `paper/data/trial_metrics.csv`

## fig05b_turn_longitudinal_smoothness — Turn-only longitudinal smoothness

[PDF](fig05b_turn_longitudinal_smoothness.pdf) · [SVG](fig05b_turn_longitudinal_smoothness.svg) · [PNG](fig05b_turn_longitudinal_smoothness.png)

**Caption.** Turn-only longitudinal smoothness, complementary to the whole-trial smoothness plots. Each dot is one trial's statistic on samples with |κ|>0.15 1/m and each black bar the mean of nine trials per method/plant. Columns distinguish command increment, command rate, and numerical body jerk with different physical units. Finite differences are formed on the original regularly sampled trajectory before phase restriction, using only the valid within-phase intervals specified in the metric exporter, rather than differencing across disjoint turn segments. No filtering is applied. The accuracy gain must be read together with its command/body smoothness tradeoff.

**Sample unit:** 72 trial-level turn summaries; n=9 per method×condition

**Processing:** Use exported turn-only finite-difference RMS metrics; descriptive mean; no smoothing.

**Sources:** `paper/data/trial_metrics.csv`

## fig05c_turn_planar_smoothness — Turn-only planar motion smoothness

[PDF](fig05c_turn_planar_smoothness.pdf) · [SVG](fig05c_turn_planar_smoothness.svg) · [PNG](fig05c_turn_planar_smoothness.png)

**Caption.** Turn-only planar motion smoothness from the simulated world-frame velocity vector [v cos(yaw), v sin(yaw)]. The first and second chronological finite differences estimate planar acceleration and jerk, respectively; their Euclidean-norm RMS values include directional changes and centripetal effects omitted by a scalar-speed derivative. Only consecutive valid within-turn samples enter each derivative. Each dot is one trial and black bars show means over nine trials per method/plant. These are finite-difference simulation diagnostics, not measured ride-comfort or hardware load tests.

**Sample unit:** 72 trial-level turn summaries; n=9 per method×condition

**Processing:** Use exported world-frame vector finite-difference norm RMS on valid within-turn samples; no filter.

**Sources:** `paper/data/trial_metrics.csv`

## fig06_runtime — Measured offline computation time

[PDF](fig06_runtime.pdf) · [SVG](fig06_runtime.svg) · [PNG](fig06_runtime.png)

**Caption.** Offline computation time: upper panels show descriptive empirical CDFs of recorded controller calls pooled within each method/plant; lower panels preserve the independent trial unit with nine trial P95 values and their arithmetic mean. The dashed 50 ms line is the nominal 20 Hz controller period. Calls within one trial are dependent. Measured wall-clock cost was not fed back as additional plant delay; this is not a real-time scheduling guarantee.

**Sample unit:** Raw calls for descriptive ECDF; n=9 trial P95 values per condition×method

**Processing:** Seconds→milliseconds; raw-call ECDF; per-trial P95; no runtime trimming.

**Sources:** `results/tracking_final/left_heldout_full_0.npz`, `results/tracking_final/left_heldout_full_1.npz`, `results/tracking_final/left_heldout_full_2.npz`, `results/tracking_final/left_heldout_ideal_0.npz`, `results/tracking_final/left_heldout_ideal_1.npz`, `results/tracking_final/left_heldout_ideal_2.npz`, `results/tracking_final/left_heldout_native_0.npz`, `results/tracking_final/left_heldout_native_1.npz`, `results/tracking_final/left_heldout_native_2.npz`, `results/tracking_final/left_heldout_tracker_0.npz`, `results/tracking_final/left_heldout_tracker_1.npz`, `results/tracking_final/left_heldout_tracker_2.npz`, `results/tracking_final/left_nominal_full_0.npz`, `results/tracking_final/left_nominal_full_1.npz`, `results/tracking_final/left_nominal_full_2.npz`, `results/tracking_final/left_nominal_ideal_0.npz`, `results/tracking_final/left_nominal_ideal_1.npz`, `results/tracking_final/left_nominal_ideal_2.npz`, `results/tracking_final/left_nominal_native_0.npz`, `results/tracking_final/left_nominal_native_1.npz`, `results/tracking_final/left_nominal_native_2.npz`, `results/tracking_final/left_nominal_tracker_0.npz`, `results/tracking_final/left_nominal_tracker_1.npz`, `results/tracking_final/left_nominal_tracker_2.npz`, `results/tracking_final/right_heldout_full_0.npz`, `results/tracking_final/right_heldout_full_1.npz`, `results/tracking_final/right_heldout_full_2.npz`, `results/tracking_final/right_heldout_ideal_0.npz`, `results/tracking_final/right_heldout_ideal_1.npz`, `results/tracking_final/right_heldout_ideal_2.npz`, `results/tracking_final/right_heldout_native_0.npz`, `results/tracking_final/right_heldout_native_1.npz`, `results/tracking_final/right_heldout_native_2.npz`, `results/tracking_final/right_heldout_tracker_0.npz`, `results/tracking_final/right_heldout_tracker_1.npz`, `results/tracking_final/right_heldout_tracker_2.npz`, `results/tracking_final/right_nominal_full_0.npz`, `results/tracking_final/right_nominal_full_1.npz`, `results/tracking_final/right_nominal_full_2.npz`, `results/tracking_final/right_nominal_ideal_0.npz`, `results/tracking_final/right_nominal_ideal_1.npz`, `results/tracking_final/right_nominal_ideal_2.npz`, `results/tracking_final/right_nominal_native_0.npz`, `results/tracking_final/right_nominal_native_1.npz`, `results/tracking_final/right_nominal_native_2.npz`, `results/tracking_final/right_nominal_tracker_0.npz`, `results/tracking_final/right_nominal_tracker_1.npz`, `results/tracking_final/right_nominal_tracker_2.npz`, `results/tracking_final/s_bend_heldout_full_0.npz`, `results/tracking_final/s_bend_heldout_full_1.npz`, `results/tracking_final/s_bend_heldout_full_2.npz`, `results/tracking_final/s_bend_heldout_ideal_0.npz`, `results/tracking_final/s_bend_heldout_ideal_1.npz`, `results/tracking_final/s_bend_heldout_ideal_2.npz`, `results/tracking_final/s_bend_heldout_native_0.npz`, `results/tracking_final/s_bend_heldout_native_1.npz`, `results/tracking_final/s_bend_heldout_native_2.npz`, `results/tracking_final/s_bend_heldout_tracker_0.npz`, `results/tracking_final/s_bend_heldout_tracker_1.npz`, `results/tracking_final/s_bend_heldout_tracker_2.npz`, `results/tracking_final/s_bend_nominal_full_0.npz`, `results/tracking_final/s_bend_nominal_full_1.npz`, `results/tracking_final/s_bend_nominal_full_2.npz`, `results/tracking_final/s_bend_nominal_ideal_0.npz`, `results/tracking_final/s_bend_nominal_ideal_1.npz`, `results/tracking_final/s_bend_nominal_ideal_2.npz`, `results/tracking_final/s_bend_nominal_native_0.npz`, `results/tracking_final/s_bend_nominal_native_1.npz`, `results/tracking_final/s_bend_nominal_native_2.npz`, `results/tracking_final/s_bend_nominal_tracker_0.npz`, `results/tracking_final/s_bend_nominal_tracker_1.npz`, `results/tracking_final/s_bend_nominal_tracker_2.npz`

## fig07_lag_nominal — Angular response/lag association, nominal

[PDF](fig07_lag_nominal.pdf) · [SVG](fig07_lag_nominal.svg) · [PNG](fig07_lag_nominal.png)

**Caption.** Seed-0 S-bend angular-response diagnostic under the nominal plant. Upper row compares target-speed reference 0.20κ(s(t)), issued angular command, and simulated body response. Lower row gives overlap-normalized, demeaned Pearson correlations corr(reference(t), response(t+lag)) over −1 to +1 s on a 0.05 s grid. Command is timestamped at t−0.05 s; response interpolation is linear solely for lag alignment. Positive lag means response trails the reference. Correlation includes closed-loop coupling and path-progress dependence; it is an association diagnostic, not an identified physical delay or causal measurement.

**Sample unit:** One preselected S-bend trial per method; dependent time samples; no confidence intervals

**Processing:** Command timestamp correction; target reference 0.20κ; per-lag overlap Pearson correlation; linear interpolation for alignment only.

**Sources:** `results/tracking_final/s_bend_nominal_full_0.npz`, `results/tracking_final/s_bend_nominal_ideal_0.npz`, `results/tracking_final/s_bend_nominal_native_0.npz`, `results/tracking_final/s_bend_nominal_tracker_0.npz`

## fig07_lag_heldout — Angular response/lag association, heldout

[PDF](fig07_lag_heldout.pdf) · [SVG](fig07_lag_heldout.svg) · [PNG](fig07_lag_heldout.png)

**Caption.** Seed-0 S-bend angular-response diagnostic under the heldout plant. Upper row compares target-speed reference 0.20κ(s(t)), issued angular command, and simulated body response. Lower row gives overlap-normalized, demeaned Pearson correlations corr(reference(t), response(t+lag)) over −1 to +1 s on a 0.05 s grid. Command is timestamped at t−0.05 s; response interpolation is linear solely for lag alignment. Positive lag means response trails the reference. Correlation includes closed-loop coupling and path-progress dependence; it is an association diagnostic, not an identified physical delay or causal measurement.

**Sample unit:** One preselected S-bend trial per method; dependent time samples; no confidence intervals

**Processing:** Command timestamp correction; target reference 0.20κ; per-lag overlap Pearson correlation; linear interpolation for alignment only.

**Sources:** `results/tracking_final/s_bend_heldout_full_0.npz`, `results/tracking_final/s_bend_heldout_ideal_0.npz`, `results/tracking_final/s_bend_heldout_native_0.npz`, `results/tracking_final/s_bend_heldout_tracker_0.npz`

## fig08_historical_route — Frozen historical reference route: synthetic replay

[PDF](fig08_historical_route.pdf) · [SVG](fig08_historical_route.svg) · [PNG](fig08_historical_route.png)

**Caption.** Supplementary synthetic closed-loop replay of the frozen historical 47.7 m inspection reference. Only Native MPPI and Guided MPC were run, once each, in the nominal synthetic plant. The reference point locations were reused; FAST-LIO and NDT were not executed and these are not historical measured robot traces. This single-route check is not pooled into the 72-trial aggregate statistics. No smoothing is applied.

**Sample unit:** One synthetic trial for each of two methods; one historical route

**Processing:** SI→mm/deg; raw sample connection; reference geometry reused unchanged.

**Sources:** `results/historical_route/historical_route_nominal_full_0.npz`, `results/historical_route/historical_route_nominal_native_0.npz`

## fig09_initial_offset_convergence — Recovery from initial tracking offsets

[PDF](fig09_initial_offset_convergence.pdf) · [SVG](fig09_initial_offset_convergence.svg) · [PNG](fig09_initial_offset_convergence.png)

**Caption.** Initial-offset recovery in the left-turn supplementary test (initial lateral offset 0.10 m and heading offset +10 deg). All three seed traces per method/plant are overlaid, without temporal smoothing or pointwise confidence bands. Dashed lines show the predeclared 10 mm lateral and 2 deg heading bands; convergence requires joint continuous residence in both bands for the declared hold duration, not merely one threshold crossing. This empirical finite-run recovery is not a stability theorem.

**Sample unit:** 24 supplementary closed-loop trials; three seeds per condition×method

**Processing:** Absolute errors, SI→mm/deg; three raw seed traces, no envelope/CI or filter.

**Sources:** `paper/supplement/convergence_heldout_full_0.npz`, `paper/supplement/convergence_heldout_full_1.npz`, `paper/supplement/convergence_heldout_full_2.npz`, `paper/supplement/convergence_heldout_ideal_0.npz`, `paper/supplement/convergence_heldout_ideal_1.npz`, `paper/supplement/convergence_heldout_ideal_2.npz`, `paper/supplement/convergence_heldout_native_0.npz`, `paper/supplement/convergence_heldout_native_1.npz`, `paper/supplement/convergence_heldout_native_2.npz`, `paper/supplement/convergence_heldout_tracker_0.npz`, `paper/supplement/convergence_heldout_tracker_1.npz`, `paper/supplement/convergence_heldout_tracker_2.npz`, `paper/supplement/convergence_nominal_full_0.npz`, `paper/supplement/convergence_nominal_full_1.npz`, `paper/supplement/convergence_nominal_full_2.npz`, `paper/supplement/convergence_nominal_ideal_0.npz`, `paper/supplement/convergence_nominal_ideal_1.npz`, `paper/supplement/convergence_nominal_ideal_2.npz`, `paper/supplement/convergence_nominal_native_0.npz`, `paper/supplement/convergence_nominal_native_1.npz`, `paper/supplement/convergence_nominal_native_2.npz`, `paper/supplement/convergence_nominal_tracker_0.npz`, `paper/supplement/convergence_nominal_tracker_1.npz`, `paper/supplement/convergence_nominal_tracker_2.npz`

## fig10_joint_settling — Joint-band settling and right censoring

[PDF](fig10_joint_settling.pdf) · [SVG](fig10_joint_settling.svg) · [PNG](fig10_joint_settling.png)

**Caption.** Joint-band settling times for every left-turn initial-offset recovery trial. Left: first continuous 1 s window within both 10 mm lateral and 2 deg heading bands. Right: time after the last observed threshold exit, accepted only when the terminal in-band dwell is at least 1 s. Filled circles denote observed events; open downward triangles denote right-censored trials at their own observation horizon. Horizons vary because trials end at geometric route completion or failure. Censored trials are neither assigned zero nor excluded. Labels c=n/N report the censored count. Last-exit settling is retrospective over a finite run, not a guarantee of future residence.

**Sample unit:** All 24 recovery attempts, including censored runs

**Processing:** Summary settling times; censored runs plotted at observation horizon with open triangles.

**Sources:** `paper/supplement/convergence_summary.json`

## fig11_openloop_step — Synthetic execution-chain step response

[PDF](fig11_openloop_step.pdf) · [SVG](fig11_openloop_step.svg) · [PNG](fig11_openloop_step.png)

**Caption.** Open-loop step response of the same synthetic execution chain. Left: command, 30 Hz rate-limited smoother output, discrete transport output, and first-order body response for nominal and held-out parameters. Right: time-parametrized command-versus-body loops with arrows showing temporal direction. A loop caused by dynamics is not evidence of material, frictional, or rate-independent hysteresis. Inputs/outputs are generated from the assumed model, not identified hardware data.

**Sample unit:** One deterministic open-loop input per plant condition

**Processing:** Raw samples connected; input–output time-parametrized loop; no filtering.

**Sources:** `paper/supplement/openloop_heldout_step.npz`, `paper/supplement/openloop_nominal_step.npz`

## fig11_openloop_multisine — Synthetic execution-chain multisine response

[PDF](fig11_openloop_multisine.pdf) · [SVG](fig11_openloop_multisine.svg) · [PNG](fig11_openloop_multisine.png)

**Caption.** Open-loop multisine response of the same synthetic execution chain. Left: command, 30 Hz rate-limited smoother output, discrete transport output, and first-order body response for nominal and held-out parameters. Right: time-parametrized command-versus-body loops with arrows showing temporal direction. A loop caused by dynamics is not evidence of material, frictional, or rate-independent hysteresis. Inputs/outputs are generated from the assumed model, not identified hardware data.

**Sample unit:** One deterministic open-loop input per plant condition

**Processing:** Raw samples connected; input–output time-parametrized loop; no filtering.

**Sources:** `paper/supplement/openloop_heldout_multisine.npz`, `paper/supplement/openloop_nominal_multisine.npz`

## fig12_assumed_frequency_response — Assumed linear-channel frequency response

[PDF](fig12_assumed_frequency_response.pdf) · [SVG](fig12_assumed_frequency_response.svg) · [PNG](fig12_assumed_frequency_response.png)

**Caption.** Analytical frequency responses of the assumed transport-delay plus first-order velocity channels for nominal and held-out parameters. The 30 Hz nonlinear rate-limited smoother, feedback, sampling residuals, and command constraints are excluded. These curves characterize an assumed channel, not a measured robot frequency response or the closed-loop controller bandwidth.

**Sample unit:** Deterministic analytical model, no statistical replicates

**Processing:** Analytic frequency-response CSV; logarithmic frequency axis; unwrapped phase.

**Sources:** `paper/data/model_frequency_response.csv`

## fig13_noisy_model_validation — Synthetic noisy-output model validation

[PDF](fig13_noisy_model_validation.pdf) · [SVG](fig13_noisy_model_validation.svg) · [PNG](fig13_noisy_model_validation.png)

**Caption.** Independent multisine validation of velocity-channel models fitted to the synthetic step experiment with added observation noise. The observation, unobserved simulation truth, and recursive fitted-model prediction are shown separately. Training and validation noise are independent. This establishes a synthetic identification procedure, not identification of the actual robot; refer to model_validation.json for assumptions, estimated parameters, and error values. No display filtering is applied.

**Sample unit:** One synthetic step fit and one independent multisine validation per plant

**Processing:** Raw noisy measurements, synthetic truth, and recursive model predictions; no display filtering.

**Sources:** `paper/supplement/noisy_model_prediction_heldout.npz`, `paper/supplement/noisy_model_prediction_nominal.npz`

## fig14_receding_horizon_diagnostics — Sampling diagnostics during offset recovery

[PDF](fig14_receding_horizon_diagnostics.pdf) · [SVG](fig14_receding_horizon_diagnostics.svg) · [PNG](fig14_receding_horizon_diagnostics.png)

**Caption.** Available sampling diagnostics for both residual methods during all initial-offset recovery runs (three seeds per plant/method). Left: effective sample size divided by the 1000 sampled sequences. Right: minimum sampled cost (dotted) and selected sequence cost (solid). Only observations with diagnostics_available=1 and command_valid=1 are plotted; zero placeholders for unavailable native/tracker diagnostics are excluded. The horizon and measured state change every call, so the cost sequence is a receding-horizon diagnostic, not optimizer convergence across iterations and not an inter-method optimality test. Costs are in arbitrary units and no temporal smoothing is used.

**Sample unit:** 12 supplementary residual-controller runs; three seeds per plant/method

**Processing:** Mask unavailable diagnostics/commands; ESS divided by 1000; raw receding-horizon costs, no filter.

**Sources:** `paper/supplement/convergence_heldout_full_0.npz`, `paper/supplement/convergence_heldout_full_1.npz`, `paper/supplement/convergence_heldout_full_2.npz`, `paper/supplement/convergence_heldout_ideal_0.npz`, `paper/supplement/convergence_heldout_ideal_1.npz`, `paper/supplement/convergence_heldout_ideal_2.npz`, `paper/supplement/convergence_nominal_full_0.npz`, `paper/supplement/convergence_nominal_full_1.npz`, `paper/supplement/convergence_nominal_full_2.npz`, `paper/supplement/convergence_nominal_ideal_0.npz`, `paper/supplement/convergence_nominal_ideal_1.npz`, `paper/supplement/convergence_nominal_ideal_2.npz`

