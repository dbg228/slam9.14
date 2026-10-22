# Group A — relative localization error

Trial status: **aborted**; stop reason: **mocap_stale**.
Failed trial: exclude from completed 2 m comparisons. Metrics below describe only evaluable samples.

Timing status: **provisional_time_offset_unverified**; offset 0.000000 s.

| Metric | Value |
|---|---|
| XY RMSE / P95 / max (mm) | 19.925 / 26.354 / 26.858 |
| Yaw RMSE (deg) | 0.045 |
| Endpoint error (mm) | 25.438 |
| Max adjacent pose / error jump (mm) | 29.108 / 19.039 |
| Reference coverage | 99.2% |
| Stop reason | mocap_stale |

C uses the same mocap stream for correction and comparison: its result measures consistency, not independent localization accuracy.
All metrics exclude the initial alignment window and cover motion plus the recorded 5-second stop tail.
When timing is unverified, dynamic RMSE is provisional. Endpoint uses the 2–5 s stationary tail; inspect reference_stationary in summary.json.
The maximum raw pose jump contains real motion; maximum error jump also provided to distinguish correction discontinuities.
See summary.json for missing data, endpoint physical overshoot, and full definitions.
