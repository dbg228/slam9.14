# Group A — relative localization error

Trial status: **completed**; stop reason: **target_reached**.


Timing status: **provisional_time_offset_unverified**; offset 0.000000 s.

| Metric | Value |
|---|---|
| XY RMSE / P95 / max (mm) | 3.014 / 4.541 / 6.968 |
| Yaw RMSE (deg) | 0.049 |
| Endpoint error (mm) | 1.562 |
| Max adjacent pose / error jump (mm) | 22.851 / 4.256 |
| Reference coverage | 100.0% |
| Stop reason | target_reached |

C uses the same mocap stream for correction and comparison: its result measures consistency, not independent localization accuracy.
All metrics exclude the initial alignment window and cover motion plus the recorded 5-second stop tail.
When timing is unverified, dynamic RMSE is provisional. Endpoint uses the 2–5 s stationary tail; inspect reference_stationary in summary.json.
The maximum raw pose jump contains real motion; maximum error jump also provided to distinguish correction discontinuities.
See summary.json for missing data, endpoint physical overshoot, and full definitions.
