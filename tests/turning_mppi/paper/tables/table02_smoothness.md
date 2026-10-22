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
