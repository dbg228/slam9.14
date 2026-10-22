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
