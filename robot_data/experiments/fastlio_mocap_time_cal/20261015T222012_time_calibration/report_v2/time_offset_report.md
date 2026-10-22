# FAST-LIO — Tracker0 时间偏移标定报告

该报告只使用独立时间标定动作包的平移速度与有符号航向角速度相关性。它没有空间对齐，没有最小化轨迹位置误差，也不修改任何在线节点。

## 结果

- 建议固定偏移：**17.0 ms**
- 定义：For each FAST-LIO transform stamped t, positive offset d evaluates the corresponding mocap kinematics at header timestamp t+d.
- 合并相关性：0.94182；零偏移：0.92905
- 相对零偏移增益：0.01277
- 近最优敏感区间：10.0 至 91.0 ms（非统计置信区间）

- 平移通道峰值：15.0 ms；航向通道峰值：116.0 ms；二者相差：101.0 ms。

**本包仅可作为初步估计，不能用于毫米级动态误差结论。** 原因：`mocap_ended_before_fastlio_recording`、`mocap_covers_less_than_95_percent_of_fastlio_duration`、`near_best_offset_interval_wider_than_30_ms`、`linear_and_yaw_channel_peaks_differ_by_more_than_30_ms`。

## 数据覆盖

- FAST-LIO 精确 TF：838 条，持续 83.699 s；来源：`tf_exact_transform_stamp`。
- 动捕（已应用显式 Tracker0→base_link 外参）：3085 条，持续 34.260 s。后续无动捕样本不会被外推或重复。

- 有效共同时间段：34.215 s，占 FAST-LIO 全段的 40.9%。

## 限制

- 动捕 header 当前来自 Ubuntu 接收时钟；该数值是该采集链的经验固定偏移，不能声称为相机曝光时刻延迟。
- 若近最优区间很宽、平移和转向通道给出相互矛盾的峰值，必须重录带连续有效动捕的标定包，不能把该值用于毫米级结论。
- 该结果应只供后续独立两米包的时间匹配使用；不能用两米精度包反向重估此偏移。

## 文件

- `time_offset_summary.json`：机器可读完整审计与参数；
- `time_offset_candidates.csv`：每个候选偏移的相关性；
- `kinematic_signals.csv`：最佳偏移下的逐帧信号；
- `time_offset_correlation.svg/png`、`kinematic_signals_at_best_offset.svg/png`：图。
