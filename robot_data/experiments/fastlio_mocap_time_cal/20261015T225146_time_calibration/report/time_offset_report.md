# FAST-LIO — Tracker0 时间偏移标定报告

该报告只使用独立时间标定动作包的平移速度与有符号航向角速度相关性。它没有空间对齐，没有最小化轨迹位置误差，也不修改任何在线节点。

## 结果

- 当前搜索峰值：**-121.0 ms**（未通过质量检查，尚不能作为已确定的补偿量）
- 定义：For each FAST-LIO transform stamped t, positive offset d evaluates the corresponding mocap kinematics at header timestamp t+d.
- 合并相关性：0.60376；零偏移：0.60139
- 相对零偏移增益：0.00237
- 近最优敏感区间：-300.0 至 191.0 ms（非统计置信区间）

- 平移通道峰值：-188.0 ms；航向通道峰值：-59.0 ms；二者相差：129.0 ms。

**本包仅可作为初步估计，不能用于毫米级动态误差结论。** 原因：`near_best_offset_interval_wider_than_30_ms`、`linear_and_yaw_channel_peaks_differ_by_more_than_30_ms`。

## 数据覆盖

实际操作顺序由用户确认：静止 → 直行停车 → 左转约 90° → 右转约 180° → 左转约 90° 回正并静止 → 最后后退返回停车。工具按时间戳配对，不依赖预设动作顺序；详见上级目录的 `experiment_notes.md`。

- FAST-LIO 精确 TF：791 条，持续 79.000 s；来源：`tf_exact_transform_stamp`。
- 动捕（已应用显式 Tracker0→base_link 外参）：7204 条，持续 80.057 s。后续无动捕样本不会被外推或重复。

- 有效共同时间段：79.000 s，占 FAST-LIO 全段的 100.0%。

## 限制

- 动捕 header 当前来自 Ubuntu 接收时钟；该数值是该采集链的经验固定偏移，不能声称为相机曝光时刻延迟。
- 若近最优区间很宽、平移和转向通道给出相互矛盾的峰值，必须重录带连续有效动捕的标定包，不能把该值用于毫米级结论。
- 该结果应只供后续独立两米包的时间匹配使用；不能用两米精度包反向重估此偏移。

## 文件

- `time_offset_summary.json`：机器可读完整审计与参数；
- `time_offset_candidates.csv`：每个候选偏移的相关性；
- `kinematic_signals.csv`：最佳偏移下的逐帧信号；
- `time_offset_correlation.svg/png`、`kinematic_signals_at_best_offset.svg/png`：图。
