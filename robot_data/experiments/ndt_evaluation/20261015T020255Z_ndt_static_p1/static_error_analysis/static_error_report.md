# 静止 NDT 误差分解

- 输入报告：`/home/zkam/test_nav/robot_data/experiments/ndt_evaluation/20261015T020255Z_ndt_static_p1/report_start_aligned_warmup12`
- NDT pose：`final`；动捕参考：Tracker0 刚体经报告中显式的 Tracker0→base_link 外参离线转换。
- 外参平移 m：`[0.0, 0.0, -0.18]`；四元数 xyzw：`[0.0, 0.0, 0.0, 1.0]`。
- 起始对齐模式会吸收常数 `mocap_world→map` 偏差及常数平面安装偏差；本报告只能分解**对齐后的残余中心偏移与时变散布**，不能声称测得绝对外参或绝对地图偏差。
- 排除的起始等待/对齐记录：19；无效参考：0；无效 NDT pose：0。

## 逐帧误差统计

| 指标 | 数值 |
| --- | ---: |
| 帧数 | 38 |
| mean(ex) / std(ex) | -1.046 / 16.480 mm |
| mean(ey) / std(ey) | -3.556 / 15.286 mm |
| XY RMSE / P95 / max | 22.487 / 45.167 / 47.834 mm |
| 残余中心 (ex, ey) | (-1.046, -3.556) mm |
| 残余中心模长 | 3.707 mm |
| 去中心随机 XY RMS | 22.180 mm |
| 中心项占 XY RMSE² | 2.72% |
| yaw RMSE / P95 / max | 0.1055 / 0.1883 / 0.2625 deg |

## 相邻帧重复匹配稳定性

- 相邻帧对：37，中位时间间隔 2.000 s，因长间隔跳过 0 对。
- NDT Δposition RMSE：25.894 mm；相对动捕 Δposition 误差 RMSE：25.870 mm。
- NDT Δyaw RMSE：0.0948 deg；相对动捕 Δyaw 误差 RMSE：0.0937 deg。

## 异常帧与可获得诊断量

- 阈值：XY error ≥ 40.0 mm；异常帧数：3；match_id：[225, 226, 241]。
- Pearson r(error, fitness)：-0.06841175880609279；r(error, correction magnitude)：0.48415241510228596。这些只是小样本相关性，不代表因果。
- 点云重叠率：unavailable: current NdtMatchDiagnostic records source/target point counts but no overlap-ratio metric
- 请查看 `outlier_frames.csv` 中每个异常帧的 fitness、校正量、航向误差、迭代次数和点数；没有记录的指标不会被臆造。

## 解释边界

若散点的残余中心远小于去中心随机 RMS，则这段对齐后的误差主要表现为时变定位/里程计/NDT链路散布，而不是残余常数偏置。但起始对齐已经消除了常数坐标系偏差，故不能用该结论否定未独立标定的 `mocap_world→map` 或 Tracker0→base_link 外参。要量化那些绝对偏差，需要独立的 map←mocap_world 标定。
