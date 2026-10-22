# 静止 NDT 误差分解

- 输入报告：`/home/zkam/test_nav/robot_data/experiments/ndt_evaluation/20261015T042528Z_v2_static_p1/report_first_ndt_relative`
- NDT pose：`final`；动捕参考：Tracker0 刚体经报告中显式的 Tracker0→base_link 外参离线转换。
- 外参平移 m：`[0.0, 0.0, -0.18]`；四元数 xyzw：`[0.0, 0.0, 0.0, 1.0]`。
- 起始对齐模式会吸收常数 `mocap_world→map` 偏差及常数平面安装偏差；本报告只能分解**对齐后的残余中心偏移与时变散布**，不能声称测得绝对外参或绝对地图偏差。
- 排除的起始等待/对齐记录：0；无效参考：2；无效 NDT pose：0。

## 逐帧误差统计

| 指标 | 数值 |
| --- | ---: |
| 帧数 | 67 |
| mean(ex) / std(ex) | 7.197 / 14.895 mm |
| mean(ey) / std(ey) | 11.688 / 13.895 mm |
| XY RMSE / P95 / max | 24.436 / 41.731 / 51.054 mm |
| 残余中心 (ex, ey) | (7.197, 11.688) mm |
| 残余中心模长 | 13.726 mm |
| 去中心随机 XY RMS | 20.217 mm |
| 中心项占 XY RMSE² | 31.55% |
| yaw RMSE / P95 / max | 0.2494 / 0.4205 / 0.4887 deg |

## 相邻帧重复匹配稳定性

- 相邻帧对：66，中位时间间隔 2.000 s，因长间隔跳过 0 对。
- NDT Δposition RMSE：26.531 mm；相对动捕 Δposition 误差 RMSE：26.546 mm。
- NDT Δyaw RMSE：0.1015 deg；相对动捕 Δyaw 误差 RMSE：0.1034 deg。

## 异常帧与可获得诊断量

- 阈值：XY error ≥ 40.0 mm；异常帧数：5；match_id：[179, 185, 186, 197, 205]。
- Pearson r(error, fitness)：0.10751146053576083；r(error, correction magnitude)：0.26513438846384607。这些只是小样本相关性，不代表因果。
- 点云重叠率：unavailable: current NdtMatchDiagnostic records source/target point counts but no overlap-ratio metric
- 请查看 `outlier_frames.csv` 中每个异常帧的 fitness、校正量、航向误差、迭代次数和点数；没有记录的指标不会被臆造。

## 解释边界

若散点的残余中心远小于去中心随机 RMS，则这段对齐后的误差主要表现为时变定位/里程计/NDT链路散布，而不是残余常数偏置。但起始对齐已经消除了常数坐标系偏差，故不能用该结论否定未独立标定的 `mocap_world→map` 或 Tracker0→base_link 外参。要量化那些绝对偏差，需要独立的 map←mocap_world 标定。
