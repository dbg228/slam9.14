# 静止 NDT 误差分解

- 输入报告：`/home/zkam/test_nav/robot_data/experiments/ndt_evaluation/20261015T042528Z_v2_static_p1/report_start_aligned_v2`
- NDT pose：`final`；动捕参考：Tracker0 刚体经报告中显式的 Tracker0→base_link 外参离线转换。
- 外参平移 m：`[0.0, 0.0, -0.18]`；四元数 xyzw：`[0.0, 0.0, 0.0, 1.0]`。
- 起始对齐模式会吸收常数 `mocap_world→map` 偏差及常数平面安装偏差；本报告只能分解**对齐后的残余中心偏移与时变散布**，不能声称测得绝对外参或绝对地图偏差。
- 排除的起始等待/对齐记录：13；无效参考：2；无效 NDT pose：0。

## 逐帧误差统计

| 指标 | 数值 |
| --- | ---: |
| 帧数 | 54 |
| mean(ex) / std(ex) | 7.609 / 15.501 mm |
| mean(ey) / std(ey) | -0.241 / 13.904 mm |
| XY RMSE / P95 / max | 21.990 / 44.844 / 50.621 mm |
| 残余中心 (ex, ey) | (7.609, -0.241) mm |
| 残余中心模长 | 7.613 mm |
| 去中心随机 XY RMS | 20.630 mm |
| 中心项占 XY RMSE² | 11.99% |
| yaw RMSE / P95 / max | 0.1587 / 0.2999 / 0.3619 deg |

## 相邻帧重复匹配稳定性

- 相邻帧对：53，中位时间间隔 2.000 s，因长间隔跳过 0 对。
- NDT Δposition RMSE：27.073 mm；相对动捕 Δposition 误差 RMSE：27.084 mm。
- NDT Δyaw RMSE：0.1057 deg；相对动捕 Δyaw 误差 RMSE：0.1075 deg。

## 异常帧与可获得诊断量

- 阈值：XY error ≥ 40.0 mm；异常帧数：5；match_id：[179, 185, 186, 197, 205]。
- Pearson r(error, fitness)：0.14709943998005337；r(error, correction magnitude)：0.25063353417707906。这些只是小样本相关性，不代表因果。
- 点云重叠率：unavailable: current NdtMatchDiagnostic records source/target point counts but no overlap-ratio metric
- 请查看 `outlier_frames.csv` 中每个异常帧的 fitness、校正量、航向误差、迭代次数和点数；没有记录的指标不会被臆造。

## 解释边界

若散点的残余中心远小于去中心随机 RMS，则这段对齐后的误差主要表现为时变定位/里程计/NDT链路散布，而不是残余常数偏置。但起始对齐已经消除了常数坐标系偏差，故不能用该结论否定未独立标定的 `mocap_world→map` 或 Tracker0→base_link 外参。要量化那些绝对偏差，需要独立的 map←mocap_world 标定。
