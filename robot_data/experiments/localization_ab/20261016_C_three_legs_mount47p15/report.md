# C 组三次试验：同源动捕一致性

reverse / forward / reverse；按方向记录，不作为三次相同正向重复。
每次绝对速度指令 0.15 m/s、位移 2 m 发零、续录 5 s；各自起始静止窗口固定对齐。
动捕不反馈到 A/B 定位。时间偏移未独立核实，动态指标为初步相对误差。

| 次数 | 方向 | 状态 | XY RMSE mm | P95 mm | Max mm | yaw RMSE ° | 终点差 mm |
|---|---|---|---:|---:|---:|---:|---:|
| 1 | reverse | completed | 0.268 | 0.213 | 3.194 | 0.0020 | 0.009 |
| 2 | forward | completed | 0.347 | 0.721 | 2.596 | 0.0033 | 0.008 |
| 3 | reverse | completed | 0.373 | 0.708 | 3.804 | 0.0029 | 0.015 |

任一失败停止整批，不自动重试；未完成的试次不得记为成功。
C 的同一动捕流同时参与全局校正和对比，以下指标不是独立绝对定位精度；FAST-LIO 内部状态和已有 TF 不被改写。

## 停车距离与安全核查

| 次数 | 2 m 发零触发位移 m | 停稳位移 m | 相对 2 m 的过冲 mm | 参考覆盖率 |
|---|---:|---:|---:|---:|
| 1 倒车 | 2.001140 | 2.007719 | 7.719 | 100% |
| 2 前进 | 2.000791 | 2.007882 | 7.882 | 100% |
| 3 倒车 | 2.000976 | 2.008208 | 8.208 | 100% |

终点差是估计位姿与同源动捕位姿的差，不是停车距离偏离 2.000 m 的误差。
三次完整到达目标并续录 5 s，无保护中断。最终静止检查和底盘零指令已核实。

## 图与原始数据

| 次数 | 轨迹及误差时序 | 误差随真实距离 | 原始包 | 逐帧数据 |
|---|---|---|---|---|
| 1 倒车 | [PNG](01_reverse/report/trajectory_errors.png) / [SVG](01_reverse/report/trajectory_errors.svg) | [PNG](01_reverse/report/errors_by_true_distance.png) | [metadata](01_reverse/bag/metadata.yaml) | [CSV](01_reverse/report/frame_errors.csv) |
| 2 前进 | [PNG](02_forward/report/trajectory_errors.png) / [SVG](02_forward/report/trajectory_errors.svg) | [PNG](02_forward/report/errors_by_true_distance.png) | [metadata](02_forward/bag/metadata.yaml) | [CSV](02_forward/report/frame_errors.csv) |
| 3 倒车 | [PNG](03_reverse/report/trajectory_errors.png) / [SVG](03_reverse/report/trajectory_errors.svg) | [PNG](03_reverse/report/errors_by_true_distance.png) | [metadata](03_reverse/bag/metadata.yaml) | [CSV](03_reverse/report/frame_errors.csv) |

三条原始包均保留。图中采样时刻按估计测量时间与动捕插值匹配，
不跨 0.1 s 参考丢帧插值；未独立标定时间偏移仍为零，不能据此宣称相机测量时刻已同步。
三趟最大相邻位姿变化分别 14.567 / 13.881 / 14.723 mm，包含真实行驶增量；
扣除动捕真实运动的最大误差跳变分别 3.281 / 2.683 / 3.843 mm。

## 与 A/B 的比较边界

C 的参考参与自身校正，不能把此处 0.268–0.373 mm 与 A/B 的独立参考误差
作为公平的绝对精度排名，也不能据此宣称系统绝对精度达到亚毫米。
C 倒/前/倒与 A/B 前/倒/前方向数量不同；C 批次前因旧 FAST-LIO 滞后而按原参数重启，
批内三趟连续运行。详见[准备和停车审计](../20261016_C_preparation/audit.md)。
要验证 C 独立精度，需要独立参考、未参与校正的独立观测，或对无动捕校正区间做单独评估。
