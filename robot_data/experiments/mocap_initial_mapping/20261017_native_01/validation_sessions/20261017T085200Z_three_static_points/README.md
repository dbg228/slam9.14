# 本轮三静止位置实测数据

本目录包含本轮全部数据与结果：

- report.md：实测报告、统计口径和限制。
- point_summary.csv：各静止窗口平均坐标、XYZ 偏差和误差统计。
- samples.csv：逐样本参考/估计坐标、时间戳、配对偏差和误差；role 区分独立验证与被排除的原位复测。
- positions.png、errors.png：位置叠加及误差图。
- points/point_01～03：三个独立位置的原始消息、同步矩阵和采集摘要。
- excluded/：初始化位置原位复测，未计入独立统计。
- failures/：两个 DDS 发布者发现失败、一个动捕缺失超时，均保留原始数据。
- frozen_inputs/：本次使用的世界标定、刚体外参、map→odom 初始化和配置快照。
- restart_initialization/：本次重启初始化的原始数据，未计入验证。
- session.json、results.json、runtime_after.json：采集索引、统计结果和结束时状态。
- software_checks.xml：软件回归 24 项通过，不能替代实测精度结果。

复现离线结果（不启动 ROS、不改变任何标定）：

```bash
cd /home/zkam/test_nav
python3 tests/mocap_anchored_mapping/tools/static_points_report.py --batch \
  /home/zkam/test_nav/robot_data/experiments/mocap_initial_mapping/20261017_native_01/validation_sessions/20261017T085200Z_three_static_points
```

测试是“动捕初始定位后 FAST-LIO 相对动捕的静止位姿误差”，不是对原 map↔mocap 世界标定的独立精度认证；也没有加载 PCD 做全局匹配。无自动运动、无 NDT、无验证数据重新拟合。

## 仅 XY 平面的新增误差图

`errors_xy.png`：仅使用三个独立位置的 95 对样本，上图为 X/Y 平均带符号偏差，下图为 XY RMSE/P95/最大值；不计入 Z 或原位复测。原三维图和数据保持不变。

```bash
cd /home/zkam/test_nav
python3 tests/mocap_anchored_mapping/tools/static_points_report.py --xy-only --batch \
  /home/zkam/test_nav/robot_data/experiments/mocap_initial_mapping/20261017_native_01/validation_sessions/20261017T085200Z_three_static_points
```

XY 误差图已按用户要求恢复为原先上下双图样式，提供 180dpi PNG（1620×1260）、PDF 和 SVG：errors_xy.png / errors_xy.pdf / errors_xy.svg。统计数值与原始数据未变。
