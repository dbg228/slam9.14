# map 与 mocap_world 的重新核查

本页记录旧的起始对齐核查。用户后续明确的“总共四次停放：前三点三维联合标定、第四点独立验证”现有独立工具，见 [固定三维世界标定](FIXED_WORLD_CALIBRATION.md)；两种方法及历史结果不要混用。

这份核查独立于轨迹控制，不发送速度、FollowPath、地图初值或TF，不更改原配置。相机重标定后，旧外参不直接复用。

## 之前实际怎么做

- 2026-09-17 的 `robot_data/config/mocap_to_map.session_20260917T094222759121Z.yaml` 保存6个静止点、每点约5s采样（449/450/450/443/450/444条）。其顶层仅有schema/model/frames/topic/parameters/points/captures，没有已验证的最终变换。旧话题还是 `/mocap/base_link/pose`，不能当成当前Tracker0标定结果。
- A/B/C定位实验复用 `fastlio_mocap_trajectory_evaluation_core.estimate_initial_static_alignment`。运动前取3s同步静止窗口，窗口在运动前0.5s结束；求一次平面旋转与平移，整轮冻结、不拟合尺度或整段轨迹。A的目标坐标是odom，B才是map，C是mocap_world，三者不能当同一map外参。
- NDT静止评估的起始窗口由单次报告配置；例如 `report_start_aligned_v2/summary.json` 用25s、13个有效匹配时刻，不是A/B/C的3s窗口。
- 已退役的S诊断此前使用起始0.5s内的第一对共同有效位姿求一次平面对齐；这同样是相对评估，不是独立的多位置全场标定。

以上起始对齐都会吸收起点的固定定位偏差。若map位姿来自FAST-LIO/NDT，其误差也会进入对齐；不能再将对齐残差称作独立的绝对地图标定精度。

## 变换方向

先把毫米换米、归一四元数，再应用已确认的完整安装变换：

`T_mocap_base = T_mocap_Tracker0 × T_Tracker0_base`

当前安装仍是Tracker0原点在base_link上方18cm，`T_Tracker0_base=(0,0,-.18m)`、单位旋转，偏移用Tracker0轴表达。不能只改frame名字。

`T_map_mocap = T_map_base × inverse(T_mocap_base)` 将 **mocap坐标转到map**。

用户所说 **map转到mocap** 应使用其逆：`T_mocap_map = inverse(T_map_mocap)`。

平面写法 `p_map=R(theta) p_mocap+t` 的逆为 `p_mocap=R(theta)^T (p_map-t)`，不能只将平移取负。工具同时保存两个方向。只做SE(2)，不会假装已标定尺度、地面倾斜或z。

## 本次只读工具

`tests/controller_tracking/tools/map_mocap_check.py` 复用原工程的完整刚体转换、严格时间匹配、静止对齐和误差函数；不再实现另一套基础算法。

它在真实 `odom→base_link` 测量时刻查询 `map→base_link`，将高频动捕匹配到这一时刻。这样无需为每个90Hz动捕样本强行插值10Hz里程计。动捕只在相邻有效且间隔≤0.10s内插值，遇无效清空支持历史；无map ready、旧TF、无配对、样本不足或不静止都会拒绝生成候选。位姿时间不是 `/odom` 桥接的now时间。当前相机时间仍是Ubuntu接收时间，动态延迟未知。

从根目录运行（先启动原动捕与正常定位，RViz给真实地图初值；不运行导航控制）：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# 小车停稳，采5s；最多等20s。目录仅为示例，请换成本次会话路径。
python3 tests/controller_tracking/tools/map_mocap_check.py capture \
  --output /tmp/map_mocap_check/point01.json --seconds 5 --timeout 20

# 只由这一个静止窗口确定候选，已存在的候选不会被覆盖
python3 tests/controller_tracking/tools/map_mocap_check.py align \
  --capture /tmp/map_mocap_check/point01.json \
  --output /tmp/map_mocap_check/candidate.json

# 其他位置由用户人工移动、停稳，再分别capture point02/03/04
python3 tests/controller_tracking/tools/map_mocap_check.py verify \
  --transform /tmp/map_mocap_check/candidate.json \
  --captures /tmp/map_mocap_check/point02.json \
             /tmp/map_mocap_check/point03.json \
             /tmp/map_mocap_check/point04.json \
  --output /tmp/map_mocap_check/verification.json

# 汇总同一会话的point02及后续记录，包含质量不通过的尝试
python3 tests/controller_tracking/tools/map_mocap_check.py report \
  --session /tmp/map_mocap_check --finalize
```

新标定编号可通过capture的 `--calibration-id` 明确传入；未提供就记录null，不编造编号。`--mocap-offset-s`默认0仅表示未施加偏移，报告仍标记时间未独立核实。

验证不会再拟合变换，禁止将训练点作为独立验证点；至少3个分布不共线的验证位置才标记覆盖充分。每点保存XY/航向残差、位置范围与数据质量。即使多位置残差很小，也只能说明与当前地图定位链一致；若要认证绝对标定，还需要地图中独立测量的对应点。

新记录保存在 `robot_data/experiments/frame_alignment_check/`，JSON记录失败尝试和同步样本，默认无CSV。过程状态和实际结果以当前会话report/readiness为准，不把数学测试通过当作实测标定通过。

`report` 只更新 `spatial_report/` 的派生报告与图，不修改candidate或原始数据。未通过质量门槛的样本保留在表中、标为仅诊断；其缺失指标保持null，不充作可用验证点。精度验收阈值尚未指定，不自动宣布高精度通过。

## 2026-10-17 本次实测

记录位于 `robot_data/experiments/frame_alignment_check/20261017T042344Z_recalibration_check/`，[最终报告](../../robot_data/experiments/frame_alignment_check/20261017T042344Z_recalibration_check/report.md)保存全部训练窗口、验证尝试、两个方向的冻结候选和限制。

共1个训练位置、3个验证位置，验证阶段5次记录。点2和点3补录通过数据质量检查，XY残差RMSE分别约24.1mm、20.4mm；点3首次和点4两次因约0.30～0.40s同步样本间隙未通过，原样保留、仅诊断，不跨缺口插值。点4不再要求用户换位置；本轮采集已结束。

实际点位具备二维分布，但只有两个合格验证位置，未满足原定的三个非共线合格验证位置；因此未认证高精度标定。候选文件逐次校验保持不变，从未重拟合验证点、修改正式TF或发出运动命令。残差包含FAST-LIO/NDT地图估计误差，不能单独归因于安装或坐标变换。

4项只读工具软件测试通过；原始失败记录与派生图均保留，没有生成CSV。动捕、雷达与只读定位继续运行，采集进程已退出。
