# NDT 匹配诊断与动捕离线评估

本功能只记录数据和离线评估。它不改变 NDT 初值、NDT 参数、匹配接受门限、`map -> odom` 发布逻辑、Nav2 参数或任何运动命令。

## 已核实的数据流和坐标语义

当前定位节点是 `src/static_map_localization/src/static_map_localizer_node.cpp`。

| 数据 | 工程中的实际来源/语义 |
| --- | --- |
| 匹配输入 | `/cloud_registered`，节点要求其 `header.frame_id` 为 `camera_init`；诊断保留该消息原始 `header.stamp`。 |
| FAST-LIO 位姿 | `/tf_lio_raw` 中 `camera_init -> body`。节点用配置的 `base_link -> sensor`（当前 `(0.37, 0.28, 0.15 m, +47.15°)`）的逆得到 `odom(camera_init) -> base_link`。 |
| NDT 初值/输出 | NDT 将 source（`camera_init` 中的注册点云）配准到固定 PCD 的 `map`，初值及 `getFinalTransformation()` 均为 `map <- odom(camera_init)`。 |
| 接受条件 | 现有条件未改变：`hasConverged`、有限 fitness、fitness `<= max_fitness_score`（默认 1.5，**越小越好**）、平移修正 `<= max_update_translation`、航向修正 `<= max_update_yaw`。 |
| 最终导航坐标 | 最后接受的 `map <- odom` 与连续 `odom -> base_link` 组成 `map -> odom -> base_link`。`/static_localization/pose` 仅在接受 NDT 后发布，时间戳是**匹配完成时刻**。它不应用于本功能的测量时刻配对。 |
| 动捕刚体/车体中心 | `./mocap_tracker0_vrpn.sh` 输入 `/vrpn_mocap/Tracker0/pose`，规范化输出 `/mocap/Tracker0/pose`（毫米转米、原始 header stamp 保留）。接收器绝不将它冒充为 `base_link`，也不发布 TF。离线评估根据 `robot_data/config/mocap_tracker0.yaml` 中明确记录的 `T_Tracker0_base_link=(0,0,-0.18 m)`、单位旋转，将每条刚体 pose 转为 base_link pose。 |

`mocap_tracker0_vrpn.sh` 使用 ROS 接收侧时间戳，不是已校准的相机采样时刻。评估可通过
`--time-offset-seconds` 使用**事先独立测得**的固定延迟，绝不会用待评估轨迹反向寻优该偏移。
本工程只报告固定起始对齐参考下的相对定位误差，不再维护绝对定位误差路径。

## 新的在线诊断记录

NDT 节点发布两个可靠 QoS 的独立话题：

```text
/static_localization/ndt_match_diagnostic
# type: static_map_localization/msg/NdtMatchDiagnostic

/static_localization/ndt_match_diagnostic_detail
# type: static_map_localization/msg/NdtMatchDiagnosticDetail
```

第一个是稳定的 V1 兼容记录；不能再修改它，否则旧 rosbag 无法按原类型读取。第二个是
新增的 V2 详细记录，内嵌 V1 的 `base` 字段。**后续实验应录制并评估 V2 detail 话题。**
旧包可以继续用 V1 分析 `T_ndt` 和 `T_final`，但不含严格点云时刻的 `T_init`；评估器会明确
把该列标为不可用，而不会用回调时刻的旧字段替代。

每次 NDT 定时器运行都有单调递增 `match_id`，包括未初始化、没有点云、点数不足、NDT 抛异常、不收敛、fitness 拒绝、增量门限拒绝及接受的情况。每条记录包括：

- 点云测量时间 `cloud_stamp`、开始/完成时间、steady-clock 计算耗时；
- 初始 `map<-odom` 与用于初值的 `map->base_link`；
- 原始 NDT `map<-odom`，以及未经过接受门限的 `map->base_link`；
- NDT 收敛、迭代数、PCL fitness、fitness 的明确定义和好坏方向；
- 接受状态和明确拒绝原因；
- 原始/下采样 source、原始/下采样 target 点数；
- 修正平移/航向大小；
- 匹配后的最终（新接受或上一次接受的）`map->base_link`。

V2 还严格记录：

- `T_init = T_map_odom(before NDT) * T_odom_base(near cloud_stamp)`；这是唯一可和同时间动捕真值比较的 FAST-LIO 初值；
- `T_ndt`（V1 `raw_ndt_map_to_base_at_cloud`）与 `T_final_at_cloud`（V1 `final_map_to_base_at_cloud`）；
- `T_init^-1*T_ndt`、`T_map_odom(before)^-1*T_map_odom(ndt)` 及其平移/有符号航向修正；
- 实际低频 `/static_localization/pose` 发布时刻的姿态和时间戳。它只用于审计发布链，**不**用于和点云时刻动捕配对；
- 此次实际配置的 `max_iterations`、`transformation_epsilon`、`step_size`、`resolution`；
- `ndt.align()` 本体耗时；V1 的总 callback 耗时仍保留，且会包含 detail 后验诊断的少量计算时间；
- 以相同下采样 target 为基准的后验最近邻计数、阈值内比例和 RMS 残差。它们是明确定义的诊断代理指标，不是 PCL NDT 的私有 Gaussian correspondence/inlier 集；
- Hessian condition number 显式为 unavailable：当前安装的 PCL 公共 API 未导出 NDT 优化 Hessian，未知值不会填 0。

为了把 NDT 结果严格评估在点云时间，节点只为**诊断**维护短暂的 `camera_init->body` 缓存，并选择离 `cloud_stamp` 不超过 `diagnostic_pose_max_skew_s`（默认 0.10 s）的最近 `odom->base_link`。`*_at_cloud` 只在该条件满足时有效；无有效 TF 会显式为 false，不会填零。它不会改变 NDT 所用的原始最新初值或任意接受判定。

PCL 的 fitness 是最终变换下的均值平方最近邻距离，不是定位真实误差，也没有被伪装成协方差。当前节点不估计协方差，消息用 `covariance_available=false` 表示未知。

`iteration_count` 直接来自 PCL `NormalDistributionsTransform::getFinalNumIteration()`；在本机
PCL 1.12 中它返回注册基类的 `nr_iterations_`，每个 NDT 外层优化循环递增一次。因此当前
反复出现 `1` 表示真实只进行了一个外层优化迭代，不是本工程写死的常数。当前默认的实际
配置是 `max_iterations=15`、`transformation_epsilon=0.01`、`step_size=0.25`、
`resolution=1.20 m`；V2 每条记录都会重复保存这些数值，便于审计。

## 构建与检查

```bash
cd /home/zkam/test_nav
source /opt/ros/humble/setup.bash
colcon build --packages-select static_map_localization --symlink-install
source install/setup.bash

ros2 topic info /static_localization/ndt_match_diagnostic -v
ros2 topic echo --once /static_localization/ndt_match_diagnostic
ros2 topic info /static_localization/ndt_match_diagnostic_detail -v
ros2 topic echo --once /static_localization/ndt_match_diagnostic_detail
```

`robot_xj_project.rviz` 会显示 `/static_localization/map_cloud`：这是**当前 NDT
实际使用的下采样固定 PCD**，坐标系为 `map`，只用于可视化和人工初始位姿估计；它不参与
新的匹配计算。此流程不需要二维栅格地图。在 RViz 保持 Fixed Frame=`map`，将视角调至
俯视，使用 **2D Pose Estimate** 在固定 PCD 的对应结构附近点击并沿车头方向拖出短线。
给出近似初值即可；NDT 负责后续细化。不要以仅有的 `/cloud_registered` 实时点云作为固定
地图来估计初值。

然后照常启动动捕和定位；不要同时启动第二份定位链：

```bash
./mocap_tracker0_vrpn.sh
./localization.sh
```

在 RViz 使用 **2D Pose Estimate** 初始化。只有 `/static_localization/ready` 为 true 后再开始正式测试。

## 录包

新开终端。下面话题名均来自本工程当前的 Tracker0 接入；录包只订阅，不将动捕反馈给定位或 Nav2。

```bash
cd /home/zkam/test_nav
source /opt/ros/humble/setup.bash
source install/setup.bash
RUN_ID="$(date -u +%Y%m%dT%H%M%SZ)_ndt_eval"
mkdir -p "robot_data/experiments/ndt_evaluation"
ros2 bag record -o "robot_data/experiments/ndt_evaluation/${RUN_ID}/bag" \
  /static_localization/ndt_match_diagnostic \
  /static_localization/ndt_match_diagnostic_detail \
  /mocap/Tracker0/pose \
  /mocap/Tracker0/diagnostics \
  /vrpn_mocap/Tracker0/pose \
  /static_localization/pose \
  /static_localization/odometry \
  /static_localization/map_to_odom \
  /static_localization/ready \
  /static_localization/status \
  /cloud_registered /odom /tf /tf_static /tf_lio_raw \
  /initialpose /initialpose_user /initialpose_checked
```

按 `Ctrl-C` 停止录包。

## 离线评估

### 初始化后的静止稳定性（当前推荐）

人工 `2D Pose Estimate` 是粗初始化。第一次 NDT 相对人工初值出现 10–15 cm 甚至更大的
修正，不能单独作为定位误差或错误更新的证据。当前静止测试分为三个阶段，阶段边界在看
正式结果之前确定：

1. 第一条同时 `converged=true`、`accepted=true` 的真实 NDT 匹配标记“初始化完成”；
   这条及此前所有记录保留在逐帧文件中，但不计入稳定性统计。
2. 默认再排除后续 **3 次真实 NDT 尝试**作为短暂稳定期；可在实验前固定选择 2 次。
   失败、未收敛和被拒绝的尝试也占一次；等待初始化、缺少点云等未进入 NDT 的记录不占次数。
3. 从下一次真实 NDT 尝试开始正式统计。机器人保持静止；全部可评估原始匹配与接受匹配
   分开统计。不能看到某条误差大之后再延长稳定期或删除该帧。

对齐使用被排除的稳定期内有效且同步的 `T_final(base_link)` 与动捕 `base_link`，只估计
**一次固定 SE(2) 变换**。正式段的 `T_init`、`T_ndt`、`T_final` 共用它，禁止分别对齐或
用正式段重新拟合。Tracker0 先按 YAML 中完整安装变换转换到 `base_link`。缺少可用稳定期
参考数据时，不能暗中改用正式段；应检查报告或补录数据。该方法不估计尺度，也不评价人工
初值的固定偏差和全局绝对定位准确度。

直接从 V2 rosbag 运行，不需要先生成旧的 25 秒窗口报告：

```bash
cd /home/zkam/test_nav
source /opt/ros/humble/setup.bash
source install/setup.bash

RUN_DIR=robot_data/experiments/ndt_cold_start/20261015T060146Z_cold_first_ndt_p1
python3 ndt_post_init_analysis.py \
  --bag "$RUN_DIR/bag" \
  --output "$RUN_DIR/report_post_init" \
  --settling-matches 3 \
  --max-interpolation-gap-seconds 0.10 \
  --tracker-to-base-config robot_data/config/mocap_tracker0.yaml

xdg-open "$RUN_DIR/report_post_init/post_init_timeseries.png"
xdg-open "$RUN_DIR/report_post_init/update_effects.png"
```

默认读取本工程的 `/static_localization/ndt_match_diagnostic_detail` 与
`/mocap/Tracker0/pose`。例中的袋记录包含初始化前等待帧，按 3 次稳定期的定义，
`match_id=8` 为初始化完成，`9–11` 为稳定期，`12–33` 为正式段；不能把正式段后来出现的
大更新重新归入初始化。其他实验替换 `RUN_DIR`，并保持同一预先选定的稳定期次数。

输出文件：

| 文件 | 内容 |
| --- | --- |
| `report.md`、`summary.json` | 阶段边界、对齐来源、参考覆盖率、分阶段统计和解释限制 |
| `frames.csv` | 所有诊断的阶段、有效性、三段位姿与误差、候选/实际 `map→odom` 更新、fitness 等 |
| `propagation.csv` | 本次最终状态到后续初值的变化，用于检查大更新之后初值是否持续偏移 |
| `event_context.csv` | 大更新附近的逐帧上下文，包括前后位姿、误差及匹配质量 |
| `event_persistence.csv` | 大更新之后各帧初值相对事件前状态的累计变化（包含中间其他 NDT 更新） |
| `tf_chain_audit.csv` | 云时刻实际 odom→base_link TF 与内部最终位姿的几何链核查 |
| `post_init_timeseries.svg/png` | 正式阶段三段误差、更新和匹配质量随时间变化 |
| `update_effects.svg/png` | 每次 NDT 净效果、XY散点与 map→odom/final 状态变化 |
| `stability_increments.svg/png` | 相邻 XY/yaw 增量和上一帧 NDT/FAST-LIO 对下一帧初值的贡献 |

三段位姿分别计算 XY RMSE、P95、最大值、相邻帧 ΔXY RMS 与 Δyaw RMS；同样报告
`map→odom` 后续状态变化。相邻帧统计和参考匹配必须保留失效情况，不能用零冒充缺失数据。
`map→odom` 的坐标状态变化不是对某个已知真值的误差，不能把其平移量直接视为
`base_link` 的真实定位误差。

对每次匹配计算 `Δe = e_ndt − e_init`，其中两者使用同一时刻动捕、同一固定对齐参考和
同一种 XY 误差定义。`Δe<0` 表示在此参考下改善，`Δe>0` 表示恶化；分别输出改善/恶化
比例、条件平均改善/恶化量以及最大恶化更新。它是**固定相对参考下**的净效果，不是对
未知绝对地图真值的判断。首次粗对齐的大修正量不参与这些统计。

大于 10、20、30 mm（严格 `>`）的更新次数与比例分别针对 NDT **候选修正**和实际应用的
`map→odom` 更新统计，并列出有效分母。拒绝候选不会改动已知的 `map→odom` 状态，
其实际应用量可为 0；没有足够状态信息时仍须标记未知，不得填 0。重点检查 30–50 mm
及更大更新前后的初值、原始结果、最终状态和后续初值，结合 fitness、迭代次数等判断
何时发生偏移、是否被接受、后续是否维持；单凭修正量大不能认定优化方向错误。

`T_final` 在诊断中是定位节点内部的云时刻 base_link 状态。在线 TF 桥使用平面化的
FAST-LIO 姿态，节点内部使用全3D外参，因此工具另读 `/tf` 核查这两套几何实现的差异。
组合本帧最终 map→odom 与云时刻 odom→base_link，只是在同一测量位姿下比较；不能声称
NDT尚未完成时，该最终状态就已广播。当前包没有录到人工初值话题，后续录包命令已加入
`/initialpose`、`/initialpose_user`、`/initialpose_checked` 供核查重复初始化。

当前包按预定规则得到的正式段包含 match 15 约 0.81 m 更新；排除的 9–11 帧也仍明显变化。
报告会明确告警“固定跳过3次不保证已稳定”。该包应解读为初始化后的持续收敛/跳变审计，
不能称作已稳定的22 mm精度结果，也不应为降低RMSE再事后增加剔除帧数。

以后采集此类包时，先启动录包，再启动定位并人工发送一次 `2D Pose Estimate`；
初始化后继续静止录制至少 60–120 秒。不要只按冷启动首帧实验等待 4 秒就停包。
全程不自动驱动车辆，不修改 NDT 参数或接受条件。动捕接收时钟仍未等价于相机测量时钟，
此流程用于静止稳定性，不据此声称动态同步精度。

### 起始静止窗口对齐后的相对误差（保留的通用模式）

车辆先在动捕覆盖内静止至少 25 s。当前 NDT 默认每 2 s 执行一次，25 s 的窗口才能在保留默认“至少 10 个匹配样本”要求时可靠完成起始对齐。工具用初始窗口中时间匹配的“最终 map 位姿”和动捕车体中心，估计一次固定的 `map <- mocap_world` SE(2)；该窗口会从误差统计中排除。原始 NDT 位姿和最终位姿始终复用这一**同一个**变换。

`robot_data/config/mocap_tracker0.yaml` 记录了用户确认的完整固定安装变换：Tracker0
刚体原点在 ROS `base_link` 正上方 0.18 m，轴向相同。因此离线工具以完整 3-D 刚体
变换（不是仅改 frame_id）生成 base_link 参考位姿；实时系统仍只保留 Tracker0 topic，
不发布任何额外 TF。

```bash
cd /home/zkam/test_nav
python3 ndt_match_evaluation.py \
  --bag robot_data/experiments/ndt_evaluation/<RUN_ID>/bag \
  --output robot_data/experiments/ndt_evaluation/<RUN_ID>/report_start_aligned \
  --ndt-topic /static_localization/ndt_match_diagnostic_detail \
  --mocap-topic /mocap/Tracker0/pose \
  --tracker-to-base-config robot_data/config/mocap_tracker0.yaml \
  --alignment-window-seconds 25 \
  --alignment-min-samples 10 \
  --alignment-max-static-motion-m 0.05 \
  --max-interpolation-gap-seconds 0.10
```

若录包从 NDT 初始收敛阶段开始，可显式指定一个**预先记录的**静止等待时间，例如
`--alignment-start-delay-seconds 12`。等待期和随后的对齐窗口都会从误差统计中排除。
该值不得由最小化待评估轨迹误差自动选择；报告会保存该值。更推荐的现场做法是：看到
`/static_localization/ready=true` 后，车辆继续静止约 12 s，再开始录包，使该参数保持默认 `0`。

动捕只在相邻有效样本间线性位置插值、SLERP 姿态插值；超过 `--max-interpolation-gap-seconds` 的缺帧绝不跨越插值。点云 header stamp 是唯一的配对时刻，NDT 完成时间只用于计算耗时。

### 仅用于后续漂移：以首个 NDT 原始输出为零点的相对误差

如果只想观察**首帧之后**的相对漂移，可使用 `first_raw_ndt` 模式。它选择
第一个同时满足“已收敛、有 NDT 原始输出、可在点云时刻匹配动捕”的记录，建立唯一固定变换；
该条 **raw NDT 输出的相对误差按定义为零**，后续仅统计相对这个首帧的变化。

```bash
python3 ndt_match_evaluation.py \
  --bag robot_data/experiments/ndt_evaluation/<RUN_ID>/bag \
  --output robot_data/experiments/ndt_evaluation/<RUN_ID>/report_first_ndt_relative \
  --ndt-topic /static_localization/ndt_match_diagnostic_detail \
  --mocap-topic /mocap/Tracker0/pose \
  --tracker-to-base-config robot_data/config/mocap_tracker0.yaml \
  --alignment-mode first_raw_ndt \
  --max-interpolation-gap-seconds 0.10
```

报告会在 `summary.json` 的 `first_raw_ndt_baseline` 记录实际使用的 `match_id` 和时间戳；
`matches.csv` 的 `is_relative_baseline=true` 标出该帧。此模式不重写在线 `map -> odom`，也不声称
评估首帧之前的历史误差；它专门用于看“首帧 NDT 之后定位链还产生了多少相对变化”。
**它不能用于判断第一帧 NDT 是否拉坏初值**，因为首帧 raw NDT 输出会被按定义置为零。

### 冷启动首帧修正量审计（使用首帧 `T_init`，不归零 NDT 输出）

此模式观测首次 NDT 相对人工粗初值的修正量；它不能判断绝对定位好坏，也不能单凭
修正量认定“拉坏初值”。必须采集**新冷启动包**，不能拿运行中的 V2 包替代。
录包先启动，随后完整启动 `localization.sh`；车静止至少 5 秒但不要发送 `2D Pose Estimate`，
再只发送一次初值并等待约 4 秒，随后立即停止定位和录包。这样第一条真正 NDT 记录中的
`T_init` 不会继承先前接受的 NDT `map -> odom` 更新。

离线分析使用：

```bash
python3 ndt_match_evaluation.py \
  --bag robot_data/experiments/ndt_cold_start/<RUN_ID>/bag \
  --output robot_data/experiments/ndt_cold_start/<RUN_ID>/report_first_init \
  --ndt-topic /static_localization/ndt_match_diagnostic_detail \
  --mocap-topic /mocap/Tracker0/pose \
  --tracker-to-base-config robot_data/config/mocap_tracker0.yaml \
  --alignment-mode first_init \
  --max-interpolation-gap-seconds 0.10
```

`first_init` 只把首帧 **优化前** `T_init` 对齐为零；它不会把 `T_ndt` 或 `T_final` 归零。
因此同一条基线记录中 `raw_error` / `final_error` 的 XY 和 yaw 就是首次 NDT 相对初值的改变。
报告中的 `first_initial_prediction_baseline` 会保存实际 `match_id`、收敛/接受状态及是否有
raw/final 输出。该模式的统计**只计这一个首帧注册**；后续记录仍保留在 `matches.csv`，但不进入
冷启动首帧统计，避免第二次 NDT 已继承首帧 `map -> odom` 更新而污染结论。该结果回答“第一次
NDT 把初值改了多少”，不把手工 2D 初值本身的偏差伪装成 NDT 误差。`summary.json` 的
`cold_start_first_match` 以及 `summary.txt` 会直接列出该帧三段误差、`T_init -> T_ndt` 修正、
fitness、真实迭代次数和接受状态。

### 静止误差分解（固定偏移与相邻帧稳定性）

对成功的静止报告运行：

```bash
python3 ndt_static_error_analysis.py \
  --report-dir robot_data/experiments/ndt_evaluation/<RUN_ID>/report_start_aligned \
  --output robot_data/experiments/ndt_evaluation/<RUN_ID>/static_error_analysis \
  --pose-kind final \
  --outlier-threshold-m 0.04
```

工具拒绝没有明确 `Tracker0 -> base_link` 外参记录的报告，避免将刚体原点直接冒充
`base_link`。输出逐帧 `ex/ey/e_xy/yaw` CSV、XY 误差散点图、误差/fitness/校正量/迭代
时序图、异常帧表和相邻 NDT 帧相对运动误差。起始对齐会吸收常数
`mocap_world -> map` 和常数安装偏差，因此散点只能分解**对齐后的残余中心与随机散布**，
不能替代独立绝对外参标定。

### 定位链误差源分离（V2 detail 包）

在上述评估报告之后运行：

```bash
python3 ndt_chain_analysis.py \
  --report-dir robot_data/experiments/ndt_evaluation/<RUN_ID>/report_start_aligned \
  --output robot_data/experiments/ndt_evaluation/<RUN_ID>/chain_analysis \
  --case-match-ids 225,226,241 \
  --case-half-window 4
```

输出 `chain_frame_errors.csv`（每帧的 `T_init/T_ndt/T_final`、`ex/ey/e_xy/yaw` 和匹配诊断）、
`chain_summary.json/.md`（三阶段分别的 RMSE/P95/max 与相邻帧增量误差）、
`case_context.csv`（每个指定 match 前后 4 帧），以及 `chain_timeseries.*`、
`chain_relative_motion.*`。其判断原则是：只有在同一条 V2 记录中，`T_init` 相比 `T_ndt`
的误差变化才可归因于此次 NDT 优化；V1 旧包会明确报告该结论不可得。

### 可选：冷启动首帧的跨试验重复性（不继承历史 NDT `map -> odom` 更新）

这是用于回答“多次刚启动的第一帧 NDT 输出是否彼此稳定”的补充实验。它不替代前文的
`first_init` 首帧修正量审计或初始化后静止稳定性测试。**冷启动只能由现场流程保证，
不能从 bag 里自动证明**：每一包必须在启动 `localization.sh` 前开始录制，且下一次试验前必须
完整停止该脚本。每包保留第一个真正进入 NDT 的诊断，不会跳过首次失败/拒绝而挑选后续成功帧。

同一物理停车点（例如 P1）重复至少 5 次。该实验给出首帧输出围绕自身质心的**重复性**，
不能把 NDT 校正量冒充真实误差，也不输出绝对定位误差。

每次冷启动的录包命令（在启动定位前运行）：

```bash
cd /home/zkam/test_nav
source /opt/ros/humble/setup.bash
source install/setup.bash
RUN_ID="$(date -u +%Y%m%dT%H%M%SZ)_cold_p1_r01"
mkdir -p "robot_data/experiments/ndt_cold_start/${RUN_ID}"
ros2 bag record -o "robot_data/experiments/ndt_cold_start/${RUN_ID}/bag" \
  /static_localization/ndt_match_diagnostic_detail \
  /static_localization/ndt_match_diagnostic \
  /mocap/Tracker0/pose /mocap/Tracker0/diagnostics \
  /vrpn_mocap/Tracker0/pose /tf /tf_static /tf_lio_raw /cloud_registered \
  /initialpose /initialpose_user /initialpose_checked
```

将车静止，启动 `./localization.sh`，在 RViz 发送一次近似的 `2D Pose Estimate`；等待约 4 秒，
让第一条 NDT 诊断产生，再依次停止 `localization.sh` 和录包。之后完全重复 5 次，保持同一 P1
及同一地图/参数。录到若干包后汇总：

```bash
cd /home/zkam/test_nav
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 ndt_cold_start_analysis.py \
  --bag robot_data/experiments/ndt_cold_start/<P1_R01>/bag \
  --bag robot_data/experiments/ndt_cold_start/<P1_R02>/bag \
  --bag robot_data/experiments/ndt_cold_start/<P1_R03>/bag \
  --bag robot_data/experiments/ndt_cold_start/<P1_R04>/bag \
  --bag robot_data/experiments/ndt_cold_start/<P1_R05>/bag \
  --output robot_data/experiments/ndt_cold_start/p1_summary
```

输出 `first_matches.csv`（首个注册和首个接受结果都保留）、`summary.*` 和
`cold_start_xy_scatter.svg/png`；它们均是相对/重复性结果。

## 建议实测序列

所有动作由操作者手动完成；本项目的脚本不会下发车体运动。

1. 起始静止至少 25 s，作为对齐窗口（该窗口不计入误差）；再在 3–5 个不同位置各静止约 30 s。
2. 在动捕覆盖区慢速直行并原路返回，重复 3 次。
3. 原地向左、向右各旋转约 90°，保持数秒。
4. 执行 L 形或矩形路线，重复 3 次。

每次单独录包，并在报告名中标注场景、速度、日期、地图版本、NDT 参数和动捕覆盖范围。

## 仍需现场核实

1. FAST-LIO `/cloud_registered` 的 header stamp 是否是去畸变参考时刻（通常需要检查实际 FAST-LIO 配置/源码和录包，而不能由本工程假定）。
2. VRPN 接收时间与相机采样时间的固定延迟，若要设置 `--time-offset-seconds`，必须独立测量。
3. Tracker0 安装外参的物理条件仍须保持：刚体原点正上方 0.18 m、轴向与 base_link
   一致。只要刚体原点、轴向或安装高度改变，就必须重新测量并更新 YAML。
