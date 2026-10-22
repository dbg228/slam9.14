# 动捕反馈下的控制器测试

本包替代已退役的 S 测试。只提供直线、左/右单向平滑圆弧。原 MPPI、底盘控制、正式导航参数及第 2 创新点规划器保持原样。正式入口仍是根目录的 `localization.sh`、`navigation.sh` 与 OmniBot 的 `robot_control.launch.py`。

架构与真实接口见 [ARCHITECTURE.md](../../ARCHITECTURE.md)，清理范围见 [CLEANUP_REPORT.md](../../docs/controller_tracking/CLEANUP_REPORT.md)。软件检查与实车检查分别记录在 [VALIDATION.md](../../docs/controller_tracking/VALIDATION.md)。

## 准备与填写

在工作区根目录执行。首次构建：

```bash
source /opt/ros/humble/setup.bash
source /home/zkam/OmniBot_ws/install/setup.bash
colcon --log-base tests/controller_tracking/.log build \
  --base-paths tests/controller_tracking_probe tests/controller_tracking \
  --build-base tests/controller_tracking/.build \
  --install-base tests/controller_tracking/.install --executor sequential
source tests/controller_tracking/.install/setup.bash
```

每个新终端均 source ROS、OmniBot 与上述实验 install。复制配置后编辑，**源文件相对路径按配置文件所在目录解析**：

```bash
cp tests/controller_tracking/config/experiment.yaml tests/controller_tracking/config/site.yaml
```

必须填写：`calibration.id`、实测地面旋转 `world_level_rotation_xyzw` 与 `level_verified`、两种场地 bounds、标记可见性/停车余量/现场停车确认。bounds 以**冻结起点、起始车头 +X、左侧 +Y**为准，包含车体扫掠和停车余量。示例尺寸不代表场地足够；工具不自动缩放路径或调速度。

沿用 `robot_data/config/mocap_tracker0.yaml` 已确认的 `T_Tracker0_base_link=(0,0,-0.18m)`，刚体定义未变。相机重新标定后给新标定编号、重新核实尺度/地面，不沿用旧地图对齐。当前时间来自 Ubuntu VRPN 接收端；相机曝光时间、真实相机延迟未测，保持 null/false，不能将其当零延迟。

`wheel_feedback.velocity_units_verified=false` 必须保留至实测确认 0x606C 单位。转向/滚动限幅阈值缺失时保持 null，不能填猜测值。

## 预览与冻结

```bash
ros2 run controller_tracking tracking_test preview \
  --config tests/controller_tracking/config/site.yaml --kind left_arc \
  --output /tmp/tracking_preview_left --publish
```

可选 kind 为 `line`、`left_arc`、`right_arc`。输出 PNG、Path JSON、扫掠范围和终点阶段覆盖诊断；无 CSV。RViz Fixed Frame 设 `tracking_world`，添加 `/tracking/reference_path` 的 Path，或用配置 `config/preview.rviz`。预览不发送目标。短圆弧可能全部落在 PathFollowCritic 的 1.4m 终点阶段内；查看 `preview_check.json`，不能由上限 0.30m/s 宣称恒速。

确认尺寸后，车停稳，在同一终端只接入动捕（原有独立接入脚本不改）：

```bash
./mocap_tracker0_vrpn.sh
```

另一个终端冻结一次测试坐标系和三种路径：

```bash
ros2 run controller_tracking tracking_test freeze \
  --config tests/controller_tracking/config/site.yaml \
  --output /home/zkam/test_nav/tests/controller_tracking/runs/batch_01
```

至少 0.5s 静止有效数据确定一次原点/朝向，地面旋转来自事先配置。之后不得修改该目录的 frame/config/path；换标定、尺寸或参数需新建 batch。同一 batch 重复实验手动复位，不重新锚定。配置当前选定的路径必须在已确认范围内，其他两种路径执行前也逐一检查。

## 启动状态链与数据检查

底盘按原方式手动启动；测试时不要同时运行正常 `navigation.sh`。FAST-LIO/NDT 不参与测试状态，测试 launch 也不启动它们。实验 TF 独立，即使原定位进程仍运行也无法进入该反馈链。

```bash
ros2 launch controller_tracking feedback_test.launch.py \
  session:=/home/zkam/test_nav/tests/controller_tracking/runs/batch_01

# 另一个终端：只检查，不发运动目标
ros2 run controller_tracking tracking_test check \
  --session tests/controller_tracking/runs/batch_01 --output /tmp/tracking_check.json
```

launch 启动 controller_server、原速度平滑器、动捕反馈及**未激活的命令门**，不发运动目标。检查 TF/命令唯一发布者、原 `/odom` 残留订阅、生命周期、参数和有效数据。正式导航与 UDP 不能同时发布 `/nav/cmd_vel`。

## 每轮先录包，再由你手动启动

每轮一个目录，不复制大录包、不自动压缩。终端 A：

```bash
ros2 run controller_tracking tracking_test record \
  --session tests/controller_tracking/runs/batch_01 \
  --kind left_arc --run tests/controller_tracking/runs/batch_01_round_01
```

车手动回到冻结起点、停稳、确认现场停车后，将摇杆居中并切 EXTERNAL。**下面命令才会启动车子，必须由你运行**。终端 B：

```bash
ros2 run controller_tracking tracking_test execute \
  --run tests/controller_tracking/runs/batch_01_round_01
```

只发送一次完整 FollowPath，controller_id=`FollowPath`、goal_checker_id=`goal_checker`；不自动接近起点、不重规划、不接入几何巡检规划器。收到 MPPI 封装内部 setPlan 回显后核实完整路径。动捕无效、记录器退出、命令超时、执行端心跳丢失、模式接管或超时会锁定停止、取消本轮并保留原因。不得丢帧后自动恢复或切回原定位。正常 goal 容差仍为 0.20m / 3.14rad，因此 action 成功不等于高精度达标。

执行结束确认 `execution.json.stop_check`，终端 A 按 Ctrl+C 结束录包后再分析；不得在写包过程中读取 SQLite。失败轮次也保留。下一轮手动复位，用 `_round_02`、`_round_03` 新目录重复同一路径，不覆盖已有 execution。

```bash
ros2 run controller_tracking tracking_test analyze --run tests/controller_tracking/runs/batch_01_round_01
ros2 run controller_tracking tracking_test compare \
  --batch tests/controller_tracking/runs/batch_01 --kind left_arc \
  --output tests/controller_tracking/runs/batch_01_comparison
```

每轮保存 `session/` 冻结快照、完整 `bag/`、`path.json`、原始请求哈希、插件接收检查、运行参数/图检查、事件 JSONL、执行结果/停车检查，以及 `analysis/` 的 JSON、PNG 和报告。**默认不生成 CSV**。

`compare --batch` 自动收集该 batch 与路径类型的所有录包轮次，包括预检失败；有轮次尚未停止录包或分析会报错，避免遗漏失败。也支持显式 `--runs`，需列全所有尝试。

## 指标及解释边界

离线用相同 base_link 参考点，计算到路径**线段投影**的有符号横向误差、±π 包裹航向差；全程和冻结阶段（lead/entry/arc/exit/tail，直线为 line）都统计。插值不外推，不跨 >max_gap_s 缺口，无效状态不填零。短间隔无效事件同样切断支持区间。速度使用与控制器相同的动捕估计，保留约半窗口延迟。MPPI 真正输入另录，可看 Nav2 速度阈值的作用。

计算耗时是测量封装对**原 MPPI computeVelocityCommands**的 steady-clock 包围时间；控制间隔另算，二者不能混淆。没有四轮真实目标舵角，因此只画实际舵角、实际下发的转向/滚动速度接口目标与反馈。未验证反馈单位时数值差仅供核查，物理执行误差为 null；没有 CAN 到达时刻，不宣称精确通信延迟。

离线汇总保留所有失败原因和分母，标定/冻结坐标/路径/状态参数哈希不同会标记不可直接同组比较。软件验证用合成数据时明确标 `software_fixture`；本轮未进行实车启动和实测。

现有 `validity` 是 normalizer 对有限位姿/四元数、frame、时间和停流的检查；VRPN PoseStamped 不含相机原生跟踪置信度/遮挡标志。正常频率发送的“冻结但未标无效”位姿无法由此独立识别，覆盖率必须按这一限制解释。

map与mocap_world另有[只读坐标核查流程](../../docs/controller_tracking/MAP_MOCAP_RECHECK.md)，包括历史对齐方法、两个变换方向和冻结后的独立位置验证。它不参与本包的动捕测试反馈，不修改导航TF。

新增[固定三维世界坐标标定](../../docs/controller_tracking/FIXED_WORLD_CALIBRATION.md)：按用户澄清，总共四次停放，同一 base_link 原点的前三个不共线位置联合求 SE(3)，第四个位置独立验证。支持 NDT 测量时刻位姿或逐位置人工录入地图坐标；提供 `world_calibration` 采集/录入/求解/验证命令，输出独立安装外参、原始 JSONL、双向标定 YAML 和误差报告，不自动修改 TF。
