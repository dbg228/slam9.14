# ROS 2 工程架构

本轮只整理测试层。正式定位、导航、几何规划及底盘代码、参数与原有修改保留。旧 S 测试先提取公共计算与探针，再按清单退役。

## 职责与配置入口

| 位置 | 职责 |
|---|---|
| Livox/FAST-LIO、`src/static_map_localization` | 正常雷达/IMU里程计、PCD NDT、map→odom与正式Nav2，入口 `livox_driver.sh`、`localization.sh`、`navigation.sh` |
| `lio_tf_to_base_tf.py`、`map_to_odom_bridge.py` | 正常定位参考点与里程计桥接，保持原入口 |
| `src/inspection_geometry_tools` | 第2创新点几何路径规划/执行，行为和已有修改保留，不进入本轮实验 |
| `src/localization_ab_test`、根定位评估脚本、`tests/ndt_match_evaluation` | A/B/C及定位评估，源码和数据保留；无关API不重写 |
| `mocap_tracker0_vrpn.sh`、`mocap_pose_normalizer.py` | 已有VRPN接入、毫米→米、四元数归一与有效性；不发布TF、不应用安装偏移 |
| `robot_data/config/mocap_tracker0.yaml` | 单一动捕单位/接入/Tracker0→base_link安装标定来源 |
| `tests/controller_tracking/.../common.py` | 迁出的共用SE(2)、角度、有界插值、路径线段投影与统计 |
| `.../state.py` | 唯一完整3D刚体偏移/地面旋转/冻结坐标变换与一致速度估计 |
| `.../geometry.py` | 按弧长采样的直线/左右单向平滑圆弧、车体扫掠及终点阶段诊断 |
| `.../feedback.py` | 整数时间戳有效性配对，动捕反馈及独立TF |
| `.../configuration.py` | 正式参数只读、实验overlay、配置差异及冻结完整性检查 |
| `.../gate.py` | 默认未激活的实验命令门，反馈/命令/执行心跳失效时锁定停止 |
| `.../experiment.py`、`cli.py` | 预览、冻结、只读检查、记录、用户手动FollowPath与每轮归档 |
| `.../analysis.py` | 全程/预定义阶段统一分析，失败轮次汇总，默认不输出CSV |
| `tests/controller_tracking_probe` | 测量委托：原样调用已安装MPPI，记录实际输入/路径/计算时间 |
| `.../world_calibration.py`、`world_capture.py` | 独立 map↔mocap_world 三维固定标定；3个停放位置拟合、1个位置验证；只读 NDT/动捕，不发布 TF/命令 |
| `/home/zkam/OmniBot_ws` | 原摇杆、仲裁、轮组逆解、ros2_control与CAN驱动，本轮不修改 |

正式Nav2配置仍是 `robot_data/config/navigation.yaml`。实验配置复制 `tests/controller_tracking/config/experiment.yaml` 为同目录 `site.yaml` 并填写实测项；freeze生成独立 `nav2.yaml`、`mocap.yaml`、`config.yaml`、`frame.json`、路径及哈希，不覆盖正式配置。

固定世界标定是额外的只读离线支线，不进入正式导航或动捕反馈链，也没有关键 TF 的发布权。命令、坐标/时间来源和输出目录见 [FIXED_WORLD_CALIBRATION.md](docs/controller_tracking/FIXED_WORLD_CALIBRATION.md)。每次会话自己的 `session.yaml`、`body_extrinsic.yaml` 和结果存放在 `robot_data/experiments/fixed_world_calibration/<会话名>/`；旧起始对齐数据与新三维标定严格区分。

## 两条运行链

正常导航：

```text
Livox/IMU → FAST-LIO → /tf_lio_raw camera_init→body
 → lio_tf_to_base_tf → /tf odom→base_link
PCD/NDT → map_to_odom_broadcaster → /tf map→odom
TF桥接 /odom → controller_server速度输入
planner / FollowPath → MPPI FollowPath DiffDrive
 → /cmd_vel_nav → OPEN_LOOP velocity_smoother → /nav/cmd_vel
 → arbitrator_controller → /reference_velocity
 → omni_velocity_controller → ros2_control速度接口 → CAN → 四轮
```

正常启动顺序仍为原底盘/传感器→定位（需要时RViz初值）→navigation.sh→用户目标。原fixed-PCD导航的map_server、Nav2 remap与正常入口不变。

动捕反馈实验：

```text
VRPN Tracker0(raw mm) → 原normalizer(m)+validity
 → T_Tracker0_base_link完整3D偏移 + 实测地面旋转
 → 一次冻结的tracking_world起点/朝向
 → 同一中点姿态/速度 → /tracking/state
 → /tracking/tf tracking_world→base_link → isolated local costmap → MPPI实际位姿
/tracking/state → MPPI实际速度（保留Nav2最小速度阈值）
固定tracking_world Path → /tracking/follow_path → 测量委托中的原MPPI
 → /tracking/controller_cmd → 原速度平滑器 → /tracking/smoothed_cmd
 → 用户手动激活command_gate → /nav/cmd_vel → 原仲裁/底盘/CAN
```

实验只启动跟踪必需的controller_server、velocity_smoother、生命周期管理、反馈及未激活命令门，不启动地图、规划器、BT、FAST-LIO或NDT。完整路径只交给FollowPath一次，不全局重规划、不自动接近起点。原定位即使仍运行，也不进入实验的私有TF或速度输入。

launch不发Twist或目标。用户运行execute才激活命令门；其他正常导航/UDP发布者并存时拒绝激活。动捕中断、执行端心跳丢失、记录器退出、模式接管或超时锁定停止并取消本轮，不自动恢复、不回退FAST-LIO。原MANUAL/STOP、回零、硬件门控保留。

## 实际接口与时间

| 话题 | 消息类型 | 参考点/坐标及时间 |
|---|---|---|
| `/vrpn_mocap/Tracker0/pose` | geometry_msgs/PoseStamped | mocap_world中的Tracker0，原始mm，Ubuntu接收时间；不是相机曝光时间 |
| `/mocap/Tracker0/pose` | PoseStamped | 同一刚体、米、同时间戳，无安装偏移/TF |
| `/mocap/Tracker0/diagnostics` | diagnostic_msgs/DiagnosticArray | 同整数ns的valid/reason、接收时间；无效/stale不填零 |
| `/tracking/base_pose` | PoseStamped | 完整安装/冻结变换后的base_link，未做速度窗口平滑，离线精度参考 |
| `/tracking/state` | nav_msgs/Odometry | header=tracking_world、child=base_link；姿态和机体vx/vy/wz共用窗口中点时间；covariance未标定，以-1显式标未知 |
| `/tracking/state_status` | std_msgs/String(JSON) | valid/reason、源/拟合ns、窗口延迟、处理耗时、有效计数 |
| `/tracking/input_pose` | PoseStamped | 原MPPI computeVelocityCommands的实际参数，原样保留查询header，不假设为源采样时刻 |
| `/tracking/input_speed` | geometry_msgs/TwistStamped | 原MPPI实际收到、经Nav2阈值处理的速度；header为调用时刻，不能冒充测速时间 |
| `/tracking/reference_path` | nav_msgs/Path | 完整请求路径，tracking_world，切线姿态 |
| `/tracking/received_plan` | Path | 原MPPI setPlan返回后的完整回显，核验当前请求的frame、stamp和点；不是局部裁剪计划 |
| `/tracking/compute_timing` | String(JSON) | sequence、调用ROS ns、steady-clock原MPPI计算ns、成功/异常 |
| `/tracking/controller_cmd`、`/tracking/smoothed_cmd` | geometry_msgs/Twist | 机体系，MPPI/平滑输出，无header，以bag接收时间记录 |
| `/nav/cmd_vel` | Twist | 命令门输出/原仲裁输入；测试时唯一command_gate，正常模式由正式平滑器发布 |
| `/reference_velocity` | Twist | 仲裁后目标，nav_w_scale=-1保持原样，比较wz需处理此反号 |
| `/omni_velocity_controller/diagnostics/ref_vel` | Twist | 内部输入/规划诊断；linear.z重载规划vx，不是垂直速度 |
| `/omni_velocity_controller/diagnostics/joint_vel` | std_msgs/Float32MultiArray | 真正接口目标[turn1..4,roll1..4]角速度rad/s，CAN编码前，不是CAN到达证明 |
| `/omni_velocity_controller/diagnostics/fai` | Float32MultiArray | raw实际角1..4、滤波实际角1..4，**不是目标舵角** |
| `/joint_states` | sensor_msgs/JointState | wheel_1..4_turn/roll位置速度；广播器时间，非逐电机同时采样 |
| `/tracking/events`、`/tracking/gate_status` | String(JSON) | 请求、接收、失败、取消、录包心跳与停止原因 |
| `/teleop/mode` | std_msgs/String | 原MANUAL/EXTERNAL/STOP，程序不替用户切模式 |

速度只在 `state.py` 实现一次：最近约0.10s有效位姿，等时间网格线性拟合求导，姿态也取同一窗口中点。相对最新样本约0.05s延迟，另加VRPN/回调延迟。遇无效、乱序或>0.05s缺口重置；不外推到now、不将旧TF刷新时间。接收时间和曝光时间有区别；相机真实延迟保持未知，不能凭轨迹重新拟合时间/坐标偏移。

## TF唯一发布者

| 模式/话题 | 关键TF | 唯一发布者 |
|---|---|---|
| 正常 `/tf_lio_raw` | camera_init→body | FAST-LIO |
| 正常 `/tf` | odom→base_link | lio_tf_to_base_tf桥接，保留LIO时间 |
| 正常 `/tf` | map→odom | static_map_localization的独立map_to_odom_broadcaster；不同时启AMCL |
| 正常 `/tf`、`/tf_static` | base_link→固定安装/关节frame | 原robot_state_publisher，沿原URDF |
| 实验 `/tracking/tf` | tracking_world→base_link | **/tracking/mocap_feedback**，仅新有效中点状态时发布，无map/odom边 |
| 实验 `/tracking/tf_static` | 机器人固定安装边 | **/tracking/mocap_feedback**只读转发正常/tf_static，拒绝child=base_link/map/odom/tracking_world |

controller/costmap的TF话题绝对remap到私有话题。正常树不能进入它们的buffer；转发仅保留机器人静态安装关系。移动轮关节TF仍在原树，不作为实验定位输入。RViz可将/tf和/tf_static重映射到实验话题，Fixed Frame=tracking_world。

## 四项参数差异及保留行为

生成的 `parameter_diff.json` 精确列出与正式选择段的四项差异：controller_server和velocity_smoother的odom_topic改成/tracking/state；local_costmap.global_frame改成tracking_world；FollowPath.plugin使用测量委托、内部仍加载同一原MPPI。所有MPPI权重、模型、采样、阈值、速度和goal checker不改。

启动上的差异是namespace/私有TF/话题、只启动跟踪服务器与新增默认关闭命令门。平滑器仍OPEN_LOOP，启动时不创建odom订阅；即使后续需要闭环，参数也已指向动捕。仲裁不读里程计。实际图检查拒绝实验节点残留/odom、/Odometry、/tf_lio_raw等输入。

局部costmap的插件、足迹与膨胀照原配置复制；原CostCritic和obstacle_layer禁用状态按此前用户要求保留，不自动启用或关闭其他机制。FollowPath DiffDrive、vx_max=.30、wz_max=.65；平滑器30Hz、wz上限.45。goal_checker容差.20m/3.14rad不改，action成功不等于高精度达标。

短场地中[PathFollowCritic](https://raw.githubusercontent.com/ros-navigation/navigation2/1.1.20/nav2_mppi_controller/src/critics/path_follow_critic.cpp)的1.4m终点阈值、[GoalCritic](https://raw.githubusercontent.com/ros-navigation/navigation2/1.1.20/nav2_mppi_controller/src/critics/goal_critic.cpp)的.7m阈值可能覆盖大部分弯道；预览计算完整终点距离的近似覆盖比例，MPPI裁剪后终点可能不同，不能将比例当内部激活日志。速度上限与恒速要求分开记录，默认未给恒速目标。

核查来源：[Nav2 1.1.20 controller_server](https://raw.githubusercontent.com/ros-navigation/navigation2/1.1.20/nav2_controller/src/controller_server.cpp)、[velocity_smoother](https://raw.githubusercontent.com/ros-navigation/navigation2/1.1.20/nav2_velocity_smoother/src/velocity_smoother.cpp)、[原MPPI接口](https://raw.githubusercontent.com/ros-navigation/navigation2/1.1.20/nav2_mppi_controller/src/controller.cpp)，以及本机 `nav_2d_utils/odom_subscriber.hpp`（最新Odometry缓存速度）。

## 启动顺序、输出与边界

底盘手动启动→动捕接入→实测场地/地面确认及preview→停稳freeze→feedback_test.launch→check→record→用户EXTERNAL并手动execute→停车核查→停止录包→analyze→同起点重复三轮compare。具体命令见 [测试README](tests/controller_tracking/README.md)。本轮代理未启动实车。

输出在 `tests/controller_tracking/runs/`：一组一个冻结batch，一轮一个目录，包含配置快照、完整bag、路径/请求/插件实收证据、事件、运行参数、执行与停车结果、JSON/PNG分析。不复制大录包，默认无CSV；失败轮次保留并进入分母。`.validation/`只放明确标记software_fixture的软件数据。

尚缺真实目标舵角、CAN到达和逐轮采样时刻、验证后的0x606C速度单位、相机曝光延迟。新相机标定id、地面旋转、场地bounds需现场填写；已确认18cm偏移保留。软件和实车结论分别见 [VALIDATION.md](docs/controller_tracking/VALIDATION.md)。
## 动捕初始定位与原生地图（2026-10-17 修订）

独立包 `tests/mocap_anchored_mapping` 不改正常定位、MPPI、底盘和几何规划算法。
`core.py` 负责同一 base_link 参考点、完整 SE(3)、静止质量和冻结输入；
`online.py` 复用 controller_tracking 的动捕有效性/原始单位核查，采集一次静止初始化，并提供只读独立点验证；
`products.py` 只检查原生 PCD、复用原切片/去噪程序，不含点坐标转换；
`cli.py` 与 `launching.py` 分离建图、重启定位、手动导航服务，`isolation.py` 隔离原生硬编码 Log/PCD 输出。

建图时 map=首次 FAST-LIO 原生世界 F0。采集 M←B 与 F0←B，保存固定 M←map 与逆变换及地图原点坐标，
全局 TF 仅 initial_anchor 发布恒等 map→camera_init、动态 camera_init→base_link。
不发布动捕刚体→base_link，也不把 M←F0 用于重写点云。原生 body TF 留私有话题。

重启定位时原生世界变为 O，沿用已保存 map←M，静止采集本次 M←B 与 O←B，
计算本次固定 map←odom，原世界标定不重拟合。native_base_bridge 唯一发布静态 map→odom、
恒等 odom→camera_init 与动态 odom→base_link；无 NDT、无初值话题、无第二个全局校正发布者。
原 robot_state_publisher 保持发布车体以下安装/关节。初始定位后仍由 LIO 更新，不是持续动捕定位。
/odom 复用原 TF 差分桥，Nav2 复用原参数逐字快照和原速度链。

数据按轮归档到 robot_data/experiments/mocap_initial_mapping/<run>/，包括原始初始化日志、
calibration.yaml、原生 raw/native_F.pcd、原生 grid/map.yaml/pgm、各次 localizations/<id> 和 validation/<id>。
不再存在 M PCD 转换或 M 高度配置。独立点只用冻结变换计算误差，不拟合/调参。
原定位/NDT 启动入口仍保留在 legacy 模式，不能与此流程同启。

实际话题、消息类型、时间戳来源、唯一 TF 表、全部终端命令与生成状态见
[当前操作说明](docs/mocap_anchored_mapping/README.md)，
退役范围见 [清理报告](docs/mocap_anchored_mapping/CHANGES.md)。
