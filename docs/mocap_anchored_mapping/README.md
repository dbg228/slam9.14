# 动捕初始定位与原生地图流程（当前方案）

2026-10-18：相机已移动，本轮重新建图，三组都在终点评价误差，第3组增加中途和终点静止纠错。使用 [本轮操作说明](../full_map_loop/README.md) 的新目录；下文 `20261017_native_01` 命令属于旧布局记录。

地图保留 FAST-LIO 原生坐标。动捕用于测定地图原点和方向、以及后续启动的初始定位。此前“map 定义成动捕世界、转换整张 PCD、重选动捕高度切片”的方案已经退役，专用代码和生成物按用户授权清理，详见 [CHANGES.md](CHANGES.md)。

## 坐标关系

令 M=`mocap_world`，B=`base_link`，F0=本次建图原生世界，O=以后重新启动的原生局部世界。`T_A_B` 将 B 坐标换算到 A。

建图时选定 **map=F0**，静止同步采样约 3 秒：

```
T_map_B   = T_F0_IMU × inverse(T_B_IMU)
T_M_B     = T_M_Tracker0 × T_Tracker0_B
T_M_map   = T_M_B × inverse(T_map_B)
T_map_M   = inverse(T_M_map)
```

每对样本计算刚体变换，SO(3) 均值与平移均值产生一个固定结果，不拟合尺度，不使用 NDT 位姿。保存后冻结。`calibration.yaml` 中 `map_origin_in_mocap_m` 就是 map 原点在动捕中的 XYZ，旋转说明 map 轴向；不能把初始车体 XYZ 直接当作 map 原点。

以后重新启动，原生世界已经换成 O，静止采集本次动捕/原生位姿：

```
T_map_B    = 已保存的 T_map_M × 当前 T_M_B
T_map_odom = T_map_B × inverse(当前 T_odom_B)
           = 已保存的 T_map_M × 本次估计的 T_M_odom
```

本次 `T_map_odom` 一次性保存并冻结。旧 `T_M_map` 不重拟合；不把上次 F0 的 TF 套给本次 O。本轮新建图、新定位均没有 NDT，旧地图不参与。

动捕基准保持当前原点、右手系/Z向上、米制，已由用户确认。Tracker0 未重建，独立 `T_Tracker0_B=[0,0,-0.18]m`、单位旋转沿用确认值。URDF `imu_joint` 的 `T_B_IMU` 为 `[0.37,0.28,0.15]m`、yaw=0.8229227423153264 rad；使用完整 SE(3)，FAST-LIO 内部 LiDAR→IMU 不重复应用。以后重新标定相机、移动动捕原点或改变刚体定义，会使这份固定世界关系失效，需要重新建立，不能静默沿用。供应商标定编号当前仍未提供。

## 本轮准备目录与文件

```
/home/zkam/test_nav/robot_data/experiments/mocap_initial_mapping/20261017_native_01/
├── config.yaml、extrinsics.yaml、mocap.yaml、fastlio.yaml、nav2.yaml
├── inputs.lock.json、products.json、state.json
├── calibration.yaml、calibration.lock.json     # 建图静止初始化后生成
├── initialization/raw.jsonl、pairs.json        # 本次原始采集和配对证据
├── raw/native_F.pcd、native_receipt.json       # 原生点坐标，save 后生成
├── grid/map.yaml、map.pgm、projection.json     # 原生切片/去噪，project 后生成
├── native_aux/Log、PCD                        # 原生硬编码输出的私有目录
├── localizations/<编号>/                     # 每次启动一份，失败也保留
│   ├── raw.jsonl、pairs.json
│   ├── initialization.yaml、initialization.lock.json  # 本次 T_map_odom
│   └── native_aux/Log、PCD
└── validation/<编号>/raw.jsonl、pairs.json、summary.json
```

本轮 `20261017_native_01` 已完成真实首次静止锚定（32 对、约 3.10 秒）、人工建图、原生 PCD 保存和二维地图投影。首次建图进程已退出；随后完成重启初始化及 3 个静止位置验证（95 对，XY RMSE=7.7848mm、3D RMSE=16.3170mm）。本次重启定位验证不等于原世界标定的独立精度认证。结果见运行目录 `validation_sessions/20261017T085200Z_three_static_points/report.md`。当前状态与本次命令见运行目录 SESSION_STATUS.md、COMMANDS.md。软件测试文件只位于包 `.validation`，不能作为实车地图。新目录不放 ROS 源码，包代码仍在 `tests/mocap_anchored_mapping`。

## 终端与启动步骤

先在原终端 Ctrl-C 停止旧定位/建图/导航。保持底盘、雷达、动捕接入；已有运行实例不重复开。新入口拒绝发现另一套定位 TF，不自动杀旧进程。实车全程人工遥控，不发导航目标。

终端 1，底盘（未运行才启动）：

```bash
source /opt/ros/humble/setup.bash
source /home/zkam/OmniBot_ws/install/setup.bash
ros2 launch omnibot_control robot_control.launch.py
```

终端 2，雷达（未运行才启动）：

```bash
cd /home/zkam/test_nav
./livox_driver.sh
```

终端 3，Windows 先开 Tracker0 VRPN，再启动接收（已运行不重复）：

```bash
cd /home/zkam/test_nav
./mocap_tracker0_vrpn.sh
```

终端 4，手柄居中、小车静止、在动捕覆盖区内：

```bash
cd /home/zkam/test_nav
RUN=/home/zkam/test_nav/robot_data/experiments/mocap_initial_mapping/20261017_native_01
./mapping_modes.sh anchored check --run "$RUN"
./mapping_modes.sh anchored preflight --mode mapping
./mapping_modes.sh anchored mapping "$RUN"
```

终端 5，观察初始化：

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /anchored_mapping/status --qos-durability transient_local
```

只有 `frozen=true`、`fault=null`、calibration.yaml 已保存，才人工遥控开始建图。不设置 2D Pose Estimate。终端 6 可以运行 `rviz2`，Fixed Frame=`map`，PointCloud2=`/anchored_mapping/cloud_F` 或 `/anchored_mapping/map_F`。map→camera_init 是恒等关系；动捕世界变换保存在标定文件，不用于改点坐标。

可选独立验证：人工换到另一个位置/朝向并停稳，保持终端 4 建图实例，终端 5 Ctrl-C 退出显示后采 3 秒：

```bash
cd /home/zkam/test_nav
RUN=/home/zkam/test_nav/robot_data/experiments/mocap_initial_mapping/20261017_native_01
./mapping_modes.sh anchored validate --run "$RUN" --context mapping
```

输出每样本 XYZ、XY、三维距离、姿态误差和 RMSE/P95/最大值，静止/时间/有效性核查也保留。**没有拟合步骤，不会更新 calibration.yaml。**用于评价固定变换后的动捕与 LIO 一致性/漂移，不是对动捕系统本身的独立计量认证。

结束建图，先停稳，保持终端 4 运行，在终端 5 保存：

```bash
./mapping_modes.sh anchored save --run "$RUN"
```

看到成功后，才在终端 4 Ctrl-C。不要调用旧 save_map.sh 保存这轮，也不要依赖退出自动保存。原生保存来自已有 map_publish_callback 累积世界扫描，未声称经过回环优化合并。

终端 5，直接按原生坐标投影：

```bash
./mapping_modes.sh anchored inspect --run "$RUN"
./mapping_modes.sh anchored project --run "$RUN"
./mapping_modes.sh anchored products --run "$RUN"
```

**没有 convert 命令，没有 M 坐标 PCD，没有 projection.yaml 需要重填。**复用现有 `pcd_to_2d_map.sh` 的全部切片与去噪参数，包括原生 z=0.05–0.35 m 与原多高度证据层；仅换输入/输出和关闭大调试输出。原始 PCD 的 SHA 在投影前后必须一致。新栅格 origin 仍按新点云范围生成，不复制旧栅格 origin。若现场地图投影质量本身不适合，这属于原生切片配置问题，不能归因于未做动捕点云转换。

后续重新启动定位，终端 4：

```bash
./mapping_modes.sh anchored localization "$RUN"
```

小车在同一动捕世界覆盖区内保持静止约 3 秒，终端 5 查看：

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /anchored_localization/status --qos-durability transient_local
```

`capture_dir` 指向本次启动目录。等待 frozen=true、fault=null；不设置 RViz 初值，不启动 NDT。新二维 map_server 会加载本轮栅格；新开 RViz 避免旧图/TF 缓存。之后可人工遥控离开动捕区域，本次 map→odom 保持固定，由 LIO 更新车体。

在另一位置独立验证本次启动（保持终端 4，停车后终端 5）：

```bash
./mapping_modes.sh anchored validate --run "$RUN" --context localization \
  --session "$RUN/localizations/替换为状态中的本次编号"
```

如需检查导航服务，终端 5：

```bash
./mapping_modes.sh anchored navigation "$RUN"
```

启动前核对本轮身份、动捕初始定位完成与原生状态新鲜；不再等待 NDT ready，也不会自动发目标。MPPI、平滑、仲裁和底盘链沿原参数/重映射。实际导航运动由用户另行手动启动。

重新建图必须新建目录，原生 F0 不能重启后续接；失败目录保留：

```bash
./mapping_modes.sh anchored prepare \
  --site /home/zkam/test_nav/tests/mocap_anchored_mapping/config/site.yaml \
  --run /home/zkam/test_nav/robot_data/experiments/mocap_initial_mapping/新的运行编号
```

原方法入口保持：

```bash
./mapping_modes.sh legacy mapping
./mapping_modes.sh legacy localization
./mapping_modes.sh legacy navigation
```

建图与定位互斥，导航另终端；勿同时启动新旧定位。

## 实际数据链、时间与唯一 TF

| 数据 | 话题/类型 | 含义 |
|---|---|---|
| 原始动捕 | /vrpn_mocap/Tracker0/pose，PoseStamped | Tracker0，mocap_world，mm，本机 VRPN 接收时刻 |
| 米制动捕 | /mocap/Tracker0/pose，PoseStamped | 同一刚体/同一 stamp，乘 0.001；不是 base_link |
| 有效性 | /mocap/Tracker0/diagnostics，DiagnosticArray | 同纳秒 stamp 配对，有限数/四元数/过期；无原生光学置信度 |
| 新建图原生位姿 | /anchored_mapping/native_odom，Odometry | camera_init→body(IMU)，lidar_end_time；twist 不作为有效速度 |
| 新定位原生位姿 | /anchored_localization/native_odom，Odometry | 本次 O→IMU；同样先换成 B |
| 原生 TF | 对应前缀 /native_tf，TFMessage | 私有 camera_init→body，不进全局树 |
| 车体位姿 | 对应前缀 /base_pose_F，PoseStamped | 本次原生世界→base_link，保留源时间 |
| 停车观测 | /reference_velocity，Twist | 最终底盘命令，无 header，记录接收时间 |
| 初始化状态 | 对应前缀 /status，String(JSON) | run/capture_dir/frozen/fault/native_stamp_ns/无 NDT |
| /odom | 原 tf_to_odom_bridge.py，Odometry | 加载阶段复用原桥，50Hz 查询最新 TF、按发布时刻差分，非动捕持续速度反馈 |

| 模式 | 全局 TF 边 | 唯一发布者 |
|---|---|---|
| 建图 | map→camera_init，恒等静态 | initial_anchor |
| 建图 | camera_init→base_link，动态 | initial_anchor |
| 重启定位 | map→odom，本次冻结静态 | native_base_bridge |
| 重启定位 | odom→camera_init，恒等静态 | native_base_bridge |
| 重启定位 | odom→base_link，动态 | native_base_bridge |
| 两者 | base_link 以下安装/关节 | 原 robot_state_publisher |
| 独立验证 | 无 TF、无反馈输出 | calibration_validation 只读采集/统计 |

不发布 Tracker0→base_link，避免第二父节点。原定位中的 /Odometry 是原生 camera_init→body；NDT 的 /static_localization/pose、/static_localization/odometry 不作为输入。

默认至少 25 对、源时跨度 ≥3 s、配对偏差 ≤25 ms、配对间隔 ≤150 ms、源龄 ≤250 ms；静止半径 ≤2 cm、姿态偏差 ≤2°，原始/归一化证据覆盖 ≥90%，且最终命令新鲜为零。不插值、不填零；失败有原始日志和原因，不静默回退 NDT。初始化成功后动捕丢失不影响该次 LIO 继续；原生时钟回退、竞争 TF 则退出，不重新锚定。

动捕接收时钟并非曝光时钟；相机延迟未知、时差 0 未经独立同步标定。静止采样可以做基准，但不能宣称动态曝光同步认证。初始锚定不会消除后续 LIO 漂移；需要持续高精度跟踪时应使用已有 `tests/controller_tracking` 的持续动捕反馈流程，两种实验结论分开。

定位入口单独覆盖 `pcd_save.pcd_save_en=false` 和 `publish.map_en=false`；建图入口保持原配置。后者关闭 FAST-LIO 每秒累计并序列化整份地图的显示输出，避免长时间定位时阻塞同一执行线程的位姿发布；不修改估计器、降采样、噪声或运动参数。此启动差异写入各次实验记录，生效需要静止重启定位并保存新的初始化，不在运动中切换。
