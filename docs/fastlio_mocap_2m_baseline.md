# FAST-LIO + Tracker0 两米路径基线实验（不运行 NDT）

## 目的和边界

本实验记录**同一次实际运动**中的 FAST-LIO 与 NOKOV `Tracker0` 轨迹，用于离线估计：

- FAST-LIO 在约 2 m 直线路径上的相对位置、航向和回程漂移；
- 动捕相对 FAST-LIO 的固定时间偏移和时间抖动；
- 后续“FAST-LIO + 动捕校正”和“FAST-LIO + NDT”比较所需的纯 FAST-LIO 基线。

它**不启用、不调用、不记录 NDT 的定位结果**，也不向 FAST-LIO、TF、Nav2 或底盘反馈动捕数据。
动捕目前是离线参考，而不是在线校正输入。所有小车运动均由操作者手动完成；本页没有任何
`cmd_vel`、导航目标或自动运动命令。

> 不要运行 `./mapping.sh`、`./localization.sh` 或 `./navigation.sh`。
> `mapping.sh` 可能保存/覆盖 PCD；`localization.sh` 会启动固定地图 NDT；`navigation.sh`
> 会启动导航。三者均不属于本实验。

本页直接启动 FAST-LIO，并显式关闭 PCD 保存；通过已有桥接节点把 FAST-LIO 的原始
`camera_init -> body` 转为车体中心的 `odom -> base_link`。`Tracker0 -> base_link`
离线外参使用当前已确认的 `(0, 0, -0.18 m)`、单位旋转，见
`robot_data/config/mocap_tracker0.yaml`。

## 开始前准备

1. 在动捕有效覆盖区地面贴两条横线，标为 `P1` 和 `P2`；用卷尺测量**车体中心
   `base_link` 的地面投影**从 P1 到 P2 为 `2.000 m`。标记车头朝向和前进行驶方向。
2. 让 Tracker0 保持当前安装状态：刚体原点在 `base_link` 正上方 18 cm，X 前、Y 左、Z 上。
   若刚体被移动、重新定义或换车，停止实验并重新核对外参。
3. 保持 XINGYING 运行、VRPN 已启用，Windows WLAN 地址仍为 `192.168.1.168`。
4. 当前 Windows 与 Ubuntu 尚未得到可靠毫秒级同步结论，因此本轮保持
   `MOCAP_USE_VRPN_TIMESTAMPS=false`。动捕 header 是 Ubuntu 接收端时间；后续离线用独立
   的时间标定包确定固定偏移，不能直接把它当相机曝光时间。
5. 先在所有已启动的 `mapping.sh`、`localization.sh`、NDT、Nav2 终端按 `Ctrl-C` 正常停止。
   不要用 `pkill` 或强制结束整个 ROS 系统。保留你已有的底盘手动控制/安全急停系统即可，
   但不要启动 Nav2。

可在一个检查终端确认没有固定地图定位节点：

```bash
cd /home/zkam/test_nav
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 node list | grep -E 'static_map_localizer|static_map_tf_broadcaster|fastlio_static_odom' || true
ros2 topic list | grep '^/static_localization/' || true
```

两条命令应没有输出；若有输出，先回到对应终端正常 `Ctrl-C` 停止，不能继续录制。

## 固定开启的终端

下面每个终端均保持运行到当天全部实验结束。若你已用同一终端启动过同名组件，不要重复启动。

### 终端 1：Livox 驱动

```bash
cd /home/zkam/test_nav
./livox_driver.sh
```

等待其连接 MID360。此脚本会按现有工程配置设置雷达直连网卡；不要同时手工修改该网卡地址。

### 终端 2：纯 FAST-LIO（无 NDT、禁止保存 PCD）

```bash
cd /home/zkam/test_nav
source /opt/ros/humble/setup.bash
source /home/zkam/fastlio_ws/install/setup.bash
source /home/zkam/test_nav/install/setup.bash

ros2 run fast_lio fastlio_mapping --ros-args \
  --params-file /home/zkam/fastlio_ws/src/FAST_LIO_ROS2/config/mid360.yaml \
  -p pcd_save.pcd_save_en:=false \
  -r /tf:=/tf_lio_raw
```

这里直接运行 `fastlio_mapping`，而非 `mapping.sh`。`-p pcd_save.pcd_save_en:=false` 是本实验
的保护项；不要删除它。`/tf` 被重映射到 `/tf_lio_raw`，避免把传感器体坐标误作为车体中心。

### 终端 3：FAST-LIO 到车体中心的 TF 桥

```bash
cd /home/zkam/test_nav
source /opt/ros/humble/setup.bash
source /home/zkam/test_nav/install/setup.bash

python3 lio_tf_to_base_tf.py --ros-args \
  -p raw_tf_topic:=/tf_lio_raw \
  -p raw_parent_frame:=camera_init \
  -p raw_child_frame:=body \
  -p odom_frame:=odom \
  -p base_frame:=base_link \
  -p sensor_x:=0.37 -p sensor_y:=0.28 -p sensor_z:=0.15 \
  -p sensor_yaw:=0.8229227423153264
```

该节点只发布 `odom -> base_link`；不发布 `map -> odom`。

### 终端 4：可选但推荐的 `/odom` 记录桥

```bash
cd /home/zkam/test_nav
source /opt/ros/humble/setup.bash
source /home/zkam/test_nav/install/setup.bash
python3 tf_to_odom_bridge.py --ros-args \
  -p odom_frame:=odom -p base_frame:=base_link \
  -p odom_topic:=/odom -p publish_rate:=50.0
```

它从已有 `odom -> base_link` TF 导出 `/odom`，不发布任何 TF，也不改变 FAST-LIO。

### 终端 5：Tracker0 被动接收

```bash
cd /home/zkam/test_nav
MOCAP_USE_VRPN_TIMESTAMPS=false ./mocap_tracker0_vrpn.sh
```

它只发布原始刚体 topic、规范化 pose 和诊断，不会发布 `base_link` pose 或 TF。

### 终端 6：启动前健康检查

```bash
cd /home/zkam/test_nav
source /opt/ros/humble/setup.bash
source /home/zkam/fastlio_ws/install/setup.bash
source /home/zkam/test_nav/install/setup.bash

ros2 topic echo --once /mocap/Tracker0/pose
ros2 topic echo --once /mocap/Tracker0/diagnostics
ros2 topic hz /mocap/Tracker0/pose
```

另开一个终端或停止上述 `hz` 后确认 FAST-LIO：

```bash
cd /home/zkam/test_nav
source /opt/ros/humble/setup.bash
source /home/zkam/fastlio_ws/install/setup.bash
source /home/zkam/test_nav/install/setup.bash

ros2 topic echo --once /cloud_registered
ros2 run tf2_ros tf2_echo odom base_link
```

看到连续的 `odom -> base_link` 后才录包。若 `/mocap/Tracker0/diagnostics` 不是 `valid=true`、
出现 `stale`、`duplicate_or_backward_timestamp`，或 TF 不连续，先停止排查，不能采集。

## 统一录包命令

每个试验**单独一个 bag**。以下完整话题集合保留原始传感器数据、FAST-LIO 原始/车体轨迹、
TF、动捕原始与规范化数据；它不包含任何 NDT 话题。

录包前先检查空间：

```bash
df -h /home/zkam
```

建议保留至少数 GB 空间。新开“录包终端”，按试验替换 `RUN_TAG`：

```bash
cd /home/zkam/test_nav
source /opt/ros/humble/setup.bash
source /home/zkam/fastlio_ws/install/setup.bash
source /home/zkam/test_nav/install/setup.bash

RUN_TAG=timing_calibration
RUN_ID="$(date -u +%Y%m%dT%H%M%SZ)_${RUN_TAG}"
RUN_DIR="robot_data/experiments/fastlio_mocap_2m/${RUN_ID}"
mkdir -p "$RUN_DIR"

ros2 bag record -o "$RUN_DIR/bag" \
  /fastlio_mocap_2m/event \
  /livox/lidar /livox/imu \
  /cloud_registered /cloud_registered_body /Odometry /path /odom \
  /tf_lio_raw /tf /tf_static \
  /vrpn_mocap/Tracker0/pose \
  /mocap/Tracker0/pose /mocap/Tracker0/diagnostics
```

等待录包终端显示 `All requested topics are subscribed` 后才开始动作；结束时在该终端按
`Ctrl-C`，等待 `Recording stopped`，再执行：

```bash
ros2 bag info "$RUN_DIR/bag"
```

不要在小车运动中切换 FAST-LIO、动捕、网络或录包进程。

## 可选事件标记终端

为了离线切分静止、加速、停车、旋转和回程，建议开一个额外终端。每次动作前后手动发一次
文本标记；它只记录操作者按键时刻，**不能用于替代精确时间同步**。

```bash
cd /home/zkam/test_nav
source /opt/ros/humble/setup.bash
source /home/zkam/test_nav/install/setup.bash

# 将引号内文字替换为当前动作
ros2 topic pub --once /fastlio_mocap_2m/event std_msgs/msg/String "{data: 'p1_static_start'}"
```

例如可使用 `forward_start`、`forward_stop`、`left90_start`、`left90_stop`、
`return_start`、`return_stop`。

## 阶段 A：独立时间偏移标定包（只做一次，不计入两米精度）

此包的目标是用速度和角速度变化的边沿估计动捕相对 FAST-LIO 的固定延迟；它不是精度结果。

1. 在 P1 静止 5 s，发 `timing_static_start`。
2. 在安全前提下手动直行约 0.5–1 m，随后明显停车；各动作前后发事件标记。
3. 静止约 3 s。
4. 原地左转约 90°，停止约 3 s；再原地右转约 180°，停止约 3 s；最后回正并静止 5 s。
5. 全程保证在动捕覆盖区；不自动发导航目标、不运行 NDT。
6. 停止录包并记录实际动作顺序、估计速度、是否出现遮挡/丢帧。

后续离线分析将只从这包得出固定 `Δt` 与不确定度。若延迟随时间明显变化，则不能把单一
固定 `Δt` 用于毫米级动态误差结论。

## 阶段 B：五次两米往返基线

五次都使用相同地图外观、雷达参数、刚体安装、车速范围和动捕覆盖区。每次独立录包；不可
把五次串到同一个 bag 里。建议命名为 `2m_r01` 至 `2m_r05`。

对每一次重复：

1. 将车体中心地面投影置于 P1，车头对准 P2；静止 5 s，标记 `p1_static_start`。
2. 启动该次录包，将 `RUN_TAG=2m_r01`（下一次改为 `2m_r02` 等）。确认订阅完成。
3. 继续静止 5 s，标记 `forward_start`，手动匀速直行 P1 到 P2。不要依赖导航，不要突然急转。
4. 车体中心到达 P2 后停止，标记 `forward_stop`，静止 5 s。
5. 若做往返：在 P2 原地缓慢转约 180°，标记 `turnaround_start/stop`；静止 2 s；标记
   `return_start`，沿同一线手动回到 P1；停止后标记 `return_stop`，再静止 5 s。
6. 在录包终端按 `Ctrl-C`，等待写盘完成，运行 `ros2 bag info "$RUN_DIR/bag"`。
7. 在实验记录中写下：试次号、P1→P2/是否回程、实际用时、人工速度档、是否偏线、是否遮挡、
   动捕 diagnostics 是否异常、操作者备注。

若只做单向两米测试，也必须每次从 P1 开始，并明确标为 `one_way`；不要将其与往返结果混在
同一个统计量中。

## 停止顺序

每次试验先停止录包；当天全部试验结束后，依次在终端 5、4、3、2、1 按 `Ctrl-C`。不需要也
不应保存 PCD、启动 NDT 或发布手工 `map -> odom` 静态 TF。

## 结果解释边界

- 两条轨迹必须来自**同一个 bag、同一次实际运动**。不能将一次 FAST-LIO 路径和另一次动捕
  路径按墙钟时间拼接比较。
- 离线比较时，先将 Tracker0 显式变换到 `base_link`，再用起始静止窗口估计一次固定
  `mocap_world -> odom` 对齐；整段轨迹不能重新拟合、不能逐段对齐、不能拟合尺度。
- 时间偏移只使用阶段 A 独立标定的 `Δt`；阶段 B 的两米误差不能反过来优化 `Δt`。
- 当前动捕既是参考数据又尚未参与在线融合。因此阶段 B 可报告“起始对齐后的 FAST-LIO 相对
  轨迹误差”，但不能宣称已验证动捕反馈融合后的绝对精度。
- 后续若将动捕反馈到定位链，同一份动捕不能同时当作独立真值。届时需要保留 P1/P2 的独立
  卷尺检查、重复到点误差或未参与融合的检查点。
- 若任一包出现动捕丢帧、无效姿态、TF 缺失或时间偏移不稳定，分析必须标记并排除对应区段，
  不能补零或重复旧位姿。
