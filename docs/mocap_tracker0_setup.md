# NOKOV / XINGYING Tracker0 被动接入

本接入只记录独立动捕参考，绝不发布 `map -> odom`、`odom -> base_link` 或任何
导航控制话题。先完成本页的连接和静止验证，再做 NDT 误差实验。

## 已选用的接收方式

本机为 ROS 2 Humble。工程中已有可执行的项目内置 Humble `vrpn_mocap` 1.1.0
客户端（`.vendor/ros-humble-vrpn/.../client_node`），优先使用它，不下载也不替换
系统组件。

NOKOV 官方 `vrpn_client_ros` 的公开说明使用 `foxy-devel` 分支，虽然其参数能配置
服务器、端口、frame 和 server timestamp，但没有把它当作 Humble 已验证依赖。当前
`vrpn_mocap` 明确支持 Humble 二进制安装以及 `server`、`port`、`frame_id`、
`update_freq`、`refresh_freq`、`use_vrpn_timestamps` 参数，故采用后者。

## 配置文件

检查 [`robot_data/config/mocap_tracker0.yaml`](../robot_data/config/mocap_tracker0.yaml)：

- 已验证的服务端为 Windows WLAN 地址 `192.168.1.168:3883`；刚体固定为
  `Tracker0`。`10.1.1.198` 是同一台 Windows 主机连接动捕相机的专用以太网地址，
  不是小车 Wi-Fi 应连接的地址；
- 原始 topic：`/vrpn_mocap/Tracker0/pose`，绝不覆盖；
- 规范化 topic：`/mocap/Tracker0/pose`，类型为 `geometry_msgs/PoseStamped`，
  `frame_id=mocap_world`；
- 当前 `position_scale_to_m: 0.001` 已由 2026-10-15 的人工约 1 m 位移验证：
  原始 VRPN 仍为毫米，规范化 topic 为米；这不是自动推断；
- 已记录用户于 2026-10-15 确认的 `T_Tracker0_base_link`：刚体原点位于
  ROS `base_link` 正上方 0.18 m，且轴均为 X 前、Y 左、Z 上，因此平移为
  `[0, 0, -0.18] m`、旋转为单位四元数；
- 即使该外参已记录，接收器仍只发布刚体 pose `/mocap/Tracker0/pose`，不会将其
  冒充为 `base_link`、不会发布 `/mocap/base_link/pose`，也不会发布 TF。离线评估
  才会显式应用该固定变换；
- 不会做 `mocap_world -> map` 坐标变换。仅重写 frame_id 不是坐标变换，本脚本只在
  VRPN 接收端原本就被配置为相同 `mocap_world` 时接受数据。

Tracker0 计划的 X 前、Y 左、Z 上与 ROS ENU/车体系约定相容，但仍须通过“向前推车
0.5 m、向左推车 0.5 m、原地逆时针转 90°”实测核验；不要仅凭软件设置截图判定。

## Ubuntu 只读网络预检

在**实际 Ubuntu 机器终端**运行。以下命令不改网卡、不改路由、不关闭防火墙：

```bash
ip -br address
ip route get 192.168.1.168
ping -c 2 -W 1 192.168.1.168
nc -vz -w 3 192.168.1.168 3883
```

`ping` 被 Windows 防火墙屏蔽不必然代表 VRPN 不可用；`nc` 的 TCP 3883 成功才是
当前接收脚本的必要条件。若失败，核对 Windows WLAN 地址仍为 `192.168.1.168`、
XINGYING 的 VRPN 服务是否监听该地址、端口是否为 3883、Windows 防火墙是否只放行
该端口，以及 Ubuntu 是否仍通过 Wi-Fi 到达 `192.168.1.0/24`。不要为了测试关闭整个
防火墙。

当前受自动化环境的网络沙箱限制，`ip route`、ping 和 TCP socket 均无法在这里执行，
所以这一步必须在实机重复；此结论不代表现场网络不通。

## 启动和检查

先确认 Windows XINGYING 正在播放、已定义 `Tracker0`，且其 VRPN 流可由 Windows
WLAN 地址 `192.168.1.168` 访问。然后在 Ubuntu：

```bash
cd /home/zkam/test_nav
./mocap_tracker0_vrpn.sh
```

脚本在连接失败时退出；它只会连接经现场验证的 `192.168.1.168:3883`，且拒绝被环境
变量改成其他刚体名。若 Windows 的 DHCP 地址日后变化，应先重新执行网络预检，再同步
更新本脚本与 YAML 中的服务器地址。成功后新开终端执行：

```bash
cd /home/zkam/test_nav
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic list | grep -E 'vrpn_mocap|mocap/Tracker0'
ros2 topic info -v /vrpn_mocap/Tracker0/pose
ros2 topic echo --once /vrpn_mocap/Tracker0/pose
ros2 topic echo --once /mocap/Tracker0/pose
ros2 topic echo --once /mocap/Tracker0/diagnostics
ros2 topic hz /vrpn_mocap/Tracker0/pose
ros2 topic hz /mocap/Tracker0/pose
```

规范化器只缩放平移并归一化四元数，保留 header stamp；遇到空/错误 frame、零或
重复/倒退时间戳、非有限数据、零四元数时不发布新 pose，而是在
`/mocap/Tracker0/diagnostics` 记录错误。超过 0.5 s 未收到原始帧时同样报 stale，
不会重发旧姿态。

## 时间戳与单位

默认配置是 `use_vrpn_timestamps:=false`，所以 `/vrpn_mocap/Tracker0/pose` 的
header stamp 是 Ubuntu VRPN 客户端接收/发布时钟，而不是已证实的相机曝光测量时刻。
这足以做“静止位置是否稳定”的第一轮检查，但不能凭此声称动态 NDT 误差严格同步。

若设置 `MOCAP_USE_VRPN_TIMESTAMPS=true`，必须先录包确认实际消息时间戳确由星影
服务器填写，再用 NTP/PTP 等方式验证 Windows 和 Ubuntu 时钟偏差；两个主机未同步时，
也不能将该时间直接用于动态误差配对。不要通过最小化轨迹误差反向估计时间偏移。

用一段已知长度的平移验证单位：若软件显示/实测移动 1.000 m，而
`/mocap/Tracker0/pose` 同样变化约 1.000，则保留 `0.001`；若原始 ROS topic 已是米，
将配置改为 `position_scale_to_m: 1.0` 后重启，并把配置与 bag 一起保存。当前 Tracker0
已完成前一种人工验证；原始数据与规范化数据都保留在验证 bag 中。

## 第一轮：静止验证

在动捕覆盖区选 3–5 个位置，每处车体完全静止约 30 s；先在第一个位置静止 5 s 观察
频率、diagnostics、位置/航向波动。这个阶段**不需要也不应启动定位或 Nav2**。每个
位置单独录包，保留 raw 和 normalized 数据：

```bash
cd /home/zkam/test_nav
source /opt/ros/humble/setup.bash
source install/setup.bash
RUN_ID="$(date -u +%Y%m%dT%H%M%SZ)_tracker0_static_p1"
mkdir -p "robot_data/experiments/mocap_tracker0/${RUN_ID}"
ros2 bag record -o "robot_data/experiments/mocap_tracker0/${RUN_ID}/bag" \
  /vrpn_mocap/Tracker0/pose \
  /mocap/Tracker0/pose \
  /mocap/Tracker0/diagnostics
```

每处按 `Ctrl-C` 停止，检查 `ros2 bag info <bag路径>`。这些是动捕链路质量记录，
用于后续起始窗口对齐后的相对 NDT 误差分析。

## 后续接入 NDT 评估的前置条件

1. 已记录 `Tracker0 -> base_link=(0,0,-0.18 m)`、单位旋转的用户确认安装外参；
   若刚体原点、轴向或安装高度改变，必须重新测量并更新 YAML。
2. 动态实验前确认两台电脑时间戳来源和同步精度。
3. 满足以上条件后，再将 raw、规范化 pose、diagnostics 与
   `/static_localization/ndt_match_diagnostic`、`/tf`、`/tf_lio_raw` 等一起录包，按
   [`ndt_match_evaluation.md`](ndt_match_evaluation.md) 运行离线评估。
