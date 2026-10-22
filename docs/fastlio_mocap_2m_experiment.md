# FAST-LIO 与 Tracker0：两米轨迹基线实验

本流程用于记录同一次运动中的 FAST-LIO `odom -> base_link` 轨迹和 Tracker0
动捕轨迹；NDT、Nav2、`map -> odom` 以及动捕反馈均不启动。它测的是 FAST-LIO
局部里程计相对动捕的两米轨迹误差，不是动捕辅助定位的精度。

## 先决条件

- Windows 星影已运行，`Tracker0` 有效，VRPN 服务可访问；
- 小车和两米直线路线全程都在动捕覆盖区；
- 使用已记录的 `Tracker0 -> base_link=(0,0,-0.18 m)` 安装变换；
- 停止已有的 `./localization.sh`、`./mapping.sh`、`./navigation.sh`。它们若仍运行，
  会启动另一条 FAST-LIO/NDT/TF 链；
- 不要使用 `mapping.sh` 作为本实验的 FAST-LIO 启动方式：该脚本退出时会请求保存地图。

本实验暂时保持 `MOCAP_USE_VRPN_TIMESTAMPS=false`。因此动捕 header 是 Ubuntu
接收时间；后续必须由专门的时间标定段给出固定延迟，不能假定 Windows 和 Ubuntu
已严格同步。

## 终端 1：Livox 驱动

```bash
cd /home/zkam/test_nav
./livox_driver.sh
```

它可能要求 `sudo` 配置 MID360 的专用有线接口。这是已有驱动流程；不要改动 Windows
动捕相机或 VRPN 网络设置。

## 终端 2：FAST-LIO-only 基线

```bash
cd /home/zkam/test_nav
bash ./fastlio_mocap_baseline.sh
```

该脚本只启动 FAST-LIO、`/tf_lio_raw` 到 `odom -> base_link` 的车体中心桥接，及
`/odom` 桥接。它显式关闭 PCD 保存；不会启动 NDT、Nav2 或发布 `map -> odom`。

## 终端 3：Tracker0 接收

```bash
cd /home/zkam/test_nav
source /opt/ros/humble/setup.bash
source install/setup.bash
MOCAP_USE_VRPN_TIMESTAMPS=false ./mocap_tracker0_vrpn.sh
```

## 终端 4：上线前检查

```bash
cd /home/zkam/test_nav
source /opt/ros/humble/setup.bash
source /home/zkam/fastlio_ws/install/setup.bash
source install/setup.bash

# 以下三个命令分别在独立监视终端运行；或逐条运行并以 Ctrl-C 结束后再运行下一条。
ros2 topic hz /mocap/Tracker0/pose
ros2 topic hz /cloud_registered
ros2 run tf2_ros tf2_echo odom base_link
```

应看到动捕约 90 Hz、`/cloud_registered` 持续更新以及 `odom -> base_link` TF。若发现
`/static_localization/*` 仍在持续发布，不要开始录包；先停止旧定位进程。

## 先录一包时间标定动作

这包**不用于报告两米精度**。它只用于估计 FAST-LIO 与动捕的固定相对时间偏移及残余
抖动。

### 终端 5：录包

```bash
cd /home/zkam/test_nav
source /opt/ros/humble/setup.bash
source /home/zkam/fastlio_ws/install/setup.bash
source install/setup.bash

RUN_DIR="robot_data/experiments/fastlio_mocap/$(date -u +%Y%m%dT%H%M%SZ)_time_calibration"
mkdir -p "$RUN_DIR"
ros2 bag record -o "$RUN_DIR/bag" \
  /mocap/Tracker0/pose \
  /vrpn_mocap/Tracker0/pose \
  /mocap/Tracker0/diagnostics \
  /tf_lio_raw /tf /tf_static \
  /cloud_registered /cloud_registered_body /odom
```

### 人工动作

1. 车静止 5 秒；
2. 人工低速前进约 0.4 m，快速停车，静止 3 秒；
3. 后退约 0.4 m，快速停车，静止 3 秒；
4. 原地左转约 90°，静止 3 秒；
5. 原地右转约 180°，静止 3 秒；
6. 左转约 90° 回正，静止 5 秒；
7. 在录包终端按 `Ctrl-C`。

不要自动驱动车。速度、加速度不必很大，但起步、停止与转向边沿应清楚；全程保持在
动捕覆盖区。把该 `RUN_DIR` 路径提供给分析工具/研究记录。

### 离线估计固定时间偏移

录包停止后，只能用这一个独立标定包运行下列工具；不要用后续两米精度包反向调这个数值：

```bash
cd /home/zkam/test_nav
source /opt/ros/humble/setup.bash
source /home/zkam/fastlio_ws/install/setup.bash
source install/setup.bash

python3 fastlio_mocap_time_calibration.py \
  --bag "$RUN_DIR/bag" \
  --output "$RUN_DIR/time_offset_report" \
  --tracker-to-base-config robot_data/config/mocap_tracker0.yaml
```

查看 `time_offset_report.md`、`time_offset_correlation.svg` 和
`kinematic_signals_at_best_offset.svg`。该工具优先读取 `/tf` 中精确的
`odom -> base_link` transform 时间戳，而不是 `/odom` 桥接节点的发布时间；它仅比较
平面速度和有符号航向角速度，**不进行空间轨迹拟合**。其正偏移 `d` 的定义是：FAST-LIO
在 `t` 的样本匹配动捕 header 为 `t + d` 的样本。

如果动捕在动作结束前停更、近最优偏移范围很宽，或平移/转向两条曲线明显不一致，必须先修复
动捕有效性并重录阶段 A；不要把这个不稳定偏移用于毫米级动态精度结论。

## 正式两米重复实验

1. 用卷尺在地面标出 P1、P2；两处车体 `base_link` 投影的目标停靠点相距 `2.000 m`；
2. 标出行驶方向，确保两点都在动捕有效区域；
3. 每次录制一个独立包；先在 P1 静止 5 秒，这是唯一用于估计固定
   `odom <- mocap_world` 对齐的窗口；
4. 手动以尽可能相同的低速从 P1 匀速驶向 P2；
5. 在 P2 静止 5 秒；
6. 返回 P1，在 P1 静止 5 秒；
7. 停止录包；共做至少 5 次独立往返。

每次正式录包使用下面命令，只修改标签，例如 `run01`、`run02`：

```bash
cd /home/zkam/test_nav
source /opt/ros/humble/setup.bash
source /home/zkam/fastlio_ws/install/setup.bash
source install/setup.bash

LABEL="p1_p2_return_run01"
RUN_DIR="robot_data/experiments/fastlio_mocap/$(date -u +%Y%m%dT%H%M%SZ)_${LABEL}"
mkdir -p "$RUN_DIR"
ros2 bag record -o "$RUN_DIR/bag" \
  /mocap/Tracker0/pose \
  /vrpn_mocap/Tracker0/pose \
  /mocap/Tracker0/diagnostics \
  /tf_lio_raw /tf /tf_static \
  /cloud_registered /cloud_registered_body /odom
```

## 数据解释边界

- 两条轨迹必须来自同一 rosbag、同一次小车运动；不能拿不同日期/不同运行的轨迹逐帧对齐；
- 动捕先由 Tracker0 变换为 `base_link`，然后只用起始静止窗口估计一次空间对齐；
- 时间上使用独立时间标定得到的固定偏移，并对动捕作短间隔插值；动捕无效、丢帧、超出
  允许插值间隔的样本要标记为缺失，不能补零；
- 不把动捕实时反馈给 FAST-LIO 的这组实验中，动捕是独立参考；若以后将动捕反馈到定位，
  同一动捕流不能再作为该融合结果的独立绝对真值。
