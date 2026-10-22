# 统一 2 m 定位实验

三组使用完全相同的 `MotionGate` 和独立速度看门狗：静止记录 5 s，
固定发布 `linear.x=0.15 m/s`、`linear.y=0`、全部角速度为零；首次检测到
动捕 **base_link 相对起点的平面位移 >=2.000 m** 时立即发零，继续录 5 s。
2 m 是停止指令触发值，实际停车有制动滑行；报告另列触发距离、停车距离和过冲。

| 组 | 输出 `/localization_ab/estimate/<组>` | 动捕用途 |
|---|---|---|
| A | FAST-LIO `odom -> base_link`，frame=odom | 仅统一距离触发和离线参考 |
| B | NDT 的实际 map->odom 修正 × 同时刻 FAST-LIO base，frame=map | 仅统一距离触发和离线参考 |
| C | 动捕对 FAST-LIO 的全局刚体修正，frame=mocap_world | 距离触发、全局修正和离线一致性比较 |

C 按同步历史局部位姿计算 `T_mocap_odom = T_mocap_base * inverse(T_odom_base)`，
并将修正用于后续局部里程计输出。C 是独立定位输出分支，不重写 FAST-LIO 内部状态，
不发布会覆盖既有 TF 的变换。C 与同一动捕的差值是**一致性误差**，不可宣称为独立绝对精度。

安装：

```bash
cd /home/zkam/test_nav
source /opt/ros/humble/setup.bash
source /home/zkam/fastlio_ws/install/setup.bash
colcon build --base-paths src/localization_ab_test --packages-select localization_ab_test
source install/setup.bash
```

先运行底盘、Livox、`fastlio_mocap_baseline.sh` 和 `mocap_tracker0_vrpn.sh`。
本包不启动 Nav2、不依赖手柄，也不请求电机回零。

默认命令只读检查，不发运动命令：

```bash
ros2 run localization_ab_test run_trial --group A
```

发车前需要独占 `/nav/cmd_vel` 和 `/teleop/mode`。现有 `teleop_node` 会持续发布模式，
必须先停止它；启动 `robot_control.launch.py` 时也要保持 UDP 服务关闭。底盘
`external_watchdog_timeout` 必须在 controller configure 时设置为 <=0.5 s（建议 0.25 s）。
原控制器只在 on_configure 缓存此参数，因此仅在线 `ros2 param set` 改显示值不保证生效。
保留实体急停，确认至少 2.5 m 净空和全程动捕覆盖后，才使用 `--route-clear`。

为本实验准备底盘的专用命令（会停止已知的 OmniBot teleop 进程，保持 STOP，
停用控制器、设置运行时 0.25 s 超时，再重新 configure 和激活；不改原配置文件）：

```bash
ros2 run localization_ab_test prepare_chassis --apply \
  --output robot_data/experiments/localization_ab/chassis_preparation.json
```

该命令不请求回零，也不发非零速度。需要回零时显式加 `--home`，该选项在 STOP 模式调用
底盘既有服务，并等待驱动的回零和速度恢复完成证据。驱动在恢复速度模式后会清零
`homing_done`，所以零值不能单独当作“尚未回零”；检查同时要求新鲜硬件状态及当前
驱动进程的 `Velocity command gate: OPEN` / Operation Group 3 完成日志，且不能有后续复位或错误。
恢复手柄控制需在测试结束后重新启动原
teleop 节点；运行时 0.25 s 超时会持续到控制器下一次重新配置。

```bash
ros2 run localization_ab_test run_trial --group A --execute --route-clear
# B：保持相同 FAST-LIO 和底盘链，单独启动 NDT 后独立初始化并等待稳定。
# 不要再运行 localization.sh 导致两套 FAST-LIO。
ros2 run static_map_localization static_map_localizer_node --ros-args \
  -p map_pcd_path:=/home/zkam/test_nav/robot_data/maps/data/scans.pcd
# 用现有 /initialpose 粗初始化，或预先测量的地图初值；不能用动捕给 A/B 定位反馈。
ros2 run localization_ab_test run_trial --group B --execute --route-clear
# C：先停止 B 的 NDT；动捕校正节点由 run_trial 启动。
ros2 run localization_ab_test run_trial --group C --execute --route-clear
```

每条发车命令仅执行一次向前 2 m，停车后保持零速度并退出，不自动倒车/重定位。
默认方向为 `forward`；只有现场明确确认倒车通道后，才可指定 `--direction reverse`，
指令变为 `linear.x=-0.15 m/s`。保护进程只接受本次指定的方向，运动方向相反也会停车；
位移 2 m、动捕停更等原保护门限不变。

用户确认整段往返通道后，可执行三次“前进 / 倒车 / 前进”，不掉头：

```bash
ros2 run localization_ab_test run_three_trials --group B --output robot_data/experiments/localization_ab/<新B批次>
# 上面仅显示计划。NDT 人工初始化并接受匹配，且现场净空、动捕覆盖和急停确认后：
ros2 run localization_ab_test run_three_trials --group B --output robot_data/experiments/localization_ab/<新B批次> --execute --route-clear
```

若 C 组仍位于上一组终点，现场确认后可改为“倒车 / 前进 / 倒车”：

```bash
# 默认仅显示计划；实际发车还需 --execute --route-clear。
ros2 run localization_ab_test run_three_trials --group C --first-direction reverse --output robot_data/experiments/localization_ab/<新C批次>
```

C 批次要求 NDT 已停止，校正只作用于 C 的输出分支，不覆盖 TF。
倒/前/倒与 A/B 的前/倒/前方向构成不同，须按方向分别展示，不当作三组配对重复。
同源动捕同时校正和评估，C 指标只能说明同源一致性，不能证明独立定位精度；
`timing_verified=false` 时也不能宣称已消除动态时间误差。

B 的启动检查要求 `/static_localization/ready=true` 和新鲜 NDT 诊断；
仅存在 NDT 进程或初始化前的 identity map→odom 不算定位就绪。
专用地图初始化界面为本包 `config/ndt_initialization.rviz`，使用固定点云地图设置
`/initialpose_user`，不从动捕生成 A/B 的定位初值。另检查本机原生 FAST-LIO
进程数，防止 DDS 发现遗漏而误认为没有重复进程。
每次独立保存原始包和逐帧评估，任一次失败立即停止后续试次，不自动重试。
三次包含正反向，报告按方向标注，不称为三次相同正向重复；FAST-LIO 和 NDT
在批内连续运行，不逐次重启。`summary.csv`、`summary.json`、`report.md` 在批次目录。
发车前等待 TF 发现最多约 10 秒，尚未发送运动指令；这与运行中的 0.20/0.30/0.50 秒
新鲜度保护不是一回事，不会把旧消息作为新消息通过检查。若第二次仅在 preflight
被阻止、未创建录包/运动节点，现场重新确认后可用 `--continue-from <原目录>` 和
新的 `--output` 接续未启动试次；原记录不改写。正在录制时不要重命名实验目录。

离线六条报告对比可使用 `compare_trials --a <A1> <A2> <A3> --b <B1> <B2> <B3>
--output <新比较目录>`，只读取既有报告，生成逐试次表与图，不发运动指令。
统计先计算每次 RMSE，再报告三次的均值与样本标准差，不把所有帧当独立试次。
比较命令可增加 `--c <C1> <C2> <C3>`，离线生成 A/B/C 九趟对比：
`abc_metrics.png/svg/pdf` 为六项逐试次统计，`abc_comparison.png/svg/pdf`
为误差随动捕位移曲线和 RMSE/终点差。图中采用简洁的方案名称及 A1–C3 标签，
实验方向、时间未核实及 C 同源参考的解释保留在配套 `report.md`、CSV 和 JSON，
不修改原始包或已生成的单次报告。
每次重新摆放起点后重新检查净空。前进速度和停止策略在三组完全相同，不按照估计误差调速。

运行目录默认 `robot_data/experiments/localization_ab/<UTC时间>_<组>/`，包含
`manifest.json`、完整 `bag/` 和各进程日志。录包所有必需话题订阅完成后，才允许开始静止段。
动捕安装外参复用 `robot_data/config/mocap_tracker0.yaml` 的完整三维固定变换。
雷达安装外参（用户于 2026-10-16 重新确认）为 `base_link -> sensor`：
平移 `[+0.37, +0.28, +0.15] m`，yaw `+47.15° = +0.8229227423153264 rad`；
即前方 37 cm、左侧 28 cm，Z 保留原 15 cm。此项与 Tracker0 安装外参不同，
也不修改 FAST-LIO 内部雷达到 IMU 的外参。旧包与旧报告保留，不自动按新值覆盖。
先前的 37.2° 来自安装位置的 `atan2(Y,X)`，不能作为安装航向；+45° 是旧估计值。
当前 +47.15° 由用户于 2026-10-16 报告实测确认；新试次固定此角度，不在测试轨迹上重拟合。
同值曾由 run09 轨迹反推为候选值，因此 run09 不能充当其独立验证数据；应另录新包验证。
当前运行中的定位桥接和 robot_state_publisher 不会自动读取修改，必须停车后重启相关节点。
动捕停更/重复旧帧、错误诊断、定位停更、录包进程失效、速度指令超时、超时不动、
偏航/横向越界或其他速度/模式发布者出现，都使独立速度保护进程锁定为 STOP。
整体计算机或 DDS 失效依靠底盘自身的短超时，不能仅靠同一台机器上的 Python 进程。

内部 `/localization_ab/request`、`recorder_ready`、`state` 和 `guard_status` 使用
BEST_EFFORT / KEEP_LAST 的新鲜状态传输，避免为旧心跳和旧指令等待可靠重传；
发布和订阅端策略保持一致。丢失仍按原 0.20/0.30 s 截止时间触发停车，未放宽保护。
实际底盘 `/nav/cmd_vel`、`/teleop/mode` 与实验事件仍保留原可靠传输。
守护进程保留第一个失败原因，退出清理阶段的 `recorder_not_ready` 不再覆盖它。

离线分析：

```bash
ros2 run localization_ab_test evaluate --trial robot_data/experiments/localization_ab/<试次目录>
```

统一生成逐帧 CSV、JSON/Markdown 统计、XY 轨迹/时序图、误差随动捕真实位移变化的 SVG/PNG。
指标包含运动到停车后 5 s 的 XY RMSE/P95/Max、yaw RMSE、停车后 2–5 s 平均终点误差、
最大相邻位姿变化与扣除真实运动后的误差跳变、参考覆盖率和停车过冲。
失败试次明确标为 aborted，并排除于“完成的 2 m 试次”比较；仍保留所有原始包及可评估片段。
最大相邻位姿变化即使动捕丢失也计算；真实误差未知时 CSV 留空，不补零。
只用开始运动前的静止窗口做一次固定 SE(2) 对齐，不拟合尺度、不用整条轨迹重新对齐。
缺失参考留空并统计；不跨 0.1 s 动捕间隔插值，不把低频 NDT 原始匹配重复计为多次匹配。

目前独立时间标定尚未得到可信单一偏移，默认保存 `mocap_offset_s=0`、
`timing_verified=false`。依然输出整段误差，但醒目标为时间未核实的初步结果；
不能把软件默认零偏移视为已同步。只有经独立标定后才能显式提供
`--mocap-offset <秒> --timing-verified`，C 在线历史校正和离线匹配共用该数值。
在 C 已使用的同一流上最小化误差寻找偏移没有独立验证意义。

测试均整理在本包：

```bash
PYTHONPATH=src/localization_ab_test:$PYTHONPATH python3 -m pytest -q src/localization_ab_test/test
# ROS 集成测试仅允许隔离 ROS_DOMAIN_ID，模拟底盘，不连接真实小车。
ROS_DOMAIN_ID=174 python3 src/localization_ab_test/test/integration_sim.py
ROS_DOMAIN_ID=174 python3 src/localization_ab_test/test/integration_sim.py --fault request
ROS_DOMAIN_ID=174 python3 src/localization_ab_test/test/integration_sim.py --fault mocap
ROS_DOMAIN_ID=174 python3 src/localization_ab_test/test/integration_sim.py --fault permit
```
