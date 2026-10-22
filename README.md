

# 项目目录说明

本文档用于说明 **robot_xj_project** 项目的完整启动顺序与各模块功能说明，建议严格按照顺序执行，项目位于**/home/zkam/test_nav**，以下是项目目录说明

## 统一 RViz（导航、定位与巡检）

统一使用以下一个 RViz 配置，不需要再打开 `inspection_geometry.rviz`：

```bash
./rviz.sh
```

该配置已包含对齐巡检地图、10 cm 网格、人工 Polygon、1 m 外扩路径与方向箭头、START/MIDDLE/END 标记，以及原有的定位和导航显示。巡检节点自身默认会打开一个 RViz；已使用统一 RViz 时，必须传入 `use_rviz:=false`，避免出现第二个 RViz：

巡检默认只把“特征对齐”用于 10 cm 网格显示；Polygon、紫色外扩路径、定位和 Nav2 均保持在原始 `/map` 坐标中，因此可直接执行 `FollowPath`。不要在实车 Nav2 模式启用 `rotate_occupancy_grid:=true`。

```bash
ros2 launch inspection_geometry_tools inspection_geometry.launch.py use_rviz:=false
```

## 正常巡检完整启动流程（OmniBot 实车）

以下各项分别在**独立终端**执行，已启动的终端不要关闭。此流程适用于当前
OmniBot 底盘、Livox MID-360、静态点云定位、Nav2 与巡检路线；使用前确认
`robot_data/maps/data/scans.pcd`、`robot_data/maps/map.yaml` 均存在。

> **不要**在此流程中启动 `ranger_can_setup.sh`、`ranger_driver.sh` 或
> `ranger_bringup`。它们是旧 Ranger 底盘流程，会与当前 OmniBot 的控制器和
> CAN 连接冲突。也不要额外启动第二个 `robot_state_publisher`。

### 终端 1：启动底盘与机器人 state

```bash
source /opt/ros/humble/setup.bash
source /home/zkam/OmniBot_ws/install/setup.bash
ros2 launch omnibot_control robot_control.launch.py
```

等待自动回零完成，并确认 `arbitrator_controller`、`omni_velocity_controller` 与
`joint_state_broadcaster` 已启动。`start_udp_server:=false` 可以避免额外 UDP
节点与 Nav2 同时发布导航速度。该终端也会发布机器人本体的 state/TF。

### 终端 2：启动雷达

```bash
cd /home/zkam/test_nav
./livox_driver.sh
```

脚本会配置 MID-360 的网卡地址并启动 Livox 驱动；若提示输入密码，按提示输入。

### 终端 3：启动静态地图定位

```bash
cd /home/zkam/test_nav
./localization.sh
```

此终端会启动 FAST-LIO 地图定位、`odom` 桥接和静态 NDT 匹配，必须持续运行。

### 终端 4：启动 `/scan` 桥接（原 `robot_state.sh`）

```bash
cd /home/zkam/test_nav
./robot_state.sh
```

该脚本只把 Livox 点云转换为 `/scan`，供二维地图、RViz 和 Nav2 使用；真正的
机器人 state 已由终端 1 的 `robot_control.launch.py` 发布。

### 终端 5：启动 RViz 并给定位初值

```bash
cd /home/zkam/test_nav
./rviz.sh
```

在 RViz 工具栏选择 **2D Pose Estimate**，在机器人实际位置附近点击并拖出与真实
车头大致一致的方向。等待定位状态显示 `NDT accepted`，或在另一终端确认：

```bash
ros2 topic echo /static_localization/ready --once
```

输出 `data: true` 后再启动导航。RViz 必须在导航前打开，是为了完成这一步初始定位。

### 终端 6：启动导航

```bash
cd /home/zkam/test_nav
./navigation.sh
```

若脚本提示尚未获得 `odom -> base_link`，保持前面终端运行，待定位初始化成功后再试。

### 终端 7：启动巡检路线

统一 RViz 已在终端 5 打开，因此此处显式关闭巡检节点自带的第二个 RViz：

```bash
source /opt/ros/humble/setup.bash
source /home/zkam/test_nav/install/setup.bash
ros2 launch inspection_geometry_tools inspection_geometry.launch.py use_rviz:=false
```

在统一 RViz 的巡检面板中加载或绘制 Polygon，生成外扩路径后执行路线。不要同时再开
一份 `inspection_geometry.launch.py`；路径执行完成或需要停止时，在终端 7 按 `Ctrl+C`。

底盘控制启动后会写 CAN 日志；长时间运行时请留意 `/tmp/can_logs` 的磁盘占用。

```

├── install 						功能包执行文件夹
├── log 							ros日志文件夹
├── robot_data  					配置相关文件夹
    ├── config						
    │   ├── navigation.yaml			导航相关配置
    │   └── robot_rviz_plugin.yaml	rviz导航插件运行记录
    ├── maps						
    │   ├── data					
    │   │   ├── Scans				闭环建图关键帧点云存放文件夹
    │   │   └── scans.pcd			合成后的完整点云地图文件
    │   ├── map.pgm					二维栅格地图文件
    │   └── map.yaml				二维栅格地图文件
    └── urdf
        └── robot.urdf				机器人简单模型文件
├── mapping.sh						建图启动脚本
├── localization.sh					定位启动脚本
├── navigation.sh					导航启动脚本
├── livox_driver.sh					雷达启动脚本
├── README.md						项目说明文档
├── makeMergedMap_open3d.py 用于合并闭环建图的pcd文件
├── robot_xj_project.rviz   默认rviz配置文件
```

# 依赖安装

```
sudo apt install libasio-dev
```

```
sudo apt install ros-humble-nav2*
```

```
sudo apt install ros-humble-gtsam
```



# 建图启动流程

对于首次使用且未进行过建图的场景，应先启动建图功能。
建图过程中基于三维激光进行闭环建图，并在完成后生成对应的二维地图。
其中，三维地图 *scans.pcd* 作为全局定位的匹配地图，二维栅格地图 *map.pgm* 用于后续的全局路径规划

## 1. 启动激光雷达

启动激光雷达驱动节点，发布点云数据

```bash
./livox_driver.sh
```

![image-20260126111555759](README.assets/image-20260126111555759.png)

**功能说明：**

* 发布 `/livox/lidar` `/livox/imu`话题（建议先录制成bag数据包，便于后续使用）
* 为建图、定位和避障提供原始激光数据

## 2. 构建点云地图

运行建图启动脚本后，程序会自动启动 RViz。
随后通过播放 bag 文件或启动3d雷达驱动，RViz 界面将正常显示点云和路径信息

```
./mapping.sh
```

数据播放完成或建图结束后，如下图所示
随后直接关闭终端，程序将自动在**/home/zkam/test_nav/robot_data/maps/data** 目录中保存关键帧点云和位姿数据。

![image-20260122160859821](README.assets/image-20260122160859821.png)

运行makeMergedMap_open3d.py文件拼接点云生成完整点云地图。

```
cd /home/zkam/test_nav
python3 makeMergedMap_open3d.py
```

## 3. 构建栅格地图

完成构建点云地图后，运行本地转换脚本，将 *robot_data/maps/data/scans.pcd* 转成 Nav2 使用的二维栅格地图 *map.pgm* 和 *map.yaml*：

```
cd /home/zkam/test_nav
./pcd_to_2d_map.sh
```

每次转换都读取 `mapping.sh` 最近保存的原始 `robot_data/maps/data/scans.pcd`，
启动时会显示其绝对路径、修改时间和大小。不会选用 `.bak_*` 备份或
`pcd_to_2d_debug` 中的过滤点云；覆盖旧二维图前会自动备份。
若点云保存到了其他路径，可显式指定：`./pcd_to_2d_map.sh /绝对路径/新地图.pcd`。

默认投影墙体常见高度层的点云，过滤范围为：

```
z_min = 0.05
z_max = 0.35
```

该范围用于过滤地面、桌面/柜子上半部分和天花板等干扰点。投影前还会执行
3 cm voxel 统计、10 cm 半径邻域去噪、垂直结构筛选。随后使用三个高度层
（`-0.15~0.05`、`0.05~0.35`、`0.35~0.60 m`）作为墙体证据：同一栅格至少
获得两个高度层支持才参与直线检测。Hough 只补长度不少于 1.5 m、原始墙点支持率
不少于 65%、间隙不超过 40 cm 的共线段；最后以 5 cm 形态学闭运算连接小缝并
进行连通域去噪。长距离完全未被雷达扫到的墙段不会被自动虚构，仍需要补扫。

![image-20260126140706013](README.assets/image-20260126140706013.png)

该脚本默认读取：

```
robot_data/maps/data/scans.pcd
```

并输出：

```
robot_data/maps/map.pgm
robot_data/maps/map.yaml
```

如需手动查看二维地图：

```
eog /home/zkam/test_nav/robot_data/maps/map.pgm
```

![image-20260126141132369](README.assets/image-20260126141132369.png)



**注意事项：**
 当二维栅格地图出现漏边、非封闭等情况时（通常由建图不完整引起），需要借助 **pgm 地图处理工具** 对地图进行修补和封闭处理，
 防止全局规划产生不合理或不可执行的路径。



## 巡检路径跟踪实验导出

巡检执行器发送完整 `FollowPath` 后，会自动记录参考路径和 Nav2 实际使用的
`map → base_link` TF 车体中心**估计**轨迹。`/Odometry` 只用作采样触发，避免把
MID360 的原始传感器位姿误当作车体中心；速度由相邻车体位姿差分计算。收到 Nav2
成功、取消或失败结果时，自动写入：

```text
ChatGPT said:
明白，你要的是“正常巡检”的完整多终端启动顺序，不是单独路线节点。我会核对各脚本的依赖与实际命令，然后把“底盘 → 雷达 → 定位 → state → 导航 → 路线 → RViz”的流程补回 README。

￼
Ran for f in ranger_can_setup.sh ranger_driver.sh livox_driver.sh localization.sh robot_state.sh navigation.sh rviz.sh; do if [ -f "$f" ]; then printf '\n===== %s =====\n' "$f" sed -n '1,260p' "$f" fi done printf '\n===== 巡检 launch 默认参数 =====\n' sed -n '1,280p' src/inspection_geometry_tools/launch/inspection_geometry.launch.py printf '\n===== README 底盘/导航现有段落 =====\n' sed -n '260,350p' README.md
ChatGPT said:
脚本检查发现当前 README 的“robot_state”说明已经过时：robot_state.sh 只发布 /scan 回退，真正的机器人状态与底盘仲裁由 OmniBot_ws 的 robot_control.launch.py 提供。为避免把旧 Ranger 驱动和新的 OmniBot 控制器同时启动造成冲突，我会先核对 OmniBot 启动文件和导航命令链，再写入准确流程。

￼
Ran printf '%s\n' '--- OmniBot 控制启动文件 ---' find /home/zkam/OmniBot_ws -path '*robot_control.launch.py' -type f -print -exec sed -n '1,340p' {} \; 2>/dev/null printf '%s\n' '--- OmniBot 控制包与 CAN/话题引用 ---' rg -n -i -C 2 'cmd_vel|/nav/cmd_vel|odom|base_link|can[0-9]|robot_state|arbitrator|controller' /home/zkam/OmniBot_ws/src 2>/dev/null | sed -n '1,800p' printf '%s\n' '--- 静态 Nav2 命令重映射 ---' rg -n -C 3 'cmd_vel|arbitrator|controller_server|velocity_smoother' src/static_map_localization robot_data/config/navigation.yaml 2>/dev/null | sed -n '1,600p' printf '%s\n' '--- 当前巡检启动锁与命令依赖 ---' sed -n '1,240p' src/inspection_geometry_tools/src/inspection_path_executor_node.cpp | rg -n -C 3 'follow_path|FollowPath|Navigate|action|server' | sed -n '1,300p'
Context automatically compacted
ChatGPT said:
我已确认当前实车应使用 OmniBot 的控制启动，而不是旧的 Ranger CAN 流程。我会把 README 补成按终端顺序可直接执行的完整巡检流程，并标明 RViz 必须在导航前用于初始化定位。

￼
Edited a file, read files, ran a command
ChatGPT said:
完整顺序已写入 README。复
robot_data/experiments/<时间>_<实验标签>/
├── trajectory.svg          # 紫色参考路径 + 红色实际估计轨迹；图头含 RMSE/MAE/最大误差
├── cross_track_error.svg   # 横向跟踪误差随时间变化
└── speed_profile.svg       # 估计速度随时间变化
```

运行 MPPI 实验时，巡检工具以标签启动：

```bash
ros2 launch inspection_geometry_tools inspection_geometry.launch.py \
  use_rviz:=false experiment_label:=mppi_01
```

将来切换到 MPC 控制器后，仅改为 `experiment_label:=mpc_01`，使用相同起点、
路径、速度和定位条件重复实验。所有结果均为可直接打开或插入论文的 SVG 矢量曲线；
失败任务也会生成图并标出失败状态。当前轨迹来自定位估计，因此它评估的是控制器的**估计跟踪误差**。

# 定位启动流程

启动全局定位功能前，需要先完成建图流程。
确认已生成三维点云地图 *scans.pcd*，以及二维栅格地图 *map.pgm* 和 *map.yaml*，并将其存放于指定的 *robot_data* 目录中

---

## 1. 启动定位脚本

定位脚本只启动定位算法，默认不再单独打开 RViz，避免和导航 RViz 重复：

```
cd /home/zkam/test_nav
./localization.sh
```

该终端必须保持运行。它启动的是源码包
`src/static_map_localization`：FAST-LIO 只提供连续的局部里程计，
`static_map_localizer_node` 只读加载
`robot_data/maps/data/scans.pcd`，在收到初始位姿后以 NDT 对固定
PCD 做地图匹配并发布 `map -> odom`。此模式不会覆盖或追加 PCD 地图。
按 `Ctrl-C` 会停止 FAST-LIO、固定地图匹配和 `/odom` 发布，Nav2 会失去
`map -> odom -> base_link` 坐标链。

推荐统一只打开这一个 RViz，用它同时完成定位校准和导航目标发送：

```
./rviz.sh
```

![image-20260126144331836](README.assets/image-20260126144331836.png)

**注意事项：**
 RViz2 对计算资源消耗较大，在模组端运行可能对算法实时性产生影响
 建议通过 ROS 2 的分布式通信方式，在外部电脑或算力更充足的平台上启动 RViz

## 2. 启动激光雷达

```bash
./livox_driver.sh
```

此时可通过 **RViz2** 显示话题 **/overall_map** 的全局点云地图以及话题 **/cloud_registered** 的实时点云数据
用户可在同一个 RViz2 中手动点击 **“2D Pose Estimate”** 按钮，设置一个大致的初始位姿，定位模块将基于该初始位姿进行匹配，并自动完成定位。
定位贴合后，继续在同一个 RViz2 中使用 **“Nav2 Goal”** 发送导航目标。

在发送 Nav2 Goal 或 FollowPath 前，另开一个终端确认定位已就绪：

```bash
source /opt/ros/humble/setup.bash
source /home/zkam/test_nav/install/setup.bash
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo map base_link
ros2 topic echo /static_localization/ready --once
```

前一条必须收到一条 `/odom`，第二条必须持续输出变换，第三条必须为
`data: true`；否则先在 RViz 重新执行 **2D Pose Estimate**，等待一次
`NDT accepted` 后再执行导航或采集标定点。

`2D Pose Estimate` 是固定 PCD 匹配的粗初值；请在机器人真实位置附近点击，
尤其要使朝向大致正确。匹配状态可查看：

```bash
ros2 topic echo /static_localization/status
```

## 动捕参考数据（Tracker0，相对误差）

当前 NOKOV / XINGYING 接入只作为独立参考，不反馈 FAST-LIO、NDT 或 Nav2。使用当前 Tracker0 接收器：

```bash
cd /home/zkam/test_nav
./mocap_tracker0_vrpn.sh
```

接收、单位、时间戳、Tracker0 到 `base_link` 的 18 cm 安装外参，以及静止验证流程见
[`docs/mocap_tracker0_setup.md`](docs/mocap_tracker0_setup.md)。当前静止评估使用
`ndt_post_init_analysis.py`：首次成功接受的 NDT 只标记初始化，再排除3次匹配后正式统计；
被排除期建立唯一固定相对坐标对齐。使用方法、首帧修正量审计及旧模式见
[`docs/ndt_match_evaluation.md`](docs/ndt_match_evaluation.md)。

# 导航模块单独重启说明

正常实车巡检请优先按上方“正常巡检完整启动流程”执行。若只需重启某一个模块，
仍须保证底盘、雷达和定位已经按该流程运行，且定位已成功接受初始位姿。

## 1. `/scan` 桥接（不是机器人坐标系）

`robot_state.sh` 仅提供 Livox 点云到 `/scan` 的回退转换；机器人 URDF、关节状态和
传感器坐标系由 OmniBot 的 `robot_control.launch.py` 发布，不要重复启动 state publisher：

```bash
cd /home/zkam/test_nav
./robot_state.sh
```

---

## 2. 启动导航系统

```bash
cd /home/zkam/test_nav
./navigation.sh 
```

静态定位模式的导航速度连接为：
`controller_server / behavior_server → /cmd_vel_nav → velocity_smoother → /nav/cmd_vel → OmniBot arbitrator_controller`。
底盘接收的是 `/nav/cmd_vel`；对应重映射在
`src/static_map_localization/launch/static_nav2.launch.py`。
修改启动文件并编译后，请在原导航终端按 `Ctrl+C`，再运行一次
`./navigation.sh` 才能生效，不能同时启动两套导航。

## 3. 旧 Ranger 底盘流程（与 OmniBot 流程互斥）

以下内容仅保留给旧 Ranger 设备排障使用。当前 OmniBot 正常巡检**不要执行**，应使用
上方终端 1 的 `omnibot_control robot_control.launch.py`。

### 3.1 配置CAN接口

配置CAN接口，需要系统密码输入orangepi

```bash
sudo ip link set can0 up type can bitrate 500000
```

### 3.2 启动底盘驱动

```bash
ros2 launch ranger_bringup ranger_mini_v2.launch.xml
```

## 动捕反馈控制器实验（独立测试模式）

新流程见 [tests/controller_tracking/README.md](tests/controller_tracking/README.md)，
工程职责、两种状态反馈链和 TF 发布者见 [ARCHITECTURE.md](ARCHITECTURE.md)。
测试使用独立动捕位姿/速度和固定直线/平滑圆弧，内部仍调用原 MPPI，
保留现有底盘执行链；launch 不发送运动目标，实车由用户手动 execute。
原正式导航入口与配置不变；旧 S 数据与专用实现已按授权清理，
清单见 [清理报告](docs/controller_tracking/CLEANUP_REPORT.md)。
## 动捕初始定位与原生建图（独立流程）

源码在 `tests/mocap_anchored_mapping`，运行数据在 `robot_data/experiments/mocap_initial_mapping/<run>/`。
地图保留 FAST-LIO 原生坐标，静止约 3 秒保存固定 map↔mocap_world 和地图原点坐标；
以后启动用动捕与本次局部 LIO 初始化一次 map→odom，新流程全程不启动 NDT。
PCD 不做坐标转换，沿用原生切片流程；初始化后由 LIO 更新，仍可能漂移。
通过 `mapping_modes.sh legacy ...` / `anchored ...` 切换，原入口保持；实车人工遥控，不自动发送目标。
完整终端命令见 [操作说明](docs/mocap_anchored_mapping/README.md)，
删除范围见 [清理报告](docs/mocap_anchored_mapping/CHANGES.md)，
软件和实车状态见 [验证记录](docs/mocap_anchored_mapping/VALIDATION.md)。
