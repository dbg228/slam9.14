

# 项目目录说明

本文档用于说明 **robot_xj_project** 项目的完整启动顺序与各模块功能说明，建议严格按照顺序执行，项目位于**/home/zkam/test_nav**，以下是项目目录说明

## 统一 RViz（导航、定位与巡检）

统一使用以下一个 RViz 配置，不需要再打开 `inspection_geometry.rviz`：

```bash
./rviz.sh
```

该配置已包含对齐巡检地图、10 cm 网格、人工 Polygon、1 m 外扩路径与方向箭头、START/MIDDLE/END 标记，以及原有的定位和导航显示。巡检节点单独启动时默认不会再打开第二个 RViz：

巡检默认只把“特征对齐”用于 10 cm 网格显示；Polygon、紫色外扩路径、定位和 Nav2 均保持在原始 `/map` 坐标中，因此可直接执行 `FollowPath`。不要在实车 Nav2 模式启用 `rotate_occupancy_grid:=true`。

```bash
ros2 launch inspection_geometry_tools inspection_geometry.launch.py
```

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
失败任务也会生成图并标出失败状态。注意：当前轨迹来自定位估计，因此它评估的是控制器的**估计跟踪误差**；论文若要声明真实
绝对精度，还需要全站仪、动捕或固定反光地标等独立真值。

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

该终端必须保持运行。不要为了关闭 RViz 或停止巡检节点而在这个终端按
`Ctrl-C`：脚本会同时停止 `lio_localization`、`map_to_odom_bridge`、
`lio_tf_to_base_tf` 和 `/odom` 发布，Nav2 会立即失去 `map -> odom ->
base_link` 坐标链。RViz 已独立为统一配置，可单独关闭或重开。

如果需要在定位脚本中顺带打开 RViz，可使用：

```
LOCALIZATION_RVIZ=true ./localization.sh
```

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
```

前一条必须收到一条 `/odom`，后一条必须持续输出变换；否则先在 RViz
重新执行 **2D Pose Estimate**，不要发送导航任务。

`2D Pose Estimate` 的点击值只是粗初值，定位器会输出附近的精配准候选值。
启动脚本默认接受最多 5 m、180° 的精配准修正；若现场需要更大初值范围，
可以在启动时调整，例如：

```bash
LOCALIZATION_MAX_INIT_SHIFT_M=8.0 ./localization.sh
```

​                                                                                                 

# 导航启动流程

在启动导航功能之前，需要先启动定位功能，并确保已完成建图流程，雷达驱动运行正常，且定位模块已成功完成定位。

## 1. 启动机器人坐标系

启动发布机器人 URDF 及各关节 / 传感器坐标系：

```bash
./robot_state.sh
# ros2 run tf2_ros static_transform_publisher   0 0 0 0 0 0   map odom
```

---

## 2. 启动导航系统

```bash
./navigation.sh 
```

## 3. 启动底盘控制驱动

### 3.1 配置CAN接口

配置CAN接口，需要系统密码输入orangepi

```bash
sudo ip link set can0 up type can bitrate 500000
```

### 3.2 启动底盘驱动

```bash
ros2 launch ranger_bringup ranger_mini_v2.launch.xml
```
