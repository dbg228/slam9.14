# 当前相机布局：动捕初始锚定建图

使用昨天同一套建图代码及完全相同安装外参；重新采集当前 mocap_world 与新 FAST-LIO 原生地图之间的固定关系。旧 PCD、昨天地图、本轮 NDT 结果分别保留。

初始化要求小车在动捕覆盖区停稳、手柄回中；采集至少 3 秒同步数据。初始化冻结后人工遥控建图，允许离开动捕覆盖区。建图不使用 NDT，不持续注入动捕纠错。

`map` 仍是首次 FAST-LIO 原生世界，PCD 不改坐标；新 `T_mocap_map` 和逆变换保存在 `calibration.yaml`。Tracker0→base_link 及 base_link→IMU/雷达固定安装关系沿用昨天，不重复应用雷达→IMU补偿。

运行日志见 `real_mapping.launch.log`；状态话题 `/anchored_mapping/status` 必须 frozen=true、fault=null 后才能移动。

建图走完整张地图、停车后，在建图仍运行时保存：

```bash
cd /home/zkam/test_nav
RUN=/home/zkam/test_nav/robot_data/experiments/mocap_initial_mapping/20261018_relocated_01
./mapping_modes.sh anchored save --run "$RUN"
```

本页是操作记录，成功状态以实时观测及冻结文件为准。
