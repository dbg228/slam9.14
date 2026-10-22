# 本次真实建图命令

当前状态：原生 PCD 已保存并验证（1,530,037 点，48,961,437 字节），下方 mapping/save 为本轮操作记录，不要重复执行。二维地图已由用户投影生成并核查通过；下方 project 也已执行，不要重复运行。

启动已经执行，当前不要重复启动：

```bash
cd /home/zkam/test_nav
RUN=/home/zkam/test_nav/robot_data/experiments/mocap_initial_mapping/20261017_native_01
./mapping_modes.sh anchored mapping "$RUN"
```

这就是首次空地图入口；启动后自动等待原生位姿，静止约 3 秒采集并冻结，不需要另发初值或采集命令。不是 localization 入口。

当前查看（新终端，不会重启建图）：

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /anchored_mapping/status --qos-durability transient_local
```

```bash
tail -f /home/zkam/test_nav/robot_data/experiments/mocap_initial_mapping/20261017_native_01/real_mapping.launch.log
```

人工遥控建图结束并停稳，保持建图运行，另开终端保存：

```bash
cd /home/zkam/test_nav
RUN=/home/zkam/test_nav/robot_data/experiments/mocap_initial_mapping/20261017_native_01
./mapping_modes.sh anchored save --run "$RUN"
./mapping_modes.sh anchored products --run "$RUN"
```

确认保存成功、PCD 文件存在后才能结束建图。当前由代理启动的 ros2 launch PID 为 29450；已保存后如需从另一个终端退出，可向这个进程单独发 SIGINT（勿按名称批量杀进程）：

```bash
kill -INT 29450
```

需要二维地图时，在保存并结束建图后：

```bash
./mapping_modes.sh anchored project --run "$RUN"
```

原生 PCD 不转换坐标；沿用原切片流程，不需要 M 系高度配置。
