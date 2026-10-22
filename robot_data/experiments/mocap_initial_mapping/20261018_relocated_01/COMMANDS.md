# 本次地图已保存：后续加载命令

当前 run 已完成建图和保存，不再运行 mapping 或重复 save。PCD 与二维地图均在本目录，旧地图未覆盖。

检查现有地图：

```bash
cd /home/zkam/test_nav
RUN=/home/zkam/test_nav/robot_data/experiments/mocap_initial_mapping/20261018_relocated_01
./mapping_modes.sh anchored check --run "$RUN"
./mapping_modes.sh anchored inspect --run "$RUN"
```

后续第 2 组在动捕覆盖区停稳，用这一份地图重新采集一次动捕初值；以下命令尚未执行：

```bash
./mapping_modes.sh anchored localization "$RUN"
# 等待 frozen=true、fault=null，在另一终端用相同 RUN：
./mapping_modes.sh anchored navigation "$RUN"
```

仍使用原安装外参和本次冻结的世界关系。每次重新启动 LIO 只重新求本轮 map→odom，不重拟合 MOCAP↔map，不重新变换 PCD。第 3 组则使用 checkpoints 入口，详见 docs/full_map_loop/README.md。
