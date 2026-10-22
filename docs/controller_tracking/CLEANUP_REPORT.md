# 旧 S 测试清理报告

按用户 2026-10-17 明确授权执行。删除前读取 git status、包清单、入口、依赖、外部引用及运行进程；旧 S 包没有其他包的运行/导入依赖，无旧 S 测试进程或录包写入者。

准确范围仅为 `/home/zkam/test_nav/tests/s_turn_diagnostics/` 内的旧实现、配置、S 专用恢复/执行脚本、测试、构建产物、缓存、历史实测与合成数据。共 **726 个普通文件，1,024,415,328 字节（约 0.95 GiB）**，另有目录和软链接；包括 rosbag、PNG、JSON/分析报告、录包日志、启动日志。旧 CSV 上轮已按用户授权清空，本轮没有扩大删除其他实验 CSV。

逐项范围在删除前落盘：[cleanup_plan.json](cleanup_plan.json)。执行后的逐项路径、类型、字节数、小文件 SHA256、软链接目标及 `removed` 结果见 [cleanup_actual.json](cleanup_actual.json)。所有列出的条目均已删除；未使用 git reset、git clean 或改写历史。

先迁出：

- `common.py`：原有角度、SE(2)、有界插值、连续线段投影、统计和哈希函数；清除 S 几何生成。
- `tools/timestamp_probe.cpp` 与 CMake：通用只读探针，改中性命名；不作为当前执行流程依赖。手动启用才输出指定 CSV。
- 控制链审计事实迁至 `CONTROL_CHAIN_AUDIT.md`，移除旧结果与失效链接。

未删除或修改 A/B/C 实验、其他定位评估实验、地图、标定、FAST-LIO/NDT/MPPI/底盘代码及配置、第 2 创新点几何规划器。删除前保存 [protected_before.json](protected_before.json)，最终复核结果见 `protected_after.json`。根 README 仅追加新流程索引，其既有内容保留。

旧 S console entry、launch 和配置有意退役；正式 `navigation.sh`、`localization.sh`、`robot_control.launch.py` 入口仍兼容。新流程只安装两个测试包，旧包自身内部的 .install/.build 一并删除，没有工作区级清理。
