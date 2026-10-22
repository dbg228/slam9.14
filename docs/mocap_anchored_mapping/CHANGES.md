# 按当前思路修订与清理

已按用户授权删除上一版地图转动捕坐标的专用实现和无实测价值的生成物；原地图、标定、旧三点/第四点实验、A/B/C、原正式规划/导航/底盘文件不在删除范围。

删除前完整记录见 [cleanup_native_plan.json](cleanup_native_plan.json)，实际删除清单见 [cleanup_native_done.json](cleanup_native_done.json)。共 **311 个旧文件/符号链接**，包含：

* 上一版独立包的 `.validation` 合成 M 地图/日志、`.build`、`.install`、`.log`、旧 pytest/Python 缓存。
* 旧 `tools/launch_smoke.py`（加载 M 地图/NDT 的专用软件脚本），替换为本轮 `native_launch_smoke.py`。
* 旧预备目录 `robot_data/maps/mocap_anchored/20261017_initial_anchor_01/` 的 9 个配置/索引文件。删除前与上轮清单核对哈希，并确认没有新增采集数据、真实 PCD 或标定；随后删除其空父目录。
* 此前生成的旧源审计、旧预备状态、旧软件检查和旧文件索引 JSON，共 6 个文档产物。保护快照 protected_before/after 保留。

代码修改：

| 位置 | 实际变更 |
|---|---|
| core.py | 保留完整 SE(3)/安装偏移/时序/静止检查；map=F0；输出 map↔mocap_world、地图原点坐标；重启变换与独立验证共用矩阵计算；拒绝旧 schema |
| online.py | 建图保存固定世界标定；每次定位在独立目录采集，保存本次 map→odom；验证只读不拟合；两模式均无 NDT；无运动命令 |
| products.py | **删除 transform_pcd、convert、M-PCD 来源分支**；仅核对原生 PCD 与原投影，投影前后 SHA 不变 |
| config/site.yaml | schema=2，map_basis=native_fastlio_mapping_world；删除 M 切片高度/null 等专用配置 |
| cli.py、mapping_modes.sh | 移除 convert 命令；增加只读 validate；新 prepare/check/save/project 指向原生方案 |
| launching.py | **删除 NDT 和校正广播节点分支**；定位时 MC/LIO 一次初始化 map→odom；导航检查冻结状态与新鲜度，不再检查 NDT ready |
| isolation.py | 保留原生硬编码 Log/PCD 隔离；定位日志与本次初始化一起归入 localizations/<id> |
| package.xml、setup.py | 版本 0.2.0；移除 static_map_localization 依赖；依赖原生 LIO、原 Nav2 和共享测试模块 |
| test/test_mapping.py、tools/software_smoke.py | 删除 M 点云转换测试；改为原点、重启方向、5cm 独立误差不被重拟合消除、原生投影等检查 |
| tools/native_launch_smoke.py | 新增实际二进制启动、无 NDT、新栅格加载、未初始化导航拒绝与原辅助目录保护检查 |
| 包 README、当前文档、根 README/ARCHITECTURE 的新增流程段落 | 替换旧 M 地图流程，给出新命令、坐标与结果状态 |

`source_audit.py`、`verify_protected.py`、三个薄 launch 入口、共享同步/四元数/统计模块保留复用，没有再堆另一套重复包。

新真实准备目录：
`/home/zkam/test_nav/robot_data/experiments/mocap_initial_mapping/20261017_native_01/`。
源码仍在 `tests/mocap_anchored_mapping/`，数据目录只存运行配置快照和结果。

当前 0.2.0 的构建和合成测试产物已重新生成在包内隐藏目录，它们与上述已删除的 0.1.0 旧产物区分；本轮软件失败尝试亦保留，不当作实测。精确当前文件列表见 [file_manifest.json](file_manifest.json)。
