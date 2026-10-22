# 动捕初始定位与原生建图

当前相机已移动、两区域共用世界坐标；全图三组测试统一在终点测误差。新建图目录、`checkpoints`/`correct` 入口和终点纠错前后测量见 [本轮说明](../../docs/full_map_loop/README.md)。原 `localization` 仍仅给一次初值。

ROS 包在 `tests/mocap_anchored_mapping`。源码/launch/默认参数在包内；每轮参数快照、原始数据、地图和标定结果归档在 `robot_data/experiments/mocap_initial_mapping/<run>/`。

* 地图保留首次 FAST-LIO 原生坐标，`map=F0`；PCD 从不做动捕坐标转换。
* 建图静止初始化约 3 秒，保存固定 `T_mocap_map` 和逆变换，并明确输出 map 原点在动捕中的 XYZ。
* 重启定位时，再静止采集动捕与本次原生局部 LIO，使用原标定计算一次 `T_map_odom`。两阶段均不启动、不订阅 NDT；没有自动发送运动目标。
* 初始化后仅 LIO 持续更新，允许动捕离开覆盖区；这不是持续动捕定位，后续仍可能漂移。
* 独立验证只计算误差，不调整世界标定或当次 map→odom。

模块：`core.py` 坐标、质量与冻结输入；`online.py` 同步采集/一次初始化/只读验证；`products.py` 原生 PCD 检查与原投影复用；`cli.py` 操作命令；`launching.py` 建图/定位/导航接线；`isolation.py` 原生固定日志目录隔离。
复用 `controller_tracking` 的四元数、有效性关联、静止统计与毫米到米证据核查，复用根目录原切片/去噪脚本。原入口、MPPI、几何规划、底盘和旧地图均保留。

完整终端流程见 [操作说明](../../docs/mocap_anchored_mapping/README.md)，验证与未完成实车项见 [VALIDATION.md](../../docs/mocap_anchored_mapping/VALIDATION.md)，删除范围见 [清理报告](../../docs/mocap_anchored_mapping/CHANGES.md)。

```bash
cd /home/zkam/test_nav
source /opt/ros/humble/setup.bash
source /home/zkam/fastlio_ws/install/setup.bash
source install/setup.bash
source tests/controller_tracking/.install/setup.bash
colcon --log-base tests/mocap_anchored_mapping/.log build \
  --base-paths tests/mocap_anchored_mapping --packages-select mocap_anchored_mapping \
  --build-base tests/mocap_anchored_mapping/.build \
  --install-base tests/mocap_anchored_mapping/.install --symlink-install
PYTHONPATH="$PWD/tests/controller_tracking:$PWD/tests/mocap_anchored_mapping:$PYTHONPATH" \
  python3 -m pytest -q tests/controller_tracking/test tests/mocap_anchored_mapping/test
```

原生进程使用本机已安装的 bubblewrap，把硬编码 `ROOT_DIR/Log` 和 `ROOT_DIR/PCD` 挂到本轮私有目录。网络、算法、原二进制和宿主原目录保持。缺少命名空间能力即拒绝启动，不退回共享写盘。
软件合成数据只放 `.validation`，带 software_fixture 标签，限制在显式 localhost 测试域运行。

## 已采集的 2 米前进定位实验

2026-10-17 已采集5组完成记录，全部尝试及失败原因见[汇总报告](../../robot_data/experiments/mocap_initial_mapping/20261017_native_01/dynamic_trials/FORWARD_STATUS.md)。每轮目录各自包含录包、原始消息、误差图、报告和冻结输入。CSV已按用户要求删除；新轮通过manifest中的 `export_csv=false` 禁用导出，汇总同样不生成CSV。

`tools/auto_2m_trial.py` 复用原有 `localization_ab_test` 的0.15m/s请求、动捕base_link位移2米发零、静止5秒及停车后5秒记录协议；它是单独的实车运动工具，建图/定位启动入口仍不发布运动命令。操作员保留手柄模式控制；人工退回后逐轮启动。本次按用户要求使用人工复位。

工具对已准备好的单轮目录默认只预检；明确加 `--execute` 才会在数据、模式、发布者、底盘看门狗和硬件状态全部通过后执行一次。单轮必须使用新目录及正确的 `manifest.json`、`watchdog_preparation.json`，不得重跑已有记录目录。离线命令不启动ROS或发布运动：

```bash
# 在对应ROS环境中，仅重新分析已有单轮原始数据
python3 tests/mocap_anchored_mapping/tools/auto_2m_trial.py \
  --trial /absolute/path/to/archived_trial --analyze-only

# 使用同一ROS环境的消息类型库，但不初始化ROS、不启动节点
# 从原始消息汇总完成轮，生成逐轮图、平均误差图和JSON，保留失败统计
python3 tests/mocap_anchored_mapping/tools/forward_trials_report.py \
  --root /home/zkam/test_nav/robot_data/experiments/mocap_initial_mapping/20261017_native_01/dynamic_trials
```

第1组与第2、3组的局部初始化不同，人工起点也有偏差，汇总报告如实记录；不能宣称严格同一路线的重复性。动捕时差尚未独立标定，XY误差不包含Z方向，也不独立证明世界标定精度。

`forward_comparison.png` 显示每轮曲线；`forward_mean_error.png` 显示按运动开始计时、0.5秒分箱的等权平均与轮间标准差。每轮每箱至少3个有效配对，最大间隔（含箱边界）不超过0.2秒；全部轮都有数据才显示均值，其他箱留缺口并报告支持轮数。不跨缺口插值，不将缺失填零。全程平均XY距离误差与RMSE分别报告，详细数值保存在 `forward_summary.json`。
