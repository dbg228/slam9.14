# 三组全图闭环的终点误差测试（2026-10-18）

2026-10-19 更新：第 3 组 `group3_mocap_correction/trial_01` 已按 **0.60 m** 紫线外扩完成整条路线，只在中途做一次动捕纠偏，终点只测误差。中途独立静止 XY RMSE 为纠偏前 **218.367 mm**、纠偏后另一批样本 **2.308 mm**；终点静止 XY RMSE **176.945 mm**（31 对样本）。原生 LIO 在本轮中未重启。结果仅交付 [PNG/SVG 图片包](../../robot_data/experiments/full_map_loop/20261018_relocated_01/group3_mocap_correction/trial_01/group3_middle_result_figures.zip)，包含定位、轨迹跟踪、路程与四轮轮速曲线。

本轮中途有长时间停车，按用户指令放宽原生位姿年龄上限至 3 秒，再以最高 0.10 m/s、0.15 rad/s 续跑；结束后已恢复临时运行参数。全样本跟踪统计保留，同时另列剔除明确停车区间的统计。缺测路段不补零、不跨缺口累计路程；本轮速度条件与前两组不同，不作为严格同条件对比。

**数据复核补正：12.991 mm 仅代表本轮较晚的 5 秒、3 帧配准一致性，不能概括稳定定位精度；63.739 m 是噪声敏感的原始 LIO 位置累加，实际物理路程未独立测定。** [复核报告](../../robot_data/experiments/full_map_loop/20261018_relocated_01/group1_ndt/trial_01/review_01/review_report.md)。

已确认：相机换位置；中途和起终点两处动捕区共用一个当前世界坐标系；按当前布局重新建图；**三组正式结果都在终点测量**。中途只作为第 3 组的纠错点，其数据属于辅助日志。

准备目录：`robot_data/experiments/full_map_loop/20261018_relocated_01/`。
新建图目录：`robot_data/experiments/mocap_initial_mapping/20261018_relocated_01/`。
第 1 组已完成原导航实车路线并导出结果：[报告、全部曲线与 CSV](../../robot_data/experiments/full_map_loop/20261018_relocated_01/group1_ndt/trial_01/final_report/experiment_report.md)。本轮按用户确认允许起终点不同。初始定位 XY RMSE 11.847 mm，终点 12.991 mm；整圈对实际下发路线的距离 RMSE 42.674 mm，对巡检紫线 48.517 mm；累计 LIO 估计路程 63.739 m。第 2、3 组所用的新 PCD 和二维地图已保存。

第 2 组 `trial_02` 已完成整条路线（354.58 秒，顺序终点判定通过）。累计 LIO 估计路程 47.574 m；参考路径 48.122 m；整圈跟踪距离 RMSE 41.853 mm。初始化后静止 XY RMSE 5.651 mm，终点静止 XY RMSE 13.360 mm（各 31 对、3 秒；不重拟合）。中途未收到新 Tracker0 位姿，标记缺测；额外固定 15 秒终点窗口有缺口，不能视为完整窗口。轮速反馈按接口原值绘制，物理单位换算尚未独立核实。[第 2 组实验图片包](../../robot_data/experiments/full_map_loop/20261018_relocated_01/group2_mocap_initial/trial_02/group2_result_figures.zip) 仅包含 PNG/SVG；第 3 组待执行。

另存的“第 2 轮测试版”已按用户要求将紫色巡检路径外扩距离设为 **0.55 m**，删除此前测试版后完成重测，正式第 2 组保持不变。本轮完整路线成功，用时 339.34 秒，参考路径 45.772 m，整圈跟踪距离 RMSE 53.760 mm。初始化后和终点静止 XY RMSE 分别为 4.463、14.385 mm（各 31 对、3 秒；初始值是同一起始位置初始化后的残差，不是空间独立验证）；额外完整 15 秒终点窗口为 14.251 mm。中途获得 42 对、4.10 秒的运动中动捕对照，XY RMSE 99.522 mm；曝光时延和设备时间偏移未标定，不能作为静态定位精度。本轮全程不输入动捕纠错。LIO 已观测片段累计 45.310 m，有一处 0.900 秒缺口未跨越累加，**完整 LIO 总路程缺测**；该值也不代表独立测量的实际路程。[测试版结果图片包（含四轮轮速）](../../robot_data/experiments/full_map_loop/20261018_relocated_01/group2_mocap_initial_test/trial_01/group2_test_result_figures.zip) 仅含 PNG/SVG，轮速反馈仍按接口原值绘制。

**测试版终点成绩复核：上述 14.385 mm 仅保留为原始 XY 数值，撤回其作为有效定位精度成绩的结论，终点待复测。** 原始 VRPN 在出发后 336.462 秒附近出现约 178.85° 的刚体姿态翻转，同期 LIO 姿态只变化 0.30°。Tracker 原点高度只变化约 4.5 mm；近乎倒置的姿态使原有 0.18 m 安装偏移反向，换算出的 base_link 高度差接近 0.35 m，终点三维姿态误差 RMSE 为 179.59°。用户说明地面存在小坡；本次标记依据是原始刚体姿态翻转，并非高度变化本身。录包中 map→odom 仅有一个固定值、定位会话未变化，没有中途或终点动捕纠错。运动中误差与停车误差不能直接比较，也不要求位置误差随路程单调增加。图片包已加入完整三维姿态/高度差诊断图并标注终点参考异常；不修补原始旋转、不重新拟合转换。正式第 2 组数据保持原样。

第1组已使用旧 PCD 完成，无需等待第2、3组重新建图。正式数据与统计口径见 [原 NDT 首轮操作](NDT_BASELINE.md)。下文“重新建图”属于第2、3组的后续工作。

| 组别 | 地图与定位 | 中途 | 正式结果 |
|---|---|---|---|
| 第1组 | 保存的 `robot_data/maps/data/scans.pcd`，原 FAST-LIO + NDT | 不输入动捕纠错 | 12.991 mm XY RMSE（固定配准一致性） |
| 第2组 | 当前布局新建地图；动捕只提供启动初值，此后 LIO | 测可见区误差，不纠错；本次中途缺测 | 终点 13.360 mm XY RMSE（3 秒、31 对） |
| 第3组 | 与第2组相同的新地图、初值和导航参数 | 停车，显式动捕纠错 | 终点纠错前；另列终点纠错后 |

原计划三组使用同一物理闭环路线。本轮第 1 组经用户确认，START/END 标记相距 0.796 m，按非同点路线执行成功。后续比较应复用本轮物理路线，或统一重做严格闭环，避免将路线变化归因于算法。第 2、3 组的停车时长与对应新地图坐标仍待固定；本说明中的后续建图命令不自动发运动目标。

第3组的主要到站成绩是**终点纠错前误差**，其中已包含中途纠错效果。终点纠错后的成绩另列，不能替换到站误差。实际停车偏离目标与估计位姿偏离动捕是两个量，分别报告。

## 新坐标关系

`robot_data/config/mocap_world_session.yaml` 标记当前世界为 `20261018_relocated_shared_world_01`。旧地图和标定保留；动捕流程拒绝启动没有当前世界编号的历史目录，防止沿用旧布局标定。原 NDT 入口、地图和正式 Nav2 参数未改。

仍令 `map=首次 FAST-LIO 原生世界`，不转换整张 PCD 的点坐标：

```text
建图：T_M_map = T_M_B × inverse(T_map_B)
启动初值/每次纠错：T_map_odom = T_map_M × T_M_B × inverse(T_odom_B)
```

`B=base_link`，使用现有完整 Tracker0/IMU 安装外参。两处共用一份新 `T_map_M`，局部纠错只更新本次 `T_map_odom`，不重拟合世界标定。

第1组重新确定“旧地图 ↔ 当前动捕”的关系，**仅用于评价，不反馈 NDT**。用户没有独立地图基准点，已选择三个独立停车位置的 NDT/动捕配准，随后单独测初始和终点，见 [原 NDT 首轮操作](NDT_BASELINE.md)。报告为固定配准关系下的一致性误差，不能认证独立绝对精度。正式初始与终点不参与拟合，不强制初始误差为零。`baseline_world_calibration/` 中独立基准点方案尚未使用。

## 先重新建图

2026-10-18 当前进度：当前布局的新 MOCAP↔map 关系已冻结（32 对、3.100 秒），人工走图已完成。原生 PCD 已保存（3188533 点），二维地图已生成（0.05 m/格，576×541），建图和录包已结束；[地图、预览和保存汇总](../../robot_data/experiments/mocap_initial_mapping/20261018_relocated_01/SESSION_STATUS.md)。第 2 组 trial_02 已完成路线和终点测量，第 3 组待执行；不再重复运行本目录 mapping/save/project。

底盘、Livox、动捕按原流程运行；切换前停止旧建图/定位/导航。小车停在起终点动捕区，终端4：

```bash
cd /home/zkam/test_nav
RUN=/home/zkam/test_nav/robot_data/experiments/mocap_initial_mapping/20261018_relocated_01
./mapping_modes.sh anchored check --run "$RUN"
./mapping_modes.sh anchored preflight --mode mapping
./mapping_modes.sh anchored mapping "$RUN"
```

另一终端先 `source /opt/ros/humble/setup.bash`，查看：

```bash
ros2 topic echo /anchored_mapping/status --qos-durability transient_local
```

等待 `frozen=true`、`fault=null` 且新 `calibration.yaml` 生成，再人工走完整张地图。停稳后保持建图实例运行，在另一终端保存：

```bash
cd /home/zkam/test_nav
RUN=/home/zkam/test_nav/robot_data/experiments/mocap_initial_mapping/20261018_relocated_01
./mapping_modes.sh anchored save --run "$RUN"
```

保存成功后退出建图，再运行 `./mapping_modes.sh anchored inspect --run "$RUN"` 和 `./mapping_modes.sh anchored project --run "$RUN"`。新 PCD、二维图、标定和采集记录均在新目录；不要用旧 `save_map.sh` 保存这一轮。

## 第2组：初值后跑全图，到终点测量

本次补充：第 2 组同时记录中途动捕可见区的定位误差和四个行走轮的轮速曲线，中途不做动捕纠错。结果目录仅交付 PNG/SVG 实验图片，不导出 CSV 或 log。无动捕数据的路段保留缺口。

巡检任务使用顺序终点判定：必须依次经过接入路径、START 和整条巡检路线，再满足原 END 容差才可成功；起终点邻近时不能提前结束。START 是途经点，不是停车目标。仍使用原几何规划界面和操作者所选路线。

停稳启动定位；观察 `/anchored_localization/status`，记录本次 `capture_dir` 为 `SESSION`。等待初始化成功后，在另一终端启动导航服务，由操作者执行固定路线：

```bash
./mapping_modes.sh anchored localization "$RUN"
# 以下在另一个终端，并设置相同 RUN：
./mapping_modes.sh anchored navigation "$RUN"
```

回到终点，取消运动目标并停稳。只读测量，不更新定位：

```bash
SESSION=/替换为本次状态中的/capture_dir
./mapping_modes.sh anchored validate --run "$RUN" --context localization --session "$SESSION"
```

终点报告在 `RUN/validation/<编号>/summary.json`，包括逐样本 XYZ、XY/三维距离统计和姿态误差。

## 第3组：中途纠错，终点分别测纠错前后

退出上一组定位和导航，同一起点停稳，运行 `./mapping_modes.sh anchored checkpoints "$RUN"`。重新打开 RViz，避免旧静态 TF 缓存；等待初值成功，记录新的 `SESSION`，再启动同一导航服务和物理路线。

中途取消导航目标、停稳，显式纠错：

```bash
./mapping_modes.sh anchored correct --run "$RUN" --session "$SESSION" --checkpoint middle
```

成功输出 `outcome=applied` 后继续剩余路线。活动导航/行为目标、非零底盘命令、错误顺序会拒绝纠错。中途误差只作日志，不进入正式对比表。

回到终点后按顺序：

1. 取消运动目标、停稳，执行上面的 `validate`，保存**终点纠错前**成绩。
2. 执行 `./mapping_modes.sh anchored correct --run "$RUN" --session "$SESSION" --checkpoint end`。
3. 成功后再执行 `validate`，用另一批静止样本保存**终点纠错后**成绩。

两次测量生成不同目录，不覆盖。只读测量锁定当前纠错版本，采样中若又发生纠错则失败，避免混合不同变换。

本模式独占动态 `map→odom`，该边不发布到 `/tf_static`；LIO 继续提供 `odom→base_link`，`odom→camera_init` 为恒等静态边。离开动捕区保留上一次修正；超时保留旧修正和失败记录，不自动重试或恢复运动。

纠错日志在 `SESSION/checkpoints/<middle或end>_<编号>/`，含原始流、配对、前后变换、纠错前误差、同窗口纠错后一致性及哈希。**同窗口一致性不是独立成绩**，正式纠错后结果使用随后那次 `validate`。

## 记录和结果口径

每轮使用新目录，从起点前录到终点测量结束：

```bash
ros2 bag record -o /替换为本轮新目录/bag \
  /tf /tf_static /odom /reference_velocity \
  /vrpn_mocap/Tracker0/pose /mocap/Tracker0/pose /mocap/Tracker0/diagnostics \
  /static_localization/ready /static_localization/ndt_match_diagnostic \
  /static_localization/map_to_odom \
  /anchored_localization/native_odom /anchored_localization/status
```

正式表只列“第1组终点、第2组终点、第3组终点纠错前、第3组终点纠错后”。第1组已完成，终点 XY RMSE 12.991 mm、航向 RMSE 1.0487°（NDT 参与固定配准）；第 2 组 trial_02 终点 XY RMSE 13.360 mm（3 秒、31 对），第 3 组两项待采集。保留位置/姿态误差、静止稳定性和失败原因；重复试次先各自统计，再做试次间比较。中途日志不作为额外测试点。

曝光时延和跨设备时间偏移未独立核实，因此使用停车静止窗口；不将未覆盖路段当作有真值的误差曲线。软件模拟结果不能作为实车成绩。

按用户补充，第1组另显示初始误差对照、终点误差曲线及累计 LIO 估计路程曲线；距离从连续 odom 轨迹积分并报告数据缺口。正式组间对比仍取终点成绩。
