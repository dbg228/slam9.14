# 原 NDT 全图路线测试：实车结果

本轮已由原 FollowPath 返回成功。使用保存的旧 PCD、FAST-LIO + NDT 与原导航参数，动捕仅用于评价。按用户确认，本轮 START/END 标记不同点，相距 0.796 m；不作为严格同点闭环试次。第 2、3 组尚未执行。

| 项目 | 结果 |
|---|---:|
| 实际下发参考路线长度（含接入） | 47.800 m |
| 累计 LIO 估计行驶路程 | 63.739 m |
| 整圈跟踪记录时长 | 355.46 s |
| 实际下发路线：整圈到路径距离 RMSE / 最大 | 42.674 / 123.179 mm |
| 巡检紫线：整圈到路径距离 RMSE / 最大 | 48.517 / 217.897 mm |
| 实际下发路线：整圈航向误差 RMSE | 6.079° |

| 定位评价窗口 | 实际配对样本 | XY RMSE | XY 最大 | 航向 RMSE |
|---|---:|---:|---:|---:|
| 初始 | 3 | 11.847 mm | 16.511 mm | 0.7306° |
| 终点 | 3 | 12.991 mm | 19.815 mm | 1.0487° |

NDT 参与了旧地图与当前动捕的三点配准。因此定位数值是**固定配准关系下的一致性误差**，不能认证独立绝对精度；标定可能吸收 NDT 的固定偏差。初始、终点窗口不参与拟合，不强制初始误差归零。三点拟合残差约 4.10 mm 不是正式定位成绩。每个停车窗口只有 3 个实际 NDT 配对样本，属于单次试验的描述统计。

## 初始、终点定位误差曲线

![初始和终点定位误差](initial_endpoint_errors.png)

[逐样本初始 CSV](initial_error_samples.csv) · [逐样本终点 CSV](endpoint_error_samples.csv) · [PDF](initial_endpoint_errors.pdf) · [SVG](initial_endpoint_errors.svg)

正式初始采用 `initial_capture_06`，在最终定位恢复及路线调整后重新采集；重启前历史初始窗口不替代本轮初始。正式初始位置距最近标定点 0.353 m，终点距最近标定点 0.259 m；此前用户将起点放在第 3 标定点的记录属于历史阶段。

终点采用闭合录包中到站后首个质量合格窗口 `endpoint_arrival_capture_02`，源时刻为导航成功后 8.943～13.943 秒。首个候选 `endpoint_arrival_capture_01` 因 NDT 样本不足及时间缺口失败并保留；按时间顺序检查数据质量，未按误差大小或匹配分数挑选窗口。长时间停车后另采的 `endpoint_capture_04` 仅为辅助，不作为到站成绩。

终点配对使用原 NDT 测量时间戳和同一时刻的有效静止动捕历史，保留 NDT 的实际输出延迟，不改时间戳、不插值、不拟合时间偏移。接收延迟不等于实时控制新鲜度达标。动捕曝光延迟和跨设备时偏未独立测定。覆盖区外没有独立动捕真值，因此不生成全程定位真值误差曲线。

## 行驶距离曲线

![累计 LIO 估计路程](distance_curve.png)

[逐样本里程 CSV](distance_samples.csv) · [PDF](distance_curve.pdf) · [SVG](distance_curve.svg)

累计 63.739 m 来自 3569 个原始时间戳的连续 LIO 底盘中心位置；正式运动区间由最终底盘指令界定，排除三点标定、重启和等待。没有超过 0.5 秒的数据缺口。

录包中的 `/tf` 底盘帧存在丢样，使用同包完整 `/tf_lio_raw`，按原定位桥既有安装外参还原底盘中心。561 个同源时间戳重叠点的最大差异为 1.78e-15 m；未插值、重拟合或改写时间戳，未计入 map→odom 的 NDT 修正。原始底盘帧缺口和恢复审计保存在 `summary.json`。

这是含位姿噪声、漂移和局部跳动的 LIO 累积估计，不是动捕全程实测距离；与 47.800 m 参考路线长度不同。单步最大位置变化 0.322 m 已保留，未用平滑后距离替代。地图轨迹折线长 62.831 m 含全局定位修正，也不作为真实里程。

## 整圈轨迹与跟踪误差

![整圈轨迹跟踪](../tracking_report_final/trajectory_tracking.png)

![整圈跟踪误差曲线](../tracking_report_final/tracking_errors.png)

![地图上的整体跟踪误差](../tracking_report_final/overall_tracking_error.png)

[逐点跟踪 CSV](../tracking_report_final/tracking_samples.csv) · [跟踪统计与图形说明](../tracking_report_final/report.md)

图中实际下发参考路线含接入段；巡检紫线另列作整圈对照，均不单独拆分接入阶段。使用 map 中的小车估计位姿与最近有限路径线段比较；不平移或旋转轨迹。跟踪误差是定位估计与参考路径的偏差，与动捕定位误差分别报告。

共 3548 个跟踪样本，超过 0.5 秒的记录缺口 13 处，曲线不跨缺口连线。Path 时间戳为巡检节点采样时刻，未保存对应 TF 源时间，不能据此进行动态动捕时间同步。到站以原导航成功状态为准，没有另行拟合终点。

## 数据与复核

正式录包 `formal_bag_01` 已正常关闭，保留所有正式初始、运动、到站与辅助停车数据；早期锁冲突失败的录包未混入正式成绩。冻结配准 SHA256：`c843f19dd414b24c7e64a6d14512caa0127ec18c5406fd99027d1fecc16f6e0e`。原 PCD、地图、原参数及所列源码均与出发前记录的哈希一致；前一天原生 PCD 也已核验保留。

[机器可读总表](experiment_summary.json) · [定位和里程详细统计](summary.json) · [输入和输出 SHA256 清单](artifact_manifest.json) · [35 项软件检查日志](../report_software_tests_final.log)

软件检查只验证工具，不冒充实车结果。报告代码及测试冻结在 `report_software_snapshot_03/`。[下载图表、CSV 与报告压缩包](ndt_baseline_results.zip)。压缩包保留目录关系，解压后打开 `final_report/experiment_report.md`。
