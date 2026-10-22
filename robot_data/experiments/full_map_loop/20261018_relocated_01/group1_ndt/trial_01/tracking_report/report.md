# 整圈地图轨迹跟踪误差

使用原 map 中小车 base_link 估计轨迹与实际下发的固定 FollowPath 比较；不拟合或平移轨迹。该项是跟踪偏差，动捕定位误差见同轮初始/终点报告。

参考路径长 47.800 m；3548 个实际样本；窗口 355.46 s。
到路径距离 RMSE 0.0427 m，P95 0.0847 m，最大 0.1232 m。
航向误差 RMSE 6.079°。

紫线偏差另保留；完整 FollowPath 含接入路径。时间戳为巡检节点采样时刻，未保存对应 TF 源时刻，不用于动态动捕同步。
地图轨迹折线长度含定位修正跳变，实际行驶距离以终点报告的连续 LIO odom 积分为准。
大于 0.5 秒的采样缺口：13；图中不跨缺口连线。

![整圈轨迹](trajectory_tracking.png)

![整圈跟踪误差](tracking_errors.png)

![地图整体误差](overall_tracking_error.png)

图形均导出 PNG/SVG/PDF；tracking_samples.csv 保留逐样本数据。
