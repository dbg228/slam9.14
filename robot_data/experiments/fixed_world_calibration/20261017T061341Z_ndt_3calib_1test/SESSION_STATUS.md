# 固定世界坐标标定实测状态

当前阶段：只读定位恢复完成，等待用户在 RViz 设置本轮唯一一次地图初值并保持静止。尚未采集标定点、未求解变换、未采集验证点。

协议：总共四次停放，同一个 base_link 原点；calib_01/02/03 为三个不共线位置，冻结后才采集独立 test_04。每点默认 4 秒，不自动移动小车，不广播标定 TF，不改定位或规划算法参数。

启动预检发现：动捕约 90 Hz、STOP 模式、最终命令零；原 FAST-LIO 进程已退出（旧 launch 日志 exit code -9），NDT 重复使用超过 650 秒前的点云并产生异常大位置。证据在 preflight_before_recovery.json 和 previous_localization_launch.log；不能拿这批数据标定。

恢复动作：仅停止此前由测试启动的只读定位链，按原 localization.sh 原参数重启。雷达、动捕、底盘和 RViz 保留；具体进程和信号见 localization_recovery_actions.json。

session.yaml 的 calibration_id 是本次人工分配的审计标签，明确不是已知厂家相机标定编号；外部编号未提供，单独记录 null。Tracker0 刚体未重建沿用用户确认的安装外参，另存 body_extrinsic.yaml。地图参照来自 NDT，不伪装成独立测量地图真值。

原始采集、配置、标定 YAML 和报告将全部放在本目录；不生成 CSV，不改写历史 frame_alignment_check 实测报告。

实车预检额外发现：NDT 尚未初始化时 final_valid 也可能为 true。已在独立采集工具增加 /static_localization/ready 与 waiting_for_initialization 联合检查，初始化前数据不会参与标定，拒绝记录原样保留。

第1点首次采集失败：NDT 仅2个配对时刻，5秒动捕正常；未改动原始记录或门槛。继续会话：/home/zkam/test_nav/robot_data/experiments/fixed_world_calibration/20261017T061341Z_ndt_3calib_1test_attempt02，定位不重启、初值不重置。
