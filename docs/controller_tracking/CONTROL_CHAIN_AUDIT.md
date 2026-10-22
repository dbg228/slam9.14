# 本机控制链审计

架构、接口、参考点及时间表见 [ARCHITECTURE.md](../../ARCHITECTURE.md)。本文件迁移旧诊断中的通用核查事实，不沿用已删除的S结果或失效链接。

Nav2依据为 `navigation.sh`、`src/static_map_localization/launch/static_nav2.launch.py`、`robot_data/config/navigation.yaml` 及本机Nav2 1.1.20：FollowPath为MPPI DiffDrive，位姿由costmap TF查询，速度为OdomSubscriber缓存并经阈值处理。正式链 `/cmd_vel_nav → velocity_smoother → /nav/cmd_vel`；实验链详见架构表。平滑器OPEN_LOOP启动时不创建odom订阅，配置仍重定向，在线图也检查残留输入。

OmniBot源码 `/home/zkam/OmniBot_ws/src/omnibot_control/src/controller/arbitrator_controller.cpp`：仅订阅teleop/mode、teleop/cmd_vel、nav/cmd_vel，无定位输入；nav_w_scale=-1、EXTERNAL watchdog100s、回零与模式门控保留。输出/reference_velocity约500Hz；当前Humble链以topic传递，不因chained_mode=true推断为共享接口直连。

`omni_velocity_controller.cpp`：约2000Hz，基于实际舵角和几何参数求四轮转向/滚动角速度，保留内部加速度、舵角滤波、防翻转及快速转向限速。joint_vel诊断读取已写入command_interfaces的turn1..4/roll1..4角速度。fai是实际原始/滤波舵角，没有真实目标舵角下发证据。

`robot_system_hardware.cpp`、`std_canopen_motor_driver.cpp`、`hardware_params.yaml`：默认RPDO1(0x200+node)、0x6040+0x60FF、SYNC500Hz，USB EMUC→Kinco FD134；转向ch0/node1..4、滚动ch1/node5..8，减速比12/10.75。UDP是可选旁路，不是必经通信链。反馈来自TPDO1的0x6064/0x606C缓存；轮1左后、2右后、3右前、4左前。

0x606C单位尚待手册/实物确认：反馈采用位置式2π/encoder/gear，命令DEC=rpm×512×encoder/1875。如同为DEC速度单位将有16.384因子疑点，但不能据此认定故障；工具保留原值、物理速度执行误差置未知。

原定位桥接传感器安装(0.37,0.28,0.15)m、yaw≈0.82292274rad；odom→base_link保留LIO时间；map→odom是NDT修正高频重播；原/odom桥接用now时间，不能单凭新鲜header认定姿态新鲜。新实验完全不用这些反馈。

Tracker0 raw毫米、normalized米、Ubuntu接收时间；已确认T_Tracker0_base_link=(0,0,-.18m)且轴向一致，用户说明相机重标定后刚体未变。该确认不能替代新世界地面/尺度核验。测试不使用旧mocap→map变换。

原障碍禁用状态保留，不自动改动；原goal_checker .20m/3.14rad到达容差也保留。因此基线成功和高精度验证须分开解释。
