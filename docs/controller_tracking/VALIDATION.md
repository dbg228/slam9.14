# 软件验证与实车状态

**软件流程已验证；本轮未启动实车，尚无新的实车精度或成功率结论。** 软件数据均标记 `software_fixture`，不能冒充真实实验基线。

## 已执行

- `controller_tracking_probe`、`controller_tracking` 两个ROS 2包独立构建通过；原MPPI以pluginlib加载成功。
- pytest覆盖完整3D刚体偏移与地面旋转、四元数异常/倾斜、±π跨界、静止冻结、时间配对（含Humble byte型DiagnosticStatus.level）、无外推与丢帧、速度窗口中点一致性、直线/左右曲率连续圆弧、线段投影、沿终点切线停车扫掠、四项配置差异/冻结篡改、故障锁定/心跳、空真值/失败统计与批次失败防遗漏。
- 生成三种路径的离线预览PNG/JSON，标明当前场地尚未填写；直线示例的保守扫掠+停车范围约 `x=[-.49,1.99], y=[-.395,.395]m`。左右60°圆弧示例范围更宽，**不能假设2m场地足够**。
- 示例圆弧的曲段100%位于完整终点1.4m内，约49.4%位于0.7m内，预览已记录这一基线限制；未自动改MPPI阈值/速度。内部裁剪终点可能不同，比例不是critic激活遥测。
- 原 `navigation.sh`、`localization.sh`、`mocap_tracker0_vrpn.sh` shell语法检查通过；正式Nav2与OmniBot launch的Python语法检查通过，正常入口未改。没有启动其导航或驱动做验证。

## 隔离ROS集成

使用显式、启动前无其他节点的 **ROS_DOMAIN_ID=192**。只启动合成动捕/理想运动fixture和测试链，未加载真实硬件、CAN或正式导航。命令：

```bash
source /opt/ros/humble/setup.bash
source tests/controller_tracking/.install/setup.bash
PYTHONPATH="$PWD/tests/controller_tracking:${PYTHONPATH:-}" \
  python3 tests/controller_tracking/tools/software_smoke.py \
  --domain 192 --output tests/controller_tracking/.validation/new_software_check
```

工具拒绝使用当前ROS domain，并检查隔离domain为空。集成检查发现并修复了Humble DiagnosticStatus.level为bytes的兼容问题，以及DDS图发现尚未完成造成的短暂空图误判；现在有界等待发现，严格保留唯一发布者/输入检查。所有早期软件尝试日志仍在 `.validation/`，不纳入实车统计。

最终通过的证据在 [verified_smoke/result.json](../../tests/controller_tracking/.validation/verified_smoke/result.json)：

| 检查 | 软件结果 |
|---|---|
| launch默认状态 | gate_status=disarmed，未激活 |
| 原定位残留订阅/命令与TF唯一发布者/生命周期 | check.json通过，无FAST-LIO/NDT速度或位姿输入 |
| 固定直线完整FollowPath | 原MPPI收到61点；当前请求frame/stamp/点核验通过，8.225s后SUCCEEDED |
| 动捕中断故障注入 | 目标执行中失效，failure=mocap_invalid_or_stale；cancel_response return_code=0 |
| 两次最终命令 | 各50个末段样本全零 |
| 两次录包 | 均正常结束exit_code=0，之后才离线分析 |
| 实际MPPI计算与周期 | 成功软件段165次计算，P95≈8.94ms、最大≈12.56ms；调用间隔约0.0500s；仅代表该软件检查负载 |
| 失败数据支持 | 中断段真值覆盖约91.9%，末尾缺失保留，不填零、不跨缺口插值 |

图表与报告在各round的analysis/，包含轨迹、误差、速度、命令、轮组和timing。软件fixture没有真实舵轮动力学或光学测量噪声，不能用其误差、时间常数或成功率评判实车控制效果。构建和单元测试最终摘要另见 `software_checks.json`。

## 尚需现场确认，未填写为实测值

1. 新相机标定编号、尺度/地面与轴向确认、实测world→level旋转；当前null/false。
2. 起点坐标下稳定动捕与无障碍地面范围、标记可见性、起停/停车余量；当前未确认。
3. 相机曝光延迟/时间同步；当前Ubuntu接收时钟已知，camera_latency_s=null，verified=false。
4. 0x606C轮速反馈单位及真实转向/滚动限幅值；当前false/null。真实目标舵角、CAN到达时刻、逐轮采样时刻、相机原生跟踪置信度/遮挡状态仍无接口。
5. 已沿用用户确认、刚体未改变的18cm安装偏移；没有拿起始对齐修正比例或倾斜。

实车待完成：标定与场地核实、静止/人工移动反馈方向和尺度检查、模式接管/动捕中断停车核查、先直线再左右圆弧的三次同起点重复记录，以及真实误差/速度/轮组诊断。正式障碍禁用基线按原要求保留；控制器/底盘主要参数和原有安全门控不改。本轮实际运动由用户手动execute。
