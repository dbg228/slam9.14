# 软件验证与待采集项

本轮是 2026-10-18 相机移位后重新建图、三组只在终点测误差的准备工作。所有模拟结果明确标记 `software_fixture`，没有真实终点成绩。

通过的验证：

- 62 项单元测试，涵盖原生建图、独立世界标定、相机世界变化拒绝旧目录、纠错变换方向、触发顺序、导航活动状态及只读测量的修正版本。
- localhost、独立 `ROS_DOMAIN_ID=209` 的模拟：中途与终点纠错；运动中请求被拒绝；动捕丢失后继续保留修正；采集超时保留旧修正并允许显式重试；map→odom 只在动态 TF 发布；纠错后的另一批样本使用当前修正，并保留预设的非零误差。
- localhost、独立 `ROS_DOMAIN_ID=210` 的原流程回归：建图、保存/投影、重启一次初始化、动捕丢失后继续 LIO、独立静止验证与原生时间回退拒绝。
- `mocap_anchored_mapping` 的独立 colcon 构建通过，新 launch 和 `correct` 命令已安装；shell 语法检查通过。
- 42 个受保护文件逐字节哈希一致，包含旧 PCD/备份、二维图、昨天的动捕地图/标定、正式导航配置、原 NDT 入口与用户已有 README。

可复核的 JSON 在 `robot_data/experiments/full_map_loop/20261018_relocated_01/`：`software_validation/`、`preservation_check.json`、`protected_before.json`、`protected_after.json`。

实际验证命令：

```bash
source /opt/ros/humble/setup.bash
PYTHONPATH="$PWD/tests/controller_tracking:$PWD/tests/mocap_anchored_mapping:$PYTHONPATH" \
  python3 -m pytest -q tests/mocap_anchored_mapping/test tests/controller_tracking/test/test_world_calibration.py
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=209 ROS_LOG_DIR=/tmp/test_nav_checkpoints_ros_logs \
  PYTHONPATH="$PWD/tests/controller_tracking:$PWD/tests/mocap_anchored_mapping:$PYTHONPATH" \
  python3 tests/mocap_anchored_mapping/tools/checkpoints_smoke.py /tmp/替换为新的模拟目录
```

仍需现场完成：当前布局的静止建图锚定、完整人工建图/保存/投影、固定三组相同物理路线和停车时长、旧 NDT 地图到当前动捕的独立评价标定、三组真实运行和终点测量。当前新运行目录没有 `calibration.yaml` 和 `raw/native_F.pcd`，不能作为已完成建图使用。
