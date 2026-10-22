# 曲率预瞄跟踪器与执行链感知残差 MPPI

目标是在同一 FAST-LIO＋NDT 定位框架下研究入弯滞后和出弯过修。按用户最新要求，**巡检轨迹跟踪不考虑避障**，只沿固定轨迹运行。当前已实现控制器、原 MPPI 插件离线适配、合成闭环测试与历史数据诊断；**尚未完成新控制器的实车 FAST-LIO＋NDT 闭环验证**。

具体方法和文献边界见 [DESIGN.md](DESIGN.md)，实验结果见 [RESULTS.md](RESULTS.md)。本目录不修改正式定位、地图、导航配置或底盘代码，不向车辆发送命令。

## 实现

- [controller.py](controller.py)：路径投影、曲率/曲率变化率前馈、跟踪反馈、残差采样、执行滞后预测、矩形足迹筛选和加权结果复核。
- [acceleration/](acceleration/)：与Python参考方程数值对照的C++预测加速，无ROS依赖。
- [native/](native/)：通过原pluginlib工厂加载已安装Nav2 1.1.20原MPPI，读取正式`InspectionStrict`参数；离线内存地图、合成状态，不是实车链路。
- [benchmark.py](benchmark.py)：同路径、同扰动、同执行约束的闭环仿真。每轮独立进程，失败计入总数。
- [history.py](history.py)：旧FAST-LIO＋NDT录包衍生数据的转弯诊断；原始数据只读。
- [shadow_node.py](shadow_node.py)：原`/odom`与同时间戳`map←odom`TF的影子接入，只发布私有诊断和提案，不连接执行器；纯自检通过，尚未连接在线定位测试。

追踪器给出基础转向，MPPI在其周围采样残差，并通过同一执行预测选择最终动作。巡检入口不传入障碍，障碍代价不参与优化。已有可选障碍分支和实验仅保留为开发记录，不属于本轮验收。

## 复现

在工作区根目录运行，依赖现有Python3、NumPy、Matplotlib、pytest、g++和本机ROS2 Humble/Nav2开发包；没有额外pip安装。

```bash
bash tests/turning_mppi/native/build.sh
python3 tests/turning_mppi/acceleration/fast_rollout.py
OPENBLAS_NUM_THREADS=1 python3 -m pytest -q tests/turning_mppi/test_controller.py
OPENBLAS_NUM_THREADS=1 python3 tests/turning_mppi/acceleration/check_equivalence.py

OPENBLAS_NUM_THREADS=1 python3 tests/turning_mppi/benchmark.py \
  --output /tmp/turning_tracking --methods native tracker ideal full \
  --scenarios left right s_bend --plants nominal heldout --seeds 0 1 2

python3 tests/turning_mppi/report.py --tracking /tmp/turning_tracking \
  --output /tmp/turning_report.md
```

输出JSON、NPZ原始轨迹与PNG，不覆盖历史录包。manifest记录参数、代码及正式导航配置哈希。`TURNING_MPPI_BACKEND=python`可切回参考方程，仅用于验证；其运行速度不足20 Hz。

只读影子接入可在已经手动启动的原定位链旁运行；默认计算巡检跟踪提案，不计算障碍代价，仍不连接车辆：

```bash
source /opt/ros/humble/setup.bash
python3 tests/turning_mppi/shadow_node.py --self-test
python3 tests/turning_mppi/shadow_node.py
```

`nominal`是开发模型，`heldout`增加执行滞后、延迟和角速度偏置，控制器仍使用固定参数。三种控制场景为左右90°平滑弯和S反向弯，弯道区间预先由路径曲率确定。所有MPPI方法50步、1000样本；纯跟踪器不采样。

## 接下来怎样做同定位实测

真实验证需沿用原FAST-LIO＋NDT进程、原地图与TF、原定位参数及同一实际下发路径，只替换控制器，并用独立测量评价。先辨识舵轮执行响应，再冻结参数和路径，交错重复运行原MPPI、纯跟踪器、消融和完整方法。不能用旧轨迹回放，推断新控制器已经改变了实车轨迹。

巡检模式继续使用已有的障碍代价关闭设置；本轮不以避障能力作为验收项。
