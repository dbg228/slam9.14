# map ↔ mocap_world 固定三维坐标标定

本轮按用户后续澄清执行：**总共停放四次，同一个 base_link 原点在前三个不共线位置标定，第四个不同位置和朝向独立验证。** 因而是 3 对标定点 + 1 对验证点，不是四组各三个标记球，也不是 9 对标定点。每次静止采集 3～5 秒，默认 4 秒。

工具只订阅数据、写文件，**不广播 TF、不设置初值、不发送速度或运动目标**。不修改定位、导航、轨迹规划算法及其参数；原有 `map_mocap_check.py` 的起始对齐报告保留，本工具不复用其候选变换。

## 实际坐标来源

|对象/接口|内容与参考点|本工具处理|
|---|---|---|
|`base_link`|车体坐标系；其自身原点始终 `[0,0,0]`|此零向量不是地图中的停放点；需世界位姿|
|`Tracker0`|刚体局部坐标系，原点不是 base_link|安装外参单独保存|
|`/vrpn_mocap/Tracker0/pose`，`geometry_msgs/PoseStamped`|mocap_world 中 Tracker0 的原始位置（当前配置为毫米）与四元数|原样存入 raw.jsonl，不当作 base_link|
|`/mocap/Tracker0/pose`，`PoseStamped`|原 normalizer 转为米的 Tracker0 世界位姿|与同整数时间戳的有效性配对，再应用安装外参|
|`/mocap/Tracker0/diagnostics`，`diagnostic_msgs/DiagnosticArray`|normalizer 有效性、时间、原因|无效记录保留；不填零|
|`/static_localization/ndt_match_diagnostic`，`static_map_localization/msg/NdtMatchDiagnostic`|map 中 base_link 的测量时刻位姿及匹配诊断|默认读 `final_map_to_base_at_cloud`；按 `cloud_stamp` 配对|
|人工逐位置测量|map 中同一个 base_link 原点的 XYZ，米|`manual` 模式逐位置录入，必须写测量方法和独立性|
|`/reference_velocity`，`geometry_msgs/Twist`|仲裁后命令，无 header|仅观察静止命令；零命令不独自证明车体静止，同时检查位姿变化|

动捕点计算：`T_mocap_base = T_mocap_Tracker0 · T_Tracker0_base`。

已有配置 `robot_data/config/mocap_tracker0.yaml` 给出的安装关系是 `T_Tracker0_base_link=(0,0,-0.18 m)`、单位旋转，已由用户确认刚体未重建。初始化时仅复制到 **`body_extrinsic.yaml`**，不在世界标定中重新优化。偏移需要先旋转到世界系，不能简单修改世界 Z。三维四元数转换复用 `state.rotation`，不做平面化，也不把刚体世界坐标直接改名为车体坐标。

**本方案不再需要三个标记球局部坐标。** 若只有 base_link 局部坐标和动捕位姿，仍缺 map 参照；本工具通过显式选择 `ndt` 或 `manual` 补齐。NDT 模式无需伪造独立地图控制点，但也不具备独立绝对真值。

当前 VRPN 配置为 Ubuntu 接收时间，不是相机曝光时刻；相机延迟未知、默认 null。`clock_offset_s` 定义为 `动捕时间 − NDT 点云时间`，默认 0 **不代表已验证同步**。仅取时间差门槛内的最近有效动捕样本，不插值、不外推、不跨丢帧补点。静止采集不能验证动态延迟。

每次采集还按相同 header 时间戳核对原始毫米数据与归一化米数据的比例及方向；默认至少 90% 的有效归一化样本应能配对原始数据，比例或坐标不符会拒绝作为标定输入。保存各话题发布者身份，要求每个关键输入恰好一个发布者，避免重复接入混淆来源。Ctrl+C 中断仍保存原始记录与失败摘要。

地图侧不使用 `/static_localization/pose` 的发布时刻，也不使用以当前时间重发的 `map→odom` 冒充新测量。默认 `final_map_to_base_at_cloud` 是当前保留的 NDT 校正在点云时刻的最终估计；匹配被拒绝时可能沿用上一校正，这种状态写入原始记录及配对表。也可在冻结前选择 `raw_ndt_map_to_base_at_cloud`，评价未经接受门控的直接 NDT 候选。二者不能混合；原始诊断同时保留 raw/final、accepted/stage、分数及计算耗时。

实车预检确认：初始化前 `final_map_to_base_at_cloud_valid` 也可能为 true，不能单看这个标志。采集器同时只读订阅 `/static_localization/ready`（std_msgs/Bool，transient local），拒绝 ready=false 或 `waiting_for_initialization` 阶段的数据；ready 的原始消息与每次配对状态一并保存，不修改原定位节点。

## 命令和采集顺序

源码入口无需重新构建；离线求解只需要 Python、numpy、PyYAML。采集另需 ROS 2 和工程现有消息类型。

```bash
cd /home/zkam/test_nav
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 tests/controller_tracking/tools/world_calibration.py --help

python3 tests/controller_tracking/tools/world_calibration.py init \
  --session robot_data/experiments/fixed_world_calibration/site_01 \
  --mocap-config robot_data/config/mocap_tracker0.yaml --map-source ndt
```

以下用 shell 变量简化命令，路径由你按本次实验命名：

```bash
CAL_SESSION=/home/zkam/test_nav/robot_data/experiments/fixed_world_calibration/site_01
CAL_TOOL=/home/zkam/test_nav/tests/controller_tracking/tools/world_calibration.py
```

先编辑 `$CAL_SESSION/session.yaml`，填写/确认：

- `calibration_id`：本次相机标定的可追溯编号或人工会话标签，不能套用旧候选。
- `map_reference_id`：本次固定地图及初始化会话的标签；`map_reference_files` 可列绝对地图路径以冻结 SHA256。当前 `localization.sh` 默认 PCD 为 `robot_data/maps/data/scans.pcd`，若实际通过环境变量覆盖，应填实际文件。
- `physical_reference_confirmed: true`：双方确实同一个 base_link 原点，安装偏移/轴向仍成立；不是程序自动测量出来的确认。
- NDT 模式确认正式节点 `base_frame=base_link`、`map_frame=map` 和所选 pose 字段；若人工模式，填写 `map_method`。
- 时间参数、静止门槛、三角形尺寸门槛及验收门槛。默认几何门槛是点间距至少 0.10 m、三角形最小高 0.05 m；这是拒绝退化的门槛，**不是已确认场地尺寸或保证精度的推荐布点距离**。
- `acceptance` 默认 null；未预先指定精度指标时只报告误差，不能宣布精度达标。

冻结前可反复检查并修改；冻结后不可改动。同一组采集失败也保留，重新采集需新建会话，不能拿第四点反馈调整本会话。

```bash
python3 "$CAL_TOOL" check-inputs --session "$CAL_SESSION"
python3 "$CAL_TOOL" freeze-inputs --session "$CAL_SESSION"
```

传感器沿用现有入口。仅在尚未运行时启动相应进程，避免重复发布：`./mocap_tracker0_vrpn.sh`、雷达原启动脚本、`./localization.sh`。NDT 若等待初值，由人在 RViz 设置一次；**四次之间不可再次设置初值、切换地图或重启定位重定义 map**。本工具不会自动启动这些进程。

人工在覆盖区内依次摆放并停稳：第 1～3 个位置形成三角形，各位置适当改变朝向。无需全自动移动，不启动导航目标。

```bash
# 位置 1 已停稳时执行
python3 "$CAL_TOOL" capture --session "$CAL_SESSION" --group calib_01
# 人工换到位置 2、停稳后执行
python3 "$CAL_TOOL" capture --session "$CAL_SESSION" --group calib_02
# 人工换到位置 3、停稳后执行
python3 "$CAL_TOOL" capture --session "$CAL_SESSION" --group calib_03
```

采集窗口由冻结配置指定；每次先预热 2 秒，再记录默认 4 秒有效窗口。完整原始 journal 包含预热及尾部排空，并以 `in_capture_window` 区分。检查各 `summary.json` 的 `quality_passed`、失败原因、有效数、时间覆盖、静止变化和 NDT 接受状态；退出码 2 表示失败，不覆盖原始数据。

若选择 **manual**，每次移动后重新测量该位置的地图 XYZ，编辑生成的 `map_inputs/calib_01.yaml` 等模板，再执行录入。不能复用上一个位置的地图坐标，更不能把 base_link 自身 `[0,0,0]` 当成地图坐标。

```bash
# manual 模式示例；每个位置分别执行相应命令
python3 "$CAL_TOOL" set-map --session "$CAL_SESSION" --group calib_01 \
  --file "$CAL_SESSION/map_inputs/calib_01.yaml"
```

完成前三点后求解一次，再人工移到第四个不同位置、改变朝向并停稳：

```bash
python3 "$CAL_TOOL" solve --session "$CAL_SESSION"
python3 "$CAL_TOOL" capture --session "$CAL_SESSION" --group test_04
# manual 模式还需填写并 set-map test_04；NDT 模式自动取本位置数据
python3 "$CAL_TOOL" verify --session "$CAL_SESSION"
```

第四点距每个标定位置至少 0.10 m、与每次标定朝向至少相差 10°，门槛可在**冻结前**根据场地确定。工具检查完整三维姿态差（常规平地等同航向差），不据此拟合旋转；违反时仍保留误差并标记协议失败。不是让实车自动走这些距离。

安装后的等价入口是 `ros2 run controller_tracking world_calibration ...`，构建命令见包 README；源码入口与安装入口调用同一实现。

## 求解、报告及比较含义

使用等权 Kabsch/SVD，**只对三个标定位置的平均 XYZ 拟合**：

```text
p_map = R_map_mocap · p_mocap + t_map_mocap
RᵀR=I，det(R)=+1，scale=1
p_mocap = R_map_mocapᵀ · p_map − R_map_mocapᵀ · t_map_mocap
```

不使用航向来掩盖共线退化；不拟合尺度、不逐位置重新对齐。三点在同一平面仍可解唯一的正旋转，但必须不共线；软件检查两边的点距和三角形最小高。三点只有最低限度冗余，不能可靠识别唯一异常点或自动剔除点；三角形各边差写入 YAML，所有点都参与，不静默选择较好子集。点标签顺序固定，必须对应同一车体原点的同次停放。

输出每个位置 XYZ 有符号误差、三维距离、XY 距离，分别汇总 RMSE、最大绝对值和样本数。标定 3 点与验证 1 点完全分开；**验证只有一个位置，因此该位置三维距离的 RMSE 和最大值相同**，不是多位置重复实验的统计结论。每个位置的原始时间变化和散布保存在 summary，不把多帧静止数据伪装成多次独立验证。

同时保存两个方向的旋转矩阵、平移及 4×4 矩阵：动捕原点在 map 中就是 `t`，地图原点在 mocap_world 中是 `−Rᵀt`。绝不能只对平移取负。

**当地图坐标来自 NDT，误差正是你要求保留的 NDT 参与下的坐标一致性误差。** 它还包含动捕、安装外参及时间配对影响，无法仅凭四个位置分离各项。固定刚体偏差可能被前三点的标定吸收；第四点保留未被固定变换解释的误差。后续比较你的方法时，应复用这份变换、参考点和数据协议，不能各方法分别重新拟合后再比较“精度提升”。若需评价真正独立的地图绝对精度，还需独立测量地图控制点。

## 数据目录与冻结约束

```text
site_01/
  session.yaml, frozen_inputs.json       # 坐标来源、时间、门槛及 SHA256
  body_extrinsic.yaml                   # 独立安装外参快照，非世界标定
  map_inputs/*.yaml                     # 人工坐标模板；NDT 模式无需填
  calib_01/ ... calib_03/
    raw.jsonl                          # 原始 VRPN/normalizer/diagnostic/NDT/命令
    summary.json                       # 平均点、散布、匹配时间、有效性及缺失
    map_coordinate.json                # 仅 manual 模式
  calibration.yaml, calibration.lock.json
  calibration_report.md                # 三点拟合误差、原点坐标、来源限制
  test_04/                             # 同上，采集时记录冻结标定哈希
  validation.json, validation_report.md # 固定变换下的独立第四点误差
```

不创建重复 ZIP、不输出 CSV、不删除旧实验。禁止覆盖采集和结果；前三点求解后文件哈希冻结；第四点目录已存在时拒绝求解。验证复核输入、原始记录及变换哈希，绝不重新拟合。需要改变标定条件时新建会话，原失败会话保留。

## 软件检查与尚缺实测

```bash
PYTHONPATH=tests/controller_tracking python3 -m pytest -q \
  tests/controller_tracking/test/test_world_calibration.py
```

测试覆盖完整三维变换/逆向/原点、三点退化、禁止缩放、第四点注入误差保留、验证不可重拟合、安装外参变更拒绝、旋转后的安装偏移、±π/四元数等价、丢帧/陈旧数据/非零命令/NDT变化。

也提供隔离域的合成 ROS 采集流程，**会发布测试话题，只能使用不同于当前实车域的空闲域**。脚本限制域号 100～232、拒绝当前域和已发现的其他节点，强制 localhost，不启动任何底盘或定位进程。以下输出必须选不存在的新目录：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 tests/controller_tracking/tools/world_calibration_smoke.py \
  --domain 195 --mocap-config robot_data/config/mocap_tracker0.yaml \
  --output tests/controller_tracking/.validation/fixed_world_calibration/my_software_fixture
```

合成流程依次执行真实 capture 命令、三点求解、独立验证和一次 Ctrl+C 中断；注入第四点 `[0.03,-0.04,0.12]m` 误差，应恢复三维 0.13m、XY 0.05m，且标定不变。此数值仅验证软件，不是本车精度。

新会话仍需填写相机标定标签、地图会话标签并确认参考点；在稳定动捕和可手动停车的范围内布置三角形与独立第四点。已有动捕原生光学置信度不可获取，normalizer 的 valid 不等于厂家跟踪置信度。动态同步未独立验证；软件通过不等于实车精度通过。详细软件验证记录见 `FIXED_WORLD_VALIDATION.md`。

## 2026-10-17 实测结果及汇总命令

已按本协议采集实车四个位置，见 [本次实测报告](../../robot_data/experiments/fixed_world_calibration/20261017T061341Z_ndt_3calib_1test_attempt02/report.md)。三点拟合三维 RMSE 11.159mm，独立第四点三维误差92.183mm、XY误差53.691mm，不能据拟合残差宣布高精度。

本机 NDT 的正式 `match_period_s=2.0` 保持不变。冻结本轮测试配置时，采集窗口取5秒、仍要求至少3个配对时刻、NDT最大间隔取2.5秒以容纳时间戳抖动。窗口开始时刻按匹配输出节拍选择，**不是按误差选择**；各点的 `_phase.json` 和 capture 日志保留执行证据。普通4秒窗口可能只含2次匹配，不满足门槛应标记失败，不能伪造第三帧或降低门槛凑结果。

本次第1点曾因时刻数不足产生一段失败窗口；第4点首段因配对/间隙质量失败，在同位置仅补采一次。第二次验证会话通过相对软链接共享原配置、前三点和冻结变换，**没有重新运行 solve**；同位置检查、补采规则和失败原始记录均保留。验证数值较大也未再补采、拟合或改参数。

新的只读报告命令可以重新生成点位叠加与 XYZ 分项图，不会修改标定 YAML：

```bash
python3 tests/controller_tracking/tools/world_calibration_report.py \
  --session robot_data/experiments/fixed_world_calibration/20261017T061341Z_ndt_3calib_1test_attempt02 \
  --verification-session robot_data/experiments/fixed_world_calibration/20261017T061341Z_ndt_3calib_1test_attempt02/validation_retry_02
```

若普通会话四段都一次通过，不需要 `--verification-session`。补采会话必须显式指定，报告工具不会按残差自动挑选。报告与图是可再生派生文件，原始 JSONL、冻结配置、标定及验证输出仍禁止覆盖。
