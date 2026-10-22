# 固定世界坐标标定：软件验证与交付记录

协议以用户最终澄清为准：**总共四次停放，同一个 base_link 原点；前三个不共线位置联合标定，第四个位置独立验证。** 每个位置不是三个球，也不需要填写三个标记球坐标。操作入口见 [完整说明](FIXED_WORLD_CALIBRATION.md)。

## 已完成的软件检查

|检查|实际结果|
|---|---|
|controller_tracking 全部测试|35 项通过，其中新世界标定测试 15 项（含实测发现的初始化状态检查）；保留原有 20 项|
|三维几何|已知带俯仰/滚转/偏航的正旋转和平移恢复正确，逆向、两个原点坐标正确|
|退化与异常|共线、重复、过近三角形拒绝；尺度不拟合；形状不一致的边长差保留|
|严格留出|只读三个标定位置求解；第四点之前冻结，之后拒绝再次求解；验证保留注入误差|
|实际 ROS 消息采集|隔离 ROS_DOMAIN_ID=195、localhost，使用合成消息，四次 capture 命令全部通过|
|单位、帧与来源|四次各 300 个有效动捕样本、15 个 NDT 配对；原始/归一化匹配率均为 100%；输入发布者均唯一|
|验证误差恢复|第四点注入 XYZ `[0.03,-0.04,0.12]m`，报告恢复三维 0.13m、XY 0.05m；变换哈希不变|
|Ctrl+C 失败保留|独立合成中断会话退出码 2，原始 JSONL 与失败 summary 保留，记录 KeyboardInterrupt；不算有效标定|
|ROS 包|controller_tracking 构建通过，安装入口 `ros2 run controller_tracking world_calibration --help` 可用|
|静态检查|四个新增实现/入口脚本 Python 编译通过；实际采集/求解代码没有 TF、速度、目标发布器|
|原工程保护|历史保护清单 836 文件无缺失，835 一致；README 的既有授权修改来自此前整理，本轮未编辑根 README|
|数据格式|新工具及合成输出没有 CSV；旧数据没有删除、转换或重写|

这些是**软件验证通过**，不是本车世界标定或跟踪精度通过。合成输出均标为 `software_fixture`，不能作为真实实验数据。

工作区全局 `git diff --check` 仍报告根 README 第 38 行原先已有的行尾空格；本轮未编辑该文件，也未为此扩大修改范围。

最终合成验收目录：

后续实测发现初始化前 `final_valid` 也可能为 true，因此采集工具增加 ready + 初始化阶段联合检查，新增两项回归测试。补充完整隔离 ROS 流程在 `software_fixture_05_ready_check`（ROS_DOMAIN_ID=197）通过；`software_fixture_04_ready_check` 是发现 DDS 端点在0.5秒预热内尚未就绪的失败合成尝试，原样保留，后续合成流程采用与实际一致的2秒预热，没有放宽精度或数据质量门槛。

- [software_checks.json](../../tests/controller_tracking/.validation/fixed_world_calibration/software_fixture_03/software_checks.json)
- [三点标定 YAML](../../tests/controller_tracking/.validation/fixed_world_calibration/software_fixture_03/calibration.yaml)
- [标定报告](../../tests/controller_tracking/.validation/fixed_world_calibration/software_fixture_03/calibration_report.md)
- [独立验证报告](../../tests/controller_tracking/.validation/fixed_world_calibration/software_fixture_03/validation_report.md)
- [中断失败记录](../../tests/controller_tracking/.validation/fixed_world_calibration/software_fixture_03/interrupted_capture/calib_01/summary.json)

同目录的 software_fixture_01/02 是开发过程中先后运行的合成流程，保留来源标记；03 是加入原始单位/发布者检查及中断保留后的最终流程。它们不进入真实实验汇总。

## 实车尚未完成的事项

2026-10-17 后续已完成实车四个位置、三点联合标定与独立第四点验证，见 [实测报告](../../robot_data/experiments/fixed_world_calibration/20261017T061341Z_ndt_3calib_1test_attempt02/report.md)。三点拟合3D RMSE为11.159mm，第四点3D误差92.183mm、XY误差53.691mm；没有重拟合或广播变换。共有6段采集，2段质量失败保留；第四点同位置仅补采一次，使用同一冻结变换。当前 `frame_alignment_check/.../report.md` 仍是此前起始对齐历史报告，保持不变，与本次新结果不能混用。

现场需要：本次相机标定可追溯标签、固定地图/初始化会话标签、确认同一个 base_link 原点及未改变的安装外参；人工在稳定覆盖区摆出三个不共线位置，再选择不同位置与朝向的第四点。不得在四次之间重设地图初值、换图或重启使地图参考改变。若要认证独立绝对精度，仍需独立地图参照；NDT 派生坐标按用户要求保留其估计误差。

相机曝光时间/延迟与跨源时间偏移未独立测量；当前原始 VRPN 不含厂家光学有效性/置信度。因此没有宣称动态时间标定、全场精度或遮挡可靠性通过。精度验收门槛未给定时保持 null，只报告数值。

## 本轮文件范围

新增：

- `tests/controller_tracking/controller_tracking/world_calibration.py`：三维无尺度求解、输入冻结、人工坐标录入、独立验证、报告与 CLI。
- `tests/controller_tracking/controller_tracking/world_capture.py`：只读原始采集、安装偏移、时间匹配、单位/发布者检查、静止质量与失败记录。
- `tests/controller_tracking/tools/world_calibration.py`：源码入口，调用同一实现。
- `tests/controller_tracking/tools/world_calibration_smoke.py`：隔离域合成消息验收，包含中断测试，不接底盘。
- `tests/controller_tracking/tools/world_calibration_report.py`：读取已经冻结及验证的结果生成叠加图、XYZ分项图和汇总；不重新拟合、不按误差选择采集窗口。
- `tests/controller_tracking/test/test_world_calibration.py`：15 项新增软件测试。
- `docs/controller_tracking/FIXED_WORLD_CALIBRATION.md`：来源、流程、命令、文件结构与比较限制。
- `docs/controller_tracking/FIXED_WORLD_VALIDATION.md`：本记录。
- `docs/controller_tracking/fixed_world_checks.json`：机器可读验收与文件清单。

修改：

- `tests/controller_tracking/setup.py`：增加独立命令入口。
- `tests/controller_tracking/package.xml`：声明使用的现有 static_map_localization 消息依赖。
- `tests/controller_tracking/README.md`、`ARCHITECTURE.md`、`docs/controller_tracking/MAP_MOCAP_RECHECK.md`：增加新流程链接及与旧对齐方法的区别。

删除：无。正式定位、MPPI、底盘、轨迹规划源码与参数无本轮修改；没有广播标定变换，也没有自动启动实车运动。

生成目录：`tests/controller_tracking/.validation/fixed_world_calibration/` 为合成验收；既有 `.build/.install/.log` 为测试包构建产物。真实采集由操作人创建 `robot_data/experiments/fixed_world_calibration/<会话>/`，每次完整保存一个目录。
