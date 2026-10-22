# 本次真实首次建图

真实锚定和人工遥控建图已完成；用户已报告停稳，原生 PCD 已通过真实 FAST-LIO 保存服务写盘。当前未请求停止建图进程。
没有加载旧 PCD，没有启动 NDT，没有调用历史标定后的重启定位入口，没有自动发运动指令。

已采集并冻结：32 对、3.099941969 秒；原始/归一化动捕证据 280/280、100%。
map 原点在 mocap_world 的 XYZ 为 [-0.3174691389, 0.0918206021, 0.3587069107] m。
初始化窗口位置一致性 RMSE 为 0.0021740788 m，仅是初始窗口残差，不是全程/独立精度认证。

calibration.yaml、initialization/raw.jsonl、initialization/pairs.json 已真实写盘。
raw/native_F.pcd 已保存：1,530,037 点，48,961,437 字节，保持 map=native_F0，坐标变换次数为 0。
SHA256: fc530f6f108e52475d37774404c93954cb400080a7bfe2d48c4235dd28b66e51
固定标定 SHA 与初始化记录一致。grid/map.yaml 与 map.pgm 已由用户运行原生投影流程生成并核查通过：0.05m/像素，557×593，栅格左下角 origin=[-16.65,-16.95,0]。此 origin 是图像栅格在 map 中的位置，不是 map 世界原点。
PCD/标定绑定、PCD/YAML/PGM 哈希及图像尺寸检查通过；没有对 PCD 再做坐标变换。
本轮不要重复运行首次 mapping/save 命令覆盖；后续若重新启动需使用明确的重启流程。

注意：config/calibration 中 data_kind 的原始配置标签包含 pending_collection，是准备时冻结的标签，未重写冻结输入；实际采集状态以 real_session_status.json、原始数据和本记录为准。

## 三个静止位置实测已完成

数据与报告：validation_sessions/20261017T085200Z_three_static_points/。三个不同位置共 95 对有效同步样本；XY RMSE=7.7848mm，3D RMSE=16.3170mm，航向 RMSE=0.029115deg。初始化与原位复测不计入统计，3 次失败尝试完整保留。

因首次建图实例已退出，本轮单独启动动捕初始化后的 FAST-LIO；原世界标定不变，本次 map→odom 初始化后冻结。结果用于验证本次重启后的静止定位误差，不能独立验证世界标定或 PCD 全局重定位精度。没有重新拟合，旧地图/标定和当前新地图哈希保持不变。新定位进程 PID 30938 保留运行。
