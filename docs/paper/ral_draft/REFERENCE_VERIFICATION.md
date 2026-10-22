# 文献核实记录

核实日期：2026年10月19日（用户时区：Asia/Shanghai）。

本表记录本轮实际查阅的原始论文和官方文档，供修改初稿时追溯引用。论文的标题、作者顺序、arXiv编号、初次提交日期及所选版本已从arXiv原始记录核对；相关方法部分已按下表查阅。本轮没有逐行检查所有证明、复现实验或完成穷尽性新颖性检索。下表中的联系与差异是依据已查阅内容作出的技术判断，不是文献原文结论。

2026-10-19建立的九篇论文条目使用所查阅的arXiv版本，`year`表示arXiv首发年份。2026-10-21新增的七篇`book/`文献按所读PDF分别记录正式期刊版或接收稿。预印本条目不等同于未曾正式发表；正式投稿前更换出版条目时，应核对完整元数据，不能拼接预印本年份与正式卷页。

| BibTeX键 | 已核实的原始记录与版本 | 本轮查阅范围及可支撑的表述 | 使用边界与待办 |
|---|---|---|---|
| `fastlio2` | Wei Xu, Yixi Cai, Dongjiao He, Jiarong Lin, Fu Zhang；[FAST-LIO2](https://arxiv.org/abs/2107.06829v1)，首发2021-07-14，v1 | [原文](https://arxiv.org/html/2107.06829v1)摘要、引言和系统/估计章节相关内容；支持紧耦合迭代卡尔曼滤波、原始点直接配准、增量地图结构的描述 | 该论文不能替代本项目FAST-LIO实际版本、配置及改动的说明；正式期刊卷页本轮未写入 |
| `liosam` | Tixiao Shan, Brendan Englot, Drew Meyers, Wei Wang, Carlo Ratti, Daniela Rus；[LIO-SAM](https://arxiv.org/abs/2007.00258v3)，首发2020-07-01，v3为2020-07-14 | [原文](https://arxiv.org/html/2007.00258v3)第III节，尤其III-D；支持因子图接入绝对观测、按雷达时间插值GPS、按协方差条件添加GPS因子的表述 | 原始记录注明IROS 2020；本文外部坐标变换更新与其联合状态优化不同，不应混称相同融合方法 |
| `mppi` | Grady Williams, Paul Drews, Brian Goldfain, James M. Rehg, Evangelos A. Theodorou；[Information Theoretic Model Predictive Control: Theory and Applications to Autonomous Driving](https://arxiv.org/abs/1707.02342v1)，首发2017-07-07，v1 | [原文](https://arxiv.org/html/1707.02342v1)信息论控制推导及Algorithm 1相关内容；支持采样滚动预测和按轨迹代价加权更新的基础介绍 | 记录写的是投稿T-RO，不能仅据此填正式出版书目信息。有限样本、约束、引导和残差更新后的本项目控制器不能直接继承原文最优性解释 |
| `rmppi` | Manan Gandhi, Bogdan Vlahov, Jason Gibson, Grady Williams, Evangelos A. Theodorou；[Robust Model Predictive Path Integral Control: Analysis and Performance Guarantees](https://arxiv.org/abs/2102.09027v1)，首发2021-02-17，v1 | [原文](https://arxiv.org/html/2102.09027v1)第III节的增广状态、反馈预测、重要性采样；其性能界具有明确的控制器与系统假设 | 原始记录关联[RA-L DOI](https://doi.org/10.1109/LRA.2021.3057563)。本项目不能把“跟踪反馈进入预测/采样”单独作为新增机制，也不能继承其证明 |
| `smoothmppi` | Taekyung Kim, Gyuhyun Park, Kiho Kwak, Jihwan Bae, Wonsuk Lee；[Smooth Model Predictive Path Integral Control without Smoothing](https://arxiv.org/abs/2112.09988v8)，首发2021-12-18，所读v8为2025-10-13 | [原文](https://arxiv.org/html/2112.09988v8)第III节；支持输入提升、导数动作采样/积分及动作变化代价的描述 | 原始记录注明RA-L/IROS 2022并关联[正式DOI](https://doi.org/10.1109/LRA.2022.3192800)；预印本首发2021与正式发表2022需区分。当前引用明确为2025修订的v8，并未假定它与2022出版版逐字一致 |
| `edgecoverage` | Zhaofeng Tian, Weisong Shi；[Edge Coverage Path Planning for Robot Mowing](https://arxiv.org/abs/2209.05405v1)，首发2022-09-12，v1 | [原文](https://arxiv.org/html/2209.05405v1)第III–IV节；支持边缘覆盖目标、机器人几何约束、大/小圆盘与滑动杆件方法的介绍 | 边界巡检路径与具有割草工作区域定义的边缘覆盖目标不同。统一距离buffer不等同于其与机器人形状相关的机制；本轮未确认正式出版条目 |
| `boustrophedon` | Rik Bähnemann, Nicholas Lawrance, Jen Jen Chung, Michael Pantic, Roland Siegwart, Juan Nieto；[Revisiting Boustrophedon Coverage Path Planning as a Generalized Traveling Salesman Problem](https://arxiv.org/abs/1907.09224v1)，首发2019-07-22，v1 | [原文](https://arxiv.org/html/1907.09224v1)相关工作和第3节几何路径生成；支持多边形带孔区域分解、各单元多扫掠方案、访问连接优化的描述 | 原始记录注明FSR 2019并关联[Springer章节DOI](https://doi.org/10.1007/978-981-15-9460-1_20)。会议举办年份不自动等于论文集出版年份；当前未填正式书章年份/页码。本文若无区域连接与排序实现，不应宣称同层次多区域优化 |
| `rep105` | Wim Meeussen；[REP 105: Coordinate Frames for Mobile Platforms官方源文件](https://raw.githubusercontent.com/ros-infrastructure/rep/master/rep-0105.rst)，创建2010-10-27，状态Active | 已读`base_link`、`odom`、`map`的定义；支持连续局部里程计参考与允许跳变的全局参考之间的结构区分 | [ROS展示页](https://www.ros.org/reps/rep-0105.html)本轮返回访问拦截，故以ROS官方仓库源文件核实。该规范不能证明本项目定位精度或控制稳定性 |
| `boostbuffer` | [Boost.Geometry官方buffer文档](https://www.boost.org/doc/libs/1_85_0/libs/geometry/doc/html/geometry/reference/algorithms/buffer/buffer_7_with_strategies.html)，固定版本1.85.0；页面注明该版本发布于2024年 | 已读描述、参数、策略和示例；支持距离缓冲、圆角/尖角连接、正负偏置属于现成几何运算的表述 | 1.85.0是此次查阅文档版本，尚不能当作机器人实装依赖版本；需从构建环境核实。标准buffer不能单独保证机器人足迹无碰撞、路径动力学可行或完整覆盖 |
| `svgmppi` | Kohei Honda, Naoki Akai, Kosuke Suzuki, Mizuho Aoki, Hirotaka Hosogaya, Hiroyuki Okuda, Tatsuya Suzuki；[Stein Variational Guided Model Predictive Path Integral Control: Proposal and Experiments with Fast Maneuvering Vehicles](https://arxiv.org/abs/2309.11040v3)，首发2023-09-20，v3为2024-03-01 | [原文](https://arxiv.org/html/2309.11040v3)第III–IV节；支持针对多模态控制分布，用修改的SVGD估计目标模态，并以名义序列和协方差引导MPPI的介绍 | 本项目路径跟踪器产生的引导控制序列与SVGD模态搜索不同；“guided MPPI”不是可独占的新颖性表述。本轮未确认正式出版条目 |
| `clismppi` | Zidong Liu, Xu Chen；[CLIS-MPPI: Closed-Loop Model Predictive Path Integral Control With Initial-State Sampling Under Execution Delay](https://arxiv.org/abs/2610.21239v1)，首发2026-10-15，v1 | [原文](https://arxiv.org/html/2610.21239v1)第II节及第III节实验设置；支持延迟引起的执行时刻状态失配、初始状态/控制联合采样和基于rollout局部反馈修正的介绍 | 已核实其存在，应纳入最新近邻比较。原始记录的评论注明CDC 2026，本轮没有核实正式会议录/出版记录，故明确按预印本引用；“考虑延迟”或“闭环MPPI”本身不足以成为本文新颖性 |

## 2026-10-21：所提供book文献的核实与引用功能

来源为用户提供的原始论文PDF。本轮核对摘要、引言、方法相关段落、实验设计与结果、讨论及结论；没有复现实验或逐项验证证明。五篇正式PDF的作者、题名、卷期、首末页及DOI从首页与页眉读取，未以网络检索结果补填。KISS-ICP和LocNDF是带RA-L接收信息的作者稿，正文及BibTeX明确采用所读版本；接收年份不当作正式出版年份。

| BibTeX键及所读文件 | 方法内容核实范围 | 本稿中的引用作用与边界 |
|---|---|---|
| `kissicp`：[RA-L.pdf](../../../book/RA-L.pdf)，接收2022-12-27 | 第I、III、IV节：运动补偿、点云采样、自适应阈值、鲁棒点到点ICP；跨数据集与组件消融 | 说明配准流程已有基础；借鉴集中主张和跨条件验证。不是当前项目NDT表现的原因证据；正式卷期DOI待核实 |
| `liogvm`：[LIO-GVM](../../../book/LIO-GVM_An_Accurate_Tightly-Coupled_Lidar-Inertial_Odometry_With_Gaussian_Voxel_Map.pdf)，9(3):2200–2207，2024；DOI 10.1109/LRA.2024.3354616 | 第II、IV-C、IV-D及V节：协方差相似性筛选与残差加权、高斯体素维护、精度/存储/耗时及消融 | 与修改匹配残差和地图表示的研究区别；不把文中的复杂度或精度移作本项目结论 |
| `locndf`：[LocNDF](../../../book/wiesmann2023ral-icra.pdf)，接收2023-06-14 | 第I、III、IV节：由传感器观测学习距离场、扫描配准、MCL、损失与网络消融、离线训练限制 | 对比连续地图表示与外部坐标管理；借鉴主张—实验对应。未核实最终卷期DOI |
| `wheelslam`：[Wheel-Legged SLAM](../../../book/Wheel-Legged_SLAM_Indoor_LiDAR-Inertial_SLAM_Integrating_Kinematic_Model_of_Wheel-Legged_Robots.pdf)，10(2):1273–1280，2025；DOI 10.1109/LRA.2024.3521184 | 第III、IV节：双雷达、运动学高度、垂直修正及重力重估因子；两因子消融 | 说明平台信息进入内部估计的做法，与本文外部变换不同。其激光测距/尺测与本文动捕评价范围不同，不能移用精度结论 |
| `fflins`：[FF-LINS](../../../book/FF-LINS_A_Consistent_Frame-to-Frame_Solid-State-LiDAR-Inertial_State_Estimator.pdf)，8(12):8525–8532，2023；DOI 10.1109/LRA.2023.3329625 | 第I–IV节：作者对一致性的讨论、帧间关联、相对因子、外参与时延在线标定及消融 | 用于说明估计器内部一致性与相对测量机制。本文未做协方差一致性验证，不据此认证本项目一致性或将固定历史地图NDT等同于作者讨论的自建地图情形 |
| `cavers`：[CAVERS](../../../book/CAVERS_Multimodal_SLAM_Data_From_a_Natural_Karstic_Cave_With_Ground_Truth_Motion_Capture.pdf)，11(10):11514–11521，2026；DOI 10.1109/LRA.2026.3726391 | 第II、III及局限段：动捕覆盖、标定指标、无硬件同步、轨迹比较；部分无动捕序列用RTAB-Map LiDAR-I为参考 | 借鉴参考来源、时序和覆盖披露。其标定Mean Ray/Wand Error不等于本项目独立位姿不确定度；另一算法参考不是独立真值 |
| `lioreproducibility`：[Unveiling Non-Reproducibility](../../../book/Unveiling_Non-Reproducibility_in_LiDAR-Inertial_Odometry.pdf)，11(1):994–1001，2026；DOI 10.1109/LRA.2025.3636030 | 第II–V节：复现性定义、五类非确定性实现、逐项干预、30次重复及矩阵加法顺序测试 | 支持记录实现/运行条件的必要性；不能证明本项目差异或控制插件变异由其中任何机制引起。2025在线发表与2026卷期年份分别保留 |

## 对当前论文论断的约束

1. **定位部分：** 原始坐标变换、标定、地图锚定和标准`map→odom`结构应作为已有工具交代。新增贡献必须来自经实现核实的观测使用、更新接受/限制或纠偏机制，且由统一条件实验验证；现阶段不能从文献对比反推本项目已具备这些机制。
2. **规划部分：** 可调边距、几何外偏置与圆角本身不是新增机制。多区域、覆盖率和足迹可行性需要对应算法及指标，不能用已有文献证明本项目尚未实现的功能。
3. **控制部分：** 反馈进入rollout、引导采样、平滑动作和延迟处理均有近邻工作。需具体比较引导序列如何产生、搜索变量是什么、执行链状态如何重建、是否修正采样权重，以及相同算力/参数预算下的实测收益。
4. **结果部分：** 文献中的性能数字、理论条件或证明不能作为本项目实验结果；在本项目不满足相应假设时，不应移用稳定性、无偏估计或最优性结论。

## 投稿前补齐

- 从出版社或作者正式提供的书目信息核对期刊/会议版；逐篇选择“正式出版版”或“明确标注的预印本”，避免混拼字段。
- 核实机器人中FAST-LIO分支、Boost版本和实际使用的MPPI基线来源，再将对应版本与论文引用绑定。
- 对最终保留的贡献做针对性近邻检索；本表仅覆盖此次初稿选用文献，不构成“首次”或“全面领先”的依据。
- 当前没有添加任何未经原始来源核实的页码或期刊DOI；arXiv记录上的相关正式DOI仅列作可追溯入口，并不表示本轮已核对完整出版社元数据。
