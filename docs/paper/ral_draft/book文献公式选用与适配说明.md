# book文献公式选用与适配说明

整理日期：2026-10-21。目的：判断七篇原始论文中哪些数学表达适合当前巡检研究，明确来源、适配条件、实现状态及验证要求。本文件是研究笔记，不是已经完成的新方法章节；不修改现有正文、算法或实验结论。

本文统一采用\({}^{A}\mathbf T_B\)将B系坐标变换至A系。M、G、O、B、I、L、Q分别为动捕世界、固定地图、当前里程计世界、车体、IMU、LiDAR及动捕刚体坐标。平移单位为m，角度为rad；位置误差最终可换算为mm。

## 1. 先区分公式的三种身份

- **现有实现的数学表达**：可按实际代码写入方法，但不能因此宣称公式本身新颖。
- **原文公式或等价重写**：注明论文、节及原式编号；只有实现相应机制后，才能作为本项目方法使用。
- **针对本项目推导的候选公式**：明确写作“候选、尚未实现验证”；不能写成原论文原式或当前算法。

已读范围为七篇论文的方法、相关实验和评价段落。对FF-LINS第5页、LIO-GVM第4页的关键公式另核对了PDF图像。下文使用统一符号重写，原式编号仅用于定位来源，不代表本稿公式编号。

| 原始论文 | 最值得使用的公式或框架 | 当前建议 | 尚需的条件 |
|---|---|---|---|
| [FF-LINS](../../../book/FF-LINS_A_Consistent_Frame-to-Frame_Solid-State-LiDAR-Inertial_State_Estimator.pdf) | III-C式(6)的加权残差优化；式(7)–(8)的坐标链和点到平面残差；式(9)–(11)的雅可比推导方式 | 优先借鉴“状态—观测—残差—权重—求解”的建模结构 | 动捕观测需重新定义；不能直接使用点云残差的雅可比 |
| [LIO-GVM](../../../book/LIO-GVM_An_Accurate_Tightly-Coupled_Lidar-Inertial_Odometry_With_Gaussian_Voxel_Map.pdf) | IV-A式(6)的协方差传播；IV-C式(11)–(14)的观测、线性化和MAP；式(15)的加权距离 | 适合候选融合模型；高斯点云分布残差暂不采用 | 状态、噪声、相关性和参数均需独立建立 |
| [KISS-ICP](../../../book/RA-L.pdf) | III-D式(5)–(8)的旋转/平移引起的点位移界；III-E式(12)的鲁棒配准 | 位移界可用于变换更新诊断；配准算法仅作相关工作 | 同一坐标系、正确变换方向、距离上界；不能把界当作实测误差 |
| [Wheel-Legged SLAM](../../../book/Wheel-Legged_SLAM_Indoor_LiDAR-Inertial_SLAM_Integrating_Kinematic_Model_of_Wheel-Legged_Robots.pdf) | III-C式(9)高度残差、式(11)重力残差 | 借鉴“利用额外物理约束”的思路；目前不直接采用 | 对应高度测量、接触/地面条件及重力定义 |
| [LocNDF](../../../book/wiesmann2023ral-icra.pdf) | III-B式(11)的距离场配准目标；III-A式(6)–(10)训练损失 | 暂不进入当前方法，可作未来地图约束方向 | 可查询的距离场/网络、训练或建图过程及新增验证 |
| [CAVERS](../../../book/CAVERS_Multimodal_SLAM_Data_From_a_Natural_Karstic_Cave_With_Ground_Truth_Motion_Capture.pdf) | III-A的时间配对、刚体对齐与轨迹RMSE评价 | 可借鉴评价组织，与现有定位残差定义对照 | 明确是否对齐、真值覆盖、同步方式及参考独立性 |
| [Unveiling Non-Reproducibility](../../../book/Unveiling_Non-Reproducibility_in_LiDAR-Inertial_Odometry.pdf) | II节重复运行RMSE的极差；V-C的矩阵置换分析 | 极差适合补充重复运行指标；矩阵置换用于专项诊断 | 固定输入与运行条件；不能沿用其1 m门槛判定本项目毫米级性能 |

七篇论文均不是本文现有多边形外偏置或MPPI控制公式的直接来源。几何模块仍应对应Boost buffer及本项目实现，控制公式仍应对应现有控制代码和已核实MPPI近邻。

## 2. 当前可以写进正文的坐标公式

**身份：现有实现；不是从FF-LINS移植的新算法。**

同一车体参考点的两个来源分别为

\[
{}^{M}\mathbf T_B
={}^M\mathbf T_Q\,{}^Q\mathbf T_B,
\qquad
{}^O\mathbf T_B
={}^O\mathbf T_I\left({}^B\mathbf T_I\right)^{-1}.
\]

首次地图关系固定后，每次启动的关系满足

\[
{}^G\mathbf T_O
=\left({}^M\mathbf T_G\right)^{-1}{}^M\mathbf T_O,
\qquad
{}^G\widehat{\mathbf T}_B(t)
={}^G\mathbf T_O\,{}^O\mathbf T_B(t).
\]

每对静止位姿得到\({}^M\mathbf T_B({}^O\mathbf T_B)^{-1}\)，当前代码分别平均其旋转和平移。该计算与“最小化某个新设计的加权残差目标”不是同一个实现，不能只更换公式就改称MAP融合。

**实现依据：**[core.py](../../../tests/mocap_anchored_mapping/mocap_anchored_mapping/core.py)中的`native_base`、`mocap_base`、`estimate`及`initial_map_odom`；[checkpoints.py](../../../tests/mocap_anchored_mapping/mocap_anchored_mapping/checkpoints.py)中的`correction_report`。

**使用条件：**外参固定、参考点一致、配对有效、动捕世界身份未改变；FAST-LIO内部激光—IMU外参不能重复应用。上述关系定义坐标转换，不提供独立精度保证。

## 3. FF-LINS：哪些公式能借鉴，哪些不能直接复制

### 3.1 帧间坐标链与点到平面测量

**来源：**FF-LINS第III-C-2节，原式(7)–(8)，印刷页8529、PDF第5页。下式是固定外参条件下的紧凑等价表达，省略原文未在这些式中展开的时间延迟。

设\(\mathbf p^{L_j}\)为j帧LiDAR点，\({}^I\mathbf T_L\)为LiDAR到IMU的外参，则其在i帧LiDAR中的坐标为

\[
\widetilde{\mathbf p}^{L_i}
=({}^I\mathbf T_L)^{-1}
({}^G\mathbf T_{I_i})^{-1}
{}^G\mathbf T_{I_j}
{}^I\mathbf T_L\widetilde{\mathbf p}^{L_j}.
\]

波浪号表示齐次点。取结果的前三维，与i帧关键帧点云地图中拟合的平面构造

\[
r_{ij}^{\mathrm{plane}}
=\mathbf n_i^\mathsf T\mathbf p^{L_i}+d_i.
\]

\(\mathbf n_i\)是单位法向，\(d_i\)为有符号平面参数，残差单位为m。这里必须具备关键帧点云、关联平面和对应的优化状态。

**对你的用途：**学习如何让残差连接两个时刻及外参，解释相对观测约束。当前外部动捕纠偏接收的是完整位姿，没有该套帧间点云关联，不能把此式直接写作已实现的动捕测量模型。

### 3.2 加权残差优化框架

**来源：**FF-LINS第III-C-1节原式(6)。按残差类别抽象后，可写作

\[
\min_{\mathcal X}
\sum_{\ell\in\mathcal L}
\|\mathbf r^{\mathrm{lidar}}_\ell(\mathcal X)\|^2_{\boldsymbol\Sigma_\ell^{-1}}
+\sum_{j\in\mathcal I}
\|\mathbf r^{\mathrm{imu}}_j(\mathcal X)\|^2_{\boldsymbol\Sigma_j^{-1}}
+\|\mathbf r_{\mathrm{prior}}(\mathcal X)\|^2.
\]

其中\(\|\mathbf r\|^2_{\mathbf W}=\mathbf r^\mathsf T\mathbf W\mathbf r\)。\(\mathcal L\)、\(\mathcal I\)分别为激光与IMU约束集合，\(\mathcal X\)为窗口内状态与待估参数。各协方差应与残差坐标、维度和噪声定义匹配。

**可借鉴的内容：**多源约束按误差和不确定性共同决定更新；原文还采用Huber代价与后续离群检查。动捕因子需要另建；使用已融合IMU的LIO输出时，不应再把同源信息作为独立IMU约束重复计入。

**不能照搬的内容：**原式(9)–(11)的雅可比针对点到平面残差、指定姿态扰动及激光—IMU外参。换成动捕位姿或map→odom状态，必须重新求导。采用类似目标也不等于继承原文的一致性结论。

## 4. 最贴合当前系统的候选模型：平面锚定变换观测

**身份：根据本项目状态与坐标关系推导的候选设计；尚未实现、尚未实验验证。不是七篇论文中的原式，也不因具有解析雅可比而自动构成创新。**

相比一次性替换完整SE(3)，可先研究平面地图—里程计关系。适用前提是近似平面运动且两个参考坐标的重力方向已经对齐；该投影忽略的roll/pitch影响需要单独检查。

### 4.1 状态与观测函数

令待估平面变换参数为\(\mathbf a_k=[t_{x,k},t_{y,k},\theta_k]^\mathsf T\)，平移位于G系，\(\theta_k\)为O到G的平面旋转。LIO车体位姿为\((q_{x,k},q_{y,k},\psi_k^O)\)。动捕通过冻结的\({}^G\mathbf T_M\)换算到B参考点后，得到\(\mathbf z_k^m=[x_k^m,y_k^m,\psi_k^m]^\mathsf T\)。观测模型为

\[
\mathbf z_k^m=h(\mathbf a_k;\mathbf q_k,\psi_k^O)+\mathbf v_k,
\quad
h=
\begin{bmatrix}
\cos\theta_k q_{x,k}-\sin\theta_k q_{y,k}+t_{x,k}\\
\sin\theta_k q_{x,k}+\cos\theta_k q_{y,k}+t_{y,k}\\
\operatorname{wrap}(\theta_k+\psi_k^O)
\end{bmatrix}.
\]

该式中的测量噪声\(\mathbf v_k\)是在局部角度分支上的近似，前两维单位为m，第三维为rad。观测残差定义为预测减观测，角度分量单独环绕：

\[
\mathbf r_k^m=
\begin{bmatrix}
h_x-x_k^m\\h_y-y_k^m\\
\operatorname{wrap}(\theta_k+\psi_k^O-\psi_k^m)
\end{bmatrix}.
\]

噪声\(\mathbf v_k\)需要反映LIO输入、动捕、时间配对及转换的不确定性；若存在共同标定偏差，其影响不宜简单当作逐帧独立白噪声。当前数据不足以指定其数值协方差。

### 4.2 解析雅可比

在固定LIO输入、对\(\mathbf a_k\)采用加性扰动、且远离角度环绕分支边界的条件下，残差雅可比为

\[
\mathbf H_k=\frac{\partial\mathbf r_k^m}{\partial\mathbf a_k}=
\begin{bmatrix}
1&0&-\sin\theta_k q_{x,k}-\cos\theta_k q_{y,k}\\
0&1&\cos\theta_k q_{x,k}-\sin\theta_k q_{y,k}\\
0&0&1
\end{bmatrix}.
\]

第三列前两项的单位为m/rad。这是本候选变换参数化下的导数，不适用于另一种左/右乘李群扰动，也不能用于声称当前代码已经在线估计误差状态。

**本轮数学核对：**固定随机种子20261021，对100组远离角度分支边界的输入作中心有限差分，步长为各参数单位下的10⁻⁶；与解析矩阵各元素的最大绝对差为1.48×10⁻⁹以内。该检查只确认局部导数计算一致，不验证传感器噪声、统计一致性或融合收益。

### 4.3 带先验的加权估计

借鉴FF-LINS的残差优化和LIO-GVM的先验—观测加权框架，可以考虑

\[
\mathbf a_k^\star=\arg\min_{\mathbf a}
\|\mathbf a\ominus\mathbf a_k^-\|^2_{(\mathbf P_k^-)^{-1}}
+\rho\!\left(\|\mathbf r_k^m(\mathbf a)\|^2_{\mathbf R_k^{-1}}\right).
\]

这里\(\ominus\)表示平移相减并将角差环绕；\(\mathbf a_k^-\)、\(\mathbf P_k^-\)为更新前参数和协方差，\(\mathbf R_k\)为有效观测协方差，\(\rho\)为待选择的鲁棒损失。无鲁棒项的加权和仅在相应独立高斯近似下具有常规MAP解释；鲁棒化后应按选定损失解释。需另行定义跨时刻传播及相关性处理，不能默认随机游走或任何噪声数值已经成立。

**验证目标：**比较不更新、现有完整SE(3)替换及候选平面加权更新在当前站位和远端位置的残差。软权重、平面约束或局部窗口都不自动阻止后续过补偿。

## 5. LIO-GVM：观测线性化与协方差可借鉴，分布匹配不能直接挪用

### 5.1 观测与局部线性化

**来源：**LIO-GVM第IV-C-2节原式(11)–(14)，印刷页2203、PDF第4页。

采用本文件的一般符号，局部观测可表达为

\[
\mathbf z_k=h(\widehat{\mathbf x}_k\boxplus\delta\mathbf x_k)+\mathbf v_k,
\qquad
h(\widehat{\mathbf x}_k\boxplus\delta\mathbf x_k)
\approx h(\widehat{\mathbf x}_k)+\mathbf H_k\delta\mathbf x_k.
\]

\(\boxplus\)的具体定义取决于状态空间；它不能与普通加法任意互换。第4节选择了平面变换的局部加性参数化，给出了对应的h与H。不要将LIO-GVM的18维误差状态与本候选3维状态混用。

### 5.2 协方差传播

**来源：**LIO-GVM第IV-A节原式(6)，印刷页2202、PDF第3页。统一符号重写为

\[
\mathbf P_{k+1}^{-}
=\mathbf F_k\mathbf P_k^{+}\mathbf F_k^\mathsf T
+\mathbf G_k\mathbf Q_k\mathbf G_k^\mathsf T.
\]

\(\mathbf F_k\)为状态传播雅可比，\(\mathbf G_k\)为噪声映射，\(\mathbf Q_k\)为过程噪声协方差。原文针对LIO内部状态；用于外部锚定参数时必须另建传播模型与噪声，不能直接沿用原矩阵或把FAST-LIO内部协方差当作map→odom协方差。

### 5.3 分布相似性只适合对应的数据对象

原式(10)对两个高斯点云局部分布的协方差定义相似度。统一符号后为

\[
s_{ij}=\sqrt{\frac{(\det\mathbf C_i\det\mathbf C_j)^{1/2}}
{\det[(\mathbf C_i+\mathbf C_j)/2]}}.
\]

式(15)再将该相似度用于加权分布间距离。这需要非奇异或经过合理正则化的点云分布协方差，以及实际对应关系。

**当前不建议：**将“动捕轨迹散布”和“LIO轨迹散布”直接代入上述s作为观测可信度。它们与原文的局部表面分布不是同一统计对象，相同散布也不能排除较大的位置偏置。

## 6. KISS-ICP：可以用于纠偏影响分析的位移界

**来源：**KISS-ICP第III-D节原式(5)–(8)，PDF第4页。

若\(\Delta\mathbf T=[\Delta\mathbf R,\Delta\mathbf t]\)，旋转角\(\vartheta\in[0,\pi]\)，且\(\|\mathbf p\|\le r_{\max}\)，则

\[
\|\Delta\mathbf R\mathbf p+\Delta\mathbf t-\mathbf p\|
\le\|\Delta\mathbf t\|+2r_{\max}\sin(\vartheta/2),
\quad
\vartheta=\arccos\!\frac{\operatorname{tr}(\Delta\mathbf R)-1}{2}.
\]

原文用于运动预测偏差引起的点位移上界，并据此构造数据关联阈值。\(r_{\max}\)与平移均为m，旋转角为rad，数值实现需将arccos输入截断到[-1,1]。

**适配本项目的离线诊断：**记\(\mathbf H_0\)、\(\mathbf H_1\)均将O变换到G，令\(\Delta\mathbf H=\mathbf H_1\mathbf H_0^{-1}\)，则\(\mathbf p_0^G=\operatorname{trans}(\mathbf H_0{}^O\mathbf T_B)\)与\(\mathbf p_1^G\)满足同类位移关系。距离界必须取相对于G原点的有效半径，变换与点不能混用坐标系。

更直接的诊断是计算同批位姿的实际差\(\mathbf p_1^G-\mathbf p_0^G\)，并分别考察平移、旋转影响。上界是总位移的保守界，不是XY误差的实测值；不能用“总转角×距离”唯一解释本项目终点误差。原文的0.1 m阈值及3σ匹配规则也不能直接作为动捕更新门槛。

## 7. Wheel-Legged SLAM：适合学习约束设计，暂不直接采用

### 7.1 高度约束

**来源：**第III-C-2节原式(9)，印刷页1276、PDF第4页。按原文记号为

\[
r_z=z_k-z_0+h_0-h_k.
\]

\(z_k,z_0\)为世界系车体高度及初始化值，\(h_k,h_0\)为运动学推得的离地高度及初始化值，单位均为m。原文在轮式模式、地面坡度合格时使用该约束。

**对你的启发：**只有存在可验证的车体高度或地面信息，才能把相应物理关系加入估计。当前没有本文轮腿运动学所需的腿部测量，不应使用其h公式。固定轮式底盘也不能未经检查就设置\(z=0\)；地图原点、安装高度、坡度及地面起伏均会影响含义。

### 7.2 重力相关约束

**来源：**第III-C-3节原式(11)，同页。原式写为

\[
\mathbf r_g=\mathbf R_k^\mathsf T\widehat{\mathbf g}_k
-\mathbf R_{k-1}^\mathsf T\widehat{\mathbf g}_{k-1}.
\]

原文根据其R与重力向量的坐标定义推导雅可比。若适配，必须先核对R的方向，保证两项位于同一参考系；这里保留原式，不将其R直接等同于本文任意\({}^A\mathbf R_B\)。重力约束主要涉及倾斜方向，单独不能观测绕重力轴的航向。原始加速度计读数含比力与运动影响，也不能直接冒充已估计重力。

**当前建议：**先检查实际roll/pitch及外参误差是否影响平面定位，再决定是否需要此类额外约束。现有外部纠偏并未实现该因子。

## 8. LocNDF：只有引入距离场后才适用

**来源：**第III-B节原式(11)，PDF第4页。

\[
(\mathbf R^\star,\mathbf t^\star)
=\arg\min_{\mathbf R,\mathbf t}
\sum_{\mathbf p_i\in\mathcal P}
D(\mathbf R\mathbf p_i+\mathbf t)^2.
\]

D为地图距离场，输入为地图坐标中的点，输出为到表面的距离，\(\mathcal P\)为待配准点云。原文由网络学习D并利用其导数迭代配准。

该目标可以帮助理解“把地图几何转换成位姿约束”，但你当前没有LocNDF网络、训练损失与距离场配准。第III-A节原式(6)–(10)的距离、表面、Eikonal与法向正则损失均不宜写入当前方法。

原式(12)的紧凑雅可比依赖点、梯度和姿态扰动约定，不能直接拿来对动捕或折线投影求导。若未来采用栅格距离变换，应另行描述分辨率、插值、未知区和梯度，它也不等于实现LocNDF。

## 9. CAVERS与不可复现性论文：目前最容易规范使用的是评价公式

### 9.1 定位残差与RMSE

**来源定位：**CAVERS第III-A节（印刷页11518，PDF第5页）以文字定义平移/旋转ATE与RMSE，没有对应的独立公式编号。下式是按本项目坐标及平面评价需要重写的定义，不标作CAVERS原式。

\[
\mathbf p_k^{G,m}=\operatorname{trans}({}^G\mathbf T_M{}^M\mathbf T_B(t_k)),
\quad
e_{xy,k}=\|\mathbf P_{xy}(\widehat{\mathbf p}_k^G-\mathbf p_k^{G,m})\|,
\quad
\mathrm{RMSE}_{xy}=\sqrt{\frac1N\sum_{k\in\mathcal V}e_{xy,k}^2}.
\]

\(\mathcal V\)是有效配对集合，\(N=|\mathcal V|\)，\(\mathbf P_{xy}\)提取平面分量。三维误差去掉投影。姿态残差可采用\(\|\operatorname{Log}[(\mathbf R_k^{G,m})^\mathsf T\widehat{\mathbf R}_k^G]^\vee\|\)，输出为rad；转为度需乘180/π。这是整体旋转误差，不是单独yaw误差。

**现有状态：**位置及整体姿态残差对应`validation_errors`；当前停点yaw另由预测与参考yaw差环绕计算。

**配套口径：**同一B参考点；G坐标；按各试次规则配对，动态锚定记录最大25 ms；接收时间戳并非已标定曝光时刻；有效区间按记录固定，排除拟合窗后另报留出窗；95对样本来自三个位置和一次重启。起始对齐后结果称相对残差，同源动捕初始化/纠偏后的结果不称独立绝对精度。

### 9.2 跨轮统计与重复运行变异

**来源定位：**不可复现性论文第II节（印刷页995，PDF第2页）以RMSE极差定义精度变异，未单独编号。设每次完整运行的指标为\(m_r\)，可报告

\[
\bar m=\frac1R\sum_{r=1}^R m_r,
\qquad
s_m=\sqrt{\frac{\sum_r(m_r-\bar m)^2}{R-1}},
\qquad
\Delta_m=\max_r m_r-\min_r m_r.
\]

R为完整运行次数，至少两次才计算样本标准差。固定同一数据和软件条件的回放，可评价实现的重复运行变异；多次实车行驶还包含运动与环境变化，不能据此全部归因于非确定性实现。高频帧数不作为R。

原文的“显著非复现”1 m判据属于其数据集和问题设定，不适合作为本项目毫米级精度阈值。原文通常30次重复是其设计，不能改写成本项目已经完成30次。

第V-C节的\(\mathbf H_{\rm perm}=\mathbf P\mathbf H\)、\(\mathbf H_{\rm perm}^\mathsf T\mathbf H_{\rm perm}=\mathbf H^\mathsf T\mathbf H\)针对置换矩阵和精确算术，用于研究浮点加法顺序；这里的H是原文测量矩阵，与本文件的H₀/H₁变换及Hₖ雅可比无关。除非开展实现复现性专项研究，不需放入你的方法。

## 10. 几何与控制：用已有实现公式，不从不相关论文补公式

当前几何模块采用\(\operatorname{Buffer}(P,d)\)，再选最大面积外环、逐边采样和切线航向。这应引用实际几何库并对照[polygon_offset_node.cpp](../../../src/inspection_geometry_tools/src/polygon_offset_node.cpp)，不是七篇文献的新几何算法。

路径误差应对参考线的有限线段计算。全图[tracking_report.py](../../../tests/full_map_loop/tracking_report.py)使用全部线段最近投影和几何法向；控制仿真使用带进度提示的局部搜索及存储航向法向。两者的计算口径应分别保留。地图估计位姿得到的是地图轨迹偏差，覆盖外没有独立真值时不能据此报告真实跟踪精度。

当前控制的曲率预瞄、命令队列、一阶响应、残差代价和加权回退公式已在正文及[控制方法](../../../tests/turning_mppi/paper/METHODS.md)中对应实现。它们的原始引用仍为MPPI、RMPPI、Smooth MPPI、SVG-MPPI及相关延迟方法；七篇定位文献不能替代其理论来源。

## 11. 建议如何进入下一版论文

现阶段正文保留现有坐标链、实际锚定计算、真实几何/控制公式与评价指标。若下一步研究局部测量模型，优先实现并验证第4节的“观测函数—残差—雅可比—加权估计”，将其与当前替换法比较；不要先把全部FF-LINS、LIO-GVM公式填入正文。

**待核实信息：**独立外参、时间偏移、G/O重力对齐、动捕与LIO噪声及共同标定相关性、候选状态传播、更新后的发布方式、远端作用及重复试验。当前静止窗口不能可靠辨识时间延迟；FF-LINS也未在原式(7)–(12)中显式展开时间延迟雅可比，不能从这些式直接宣称已完成在线时间标定。

**可能受到的审稿质疑：**位姿因子及加权最小二乘本身已有研究基础，新增机制是什么；为什么观测可信度模型合理；共同标定和LIO相关性如何处理；局部权重更新能否通过远端验证；解析雅可比是否经过数值核对；平面约束是否掩盖三维姿态或外参误差；评价是否复用了拟合参考。
