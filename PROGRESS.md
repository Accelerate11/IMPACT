# IMPACT 执行进度

更新时间：2026-10-20

| 阶段 | 状态 | 证据/说明 |
|---|---|---|
| P0 仓库审计 | PASS | `docs/CURRENT_REPO_AUDIT.md` |
| P0 symlink 构建 | PASS | WSL `impact_p0_*`；5 包成功，8/8 C++ 测试通过 |
| P1 接口调试 | PASS | `runs/p1_20260822T124702Z_387`；闭环 70.829 s，rosbag 10,103 条 |
| P1 600 s 正式 Gate | PASS | `runs/p1_20260822T125115Z_388`；summary PASS，rosbag 38,231 条 |
| P2 Mid-360 契约 | PASS | `experiments/results/sensor_validation/p2_20260822T131950Z_1210`；600.435 s，11/11 Gate |
| P3 FAST-LIO2 | PASS | structured room 与 long corridor 正式 Gate、rosbag、真值隔离均通过 |
| P4 External Navigation | PASS | `docs/P4_EXTERNAL_NAV_VALIDATION_REPORT.md`；GPS-off 矩形闭环、ATE 5.09 mm |
| P5 Baseline Map + Frontier + EGO | PASS | `docs/P5_BASELINE_VALIDATION_REPORT.md`；structured_room 自动探索、0 碰撞 |
| P6 Directional Integrity Predictor | PASS | `docs/P6_DIRECTIONAL_INTEGRITY_REPORT.md`；真实 FAST-LIO 点面信息矩阵、方向 PL |
| P7 Protection Level Calibration | PASS | `docs/P7_PROTECTION_LEVEL_CALIBRATION_REPORT.md`；训练/测试隔离、95% coverage Gate |
| P8 Alert Limit | PASS | `docs/P8_ALERT_LIMIT_REPORT.md`；逐轨迹净空与静态障碍 AL Gate |
| P9 Integrity Margin | PASS | `docs/P9_INTEGRITY_MARGIN_REPORT.md`；同协方差宽/窄场景硬认证 Gate |
| P10 Minimum-Excitation Active Perception | PASS | `docs/P10_MINIMUM_EXCITATION_ACCEPTANCE_REPORT.md`；三组 Gazebo+FAST-LIO 飞行、可视化与隔离 Gate |
| P11 Integrity-Constrained Exploration | PASS | `docs/P11_INTEGRITY_CONSTRAINED_EXPLORATION_REPORT.md`；ROS 合同、双臂 Gazebo、RViz/Gazebo 可视化与隔离 Gate |
| P12 Dynamic Obstacle / Dynamic Map | PASS | `docs/P12_DYNAMIC_OBSTACLE_ACCEPTANCE_REPORT.md`；LiDAR-only 动态体素、制动重规划、TTL 重开、全走廊 Gazebo/RViz 与隔离 Gate |
| P13 Latency-Aware Safety | PASS | `docs/P13_LATENCY_AWARE_SAFETY_ACCEPTANCE_REPORT.md`；50/200 ms 双轮次、p99→AL→速度闭环、P12 保持性与 Gazebo/RViz Gate |
| P14 Fault Injection & Resilient Autonomy | PASS | `docs/P14_FAULT_INJECTION_ACCEPTANCE_REPORT.md`；十类故障矩阵、持续 LiDAR 失效安全、P12/P13 保持性与 Gazebo/RViz Gate |
| P15 Map-Derived Integrity Planning | PASS | `docs/P15_RESEARCH_GRADE_ACCEPTANCE_REPORT.md`；在线三维 lattice、map-derived 指标、独立真值完整性评价与只改变硬门的配对 Gate |
| 复杂场景完整正常飞行展示 | PASS | `docs/COMPLEX_SCENE_FULL_AUTONOMY_DEMO_REPORT.md`；无故障注入、24 m 双航程、Gazebo+RViz、GPU 与隔离 Gate |
| 三维复杂场景完整算法 | PASS | `docs/COMPLEX_3D_FULL_AUTONOMY_REPORT.md`；right→left→up→direct、真实爬升、24 m 双航程、无故障注入 |
| 复杂组合三维完整算法 | PASS | `docs/COMPLEX_COMPOSITIONAL_3D_ACCEPTANCE_REPORT.md`；right→left→up_right→direct、动态障碍、P13 p99、GPU Gazebo+RViz |
| R0 独立研究基线复验 | PASS | `evidence/RESEARCH_R0/r0_20261018T154037Z_313/summary.json`；14 包新建、140 Python、8 桥接测试、GPS-off P4/AP 动力学闭环 |
| R1 研究链 AP 执行接口 | PASS（短航段 SIL） | `evidence/RESEARCH_R1/r1_suite_20261018T162249Z_408/summary.json`；195 项测试、正常/旧指令重放分别 28/33 项检查；Gazebo GPU 退出清理仍有独立环境问题 |
| R2A 多源仿真采集契约 | PASS（旁路 SIL） | `evidence/RESEARCH_R2/r2_suite_20261018T165613Z_320/summary.json`；250 Python、18 C++ 检查，AP/多源/存储 28/45/29 项检查；硬件及多源状态融合仍未就绪 |
| R2B1 光流接收与共享尺度软件基础 | PASS（软件/合成） | `evidence/RESEARCH_R2B1/r2b1_suite_20261019T102246Z/summary.json`；371 测试、12 项 PTY、22 项合成检查；未接真实硬件或共同状态融合 |
| R2B2 共同状态 LIVO 回放 | PASS（离线仿真） | `evidence/RESEARCH_R2B2/r2b2_suite_20261019T141249Z/summary.json`；408 测试、424 个完整配对时刻、422 次真实视觉校正；光流/硬件/新 AP 飞行未验证 |
| R2B3A 光流几何与相关更新前置验证 | PASS（原图回放/合成） | `evidence/RESEARCH_R2B3A/r2b3a_suite_20261020T090255Z/summary.json`；528 测试、43,843 条原图轨迹、360,000 次合成试验；实际光流状态更新为 0 |
| 论文候选成果保全 | PASS（字节/声明边界） | `evidence/PAPER_PRESERVATION/paper_baseline_20261020T092036Z/claim-ledger.json`；548 份源快照、1,979 份旧证据 SHA、6 组有界论文候选表述；不等于原创或硬件已证明 |
| R2B3B0 多窗联合状态与两视图几何 | PASS（线性合成/条件几何） | `evidence/RESEARCH_R2B3B0/r2b3b0_suite_20261020T094534Z/summary.json`；632 测试、36,000 独立轨迹/144,000 相关窗口、72 次批量条件解核对；真实光流融合仍待完成 |
| R2B3B1 测距与噪声采集 | 部分完成；最新 AP 起飞 FAIL | 711 Python、93 桥接检查；一段4,188同戳测距观测独立59项复核通过；第3正式轮FCU里程计接收中断，阶段不能记PASS，所有失败保留 |

## 2026-10-20：论文成果保全及 R2B3B0 软件验收

- 按用户授权，先建立独立论文保全基线
  `evidence/PAPER_PRESERVATION/paper_baseline_20261020T092036Z/`，封存本次开始时的
  548 份源码/配置/文档（约 4.5 MB），对已有 1,979 份证据（42,618,537 字节）建立
  全文件 SHA 索引。保全共 573 文件，manifest SHA 为
  `94027796ddf56e124d4502e3710ae8581eb802f6415ddd737b36c6a7d1414582`。
  6 组 claim ledger 分别保留 P10、P13、P15、R1、R2B2、R2B3A 的指标、原 JSON 指针、
  来源 SHA、可写表述及不可扩大的结论。13 份小引用副本支持 16 个来源引用；旧数据
  保持原位置，大 bag 及构建树未复制，也未声称已重新读取所有历史外置数据。
- 当前工作树快照不自动等于所有历史实验的原始源码；历史实验自身的源/配置/构建
  绑定继续保留。源文件可以继续迭代，保全复验检查冻结副本；旧证据不能被改写。
  P15 单场景收益、旧经验覆盖、短 AP SIL、单 bag 视觉融合和合成数学结论保持各自
  边界，不作为已证明原创性、泛化、整任务风险保证或硬件有效性的证据。
- 新阶段为 `R2B3B0_SOFTWARE_PASS`，归档于
  `evidence/RESEARCH_R2B3B0/r2b3b0_suite_20261020T094534Z/`。新增 `joint_state.py`：
  19 个线性 active 坐标＋1 个持久尺度 nuisance，按需追加起点 6 维随机克隆，
  正确传播/更新所有保留交叉块，并按主子矩阵边缘化旧 clone；保留确定性克隆的
  奇异 PSD，不做 jitter、特征值裁剪或凭空假定未知相关性为零。19 维只是声明的
  线性夹具，不能解释成已经接入真实 LIVO 的旋转误差状态。
- 冻结模型跨 4 窗，时长为 60/110/80/150 ms；每窗依次执行 pose-like、range-like、
  two-pose-flow-like 三类线性观测。完整联合模型与两个明确错误的对照使用同一批输入：
  每窗重置共享尺度（第二窗起）、清零 clone 交叉块。3 个 seed 各 12,000 条，合计
  36,000 条独立合成轨迹、144,000 个**跨窗相关**的窗口；同一批数据不能按方法数
  或窗口数重复计作独立样本。
- 独立 primitive-latent 批量条件解核对完整模型 72 次 clone/传播/更新/边缘化操作，
  最大均值差 `1.110223e-15`、协方差差 `1.249001e-16`；向量化样本与内核的最大
  均值差 `5.551115e-16`。方向固定为速度 XY 的 `[1,0]`，逐窗标称 95% **边缘**
  区间覆盖如下（单位 %），不是二维联合覆盖、任务推导方向或整轨迹风险保证：

  | 窗口 | 完整联合 | 重置共享尺度 | 清零 clone 交叉块 |
  |---|---|---|---|
  | 1 | 94.8750 | 94.8750 | 95.0722 |
  | 2 | 95.0139 | 94.9444 | 95.4583 |
  | 3 | 95.0250 | 94.8056 | 95.4278 |
  | 4 | 94.9750 | 94.5056 | 95.7722 |

  完整模型各窗协方差经验误差均通过事前 5% 门。消融效果的方向和大小不是通过条件，
  不把这些较小覆盖差异写作显著性或真实传感器改进。原始 24 个数组全部读回，独立
  审计重建三种模型、RNG、全历史 Joseph 顺序解、批量解、79 条首 seed 操作台账及
  四窗统计，70 项检查通过，并保存最终 20 坐标的独立诊断。
- 新增 `two_view_geometry.py`，在显式给定 bias/depth/位姿/外参下进行按时间顺序
  的分段中点 SO(3) 连乘、bias 导数和带杆臂的两时刻原始像平面投影；128 组几何
  case＋3 组旋转 profile 共 139 项门通过。独立 quaternion/static-point oracle 的
  最大旋转矩阵差 `9.104e-15`、bias 导数差 `1.508e-9`、投影差 `2.776e-16`、两视图
  导数差 `1.698e-10`。构造的换轴运动中，分量积分的 Exp 与时序连乘相差
  `0.1764811275 rad`，这不是原 R2A 航段或实物 IMU 的观测误差。
- 此积分器是声明的离散中点模型，不是任意连续角速度的精确积分。几何与联合
  Gaussian 在本轮分别验证，尚未组成一个非线性估计器；所有几何 covariance 保持
  未知，bias/depth 由夹具显式提供，未进行实际标定。相邻图像共享噪声、真实地图
  相关性、物理测距和原始像素融合尚未验证，真实录制光流状态更新仍为 0。
- 632/632 全套 Python 回归、14＋1 包隔离构建、44 项最终只读 QA 及 5 项原生保全
  SHA 复核通过。原 29 个 LIVO 源文件、433 个受保护旧文件和外部 AP/Gazebo 资产
  保持不变。正式第一轮完整 PASS；开发期仅 100 样本/seed 的接口烟测未过正式统计
  门，已标记 DEVELOPMENT_ONLY 保留，正式仍按事前冻结 12,000/seed 运行，未调门限。
  几何单元测试开发中曾用 `2e-17` 判断静止零位移，因 `5.55e-17` 舍入误差失败；
  修正单元断言至 `2e-15`，正式预测/导数门仍为 `2e-12/1e-7`，没有改变正式协议。
  封存前用冻结独立审计器复算，3＋128 条几何记录按冻结 driver 完整再生并逐字节一致；
  原成果保全在实验前后、最终 QA 与归档后均复验。新增归档 94 文件，3 个大原始文件
  按 SHA 留在原路径，封存后的全部 93 条 manifest 记录、完整文件集合及外置原件
  均再次只读核对通过。
- 本轮随机克隆/联合 Gaussian/时序旋转均属已有基础方法，参考
  [OpenVINS 状态与交叉协方差管理](https://docs.openvins.com/classov__msckf_1_1StateHelper.html)
  和[离散 IMU 传播](https://docs.openvins.com/propagation_discrete.html)。新增结果可用于
  方法实现与补充验证，不能充当项目核心创新已成立。下一步补含明确测距及噪声模型
  的新仿真输入，再将两时刻像素观测、深度 nuisance 与真实共同状态接起来；感知—
  运动联合决策、AP 新全任务闭环及 Orin/X25 实测仍待完成。

## 2026-10-20：R2B3A 原图几何与单窗相关更新验收

- 本轮为 `R2B3A_PRECONDITIONS_PASS`，证据位于
  `evidence/RESEARCH_R2B3A/r2b3a_suite_20261020T090255Z/`。新增逐特征原始像素追踪、
  给定转角的精确投影/导数、显式交叉协方差的单窗 Gaussian 内核及两个独立审计器。
  LIVO 的 29 个源文件与 R2B2 封存版本完全一致；实际光流状态更新为 0，整体 R2B、
  共同状态光流融合、硬件和新 AP 飞行均未通过本轮验证。
- 只读原 R2A bag，重新提取 1,285 张下视 RGB，核对同戳 CameraInfo、16,885 条 IMU
  和全部原始消息引用。原来 850 个有效角光流窗口中，没有一个窗口的两端都匹配保留的
  前视输入时刻。因此从原图分别建立 1,284 个相邻窗口、428 个保留前视输入节拍窗口，
  间隔中位数为 66/198 ms；全体范围分别为 32–67/165–199 ms，不能一概写成固定周期。
  新前端仅依靠像素及前后向追踪筛选，不使用 IMU 选点，也不输出旧版的鲁棒中位数。
- 两种窗口分别有 851/283 个通过至少 12 点的追踪门，共 32,993/10,850 条轨迹。
  “有效”仅表示追踪门通过，不能视作已有尺度或已标定的融合量测。全部 1,285 张原图
  有唯一台账；独立读取 22,018 条白名单原始 CDR，重算全部追踪点的 LK/FB、像素射线、
  陀螺积分和旋转投影/导数，23 项独立审计检查通过。未读取真值、旧里程计或 TF 作为
  算法输入，没有初始化 ROS 或向 AP 发布。aligned 对齐的是 429 个保留前视**输入**
  时刻，尚未筛成两端都有实际 LIVO 状态及 clone 历史的窗口。
- 原 bag 没有可用的下视米制距离、像素/陀螺积分噪声模型、状态与陀螺交叉协方差及
  跨时刻协方差；IMU 原始 covariance 全零按未知处理。所有窗口均保留这些缺项并阻止
  状态融合，没有用真值高度补尺度，也没有将追踪质量或 FB 误差充作协方差。
  `Exp(-gamma)` 投影及导数只对给定 gamma 代数精确；当前 gamma 是未去偏的逐轴
  梯形积分，不是一般非交换 SO(3) 时序积分或已标定的物理旋转补偿。
- 单窗内核要求显式提供 `C = Cov(x - prior_mean, v)`，检查完整联合 PSD，未知 C
  必须拒绝；不通过抖动、裁剪特征值或重复使用原始 C/R 伪造多窗滤波。冻结的 19 维
  线性 Gaussian fixture 仅观测两个速度分量，每个相关条件 3 个 seed×40,000 次，
  共 360,000 个独立合成样本，两种方法共享同一批数据，点估计在浮点舍入精度内一致，
  最大差为 `2.22e-16 m/s`。
  标称 95% 方向区间覆盖率如下，括号内为独立样本的 Wilson 95% 区间（单位 %）：

  | 声明相关系数 | 保留 C | 错误设 C=0 |
  |---|---|---|
  | -0.9 | 95.0183（94.8938–95.1400） | 84.5658（84.3603–84.7691） |
  | 0 | 94.9967（94.8719–95.1186） | 94.9967（94.8719–95.1186） |
  | +0.9 | 94.9825（94.8575–95.1046） | 100.0000（99.9968–100.0000） |

  该对照说明忽略相关项既可能过度自信，也可能过度保守；不是实际 Livox/微空误差
  标定、定位精度提升或 PL。18 项合成检查及 188 项独立原数组审计通过，54 个数组
  全部读回并独立重算。开发期相同 seed 的复跑不计为额外独立样本。
- 528/528 Python 回归、14 个基础包和 1 个 LIVO 包隔离构建、33 项最终独立 QA
  通过。第一次正式 attempt 因独立解码器拒绝 CDR 尾部补齐判 FAIL，完整保留。
  诊断确认首条 CameraInfo 的 373 字节前缀与官方重序列化逐字节一致，原记录末尾
  多 3 个零字节；其余补齐消息的语义字段另作复核。修复只接受空尾部或恰好到下一个
  四字节边界的零字节，其他内容继续拒绝，并保留原 payload 的全 SHA 校验；新增
  5 个严格正/反例后重新跑完整正式轮次，未降低原实验门限。
- 归档共 152 个文件，包含成功/失败轮次、诊断、源/配置快照、独立 QA、图和 SHA
  清单；10 个大原始文件按 SHA 留在原 Windows/WSL 位置。封存前用冻结审计器复算，
  核对所有实验内容及 attempt/development 集合不变。原始 7 个 bag 文件、受保护的
  433 个旧文件、AP 与外部仿真资产未改变。
- 学术复核：[OpenVINS 的状态克隆与交叉协方差维护](https://docs.openvins.com/classov__msckf_1_1StateHelper.html)
  及 [SEVIS 的地图 nuisance 相关性](https://openaccess.thecvf.com/content_CVPR_2019/papers/Geneva_An_Efficient_Schmidt-EKF_for_3D_Visual-Inertial_SLAM_CVPR_2019_paper.pdf)
  均为已有方法，本轮不作为创新；[IMU 状态与协方差传播](https://docs.openvins.com/propagation.html)
  及 [AP 官方光流/测距检查](https://ardupilot.org/copter/docs/common-optical-flow-sensor-setup.html)
  也是后续接入前提。下一步先做离线两时刻联合状态原型：维护 pose clone、距离 nuisance
  及所有交叉块，优先用原始像素重投影，再新录含显式噪声测距的仿真。通过后才接实际
  LIVO 光流更新，随后验证任务方向感知收益与算力预算的研究假设。微空实物型号及其
  内部陀螺/旋转补偿语义仍未确认，不能将本轮共享 Livox IMU 的仿真关系直接套到实物。

## 2026-10-19：R2B2 共同状态融合与完整回放验收

- 本轮为 `R2B2_REPLAY_PASS`。新增 `xq_fast_livo2`，视觉直接修改后续 IMU 传播和
  LiDAR 更新使用的同一个 19 维状态及协方差；不是三个里程计输出的拼接。对照臂保持
  相同 LIVO 调度和视觉地图，只禁用视觉 EKF 校正。作者官方源码为 ROS 1，移植来自
  社区 Humble 固定提交 `837b7bbc1431cb04cf936528e52c83c835efba8e`，GPL-2.0 和来源
  说明保留；不宣称作者官方 ROS 2 支持。Sophus/vikit 单独固定版本、打补丁和隔离安装。
- 输入来自 R2A 原始只读数据库，每臂精确发布并接收 844 帧点云、16,885 条 IMU、
  429 张前视图像；RGB 每 3 张取 1 张，按原采集时钟以 0.5 倍速回放，CDR/header
  不改写。点云为 Gazebo 瞬时快照、无真实 Mid-360 单点扫描时间，不能据此验证真实
  扫描去畸变或曝光同步。发布器及估计节点均不接真值、旧里程计、TF 或 AP 控制通道；
  评价真值另从原始数据库重建核对。
- 每臂 429 张图恰一条终结账本：3 张启动丢帧、1 张 IMU 初始化、424 张完成融合、
  1 张末尾覆盖不足。`7.128 s` 的 IMU 初始化调用未产出 LiDAR 云，禁止伪造有效里程计；
  有效输出从 `7.326 s` 至 `91.081 s`。末图 `91.279 s` 超过最后 LiDAR `91.200 s`，
  显式记为 terminal_uncovered。两臂全部原始图像分类/时刻序列及账本 SHA 一致，
  COMPLETE 必须等待完整输入和实际处理/flush，不能用接收计数替代队列处理完成。
- 视觉臂 424 个有效时刻中有 422 次满足冻结条件的实际状态/协方差校正；对照臂所有
  视觉前后位姿与协方差变化严格为零。不足 6 个选择点时真实跳过 EKF 校正，仍执行其余
  视觉地图流程；点数门不是可观性证明。独立重算位姿、完整 19 维协方差及其 6 维
  发布映射，检查有限性、对称性和相关尺度 PSD；协方差未被修复或转称 PL。
- 每臂 424 个时刻全部匹配原始真值，仅做首对 SE(3) 对齐，不拟合尺度或反复对齐。
  LIVO 的 ATE RMS/最大值为 `0.004881757 / 0.047376126 m`，关闭视觉校正为
  `0.004963916 / 0.047326230 m`。两臂轨迹差 RMS `0.000526892 m`；本条理想仿真
  航段不能证明一般精度优势、统计显著性或完整性。轨迹、离地高度和误差曲线已归档。
- 408/408 Python 回归、14 个基础包及 1 个融合包隔离构建通过；18 项数值检查、42 项
  证据检查、每臂 21 项处理完整性检查通过。源码/依赖/基础构建与实际 LIVO 安装产物
  绑定，实验前后重新核验；新源配旧二进制会拒绝。最终没有回放节点残留，原始 7 个
  bag 文件及 R0 保护的 433 个旧文件内容和集合均未变化。
- 四个正式失败 attempt 全部保留：相机服务初始化/默认 DDS 路径失败；BEST_EFFORT
  回放实收仅 `696/844` 点云、`383/429` 图像；以及完整回放发现的首空云误分类、
  2 点仍校正和小角诊断近似错误。最后一类 33 条角差不一致最大 `6.105e-12 rad`，
  来自 `sin(theta)` 近似而非真实角度；仅诊断改为精确四元数计算，上游算法 Log 未改。
  修复启动等待、初始图像/IMU 缓存、可靠传输、真实点数门与初始化分类后重新运行，
  未降低精度、数量或 `1e-12` 一致性门限。
- 本机 WSL mirrored 的默认 DDS 回环失败，经同域对照，逐轮绑定当前网卡的 UDPv4
  profile 可跨进程读取相机参数；未改系统网络/防火墙。12 个诊断案例、195 个诊断
  文件保留，具体内核/防火墙成因尚未证明；本配置不是实机 QoS/时延结论。轻量证据
  共 597 文件，含失败、源码/配置、构建/测试、独立 QA、图和 SHA 清单；大 JSONL
  及索引按 SHA 留在 WSL，归档时用冻结评价器重算并复核全部输入与 attempt 集合。
- 学术复核：[FAST-LIVO2 on Resource-Constrained Platforms](https://arxiv.org/abs/2501.13876)
  已有退化感知的视觉帧选择与混合地图，因此本轮是融合基线与验证基础，不作为创新。
  下一步在共同状态中接入下视光流，显式验证共享 IMU、地图和距离尺度误差及对应
  消融，再研究任务裕度与主动感知。真实传感器标定、Orin 热稳态、X25/AP 新闭环
  以及统计完整性本轮均未运行，硬件保持 NOT_READY。

## 2026-10-19：R2B1 原始接收与共享尺度误差验证

- 本轮为 `R2B1_SOFTWARE_PASS`，不是整体 R2B 或多源融合完成。新增纯 Python
  `micolink.py`、Linux TTY 采集/确定性回放脚本、共享尺度乘积矩，以及独立测试和
  冻结合成实验。现有 AP、R2A 合同与控制路径未改，不向 AP 发布光流或新融合状态。
- 官方 Micolink 0x51 图与示例给出 20 字节 payload、27 字节完整帧、包含头部的
  uint8 累加校验。图确认 `tof_status=1`、`flow_status=1` 为模块报告可用；质量值
  未标定为概率/协方差。线上无版本字段，字节序没有明确文字规定，本实现显式采用
  本地 LE profile；真实模块固件兼容性须另验，所有当前字节 fixture 都是 synthetic。
- 接收支持分片、噪声/损坏后的重同步、长度/校验/未知 ID 拒绝；保留原始字节、块
  偏移、单调接收时间与字段。重复/过期数据不刷新接受时间，允许有限的序号与毫秒
  计数回绕；重启、模糊跳变和长间隔需显式新 session。接收新鲜度与采集时钟、陀螺
  覆盖、旋转补偿和米制速度严格分开；后四项仍未验证，不从系统计数伪造曝光时间。
- 独立 94 项解析/计时测试、12 项归档回放测试、15 项数学测试加入原 250 项回归，
  最终 371/371 通过；14 包隔离构建和构建新鲜度通过。数学测试使用独立 Gauss-Hermite
  数值积分、相关尺度秩一闭式和有限差分检查。开发中发现并修复协方差全局容差掩盖
  小量非法子块及超大整数异常类型两项错误，原 2 failed/13 passed 日志保留。
- Linux PTY 实际经过 TTY 读取/采集进程：228 字节、39 个实际读取块、6 个合法布局
  帧，其中重复帧不推进计时，5 帧通过串口序列检查。损坏校验、超长头、未知 ID、
  无效状态和静默超时覆盖，12 项检查通过；原始 bytes/块时间回放一致。它不是真实
  微空模块捕获，不证明采集时间同步、实际 UART 电气兼容或 AP 入站已通过。
- `scale_uncertainty.py` 在已声明联合 Gaussian 输入下计算 `v=d*q` 精确一、二阶矩，
  包括两轴、尺度与速率的交叉项；不自动等同 Micolink 或 R2A 角流，不提供 PL。
  窗内各帧独立速率噪声、同一个距离误差时，只缩减速率方差，不缩减共同尺度方差。
- 冻结配置 `config/research_shared_scale_study.json`：3 seeds、每格 120,000 个独立
  合成窗口，N=1/10/40、已知距离控制与共同距离压力两组，共 720,000 trials。
  0.12 m 测距标准差等参数为人工压力设定，不是微空实测精度；两种方法使用相同
  速度点估计，只比较正确窗协方差与错误的“完整单帧协方差/N”，后者仍保留 XY 项。
- 共同尺度压力组的 95% 方向区间实际覆盖率：N=1 为 94.930%/94.930%，N=10 为
  95.007%/48.289%，N=40 为 94.912%/25.333%（保留共享尺度/假设逐帧独立）。
  N=40 的 Wilson 95% 区间分别为 [94.786%,95.035%]/[25.087%,25.579%]；已知
  距离控制中两者一致，N=40 为 95.046%。最大经验/预测协方差相对 Frobenius 误差
  0.649%，22 项预声明检查通过。这是特定模型反例，不是定位精度提升或完整性保证。
- 原始 `samples.npz` 17,291,302 bytes 留在 WSL，以 SHA-256 索引；独立完整读取
  36 个数组、720,000 个窗口并重算经验协方差和覆盖率，哈希前后相同。轻量证据含
  代码/配置快照、构建/测试日志、失败日志、全部串口 bytes 与图，位于本节表格路径。
  原 433 个受保护文件的内容和集合及四处原 Gazebo 资产审计均 PASS。
- 学术复核：[FAST-LIVO2 IV-D](https://arxiv.org/html/2408.14035v2#S4.SS4) 的顺序更新
  依赖给定状态后的量测噪声独立，不能据此推出共享 IMU/地图/尺度的估计输出独立。
  [SEVIS，CVPR 2019](https://openaccess.thecvf.com/content_CVPR_2019/html/Geneva_An_Efficient_Schmidt-EKF_for_3D_Visual-Inertial_SLAM_CVPR_2019_paper.html)
  已处理地图不确定性和活动状态交叉协方差；本轮基础统计与接收器不作为论文创新。
  [官方协议图和源码](https://micoair.cn/zh/docs/sensors/micolink) 是接收依据。
- 下一阶段仍是共同状态 LIVO/flow 基线、真实时间/外参/尺度标定和依赖消融；新 AP
  飞行、真实传感器、Orin 热稳态与统计完整性本轮未运行或宣告通过。

## 2026-10-19：硬件候选与 R2B 台架计划

- 用户确认 Livox Mid-360、Orin NX、CUAV X25 MEGA、ArduPilot 和微空科技光流；
  光流具体型号及前视相机尚未选定。新增 `config/research_hardware_plan.json`，
  只记录选型候选与验收顺序，运行配置仍使用原 R2A 仿真合同，硬件保持 NOT_READY。
- 光流主候选为微空 MTF-01P：官方列 100 Hz、8 g、3.3 V UART/115200、5 V 供电，
  自带 ToF 可作为距离尺度候选；12 m 标称测距有反射率/照度条件，不能当作光流有效
  高度保证。MTF-02P 为重量受限时的 50 Hz 备选。01P 精度与 02P 室外量程的中英文
  资料有差异，计划中保留冲突，不据此宣称精度或使用高度已经通过。
- 相机主候选为大恒 MER2-160-227U3M 非 `-L`：1440×1080、IMX273 全局快门、
  USB3、Mono、C/CS、触发 I/O；先以 30 Hz 为采集目标，再评估 60 Hz。厂商提供
  ARMv8 SDK，但推荐平台列表较旧，目标 Orin/JetPack/ROS 2 组合仍需厂商确认与台架
  验收，图像编码也需适配。FAST-LIVO2 作者同款海康 MV-CA013-21UC 为复现备选，
  当期供货未核实。型号均为推荐候选，未记作用户已选定或已购入。
- Micolink 原始字段为系统 `time_ms`、距离/状态、`cm/s@1m` 光流与质量；没有被证明
  的曝光/积分窗口或陀螺积分。模块时间不直接当曝光终点，100 Hz 不直接当积分时间；
  倾斜/非平面地面、旋转补偿、外参和时钟映射验证前不声明米制速度有效。共同 ToF
  尺度可关联两个速度轴，物理独立模块不代表统计独立。
- 主研究链计划为 Mid-360/相机/光流→Orin 共同状态估计→ExternalNav→X25/AP。
  光流厂商 `Mavlink_apm` 直连 AP 单独作为基线，不能与 Orin 的 Micolink 处理链假定
  等价；ExternalNav 已使用光流时，不在 AP 再无条件按独立来源重复融合。仅经 Orin
  转发的光流不能被称作 Orin 失效后的独立后备，后备物理链路和切换需另行验证。
- 2026-10-19 官方 stable/X25 版本文件为 ArduCopter 4.7.2 OFFICIAL，commit
  `fb773567896014da007bca58358704d4a0d8516f`；这是公开版本观察，不代表实板当前
  固件。现有 SITL 4.5.7 证据继续固定；实板需读取 AUTOPILOT_VERSION/启动日志、
  固件哈希与参数，按实际版本重验光流入站、ExternalNav、旧指令拒绝和源切换。
- R2B 台架顺序已记录为 H0 身份/原始采集，H1 时间/几何/尺度，H2 高度/纹理/光照/
  反射/倾斜可用域，H3 共同状态消融与 AP 切换。标定和测试分开，未知项不填仿真值；
  这次只校核计划 JSON 与文档，没有产生新的仿真、实机或融合精度通过结论。
- 主源：[微空 01P 手册](https://micoair.cn/zh/docs/sensors/sensors/mtf-01p-sensors)、
  [Micolink 协议](https://micoair.com/docs/decoding-micolink-messages-from-mtf-01/)、
  [大恒相机](https://en.daheng-imaging.com/show-106-1968-1.html)、
  [ARM SDK](https://en.daheng-imaging.com/list-59-1.html)、
  [作者同步硬件](https://github.com/xuankuzcr/LIV_handhold)、
  [AP 实际版本文件](https://firmware.ardupilot.org/Copter/stable/CUAV-X25-MEGA/firmware-version.txt)。

## 2026-10-19：R2A 相机、角光流与来源契约

- 结论为 `R2A_INTERFACE_PASS`，不是整体 R2 或融合算法 PASS。新增前视/专用下视
  两路 Gazebo 相机与 Harmonic 原生桥，使用显式仿真内参/光学轴/外参；飞行仍为
  FAST-LIO→ExternalNav→AP，相机/光流旁路采集，AP 固件未改。
- 配置为 `config/research_multisensor_contract.json` 和
  `config/research_multisensor_gate.json`；入口
  `bash scripts/run_ap_research_gate.sh --multisensor-contract`。
  相机/独立光流模块型号与实机标定尚待补充，hardware_readiness 保持 NOT_READY。
- 光流只用下视像素、CameraInfo 与同期 Livox IMU：前后向 LK、鲁棒中值、完整时间窗
  陀螺积分与旋转投影补偿。角流是内部图像轴的归一化位移小角度模型；质量是跟踪
  支持启发式，不是概率/PL。无有效距离时米制速度无效，不用 LIO 高度或真值填尺度。
  共享 IMU 依赖显式记录，`independent_fusion_ready=false`。
- LiDAR 仿真明确标注瞬时快照、逐点时间为零；相机为瞬时曝光，均不等同真实驱动
  已标定。250 项 Python、18 项 C++/退出检查通过。另修复 ExternalNav 边界：重复/
  倒退/零采集时间及非法协方差被拒绝，拒绝输入不刷新健康；保留原采集时间诊断。
- 最终轮 `r2_contract_20261018T165306Z_447`：AP 28 项、多源 45 项、离线元数据
  故障回放 10 案例、存储/相机审计 29 项均通过；任务 99.001 s，真值净前进
  `2.000978 m`，初始对齐 ATE RMS `0.006212055 m`，最大误差 `0.014651092 m`。
- LiDAR 10 Hz/828 帧、IMU 200 Hz/16,551 条，前视/下视各约 15.151 Hz/1,277 帧。
  1,278 条角流中 850 条有效、428 条不可用/被拒绝；850 条均有独立观察到的完整
  IMU 窗口支持。低纹理、少跟踪点等无效结果保留。时间无重复/倒退；两路原生
  CameraInfo 验证均无拒绝、无 SDF 回退。图像质量固定取研究第一帧，另存起降画面。
- 最终回调年龄 p99 为 LiDAR/IMU/前视/下视/角流 `64/7/33/48/53 ms`；这是相对
  缓存仿真 `/clock` 的调度诊断，不是硬件同步精度或 Orin 时延。十项故障是已采集
  样例的离线元数据变换，不是实际像素故障下的融合精度或统计完整性实验。
- 三轮成败全部保留。首轮发现 FAST-LIO 动态 TF 与原静态 TF 对 livox_imu 的双父
  冲突，以及落地图像覆盖研究图像诊断；已修为 `map→imu→base→lidar/cameras` 并固定
  采样阶段，数值阈值未降低。第二轮 AP/多源与后加存储审计全部通过。
- 首轮另触发 rosbag2 0.15.16 的 zstd EOF 边界问题：独立解码后 SQLite 内容及
  127,801 条全部 BLOB 可读，但压缩结束帧缺失，标准读法漏最后 128 KiB，严格审计
  仍记 FAIL，原包未修补。最终改无压缩 SQLite、resilient（WAL/NORMAL）、256 MiB
  分段；六个数据库完整性、全部 128,682 条载荷读取及总数/逐 topic/逐文件计数均通过。
- 审计工具 `scripts/audit_multisensor_bag.py` 只在私有临时副本中解压/打开 SQLite，
  前后复核原 bag 文件集合、大小和 SHA-256；所有目标 JSON 反序列化，其余传感器
  做代表性语义抽样。以后 R2 入口自动追加该审计；本轮最终审计在闭合后独立执行。
  归档还绑定审计源码 SHA 与实际原 bag 哈希，防止用旧审计通过新原始文件。
- 证据包含三轮成败、两轮构建日志与相机 PNG；`2,160,070,222 bytes` 原始大文件
  留在 WSL 并作 SHA-256 索引。原 433 个受保护文件及集合均未变，外部资产审计通过。
  Gazebo GPU 前两轮退出 SIGSEGV，最终轮超时后有界强制回收；ROS/recorder 正常
  退出，最终无本轮节点及 5760/9002 端口残留，环境退出问题仍单列。
- 主源依据：[FAST-LIVO2](https://arxiv.org/html/2408.14035v2)、
  [Livox 时间同步](https://github.com/Livox-SDK/livox_wiki_en/blob/master/source/tutorials/new_product/common/time_sync.rst)、
  [REP-103 optical 轴](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst#suffix-frames)。
  [AP 4.5.7 光流后端](https://github.com/ArduPilot/ardupilot/blob/Copter-4.5.7/libraries/AP_OpticalFlow/AP_OpticalFlow_MAV.cpp)
  不直接消费本轮内部角流或 OPTICAL_FLOW_RAD，未声明其进入 AP EKF。
- 下一步 R2B：真实设备/标定契约、共同状态融合基线、尺度及共享 IMU/地图误差消融；
  完整性界、Orin、统计验证和实飞分别属于后续阶段。

## 2026-10-19：R1 研究链接入 ArduPilot

- 研究问题：P13/P15 的在线规划能否通过 MAVROS→AP→Gazebo 动力学执行，并在上游
  指令失活、桥接进程仍运行时识别旧指令。新增原子速度/生成时间/源 odom 时间/序号
  消息、AP 速度执行器、任务使能与禁用确认交接、独立真值 evaluator、分阶段 launch
  和验收脚本。P4 起飞/悬停后预热地图，地图 READY 后才启动研究控制。
- 配置在首次运行前固定：2 m 航段、0.2 m/s 速度上限、0.35 s 命令年龄；真值净前进
  ≥1.65 m、ATE RMS ≤0.1 m、最大位置误差 ≤0.3 m；0.9 s 旧指令重放冻结窗内，
  首次零速发布延迟 ≤0.55 s。沿用原 P7 标定和在线地图 5×1 lattice，不降低阈值。
- 14 包隔离增量构建通过，195 项 Python 回归/ROS 节点生命周期/执行器/任务交接/
  验收检查通过。源码与安装新鲜度在两轮飞行前后均通过。
- 正常轮 `r1_normal_20261018T161757Z_1422`：28 项检查全部通过，任务 95.401 s；
  真值净前进 `2.004544 m`，初始位姿对齐后的 ATE RMS `0.006287827 m`、最大
  `0.014271384 m`；实际执行 `lattice_y_+0.35_z_+0.00`，完成 1 段/1 次决策。
  研究结束禁用确认 `0.126155 s`，再进入 FCU 位置保持、LAND 与 disarm。
- 重放轮 `r1_replay_20261018T162021Z_402`：33 项检查全部通过，任务 88.901 s；
  真值净前进 `2.002242 m`，ATE RMS `0.006472483 m`、最大 `0.014264764 m`。
  拒绝原序号/原时间戳重放 43 次，冻结后 `0.349367 s` 首次发出零速目标，累计
  12 次冻结零速发布；恢复新指令后完成任务。此时间不是机体实际制动/停止时间。
- 两轮唯一 MAVROS 速度发布者为 `impact_ap_execution`；旧 Gazebo 直驱通道无订阅。
  真值话题图只有桥、独立 evaluator 与 recorder。禁用后发布计数为 0；位置保持在
  新鲜且序号不倒退的禁用确认之后启动。任务运行期日志审计无节点异常。
- 全部 6 次尝试均保留：前 4 次分别在结果路径审计、MAVROS 参数就绪、ROS 内置
  executor 属性冲突、错误订阅 P11 完成状态处失败。已修复路径白名单、就绪重试、
  命名冲突及 P12 实际状态话题；另补了禁用确认握手。5 轮构建/测试日志也全部归档，
  包含一次 Humble 生命周期测试错误断言。失败结果未改写为成功。
- 两轮任务完成后的 Gazebo GPU 清理均出现 SIGSEGV；任务 Gate 与环境退出分别
  记录，不能声称全部进程干净退出。最新运行器已避免 ROS launch 双重 SIGINT，
  重放轮的研究 ROS 节点均正常退出，但 GPU 清理问题仍未解决。最终进程与端口检查
  未发现本轮 SITL/MAVROS/Gazebo/研究节点残留，5760/9002 已释放。
- 旧工作区及 AP 二进制/默认参数/插件共 433 个受保护文件，内容与文件集合完全一致；
  模型/世界隔离审计通过。轻量证据见本节对应 `evidence/RESEARCH_R1/`；
  `1,478,133,412 bytes` 原始大文件留在 WSL，以 SHA-256、大小和路径索引。
- 源码复核：[AP 4.5.7](https://github.com/ArduPilot/ardupilot/blob/Copter-4.5.7/ArduCopter/mode_guided.cpp#L496-L510)
  按接收时刻刷新速度超时，故旧指令重发需要上游生成时间/序号检查；
  [MAVROS 官方速度插件](https://github.com/mavlink/mavros/blob/ros2/mavros/src/plugins/setpoint_velocity.cpp#L94)
  输入按 ENU 转到 LOCAL_NED，`frame_id` 本身不进行 TF。运行时也读取参数确认。
- 边界与下一步：这是短航段接口 SIL，尚未验证多源融合、Orin、复杂任务统计完整性
  或实飞。R2 先建立 Livox/相机/下视光流的时间、外参、尺度和数据来源契约；以
  [FAST-LIVO2](https://arxiv.org/html/2408.14035v2#S4.SS4) 为融合参考，研究共用地图、
  时间偏差和外参误差引起的相关性及 PL 覆盖失配，再进入后续校准/消融。

## 2026-10-18：R0 独立研究基线复验

- 用户授权使用 WSL 进行算法迭代和仿真，逐步汇报并检索/复核。研究平台调整为
  Livox 激光雷达、相机、下视光流、IMU 和 Orin NX，继续 ArduPilot；本轮只复验现有
  LiDAR/IMU + AP SIL 基线，新增多源与 Orin 尚无本轮实验结论。
- Windows 源码为 `D:\CODEX\impact`，新增独立 WSL 副本
  `/home/accelerate/impact_research_ws`。从源新建 14 个 ROS 包，耗时 10 min 58 s；
  `PYTHONNOUSERSITE=1`，包顺序构建、包内并行度 2，未更改系统依赖。
- 140 项现有 Python 回归、8 项桥接转换/退出回归全部通过。新增构建新鲜度工具的
  6 项测试通过，覆盖源码/安装更改、三个根路径错配、缓存和缺失 manifest。
- 新增 `scripts/check_build_manifest.py` 与 `scripts/run_research_baseline.sh`。
  运行前后核对源码和安装摘要，实际加载的 ROS 包均来自新副本；新运行器也向 P4
  子进程转发中断，以便回收其拥有的独立进程组。
- 本机 MAVROS 日志确认 `ArduCopter V4.5.7 (b7207551)`。GPS/GPS2 禁用，EKF3 使用
  FAST-LIO ExternalNav；任务完成起飞、悬停、2 m 边长矩形、返航、降落和 disarm，
  耗时 95.301 s。原有 Gate 阈值未调整，结果 PASS。
- 独立真值匹配 545 个样本，ATE RMS `0.005973794 m`、最大位置误差
  `0.014730810 m`。这些是固定结构化房间的仿真结果，不作为实机精度或统计安全保证。
- Ground Truth 图只有桥发布者、评价节点与 rosbag recorder；算法与控制未订阅。
  旧工作区受保护源码/脚本/配置/文档和 AP 二进制/默认参数/插件共 433 个文件，
  本轮前后内容与文件集合一致；既有模型/世界隔离审计也 PASS。
- 原始结果留在新 WSL 副本的 `experiments/results/research_baseline/` 和
  `experiments/results/external_nav/`。轻量证据已归档，146,063,690 bytes 大型记录以
  路径、大小和 SHA-256 索引保留。SITL、MAVROS、Gazebo 已回收。
- 版本检索结论：4.5.7 的速度目标接收用当前接收时间刷新超时，旧命令重发可绕过上游
  失活检测；速度超时归零与纯位置目标停流行为不同，不能统称自动 LAND。来源：
  [4.5.7 Guided](https://github.com/ArduPilot/ardupilot/blob/Copter-4.5.7/ArduCopter/mode_guided.cpp)、
  [4.5.7 MAVLink 接收](https://github.com/ArduPilot/ardupilot/blob/Copter-4.5.7/ArduCopter/GCS_Mavlink.cpp)。
- 下一阶段 R1：为 P13/P15 接入 AP 速度执行器与唯一控制权交接，保留命令生成时间，
  验证坐标方向、陈旧命令拒绝、ExternalNav 健康和起飞后任务时钟。R0 PASS 不等同
  高阶完整性算法已经接入 AP。

## 已解决问题

1. Humble symlink-install 与用户级 setuptools 82 不兼容：通过单次构建的
   `PYTHONNOUSERSITE=1` 使用系统 setuptools 59.6，未修改全局环境。
2. ROS 2 CLI 跨域 daemon 使 `set_stream_rate` 发现不稳定：将
   `REQUEST_DATA_STREAM` 移入 P1 rclpy 状态机，与 MAVROS 控制共享同一上下文。
3. P1 运行器仅管理本轮进程组，并补齐 INT→TERM→KILL 有界回收。
4. P2 验证器在首帧点云到达前误把“尚未观测字段”判作 FAIL：改为只有收到数据后
   出现时间戳、frame、有限值或布局违规才提前失败，并新增回归测试。
5. P3 初始阶跃速度给运动学模型制造非物理 IMU 冲击，第三次换向后 FAST-LIO2
   点到面匹配失效；改为 C1 连续的平滑梯形速度与正负偏航激励后，未调低 Gate、
   未使用真值，两个正式场景均为 0 次有效点匹配失败。
6. P3 runner 最初向 launch 整个进程组发送 SIGINT，Python 节点收到重复信号并留下
   伪关停错误；改为先只通知 session leader，由 launch 单次转发，超时后才整组回收。
7. P4 起飞命令被后续位置设定点覆盖 Guided TakeOff 子模式；起飞/爬升阶段改由飞控
   独占，稳定到达后才开始位置闭环。
8. P4 位姿差分速度的滤波延迟导致 2 m 航段持续振荡；改为由 FAST-LIO ESIKF 直接
   发布机体系速度与协方差，正式轮次 537/537 帧使用 ESKF 速度、0 帧回退。
9. MAVROS 参数服务先于 FCU 参数拉取完成而出现 `NOT_SET`；改为节流重试，真实值
   不符合 GPS-off/ExternalNav 契约时仍立即失败。
10. P5 水平栅格最初投影地板/天花板点形成假障碍环；改为飞行高度切片，三维完整
    点云仍独立送入 EGO。
11. EGO 点云障碍只在 XY 使用配置安全半径、Z 固定膨胀 0.10 m，导致中心轨迹越过
    柜顶而机体包络擦碰；Z 改用同一 0.35 m 半径后正式复验 0 碰撞。
12. P6 第一次隔离构建把 Eigen 三维零向量用于 3×3 矩阵初始化，编译期尺寸断言拒绝；
    改为 `M3D::Zero()` 后 13 包完整构建并重新生成源码/安装树 manifest。
13. P6 新结果目录最初未列入外部资产审计输出白名单，运行器在启动任何仿真核心前
    主动拒绝；仅增加项目内 `impact_p6` 输出路径，不改变外部资产审计范围。

## P0 正式结果

- 完成现有仓库、ROS 2 包、launch、配置、MAVROS/ArduPilot 与隔离边界审计。
- ROS 2 Humble symlink 构建 5 个初始核心包成功；`xq_gz_bridge` C++ 测试 8/8。
- Gazebo world/model SDF 校验通过；既有外部项目 world/model 字节级不变。
- 建立项目专属构建树、DDS domain、Gazebo partition 与 `xq_` 资源前缀约束。
- 独立报告：`docs/P0_REPOSITORY_VALIDATION_REPORT.md`。

## P1 正式结果

- 连续墙钟运行：600 s；SITL、Gazebo、MAVROS 无提前退出。
- 飞行闭环：80.034 s；GUIDED、ARM、TAKEOFF、30 s HOVER、LAND 全部确认。
- 终态：`connected=true`、`armed=false`、`mode=LAND`。
- rosbag：38,231 条，其中 state 647、odom 12,046、IMU 12,769、GPS 12,769。
- 隔离：旧 Gazebo world/model 前后 SHA-256 完全一致；无残留核心进程。

## P2 正式结果

- 连续单调采集：600.435 s；LiDAR/IMU header 分别覆盖 599.495/599.553 s。
- LiDAR：5,996 帧，10.00008 Hz，最大间隔 0.100 s，每帧 23,040 点。
- IMU：119,912 帧，200.00083 Hz，最大间隔 0.005 s。
- 时间戳、frame、TF、有限值、字段布局、点数共 11 项 Gate 全部通过。
- rosbag：10 个 zstd 分卷、1.5 GiB、365,535 条消息。
- 隔离：旧 Gazebo world/model 前后 SHA-256 完全一致；日志无错误，核心进程无残留。
- 边界：P2 为 Mid-360S-like 数据契约，不代表真实 Livox 扫描或 FAST-LIO2 已验证。

## P3 正式结果

- `structured_room`：`experiments/results/localization/p3_structured_room_20260822T142413Z_6016`；
  86.033 s、857 个匹配样本、10.0 Hz、ATE RMS 0.0330 m、最大位置误差 0.0444 m。
- `long_corridor`：`experiments/results/localization/p3_long_corridor_20260822T142613Z_6965`；
  85.171 s、848 个匹配样本、10.0 Hz、ATE RMS 0.0507 m、最大位置误差 0.0669 m。
- 两场景最大 odom 间隔均为 0.100 s，时间戳单调、数值有限、无明显跳变；运行日志
  均为 0 条 ERROR、0 次 `No Effective Points`。
- FAST-LIO2 运行时订阅图只含 `/livox/lidar`、`/livox/imu`、`/clock` 与参数事件；
  `/xq/eval/*` 只由评估器消费。
- 两轮均保存 zstd rosbag、配置/源码/构建哈希和旧资产前后审计，外部地图模型不变。
- 边界：该结果证明 Gazebo SIL 中项目本地 FAST-LIO2 基线，不代表真实 Mid-360S、
  Atlas 200I DK A2 满载性能、室外定位或实机 ATE。

## P4 正式结果

- 证据：`experiments/results/external_nav/p4_20260822T154217Z_10946`；任务闭环
  102.780 s，2 m 起飞、悬停、2 m × 2 m 矩形、返回、降落和上锁全部完成。
- 12 项飞控参数逐项核验；`GPS_TYPE/GPS_TYPE2=0`、SITL GPS 禁用，EKF3 的位置、
  速度、高度和航向源均为 ExternalNav。
- FAST-LIO ATE RMS 0.00509 m、最大误差 0.01405 m、554 个匹配样本；ExternalNav
  537 输入/537 输出、0 拒绝、ESKF 速度 537、位姿差分回退 0。
- rosbag 106.271 s、139.5 MiB、74,402 条消息；真值仅供评估，未进入算法或飞控。
- 外部 Gazebo/ArduPilot 资产运行前后字节级一致；本项目运行副本、SITL 状态和日志
  全部位于独立结果目录。
- 边界：该结果证明 GPS-off Gazebo/SITL 闭环，不代表真实飞控、Atlas 或实机无 GPS 飞行。

## P5 正式结果

- 证据：`experiments/results/baseline_v1/p5_20260823T042657Z_29091`；任务闭环
  281.101 s，4 个自动 Frontier 目标、110 条 EGO B-spline，自动结束并降落。
- 0.10 m 导航栅格，0.05 m 评估地图；真实轨迹 43.170 m、7,594 个空中真值样本、
  0 碰撞，最小障碍净空 0.250 m。
- Frontier 终态 0 cell / 0 cluster；没有人工航点，选择目标为 `J=I-0.18d`。
- Ground Truth 只由 rosbag 和评估器订阅；ExternalNav、Frontier、EGO 和任务节点均
  未订阅，飞控 GPS/SITL GPS 仍关闭。
- rosbag 291.259 s、228,886 条消息、455 MiB；外部地图模型前后字节级一致。
- 边界：该结果证明 structured-room 的 BASELINE_V1 SIL，不代表 IMPACT 创新、
  动态障碍、Atlas 或实机完成。

## P6 正式结果

- 证据：`experiments/results/impact_p6/p6_20260823T065905Z_36386`；P6 在线 Gate 与
  完整 295.498 s rosbag 分析均为 PASS。
- 全程 1,467 帧真实 FAST-LIO 几何与 1,467 帧方向完整性输出；有效点面约束
  1,196–4,074/帧，条件数 1.443–5.464，弱方向 PL 0.0233–0.0529 m。
- 弱轴主导次数 X/Y/Z 为 675/792/0，证明输出随真实扫描几何改变，而非单一标量分数；
  `P_int` 全程正定，PL 方程最大数值误差 `6.94e-18`。
- P6 不订阅 Ground Truth、不控制规划；P5 闭环同时保持 4 Frontier、125 B-spline、
  0 碰撞、自动降落解锁，外部地图模型不变。
- 边界：`k_alpha=3.0` 尚未用独立训练/测试场景校准；覆盖率、Alert Limit、
  Integrity Margin、Atlas 和实机仍未验证。

## P7 正式结果

- 证据：`experiments/results/impact_p7/p7_20260823T082157Z_48605`；训练、冻结标定、
  两场景独立验证与总 Gate 全部 PASS。
- 训练只使用 structured room 与 long corridor；训练集最坏跨场景 q95 比值
  `4.4887300142` 作为所有方向统一迁移储备，测试数据未调参。
- 冻结 `k95` 为 X/Y/Z/弱方向 `10.251700 / 26.391371 / 51.234940 / 10.183846`；
  标定文件测试前后 SHA-256 一致。
- 两个全新验证轨迹共 5,256 个样本×方向，95%/99% 覆盖率均为 100%，95% 漏检率 0；
  ATE RMS 为 0.05024 m 与 0.04326 m。
- P8 Alert Limit 尚未定义，故 false-alarm rate 为 N/A；P7 不把它伪装成 0。
- 四轮外部地图模型审计均 PASS，预测器无真值订阅且无控制反馈，结束后无残留进程。

## P8 正式结果

- 证据：`experiments/results/impact_p8/p8_20260823T091322Z_57082`；静态障碍 Alert
  Limit 的 11 项自动 Gate 全部 PASS。
- 对 104 条 EGO B-spline 的当前剩余轨迹做 de Boor 采样；526 条输出、单轨迹最多
  82 个采样点、单帧最多 8,770 个局部静态障碍点。
- 几何净空 0.0541–1.0111 m，AL -0.5446–0.4356 m，均值 0.0893 m；最大时延储备
  0.06135 m。
- 最近点几何最大误差 `1.98e-13 m`、方向范数误差 `2.88e-12`、AL 方程误差 0。
- 107/526 输出为负 AL，表示对应未来轨迹段没有定位误差预算；P8 只报告，不控制规划。
- Ground Truth 隔离、P7 prerequisite、输出 rosbag 和外部资产字节级审计全部通过。
- Gazebo 在线录制复核：`p8_gazebo_final_20260823T104800Z` 同样 PASS，97 条 B-spline、
  436 条 AL、7,588 个碰撞样本、零碰撞并自动降落；`gz_record/state.tlog` 已封盘。
- WSLg GUI 与正式 Gate 解耦：回放使用独立 `GZ_PARTITION`，顶视窗口只透明房顶，
  四墙、地板、内部隔断和录制的无人机运动均保留。

## P9 正式结果

- 证据：`experiments/results/impact_p9/p9_20260823T135112Z_1110615`；11 项 Integrity
  Margin 自动 Gate 全部 PASS，Gazebo state 与 ROS 话题同步封盘。
- P8 输出扩展为整条剩余轨迹的 AL/障碍方向剖面；P9 对每个样本计算
  `PL(a)=k95*sqrt(a^T P_int a)` 与 `M=AL-PL`，再取 `M_min`，没有用最小 AL 单点近似。
- P7 train-only 冻结校准 SHA-256 为
  `771bdffcf3d4422d4641424dab326a08aa5be2b0dffd7f9d2f2f9ff82ea9f038`；任意障碍方向
  使用四个已校准方向中最大 `k95=51.234940` 作为保守统一系数。
- 相同 `P_int=diag(1.6e-5,1.6e-5,1.6e-5) m²` 下，两场景 PL 均为
  `0.204940 m`；wide room `M_min=0.740893 m`，ACCEPT；narrow passage
  `M_min=-0.157860 m`，REJECT；储备阈值 `0.10 m`。
- 两条候选 B-spline 只向下游转发 wide room 的 ID 9001；narrow passage ID 9002
  在传输层被阻断。判决是布尔硬门，不存在 `cost += lambda * integrity`。
- rosbag 4.406 s、352 条消息；候选轨迹 2、认证输出 1、Margin 42；65 项算法回归测试
  全部通过，Ground Truth 未进入节点图，外部地图模型字节级不变。
- Gazebo+RViz 双窗口已实测：Gazebo 世界没有房顶、保留宽/窄空间墙体；RViz 同时显示
  绿色 ACCEPT、红色 REJECT、AL/PL/Margin 与 PL 包络。

## P10 正式结果

- 算法/传输契约证据：
  `experiments/results/impact_p10/contract_20260823T154236Z_14881`；baseline 预测 Margin
  `0.058574 m`，只有 left-lateral 满足冻结硬约束并被发布，11 项检查全部 PASS。
- 正式三臂飞行证据：
  `experiments/results/impact_p10/gate_20260823T154534Z_16472`；baseline、yaw-only、
  minimum-excitation 分别独立运行 Gazebo、LiDAR/IMU、FAST-LIO、P6/P10 与 evaluator。
- 共同决策快照预测最低 Margin 为 `0.045083 / 0.045083 / 0.419631 m`；实际最低
  Margin 为 `-0.320630 / -0.448158 / +0.104633 m`。baseline 与 yaw-only 均不足，
  minimum-excitation 选择并执行 `right_lateral`，16 项汇总检查全部 PASS。
- ATE RMS 为 `0.121176 / 0.085154 / 0.117578 m`，弱方向误差 RMS 为
  `0.116408 / 0.074186 / 0.113459 m`；minimum-excitation 相对 baseline 多走
  `0.065915 m`，三组任务时间均为 `43.0 s`。
- 在线 Information Map 只消费 FAST-LIO `/cloud_registered`，由时间衰减 voxel surfel
  生成信息矩阵；协方差预测冻结下界为 `1e-5 m²`，避免重复观测导致虚假零不确定性。
  Ground Truth 只由 evaluator/logger 消费，飞行控制器明确报告未订阅真值。
- 可视化证据：
  `experiments/results/impact_p10/visual_20260823T154916Z_18660`；飞行与录制均 PASS，
  Gazebo `state.tlog` 507,904 bytes，RViz bag 17,824,985 bytes，开放屋顶并保留墙体。
- 77/77 算法回归测试、13 包隔离构建、ROS 契约、三臂正式 Gate、Gazebo/RViz 录制和
  外部资产字节级审计全部 PASS；源码/安装树 SHA-256 分别为
  `7c4e26e0feaaabf4d09bff91652bbd7d25a543db0a80dfae4db6f46cdbd50767` 与
  `16ce2f5f24fe237c22ba5c39c423ed8e6f36a4a1f62a032811e12cf009078133`。
- 边界：证明固定静态长走廊 Gazebo SIL 的有限候选主动恢复，不代表动态障碍、连续
  动作全局最优、Atlas 满载性能或实机安全保证。

## P11 正式结果

- ROS 合同证据：`experiments/results/impact_p11/contract_20260827T053924Z_1726`；
  高信息完整性不足、安全低信息、碰撞越界和返航能量不足四类候选均按独立硬门处理，
  Margin 未进入效用，选择器节点图无 Ground Truth。
- 全走廊双臂证据：`experiments/results/impact_p11/gate_20260827T101925Z_22733`；
  两臂均从 Gazebo `x=-12 m` 飞至 `x≈+12 m`，净前进 `23.984448 / 23.993332 m`；
  四个滚动 batch 逐段重建候选并重新硬认证，24 项配对检查全部 PASS。
- information-only 四段均 direct；integrity-constrained 序列为
  `right, direct, right, direct`。实际最低 Margin 为 `-0.299559 / +0.121361 m`，
  完整性约束提升 `0.420920 m`，约束臂四段最低值均不低于 `+0.121361 m`。
- 平均每 batch 信息收益损失 `0.125`、额外路径 `0.260325 m`、时间开销 `0 s`、
  ATE 增量 `0.008695 m`。
- 可视化证据：`experiments/results/impact_p11/visual_20260827T102347Z_24548`；Gazebo
  `state.tlog` 1,404,928 bytes、RViz bag 目录 189,121,443 bytes，开顶保留墙体，
  双窗口已实际启动并显示四个 batch。
- 83/83 算法测试、13 包隔离构建、合同 Gate、正式飞行 Gate、可视化录制/回放和
  外部资产逐字节审计全部 PASS。
- 边界：验证固定静态场景 24 m 走廊中有限 Frontier 多视点候选的滚动硬认证与选择，不代表
  Frontier 覆盖率、动态障碍、连续全局最优、Atlas 满载或实机安全保证。

## P12 正式结果

- 正式证据：`experiments/results/impact_p12/gate_20260827T165121Z_879`；13 项自动检查
  全部 PASS，动态检测/重规划延迟 `0.1325 / 0.2100 s`。
- 障碍离开后动态残留 `4.025 s`，`5.25 s` 完成 TTL 清除与连续空闲确认并发布重开事件；
  动态体素峰值 55，重规划事件恰为 brake/reopen 两次。
- 四个滚动 batch 全部完成，净前进 `23.985103 m`、真值路径 `24.505402 m`、ATE RMS
  `0.075913 m`、最低物理净空 `3.637212 m`。
- 静态结构保留率 `1.007018`；算法节点无 Ground Truth 订阅；外部 Gazebo 地图/模型
  逐字节隔离审计 PASS。
- 可视化原件：同一 PASS 轮次含 277 MB 压缩 rosbag 与 1.8 MB Gazebo state；
  `scripts/view_p12_combined.sh` 同时打开 Gazebo 和 RViz，开顶且保留墙体。
- 93/93 自研自主算法测试（含 10/10 P12 定向测试）、13 包隔离构建、SDF 验证、正式飞行 Gate 和录制回放通过。
- 边界：单刚体动态障碍、固定室内长走廊 Gazebo SIL；不代表多目标语义跟踪、Atlas
  满载、HIL 或实机安全保证。

## P13 正式结果

- 正式证据：`experiments/results/impact_p13/gate_20260828T055346Z_1323`；9 项配对 Gate
  全部 PASS，两轮 P13 trial 和两轮 P12 保持性检查均通过。
- 50/200 ms profile 的 planner p50 为 `50.10 / 200.24 ms`，端到端 p99 为
  `150.51 / 301.29 ms`，差值 `150.79 ms`。
- 高时延未缓解 AL 从 `0.16773` 降至 `0.07714 m`、Margin 降至 `-0.02286 m`；降速
  后速度上限 `0.14500 m/s`、AL `0.16000 m`、Margin 恢复到 `0.06000 m`。
- 两轮净前进 `23.99 / 23.96 m`，ATE RMS `0.084 / 0.079 m`；高时延任务时间由
  `132.4 s` 增至 `221.4 s`。
- 外部资产字节级未变；Gazebo+RViz 高时延正式录制双窗口已启动验证，开顶并保留墙体。
- 98/98 自研自主算法测试（含 5/5 P13 定向测试）和 13 包隔离构建通过。
- 边界：固定 Gazebo SIL 和软件规划负载；Atlas 满载、组合故障、HIL 与实机属于后续阶段。

## P14 正式结果

- 正式证据：`experiments/results/impact_p14/gate_20260828T072511Z_12130`；统一 Gate 的
  matrix、persistent-LiDAR emergency、P12/P13 保持性、同世界与隔离检查全部 PASS。
- 十类故障均实际作用于代理或处理链；状态映射覆盖 NORMAL、CAUTIOUS、RECOVERY、
  BRAKE、RETURN，20% 丢包窗为 30 发/6 丢。
- 矩阵轮次净前进 `23.9495 m`、ATE RMS `0.08834 m`；P12 动态检测/重规划为
  `0.1575/0.0625 s`，P13 端到端 p99 为 `298.22 ms`、安全 Margin `0.0600 m`。
- 持续 LiDAR 中断实际依次进入 `NORMAL→CAUTIOUS→RECOVERY→BRAKE→HOVER→LAND`，
  高度下降 `0.5278 m`，最终速度 `0 m/s`；Ground Truth 仅供独立 evaluator。
- 102/102 自研算法测试、14 包隔离构建和 Gazebo+RViz 双窗口正式录制回放均通过；
  Gazebo 开顶并保留墙体，外部资产逐字节未变。
- 边界：固定 Gazebo SIL 与项目内故障代理；真实传感器驱动、Atlas 满载/功耗温升、
  openEuler、CUAV、HIL 和实机安全仍属于硬件部署阶段。

## 复杂场景完整正常飞行正式结果

- 正式证据：
  `experiments/results/impact_complex_comparison/gate_20260828T122049Z_319`；不启动故障
  注入，信息优先与完整性约束两航程的 P11/P12/P13 evaluator 和 17 项汇总检查均 PASS。
- 两航程均完成 4 个滚动 batch 和约 24 m 净前进；完整算法执行
  `right, right, direct, direct`，基线执行 `direct × 4`。
- 实际最低 Margin 为 `-0.466501 / +0.124017 m`，完整算法提升 `0.590518 m`；额外
  路径 `0.712621 m`，任务时间开销 `0.050 s`。
- 两航程均触发 2 次动态制动/重开；完整算法最小物理净空 `1.753211 m`，P13 端到端
  p99 `161.768 ms`，最终安全 Margin `0.061590 m`。
- GPU 渲染器为 `D3D12 (NVIDIA GeForce RTX 4060 Laptop GPU)`；场景只拆屋顶、保留
  墙体、隔墙、六层货架、门架、杂物、完整性挑战区和运动体。
- 外部 Gazebo 地图/模型前后字节级一致；核心算法节点无 Ground Truth 订阅。Git 轻量
  证据位于 `evidence/COMPLEX_DEMO/gate_20260828T122049Z_319/`，大型 rosbag 留在 WSL
  并以 SHA-256 索引。

## 双向复杂场景正式结果

- 新增独立世界 `xq_complex_bidirectional_warehouse.sdf`；第二个完整性挑战区镜像，
  精确 Gate 要求 `right, left, direct, direct`，用于排除固定右绕。
- 正式证据：
  `experiments/results/impact_complex_comparison/gate_20260828T125712Z_315`；基线为
  `direct × 4`，完整算法精确执行右/左/直/直，干预恰为 2 次。
- 实际最低 Margin 从 `-0.492817 m` 提升到 `+0.146311 m`，提升 `0.639128 m`；额外
  路径 `0.743725 m`，任务时间开销 `0.100 s`。
- 两臂净前进均约 24 m、动态重规划均为 2 次；完整算法 ATE RMS `0.066186 m`、动态
  净空 `1.906157 m`、P13 p99 `157.282 ms`、最终安全 Margin `0.064046 m`。
- revision 3 在正式双航程前冻结；所有场景改动均增加真实可观测几何或把结构移出保护
  管，未降低任何冻结阈值。外部资产隔离审计 PASS，故障注入关闭。
- Git 轻量证据位于
  `evidence/COMPLEX_DEMO/bidirectional_gate_20260828T125712Z_315/`；完整报告见
  `docs/COMPLEX_SCENE_SUITE_REPORT.md`。

## 三维复杂场景完整正常飞行正式结果

- 新增独立开顶世界 `xq_complex_3d_warehouse.sdf` 和默认关闭、仅由该场景显式启用的
  `geometry_rich_up` 候选；现有场景和其他项目地图不受影响。
- 冻结前单臂诊断 `gate_20260828T134940Z_10283` 的第三批候选中，直/右/左 Margin
  分别为 `-0.095/-0.098/-0.103 m`，UP 为 `+0.498 m`，算法自然选择 UP。
- revision 2 在首次双臂前冻结；正式 Gate
  `experiments/results/impact_complex_comparison/gate_20260828T135423Z_11540` PASS。
- 基线精确执行 `direct × 4`；完整算法精确执行 `right, left, up, direct`。实际最低
  Margin 从 `-0.509323 m` 提升到 `+0.147368 m`，提升 `+0.656691 m`。
- Ground Truth 最大垂向偏移为 `0.115599 / 0.660182 m`，证明 UP 已真实执行；两臂均
  完成约 24 m，完整算法额外路径 `1.104322 m`，任务时间反而减少 `7.4725 s`。
- 动态安全、P13 包络、ATE、Ground Truth 隔离、冻结阈值与外部资产审计均 PASS；轻量
  证据位于 `evidence/COMPLEX_DEMO/spatial_gate_20260828T135423Z_11540/`。

## 复杂组合三维完整算法正式结果

- 新增独立开顶世界 `xq_complex_compositional_warehouse.sdf`，保留墙体和内部结构；正常
  飞行栈同时覆盖 P6/P10/P11/P12/P13，不启动 P14 故障注入。
- revision 3 在独立信息优先臂 `gate_20260902T121313Z_326` 与完整性约束臂
  `gate_20260902T121632Z_422` 均通过后冻结。正式双臂 Gate
  `gate_20260902T122517Z_286` 的 26 项汇总检查全部通过。
- 信息优先精确执行 `direct × 4`；完整性约束精确执行
  `right, left, up_right, direct`，恰有 3 次完整性干预。
- 实际最低 Margin 为 `-0.078022 / +0.158314 m`，提升 `0.236336 m`；两臂净前进
  `24.007881 / 23.998857 m`，约束臂额外路径 `1.590837 m`，任务时间少 `15.4875 s`。
- 约束臂最大垂向位移 `1.052533 m`、同时横向+垂向位移 `0.801914 m`；动态障碍最小
  物理净空 `3.503246 m`，P13 p99 `186.868 ms`，均满足冻结 Gate。
- 稀疏动态地图将典型活动体素从约 5.9 万降到约 1.2–1.5 万，并加入有界实际观测信息
  记忆、端到端时延因果闭合和 15 s 动态转静态确认窗；120 项自研测试通过。
- GPU 为 `D3D12 (NVIDIA GeForce RTX 4060 Laptop GPU)`；外部资产前后字节级一致，
  Ground Truth 仅供 evaluator。轻量证据位于
  `evidence/COMPLEX_DEMO/compositional_gate_20260902T122517Z_286/`。

## P15 正式结果

- 正式证据：
  `experiments/results/impact_complex_comparison/p15_research_20260903_final_v4`；P11/P12/P13
  两臂 Gate 和比较器 20 项检查全部 PASS，两个实验臂均完成约 24 m 净前进。
- 每个滚动窗在线生成 5×2 三维 lattice；任务收益、碰撞概率、运动能耗和返航储备由
  在线地图与候选轨迹计算，不再按候选名称赋值。完整性 Margin 只作硬过滤。
- 信息优先与完整性约束臂的独立真值最低 Margin 为 `0.021936 / 0.281308 m`，提升
  `0.259372 m`；Availability 为 `0.807069 / 1.000000`，提升 `0.192931`；两臂
  HMI 和真实安全违规均为 0，PL empirical coverage 均为 1.0。
- 约束臂路径开销 `0.6701%`，任务时间开销 `-2.5993%`，ATE 绝对变化
  `+0.000739 m`；完整性硬门在第二、第三 batch 改变选择，最小干预效用带实际应用
  三次。
- 动态安全改为当前活动轨迹查询，并预声明 5 体素、0.45 m 连通支持；运行时 Margin
  guard、progress watchdog、中断能耗记账和完成/关闭/中断/已执行四类计数已纳入测试。
- 两臂运行时状态均报告 4 个完成段、4 个关闭规划窗和 4 个已执行决策；约束臂 0 次
  中断。两个 launch 日志均无 Traceback、节点死亡和 ROS `[ERROR]`。
- 140/140 Python 算法与验收契约测试、隔离构建和 GPU Gazebo 正式运行通过；renderer
  为 `D3D12 (NVIDIA GeForce RTX 4060 Laptop GPU)`。轻量证据位于 `evidence/P15/`，
  大型 rosbag 以 SHA-256 索引留在 WSL。
- 边界：固定复杂组合世界的一次配对 SIL 机制验证；多 seed、多场景、公开 baseline、
  统计显著性、Atlas HIL 与实机仍待完成。

## 2026-10-20：R2B3B1 测距观测已复核，AP 回归尚未通过

- 新模型 `xq_iris_multisensor_range_ardupilot`、新世界 `xq_r2b3_range` 加入同轴下视
  50 Hz 单束距离：0.05–10 m，带噪束声明每样本 Gaussian 标准差0.01 m，无噪参考仅供
  评价。原相机、Livox及AP物理模型保持原定义，旧world/model和旧bag不改写。新IMU
  话题保留原观测并标注SDF每样本方差（gyro 4e-8、accel 4e-6），方向协方差未知；
  这不是噪声密度或物理标定。单束距离不能直接当作全部图像特征的深度。
- Gazebo开发实测发现单样本零跨度的水平/垂直angle_step均为NaN。协议revision2仅允许
  在严格1×1、角端点均有限0的条件下把该未定义派生间隔表达为ROS增量0；每条原生
  字段、原double距离、原采集时戳及转换原因写入台账。NaN距离、无穷或非零步长等仍
  拒绝；没有重新采样距离或改时间。74项新C++测试、79项独立审计器测试通过，全套
  711项Python、93项桥接检查及14＋1包构建通过。
- 三次正式尝试全部保留。第1轮因DDS多参与者发现不足，未取得LIO输入，未起飞；
  第2轮mission、28项AP接口门和45项多源门通过，但camera存储审计早于关闭标记，
  runner停在analysis并判FAIL。之后只读复核写新目录，原FAIL没有覆盖。第3轮使用
  修正收尾流程，但实际起飞失败、FCU进入LAND，因此最新实验仍FAIL，不能回选第2
  轮冒充阶段完整通过。GPU退出清理仍有独立异常，不能写全部进程无错误退出。
- 第2轮观测的独立59项复核通过：4,188个同采集时戳距离对，残差均值+0.027579 mm，
  样本标准差10.025863 mm（设定10 mm），标称95%区间覆盖94.961796%，lag-1相关
  0.0079625。Range/native scan/原double→float32台账逐条核对，IMU原值保持，三个
  含参考信息话题在所存图快照中仅记录器订阅。此处只支持该观测段的仿真噪声复核。
- DDS最小10进程对照复现：Fast DDS默认初始发现范围4时，晚期8/9可被早期记录器看到，
  却互收0/0条；范围64后互收40/40条且无XML解析错误。新runner仅扩大发现范围，
  没有改算法或统计门。依据[Fast DDS初始参与者发现说明](https://fast-dds.docs.eprosima.com/en/v3.1.3/fastdds/use_cases/wifi/initial_peers.html)，
  未给固定端口时应覆盖预期参与者数。旧小规模回放默认范围4保持原行为。
- FCU原生BIN进一步定位风险：第2轮VISP/VISV各738条，到达最大间隔0.159946 s、
  超300 ms间隔0次；第3轮各87条、79个不同到达时间，最大空窗4.799717 s，超300 ms
  36次。ROS适配器第3轮仍有约595次输出，说明ROS端healthy不足以证明飞控收到观测。
  AP原生视觉里程计健康条件为最近300 ms收到更新。故障段位于ROS输出之后、FCU接收
  之前；尚未分清DDS/MAVROS/传输哪个环节。两轮仿真clock无回跳，成功轮也有MAVROS
  timejump警告，因此不能直接归因时间戳拒包。冻结报告位于
  `runs/research_r2b3b1_fcu_receive_diagnostic_20261020T110143Z/report.json`。
- 诊断时一次mode=ro读取在第2轮新bag旁产生12个新增WAL/SHM副文件（6个WAL均0字节，
  6个SHM各32768字节）。诊断连接关闭、原7文件SHA完全匹配后，仅清理这些新增副文件，
  原DB字节及文件集恢复并留独立日志。最终复核发现原fuser占用检查调用无效，不能据其
  返回码证明无人占用；该检查缺陷另存纠正记录，原始记录不改写。后续只用immutable或
  私有副本。旧论文保全与图集未受此事影响。
- 本阶段必须以INCOMPLETE封存，包含4次构建、3次正式轮、所有开发/失败/复核与可视化。
  实际光流状态更新仍0，未证明新共同状态融合、AP全任务稳定性、Orin/X25或实机有效性。
  下一步先做不解锁的分段通信诊断，核对MAVROS输入、MAVLink发出和FCU实际收到的
  频率/时间，再制定FCU端健康启动门；修复后才重做飞行并连接非线性光流更新。
- 已按上述边界封存于
  `evidence/RESEARCH_R2B3B1/r2b3b1_suite_20261020T111324Z_12542/`：720个文件、
  18个大型原始文件SHA索引、19个构建/正式/开发目录。归档完整性PASS，实验状态仍为
  `R2B3B1_INCOMPLETE_AP_REGRESSION_FAILED`；manifest SHA-256为
  `5e4df511145b292015d8e9280ce4cab48eb026af4aa67a46822447268c13be3b`。
  既有论文基线、R2B3B0证据及原图集在封存前后复核通过。

## 2026-10-20：R2B3B2 未解锁通信分段诊断

- 目标是区分adapter发布、MAVROS内部话题、TCP线上ODOMETRY及FCU原生接收，
  本阶段没有解锁、起飞或控制任务。旧R2B3B1仍为INCOMPLETE，真实光流状态更新仍0。
  不启动旧base launch（它含自动任务节点），专用runner仅用原bridge/adapter、可选
  FAST-LIO和shadow传感器；关闭GT发布。LOG_DISARMED仅写入本轮运行参数。
- 本机MAVROS 2.14.0的飞控通路是TCP，而UAS与router间的ROS话题使用BEST_EFFORT。
  依据[UAS官方源码](https://github.com/mavlink/mavros/blob/2.14.0/mavros/src/lib/mavros_uas.cpp)
  和[router官方源码](https://github.com/mavlink/mavros/blob/2.14.0/mavros/src/lib/mavros_router.cpp)，
  新观测加入`/uas1/mavlink_sink`、`/uas1/mavlink_source`及透明TCP字节代理；观测器
  收到话题不能证明另一个订阅者回调已执行。代理不自主发MAVLink或重编码，记录双向
  字节数/SHA，并阻断飞行/执行器/参数写入及未列明只读命令。
- 新协议`config/research_extnav_link_probe.json`保留FCU原生boot时间严格<300ms的
  到达间隔门，同时要求两路VIS日志足量、Ign=0、质量达标及未解锁。用adapter输出
  `stamp_ns//1000`、wire `time_usec`与FCU `RTimeUS`对齐；源采集戳另存，不混淆
  wall、monotonic、Gazebo和AP boot时间，也不跨未校准时钟计算端到端延迟。
- 第1轮`r2b3b2_synthetic_lean_20261020T132719Z_22810`使用10Hz恒定诊断位姿，经原
  adapter/MAVROS送入AP，属于通信输入而非估计器成果。原runner/link/proxy全部FAIL。
  冻结后的独立只读复核显示整轮wire 872条、VISP/VISV各872条，RTimeUS逐条同序；
  稳态787个标识在adapter、UAS sink、wire和FCU两路全部对应。AP boot最大到达间隔
  0.104965s，Ign均0、Q均0、DSF拒写计数0、无ARM记录。
- 第1轮没有请求FCU数据流，缺少SYSTEM_TIME窗口边界，原分析器窗口内FCU计数0是
  边界缺失后的空选集，不能解释为飞控零接收。192字节BAD_DATA均为SITL启动文字；
  收尾先停SITL还产生人为TCP reset。原失败保持，独立复核报告另存于
  `runs/research_r2b3b2_first_probe_diagnostic_20261020T133815Z/report.json`，60个原始
  文件集合及SHA前后相同。
- 协议revision2在下一轮之前明确记录修正：只读STREAM_ALL=20Hz（旧mission本来就
  请求该频率）、严格限定的首有效FCU帧前启动ASCII分类、先排空代理再停两端。
  增加SYSTEM_TIME边界接近host窗口的要求；连续性PASS仅限实际观测的FCU boot
  子窗口，输出未覆盖的host首尾，不把中途停止后的无消息长尾删掉。没有放宽300ms
  或时间戳严格递增门。新增19项真实TCP测试、9项时间边界/配对测试全部通过，记录
  位于`runs/research_r2b3b2_tools_20261020T134359Z_329/`。
- 第2轮`r2b3b2_lio_full_20261020T134400Z_345`使用真实FAST-LIO、原B1多传感器世界、
  shadow与录包，无mission/execution节点。80.000554s host稳态窗口对应49.999s的
  SYSTEM_TIME界定FCU boot子窗口，未覆盖host头尾0.007107/0.018128s。观察到500条
  源里程计；adapter、sink、wire、VISP和VISV的500个传输时间标识完全对应（源采集戳
  属于另一时间域，不能直接等同传输戳）；两路FCU到达最大间隔（含头尾）
  0.142950s，连续性子门通过，Ign均0、Q均0。整轮两路各552条，未发生ARM。
- 第2轮整体仍FAIL：adapter/wire时间戳出现一次0.307135s回退，且启动文字末尾
  `AP_IRLock_SITL::init()`在首有效MAVLink帧之后出现，按保守分类策略未放行，代理
  完整分类门失败；双向已转发字节/SHA仍完全一致。不能把这个FAIL写成全链路PASS。
- 两轮都观察到host系统wall clock回退，源采集戳和Gazebo时钟保持前进。第1轮输出戳
  三次回退0.464856/0.401650/0.425163s，第2轮一次回退0.307135s。观察器wall同步
  回退，而monotonic前进；首轮journal也有clock change记录，具体校时来源仍未知。
  没有改WSL或主机校时服务。恒定输入轮仍全部抵达，不能把回退直接认定为旧B1
  4.80s空窗的原因，也不能因本轮未复现就认定旧故障消失。
- 结论限于两次未解锁通路观测及可复现的时间域问题；没有证明MAVROS内部DDS是旧
  故障原因、EKF确实融合、飞行稳定、Orin/X25或真实光流融合有效。下一步先实现可审计
  的clock-domain guard与FCU侧持续就绪证据；源戳不前进/时钟跳变不得被改戳包装成
  新鲜健康数据。稳定时基映射需要单独对照验证，之后才安排新的完整AP飞行回归。
- 两轮独立对照及诊断图已冻结于
  `runs/research_r2b3b2_comparison_20261020T135035Z/`：完整wire/VISP/VISV逐条有序匹配
  分别872/552条，原生FCU全记录段最大间隔104.965/144.956ms；原60/92个文件集合、
  大小及SHA复核前后不变。全记录段内部间隔不覆盖未观测首尾；不能与第2轮含边界的
  稳态子窗口统计混为一谈。GPU退出异常和严格代理分类FAIL也保留于原shutdown记录，
  没有宣称所有进程正常退出。
- 已封存于`evidence/RESEARCH_R2B3B2/r2b3b2_suite_20261020T140147Z_337/`：198个证据
  文件、4个大型原始DB文件索引、两次原始FAIL。manifest SHA-256为
  `35d5008769564ed68f0b28de7a23f3918a86178775429379e83b3d5749a09ad6`。归档完整性
  PASS；完整论文基线、B1/B0/旧图集在封存前后复核PASS，实验本身没有提升为PASS。

## 2026-10-20：R2B3B3 时钟保护与飞控采样就绪诊断

- 新协议 `config/research_extnav_clock_guard.json` 在两次实测前冻结。原P4适配器加入
  默认启用的时钟保护：采集戳、传输wall戳及host monotonic必须前进；wall与monotonic
  的相邻残差/相对初始偏移超过50ms时锁存故障，拒绝触发帧及后续输出，重建节点才能
  清除。没有夹紧、改写采集戳或自动恢复。传输时钟采样用monotonic前后包围，跨度超过
  5ms仅标为采样不确定并停止该次输出，不能冒称已证明wall跳变。50ms/5ms是本轮预先
  声明的工程预算，并非硬件标定或传感器到FCU时间误差界。
- 新鲜度使用host monotonic接收年龄，350ms过期；本地就绪要求至少4Hz及连续5s有效。
  新只读监视器另外要求FCU SYS_STATUS的VISION_POSITION present/enabled/health均真、
  最近样本不超过350ms、连续健康5s、连接且未解锁、参数证据有效。每个SYS_STATUS
  验证实际system/component=1/1；参数同时核对只读MAVROS GetParameters、线上逐名
  PARAM_VALUE与FCU BIN。`PLND_ENABLED=0`排除精确降落模块共用健康位的干扰，
  `VISO_TYPE=2`与`LOG_DISARMED=1`同时核实。依据
  [ArduPilot GCS源码](https://github.com/ArduPilot/ardupilot/blob/Copter-4.5.7/libraries/GCS_MAVLink/GCS.cpp)
  和[MAVLink SYS_STATUS定义](https://mavlink.io/en/messages/common.html#SYS_STATUS)，这只是
  采样的视觉里程计后端收件健康，不是逐帧ACK、EKF融合证明或飞行许可。监视器不连接
  ARM/TAKEOFF服务；旧mission在VERIFY_NAV后缺少连续复查的问题尚未集成修复。
- 离线回放位于 `runs/research_r2b3b3_clock_replay_20261020T142452Z/`：使用B2观察器
  共同采样的原始wall/monotonic，采集输入只用明确标注的合成ordinal，不重构真实LIO
  采集时序。LIO首次故障索引184，阻断364/548条；synthetic索引204，阻断668/872条，
  均包含触发帧。保留相同monotonic的无跳变合成正对照全部转发，7种受控故障及超时
  检查通过。epoch＋monotonic仅作离线算术对照，没有部署到MAVROS/AP。
- 第一轮 `r2b3b3_synthetic_lean_20261020T145204Z_272` 完成90s观察及102s只读监视。
  第142次源观察时传输wall相邻回退323.312310ms，monotonic前进101.204634ms，残差
  为-424.516944ms，触发保护。线上ODOMETRY、VISP、VISV各141条有序对应；触发后
  output_count固定141，1466条节点后缀healthy/clock_ready均假，859条监视后缀ready
  均假。首个故障后不健康FCU监视样本约245.598ms后出现，仅是采样观测时间差。
  70项布尔门全真，时钟保护子门PASS；但自然故障先于20s/2s预声明暂停，暂停恢复门
  PREEMPTED，整体分析INCOMPLETE，原runner为FAIL/exit1，均原样保留。
- 第二轮 `r2b3b3_lio_full_20261020T145525Z_899` 使用FAST-LIO、原测距/相机/光流
  shadow及录包，不启任务/执行节点。70项预声明布尔门均通过，诊断/runner均PASS；
  自然时钟异常触发后output_count固定268，节点和监视就绪保持假，末端FCU健康为假。
  这里PASS仅表示规定的未解锁诊断成立，不能称时钟已修复、飞行可用或全链路无丢失。
  adapter观察器确有268个唯一递增传输戳，MAVROS sink、wire、VISP、VISV均261条；
  后261个adapter戳与这些通路有序一致，差额恰是最早7个adapter输出。TCP代理覆盖
  启动段并确无这7条，但独立sink/source观察分支首事件晚于这7条，不能据其空集证明
  插件未发或定位在sink之前。缺口仅能界定在adapter输出到TCP之间，具体DDS发现/
  回调原因未知，日志也没有TF错误证据；不能用它解释B1中途4.80s空窗。Q均0/未知，
  Ign均0，不证明EKF实际融合。
- 两轮都曾观测到只读ready=true，但时钟故障后均撤销且最终false；不能据ever_ready
  授权解锁。两轮owned进程组最终都关闭，但Gazebo需强制终止，原shutdown=ISSUES
  保留，不能称清理全部正常。代理保留原字节及SHA，只新增精确、单次、10s内的
  `AP_IRLock_SITL::init()`启动行分类；其它未分类字节/CRC错误仍拒绝。
- 当前全套Python回归 **999项PASS**，含81项保护模块、103项节点/保护集成测试，
  以及新增30项暂停/收尾边界和17项分析器证据边界测试（均包含在999项中，部分核心/
  集成集合还互相包含，不能重复相加）。最新14＋1包构建与593项package回归PASS，冻结源与实际安装适配器/
  guard的SHA逐一相同。952项早期全回归与两次构建结果也保留，不覆盖旧版本记录。
- 新诊断图位于 `runs/research_r2b3b3_visuals_20261020T150008Z/`，提供PNG/PDF/SVG、
  绘图数组及输入SHA；FCU健康使用离散采样点，避免把遥测空缺画成连续健康。离线
  回放和实测时间线分开，不作为算法精度或优越性图。完整研究创新仍待实际共同状态
  光流更新、相关项消融及预算约束策略对照；本轮实际光流状态更新仍0。
- 下一步先验证稳定时基映射及启动段7帧缺口，再补不被时钟故障混淆的暂停恢复对照，
  集成持续就绪复查后才重做AP完整飞行回归。B1仍INCOMPLETE、B2两次FAIL保持；
  Orin NX、CUAV X25 MEGA及真实传感器验证尚未开展，硬件状态仍NOT_READY。
- 已封存至 `evidence/RESEARCH_R2B3B3/r2b3b3_suite_20261020T150451Z_320/`：339个文件、
  2次实测、2次构建、5个原始大文件/DB索引，包含全部开发、失败、独立复核和图。
  manifest SHA-256为 `251a07aef59595e2eaa278778ca8f70588ca53e8790aa40e3f1b13f886af2373`。
  归档完整性PASS；输入文件集合/大小/SHA及旧论文、B2/B1/B0/图集在封存前后复核PASS。
  归档内文档为封存时快照，本条路径与SHA在封存后追加，不覆盖归档。
- 独立最终QA **51/51项PASS**：339文件完整集合/大小/SHA、两轮各自原状态、前后
  保全报告及5个原始大文件实际SHA均核实。报告在
  `runs/r2b3b3_final_qa_20261020T150946Z/report.json`，只写新QA目录，不改封存文件。

## 2026-10-21：R2B3B4 离线传输时基与启动契约

- 本轮专用研究只做纯软件、原记录时钟/图快照回放，以及原版AP时间修正C++的隔离运行；
  这些专用研究不启动ROS节点。全套回归另含既有domain211隔离ROS节点单测，执行功能
  禁用并断言发布控制数0；没有启动AP/MAVROS/Gazebo，没有SQLite连接、宿主校时、候选时间戳发送、
  固件烧写或新硬件部署。原P4适配器和B3 raw-wall保护默认行为未改，新模块未集成。
  主协议 `config/research_extnav_transport_study.json` 初版在隔离研究执行前冻结；scope
  revision2仅更正上述全回归/专用研究的范围措辞，原绑定副本不变。启动门
  审查后另立revision2协议 `config/research_extnav_startup_gate.json`，不覆盖早期副本。
- 新 `ExtnavTransportClock` 用首个≤5ms可信wall/monotonic括号固定epoch，之后只加
  monotonic经过时间；原wall及回退/漂移完整保留，不随校时重置anchor、不clamp或+1。
  单次采样跨度>5ms只拒绝该观察；时钟顺序错误、映射越界及微秒量化重复永久锁存。
  它不接收源位姿/采集戳，因此单独不能证明源新鲜度；必须与原采集戳/接收年龄保护
  共同使用。62项模块测试通过，仍非UTC、传感器采集或FCU时钟标定。
- 依据[MAVROS 2.14.0出站源码](https://github.com/mavlink/mavros/blob/2.14.0/mavros_extras/src/plugins/odom.cpp#L316)，
  ODOMETRY出站直接把header纳秒截为微秒，不应用UAS入站timesync offset。依据
  [AP固定版本源码](https://github.com/ArduPilot/ardupilot/blob/b7207551/libraries/GCS_MAVLink/GCS_Common.cpp#L3686)，
  接收侧另用JitterCorrection估计boot时间；它的偏移重估/500ms滞后限制以及EKF后续
  延迟/IMU缓冲处理，都不能证明观测采集时刻已校准。8个本机AP源与官方b7207551原文
  字节相同；MAVROS依据已装2.14.0对应官方tag、本机headers及包校验，未冒称有本机cpp。
- 冻结B3节点状态中的sampling三元组，按原序去重并保留每条源事件/行号及原括号。
  synthetic有864个观测括号，raw guard只准许141个，映射算术准许864个；LIO有527个，
  分别267/527个。两组候选ns和floor(ns/1000)均严格递增。这里源token是明确标注的
  synthetic ordinal，不能称重构真实LIO采集时序、AP实际发送量或新鲜度改善；LIO原
  节点268次输出不等于回放267次raw许可，观察缺失没有补齐。10类受控夹具、无跳变
  正对照及超时检查通过；重复/冻结源仍拒绝。脚本13项单测通过。完整结果在
  `runs/research_r2b3b4_clock_replay_20261021T064309Z_512/`。
- 使用逐字节复制的原 `JitterCorrection.cpp/.h`，只给无HAL调用的算法提供cstdint
  stub；static存储确保初始状态清零，每个分支单独进程。6类各240步、raw/mapped
  两分支共2880次确定性调用与独立整数参考逐条一致，41检查PASS。这不是AP运行、
  EKF或传感器实测：固定80ms延迟夹具可得到“arrival−corrected=0”，但相对已知合成
  采集时刻仍误差80ms；冻结采集点时该误差可增长到23.905s而前者仍0。故不能以
  校正时间落界、Ign=0或SYS健康替代采集时间标定。结果在
  `runs/research_r2b3b4_ap_time_study_20261021T063654Z/`；图/数组在
  `runs/research_r2b3b4_visuals_20261021T064709Z/`，已检查PNG布局。
- 新启动门匹配真实MAVROS `odometry`、namespace `/uav1/mavros`、Odometry消息类型
  及RELIABLE/VOLATILE；观察器/录包器订阅不计为MAVROS准备。首次54项测试和11项
  回放虽PASS，但审查发现旧逻辑仅看两次查询的经过时间，远隔两点也可能ready，
  不足以支持有界查询间隔的采样可见性。这些原PASS报告没有改写。revision2追加
  默认250ms、最多350ms查询间隔门，超限即同GID也清窗；1s内需连续受控查询，
  仍不证明采样之间实际存在、回调执行或FCU收到。修正版71项单测通过；新回放
  13检查/79条合成查询通过，真实before/after快照各用新gate、均不推断1s可见。
  修正回放 `runs/research_r2b3b4_startup_replay_20261021T065538Z/`；旧/新相同稀疏
  查询的直接反例及新密集查询对照6检查通过，独立保留在
  `runs/research_r2b3b4_startup_revision_audit_20261021T065800Z/`。原7条启动缺口仍未修复。
- 全套源码回归 **1145项PASS**；559个src/scripts/config及驱动文件完整快照，集合/
  大小/SHA前后相同。执行的是canonical源码，未替换WSL已安装软件/构建清单。
  结果 `runs/research_r2b3b4_regression_20261021T065452Z_530/`。阶段before保全核实
  B3精确339文件和旧B2/B1/B0/图集、完整论文基线均PASS；后续封存再做前后复核。
- 原回归summary中的 `AP_or_ROS_nodes_started=false` 元数据过宽：真实已有单测
  `test_ap_execution_node.py`在domain211创建/旋转一个禁用执行节点，且断言控制发布0。
  1145项通过和运行内源码SHA不受影响，原summary与源码快照不重写；当前驱动元数据
  已修正，不为此重跑算法。第一次未完成封存因补充范围更正由root停止，原部分文件
  保留，新最终封存另建目录，scope更正及中止记录随新证据保留。
- 最终封存 `evidence/RESEARCH_R2B3B4/r2b3b4_offline_suite_20261021T070702Z_434/`：
  749个文件、11个离线来源目录，manifest SHA-256
  `6687b1dc87866741501a2d35487f83f8fd4fd8a5fd19e1bd51af849290a05213`。完整性PASS，
  原/新启动门、稀疏查询缺陷、范围更正、1145回归及图均保留；源文件集/大小/SHA及
  论文/B3/B2/B1/B0/图集在封存前后复核PASS。没有新增AP实测、bag或正式部署。
  本条路径/SHA为封存后后记，未修改归档内文档或原报告。
- 最终独立QA **29/29项PASS**：749文件集合/大小/SHA前后相同，559源码快照、
  1145回归、原/新启动门及scope更正、6份前后保全记录均核实；报告
  `runs/r2b3b4_final_qa_20261021T071454Z_38952/report.json`。只写新QA目录，不改归档。

## 2026-10-21：距离硬件部署的实际缺口

当前是可复现的软件研究原型，hardware_readiness仍NOT_READY；不能按单测数量估算
实飞完成比例。无桨上板采集/运算可以与软件研发并行，闭环飞行需要后面的门通过。

| 工作 | 已有基础 | 仍需完成的验收 |
|---|---|---|
| 实际共同状态融合 | 联合相关模型、几何和噪声的离线验证 | 将光流/尺度/共享IMU相关项接入真实非线性状态更新，保留每次状态/协方差变化；实际flow状态更新当前0；做融合/相关项/退化场景消融 |
| AP端到端与任务 | FAST-LIO→ExternalNav通路、分段未解锁诊断、原时钟保护 | 候选时基和精确启动门的预声明线上验证，源→发布→wire→FCU逐条台账、7条启动缺口与暂停恢复；ARM/TAKEOFF前持续复查；最新失败的完整AP任务重新通过 |
| 传感器与标定 | Mid-360确定，已有仿真输入和来源契约 | 前视相机/微空具体型号及接口仍未选定；真实驱动、轴/单位、内外参、杆臂、时间偏移/漂移、光流旋转补偿/尺度/有效高度实测 |
| Orin NX | WSL软件/编译基础 | 在实际JetPack/驱动/ROS组合运行；测满载时延分布、丢帧、带宽、CPU/GPU/内存和热稳态，保证规定截止期 |
| CUAV X25 MEGA | 飞控型号确定，继续AP | 读取实际板卡/固件身份、hash及完整参数，核对该版本ExternalNav与端口/坐标/时间语义；同一光流若已进入ExternalNav，避免未经相关性处理再作为AP独立观测 |
| 上机验证 | 只有计划，未开展本轮真实硬件测试 | 无桨地面采集和受控输入、HIL/板上联调、断流/重启/Orin失联与切换；再逐步做受控低速飞行和退化场景验证 |

近期顺序：完成B4封存；单独声明下一轮未解锁线上协议，同时开始准备Orin无桨采集
与实际传感器标定；并行推进非线性共同状态融合。AP全任务与地面/HIL验收之前不提升
为实飞就绪。相机/光流候选仅为规划，当前没有新采购或已验证硬件兼容性的结论。

## 2026-10-21：先完成所有非硬件工作，固定 AP 4.7.1

用户本轮范围为硬件以外的软件/仿真完成，固件目标明确为官方 ArduCopter 4.7.1。
此前“可并行上板”是旧建议，不是本轮执行项；本轮不开展硬件采购、刷写或实际飞行。
验收矩阵见 `docs/SOFTWARE_COMPLETION_ACCEPTANCE.md`，过程与失败记录见
`docs/SOFTWARE_COMPLETION_LOG.md`。

28维实际融合核心313项数学/行为检查PASS，独立joint_livo构建与旧前缀保全PASS；
真实回放首次因15Hz相机与10Hz瞬时LiDAR没有逐图新点云而FAIL，原始失败已保留。
正在修复无新scan时的真实IMU/VIO处理、传播起点未初始化及空cloud deskew历史残留，
重新编译并用真实ImuProcess行为fixture验证。尚不能把接线/单测PASS写成实际融合完成。

AP4.7.1官方源码commit `dbe792162d06cab66c3475fd5556bf7a120f119e` 独立编译PASS，
无解锁完整参数/version smoke PASS，两次前序失败保留。新版本参数迁移含
`VISO_TYPE=1`、`GPS1_TYPE/GPS2_TYPE`、`SIM_GPS1_ENABLE/SIM_GPS2_ENABLE`、
`ARMING_SKIPCHK=0`。稳定时基/精确订阅启动门/持续FCU就绪及完整任务仍分别验收，
不继承4.5.7旧PASS。

## 2026-10-21：实际融合和首篇初稿交付

上述首次融合失败之后，原生 IMU 初始化与空点云处理已修复，并通过真实库行为24项检查。
同输入五组真实非线性回放和独立审计通过；Full 在原生状态/协方差产生37次非零更新。
Joint/no-flow 与 Full 的位置 RMS 为7.203/6.645 mm，两项相关消融为6.768/6.742 mm。
这是单段录制仿真的描述性结果，不代表多场景统计优越性或已校准完整性。

AP4.7.1新自治软件769项、版本工具32项及ROS夹具8项检查通过；两组无解锁线上
试验分别配对557/585条 source→wire→FCU，暂停恢复就绪门有实际验证。未用新融合
进行完整AP任务；旧满载缺口/中断也不能由精简试验宣布修复。

用户将当前优先级转为CMICA2026 Winter初稿。交付位于 `output/pdf/`，含英文正文、
全部数学附录及可编辑LaTeX，作者信息暂留空。完整软件目标仍未全部完成：强串联基线
实际接入、多场景/多seed退化、新状态保护级校准和AP4.7.1完整闭环任务仍待验证。
历史论文、B0–B4及原图保全再检PASS，见 `docs/SOFTWARE_COMPLETION_LOG.md`。

## 2026-10-22：服务器迁移与复现包

服务器目标按用户确认固定为 Ubuntu22.04/x86_64，ROS2 Humble、Gazebo Harmonic、AP4.7.1。
仓库入口统一为 `server/`，原 README 保留于 `docs/README_LEGACY_P0_P15.md`；
中英文 PDF/可编辑 LaTeX 位于 `papers/`，新硬件 Demo 采集计划位于 `docs/HARDWARE_DEMO_PLAN.md`。

在 WSL 的全新 staging/runtime 路径实际完成六项离线研究、五组原始输入 LIVO 回放与独立评估、
14 包基础构建与773项 Python 测试、30秒基础无界面 smoke、AP4.7.1官方源码重新构建、
无解锁参数/version smoke与 Gazebo 插件重建。具体失败、修正、二进制身份与范围在 `server/` 记录。
这些是重定位复现验证，不代表已经在用户外部服务器或实机执行。

Git 只存源码、参数、文稿与轻证据；Release资源保存 B1 全部分卷、固定 Sophus/vikit、
独立AP4.7.1 Git递归子模块、原Gazebo插件/模型快照，另提供历史研究封存。
下载器逐分卷及整包核对 SHA-256，解包后再逐文件校验，不覆盖既有未验证数据。
源码与原型的静态身份同时明确记录原始字节和 Git/Linux LF 字节，不接受任意归一化。

新融合与历史规划/AP完整闭环、强串联基线实际接入、多场景退化/多seed统计、保护水平校准、
Orin实时性和实际标定仍单独验收；本次可移植封包不提升这些未完成项为PASS。

高级入口最终改用与基础 smoke 相同的本机 FastDDS builtin SHM/UDP；前序自增 loopback UDP
配置失败已保留。新 p15_lattice/seed501 的30秒 SMOKE 实际 PASS：LiDAR231、IMU5928、
原生FAST-LIO里程计296、完好性296、静态地图148、决策54、状态30。完整任务仍未通过，
未连接AP或新joint；结果和启动、退出、重复stamp边界见 server/advanced_simulation_validation.json。
