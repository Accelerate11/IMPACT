# Codex 执行计划

本计划以 `IMPACT - XH-202629 — Codex 仿真与算法执行企划` 为主约束，
采用逐阶段 Gate，不把“节点启动”当作“算法有效”。

| 阶段 | 交付 | Gate |
|---|---|---|
| P0 | 仓库/环境审计、独立构建 | 5 包构建成功，现有测试通过 |
| P1 | ArduPilot + Gazebo + MAVROS 单机闭环 | 10 分钟稳定，ARM→起飞→悬停→降落，日志齐全 |
| P2 | Mid-360 数据契约 | `/livox/lidar`、`/livox/imu` 字段/频率/时间戳/TF 正确 |
| P3 | FAST-LIO2 基线 | odom/path/cloud 正常，ATE 与资源占用可计算 |
| P4 | External Navigation 闭环 | GPS 关闭，LIO→MAVROS→EKF3 完成矩形飞行与降落 |
| P5 | Baseline Map + Frontier + EGO | 建图、前沿和规划基线闭环可重复 |
| P6 | IMPACT Innovation 1 | 退化/可观测性状态与规划代价增量有效 |
| P7-P9 | Protection Level、Alert Limit、Integrity Margin | 标定、告警与完整性裕度 Gate |
| P10 | Minimum-Excitation Active Perception | 三臂 Gazebo+FAST-LIO 对比、实际 Margin 恢复、代价有界与可视化 Gate；PASS |
| P11 | Integrity-Constrained Exploration | 24 m 四批滚动硬认证、双臂 Gazebo+FAST-LIO、实际 Margin 储备、代价有界、Gazebo/RViz Gate；PASS |
| P12 | Dynamic Obstacle / Dynamic Map | 12 m LiDAR 动态检测、4 m 制动前视、TTL 重开、24 m Gazebo+FAST-LIO、Gazebo/RViz Gate；PASS |
| P13 | Latency-Aware Safety | 50/200 ms 双轮次、真实时间链、p99→AL→速度、P12 保持性、Gazebo/RViz Gate；PASS |
| P14 | 故障注入 | 十类确定性故障、状态机降级、持续 LiDAR LAND、P12/P13 保持性、Gazebo/RViz Gate；PASS |
| 复杂正常飞行套件 | 三个独立开顶仓库世界 | 同向、双向与三维 `right→left→up→direct` 冻结双臂 Gate；无故障注入；PASS |

当前原则：P0/P1 代码和证据先合格；P2 起每次只引入一个变量，并固定随机
种子、场景、初始位姿、目标点和故障时序。所有实验保存在时间戳目录，至少
包含参数快照、rosbag、原始日志、指标 JSON 和外部资产隔离审计。P14 已完成旧链路 SIL
验收；新多源研究链仍须完成下述融合与 AP 整任务仿真，再进入对应硬件验证。
正常飞行可视化使用 `run_complex_3d_live_visualization.sh`，不启动
P14 故障注入。研究迭代按下述 2026-10-18 授权推进，不自动提交或推送代码。

## 2026-10-18：研究迭代与分步验证

用户已授权使用现有 WSL 环境进行算法迭代和仿真，每完成一步汇报结果并检索/复核相关研究。
研究平台调整为 Livox 激光雷达、相机、下视光流、IMU 和 NVIDIA Orin NX，继续使用
ArduPilot。新增传感器、Orin 与实飞结论必须有新证据，不沿用原 Atlas/openEuler 目标平台的描述。

Windows 源码主副本为 `D:\CODEX\impact`；新增 WSL 研究副本为
`/home/accelerate/impact_research_ws`。不调用会镜像并清理旧副本的 `sync_to_wsl.ps1`，
不复用旧副本的构建/安装树。AP 二进制、默认参数和 Gazebo 插件只读复用；SITL 运行时文件
必须进入新研究副本的当轮结果目录。

| 阶段 | 目标与验收边界 |
|---|---|
| R0 | 独立构建、构建新鲜度检查、现有 Python/C++ 回归与 P4 GPS-off AP 动力学飞行复验；不等同 P15/AP 集成 |
| R1 | 短航段接口 SIL PASS：195 项测试、正常/重放 28/33 项检查；带时间与序号的 AP 速度执行、禁用确认交接；GPU 退出清理有独立环境问题 |
| R2 | R2A：多源仿真采集/角光流旁路契约；R2B：真实驱动、时间/外参/尺度标定与共同状态融合基线。硬件未就绪不记作整体 PASS |
| R2B1 | 软件前置阶段 PASS：Micolink 原始接收/回放与共享尺度 Gaussian 乘积矩，371 测试、PTY 和 720,000 合成窗口；硬件/共同状态融合仍未通过 |
| R2B2 | 离线共同状态 LIVO 基线 PASS：408 测试、424 个完整配对时刻、422 次真实视觉校正；完整接收/逐帧处理与姿态/协方差独立重算通过；不等于光流融合、实机或新 AP 飞行 |
| R2B3A | 光流接入前置验证 PASS：528 测试、43,843 条原图轨迹和 360,000 次冻结合成试验；未知尺度/噪声/跨时刻协方差必须阻止实际融合，状态更新为 0 |
| R2B3B0 | 软件前置 PASS：632 测试、36,000 条独立四窗线性轨迹、72 次批量条件解核对；给定参数的旋转/两视图几何通过，未接真实录制光流或非线性 LIVO |
| R2B3B1 | INCOMPLETE：测距观测段59项复核通过，711 Python/93桥接检查通过；最新AP起飞失败，FCU里程计接收存在4.80s空窗；720文件及18大型原始文件索引已封存于 `evidence/RESEARCH_R2B3B1/r2b3b1_suite_20261020T111324Z_12542/`，禁止以早一轮成功替代最新失败 |
| R2B3B2 | 未解锁通信诊断完成，整体仍FAIL：恒定输入条件wire整轮872条、LIO条件wire整轮552条与FCU两路对应；LIO稳态500个transport key匹配、FCU最大gap143ms；发现host墙钟/输出戳回退，未解释旧4.80s空窗，28项新工具测试通过。198文件及4大型DB索引封存于 `evidence/RESEARCH_R2B3B2/r2b3b2_suite_20261020T140147Z_337/` |
| R2B3B | 进行中：先处理时间域跳变并建立FCU端持续健康门，再复验完整AP回归、连接非线性两时刻共同状态与原始像素观测。未解锁通路观测不能替代起飞/任务闭环 |
| R3 | 任务相关方向误差界、整段轨迹校准和最小干预主动感知，固定协议做多地图配对及消融 |
| R4 | Orin 计算在环、热稳态及真实飞控接口；量测端到端尾部时延、跟踪和失效处理 |
| R5 | 受控实飞与未见场所外部验证，按整任务报告效应及置信区间 |

每阶段报告包括：研究问题、检索与版本核对、实际修改、实验配置和原始证据、通过/失败、
证明边界、下一步。失败实验保留，不通过降低阈值将其改写为成功。

R0 入口为 `bash scripts/run_research_baseline.sh`；启动前使用
`python3 scripts/check_build_manifest.py --workspace-root <当前工作区>` 检查源码与安装树。
AP 接口以本机实际固件版本为准；4.5.7 的速度接收会按当前接收时间更新超时，因此后续
执行器必须检查上游命令生成时间，不能靠重复发送旧命令维持控制。

R1 入口为 `bash scripts/run_ap_research_gate.sh` 和同脚本的 `--replay-test`。
配置固定在 `config/research_ap_gate.json`：2 m 航段、0.2 m/s 速度上限、
0.35 s 命令年龄、0.5 s 源里程计年龄；真值净前进至少 1.65 m，ATE RMS
不超过 0.1 m、最大位置误差不超过 0.3 m。重放实验在运动后冻结新指令 0.9 s，
原始序号/生成时间不变，首次零速发布必须在 0.55 s 内；该指标不是机体实际停止时间。
GPS-off 起飞与悬停后才预热地图，地图 READY 后启动 P13/P15；仅 AP 执行器
向 MAVROS 发布速度，旧 Gazebo 直驱通道无订阅。研究完成先禁用速度、等待带序号的
执行器确认，再进行 FCU 坐标下的位置保持、LAND 与 disarm。独立真值只供 evaluator。

R1 本阶段只验证短航段 AP 动力学接口。现有 P7 标定、5×1 平面 lattice 和测得时延
保持冻结；不据此声明复杂 24 m 任务、多源融合、Orin 热稳态或统计完整性已经通过。
每轮归档源文件/参数/标定快照、构建摘要、话题图、readiness、mission/evaluation/gate
结果与 rosbag 索引。任务运行检查与仿真退出检查分别报告，退出后的 GPU 清理异常
不能被描述为任务运行中失效，也不能被归入“全部进程干净退出”。
2026-10-19 归档：`evidence/RESEARCH_R1/r1_suite_20261018T162249Z_408/summary.json`。
正常/重放真值净前进为 2.004544/2.002242 m；旧指令拒绝 43 次，首次零速发布延迟
0.349367 s。6 次成功/失败尝试和 5 轮构建测试日志全部保留，原 433 个文件不变。

版本复核来源：[Copter 4.5.7 Guided](https://github.com/ArduPilot/ardupilot/blob/Copter-4.5.7/ArduCopter/mode_guided.cpp#L496-L510)、
[MAVROS ROS 2 速度坐标转换](https://github.com/mavlink/mavros/blob/ros2/mavros/src/plugins/setpoint_velocity.cpp#L94)。
R2 融合基线参考 [FAST-LIVO2 论文 IV-D](https://arxiv.org/html/2408.14035v2#S4.SS4)
与[作者实现](https://github.com/hku-mars/FAST-LIVO2)：先核验共同状态、条件独立假设以及
共用地图/时间偏差/外参误差的影响，再设计相关性消融；不将“多传感器组合”本身作为创新。

R2A 入口为 `bash scripts/run_ap_research_gate.sh --multisensor-contract`。
冻结配置分别为 `config/research_multisensor_contract.json`（语义与依赖）和
`config/research_multisensor_gate.json`（验收阈值）。新增独立世界/机体和 Harmonic 原生
相机桥，前视/下视两路均为 320×240、15 Hz；内参、光学轴、外参均为显式仿真值。
光流只由专用下视像素及同期 Livox IMU 计算，使用前后向 LK、旋转投影补偿及有界
等待队列；无距离尺度时不发布有效米制速度。共享 IMU 的估计结果不直接按独立处理。

验收继续使用 R1 的 AP 动力学航段；附加检查真实渲染像素、原始采集时间/频率、
CameraInfo 匹配、单父节点 TF、完整陀螺窗口及源依赖。图像质量诊断固定取进入
RESEARCH 后每路第一帧，落地后的近距离画面另报；光流有效/无效计数覆盖整轮。
十项离线故障使用已采集样例进行元数据变换，检验重放/时偏/轴反射/标定/缺 IMU/
未知尺度/独立性误用的拒绝行为，不宣称已经做了真实像素故障下的融合精度实验。

本机现有 Livox 仿真是瞬时光线快照，FAST-LIO 默认路径令逐点时间为零，不是
真实 Mid-360 扫描模式；相机是瞬时曝光，也不代表真实曝光或滚动快门已标定。
ROS 回调相对缓存 `/clock` 的年龄只作诊断，可能含跨话题调度偏差，不能称作硬件
时钟同步精度。原始录包需额外验证可回读性，不能仅凭 metadata 的消息数验收。

2026-10-19 R2A 仿真旁路合同 PASS：250 项 Python、18 项 C++/退出检查，最终轮
AP/多源/原始存储分别 28/45/29 项检查及 10 个元数据故障案例通过。真值净前进
2.000978 m，850 条有效角光流均有完整 IMU 窗口。证据位于
`evidence/RESEARCH_R2/r2_suite_20261018T165613Z_320/summary.json`；三轮成败均保留。
R2 后续录制采用无压缩 SQLite、resilient、256 MiB 分段，并自动运行
`scripts/audit_multisensor_bag.py` 做私有副本的完整读取及源哈希复核；原因是首轮
触发本机 rosbag2 0.15.16 的 zstd EOF 边界问题。最终六个分段及 128,682 条消息
已通过。硬件标定、多源状态融合、AP 光流入站与统计完整性均未宣告通过。
压缩问题对应[固定版本压缩器](https://github.com/ros2/rosbag2/blob/0.15.16/rosbag2_compression_zstd/src/rosbag2_compression_zstd/zstd_compressor.cpp#L62-L92)
和[解压器](https://github.com/ros2/rosbag2/blob/0.15.16/rosbag2_compression_zstd/src/rosbag2_compression_zstd/zstd_decompressor.cpp#L60-L88)。

[AP 4.5.7 光流后端](https://github.com/ArduPilot/ardupilot/blob/Copter-4.5.7/libraries/AP_OpticalFlow/AP_OpticalFlow_MAV.cpp)
接收旧式 OPTICAL_FLOW，并使用接收时钟和 AHRS 陀螺；本轮内部图像轴归一化角流
不是 MAVLink OPTICAL_FLOW_RAD 线协议，未声明其进入 AP EKF。
[Livox 官方时间同步](https://github.com/Livox-SDK/livox_wiki_en/blob/master/source/tutorials/new_product/common/time_sync.rst)
和 [REP-103 光学轴](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst#suffix-frames)
作为真实驱动接入契约的依据。

2026-10-19 硬件选型讨论：用户确认飞控 CUAV X25 MEGA、光流厂商微空科技，具体光流
与前视相机尚未选定。`config/research_hardware_plan.json` 单独记录 MTF-01P 和大恒
MER2-160-227U3M 非 -L 的推荐候选、主源和未验证项，不是 launch 使用的硬件合同。
相机 ARMv8 SDK 不等同已验证 Orin/JetPack；实板固件未知，官方 stable 观察为
ArduCopter 4.7.2，不自动替换已冻结的 SITL 4.5.7。所有实机外参/时钟/曝光/光流尺度
继续无效，硬件 NOT_READY。

R2B 硬件按 H0 原始采集/身份、H1 时钟/几何/尺度、H2 传感器可用域、H3 共同状态/
AP 接口消融顺序验收。Micolink 系统时间不直接充当曝光终点，归一化速度不直接作为
有效米制速度；厂商 Mavlink_apm→AP 为单独基线。主链 Orin 已融合光流时，不能令 AP
再次按独立来源重复融合；独立后备还须验证脱离 Orin 的物理链路与切换。选型不阻止
软件迭代，但候选规格不计作真实驱动或统计完整性证据。

2026-10-19 R2B1 软件前置阶段入口为 `bash scripts/run_r2b1_software_gate.sh`。
它在新研究 WSL 中隔离构建并回归、运行 Linux synthetic PTY 采集/回放和冻结共享尺度
合成实验；不启动 Gazebo/AP，不改控制输出。证据：
`evidence/RESEARCH_R2B1/r2b1_suite_20261019T102246Z/summary.json`。
371 项 Python 测试、12 项 PTY、22 项合成检查和 433 文件保护审计通过；原始 36 个
数组及覆盖率另经完整回读。开发时两项数值边界失败与最终通过均保留。

`micolink.py` 的接受仅表示本地布局、串口序列和接收时间契约通过；线上没有协议版本
字段，本地 LE 布局仍须真实模块验证。模块系统时间不代表曝光时间，协议未证明的
旋转/尺度/时钟/不确定性不自动填值。共有测距在 N 帧内是同一个潜在误差时，不能
按逐帧独立把它的方差除以 N；本轮 c=0、Gaussian、人为压力参数的覆盖率反例仅作
算法验证基础，既非通用 PL 保证，也不证明 FAST-LIVO2 具有相同失效。

2026-10-19 R2B2 使用新增 `xq_fast_livo2` 包。作者官方主分支为 ROS 1；本轮采用
固定提交的社区 Humble 移植，保留 GPL-2.0 和来源说明，依赖安装到新研究副本的
`livo_deps/`，构建/安装树为 `livo_build/`、`livo_install/`。不能把社区移植写成作者
官方 ROS 2 支持。`build_livo_isolated.sh` 在编译前后核对源码、依赖和基础构建身份，
成功后绑定实际安装产物；`check_livo_build_manifest.py` 在实验前后复核。

回放使用冻结 R2A 原始数据库，读取模式为只读 immutable；点云明确采用无单点扫描
时间的 Gazebo 瞬时快照模型，RGB 图像每 3 张保留 1 张。发布器只发原始点云、IMU、
前视图像及按原采集时钟生成的 `/clock`，保持 CDR 与采集 header，不发真值、旧里程计、
TF 或 AP 消息。真值仅交给离线评价器，另从原始数据库重建核对。两臂保持同一 LIVO
调度与视觉地图流程，只改变视觉 EKF 校正开关；最小 6 个选择点仅是本轮固定支持门，
不表示已证明六自由度可观性。

本机 WSL mirrored 网络的默认 DDS 回环配置存在跨进程发现失败；采用逐轮生成、绑定
当前网卡地址的 Fast DDS UDPv4 profile，三路回放两端均为 RELIABLE。该配置不修改系统
网络，也不验证真实驱动 QoS 或硬件时延。启动必须等待发布器 READY；停止必须等待
`processing-result.json` 的原子 COMPLETE，再独立连接全部原始图像、处理账本、视觉
诊断和里程计。初始化和末尾覆盖不足必须明记，不能拿接收计数代替处理完成。

当前已有冻结输入索引时，入口依次为 `bash scripts/build_isolated.sh`、
`bash scripts/build_livo_isolated.sh`、`bash scripts/run_livo_regression.sh` 和
`bash scripts/run_livo_replay_gate.sh`。固定协议在 `config/research_livo_replay_gate.json`；
任何失败保留独立 attempt，不能挑旧 PASS 代替最新结果。归档前用冻结评价器重新计算，
复核输入 SHA、原始数据库、受保护旧文件和全部 attempt 集合。

正式结果为 `evidence/RESEARCH_R2B2/r2b2_suite_20261019T141249Z/summary.json`：
每臂接收 844 点云/16,885 IMU/429 图，429 图分类为 3 启动、1 初始化、424 完成、
1 末尾覆盖不足；两臂有效输出时刻全部一致。18 项数值、42 项证据和每臂 21 项处理
检查通过，四个失败 attempt 完整保留。单条仿真 LIVO/关闭视觉的 ATE RMS 分别为
4.881757/4.963916 mm，不构成一般精度优势或 PL 证明。

学术复核：[FAST-LIVO2 on Resource-Constrained Platforms](https://arxiv.org/abs/2501.13876)
已有基于激光退化的视觉帧选择和混合地图管理，因此视觉启停、帧率调节与本轮融合接通
不作为创新。下一阶段需要量化共享 IMU、地图和距离尺度误差如何影响整任务安全裕度，
再验证主动感知是否恢复可靠裕度；不能从一次仿真 ATE 或滤波协方差直接推出 PL。

2026-10-20 R2B3A 正式验收归档为
`evidence/RESEARCH_R2B3A/r2b3a_suite_20261020T090255Z/summary.json`，入口为
`bash scripts/run_r2b3a_software_gate.sh`，协议固定在
`config/research_flow_precondition_gate.json`。原始像素追踪、单窗相关 Gaussian 内核
与两个独立审计器均已验证；原 29 个 LIVO 源文件未改变，不能据此称光流已进入共同状态。
全套 528 项 Python 测试通过；原图审计 23 项、合成检查 18 项、原数组独立审计 188 项、
最终只读 QA 33 项通过。第一次 CDR 补齐解析失败与最后一次完整 PASS 均保留，不能覆盖失败。

重建相邻下视窗口 1,284 个、保留前视输入节拍窗口 428 个，分别 851/283 个通过像素
追踪门；43,843 条轨迹全部检查。两种节拍中位数 66/198 ms，但 aligned 并不保证两端
都是有效 LIVO 状态。所有窗口都因缺少米制深度、像素/陀螺噪声与状态/跨时刻相关项而
禁止更新，不使用真值补尺度。给定 gamma 的解析投影通过不表示逐轴积分已完成实际
IMU 去偏、非交换旋转积分或噪声传播。真实微空模块的旋转补偿/内部 gyro 语义仍须实测。

合成模型仅证明相关项的作用：rho=-0.9/0/+0.9 各 120,000 个独立样本，点估计在浮点
舍入精度内一致（最大差 `2.22e-16 m/s`），正确模型方向覆盖率为 95.0183/94.9967/94.9825%，错误 C=0 为
84.5658/94.9967/100%。这不是实际传感器标定或 PL；开发复跑不增加独立样本数。
标准相关 Gaussian、随机 pose clone 和地图 nuisance 相关性已有
[OpenVINS](https://docs.openvins.com/classov__msckf_1_1StateHelper.html) 与
[SEVIS](https://openaccess.thecvf.com/content_CVPR_2019/papers/Geneva_An_Efficient_Schmidt-EKF_for_3D_Visual-Inertial_SLAM_CVPR_2019_paper.pdf)
等先例，不作为本项目创新。

R2B3B 按以下门推进，首先维持离线、无 AP 发布的范围：

| 门 | 实现与验证 | 通过后允许的结论 |
|---|---|---|
| 联合状态 | 19 维活动状态＋起点 6 维 pose clone＋深度/局部平面 nuisance，完整传播和更新交叉块；对照批量条件解、有限差分与多窗 Monte Carlo | 声明联合模型下的软件正确 |
| 观测与旋转 | 原始两视图重投影、去偏 SO(3) 逐段传播；恒轴/交替轴、纯转动、杆臂、时间偏移；先不重叠图像对，再单独处理共享图像 | 声明模型下的几何、时序与共享 IMU 处理正确 |
| 尺度与像素噪声 | 新录有显式噪声模型的下视测距仿真；区分光轴深度、斜距和高度；平面先行，再斜坡/台阶/低纹理；标定与测试轨迹分离 | 指定仿真域内的观测模型成立 |
| 实际共同状态消融 | 固定输入/调度，对比 LIVO、完整依赖的 LIVO＋flow、仅清零共享 gyro 相关、仅清零共同尺度相关；记录每次实际状态/协方差变化 | flow 对状态的实际作用及依赖项影响可复核 |

每次 LiDAR/视觉更新、初始化和边缘化都必须更新相关块；若仍用预补偿 flow，则传播
`C=-P_xgamma Jgamma^T` 以及后续窗口的条件噪声均值和协方差，不能逐窗复用原 C/R。
现有 LIVO 地图相关性尚未全面审计，局部联合原型通过也不推出全系统统计一致性。
后续才验证“在算力预算下，预测观测对任务方向不确定性的实际增益并选择感知动作”
的研究假设，随后完成 Orin 热稳态和 X25/AP ExternalNav 实机闭环。AP 固件继续沿用
现有冻结链路，硬件始终保持 NOT_READY，直到对应实测门通过。

2026-10-20 用户要求继续实验并保留已有可写论文成果。保全基线为
`evidence/PAPER_PRESERVATION/paper_baseline_20261020T092036Z/`：548 份当前工作树
源/配置/文档快照、1,979 份旧证据 SHA 索引、6 组带边界的 claim ledger。保全目录
573 文件已封存，既有结果保持原位置，不自动把当前源版本视为所有历史实验版本。
后续每轮只读复验旧文件 SHA/集合及冻结源副本，新增实验使用明确排除的新目录；
不修改旧 evidence、不执行会删除旧研究副本的镜像同步、不自动 git 提交或推送。

R2B3B0 入口 `bash scripts/run_r2b3b0_software_gate.sh`，协议
`config/research_joint_window_gate.json`；正式归档为
`evidence/RESEARCH_R2B3B0/r2b3b0_suite_20261020T094534Z/summary.json`。19＋1 持久
nuisance＋6 clone 的线性模型在4窗内显式传播全部交叉块，与 primitive 全历史条件解
72 次核对：均值最大差 `1.11e-15`、协方差 `1.25e-16`。36,000 条独立合成轨迹含
144,000 个相关窗口，逐窗固定 vx 95% 边缘覆盖为 94.8750/95.0139/95.0250/94.9750%。
不能称为任务方向校准、二维联合覆盖、整任务风险保证或实际传感器改进。

632 项回归、14＋1 包构建、70 项独立联合审计（24 数组/79 台账）、139 项条件几何门
及最终44＋5项只读 QA 通过。给定 bias 的中点 SO(3) 连乘与给定深度的两视图几何
分别通过独立 quaternion/静态世界点数值检查；这两块尚未接成非线性滤波器。
本轮实际录制光流状态更新为 0，29 个 LIVO 源文件保持原封存版本；无新 AP 飞行、
Orin/X25 实测或硬件结论。开发小样本烟测及单位断言的修正保留说明，不降低正式门限。

R2B3B1已补新仿真测距、声明噪声及有效域；一段4,188同戳观测经独立复核，完整阶段
仍INCOMPLETE。最新AP复跑出现FCU视觉里程计接收秒级空窗及起飞失败，先做不解锁的
通信分段诊断与FCU端健康启动门，再连接非线性两时刻状态与原始像素观测。
待共同状态融合通过，再尽早进行同相关模型/同延迟模型/同预算的联合策略与强串联
基线对照，验证核心 idea；后续才扩展多地图/多 seed 与 AP 整任务复验。标准数学
软件验证只作基础或附录证据，不作为原创性已成立的证明。

R2B3B3已完成一次离线clock-guard回放与两组未解锁实测。入口
`python3 scripts/run_extnav_guard_probe.py --source synthetic --load lean` 或
`--source lio --load full`（WSL研究workspace、ROS环境及已验证安装版本），协议为
`config/research_extnav_clock_guard.json`。时钟异常锁存并停止输出，350ms接收新鲜度、
4Hz/5s本地连续窗口与FCU SYS_STATUS采样健康分别审计；50ms一致性预算不能当标定界。
无跳变正对照及受控故障通过，记录回放阻断364/548、668/872条（含触发帧）。

synthetic/lean原分析INCOMPLETE：故障安全反应成立，但自然时钟故障抢先打断暂停恢复
对照，原runner保留FAIL。LIO/full的70项诊断门PASS，wire/VISP/VISV261条有序对应；
adapter268条的最早7条没有出现在覆盖启动的wire记录；独立sink观察分支启动较晚，
不能进一步定位缺口，原因待查。两轮时钟
故障后就绪均撤销；这些结果不解决B1的4.80s空窗或证明EKF/飞行成功。全回归999项PASS。

下一门先验证稳定时基映射（epoch＋monotonic当前仅离线候选）、启动段通路准备及隔离的
暂停/恢复；之后把持续FCU就绪复查接入ARM/TAKEOFF路径，再完整重做AP飞行回归。
Gazebo强制退出问题继续单列。旧失败和论文保全证据不覆盖，不把diagnostic PASS、
monitor完成PASS或ever_ready当作飞行许可。核心研究的实际共同状态融合与策略消融
仍需按前述门推进，实际flow状态更新仍0，硬件NOT_READY。

本步完整封存：`evidence/RESEARCH_R2B3B3/r2b3b3_suite_20261020T150451Z_320/`，339文件、
2轮实测/2次构建/5个大文件或DB索引；manifest SHA-256
`251a07aef59595e2eaa278778ca8f70588ca53e8790aa40e3f1b13f886af2373`。归档完整性与
旧论文证据保全均PASS；实验状态分别保留INCOMPLETE/PASS，不能提升为飞行已修复。

2026-10-21 R2B3B4完成离线时基/启动契约验证，主协议
`config/research_extnav_transport_study.json`，启动修正版独立协议
`config/research_extnav_startup_gate.json`。62项时基、71项启动门和13项回放单测均包含在
全套1145项PASS中；559个源码/配置/驱动文件快照及前后集合/SHA一致，没有安装部署。
两个记录的864/527个观测括号候选ns/us严格递增；源用syntheticordinal，不重构真实
采集年龄。原AP JitterCorrection C++隔离对照41项通过，同时证明其滞后落界不等于采集
校准：合成固定80ms延迟可以显示0ms校正收件滞后。原P4/B3行为未改，候选未发AP。

启动初版54/11PASS证据保持，审查发现其缺少最大查询间隔。revision2用250ms默认
gap/1s窗口，71项单测、新回放13检查/79条受控查询及独立6项反例通过，不能从两幅
真实图快照推断连续可见。原7条启动缺口仍未解决。下一步单独预声明未解锁on-wire
协议，使用实际源/发布台账、wire/FCU完整配对及独立就绪证据，再补暂停/恢复与完整
AP任务；不能用“单调”或“FCU时间落界”代替采集时间/延迟标定。

硬件部署仍需实际非线性flow共同状态更新和消融（当前0）、相机/微空具体选型与标定、
Orin驱动/满载截止期与热稳态、实际X25固件/参数/接口身份、无桨地面及HIL/失联切换，
再受控飞行。可并行开展Orin无桨采集与软件研发，不能把软件单测完成度当实飞百分比。

B4最终离线封存：`evidence/RESEARCH_R2B3B4/r2b3b4_offline_suite_20261021T070702Z_434/`，
749文件/11来源，manifest SHA-256
`6687b1dc87866741501a2d35487f83f8fd4fd8a5fd19e1bd51af849290a05213`。归档/前后保全PASS。
全回归含domain211禁用执行节点单测；原summary过宽的“无ROS节点”字段不改写，范围
更正已独立保留，专用离线研究仍无AP/MAVROS/Gazebo或候选发送。此前未完成封存因
补充更正停止，原部分文件保留；不能将归档PASS提升为硬件就绪。

用户本轮要求：先完成所有非硬件部分；后续统一官方AP4.7.1。执行范围与逐项验收
已落入 `docs/SOFTWARE_COMPLETION_ACCEPTANCE.md`。硬件选型/采购/刷写/上板采集与
飞行均不作为本轮执行项。28维raw两视图/ToF共同状态已实现并独立构建，真实回放
首个失败正在修复；状态保持未完成，不用313项核心检查或固件无解锁smoke代替
同输入强基线、指定相关项消融、多场景/多seed退化及AP4.7.1完整任务。

本轮新发现的原IMU传播起点未初始化与empty-cloud历史残留，会在新实现统一修复并
重做比较。历史source/binary/结果不可改写为修复后的结果，原论文归档继续保全。

2026-10-21后续更新：实际非线性同输入五组回放和独立审计已通过，Full原生状态/
协方差非零更新37次，主对照Joint/no-flow与Full位置RMS为7.203/6.645mm。
AP4.7.1无解锁线上两组557/585条完整配对及暂停恢复已验证，但未完成新融合驱动的
AP全任务。强串联原型尚未接入实际比较；多seed退化、保护级新校准与任务策略联合
验证仍待做。按用户新优先级先交付CMICA2026 Winter英文初稿+完整数学附录+LaTeX，
路径 `output/pdf/`；不把初稿完成标成所有非硬件工作完成。旧成果最终保全PASS。

## 2026-10-22：服务器交付入口

`server/` 为后续 Linux 服务器的统一入口；首先按 README 安装依赖、下载固定资源并做静态覆盖校验，
然后分别运行离线机制检查、五组 LIVO 回放、基础/复杂 Gazebo 场景与 AP4.7.1 未解锁验证。
每次实验使用新输出路径，保留失败，不搬运旧 install 或修改绑定清单绕过身份不匹配。
交付边界和下一步缺口见 PROGRESS、server/README.md 与 docs/SOFTWARE_COMPLETION_ACCEPTANCE.md。
