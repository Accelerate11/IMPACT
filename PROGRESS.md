# IMPACT 执行进度

更新时间：2026-09-02

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
| 复杂场景完整正常飞行展示 | PASS | `docs/COMPLEX_SCENE_FULL_AUTONOMY_DEMO_REPORT.md`；无故障注入、24 m 双航程、Gazebo+RViz、GPU 与隔离 Gate |
| 三维复杂场景完整算法 | PASS | `docs/COMPLEX_3D_FULL_AUTONOMY_REPORT.md`；right→left→up→direct、真实爬升、24 m 双航程、无故障注入 |
| 复杂组合三维完整算法 | PASS | `docs/COMPLEX_COMPOSITIONAL_3D_ACCEPTANCE_REPORT.md`；right→left→up_right→direct、动态障碍、P13 p99、GPU Gazebo+RViz |

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
