# IMPACT：基于国产算力的无人机具身智能实时感知与决策系统

IMPACT（Integrity-Margin Planning and Active Control）是一套面向 **GNSS拒止、未知动态环境** 的单机无人机自主感知与决策系统。

项目基于 **Livox Mid-360S + Huawei Atlas 200I DK A2 + CUAV 7-Nano V2 + openEuler** 构建完整机载闭环，实现激光惯性定位、三维建图、自主探索、动态避障、在线重规划与故障安全决策。

本项目对应赛题：

> **XH-202629：基于国产算力的无人机具身智能实时感知与决策系统实现**

---

## ✨ 核心特性

* **GNSS拒止自主定位**：基于 Mid-360S 与 IMU 的高频 LiDAR-Inertial Odometry
* **实时三维建图**：构建局部导航地图与高精度全局地图
* **自主未知环境探索**：自动选择下一最佳探索区域，无需人工航点
* **完整性感知规划**：根据定位可信度动态调整探索与飞行策略
* **动态障碍感知**：结合 LiDAR 与视觉语义进行在线地图更新
* **实时轨迹重规划**：基于 EGO-Planner 构建安全局部轨迹
* **主动感知**：定位退化时主动改变运动方式以获得更有效的环境观测
* **弹性安全决策**：支持减速、重定位、刹停、悬停、返航、降落与人工接管
* **国产平台部署**：核心算法运行于 Atlas 200I DK A2 与 openEuler

---

## 🧠 核心算法链路

```text
              Mid-360S + IMU
                     │
                     ▼
          LiDAR-Inertial Odometry
                     │
          Pose + Covariance + Integrity
                     │
                     ▼
          Dynamic Semantic Mapping
                     │
                     ▼
       Integrity-Aware Exploration
                     │
                     ▼
             EGO-Planner
                     │
                     ▼
        Trajectory Safety Verification
                     │
                     ▼
           CUAV 7-Nano V2
                     │
                     ▼
                UAV Motion
                     │
                     └──────────────► New Observation
```

系统的核心思想是：

> **不仅判断“无人机在哪里”，还判断“我们有多确定无人机在哪里”，并让这种定位可信度直接影响下一步飞行决策。**

我们定义导航完整性裕度：

[
M = AL - PL
]

其中：

* `PL`：当前导航系统能够保证的定位误差范围
* `AL`：当前环境和任务能够容忍的最大定位误差

当 `M > 0` 时继续执行任务；当裕度不足时，系统主动减速、改变视点或执行重定位；当轨迹无法满足安全约束时，拒绝执行并进入刹停或降级模式。

---

## 🔬 主要研究方向

### 1. Directional Navigation Integrity

从 LiDAR-Inertial 状态估计中提取方向可观测性、协方差、残差一致性与动态污染信息，构建方向相关的导航保护水平。

### 2. Integrity-Margin-Constrained Exploration

将导航完整性作为自主探索的硬约束，在保证定位和飞行安全的前提下最大化未知区域信息增益。

### 3. Minimum-Excitation Active Perception

当定位即将退化时，通过最小幅度的侧移、升降、速度调节或视点变化主动获取新的几何约束。

### 4. Latency-Aware Safety Planning

将机载计算延迟、定位不确定性、速度和动态障碍统一纳入轨迹安全包络，实现资源受限平台上的实时安全规划。

---

## 🛠 硬件平台

| 模块   | 设备                      |
| ---- | ----------------------- |
| 激光雷达 | Livox Mid-360S          |
| 机载计算 | Huawei Atlas 200I DK A2 |
| 飞控   | CUAV 7-Nano V2          |
| 操作系统 | openEuler               |
| 飞控固件 | ArduPilot               |
| 通信   | MAVLink / MAVROS2       |
| 视觉   | 全局快门 RGB Camera         |
| 辅助测距 | Downward Range Finder   |

---

## 💻 软件栈

```text
openEuler
├── ROS 2 Humble
├── CANN / AscendCL
├── Livox SDK2
├── FAST-LIO2
├── EGO-Planner
├── MAVROS2
└── ArduPilot
```

FAST-LIO2、EGO-Planner 等成熟开源算法作为系统基线使用，本项目的主要研究工作集中在 **导航完整性建模、主动感知、完整性约束探索、实时安全决策和国产平台工程化部署**。

---

## 🎯 设计指标

| 指标         |       目标 |
| ---------- | -------: |
| 室内 ATE RMS | ≤ 0.30 m |
| 室外 ATE RMS | ≤ 0.50 m |
| 三维地图分辨率    |   ≤ 5 cm |
| 定位 / 建图频率  |  ≥ 10 Hz |
| 在线重规划延迟    |    ≤ 2 s |
| 通信丢包测试     |    ≤ 20% |
| 机载计算平台功耗   |   ≤ 30 W |

---

## 🛡 安全状态

```text
NORMAL
  │
  ├── CAUTIOUS
  │
  ├── RELOCALIZE
  │
  ├── BRAKE
  │
  ├── HOVER
  │
  ├── RETURN
  │
  └── LAND

MANUAL_OVERRIDE 始终具有最高优先级
```

任何上层算法异常都不能绕过飞控安全机制直接驱动无人机。

---

## 📌 当前状态

项目正在持续开发与实机验证中，重点推进：

* [ ] Mid-360S + FAST-LIO2 稳定定位
* [ ] Atlas 200I DK A2 / openEuler 部署
* [ ] External Navigation 接入 ArduPilot
* [ ] 三维局部地图
* [ ] EGO-Planner 实机规划
* [ ] 导航完整性估计
* [ ] 自主探索
* [ ] 动态障碍感知
* [ ] 主动感知与退化恢复
* [ ] 故障注入与安全降级
* [ ] 功耗、ATE 与实时性测试

---

## 📄 License

本项目包含自研代码及多个第三方开源组件。各第三方组件版权与许可证归原作者所有，最终许可证信息请参考仓库中的 `LICENSE` 与第三方依赖说明。

---

## Acknowledgements

感谢以下优秀开源项目及其作者：

* FAST-LIO2
* EGO-Planner
* Livox ROS Driver 2
* ROS 2
* MAVROS
* ArduPilot

本项目在相关工作的基础上进行面向国产算力无人机具身自主飞行的系统级研究与工程实现。

---

**IMPACT — Know where you are, know how certain you are, and act accordingly.**

**不仅知道“在哪里”，还知道“有多可信”，并据此决定“下一步怎么飞”。**


本工作区依据 `玄穹X1_XH-202629_完整企划书_V1.0.txt` 搭建，面向
ROS 2 Humble 与 Gazebo Harmonic（gz-sim8）。它与 WSL 中现有的
`cuadc_ws`、`ardupilot` 和 `ardupilot_gazebo` 完全隔离，不会修改任何
既有 world、model 或构建产物。

P0–P8 的阶段记录、独立验收报告与轻量原始证据统一入口见
[`docs/PHASE_ARCHIVE_INDEX.md`](docs/PHASE_ARCHIVE_INDEX.md)。大型 rosbag 与 Gazebo
record 不进入普通 Git 历史，其 SHA-256、大小和原始路径保存在 `evidence/P*/`。

当前验证层级为算法级 SIL：

- `daf_lio_proxy_2d`：验证退化指标、协方差传播及规划反馈接口；
- `td_semmap_2d`：验证占据、动态置信与 TTL 清除；
- `oaer_2d`：验证信息增益、可观测性、风险、时间、能耗及安全可达前沿联合评分；
- `r2_ego_proxy_2d`：验证自适应安全半径、在线膨胀路径监视、规划期限与 BRAKE 行为；
- 3D→2D 感知适配：按机体系 z 高度切片，排除地板/天花板伪障碍；
- `sentinel_fsm`：验证故障检测、降级、恢复和人工接管；
- `xq_network_relay`：在项目话题内执行可复现的 ground-link 丢包，不修改 WSL 网络；
- F1–F8 项目内故障矩阵：camera、NPU、planner、LiDAR、定位高协方差、
  ground-link、CPU 负载代理和低电量；
- Gazebo 专属场景、传感器桥、动态障碍和指标节点。

这些代理验证控制逻辑与接口，不等同于 EGO-Planner 三维优化器。P3 已另行接入
项目本地 FAST-LIO2 ESIKF/ikd-Tree 源码并完成 Gazebo SIL 验收，但仍不能替代真实
Mid-360S、Atlas 实机功耗/温升、实机 ATE 或 5 cm 三维地图精度证据。报告必须保留
`SIMULATED` / `UNVERIFIED` 边界。

按 IMPACT 执行企划新增了独立的 P1 真飞控基线：ArduCopter SITL、
ArduPilotPlugin、Gazebo Harmonic 与 MAVROS 在 `/uav1/mavros/*` 完成
GUIDED、ARM、起飞、悬停和降落。P1 不启用上述代理控制器，也不代表
P2 之后的 Mid-360、FAST-LIO2 或 IMPACT 已完成。

P2 在同一专属 Gazebo 场景上建立了与后续定位算法解耦的传感器边界：
`/livox/lidar`（`sensor_msgs/PointCloud2`，`livox_frame`，10 Hz）与
`/livox/imu`（`sensor_msgs/Imu`，`livox_imu`，200 Hz）。桥接层支持固定种子的
丢帧、时间戳抖动、运动畸变和外参误差；基线配置关闭人为故障，只保留 SDF 中的
LiDAR/IMU 高斯噪声。P2 只证明 Mid-360-like 仿真数据契约，不等同于真实 Livox
驱动或真实扫描模式。

P3 使用固化在 `src/xq_fast_lio` 的 BSD 许可 FAST-LIO2 ROS 2 源码快照，保留
ESIKF、IMU 传播、点到面更新与 ikd-Tree 地图算法，输出 `/localization/odom`。
算法运行时禁止订阅 `/xq/eval/*`；评估器以首帧 SE(2) 对齐计算 ATE、RPE、位置/
偏航误差、频率和处理延迟。`structured_room` ATE RMS 为 0.0330 m，
`long_corridor` 为 0.0507 m，两场景 10 Hz 连续输出并通过 P3 Gate。

P4 将 FAST-LIO2 位姿与 ESIKF 机体系速度经 `/uav1/mavros/odometry/out` 送入
ArduPilot EKF3；GPS 与 SITL GPS 全部关闭。正式 Gate 完成 2 m 起飞、悬停、
2 m × 2 m 矩形、返回和降落，ATE RMS 为 0.00509 m。ground truth 只供独立评估，
不进入算法或飞控。

P5 已完成 0.10 m 建图、Frontier 自动选择与官方 ROS2 EGO-Planner 闭环。P6 在
FAST-LIO 点面更新内部直接构造方向信息矩阵，并由独立节点输出完整性协方差和方向
Protection Level。P7 已用两个训练场景冻结 train-only 校准，并在两条新轨迹上通过
95%/99% 覆盖 Gate。P8 已对 EGO B-spline 与静态地图计算环境 Alert Limit；P9 的
`AL-PL` Integrity Margin 与硬拒绝仍未启用。

## 隔离约束

- WSL 运行副本：`/home/accelerate/xuanqiong_x1_sim_ws`
- ROS 命名空间：`/xq/agent_01`
- P1 MAVROS 命名空间：`/uav1/mavros`
- Gazebo 资源名：全部使用 `xq_` 前缀
- 每次运行使用专属 `ROS_DOMAIN_ID` 与唯一 `GZ_PARTITION`
- 不对 WSL `lo` / `eth0` 使用全局 `tc netem`；丢包由项目内 relay 实现
- 不执行 `killall` / 全局 `pkill`，只终止本次 launch 的进程组
- 不修改 `/home/accelerate/cuadc_ws`、`/home/accelerate/ardupilot*`

## 目录

```text
src/xq_sim_interfaces  自定义消息
src/xq_gz_assets       专属 world/model（由本项目安装）
src/xq_gz_bridge       gz-transport13 与 ROS 2 桥
src/xq_autonomy        算法代理、监督器、网络与指标核心
src/xq_sim_bringup     启动、配置、运行证据
scripts                同步、构建、运行与隔离审计
```

P1 正式基线（默认 600 秒）：

```bash
cd /home/accelerate/xuanqiong_x1_sim_ws
bash scripts/run_p1_flight_baseline.sh
```

P2 Mid-360-like 传感器正式 Gate（默认 600 秒）：

```bash
cd /home/accelerate/xuanqiong_x1_sim_ws
bash scripts/run_p2_sensor_validation.sh
```

P3 FAST-LIO2 正式 Gate（两个场景分别执行）：

```bash
cd /home/accelerate/xuanqiong_x1_sim_ws
bash scripts/run_p3_fast_lio.sh --scenario structured_room --duration 75
bash scripts/run_p3_fast_lio.sh --scenario long_corridor --duration 75
```

P4 GPS-off FAST-LIO ExternalNav 正式 Gate：

```bash
cd /home/accelerate/xuanqiong_x1_sim_ws
bash scripts/run_p4_external_nav.sh --minimum-eval-duration 70
```

P4 PASS rosbag 的 RViz 动态复现：

```bash
cd /home/accelerate/xuanqiong_x1_sim_ws
bash scripts/view_p4_rviz.sh
```

P5 structured-room BASELINE_V1 正式 Gate 与 RViz 复现：

```bash
cd /home/accelerate/xuanqiong_x1_sim_ws
bash scripts/run_p5_baseline.sh
bash scripts/view_p5_rviz.sh
```

P6 Directional Integrity Predictor 正式 Gate 与 RViz 复现：

```bash
cd /home/accelerate/xuanqiong_x1_sim_ws
bash scripts/run_p6_directional_integrity.sh
bash scripts/view_p6_rviz.sh
```

P7 train-only Protection Level 校准、独立验证与校准 PL95 RViz 复现：

```bash
cd /home/accelerate/xuanqiong_x1_sim_ws
bash scripts/run_p7_calibration.sh
bash scripts/view_p7_rviz.sh
```

P8 静态障碍 Alert Limit Gate 与 RViz 复现：

```bash
cd /home/accelerate/xuanqiong_x1_sim_ws
bash scripts/run_p8_alert_limit.sh
bash scripts/run_p8_live_gazebo.sh
bash scripts/view_p8_rviz.sh
```

从 P8 起每阶段同时保留 headless Gate、Gazebo record/replay 和 RViz 算法可视化，约定见
`docs/VISUALIZATION_POLICY.md`。

详细命令见 `docs/RUNBOOK.md`，当前验证状态见
`docs/VALIDATION_REPORT.md`，逐项完成性边界见
`docs/COMPLETION_AUDIT.md`。
