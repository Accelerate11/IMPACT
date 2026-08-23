# IMPACT

**Integrity-Margin Planning and Active Control for GNSS-Denied UAV Exploration**

IMPACT 是一套面向 **GNSS 拒止、未知环境与动态场景** 的单机无人机自主感知与决策框架。

项目以 **Livox Mid-360S + Huawei Atlas 200I DK A2 + CUAV 7-Nano V2 + openEuler** 为目标实机平台，以 **ROS 2 + ArduPilot + FAST-LIO2 + EGO-Planner** 为稳定基线，研究导航完整性约束下的自主探索、主动感知、动态避障与故障安全决策。

> **核心思想：不仅估计“无人机在哪里”，还估计“当前定位误差是否足以安全执行下一条轨迹”。**

项目对应：

> **XH-202629 — 基于国产算力的无人机具身智能实时感知与决策系统实现**

---

## 当前状态

> **2026-08-23：SIL 仿真 P0–P8 已通过，P9–P14 待执行。**

当前仓库不是仅包含方案文档的空工程，已经建立从 Gazebo / ArduPilot SITL 到 FAST-LIO2、Frontier、EGO-Planner 和导航完整性估计的可重复仿真链路。

| Phase | 内容                                           | 状态     |
| ----- | -------------------------------------------- | ------ |
| P0    | 仓库、环境与隔离审计                                   | ✅ PASS |
| P1    | ArduPilot SITL + Gazebo + MAVROS             | ✅ PASS |
| P2    | Mid-360S-like LiDAR / IMU 数据契约               | ✅ PASS |
| P3    | FAST-LIO2 GNSS-denied 定位基线                   | ✅ PASS |
| P4    | FAST-LIO2 → MAVROS → ArduPilot ExternalNav   | ✅ PASS |
| P5    | 3D Mapping + Frontier + EGO Baseline         | ✅ PASS |
| P6    | Directional Integrity Predictor              | ✅ PASS |
| P7    | Protection Level Calibration                 | ✅ PASS |
| P8    | Environment-dependent Alert Limit            | ✅ PASS |
| P9    | Integrity Margin + Hard Trajectory Rejection | ⏳ NEXT |
| P10   | Minimum-Excitation Active Perception         | ⏳ TODO |
| P11   | Integrity-Constrained Exploration            | ⏳ TODO |
| P12   | Dynamic Obstacle / Dynamic Map               | ⏳ TODO |
| P13   | Latency-Aware Safety                         | ⏳ TODO |
| P14   | Fault Injection & Resilient Autonomy         | ⏳ TODO |
| HW    | Mid-360S + Atlas + CUAV 实机闭环                 | ⏳ TODO |

完整执行记录见：

* [`PROGRESS.md`](PROGRESS.md)
* [`CODEX_EXECUTION_PLAN.md`](CODEX_EXECUTION_PLAN.md)
* [`docs/PHASE_ARCHIVE_INDEX.md`](docs/PHASE_ARCHIVE_INDEX.md)

---

# 1. 为什么做 IMPACT

传统无人机探索系统通常解决两个问题：

```text
Where am I?
        ↓
Where should I go?
```

但在 GNSS 拒止环境中，仅有一个位姿估计并不足够。

即使规划轨迹在地图上没有碰撞，如果：

* LiDAR 几何约束正在退化；
* 定位误差正在增大；
* 当前通道非常狭窄；
* 动态障碍预测存在不确定性；
* 机载计算延迟突然增大；

那么一条名义上可行的轨迹仍可能是不安全的。

IMPACT 因此引入两个量。

### Protection Level — `PL`

导航系统在给定置信水平下能够保证的定位误差范围：

[
PL(\mathbf d)
]

### Alert Limit — `AL`

当前环境、机体尺寸、障碍净空与任务允许的最大定位误差：

[
AL(\mathbf d)
]

最终定义导航完整性裕度：

[
\boxed{M = AL - PL}
]

因此：

```text
M > 0
│
└── 当前定位可信度足以执行轨迹

M ≈ 0
│
└── 减速 / 主动获取更好的观测

M < 0
│
└── 拒绝轨迹 / 恢复定位 / BRAKE
```

IMPACT 的目标不是始终追求最低定位误差，而是：

> **在满足任务安全所需定位完整性的前提下，以尽可能小的额外时间、路径与能耗完成自主探索。**

---

# 2. 核心算法链路

```text
                     Mid-360S + IMU
                            │
                            ▼
                       FAST-LIO2
                            │
                Pose / Velocity / Geometry
                            │
                            ▼
              Directional Integrity Predictor
                            │
                   Protection Level (PL)
                            │
                            │
        Local Map ──────────┼────────── Environment
                            │
                            ▼
                     Alert Limit (AL)
                            │
                            ▼
                 Integrity Margin M=AL-PL
                            │
             ┌──────────────┴──────────────┐
             │                             │
           M > 0                         M ≤ 0
             │                             │
             ▼                             ▼
       Continue Task             Active Recovery / Brake
             │                             │
             └──────────────┬──────────────┘
                            ▼
                     Frontier Explorer
                            │
                            ▼
                       EGO-Planner
                            │
                            ▼
                  Trajectory Certification
                            │
                            ▼
                    MAVROS / MAVLink
                            │
                            ▼
                  ArduPilot / CUAV FCU
                            │
                            ▼
                         UAV Motion
                            │
                            └──────────► New Observation
```

当前 P8 已完成 `PL` 与 `AL` 的独立计算。

**`M = AL - PL` 的在线硬轨迹认证从 P9 开始。**

---

# 3. 研究创新

IMPACT 不把 FAST-LIO2、Frontier 或 EGO-Planner 重新命名为“自研算法”。

这些成熟开源方法被作为可复现 Baseline，项目创新主要集中在它们之间过去被弱化的 **不确定性跨层传播**。

## 3.1 Directional Navigation Integrity

FAST-LIO2 不仅输出位姿，还从实际点到面更新中构造方向信息矩阵：

[
\Lambda_p
=========

\sum_i w_i \mathbf n_i\mathbf n_i^\top
]

进一步得到：

* 几何信息特征值；
* 最弱定位方向；
* 完整性协方差；
* 方向 Protection Level。

因此系统知道的不只是：

```text
localization = GOOD / BAD
```

而是：

```text
哪个方向正在失去约束？
误差可能达到多大？
```

---

## 3.2 Task-Dependent Alert Limit

定位误差是否危险取决于环境。

同样 `0.15 m` 的误差：

```text
5 m 宽大厅 → 可能完全安全

0.8 m 狭窄通道 → 可能不可接受
```

因此 IMPACT 根据：

* 障碍净空；
* 机体尺寸；
* 跟踪误差；
* 动态障碍；
* 飞行速度；
* 系统延迟；

实时计算任务允许误差 `AL`。

---

## 3.3 Integrity-Margin-Constrained Planning

从 P9 开始，导航完整性不作为普通规划代价：

```text
cost += λ * localization_uncertainty
```

而作为 **硬约束**：

[
M_{\min}(\tau) \ge M_{\text{reserve}}
]

如果一条轨迹不能保证足够的导航完整性，即使它：

* 路径最短；
* 信息增益最高；
* EGO 判断无碰撞；

仍然必须被拒绝。

---

## 3.4 Minimum-Excitation Active Perception

当名义轨迹无法满足完整性约束时，P10 将主动生成少量恢复动作，例如：

* 横向侧移；
* 垂向调整；
* 降低速度；
* 短时停留；
* 改变观测位置；
* 回退至高质量定位区域。

目标不是最大化“可观测性分数”，而是寻找：

> **能够重新使 `M > 0` 的最小代价动作。**

---

## 3.5 Latency-Aware Safety

在真实机载计算机上，计算延迟本身就是飞行风险。

IMPACT 将使用高分位端到端延迟：

[
L_{p99}
]

计算延迟导致的额外运动距离：

[
r_{\text{latency}}
==================

vL_{p99}
+
\frac{1}{2}a_{\max}L_{p99}^2
]

并直接缩小 `AL`。

因此：

```text
CPU / NPU load ↑
        ↓
Latency ↑
        ↓
Alert Limit ↓
        ↓
Integrity Margin ↓
        ↓
Speed ↓ / Replan / Recovery
```

---

# 4. 目标硬件平台

| 模块                | 目标设备                      |
| ----------------- | ------------------------- |
| LiDAR             | Livox Mid-360S            |
| Onboard Computer  | Huawei Atlas 200I DK A2   |
| Flight Controller | CUAV 7-Nano V2            |
| OS                | openEuler                 |
| Autopilot         | ArduPilot                 |
| Middleware        | ROS 2                     |
| AI Runtime        | CANN / AscendCL           |
| Communication     | MAVLink / MAVROS2         |
| Vision            | Global-shutter RGB Camera |
| Auxiliary Sensor  | Downward Range Finder     |

核心自主任务设计为 **全部机载运行**，不依赖云端或地面计算机完成安全关键闭环。

---

# 5. 当前仿真平台

当前算法开发和 Gate 验证采用：

```text
Ubuntu 22.04
├── ROS 2 Humble
├── Gazebo Harmonic / gz-sim8
├── ArduPilot SITL
├── MAVROS2
├── FAST-LIO2
└── EGO-Planner
```

仿真环境与最终 Atlas / openEuler 部署环境分离。

这样可以首先验证：

```text
Algorithm Correctness
        ↓
Closed-loop SIL
        ↓
Fault Injection
        ↓
Hardware Porting
        ↓
Real Flight
```

而不是在实机上同时调试算法、驱动、飞控和结构问题。

---

# 6. 已完成验证

## P1 — Flight Baseline

ArduCopter SITL、Gazebo Harmonic 与 MAVROS 已完成：

```text
GUIDED
→ ARM
→ TAKEOFF
→ HOVER
→ LAND
```

正式 Gate 连续运行 **600 s**。

---

## P2 — Mid-360S-like Sensor Contract

仿真输出：

```text
/livox/lidar
sensor_msgs/msg/PointCloud2
10 Hz

/livox/imu
sensor_msgs/msg/Imu
200 Hz
```

正式验证：

* LiDAR：5996 帧；
* 实测频率：10.00008 Hz；
* IMU：119912 帧；
* 实测频率：200.00083 Hz；
* 时间戳、TF、有限值和字段契约全部通过。

> 该阶段验证的是 **Mid-360S-like 仿真接口**，不等于真实 Livox 驱动验证。

---

## P3 — FAST-LIO2

仓库内 `src/xq_fast_lio` 已接入 FAST-LIO2 ROS 2 源码。

Gazebo SIL：

| 场景              |  ATE RMS |
| --------------- | -------: |
| Structured Room | 0.0330 m |
| Long Corridor   | 0.0507 m |

两场景均：

* 10 Hz 连续定位；
* 无 Ground Truth 输入算法；
* 无 GPS 输入；
* Ground Truth 仅由独立 evaluator 使用。

---

## P4 — GPS-Off External Navigation

完成：

```text
FAST-LIO2
    ↓
MAVROS ODOMETRY
    ↓
ArduPilot EKF3
    ↓
Position Control
```

SITL GPS 与 GPS 类型均关闭。

完成：

```text
2 m Takeoff
→ Hover
→ 2 m × 2 m Rectangle
→ Return
→ Land
→ Disarm
```

该正式 SIL 轮次 ATE RMS：

```text
0.00509 m
```

> 该数据属于 Gazebo / SITL 仿真，不代表实机 ATE。

---

## P5 — Baseline Autonomous Exploration

当前 Baseline：

```text
FAST-LIO2
    ↓
0.10 m Navigation Map
    ↓
Frontier
    ↓
J = Information Gain - λ × Distance
    ↓
EGO-Planner
```

正式结果：

* 4 个自动 Frontier 目标；
* 110 条 EGO B-spline；
* 无人工航点；
* 43.170 m 飞行轨迹；
* 0 collision；
* 自动结束并降落。

这就是后续 IMPACT 创新的统一对照基线：

```text
BASELINE_V1
```

---

## P6 — Directional Integrity Predictor

已在 FAST-LIO 实际点面更新内部构造方向信息矩阵。

正式运行：

* 1467 帧 FAST-LIO 几何信息；
* 1467 帧方向完整性输出；
* 每帧有效点面约束约 1196–4074；
* 条件数约 1.443–5.464；
* 弱方向 PL 约 0.0233–0.0529 m；
* 弱轴会随真实扫描几何改变。

P6 不使用 Ground Truth 控制，也不改变规划器行为。

---

## P7 — Protection Level Calibration

使用独立：

```text
Training Scenarios
        ↓
Freeze Calibration
        ↓
Independent Test Trajectories
```

完成 Protection Level 校准。

当前验证中：

* 训练与测试数据隔离；
* 标定参数在测试前冻结；
* 95% / 99% Coverage Gate 通过；
* Ground Truth 仅用于离线评价。

---

## P8 — Alert Limit

已经能够沿 EGO B-spline 根据：

```text
Trajectory
+
Static Map
+
Vehicle Envelope
+
Obstacle Clearance
```

计算环境相关 `Alert Limit`。

P8 的静态障碍 Alert Limit 自动 Gate 已通过。

但目前：

```text
PL      ✅
AL      ✅
M=AL-PL ⏳ P9
```

所以 **当前还没有启用 Integrity Margin 对真实规划结果进行硬拒绝。**

---

# 7. Codex 开发顺序

本仓库采用严格阶段 Gate。

**Codex 不允许跳过前置阶段直接实现最终算法。**

执行总纲：

[`CODEX_EXECUTION_PLAN.md`](CODEX_EXECUTION_PLAN.md)

当前阶段顺序：

```text
P0  Repository Audit
 │
 ▼
P1  ArduPilot + Gazebo + MAVROS
 │
 ▼
P2  Mid-360 Sensor Contract
 │
 ▼
P3  FAST-LIO2
 │
 ▼
P4  External Navigation
 │
 ▼
P5  Baseline Exploration
 │
 ▼
P6  Directional Integrity
 │
 ▼
P7  Protection Level Calibration
 │
 ▼
P8  Alert Limit
 │
 ▼
P9  Integrity Margin          ← CURRENT NEXT STEP
 │
 ▼
P10 Minimum Excitation
 │
 ▼
P11 Integrity Exploration
 │
 ▼
P12 Dynamic Obstacles
 │
 ▼
P13 Latency-aware Safety
 │
 ▼
P14 Fault Injection
 │
 ▼
Hardware Deployment
```

每个阶段必须：

1. 编译；
2. 实际运行；
3. 自动 Gate；
4. 保存参数；
5. 保存日志；
6. 保存 rosbag 或其外部索引；
7. 计算指标；
8. 更新 `PROGRESS.md`；
9. 通过后才进入下一阶段。

---

# 8. 接下来要做什么

## P9 — Integrity Margin

实现：

[
M_j(t)=AL_j(t)-PL_j(t)
]

以及：

[
M_{\min}(\tau)=\min_{t,j}M_j(t)
]

轨迹认证：

```text
M_min >= reserve
        │
        └── ACCEPT

M_min < reserve
        │
        └── REJECT
```

重点测试：

```text
Wide Room
vs
Narrow Passage
```

必须证明：

> 相同定位质量下，宽阔环境中的轨迹可以通过，而狭窄环境中的危险轨迹能够被完整性约束拒绝。

---

## P10 — Minimum-Excitation Active Perception

当轨迹因完整性不足被拒绝：

```text
Baseline trajectory
       ↓
Integrity insufficient
       ↓
Generate recovery candidates
       ├── left/right lateral
       ├── vertical
       ├── slow down
       ├── short observation
       └── backtrack
       ↓
Predict future information
       ↓
Find minimum-cost candidate with M > 0
```

---

## P11 — Integrity-Constrained Exploration

将 Frontier 从：

[
\max ; InformationGain-\lambda Distance
]

升级为：

[
\max_\tau
\quad
InformationGain-\lambda_TT-\lambda_EE
]

subject to：

[
M_{\min}(\tau)\ge M_{\text{reserve}}
]

使安全完整性成为硬约束，而不是可被其他收益抵消的软权重。

---

## P12 — Dynamic Environment

加入：

* dynamic confidence；
* temporary occupancy；
* TTL decay；
* moving obstacle；
* online passage reopening。

实现：

```text
Obstacle appears
        ↓
Map update
        ↓
Replan
        ↓
Obstacle leaves
        ↓
Dynamic occupancy decay
        ↓
Passage becomes free again
```

---

## P13 — Latency-Aware Safety

记录完整端到端时间链：

```text
sensor
→ localization
→ mapping
→ planning
→ certification
→ command
```

计算：

```text
p50
p95
p99
max
```

并让 `p99` 延迟直接进入 `AL`。

---

## P14 — Fault Injection

计划加入：

* LiDAR dropout；
* IMU dropout；
* timestamp jitter；
* odometry delay；
* planner timeout；
* camera failure；
* CPU load；
* 20% packet loss；
* covariance inflation；
* low battery。

最终验证：

```text
NORMAL
  ↓
CAUTIOUS
  ↓
RECOVERY
  ↓
BRAKE
  ↓
HOVER
  ↓
RETURN / LAND
```

---

# 9. Repository Structure

当前主要目录：

```text
IMPACT/
│
├── CODEX_EXECUTION_PLAN.md
├── PROGRESS.md
├── SPEC_TRACEABILITY.md
│
├── config/
│
├── docs/
│   ├── CURRENT_REPO_AUDIT.md
│   ├── PHASE_ARCHIVE_INDEX.md
│   ├── P4_EXTERNAL_NAV_VALIDATION_REPORT.md
│   ├── P5_BASELINE_VALIDATION_REPORT.md
│   ├── P6_DIRECTIONAL_INTEGRITY_REPORT.md
│   ├── P7_PROTECTION_LEVEL_CALIBRATION_REPORT.md
│   └── P8_ALERT_LIMIT_REPORT.md
│
├── evidence/
│   └── P*/
│
├── scripts/
│   ├── build_isolated.sh
│   ├── run_p1_flight_baseline.sh
│   ├── run_p2_sensor_validation.sh
│   ├── run_p3_fast_lio.sh
│   ├── run_p4_external_nav.sh
│   ├── run_p5_baseline.sh
│   ├── run_p6_directional_integrity.sh
│   ├── run_p7_calibration.sh
│   └── run_p8_alert_limit.sh
│
└── src/
    ├── xq_autonomy/
    ├── xq_ego_planner/
    ├── xq_fast_lio/
    ├── xq_gz_assets/
    ├── xq_gz_bridge/
    ├── xq_livox_interfaces/
    ├── xq_sim_bringup/
    ├── xq_sim_interfaces/
    └── ...
```

---

# 10. Build

当前开发环境基于 ROS 2 Humble。

建议首先执行隔离构建：

```bash
./scripts/build_isolated.sh
```

任何新阶段开始前都应确保现有 Gate 没有因修改而回归。

---

# 11. Reproduce Existing Phases

已存在的阶段运行脚本：

```bash
# P1 — ArduPilot / Gazebo flight baseline
./scripts/run_p1_flight_baseline.sh

# P2 — Mid-360-like sensor validation
./scripts/run_p2_sensor_validation.sh

# P3 — FAST-LIO2
./scripts/run_p3_fast_lio.sh

# P4 — GPS-off ExternalNav
./scripts/run_p4_external_nav.sh

# P5 — Mapping + Frontier + EGO baseline
./scripts/run_p5_baseline.sh

# P6 — Directional Integrity
./scripts/run_p6_directional_integrity.sh

# P7 — Protection Level calibration
./scripts/run_p7_calibration.sh

# P8 — Alert Limit
./scripts/run_p8_alert_limit.sh
```

部分阶段同时提供 RViz / Gazebo replay 工具。

具体参数、证据目录和 Gate 结果以：

[`PROGRESS.md`](PROGRESS.md)

以及：

[`docs/PHASE_ARCHIVE_INDEX.md`](docs/PHASE_ARCHIVE_INDEX.md)

为准。

---

# 12. Ground Truth Policy

这是整个项目的强制约束。

Gazebo Ground Truth：

```text
                     Ground Truth
                          │
                          ▼
                  Evaluator / Logger
```

**禁止进入：**

```text
FAST-LIO2
Planner
Frontier
ExternalNav
ArduPilot EKF
Mission Decision
Integrity Predictor
```

因此：

> Ground Truth 只用于证明算法效果，不能用于让算法取得效果。

任何破坏该隔离原则的实验结果都应视为无效。

---

# 13. Evidence Policy

每个正式 Gate 尽量保存：

```text
git commit
config snapshot
random seed
world
initial state
stdout / stderr
rosbag
metrics
source hash
build hash
external asset audit
```

大型：

```text
rosbag
Gazebo record
```

不直接进入普通 Git 历史。

仓库保存：

* SHA-256；
* 文件大小；
* 原始路径；
* 轻量分析结果。

目的不是“留很多日志”，而是让每个性能结论都可以追溯。

---

# 14. Simulation ≠ Real Flight

当前仓库验证层级主要为：

> **SIL — Software-in-the-Loop**

因此以下数据必须标记为：

```text
SIMULATED
```

而不能写成实机成绩。

尚需真实硬件验证的内容包括：

* 真实 Livox Mid-360S 扫描模式；
* Livox ROS Driver 2；
* Atlas 200I DK A2 ARM 性能；
* openEuler 实时性能；
* CANN / NPU 负载；
* Atlas 实测功耗；
* 整机温升；
* CUAV 7-Nano V2 实飞；
* 真实振动；
* 真实时间同步误差；
* 室内真实 ATE；
* 室外真实 ATE；
* 5 cm 三维地图实测精度；
* 真实动态障碍；
* 真实 GNSS 拒止飞行。

仿真结果的作用是：

> **提前验证算法正确性、接口、状态机和故障逻辑，而不是替代真实飞行。**

---

# 15. Competition Targets

最终目标指标：

| 指标                          |   Target |
| --------------------------- | -------: |
| Indoor ATE RMS              | ≤ 0.30 m |
| Outdoor ATE RMS             | ≤ 0.50 m |
| 3D Map Resolution           |     5 cm |
| Localization / Mapping Rate |  ≥ 10 Hz |
| Online Replanning           |    ≤ 2 s |
| Communication Packet Loss   |    ≤ 20% |
| Onboard Computing Power     |   ≤ 30 W |

这些是目标与验收条件。

**只有真实测试完成后，才会在仓库中标记为 VERIFIED。**

---

# 16. Development Principles

所有后续开发，包括 Codex 自动开发，都遵循：

### Baseline First

先建立稳定 Baseline，再实现创新。

### One Variable per Phase

每个阶段只引入一个主要变量。

### Hard Gates

节点能启动 ≠ 算法有效。

必须通过量化 Gate。

### No Hidden Ground Truth

算法绝不能使用仿真真值。

### Fail Explicitly

失败必须记录为：

```text
FAIL
```

而不是：

```text
almost works
expected to pass
```

### Safety over Mission

安全约束不能被任务收益抵消。

### Simulation before Hardware

```text
Offline
→ SIL
→ Fault Injection
→ Bench
→ Tethered Flight
→ Real Flight
```

---

# 17. Research Baselines

IMPACT 建立在成熟开源研究之上，包括：

* FAST-LIO2
* EGO-Planner
* Frontier-based Exploration
* Livox ROS Driver 2
* ROS 2
* MAVROS
* ArduPilot

本项目不会把这些已有工作重新包装为原创算法。

主要研究贡献聚焦于：

1. **Directional Navigation Integrity**
2. **Protection Level Calibration**
3. **Task-dependent Alert Limit**
4. **Integrity-Margin-Constrained Planning**
5. **Minimum-Excitation Active Perception**
6. **Latency-aware Safety**
7. **Resilient Onboard Autonomy**

---

# 18. Target Deployment

最终实机链路：

```text
Livox Mid-360S
      │
      ▼
Atlas 200I DK A2
openEuler + ROS 2
      │
      ├── FAST-LIO2
      ├── Integrity
      ├── Mapping
      ├── Exploration
      ├── Planning
      └── Safety Supervisor
      │
      ▼
MAVROS / MAVLink
      │
      ▼
CUAV 7-Nano V2
ArduPilot
      │
      ▼
Flight Control
```

Atlas 负责：

```text
Perception
Localization
Mapping
Planning
Decision
```

CUAV 负责：

```text
Attitude
Rate Control
Position Control
Motor Control
Failsafe
```

安全关键的姿态内环不会交给 Linux 伴随计算机。

---

# 19. Single-UAV Scope

当前项目只制造和实飞 **一架无人机**。

因此：

* 核心算法不能依赖第二架无人机；
* 所有主要指标由单机实物完成；
* 多智能体能力仅保留软件接口和仿真扩展；
* 不把虚拟多机实验描述为多机实飞。

项目优先目标是把：

> **单机 GNSS-denied embodied autonomy**

真正做完整。

---

# 20. Acknowledgements

感谢以下开源项目及其作者：

* FAST-LIO2
* EGO-Planner
* ArduPilot
* MAVROS
* ROS 2
* Gazebo
* Livox ROS Driver 2

IMPACT 在这些成熟工作的基础上，进一步研究导航完整性如何从状态估计传播到探索、规划与安全决策。

---

# 21. License

仓库目前尚未正式声明统一的项目许可证。

第三方组件继续遵循其各自原始许可证。

在正式发布或分发自研代码前，将补充项目 `LICENSE`、第三方依赖和许可证清单。

---

## IMPACT

> **Know where you are. Know how certain you are. Act accordingly.**

**不仅知道“在哪里”，还知道“有多可信”，并据此决定“下一步是否应该飞、应该怎么飞”。**
