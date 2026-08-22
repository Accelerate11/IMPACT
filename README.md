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
