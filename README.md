# 玄穹-X1 Gazebo 算法仿真工作区

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
