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
包含参数快照、rosbag、原始日志、指标 JSON 和外部资产隔离审计。P14 已完成 SIL 验收，
下一阶段为硬件部署；正常飞行可视化使用 `run_complex_3d_live_visualization.sh`，不启动
P14 故障注入。未收到明确命令前不自动开始、不自动提交代码。
