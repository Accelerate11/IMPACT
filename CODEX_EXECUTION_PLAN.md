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
| P10-P14 | 主动感知、完整性探索、动态障碍、时延安全、故障注入 | 按原企划逐项验收 |

当前原则：P0/P1 代码和证据先合格；P2 起每次只引入一个变量，并固定随机
种子、场景、初始位姿、目标点和故障时序。所有实验保存在时间戳目录，至少
包含参数快照、rosbag、原始日志、指标 JSON 和外部资产隔离审计。
