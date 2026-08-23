# 玄穹-X1 仿真目标完成性审计

审计时间：2026-08-22（Asia/Shanghai）

| 要求/交付 | 当前证据 | 判定 |
|---|---|---|
| 不破坏 WSL 既有项目地图 | 每轮独立 ROS 域/Gazebo 分区；30 s 与 120 s 运行前后旧 world/model 字节哈希一致 | PASS |
| WSL 原生构建 | 5 packages 构建成功，源码/安装树哈希写入构建清单并由 runner 复核 | PASS |
| Gazebo 场景与传感器 | 专属 20×16 m world、移动障碍、720×32@10 Hz LiDAR、200 Hz IMU；world/model 均 `Valid` | PASS |
| ROS/Gazebo 专属桥 | 8 tests、0 failure；PointCloud2/IMU/cmd_vel/评估真值端到端可用 | PASS |
| P2 Mid-360S-like 契约 | 600.435 s；LiDAR 10.00008 Hz、IMU 200.00083 Hz；时间戳/TF/有限值 PASS | PASS，非真实 Livox 扫描 |
| 算法与验收契约单测 | WSL 原生 46/46 PASS | PASS |
| R1 室内 ATE <=0.3 m | 无故障 30 s 代理 ATE RMS 0.1535 m | `SIMULATED_PASS`，非正式实飞 ATE |
| R2 室外 ATE <=0.5 m | 无室外正式协议 | `UNVERIFIED` |
| R3 5 cm 三维地图 | 当前 0.10 m 二维代理栅格 | `UNVERIFIED` |
| R4 >=10 Hz | raw LiDAR/IMU、定位质量、odom、地图严格门全部通过 | `SIMULATED_PASS` |
| R5 20% 丢包核心不中断 | F6 确定性 relay、计数口径和 odom/map 连续性自动验收通过 | `SIMULATED_PASS`，非跨机地图交换验收 |
| R6 重规划 <=2 s | 代理障碍事件与 BRAKE/恢复已观测；生产 EGO 与正式触发协议不存在 | `UNVERIFIED` |
| R7 故障弹性 | F1–F8 8/8 覆盖、窗口完整、逐项 `SIMULATED_PASS` | 代理层 PASS，非硬件/实飞验收 |
| R8 多机支持 | 定义 agent/submap/frontier 接口，未运行多智能体 | `UNVERIFIED` |
| R9 <=30 W | Gazebo 无法测 Atlas 输入功率与温升 | `UNVERIFIED` |
| Atlas/openEuler 部署 | 本轮环境为 Ubuntu 22.04/ROS 2 Humble/Gazebo Harmonic | 不属于本轮证明范围 |

最终证据：

- 基线：`runs/smoke_20260822T073343Z_730`；
- 故障矩阵：`runs/smoke_20260822T073534Z_317`；
- P2：`experiments/results/sensor_validation/p2_20260822T131950Z_1210`；
- Python：46/46 PASS；C++/退出竞态：8/8 PASS；SDF：2/2 Valid。

只有无故障基线用于算法频率与代理 ATE 门；故障矩阵用于 F1–F8 响应验收。
所有正式硬件、生产算法、室外、多机和功耗结论继续保留 `UNVERIFIED`。
