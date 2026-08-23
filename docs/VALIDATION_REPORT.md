# 玄穹-X1 Gazebo SIL 验证记录

## IMPACT P1 真飞控增量（2026-08-22）

在原算法代理 SIL 旁路增加了 ArduCopter 4.5.7 SITL、Gazebo Harmonic
8.13.0、ArduPilotPlugin 与 MAVROS 的真飞控闭环，不改变原代理指标结论。
正式轮次 `runs/p1_20260822T125115Z_388` 连续运行 600 s，80.034 s 内完成
GUIDED→ARM→TAKEOFF→30 s HOVER→LAND，终态已解锁且 FCU 仍连接。rosbag
记录 38,231 条 MAVROS state/odom/IMU/GPS 消息；外部地图模型哈希审计 PASS。

本结果只将企划 P1 标记为 `SIMULATED_PASS`，不提升 FAST-LIO2、三维地图、
EGO-Planner、IMPACT 或 Atlas 硬件条目的 `UNVERIFIED` 状态。

## IMPACT P2 Mid-360S-like 增量（2026-08-22）

正式轮次
`experiments/results/sensor_validation/p2_20260822T131950Z_1210` 连续单调采集
600.435 s。LiDAR 为 10.00008 Hz、5,996 帧、每帧 23,040 点；IMU 为
200.00083 Hz、119,912 帧。时间戳严格单调，TF、frame、点云字段、点数、NaN/Inf
与最大间隔共 11 项 Gate 全部通过。rosbag 录制 365,535 条并分为 10 个 zstd 文件；
外部地图/模型哈希审计 PASS，结束后无核心进程残留。

P2 详细参数、逐项证据、主机 realtime 校时差异和 P3 接口边界见
`docs/P2_SENSOR_VALIDATION_REPORT.md`。本结果将 P2 标记为 `PASS`，但真实 Livox
扫描机制、livox_ros_driver2 与 FAST-LIO2 仍为 `UNVERIFIED`。

更新时间：2026-08-22（Asia/Shanghai）

## 结论

本工作区的独立 Gazebo 场景、专属 Gazebo/ROS 2 桥、二维算法代理、闭环控制、
故障注入、指标采集和隔离审计已经原生构建并实际运行。30 秒无故障严格基线与
120 秒 F1–F8 故障矩阵均以退出码 0 完成。

结论层级仍为 `GAZEBO_SIL_PROXY`。它证明本工作区内的接口、数据链、二维代理算法和
Sentinel 降级闭环能够运行，不得替代 FAST-LIO2、EGO-Planner、Atlas HIL 或实飞验收。

## 最终构建与静态测试

| 项目 | 结果 |
|---|---:|
| 隔离构建 | PASS，5 packages |
| 源码树 SHA-256 | `366d835913fbaf6910313540a37948fd59f95898631d901a4bd2dea1bb0679e4` |
| 安装树 SHA-256 | `4fcb9f30ba7c3ef176c4720e44d27d1b3fb8f237a29664f4967fd4c2dff132c5` |
| Python 算法/验收契约 | 46/46 PASS |
| Gazebo world/model SDF | `Valid` / `Valid` |
| C++ 桥与退出竞态 | 8 tests，0 errors，0 failures |

## 30 秒无故障严格基线

证据目录：
`/home/accelerate/xuanqiong_x1_sim_ws/runs/smoke_20260822T073343Z_730`

运行命令：

```bash
bash scripts/run_smoke.sh --duration 30 --require-algorithm-pass
```

| 指标 | 观测 | 判定 |
|---|---:|---:|
| 室内代理 ATE RMS | 0.1535 m（门槛 0.3 m） | `SIMULATED_PASS` |
| raw LiDAR | 均值 10.001 Hz，最差 1 s 窗 10 Hz | `SIMULATED_PASS` |
| raw IMU | 约 200 Hz | `SIMULATED_PASS` |
| 定位质量/地图 | 10 Hz | `SIMULATED_PASS` |
| 代理 odom | 均值 19.804 Hz，最差 1 s 窗 14 Hz | `SIMULATED_PASS` |
| 代理规划 | 11 次、10 次接受，最大 0.178 s | `OBSERVED_PROXY_ONLY` |
| 障碍触发代理 | 4 次、3 次直接接受；成功事件最大 0.031 s | `OBSERVED_PROXY_ONLY` |
| 进程/配置/隔离审计 | 无崩溃；配置哈希 PASS；旧资产前后哈希一致 | PASS |

一次障碍触发发生时，起点已在当前膨胀安全缓冲区内，系统先制动；0.5 秒后的停止状态
重规划被接受，总恢复约 0.615 秒。该行为是代理层安全恢复证据，但仍不计正式 R6。

## 120 秒 F1–F8 故障矩阵

证据目录：
`/home/accelerate/xuanqiong_x1_sim_ws/runs/smoke_20260822T073534Z_317`

运行命令：

```bash
bash scripts/run_smoke.sh --with-faults --duration 120
```

- 8/8 计划故障均被观测，无缺失或意外 fault ID；
- 8/8 响应窗口完整，逐项 `status=SIMULATED_PASS`；
- F1/F2：进入 `GEOMETRY_ONLY`，核心 odom/map 连续；
- F3：规划 deadline/BRAKE 代理响应通过；
- F4：依次进入 `CAUTIOUS -> HOVER -> LAND`，恢复后回到 `NORMAL`；
- F5：进入 `RELOCALIZE`，恢复后回到 `NORMAL`；
- F6：项目内确定性约 20% ground-link 丢包及核心流连续性通过；
- F7：进入 `ESSENTIAL_ONLY`，只卸载非关键 OAER 作业；
- F8：进入 `RETURN`，恢复后回到 `NORMAL`；
- 外部 world/model 资产运行前后 SHA-256 一致。

故障运行中的定位质量/地图全程频率失败是 F4 主动暂停 LiDAR 3.2 秒的预期后果；
120 秒代理 ATE 漂移也不覆盖无故障严格基线结论。故障验收与无故障算法门分开判读。

## 本轮修正的根因

1. 大点云在 WSL DDS 链上使用 best-effort depth-5，端到端仅约 6–8.5 Hz；本地
   sensor-to-estimator 链改为 reliable depth-20，故意丢包只保留在项目 relay。
2. 3D Mid-360 点云直接丢弃 z 投影到二维，使地板/天花板回波形成伪障碍闭环；现按
   `[-0.55, 0.85] m` 机体系高度切片，并用 NumPy 向量化解析。
3. OAER 前沿目标未约束到膨胀障碍后的起点连通域；现与 A* 共用同一可达性模型。
4. 在线路径检查只看原始占用格；现与规划共用机体、定位 3σ、动态余量的膨胀模型。
5. 误将 0.8 s 规划截止时间当成车辆反应延迟；现独立使用 0.20 s 感知/控制反应时间。
6. Python 障碍膨胀阻塞控制回调；现用 NumPy 切片实现等价欧氏圆盘膨胀。

## 明确保留的未验证项

- 真实 FAST-LIO2 ESIKF、点到面优化与正式室内/室外 ATE；
- 真实 EGO-Planner B-spline 优化和“首次障碍确认到安全轨迹”的正式 R6；
- 真实 5 cm 三维地图精度；
- Atlas 200I DK A2 的 30 W、温升与满载实时性；
- 跨主机 DDS、2–3 机 SITL/HIL 与多机实飞。
