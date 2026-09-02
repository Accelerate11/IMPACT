# P14 Fault Injection & Resilient Autonomy 验收报告

## 结论

P14 正式 Gate **PASS**。十种企划故障均被确定性注入并产生预期降级模式；持续 LiDAR
中断实际走完 `NORMAL → CAUTIOUS → RECOVERY → BRAKE → HOVER → LAND`。矩阵轮次
同时保留 P12 动态障碍能力、P13 时延安全闭环和 24 m 全走廊任务。

正式证据：

```text
/home/accelerate/xuanqiong_x1_sim_ws/experiments/results/impact_p14/
gate_20260828T072511Z_12130
```

## 故障矩阵

| ID | 故障 | 观测模式 |
|---|---|---|
| F1 | camera failure | NORMAL（LiDAR 几何回退） |
| F2 | 20% packet loss | NORMAL（故障窗 30 发/6 丢） |
| F3 | CPU load | CAUTIOUS |
| F4 | IMU dropout | CAUTIOUS |
| F5 | timestamp jitter | CAUTIOUS |
| F6 | odometry delay | RECOVERY |
| F7 | covariance inflation | RECOVERY |
| F8 | planner delay | BRAKE |
| F9 | low battery | RETURN |
| F10 | short LiDAR dropout | CAUTIOUS → RECOVERY |

矩阵飞行净前进 `23.9495 m`，ATE RMS `0.08834 m`。传感器代理实际统计为 LiDAR
`1807 received / 8 dropped`、IMU `44697 / 139`、413 条时间戳抖动、15 条延迟里程计、
16 条协方差膨胀；规划超时和 CPU 工作循环也由控制器分别计数。

## 持续 LiDAR 失效安全

持续中断轮次观测到完整有序序列：

```text
NORMAL → CAUTIOUS → RECOVERY → BRAKE → HOVER → LAND
```

初始高度 `1.2000 m`，最终高度 `0.6722 m`，实际下降 `0.5278 m`，最终速度
`0.0000 m/s`；代理实际丢弃 57 帧 LiDAR。算法节点不订阅 Ground Truth，下降量与末速
由独立 evaluator 使用 Gazebo 真值确认。

## P12 / P13 保持性

- P12：动态检测 `0.1575 s`、重规划 `0.0625 s`、动态残留 `0.0200 s`、重开事件有效，
  动态体素峰值 60、最低物理净空 `3.6277 m`、静态保留率 `1.00759`。
- P13：551 个时延样本，端到端 p50/p95/p99/max 为
  `149.78/249.76/298.22/308.36 ms`；最终速度上限 `0.1977 m/s`，安全 AL/Margin
  `0.1600/0.0600 m`。
- 两项保持性 evaluator 与 P14 使用同一条 `23.9495 m` 飞行轨迹和同一世界哈希。

## 研发中保留的失败

1. 初版 P12 保持性把低频地图观察时间误当成障碍真实离开时刻，产生负重开延迟；改为
   以场景 `occupied true→false` 为因果锚点，同时保留 raw observer skew。
2. 初版 LAND 依赖持续中断后冻结的 LIO 高度，状态虽进入 LAND 但机体未下降；改为
   命令积分下降，Ground Truth 仍只在 evaluator 核验。
3. 一轮随机几何使 P11 在末段重复硬拒绝并永久悬停；增加严格前置条件下的有界最小
   激励重采样，不执行被拒轨迹、不放宽完整性 Gate。

## 验证与可视化复现

```text
102/102 Python 算法测试 PASS
14/14 isolated ROS packages build PASS
P14 unified Gate PASS
Gazebo + RViz combined replay smoke PASS
```

重新运行正式 Gate：

```bash
cd /home/accelerate/xuanqiong_x1_sim_ws
bash scripts/run_p14_fault_gate.sh
```

同时打开 Gazebo 与 RViz：

```bash
cd /home/accelerate/xuanqiong_x1_sim_ws
bash scripts/view_p14_combined.sh
```

Gazebo 使用正式矩阵录制，开顶且保留墙体；RViz 显示飞行路径、动态地图、候选轨迹、
时延安全量以及故障状态阶梯。双窗口已实际启动验证。

## 证明边界

本阶段证明固定单机 Gazebo SIL 和项目内故障代理下的软件容错逻辑。CPU load 不是 Atlas
满载/NPU 实测，20% packet loss 不是跨机网络验收，low battery 不是电池硬件放电试验；
传感器驱动、openEuler、CUAV、Atlas 功耗温升、HIL 与实机安全仍须硬件阶段验证。
