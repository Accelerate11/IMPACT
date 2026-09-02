# 复杂场景完整飞行算法验收报告

验收日期：2026-08-28（Asia/Shanghai）

## 结论

复杂仓库完整正常飞行算法正式比较为 **PASS**。本轮不启动 P14 故障注入节点，不展示
故障降级；Gazebo 与 RViz 展示的是从仿真传感器、FAST-LIO、完整性估计、滚动候选规划、
动态地图到时延安全控制的完整在线闭环。

正式原件：
`/home/accelerate/xuanqiong_x1_sim_ws/experiments/results/impact_complex_comparison/gate_20260828T122049Z_319`

Git 轻量证据：
`evidence/COMPLEX_DEMO/gate_20260828T122049Z_319/`

## 场景与算法链

场景为 34 m × 16 m 开顶仓库/办公混合空间：只移除屋顶，保留外墙、隔墙、六层连续
观测货架、局部货架、三组门架、杂物、两个完整性挑战面板、几何丰富侧翼和两个运动体。
正式运行使用 `D3D12 (NVIDIA GeForce RTX 4060 Laptop GPU)`。

在线算法链如下：

```text
Gazebo LiDAR/IMU
  → FAST-LIO 定位与点面几何
  → P6 方向完整性/Protection Level
  → P10 在线 Information Map
  → P11 四段滚动 Frontier 候选 + 完整性/碰撞/能量硬过滤
  → P12 LiDAR-only 动静态体素、BRAKE、TTL 清除与通道重开
  → P13 实测端到端 p99 进入 Alert Limit 与速度包络
  → 全 24 m 闭环飞行
```

Ground Truth 只进入独立 evaluator；三个核心算法节点的 DDS 订阅图均已审计，不订阅
`/xq/eval/*`、`ground_truth` 或 Gazebo model pose。

## 正式同场景对比

两航程使用相同世界 SHA-256、冻结 P7 标定、传感器、FAST-LIO、动态地图、碰撞/能量门、
P13 时延安全和任务距离。唯一差异是 P11 是否在任务效用选择前启用完整性硬可行过滤。

| 指标 | 信息优先基线 | IMPACT 完整算法 |
|---|---:|---:|
| 选择序列 | direct × 4 | right, right, direct, direct |
| 实际最小完整性 Margin | -0.466501 m | +0.124017 m |
| 净前进 | 24.000902 m | 23.999299 m |
| 真值路径长度 | 24.111017 m | 24.823639 m |
| 任务时间 | 132.750 s | 132.800 s |
| ATE RMS | 0.064054 m | 0.073515 m |
| 动态重规划 | 2 | 2 |
| 动态障碍最小物理净空 | 1.847896 m | 1.753211 m |
| P13 端到端 p99 | 160.220 ms | 161.768 ms |
| P13 最终安全 Margin | 0.062439 m | 0.061590 m |

完整性硬约束带来 `+0.590518 m` 的实际 Margin 提升，代价为 `0.712621 m` 额外路径和
`0.050 s` 任务时间开销。两航程都飞完 24 m，并各完成两次动态制动/重开；所有组件
evaluator 和 17 项比较检查均通过。

## 本轮算法修正

1. 修复 `lateral_offset_m` 与 `lateral_candidate_shape` 只传给动态地图、未传给实际
   P13 飞行控制器的 launch 参数链问题。
2. 按两侧实体障碍共同决定的净空带设计候选通道：货架和观测侧翼外移一个 0.25 m
   地图体素，认证横移设为 0.68 m；没有降低任何冻结验收阈值。
3. 侧向候选保持到越过挑战面板及保护半径后再回中，消除切过面板尾角的轨迹时序错误。
4. P11–P13 定向回归 26/26 通过；SDF 为 `Valid`，正式双航程和外部资产隔离审计通过。

## 可视化复现

在 Ubuntu-22.04 终端执行：

```bash
cd /home/accelerate/xuanqiong_x1_sim_ws
bash scripts/run_complex_live_visualization.sh
```

脚本同时打开 Gazebo 与 RViz，使用独立 `ROS_DOMAIN_ID`、`GZ_PARTITION` 和项目专属资源
路径。Gazebo 为开顶保留墙体的复杂场景；RViz 显示点云、FAST-LIO 路径、动静态体素、
Frontier 候选、认证轨迹、完整性选择和 P13 时延安全状态。退出终端中的脚本只清理本次
可视化会话。

## 证据与边界

仓库保存比较 JSON、P11/P12/P13 原始结果、运行环境、节点图、配置/源码/场景快照、
配置哈希、外部资产前后哈希、隔离审计和 rosbag metadata。约 618 MiB 的两组压缩
rosbag 保留在 WSL 原始目录，Git 证据中保存其字节数与 SHA-256。

本报告证明固定复杂 Gazebo SIL 场景中的完整正常飞行闭环，不声称覆盖任意未知拓扑、
Atlas 满载、真实 Livox 噪声、HIL 或实机安全。
