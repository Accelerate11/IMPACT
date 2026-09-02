# IMPACT 三维复杂场景完整正常飞行验收报告

验收日期：2026-08-28（Asia/Shanghai）

## 结论

`xq_complex_3d_warehouse` 的冻结双臂 Gazebo SIL Gate 为 **PASS**。完整算法在同一条
24 m 任务中依次执行 `right → left → up → direct`，同时完成动态障碍检测、制动、
TTL 通道重开、FAST-LIO 定位、P11 完整性硬认证和 P13 p99 时延安全闭环。展示和正式
Gate 均不启动 P14 故障注入。

本场景保留外墙、隔墙、货架、门架、杂物和两个运动体，仅移除屋顶。第三滚动航段设置
横跨直/左/右通道的低门槛。它在真实 Mid-360 视场内可提前建图；三条平面候选虽可完成
物理穿越，但不足以满足冻结的 `+0.10 m` 定位完整性储备，向上候选则在门槛上方保持
0.70 m 高度偏移并于航段末恢复巡航高度。

## 冻结与正式轮次

- 单臂设计诊断：
  `/home/accelerate/xuanqiong_x1_sim_ws/experiments/results/impact_complex_comparison/gate_20260828T134940Z_10283`
- 冻结配置：`config/complex_3d_thresholds.json` revision 2；正式双臂结果未用于调参；
- 正式双臂原件：
  `/home/accelerate/xuanqiong_x1_sim_ws/experiments/results/impact_complex_comparison/gate_20260828T135423Z_11540`
- Git 轻量证据：
  `evidence/COMPLEX_DEMO/spatial_gate_20260828T135423Z_11540/`

## 正式结果

| 指标 | 信息优先基线 | IMPACT 完整算法 |
|---|---:|---:|
| 选择序列 | direct × 4 | right, left, up, direct |
| 净前进 | 24.002391 m | 23.999173 m |
| 实际最小完整性 Margin | -0.509323 m | +0.147368 m |
| Ground Truth 最大垂向偏移 | 0.115599 m | 0.660182 m |
| Ground Truth 路径长度 | 24.113970 m | 25.218293 m |
| 任务时间 | 140.1300 s | 132.6575 s |
| ATE RMS | 0.089344 m | 0.068229 m |
| 动态重规划事件 | 3 | 2 |
| 动态障碍最小物理净空 | 1.873013 m | 2.073552 m |
| P13 端到端 p99 | 158.777 ms | 155.707 ms |
| P13 最终安全 Margin | 0.063230 m | 0.064905 m |

完整算法实际 Margin 提升 `+0.656691 m`，增加 `1.104322 m` 路径，但任务时间没有
增加，反而减少 `7.4725 s`。正式 Gate 的精确干预对为
`direct→right`、`direct→left`、`direct→up`，干预次数恰为 3；最后一段恢复任务效用
最高的直飞，证明安全区不会无谓绕行。

## 第三段三维决策证据

冻结前单臂诊断中，第三批候选预测 Margin 为：

| 候选 | 预测 Margin | 硬约束结果 |
|---|---:|---|
| high_information_direct | -0.095 m | REJECT |
| geometry_rich_right | -0.098 m | REJECT |
| geometry_rich_left | -0.103 m | REJECT |
| geometry_rich_up | +0.498 m | ACCEPT |

正式结果另以 Ground Truth 验证实际垂向包络为 `0.660182 m`，因此不是只改变候选标签；
算法确实执行了三维爬升轨迹。

## 公平性与边界

两臂使用同一冻结世界、Mid-360/IMU、FAST-LIO、动态地图、碰撞/能量约束、P13 时延
安全参数和 P7 train-only 标定；唯一差异是任务效用最大化之前是否执行完整性硬过滤。
算法节点不订阅 Ground Truth，真值只进入 evaluator。外部 Gazebo 地图和模型在正式轮次
前后逐字节一致。GPU renderer 为
`D3D12 (NVIDIA GeForce RTX 4060 Laptop GPU)`。

该 Gate 证明固定复杂 Gazebo SIL 场景中的左右方向自适应、三维越障、动态通道重开和
完整航程协同，不替代真实 Livox 噪声、Atlas 满载、HIL 或实机飞行安全认证。

## 复现

Gazebo + RViz 完整正常飞行（无故障注入）：

```bash
cd /home/accelerate/xuanqiong_x1_sim_ws
bash scripts/run_complex_3d_live_visualization.sh
```

冻结双臂 Gate：

```bash
cd /home/accelerate/xuanqiong_x1_sim_ws
XQ_COMPLEX_THRESHOLDS=config/complex_3d_thresholds.json \
bash scripts/run_complex_algorithm_comparison.sh
```
