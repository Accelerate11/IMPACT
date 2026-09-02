# IMPACT 复杂场景套件正式报告

验收日期：2026-08-28（Asia/Shanghai）

## 结论

无故障完整飞行算法已在三个独立开顶复杂仓库场景中完成正式同场景双航程比较，场景套件
状态为 **PASS**。第二个场景镜像第二个完整性挑战区，要求同一任务先右绕、再左绕；第三
个场景再加入传感器可提前观测的低门槛，要求算法执行真实向上跨越，从运行证据上排除
“避障方向写死”、只会二维绕行或只为单侧地图调参的解释。

所有展示均关闭 P14 故障注入。Gazebo 保留外墙、隔墙、货架、门架、杂物与运动体，只
移除屋顶；RViz 显示 FAST-LIO、点云、动静态体素、三/四候选轨迹、完整性硬门、最终认证
轨迹和 P13 时延安全包络。GPU 渲染器为
`D3D12 (NVIDIA GeForce RTX 4060 Laptop GPU)`。

## 场景套件

| 场景 | 完整算法选择序列 | 基线最小 Margin | IMPACT 最小 Margin | 提升 |
|---|---|---:|---:|---:|
| `xq_complex_warehouse` | right, right, direct, direct | -0.466501 m | +0.124017 m | +0.590518 m |
| `xq_complex_bidirectional_warehouse` | right, left, direct, direct | -0.492817 m | +0.146311 m | +0.639128 m |
| `xq_complex_3d_warehouse` | right, left, up, direct | -0.509323 m | +0.147368 m | +0.656691 m |

三个基线均执行 `direct × 4`。前两个完整算法恰好干预两次；三维场景恰好干预三次并在
最后一个安全航段恢复任务效用最高的直行，因此同时证明：

1. Margin 不足时，任务收益不能抵消完整性硬约束；
2. 绕行方向由在线环境几何和预测 AL/PL 决定，并非固定向右或向左；
3. 环境安全时，算法不会持续绕行或为追求更高 Margin 无谓牺牲任务效率；
4. 动态障碍制动/TTL 重开和 P13 延迟包络与 P11 决策能够在同一航程共存。
5. 三维候选不是可视化标签：Ground Truth 记录到 `0.660182 m` 的真实垂向包络。

## 双向正式结果

正式原件：
`/home/accelerate/xuanqiong_x1_sim_ws/experiments/results/impact_complex_comparison/gate_20260828T125712Z_315`

轻量证据：
`evidence/COMPLEX_DEMO/bidirectional_gate_20260828T125712Z_315/`

| 指标 | 信息优先基线 | IMPACT 完整算法 |
|---|---:|---:|
| 净前进 | 24.009559 m | 24.000373 m |
| 真值路径长度 | 24.097370 m | 24.841096 m |
| 任务时间 | 132.5575 s | 132.6575 s |
| ATE RMS | 0.088057 m | 0.066186 m |
| 动态重规划 | 2 | 2 |
| 动态障碍最小物理净空 | 1.761571 m | 1.906157 m |
| P13 端到端 p99 | 152.912 ms | 157.282 ms |
| P13 最终安全 Margin | 0.066424 m | 0.064046 m |

完整算法以 `0.743725 m` 额外路径和 `0.100 s` 时间开销换取 `0.639128 m` 的实际
完整性 Margin 提升；定位 ATE 未退化，反而降低 `0.021871 m`。P11/P12/P13 组件、
精确序列、干预计数、任务完成、动态安全、P13 包络、Ground Truth 隔离和外部资产审计
全部通过。

## 三维正式结果

正式原件：
`/home/accelerate/xuanqiong_x1_sim_ws/experiments/results/impact_complex_comparison/gate_20260828T135423Z_11540`

轻量证据：
`evidence/COMPLEX_DEMO/spatial_gate_20260828T135423Z_11540/`

冻结 Gate 精确观察到三组干预：`direct→right`、`direct→left`、`direct→up`。实际最低
Margin 从 `-0.509323 m` 提升到 `+0.147368 m`，提升 `+0.656691 m`；Ground Truth
路径仅增加 `1.104322 m`，IMPACT 任务时间反而减少 `7.4725 s`。两臂均飞满约 24 m，
完整算法 ATE RMS 为 `0.068229 m`，动态障碍最小物理净空为 `2.073552 m`，P13 p99
为 `155.707 ms`。完整验收见
`docs/COMPLEX_3D_FULL_AUTONOMY_REPORT.md`。

## 设计与冻结过程

- revision 1 在首个单臂诊断中证明 `right→left`，但第一门架留下 13.4 mm 裕度缺口；
- revision 2 将门架移出认证保护管并使用 0.70 m 净空中心线，暴露终点 z 向 PL 峰值；
- revision 3 增加两层侧置水平观测架、外移两个局部立柱/门架，中央保持 3.05 m 开口且
  屋顶完全开放；第三次单臂诊断四段 Margin 为
  `+0.2044、+0.1424、+0.1996、+0.1876 m`；
- revision 3 在正式双航程前冻结，正式结果没有再用于修改世界、阈值或算法参数。
- 三维场景 revision 2 在单臂诊断 `gate_20260828T134940Z_10283` 后冻结；其第三批平面
  候选 Margin 均为负，UP 为 `+0.498 m`，冻结后的首次双臂正式 Gate 即 PASS。

所有调整都增加真实可观测几何或修正物理保护管，不降低 `+0.10 m` Margin 储备、
`0.75 m` 物理净空或其他冻结阈值。

## 复现

双向场景 Gazebo + RViz 实时展示：

```bash
cd /home/accelerate/xuanqiong_x1_sim_ws
XQ_COMPLEX_WORLD=xq_complex_bidirectional_warehouse.sdf \
XQ_COMPLEX_LATERAL_OFFSET=0.70 \
bash scripts/run_complex_live_visualization.sh
```

三维场景完整算法 Gazebo + RViz（推荐，无故障注入）：

```bash
cd /home/accelerate/xuanqiong_x1_sim_ws
bash scripts/run_complex_3d_live_visualization.sh
```

双向场景正式双航程 Gate：

```bash
cd /home/accelerate/xuanqiong_x1_sim_ws
XQ_COMPLEX_THRESHOLDS=config/complex_bidirectional_thresholds.json \
bash scripts/run_complex_algorithm_comparison.sh
```

三维场景正式双航程 Gate：

```bash
cd /home/accelerate/xuanqiong_x1_sim_ws
XQ_COMPLEX_THRESHOLDS=config/complex_3d_thresholds.json \
bash scripts/run_complex_algorithm_comparison.sh
```

原始同向场景仍可使用默认命令复现；新增世界、配置和结果均为项目专属文件，没有覆盖
原场景或其他项目地图。

## 证据边界

Git 保存正式比较 JSON、两臂 P11/P12/P13 原始结果、节点图、运行环境、冻结配置、
算法/launch/场景快照与哈希、外部资产审计和 rosbag metadata。大型压缩 rosbag 保留在
WSL 正式目录，Git 轻量证据保存字节数与 SHA-256。

该套件证明三种固定复杂 Gazebo SIL 几何中的方向与三维动作自适应完整飞行，不代表任意未知拓扑、
全局连续动作最优、真实 Livox 噪声、Atlas 满载、HIL 或实机安全。
