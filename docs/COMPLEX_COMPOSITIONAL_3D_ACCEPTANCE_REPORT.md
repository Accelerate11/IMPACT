# 复杂组合三维场景完整算法验收报告

## 结论

复杂组合三维正常飞行算法正式双臂 Gate 为 **PASS**。完整性约束臂在同一任务中精确
执行 `geometry_rich_right → geometry_rich_left → geometry_rich_up_right →
high_information_direct`，信息优先对照臂执行 `high_information_direct × 4`。两臂均完成
24 m 全航程、动态障碍制动与通道重开，并保持 P13 时延安全包络。

本 Gate 不启动 P14 故障注入。Gazebo Ground Truth 只进入 evaluator/logger，不进入
FAST-LIO、地图、候选生成、轨迹筛选或飞行控制器。

## 正式证据

- WSL 原始轮次：
  `experiments/results/impact_complex_comparison/gate_20260902T122517Z_286/`
- Git 轻量归档：
  `evidence/COMPLEX_DEMO/compositional_gate_20260902T122517Z_286/`
- 汇总 Gate：`COMPLEX_COMPOSITIONAL_3D_FULL_AUTONOMY_COMPARISON`
- 状态：`PASS`
- 场景：`xq_complex_compositional_warehouse.sdf`
- 渲染器：`D3D12 (NVIDIA GeForce RTX 4060 Laptop GPU)`
- 外部 Gazebo 地图/模型：前后 SHA-256 完全一致

轻量归档保存 comparison JSON、P11/P12/P13 单臂结果、配置与算法源码快照、节点图、
run.env、launch/rosbag 日志和 rosbag metadata。两份大型压缩 rosbag 留在 WSL，并由
`LARGE_ARTIFACTS.sha256` 与 `LARGE_ARTIFACTS.bytes` 做字节级索引。

## 冻结 Gate 与正式结果

阈值 revision 3 在两条独立预正式诊断臂均通过 P11/P12/P13 后冻结；正式双臂之前没有
再按结果修改阈值。关键冻结要求与结果如下：

| 指标 | 信息优先 | 完整性约束 | Gate |
|---|---:|---:|---:|
| 实际最低完整性 Margin | -0.078022 m | +0.158314 m | 对照 ≤ 0；约束 ≥ 0.10 m |
| Ground Truth 路径长度 | 24.129211 m | 25.720048 m | 额外路径 ≤ 2.0 m |
| 任务时间 | 164.3000 s | 148.8125 s | 约束臂开销 ≤ 8.0 s |
| ATE RMS | 0.096067 m | 0.067872 m | 两臂 ≤ 0.35 m |
| 净前进 | 24.007881 m | 23.998857 m | 两臂 ≥ 23.5 m |
| 最大垂向位移 | 0.117629 m | 1.052533 m | 对照 ≤ 0.15；约束 ≥ 0.50 m |
| 同时横向+垂向位移 | 0.000037 m | 0.801914 m | 对照 ≤ 0.15；约束 ≥ 0.50 m |
| 动态重规划次数 | 5 | 3 | 两臂 ≥ 2 |
| 最小动态障碍物理净空 | 2.604683 m | 3.503246 m | 两臂 ≥ 0.75 m |
| P13 端到端 p99 | 177.178 ms | 186.868 ms | 两臂 ≤ 220 ms |
| P13 最终安全 Margin | 0.060 m | 0.060 m | 两臂 ≥ 0.060 m |

完整性硬过滤带来 `+0.236336 m` 的实际最低 Margin 增益；额外路径为 `1.590837 m`，
任务时间反而减少 `15.4875 s`。汇总 JSON 中 26 项检查全部为 `true`。

## 本轮算法优化

### 1. 稀疏动态体素地图

未知空间的 free-ray 不再为每个自由格分配完整体素对象，而是存入按 XY 列组织的
稀疏自由证据；只有已存在的占据/动态体素才接受逐体素 free 更新。端点判定缓存列自由
查询，并仅在列证据不足时回退到 27 邻域搜索。地图状态快照在一次遍历内同时生成统计量
和动态路径信息。

这一修改把典型活动体素量由约 5.9 万降到约 1.2–1.5 万；scan 更新由约 67 ms 降到
31–33 ms，端点阶段由约 34 ms 降到约 5 ms，状态发布由约 31 ms 降到约 5–8 ms。

### 2. 端到端时延的因果闭合

若当前 odometry 已覆盖传感器时间戳，定位阶段在传感器回调内完成；而来自同一 P12
数据链的地图到达，本身就是定位依赖已完成的保守上界，可用于闭合偶发滞后的重复 odom
订阅。所有时间仍由单调时钟按真实消息到达顺序测量，没有扣除或伪造处理时间。

### 3. 有界观测信息记忆

P6 新增 `TemporalInformationMemory`：在 map 坐标系中对 FAST-LIO 实际观测到的信息矩阵
做有界指数遗忘累积。复杂组合场景显式启用 3 s horizon、最多 20 个等效帧；默认值为
关闭，因此其他项目和既有场景行为不变。该机制让 P11 对多步候选的预测信息增益和实际
执行后的 P6 完整性评价使用同一物理量。

### 4. 动态障碍生命周期

初始 revision 2 正式尝试暴露出一个真实回归：停止的运动体过早转为可逆静态，通道在
障碍仍占据时被重开，最小物理净空为负。最终采用 15 s 停驻确认窗；移动障碍在确认窗
结束前离开，而新揭露的固定几何仍能在充分证据后转为静态。失败轮次保留在 WSL，未被
作为正式 PASS 证据。

## 验证

- 自研 Python 测试：`120 passed`
- 正式双臂 P11/P12/P13 evaluator：全部 PASS
- 外部资产隔离：PASS
- Ground Truth 算法订阅审计：PASS
- 故障注入关闭：PASS
- GPU 渲染：D3D12 / RTX 4060 Laptop GPU

## 复现

Gazebo + RViz 联合可视化（开顶、保留全部墙体，正常飞行算法，无故障注入）：

```bash
cd /home/accelerate/xuanqiong_x1_sim_ws
bash scripts/run_complex_compositional_live_visualization.sh
```

正式无 GUI 双臂验收：

```bash
cd /home/accelerate/xuanqiong_x1_sim_ws
XQ_COMPLEX_THRESHOLDS=config/complex_compositional_thresholds.json \
  bash scripts/run_complex_algorithm_comparison.sh
```

## 验证边界

当前结论属于 WSL2 + Gazebo SIL。Atlas 200I DK A2 上的实时负载、温升与功耗，Mid-360S
真实噪声和时间同步、CUAV ExternalNav/HIL、实机气动与安全场地测试仍属于硬件部署阶段。
