# P11 Integrity-Constrained Exploration 验收报告

验收完成日期：2026-08-27（Asia/Shanghai）

正式证据：

- ROS 合同：`/home/accelerate/xuanqiong_x1_sim_ws/experiments/results/impact_p11/contract_20260827T053924Z_1726`
- 24 m 双臂 Gazebo Gate：`/home/accelerate/xuanqiong_x1_sim_ws/experiments/results/impact_p11/gate_20260827T101925Z_22733`
- 最终 Gazebo/RViz 录制：`/home/accelerate/xuanqiong_x1_sim_ws/experiments/results/impact_p11/visual_20260827T102347Z_24548`

## 结论

P11 为 `PASS`。无人机不再执行一个 7.5 m 决策窗后停止，而是从 Gazebo 真值
`x=-12 m` 飞至 `x≈+12 m`，完成 24 m 全走廊。控制器以 FAST-LIO 坐标中的相对任务
终点为准，按 `7.5 + 7.5 + 7.5 + 1.5 m` 四段滚动生成 Frontier 观察轨迹；每段都
重新读取在线点云、Information Map 和 P6/P7 完整性状态，发布新 batch 并重新硬认证。

约束臂序列为
`geometry_rich_right -> high_information_direct -> geometry_rich_right -> high_information_direct`。
信息优先臂四段均选择 `high_information_direct`。两臂都飞完整条走廊，但只有约束臂
全程保持冻结的 `M_min >= 0.10 m` 储备。

## 算法与硬约束

P11 保留 P5 Frontier 和局部 B-spline 作为上游候选生成，对同一 Frontier 的多个观察
轨迹预测 `M_min(tau) = min_t(AL(t) - PL(t))`。候选必须先同时满足：

- `M_min >= 0.10 m`；
- `P_collision <= 0.01`；
- `E_flight + E_return <= E_remaining`。

之后才按 `J=w_I I-w_T T-w_E E` 排序。Margin 不进入效用；无硬可行候选时
fail-closed。每个滚动 batch 使用唯一 batch/trajectory ID，旧决策不能被下一段复用。

专属 P11 世界保持开放顶部和全部墙体。侧墙下方加入不侵入航道、不遮挡顶视图的水平
观测边，为 Mid-360 提供持续 z 法向约束；同时保留两处局部完整性选择岛。既有其他
Gazebo world/model 未修改。

## 正式 24 m 双臂 Gazebo Gate

固定世界、随机种子 `20260827`、初始状态、P7 train-only 校准、候选规则和阈值，
分别运行 `information_only` 与 `integrity_constrained`。每臂独立启动 Gazebo、
LiDAR/IMU、FAST-LIO、P6、Information Map、P11、控制器和 evaluator。

| 指标 | Information-only | Integrity-constrained |
|---|---:|---:|
| 决策 batch | 4 | 4 |
| 选择序列 | direct × 4 | right, direct, right, direct |
| 实际最低 Margin | -0.299559 m | +0.121361 m |
| 四段最低 Margin | -0.2153, +0.8741, -0.2996, +0.7847 m | +0.3201, +0.8221, +0.1214, +0.7918 m |
| 执行信息收益（总计） | 4.00 | 3.50 |
| 平均每 batch 信息收益 | 1.000 | 0.875 |
| ATE RMS | 0.077903 m | 0.086599 m |
| 真值路径长度 | 24.098819 m | 24.359144 m |
| 真值净前进 | 23.984448 m | 23.993332 m |
| 真值终点 x | +11.984448 m | +11.993332 m |
| 任务时间 | 108.30 s | 108.30 s |

约束臂使实际最低 Margin 提升 `0.420920 m`；平均每 batch 信息收益损失 `0.125`，
额外路径 `0.260325 m`，任务时间开销 `0 s`，ATE 增量 `0.008695 m`。24 项配对检查
全部为 true，包括全走廊终点、滚动 batch 完整、预测硬拒绝、实际储备、碰撞与返航
能量、代价有界、冻结校准、Ground Truth 隔离和 Margin 不进入效用。

## 可视化验收

最终录制轮次为 `PASS`：

- Gazebo `state.tlog`：1,404,928 bytes；
- RViz rosbag 目录：189,121,443 bytes；
- Gazebo 世界开放顶部、墙体和内部特征全部保留；
- RViz 显示四个滚动 batch 的红色未约束候选和绿色硬可行执行轨迹；
- 同时显示 FAST-LIO/真值轨迹、点云、surfel 信息与决策说明；
- WSLg 双窗口实际启动验证，RViz OpenGL 4.2 与 Gazebo 回放均正常。

## 构建、测试与隔离

- `xq_autonomy` 全量回归：83/83 PASS；
- P11 定向算法测试：6/6 PASS；
- 13 个 ROS 2 包隔离构建成功；
- P11 合同、正式双臂飞行、可视化录制/回放全部 PASS；
- 源码树 SHA-256：`8d97f71e19f5bcb6ccb8de6b472ec19212a0f64ca959d7656fd1c7ecc939db77`；
- 安装树 SHA-256：`51777d7f45e7e32cdb7dc9702c7560b396702a42cb94849e170f0d3cc9ee80e0`；
- P7 校准 SHA-256：`771bdffcf3d4422d4641424dab326a08aa5be2b0dffd7f9d2f2f9ff82ea9f038`；
- P11 world SHA-256：`79b5ed6d235956af6e5b8fcd6a5968eaa1813cc4aa63d4e28bcfca37634e27b8`；
- 既有外部项目地图/模型前后字节级不变。

## 复现

```bash
cd /home/accelerate/xuanqiong_x1_sim_ws
bash scripts/build_isolated.sh
bash scripts/run_p11_contract_gate.sh
bash scripts/run_p11_flight_gate.sh
bash scripts/run_p11_visual_capture.sh
```

同时回放本报告冻结轮次：

```bash
bash scripts/view_p11_combined.sh \
  /home/accelerate/xuanqiong_x1_sim_ws/experiments/results/impact_p11/visual_20260827T102347Z_24548
```

## 证明边界

P11 证明的是固定静态 Gazebo SIL 中，24 m 走廊上的有限 Frontier 多轨迹滚动硬认证与
选择闭环。它不证明动态障碍、任意未知环境、连续动作全局最优、真实 Livox 噪声、
Atlas 满载实时性或实机安全。P12 动态地图、P13 时延安全、P14 故障注入和硬件闭环
仍未完成。
