# P10 Minimum-Excitation Active Perception 设计记录

状态：`PASS`。本文件保留算法设计；正式验收结论见
`docs/P10_MINIMUM_EXCITATION_ACCEPTANCE_REPORT.md`。

前置条件：P9 Integrity Margin 正式 Gate `PASS`，提交 `db3686f`。

## 1. 要解决的问题

当名义轨迹的 `M_min < M_reserve` 时，P10 不直接放弃任务，也不把完整性写成可被其他
收益抵消的软代价。系统只在少量离散恢复候选中寻找：

1. 预测后整条轨迹满足 `M_min >= M_reserve`；
2. 在所有满足硬约束的候选中，额外时间、能耗代理和路径最小。

这与“选择可观测性最高的动作”不同。一个动作即使带来更大的信息矩阵，只要另一个
更小动作已经恢复完整性且代价更低，就必须选择后者。

## 2. 冻结候选集合

P10 v1 使用原企划的有限动作集：

| ID | 动作 |
|---:|---|
| 0 | baseline |
| 1 | left lateral offset，0.40 m |
| 2 | right lateral offset，0.40 m |
| 3 | up offset，0.25 m |
| 4 | down offset，0.25 m |
| 5 | slow trajectory，时间尺度 0.50 |
| 6 | short hover / observation，1.0 s |
| 7 | backtrack to previous high-quality pose |

`yaw-only` 只作为 Gate P10 对比组，不进入 IMPACT v1 最小激励动作集合。

## 3. Information Map 与量纲

每个本地 surfel 保存：位置、单位法向、`static_confidence`、`geometry_quality` 和
`last_seen`。候选轨迹附近的有效 surfel 构造：

`Lambda_hat = sum(w_i * n_i * n_i^T)`

其中 `w_i` 同时包含静态置信、几何质量、时间衰减和显式的逆方差尺度。因此
`Lambda_hat` 与 `P^-1` 量纲一致，不能把无量纲法向外积直接与协方差逆矩阵相加。

未来协方差按 information form 顺序更新：

`P_(k+1) = inverse(inverse(P_k) + Lambda_hat_k)`

为防止重复 surfel 的独立性近似把未来协方差压到物理上不可信的零值，在线预测在每一步
施加冻结的特征值下界 `lambda_min(P) >= 1e-5 m^2`（单轴标准差约 `3.16 mm`）。
它是预测模型的不确定性下界，不改变 P9 的 Margin 定义或 `M_reserve=0.10 m`。

所有增量必须半正定；非有限值、非单位法向、非正定先验或不对齐剖面均 fail-closed。

## 4. 硬约束与选择

每个候选的每个采样点复用 P9 公式：

`PL_j = k95 * sqrt(a_j^T * P_j * a_j)`

`M_j = AL_j - PL_j`

`M_min = min_j(M_j)`

只有 `M_min >= M_reserve` 的候选进入代价比较：

`J = DeltaT + lambda_E * E_extra + lambda_D * D_extra`

若 baseline 已满足硬约束，直接保留 baseline，不产生无必要激励；若没有任何候选恢复
完整性，返回 `recovery_found=false`，不向下游发布轨迹。

## 5. Gate P10

固定 `long_corridor`、随机种子、初始状态、目标和地图，对比：

- Baseline；
- Yaw-only；
- Minimum-Excitation。

三组统一记录 ATE、weak-direction error、最小 Integrity Margin、额外路径和任务时间。
Ground Truth 只能进入 evaluator/logger，不能进入 Information Map、候选生成、预测器或
选择器。正式 Gate 阈值必须在正式数据生成前冻结，不能根据结果回调。

从 P8 起的可视化策略继续有效：P10 必须同时提供 headless Gate、Gazebo record/replay
和 RViz 候选/预测协方差/Margin/最终选择可视化，Gazebo 保留墙体并移除房顶。

## 6. 最终实现与验收状态

- 纯算法文件：`src/xq_autonomy/xq_autonomy/minimum_excitation.py`；
- 回归测试：`src/xq_autonomy/test/test_p10_minimum_excitation.py`；
- P10 与既有算法全量回归：77/77 `PASS`；
- 13 个 ROS 2 包隔离构建成功；
- ROS 2 算法/传输契约 Gate：`PASS`；baseline 预测 Margin `0.058574 m`，只有
  left-lateral 恢复为可行并被发布。该确定性契约只验证算法与传输，不替代飞行 Gate；
- 正式 long-corridor 三组 Gazebo + FAST-LIO 飞行 Gate：`PASS`。共同决策快照下
  baseline/yaw-only 预测最低 Margin 均为 `0.045083 m`，minimum-excitation 为
  `0.419631 m`；实际最低 Margin 分别为 `-0.320630 / -0.448158 / +0.104633 m`；
- minimum-excitation 选择并执行 `right_lateral`，相对 baseline 仅增加 `0.065915 m`
  路径，任务时间开销 `0 s`；16 项汇总检查全部通过；
- 独立 Gazebo/RViz 可视化录制：`PASS`。Gazebo 状态日志与 ROS bag 均已封盘，世界
  移除房顶但保留墙体。
