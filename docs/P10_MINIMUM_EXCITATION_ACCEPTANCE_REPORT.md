# P10 Minimum-Excitation Active Perception 验收报告

验收完成日期：2026-08-23（Asia/Shanghai）

正式三组飞行证据：

`/home/accelerate/xuanqiong_x1_sim_ws/experiments/results/impact_p10/gate_20260823T154534Z_16472`

正式可视化证据：

`/home/accelerate/xuanqiong_x1_sim_ws/experiments/results/impact_p10/visual_20260823T154916Z_18660`

## 结论

P10 为 `PASS`。当 P9 的名义轨迹不能满足 `M_min >= 0.10 m` 时，系统从冻结的有限
恢复集合预测未来信息增量，只允许满足完整性硬约束的候选进入代价比较，并执行其中
额外路径、时间和能耗代理最小的候选。若 baseline 本身可行则保持 baseline；若所有
候选均不可行则 fail-closed，不向下游发布恢复轨迹。

P10 没有把可观测性或 Integrity Margin 写成可被任务收益抵消的软代价。Ground Truth
只进入独立 evaluator/logger，不进入 surfel Information Map、候选生成、协方差预测、
选择器或飞行控制器。

## 算法链路

在线 Information Map 由 FAST-LIO 的 `/cloud_registered` 构建时间衰减 voxel surfel，
每个 surfel 保存位置、单位法向、静态置信、几何质量与最近观测时间。候选附近信息按

`Lambda_hat = sum(w_i n_i n_i^T)`

构造，并以 information form 更新：

`P_(k+1) = inverse(inverse(P_k) + Lambda_hat_k)`。

预测器每步施加冻结下界 `lambda_min(P) >= 1e-5 m²`，避免重复 surfel 的独立性近似
把未来协方差压到不可信的零值。该下界在正式 Gate 前冻结，不改变 P9 的 PL/Margin
定义或 `M_reserve=0.10 m`。

候选集合为 baseline、左右侧移、上下偏移、减速、短停留和回退；yaw-only 仅作为
对比组。每个候选沿完整轨迹复用 P9 公式计算 `PL` 与 `M=AL-PL`，硬可行后才比较：

`J = DeltaT + lambda_E E_extra + lambda_D D_extra`。

## 正式三组飞行 Gate

固定项目本地 `xq_p10_long_corridor.sdf`、随机种子 `20260822`、初始状态、目标、P7
冻结校准和阈值，对 baseline、yaw-only、minimum-excitation 分别运行 Gazebo、仿真
LiDAR/IMU、FAST-LIO、P6、P10 Information Map、选择器、控制器与 evaluator。

| 指标 | Baseline | Yaw-only | Minimum-Excitation |
|---|---:|---:|---:|
| 预测最低 Margin | 0.045083 m | 0.045083 m | 0.419631 m |
| 实际最低 Margin | -0.320630 m | -0.448158 m | +0.104633 m |
| ATE RMS | 0.121176 m | 0.085154 m | 0.117578 m |
| 弱方向误差 RMS | 0.116408 m | 0.074186 m | 0.113459 m |
| 真值路径长度 | 7.506819 m | 7.704924 m | 7.572734 m |
| 任务时间 | 43.0 s | 43.0 s | 43.0 s |

minimum-excitation 选择并真实执行 `right_lateral`。相对 baseline 多走 `0.065915 m`，
时间开销 `0 s`；实际 Margin 增益 `0.425263 m`。baseline 和 yaw-only 均未恢复完整性，
因此“仅转动视角即可解决退化”的对照假设在该固定走廊中未成立。

汇总 16 项检查全部为 true，包括三臂各自通过、共同决策快照预测、恢复候选选择、实际
完整性恢复、额外路径/时间有界、ATE 与弱方向误差没有实质退化、冻结校准一致和指标
有限。三个飞行 arm 均保存独立 zstd rosbag、节点图、运行参数与日志。

## 可视化验收

独立 minimum-excitation 可视化轮次为 `PASS`：

- Gazebo `gz_record/state.tlog`：507,904 bytes；
- RViz ROS bag：17,824,985 bytes（zstd）；
- 选择：`right_lateral`，仅该候选满足冻结硬约束；
- 实际最低 Margin：`+0.142653 m`；最大侧向激励：`0.411429 m`；
- Gazebo 世界开放屋顶、保留外围墙体和内部完整性特征板；
- 外部 Gazebo map/model 前后 SHA-256 完全一致。

RViz 回放显示 baseline 与全部恢复候选、预测 Margin、可行性、预测协方差包络、最终
选择、FAST-LIO 轨迹及点云；Gazebo 回放显示同一轮飞行的开放屋顶场景和机体运动。

## 构建、测试与隔离

- `xq_autonomy` 全量回归：77/77 PASS；
- 13 个 ROS 2 包隔离构建成功；
- ROS 算法/传输契约 Gate：PASS；
- 正式三组长走廊飞行 Gate：PASS；
- Gazebo/RViz 可视化录制 Gate：PASS；
- 源码树 SHA-256：`7c4e26e0feaaabf4d09bff91652bbd7d25a543db0a80dfae4db6f46cdbd50767`；
- 安装树 SHA-256：`16ce2f5f24fe237c22ba5c39c423ed8e6f36a4a1f62a032811e12cf009078133`；
- P7 校准 SHA-256：`771bdffcf3d4422d4641424dab326a08aa5be2b0dffd7f9d2f2f9ff82ea9f038`；
- 正式 world SHA-256：`b813140df1f9fe7566a01f876a5978f83af2ded219bc4c04c5f01615da6687fa`；
- 既有 `cuadc_ws` 与 `ardupilot_gazebo` world/model 字节级不变。

## 复现

重新构建并运行正式三组 Gate：

```bash
cd /home/accelerate/xuanqiong_x1_sim_ws
bash scripts/build_isolated.sh
bash scripts/run_p10_flight_gate.sh
```

生成一轮新的 Gazebo + RViz 可视化证据：

```bash
bash scripts/run_p10_visual_capture.sh
```

同时回放最新 PASS 轮次：

```bash
bash scripts/view_p10_combined.sh
```

也可指定本报告的冻结结果：

```bash
bash scripts/view_p10_combined.sh \
  /home/accelerate/xuanqiong_x1_sim_ws/experiments/results/impact_p10/visual_20260823T154916Z_18660
```

## 证明边界

P10 证明的是固定静态长走廊 Gazebo SIL 中，基于实际 FAST-LIO 注册点云的有限候选
minimum-excitation 恢复闭环。它不证明连续动作全局最优、动态障碍处理、真实 Livox
噪声、Atlas 满载实时性、任意环境统计覆盖或实机安全保证。P11 的完整性约束 Frontier
探索、P12 动态地图、P13 时延安全和 P14 故障注入仍未开始。
