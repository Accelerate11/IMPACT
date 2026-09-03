# P15 Map-Derived Integrity Planning 验收报告

验收完成日期：2026-09-03（Asia/Shanghai）

## 结论

P15 正式配对 Gate **PASS**。两个实验臂均在同一个复杂组合三维仓库中完成约 24 m
全航程；唯一算法消融变量为完整性硬过滤。约束臂把独立 Ground Truth 最低真实 Margin
从 `0.021936 m` 提升到 `0.281308 m`，完整性可用率从 `0.807069` 提升到 `1.000000`，
路径开销仅 `0.6701%`，任务时间反而减少 `2.5993%`。

正式原始结果：

```text
/home/accelerate/xuanqiong_x1_sim_ws/experiments/results/
impact_complex_comparison/p15_research_20260903_final_v4
```

Git 轻量证据：`evidence/P15/`。

## 因果比较契约

- 世界：`xq_complex_compositional_warehouse.sdf`，SHA-256
  `8969a5e489c07a89783e7b3fa3f3fce92f98fc5b5421c62f249411562ccfddf9`。
- 两臂共享 5×2 三维 lattice、online-map 指标、碰撞门、运动/返航能量门、动态地图、
  时延闭环、随机与场景时序。
- information-only 基线仅省略完整性硬门；约束臂的 Margin 不进入任务效用。
- Ground Truth 只由 evaluator 消费；节点图和结果字段均报告算法未订阅真值。
- 运行时数据面确认控制器和地图均实际采用 `active_trajectory`、5 体素连通支持、
  `0.45 m` 聚类半径，P13 控制器采用 `current_margin_replan`。

## 正式指标

| 指标 | Information-only | Integrity-constrained | 效果 |
|---|---:|---:|---:|
| 净前进 | 23.998965 m | 23.996750 m | 两臂完整任务 |
| ATE RMS | 0.061599 m | 0.062337 m | +0.000739 m |
| 弱方向误差 RMS | 0.048913 m | 0.048400 m | -0.000514 m |
| 预测最低 Margin | -0.086659 m | 0.159199 m | +0.245858 m |
| 独立真值最低 Margin | 0.021936 m | 0.281308 m | +0.259372 m |
| Availability | 0.807069 | 1.000000 | +0.192931 |
| False-alarm rate | 0.192931 | 0.000000 | -0.192931 |
| PL empirical coverage | 1.000000 | 1.000000 | 保持 |
| HMI / 真实安全违规 | 0 / 0 | 0 / 0 | 均为零 |
| 真值路径长度 | 25.027473 m | 25.195195 m | +0.167721 m（+0.6701%） |
| 任务时间 | 135.1325 s | 131.6200 s | -3.5125 s（-2.5993%） |

四个规划窗均实际执行并完成。Information-only 序列为
`lattice_y_-0.35_z_+0.00 → direct → lattice_y_+0.00_z_+1.10 → direct`；约束臂为
`lattice_y_-0.35_z_+0.00 → lattice_y_+0.35_z_+0.00 → lattice_y_-0.35_z_+1.10 → direct`。
第二、第三规划窗发生完整性硬干预，最小干预效用带在三个 batch 中实际参与决策。

约束臂状态计数为 `segments_completed=4`、`planning_windows_closed=4`、
`decisions_applied=4`、`interrupted_decisions=0`。因此 PASS 不是由接受四次决策而未飞完
任务造成的假阳性。

## 验收门与运行质量

`comparison-result.json` 中 20 项检查全部为 `true`，包括 P11/P12/P13 双臂 PASS、
独立真值样本、全航程、PL coverage、可用率增益、真实 Margin 增益、路径/时间/ATE
有界、硬干预、HMI 为零、运行时配置采用和 Ground Truth 隔离。

```text
140/140 Python 算法与验收契约测试 PASS
isolated ROS package build PASS
information_only launch.log: no Traceback / process died / [ERROR]
integrity_constrained launch.log: no Traceback / process died / [ERROR]
GPU renderer: D3D12 (NVIDIA GeForce RTX 4060 Laptop GPU)
```

运行器标准输出曾出现一次 Gazebo Transport 的
`NodeShared::Publish() Error: Interrupted system call`；两个算法 launch 日志均干净，
所有 evaluator 和最终 Gate 均 PASS，故记录为非功能性模拟器 transport 提示，而不是
算法节点错误。

## 研发中保留的失败

1. 早期正式比较中约束臂可用率和真实 Margin 增益不足；由此增加独立真值指标，而非
   放宽结果阈值。
2. 在线 current-Margin guard 暴露出 1–4 个动态残差体素可被持续刷新并阻塞任务；
   revision 3 预声明 5 体素连通支持，结果阈值保持 revision 2 不变。
3. 一轮约束臂飞完 24 m，但旧 evaluator 把“已接受决策数”误当完成规划窗；随后拆分
   完成、关闭、中断和已执行计数，并加入中断能耗按比例记账测试。
4. ROS CLI 参数审计在 teardown 阶段可能阻塞；正式协议改为由控制器、地图和 P13
   evaluator 在结果 JSON 中报告实际采用配置。
5. 多个节点在 ROS context 关闭时存在日志竞态；现仅在 context 已关闭时抑制异常，
   活跃 context 的异常仍重新抛出，最终双臂日志干净。

## 复现

正式无 GUI 配对 Gate：

```bash
cd /home/accelerate/xuanqiong_x1_sim_ws
bash scripts/run_p15_research_comparison.sh
```

Gazebo + RViz 同时显示完整正常飞行算法：

```bash
cd /home/accelerate/xuanqiong_x1_sim_ws
bash scripts/run_p15_live_visualization.sh
```

Gazebo 使用 GPU、只移除屋顶并保留外墙/隔墙/货架/门架/杂物和运动体；RViz 同步显示
点云、FAST-LIO、动静态地图、lattice 候选、认证轨迹和完整性/动态/时延状态。

## 证明边界

本结果证明固定复杂组合仓库、固定协议的一次配对 Gazebo SIL 中，map-derived 候选指标
和完整性硬过滤能产生显著的机制级安全收益，且未牺牲任务完成。它还不是顶刊论文所需
的统计证据：至少还需多 seed、多地图、难度分层、强 baseline、置信区间/显著性检验、
计算资源消耗和 HIL/实机外部有效性。
