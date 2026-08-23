# IMPACT P6 Directional Integrity Predictor 验收报告

验收完成日期：2026-08-23（Asia/Shanghai）

最终证据目录：

`/home/accelerate/xuanqiong_x1_sim_ws/experiments/results/impact_p6/p6_20260823T065905Z_36386`

## 结论

P6 为 `PASS`。FAST-LIO 在真实点面更新入口直接累计平移方向信息矩阵
`Lambda_p = sum(w_i n_i n_i^T)`；独立完整性节点以 FAST-LIO ESKF 平移协方差计算
`P_int = kappa P_LIO + eta(Lambda_p + epsilon I)^-1`，并输出任意方向
`PL(d) = k_alpha sqrt(d^T P_int d)`。算法不订阅 Ground Truth，也不在 P6 向规划器或
飞控反馈控制量。

这里的 `k_alpha=3.0` 仍是可配置常数，不是宣称完成统计覆盖率校准；Ground Truth
训练/测试分离校准属于 P7。

## 第一性原理实现

- 法向量与残差来自 FAST-LIO `h_share_model()` 的已接受点面约束，平移雅可比就是
  `n_i^T`，不是从渲染点云或真值轨迹反推。
- `w_residual = 1 / (1 + (|r|/sigma_r)^2)`；`sigma_r` 参数化。
- `w_geometry` 由同一近邻平面支撑集的特征值分离度计算并带参数化下限。
- P6 v1 按企划保留 `w_static = w_timing = 1`，后续故障与延迟阶段再接入。
- `kappa = 1 + a_D D + a_N NNIS + a_T T_j + a_R R_d` 的所有系数、输入项、
  `eta`、`epsilon`、`k_alpha` 和数值下限均为 ROS 参数，无硬编码安全结论。
- 默认功能开关关闭；P5 BASELINE_V1 不启用额外几何计算，P6 launch 才显式打开。

## 在线 Gate 与完整 rosbag

| 指标 | 结果 | 判定 |
|---|---:|---:|
| 在线方程 Gate | 80 geometry + 80 integrity | PASS |
| 完整 rosbag 消息 | 1,467 + 1,467 | PASS |
| 有效点面约束 | 1,196–4,074 / 帧 | PASS |
| 条件数 | 1.443–5.464 | 记录 |
| 弱方向 PL | 0.0233–0.0529 m | 记录 |
| X/Y/Z 轴向 PL 最大值 | 0.0529 / 0.0425 / 0.0274 m | 记录 |
| 弱轴主导次数 X/Y/Z | 675 / 792 / 0 | 方向响应存在 |
| `P_int` 最小特征值 | `2.366e-5` | 正定 |
| PL 方程最大数值误差 | `6.94e-18` | PASS |
| Ground Truth 输入 | 无 | PASS |
| 规划反馈 | P6 禁用 | PASS |

完整飞行同时保持 P5 基线闭环：4 个自动 Frontier、125 条 EGO B-spline、自动结束、
降落并解锁；空中 238.787 s、真实航迹 42.141 m、7,405 个评估样本、0 碰撞，最小
净空 0.300 m。外部 Gazebo/ArduPilot 地图模型前后字节级一致。

rosbag 为 295.498 s、223,679 条消息、436.6 MiB（zstd）。全程统计保存在
`p6-full-bag-analysis.json`，在线 Gate 保存在 `integrity-result.json`，总结果在
`summary.json`。

## 可追溯性与边界

- 源码树 SHA-256：`b1d7f2dbf418ed458efce6994636a0c5a8c2c65a37826cb9621f882f3c17e9b1`。
- 安装树 SHA-256：`f7f73e314a62cd2c7da14337d39eec362ccbbcead78bfa57d0a4acc22090bf0c`。
- 单元测试 4/4：均匀房间、走廊弱轴、参数化 kappa、方向 PL 方程。
- 本结果是 Gazebo/SITL 算法级 SIL；不代表 P7 覆盖率校准、P8 Alert Limit、P9
  Integrity Margin、真实 Mid-360S、Atlas 或实机完整性保证。

## 复现与 RViz

```bash
cd /home/accelerate/xuanqiong_x1_sim_ws
bash scripts/build_isolated.sh
bash scripts/run_p6_directional_integrity.sh
```

最终 PASS rosbag 的 RViz 动态复现：

```bash
cd /home/accelerate/xuanqiong_x1_sim_ws
bash scripts/view_p6_rviz.sh
```

RViz 中 `P6 Directional Integrity` 显示三轴 PL、紫红色弱方向箭头、半透明 PL 包络和
实时数值文本；为了肉眼可见，几何长度放大 20 倍，文本中的米制数值未缩放。

