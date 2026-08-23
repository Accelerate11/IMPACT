# IMPACT P7 Protection Level 校准验收报告

验收完成日期：2026-08-23（Asia/Shanghai）

最终证据目录：

`/home/accelerate/xuanqiong_x1_sim_ws/experiments/results/impact_p7/p7_20260823T082157Z_48605`

## 结论

P7 为 `PASS`。P7 使用真实定位误差
`e_k = p_k - p_hat_k` 与 P6 的完整性协方差，按方向计算
`s_k = |d^T e_k| / sqrt(d^T P_int d)`。两个训练场景先生成冻结标定文件，随后两个
全新轨迹只读取该文件。独立验证共 5,256 个“样本×方向”，95% 与 99% 覆盖率均为
100%，95% 漏检率为 0。

## 训练与冻结规则

- 训练场景：`structured_room`、`long_corridor`，每场景 657 个同步样本。
- 基础分位数：每个方向先取各训练场景的 higher empirical q95/q99，再取场景最大值。
- 训练集场景迁移储备：四方向训练 q95 的跨场景比值最大值为 `4.4887300142`；同一个
  因子统一乘到所有方向，测试数据不参与方向或系数选择。
- 冻结 `k95`：X `10.251700`、Y `26.391371`、Z `51.234940`、弱方向 `10.183846`。
- 冻结 `k99`：X `11.873128`、Y `27.327888`、Z `52.328467`、弱方向 `11.766835`。
- 标定 SHA-256：`771bdffcf3d4422d4641424dab326a08aa5be2b0dffd7f9d2f2f9ff82ea9f038`；
  测试前、测试后和最终文件三者一致。

该规则有意保守：只有两个训练场景时，直接把单场景 q95 当作未知轨迹的 q95 没有
场景迁移保证。P7 使用训练集内部实际观测到的最坏迁移幅度作为统一储备，不接触最终
验证轨迹。

## 独立验证结果

| 场景 | ATE RMS | 95% 覆盖 | 99% 覆盖 | X/Y/Z/弱方向平均 PL95 |
|---|---:|---:|---:|---:|
| structured room | 0.05024 m | 100% | 100% | 0.1166 / 0.1955 / 0.3075 / 0.1159 m |
| long corridor | 0.04326 m | 100% | 100% | 0.1362 / 0.1780 / 0.3339 / 0.1353 m |
| 聚合 | — | 100% | 100% | 5,256 个样本×方向 |

两场景每个方向均为 657 样本，所有方向的 95% 漏检事件数均为 0。P8 尚未定义
Alert Limit，因此 P7 的 false-alarm rate 明确为 `null/N/A`，不伪造阈值。

## 失败诊断与修正

首轮开发验证保留在
`experiments/results/impact_p7/p7_20260823T081317Z_43375`。它暴露出两个真实问题：
直接取训练场景最大 q95 在新轨迹上欠覆盖；原走廊测试先留下净偏航再沿机体系倒飞，
使模型撞向侧壁并在静止退化段发散。该轮为 `FAIL`，未写成成功结果。

修正后只从训练集估计统一场景迁移储备，并使用未参与调整的新验证轨迹。走廊验证采用
对称正负偏航，净航向为零，保持轨迹在走廊内；没有降低 P3 Gate。

## 隔离、数据边界与可追溯性

- P6 predictor 不订阅 Ground Truth；真值只进入 P3 evaluator 和 P7 collector。
- P7 predictor 不控制规划或飞控。
- 四轮外部资产审计均 PASS，既有 Gazebo/ArduPilot 地图模型字节级不变。
- 每轮保留 zstd rosbag、topic graph、算法配置、源码树与安装树证据。
- 源码树 SHA-256：`e8bc666735583f8198657f539bc7537caeeaa017e74b5568b4b865fde97b5799`。
- 安装树 SHA-256：`80df6966fa54111c70603d7cb298c5320ef543945669cbb5dbd9e989b3ffd2b1`。
- 结束后无 Gazebo、FAST-LIO、rosbag 或 P7 残留进程。

## 复现与 RViz

完整 P7 Gate：

```bash
cd /home/accelerate/xuanqiong_x1_sim_ws
bash scripts/build_isolated.sh
bash scripts/run_p7_calibration.sh
```

最终 PASS 结构化房间 rosbag 的校准 PL95 RViz 复现：

```bash
cd /home/accelerate/xuanqiong_x1_sim_ws
bash scripts/view_p7_rviz.sh
```

RViz 箭头、包络与文本使用冻结 `k95`；脚本已播放 bag，不要再启动第二个
`ros2 bag play --clock`。

## 边界

P7 证明的是这两个训练域与两个独立验证轨迹上的 Gazebo SIL 覆盖。100% 是本次有限
样本结果，不等于总体覆盖率为 100%，也不代表真实 Mid-360S、Atlas、实机、P8
Alert Limit、P9 Integrity Margin 或故障注入性能已完成。
