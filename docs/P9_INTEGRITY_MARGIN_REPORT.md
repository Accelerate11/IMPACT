# IMPACT P9 Integrity Margin 验收报告

验收完成日期：2026-08-23（Asia/Shanghai）

正式证据目录：

`/home/accelerate/xuanqiong_x1_sim_ws/experiments/results/impact_p9/p9_20260823T135112Z_1110615`

## 结论

P9 为 `PASS`。实现对候选轨迹每个剩余采样点计算：

`PL(a_j) = k95 * sqrt(a_j^T P_int a_j)`

`M_j = AL_j - PL(a_j)`，`M_min = min_j M_j`

当 `M_min >= margin_reserve` 才把候选 B-spline 从
`/planning/candidate_bspline` 转发到 `/planning/bspline`；否则不转发。该判决是独立
布尔硬门，没有进入 EGO 加权代价，也不能被信息增益、距离或其他收益抵消。

## 为什么扩展 P8 轨迹剖面

P8 原消息只给出最小几何净空对应的单点。由于 `P_int` 具有方向性，最小 AL 点不一定
是最小 `AL-PL` 点。P9 因而没有使用单点近似，而是让 P8 同时发布整条剩余 B-spline
的采样点、最近障碍、单位障碍方向、净空和 AL 数组；这些数组使用新增且独立的
`/integrity/alert_limit_profile`，原 `/integrity/alert_limit` 消息保持 P8 字节契约不变。
P9 对齐同一索引逐项求 PL 和 Margin，再取全局最小值。单元测试专门构造了“最小 AL
点不是最小 Margin 点”的反例。

## P7 校准使用规则

- 冻结训练产物 SHA-256：
  `771bdffcf3d4422d4641424dab326a08aa5be2b0dffd7f9d2f2f9ff82ea9f038`。
- 节点启动时验证 `train_only=true`、`test_data_used=false` 和可选 SHA；不读取 Ground Truth。
- P7 校准了 X/Y/Z/弱方向，而 P9 障碍法向可以任意。为继续使用企划定义的单一
  `k_alpha` 公式，P9 取冻结四方向的最大 `k95=51.2349402405`，不做方向插值，也不
  使用 P9 Gate 数据调参。

这是对已有四方向系数的保守复用，但不能被表述为“所有连续方向已独立完成 95% 覆盖
验证”；任意方向覆盖率仍需以后增加训练/验证方向集合才能作统计结论。

## Gate P9

Gazebo 构造一个无房顶、保留全部墙体的认证实验室：wide room 净宽 3.0 m，narrow
passage 净宽 1.2 m，两条中心线候选轨迹几何形式相同。两场景使用完全相同的定位完整性
协方差：

`P_int = diag(1.6e-5, 1.6e-5, 1.6e-5) m^2`

`margin_reserve = 0.10 m`。正式结果如下：

| 指标 | wide room | narrow passage |
|---|---:|---:|
| 轨迹 ID | 9001 | 9002 |
| 最大/关键 AL | 0.945833 m | 0.047080 m |
| 方向 PL | 0.204940 m | 0.204940 m |
| Integrity Margin | +0.740893 m | -0.157860 m |
| 判决 | ACCEPT | REJECT |
| 下游 B-spline | 已发布 | 已阻断 |

两侧 PL 数值之差在浮点精度内为零，`P_int` 最大差为 0；几何空间越窄，AL 与 Margin
单调减小，符合 Gate 要求和直觉。11 项检查全部 PASS：消息、同协方差、宽接受、窄拒绝、
传输通过/阻断、Margin 方程、布尔判决、几何排序和非加权硬约束。

## 运行与可追溯性

- rosbag：4.406 s、352 条消息；候选 B-spline 2 条、下游 B-spline 1 条、Margin 42 条。
- Gazebo：`gz_record/state.tlog` 已保存；SDF 校验 `Valid`。
- 数学/算法回归：P8/P9 定向测试 9/9，全 `xq_autonomy` 测试 65/65。
- `/integrity/margin` 与 `/planning` 硬门节点图无 `/xq/eval/*`。
- 既有 `cuadc_ws` 与 `ardupilot_gazebo` world/model 前后 SHA-256 完全一致。
- 双窗口回放实测无 ERROR/Traceback；Gazebo 顶视相机、开顶和保留墙体确认已写入
  `gazebo-replay-view.txt`。
- 最终隔离构建源码树 SHA-256：
  `a1528129f80e2164b76c3702be64e4d48978dc900ec1a4e7f0f374253a63cb8f`；安装树：
  `1403c6afdd0e64f398a36d698b96e54957a0951609a60cdcda7d01988661c083`。

## 复现

正式 Gate：

```bash
cd /home/accelerate/xuanqiong_x1_sim_ws
bash scripts/build_isolated.sh
bash scripts/run_p9_integrity_margin.sh
```

立即同时打开正式结果的 Gazebo 与 RViz：

```bash
cd /home/accelerate/xuanqiong_x1_sim_ws
bash scripts/view_p9_combined.sh
```

重新运行 Gate，PASS 后自动打开两个窗口：

```bash
bash scripts/run_p9_live_gazebo.sh
```

RViz 中绿色为 wide-room ACCEPT，红色为 narrow-passage REJECT，半透明球表示关键方向
PL 包络，文本直接显示 AL、PL、`M_min` 和储备阈值。

## 边界

P9 证明的是确定性静态几何 Gate、轨迹认证公式和消息传输硬门。Gazebo 场景没有执行
ArduPilot 动态飞行；REJECT 后的主动感知、候选重生成与恢复动作属于 P10 以后。P9
不证明动态障碍、任意方向总体覆盖、真实 Livox/Atlas 或实机安全保证。
