# IMPACT P8 Static-Obstacle Alert Limit 验收报告

验收完成日期：2026-08-23（Asia/Shanghai）

最终证据目录：

`/home/accelerate/xuanqiong_x1_sim_ws/experiments/results/impact_p8/p8_20260823T091322Z_57082`

Gazebo 在线录制证据：

`/home/accelerate/xuanqiong_x1_sim_ws/experiments/results/impact_p8/p8_gazebo_final_20260823T104800Z`

## 结论

P8 为 `PASS`。节点对 EGO-Planner 发布的三阶 B-spline 使用 de Boor 算法采样，只保留
当前仿真时刻之后仍有效的轨迹段；在 `/xq/p5/cloud_map` 的静态环境点中搜索每个轨迹
采样点的最近障碍，输出整条剩余轨迹最严格的 Alert Limit：

`AL = d - r_body - r_base - r_tracking - r_dynamic - r_latency`

其中 `r_latency = v*L_p99 + 0.5*a_max*L_p99^2`。P8 不订阅 Ground Truth，也不向
EGO、任务节点或飞控反馈；`AL-PL` 的硬判决属于 P9。

## 参数与数据边界

- `r_body = 0.35 m`，与 P5 碰撞评估机体半径一致。
- `r_base = 0.10 m`，`r_tracking = 0.10 m`。
- P8 v1 只处理静态障碍，`r_dynamic = 0` 被启动时强制检查。
- `L_p99 = 0.10 s`，`a_max = 1.0 m/s^2`；速度来自 FAST-LIO odom，速度模长不受
  body/map 坐标旋转影响。
- 轨迹源为 `/planning/bspline`，障碍源为 `/xq/p5/cloud_map`。
- 仿真输入是 P6 完整 Gazebo 自主探索 rosbag 的不可变重放；P7 冻结标定哈希作为
  阶段先决条件保存，但 P8 的环境 AL 计算本身不使用 PL。

## 正式 Gate

| 指标 | 结果 | 判定 |
|---|---:|---:|
| Alert Limit 消息 | 526 | PASS |
| 不同 EGO 轨迹 ID | 104 | PASS |
| 单条轨迹最大采样数 | 82 | PASS |
| 单帧最大静态障碍点 | 8,770 | PASS |
| 几何净空范围 | 0.0541–1.0111 m | 有环境响应 |
| AL 范围 | -0.5446–0.4356 m | 记录 |
| AL 均值 / 中位数 | 0.0893 / 0.1751 m | 记录 |
| AL q05 / q95 | -0.4765 / 0.3095 m | 记录 |
| 最大时延储备 | 0.06135 m | 方程一致 |
| 最近点距离最大误差 | `1.98e-13 m` | PASS |
| 方向单位范数最大误差 | `2.88e-12` | PASS |
| AL/时延方程最大误差 | `0` | PASS |

11 项自动 Gate 全部通过：消息数量、多个真实 EGO 轨迹、有限输出、轨迹采样、障碍点、
静态障碍契约、点云来源、单位方向、最近点几何、AL 方程和环境净空变化。

## 负 Alert Limit 的含义

107/526（20.34%）消息、27 条轨迹 ID 出现负 AL。最低样本的未来 B-spline 点与新观测
静态障碍距离约 0.0541 m；在扣除 0.35 m 机体和其他储备后，允许的定位误差预算自然
为负。这不是 P8 数值失败，也不等同于已经定义告警：它表示该轨迹段没有剩余误差预算。

P8 首轮开发重放曾同时评估已执行的 B-spline 历史段；正式版本用轨迹 `start_time` 和
当前点云时间裁掉历史前缀。修正后负 AL 仍出现在同类未来轨迹/新障碍组合，证明其来源
是环境几何而非轨迹时间错配。P9 必须把这些点与方向 PL 比较并作为硬约束处理。

## 隔离与可追溯性

- P8 节点图中没有 `/xq/eval/*`；Ground Truth 不进入 AL。
- 运行前后既有 Gazebo/ArduPilot 地图模型字节级一致。
- P7 prerequisite SHA、输入/输出 rosbag、节点图、完整日志和 JSON Gate 均已保存。
- 单元测试 5/5：B-spline 采样、有效时间域、最近点/AL 方程、负 AL、时延单调性。
- 源码树 SHA-256：`fabf25a665c91cc0e6a5d6a6560a8ffcf10b6e7bde2714a3cd0a9bfe72fe7591`。
- 安装树 SHA-256：`6e73bd18e6f11129d8891cc5b6c8d58620efe727bf10ffcff3433d98b1167e36`。

## 复现与 RViz

```bash
cd /home/accelerate/xuanqiong_x1_sim_ws
bash scripts/build_isolated.sh
bash scripts/run_p8_alert_limit.sh
bash scripts/run_p8_live_gazebo.sh
bash scripts/view_p8_rviz.sh
```

RViz 中蓝球是当前剩余 B-spline 的临界采样点，橙球是最近静态障碍，箭头表示障碍
方向；绿色表示仍有误差预算，红色表示 `AL < 0`。这只是 P8 环境预算可视化，不代表
P9 告警/拒绝状态机已经启用。

`run_p8_live_gazebo.sh` 启动完整 SITL + Gazebo server/client + FAST-LIO + Frontier +
EGO + P8 在线栈。正式 Gate 以 headless server 运行并录制 `gz_record/state.tlog`；Gate
PASS 后才打开独立 Gazebo 回放，避免 WSLg 渲染负载改变探索墙钟结果。该在线录制轮
同样 PASS：97 条 B-spline、436 条 AL、7,588 个碰撞评估样本、零碰撞、自动降落；
外部资产不变。回放只把房顶透明，四墙、地板和内部隔断保留。

## 边界

当前只验证静态障碍与局部已观测点云；未处理动态障碍预测、网络时延变化、候选轨迹
硬拒绝、恢复动作、真实 Livox/Atlas 或实机安全保证。P8 的负 AL 不能被普通代价收益
抵消，下一阶段 P9 必须实现 `M=AL-PL` 硬约束。
