# P11 Integrity-Constrained Exploration 设计记录

状态：`FROZEN_AND_ACCEPTED`。正式结果见
`docs/P11_INTEGRITY_CONSTRAINED_EXPLORATION_REPORT.md`。

前置条件：P10 Minimum-Excitation Active Perception 正式 Gate `PASS`，提交 `a17f152`。

## 1. 设计边界

P11 不重新实现 Frontier。Frontier、粗路径和局部 B-spline 继续由已有基线产生，P11
只在候选集合上增加跨层硬认证：

`Frontier -> B-spline -> predicted PL -> AL -> Margin -> hard filter -> utility`。

候选同时满足 `M_min>=M_reserve`、碰撞概率和返航能量约束才进入任务效用排序；集合
为空时 fail-closed。

## 2. 滚动全走廊任务

任务终点必须相对于第一帧稳定 FAST-LIO odom 建立，不能把 Gazebo 世界坐标直接当成
FAST-LIO 坐标。24 m 任务由 `rolling_horizon_distances()` 划分成
`7.5 + 7.5 + 7.5 + 1.5 m`。每段结束后：

1. 累计已执行轨迹的能量代价；
2. 读取新的 odom、点云、Information Map 与 Directional Integrity；
3. 生成新 batch 和唯一 trajectory ID；
4. 对所有候选重新执行完整性、碰撞和能量硬认证；
5. 仅执行本 batch 的有效选择。

达到相对 24 m 终点并完成 4 个 batch 后才允许 `COMPLETE`。旧决策、旧轨迹或单段计时
均不能使任务提前结束。

## 3. 任务效用

冻结效用为 `J = w_I I - w_T T - w_E E`。Margin、PL 和可观测性不是软奖励；它们先
执行布尔硬过滤。这样在多个安全候选中仍选择任务收益最高者，而不是无条件追求最大
定位精度。

信息收益在全任务报告中同时记录总计和每 batch 均值。滚动任务的有界信息损失以每
batch 均值比较，避免把单窗口阈值错误地直接应用到多个窗口的累计量。

## 4. 场景与实际完整性

专属世界开放顶部并保留墙体。开发诊断证明，只扩展飞行距离会让远端竖直 Protection
Level 因 Mid-360 下视约束不足而膨胀；单纯升高会越出有效扫描带并触发 FAST-LIO
`No point`。最终场景在侧墙下方加入低位水平观测边，其上表面进入 Mid-360 垂直视场，
提供 z 法向约束但不形成房顶、不遮挡顶视图、不侵入候选航道。安全候选横向偏置冻结
为 0.60 m，实体最小侧向净空仍约 1.35 m。

## 5. 正式 Gate

同一冻结世界下比较：

- `information_only`：四段均执行效用最高的 direct；
- `integrity_constrained`：每段执行 P11 重新认证后的候选。

两臂都必须完成 24 m，且约束臂必须满足全程实际 Margin 储备、碰撞/能量门、信息损失、
额外路径、任务时间和 ATE 有界。Ground Truth 只能进入 evaluator/logger，不能进入
Frontier 元数据、Information Map、预测、选择器或飞行控制器。
