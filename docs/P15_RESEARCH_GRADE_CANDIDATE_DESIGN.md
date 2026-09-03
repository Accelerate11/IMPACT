# P15 Map-Derived Integrity Planning 设计记录

状态：`PASS`。正式验收结论见
`docs/P15_RESEARCH_GRADE_ACCEPTANCE_REPORT.md`。

## 目标

P15 不再把候选名称绑定到人工设定的“信息收益、碰撞概率和能耗”，而是从在线地图与
候选轨迹本身推导这些量，并建立只改变完整性硬约束的配对因果比较。目标是回答：在
任务效用、碰撞门和返航能量门相同的条件下，导航完整性硬过滤能否以有界任务代价提高
真实安全裕度和可用率。

## 在线候选与任务模型

每个滚动规划窗生成 5 个横向层级与 2 个高度层级组成的三维格点，共 10 条候选轨迹。
候选使用唯一 ID，不依赖 `direct/right/up` 等名称赋值。任务收益为：

```text
G_i = 0.85 * displacement_i / path_length_i
    + 0.15 * normalized_map_observation_i
```

`map_observation` 只使用在线 surfel 的位置、静态置信度、几何质量和最后观测时间；低
置信、陈旧且能被该轨迹唯一观测的 surfel 获得更高权重。定位信息矩阵仍只进入完整性
预测，不与任务观测收益混用。

碰撞概率由逐点 Alert Limit 和跟踪残差的单侧高斯代理计算。能耗由轨迹长度、速度平方
阻力、爬升和加速度代理组成，并显式加入候选末端到 home 的保守返航储备。两个比较臂
都必须通过碰撞和总能量硬门。

## 完整性选择原则

对每个候选预测方向 Protection Level，并计算整条候选的最小
`Margin = Alert Limit - Protection Level`。约束臂先执行：

```text
integrity feasible AND collision feasible AND energy feasible
```

再在可行集合中最大化任务效用。Margin 不进入效用，也不靠权重折中。信息优先基线与
约束臂共享候选、地图指标、碰撞门和能量门，唯一消融变量是完整性硬过滤。

当多条已认证候选的任务效用相差不超过预声明的 `0.05` 分辨带时，选择总能耗和时间
更小的候选。这是最小干预规则，不使用 Margin 对已通过硬门的候选再次排序。

## 在线安全与任务活性

- 动态障碍只查询当前活动轨迹，避免无关旧候选触发制动。
- 动态证据必须形成半径 `0.45 m` 内至少 5 个体素的连通支持；1–4 个配准残差不会
  永久刷新制动状态。
- 使用 train-only 冻结的 `k95` 监视当前飞行轨迹 Margin；低于 `0.12 m` 时允许关闭
  当前规划窗、按已执行比例计入能耗并重新规划。
- `segments_completed`、`planning_windows_closed`、`interrupted_decisions` 和
  `decisions_applied` 分开计数，不能再用“接受过决策”冒充“飞完一段”。
- 终端 progress watchdog 要求 Ground Truth evaluator 观测到至少 `23.5 m` 净前进；
  算法控制器本身不订阅 Ground Truth。

## 独立评价

Gazebo Ground Truth 只进入 evaluator。对时间对齐的真实方向误差独立计算：

- Protection Level empirical coverage 与 tightness；
- availability、false alarm、真实安全违规；
- HMI：真实误差越过 Alert Limit，但在线监视仍宣称 `PL <= AL`；
- realized minimum margin，而非只报告算法内部预测 Margin。

运行器同时保存声明参数和 P11/P12/P13 节点实际报告的运行时配置，防止“命令行写了参数
但节点没有采用”。任一 evaluator 失败、节点死亡、Traceback 或 ROS `[ERROR]` 都使
正式 Gate 失败。

## 研究边界

P15 是从模块演示走向可发表实验协议的一步，但仍是一个固定 Gazebo 世界的一次配对
SIL 试验。它不证明跨场景泛化、统计显著性、真实传感器分布外鲁棒性、Atlas 实时性或
实机安全。后续论文实验必须增加多随机种子、多地图难度分层和公开 baseline。
