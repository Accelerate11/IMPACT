# P12 Dynamic Obstacle / Dynamic Map 验收报告

状态：**PASS**  
正式轮次：`experiments/results/impact_p12/gate_20260827T165121Z_879`  
运行层级：Gazebo Harmonic SIL + Mid-360-like LiDAR + FAST-LIO2  
场景：30 m 开顶长走廊，墙体完整保留，单个橙色动态箱横穿航路

## 1. 验收目标

P12 只引入“动态环境”变量，并验证完整闭环：障碍出现后由 LiDAR 几何检测，动态体素
进入规划走廊，控制器制动并发布重规划事件；障碍离开后动态置信度按

`D(t+dt) = D(t) exp(-dt/tau)`

衰减，TTL 清除且连续确认后重新开放通道，最后飞完 24 m。Ground Truth 只允许进入
独立 evaluator，不得被动态地图、探索选择器或飞行控制器订阅。

## 2. 算法实现

- 0.25 m 稀疏三维体素分别维护 occupancy、static confidence、dynamic confidence、
  last seen 和 last free；静态层不经过 TTL 删除。
- 启动 12 s 后冻结已观测静态基线。动态候选必须同时满足：历史 free→occupied、
  不邻近冻结静态结构、位于真实 0.70 m 轨迹走廊内。
- 动态建图探测范围为 LiDAR 的 12 m，规划制动前视为 4 m；“看见”与“必须制动”分离。
- 规划只消费 path-certified dynamic layer，避免移动视点首次揭露的固定表面污染安全层。
- 动态点离开后按 `tau=3 s` 指数衰减；低于 0.08 后清除，再经 1 s 连续空闲确认开放。
- Gazebo 自定义原生运动 System 按仿真时钟驱动障碍，避免一次性 transport 发布器把
  “命令发送成功”误记为“物体已到位”。

## 3. 冻结 Gate 与结果

| 指标 | Gate | 正式结果 | 判定 |
|---|---:|---:|---|
| 动态检测延迟 | <= 1.5 s | 0.1325 s | PASS |
| 制动/重规划延迟 | <= 0.5 s | 0.2100 s | PASS |
| 动态残留时间 | <= 10 s | 4.0250 s | PASS |
| 通道重开时间 | 0–12 s | 5.2500 s | PASS |
| 静态结构保留率 | >= 0.98 | 1.0070 | PASS |
| 障碍物理净空 | >= 0.75 m | 3.6372 m | PASS |
| 净前进 | >= 23.5 m | 23.9851 m | PASS |
| FAST-LIO ATE RMS | <= 0.35 m | 0.07591 m | PASS |
| 动态体素峰值 | >= 3 | 55 | PASS |
| 重规划事件 | brake + reopen | 2 | PASS |

控制器最终状态为 `COMPLETE`，四个滚动 batch 全部完成，真值路径长 24.5054 m；
算法节点图确认只订阅 `/localization/odom`、LiDAR/动态地图/完整性与规划话题，未订阅
`/xq/eval/*`、Ground Truth 或 Gazebo model pose。

## 4. 关键失败迭代与修复

1. 最初的 `gz topic` 速度命令实际只把障碍从 `y=3.4` 移至 `y=2.98 m`。改为 Gazebo
   原生 System 后实测阻塞阶段 `y≈0`。
2. 固定观测板边缘被膨胀扫掠误报。改为原始米制 0.70 m 半径，并要求历史
   free→occupied；不删除墙体、不降低净空 Gate。
3. 通用动态证据会把移动视点新揭露的固定表面带入规划。保留通用统计，但规划/RViz
   红色层只使用 path-certified voxels。
4. 4 m 规划前视导致检测延迟 3.10 s。将 12 m 建图检测与 4 m 制动前视解耦后，延迟
   降为 0.1325 s，验收门槛保持 1.5 s。
5. 一次 DDS 启动中控制器 RELIABLE odom reader 未发现发布者。增加兼容两种发布策略的
   BEST_EFFORT reader，并按消息时间戳去重；18 s 冒烟验证稳定进入 `EXECUTE`。

这些 FAIL 轮次均保留在 WSL `experiments/results/impact_p12/`，未被当作正式证据。

## 5. 可视化与复现

正式 PASS 同时保存 277 MB 压缩 rosbag 和 1.8 MB Gazebo state recording。开顶、墙体保留。

```bash
cd /home/accelerate/xuanqiong_x1_sim_ws
bash scripts/view_p12_combined.sh
```

RViz：青色为静态结构，红色为 LiDAR-only 动态体素，绿色为已选轨迹，橙色为 FAST-LIO
轨迹；Evaluation Truth 默认关闭。Gazebo 显示同一正式轮次的墙体、无人机和橙色移动障碍。

重新执行正式 Gate：

```bash
cd /home/accelerate/xuanqiong_x1_sim_ws
bash scripts/run_p12_flight_gate.sh
```

## 6. 隔离与边界

- 外部资产审计：`PASS: pre-existing Gazebo map/model assets are byte-for-byte unchanged.`
- P12 仅证明固定室内长走廊中单个刚性动态障碍的 Gazebo SIL 能力；不代表多目标跟踪、
  人体预测、非视距动态物体、Atlas 满载实时性、HIL 或实机安全保证。
- 视觉语义没有进入本阶段；动态分类完全来自 LiDAR 几何和定位里程计。
