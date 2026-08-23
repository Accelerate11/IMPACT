# IMPACT P5 Baseline Map + Frontier + EGO 验收报告

验收完成日期：2026-08-23（Asia/Shanghai）

最终证据目录：

`/home/accelerate/xuanqiong_x1_sim_ws/experiments/results/baseline_v1/p5_20260823T042657Z_29091`

## 结论

Gate P5 为 `PASS`。在项目独立的 `xq_p5_structured_room` 中，系统以 FAST-LIO2
输出为唯一自主定位输入，完成 0.10 m 导航建图、Frontier 聚类、候选视点与
`J=I-lambda*d` 目标选择、官方 ROS 2 EGO-Planner B-spline 规划、MAVROS/ArduPilot
闭环执行、Frontier 自动耗尽判定和自动降落。没有人工航点，Ground Truth 只进入
rosbag 和 0.05 m 评估器。

本结论是 Gazebo Harmonic + ArduPilot SITL + ROS 2 的算法级 SIL 证据，不等同于
真实 Mid-360S、真实飞控、Atlas 200I DK A2 或实机避障证据。

## 正式 Gate 结果

| 指标 | 结果 | 判定 |
|---|---:|---:|
| 导航栅格分辨率 | 0.10 m | PASS |
| 评估地图分辨率 | 0.05 m（400 × 320） | PASS |
| 目标生成方式 | Frontier，`J=I-0.18d` | PASS |
| 人工航点 | 0 | PASS |
| 自动 Frontier 目标 | 4 | PASS |
| 到达 / 超时后重选 | 2 / 2 | 记录 |
| EGO B-spline | 110 | PASS |
| 探索终态 | 0 Frontier cell / 0 cluster | PASS |
| 已知导航单元 | 31,681（40.41% 全局方形缓冲区） | 记录 |
| 任务闭环时间 | 281.101 s | 记录 |
| 空中评估时间 | 237.705 s | PASS |
| Ground Truth 样本 | 7,594 | PASS |
| 真实轨迹长度 | 43.170 m | 记录 |
| 碰撞样本 | 0 | PASS |
| 最小障碍净空 | 0.250 m | PASS |
| 终态 | LAND、disarmed | PASS |
| Ground Truth 泄漏 | 无 | PASS |
| 外部地图/模型变化 | 无 | PASS |

`known_fraction` 的分母是以 LIO 原点为中心的 28 m × 28 m 规划缓冲区，包含房间外
不可达区域，因此不作为室内可通行面积覆盖率；P5 Gate 的完成条件是所有可达
Frontier 消失，而不是用该分数伪称房间覆盖率。

## 关键问题与修正

早期运行把地板和天花板点投影到水平 Frontier 栅格，形成原点附近的假障碍环。
最终二维导航层只使用 `2.0 ± 0.45 m` 飞行走廊内的点，完整三维点云仍送给 EGO。

第一条有效闭环轨迹随后从 P4 调试房间的柜体上方穿过。rosbag 还原显示机体中心在
约 2.21 m 高度越过 2.0 m 柜顶；EGO 上游点云路径在 XY 按安全半径膨胀，在 Z
却固定只膨胀一个 0.10 m voxel，中心轨迹无碰撞但真实机体包络擦顶。项目本地补丁
将 Z 膨胀改为与 XY 相同的 0.35 m 配置半径，并将正式 Gate 切换到企划指定的独立
`structured_room`。最终两次 structured-room 闭环均为 0 碰撞，正式复验生成完整
`summary.json`。

## 可追溯性与隔离

- EGO 来源：`ZJU-FAST-Lab/ego-planner-swarm` 的 `ros2_version`，commit
  `23a8d5a191711dd65633df689b0b37ac07718416`；项目补丁记录在
  `src/xq_ego_planner/PROVENANCE.md`。
- 源码树 SHA-256：`992dacf53c64ccdb3a0f6bed30b6c735879970f002b50b4f700c402f122c57fa`。
- 安装树 SHA-256：`f367f605106378def367a375366e1c30d2b0054036ea819b59e96d9beedc5944`。
- rosbag：291.259 s、228,886 条消息、455 MiB（zstd）。
- ROS2 图证明 `/xq/p5/frontier_goal` 唯一发布者为 `xq_p5_frontier`，订阅者包含
  `xq_p5_ego_planner`；Ground Truth 订阅者只有 rosbag 与 `xq_p5_evaluator`。
- 每轮使用独立 DDS domain、Gazebo partition、SITL EEPROM/DataFlash 和进程组；
  外部 Gazebo/ArduPilot 地图模型运行前后 SHA-256 完全一致。

## 复现

完整 Gate：

```bash
cd /home/accelerate/xuanqiong_x1_sim_ws
bash scripts/build_isolated.sh
bash scripts/run_p5_baseline.sh
```

最终 PASS rosbag 的 RViz 动态复现：

```bash
cd /home/accelerate/xuanqiong_x1_sim_ws
bash scripts/view_p5_rviz.sh
```

RViz Fixed Frame 为 `xq_lio_map`，应显示 0.10 m Navigation Obstacles、Mapped Cloud、
Frontiers、绿色 EGO B-Spline、FAST-LIO/评估真值轨迹和 UAV。脚本会净化录制时钟；
不要另开 `ros2 bag play --clock`，否则会重新引入 RViz 时间回跳。

P5 PASS 只允许按原企划进入 P6 Directional Integrity Predictor；Protection Level、
Alert Limit、Integrity Margin、主动恢复、动态障碍、Atlas 与实机仍未验证。
