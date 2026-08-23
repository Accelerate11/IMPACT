# P8 以后仿真可视化交付约定

从 P8 起，每个新增阶段同时保留三种入口：

1. 无 GUI 自动 Gate：用于可重复验收、CI 和指标生成。
2. Gazebo record/replay：Gate 同步录制状态，独立窗口回放世界与无人机运动。
3. RViz replay/live：显示 FAST-LIO、地图、轨迹、PL、AL、Margin 和后续状态机。

Gazebo GUI 是可视化入口，不代替 JSON Gate、rosbag、节点图和资产哈希证据。WSLg
软件渲染会改变墙钟负载，因此正式 Gate 不挂载 GUI：先无界面运行并录制，再用独立
`GZ_PARTITION` 回放。关闭回放窗口只结束回放，不会改写已经冻结的 Gate 结果。

所有 live 入口必须继续遵守：

- 使用 `/home/accelerate/xuanqiong_x1_sim_ws` 专用工作区；
- 使用唯一 `ROS_DOMAIN_ID` 和 `GZ_PARTITION`；
- world/model 使用 `xq_` 前缀；
- 不修改 `/home/accelerate/cuadc_ws`、`/home/accelerate/ardupilot_gazebo` 的既有地图；
- 自动 Gate 默认 headless；Gazebo 可视化使用本阶段录制的只读状态回放；
- 回放可加载 GUI-only 静态 shell，但不得进入碰撞、LiDAR、规划或评估；
- P8 顶视窗口只将房顶 link 透明，地板、四墙和内部隔断保留；
- 同一阶段同时提供 RViz 算法复现命令。

P8 当前入口：

```bash
# 正式无 GUI Gate
bash scripts/run_p8_alert_limit.sh

# 完整在线 Gate + Gazebo 状态录制；PASS 后自动打开 Gazebo + RViz
bash scripts/run_p8_live_gazebo.sh

# 不重跑 Gate，立即用已有结果同时打开 Gazebo + RViz
bash scripts/view_p8_combined.sh \
  experiments/results/impact_p8/p8_gazebo_final_20260823T104800Z

# 单独重放已验收录制
bash scripts/view_p8_gazebo_replay.sh \
  experiments/results/impact_p8/p8_gazebo_final_20260823T104800Z

# 已验收结果的 RViz 算法回放
bash scripts/view_p8_rviz.sh
```
