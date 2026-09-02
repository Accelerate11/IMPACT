# P13 Latency-Aware Safety 设计记录

## 目标

在完全相同的 Gazebo 动态走廊、LiDAR、FAST-LIO、P11/P12 算法和几何参数下，只把
规划处理延迟从 50 ms 改为 200 ms，证明实测端到端 p99 会收紧 Alert Limit，并使
控制器选择更保守的速度。

## 时间链

每次新 LiDAR/地图周期记录：

```text
sensor_timestamp → receive_timestamp → localization_done → map_done
→ planner_trigger → planner_done → trajectory_certified → command_sent
```

传感器时间戳属于 ROS 仿真时钟；处理阶段使用系统时钟，持续时间使用 steady clock，
避免系统时钟校准跳变。统计采用不插值的 nearest-rank p50/p95/p99/max，安全计算只使用
p99，不使用平均值。

## 安全包络

```text
r_latency = v * L_p99 + 0.5 * a_max * L_p99^2
AL_raw    = d - r_body - r_base - r_tracking - r_dynamic - r_latency
M_raw     = AL_raw - PL
```

若 `M_raw` 小于冻结储备，则反解最大允许速度，使缓解后的 `M_safe >= M_required`。
这一区分很重要：raw AL 证明时延本身造成的风险，safe AL/Margin 证明降速确实恢复了
安全储备。

候选轨迹如果被 P11 硬认证拒绝，P13 先悬停 2 s，再用新点云和全新 batch id 重新采样；
最多 6 次，任何被拒轨迹均不会下发。

## 数据边界

- 控制器只订阅 LiDAR、FAST-LIO、地图、完整性和规划话题。
- Ground Truth 只进入 P12/P13 evaluator。
- P13 复用 P12 世界，不修改任何既有地图或模型。
- Gazebo 继续开顶并保留全部墙体。
