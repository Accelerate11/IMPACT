# P14 Fault Injection & Resilient Autonomy 设计记录

## 目标

在 P12 已验收的开顶动态走廊和 P13 时延安全闭环上，建立确定性、可回放、可逐项
验收的故障注入层。故障必须真正作用于算法输入或处理链，而不是只发布一个标签；
安全监督器必须在不订阅 Ground Truth 的条件下把故障映射到有界降级动作。

## 故障执行面

统一入口：

```bash
ros2 launch impact_fault_injection sim_fault.launch.py \
  fault:=lidar_dropout start_time:=18.0 duration:=12.0
```

正式矩阵由固定随机种子和 JSON 时序表驱动，覆盖：

| 故障 | 实际作用 |
|---|---|
| LiDAR / IMU dropout | 传感器代理真实丢弃消息 |
| timestamp jitter | 修改转发消息时间戳 |
| odometry delay | 延迟 FAST-LIO 输出 |
| covariance inflation | 放大定位协方差 |
| planner delay | 控制器回调内有界阻塞 |
| camera failure | 切换为 LiDAR 几何回退 |
| CPU load | 有界计算负载并卸载非关键任务 |
| 20% packet loss | 项目内 ground-link relay 按种子精确丢包 |
| low battery | 触发返航状态，不伪造飞控电池硬件 |

代理只改写本项目命名空间内的数据流，不修改 WSL 网络、ROS 全局配置、其他项目地图
或模型。P14 复用 P12 世界，场景 SHA-256 固定为
`12c2cbbfdb76c1ccfacde2a0d7db590c37e25ff7553fc5edb69b294e144712fd`。

## 安全状态机

```text
NORMAL → CAUTIOUS → RECOVERY → BRAKE → HOVER → LAND
                                  └────────────→ RETURN
```

- 短时/可替代故障优先保持任务：相机失败使用几何回退，地面链路丢包不进入飞行闭环。
- CPU、IMU、时间戳异常进入 `CAUTIOUS`；延迟里程计或协方差膨胀进入 `RECOVERY`。
- 规划超时立即 `BRAKE`；低电量进入 `RETURN`。
- 持续 LiDAR 中断按 0.5/1.2/1.8/3.0 s 阈值依次进入
  `RECOVERY/BRAKE/HOVER/LAND`。
- LAND 使用命令积分的有界下降，不依赖中断后冻结的 LIO；Gazebo 真值只由独立
  evaluator 用于核验真实下降和末速。

P11 硬认证仍不可放宽、被拒轨迹仍不可执行。若无故障、P12 通道已重开且局部动态区
清空，但重复硬拒绝造成观测死锁，控制器仅允许执行幅值 0.35 m、速度 0.08 m/s 的
最小激励横移后重新认证。

## 验收结构

1. `matrix`：在完成 24 m 走廊飞行期间依次执行十种故障，并同时跑 P12/P13 保持性
   evaluator。
2. `emergency`：持续 12 s LiDAR 中断，验证完整有序的失效安全状态序列和真实下降。
3. 总 Gate 只在两轮均 PASS、P12/P13 均保持、世界哈希一致、Ground Truth 隔离和
   外部资产逐字节审计均通过时返回 PASS。

Gazebo 回放继续开顶、保留墙体；RViz 叠加故障、状态、路径、动态地图、时延和完整性
信息。
