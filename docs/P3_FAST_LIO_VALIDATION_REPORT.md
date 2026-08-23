# IMPACT P3 FAST-LIO2 定位基线验收报告

验收日期：2026-08-22（Asia/Shanghai）

最终证据目录：

- structured room：`/home/accelerate/xuanqiong_x1_sim_ws/experiments/results/localization/p3_structured_room_20260822T142413Z_6016`
- long corridor：`/home/accelerate/xuanqiong_x1_sim_ws/experiments/results/localization/p3_long_corridor_20260822T142613Z_6965`

## 结论

Gate P3 为 `PASS`。项目本地 FAST-LIO2 在两个独立 Gazebo 场景中均以 10 Hz 连续
输出 `/localization/odom`，时间戳单调、数值有限、无明显跳变。结构化房间 ATE RMS
为 0.0330 m，低于 0.30 m Gate；长走廊 ATE RMS 为 0.0507 m。两轮算法日志均为
0 次 `No Effective Points`、0 条 ERROR。

该结论是 Gazebo/ROS 2 SIL 算法证据，不是 Livox Mid-360S 实机、Atlas 200I DK A2
满载、室外环境或飞行实机证据。

## 算法与数据边界

`src/xq_fast_lio` 是从既有只读 `FAST_LIO_ROS2` 源码固化出的项目本地 BSD 许可快照。
改动限定为包/namespace 隔离、P2 PointCloud2 与 QoS 适配、输出话题/frame 参数化及
禁止运行时写源码目录；ESIKF、IMU 传播、点到面更新和 ikd-Tree 地图算法保持原实现。

算法只消费：

- `/livox/lidar`：10 Hz `sensor_msgs/PointCloud2`；
- `/livox/imu`：200 Hz `sensor_msgs/Imu`；
- `/clock` 与 ROS 参数事件。

运行时 `algorithm-graph.txt` 证明 `/xq_fast_lio` 不订阅 `/xq/eval/*`。ground truth
只发布到 `/xq/eval/agent_01/ground_truth` 并由 `xq_p3_evaluator` 消费。

P2 点云为瞬时完整帧，没有逐点时间字段。P3 使用通用 PointXYZI 预处理路径，不伪造
Livox offset time；因此本结果不代表真实 Mid-360S 非重复扫描与逐点去畸变性能。

## 从失稳到可验证基线

初始轨迹直接把速度从 0 阶跃到 ±0.4 m/s。Gazebo VelocityControl 会在单个物理步长
内完成速度突变，IMU 因而产生真实飞行器不可能持续跟随的冲击；第三次换向后状态预测
越出点到面关联域并持续报告 `No Effective Points`。这不是可用调大噪声掩盖的问题。

修复将平移和偏航指令改为 C1 连续平滑梯形曲线，峰值速度与场景路径范围不变；偏航
使用正负对称激励并回到初始航向。40 秒诊断跑次随即恢复为 ATE RMS 0.0198 m、
0 次匹配失败，之后才执行正式 Gate。算法没有读取真值，也没有放宽 ATE、频率或跳变
门槛。

## 正式 Gate 结果

| 指标 | structured room | long corridor | 判定 |
|---|---:|---:|---:|
| 连续评估时长 | 86.033 s | 85.171 s | PASS |
| 匹配样本 | 857 | 848 | PASS |
| odom 平均频率 | 10.0 Hz | 10.0 Hz | PASS |
| odom 最大间隔 | 0.100 s | 0.100 s | PASS |
| ATE RMS | 0.0330 m | 0.0507 m | structured PASS |
| 位置误差均值 | 0.0304 m | 0.0469 m | 记录 |
| 位置误差最大值 | 0.0444 m | 0.0669 m | PASS |
| 偏航误差 RMS | 0.1567° | 0.0221° | PASS |
| 1 s 平移 RPE RMS | 0.00258 m | 0.00473 m | PASS |
| 平均处理延迟 | 0.0388 s | 0.0382 s | 记录 |
| 最大处理延迟 | 0.330 s | 0.095 s | 记录 |
| 最大位置步长 | 0.0437 m | 0.0483 m | PASS |
| 最大估计速度 | 0.437 m/s | 0.483 m/s | PASS |
| 非单调/非有限值 | 0 / 0 | 0 / 0 | PASS |
| 运行期匹配失败 | 0 | 0 | PASS |
| FAST-LIO 真值订阅 | 无 | 无 | PASS |
| 外部地图/模型变化 | 无 | 无 | PASS |

## rosbag、构建与隔离证据

- structured room rosbag：184.3 MiB、49,596 条消息；LiDAR 729、IMU 14,587、
  odom 730、评估真值 3,647、TF 730、时钟 29,171。
- long corridor rosbag：180.7 MiB、48,958 条消息；LiDAR 720、IMU 14,399、
  odom 720、评估真值 3,600、TF 720、时钟 28,797。
- 源码树 SHA-256：`ca9121bf712799622e364b496ab7cb60f4b371081123b2fdf488d8c50649a763`。
- 安装树 SHA-256：`244631c6585c5102d3d10a367de5fe923e48a7bf9196b2d50201936a61048b77`。
- 原企划 TXT SHA-256：`cf724bb56599022e4e9b3043e59cecff0ec67f2526360aa6a04b3c76d1d30f53`。
- IMPACT 执行企划 SHA-256：`495bfe2778b9b377b150cc5d89ffd07d7deef4ad349b98196f944e9022dc6403`。
- C++ 转换测试 6/6、桥关闭竞态 10/10 通过；包级汇总 8 项、0 failure。
- 每轮使用独立 DDS domain、唯一 Gazebo partition 和本轮进程组；没有全局
  `pkill/killall`。`/home/accelerate/cuadc_ws`、`/home/accelerate/ardupilot*` 等
  外部 world/model 前后 SHA-256 字节级一致。

## P4 前置边界

P3 PASS 允许按原 IMPACT 执行企划开始 P4 External Navigation 闭环，但不代表飞控已
使用 LIO、GPS 已关闭、三维地图、EGO-Planner、IMPACT 创新、Atlas 或实机完成。
P4 必须显式核验 ArduPilot EKF 源、GPS 参数、真值隔离与完整飞行任务。
