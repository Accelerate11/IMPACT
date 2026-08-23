# IMPACT P2 Mid-360S-like 传感器验收报告

验收日期：2026-08-22（Asia/Shanghai）

正式证据目录：
`/home/accelerate/xuanqiong_x1_sim_ws/experiments/results/sensor_validation/p2_20260822T131950Z_1210`

## 结论

Gate P2 为 `PASS`。同一隔离 Gazebo/ROS 2 进程链连续采集 600.435 秒，LiDAR 与
IMU 的 header 时间分别覆盖 599.495 秒和 599.553 秒。点云稳定为 10.00008 Hz，
IMU 为 200.00083 Hz；时间戳严格单调，TF、frame、字段、点数与有限值检查全部通过。

该结论只证明 Mid-360S-like 仿真传感器与 ROS 2 数据契约。它不证明 Livox
Mid-360S 的真实非重复扫描模式、真实驱动、逐点时间模型或 FAST-LIO2 性能。

## 从物理量到算法接口

1. 传感器生成层：Gazebo SDF 生成 360°×59°、720×32 点、10 Hz 点云，量程噪声为
   零均值高斯噪声、标准差 0.015 m；IMU 以 200 Hz 生成，并分别配置角速度和线加速度噪声。
2. 误差条件层：专属 C++ 桥提供固定随机种子的消息丢帧、LiDAR/IMU 时间戳抖动、
   基于扫描周期与设定运动速度的点云运动畸变，以及六自由度 LiDAR 外参误差。
3. 数据契约层：只向后续算法暴露 `/livox/lidar`、`/livox/imu` 和静态 TF；P2 关闭
   ground-truth 发布，P3 不依赖 Gazebo 私有话题。
4. 验收层：验证器直接遍历 PointCloud2 的 `x/y/z` 布局和所有点，持续累计频率、
   最大间隔、时间戳、frame、TF、NaN/Inf 与点数；结果每秒原子写入 JSON。

基线配置关闭人为丢帧、抖动、运动畸变和外参偏差，只保留 SDF 物理噪声。这样 P2
先固定名义数据契约；后续退化/鲁棒性实验再逐项打开条件参数，不把多个误差源混成
一个无法归因的基线结果。

## 正式 Gate 结果

| 检查 | 正式观测 | 判定 |
|---|---:|---:|
| 单调采集时长 | 600.435 s | PASS |
| LiDAR header 时间跨度 | 599.495 s | PASS |
| LiDAR 平均频率 | 10.00008 Hz | PASS |
| LiDAR 最大相邻间隔 | 0.100 s（门槛 0.250 s） | PASS |
| IMU header 时间跨度 | 599.553 s | PASS |
| IMU 平均频率 | 200.00083 Hz（门槛 >=100 Hz） | PASS |
| IMU 最大相邻间隔 | 0.005 s（门槛 0.050 s） | PASS |
| 验证器消息数 | LiDAR 5,996；IMU 119,912 | PASS |
| 点云规模 | 每帧 23,040 点 | PASS |
| 点云字段 | `x/y/z/intensity/ring` | PASS |
| 时间戳 | 0 个非单调、0 个零时间戳 | PASS |
| frame / TF | `xq_base_link -> livox_frame/livox_imu` | PASS |
| NaN/Inf / 布局违规 | 0 / 0 | PASS |
| 外部地图/模型哈希 | 前后完全一致 | PASS |
| 日志错误扫描 / 核心进程残留 | 0 / 0 | PASS |

TF 实测为：

- `xq_base_link -> livox_frame`：平移 `[0.04, 0.0, 0.135] m`，单位四元数；
- `xq_base_link -> livox_imu`：平移 `[0.0, 0.0, 0.015] m`，单位四元数。

## rosbag 证据

rosbag 为 10 个 60 秒上限的 zstd 文件分卷，总大小 1.5 GiB、365,535 条消息：

| 话题 | 类型 | 录制数 |
|---|---|---:|
| `/livox/lidar` | `sensor_msgs/msg/PointCloud2` | 5,993 |
| `/livox/imu` | `sensor_msgs/msg/Imu` | 119,846 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 2 |
| `/clock` | `rosgraph_msgs/msg/Clock` | 239,694 |

rosbag metadata 的主机 realtime 接收跨度为 552.660 秒，短于单调时钟的
600.435 秒；同一轮 ROS 日志的 realtime 也发生了相同约 47 秒偏移。这里不把该值
冒充连续时长：连续性由不受系统校时影响的 `time.monotonic()`、599.5 秒传感器 header
跨度及 5,993/119,846 条实际录包共同交叉证明。

## 回归与隔离

- 隔离构建：5 个包成功；源码树 SHA-256
  `aa30be255eb157c7925520c3b1787bca7e77f44cb39dd9e5aef07a012af97266`；
- Python：46/46 PASS，其中新增“首帧未到达必须保持 IN_PROGRESS”回归；
- C++/包级：8 项、0 failure，含 10/10 关闭竞态回归；
- 90 秒初测与修复后 60 秒复测均 PASS；
- 正式运行只管理本轮进程组，未使用全局 `pkill/killall`；
- `/home/accelerate/cuadc_ws`、`/home/accelerate/ardupilot*` 等旧地图和模型前后
  SHA-256 字节级一致。

## P3 稳定边界

P3 FAST-LIO2 只能消费以下 P2 输出：

- `/livox/lidar`，frame=`livox_frame`，PointCloud2；
- `/livox/imu`，frame=`livox_imu`，Imu；
- `/tf_static` 中 `xq_base_link -> livox_frame/livox_imu`。

P3 必须继续禁止算法订阅 ground truth，并单独建立 structured room、degraded geometry、
dynamic obstacle 的 ATE/频率 Gate。P2 PASS 不自动提升 P3 状态。
