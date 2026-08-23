# 玄穹-X1 Gazebo SIL 运行手册

## 1. 已核对的运行环境

- WSL2：Ubuntu 22.04.5 LTS
- ROS 2：Humble Desktop 0.10.0
- Gazebo：Harmonic，`gz sim` 8.13.0
- Gazebo C++ 接口：`gz-msgs10` 10.3.2、`gz-transport13` 13.5.0
- 可选飞控底座：ArduCopter SITL 4.5.7（本阶段不把算法载体等同于飞控验证）

Windows 目录 `D:\jbgs\xuanqiong_x1_sim_ws` 是源码主副本，WSL 运行副本固定为
`/home/accelerate/xuanqiong_x1_sim_ws`。本工作区不 source、include 或构建
`cuadc_ws`、`ardupilot_gazebo` 等既有仿真项目。

## 2. 同步与构建

在 Windows PowerShell 中运行：

```powershell
Set-Location D:\jbgs\xuanqiong_x1_sim_ws
.\scripts\sync_to_wsl.ps1
```

同步脚本使用 WSL UNC 路径，只镜像本项目的 `src/scripts/docs`，忽略 Python 测试缓存，
并只会清理由本项目固定
WSL 源码目录中已经从 Windows 主副本删除的受控文件。

随后在 Ubuntu-22.04 WSL 中运行：

```bash
cd /home/accelerate/xuanqiong_x1_sim_ws
bash scripts/build_isolated.sh
```

不要在 MAVROS 或其他仿真仍运行时执行 `wsl --shutdown`、重启 WSL 服务或结束
`vmmemWSL`；这些操作会终止同一发行版中的其他项目。本轮曾在用户明确授权并确认
其他任务可停止后重启 `Ubuntu-22.04`，这不是日常运行步骤。

构建脚本会清除继承的 ROS/Gazebo overlay 环境，只构建当前 `src/`，输出到专属的
`xq_build/`、`xq_install/` 和 `xq_log/`。成功后会生成
`xq_install/.xq_build_manifest.json`；运行脚本会重新计算源码树和安装树哈希，过期安装
会被拒绝，不能误跑旧代码。

## 3. 自动运行

30 秒无故障冒烟测试：

```bash
cd /home/accelerate/xuanqiong_x1_sim_ws
bash scripts/run_smoke.sh --duration 30 --require-algorithm-pass
```

确定性 F1–F8 故障矩阵（默认 120 秒墙钟）：

```bash
bash scripts/run_smoke.sh --with-faults
```

故障按仿真时间注入：F1 camera 3 s、F2 NPU 6 s、F3 planner 9 s、F4 LiDAR 13 s、
F5 定位高协方差 17 s、F6 ground-link 21 s、F7 CPU 负载代理 28 s、F8 低电量 33 s；
最后一个窗口在 35 仿真秒结束。CPU 项只在本项目内关闭非关键 OAER 重规划并标记
`ESSENTIAL_ONLY`，不会对 WSL 施加真实 CPU 压力。NPU 项的“恢复”是定时恢复代理，
不是实进程重启证据。

脚本会在计划事件缺失、响应窗不完整或任一逐故障检查非 `SIMULATED_PASS` 时非零退出。
F6 还要求至少 40 个窗口心跳、活动 fault ID 匹配、观测和计数差丢包率均在 15%–25%，
以及代理 odom/map 连续；不能以墙钟运行结束冒充故障矩阵完成。

可选 GUI 调试：

```bash
bash scripts/run_smoke.sh --gui --duration 30
```

每次运行都会只读扫描 `/proc`，从 Linux 建议范围 32–101 选择当时未占用的
`ROS_DOMAIN_ID`，并生成唯一 `GZ_PARTITION`；DDS 仅限本机。停止时只向本次 launch 的
专属进程组发信号，不使用 `killall`、全局 `pkill` 或主机网络级 `tc netem`。20% 丢包由
`xq_network_relay` 在项目 heartbeat 话题内执行，窗口外配置丢包率严格为 0。

WSL 的默认 D3D/EGL 组合会使 GPU lidar 崩溃；headless runner 已固定使用
Mesa llvmpipe 与 surfaceless EGL。GUI 仅用于人工调试，不作为自动验收前提。

## 4. 证据与判读

运行结果写入 `runs/smoke_<UTC时间>_<PID>/`：

- `metrics.json`：机器可读指标和 `SIMULATED_PASS` / `SIMULATED_FAIL` 状态；
- `report.md`：简要中文报告；
- `state_timeline.json`：Sentinel 状态转换；
- `health_samples.json`：模块健康证据；
- `fault_events.json`：启用故障注入时的事件记录；
- `fault_response_evidence.json`：F1–F8 的预期检查、观测值和三态结论；
- `network_stats.json`：项目内 ground-link TX/RX/丢包窗口统计；
- `configuration/` 与 `configuration-manifest.json`：本次实际使用的 world、model、
  bridge、stack、fault schedule、launch 和构建清单快照及 SHA-256；
- `ros_logs/`：本次专属 ROS 日志；
- `launch.log`：Gazebo 与 ROS 节点日志；
- `external-assets.before.sha256` / `external-assets.after.sha256`：旧地图前后快照；
- `isolation-audit.txt`：字节级隔离审计结果。

自动报告中的 ATE、频率和代理规划结果均属于 `GAZEBO_SIL_PROXY`。正式 R6
“障碍首次确认到新安全轨迹”当前明确为 `UNVERIFIED`，周期/no-path 代理事件不能替代。
以下内容也必须保持
`UNVERIFIED`，不能由本仿真替代：Atlas 机载功耗与温升、真实 FAST-LIO2 / EGO-Planner
性能、正式室内/室外 ATE、真实 5 cm 三维地图质量、跨主机 DDS、多机实飞。

## 5. 单元与接口测试

```bash
cd /home/accelerate/xuanqiong_x1_sim_ws
source /opt/ros/humble/setup.bash
source xq_install/setup.bash

PYTHONPATH=src/xq_autonomy python3 -m pytest -q \
  src/xq_autonomy/test

colcon --log-base xq_test_log test \
  --build-base xq_build --install-base xq_install \
  --packages-select xq_gz_bridge
colcon test-result --test-result-base xq_build --verbose
```

Python 测试当前为 46 项，覆盖退化感知、动态地图 TTL、OAER、可达前沿、向量化障碍
膨胀等价性、规划 deadline/BRAKE、Sentinel F1–F8
边界、项目内确定性 20% 丢包、逐故障验收契约、P2 首帧启动状态、真值源码隔离和指标公式。C++ 测试覆盖 Gazebo/ROS
时间戳、点云、IMU、真值、Twist 转换及活跃回调退出竞态。

## 6. 真值隔离检查

算法栈允许的订阅仅为传感器和测试故障话题：

```bash
ros2 node info /xq_stack_node
```

输出中不得出现 `/xq/eval/`。`/xq/eval/agent_01/ground_truth` 只允许 metrics 节点订阅。
自动报告只把这一点标为“契约与源码审计”；若没有保存上述运行时 graph 输出，仍保持
`runtime_graph_audit=UNVERIFIED`。

`xq_base_link -> xq_mid360_link` 与 `xq_base_link -> xq_imu_link` 已由 bringup 发布静态 TF，
位姿与 SDF 固定关节一致。

## 7. 下一阶段接入点

当前 `daf_lio_proxy_2d` 和 `r2_ego_proxy_2d` 用于先验证接口、状态机和闭环，不冒充
生产算法。后续可沿既定话题契约逐步替换为：

1. `livox_ros_driver2` / 仿真适配器与真实 FAST-LIO2；
2. 三维 TD-SemMap、OAER 与 EGO-Planner B-spline 优化器；
3. ArduPilot SITL + MAVROS ExternalNav；
4. 独立 namespace/端口的 2–3 机 SITL 与项目级丢包 relay；
5. Atlas 200I DK A2 HIL 和实机验收。

## 8. P1 ArduPilot + Gazebo + MAVROS 真飞控基线

先同步并构建 Windows 源码，再在 WSL 执行：

```bash
cd /home/accelerate/xuanqiong_x1_sim_ws
bash scripts/build_isolated.sh
bash scripts/run_p1_flight_baseline.sh --duration 600 --hover 30
```

状态机顺序为 `WAIT_FCU -> SET_STREAM -> WAIT_ODOM -> SET_GUIDED -> ARM ->
TAKEOFF -> ASCEND -> HOVER -> LAND -> DESCEND -> DONE`。它会等待 EKF 本地
里程计，而不是在 heartbeat 到达后立即盲目解锁。

运行器启动前检查 TCP 5760 与 UDP 9002；任一端口已有使用者时直接退出，
绝不结束占用进程。运行时使用独立 DDS domain 和 Gazebo partition，仅在
退出时回收本轮记录的进程组。结果位于 `runs/p1_<UTC>_<PID>/`：

- `mission-result.json`：逐状态、服务接受结果、高度与最终 PASS/FAIL；
- `summary.json`：600 秒 Gate、rosbag 和隔离审计汇总；
- `rosbag/`：MAVROS state、odom、IMU、GPS；
- `sitl.log`、`gazebo.log`、`mavros.log`、`mission.log`：原始日志；
- `runtime-dependencies.sha256`：本轮二进制、参数、插件和模型哈希；
- `external-assets.*.sha256`、`isolation-audit.txt`：旧地图/模型未变证明。

P1 只证明真飞控闭环和基础通信稳定。没有 `/livox/lidar` 数据契约、
FAST-LIO2 ATE 和三维规划证据时，不得宣称 P2-P8 完成。

## 9. P2 Mid-360-like 传感器契约

先同步并隔离构建，然后执行默认 600 秒 Gate：

```bash
cd /home/accelerate/xuanqiong_x1_sim_ws
bash scripts/build_isolated.sh
bash scripts/run_p2_sensor_validation.sh --duration 600
```

运行器只加载本项目安装树，以独立 DDS domain、Gazebo partition 和进程组启动
headless Gazebo、专属桥、静态 TF 与验证器。结果固定写入
`experiments/results/sensor_validation/p2_<UTC>_<PID>/`，包含：

- `sensor-validation.json`：连续频率、最大间隔、时间戳、字段、点数、TF、NaN/Inf；
- `summary.json`：P2 Gate 汇总；
- `rosbag/`：按 60 秒切分并以 zstd 文件压缩的 LiDAR、IMU、TF 与时钟；
- `configuration/`、`configuration.sha256`：实际 world、model、桥配置、launch 和构建清单；
- `external-assets.*.sha256`、`isolation-audit.txt`：旧项目地图/模型未变证据；
- `launch.log`、`rosbag.log`、`ros_logs/`：本轮原始日志。

Gate 要求墙钟连续不少于 600 秒、点云 9.5–10.5 Hz、IMU 不低于 100 Hz、
点云最大样本间隔不高于 0.25 秒、IMU 不高于 0.05 秒、时间戳严格单调、frame/TF
正确、点云含 `x/y/z` 且每帧不少于 1000 点、所有坐标与 IMU 数值有限。

P3 的稳定输入边界为：

| 输入 | 类型 | frame | 基线频率 |
|---|---|---|---:|
| `/livox/lidar` | `sensor_msgs/msg/PointCloud2` | `livox_frame` | 10 Hz |
| `/livox/imu` | `sensor_msgs/msg/Imu` | `livox_imu` | 200 Hz |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | `xq_base_link -> livox_*` | 静态 |

Gazebo 私有话题 `/xq/lidar/points`、`/xq/imu` 不属于 P3 算法接口。真实
Mid-360 的非重复扫描模式、逐点时间和 Livox 自定义消息仍为 `UNVERIFIED`；P3 接入
FAST-LIO2 前必须明确采用 PointCloud2 适配器还是 livox_ros_driver2 数据路径。

## 10. P3 FAST-LIO2 定位基线

P3 选择 P2 已验证的标准 `PointCloud2` 路径；仿真帧没有逐点采样时间，因此不伪造
Livox offset time。先运行结构化房间，再使用完全相同构建运行长走廊：

```bash
cd /home/accelerate/xuanqiong_x1_sim_ws
bash scripts/build_isolated.sh
bash scripts/run_p3_fast_lio.sh --scenario structured_room --duration 75
bash scripts/run_p3_fast_lio.sh --scenario long_corridor --duration 75
```

运行器将结果写入 `experiments/results/localization/p3_<scenario>_<UTC>_<PID>/`：

- `evaluation.json`：ATE、RPE、位置/偏航误差、频率、间隔、处理延迟与跳变检查；
- `algorithm-graph.txt`：必须证明 `/xq_fast_lio` 没有 `/xq/eval/*` 订阅；
- `rosbag/`：LiDAR、IMU、odom、评估真值、TF 与时钟，按 60 秒 zstd 分卷；
- `configuration/`、`fast-lio-source-tree.sha256`：场景、配置、算法来源与源码哈希；
- `external-assets.*.sha256`、`isolation-audit.txt`：旧项目地图/模型未变证明。

Gate 要求 `/localization/odom` 不低于 10 Hz、最大间隔不高于 0.25 秒、时间戳单调、
数值有限、无明显跳变；`structured_room` 另要求 ATE RMS 不高于 0.30 m。轨迹使用
C1 连续平滑速度，避免运动学速度控制器产生不符合真实飞行器动力学的加速度冲击。
P3 PASS 只允许进入原企划的 P4 External Navigation；飞控闭环、三维建图、规划、
Atlas 和实机指标仍未验证。

## 11. P4 GPS-off ExternalNav 闭环

构建后运行单条正式命令：

```bash
cd /home/accelerate/xuanqiong_x1_sim_ws
bash scripts/build_isolated.sh
bash scripts/run_p4_external_nav.sh --minimum-eval-duration 70
```

运行器在本轮独立目录中启动 ArduPilot SITL、MAVROS、Gazebo、FAST-LIO、ExternalNav
适配器、评估器和任务状态机。飞控必须逐项读回 GPS-off 与 EKF3 ExternalNav 参数，
随后完成 2 m 起飞、悬停、2 m × 2 m 矩形、返回和降落。任何阶段失败均返回非零，
不会只因节点存在或起飞成功而判 PASS。

结果位于 `experiments/results/external_nav/p4_<UTC>_<PID>/`，重点文件为：

- `summary.json`、`mission-result.json`：正式 Gate 和飞行状态机证据；
- `localization-evaluation.json`：飞行期间 ATE/RPE、跳变与延迟；
- `external-nav-topic-graph.txt`、`ground-truth-topic-graph.txt`：数据流与真值隔离；
- `sitl_runtime/logs/*.BIN`：ArduPilot DataFlash；
- `rosbag/`、`mavros.log`、`p4-stack.log`：可追溯原始证据；
- `isolation-audit.txt`：既有 Gazebo/ArduPilot 地图模型未变证明。

从结果 JSON 生成轨迹图：

```bash
python3 scripts/plot_p4_result.py \
  experiments/results/external_nav/p4_<UTC>_<PID>
```

动态 RViz 复现最终 PASS rosbag（自动补齐动态 TF、两条 Path、无人机 Marker，并过滤
录制时钟的乱序样本）：

```bash
cd /home/accelerate/xuanqiong_x1_sim_ws
bash scripts/view_p4_rviz.sh
```

也可显式指定其他 P4 结果目录：

```bash
bash scripts/view_p4_rviz.sh \
  /home/accelerate/xuanqiong_x1_sim_ws/experiments/results/external_nav/p4_<UTC>_<PID>
```

RViz 应显示 `P4 LiDAR`、`FAST-LIO Path`、`Evaluation Truth Path` 和 `UAV`，Fixed
Frame 必须为 `xq_lio_map`。不要额外运行 `ros2 bag play --clock`，bag 已含仿真时钟，
第二个 `/clock` 发布器会造成 RViz jump-back 重置。

P4 PASS 只允许进入 P5 Baseline Map + Frontier + EGO；真实 Mid-360S、真实飞控、
Atlas 和实机无 GPS 飞行仍为 `UNVERIFIED`。

## 12. P5 Baseline Map + Frontier + EGO

完整 Gate：

```bash
cd /home/accelerate/xuanqiong_x1_sim_ws
bash scripts/build_isolated.sh
bash scripts/run_p5_baseline.sh
```

运行器在独立 `xq_p5_structured_room` 中启动 FAST-LIO2、0.10 m 导航栅格、Frontier
候选视点选择、官方 ROS2 EGO-Planner、GPS-off ExternalNav、ArduPilot SITL 与
0.05 m 评估器。只有任务自动结束/降落、0 碰撞、真值隔离和外部资产审计全部通过，
才写出 `summary.json` 并返回 0。

最终 PASS rosbag 的 RViz 动态复现：

```bash
cd /home/accelerate/xuanqiong_x1_sim_ws
bash scripts/view_p5_rviz.sh
```

也可指定其他 P5 结果目录：

```bash
bash scripts/view_p5_rviz.sh \
  /home/accelerate/xuanqiong_x1_sim_ws/experiments/results/baseline_v1/p5_<UTC>_<PID>
```

Fixed Frame 为 `xq_lio_map`。应看到 Navigation Obstacles、Mapped Cloud、Frontiers、绿色
EGO B-Spline、FAST-LIO/评估真值轨迹与 UAV。脚本已经播放 bag，不要另开第二个
`ros2 bag play --clock`。

## 13. P6 Directional Integrity Predictor

完整 Gate：

```bash
cd /home/accelerate/xuanqiong_x1_sim_ws
bash scripts/build_isolated.sh
bash scripts/run_p6_directional_integrity.sh
```

结果写入 `experiments/results/impact_p6/p6_<UTC>_<PID>/`。关键证据为
`integrity-result.json`、`p6-full-bag-analysis.json`、`summary.json`、
`p6-node-graph.txt` 和包含 `/localization/geometry`、`/integrity/directional` 的 rosbag。

RViz 动态复现：

```bash
cd /home/accelerate/xuanqiong_x1_sim_ws
bash scripts/view_p6_rviz.sh
```

紫红箭头为当前弱方向，RGB 箭头为 map-X/Y/Z 方向 PL，半透明椭球为轴向 PL 包络；
长度为可见性放大 20 倍，悬浮文本仍显示真实米制 PL。P6 不向规划器反馈，P7 校准
完成前不得把 `k_alpha=3.0` 表述为已验证的 95%/99% 覆盖率。

## 14. P7 Protection Level 训练/验证校准

完整 Gate：

```bash
cd /home/accelerate/xuanqiong_x1_sim_ws
bash scripts/build_isolated.sh
bash scripts/run_p7_calibration.sh
```

运行器依次执行 structured-room/long-corridor 两轮训练，写入并冻结
`p7-calibration.json`，再执行两条未参与标定的新验证轨迹。任何训练/测试 capture、
P3 定位 Gate、覆盖率 Gate、标定哈希或外部资产审计失败都会返回非零。

结果位于 `experiments/results/impact_p7/p7_<UTC>_<PID>/`，重点文件为：

- `p7-calibration.json`：只由训练输入生成的方向 `k95/k99` 与场景迁移储备；
- `calibration.before-tests.sha256` / `calibration.after-tests.sha256`：冻结证明；
- `summary.json`：两场景与聚合 95%/99% 覆盖率；
- `train_*` / `test_*`：各轮 capture、P3 评估、topic graph、rosbag 与隔离审计。

最终 PASS 结果的校准 PL95 RViz 动态复现：

```bash
cd /home/accelerate/xuanqiong_x1_sim_ws
bash scripts/view_p7_rviz.sh
```

也可显式指定其他 P7 总结果目录：

```bash
bash scripts/view_p7_rviz.sh \
  /home/accelerate/xuanqiong_x1_sim_ws/experiments/results/impact_p7/p7_<UTC>_<PID>
```

Fixed Frame 为 `xq_lio_map`。箭头、包络和文本显示冻结 `k95` 对应的 PL95；脚本已经
播放 `test_structured_room/rosbag`，不要另开第二个 `/clock` 发布器。P7 没有 Alert
Limit，因此该阶段 false-alarm rate 为 N/A；P8 当前状态见下一节，P9 与实机完整性
仍未验证。

## 15. P8 Static-Obstacle Alert Limit

构建并重放完整 Gazebo 自主探索证据：

```bash
cd /home/accelerate/xuanqiong_x1_sim_ws
bash scripts/build_isolated.sh
bash scripts/run_p8_alert_limit.sh
```

Gazebo 在线录制与独立可视化回放：

```bash
cd /home/accelerate/xuanqiong_x1_sim_ws
bash scripts/run_p8_live_gazebo.sh
```

该命令先以无 GUI 方式完成在线自主探索、P8 Gate 和 `state.tlog` 录制；全部 PASS 后
自动同时打开独立 Gazebo 与 RViz 回放。这样 GUI 帧率不会改变算法墙钟 Gate。回放顶视角仅把房顶
透明，四面墙、地板和内部隔断保留。

单独复现已录制的 Gazebo 结果：

```bash
bash scripts/view_p8_gazebo_replay.sh \
  /home/accelerate/xuanqiong_x1_sim_ws/experiments/results/impact_p8/p8_gazebo_final_20260823T104800Z
```

立即同时打开 Gazebo 与 RViz（不重跑 Gate）：

```bash
bash scripts/view_p8_combined.sh \
  /home/accelerate/xuanqiong_x1_sim_ws/experiments/results/impact_p8/p8_gazebo_final_20260823T104800Z
```

默认输入为 P6 最终完整仿真 rosbag。运行器只重放 `/clock`、FAST-LIO odom、EGO
B-spline 和项目局部静态点云，并记录 `/integrity/alert_limit`。结果写入
`experiments/results/impact_p8/p8_<UTC>_<PID>/`，关键证据为：

- `alert-limit-result.json` / `summary.json`：11 项 Gate 与 AL 分布；
- `p8-node-graph.txt`：输入契约和 Ground Truth 隔离；
- `rosbag/`：轨迹、点云、odom、AL 与 debug；
- `p7-prerequisite.sha256`：P7 阶段先决条件；
- `isolation-audit.txt`：既有地图模型未变。

RViz 动态复现：

```bash
cd /home/accelerate/xuanqiong_x1_sim_ws
bash scripts/view_p8_rviz.sh
```

蓝球为临界轨迹采样点，橙球为最近静态障碍，箭头指向障碍。绿色文本表示 AL 非负，
红色 `NO ERROR BUDGET` 表示 `AL < 0`；它不是告警状态机。P8 不反馈 EGO，P9 才将
校准 PL 与 AL 合成为不可被收益抵消的 Integrity Margin 硬约束。

## 16. P9 Integrity Margin 硬认证

构建并运行正式 Gate：

```bash
cd /home/accelerate/xuanqiong_x1_sim_ws
bash scripts/build_isolated.sh
bash scripts/run_p9_integrity_margin.sh
```

运行器在同一个开顶 Gazebo 认证世界中构造净宽 3.0 m 的 wide room 与净宽 1.2 m 的
narrow passage；两条轨迹共用完全相同的 `P_int`。结果保存到
`experiments/results/impact_p9/p9_<UTC>_<PID>/`。重点文件为：

- `p9-gate-result.json`：宽 ACCEPT、窄 REJECT、同协方差和硬门检查；
- `summary.json`：rosbag、Gazebo record、真值隔离与外部资产审计总结果；
- `margin-node-graph.txt` / `gate-node-graph.txt`：算法输入输出边界；
- `p7-calibration.sha256`：train-only 冻结校准先决条件；
- `rosbag/` / `gz_record/`：ROS 与 Gazebo 原始回放证据。

不重跑 Gate，立即同时打开正式结果的 Gazebo 与 RViz：

```bash
cd /home/accelerate/xuanqiong_x1_sim_ws
bash scripts/view_p9_combined.sh
```

也可显式指定结果，或分别打开：

```bash
bash scripts/view_p9_combined.sh \
  experiments/results/impact_p9/p9_20260823T135112Z_1110615
bash scripts/view_p9_gazebo_replay.sh \
  experiments/results/impact_p9/p9_20260823T135112Z_1110615
bash scripts/view_p9_rviz.sh
```

一条命令重新验收并在 PASS 后打开两个窗口：

```bash
bash scripts/run_p9_live_gazebo.sh
```

Gazebo 无房顶但保留宽/窄空间墙体；RViz 绿色表示 wide-room ACCEPT，红色表示
narrow-passage REJECT，并显示 AL、方向 PL、最小 Margin 与储备。该阶段拒绝时不向
`/planning/bspline` 发布候选轨迹；P10 才实现拒绝后的主动感知/恢复。
