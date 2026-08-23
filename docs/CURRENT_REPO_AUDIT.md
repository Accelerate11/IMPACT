# 当前仓库审计（IMPACT 执行基线）

审计日期：2026-08-22  
Windows 源码：`D:\jbgs\xuanqiong_x1_sim_ws`  
WSL 运行副本：`/home/accelerate/xuanqiong_x1_sim_ws`

## 1. 结论

仓库已经具备一个可复现、项目隔离的 Gazebo Harmonic 算法级 SIL，现有
2D 代理定位、建图、探索、规划、监督器和 F1-F8 故障注入均有测试证据。
它不能被表述为 FAST-LIO2、EGO-Planner 或真飞控验证。

依据 `IMPACT - XH-202629 — Codex 仿真与算法执行企划`，本次不推倒现有
代理基线，而是在其旁路增加 P1 真飞控基线。P1 使用 ArduCopter SITL、
Gazebo Harmonic 的 ArduPilotPlugin 和 ROS 2 MAVROS，接口置于
`/uav1/mavros/*`；原 `/xq/agent_01/*` 代理栈不变。

## 2. 仓库包与职责

| 包 | 当前职责 | IMPACT 阶段定位 |
|---|---|---|
| `xq_sim_interfaces` | 健康、定位质量、故障、重规划等自定义消息 | 后续 P4/P8/P9 可复用接口 |
| `xq_gz_assets` | 项目专属 world/model，名称均为 `xq_` 前缀 | P1 场景及后续传感器载体 |
| `xq_gz_bridge` | gz-transport13 到 ROS 2 的点云、IMU、真值桥 | P2/P3 传感器接入基础 |
| `xq_autonomy` | 2D 代理 LIO、地图、OAER、A*、Sentinel、指标 | 对照组；新增 P1 飞行状态机 |
| `xq_sim_bringup` | 代理 SIL 启动和隔离环境变量 | 保留原基线，P1 由独立运行器启动 |

## 3. 已审计入口

- 构建：`scripts/build_isolated.sh`
- 代理 SIL：`src/xq_sim_bringup/launch/xq_sil.launch.py`
- 主代理节点：`src/xq_autonomy/xq_autonomy/stack_node.py`
- 参数：`src/xq_autonomy/config/stack.yaml`
- Gazebo 世界：`src/xq_gz_assets/worlds/xq_indoor_office.sdf`
- 代理飞行器：`src/xq_gz_assets/models/xq_iris_mid360/model.sdf`
- 现有测试：Python 算法/指标测试与 C++ 转换/关闭竞态测试

原模型明确关闭重力并使用 `VelocityControl`，因此只能验证算法接口和
安全逻辑，不能证明飞控闭环。此次新增的
`xq_p1_ardupilot_empty.sdf` 使用真实刚体动力学与 ArduPilotPlugin。

## 4. WSL 依赖审计

| 依赖 | 实测状态 |
|---|---|
| Ubuntu | 22.04.5 LTS |
| ROS 2 | Humble，位于 `/opt/ros/humble` |
| Gazebo | Harmonic `gz-sim 8.13.0` |
| MAVROS | `mavros`、`mavros_msgs`、`mavros_extras` 已安装 |
| ArduPilot | `/home/accelerate/ardupilot`，Copter 4.5.7 SITL 可执行文件已构建 |
| Gazebo 插件 | `/home/accelerate/ardupilot_gazebo/build/libArduPilotPlugin.so` |
| 可用传感/算法资产 | Livox SDK2、FAST_LIO_ROS2 等存在，但尚未纳入本项目运行时 |

`/home/accelerate/ardupilot` 与 `/home/accelerate/ardupilot_gazebo` 当前不是
Git 工作树，不能用提交号证明版本。因此每轮 P1 都保存实际二进制、参数、
world、model 和插件的 SHA-256。

## 5. P0 构建结果

严格使用独立目录：

```text
impact_p0_build/
impact_p0_install/
impact_p0_log/
impact_p0_test_log/
```

执行 `colcon build --symlink-install` 后 5 个包构建成功；C++ 转换测试 6 项、
关闭竞态回归 2 项，共 8/8 通过。用户级 setuptools 82 与 Humble 的
develop 流程不兼容，构建时使用 `PYTHONNOUSERSITE=1` 选择系统 setuptools
59.6，未改动全局 Python 环境。

## 6. 隔离边界

- 不修改 `/home/accelerate/cuadc_ws`、`/home/accelerate/ardupilot`、
  `/home/accelerate/ardupilot_gazebo`。
- P1 只读引用 ArduPilot 模型、参数和插件；每轮前后对既有 world/model
  做完整 SHA-256 比较。
- 启动前检查 TCP 5760 和 UDP 9002；端口被占用则直接退出，不清理占用者。
- 每轮使用独立 `ROS_DOMAIN_ID`、`GZ_PARTITION`、`ROS_LOG_DIR`。
- 禁止 `killall`/全局 `pkill`；只向本轮记录且验证过的进程组发送信号。
- P1 数据放在 `runs/p1_<UTC>_<PID>/`，包含 rosbag、日志、环境和依赖哈希。

## 7. 与企划目标的差距

1. P1 真飞控闭环已具备，P2 的 Mid-360 原始消息契约尚未验证。
2. 当前定位是 `daf_lio_proxy_2d`，不是 FAST-LIO2。
3. 当前建图是二维占据代理，不是 10 cm / 5 cm 三维体素地图。
4. 当前规划是二维 OAER + A* 代理，不是三维 EGO-Planner。
5. IMPACT 尚未实现；必须先完成 P2-P7 基线与统计接口，才能开始 P8。
6. Atlas 200I DK、昇腾算子、功耗和温升只能在硬件阶段验证。

## 8. 下一决策

保持“真飞控通路”和“算法代理对照组”并存。下一阶段先把 Gazebo Mid-360
统一为 `/livox/lidar` 与 `/livox/imu`，记录 PointCloud2 字段、频率、时间戳
及 TF；随后接入本项目隔离副本中的 FAST-LIO2。只有基线 ATE 和资源统计
稳定后，才加入 DAF 与 IMPACT，避免把接口故障误判为算法收益。

## 9. P1 Gate 结果

正式证据目录：`/home/accelerate/xuanqiong_x1_sim_ws/runs/p1_20260822T125115Z_388`

- 600 s 连续运行 PASS；
- ARM→TAKEOFF→30 s HOVER→LAND 在 80.034 s 内完成；
- 终态 MAVROS `connected=true`、`armed=false`；
- rosbag 共 38,231 条消息；
- 外部 Gazebo world/model 前后哈希一致；
- 退出后无 ArduCopter、MAVROS、Gazebo、rosbag 残留进程。
