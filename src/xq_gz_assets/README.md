# xq_gz_assets

`xq_gz_assets` 是玄穹 X1 的独立 Gazebo Harmonic（`gz sim 8`）资产包。它只安装本包的
world、model 和资源路径环境钩子，不读取、不 include，也不改写下列已有项目：

- `/home/accelerate/cuadc_ws/src/uav_slam_sim`
- `/home/accelerate/ardupilot_gazebo`

所有几何均由 SDF 基础体组成，不依赖 Fuel、网络下载、外部 mesh 或其他项目地图。

## 资产与接口

- world：`worlds/xq_indoor_office.sdf`，SDF 1.10，world 名 `xq_indoor_office`。
- model：`models/xq_iris_mid360`，world 中实例名为 `xq_agent_01`。
- 场景：20 m × 16 m 有顶办公室，包含环形走廊、非对称隔墙、柱、桌柜、箱体和一个沿南侧走廊以 -0.10 m/s 确定运动的可控动态障碍。
- UAV：无重力的 6-DoF 算法测试载体；它用于 SLAM/规划闭环，不声称复现真实飞行动力学。
- GPU lidar：720 × 32、10 Hz、约 23 万点/秒、360° 水平视场、59° 垂直视场、40 m 量程。
- IMU：200 Hz，带确定配置的高斯测量噪声。
- 真值 odometry：50 Hz，只允许接入评测链，禁止算法节点订阅。

| 用途 | Gazebo Transport 接口 |
|---|---|
| 原始规则扫描 | `/xq/lidar` (`gz.msgs.LaserScan`) |
| 点云 | `/xq/lidar/points` (`gz.msgs.PointCloudPacked`) |
| IMU | `/xq/imu` |
| UAV 速度命令 | `/model/xq_agent_01/cmd_vel` (`gz.msgs.Twist`) |
| 评测真值 | `/model/xq_agent_01/odometry` (`gz.msgs.Odometry`) |
| 动态障碍命令 | `/model/xq_dynamic_obstacle/cmd_vel` (`gz.msgs.Twist`) |

桥接后的契约帧名为 `xq_world`、`xq_base_link`、`xq_mid360_link` 和 `xq_imu_link`；Gazebo
传感器原始消息保留默认 scoped entity frame，由桥接层在 ROS 输出时规范化。机器可读契约位于
`config/xq_asset_manifest.yaml`。

## 在 WSL 的独立副本中校验

Windows 的 `D:\jbgs` 是源码主副本。若 `/mnt/d` 当前不可用，不要重挂载；从 PowerShell 将本包复制到
`\\wsl.localhost\Ubuntu-22.04\home\accelerate\xuanqiong_x1_sim_ws\src\xq_gz_assets`
后，在 WSL ext4 文件系统中运行：

```bash
XQ_ASSETS=/home/accelerate/xuanqiong_x1_sim_ws/src/xq_gz_assets
export SDF_PATH="$XQ_ASSETS/models"
export GZ_SIM_RESOURCE_PATH="$XQ_ASSETS/models"

gz sdf -k "$XQ_ASSETS/models/xq_iris_mid360/model.sdf"
gz sdf -k "$XQ_ASSETS/worlds/xq_indoor_office.sdf"
```

使用独立 Transport 分区启动无界面仿真，不会发现或干扰其他 Gazebo 实例：

```bash
export GZ_PARTITION="xq_office_${USER}_$$"

# 当前 WSLg / D3D 渲染后端若在 GPU lidar 初始化时崩溃，使用已验证的软件渲染：
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe
export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe
export EGL_PLATFORM=surfaceless

timeout 20s gz sim -s -r --headless-rendering --iterations 800 \
  "$XQ_ASSETS/worlds/xq_indoor_office.sdf"
```

快速查看接口：

```bash
gz topic -l | grep -E '^/xq/|^/model/xq_'
gz topic -e -t /xq/imu
gz topic -e -t /xq/lidar/points
```

发送测试速度（先使用小速度并确认路径净空）：

```bash
gz topic -t /model/xq_agent_01/cmd_vel -m gz.msgs.Twist \
  -p 'linear: {x: 0.20}, angular: {z: 0.10}'
```

## 可选 ament 安装

只构建本包，避免触发同一工作区中尚未就绪的算法包：

```bash
source /opt/ros/humble/setup.bash
cd /home/accelerate/xuanqiong_x1_sim_ws
colcon build --packages-select xq_gz_assets --symlink-install
source install/setup.bash
gz sim -s -r --headless-rendering \
  "$(ros2 pkg prefix xq_gz_assets)/share/xq_gz_assets/worlds/xq_indoor_office.sdf"
```

## 隔离约束

1. world 只 include `model://xq_iris_mid360`，该模型随本包安装。
2. 不把已有工作区加入测试用 `GZ_SIM_RESOURCE_PATH`。
3. 每次自动测试设置新的 `GZ_PARTITION`。
4. 真值话题在 ROS 侧应仅桥接到 `/xq/eval/agent_01/ground_truth`，并由启动审计确认算法订阅图中不存在该命名空间。
5. 当前模型是算法载体；ArduPilot SITL/HIL 应由单独 bringup 包按需接入，不能用本模型的 VelocityControl 结果代替飞控验证。
