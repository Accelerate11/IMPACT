# 无界面主仿真与完整基础 ROS 包构建

这两个入口只负责服务器部署，不修改算法。目标是 Ubuntu 22.04 / x86_64，ROS 2 Humble、Gazebo Harmonic 8。安装系统依赖由主 README/安装脚本负责；本入口不会 apt 安装或升级依赖。

## 新构建

```bash
bash server/build_workspace.sh --root "$PWD" \
  --runtime-root /absolute/path/impact_runtime/base --jobs 1
```

`--root` 是解压后带 src/scripts/config 的包根；runtime 必须在包根之外。每次新建 `build_<UTC>_<PID>`，不覆盖失败、不复用旧 install，不 source 原研究工作区。直接从包内 canonical src 构建 **14 个包**；`xq_fast_livo2` 由独立 Sophus/vikit/LIVO 入口构建，因此明确排除，其他 14 个包不能以旧 xq_install 补齐。

使用单 executor、默认单编译线程、Release、merge-install。首轮 `BUILD_TESTING=OFF`；这是构建，不等于测试全部完成。构建记录包括源文件集/SHA 前后、实际命令、环境、colcon 完整日志、选中/安装包集合与结果。失败目录均保留。若系统缺依赖，先在日志中确认，并在没有其他绑定 dpkg 身份的实验运行时按 README 安装；不要在正式回放中途 apt。

## 30 秒观测 smoke

```bash
python3 server/run_headless_sim.py --root "$PWD" \
  --runtime-root /absolute/path/impact_runtime/base \
  --duration 30 --renderer software
```

默认选择该 runtime 下最新 build，要求 PASS 且 source set/SHA 仍匹配；不会回选旧 PASS。也可显式传 `--build-attempt /absolute/path/build_...`。

入口直接执行：

```text
ros2 launch xq_sim_bringup xq_sil.launch.py
  headless:=true start_metrics:=false start_network_relay:=false
  inject_faults:=false start_stack:=true
  run_dir:=<new-run> ros_domain_id:=<unused> gz_partition:=<unique>
```

启动使用 /opt/ros/humble 加本次 install/local_setup.bash。没有旧 overlay。脚本扫描可见进程的 ROS_DOMAIN_ID，在 Linux 安全域 32..101 中选择未占用值，可用 `--ros-domain-id` 显式指定未占用值。Gazebo partition、ROS 日志和 XDG cache/config 均为独占新路径；不启动 GUI、不依赖旧 WSL GUI/D3D12 配置，也不改系统 HOME。

软件渲染模式设置 EGL surfaceless、Mesa llvmpipe 与 LIBGL_ALWAYS_SOFTWARE，不启动 GUI。仍需真实可用的渲染后端；headless 不会消除 gpu_lidar 的渲染需求。`--renderer auto` 允许使用服务器已有后端，需单独记录/验证。

观察器先等待 clock、LiDAR、IMU 和代理 odometry 出现，然后开始完整的 30 秒墙钟窗口。默认启动等待上限 60 秒，可显式调整 `--startup-timeout`。检查范围：

- clock、非空点云、有限 IMU/代理位置及实际推进的原始传感器戳；
- LiDAR 至少10条、IMU至少100条、代理odometry至少10条，map与health有观测；这只是smoke最低门，不是宣称实时频率；
- 算法代理在图中存在，未订阅 /xq/eval，未出现 MAVROS/AP mission/execution 节点；
- 相关ROS包只来自本次新install；
- 源码与第三方模型/world文件集及SHA前后一致。

每条观测的采集戳和接收monotonic_ns写入JSONL，统计与ROS graph写入observer-result.json，最终result.json独立保存。只清理本次捕获、/proc start identity未改变的子进程；不做全局pkill。需要SIGTERM/SIGKILL时原样记录，不将强制清理描述为优雅收尾。

## 这项测试不代表什么

默认场景是自带基础几何的 xq_indoor_office，机体使用无重力 VelocityControl 算法载体。算法路径是历史2D定位/地图/Frontier/规划代理；没有启动FAST-LIO或FAST-LIVO2映射进程，没有相机/range桥，没有AP或MAVROS，没有ARM/TAKEOFF请求。

因此PASS只证明本服务器上的新构建基础包能运行Gazebo+LiDAR/IMU+代理链。它不能替代新28维raw-flow回放、多传感器采集、EGO实际轨迹执行、AP4.7.1任务、硬件验证或统计性能比较。外部 ArduPilot Gazebo snapshot 模型保留在包中且按只读来源核验，但该自包含场景不加载AP机体/插件。

## 本次验证记录

实际构建与测试的唯一目录、状态及计数在 `simulation_validation.json` 记录；轻量证据在 `validation/base_20261021T165352Z_426/`。本次新构建14包PASS；新install下colcon pytest通过773项（xq_autonomy769、impact_fault_injection4），C++单元测试未构建，不能混称全包测试覆盖。

本次headless完整观察30.00497秒，clock11906条、LiDAR189帧、IMU5953条、代理odom588条、map298条、health472条。点云抽样累计12096个有限非零XYZ点；LiDAR/IMU非递增戳和无效payload均为0。29个安装资源与包内源逐项SHA一致，438个src及外部模型/world文件集与SHA前后相同。所有本轮进程均SIGINT正常退出，无TERM/KILL升级、无遗留owned进程。

LiDAR观察速率约6.30Hz，不是10Hz实时性能通过，也不证明全部生成点云均送达。health是多模块同戳消息，不能把health跨模块的重复戳当成传感器重复采集。

首次预构建attempt因colcon list从/尝试创建log目录而被拒绝，未进入编译；其原日志和无result的INCOMPLETE状态保留。新launcher已显式指定list-log/cwd到独占runtime。没有修改算法来获得PASS。服务器安装差异和新机器性能仍须重跑观察。

