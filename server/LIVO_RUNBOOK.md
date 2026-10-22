# Ubuntu 22.04 x86_64：LIVO 联合光流回放

本手册对应论文中的原生 LIVO、联合模型禁用光流、完整联合模型、移除陀螺交叉项、移除平面交叉项五组实验，以及独立的联合核心与串联基线原型。五组算法均使用 CPU；不需要 NVIDIA GPU、CUDA、显示器、RViz 或正在运行的 Gazebo。这里回放此前记录的仿真传感器，并不重新执行 AP 4.7.1 闭环飞行。AP/Gazebo 的运行说明见整个软件包的主 README。

本目录是仓库与交付包统一的服务器入口。以下命令从项目根目录执行。所有脚本支持 `--root /绝对/包路径`，不依赖 `/home/accelerate`、WSL 或 Windows 挂载点。

## 包内必须保留的文件

| 路径 | 用途 |
| --- | --- |
| `src/xq_fast_livo2/` 全目录 | 原生 19 维 LIVO、28 维联合核心、IMU/LIO/VIO 接入、配置、独立 C++ 检查、许可证与来源说明；本次规范源码为 Windows 工作区的 34 个文件 |
| `src/xq_livox_interfaces/` 全目录 | 构建所需的 ROS 2 消息接口；回放输入本身为标准 PointCloud2 |
| `scripts/build_joint_livo_isolated.sh`、`scripts/check_joint_livo_build_manifest.py` | 原样执行的独立构建与源码/依赖/二进制清单审计 |
| `scripts/run_joint_livo_replay.{sh,py}` | 原样执行的五组回放调度 |
| `scripts/replay_joint_livo_inputs.py` | 原始采集时刻、CDR 载荷、五路输入白名单与累计屏障 |
| `scripts/prepare_livo_transport.py` | 每次运行独立的 Fast DDS 配置 |
| `scripts/analyze_joint_livo_replay.py`、`scripts/analyze_livo_replay.py` | 与估计进程分离的真值评估和像素/J/K/Joseph 更新重建 |
| `scripts/prepare_joint_livo_selection.py` | 预声明的选择/丢失输入工具；不等于这些退化实验已经完成 |
| `config/research_joint_livo_replay.json`、`config/livo_dependency_patches/` 全目录 | 固定五组、工程噪声、时间预算，以及 vikit 构建导出和有界参数服务探针 |
| `server/{livo_server.py,build_livo_server.sh,run_livo_server.sh}` | 部署包装器；不改变残差、协方差、门限、输入采样或五组定义 |
| `third_party/livo/{Sophus,rpg_vikit}/`、`vendor-manifest.json` | 从固定 Git 对象导出的干净依赖源码及逐文件哈希；不携带旧编译产物 |
| `data/b1/rosbag/metadata.yaml`、`rosbag_0.db3` 至 `rosbag_5.db3` | 同一段 B1 原始记录；必须完整保留全部分卷和元数据，不裁剪/重写数据库 |
| 原 `sensor-index.json` 及 `source-files.*.json`、`source-stamps.json`、`pair-profile.json`、`prepare-result.json`、`evaluator/` | 原结果的参考证据。可统一放在 `data/b1/reference/`；里面的旧绝对路径应保留为历史记录，新服务器重新生成自己的索引 |
| `prototypes/serial_position/` 全目录 | 串联工程基线的独立原型、协议与验证入口；尚未接入五组 native replay |

原始 B1 索引在开发仓库的 `runs/software_completion_joint_input_index_20261021T074341Z_324/`。原始 bag 在 WSL 的 `experiments/results/research_multisensor/r2b3b1_20261020T104238Z_8607/rosbag/`。两者不是算法运行时必须保持的目录名。

原 bag 共 **1,503,077,896 字节（约 1.40 GiB）**，包括 26,120 字节的元数据和六个 db3。原传感器索引 12,470,749 字节。两个依赖的干净 Git 导出合计约 1.35 MB，而旧 `third_party/` 的 668 MB 包含历史构建，无需原样复制。原完整组与四个对照组的历史结果约 936 MB，可作为论文证据另存，不能替代原 bag。为新构建、五组 JSONL 日志和评估建议先预留 **10 GiB 可用空间**；这是容量建议，并非测得的上限。原构建已在 8 GiB WSL 上以单任务完成，服务器建议至少 8 GiB 内存；16 GiB 更充裕。

## 依赖

目标为 Ubuntu 22.04、x86_64、ROS 2 Humble，ROS 安装在 `/opt/ros/humble`。先按机器管理方式配置官方 ROS 2 apt 源，再安装以下依赖。安装和软件包升级应在构建前完成；原审计器会绑定整机 dpkg 软件包版本，构建后再升级依赖会使清单失效。

```bash
sudo apt-get update
sudo apt-get install -y \
  build-essential cmake git pkg-config iproute2 \
  libeigen3-dev libopencv-dev libpcl-dev libfmt-dev libboost-thread-dev \
  python3-colcon-common-extensions python3-numpy python3-yaml \
  ros-humble-ros-base ros-humble-rclcpp ros-humble-rclpy \
  ros-humble-sensor-msgs ros-humble-nav-msgs ros-humble-geometry-msgs \
  ros-humble-visualization-msgs ros-humble-std-msgs ros-humble-rosgraph-msgs \
  ros-humble-tf2-ros ros-humble-tf2-geometry-msgs \
  ros-humble-pcl-conversions ros-humble-cv-bridge ros-humble-image-transport \
  ros-humble-rosidl-default-generators ros-humble-rosidl-runtime-py \
  ros-humble-rmw-fastrtps-cpp
```

`Sophus` 必须使用提交 `de0f8d3d92bf776271e16de56d1803940ebccab9`（配置版本 1.22.10）；`rpg_vikit` 使用 `4b7abc838f5d2ca9137f70f122eaaeff9eaf0f50`。包装器从随包源码重新构建，不下载最新版本，也不把这些库安装到系统目录。`config/livo_dependency_patches/` 的显式补丁只修复 ROS 2 参数服务的有界发现/错误处理，构建包装保持上游数值相机模型。

## 一次完整重建与五组运行

```bash
python3 server/livo_server.py vendor-verify --vendor-root third_party/livo

bash server/build_livo_server.sh --root "$PWD" \
  --runtime-root "$PWD/server_runtime/livo" --jobs 1

bash server/run_livo_server.sh check --root "$PWD" \
  --runtime-root "$PWD/server_runtime/livo"

bash server/run_livo_server.sh prepare --root "$PWD" \
  --runtime-root "$PWD/server_runtime/livo" \
  --bag "$PWD/data/b1/rosbag"

bash server/run_livo_server.sh replay --root "$PWD" \
  --runtime-root "$PWD/server_runtime/livo" \
  --output "$PWD/server_runtime/livo/runs/five_modes_001"

bash server/run_livo_server.sh evaluate --root "$PWD" \
  --runtime-root "$PWD/server_runtime/livo" \
  --run "$PWD/server_runtime/livo/runs/five_modes_001" \
  --output "$PWD/server_runtime/livo/evaluations/five_modes_001"
```

`runtime-root`、输入准备输出、每次回放输出及评估输出必须使用新目录。失败目录会保留，不自动删除，也不会覆盖原始数据或旧验证证据。需要重建时换用新的 `--runtime-root`，并对后续命令使用同一路径。

包装器在 `runtime-root/workspace/` 原样复制规范 LIVO 源码、消息包、原始脚本与配置。它先仅构建 `xq_livox_interfaces`，生成明确注明“仅消息包”的 `xq_install/.xq_build_manifest.json`，再构建依赖并执行原 `build_joint_livo_isolated.sh`。原清单审计器会保护两个新建且确实为空的旧 prefix `livo_build/`、`livo_install/`，并把实际构建绑定到源码、依赖、工具链、本机软件包、链接库和二进制。这里没有伪装成原完整 Gazebo workspace 构建，也没有绕过原审计器。新的阶段清单还会拒绝规范源码变化、stage 中被改写的算法文件以及新增源文件。

新索引保存在 `runtime-root/inputs/original/`，包含仅供离线评估的 `evaluator/truth.jsonl` 与 `range-reference.jsonl`。索引会包含本机实际 bag 路径，因此不要直接运行带 `/home/accelerate` 路径的旧索引，也不要仅进行无审计的字符串替换。原准备工具以 `mode=ro&immutable=1` 打开 SQLite，并对源文件前后哈希；不向原 bag 写入任何内容。尚未关闭或带 `-wal`、`-shm`、`-journal` 旁路文件的数据库会被拒绝。

默认五组为 `native joint_no_flow full zero_gyro zero_plane`。如需先诊断完整组，可在 replay 后追加 `--modes full`；这不能替代最终五组对照。主要光流对照是 `joint_no_flow`。串联原型不是一个可选择的第六组。

回放保持原配置 `replay_rate=0.5`、每个图像端点 1 px 工程标准差、测距 0.01 m、局部斜率 0.05 和最大帧对 0.3 s，不按服务器结果调参。单组原始采集长度约 83.753 s；每组原生超时上限为 420 s，五组按顺序执行。不要因为服务器较慢而把未完成处理标为成功；若需要另设运行预算，另建并记录明确不同的实验配置，保留原失败结果。

Fast DDS 配置从本机 `ip -j -4 route get 1.1.1.1` 读取默认 IPv4 接口；这条命令不发送网络请求，也不需要互联网，但机器必须有有效的非 loopback IPv4 路由。每次回放采用独立 ROS domain 和运行局部传输配置，检查参数服务与输入话题。仅有 loopback 的隔离网络 namespace 目前不属于此原 transport helper 的支持范围；不要把网络发现失败解释成算法退化。一次只运行一套回放，或用不同的 `IMPACT_JOINT_ROS_DOMAIN` 显式隔离。

## 完成条件与输出

构建完成应有 `server-build-result.json` 的 `PASS`、原 `joint_livo_install/.impact-joint-build-manifest.json`，以及依赖 consumer 的 delayed/missing/wrong-type 三项通过日志。它们证明构建身份与依赖可用，不证明精度或硬件性能。

回放每组都有 `publisher-result.json`、`processing-result.json`、`run-result.json`、`joint-updates.jsonl`、`visual-updates.jsonl` 与 `odometry.jsonl`。完整五组要求套件 `suite-result.json` 为 `PASS`，每组五路接收计数/尾时间戳/最终屏障均经核对。发布进程退出、发布计数正确或仅有里程计消息均不代表估计器完成处理。

评估输出 `report.json` 与各组轨迹 JSON。离线评估器重算原始像素预测、28 维雅可比、双端点噪声、增益、实际状态注入及两次 SO(3) 重置后的 Joseph 协方差；真值只在估计进程退出后读取，不会成为估计器输入。参考记录有 16,586 条 IMU、830 帧激光雷达、各 1,270 帧前/下视图像与 4,188 条测距。旧论文结果为 37 次实际完整模型光流更新；服务器重新运行应报告实际结果、全部失败和平台差异，不能只复制该数字。

## 不需要 ROS 的两个独立核心检查

联合核心只依赖 C++17 和 Eigen。下面创建新的检查目录：

```bash
livo_core_out="server_runtime/joint_core_$(date -u +%Y%m%dT%H%M%SZ)"
mkdir -p "$livo_core_out"
g++ -std=c++17 -O0 -g0 -Wall -Wextra -Werror \
  -I/usr/include/eigen3 -Isrc/xq_fast_livo2/include \
  src/xq_fast_livo2/test/test_research_joint_flow.cpp \
  -o "$livo_core_out/test_research_joint_flow"
"$livo_core_out/test_research_joint_flow" | tee "$livo_core_out/test.log"

python3 prototypes/serial_position/validate_serial_prototype.py \
  --joint-header src/xq_fast_livo2/include/research_joint_flow.h \
  --output-dir "server_runtime/serial_$(date -u +%Y%m%dT%H%M%SZ)"
```

开发仓库中的串联原型位于 `runs/software_completion_serial_prototype/`；交付包迁到 `prototypes/serial_position/` 时必须保持原型四个文件字节不变。其验证器输出协议/源码前后哈希、编译命令、检查结果和二进制哈希。两个独立核心检查不建立 native replay、AP 闭环、硬件性能或统计优越性结论。

## 打包者：从现有固定 Git 对象导出依赖

不应将打过补丁的依赖工作树当成干净上游源再次打补丁。以下入口读取固定提交的 Git 对象并生成逐文件清单，不读取工作树修改：

```bash
python3 server/livo_server.py vendor-export \
  --sophus-repository /path/to/pinned/Sophus \
  --vikit-repository /path/to/pinned/rpg_vikit \
  --output /path/to/stage/third_party/livo
```

随包携带原许可证、来源说明与该清单。迁移后重新构建，不搬运 WSL 的 `xq_install`、`livo_deps`、`joint_livo_install` 或绝对路径构建清单。论文现有证据仍限于单段记录的局部平面、固定地图及所声明的相关项模型，不能由可移植部署推导为多随机种子退化实验、全局一致性或 AP 4.7.1 新估计器闭环验证已经完成。
