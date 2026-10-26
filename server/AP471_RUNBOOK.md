# AP 4.7.1 服务器准备与未解锁验证

适用环境为 Ubuntu 22.04、x86_64、系统 Python 3.10、ROS 2 Humble、Gazebo Harmonic（gz-sim8）。本文件只覆盖 AP/MAVROS/Gazebo 接口；论文新融合算法、数据回放和统计实验按总包 README 执行。

AP 固定官方 `Copter-4.7.1`，提交 `dbe792162d06cab66c3475fd5556bf7a120f119e`。历史 ArduCopter 4.5.7 飞行数据不会因服务器重新构建而变成 4.7.1 的飞行证据。新融合估计器尚未在 AP4.7.1 已解锁闭环中联合验证。这里的所有包装器均没有 ARM、TAKEOFF 或任务执行入口。

## 1. 总包布局

```text
ROOT/
  src/ scripts/ config/ docs/         # 当前项目算法与配置，只读输入
  server/                            # 服务器入口与验证记录
  third_party/
    ardupilot/                       # 精确提交与递归子模块，独立 Git 元数据
    ap471_export_manifest.json        # 导出时的真实提交及已跟踪源码 SHA
    ardupilot_gazebo/                 # 本机实际插件源码快照，含模型和许可证
    livo/                            # 由主 README 管理的新融合第三方库
  data/b1/rosbag/                     # 原始采集回放，按主 README
  prototypes/serial_position/         # 尚未接入实验组的串联原型
  server_runtime/                    # 本服务器新构建/运行结果，不能替代旧证据
```

`ap471_server.py --root ROOT` 默认仅向 `ROOT/server_runtime/` 创建新目录；也可用全局 `--workspace /独立路径`。原源码、许可证和历史 evidence 不修改。每次失败保留原目录和 `status.json`，不会覆盖上一次 PASS。

## 2. 外部源与许可证

必须随包携带 AP 完整源树、所有递归子模块及各自许可证，顶层许可证为 `COPYING.txt`。不能仅复制历史 `arducopter` ELF，也不能把工作树中指向旧机器绝对路径的 `.git` 文件打包。

制包端用以下工具离线导出真实现有提交：

```bash
PYTHONDONTWRITEBYTECODE=1 python3 server/ap471_export_source.py \
  --source /原机器/独立AP471源码/source \
  --destination /STAGE/third_party/ardupilot \
  --manifest /STAGE/third_party/ap471_export_manifest.json
```

工具逐仓库执行本地 `git clone --no-hardlinks --no-checkout`，检出真实主仓/子模块提交，建立独立 `.git` 目录，验证 common-dir 不逃逸、无借用对象、`git fsck` 和递归已跟踪源码 SHA。原仓库只读。浅克隆会保留其真实 shallow 标志，不伪称包含全历史。只要原有子模块完整，此步骤不访问网络。失败的目标目录须保留并使用新目标重试。

历史 Gazebo 插件来自 `/home/accelerate/ardupilot_gazebo`，其源码目录**没有 `.git`**。因此提交身份必须记录为 `UNKNOWN_SOURCE_SNAPSHOT`，不能用今天官方 `main` 的提交冒充。随包复制这个实际目录的源码、`cmake/`、`include/`、`src/`、`config/`、`hooks/`、`models/`、`worlds/`、`tests/`、`CMakeLists.txt`、`package.xml`、`README.md` 和 `LICENSE.md`；保留其他源文件。排除根 `build/`、`.git`、`__pycache__`、`.pytest_cache` 和 `*.pyc`，不携带旧 `.so`、CMakeCache 或 Makefile。审计快照为 94 文件、85,926,135 字节，文件集及 SHA 见 `ap471_portability.json`。`LICENSE.md` 为 LGPLv3；Iris 模型的作者与 PX4 来源说明也必须保留。

当前 AP 场景的资源链为：

```text
src/xq_gz_assets/worlds/xq_r2b3_range.sdf
  -> model://xq_iris_multisensor_range_ardupilot
  -> model://iris_with_ardupilot          (第三方插件 models/)
  -> model://iris_with_standoffs         (第三方插件 models/)
  -> iris_with_standoffs/meshes/*.stl, *.dae
```

当前资源链不加载 `/home/accelerate/cuadc_ws` 或旧 `xuanqiong_x1_sim_ws`。这些路径出现在旧 README 的来源说明中，不应成为服务器运行依赖。旧 `uav_slam_sim` 的 `package.xml` 声明 LGPL-3.0-only，但该快照未找到单独 LICENSE 文件；不要将整个旧工作区不加核查地复制为本场景依赖。根包中现有 xq 源码及其来源说明仍需保留。

## 3. 系统与 Python 依赖

需要预先安装 ROS 2 Humble 与 OSRF Gazebo Harmonic 软件源。Harmonic 的 Ubuntu 22.04 二进制安装方式见 [Gazebo 官方安装文档](https://gazebosim.org/docs/harmonic/install_ubuntu/)。不要将 Humble 默认的 Fortress 或 Gazebo Classic 与本项目 `gz-sim8/gz-msgs10/gz-transport13` 链接目标混用。

在已配置的软件源上，所需包族为：

```bash
sudo apt-get install build-essential git cmake pkg-config ccache \
  python3-dev python3-venv python3-pip python3-numpy python3-opencv python3-pytest \
  python3-colcon-common-extensions python3-rosdep geographiclib-tools \
  ros-humble-ros-base ros-humble-mavros ros-humble-mavros-extras \
  ros-humble-mavros-msgs ros-humble-tf2-ros ros-humble-geographic-msgs \
  gz-harmonic libgz-sim8-dev libgz-msgs10-dev libgz-transport13-dev rapidjson-dev \
  libopencv-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl
```

这是依赖包族列表，不能代替精确版本锁定。实际历史接口验证的版本为 MAVROS `2.14.0`（主包 `2.14.0-1jammy.20260422.083708`）、gz-sim8 `8.13.0-1~jammy`、gz-msgs10 `10.3.2-1~jammy`、gz-transport13 `13.5.0-1~jammy`。用 `apt-cache policy` 确认可获得的版本；需要同版环境时应使用这些版本对应的仓库快照或离线 `.deb`。不要静默升级后沿用旧版验证结论。完整包版本由每台服务器重新审计记录。

插件依赖与构建方式依据 [ArduPilot 官方插件说明](https://github.com/ArduPilot/ardupilot_gazebo/blob/main/README.md)。此链接用于依赖接口说明；随包的实际插件身份始终以快照 SHA 为准。

MAVROS 即使关闭 GNSS，也仍需 GeographicLib geoid 数据用于坐标/高度接口。按 [MAVROS 官方安装说明](https://github.com/mavlink/mavros/blob/ros2/docs/installation.md) 安装其数据集，并确认 `/usr/share/GeographicLib/geoids/egm96-5.pgm` 存在。例如在已 source Humble 的环境中，由管理员运行安装包提供的 `install_geographiclib_datasets.sh`；包装器不会自动 sudo 或修改系统。

AP 编译使用独立 venv，固定 `pexpect==4.9.0`、`ptyprocess==0.7.0`、`empy==3.3.4`、`future==1.0.0`、`lxml==6.1.1`、`pymavlink==2.4.49`、`fastcrc==0.3.6`、`pyserial==3.5`。`prepare --wheelhouse /已预下载目录` 可离线安装这些 wheel；完整离线制包还需携带其传递依赖 wheel 和许可证。不要将这套 venv 混入 ROS 的 Python ABI。

## 4. 新服务器构建与固件 smoke

```bash
ROOT=/srv/impact
python3 "$ROOT/server/ap471_server.py" --root "$ROOT" audit
python3 "$ROOT/server/ap471_server.py" --root "$ROOT" prepare --jobs 2
```

`prepare` 自动使用 `ROOT/third_party/ardupilot`，先复制到本次可写源快照再编译；若该目录不存在才联网克隆官方固定 tag 并校验 commit。也可显式 `prepare --source /完整离线AP源树`。它保存编译器、venv freeze、源文件集、递归子模块、ELF SHA 与独立 build 路径。新 ELF 的 SHA 应在本服务器重新生成，不能要求不同路径/编译环境恰好复现历史 ELF 哈希。

从最后输出读取本次准备目录，将其填入后续命令：

```bash
PREP=/srv/impact/server_runtime/results/ap471_prepare_本次标识
python3 "$ROOT/server/ap471_server.py" --root "$ROOT" smoke \
  --preparation "$PREP" --instance 20 --base-port 5760
```

此 smoke 使用 AP 内置 `quad` 动力学，只读取版本、完整参数表并观察未解锁心跳；复用原始冻结观察器的允许发送集合，只有 HEARTBEAT、PARAM_REQUEST_LIST 和请求 AUTOPILOT_VERSION 的 COMMAND_LONG(512)。不启动 ROS/Gazebo，不写飞行参数、不切模式、不 ARM。参数通过本次新 SITL 的 defaults 文件加载，保留原生解锁检查 `ARMING_SKIPCHK=0`。17 个必需值见随包 `config/research_ap471_gate.json`，其中 `VISO_TYPE=1`、`PLND_ENABLED=0`、`GPS1_TYPE=GPS2_TYPE=0`、`SIM_GPS1_ENABLE=SIM_GPS2_ENABLE=0`。从 4.5.7 迁移时不能沿用 `GPS_TYPE` 或已移除的 `ARMING_CHECK` 参数名称。

该固件按顺序解析参数：`-I` 会对仍为默认5760的端口加实例偏移。包装器明确先传 `-I`、再传 `--base-port`，使显式端口始终为绝对串口基础端口；观察器不再叠加 `10*instance`。例如 `--base-port 18070 --instance 37` 的观察器连接 TCP18070，并检查18072/18073的占用。旧 runner 的非零 instance 端口推算不可直接照搬；新包装器已按实际 SITL 监听行为修正。

smoke 结果的 `PASS` 仅表示此次固件身份、完整回读、未解锁观测和正常退出；不表示 ExternalNav 接收、EKF 融合、相机渲染或飞行闭环通过。异常终止升级也会保留并报告。

## 5. 独立 autonomy 与插件构建

先按主 README 在新服务器构建全包 underlay，包含本地 `xq_sim_interfaces`、`quadrotor_msgs`、`traj_utils` 等接口。AP autonomy 包装器不会凭空生成这些上游接口。

```bash
# Replace BASE_BUILD with the successful build_<UTC>_<PID> directory printed by build_workspace.sh.
BASE_BUILD=/absolute/path/impact-runs/base/build_本次标识
UNDERLAY="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["install"])' "$BASE_BUILD/build-result.json")"
python3 "$ROOT/server/ap471_server.py" --root "$ROOT" build-autonomy --underlay "$UNDERLAY"
python3 "$ROOT/server/ap471_server.py" --root "$ROOT" build-plugin \
  --plugin-source "$ROOT/third_party/ardupilot_gazebo" --jobs 2
```

autonomy 构建从 canonical 源复制本次冻结快照，在全新 build/install/log 前缀执行 `colcon --packages-select xq_autonomy` 与该快照的测试。它核对输入和只读 underlay 前后文件集/SHA、所有已安装 Python 模块对源 SHA 的对应，并生成新 install 下的 `.xq_build_manifest.json`。该 manifest 同时保存完整 install 文件集（自身除外）及 underlay 文件集，不复用旧机器的绝对 `installed_path`。

插件构建只在新快照与 build 目录运行 `GZ_VERSION=harmonic cmake ...`。输出 `plugin-build-manifest.json` 保存实际源树/二进制 SHA；无 Git 的来源明确保留 UNKNOWN 身份。不会覆盖随包第三方树或旧 `.so`。

## 6. 无 GUI 的 Gazebo 准备

OGRE2 相机/LiDAR 仍需可工作的 EGL 渲染环境；`-s` 本身并不能证明传感器可渲染。Gazebo 官方支持 [EGL headless rendering](https://gazebosim.org/api/sim/8/headless_rendering.html)，运行参数为 `--headless-rendering`，无需 WSL 的 D3D12 检查。无 GPU 的软件渲染只可用于功能诊断，不能替代 Orin NX 或服务器实时性能证据。

将插件构建结果的 source/build 路径填入：

```bash
PLUGIN_SOURCE=/srv/impact/server_runtime/sources/ap471_gazebo_本次标识
PLUGIN_BUILD=/srv/impact/server_runtime/build/ap471_gazebo_本次标识
eval "$(python3 "$ROOT/server/ap471_server.py" --root "$ROOT" headless-env \
  --plugin-source "$PLUGIN_SOURCE" --plugin-build "$PLUGIN_BUILD" \
  --assets "$ROOT/src/xq_gz_assets")"
gz sim -s -r --headless-rendering "$ROOT/src/xq_gz_assets/worlds/xq_r2b3_range.sdf"
```

以上最后一行只启动 Gazebo；JSON lockstep 场景通常还需要 AP 对端才推进。不要将等待对端误认作算法死锁，也不要将模型成功加载当作相机正常发布。使用动态障碍物的 P12/P15 场景还需将本次 `xq_gz_bridge` 插件安装目录追加到 `GZ_SIM_SYSTEM_PLUGIN_PATH`。

可先冻结不执行的完整 JSON/MAVROS/Gazebo 命令计划：

```bash
python3 "$ROOT/server/ap471_server.py" --root "$ROOT" plan-gazebo \
  --preparation "$PREP" --plugin-source "$PLUGIN_SOURCE" \
  --plugin-build "$PLUGIN_BUILD" --underlay "$UNDERLAY" --ros-domain-id 120
```

输出 `launch-plan.json` 明确标注 `NOT_EXECUTED`，包含 AP JSON、未解锁 wire guard、MAVROS、Gazebo headless 与 LiDAR/IMU bridge 的参数数组、资源环境和依赖 SHA。必须由 wire guard 先连接 SITL，再让 MAVROS 连接 guard；不得绕过 guard 直接连接 SITL。计划不启动估计器、mission 或 ARM，生成计划 PASS 不等于执行 PASS。Iris 原模型固定使用 UDP9002，AP JSON 使用 UDP9003/9002；TCP 使用6760–6763。不同 DDS domain/GZ partition 不会隔离同一 UDP 端口，不得并发启动两个未改端口的世界。

## 7. 尚需在目标服务器完成的验证

原 `run_ap471_extnav_probe.py` 及其基类绑定旧路径、433 文件保护清单和 WSL NVIDIA renderer 字符串，**不能直接作为便携服务器 live runner**。保留原脚本作为历史来源，新增计划不会通过删检查或改旧 PASS 来假装完成迁移。目标服务器仍须运行并记录 EGL 相机/点云采样、原始采集时间顺序、MAVROS exact endpoint、完整 wire ODOMETRY→FCU VISP/VISV 对齐、停源撤权/重新预热、源/安装 SHA 与所有已拥有进程组清理。新融合估计器的 AP4.7.1 真实闭环、全负载启动缺口/长中断、硬件时序与 Orin 性能均未因制包而完成。

已有 WSL 证据为 4.7.1 未解锁接口的 557/585 帧完整匹配；Gazebo 关闭升级另有 ISSUES。它们可以随论文成果保留，不能作为这台新服务器已经通过的结果。服务器每一步都应保留自己的 FAIL/PASS、时间、文件集和 SHA。

## 8. 本次便携包装验证记录

本包已在现有 WSL2 Ubuntu 22.04/x86_64 中，将源码迁移到全新 Linux stage，并使用独立运行前缀验证；这不是一台新配置的远程服务器。完整记录见 `server/AP471_SERVER_VALIDATION.json`，原始文件在 `provenance/ap471_server_validation_20261022/`，其 `evidence-files.json` 保存完整文件集与 SHA。记录包含失败尝试，不以重试覆盖。

| 实际执行项目 | 结果与边界 |
| --- | --- |
| AP 离线源导出 | 22 个自包含 Git 仓库、24,682 个已跟踪源码对象、2 个保留的相对符号链接；原始源码 SHA 未变 |
| 新 AP 构建 | 官方 4.7.1 固定提交，jobs=2；ELF SHA `cfdb02801c889f3fce873e9bc206895c2d1b537afed5d78db2e2315bc026fabe` |
| 无解锁 smoke | 显式 TCP18070/instance37，以及最终代码 TCP5760/instance83，均完整回读 1347/1347 参数，17/17 必需值符合，观测27个未解锁心跳，固件版本及提交前缀正确 |
| smoke 关闭 | 两次均由本次拥有的进程组接收 SIGINT 正常结束，无升级信号；输入依赖前后 SHA 相同 |
| 新插件构建 | jobs=1；94个源文件与审计快照一致，Git 身份仍为 UNKNOWN；新 `.so` SHA `19556a4933d1b1ebfa63c97de0f4df5e3d8eb3badccc7af7e7013aadd3d66c9b` |
| headless 环境导出 | 已执行并保存 shell 导出内容；未启动 Gazebo、相机/EGL 或 MAVROS |

第一次 AP 准备因包装器误排除所有名为 `build` 的目录，遗漏了 ChibiOS/CrashDebug 中合法的已跟踪源目录，在编译前被源完整性检查拒绝；修正为仅排除根构建目录后，在新路径重试通过。第一次 smoke 因显式基础端口被重复叠加 instance 偏移而连接超时；修正绝对端口和参数顺序后，两种端口边界均实际通过。两次失败的目录和其当时的包装器源码均保留。

本轮未安装或升级系统包、未修改系统 Python；AP Python 依赖仅安装于独立 venv。并行大型编译期间出现过 WSL 新进程启动超时及一次旧工作目录挂载警告，已有 AP 编译会话继续完成；没有重启 WSL、调整主机时钟或终止其他任务。此环境事件单独记录，不能据此判断算法优劣。

本轮未执行包装器的 `build-autonomy` 分支或 Gazebo launch plan，也未执行新的实时 ExternalNav/融合/飞行任务。总包其他算法构建与回放结果应查其对应验证记录；不能用这里的固件 smoke PASS 替代。
