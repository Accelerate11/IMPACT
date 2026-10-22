# IMPACT 服务器包：算法与仿真覆盖审计

本说明配套 Ubuntu 22.04 / x86_64 服务器包。清单以当前 Windows 主仓库的 `src/`、`scripts/`、`config/` 为算法来源，记录 597 个普通文件、5,188,195 字节、10 个顶层组件和 15 个实际 ROS 包。所有逐文件 SHA256、依赖、SDF 资产和 WSL 差异见 `coverage_manifest.json`。

这是静态覆盖与依赖审计，不是“所有历史入口均已在新服务器运行通过”的证明。新环境构建、数值验证、Gazebo 性能及 AP 执行结果须由各自新日志支持。本文未启动仿真、AP 或 ROS 节点，未构建、安装或打开 SQLite，未修改算法和旧证据。

为兼容 Git 在 Linux 的 LF 签出，每个597源文件及4个串联原型源均保留原 `bytes/sha256`，另明确记录 `linux_lf_bytes/linux_lf_sha256`。后者只由严格UTF-8文本的CRLF→LF得到；二进制原样。校验器直接对候选原始字节哈希，只接受这两组明确身份，不做宽泛归一化。完整LF导出通过与同尺寸单字节篡改拒绝的独立夹具见 `linux_lf_identity_validation.json`；原601源字节保持不变。历史校准副本也记录两种明确身份。

## 目录与来源

| 内容 | 包内位置 | 原始来源 / 要点 |
|---|---|---|
| 全部主仓库算法 | `src/`, `scripts/`, `config/`, `docs/` | `D:/CODEX/impact`；不要以较旧的 WSL 主副本替代 |
| 新便携入口和本审计 | `server/` | `packaging/server/` 的副本 |
| FAST-LIVO2 外部库 | `third_party/livo/{Sophus,rpg_vikit}` | 库来源、补丁与构建另有清单 |
| AP 4.7.1 | `third_party/ardupilot/` | 精确源码与构建身份由 AP 打包清单约束 |
| Gazebo AP 插件与机体 | `third_party/ardupilot_gazebo/` | 必须包括 models/worlds/许可证及插件源 |
| 五传感器原始输入 | `data/b1/rosbag/` | 6 个 DB3 + metadata，原 acquisition/CDR 不变 |
| 独立串联原型 | `prototypes/serial_position/` | 原来位于 `runs/software_completion_serial_prototype/`，不在 src |
| 论文 PDF 与源包 | `papers/` | 中英文 LaTeX/PDF，全部推导与历史规划公式 |
| 新构建与运行 | `server_runtime/` / 新结果目录 | 不直接信任旧 WSL 安装前缀或动态库 |

顶层 `xq_ego_planner` 内含 `bspline_opt`、`ego_planner`、`path_searching`、`plan_env`、`quadrotor_msgs`、`traj_utils` 六个包。因此“十个源码目录”与“十五个 colcon 包”是不同计数。其他九个顶层组件为 `impact_fault_injection`、`xq_autonomy`、`xq_fast_lio`、`xq_fast_livo2`、`xq_gz_assets`、`xq_gz_bridge`、`xq_livox_interfaces`、`xq_sim_bringup`、`xq_sim_interfaces`。

## 算法覆盖与证据边界

下表对应机器清单的 21 个组；每组的全部必需路径、源身份和论文 label 均在 JSON 中。

| 算法/机制 | 主要实现 | 当前可支持的范围 |
|---|---|---|
| FAST-LIO2 基线 | `src/xq_fast_lio/` | 历史 LiDAR/IMU 定位及旧任务层 |
| FAST-LIVO2 原生估计 | `src/xq_fast_livo2/src/{LIVMapper,IMU_Processing,vio,voxel_map}.*` | 原生 19 维状态；保留固定地图/独立原生测量近似 |
| 非线性 28 维联合状态 | `include/research_joint_flow.h`、`src/research_joint_integration.cpp` | 实际原生状态及协方差更新；完整单段回放 full 有 37 次真实光流更新 |
| 五流不可变回放与评估 | `replay_joint_livo_inputs.py`、`run_joint_livo_replay.py`、`analyze_joint_livo_replay.py` | 同一原始输入的五方法比较，采集桶累计 barrier，不把 DDS 到达当采集时间 |
| 共享尺度不确定性 | `scale_uncertainty.py`、`run_shared_scale_study.py` | 合成统计机制，点估计相同，非硬件/图像融合精度试验 |
| 单窗相关 Gaussian | `correlated_update.py`、`run_correlated_flow_study.py` | 声明线性夹具和独立审计，非实时估计器 |
| 跨窗 nuisance/clone | `joint_state.py`、`run_joint_window_study.py` | 19+1+6 线性模型；不能和非线性 19+6+3 混用 |
| 两视图/有序 gyro 几何 | `two_view_geometry.py`、`flow_geometry.py` | 几何/时序夹具与早期观测前置条件 |
| 下视跟踪与 MicoLink | `downward_flow*.py`、`micolink.py`、`run_micolink_pty_gate.py` | 渲染观测、协议和 PTY 软件验证；无真实光流硬件标定 |
| 传感器契约与元数据故障 | `multisensor_contract.py`、`run_multisensor_fault_matrix.py` | 戳、外参、尺度、来源检查；非物理低纹理/退化性能 |
| 多传感器 Gazebo 采集 | range 模型/world、camera/range bridge | 原 B1 输入采集与测距白噪声检查，不能取消原 AP 失败 |
| 方向信息与膨胀 | `integrity.py` | 旧 P6 工程模型，不等同新联合估计器的校准 PL |
| 经验校准 | `calibrate_p7.py`、P7 collector | 旧估计器 train-only 因子与 held-out 覆盖，不能直接迁移 |
| AL/PL 硬裕度 | `alert_limit.py`、`integrity_margin.py`、P9 gate | 采样点地图测试，非连续扫掠体积或物理制动证明 |
| 最小激励/信息预测 | `minimum_excitation.py`、`surfel_map.py`、P10 | 离散候选与启发式预测，非全局最优控制 |
| 地图收益/碰撞/能耗及选择 | `candidate_metrics.py`、`integrity_exploration.py`、P11/P15 | 历史在线地图比较；能耗/碰撞项是代理量 |
| 地图/Frontier/EGO | mapping/exploration/planning 与 vendored EGO | 同时保留旧 Python proxy 和真实 ROS EGO 包，不混称 |
| 动态障碍 | `dynamic_voxel_map.py`、`dynamic_planning.py`、P12 plugin | 历史动态体素/制动/重开仿真 |
| 时延/故障/弹性状态机 | `latency_safety.py`、`sentinel.py`、`impact_fault_injection` | 旧 P13/P14 与确定性注入工具 |
| 时钟/启动/FCU持续门控 | extnav clock/guard/startup/graph、FCU gate、P4/mission | 离线研究与 AP4.7.1 不解锁链路证据分开；不是新融合闭环任务 |
| 输入选择/丢失叠加 | `prepare_joint_livo_selection.py` | 6 类 × 5 个选择种子的索引 dry-run；未跑正式退化多法矩阵 |

独立串联位置原型的四个源文件也必须包含：`research_serial_position.h`、`test_research_serial_position.cpp`、`validate_serial_prototype.py`、`protocol.json`。它只完成独立数学/合成检查，未接入原生回放；不是论文里已经跑过的强串联对比臂。

目前只有同一段约 83.753 s 的既有飞行输入。输入选择种子改变同一飞行的故障时间，不是独立飞行重复。低纹理、非平地面、新模拟噪声、跨场景统计和完整退化矩阵仍属于后续实验；不能用“工具已收录”冒充这些实验完成。

## 完整公式到实现的对应关系

论文方法和推导中的原生传播、range/clone 增广、ray-plane 投影、端点像素噪声、原生条件更新、Joseph 更新、SO(3) reset、gyro 标记分量及两项交叉协方差消融，均由 `xq_fast_livo2` 的原生代码与 `research_joint_flow.h` / `research_joint_integration.cpp` 组成。具体 label 映射在 JSON。可选新 range 更新实现的存在不表示当前低占空回放消耗了全部 range 样本。

历史规划附录的完整公式另对应：

| 公式内容 | 实现 |
|---|---|
| 方向观测信息、时间记忆、信息退化膨胀与方向 PL | `integrity.py`，P6 wrapper |
| 首次对齐误差、higher 分位数、train-only 因子 | `calibrate_p7.py`，P7 collector |
| de Boor、点地图 AL、延迟预留 | `alert_limit.py` |
| grid inflation 与独立制动 helper | `planning.py` |
| 最小方向裕度、整轨迹硬过滤 | `integrity_margin.py`，P9 wrapper |
| surfel 信息预测、离散最小激励目标 | `minimum_excitation.py`，P10 |
| 地图收益、点态碰撞代理、运动/返航能耗 | `candidate_metrics.py` |
| hard feasibility 后的效用选择、滚动与中断成本 | `integrity_exploration.py`，P11 controller |
| 时延统计、nearest-rank、速度上限 | `latency_safety.py`，P13 controller |
| 动态重开/恢复及故障优先级 | dynamic 模块、P13/P14 controller、`sentinel.py` |
| monotonic 映射、永久故障锁存、acquisition age/rate | extnav transport/guard 模块 |
| FCU external-vision bitmask 与持续就绪、速度 slew | `fcu_extnav_gate.py`、P4 与 AP mission |

这些历史规划数学尚未和新的 raw-flow 联合状态一起重校准、重新验证。滤波协方差 PSD 不自动成为 PL，旧训练因子不能直接代入新协方差。

## 数据与场景资产

原始输入路径：

```text
/home/accelerate/impact_research_ws/experiments/results/research_multisensor/
r2b3b1_20261020T104238Z_8607/rosbag
```

7 个文件合计 **1,503,077,896 字节**：metadata 26,120 字节；6 个 DB3 分别为 270,364,672 / 270,528,512 / 269,950,976 / 269,053,952 / 269,193,216 / 153,960,448 字节。全部 SHA 在机器清单中。不要只打 12.47 MB sensor-index 而遗漏 DB；publisher 要从原 DB 行读取与核对 CDR。

五流输入计数为 IMU 16,586、LiDAR 830、前视 1,270、下视 1,270、range 4,188，共 28,144。原 index 指向旧绝对路径；新包必须对 `data/b1/rosbag` 重新 prepare 到新目录，而不是改原 source hash 或改 acquisition header。真值和无噪声 range 只导出给独立 evaluator。

前/下视 15 Hz 与 range 50 Hz 并非每帧同戳：127 个 exact seed，每次取下一原始图像，seed 间隔约 0.66 s。正式记录完成 123 对；其余 range 收到并审计，当前 profile 不融合。不能把它写成全 50 Hz range 利用率。

仓库内有 **21 个 SDF**，全部 XML 可解析；实际 SDFormat/Gazebo 加载仍须新运行验证。SDF 唯一外部机体链为：

```text
iris_with_ardupilot
  -> iris_with_standoffs
     -> meshes/iris_collision.stl
     -> meshes/iris.dae
     -> meshes/iris_prop_ccw.dae
     -> meshes/iris_prop_cw.dae
```

原 `/home/accelerate/ardupilot_gazebo/models` 有 53 文件、85,681,761 字节；worlds 有 5 文件、38,546 字节。本清单保留逐文件 SHA。AP 插件需从对应源重新构建，不能假设旧二进制 ABI 可迁移。另有项目自有 `libxq_p12_obstacle_motion_system.so` 需随 bridge 构建并加入插件路径。

旧 `/home/accelerate/cuadc_ws/src/uav_slam_sim` 的模型/world 合计 22,503,547 字节，本仓库 SDF 不引用它们，且未发现根 LICENSE；仅保留存在/大小/SHA 审计记录，不作为必需资源打包。AP Gazebo 根许可证文件为 `LICENSE.md`（内容为 LGPLv3）；保留原模型 provenance、许可和 notices。主 ROS 包声明混合 Apache-2.0、BSD、GPL-2.0、GPL-3.0-only，不能把整包称为单一 Apache 授权。

## 可便携运行的离线命令

以下命令的 CLI、配置读取与路径行为已按源码核对；是否在本次服务器 stage 运行通过，要看新的运行记录。设当前目录为 bundle 根，每个输出目录必须新建且独占：

```bash
export PYTHONPATH="$PWD/src/xq_autonomy${PYTHONPATH:+:$PYTHONPATH}"
export MPLBACKEND=Agg
python3 scripts/run_shared_scale_study.py \
  --config config/research_shared_scale_study.json --output NEW/shared_scale
python3 scripts/run_correlated_flow_study.py \
  --protocol config/research_flow_precondition_gate.json --output NEW/correlated
python3 scripts/run_joint_window_study.py \
  --protocol config/research_joint_window_gate.json --output NEW/joint_linear
python3 scripts/run_two_view_geometry_study.py \
  --protocol config/research_joint_window_gate.json --output NEW/two_view
python3 scripts/run_joint_flow_core_validation.py \
  --workspace-root "$PWD" --output-dir NEW/nonlinear_core
python3 prototypes/serial_position/validate_serial_prototype.py \
  --joint-header "$PWD/src/xq_fast_livo2/include/research_joint_flow.h" \
  --output-dir NEW/serial_core
```

前四项需要 Python NumPy，shared-scale 绘图需要 Matplotlib；C++ core/serial 需要 g++ 和 `/usr/include/eigen3`。这些合成研究不需要原 bag、Gazebo 或 AP。`run_correlated_flow_study.py` 只读取协议的 `synthetic_study`，不会打开同配置里用于另一工具的旧 bag 路径。几何/相关/跨窗审计脚本也应同时收录，不能只保留原作者计算器。

新原 bag index 的入口：

```bash
source /opt/ros/humble/setup.bash
python3 scripts/replay_joint_livo_inputs.py prepare \
  --bag "$PWD/data/b1/rosbag" --output NEW/joint_input_index
```

实际五法回放还需要新构建的 LIVO/ROS 环境与便携 runner。它与 core 数学检查是两道不同门。原 `run_joint_livo_replay.py` 的 build/install 约束应由服务器适配入口处理，不能仅因复制了脚本就宣称全部入口可运行。

## CPU/headless 场景与限制

已有资产 README 记录过以下无 AP 的算法载体场景路径。这里提供的是既有环境事实与可运行命令形态，不是本次新服务器帧率/实时性承诺：

```bash
export GZ_PARTITION="impact_server_${USER}_$$"
export GZ_SIM_RESOURCE_PATH="$PWD/src/xq_gz_assets/models"
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe
export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe
export EGL_PLATFORM=surfaceless
timeout 20s gz sim -s -r --headless-rendering --iterations 800 \
  "$PWD/src/xq_gz_assets/worlds/xq_indoor_office.sdf"
```

这个 model 是无重力 VelocityControl 算法载体，不能代替 AP 物理闭环。相机和 gpu_lidar 即使服务器无 GUI 仍需可用的 EGL/渲染后端；llvmpipe 是软件渲染，不是移除了渲染需求。若无 renderer，可先运行纯数学/core/原始记录回放，不能以未生成图像或 LiDAR 的空场景替代传感器验证。

AP range 采集旧入口为 `run_r2b3b1_acquisition_gate.sh`：world `xq_r2b3_range.sdf`，运行 camera/range/IMU/LiDAR/AP 并录 rosbag。它依赖旧 workspace、外部资产和历史保全路径，不能在新根直接运行；使用已审阅的新 AP4.7.1/服务器 launcher，不静默修改旧脚本。

## 绝对路径与历史重放输入

WSL `/home/accelerate/impact_research_ws` 主副本比 canonical 少 **44** 文件：src10、scripts23、config11；另 src 三处及 prepare_livo_dependencies.sh 不同。完整逐项 SHA 差异在 JSON。不要从主 WSL 副本推断当前算法版本，独立旧 install 也不能替代当前源码构建。

历史 shell gate 具有精确 workspace 校验与旧保全依赖，保留这些约束是为了保护旧结果。历史 `replay_extnav_clock_guard.py`、`replay_extnav_transport_clock.py`、`replay_extnav_startup_gate.py`、`study_extnav_ap_time_correction.py` 有旧 canonical、B3 archive、AP 源路径及绑定 SHA；要复现历史观察必须额外导出相应冻结输入，并用新 launcher 显式传路径/映射。不能拿当前模块冒充旧冻结模块，也不能把未带原输入的工具标为复现通过。

本清单未把系统 apt/ROS/Gazebo 依赖伪装成完整离线软件镜像；是否带 apt 缓存、预下载依赖或网络安装说明，由主打包 README 明示。

## 独立静态校验入口

```bash
python3 server/independent_verify_bundle.py \
  --workspace "$PWD" --output NEW/static_coverage.json
```

默认校验 597 源文件 bytes/SHA、15 ROS 包名与 license 声明、21 算法组、4 个串联原型源、外部模型/world exact set/SHA、SDF URI 闭包、B1 bag exact set/size/SHA；拒绝别名或 SQLite sidecar。无需 ROS，不执行包中算法，不打开 SQLite，不覆盖报告。初期不带 raw data 的检查可显式加 `--skip-rawdata`，报告会记录 rawdata_checked=false，不能当成完整数据校验。

工具还输出源中的 host-specific 绝对路径、建议迁移位置及其存在性；它不会重写程序。静态 PASS 不掩盖 historical entrypoint 适配工作，`portable_entrypoints_verified` 明确为 false。主 README 的新运行记录才是便携入口可执行性的证据。

