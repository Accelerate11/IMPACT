# 便携三维复杂场景与多种子实验入口

入口：`python3 server/run_scenario.py`。先按主 README 构建新的14包基础环境，再提供包根与外部runtime根。原来的 complex/P15 shell依赖旧xq_install形状、旧保护清单、GUI或WSL渲染配置；本入口不调用这些旧shell，直接复用包内原样的P13/P14 ROS launch与新install。

## 实际执行链

P13配置运行原生FAST-LIO2、P6方向信息、P10 surfel信息地图、P11轨迹候选硬过滤/选择、P12动态体素/障碍驱动、P13时延与速度门控以及独立评估器。P15 profile启用5×2 lattice、online_map指标、active_trajectory动态查询和current_margin_replan。它比基础2D proxy smoke多了真实FAST-LIO估计与历史三维任务模块。

机体仍为Gazebo VelocityControl算法载体；**没有AP、MAVROS、ARM、真实旋翼动力学或新28维raw-flow联合估计器**，也没有启动EGO executable（历史P11/P15候选由其自身B-spline/lattice代码生成）。不能把这些实现称成新的joint/AP闭环。旧P7校准只用于历史FAST-LIO任务层，不适用于新的joint estimator或任意新环境。

## 场景选择

| --scenario | world | 主要设置 |
|---|---|---|
| warehouse | xq_complex_warehouse.sdf | 复杂平面仓库、原complex候选/动态配置 |
| warehouse_3d | xq_complex_3d_warehouse.sdf | 允许垂向候选 |
| bidirectional | xq_complex_bidirectional_warehouse.sdf | 双向几何挑战 |
| compositional | xq_complex_compositional_warehouse.sdf | right/left/vertical组合挑战的旧配置 |
| p15_lattice | 同上 | 5×2 lattice、地图派生收益/能耗/碰撞代理、运行时裕度重规划 |
| p14_fault | xq_p12_dynamic_obstacle.sdf | 原P14 sensor proxy与故障状态机，仅integrity_constrained |

前五项接受 `--variant information_only` 或 `integrity_constrained`，P14明确拒绝未实现的information_only对比。原JSON阈值不放宽、算法源码不重写。

## 单个短观测

```bash
python3 server/run_scenario.py \
  --root "$PWD" --runtime-root /absolute/path/impact_runtime/base \
  --scenario p15_lattice --variant integrity_constrained \
  --seed 501 --duration 60 --renderer software
```

runtime默认检查最新基础build，要求该build为PASS；可显式指定 `--build-attempt`，不会自动回选旧PASS。每个run独占目录、ROS域、Gazebo partition、日志和输出，清理只涉及本轮跟踪的进程。原始源码与第三方模型/world文件集及SHA前后核对，安装launch与FAST-LIO/bridge配置和包内源核对。

`--duration`是全部必需主题（含native odometry、方向信息、静态地图、候选decision和控制status）出现后的墙钟观察长度，不是完成24m任务的保证。启动默认上限90秒，可用 `--startup-timeout` 显式指定。启动不全时报告FAIL并保存已收到的total_received，而不把观测窗口的0计数误写成全程无传感器。

短测试的PASS只代表SMOKE门满足。P11/P12/P13/P14原评估器的结果原样保留；未完成任务通常无最终评估文件或为失败。总报告的mission_status与acceptance_mode单独列出，不把SMOKE PASS升级成mission PASS。

## 完整任务模式与多场景/多种子计划

```bash
for scenario in warehouse warehouse_3d bidirectional compositional p15_lattice; do
  for seed in 501 502 503 504 505; do
    for variant in information_only integrity_constrained; do
      python3 server/run_scenario.py \
        --root "$PWD" --runtime-root /absolute/path/impact_runtime/base \
        --scenario "$scenario" --variant "$variant" --seed "$seed" \
        --duration 240 --require-mission-complete --renderer software
    done
  done
done
```

`--require-mission-complete`额外要求该场景全部原评估器PASS，缺少或失败均不通过。240秒只是可调整的运行预算，不保证任务结束。保留全部失败与未完run后再统计；上述是可执行实验命令，**不是已经完成的多场景/多种子统计结果**。建议先用单run确认服务器吞吐，再串行跑矩阵，避免并发渲染/大点云争用。

真实Gazebo seed通过runtime专属gz shim插入 `gz sim --seed N`，实际argv和shim/real-gz SHA均保留。原bridge conditioning随机种子仍固定20260822；已有fault schedule的显式种子不被重写。这里的seed不会随机生成新几何场景，也不能宣称改变了所有随机源。默认几何/障碍时序仍来自命名场景。

## 故障参数

原P14矩阵：

```bash
python3 server/run_scenario.py --root "$PWD" \
  --runtime-root /absolute/path/impact_runtime/base \
  --scenario p14_fault --seed 501 --duration 240 --require-mission-complete
```

指定新故障文件（按config/p14_matrix_schedule.json的schema）：

```bash
python3 server/run_scenario.py --root "$PWD" \
  --runtime-root /absolute/path/impact_runtime/base \
  --scenario p14_fault --fault-file /absolute/path/my_faults.json \
  --seed 502 --duration 120
```

单个可控故障：

```bash
python3 server/run_scenario.py --root "$PWD" \
  --runtime-root /absolute/path/impact_runtime/base \
  --scenario p14_fault --fault lidar_dropout --fault-start 20 \
  --fault-duration 2 --fault-severity 1 --seed 503 --duration 60
```

fault-file原字节、原显式种子与SHA保留；单个故障配置由CLI创建并采用该seed。其他场景明确拒绝故障选项，避免发布无人消费的事件却声称已注入。故障start时间是原注入节点开始收到有效仿真clock后的相对秒数，不是观察窗口的起点。若短run未到故障窗口，结果标NOT_OBSERVED，不能声称完成恢复验证。故障种类包含相机/链路等历史接口机制，不表示本场景部署了真实相机或完整新融合。

## 渲染、DDS与可复现边界

默认软件EGL/llvmpipe，无GUI、无WSL D3D12依赖；`--renderer auto`可用已有服务器渲染后端，但要重新记录与验证。复杂几何和原生FAST-LIO更耗CPU，不能套用基础smoke的点云速率或实时性结论。

最终入口复用已通过基础smoke的ROS_LOCALHOST_ONLY=1、FastDDS默认builtin SHM/UDP环境，清除继承的FastDDS profile，不生成额外传输覆盖，不更改系统/WSL/firewall。初版新增loopback UDP-only profile的高级run启动失败，完整保留后才修正部署配置；算法参数和阈值未改。

独立368640字节、10条消息的ROS运输夹具仍记录FAIL，包括显式ready/40秒存活的默认builtin夹具（domain86收到0条），不得声称该夹具或任意域通信已经解决。实际高级domain41另有原始LiDAR324条总接收、其中231条处于声明窗口。夹具/域差异尚未定位，跨服务器、跨域、并发负载仍须重新验证。不能通过调低算法阈值或重复发布旧状态伪装通信恢复。

冻结P7副本及PROVENANCE位于server/calibration。保留原SHA，并明确接受Git Linux LF签出的第二组精确SHA；不对任意文件作宽泛归一化。自定义 `--calibration-file` 是显式输入，仍要求train_only=true/test_data_used=false，并记录实际SHA；使用者须另外证明其适用域。

## 本次实际状态

基础14包构建、773项Python包测试和30秒LiDAR/IMU+2Dproxy smoke已通过，见SIMULATION_RUNBOOK.md。

首个高级p15_lattice/seed501/30秒run保留为**FAIL**：窗口未满足启动条件，clock/IMU/status/空静态地图有数据，但LiDAR、原生odom、方向信息和decision未出现，控制器停在WAIT_INITIALIZATION。原始结果、实际Gazebo seed、输入前后SHA及清理记录均保留；该失败没有被写成算法优越或任务通过。宿主/WSL中断原因未确认，不将其归因于OOM。

移除自增profile后，同一p15_lattice/seed501完成一次有界**SMOKE PASS**：启动18.027秒，观察窗口30.0008秒；窗口内LiDAR231、IMU5928、native FAST-LIO odometry296、方向信息296、静态地图148、decision54、status30。估计最大位移4.14米，控制器到BRAKE_DYNAMIC；launch与observer均SIGINT正常exit0，无TERM/KILL及遗留自有进程。输入文件集/SHA前后不变、实际Gazebo seed与新install身份均通过。

这仍然不是完整任务：mission_status=INCOMPLETE_OR_FAILED，decision有53次重复原stamp发布，不能把54次接收当54个新的规划结果，也不能把估计位移当独立真值精度。详细原始结果、旧FAIL及运输夹具见 `advanced_simulation_validation.json` 与validation目录。没有已完成的新joint在线复杂场景、AP任务或多种子统计实验。

