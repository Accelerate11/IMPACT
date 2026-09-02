# 需求追踪

| 企划书要求 | 本工作区证据 | 证明边界 |
|---|---|---|
| R1 室内 ATE <=0.3 m | P3 structured room 的真实 FAST-LIO2 首帧 SE(2) 对齐 ATE RMS=0.0330 m | 仅 Gazebo 室内 SIL，不是正式实机 ATE |
| R2 室外 ATE <=0.5 m | `UNVERIFIED` | 当前没有室外峡谷/城市峡谷场景和真值口径 |
| R3 5 cm 地图 | 记录栅格配置及已知尺寸场景 | 栅格尺寸不证明真实三维地图 5 cm 几何精度，保持 `UNVERIFIED` |
| R4 >=10 Hz | P3 `/localization/odom` 在 structured room/long corridor 均为 10.0 Hz，最大间隔 0.100 s | 证明 Gazebo SIL 中 FAST-LIO2，不证明 Atlas 满载频率 |
| R5 20% 丢包 | 项目内确定性 ground-link heartbeat relay 与故障窗统计 | 不污染 WSL 全局网络；不等同于多机地图交换验收 |
| R6 重规划 <=2 s | P12 正式单动态障碍检测 0.1325 s、BRAKE 重规划 0.2100 s | 仅固定单障碍 Gazebo SIL；多障碍、Atlas 满载与实机仍 `UNVERIFIED` |
| R7 故障弹性 | F1–F8 `FaultEvent`、逐故障验收检查与 Sentinel 状态时间线 | CPU 项仅为项目内负载代理；NPU 恢复仅为定时恢复代理；实飞仍需分级验证 |
| R8 多机支持 | agent/submap/frontier 接口定义 | 尚无多智能体运行节点；多机 SITL/HIL/实飞均为 `UNVERIFIED` |
| R9 <=30 W | 标记 `UNVERIFIED` | 必须用 Atlas 输入端功率仪实测 |
| P10 最小激励主动感知 | `docs/P10_MINIMUM_EXCITATION_ACCEPTANCE_REPORT.md`、`evidence/P10/` | 仅固定静态长走廊 Gazebo SIL；不代表连续动作全局最优或实机保证 |
| P11 完整性约束探索 | `docs/P11_INTEGRITY_CONSTRAINED_EXPLORATION_REPORT.md`、`evidence/P11/` | 仅固定静态 Gazebo SIL 的 24 m、四批冻结 Frontier 滚动认证与选择；不代表动态环境、全局最优或实机保证 |
| P12 动态障碍与动态地图 | `docs/P12_DYNAMIC_OBSTACLE_ACCEPTANCE_REPORT.md`、`evidence/P12/` | 单刚体、固定时序、室内长走廊 Gazebo SIL；无视觉语义、多目标预测、HIL 或实机保证 |
| P13 时延感知安全 | `docs/P13_LATENCY_AWARE_SAFETY_ACCEPTANCE_REPORT.md`、`evidence/P13/` | 50/200 ms 软件规划负载、固定动态走廊 Gazebo SIL；不代表 Atlas 满载、组合故障或实机最坏时延 |
| P14 故障注入与弹性自治 | `docs/P14_FAULT_INJECTION_ACCEPTANCE_REPORT.md`、`evidence/P14/` | 十类项目内确定性故障与持续 LiDAR 降级，固定单机 Gazebo SIL；不代表真实硬件故障、Atlas/openEuler/HIL 或实机安全 |

所有自动报告必须包含输入 TXT 的 SHA-256、场景哈希、随机种子、配置
快照和运行层级，防止把设计目标或仿真结果误写成实测结果。
