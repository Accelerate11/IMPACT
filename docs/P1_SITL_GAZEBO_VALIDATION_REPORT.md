# P1 ArduPilot SITL + Gazebo + MAVROS 验收报告

## 结论

Gate P1：**PASS**。

正式证据轮次：
`/home/accelerate/xuanqiong_x1_sim_ws/runs/p1_20260822T125115Z_388`。

## 闭环

```text
Gazebo Harmonic → ArduPilot Copter SITL → MAVROS2
                                     ↓
                    ARM → TAKEOFF → HOVER → LAND
```

## 指标

| 指标 | 结果 |
|---|---:|
| 连续墙钟运行 | 600 s |
| 飞行任务状态 | PASS |
| 任务闭环耗时 | 80.034 s |
| 悬停时间 | 30.0 s |
| 起飞高度门限触发值 | 1.638 m |
| 终态 | connected、disarmed、LAND |
| rosbag | 存在 |
| 外部 Gazebo 资产 | 字节级不变 |

状态机依次完成 FCU 连接、数据流请求、本地里程计门禁、GUIDED、解锁、起飞、爬升、
悬停、降落与上锁；服务 ACK 和状态话题均被记录，未以固定 sleep 代替状态确认。

## 证据

- `evidence/P1/summary.json`
- `evidence/P1/mission-result.json`
- `evidence/P1/final-state.txt`
- `evidence/P1/isolation-audit.txt`
- `docs/VALIDATION_REPORT.md`

## 边界

P1 使用 SITL/Gazebo 基础定位闭环，尚未证明 Mid-360S 数据契约、FAST-LIO2、GPS-off
ExternalNav 或 IMPACT 算法。
