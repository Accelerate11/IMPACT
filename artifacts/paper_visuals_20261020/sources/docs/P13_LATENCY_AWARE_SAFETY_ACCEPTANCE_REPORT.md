# P13 Latency-Aware Safety 验收报告

## 结论

P13 正式 Gate **PASS**。同一 Gazebo 世界中，50 ms 与 200 ms 人工规划负载均完成
24 m 全走廊飞行、P12 动态障碍制动/TTL 重开和 P11 滚动硬认证。实测 p99 直接进入
Alert Limit；高时延轮次通过降速恢复最低完整性裕度。

正式证据：

```text
/home/accelerate/xuanqiong_x1_sim_ws/experiments/results/impact_p13/
gate_20260828T055346Z_1323
```

## 正式指标

| 指标 | 50 ms profile | 200 ms profile |
|---|---:|---:|
| planner processing p50 | 50.10 ms | 200.24 ms |
| end-to-end p99 | 150.51 ms | 301.29 ms |
| 未缓解 AL | 0.16773 m | 0.07714 m |
| 未缓解 Margin | +0.06773 m | -0.02286 m |
| 缓解后 AL | 0.16773 m | 0.16000 m |
| 缓解后 Margin | +0.06773 m | +0.06000 m |
| 速度上限 | 0.42000 m/s | 0.14500 m/s |
| 任务时间 | 132.4 s | 221.4 s |
| 净前进 | 23.99 m | 23.96 m |
| ATE RMS | 0.084 m | 0.079 m |

端到端 p99 增加 `150.79 ms`，未缓解 AL 收紧 `0.09058 m`，速度上限减少
`0.27500 m/s`。两轮 P13 trial、两轮 P12 保持性 Gate、同世界哈希、Ground Truth
隔离和外部资产逐字节审计全部通过。

## 研发中保留的失败

1. 诊断 Domain ID 超出 DDS 合法范围，全部节点拒绝启动；正式脚本改为 160–204。
2. 低时延轮次传空 `gz_record_path` 被 launch 拒绝；改为仅在高时延轮次传参。
3. 首次比较把“降速后 AL”用于因果对比，因裕度恢复而只差 7 mm；修正为同时报告
   raw AL 与 safe AL，不降低 20 mm 对比阈值。
4. 曾尝试共同降低名义速度，但改变了到达第三段时的观测几何并触发 P11 硬拒绝；该
   方案撤销，恢复相同 0.42 m/s 基线。
5. P11 单次硬拒绝原本会永久悬停；新增保持悬停的有界重采样，不放宽 Gate、不执行
   被拒轨迹。最终正式轮次无需重试，但该失效路径已封闭。

## 可视化复现

```bash
cd /home/accelerate/xuanqiong_x1_sim_ws
bash scripts/view_p13_combined.sh
```

Gazebo 显示开顶、保留墙体的动态障碍全程；RViz 显示静态/动态体素、FAST-LIO 路径、
选中轨迹，以及 p99、raw/safe AL、Margin 和速度上限。双窗口已实际启动验证。

重新运行正式 Gate：

```bash
bash scripts/run_p13_flight_gate.sh
```

## 证明边界

本阶段证明固定单机 Gazebo SIL、单动态障碍与软件注入规划负载下的时延因果链，不代表
Atlas 满载调度、真实相机负载、网络抖动、HIL 或实机最坏时延。上述故障组合属于 P14
和硬件阶段。
