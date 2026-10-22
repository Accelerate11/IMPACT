# 服务器验证记录

本目录内结果来自 WSL Ubuntu 22.04 / x86_64 的新路径、新构建前缀。原始失败及其后修正分别保留，不将旧机器已安装的二进制当成服务器新构建。

| 入口 | 实际验证 | 证据 |
|---|---|---|
| `offline_studies.py` | 六项离线数学/核心研究均 PASS | `validation/offline/summary.json` |
| `build_livo_server.sh` / `run_livo_server.sh` | 新路径重建、完整五组同输入回放、独立数理重建与评估 PASS | `LIVO_SERVER_VALIDATION.json`、`validation/livo/` |
| `build_workspace.sh` | 14 包构建及 773 项 Python 测试 PASS | `simulation_validation.json`、`validation/base_*/` |
| `run_headless_sim.py` | 30 秒 LiDAR/IMU/历史代理链 smoke PASS | 同上；不代表新融合或 AP 闭环 |
| `ap471_server.py` | 官方 4.7.1 新构建、未解锁版本和参数 smoke、Gazebo 插件重建 PASS | `AP471_SERVER_VALIDATION.json`；完整原日志在总包 provenance |

`run_scenario.py` 的 p15_lattice/seed501 30秒 SMOKE 已实际通过，见 `advanced_simulation_validation.json`；完整任务仍未通过，其他场景尚需分别验收。失败与通信夹具原记录也保留。下视模块需要的图像/轨迹接口、传感器标定、Orin 实时性与新融合/AP 完整任务仍需各自验证。

`coverage_manifest.json` 是静态源码/资源身份与公式映射，不是数值正确性报告。论文完整模型在参考单段仿真数据上的结果和次数已保留，重新运行时应报告本次实际输出。

历史研究完整封存在 [Release](https://github.com/Accelerate11/IMPACT/releases/tag/server-sim-2026.10.22) 的 `IMPACT_Research_Evidence_20261022.tar.gz`。原始 B1 全分卷和第三方源码由 `resources.json` 固定，`fetch_resources.py` 可下载、校验并安装。
