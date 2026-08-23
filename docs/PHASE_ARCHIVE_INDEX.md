# P0–P10 阶段记录与验收归档索引

本索引是 GitHub 轻量归档入口。所有阶段均在 `PROGRESS.md` 保留正式结果记录，并有独立
验收报告。`evidence/` 保存正式 PASS 轮次的小型原始 JSON、配置哈希、节点图、bag
metadata 与隔离审计。

| 阶段 | 状态 | 阶段记录 | 验收报告 | 轻量证据 |
|---|---|---|---|---|
| P0 | PASS | `PROGRESS.md` | `docs/P0_REPOSITORY_VALIDATION_REPORT.md` | `evidence/P0/` |
| P1 | PASS | `PROGRESS.md` | `docs/P1_SITL_GAZEBO_VALIDATION_REPORT.md` | `evidence/P1/` |
| P2 | PASS | `PROGRESS.md` | `docs/P2_SENSOR_VALIDATION_REPORT.md` | `evidence/P2/` |
| P3 | PASS | `PROGRESS.md` | `docs/P3_FAST_LIO_VALIDATION_REPORT.md` | `evidence/P3/` |
| P4 | PASS | `PROGRESS.md` | `docs/P4_EXTERNAL_NAV_VALIDATION_REPORT.md` | `evidence/P4/` |
| P5 | PASS | `PROGRESS.md` | `docs/P5_BASELINE_VALIDATION_REPORT.md` | `evidence/P5/` |
| P6 | PASS | `PROGRESS.md` | `docs/P6_DIRECTIONAL_INTEGRITY_REPORT.md` | `evidence/P6/` |
| P7 | PASS | `PROGRESS.md` | `docs/P7_PROTECTION_LEVEL_CALIBRATION_REPORT.md` | `evidence/P7/` |
| P8 | PASS | `PROGRESS.md` | `docs/P8_ALERT_LIMIT_REPORT.md` | `evidence/P8/` |
| P9 | PASS | `PROGRESS.md` | `docs/P9_INTEGRITY_MARGIN_REPORT.md` | `evidence/P9/` |
| P10 | PASS | `PROGRESS.md` | `docs/P10_MINIMUM_EXCITATION_ACCEPTANCE_REPORT.md` | `evidence/P10/` |

## 大型原始证据策略

下列文件不直接进入普通 Git 仓库：rosbag 数据库与压缩分卷、Gazebo `state.tlog`、
构建/安装树、SITL EEPROM 和运行缓存。它们仍保存在 WSL 正式结果目录；GitHub 归档中
保存 `LARGE_ARTIFACTS.sha256`，记录相对路径、字节数和 SHA-256，可对本地原件做
字节级核验。

该策略不会把“报告存在”当成算法有效；阶段 PASS 仍以报告列明的运行 Gate、原始
metrics/summary 和隔离审计共同判定。
