param(
  [string]$WslDistro = "Ubuntu-22.04",
  [string]$OnlyPhase = ""
)

$ErrorActionPreference = "Stop"
$repoRoot = Split-Path -Parent $PSScriptRoot
$wslRoot = "\\wsl.localhost\$WslDistro\home\accelerate\xuanqiong_x1_sim_ws"
$archiveRoot = Join-Path $repoRoot "evidence"

function Copy-EvidenceFile {
  param([string]$Phase, [string]$SourceRelative, [string]$DestinationRelative)
  $source = Join-Path $wslRoot ($SourceRelative -replace '/', '\')
  if (-not (Test-Path -LiteralPath $source -PathType Leaf)) {
    throw "Evidence source missing: $source"
  }
  $destination = Join-Path (Join-Path $archiveRoot $Phase) ($DestinationRelative -replace '/', '\')
  $destinationDirectory = Split-Path -Parent $destination
  New-Item -ItemType Directory -Force -Path $destinationDirectory | Out-Null
  Copy-Item -LiteralPath $source -Destination $destination -Force
}

$files = @(
  @("P0", "IMPACT_EXECUTION_PLAN_SHA256", "IMPACT_EXECUTION_PLAN_SHA256"),
  @("P0", "SOURCE_SPEC_SHA256", "SOURCE_SPEC_SHA256"),
  @("P1", "runs/p1_20260822T125115Z_388/summary.json", "summary.json"),
  @("P1", "runs/p1_20260822T125115Z_388/mission-result.json", "mission-result.json"),
  @("P1", "runs/p1_20260822T125115Z_388/final-state.txt", "final-state.txt"),
  @("P1", "runs/p1_20260822T125115Z_388/run.env", "run.env"),
  @("P1", "runs/p1_20260822T125115Z_388/isolation-audit.txt", "isolation-audit.txt"),
  @("P1", "runs/p1_20260822T125115Z_388/rosbag/metadata.yaml", "rosbag-metadata.yaml"),
  @("P2", "experiments/results/sensor_validation/p2_20260822T131950Z_1210/summary.json", "summary.json"),
  @("P2", "experiments/results/sensor_validation/p2_20260822T131950Z_1210/sensor-validation.json", "sensor-validation.json"),
  @("P2", "experiments/results/sensor_validation/p2_20260822T131950Z_1210/configuration.sha256", "configuration.sha256"),
  @("P2", "experiments/results/sensor_validation/p2_20260822T131950Z_1210/isolation-audit.txt", "isolation-audit.txt"),
  @("P2", "experiments/results/sensor_validation/p2_20260822T131950Z_1210/rosbag/metadata.yaml", "rosbag-metadata.yaml"),
  @("P3", "experiments/results/localization/p3_structured_room_20260822T142413Z_6016/summary.json", "structured-room-summary.json"),
  @("P3", "experiments/results/localization/p3_structured_room_20260822T142413Z_6016/evaluation.json", "structured-room-evaluation.json"),
  @("P3", "experiments/results/localization/p3_structured_room_20260822T142413Z_6016/algorithm-graph.txt", "structured-room-algorithm-graph.txt"),
  @("P3", "experiments/results/localization/p3_structured_room_20260822T142413Z_6016/isolation-audit.txt", "structured-room-isolation-audit.txt"),
  @("P3", "experiments/results/localization/p3_structured_room_20260822T142413Z_6016/rosbag/metadata.yaml", "structured-room-rosbag-metadata.yaml"),
  @("P3", "experiments/results/localization/p3_long_corridor_20260822T142613Z_6965/summary.json", "long-corridor-summary.json"),
  @("P3", "experiments/results/localization/p3_long_corridor_20260822T142613Z_6965/evaluation.json", "long-corridor-evaluation.json"),
  @("P3", "experiments/results/localization/p3_long_corridor_20260822T142613Z_6965/algorithm-graph.txt", "long-corridor-algorithm-graph.txt"),
  @("P3", "experiments/results/localization/p3_long_corridor_20260822T142613Z_6965/isolation-audit.txt", "long-corridor-isolation-audit.txt"),
  @("P3", "experiments/results/localization/p3_long_corridor_20260822T142613Z_6965/rosbag/metadata.yaml", "long-corridor-rosbag-metadata.yaml"),
  @("P4", "experiments/results/external_nav/p4_20260822T154217Z_10946/summary.json", "summary.json"),
  @("P4", "experiments/results/external_nav/p4_20260822T154217Z_10946/mission-result.json", "mission-result.json"),
  @("P4", "experiments/results/external_nav/p4_20260822T154217Z_10946/localization-evaluation.json", "localization-evaluation.json"),
  @("P4", "experiments/results/external_nav/p4_20260822T154217Z_10946/external-nav-topic-graph.txt", "external-nav-topic-graph.txt"),
  @("P4", "experiments/results/external_nav/p4_20260822T154217Z_10946/ground-truth-topic-graph.txt", "ground-truth-topic-graph.txt"),
  @("P4", "experiments/results/external_nav/p4_20260822T154217Z_10946/isolation-audit.txt", "isolation-audit.txt"),
  @("P4", "experiments/results/external_nav/p4_20260822T154217Z_10946/rosbag/metadata.yaml", "rosbag-metadata.yaml"),
  @("P5", "experiments/results/baseline_v1/p5_20260823T042657Z_29091/summary.json", "summary.json"),
  @("P5", "experiments/results/baseline_v1/p5_20260823T042657Z_29091/mission-result.json", "mission-result.json"),
  @("P5", "experiments/results/baseline_v1/p5_20260823T042657Z_29091/evaluation-result.json", "evaluation-result.json"),
  @("P5", "experiments/results/baseline_v1/p5_20260823T042657Z_29091/frontier-goal-graph.txt", "frontier-goal-graph.txt"),
  @("P5", "experiments/results/baseline_v1/p5_20260823T042657Z_29091/ground-truth-graph.txt", "ground-truth-graph.txt"),
  @("P5", "experiments/results/baseline_v1/p5_20260823T042657Z_29091/isolation-audit.txt", "isolation-audit.txt"),
  @("P5", "experiments/results/baseline_v1/p5_20260823T042657Z_29091/rosbag/metadata.yaml", "rosbag-metadata.yaml"),
  @("P6", "experiments/results/impact_p6/p6_20260823T065905Z_36386/summary.json", "summary.json"),
  @("P6", "experiments/results/impact_p6/p6_20260823T065905Z_36386/integrity-result.json", "integrity-result.json"),
  @("P6", "experiments/results/impact_p6/p6_20260823T065905Z_36386/p6-full-bag-analysis.json", "full-bag-analysis.json"),
  @("P6", "experiments/results/impact_p6/p6_20260823T065905Z_36386/p6-node-graph.txt", "node-graph.txt"),
  @("P6", "experiments/results/impact_p6/p6_20260823T065905Z_36386/isolation-audit.txt", "isolation-audit.txt"),
  @("P6", "experiments/results/impact_p6/p6_20260823T065905Z_36386/rosbag/metadata.yaml", "rosbag-metadata.yaml"),
  @("P7", "experiments/results/impact_p7/p7_20260823T082157Z_48605/summary.json", "summary.json"),
  @("P7", "experiments/results/impact_p7/p7_20260823T082157Z_48605/p7-calibration.json", "p7-calibration.json"),
  @("P7", "experiments/results/impact_p7/p7_20260823T082157Z_48605/calibration.before-tests.sha256", "calibration.before-tests.sha256"),
  @("P7", "experiments/results/impact_p7/p7_20260823T082157Z_48605/calibration.after-tests.sha256", "calibration.after-tests.sha256"),
  @("P7", "experiments/results/impact_p7/p7_20260823T082157Z_48605/test_structured_room/summary.json", "test-structured-room-summary.json"),
  @("P7", "experiments/results/impact_p7/p7_20260823T082157Z_48605/test_structured_room/evaluation.json", "test-structured-room-evaluation.json"),
  @("P7", "experiments/results/impact_p7/p7_20260823T082157Z_48605/test_structured_room/p7-capture.json", "test-structured-room-capture.json"),
  @("P7", "experiments/results/impact_p7/p7_20260823T082157Z_48605/test_long_corridor/summary.json", "test-long-corridor-summary.json"),
  @("P7", "experiments/results/impact_p7/p7_20260823T082157Z_48605/test_long_corridor/evaluation.json", "test-long-corridor-evaluation.json"),
  @("P7", "experiments/results/impact_p7/p7_20260823T082157Z_48605/test_long_corridor/p7-capture.json", "test-long-corridor-capture.json"),
  @("P8", "experiments/results/impact_p8/p8_gazebo_final_20260823T104800Z/summary.json", "summary.json"),
  @("P8", "experiments/results/impact_p8/p8_gazebo_final_20260823T104800Z/alert-limit-result.json", "alert-limit-result.json"),
  @("P8", "experiments/results/impact_p8/p8_gazebo_final_20260823T104800Z/evaluation-result.json", "evaluation-result.json"),
  @("P8", "experiments/results/impact_p8/p8_gazebo_final_20260823T104800Z/mission-result.json", "mission-result.json"),
  @("P8", "experiments/results/impact_p8/p8_gazebo_final_20260823T104800Z/p8-node-graph.txt", "node-graph.txt"),
  @("P8", "experiments/results/impact_p8/p8_gazebo_final_20260823T104800Z/gazebo-replay-view.txt", "gazebo-replay-view.txt"),
  @("P8", "experiments/results/impact_p8/p8_gazebo_final_20260823T104800Z/isolation-audit.txt", "isolation-audit.txt"),
  @("P8", "experiments/results/impact_p8/p8_gazebo_final_20260823T104800Z/rosbag/metadata.yaml", "rosbag-metadata.yaml"),
  @("P9", "experiments/results/impact_p9/p9_20260823T135112Z_1110615/summary.json", "summary.json"),
  @("P9", "experiments/results/impact_p9/p9_20260823T135112Z_1110615/p9-gate-result.json", "p9-gate-result.json"),
  @("P9", "experiments/results/impact_p9/p9_20260823T135112Z_1110615/margin-node-graph.txt", "margin-node-graph.txt"),
  @("P9", "experiments/results/impact_p9/p9_20260823T135112Z_1110615/gate-node-graph.txt", "gate-node-graph.txt"),
  @("P9", "experiments/results/impact_p9/p9_20260823T135112Z_1110615/run.env", "run.env"),
  @("P9", "experiments/results/impact_p9/p9_20260823T135112Z_1110615/p7-calibration.sha256", "p7-calibration.sha256"),
  @("P9", "experiments/results/impact_p9/p9_20260823T135112Z_1110615/p9-world.sha256", "p9-world.sha256"),
  @("P9", "experiments/results/impact_p9/p9_20260823T135112Z_1110615/sdf-validation.txt", "sdf-validation.txt"),
  @("P9", "experiments/results/impact_p9/p9_20260823T135112Z_1110615/isolation-audit.txt", "isolation-audit.txt"),
  @("P9", "experiments/results/impact_p9/p9_20260823T135112Z_1110615/gazebo-replay-view.txt", "gazebo-replay-view.txt"),
  @("P9", "experiments/results/impact_p9/p9_20260823T135112Z_1110615/rosbag/metadata.yaml", "rosbag-metadata.yaml"),
  @("P10", "experiments/results/impact_p10/contract_20260823T154236Z_14881/contract-result.json", "contract-result.json"),
  @("P10", "experiments/results/impact_p10/contract_20260823T154236Z_14881/selector-node-graph.txt", "contract-selector-node-graph.txt"),
  @("P10", "experiments/results/impact_p10/contract_20260823T154236Z_14881/isolation-audit.txt", "contract-isolation-audit.txt"),
  @("P10", "experiments/results/impact_p10/contract_20260823T154236Z_14881/rosbag/metadata.yaml", "contract-rosbag-metadata.yaml"),
  @("P10", "experiments/results/impact_p10/gate_20260823T154534Z_16472/summary.json", "flight-gate-summary.json"),
  @("P10", "experiments/results/impact_p10/gate_20260823T154534Z_16472/configuration.sha256", "configuration.sha256"),
  @("P10", "experiments/results/impact_p10/gate_20260823T154534Z_16472/configuration/p10_gate_thresholds.json", "p10-gate-thresholds.json"),
  @("P10", "experiments/results/impact_p10/gate_20260823T154534Z_16472/configuration/build-manifest.json", "build-manifest.json"),
  @("P10", "experiments/results/impact_p10/gate_20260823T154534Z_16472/baseline/flight-result.json", "baseline-flight-result.json"),
  @("P10", "experiments/results/impact_p10/gate_20260823T154534Z_16472/yaw_only/flight-result.json", "yaw-only-flight-result.json"),
  @("P10", "experiments/results/impact_p10/gate_20260823T154534Z_16472/minimum_excitation/flight-result.json", "minimum-excitation-flight-result.json"),
  @("P10", "experiments/results/impact_p10/gate_20260823T154534Z_16472/minimum_excitation/xq_p10_active_perception-graph.txt", "active-perception-node-graph.txt"),
  @("P10", "experiments/results/impact_p10/gate_20260823T154534Z_16472/minimum_excitation/xq_p10_flight_controller-graph.txt", "flight-controller-node-graph.txt"),
  @("P10", "experiments/results/impact_p10/gate_20260823T154534Z_16472/minimum_excitation/xq_p10_information_map-graph.txt", "information-map-node-graph.txt"),
  @("P10", "experiments/results/impact_p10/gate_20260823T154534Z_16472/minimum_excitation/run.env", "minimum-excitation-run.env"),
  @("P10", "experiments/results/impact_p10/gate_20260823T154534Z_16472/isolation-audit.txt", "flight-gate-isolation-audit.txt"),
  @("P10", "experiments/results/impact_p10/gate_20260823T154534Z_16472/baseline/rosbag/metadata.yaml", "baseline-rosbag-metadata.yaml"),
  @("P10", "experiments/results/impact_p10/gate_20260823T154534Z_16472/yaw_only/rosbag/metadata.yaml", "yaw-only-rosbag-metadata.yaml"),
  @("P10", "experiments/results/impact_p10/gate_20260823T154534Z_16472/minimum_excitation/rosbag/metadata.yaml", "minimum-excitation-rosbag-metadata.yaml"),
  @("P10", "experiments/results/impact_p10/visual_20260823T154916Z_18660/visualization.json", "visualization.json"),
  @("P10", "experiments/results/impact_p10/visual_20260823T154916Z_18660/flight-result.json", "visual-flight-result.json"),
  @("P10", "experiments/results/impact_p10/visual_20260823T154916Z_18660/isolation-audit.txt", "visual-isolation-audit.txt"),
  @("P10", "experiments/results/impact_p10/visual_20260823T154916Z_18660/rosbag/metadata.yaml", "visual-rosbag-metadata.yaml"),
  @("P11", "experiments/results/impact_p11/contract_20260827T053924Z_1726/contract-result.json", "contract-result.json"),
  @("P11", "experiments/results/impact_p11/contract_20260827T053924Z_1726/selector-node-graph.txt", "contract-selector-node-graph.txt"),
  @("P11", "experiments/results/impact_p11/contract_20260827T053924Z_1726/isolation-audit.txt", "contract-isolation-audit.txt"),
  @("P11", "experiments/results/impact_p11/contract_20260827T053924Z_1726/rosbag/metadata.yaml", "contract-rosbag-metadata.yaml"),
  @("P11", "experiments/results/impact_p11/gate_20260827T101925Z_22733/summary.json", "flight-gate-summary.json"),
  @("P11", "experiments/results/impact_p11/gate_20260827T101925Z_22733/configuration.sha256", "configuration.sha256"),
  @("P11", "experiments/results/impact_p11/gate_20260827T101925Z_22733/configuration/p11_gate_thresholds.json", "p11-gate-thresholds.json"),
  @("P11", "experiments/results/impact_p11/gate_20260827T101925Z_22733/configuration/build-manifest.json", "build-manifest.json"),
  @("P11", "experiments/results/impact_p11/gate_20260827T101925Z_22733/information_only/flight-result.json", "information-only-flight-result.json"),
  @("P11", "experiments/results/impact_p11/gate_20260827T101925Z_22733/integrity_constrained/flight-result.json", "integrity-constrained-flight-result.json"),
  @("P11", "experiments/results/impact_p11/gate_20260827T101925Z_22733/integrity_constrained/xq_p11_integrity_exploration-graph.txt", "integrity-exploration-node-graph.txt"),
  @("P11", "experiments/results/impact_p11/gate_20260827T101925Z_22733/integrity_constrained/xq_p11_flight_controller-graph.txt", "flight-controller-node-graph.txt"),
  @("P11", "experiments/results/impact_p11/gate_20260827T101925Z_22733/integrity_constrained/xq_p10_information_map-graph.txt", "information-map-node-graph.txt"),
  @("P11", "experiments/results/impact_p11/gate_20260827T101925Z_22733/integrity_constrained/run.env", "integrity-constrained-run.env"),
  @("P11", "experiments/results/impact_p11/gate_20260827T101925Z_22733/isolation-audit.txt", "flight-gate-isolation-audit.txt"),
  @("P11", "experiments/results/impact_p11/gate_20260827T101925Z_22733/information_only/rosbag/metadata.yaml", "information-only-rosbag-metadata.yaml"),
  @("P11", "experiments/results/impact_p11/gate_20260827T101925Z_22733/integrity_constrained/rosbag/metadata.yaml", "integrity-constrained-rosbag-metadata.yaml"),
  @("P11", "experiments/results/impact_p11/visual_20260827T102347Z_24548/visualization.json", "visualization.json"),
  @("P11", "experiments/results/impact_p11/visual_20260827T102347Z_24548/flight-result.json", "visual-flight-result.json"),
  @("P11", "experiments/results/impact_p11/visual_20260827T102347Z_24548/isolation-audit.txt", "visual-isolation-audit.txt"),
  @("P11", "experiments/results/impact_p11/visual_20260827T102347Z_24548/rosbag/metadata.yaml", "visual-rosbag-metadata.yaml"),
  @("P12", "experiments/results/impact_p12/gate_20260827T165121Z_879/flight-result.json", "flight-result.json"),
  @("P12", "experiments/results/impact_p12/gate_20260827T165121Z_879/run.env", "run.env"),
  @("P12", "experiments/results/impact_p12/gate_20260827T165121Z_879/configuration.sha256", "configuration.sha256"),
  @("P12", "experiments/results/impact_p12/gate_20260827T165121Z_879/configuration/p12_gate_thresholds.json", "p12-gate-thresholds.json"),
  @("P12", "experiments/results/impact_p12/gate_20260827T165121Z_879/configuration/build-manifest.json", "build-manifest.json"),
  @("P12", "experiments/results/impact_p12/gate_20260827T165121Z_879/xq_p12_dynamic_map-graph.txt", "dynamic-map-node-graph.txt"),
  @("P12", "experiments/results/impact_p12/gate_20260827T165121Z_879/xq_p12_flight_controller-graph.txt", "flight-controller-node-graph.txt"),
  @("P12", "experiments/results/impact_p12/gate_20260827T165121Z_879/xq_p11_integrity_exploration-graph.txt", "exploration-node-graph.txt"),
  @("P12", "experiments/results/impact_p12/gate_20260827T165121Z_879/isolation-audit.txt", "isolation-audit.txt"),
  @("P12", "experiments/results/impact_p12/gate_20260827T165121Z_879/rosbag/metadata.yaml", "rosbag-metadata.yaml"),
  @("P13", "experiments/results/impact_p13/gate_20260828T055346Z_1323/p13-gate-result.json", "p13-gate-result.json"),
  @("P13", "experiments/results/impact_p13/gate_20260828T055346Z_1323/configuration.sha256", "configuration.sha256"),
  @("P13", "experiments/results/impact_p13/gate_20260828T055346Z_1323/configuration/p13_gate_thresholds.json", "p13-gate-thresholds.json"),
  @("P13", "experiments/results/impact_p13/gate_20260828T055346Z_1323/configuration/build-manifest.json", "build-manifest.json"),
  @("P13", "experiments/results/impact_p13/gate_20260828T055346Z_1323/low_50ms/trial-result.json", "low-50ms-trial-result.json"),
  @("P13", "experiments/results/impact_p13/gate_20260828T055346Z_1323/low_50ms/p12-retention-result.json", "low-50ms-p12-retention-result.json"),
  @("P13", "experiments/results/impact_p13/gate_20260828T055346Z_1323/low_50ms/run.env", "low-50ms-run.env"),
  @("P13", "experiments/results/impact_p13/gate_20260828T055346Z_1323/low_50ms/xq_p13_flight_controller-graph.txt", "low-50ms-controller-node-graph.txt"),
  @("P13", "experiments/results/impact_p13/gate_20260828T055346Z_1323/low_50ms/rosbag/metadata.yaml", "low-50ms-rosbag-metadata.yaml"),
  @("P13", "experiments/results/impact_p13/gate_20260828T055346Z_1323/high_200ms/trial-result.json", "high-200ms-trial-result.json"),
  @("P13", "experiments/results/impact_p13/gate_20260828T055346Z_1323/high_200ms/p12-retention-result.json", "high-200ms-p12-retention-result.json"),
  @("P13", "experiments/results/impact_p13/gate_20260828T055346Z_1323/high_200ms/run.env", "high-200ms-run.env"),
  @("P13", "experiments/results/impact_p13/gate_20260828T055346Z_1323/high_200ms/xq_p13_flight_controller-graph.txt", "high-200ms-controller-node-graph.txt"),
  @("P13", "experiments/results/impact_p13/gate_20260828T055346Z_1323/high_200ms/rosbag/metadata.yaml", "high-200ms-rosbag-metadata.yaml"),
  @("P13", "experiments/results/impact_p13/gate_20260828T055346Z_1323/isolation-audit.txt", "isolation-audit.txt"),
  @("P14", "experiments/results/impact_p14/gate_20260828T072511Z_12130/p14-gate-result.json", "p14-gate-result.json"),
  @("P14", "experiments/results/impact_p14/gate_20260828T072511Z_12130/configuration.sha256", "configuration.sha256"),
  @("P14", "experiments/results/impact_p14/gate_20260828T072511Z_12130/configuration/p14_gate_thresholds.json", "p14-gate-thresholds.json"),
  @("P14", "experiments/results/impact_p14/gate_20260828T072511Z_12130/configuration/p14_matrix_schedule.json", "p14-matrix-schedule.json"),
  @("P14", "experiments/results/impact_p14/gate_20260828T072511Z_12130/configuration/p14_emergency_schedule.json", "p14-emergency-schedule.json"),
  @("P14", "experiments/results/impact_p14/gate_20260828T072511Z_12130/configuration/.xq_build_manifest.json", "build-manifest.json"),
  @("P14", "experiments/results/impact_p14/gate_20260828T072511Z_12130/matrix/trial-result.json", "matrix-trial-result.json"),
  @("P14", "experiments/results/impact_p14/gate_20260828T072511Z_12130/matrix/p12-retention-result.json", "matrix-p12-retention-result.json"),
  @("P14", "experiments/results/impact_p14/gate_20260828T072511Z_12130/matrix/p13-retention-result.json", "matrix-p13-retention-result.json"),
  @("P14", "experiments/results/impact_p14/gate_20260828T072511Z_12130/matrix/run.env", "matrix-run.env"),
  @("P14", "experiments/results/impact_p14/gate_20260828T072511Z_12130/matrix/impact_p14_controller-graph.txt", "matrix-controller-node-graph.txt"),
  @("P14", "experiments/results/impact_p14/gate_20260828T072511Z_12130/matrix/impact_sensor_proxy-graph.txt", "matrix-sensor-proxy-node-graph.txt"),
  @("P14", "experiments/results/impact_p14/gate_20260828T072511Z_12130/matrix/xq_p12_dynamic_map-graph.txt", "matrix-dynamic-map-node-graph.txt"),
  @("P14", "experiments/results/impact_p14/gate_20260828T072511Z_12130/matrix/xq_p11_integrity_exploration-graph.txt", "matrix-exploration-node-graph.txt"),
  @("P14", "experiments/results/impact_p14/gate_20260828T072511Z_12130/matrix/rosbag/metadata.yaml", "matrix-rosbag-metadata.yaml"),
  @("P14", "experiments/results/impact_p14/gate_20260828T072511Z_12130/emergency/trial-result.json", "emergency-trial-result.json"),
  @("P14", "experiments/results/impact_p14/gate_20260828T072511Z_12130/emergency/run.env", "emergency-run.env"),
  @("P14", "experiments/results/impact_p14/gate_20260828T072511Z_12130/emergency/impact_p14_controller-graph.txt", "emergency-controller-node-graph.txt"),
  @("P14", "experiments/results/impact_p14/gate_20260828T072511Z_12130/emergency/impact_sensor_proxy-graph.txt", "emergency-sensor-proxy-node-graph.txt"),
  @("P14", "experiments/results/impact_p14/gate_20260828T072511Z_12130/emergency/rosbag/metadata.yaml", "emergency-rosbag-metadata.yaml"),
  @("P14", "experiments/results/impact_p14/gate_20260828T072511Z_12130/external-assets.before.sha256", "external-assets.before.sha256"),
  @("P14", "experiments/results/impact_p14/gate_20260828T072511Z_12130/external-assets.after.sha256", "external-assets.after.sha256"),
  @("P14", "experiments/results/impact_p14/gate_20260828T072511Z_12130/isolation-audit.txt", "isolation-audit.txt")
)

foreach ($entry in $files) {
  if ($OnlyPhase -and $entry[0] -ne $OnlyPhase) { continue }
  Copy-EvidenceFile -Phase $entry[0] -SourceRelative $entry[1] -DestinationRelative $entry[2]
}

$largeArtifacts = @{
  P1 = @("runs/p1_20260822T125115Z_388/rosbag")
  P2 = @("experiments/results/sensor_validation/p2_20260822T131950Z_1210/rosbag")
  P3 = @(
    "experiments/results/localization/p3_structured_room_20260822T142413Z_6016/rosbag",
    "experiments/results/localization/p3_long_corridor_20260822T142613Z_6965/rosbag"
  )
  P4 = @("experiments/results/external_nav/p4_20260822T154217Z_10946/rosbag")
  P5 = @("experiments/results/baseline_v1/p5_20260823T042657Z_29091/rosbag")
  P6 = @("experiments/results/impact_p6/p6_20260823T065905Z_36386/rosbag")
  P7 = @(
    "experiments/results/impact_p7/p7_20260823T082157Z_48605/train_structured_room/rosbag",
    "experiments/results/impact_p7/p7_20260823T082157Z_48605/train_long_corridor/rosbag",
    "experiments/results/impact_p7/p7_20260823T082157Z_48605/test_structured_room/rosbag",
    "experiments/results/impact_p7/p7_20260823T082157Z_48605/test_long_corridor/rosbag"
  )
  P8 = @(
    "experiments/results/impact_p8/p8_gazebo_final_20260823T104800Z/rosbag",
    "experiments/results/impact_p8/p8_gazebo_final_20260823T104800Z/gz_record"
  )
  P9 = @(
    "experiments/results/impact_p9/p9_20260823T135112Z_1110615/rosbag",
    "experiments/results/impact_p9/p9_20260823T135112Z_1110615/gz_record"
  )
  P10 = @(
    "experiments/results/impact_p10/contract_20260823T154236Z_14881/rosbag",
    "experiments/results/impact_p10/gate_20260823T154534Z_16472/baseline/rosbag",
    "experiments/results/impact_p10/gate_20260823T154534Z_16472/yaw_only/rosbag",
    "experiments/results/impact_p10/gate_20260823T154534Z_16472/minimum_excitation/rosbag",
    "experiments/results/impact_p10/visual_20260823T154916Z_18660/rosbag",
    "experiments/results/impact_p10/visual_20260823T154916Z_18660/gz_record"
  )
  P11 = @(
    "experiments/results/impact_p11/contract_20260827T053924Z_1726/rosbag",
    "experiments/results/impact_p11/gate_20260827T101925Z_22733/information_only/rosbag",
    "experiments/results/impact_p11/gate_20260827T101925Z_22733/integrity_constrained/rosbag",
    "experiments/results/impact_p11/visual_20260827T102347Z_24548/rosbag",
    "experiments/results/impact_p11/visual_20260827T102347Z_24548/gz_record"
  )
  P12 = @(
    "experiments/results/impact_p12/gate_20260827T165121Z_879/rosbag",
    "experiments/results/impact_p12/gate_20260827T165121Z_879/gz_record"
  )
  P13 = @(
    "experiments/results/impact_p13/gate_20260828T055346Z_1323/low_50ms/rosbag",
    "experiments/results/impact_p13/gate_20260828T055346Z_1323/high_200ms/rosbag",
    "experiments/results/impact_p13/gate_20260828T055346Z_1323/high_200ms/gz_record"
  )
  P14 = @(
    "experiments/results/impact_p14/gate_20260828T072511Z_12130/matrix/rosbag",
    "experiments/results/impact_p14/gate_20260828T072511Z_12130/matrix/gz_record",
    "experiments/results/impact_p14/gate_20260828T072511Z_12130/emergency/rosbag"
  )
}

foreach ($phase in $largeArtifacts.Keys) {
  if ($OnlyPhase -and $phase -ne $OnlyPhase) { continue }
  $manifest = @("# SHA256  BYTES  WSL_RELATIVE_PATH")
  foreach ($relativeDirectory in $largeArtifacts[$phase]) {
    $directory = Join-Path $wslRoot ($relativeDirectory -replace '/', '\')
    Get-ChildItem -LiteralPath $directory -File -Recurse |
      Where-Object { $_.Name -match '\.(db3|zstd|tlog|tlog-journal)$' } |
      Sort-Object FullName |
      ForEach-Object {
        $hash = (Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256).Hash.ToLowerInvariant()
        $relativePath = $_.FullName.Substring($wslRoot.Length + 1).Replace('\', '/')
        $manifest += "$hash  $($_.Length)  $relativePath"
      }
  }
  $phaseDirectory = Join-Path $archiveRoot $phase
  New-Item -ItemType Directory -Force -Path $phaseDirectory | Out-Null
  $manifestPath = Join-Path $phaseDirectory "LARGE_ARTIFACTS.sha256"
  $utf8WithoutBom = New-Object System.Text.UTF8Encoding($false)
  [System.IO.File]::WriteAllText($manifestPath, (($manifest -join "`n") + "`n"), $utf8WithoutBom)
}

if ($OnlyPhase) {
  Write-Host "Archived $OnlyPhase lightweight evidence to $archiveRoot"
} else {
  Write-Host "Archived P0-P14 lightweight evidence to $archiveRoot"
}
