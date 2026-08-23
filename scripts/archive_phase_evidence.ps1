param(
  [string]$WslDistro = "Ubuntu-22.04"
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
  @("P9", "experiments/results/impact_p9/p9_20260823T135112Z_1110615/rosbag/metadata.yaml", "rosbag-metadata.yaml")
)

foreach ($entry in $files) {
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
}

foreach ($phase in $largeArtifacts.Keys) {
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

Write-Host "Archived P0-P9 lightweight evidence to $archiveRoot"
