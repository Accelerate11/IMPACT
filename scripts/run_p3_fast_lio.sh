#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
workspace_root="$(cd -- "${script_dir}/.." && pwd -P)"
scenario="structured_room"
duration_s=75
requested_result_dir=""
p7_split=""
trajectory_variant="baseline"
calibration_file=""

usage() {
  echo "Usage: $0 [--scenario structured_room|long_corridor] [--duration SECONDS] [--result-dir PATH] [--p7-split train|test --trajectory-variant train|test --calibration-file PATH]" >&2
}

while (($#)); do
  case "$1" in
    --scenario) scenario="$2"; shift 2 ;;
    --duration) duration_s="$2"; shift 2 ;;
    --result-dir) requested_result_dir="$2"; shift 2 ;;
    --p7-split) p7_split="$2"; shift 2 ;;
    --trajectory-variant) trajectory_variant="$2"; shift 2 ;;
    --calibration-file) calibration_file="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) usage; exit 2 ;;
  esac
done
[[ "${scenario}" == "structured_room" || "${scenario}" == "long_corridor" ]] || {
  echo "Unsupported P3 scenario: ${scenario}" >&2; exit 2;
}
[[ "${duration_s}" =~ ^[1-9][0-9]*$ ]] || { echo "Invalid duration" >&2; exit 2; }
((duration_s >= 65)) || { echo "P3 duration must be at least 65 seconds." >&2; exit 2; }
[[ -z "${p7_split}" || "${p7_split}" == train || "${p7_split}" == test ]] || { echo "Invalid P7 split." >&2; exit 2; }
if [[ -n "${p7_split}" ]]; then
  [[ "${trajectory_variant}" == train || "${trajectory_variant}" == test || "${trajectory_variant}" == validation ]] || { echo "P7 requires train/test/validation trajectory variant." >&2; exit 2; }
  [[ "${p7_split}" != test || -f "${calibration_file}" ]] || { echo "P7 test requires frozen calibration file." >&2; exit 2; }
fi

for required in \
  "${workspace_root}/xq_install/setup.bash" \
  "${workspace_root}/xq_install/.xq_build_manifest.json" \
  "${workspace_root}/SOURCE_SPEC_SHA256" \
  "${workspace_root}/IMPACT_EXECUTION_PLAN_SHA256" \
  "${workspace_root}/scripts/audit_external_assets.sh"; do
  [[ -f "${required}" ]] || { echo "Missing dependency: ${required}" >&2; exit 2; }
done

timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
if [[ -n "${p7_split}" ]]; then
  results_root="${workspace_root}/experiments/results/impact_p7"
else
  results_root="${workspace_root}/experiments/results/localization"
fi
if [[ -n "${requested_result_dir}" ]]; then
  result_dir="$(realpath -m -- "${requested_result_dir}")"
else
  result_dir="${results_root}/p3_${scenario}_${timestamp}_$$"
fi
case "${result_dir}" in
  "${results_root}"/*) ;;
  *) echo "Result directory must stay below ${results_root}." >&2; exit 2 ;;
esac
mkdir -p -- "${result_dir}/configuration" "${result_dir}/ros_logs"

unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
unset GZ_SIM_RESOURCE_PATH IGN_GAZEBO_RESOURCE_PATH SDF_PATH
unset RMW_IMPLEMENTATION FASTRTPS_DEFAULT_PROFILES_FILE CYCLONEDDS_URI
set +u
source /opt/ros/humble/setup.bash
source "${workspace_root}/xq_install/setup.bash"
set -u

export ROS_DOMAIN_ID=$((102 + (10#$(date +%S) + $$) % 90))
export ROS_LOCALHOST_ONLY=1
export ROS2CLI_NO_DAEMON=1
export GZ_PARTITION="xq_p3_${scenario}_${USER:-wsl}_${timestamp}_$$"
export ROS_LOG_DIR="${result_dir}/ros_logs"
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe
export GALLIUM_DRIVER=llvmpipe
export EGL_PLATFORM=surfaceless
export QT_QPA_PLATFORM=offscreen

install_root="${workspace_root}/xq_install"
world="${install_root}/xq_gz_assets/share/xq_gz_assets/worlds/xq_p3_${scenario}.sdf"
model="${install_root}/xq_gz_assets/share/xq_gz_assets/models/xq_iris_mid360/model.sdf"
bridge_config="${install_root}/xq_gz_bridge/share/xq_gz_bridge/config/p3_fast_lio.yaml"
fast_lio_config="${install_root}/xq_fast_lio/share/xq_fast_lio/config/xq_p3.yaml"
launch_file="${install_root}/xq_sim_bringup/share/xq_sim_bringup/launch/xq_p3_fast_lio.launch.py"
evaluation_file="${result_dir}/evaluation.json"
if [[ -n "${p7_split}" ]]; then
  launch_file="${install_root}/xq_sim_bringup/share/xq_sim_bringup/launch/xq_p7_calibration_capture.launch.py"
fi
p7_result_file="${result_dir}/p7-capture.json"
for required in "${world}" "${model}" "${bridge_config}" "${fast_lio_config}" "${launch_file}"; do
  [[ -f "${required}" ]] || { echo "Missing installed P3 input: ${required}" >&2; exit 2; }
done

cp -- "${world}" "${result_dir}/configuration/world.sdf"
cp -- "${model}" "${result_dir}/configuration/model.sdf"
cp -- "${bridge_config}" "${result_dir}/configuration/p3_fast_lio_bridge.yaml"
cp -- "${fast_lio_config}" "${result_dir}/configuration/xq_p3_fast_lio.yaml"
cp -- "${launch_file}" "${result_dir}/configuration/xq_p3_fast_lio.launch.py"
cp -- "${workspace_root}/src/xq_fast_lio/PROVENANCE.md" "${result_dir}/configuration/FAST_LIO_PROVENANCE.md"
cp -- "${workspace_root}/src/xq_autonomy/xq_autonomy/p3_trajectory_node.py" \
  "${result_dir}/configuration/p3_trajectory_node.py"
cp -- "${workspace_root}/src/xq_autonomy/xq_autonomy/p3_evaluator_node.py" \
  "${result_dir}/configuration/p3_evaluator_node.py"
cp -- "${install_root}/.xq_build_manifest.json" "${result_dir}/configuration/build-manifest.json"
cp -- "${workspace_root}/SOURCE_SPEC_SHA256" "${result_dir}/configuration/SOURCE_SPEC_SHA256"
cp -- "${workspace_root}/IMPACT_EXECUTION_PLAN_SHA256" \
  "${result_dir}/configuration/IMPACT_EXECUTION_PLAN_SHA256"
sha256sum "${result_dir}"/configuration/* >"${result_dir}/configuration.sha256"
find "${workspace_root}/src/xq_fast_lio" -type f -print0 | sort -z | \
  xargs -0 sha256sum >"${result_dir}/fast-lio-source-tree.sha256"
cat >"${result_dir}/run.env" <<EOF
run_started_utc=${timestamp}
scenario=${scenario}
phase7_split=${p7_split:-none}
trajectory_variant=${trajectory_variant}
minimum_duration_s=${duration_s}
ros_domain_id=${ROS_DOMAIN_ID}
gz_partition=${GZ_PARTITION}
lidar_topic=/livox/lidar
imu_topic=/livox/imu
odom_topic=/localization/odom
evaluation_ground_truth_topic=/xq/eval/agent_01/ground_truth
algorithm_ground_truth_subscription_forbidden=true
EOF

before_audit="${result_dir}/external-assets.before.sha256"
after_audit="${result_dir}/external-assets.after.sha256"
bash "${script_dir}/audit_external_assets.sh" snapshot "${before_audit}" >/dev/null

declare -a pids=()
declare -a labels=()
audit_done=false

start_group() {
  local label="$1" log_file="$2"
  shift 2
  setsid "$@" >"${log_file}" 2>&1 < /dev/null &
  local pid=$!
  sleep 0.25
  local pgid
  pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d '[:space:]')"
  [[ "${pgid}" == "${pid}" ]] || {
    echo "Could not isolate ${label} process group." >&2
    return 4
  }
  pids+=("${pid}")
  labels+=("${label}")
  echo "${pid}" >"${result_dir}/${label}.pid"
}

stop_groups() {
  local index pid pgid round alive
  for ((index=${#pids[@]}-1; index>=0; index--)); do
    pid="${pids[index]}"
    pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d '[:space:]' || true)"
    # Notify only the session leader first. ros2 launch forwards one SIGINT to
    # each child; signaling the whole group here would deliver a second SIGINT
    # directly and can trigger a spurious rclpy guard-condition error.
    [[ "${pgid}" == "${pid}" ]] && kill -INT -- "${pid}" 2>/dev/null || true
  done
  for round in 1 2 3 4 5 6 7 8 9 10; do
    alive=false
    for pid in "${pids[@]}"; do kill -0 "${pid}" 2>/dev/null && alive=true; done
    [[ "${alive}" == false ]] && break
    sleep 1
  done
  for pid in "${pids[@]}"; do
    pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d '[:space:]' || true)"
    [[ "${pgid}" == "${pid}" ]] && kill -TERM -- "-${pgid}" 2>/dev/null || true
  done
  for round in 1 2 3 4 5; do
    alive=false
    for pid in "${pids[@]}"; do kill -0 "${pid}" 2>/dev/null && alive=true; done
    [[ "${alive}" == false ]] && break
    sleep 1
  done
  for pid in "${pids[@]}"; do
    pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d '[:space:]' || true)"
    [[ "${pgid}" == "${pid}" ]] && kill -KILL -- "-${pgid}" 2>/dev/null || true
  done
  for pid in "${pids[@]}"; do wait "${pid}" 2>/dev/null || true; done
}

finish_audit() {
  [[ "${audit_done}" == false ]] || return 0
  bash "${script_dir}/audit_external_assets.sh" snapshot "${after_audit}" >/dev/null
  bash "${script_dir}/audit_external_assets.sh" compare \
    "${before_audit}" "${after_audit}" >"${result_dir}/isolation-audit.txt" 2>&1
  audit_done=true
}

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  set +e
  stop_groups
  finish_audit
  local audit_status=$?
  ((audit_status == 0)) || status="${audit_status}"
  exit "${status}"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

duration_parameter="${duration_s}.0"
if [[ -n "${p7_split}" ]]; then
  p7_launch_arguments=(
    world_file:="${world}" scenario:="${scenario}" split:="${p7_split}"
    trajectory_variant:="${trajectory_variant}" p3_result_file:="${evaluation_file}"
    p7_result_file:="${p7_result_file}" minimum_duration_s:="${duration_parameter}"
  )
  if [[ "${p7_split}" == test ]]; then
    p7_launch_arguments+=(calibration_file:="${calibration_file}")
  fi
  start_group launch "${result_dir}/launch.log" \
    ros2 launch xq_sim_bringup xq_p7_calibration_capture.launch.py "${p7_launch_arguments[@]}"
else
  start_group launch "${result_dir}/launch.log" \
    ros2 launch xq_sim_bringup xq_p3_fast_lio.launch.py \
    world_file:="${world}" scenario:="${scenario}" \
    result_file:="${evaluation_file}" minimum_duration_s:="${duration_parameter}"
fi

ready_deadline=$((SECONDS + 90))
while ((SECONDS < ready_deadline)); do
  if [[ -s "${evaluation_file}" ]] && grep -q '"count": [1-9]' "${evaluation_file}"; then
    break
  fi
  kill -0 "${pids[0]}" 2>/dev/null || {
    echo "P3 launch exited before localization readiness." >&2
    tail -n 120 "${result_dir}/launch.log" >&2 || true
    exit 5
  }
  sleep 1
done
[[ -s "${evaluation_file}" ]] || { echo "P3 evaluator produced no report." >&2; exit 6; }

ros2 node info /xq_fast_lio >"${result_dir}/algorithm-graph.txt"
subscriber_block="$(sed -n '/Subscribers:/,/Publishers:/p' "${result_dir}/algorithm-graph.txt")"
if grep -q '/xq/eval/\|ground_truth' <<<"${subscriber_block}"; then
  echo "Ground truth leaked into FAST-LIO subscriber graph." >&2
  exit 7
fi
if [[ -n "${p7_split}" ]]; then
  ros2 node info /xq_p6_directional_integrity >"${result_dir}/p6-predictor-graph.txt"
  p6_subscribers="$(sed -n '/Subscribers:/,/Publishers:/p' "${result_dir}/p6-predictor-graph.txt")"
  if grep -q '/xq/eval/\|ground_truth' <<<"${p6_subscribers}"; then
    echo "Ground truth leaked into P6 predictor during P7." >&2
    exit 12
  fi
fi

start_group rosbag "${result_dir}/rosbag.log" \
  ros2 bag record -o "${result_dir}/rosbag" \
  --compression-mode file --compression-format zstd \
  --compression-threads 2 --max-bag-duration 60 \
  /livox/lidar /livox/imu /localization/odom \
  /xq/eval/agent_01/ground_truth /tf /tf_static /clock \
  $([[ -n "${p7_split}" ]] && printf '%s' '/localization/geometry /integrity/directional' || true)

evaluation_deadline=$((SECONDS + duration_s + 150))
status="IN_PROGRESS"
while ((SECONDS < evaluation_deadline)); do
  kill -0 "${pids[0]}" 2>/dev/null || {
    echo "P3 launch exited during evaluation." >&2
    tail -n 120 "${result_dir}/launch.log" >&2 || true
    exit 8
  }
  status="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["status"])' \
    "${evaluation_file}" 2>/dev/null || echo IN_PROGRESS)"
  [[ "${status}" == "FAIL" ]] && {
    cat "${evaluation_file}" >&2
    exit 9
  }
  [[ "${status}" == "PASS" ]] && break
  sleep 1
done
[[ "${status}" == "PASS" ]] || { echo "P3 did not reach PASS before deadline." >&2; exit 10; }
if [[ -n "${p7_split}" ]]; then
  p7_deadline=$((SECONDS + 30))
  while [[ ! -s "${p7_result_file}" ]] && ((SECONDS < p7_deadline)); do sleep 1; done
  [[ -s "${p7_result_file}" ]] || { echo "P7 collector result missing." >&2; exit 13; }
  [[ "$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["status"])' "${p7_result_file}")" == PASS ]] || {
    cat "${p7_result_file}" >&2; exit 14;
  }
fi

stop_groups
finish_audit
[[ -f "${result_dir}/rosbag/metadata.yaml" ]] || {
  echo "rosbag metadata is missing." >&2
  exit 11
}
python3 - "${result_dir}" <<'PY'
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

root = Path(sys.argv[1])
evaluation = json.loads((root / "evaluation.json").read_text(encoding="utf-8"))
metrics = evaluation["metrics"]
summary = {
    "schema_version": 1,
    "gate": "P3_FAST_LIO2_BASELINE",
    "scenario": evaluation["scenario"],
    "status": evaluation["status"],
    "capture_wall_duration_s": evaluation["capture_wall_duration_s"],
    "odom_frequency_hz": evaluation["odom"]["frequency_hz"],
    "odom_max_gap_s": evaluation["odom"]["max_gap_s"],
    "ate_rms_m": metrics["ate_rms_m"],
    "rpe_translation_rms_m_1s": metrics["rpe_translation_rms_m_1s"],
    "yaw_error_rms_deg": metrics["yaw_error_rms_deg"],
    "processing_latency_mean_s": metrics["processing_latency_mean_s"],
    "processing_latency_max_s": metrics["processing_latency_max_s"],
    "checks": evaluation["checks"],
    "rosbag_present": (root / "rosbag" / "metadata.yaml").is_file(),
    "algorithm_ground_truth_isolated": "/xq/eval/" not in (
        root / "algorithm-graph.txt"
    ).read_text(encoding="utf-8").split("Publishers:", 1)[0],
    "external_assets_unchanged": "PASS:" in (
        root / "isolation-audit.txt"
    ).read_text(encoding="utf-8"),
    "generated_at_utc": datetime.now(timezone.utc).isoformat(),
}
if (root / "p7-capture.json").is_file():
    summary["p7_capture"] = json.loads((root / "p7-capture.json").read_text(encoding="utf-8"))
    raw = summary["p7_capture"].get("raw_directional_samples")
    if raw:
        summary["p7_capture"]["raw_directional_samples"] = {
            name: {"count": len(values.get("ratio", []))}
            for name, values in raw.items()
        }
    summary["gate"] = "P7_PROTECTION_LEVEL_CALIBRATION_CAPTURE"
(root / "summary.json").write_text(
    json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
)
print(json.dumps(summary, ensure_ascii=False, indent=2))
PY

echo "PASS: P3 FAST-LIO2 ${scenario} Gate completed."
echo "Results: ${result_dir}"
