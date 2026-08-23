#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
workspace_root="$(cd -- "${script_dir}/.." && pwd -P)"
default_source="${workspace_root}/experiments/results/impact_p6/p6_20260823T065905Z_36386/rosbag"
source_bag="${1:-${default_source}}"
timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
result_dir="${workspace_root}/experiments/results/impact_p8/p8_${timestamp}_$$"
result_file="${result_dir}/alert-limit-result.json"
p7_calibration="${workspace_root}/experiments/results/impact_p7/p7_20260823T082157Z_48605/p7-calibration.json"

[[ -f "${source_bag}/metadata.yaml" ]] || { echo "Source simulation bag not found: ${source_bag}" >&2; exit 2; }
[[ -f "${p7_calibration}" ]] || { echo "P7 prerequisite calibration not found: ${p7_calibration}" >&2; exit 2; }
mkdir -p -- "${result_dir}"

unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
set +u
source /opt/ros/humble/setup.bash
source "${workspace_root}/xq_install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-98}"
export ROS_LOCALHOST_ONLY=1
export ROS2CLI_NO_DAEMON=1

declare -a managed_pids=()
cleanup() {
  local pid pgid
  trap - EXIT INT TERM
  set +e
  for pid in "${managed_pids[@]}"; do
    pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d '[:space:]')"
    [[ -n "${pgid}" && "${pgid}" == "${pid}" ]] && kill -INT -- "-${pgid}" 2>/dev/null
  done
  for pid in "${managed_pids[@]}"; do wait "${pid}" 2>/dev/null; done
}
trap cleanup EXIT INT TERM

start_group() {
  local log_file="$1"
  shift
  setsid "$@" >"${log_file}" 2>&1 < /dev/null &
  managed_pids+=("$!")
}

bash "${script_dir}/audit_external_assets.sh" snapshot "${result_dir}/external-assets.before.sha256" >/dev/null
sha256sum "${p7_calibration}" >"${result_dir}/p7-prerequisite.sha256"
{
  echo "run_started_utc=${timestamp}"
  echo "source_simulation_bag=${source_bag}"
  echo "p7_calibration=${p7_calibration}"
  echo "ros_domain_id=${ROS_DOMAIN_ID}"
  echo "static_obstacles_only=true"
  echo "planner_feedback_enabled=false"
} >"${result_dir}/run.env"

start_group "${result_dir}/p8-stack.log" ros2 launch xq_sim_bringup xq_p8_alert_limit.launch.py result_file:="${result_file}"
timeout 30 bash -c 'until ros2 node list 2>/dev/null | grep -qx /xq_p8_alert_limit; do sleep 0.25; done'
ros2 node info /xq_p8_alert_limit >"${result_dir}/p8-node-graph.txt"
if grep -Fq '/xq/eval/' "${result_dir}/p8-node-graph.txt"; then
  echo "Ground Truth leaked into P8 Alert Limit node." >&2
  exit 8
fi

start_group "${result_dir}/output-bag.log" ros2 bag record \
  --compression-mode file --compression-format zstd -o "${result_dir}/rosbag" \
  /clock /localization/odom /xq/p5/cloud_map /planning/bspline \
  /integrity/alert_limit /integrity/alert_limit_debug
sleep 2

start_group "${result_dir}/source-replay.log" ros2 bag play "${source_bag}" --rate 4.0 \
  --topics /clock /localization/odom /xq/p5/cloud_map /planning/bspline
replay_pid="${managed_pids[-1]}"
wait "${replay_pid}"
sleep 2

[[ -s "${result_file}" ]] || { echo "P8 evaluator result missing." >&2; exit 9; }
status="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["status"])' "${result_file}")"
if [[ "${status}" != PASS ]]; then
  cat "${result_file}" >&2
  exit 10
fi

record_pid="${managed_pids[1]}"
record_pgid="$(ps -o pgid= -p "${record_pid}" | tr -d '[:space:]')"
kill -INT -- "-${record_pgid}" 2>/dev/null || true
wait "${record_pid}" 2>/dev/null || true

bash "${script_dir}/audit_external_assets.sh" snapshot "${result_dir}/external-assets.after.sha256" >/dev/null
bash "${script_dir}/audit_external_assets.sh" compare \
  "${result_dir}/external-assets.before.sha256" "${result_dir}/external-assets.after.sha256" \
  | tee "${result_dir}/isolation-audit.txt"

python3 - "${result_dir}" "${source_bag}" <<'PY'
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

root = Path(sys.argv[1])
evaluation = json.loads((root / "alert-limit-result.json").read_text(encoding="utf-8"))
summary = {
    "schema_version": 1,
    "gate": "P8_STATIC_OBSTACLE_ALERT_LIMIT",
    "status": "PASS",
    "source_simulation_bag": sys.argv[2],
    "alert_limit": evaluation,
    "rosbag_present": (root / "rosbag" / "metadata.yaml").is_file(),
    "ground_truth_isolated": "/xq/eval/" not in (root / "p8-node-graph.txt").read_text(encoding="utf-8"),
    "external_assets_unchanged": "PASS:" in (root / "isolation-audit.txt").read_text(encoding="utf-8"),
    "p7_prerequisite_frozen": True,
    "planner_feedback_enabled": False,
    "generated_at_utc": datetime.now(timezone.utc).isoformat(),
}
(root / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
print(json.dumps(summary, indent=2))
PY

echo "PASS: P8 static-obstacle Alert Limit Gate completed."
echo "Results: ${result_dir}"
