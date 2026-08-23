#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
workspace_root="$(cd -- "${script_dir}/.." && pwd -P)"
timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
default_result_dir="${workspace_root}/experiments/results/impact_p10/contract_${timestamp}_$$"
result_dir="$(realpath -m -- "${1:-${default_result_dir}}")"
case "${result_dir}" in
  "${workspace_root}"/experiments/results/impact_p10/*) ;;
  *) echo "P10 result directory must stay under this workspace's impact_p10 results." >&2; exit 2 ;;
esac
mkdir -p -- "${result_dir}/ros_logs"
result_file="${result_dir}/contract-result.json"
calibration="${workspace_root}/evidence/P7/p7-calibration.json"
[[ -f "${calibration}" ]] || { echo "P7 calibration missing: ${calibration}" >&2; exit 2; }

unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
unset GZ_SIM_RESOURCE_PATH IGN_GAZEBO_RESOURCE_PATH SDF_PATH
set +u
source /opt/ros/humble/setup.bash
source "${workspace_root}/xq_install/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-125}"
export ROS_LOCALHOST_ONLY=1
export ROS2CLI_NO_DAEMON=1
export ROS_LOG_DIR="${result_dir}/ros_logs"

declare -a managed_pids=()
start_group() {
  local log_file="$1"
  shift
  setsid "$@" >"${log_file}" 2>&1 < /dev/null &
  managed_pids+=("$!")
}
stop_groups() {
  local pid pgid round alive
  for pid in "${managed_pids[@]}"; do
    pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d '[:space:]' || true)"
    [[ "${pgid}" == "${pid}" ]] && kill -INT -- "${pid}" 2>/dev/null || true
  done
  for round in 1 2 3 4 5; do
    alive=false
    for pid in "${managed_pids[@]}"; do kill -0 "${pid}" 2>/dev/null && alive=true; done
    [[ "${alive}" == false ]] && break
    sleep 1
  done
  for pid in "${managed_pids[@]}"; do
    kill -TERM -- "-${pid}" 2>/dev/null || true
  done
  for round in 1 2 3 4 5; do
    alive=false
    for pid in "${managed_pids[@]}"; do pgrep -g "${pid}" >/dev/null && alive=true; done
    [[ "${alive}" == false ]] && break
    sleep 1
  done
  for pid in "${managed_pids[@]}"; do
    pgrep -g "${pid}" >/dev/null && kill -KILL -- "-${pid}" 2>/dev/null || true
  done
  for pid in "${managed_pids[@]}"; do wait "${pid}" 2>/dev/null || true; done
  managed_pids=()
}
cleanup() {
  local status=$?
  trap - EXIT INT TERM
  set +e
  stop_groups
  exit "${status}"
}
trap cleanup EXIT INT TERM

bash "${script_dir}/audit_external_assets.sh" snapshot \
  "${result_dir}/external-assets.before.sha256" >/dev/null
calibration_sha="$(sha256sum "${calibration}" | cut -d ' ' -f 1)"
{
  echo "run_started_utc=${timestamp}"
  echo "scope=P10_ROS_CONTRACT_NOT_FORMAL_FLIGHT_GATE"
  echo "ros_domain_id=${ROS_DOMAIN_ID}"
  echo "p7_calibration_sha256=${calibration_sha}"
  echo "ground_truth_subscribed=false"
  echo "hard_constraint=true"
} >"${result_dir}/run.env"

start_group "${result_dir}/rosbag.log" ros2 bag record \
  --compression-mode file --compression-format zstd -o "${result_dir}/rosbag" \
  /planning/p10/baseline_bspline /planning/active_perception_candidates \
  /planning/active_perception_bspline /integrity/information_map \
  /integrity/directional /integrity/active_perception_decision \
  /integrity/active_perception_debug /xq/p5/cloud_map
start_group "${result_dir}/launch.log" ros2 launch xq_sim_bringup \
  xq_p10_contract_gate.launch.py result_file:="${result_file}" \
  calibration_file:="${calibration}" calibration_sha256:="${calibration_sha}"

deadline=$((SECONDS + 30))
status="IN_PROGRESS"
while ((SECONDS < deadline)); do
  if [[ -s "${result_file}" ]]; then
    status="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["status"])' "${result_file}")"
    [[ "${status}" == PASS ]] && break
    [[ "${status}" == FAIL ]] && { cat "${result_file}" >&2; exit 9; }
  fi
  sleep 0.25
done
[[ "${status}" == PASS ]] || { echo "P10 contract Gate timed out." >&2; exit 10; }

ros2 node info /xq_p10_active_perception >"${result_dir}/selector-node-graph.txt"
subscriber_block="$(sed -n '/Subscribers:/,/Publishers:/p' "${result_dir}/selector-node-graph.txt")"
if grep -q '/xq/eval/\|ground_truth' <<<"${subscriber_block}"; then
  echo "Ground Truth leaked into the P10 selector graph." >&2
  exit 11
fi

stop_groups
bash "${script_dir}/audit_external_assets.sh" snapshot \
  "${result_dir}/external-assets.after.sha256" >/dev/null
bash "${script_dir}/audit_external_assets.sh" compare \
  "${result_dir}/external-assets.before.sha256" \
  "${result_dir}/external-assets.after.sha256" | tee "${result_dir}/isolation-audit.txt"
[[ -f "${result_dir}/rosbag/metadata.yaml" ]] || { echo "P10 contract rosbag missing." >&2; exit 12; }

echo "PASS: P10 online ROS contract Gate completed (not formal flight Gate)."
echo "Results: ${result_dir}"
