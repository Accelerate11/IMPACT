#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
workspace_root="$(cd -- "${script_dir}/.." && pwd -P)"
timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
result_dir="$(realpath -m -- "${1:-${workspace_root}/experiments/results/impact_p11/contract_${timestamp}_$$}")"
case "${result_dir}" in
  "${workspace_root}"/experiments/results/impact_p11/*) ;;
  *) echo "P11 contract results must stay under this workspace." >&2; exit 2 ;;
esac
mkdir -p -- "${result_dir}/ros_logs"
calibration="${workspace_root}/evidence/P7/p7-calibration.json"
[[ -f "${calibration}" ]] || { echo "P7 calibration missing." >&2; exit 2; }

unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
unset GZ_SIM_RESOURCE_PATH IGN_GAZEBO_RESOURCE_PATH SDF_PATH
set +u
source /opt/ros/humble/setup.bash
source "${workspace_root}/xq_install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-126}"
export ROS_LOCALHOST_ONLY=1
export ROS2CLI_NO_DAEMON=1
export ROS_LOG_DIR="${result_dir}/ros_logs"

declare -a pids=()
stop_groups() {
  local pid round alive
  for pid in "${pids[@]}"; do kill -INT -- "-${pid}" 2>/dev/null || true; done
  for round in 1 2 3 4 5; do
    alive=false
    for pid in "${pids[@]}"; do pgrep -g "${pid}" >/dev/null && alive=true; done
    [[ "${alive}" == false ]] && break
    sleep 1
  done
  for pid in "${pids[@]}"; do kill -TERM -- "-${pid}" 2>/dev/null || true; done
  for pid in "${pids[@]}"; do wait "${pid}" 2>/dev/null || true; done
  pids=()
}
cleanup() { local status=$?; trap - EXIT INT TERM; set +e; stop_groups; exit "${status}"; }
trap cleanup EXIT INT TERM

bash "${script_dir}/audit_external_assets.sh" snapshot \
  "${result_dir}/external-assets.before.sha256" >/dev/null
calibration_sha="$(sha256sum "${calibration}" | cut -d ' ' -f 1)"
{
  echo "scope=P11_ROS_CONTRACT_NOT_FORMAL_FLIGHT_GATE"
  echo "ground_truth_subscribed=false"
  echo "hard_constraint=true"
  echo "margin_in_utility=false"
  echo "calibration_sha256=${calibration_sha}"
} >"${result_dir}/run.env"

setsid ros2 bag record --compression-mode file --compression-format zstd \
  -o "${result_dir}/rosbag" \
  /planning/p11/frontier_candidate_set /planning/p11/frontier_candidates \
  /planning/p11/selected_bspline /planning/p11/unconstrained_bspline \
  /integrity/exploration_decision /integrity/exploration_debug \
  /integrity/information_map /integrity/directional /xq/p5/cloud_map \
  >"${result_dir}/rosbag.log" 2>&1 < /dev/null & pids+=("$!")
setsid ros2 launch xq_sim_bringup xq_p11_contract_gate.launch.py \
  result_file:="${result_dir}/contract-result.json" \
  calibration_file:="${calibration}" calibration_sha256:="${calibration_sha}" \
  >"${result_dir}/launch.log" 2>&1 < /dev/null & pids+=("$!")

deadline=$((SECONDS + 30))
status=IN_PROGRESS
while ((SECONDS < deadline)); do
  if [[ -s "${result_dir}/contract-result.json" ]]; then
    status="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["status"])' "${result_dir}/contract-result.json")"
    [[ "${status}" == PASS ]] && break
    if [[ "${status}" == FAIL ]]; then
      cat "${result_dir}/contract-result.json" >&2
      tail -n 100 "${result_dir}/launch.log" >&2 || true
      exit 9
    fi
  fi
  sleep 0.25
done
[[ "${status}" == PASS ]] || { echo "P11 contract Gate timed out." >&2; exit 10; }

ros2 node info /xq_p11_integrity_exploration >"${result_dir}/selector-node-graph.txt"
subscribers="$(sed -n '/Subscribers:/,/Publishers:/p' "${result_dir}/selector-node-graph.txt")"
if grep -q '/xq/eval/\|ground_truth' <<<"${subscribers}"; then
  echo "Ground Truth leaked into P11 selector." >&2; exit 11
fi
stop_groups
bash "${script_dir}/audit_external_assets.sh" snapshot \
  "${result_dir}/external-assets.after.sha256" >/dev/null
bash "${script_dir}/audit_external_assets.sh" compare \
  "${result_dir}/external-assets.before.sha256" \
  "${result_dir}/external-assets.after.sha256" >"${result_dir}/isolation-audit.txt"
[[ -f "${result_dir}/rosbag/metadata.yaml" ]] || { echo "P11 contract rosbag missing." >&2; exit 12; }
echo "PASS: P11 online ROS hard-constraint contract Gate completed."
echo "Results: ${result_dir}"
