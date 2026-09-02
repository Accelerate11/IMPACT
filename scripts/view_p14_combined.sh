#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
workspace_root="$(cd -- "${script_dir}/.." && pwd -P)"
default_run="$(for candidate in $(ls -td "${workspace_root}"/experiments/results/impact_p14/gate_* 2>/dev/null); do
  [[ -f "${candidate}/p14-gate-result.json" ]] || continue
  [[ "$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["status"])' "${candidate}/p14-gate-result.json")" == PASS ]] && { echo "${candidate}"; break; }
done)"
run_dir="$(realpath -e -- "${1:-${default_run}}")"
for required in DISPLAY WAYLAND_DISPLAY XDG_RUNTIME_DIR; do
  [[ -n "${!required:-}" ]] || { echo "WSLg variable ${required} is empty." >&2; exit 3; }
done
declare -a pids=()
cleanup() {
  local status=$? pid; trap - EXIT INT TERM; set +e
  for pid in "${pids[@]}"; do kill -INT -- "-${pid}" 2>/dev/null || true; done
  sleep 1
  for pid in "${pids[@]}"; do kill -TERM -- "-${pid}" 2>/dev/null || true; done
  exit "${status}"
}
trap cleanup EXIT INT TERM
setsid bash "${script_dir}/view_p14_gazebo_replay.sh" "${run_dir}" & pids+=("$!")
setsid bash "${script_dir}/view_p14_rviz.sh" "${run_dir}" & pids+=("$!")
echo "Opening accepted P14 Gazebo + RViz replay. Press Ctrl+C to close both."
while :; do
  for pid in "${pids[@]}"; do kill -0 "${pid}" 2>/dev/null || exit 0; done
  sleep 1
done
