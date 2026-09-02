#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
workspace_root="$(cd -- "${script_dir}/.." && pwd -P)"
default_run="$(ls -td "${workspace_root}"/experiments/results/impact_p11/visual_* | head -1)"
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
setsid bash "${script_dir}/view_p11_gazebo_replay.sh" "${run_dir}" & pids+=("$!")
setsid bash "${script_dir}/view_p11_rviz.sh" "${run_dir}" & pids+=("$!")
echo "Opening P11 Gazebo + RViz dual replay. Close either window or press Ctrl+C."
while :; do
  for pid in "${pids[@]}"; do kill -0 "${pid}" 2>/dev/null || exit 0; done
  sleep 1
done
