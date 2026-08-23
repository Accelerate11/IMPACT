#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
workspace_root="$(cd -- "${script_dir}/.." && pwd -P)"
default_run="${workspace_root}/experiments/results/impact_p8/p8_gazebo_final_20260823T104800Z"
run_dir="$(realpath -e -- "${1:-${default_run}}")"

[[ -f "${run_dir}/summary.json" ]] || {
  echo "P8 summary missing: ${run_dir}/summary.json" >&2
  exit 2
}
[[ -f "${run_dir}/rosbag/metadata.yaml" ]] || {
  echo "P8 rosbag missing: ${run_dir}/rosbag/metadata.yaml" >&2
  exit 2
}
[[ -d "${run_dir}/gz_record" ]] || {
  echo "P8 Gazebo recording missing: ${run_dir}/gz_record" >&2
  exit 2
}

python3 - "${run_dir}/summary.json" <<'PY'
import json, sys
status = json.load(open(sys.argv[1], encoding="utf-8")).get("status")
if status != "PASS":
    raise SystemExit(f"P8 result is not PASS: status={status!r}")
PY

for required in DISPLAY WAYLAND_DISPLAY XDG_RUNTIME_DIR; do
  [[ -n "${!required:-}" ]] || {
    echo "WSLg variable ${required} is empty. Run this command inside Ubuntu-22.04 under WSLg." >&2
    exit 3
  }
done

declare -a viewer_pids=()
cleanup() {
  local status=$? pid pgid
  trap - EXIT INT TERM
  set +e
  for pid in "${viewer_pids[@]}"; do
    pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d '[:space:]')"
    [[ -n "${pgid}" ]] && kill -INT -- "-${pgid}" 2>/dev/null
  done
  sleep 1
  for pid in "${viewer_pids[@]}"; do
    pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d '[:space:]')"
    [[ -n "${pgid}" ]] && kill -TERM -- "-${pgid}" 2>/dev/null
  done
  for pid in "${viewer_pids[@]}"; do wait "${pid}" 2>/dev/null; done
  exit "${status}"
}
trap cleanup EXIT INT TERM

setsid bash "${script_dir}/view_p8_gazebo_replay.sh" "${run_dir}" &
viewer_pids+=("$!")
setsid bash "${script_dir}/view_p8_rviz.sh" "${run_dir}" &
viewer_pids+=("$!")

echo "P8 dual visualization: ${run_dir}"
echo "Opening Gazebo and RViz. Close either window or press Ctrl+C to stop both."

while :; do
  for pid in "${viewer_pids[@]}"; do
    kill -0 "${pid}" 2>/dev/null || exit 0
  done
  sleep 1
done
