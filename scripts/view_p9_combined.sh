#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
workspace_root="$(cd -- "${script_dir}/.." && pwd -P)"
default_run="${workspace_root}/experiments/results/impact_p9/p9_20260823T135112Z_1110615"
run_dir="$(realpath -e -- "${1:-${default_run}}")"

python3 - "${run_dir}/summary.json" <<'PY'
import json, sys
if json.load(open(sys.argv[1], encoding="utf-8")).get("status") != "PASS":
    raise SystemExit("P9 replay requires a PASS result")
PY
for required in DISPLAY WAYLAND_DISPLAY XDG_RUNTIME_DIR; do
  [[ -n "${!required:-}" ]] || { echo "WSLg variable ${required} is empty." >&2; exit 3; }
done

declare -a pids=()
cleanup() {
  local status=$? pid pgid; trap - EXIT INT TERM; set +e
  for pid in "${pids[@]}"; do
    pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d '[:space:]')"
    [[ -n "${pgid}" ]] && kill -INT -- "-${pgid}" 2>/dev/null
  done
  sleep 1
  for pid in "${pids[@]}"; do
    pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d '[:space:]')"
    [[ -n "${pgid}" ]] && kill -TERM -- "-${pgid}" 2>/dev/null
  done
  for pid in "${pids[@]}"; do wait "${pid}" 2>/dev/null; done
  exit "${status}"
}
trap cleanup EXIT INT TERM
setsid bash "${script_dir}/view_p9_gazebo_replay.sh" "${run_dir}" & pids+=("$!")
setsid bash "${script_dir}/view_p9_rviz.sh" "${run_dir}" & pids+=("$!")
echo "Opening P9 Gazebo + RViz dual replay. Close either window or press Ctrl+C."
while :; do
  for pid in "${pids[@]}"; do kill -0 "${pid}" 2>/dev/null || exit 0; done
  sleep 1
done
