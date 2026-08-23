#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
workspace_root="$(cd -- "${script_dir}/.." && pwd -P)"
default_run="${workspace_root}/experiments/results/impact_p9/p9_20260823T135112Z_1110615"
run_dir="$(realpath -e -- "${1:-${default_run}}")"
[[ -f "${run_dir}/summary.json" ]] || { echo "P9 summary missing: ${run_dir}" >&2; exit 2; }
[[ -f "${run_dir}/rosbag/metadata.yaml" ]] || { echo "P9 rosbag missing: ${run_dir}" >&2; exit 2; }

unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
set +u
source /opt/ros/humble/setup.bash
source "${workspace_root}/xq_install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-109}"
export ROS_LOCALHOST_ONLY=1
export ROS2CLI_NO_DAEMON=1

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

setsid ros2 run xq_autonomy xq_p9_replay_visualizer --ros-args -p use_sim_time:=true \
  >"${run_dir}/rviz-visualizer.log" 2>&1 & pids+=("$!")
setsid rviz2 -d "${workspace_root}/config/p5_replay.rviz" --ros-args -p use_sim_time:=true \
  >"${run_dir}/rviz.log" 2>&1 & rviz_pid=$!; pids+=("${rviz_pid}")
sleep 3
setsid ros2 bag play "${run_dir}/rosbag" --clock \
  >"${run_dir}/rviz-bag-play.log" 2>&1 & pids+=("$!")

echo "RViz P9 replay: green wide-room ACCEPT, red narrow-passage REJECT."
echo "Close RViz or press Ctrl+C to stop."
wait "${rviz_pid}" || true
