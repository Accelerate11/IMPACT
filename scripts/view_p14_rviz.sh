#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
workspace_root="$(cd -- "${script_dir}/.." && pwd -P)"
default_run="$(for candidate in $(ls -td "${workspace_root}"/experiments/results/impact_p14/gate_* 2>/dev/null); do
  [[ -f "${candidate}/p14-gate-result.json" ]] || continue
  [[ "$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["status"])' "${candidate}/p14-gate-result.json")" == PASS ]] && { echo "${candidate}"; break; }
done)"
run_dir="$(realpath -e -- "${1:-${default_run}}")"
bag="${run_dir}/matrix/rosbag"
[[ -f "${bag}/metadata.yaml" ]] || { echo "P14 matrix rosbag missing: ${bag}" >&2; exit 2; }
[[ -f "${workspace_root}/config/p14_replay.rviz" ]] || { echo "P14 RViz config missing." >&2; exit 2; }
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
set +u; source /opt/ros/humble/setup.bash; source "${workspace_root}/xq_install/setup.bash"; set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-131}"
export ROS_LOCALHOST_ONLY=1 ROS2CLI_NO_DAEMON=1
declare -a pids=()
cleanup() {
  local status=$? pid; trap - EXIT INT TERM; set +e
  for pid in "${pids[@]}"; do kill -INT -- "-${pid}" 2>/dev/null || true; done
  sleep 1
  for pid in "${pids[@]}"; do kill -TERM -- "-${pid}" 2>/dev/null || true; done
  for pid in "${pids[@]}"; do wait "${pid}" 2>/dev/null || true; done
  exit "${status}"
}
trap cleanup EXIT INT TERM
log_prefix="/tmp/xq_p14_rviz_${ROS_DOMAIN_ID}"
setsid python3 "${workspace_root}/scripts/xq_clock_sanitizer.py" >"${log_prefix}_clock.log" 2>&1 < /dev/null & pids+=("$!")
setsid python3 "${workspace_root}/scripts/xq_p5_replay_visualizer.py" --ros-args -p use_sim_time:=true >"${log_prefix}_vehicle.log" 2>&1 < /dev/null & pids+=("$!")
setsid ros2 run xq_autonomy xq_p11_replay_visualizer --ros-args -p use_sim_time:=true >"${log_prefix}_planning.log" 2>&1 < /dev/null & pids+=("$!")
setsid ros2 run xq_autonomy xq_p13_replay_visualizer --ros-args -p use_sim_time:=true >"${log_prefix}_latency.log" 2>&1 < /dev/null & pids+=("$!")
setsid ros2 run impact_fault_injection impact_p14_visualizer --ros-args -p use_sim_time:=true >"${log_prefix}_fault.log" 2>&1 < /dev/null & pids+=("$!")
setsid ros2 bag play "${bag}" --rate 0.5 --delay 4 --remap /clock:=/xq/recorded_clock /tf_static:=/xq/recorded_tf_static >"${log_prefix}_bag.log" 2>&1 < /dev/null & pids+=("$!")
echo "P14 RViz replay: ${run_dir}/matrix"
echo "Color halo/text = resilient state; bottom ladder highlights NORMAL→...→LAND."
/opt/ros/humble/bin/rviz2 -d "${workspace_root}/config/p14_replay.rviz" --ros-args -p use_sim_time:=true
