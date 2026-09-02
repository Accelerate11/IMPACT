#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
workspace_root="$(cd -- "${script_dir}/.." && pwd -P)"
default_run="$(for candidate in $(ls -td "${workspace_root}"/experiments/results/impact_p12/gate_* 2>/dev/null); do
  [[ -f "${candidate}/flight-result.json" ]] || continue
  [[ "$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["status"])' "${candidate}/flight-result.json")" == PASS ]] && { echo "${candidate}"; break; }
done)"
run_dir="$(realpath -e -- "${1:-${default_run}}")"
bag="${run_dir}/rosbag"
[[ -f "${bag}/metadata.yaml" ]] || { echo "P12 rosbag missing: ${bag}" >&2; exit 2; }
[[ -f "${workspace_root}/config/p12_replay.rviz" ]] || { echo "P12 RViz config missing." >&2; exit 2; }

unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
set +u
source /opt/ros/humble/setup.bash
source "${workspace_root}/xq_install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-98}"
export ROS_LOCALHOST_ONLY=1
export ROS2CLI_NO_DAEMON=1

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
log_prefix="/tmp/xq_p12_rviz_${ROS_DOMAIN_ID}"

setsid python3 "${workspace_root}/scripts/xq_clock_sanitizer.py" \
  >"${log_prefix}_clock.log" 2>&1 < /dev/null & pids+=("$!")
setsid python3 "${workspace_root}/scripts/xq_p5_replay_visualizer.py" \
  --ros-args -p use_sim_time:=true \
  >"${log_prefix}_vehicle.log" 2>&1 < /dev/null & pids+=("$!")
setsid ros2 run xq_autonomy xq_p11_replay_visualizer --ros-args -p use_sim_time:=true \
  >"${log_prefix}_planning.log" 2>&1 < /dev/null & pids+=("$!")
setsid ros2 bag play "${bag}" --rate 0.5 --delay 4 \
  --remap /clock:=/xq/recorded_clock /tf_static:=/xq/recorded_tf_static \
  >"${log_prefix}_bag.log" 2>&1 < /dev/null & pids+=("$!")

echo "P12 RViz replay: ${run_dir}"
echo "Cyan=static structure, red=LiDAR dynamic voxels, green=selected trajectory."
echo "Close RViz to stop replay cleanly."
/opt/ros/humble/bin/rviz2 -d "${workspace_root}/config/p12_replay.rviz" \
  --ros-args -p use_sim_time:=true
