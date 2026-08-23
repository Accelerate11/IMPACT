#!/usr/bin/env bash
set -euo pipefail

workspace_root="/home/accelerate/xuanqiong_x1_sim_ws"
if [[ "${XQ_P8_REPLAY:-0}" == 1 ]]; then
  default_run="${workspace_root}/experiments/results/impact_p8/p8_20260823T091322Z_57082"
elif [[ "${XQ_P7_REPLAY:-0}" == 1 ]]; then
  default_run="${workspace_root}/experiments/results/impact_p7/p7_20260823T082157Z_48605"
elif [[ "${XQ_P6_REPLAY:-0}" == 1 ]]; then
  default_run="${workspace_root}/experiments/results/impact_p6/p6_20260823T065905Z_36386"
else
  default_run="${workspace_root}/experiments/results/baseline_v1/p5_20260823T042657Z_29091"
fi
run_dir="${1:-${default_run}}"
if [[ "${XQ_P7_REPLAY:-0}" == 1 ]]; then
  bag="${run_dir}/test_structured_room/rosbag"
else
  bag="${run_dir}/rosbag"
fi

[[ -f "${bag}/metadata.yaml" ]] || {
  echo "P5 rosbag not found: ${bag}" >&2
  echo "Usage: $0 [P5_RESULT_DIRECTORY]" >&2
  exit 2
}
[[ -f "${workspace_root}/config/p5_replay.rviz" ]] || {
  echo "P5 RViz config is missing: ${workspace_root}/config/p5_replay.rviz" >&2
  echo "Run scripts/sync_to_wsl.ps1 from Windows, then retry." >&2
  exit 2
}

unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
set +u
source /opt/ros/humble/setup.bash
source "${workspace_root}/xq_install/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-97}"
export ROS_LOCALHOST_ONLY=1
export ROS2CLI_NO_DAEMON=1

declare -a helper_pids=()
cleanup() {
  local pid pgid
  trap - EXIT INT TERM
  set +e
  for pid in "${helper_pids[@]}"; do
    pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d '[:space:]')"
    [[ "${pgid}" == "${pid}" ]] && kill -INT -- "-${pgid}" 2>/dev/null
  done
  for pid in "${helper_pids[@]}"; do wait "${pid}" 2>/dev/null; done
}
trap cleanup EXIT INT TERM

log_prefix="/tmp/xq_p5_rviz_${ROS_DOMAIN_ID}"

# The recorded /clock contains occasional fine-grained out-of-order callbacks.
# Remap it and publish only monotonic samples so RViz never repeatedly resets.
setsid python3 "${workspace_root}/scripts/xq_clock_sanitizer.py" \
  >"${log_prefix}_clock.log" 2>&1 < /dev/null &
helper_pids+=("$!")

if [[ "${XQ_P8_REPLAY:-0}" == 1 ]]; then
  setsid python3 "${workspace_root}/scripts/xq_p8_replay_visualizer.py" \
    --ros-args -p use_sim_time:=true \
    >"${log_prefix}_p8_visualizer.log" 2>&1 < /dev/null &
  helper_pids+=("$!")
elif [[ "${XQ_P7_REPLAY:-0}" == 1 ]]; then
  [[ -f "${run_dir}/p7-calibration.json" ]] || {
    echo "P7 calibration not found: ${run_dir}/p7-calibration.json" >&2; exit 2;
  }
  setsid python3 "${workspace_root}/scripts/xq_p6_replay_visualizer.py" \
    --ros-args -p use_sim_time:=true -p visual_scale:=8.0 \
    -p calibration_file:="${run_dir}/p7-calibration.json" \
    >"${log_prefix}_p7_visualizer.log" 2>&1 < /dev/null &
  helper_pids+=("$!")
elif [[ "${XQ_P6_REPLAY:-0}" == 1 ]]; then
  setsid python3 "${workspace_root}/scripts/xq_p6_replay_visualizer.py" \
    --ros-args -p use_sim_time:=true -p visual_scale:=20.0 \
    >"${log_prefix}_p6_visualizer.log" 2>&1 < /dev/null &
  helper_pids+=("$!")
fi

# FAST-LIO odometry is the missing dynamic TF in the Gate bag.  The helper also
# accumulates LIO/truth Paths and publishes a visible vehicle arrow.
setsid python3 "${workspace_root}/scripts/xq_p5_replay_visualizer.py" \
  --ros-args -p use_sim_time:=true \
  >"${log_prefix}_visualizer.log" 2>&1 < /dev/null &
helper_pids+=("$!")

setsid ros2 run tf2_ros static_transform_publisher \
  --x 0.04 --y 0.0 --z 0.12 --roll 0.0 --pitch 0.0 --yaw 0.0 \
  --frame-id livox_imu --child-frame-id livox_frame \
  --ros-args -p use_sim_time:=true \
  >"${log_prefix}_static_tf.log" 2>&1 < /dev/null &
helper_pids+=("$!")

# Do not use `ros2 bag play --clock`: the bag already contains /clock and a
# second clock publisher is exactly what causes RViz's jump-back reset loop.
setsid ros2 bag play "${bag}" --rate 0.5 --delay 4 \
  --remap /clock:=/xq/recorded_clock /tf_static:=/xq/recorded_tf_static \
  >"${log_prefix}_bag.log" 2>&1 < /dev/null &
helper_pids+=("$!")

echo "P5/P6/P7/P8 RViz replay: ${run_dir}"
echo "Close RViz to stop this replay cleanly."
/opt/ros/humble/bin/rviz2 \
  -d "${workspace_root}/config/p5_replay.rviz" \
  --ros-args -p use_sim_time:=true
