#!/usr/bin/env bash
set -euo pipefail

workspace_root="/home/accelerate/xuanqiong_x1_sim_ws"
scenario="${1:-structured_room}"
case "${scenario}" in
  structured_room)
    bag="${workspace_root}/experiments/results/localization/p3_structured_room_20260822T142413Z_6016/rosbag"
    ;;
  long_corridor)
    bag="${workspace_root}/experiments/results/localization/p3_long_corridor_20260822T142613Z_6965/rosbag"
    ;;
  *)
    echo "Usage: $0 [structured_room|long_corridor]" >&2
    exit 2
    ;;
esac

set +u
source /opt/ros/humble/setup.bash
source "${workspace_root}/xq_install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-96}"
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

setsid python3 "${workspace_root}/scripts/xq_clock_sanitizer.py" \
  >"/tmp/xq_p3_rviz_clock_${ROS_DOMAIN_ID}.log" 2>&1 < /dev/null &
helper_pids+=("$!")

setsid ros2 run tf2_ros static_transform_publisher \
  --x 0.04 --y 0.0 --z 0.12 --roll 0.0 --pitch 0.0 --yaw 0.0 \
  --frame-id livox_imu --child-frame-id livox_frame \
  >"/tmp/xq_p3_rviz_static_${ROS_DOMAIN_ID}.log" 2>&1 < /dev/null &
helper_pids+=("$!")

# Preserve simulated time while filtering fine-grained out-of-order recorded
# clock callbacks, and replace the conflicting static TF with the valid direct
# IMU-to-LiDAR extrinsic used by FAST-LIO.
setsid ros2 bag play "${bag}" --rate 0.5 --delay 3 \
  --remap /clock:=/xq/recorded_clock /tf_static:=/xq/recorded_tf_static \
  >"/tmp/xq_p3_rviz_bag_${ROS_DOMAIN_ID}.log" 2>&1 < /dev/null &
helper_pids+=("$!")

/opt/ros/humble/bin/rviz2 \
  -d "${workspace_root}/config/p3_replay.rviz" \
  --ros-args -p use_sim_time:=true
