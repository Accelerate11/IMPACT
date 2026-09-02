#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
workspace_root="$(cd -- "${script_dir}/.." && pwd -P)"
timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
result_dir="$(realpath -m -- "${1:-${workspace_root}/experiments/results/impact_p12/gate_${timestamp}_$$}")"
case "${result_dir}" in
  "${workspace_root}"/experiments/results/impact_p12/*) ;;
  *) echo "P12 Gate results must stay under this workspace." >&2; exit 2 ;;
esac
mkdir -p -- "${result_dir}/configuration" "${result_dir}/ros_logs"

for required in \
  "${workspace_root}/xq_install/setup.bash" \
  "${workspace_root}/xq_install/.xq_build_manifest.json" \
  "${workspace_root}/evidence/P7/p7-calibration.json" \
  "${workspace_root}/config/p12_gate_thresholds.json" \
  "${workspace_root}/scripts/audit_external_assets.sh"; do
  [[ -f "${required}" ]] || { echo "Missing P12 Gate dependency: ${required}" >&2; exit 2; }
done

unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
unset GZ_SIM_RESOURCE_PATH IGN_GAZEBO_RESOURCE_PATH SDF_PATH
set +u
source /opt/ros/humble/setup.bash
source "${workspace_root}/xq_install/setup.bash"
set -u
world="${workspace_root}/xq_install/xq_gz_assets/share/xq_gz_assets/worlds/xq_p12_dynamic_obstacle.sdf"
calibration="${workspace_root}/evidence/P7/p7-calibration.json"
thresholds="${workspace_root}/config/p12_gate_thresholds.json"
[[ -f "${world}" ]] || { echo "Installed P12 world missing." >&2; exit 2; }
value() { python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))[sys.argv[2]])' "${thresholds}" "$1"; }
calibration_sha="$(sha256sum "${calibration}" | cut -d ' ' -f 1)"

cp -- "${thresholds}" "${result_dir}/configuration/p12_gate_thresholds.json"
cp -- "${world}" "${result_dir}/configuration/xq_p12_dynamic_obstacle.sdf"
cp -- "${calibration}" "${result_dir}/configuration/p7-calibration.json"
cp -- "${workspace_root}/xq_install/.xq_build_manifest.json" "${result_dir}/configuration/build-manifest.json"
for source in \
  src/xq_sim_bringup/launch/xq_p12_flight.launch.py \
  src/xq_autonomy/xq_autonomy/dynamic_voxel_map.py \
  src/xq_autonomy/xq_autonomy/dynamic_planning.py \
  src/xq_autonomy/xq_autonomy/p12_dynamic_map_node.py \
  src/xq_autonomy/xq_autonomy/p12_flight_controller_node.py \
  src/xq_autonomy/xq_autonomy/p12_flight_evaluator_node.py \
  src/xq_autonomy/xq_autonomy/p12_obstacle_driver_node.py; do
  cp -- "${workspace_root}/${source}" "${result_dir}/configuration/$(basename "${source}")"
done
sha256sum "${result_dir}"/configuration/* >"${result_dir}/configuration.sha256"

bash "${script_dir}/audit_external_assets.sh" snapshot \
  "${result_dir}/external-assets.before.sha256" >/dev/null
audit_done=false
declare -a pids=()
stop_groups() {
  local pid round alive
  for pid in "${pids[@]}"; do kill -INT -- "-${pid}" 2>/dev/null || true; done
  for round in 1 2 3 4 5 6 7 8 9 10; do
    alive=false
    for pid in "${pids[@]}"; do pgrep -g "${pid}" >/dev/null && alive=true; done
    [[ "${alive}" == false ]] && break
    sleep 1
  done
  for pid in "${pids[@]}"; do kill -TERM -- "-${pid}" 2>/dev/null || true; done
  for round in 1 2 3 4 5; do
    alive=false
    for pid in "${pids[@]}"; do pgrep -g "${pid}" >/dev/null && alive=true; done
    [[ "${alive}" == false ]] && break
    sleep 1
  done
  for pid in "${pids[@]}"; do pgrep -g "${pid}" >/dev/null && kill -KILL -- "-${pid}" 2>/dev/null || true; done
  for pid in "${pids[@]}"; do wait "${pid}" 2>/dev/null || true; done
  pids=()
}
finish_audit() {
  [[ "${audit_done}" == false ]] || return 0
  bash "${script_dir}/audit_external_assets.sh" snapshot "${result_dir}/external-assets.after.sha256" >/dev/null
  bash "${script_dir}/audit_external_assets.sh" compare \
    "${result_dir}/external-assets.before.sha256" "${result_dir}/external-assets.after.sha256" \
    >"${result_dir}/isolation-audit.txt"
  audit_done=true
}
cleanup() { local status=$?; trap - EXIT INT TERM; set +e; stop_groups; finish_audit; exit "${status}"; }
trap cleanup EXIT INT TERM

export ROS_DOMAIN_ID=$((190 + ($$ % 20)))
export ROS_LOCALHOST_ONLY=1
export ROS2CLI_NO_DAEMON=1
export GZ_PARTITION="xq_p12_gate_${timestamp}_$$"
export ROS_LOG_DIR="${result_dir}/ros_logs"
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe
export GALLIUM_DRIVER=llvmpipe
export EGL_PLATFORM=surfaceless
export QT_QPA_PLATFORM=offscreen
{
  echo "ros_domain_id=${ROS_DOMAIN_ID}"
  echo "gz_partition=${GZ_PARTITION}"
  echo "world_sha256=$(sha256sum "${world}" | cut -d ' ' -f 1)"
  echo "calibration_sha256=${calibration_sha}"
  echo "ground_truth_policy=evaluator_and_scenario_only"
  echo "dynamic_detection_input=livox_lidar_plus_localization_odom"
  echo "roof=removed"
  echo "walls=preserved"
} >"${result_dir}/run.env"

setsid ros2 bag record --compression-mode file --compression-format zstd \
  -o "${result_dir}/rosbag" \
  /livox/lidar /livox/imu /localization/odom /localization/geometry \
  /cloud_registered /mapping/p12/dynamic_voxels /mapping/p12/static_voxels \
  /mapping/p12/occupancy /mapping/p12/status /planning/p12/replan_event \
  /integrity/directional /integrity/information_map /integrity/exploration_decision \
  /planning/p11/frontier_candidate_set /planning/p11/frontier_candidates \
  /planning/p11/selected_bspline /xq/p12/flight_status \
  /xq/eval/p12/obstacle_state /xq/eval/agent_01/ground_truth /tf /tf_static /clock \
  >"${result_dir}/rosbag.log" 2>&1 < /dev/null & pids+=("$!")
setsid ros2 launch xq_sim_bringup xq_p12_flight.launch.py \
  world_file:="${world}" calibration_file:="${calibration}" \
  calibration_sha256:="${calibration_sha}" thresholds_file:="${thresholds}" \
  result_file:="${result_dir}/flight-result.json" \
  gz_record_path:="${result_dir}/gz_record" \
  voxel_size_m:="$(value voxel_size_m)" \
  dynamic_ttl_s:="$(value dynamic_ttl_s)" \
  dynamic_occupied_threshold:="$(value dynamic_occupied_threshold)" \
  dynamic_clear_threshold:="$(value dynamic_clear_threshold)" \
  static_confirmation_hits:="$(value static_confirmation_hits)" \
  free_confirmation_rays:="$(value free_confirmation_rays)" \
  path_clearance_radius_m:="$(value path_clearance_radius_m)" \
  planning_lookahead_m:="$(value planning_lookahead_m)" \
  clear_confirmation_s:="$(value clear_confirmation_s)" \
  mission_distance_m:="$(value mission_distance_m)" \
  >"${result_dir}/launch.log" 2>&1 < /dev/null & pids+=("$!")
launch_pid="${pids[1]}"

deadline=$((SECONDS + 210))
while [[ ! -s "${result_dir}/flight-result.json" ]] && ((SECONDS < deadline)); do
  kill -0 "${launch_pid}" 2>/dev/null || {
    echo "P12 launch exited early." >&2
    tail -n 180 "${result_dir}/launch.log" >&2 || true
    exit 5
  }
  sleep 1
done
[[ -s "${result_dir}/flight-result.json" ]] || {
  echo "P12 evaluator timed out." >&2
  tail -n 180 "${result_dir}/launch.log" >&2 || true
  exit 6
}

for node in xq_p12_dynamic_map xq_p12_flight_controller xq_p11_integrity_exploration; do
  ros2 node info "/${node}" >"${result_dir}/${node}-graph.txt"
  subscribers="$(sed -n '/Subscribers:/,/Publishers:/p' "${result_dir}/${node}-graph.txt")"
  if grep -q '/xq/eval/\|ground_truth\|/model/' <<<"${subscribers}"; then
    echo "Ground Truth leaked into algorithm node ${node}." >&2
    exit 8
  fi
done
stop_groups
finish_audit
[[ -f "${result_dir}/rosbag/metadata.yaml" ]] || { echo "P12 rosbag missing." >&2; exit 9; }
status="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["status"])' "${result_dir}/flight-result.json")"
if [[ "${status}" != PASS ]]; then
  cat "${result_dir}/flight-result.json" >&2
  exit 7
fi
echo "PASS: P12 LiDAR dynamic obstacle, TTL decay, online passage reopening, and full-corridor flight completed."
echo "Results: ${result_dir}"

