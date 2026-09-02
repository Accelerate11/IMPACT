#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
workspace_root="$(cd -- "${script_dir}/.." && pwd -P)"
timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
result_dir="$(realpath -m -- "${1:-${workspace_root}/experiments/results/impact_p13/gate_${timestamp}_$$}")"
case "${result_dir}" in
  "${workspace_root}"/experiments/results/impact_p13/*) ;;
  *) echo "P13 Gate results must stay under this workspace." >&2; exit 2 ;;
esac
mkdir -p -- "${result_dir}/configuration" "${result_dir}/ros_logs"

for required in \
  "${workspace_root}/xq_install/setup.bash" \
  "${workspace_root}/xq_install/.xq_build_manifest.json" \
  "${workspace_root}/evidence/P7/p7-calibration.json" \
  "${workspace_root}/config/p12_gate_thresholds.json" \
  "${workspace_root}/config/p13_gate_thresholds.json" \
  "${workspace_root}/scripts/audit_external_assets.sh"; do
  [[ -f "${required}" ]] || { echo "Missing P13 Gate dependency: ${required}" >&2; exit 2; }
done

unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
unset GZ_SIM_RESOURCE_PATH IGN_GAZEBO_RESOURCE_PATH SDF_PATH
set +u
source /opt/ros/humble/setup.bash
source "${workspace_root}/xq_install/setup.bash"
set -u
world="${workspace_root}/xq_install/xq_gz_assets/share/xq_gz_assets/worlds/xq_p12_dynamic_obstacle.sdf"
calibration="${workspace_root}/evidence/P7/p7-calibration.json"
p12_thresholds="${workspace_root}/config/p12_gate_thresholds.json"
thresholds="${workspace_root}/config/p13_gate_thresholds.json"
value() { python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))[sys.argv[2]])' "${thresholds}" "$1"; }
p12_value() { python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))[sys.argv[2]])' "${p12_thresholds}" "$1"; }
calibration_sha="$(sha256sum "${calibration}" | cut -d ' ' -f 1)"
world_sha="$(sha256sum "${world}" | cut -d ' ' -f 1)"

cp -- "${thresholds}" "${result_dir}/configuration/p13_gate_thresholds.json"
cp -- "${p12_thresholds}" "${result_dir}/configuration/p12_gate_thresholds.json"
cp -- "${world}" "${result_dir}/configuration/xq_p12_dynamic_obstacle.sdf"
cp -- "${calibration}" "${result_dir}/configuration/p7-calibration.json"
cp -- "${workspace_root}/xq_install/.xq_build_manifest.json" "${result_dir}/configuration/build-manifest.json"
for source in \
  src/xq_sim_bringup/launch/xq_p13_flight.launch.py \
  src/xq_autonomy/xq_autonomy/latency_safety.py \
  src/xq_autonomy/xq_autonomy/p13_flight_controller_node.py \
  src/xq_autonomy/xq_autonomy/p13_flight_evaluator_node.py \
  scripts/analyze_p13_gate.py; do
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

export ROS_LOCALHOST_ONLY=1
export ROS2CLI_NO_DAEMON=1
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe
export GALLIUM_DRIVER=llvmpipe
export EGL_PLATFORM=surfaceless
export QT_QPA_PLATFORM=offscreen

run_trial() {
  local profile="$1" planner_delay_ms="$2" record_gazebo="$3"
  local trial_dir="${result_dir}/${profile}" gz_record_path=""
  local -a gz_record_argument=()
  mkdir -p -- "${trial_dir}"
  export ROS_DOMAIN_ID=$((160 + ($$ % 15)))
  [[ "${profile}" == high_200ms ]] && export ROS_DOMAIN_ID=$((ROS_DOMAIN_ID + 20))
  export GZ_PARTITION="xq_p13_${profile}_${timestamp}_$$"
  export ROS_LOG_DIR="${result_dir}/ros_logs/${profile}"
  mkdir -p -- "${ROS_LOG_DIR}"
  if [[ "${record_gazebo}" == true ]]; then
    gz_record_path="${trial_dir}/gz_record"
    gz_record_argument=("gz_record_path:=${gz_record_path}")
  fi
  {
    echo "profile=${profile}"
    echo "planner_delay_ms=${planner_delay_ms}"
    echo "ros_domain_id=${ROS_DOMAIN_ID}"
    echo "gz_partition=${GZ_PARTITION}"
    echo "world_sha256=${world_sha}"
    echo "calibration_sha256=${calibration_sha}"
    echo "ground_truth_policy=evaluator_and_scenario_only"
    echo "latency_statistic=nearest_rank_p99"
    echo "roof=removed"
    echo "walls=preserved"
  } >"${trial_dir}/run.env"

  setsid ros2 bag record --compression-mode file --compression-format zstd \
    -o "${trial_dir}/rosbag" \
    /livox/lidar /livox/imu /localization/odom /localization/geometry /cloud_registered \
    /mapping/p12/dynamic_voxels /mapping/p12/static_voxels /mapping/p12/status \
    /planning/p12/replan_event /integrity/p13/latency_trace /xq/p13/flight_status \
    /integrity/directional /integrity/information_map /integrity/exploration_decision \
    /integrity/exploration_debug \
    /planning/p11/frontier_candidate_set /planning/p11/frontier_candidates \
    /planning/p11/selected_bspline /xq/p12/flight_status \
    /xq/eval/p12/obstacle_state /xq/eval/agent_01/ground_truth /tf /tf_static /clock \
    >"${trial_dir}/rosbag.log" 2>&1 < /dev/null & pids+=("$!")
  setsid ros2 launch xq_sim_bringup xq_p13_flight.launch.py \
    world_file:="${world}" calibration_file:="${calibration}" \
    calibration_sha256:="${calibration_sha}" thresholds_file:="${thresholds}" \
    p12_thresholds_file:="${p12_thresholds}" \
    p12_result_file:="${trial_dir}/p12-retention-result.json" \
    p13_result_file:="${trial_dir}/trial-result.json" \
    "${gz_record_argument[@]}" latency_profile:="${profile}" \
    planner_delay_ms:="${planner_delay_ms}" \
    voxel_size_m:="$(p12_value voxel_size_m)" \
    dynamic_ttl_s:="$(p12_value dynamic_ttl_s)" \
    dynamic_occupied_threshold:="$(p12_value dynamic_occupied_threshold)" \
    dynamic_clear_threshold:="$(p12_value dynamic_clear_threshold)" \
    static_confirmation_hits:="$(p12_value static_confirmation_hits)" \
    free_confirmation_rays:="$(p12_value free_confirmation_rays)" \
    path_clearance_radius_m:="$(p12_value path_clearance_radius_m)" \
    planning_lookahead_m:="$(p12_value planning_lookahead_m)" \
    clear_confirmation_s:="$(p12_value clear_confirmation_s)" \
    mission_distance_m:="$(value mission_distance_m)" \
    geometric_clearance_m:="$(value geometric_clearance_m)" \
    fixed_buffer_m:="$(value fixed_buffer_m)" \
    protection_level_m:="$(value protection_level_m)" \
    required_margin_m:="$(value required_margin_m)" \
    maximum_speed_mps:="$(value maximum_speed_mps)" \
    maximum_acceleration_mps2:="$(value maximum_acceleration_mps2)" \
    >"${trial_dir}/launch.log" 2>&1 < /dev/null & pids+=("$!")
  local launch_pid="${pids[1]}" deadline=$((SECONDS + 330))
  while [[ ! -s "${trial_dir}/trial-result.json" ]] && ((SECONDS < deadline)); do
    kill -0 "${launch_pid}" 2>/dev/null || {
      echo "P13 ${profile} launch exited early." >&2
      tail -n 180 "${trial_dir}/launch.log" >&2 || true
      return 5
    }
    sleep 1
  done
  [[ -s "${trial_dir}/trial-result.json" ]] || {
    echo "P13 ${profile} evaluator timed out." >&2
    tail -n 180 "${trial_dir}/launch.log" >&2 || true
    return 6
  }
  local wait_p12=$((SECONDS + 10))
  while [[ ! -s "${trial_dir}/p12-retention-result.json" ]] && ((SECONDS < wait_p12)); do sleep 1; done
  [[ -s "${trial_dir}/p12-retention-result.json" ]] || { echo "P12 retention result missing." >&2; return 6; }
  for node in xq_p13_flight_controller xq_p12_dynamic_map xq_p11_integrity_exploration; do
    ros2 node info "/${node}" >"${trial_dir}/${node}-graph.txt"
    subscribers="$(sed -n '/Subscribers:/,/Publishers:/p' "${trial_dir}/${node}-graph.txt")"
    if grep -q '/xq/eval/\|ground_truth\|/model/' <<<"${subscribers}"; then
      echo "Ground Truth leaked into algorithm node ${node}." >&2
      return 8
    fi
  done
  stop_groups
  [[ -f "${trial_dir}/rosbag/metadata.yaml" ]] || { echo "P13 ${profile} rosbag missing." >&2; return 9; }
  [[ "$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["status"])' "${trial_dir}/trial-result.json")" == PASS ]] || return 7
  [[ "$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["status"])' "${trial_dir}/p12-retention-result.json")" == PASS ]] || return 7
}

run_trial low_50ms "$(value low_planner_delay_ms)" false
run_trial high_200ms "$(value high_planner_delay_ms)" true
python3 "${script_dir}/analyze_p13_gate.py" \
  --low "${result_dir}/low_50ms/trial-result.json" \
  --high "${result_dir}/high_200ms/trial-result.json" \
  --low-p12 "${result_dir}/low_50ms/p12-retention-result.json" \
  --high-p12 "${result_dir}/high_200ms/p12-retention-result.json" \
  --thresholds "${thresholds}" --world-sha256 "${world_sha}" \
  --output "${result_dir}/p13-gate-result.json" >"${result_dir}/analysis.log"
finish_audit
echo "PASS: P13 measured p99 latency tightened AL and produced a more conservative speed envelope."
echo "Results: ${result_dir}"
