#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
workspace_root="$(cd -- "${script_dir}/.." && pwd -P)"
timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
result_dir="$(realpath -m -- "${1:-${workspace_root}/experiments/results/impact_p14/gate_${timestamp}_$$}")"
case "${result_dir}" in
  "${workspace_root}"/experiments/results/impact_p14/*) ;;
  *) echo "P14 Gate results must stay under this workspace." >&2; exit 2 ;;
esac
mkdir -p -- "${result_dir}/configuration" "${result_dir}/ros_logs"

for required in \
  "${workspace_root}/xq_install/setup.bash" \
  "${workspace_root}/xq_install/.xq_build_manifest.json" \
  "${workspace_root}/evidence/P7/p7-calibration.json" \
  "${workspace_root}/config/p12_gate_thresholds.json" \
  "${workspace_root}/config/p13_gate_thresholds.json" \
  "${workspace_root}/config/p14_gate_thresholds.json" \
  "${workspace_root}/config/p14_matrix_schedule.json" \
  "${workspace_root}/config/p14_emergency_schedule.json" \
  "${workspace_root}/scripts/audit_external_assets.sh"; do
  [[ -f "${required}" ]] || { echo "Missing P14 dependency: ${required}" >&2; exit 2; }
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
p13_thresholds="${workspace_root}/config/p13_gate_thresholds.json"
p14_thresholds="${workspace_root}/config/p14_gate_thresholds.json"
matrix_schedule="${workspace_root}/config/p14_matrix_schedule.json"
emergency_schedule="${workspace_root}/config/p14_emergency_schedule.json"
value() { python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))[sys.argv[2]])' "${p13_thresholds}" "$1"; }
p12_value() { python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))[sys.argv[2]])' "${p12_thresholds}" "$1"; }
calibration_sha="$(sha256sum "${calibration}" | cut -d ' ' -f 1)"
world_sha="$(sha256sum "${world}" | cut -d ' ' -f 1)"

for source in \
  "${p14_thresholds}" "${matrix_schedule}" "${emergency_schedule}" "${p12_thresholds}" "${p13_thresholds}" \
  "${calibration}" "${world}" "${workspace_root}/xq_install/.xq_build_manifest.json" \
  "${workspace_root}/src/xq_sim_bringup/launch/xq_p14_fault_injection.launch.py" \
  "${workspace_root}/src/impact_fault_injection/impact_fault_injection/fault_model.py" \
  "${workspace_root}/src/impact_fault_injection/impact_fault_injection/supervisor.py" \
  "${workspace_root}/src/impact_fault_injection/impact_fault_injection/sensor_proxy_node.py" \
  "${workspace_root}/src/impact_fault_injection/impact_fault_injection/p14_controller_node.py" \
  "${workspace_root}/src/impact_fault_injection/impact_fault_injection/p14_evaluator_node.py"; do
  cp -- "${source}" "${result_dir}/configuration/$(basename "${source}")"
done
sha256sum "${result_dir}"/configuration/* >"${result_dir}/configuration.sha256"

bash "${script_dir}/audit_external_assets.sh" snapshot "${result_dir}/external-assets.before.sha256" >/dev/null
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
  local trial="$1" schedule="$2" deadline_s="$3" record_gazebo="$4"
  local trial_dir="${result_dir}/${trial}" gz_record_path=""
  local -a record_arg=()
  mkdir -p -- "${trial_dir}"
  if [[ "${trial}" == matrix ]]; then
    export ROS_DOMAIN_ID=$((198 + ($$ % 8)))
  else
    export ROS_DOMAIN_ID=$((216 + ($$ % 8)))
  fi
  export GZ_PARTITION="xq_p14_${trial}_${timestamp}_$$"
  export ROS_LOG_DIR="${result_dir}/ros_logs/${trial}"
  mkdir -p -- "${ROS_LOG_DIR}"
  if [[ "${record_gazebo}" == true ]]; then
    gz_record_path="${trial_dir}/gz_record"
    record_arg=("gz_record_path:=${gz_record_path}")
  fi
  {
    echo "trial=${trial}"
    echo "schedule=$(basename "${schedule}")"
    echo "schedule_sha256=$(sha256sum "${schedule}" | cut -d ' ' -f 1)"
    echo "seed_policy=schedule_frozen"
    echo "ros_domain_id=${ROS_DOMAIN_ID}"
    echo "gz_partition=${GZ_PARTITION}"
    echo "world_sha256=${world_sha}"
    echo "calibration_sha256=${calibration_sha}"
    echo "ground_truth_policy=evaluator_only"
    echo "roof=removed"
    echo "walls=preserved"
  } >"${trial_dir}/run.env"

  setsid ros2 bag record --compression-mode file --compression-format zstd \
    -o "${trial_dir}/rosbag" \
    /localization/odom /localization/geometry /cloud_registered \
    /mapping/p12/dynamic_voxels /mapping/p12/static_voxels /mapping/p12/status \
    /planning/p12/replan_event /integrity/p13/latency_trace /xq/p13/flight_status \
    /impact/fault_event /impact/fault_proxy_status /impact/p14/safety_status \
    /integrity/directional /integrity/information_map /integrity/exploration_decision \
    /planning/p11/frontier_candidate_set /planning/p11/frontier_candidates \
    /planning/p11/selected_bspline /xq/p12/flight_status \
    /xq/eval/p12/obstacle_state /xq/eval/agent_01/ground_truth /tf /tf_static /clock \
    >"${trial_dir}/rosbag.log" 2>&1 < /dev/null & pids+=("$!")
  setsid ros2 launch xq_sim_bringup xq_p14_fault_injection.launch.py \
    world_file:="${world}" trial:="${trial}" schedule_file:="${schedule}" \
    thresholds_file:="${p14_thresholds}" calibration_file:="${calibration}" \
    calibration_sha256:="${calibration_sha}" p12_thresholds_file:="${p12_thresholds}" \
    p13_thresholds_file:="${p13_thresholds}" p14_result_file:="${trial_dir}/trial-result.json" \
    p12_result_file:="${trial_dir}/p12-retention-result.json" \
    p13_result_file:="${trial_dir}/p13-retention-result.json" "${record_arg[@]}" \
    planner_delay_ms:="$(value low_planner_delay_ms)" \
    voxel_size_m:="$(p12_value voxel_size_m)" dynamic_ttl_s:="$(p12_value dynamic_ttl_s)" \
    dynamic_occupied_threshold:="$(p12_value dynamic_occupied_threshold)" \
    dynamic_clear_threshold:="$(p12_value dynamic_clear_threshold)" \
    static_confirmation_hits:="$(p12_value static_confirmation_hits)" \
    free_confirmation_rays:="$(p12_value free_confirmation_rays)" \
    path_clearance_radius_m:="$(p12_value path_clearance_radius_m)" \
    planning_lookahead_m:="$(p12_value planning_lookahead_m)" \
    clear_confirmation_s:="$(p12_value clear_confirmation_s)" \
    mission_distance_m:="$(value mission_distance_m)" geometric_clearance_m:="$(value geometric_clearance_m)" \
    fixed_buffer_m:="$(value fixed_buffer_m)" protection_level_m:="$(value protection_level_m)" \
    required_margin_m:="$(value required_margin_m)" maximum_speed_mps:="$(value maximum_speed_mps)" \
    maximum_acceleration_mps2:="$(value maximum_acceleration_mps2)" \
    >"${trial_dir}/launch.log" 2>&1 < /dev/null & pids+=("$!")
  local launch_pid="${pids[1]}" deadline=$((SECONDS + deadline_s))
  while [[ ! -s "${trial_dir}/trial-result.json" ]] && ((SECONDS < deadline)); do
    kill -0 "${launch_pid}" 2>/dev/null || {
      echo "P14 ${trial} launch exited early." >&2
      tail -n 200 "${trial_dir}/launch.log" >&2 || true
      return 5
    }
    sleep 1
  done
  [[ -s "${trial_dir}/trial-result.json" ]] || {
    echo "P14 ${trial} evaluator timed out." >&2
    tail -n 200 "${trial_dir}/launch.log" >&2 || true
    return 6
  }
  if [[ "${trial}" == matrix ]]; then
    local retention_deadline=$((SECONDS + 15))
    while { [[ ! -s "${trial_dir}/p12-retention-result.json" ]] || [[ ! -s "${trial_dir}/p13-retention-result.json" ]]; } \
      && ((SECONDS < retention_deadline)); do sleep 1; done
    [[ -s "${trial_dir}/p12-retention-result.json" && -s "${trial_dir}/p13-retention-result.json" ]] || {
      echo "P14 retention result missing." >&2; return 7;
    }
  fi
  for node in impact_p14_controller impact_sensor_proxy xq_p12_dynamic_map xq_p11_integrity_exploration; do
    ros2 node info "/${node}" >"${trial_dir}/${node}-graph.txt"
    subscribers="$(sed -n '/Subscribers:/,/Publishers:/p' "${trial_dir}/${node}-graph.txt")"
    if grep -q '/xq/eval/\|ground_truth\|/model/' <<<"${subscribers}"; then
      echo "Ground Truth leaked into P14 algorithm node ${node}." >&2
      return 8
    fi
  done
  stop_groups
  [[ -f "${trial_dir}/rosbag/metadata.yaml" ]] || { echo "P14 ${trial} rosbag missing." >&2; return 9; }
  [[ "$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["status"])' "${trial_dir}/trial-result.json")" == PASS ]] || return 10
  if [[ "${trial}" == matrix ]]; then
    [[ "$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["status"])' "${trial_dir}/p12-retention-result.json")" == PASS ]] || return 11
    [[ "$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["status"])' "${trial_dir}/p13-retention-result.json")" == PASS ]] || return 12
  fi
}

run_trial matrix "${matrix_schedule}" 360 true
run_trial emergency "${emergency_schedule}" 150 false
python3 "${script_dir}/analyze_p14_gate.py" \
  --matrix "${result_dir}/matrix/trial-result.json" \
  --emergency "${result_dir}/emergency/trial-result.json" \
  --p12-retention "${result_dir}/matrix/p12-retention-result.json" \
  --p13-retention "${result_dir}/matrix/p13-retention-result.json" \
  --world "${world}" --output "${result_dir}/p14-gate-result.json" \
  >"${result_dir}/analysis.log"
finish_audit
echo "PASS: P14 deterministic fault matrix and persistent-LiDAR fail-safe landing passed."
echo "Results: ${result_dir}"
