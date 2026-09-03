#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
workspace_root="$(cd -- "${script_dir}/.." && pwd -P)"
timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
result_dir="$(realpath -m -- "${1:-${workspace_root}/experiments/results/impact_complex_comparison/gate_${timestamp}_$$}")"
comparison_thresholds="$(realpath -m -- "${XQ_COMPLEX_THRESHOLDS:-${workspace_root}/config/complex_comparison_thresholds.json}")"
comparison_analyzer="$(realpath -m -- "${XQ_COMPLEX_ANALYZER:-${workspace_root}/scripts/analyze_complex_comparison.py}")"
case "${result_dir}" in
  "${workspace_root}"/experiments/results/impact_complex_comparison/*) ;;
  *) echo "Complex comparison results must stay under this workspace." >&2; exit 2 ;;
esac
case "${comparison_thresholds}" in
  "${workspace_root}"/config/*.json) ;;
  *) echo "Complex comparison thresholds must stay under the project config directory." >&2; exit 2 ;;
esac
case "${comparison_analyzer}" in
  "${workspace_root}"/scripts/*.py) ;;
  *) echo "Complex comparison analyzer must stay under the project scripts directory." >&2; exit 2 ;;
esac
mkdir -p -- "${result_dir}/configuration" "${result_dir}/ros_logs"

for required in \
  "${workspace_root}/xq_install/setup.bash" \
  "${workspace_root}/xq_install/.xq_build_manifest.json" \
  "${workspace_root}/evidence/P7/p7-calibration.json" \
  "${workspace_root}/config/complex_dynamic_thresholds.json" \
  "${comparison_thresholds}" \
  "${workspace_root}/config/p13_gate_thresholds.json" \
  "${comparison_analyzer}" \
  "${workspace_root}/scripts/audit_external_assets.sh"; do
  [[ -f "${required}" ]] || { echo "Missing complex-comparison dependency: ${required}" >&2; exit 2; }
done

unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
unset GZ_SIM_RESOURCE_PATH IGN_GAZEBO_RESOURCE_PATH SDF_PATH
unset LIBGL_ALWAYS_SOFTWARE MESA_LOADER_DRIVER_OVERRIDE GALLIUM_DRIVER EGL_PLATFORM QT_QPA_PLATFORM
set +u
source /opt/ros/humble/setup.bash
source "${workspace_root}/xq_install/setup.bash"
set -u

calibration="${workspace_root}/evidence/P7/p7-calibration.json"
p12_thresholds="${workspace_root}/config/complex_dynamic_thresholds.json"
p13_thresholds="${workspace_root}/config/p13_gate_thresholds.json"
world_filename="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["world"])' "${comparison_thresholds}")"
lateral_offset_m="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("lateral_offset_m", 0.68))' "${comparison_thresholds}")"
enable_vertical_candidate="$(python3 -c 'import json,sys; print(str(bool(json.load(open(sys.argv[1])).get("enable_vertical_candidate", False))).lower())' "${comparison_thresholds}")"
enable_diagonal_vertical_candidates="$(python3 -c 'import json,sys; print(str(bool(json.load(open(sys.argv[1])).get("enable_diagonal_vertical_candidates", False))).lower())' "${comparison_thresholds}")"
vertical_offset_m="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("vertical_offset_m", 0.70))' "${comparison_thresholds}")"
integrity_information_memory_horizon_s="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("integrity_information_memory_horizon_s", 0.0))' "${comparison_thresholds}")"
integrity_information_memory_max_frames="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("integrity_information_memory_max_frames", 20.0))' "${comparison_thresholds}")"
segment_goal_tolerance_m="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("segment_goal_tolerance_m", 0.25))' "${comparison_thresholds}")"
post_dynamic_static_confirmation_s="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("post_dynamic_static_confirmation_s", 0.0))' "${comparison_thresholds}")"
reversible_static_ttl_s="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("reversible_static_ttl_s", 0.0))' "${comparison_thresholds}")"
maximum_rays="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("maximum_rays", 900))' "${comparison_thresholds}")"
obstacle_enter_start_s="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("obstacle_enter_start_s", 24.0))' "${comparison_thresholds}")"
obstacle_enter_end_s="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("obstacle_enter_end_s", 28.0))' "${comparison_thresholds}")"
obstacle_leave_start_s="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("obstacle_leave_start_s", 44.0))' "${comparison_thresholds}")"
obstacle_leave_end_s="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("obstacle_leave_end_s", 48.0))' "${comparison_thresholds}")"
candidate_generation_mode="${XQ_CANDIDATE_GENERATION_MODE:-legacy}"
candidate_metric_source="${XQ_CANDIDATE_METRIC_SOURCE:-metadata}"
lattice_lateral_levels="${XQ_LATTICE_LATERAL_LEVELS:-5}"
lattice_vertical_levels="${XQ_LATTICE_VERTICAL_LEVELS:-2}"
task_progress_weight="${XQ_TASK_PROGRESS_WEIGHT:-0.85}"
task_map_age_time_constant_s="${XQ_TASK_MAP_AGE_TIME_CONSTANT_S:-20.0}"
research_energy_remaining="${XQ_RESEARCH_ENERGY_REMAINING:-32.0}"
utility_indifference_band="${XQ_UTILITY_INDIFFERENCE_BAND:-0.0}"
dynamic_path_query_mode="${XQ_DYNAMIC_PATH_QUERY_MODE:-forward_axis}"
minimum_dynamic_cluster_points="${XQ_MINIMUM_DYNAMIC_CLUSTER_POINTS:-1}"
dynamic_cluster_radius_m="${XQ_DYNAMIC_CLUSTER_RADIUS_M:-0.45}"
terminal_extension_mode="${XQ_TERMINAL_EXTENSION_MODE:-fixed}"
runtime_integrity_guard_mode="${XQ_RUNTIME_INTEGRITY_GUARD_MODE:-disabled}"
runtime_integrity_margin_m="${XQ_RUNTIME_INTEGRITY_MARGIN_M:-0.12}"
[[ "${world_filename}" == "$(basename -- "${world_filename}")" ]] || {
  echo "Complex world must be a project-local installed world filename." >&2; exit 2;
}
world="${workspace_root}/xq_install/xq_gz_assets/share/xq_gz_assets/worlds/${world_filename}"
[[ -f "${world}" ]] || { echo "Installed complex world is missing." >&2; exit 2; }
value() { python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))[sys.argv[2]])' "$1" "$2"; }
calibration_sha="$(sha256sum "${calibration}" | cut -d ' ' -f 1)"
world_sha="$(sha256sum "${world}" | cut -d ' ' -f 1)"
renderer="$(glxinfo -B 2>/dev/null | sed -n 's/^OpenGL renderer string: //p' | head -n 1)"

cp -- "${comparison_thresholds}" "${result_dir}/configuration/complex_comparison_thresholds.json"
cp -- "${p12_thresholds}" "${result_dir}/configuration/complex_dynamic_thresholds.json"
cp -- "${p13_thresholds}" "${result_dir}/configuration/p13_gate_thresholds.json"
cp -- "${world}" "${result_dir}/configuration/${world_filename}"
cp -- "${calibration}" "${result_dir}/configuration/p7-calibration.json"
cp -- "${workspace_root}/xq_install/.xq_build_manifest.json" "${result_dir}/configuration/build-manifest.json"
for source in \
  src/xq_sim_bringup/launch/xq_p13_flight.launch.py \
  src/xq_autonomy/xq_autonomy/integrity_exploration.py \
  src/xq_autonomy/xq_autonomy/candidate_metrics.py \
  src/xq_autonomy/xq_autonomy/integrity_evaluation.py \
  src/xq_autonomy/xq_autonomy/integrity.py \
  src/xq_autonomy/xq_autonomy/dynamic_planning.py \
  src/xq_autonomy/xq_autonomy/p6_directional_integrity_node.py \
  src/xq_autonomy/xq_autonomy/p11_integrity_exploration_node.py \
  src/xq_autonomy/xq_autonomy/p11_flight_controller_node.py \
  src/xq_autonomy/xq_autonomy/p11_flight_evaluator_node.py \
  src/xq_autonomy/xq_autonomy/dynamic_voxel_map.py \
  src/xq_autonomy/xq_autonomy/p12_dynamic_map_node.py \
  src/xq_autonomy/xq_autonomy/p12_flight_controller_node.py \
  src/xq_autonomy/xq_autonomy/p12_flight_evaluator_node.py \
  src/xq_autonomy/xq_autonomy/p13_flight_controller_node.py \
  src/xq_autonomy/xq_autonomy/p13_flight_evaluator_node.py \
  scripts/analyze_p13_latency_bag.py \
  "${comparison_analyzer#${workspace_root}/}"; do
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

export ROS_LOCALHOST_ONLY=1 ROS2CLI_NO_DAEMON=1
export MESA_D3D12_DEFAULT_ADAPTER_NAME="${MESA_D3D12_DEFAULT_ADAPTER_NAME:-NVIDIA}"

read -r -a variants <<<"${XQ_COMPLEX_VARIANTS:-information_only integrity_constrained}"
for variant in "${variants[@]}"; do
  case "${variant}" in
    information_only|integrity_constrained) ;;
    *) echo "Unsupported complex comparison variant: ${variant}" >&2; exit 2 ;;
  esac
done
for index in "${!variants[@]}"; do
  variant="${variants[index]}"
  arm_dir="${result_dir}/${variant}"
  mkdir -p -- "${arm_dir}" "${result_dir}/ros_logs/${variant}"
  export ROS_DOMAIN_ID=$((190 + index + ($$ % 8)))
  export GZ_PARTITION="xq_complex_compare_${variant}_${timestamp}_$$"
  export ROS_LOG_DIR="${result_dir}/ros_logs/${variant}"
  {
    echo "variant=${variant}"
    echo "ros_domain_id=${ROS_DOMAIN_ID}"
    echo "gz_partition=${GZ_PARTITION}"
    echo "world_sha256=${world_sha}"
    echo "calibration_sha256=${calibration_sha}"
    echo "renderer=${renderer:-unavailable}"
    echo "scenario=${world_filename}"
    echo "lateral_offset_m=${lateral_offset_m}"
    echo "enable_vertical_candidate=${enable_vertical_candidate}"
    echo "enable_diagonal_vertical_candidates=${enable_diagonal_vertical_candidates}"
    echo "vertical_offset_m=${vertical_offset_m}"
    echo "integrity_information_memory_horizon_s=${integrity_information_memory_horizon_s}"
    echo "integrity_information_memory_max_frames=${integrity_information_memory_max_frames}"
    echo "segment_goal_tolerance_m=${segment_goal_tolerance_m}"
    echo "post_dynamic_static_confirmation_s=${post_dynamic_static_confirmation_s}"
    echo "reversible_static_ttl_s=${reversible_static_ttl_s}"
    echo "maximum_rays=${maximum_rays}"
    echo "obstacle_enter_start_s=${obstacle_enter_start_s}"
    echo "obstacle_enter_end_s=${obstacle_enter_end_s}"
    echo "obstacle_leave_start_s=${obstacle_leave_start_s}"
    echo "obstacle_leave_end_s=${obstacle_leave_end_s}"
    echo "candidate_generation_mode=${candidate_generation_mode}"
    echo "candidate_metric_source=${candidate_metric_source}"
    echo "lattice_lateral_levels=${lattice_lateral_levels}"
    echo "lattice_vertical_levels=${lattice_vertical_levels}"
    echo "task_progress_weight=${task_progress_weight}"
    echo "task_map_age_time_constant_s=${task_map_age_time_constant_s}"
    echo "research_energy_remaining=${research_energy_remaining}"
    echo "utility_indifference_band=${utility_indifference_band}"
    echo "dynamic_path_query_mode=${dynamic_path_query_mode}"
    echo "minimum_dynamic_cluster_points=${minimum_dynamic_cluster_points}"
    echo "dynamic_cluster_radius_m=${dynamic_cluster_radius_m}"
    echo "terminal_extension_mode=${terminal_extension_mode}"
    echo "runtime_integrity_guard_mode=${runtime_integrity_guard_mode}"
    echo "runtime_integrity_margin_m=${runtime_integrity_margin_m}"
    echo "flight_mode=full_normal_autonomy_no_fault_injection"
    echo "comparison_difference=integrity_hard_filter_only"
    echo "recording_profile=low_interference_algorithm_evidence"
    echo "ground_truth_policy=evaluation_only"
  } >"${arm_dir}/run.env"

  # P2/P3 already archive the full-rate LiDAR/IMU data-contract evidence.
  # This benchmark records the causal algorithm/trajectory/evaluation chain
  # only: adding another subscriber for every raw point cloud measurably
  # perturbs the p99 latency that P13 is required to control.
  setsid nice -n 10 ros2 bag record --compression-mode file --compression-format zstd \
    -o "${arm_dir}/rosbag" \
    /localization/odom /localization/geometry \
    /mapping/p12/dynamic_voxels /mapping/p12/status \
    /planning/p12/replan_event /integrity/p13/latency_trace /xq/p13/flight_status \
    /integrity/directional /integrity/information_map /integrity/exploration_decision \
    /integrity/exploration_debug /planning/p11/frontier_candidate_set \
    /planning/p11/frontier_candidates /planning/p11/selected_bspline \
    /planning/p11/unconstrained_bspline /xq/p11/flight_status /xq/p12/flight_status \
    /xq/p3/cmd_vel \
    /xq/eval/p12/obstacle_state /xq/eval/agent_01/ground_truth /tf /tf_static /clock \
    >"${arm_dir}/rosbag.log" 2>&1 < /dev/null & pids+=("$!")

  setsid ros2 launch xq_sim_bringup xq_p13_flight.launch.py \
    world_file:="${world}" headless_rendering:=false \
    calibration_file:="${calibration}" calibration_sha256:="${calibration_sha}" \
    thresholds_file:="${p13_thresholds}" p12_thresholds_file:="${p12_thresholds}" \
    p11_result_file:="${arm_dir}/p11-result.json" \
    p12_result_file:="${arm_dir}/p12-result.json" \
    p13_result_file:="${arm_dir}/p13-result.json" \
    enable_p11_evaluator:=true flight_variant:="${variant}" \
    latency_profile:=complex_50ms planner_delay_ms:=50.0 \
    integrity_information_memory_horizon_s:="${integrity_information_memory_horizon_s}" \
    integrity_information_memory_max_frames:="${integrity_information_memory_max_frames}" \
    voxel_size_m:="$(value "${p12_thresholds}" voxel_size_m)" \
    dynamic_ttl_s:="$(value "${p12_thresholds}" dynamic_ttl_s)" \
    dynamic_occupied_threshold:="$(value "${p12_thresholds}" dynamic_occupied_threshold)" \
    dynamic_clear_threshold:="$(value "${p12_thresholds}" dynamic_clear_threshold)" \
    static_confirmation_hits:="$(value "${p12_thresholds}" static_confirmation_hits)" \
    post_dynamic_static_confirmation_s:="${post_dynamic_static_confirmation_s}" \
    reversible_static_ttl_s:="${reversible_static_ttl_s}" \
    maximum_rays:="${maximum_rays}" \
    free_confirmation_rays:="$(value "${p12_thresholds}" free_confirmation_rays)" \
    path_clearance_radius_m:="$(value "${p12_thresholds}" path_clearance_radius_m)" \
    planning_lookahead_m:="$(value "${p12_thresholds}" planning_lookahead_m)" \
    clear_confirmation_s:="$(value "${p12_thresholds}" clear_confirmation_s)" \
    mission_distance_m:=24.0 lateral_offset_m:="${lateral_offset_m}" lateral_candidate_shape:=challenge_then_center \
    enable_vertical_candidate:="${enable_vertical_candidate}" vertical_offset_m:="${vertical_offset_m}" \
    enable_diagonal_vertical_candidates:="${enable_diagonal_vertical_candidates}" \
    candidate_generation_mode:="${candidate_generation_mode}" \
    candidate_metric_source:="${candidate_metric_source}" \
    lattice_lateral_levels:="${lattice_lateral_levels}" \
    lattice_vertical_levels:="${lattice_vertical_levels}" \
    task_progress_weight:="${task_progress_weight}" \
    task_map_age_time_constant_s:="${task_map_age_time_constant_s}" \
    research_energy_remaining:="${research_energy_remaining}" \
    utility_indifference_band:="${utility_indifference_band}" \
    dynamic_path_query_mode:="${dynamic_path_query_mode}" \
    minimum_dynamic_cluster_points:="${minimum_dynamic_cluster_points}" \
    dynamic_cluster_radius_m:="${dynamic_cluster_radius_m}" \
    terminal_extension_mode:="${terminal_extension_mode}" \
    runtime_integrity_guard_mode:="${runtime_integrity_guard_mode}" \
    runtime_integrity_margin_m:="${runtime_integrity_margin_m}" \
    segment_goal_tolerance_m:="${segment_goal_tolerance_m}" \
    geometric_clearance_m:=0.82 fixed_buffer_m:=0.58 protection_level_m:=0.10 \
    required_margin_m:=0.06 maximum_speed_mps:=0.42 maximum_acceleration_mps2:=0.8 \
    rejected_candidate_retry_s:=1.0 maximum_candidate_retries:=60 \
    integrity_recovery_speed_mps:=0.08 integrity_recovery_max_offset_m:=0.35 \
    integrity_recovery_half_period_s:=3.0 obstacle_x_m:=-4.5 obstacle_park_y_m:=6.6 \
    obstacle_blocked_y_m:=0.0 obstacle_z_m:=1.0 obstacle_enter_start_s:="${obstacle_enter_start_s}" \
    obstacle_enter_end_s:="${obstacle_enter_end_s}" obstacle_leave_start_s:="${obstacle_leave_start_s}" \
    obstacle_leave_end_s:="${obstacle_leave_end_s}" \
    >"${arm_dir}/launch.log" 2>&1 < /dev/null & pids+=("$!")
  launch_pid="${pids[1]}"
  # Runtime configuration is audited from the same status messages consumed
  # by the evaluators and recorded in the bag.  Avoid ros2cli parameter calls:
  # each call creates an extra DDS participant and can block independently of
  # an otherwise healthy data plane.
  deadline=$((SECONDS + 330))
  while [[ ! -s "${arm_dir}/p13-result.json" ]] && ((SECONDS < deadline)); do
    kill -0 "${launch_pid}" 2>/dev/null || {
      echo "Complex ${variant} launch exited early." >&2
      tail -n 180 "${arm_dir}/launch.log" >&2 || true
      exit 5
    }
    sleep 1
  done
  [[ -s "${arm_dir}/p13-result.json" ]] || {
    echo "Complex ${variant} timed out." >&2
    tail -n 180 "${arm_dir}/launch.log" >&2 || true
    exit 6
  }
  wait_deadline=$((SECONDS + 15))
  while { [[ ! -s "${arm_dir}/p11-result.json" ]] || [[ ! -s "${arm_dir}/p12-result.json" ]]; } \
    && ((SECONDS < wait_deadline)); do sleep 1; done
  [[ -s "${arm_dir}/p11-result.json" && -s "${arm_dir}/p12-result.json" ]] || {
    echo "Complex ${variant} evaluator result missing." >&2; exit 6;
  }
  for node in xq_p13_flight_controller xq_p12_dynamic_map xq_p11_integrity_exploration; do
    graph_ready=false
    # ros2cli discovery can leave a DDS participant blocked during heavily
    # loaded Gazebo teardown.  TERM alone is not a hard bound, so force KILL
    # after two seconds and keep the whole evidence audit under 24 seconds.
    for attempt in 1 2 3; do
      if timeout --kill-after=2s 6s ros2 node info "/${node}" \
        >"${arm_dir}/${node}-graph.txt" 2>/dev/null; then
        graph_ready=true
        break
      fi
      sleep 0.5
    done
    [[ "${graph_ready}" == true ]] || {
      echo "DDS graph audit could not resolve /${node} after bounded retry." >&2
      exit 8
    }
    subscribers="$(sed -n '/Subscribers:/,/Publishers:/p' "${arm_dir}/${node}-graph.txt")"
    if grep -q '/xq/eval/\|ground_truth\|/model/' <<<"${subscribers}"; then
      echo "Ground Truth leaked into algorithm node ${node}." >&2; exit 8
    fi
  done
  stop_groups
  [[ -f "${arm_dir}/rosbag/metadata.yaml" ]] || { echo "Complex ${variant} rosbag missing." >&2; exit 9; }
  if grep -Eq 'Traceback|process has died|\[ERROR\]' "${arm_dir}/launch.log"; then
    echo "Complex ${variant} launch log contains a node failure or traceback." >&2
    grep -En 'Traceback|process has died|\[ERROR\]' "${arm_dir}/launch.log" >&2 || true
    exit 10
  fi
  for result in p11-result.json p12-result.json p13-result.json; do
    [[ "$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["status"])' "${arm_dir}/${result}")" == PASS ]] || {
      echo "Complex ${variant} ${result} failed." >&2
      cat "${arm_dir}/${result}" >&2
      exit 7
    }
  done
done

if [[ ${#variants[@]} -ne 2 ]] ||
   [[ " ${variants[*]} " != *" information_only "* ]] ||
   [[ " ${variants[*]} " != *" integrity_constrained "* ]]; then
  finish_audit
  echo "PASS: requested complex diagnostic arm completed; comparison was not claimed."
  echo "GPU renderer: ${renderer:-unavailable}"
  echo "Results: ${result_dir}"
  exit 0
fi

python3 "${comparison_analyzer}" \
  --result-dir "${result_dir}" --thresholds "${comparison_thresholds}" \
  --world-sha256 "${world_sha}" --output "${result_dir}/comparison-result.json" \
  >"${result_dir}/analysis.log"
finish_audit
echo "PASS: complex full-autonomy comparison isolated the integrity hard-filter advantage."
echo "GPU renderer: ${renderer:-unavailable}"
echo "Results: ${result_dir}"
