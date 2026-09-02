#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
workspace_root="$(cd -- "${script_dir}/.." && pwd -P)"
timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
result_dir="$(realpath -m -- "${1:-${workspace_root}/experiments/results/impact_p11/gate_${timestamp}_$$}")"
case "${result_dir}" in
  "${workspace_root}"/experiments/results/impact_p11/*) ;;
  *) echo "P11 Gate results must stay under this workspace." >&2; exit 2 ;;
esac
mkdir -p -- "${result_dir}/configuration"

for required in \
  "${workspace_root}/xq_install/setup.bash" \
  "${workspace_root}/xq_install/.xq_build_manifest.json" \
  "${workspace_root}/evidence/P7/p7-calibration.json" \
  "${workspace_root}/config/p11_gate_thresholds.json" \
  "${workspace_root}/scripts/analyze_p11_gate.py" \
  "${workspace_root}/scripts/audit_external_assets.sh"; do
  [[ -f "${required}" ]] || { echo "Missing P11 Gate dependency: ${required}" >&2; exit 2; }
done

unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
unset GZ_SIM_RESOURCE_PATH IGN_GAZEBO_RESOURCE_PATH SDF_PATH
set +u
source /opt/ros/humble/setup.bash
source "${workspace_root}/xq_install/setup.bash"
set -u
world="${workspace_root}/xq_install/xq_gz_assets/share/xq_gz_assets/worlds/xq_p11_integrity_exploration.sdf"
calibration="${workspace_root}/evidence/P7/p7-calibration.json"
thresholds="${workspace_root}/config/p11_gate_thresholds.json"
[[ -f "${world}" ]] || { echo "Installed P11 world missing." >&2; exit 2; }
value() { python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))[sys.argv[2]])' "${thresholds}" "$1"; }
calibration_sha="$(sha256sum "${calibration}" | cut -d ' ' -f 1)"

cp -- "${thresholds}" "${result_dir}/configuration/p11_gate_thresholds.json"
cp -- "${world}" "${result_dir}/configuration/xq_p11_integrity_exploration.sdf"
cp -- "${calibration}" "${result_dir}/configuration/p7-calibration.json"
cp -- "${workspace_root}/xq_install/.xq_build_manifest.json" "${result_dir}/configuration/build-manifest.json"
for source in \
  src/xq_sim_bringup/launch/xq_p11_flight.launch.py \
  src/xq_autonomy/xq_autonomy/integrity_exploration.py \
  src/xq_autonomy/xq_autonomy/p11_integrity_exploration_node.py \
  src/xq_autonomy/xq_autonomy/p11_flight_controller_node.py \
  src/xq_autonomy/xq_autonomy/p11_flight_evaluator_node.py \
  scripts/analyze_p11_gate.py; do
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

variants=(information_only integrity_constrained)
for index in "${!variants[@]}"; do
  variant="${variants[index]}"
  arm_dir="${result_dir}/${variant}"
  mkdir -p -- "${arm_dir}/ros_logs"
  export ROS_DOMAIN_ID=$((160 + index + ($$ % 20)))
  export ROS_LOCALHOST_ONLY=1
  export ROS2CLI_NO_DAEMON=1
  export GZ_PARTITION="xq_p11_${variant}_${timestamp}_$$"
  export ROS_LOG_DIR="${arm_dir}/ros_logs"
  export LIBGL_ALWAYS_SOFTWARE=1
  export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe
  export GALLIUM_DRIVER=llvmpipe
  export EGL_PLATFORM=surfaceless
  export QT_QPA_PLATFORM=offscreen
  {
    echo "variant=${variant}"
    echo "ros_domain_id=${ROS_DOMAIN_ID}"
    echo "gz_partition=${GZ_PARTITION}"
    echo "world_sha256=$(sha256sum "${world}" | cut -d ' ' -f 1)"
    echo "calibration_sha256=${calibration_sha}"
    echo "ground_truth_policy=evaluator_and_logger_only"
    echo "hard_constraint=true"
    echo "margin_in_utility=false"
  } >"${arm_dir}/run.env"

  setsid ros2 bag record --compression-mode file --compression-format zstd \
    -o "${arm_dir}/rosbag" \
    /livox/lidar /livox/imu /localization/odom /localization/geometry \
    /cloud_registered /xq/p5/cloud_map /integrity/information_map \
    /integrity/directional /integrity/exploration_decision \
    /planning/p11/frontier_candidate_set /planning/p11/frontier_candidates \
    /planning/p11/selected_bspline /planning/p11/unconstrained_bspline \
    /xq/p11/flight_status /xq/eval/agent_01/ground_truth /tf /tf_static /clock \
    >"${arm_dir}/rosbag.log" 2>&1 < /dev/null & pids+=("$!")
  setsid ros2 launch xq_sim_bringup xq_p11_flight.launch.py \
    world_file:="${world}" variant:="${variant}" \
    calibration_file:="${calibration}" calibration_sha256:="${calibration_sha}" \
    result_file:="${arm_dir}/flight-result.json" \
    visibility_radius_m:="$(value visibility_radius_m)" \
    information_scale:="$(value information_scale_m_inv2)" \
    minimum_prediction_variance_m2:="$(value minimum_prediction_variance_m2)" \
    margin_reserve_m:="$(value margin_reserve_m)" \
    collision_probability_limit:="$(value collision_probability_limit)" \
    energy_remaining:="$(value energy_remaining)" \
    mission_distance_m:="$(value mission_distance_m)" \
    information_weight:="$(value information_weight)" \
    travel_time_weight:="$(value travel_time_weight)" \
    energy_weight:="$(value energy_weight)" \
    direct_information_gain:="$(value direct_information_gain)" \
    safe_information_gain:="$(value safe_information_gain)" \
    >"${arm_dir}/launch.log" 2>&1 < /dev/null & pids+=("$!")
  launch_pid="${pids[1]}"

  deadline=$((SECONDS + 180))
  while [[ ! -s "${arm_dir}/flight-result.json" ]] && ((SECONDS < deadline)); do
    kill -0 "${launch_pid}" 2>/dev/null || {
      echo "P11 ${variant} launch exited early." >&2
      tail -n 120 "${arm_dir}/launch.log" >&2 || true
      exit 5
    }
    sleep 1
  done
  [[ -s "${arm_dir}/flight-result.json" ]] || { echo "P11 ${variant} evaluator timed out." >&2; exit 6; }
  [[ "$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["status"])' "${arm_dir}/flight-result.json")" == PASS ]] || {
    cat "${arm_dir}/flight-result.json" >&2; exit 7
  }
  for node in xq_p11_integrity_exploration xq_p10_information_map xq_p11_flight_controller; do
    ros2 node info "/${node}" >"${arm_dir}/${node}-graph.txt"
    subscribers="$(sed -n '/Subscribers:/,/Publishers:/p' "${arm_dir}/${node}-graph.txt")"
    if grep -q '/xq/eval/\|ground_truth' <<<"${subscribers}"; then
      echo "Ground Truth leaked into algorithm node ${node}." >&2; exit 8
    fi
  done
  stop_groups
  [[ -f "${arm_dir}/rosbag/metadata.yaml" ]] || { echo "P11 ${variant} rosbag missing." >&2; exit 9; }
done

python3 "${script_dir}/analyze_p11_gate.py" "${result_dir}" "${thresholds}" \
  >"${result_dir}/summary.stdout.json"
finish_audit
echo "PASS: P11 two-arm integrity-constrained exploration Gazebo Gate completed."
echo "Results: ${result_dir}"
