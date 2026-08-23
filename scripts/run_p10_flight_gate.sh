#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
workspace_root="$(cd -- "${script_dir}/.." && pwd -P)"
timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
default_result_dir="${workspace_root}/experiments/results/impact_p10/gate_${timestamp}_$$"
result_dir="$(realpath -m -- "${1:-${default_result_dir}}")"
case "${result_dir}" in
  "${workspace_root}"/experiments/results/impact_p10/*) ;;
  *) echo "P10 Gate results must stay under this workspace." >&2; exit 2 ;;
esac
mkdir -p -- "${result_dir}/configuration"

for required in \
  "${workspace_root}/xq_install/setup.bash" \
  "${workspace_root}/xq_install/.xq_build_manifest.json" \
  "${workspace_root}/evidence/P7/p7-calibration.json" \
  "${workspace_root}/config/p10_gate_thresholds.json" \
  "${workspace_root}/scripts/analyze_p10_gate.py" \
  "${workspace_root}/scripts/audit_external_assets.sh"; do
  [[ -f "${required}" ]] || { echo "Missing P10 Gate dependency: ${required}" >&2; exit 2; }
done

unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
unset GZ_SIM_RESOURCE_PATH IGN_GAZEBO_RESOURCE_PATH SDF_PATH
set +u
source /opt/ros/humble/setup.bash
source "${workspace_root}/xq_install/setup.bash"
set -u

world="${workspace_root}/xq_install/xq_gz_assets/share/xq_gz_assets/worlds/xq_p10_long_corridor.sdf"
calibration="${workspace_root}/evidence/P7/p7-calibration.json"
thresholds="${workspace_root}/config/p10_gate_thresholds.json"
[[ -f "${world}" ]] || { echo "Installed P10 world missing: ${world}" >&2; exit 2; }
visibility="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["visibility_radius_m"])' "${thresholds}")"
information_scale="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["information_scale_m_inv2"])' "${thresholds}")"
prediction_variance="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["minimum_prediction_variance_m2"])' "${thresholds}")"
margin_reserve="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["margin_reserve_m"])' "${thresholds}")"
calibration_sha="$(sha256sum "${calibration}" | cut -d ' ' -f 1)"

cp -- "${thresholds}" "${result_dir}/configuration/p10_gate_thresholds.json"
cp -- "${world}" "${result_dir}/configuration/xq_p10_long_corridor.sdf"
cp -- "${calibration}" "${result_dir}/configuration/p7-calibration.json"
cp -- "${workspace_root}/xq_install/.xq_build_manifest.json" \
  "${result_dir}/configuration/build-manifest.json"
cp -- "${workspace_root}/src/xq_sim_bringup/launch/xq_p10_flight.launch.py" \
  "${result_dir}/configuration/xq_p10_flight.launch.py"
cp -- "${workspace_root}/src/xq_autonomy/xq_autonomy/p10_active_perception_node.py" \
  "${result_dir}/configuration/p10_active_perception_node.py"
cp -- "${workspace_root}/src/xq_autonomy/xq_autonomy/p10_information_map_node.py" \
  "${result_dir}/configuration/p10_information_map_node.py"
cp -- "${workspace_root}/src/xq_autonomy/xq_autonomy/p10_flight_controller_node.py" \
  "${result_dir}/configuration/p10_flight_controller_node.py"
cp -- "${workspace_root}/src/xq_autonomy/xq_autonomy/p10_flight_evaluator_node.py" \
  "${result_dir}/configuration/p10_flight_evaluator_node.py"
cp -- "${workspace_root}/scripts/analyze_p10_gate.py" \
  "${result_dir}/configuration/analyze_p10_gate.py"
sha256sum "${result_dir}"/configuration/* >"${result_dir}/configuration.sha256"

bash "${script_dir}/audit_external_assets.sh" snapshot \
  "${result_dir}/external-assets.before.sha256" >/dev/null
audit_done=false
declare -a managed_pids=()

stop_groups() {
  local pid pgid round alive
  for pid in "${managed_pids[@]}"; do
    pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d '[:space:]' || true)"
    [[ "${pgid}" == "${pid}" ]] && kill -INT -- "${pid}" 2>/dev/null || true
  done
  for round in 1 2 3 4 5 6 7 8 9 10; do
    alive=false
    for pid in "${managed_pids[@]}"; do kill -0 "${pid}" 2>/dev/null && alive=true; done
    [[ "${alive}" == false ]] && break
    sleep 1
  done
  for pid in "${managed_pids[@]}"; do
    # start_group uses setsid, therefore the original leader PID remains the
    # process-group ID even if ros2 launch exits before a Gazebo child.
    kill -TERM -- "-${pid}" 2>/dev/null || true
  done
  for round in 1 2 3 4 5; do
    alive=false
    for pid in "${managed_pids[@]}"; do pgrep -g "${pid}" >/dev/null && alive=true; done
    [[ "${alive}" == false ]] && break
    sleep 1
  done
  for pid in "${managed_pids[@]}"; do
    pgrep -g "${pid}" >/dev/null && kill -KILL -- "-${pid}" 2>/dev/null || true
  done
  for pid in "${managed_pids[@]}"; do wait "${pid}" 2>/dev/null || true; done
  managed_pids=()
}

finish_audit() {
  [[ "${audit_done}" == false ]] || return 0
  bash "${script_dir}/audit_external_assets.sh" snapshot \
    "${result_dir}/external-assets.after.sha256" >/dev/null
  bash "${script_dir}/audit_external_assets.sh" compare \
    "${result_dir}/external-assets.before.sha256" \
    "${result_dir}/external-assets.after.sha256" >"${result_dir}/isolation-audit.txt"
  audit_done=true
}

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  set +e
  stop_groups
  finish_audit
  exit "${status}"
}
trap cleanup EXIT INT TERM

variants=(baseline yaw_only minimum_excitation)
for index in "${!variants[@]}"; do
  variant="${variants[index]}"
  arm_dir="${result_dir}/${variant}"
  mkdir -p -- "${arm_dir}/ros_logs"
  export ROS_DOMAIN_ID=$((140 + index + ($$ % 20)))
  export ROS_LOCALHOST_ONLY=1
  export ROS2CLI_NO_DAEMON=1
  export GZ_PARTITION="xq_p10_${variant}_${timestamp}_$$"
  export ROS_LOG_DIR="${arm_dir}/ros_logs"
  export LIBGL_ALWAYS_SOFTWARE=1
  export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe
  export GALLIUM_DRIVER=llvmpipe
  export EGL_PLATFORM=surfaceless
  export QT_QPA_PLATFORM=offscreen
  cat >"${arm_dir}/run.env" <<EOF
variant=${variant}
ros_domain_id=${ROS_DOMAIN_ID}
gz_partition=${GZ_PARTITION}
world_sha256=$(sha256sum "${world}" | cut -d ' ' -f 1)
calibration_sha256=${calibration_sha}
visibility_radius_m=${visibility}
information_scale_m_inv2=${information_scale}
minimum_prediction_variance_m2=${prediction_variance}
margin_reserve_m=${margin_reserve}
ground_truth_policy=evaluator_and_logger_only
EOF

  setsid ros2 bag record --compression-mode file --compression-format zstd \
    -o "${arm_dir}/rosbag" \
    /livox/lidar /livox/imu /localization/odom /localization/geometry \
    /cloud_registered /xq/p5/cloud_map /integrity/information_map \
    /integrity/directional /integrity/active_perception_decision \
    /planning/p10/baseline_bspline /planning/active_perception_candidates \
    /planning/active_perception_bspline /xq/p10/flight_status \
    /xq/eval/agent_01/ground_truth /tf /tf_static /clock \
    >"${arm_dir}/rosbag.log" 2>&1 < /dev/null &
  managed_pids+=("$!")
  setsid ros2 launch xq_sim_bringup xq_p10_flight.launch.py \
    world_file:="${world}" variant:="${variant}" \
    calibration_file:="${calibration}" calibration_sha256:="${calibration_sha}" \
    result_file:="${arm_dir}/flight-result.json" \
    visibility_radius_m:="${visibility}" information_scale:="${information_scale}" \
    minimum_prediction_variance_m2:="${prediction_variance}" \
    margin_reserve_m:="${margin_reserve}" \
    >"${arm_dir}/launch.log" 2>&1 < /dev/null &
  managed_pids+=("$!")

  deadline=$((SECONDS + 100))
  while [[ ! -s "${arm_dir}/flight-result.json" ]] && ((SECONDS < deadline)); do
    kill -0 "${managed_pids[1]}" 2>/dev/null || {
      echo "P10 ${variant} launch exited early." >&2
      tail -n 120 "${arm_dir}/launch.log" >&2 || true
      exit 5
    }
    sleep 1
  done
  [[ -s "${arm_dir}/flight-result.json" ]] || {
    echo "P10 ${variant} evaluator timed out." >&2; exit 6;
  }
  [[ "$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["status"])' "${arm_dir}/flight-result.json")" == PASS ]] || {
    cat "${arm_dir}/flight-result.json" >&2; exit 7;
  }

  for node in xq_p10_active_perception xq_p10_information_map xq_p10_flight_controller; do
    ros2 node info "/${node}" >"${arm_dir}/${node}-graph.txt"
    subscribers="$(sed -n '/Subscribers:/,/Publishers:/p' "${arm_dir}/${node}-graph.txt")"
    if grep -q '/xq/eval/\|ground_truth' <<<"${subscribers}"; then
      echo "Ground Truth leaked into algorithm node ${node}." >&2; exit 8
    fi
  done
  stop_groups
  [[ -f "${arm_dir}/rosbag/metadata.yaml" ]] || {
    echo "P10 ${variant} rosbag metadata missing." >&2; exit 9;
  }
done

python3 "${script_dir}/analyze_p10_gate.py" "${result_dir}" "${thresholds}" \
  >"${result_dir}/summary.stdout.json"
finish_audit
echo "PASS: P10 long-corridor three-arm flight Gate completed."
echo "Results: ${result_dir}"
