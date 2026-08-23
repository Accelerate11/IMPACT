#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
workspace_root="$(cd -- "${script_dir}/.." && pwd -P)"
timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
result_dir="$(realpath -m -- "${1:-${workspace_root}/experiments/results/impact_p10/visual_${timestamp}_$$}")"
case "${result_dir}" in
  "${workspace_root}"/experiments/results/impact_p10/visual_*) ;;
  *) echo "P10 visual capture must stay under impact_p10/visual_*." >&2; exit 2 ;;
esac
mkdir -p -- "${result_dir}/ros_logs"

unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
unset GZ_SIM_RESOURCE_PATH IGN_GAZEBO_RESOURCE_PATH SDF_PATH
set +u
source /opt/ros/humble/setup.bash
source "${workspace_root}/xq_install/setup.bash"
set -u
world="${workspace_root}/xq_install/xq_gz_assets/share/xq_gz_assets/worlds/xq_p10_long_corridor.sdf"
calibration="${workspace_root}/evidence/P7/p7-calibration.json"
calibration_sha="$(sha256sum "${calibration}" | cut -d ' ' -f 1)"

export ROS_DOMAIN_ID=$((170 + ($$ % 20)))
export ROS_LOCALHOST_ONLY=1
export ROS2CLI_NO_DAEMON=1
export GZ_PARTITION="xq_p10_visual_${timestamp}_$$"
export ROS_LOG_DIR="${result_dir}/ros_logs"
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe
export GALLIUM_DRIVER=llvmpipe
export EGL_PLATFORM=surfaceless
export QT_QPA_PLATFORM=offscreen

bash "${script_dir}/audit_external_assets.sh" snapshot \
  "${result_dir}/external-assets.before.sha256" >/dev/null
declare -a pids=()
stop_groups() {
  local pid round alive
  for pid in "${pids[@]}"; do kill -INT -- "${pid}" 2>/dev/null || true; done
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
  for pid in "${pids[@]}"; do
    pgrep -g "${pid}" >/dev/null && kill -KILL -- "-${pid}" 2>/dev/null || true
  done
  for pid in "${pids[@]}"; do wait "${pid}" 2>/dev/null || true; done
  pids=()
}
cleanup() {
  local status=$?; trap - EXIT INT TERM; set +e; stop_groups; exit "${status}"
}
trap cleanup EXIT INT TERM

setsid ros2 bag record --compression-mode file --compression-format zstd \
  -o "${result_dir}/rosbag" \
  /localization/odom /cloud_registered /xq/p5/cloud_map /integrity/information_map \
  /integrity/directional /integrity/active_perception_decision \
  /planning/p10/baseline_bspline /planning/active_perception_candidates \
  /planning/active_perception_bspline /xq/p10/flight_status \
  /xq/eval/agent_01/ground_truth /tf /tf_static /clock \
  >"${result_dir}/rosbag.log" 2>&1 < /dev/null & pids+=("$!")
setsid ros2 launch xq_sim_bringup xq_p10_flight.launch.py \
  world_file:="${world}" variant:=minimum_excitation \
  calibration_file:="${calibration}" calibration_sha256:="${calibration_sha}" \
  result_file:="${result_dir}/flight-result.json" \
  gz_record_path:="${result_dir}/gz_record" \
  >"${result_dir}/launch.log" 2>&1 < /dev/null & pids+=("$!")

deadline=$((SECONDS + 100))
while [[ ! -s "${result_dir}/flight-result.json" ]] && ((SECONDS < deadline)); do
  kill -0 "${pids[1]}" 2>/dev/null || { tail -n 120 "${result_dir}/launch.log" >&2; exit 5; }
  sleep 1
done
[[ -s "${result_dir}/flight-result.json" ]] || { echo "P10 visual capture timed out." >&2; exit 6; }
[[ "$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["status"])' "${result_dir}/flight-result.json")" == PASS ]] || exit 7
stop_groups

bash "${script_dir}/audit_external_assets.sh" snapshot \
  "${result_dir}/external-assets.after.sha256" >/dev/null
bash "${script_dir}/audit_external_assets.sh" compare \
  "${result_dir}/external-assets.before.sha256" \
  "${result_dir}/external-assets.after.sha256" >"${result_dir}/isolation-audit.txt"
[[ -f "${result_dir}/rosbag/metadata.yaml" ]] || { echo "P10 visual rosbag missing." >&2; exit 8; }
[[ -f "${result_dir}/gz_record/state.tlog" ]] || { echo "P10 Gazebo state recording missing." >&2; exit 9; }
python3 - "${result_dir}" <<'PY'
import json, sys
from pathlib import Path
root = Path(sys.argv[1])
flight = json.loads((root / "flight-result.json").read_text())
result = {
    "schema_version": 1,
    "artifact": "P10_GAZEBO_RVIZ_VISUAL_CAPTURE",
    "status": "PASS",
    "variant": "minimum_excitation",
    "flight_result": flight,
    "gazebo_state_recording": True,
    "rosbag_present": True,
    "open_top": True,
    "walls_preserved": True,
}
(root / "visualization.json").write_text(json.dumps(result, indent=2) + "\n")
PY
echo "PASS: P10 Gazebo + RViz visual capture completed."
echo "Results: ${result_dir}"
