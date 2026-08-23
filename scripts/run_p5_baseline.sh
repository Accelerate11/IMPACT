#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
workspace_root="$(cd -- "${script_dir}/.." && pwd -P)"
requested_run_dir=""
phase6=false
phase8=false
gazebo_gui=false
gazebo_record=false
while (($#)); do
  case "$1" in
    --run-dir) requested_run_dir="$2"; shift 2 ;;
    --phase6) phase6=true; shift ;;
    --phase8) phase8=true; shift ;;
    --gazebo-gui) gazebo_gui=true; shift ;;
    --gazebo-record) gazebo_record=true; shift ;;
    -h|--help) echo "Usage: $0 [--run-dir PATH] [--phase6|--phase8] [--gazebo-gui] [--gazebo-record]"; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done
[[ "${phase6}" == false || "${phase8}" == false ]] || { echo "Choose only one phase extension." >&2; exit 2; }

for required in \
  "${workspace_root}/xq_install/setup.bash" \
  "${workspace_root}/xq_install/.xq_build_manifest.json" \
  "${workspace_root}/scripts/audit_external_assets.sh" \
  "${workspace_root}/xq_install/ego_planner/lib/ego_planner/ego_planner_node" \
  "${workspace_root}/xq_install/xq_autonomy/share/xq_autonomy/config/xq_p4_extnav.parm" \
  /home/accelerate/ardupilot/build/sitl/bin/arducopter \
  /home/accelerate/ardupilot_gazebo/build/libArduPilotPlugin.so; do
  [[ -e "${required}" ]] || { echo "Missing dependency: ${required}" >&2; exit 2; }
done
if ss -H -ltnp | grep -Eq '(:|\])5760[[:space:]]'; then
  echo "TCP 5760 is in use; refusing to disturb another SITL." >&2; exit 3
fi
if ss -H -lunp | grep -Eq '(:|\])9002[[:space:]]'; then
  echo "UDP 9002 is in use; refusing to disturb another vehicle." >&2; exit 3
fi

timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
if [[ -n "${requested_run_dir}" ]]; then
  run_dir="$(realpath -m -- "${requested_run_dir}")"
elif [[ "${phase6}" == true ]]; then
  run_dir="${workspace_root}/experiments/results/impact_p6/p6_${timestamp}_$$"
elif [[ "${phase8}" == true ]]; then
  run_dir="${workspace_root}/experiments/results/impact_p8/p8_live_${timestamp}_$$"
else
  run_dir="${workspace_root}/experiments/results/baseline_v1/p5_${timestamp}_$$"
fi
case "${run_dir}" in
  "${workspace_root}"/experiments/results/baseline_v1/*) [[ "${phase6}" == false && "${phase8}" == false ]] || { echo "Phase extensions cannot write into baseline_v1." >&2; exit 2; } ;;
  "${workspace_root}"/experiments/results/impact_p6/*) [[ "${phase6}" == true ]] || { echo "P5 baseline cannot write into impact_p6." >&2; exit 2; } ;;
  "${workspace_root}"/experiments/results/impact_p8/*) [[ "${phase8}" == true ]] || { echo "Only P8 live may write into impact_p8." >&2; exit 2; } ;;
  *) echo "Run directory is outside the phase-owned results directory." >&2; exit 2 ;;
esac
mkdir -p -- "${run_dir}/ros_logs" "${run_dir}/sitl_runtime"

unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
unset GZ_SIM_RESOURCE_PATH IGN_GAZEBO_RESOURCE_PATH SDF_PATH
unset RMW_IMPLEMENTATION FASTRTPS_DEFAULT_PROFILES_FILE CYCLONEDDS_URI
set +u
source /opt/ros/humble/setup.bash
source "${workspace_root}/xq_install/setup.bash"
set -u

export ROS_DOMAIN_ID=$((182 + (10#$(date +%S) + $$) % 50))
export ROS_LOCALHOST_ONLY=1
export ROS2CLI_NO_DAEMON=1
export GZ_PARTITION="xq_p5_${USER:-wsl}_${timestamp}_$$"
export ROS_LOG_DIR="${run_dir}/ros_logs"
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe
export GALLIUM_DRIVER=llvmpipe
export EGL_PLATFORM=surfaceless
export QT_QPA_PLATFORM=offscreen
export GZ_SIM_RESOURCE_PATH="${workspace_root}/xq_install/xq_gz_assets/share/xq_gz_assets/models:/home/accelerate/ardupilot_gazebo/models:/home/accelerate/ardupilot_gazebo/worlds"
export IGN_GAZEBO_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH}"
export SDF_PATH="${GZ_SIM_RESOURCE_PATH}"
export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/accelerate/ardupilot_gazebo/build

world="${workspace_root}/xq_install/xq_gz_assets/share/xq_gz_assets/worlds/xq_p5_structured_room.sdf"
fcu_params="${workspace_root}/xq_install/xq_autonomy/share/xq_autonomy/config/xq_p4_extnav.parm"
mission_result="${run_dir}/mission-result.json"
evaluation_result="${run_dir}/evaluation-result.json"
integrity_result="${run_dir}/integrity-result.json"
alert_limit_result="${run_dir}/alert-limit-result.json"

{
  echo "run_started_utc=${timestamp}"
  echo "baseline=BASELINE_V1"
  echo "navigation_resolution_m=0.10"
  echo "evaluation_resolution_m=0.05"
  echo "frontier_objective=J=I-lambda*d"
  echo "ego_upstream_commit=23a8d5a191711dd65633df689b0b37ac07718416"
  echo "ros_domain_id=${ROS_DOMAIN_ID}"
  echo "gz_partition=${GZ_PARTITION}"
  echo "world=${world}"
  echo "phase6_directional_integrity=${phase6}"
  echo "phase8_alert_limit=${phase8}"
  echo "gazebo_gui=${gazebo_gui}"
  echo "gazebo_record=${gazebo_record}"
} >"${run_dir}/run.env"
sha256sum "${world}" "${fcu_params}" \
  "${workspace_root}/xq_install/ego_planner/lib/ego_planner/ego_planner_node" \
  "${workspace_root}/xq_install/ego_planner/lib/ego_planner/traj_server" \
  /home/accelerate/ardupilot/build/sitl/bin/arducopter \
  /home/accelerate/ardupilot_gazebo/build/libArduPilotPlugin.so \
  >"${run_dir}/runtime-dependencies.sha256"
cp -- "${workspace_root}/xq_install/.xq_build_manifest.json" "${run_dir}/xq-build-manifest.json"

before_audit="${run_dir}/external-assets.before.sha256"
after_audit="${run_dir}/external-assets.after.sha256"
bash "${script_dir}/audit_external_assets.sh" snapshot "${before_audit}" >/dev/null
declare -a pids=() labels=()
audit_done=false

start_group() {
  local label="$1" log="$2"; shift 2
  setsid "$@" >"${log}" 2>&1 < /dev/null &
  local pid=$! pgid
  sleep 0.2
  pgid="$(ps -o pgid= -p "${pid}" | tr -d '[:space:]')"
  [[ "${pgid}" == "${pid}" ]] || { echo "Process isolation failed: ${label}" >&2; return 4; }
  pids+=("${pid}"); labels+=("${label}"); echo "${pid}" >"${run_dir}/${label}.pid"
}
stop_groups() {
  local i pid pgid
  for ((i=${#pids[@]}-1; i>=0; i--)); do
    pid="${pids[i]}"; pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d '[:space:]' || true)"
    [[ "${pgid}" == "${pid}" ]] && kill -INT -- "-${pgid}" 2>/dev/null || true
  done
  for _ in {1..12}; do
    local alive=false; for pid in "${pids[@]}"; do kill -0 "${pid}" 2>/dev/null && alive=true; done
    [[ "${alive}" == false ]] && break; sleep 1
  done
  for pid in "${pids[@]}"; do
    pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d '[:space:]' || true)"
    [[ "${pgid}" == "${pid}" ]] && kill -TERM -- "-${pgid}" 2>/dev/null || true
  done
  for _ in {1..5}; do
    local alive=false; for pid in "${pids[@]}"; do kill -0 "${pid}" 2>/dev/null && alive=true; done
    [[ "${alive}" == false ]] && break; sleep 1
  done
  for pid in "${pids[@]}"; do
    pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d '[:space:]' || true)"
    [[ "${pgid}" == "${pid}" ]] && kill -KILL -- "-${pgid}" 2>/dev/null || true
  done
  for pid in "${pids[@]}"; do wait "${pid}" 2>/dev/null || true; done
}
finish_audit() {
  [[ "${audit_done}" == false ]] || return 0
  bash "${script_dir}/audit_external_assets.sh" snapshot "${after_audit}" >/dev/null
  bash "${script_dir}/audit_external_assets.sh" compare "${before_audit}" "${after_audit}" >"${run_dir}/isolation-audit.txt"
  audit_done=true
}
cleanup() {
  local status=$?; trap - EXIT INT TERM; set +e
  stop_groups; finish_audit || status=$?; exit "${status}"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

wait_log() {
  local file="$1" pattern="$2" timeout_s="$3" label="$4" deadline=$((SECONDS + $3))
  while ((SECONDS < deadline)); do
    [[ -f "${file}" ]] && grep -Fq -- "${pattern}" "${file}" && return 0
    sleep 0.5
  done
  echo "Timeout waiting for ${label}: ${pattern}" >&2; return 1
}
assert_core_alive() {
  local i; for i in 0 1 2 3 4; do
    kill -0 "${pids[i]}" 2>/dev/null || { echo "Core exited: ${labels[i]}" >&2; return 1; }
  done
}
start_gazebo_gui() {
  [[ "${gazebo_gui}" == true ]] || return 0
  # WSLg's D3D12 OpenGL path is unstable with gz-gui / OGRE2 on this image.
  # Defer the known-stable visible X11 + llvmpipe client until the flight stack
  # is healthy, and lower its priority so visualization cannot starve SITL,
  # sensor rendering, localization, or planning. LP_NUM_THREADS is deliberately
  # left unset: limiting it crashes this OGRE2 / llvmpipe combination.
  start_group gazebo_gui "${run_dir}/gazebo-gui.log" nice -n 10 env \
    -u QT_QPA_PLATFORM -u EGL_PLATFORM gz sim -g -v 3
  gazebo_gui_pid="${pids[-1]}"
  sleep 3
  kill -0 "${gazebo_gui_pid}" 2>/dev/null || {
    echo "Gazebo GUI client exited during startup; see ${run_dir}/gazebo-gui.log" >&2
    exit 19
  }
  bash "${script_dir}/configure_gazebo_view.sh" "${run_dir}/gazebo-view.txt" xq_p5_structured_room || {
    echo "Gazebo GUI view setup failed; see ${run_dir}/gazebo-view.txt" >&2
    exit 20
  }
}

pushd "${run_dir}/sitl_runtime" >/dev/null
start_group sitl "${run_dir}/sitl.log" \
  /home/accelerate/ardupilot/build/sitl/bin/arducopter \
  -S --model JSON --speedup 1 --slave 0 --wipe \
  --defaults "/home/accelerate/ardupilot/Tools/autotest/default_params/copter.parm,/home/accelerate/ardupilot/Tools/autotest/default_params/gazebo-iris.parm,${fcu_params}" \
  --sim-address=127.0.0.1 -I0
popd >/dev/null
wait_log "${run_dir}/sitl.log" "SERIAL0 on TCP port 5760" 30 SITL
start_group mavros "${run_dir}/mavros.log" ros2 launch mavros apm.launch fcu_url:=tcp://127.0.0.1:5760 namespace:=uav1/mavros
wait_log "${run_dir}/sitl.log" "Loaded defaults" 45 defaults
gazebo_command=(gz sim -r -s --headless-rendering -v 3)
if [[ "${gazebo_record}" == true ]]; then
  gazebo_command+=(--record-path "${run_dir}/gz_record" --record-period 0.05)
fi
gazebo_command+=("${world}")
start_group gazebo "${run_dir}/gazebo.log" "${gazebo_command[@]}"
wait_log "${run_dir}/sitl.log" "JSON received" 90 Gazebo
wait_log "${run_dir}/mavros.log" "Got HEARTBEAT" 60 MAVROS
bag_topics=(
  /clock /livox/lidar /livox/imu /localization/odom /xq/p5/ego_odom /xq/p5/cloud_map
  /xq/p5/navigation_map /xq/p5/frontiers /xq/p5/frontier_goal /xq/p5/exploration/status
  /planning/bspline /position_cmd /xq/p5/ego_adapter/status
  /uav1/mavros/odometry/out /uav1/mavros/local_position/odom /uav1/mavros/state
  /xq/eval/p5/ground_truth
)
if [[ "${phase6}" == true ]]; then
  bag_topics+=(/localization/geometry /integrity/directional /integrity/debug)
fi
if [[ "${phase8}" == true ]]; then
  bag_topics+=(/integrity/alert_limit /integrity/alert_limit_debug)
fi
start_group rosbag "${run_dir}/rosbag.log" ros2 bag record --compression-mode file --compression-format zstd -o "${run_dir}/rosbag" "${bag_topics[@]}"
if [[ "${phase6}" == true ]]; then
  start_group p5_stack "${run_dir}/p5-stack.log" ros2 launch xq_sim_bringup xq_p6_directional_integrity.launch.py \
    evaluation_result_file:="${evaluation_result}" integrity_result_file:="${integrity_result}"
elif [[ "${phase8}" == true ]]; then
  start_group p5_stack "${run_dir}/p5-stack.log" ros2 launch xq_sim_bringup xq_p8_live.launch.py \
    evaluation_result_file:="${evaluation_result}" alert_limit_result_file:="${alert_limit_result}"
else
  start_group p5_stack "${run_dir}/p5-stack.log" ros2 launch xq_sim_bringup xq_p5_baseline.launch.py evaluation_result_file:="${evaluation_result}"
fi

timeout 110 ros2 topic echo --once /localization/odom >"${run_dir}/first-localization-odom.txt" 2>&1 || {
  echo "FAST-LIO did not publish." >&2; exit 5;
}
if [[ "${phase6}" == true ]]; then
  timeout 30 ros2 topic echo --once /integrity/directional >"${run_dir}/first-directional-integrity.txt" 2>&1 || {
    echo "P6 Directional Integrity did not publish." >&2; exit 6;
  }
fi
assert_core_alive
ros2 topic list -t >"${run_dir}/ros-topics.txt"
ros2 node list >"${run_dir}/ros-nodes.txt"
ros2 topic info /xq/p5/frontier_goal -v >"${run_dir}/frontier-goal-graph.txt"
ros2 topic info /xq/eval/p5/ground_truth -v >"${run_dir}/ground-truth-graph.txt"
if [[ "${phase6}" == true ]]; then
  ros2 node info /xq_p6_directional_integrity >"${run_dir}/p6-node-graph.txt"
fi
if [[ "${phase8}" == true ]]; then
  ros2 node info /xq_p8_alert_limit >"${run_dir}/p8-node-graph.txt"
fi

start_group mission "${run_dir}/mission.log" ros2 run xq_autonomy xq_p5_mission --ros-args \
  -r __node:=xq_p5_mission -p result_file:="${mission_result}" \
  -p mission_timeout_s:=430.0 -p takeoff_altitude_m:=2.0
if [[ "${phase8}" == true ]]; then
  timeout 180 ros2 topic echo --once /integrity/alert_limit >"${run_dir}/first-alert-limit.txt" 2>&1 || {
    echo "P8 Alert Limit did not publish after mission start." >&2; exit 6;
  }
fi
start_gazebo_gui
deadline=$((SECONDS + 450))
while [[ ! -s "${mission_result}" ]] && ((SECONDS < deadline)); do assert_core_alive; sleep 1; done
[[ -s "${mission_result}" ]] || { echo "Mission result missing." >&2; exit 7; }
[[ "$(python3 -c 'import json,sys;print(json.load(open(sys.argv[1]))["status"])' "${mission_result}")" == PASS ]] || {
  cat "${mission_result}" >&2; exit 8;
}
sleep 2
[[ -s "${evaluation_result}" ]] || { echo "Evaluation result missing." >&2; exit 9; }
[[ "$(python3 -c 'import json,sys;print(json.load(open(sys.argv[1]))["status"])' "${evaluation_result}")" == PASS ]] || {
  cat "${evaluation_result}" >&2; exit 10;
}
if [[ "${phase6}" == true ]]; then
  [[ -s "${integrity_result}" ]] || { echo "P6 integrity result missing." >&2; exit 13; }
  [[ "$(python3 -c 'import json,sys;print(json.load(open(sys.argv[1]))["status"])' "${integrity_result}")" == PASS ]] || {
    cat "${integrity_result}" >&2; exit 14;
  }
  if grep -Fq '/xq/eval/p5/ground_truth' "${run_dir}/p6-node-graph.txt"; then
    echo "Ground Truth leaked into P6 predictor." >&2; exit 15
  fi
fi
if [[ "${phase8}" == true ]]; then
  [[ -s "${alert_limit_result}" ]] || { echo "P8 Alert Limit result missing." >&2; exit 16; }
  [[ "$(python3 -c 'import json,sys;print(json.load(open(sys.argv[1]))["status"])' "${alert_limit_result}")" == PASS ]] || {
    cat "${alert_limit_result}" >&2; exit 17;
  }
  if grep -Fq '/xq/eval/' "${run_dir}/p8-node-graph.txt"; then
    echo "Ground Truth leaked into P8 Alert Limit." >&2; exit 18
  fi
fi
if grep -Eq '^Node name: (xq_fast_lio|xq_p5_frontier|xq_p5_ego_planner|xq_p5_ego_command|xq_p5_mission)$' "${run_dir}/ground-truth-graph.txt"; then
  echo "Ground truth leaked into autonomy." >&2; exit 11
fi
grep -q '^Publisher count: 1$' "${run_dir}/frontier-goal-graph.txt" &&
grep -q '^Node name: xq_p5_frontier$' "${run_dir}/frontier-goal-graph.txt" &&
grep -q '^Node name: xq_p5_ego_planner$' "${run_dir}/frontier-goal-graph.txt" || {
  echo "Frontier goal publisher graph is wrong." >&2; exit 12;
}

stop_groups
finish_audit
python3 - "${run_dir}" "${phase6}" "${phase8}" "${gazebo_gui}" "${gazebo_record}" <<'PY'
import json, sys
from datetime import datetime, timezone
from pathlib import Path
run = Path(sys.argv[1])
phase6 = sys.argv[2].lower() == "true"
phase8 = sys.argv[3].lower() == "true"
gazebo_gui = sys.argv[4].lower() == "true"
gazebo_record = sys.argv[5].lower() == "true"
mission = json.loads((run / "mission-result.json").read_text())
evaluation = json.loads((run / "evaluation-result.json").read_text())
summary = {
    "schema_version": 1,
    "gate": "P6_DIRECTIONAL_INTEGRITY_PREDICTOR" if phase6 else ("P8_STATIC_OBSTACLE_ALERT_LIMIT_LIVE" if phase8 else "P5_BASELINE_MAP_FRONTIER_EGO"),
    "baseline": "BASELINE_V1+P6_NON_CONTROLLING" if phase6 else ("BASELINE_V1+P8_NON_CONTROLLING" if phase8 else "BASELINE_V1"),
    "status": "PASS", "mission_checks": mission["checks"], "exploration": mission["exploration"],
    "ego_bspline_count": mission["bspline_count"], "evaluation": evaluation,
    "rosbag_present": (run / "rosbag" / "metadata.yaml").is_file(),
    "ground_truth_isolated": True,
    "external_assets_unchanged": "PASS:" in (run / "isolation-audit.txt").read_text(),
    "gazebo_gui_requested": gazebo_gui,
    "gazebo_record_present": (run / "gz_record").is_dir() if gazebo_record else False,
    "generated_at_utc": datetime.now(timezone.utc).isoformat(),
}
if phase6:
    summary["directional_integrity"] = json.loads((run / "integrity-result.json").read_text())
if phase8:
    summary["alert_limit"] = json.loads((run / "alert-limit-result.json").read_text())
(run / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n")
print(json.dumps(summary, ensure_ascii=False, indent=2))
PY
if [[ "${phase6}" == true ]]; then
  echo "PASS: P6 Directional Integrity Predictor ran on the live P5 autonomy stack."
elif [[ "${phase8}" == true ]]; then
  echo "PASS: P8 Alert Limit ran on the live Gazebo P5 autonomy stack."
else
  echo "PASS: P5 BASELINE_V1 autonomous exploration completed."
fi
echo "Results: ${run_dir}"
