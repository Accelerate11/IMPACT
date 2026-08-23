#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
workspace_root="$(cd -- "${script_dir}/.." && pwd -P)"
requested_run_dir=""
minimum_eval_duration_s=70

usage() {
  echo "Usage: $0 [--minimum-eval-duration SECONDS] [--run-dir PATH]" >&2
}

while (($#)); do
  case "$1" in
    --minimum-eval-duration) minimum_eval_duration_s="$2"; shift 2 ;;
    --run-dir) requested_run_dir="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) usage; exit 2 ;;
  esac
done
[[ "${minimum_eval_duration_s}" =~ ^[1-9][0-9]*$ ]] || {
  echo "Invalid minimum evaluation duration." >&2; exit 2;
}
((minimum_eval_duration_s >= 60)) || {
  echo "P4 evaluation must cover at least 60 seconds." >&2; exit 2;
}

for required in \
  "${workspace_root}/xq_install/setup.bash" \
  "${workspace_root}/scripts/audit_external_assets.sh" \
  "${workspace_root}/xq_install/xq_autonomy/share/xq_autonomy/config/xq_p4_extnav.parm" \
  "/home/accelerate/ardupilot/build/sitl/bin/arducopter" \
  "/home/accelerate/ardupilot_gazebo/build/libArduPilotPlugin.so"; do
  [[ -e "${required}" ]] || { echo "Missing dependency: ${required}" >&2; exit 2; }
done

if ss -H -ltnp | grep -Eq '(:|\])5760[[:space:]]'; then
  echo "TCP 5760 is already in use; refusing to interfere with another SITL." >&2
  exit 3
fi
if ss -H -lunp | grep -Eq '(:|\])9002[[:space:]]'; then
  echo "UDP 9002 is already in use; refusing to interfere with another Gazebo vehicle." >&2
  exit 3
fi

timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
if [[ -n "${requested_run_dir}" ]]; then
  run_dir="$(realpath -m -- "${requested_run_dir}")"
else
  run_dir="${workspace_root}/experiments/results/external_nav/p4_${timestamp}_$$"
fi
case "${run_dir}" in
  "${workspace_root}"/experiments/results/external_nav/*) ;;
  *) echo "Run directory must stay below experiments/results/external_nav." >&2; exit 2 ;;
esac
mkdir -p -- "${run_dir}/ros_logs" "${run_dir}/sitl_runtime"

unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
unset GZ_SIM_RESOURCE_PATH IGN_GAZEBO_RESOURCE_PATH SDF_PATH
unset RMW_IMPLEMENTATION FASTRTPS_DEFAULT_PROFILES_FILE CYCLONEDDS_URI
set +u
source /opt/ros/humble/setup.bash
source "${workspace_root}/xq_install/setup.bash"
set -u

export ROS_DOMAIN_ID=$((102 + (10#$(date +%S) + $$) % 80))
export ROS_LOCALHOST_ONLY=1
export ROS2CLI_NO_DAEMON=1
export GZ_PARTITION="xq_p4_${USER:-wsl}_${timestamp}_$$"
export ROS_LOG_DIR="${run_dir}/ros_logs"
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe
export GALLIUM_DRIVER=llvmpipe
export EGL_PLATFORM=surfaceless
export QT_QPA_PLATFORM=offscreen
export GZ_SIM_RESOURCE_PATH="${workspace_root}/xq_install/xq_gz_assets/share/xq_gz_assets/models:/home/accelerate/ardupilot_gazebo/models:/home/accelerate/ardupilot_gazebo/worlds"
export IGN_GAZEBO_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH}"
export SDF_PATH="${GZ_SIM_RESOURCE_PATH}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="/home/accelerate/ardupilot_gazebo/build"

world="${workspace_root}/xq_install/xq_gz_assets/share/xq_gz_assets/worlds/xq_p4_external_nav.sdf"
p4_params="${workspace_root}/xq_install/xq_autonomy/share/xq_autonomy/config/xq_p4_extnav.parm"
mission_result="${run_dir}/mission-result.json"
evaluation_result="${run_dir}/localization-evaluation.json"
[[ -f "${world}" ]] || { echo "Installed P4 world is missing." >&2; exit 2; }

cat >"${run_dir}/run.env" <<EOF
run_started_utc=${timestamp}
minimum_eval_duration_s=${minimum_eval_duration_s}
ros_domain_id=${ROS_DOMAIN_ID}
gz_partition=${GZ_PARTITION}
world=${world}
mavros_namespace=/uav1/mavros
external_nav_topic=/uav1/mavros/odometry/out
gps_disabled=GPS_TYPE:0,SIM_GPS_DISABLE:1
sitl_runtime_dir=${run_dir}/sitl_runtime
EOF

sha256sum \
  "${world}" \
  "${p4_params}" \
  "${workspace_root}/xq_install/xq_gz_assets/share/xq_gz_assets/models/xq_iris_mid360_ardupilot/model.sdf" \
  "${workspace_root}/xq_install/xq_fast_lio/share/xq_fast_lio/config/xq_p4.yaml" \
  "${workspace_root}/xq_install/xq_gz_bridge/share/xq_gz_bridge/config/p4_external_nav.yaml" \
  /home/accelerate/ardupilot/build/sitl/bin/arducopter \
  /home/accelerate/ardupilot/Tools/autotest/default_params/copter.parm \
  /home/accelerate/ardupilot/Tools/autotest/default_params/gazebo-iris.parm \
  /home/accelerate/ardupilot_gazebo/build/libArduPilotPlugin.so \
  /home/accelerate/ardupilot_gazebo/models/iris_with_ardupilot/model.sdf \
  /home/accelerate/ardupilot_gazebo/models/iris_with_standoffs/model.sdf \
  >"${run_dir}/runtime-dependencies.sha256"
cp -- "${workspace_root}/xq_install/.xq_build_manifest.json" "${run_dir}/xq-build-manifest.json"

before_audit="${run_dir}/external-assets.before.sha256"
after_audit="${run_dir}/external-assets.after.sha256"
bash "${script_dir}/audit_external_assets.sh" snapshot "${before_audit}" >/dev/null

declare -a pids=()
declare -a labels=()
cleanup_done=false

start_group() {
  local label="$1" log_file="$2"
  shift 2
  setsid "$@" >"${log_file}" 2>&1 < /dev/null &
  local pid=$!
  local pgid
  sleep 0.2
  pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d '[:space:]')"
  [[ "${pgid}" == "${pid}" ]] || {
    echo "Failed to isolate process group for ${label}." >&2; return 4;
  }
  pids+=("${pid}")
  labels+=("${label}")
  echo "${pid}" >"${run_dir}/${label}.pid"
}

stop_groups() {
  local index pid pgid round
  for ((index=${#pids[@]}-1; index>=0; index--)); do
    pid="${pids[index]}"
    pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d '[:space:]' || true)"
    [[ "${pgid}" == "${pid}" ]] && kill -INT -- "-${pgid}" 2>/dev/null || true
  done
  for round in 1 2 3 4 5 6 7 8 9 10; do
    local alive=false
    for pid in "${pids[@]}"; do kill -0 "${pid}" 2>/dev/null && alive=true; done
    [[ "${alive}" == false ]] && break
    sleep 1
  done
  for pid in "${pids[@]}"; do
    pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d '[:space:]' || true)"
    [[ "${pgid}" == "${pid}" ]] && kill -TERM -- "-${pgid}" 2>/dev/null || true
  done
  for round in 1 2 3 4 5; do
    local alive=false
    for pid in "${pids[@]}"; do kill -0 "${pid}" 2>/dev/null && alive=true; done
    [[ "${alive}" == false ]] && break
    sleep 1
  done
  for pid in "${pids[@]}"; do
    pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d '[:space:]' || true)"
    [[ "${pgid}" == "${pid}" ]] && kill -KILL -- "-${pgid}" 2>/dev/null || true
  done
  for pid in "${pids[@]}"; do wait "${pid}" 2>/dev/null || true; done
}

finish_audit() {
  [[ "${cleanup_done}" == false ]] || return 0
  bash "${script_dir}/audit_external_assets.sh" snapshot "${after_audit}" >/dev/null
  bash "${script_dir}/audit_external_assets.sh" compare \
    "${before_audit}" "${after_audit}" >"${run_dir}/isolation-audit.txt" 2>&1
  cleanup_done=true
}

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  set +e
  stop_groups
  finish_audit
  local audit_status=$?
  ((audit_status == 0)) || status="${audit_status}"
  exit "${status}"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

wait_log() {
  local file="$1" pattern="$2" timeout_s="$3" label="$4"
  local deadline=$((SECONDS + timeout_s))
  while ((SECONDS < deadline)); do
    [[ -f "${file}" ]] && grep -Fq -- "${pattern}" "${file}" && return 0
    sleep 0.5
  done
  echo "Timeout waiting for ${label}: ${pattern}" >&2
  return 1
}

assert_core_alive() {
  local index
  for index in 0 1 2 3 4; do
    if ! kill -0 "${pids[index]}" 2>/dev/null; then
      echo "Core process exited early: ${labels[index]}" >&2
      return 1
    fi
  done
}

pushd "${run_dir}/sitl_runtime" >/dev/null
start_group sitl "${run_dir}/sitl.log" \
  /home/accelerate/ardupilot/build/sitl/bin/arducopter \
  -S --model JSON --speedup 1 --slave 0 --wipe \
  --defaults "/home/accelerate/ardupilot/Tools/autotest/default_params/copter.parm,/home/accelerate/ardupilot/Tools/autotest/default_params/gazebo-iris.parm,${p4_params}" \
  --sim-address=127.0.0.1 -I0
popd >/dev/null
wait_log "${run_dir}/sitl.log" "SERIAL0 on TCP port 5760" 30 "SITL MAVLink listener"

start_group mavros "${run_dir}/mavros.log" \
  ros2 launch mavros apm.launch \
  fcu_url:=tcp://127.0.0.1:5760 namespace:=uav1/mavros
wait_log "${run_dir}/sitl.log" "Loaded defaults" 45 "ArduPilot defaults"

start_group gazebo "${run_dir}/gazebo.log" \
  gz sim -r -s --headless-rendering -v 3 "${world}"
wait_log "${run_dir}/sitl.log" "JSON received" 90 "SITL-Gazebo JSON link"
wait_log "${run_dir}/mavros.log" "Got HEARTBEAT" 60 "MAVROS heartbeat"

start_group rosbag "${run_dir}/rosbag.log" \
  ros2 bag record --compression-mode file --compression-format zstd \
  -o "${run_dir}/rosbag" \
  /clock /livox/lidar /livox/imu /localization/odom \
  /uav1/mavros/odometry/out /uav1/mavros/local_position/odom \
  /uav1/mavros/state /uav1/mavros/imu/data \
  /xq/p4/extnav/status /xq/eval/p4/ground_truth

start_group p4_stack "${run_dir}/p4-stack.log" \
  ros2 launch xq_sim_bringup xq_p4_external_nav.launch.py \
  evaluation_result_file:="${evaluation_result}" \
  minimum_duration_s:="${minimum_eval_duration_s}.0"

# Wait for the real algorithm output, not only process startup.
timeout 100 ros2 topic echo --once /localization/odom \
  >"${run_dir}/first-localization-odom.txt" 2>&1 || {
    echo "FAST-LIO did not publish /localization/odom." >&2
    exit 5
  }
assert_core_alive

ros2 topic list -t >"${run_dir}/ros-topics.txt" 2>&1
ros2 node list >"${run_dir}/ros-nodes.txt" 2>&1
ros2 service list -t >"${run_dir}/ros-services.txt" 2>&1
gz topic -l >"${run_dir}/gz-topics.txt" 2>&1
ros2 topic info /uav1/mavros/odometry/out -v \
  >"${run_dir}/external-nav-topic-graph.txt" 2>&1
ros2 topic info /xq/eval/p4/ground_truth -v \
  >"${run_dir}/ground-truth-topic-graph.txt" 2>&1

grep -q '/uav1/mavros/odometry/out' "${run_dir}/ros-topics.txt" || {
  echo "MAVROS odometry input topic is absent." >&2; exit 6;
}
grep -q '/xq/p4/lidar/points' "${run_dir}/gz-topics.txt" || {
  echo "P4 LiDAR Gazebo topic is absent." >&2; exit 6;
}
grep -q '/xq/p4/imu' "${run_dir}/gz-topics.txt" || {
  echo "P4 IMU Gazebo topic is absent." >&2; exit 6;
}

start_group mission "${run_dir}/mission.log" \
  ros2 run xq_autonomy xq_p4_mission --ros-args \
  -p result_file:="${mission_result}" \
  -p mission_timeout_s:=240.0 \
  -p takeoff_altitude_m:=2.0 \
  -p square_side_m:=2.0 \
  -p hover_duration_s:=5.0

mission_deadline=$((SECONDS + 260))
while [[ ! -s "${mission_result}" ]] && ((SECONDS < mission_deadline)); do
  assert_core_alive
  sleep 1
done
[[ -s "${mission_result}" ]] || { echo "Mission result was not produced." >&2; exit 7; }
mission_status="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["status"])' "${mission_result}")"
[[ "${mission_status}" == "PASS" ]] || { cat "${mission_result}" >&2; exit 8; }

evaluation_deadline=$((SECONDS + minimum_eval_duration_s + 30))
evaluation_status="IN_PROGRESS"
while ((SECONDS < evaluation_deadline)); do
  assert_core_alive
  if [[ -s "${evaluation_result}" ]]; then
    evaluation_status="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["status"])' "${evaluation_result}")"
    [[ "${evaluation_status}" != "IN_PROGRESS" ]] && break
  fi
  sleep 1
done
[[ "${evaluation_status}" == "PASS" ]] || {
  [[ -s "${evaluation_result}" ]] && cat "${evaluation_result}" >&2
  echo "P4 in-flight FAST-LIO evaluation did not pass." >&2
  exit 9
}

timeout 10 ros2 topic echo --once /uav1/mavros/state \
  >"${run_dir}/final-state.txt" 2>&1
grep -q 'connected: true' "${run_dir}/final-state.txt" || {
  echo "MAVROS was not connected at P4 completion." >&2; exit 10;
}

# Ground truth must have exactly the project bridge publisher and evaluator
# subscriber.  The algorithm and flight-control nodes must not appear.
if grep -Eq 'Node name: /(xq_fast_lio|xq_p4_external_nav|xq_p4_mission)' \
  "${run_dir}/ground-truth-topic-graph.txt"; then
  echo "Ground truth leaked into an algorithm or flight-control node." >&2
  exit 11
fi

stop_groups
finish_audit

python3 - "${run_dir}" <<'PY'
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

run = Path(sys.argv[1])
mission = json.loads((run / "mission-result.json").read_text(encoding="utf-8"))
evaluation = json.loads((run / "localization-evaluation.json").read_text(encoding="utf-8"))
metadata = run / "rosbag" / "metadata.yaml"
isolation = (run / "isolation-audit.txt").read_text(encoding="utf-8")
summary = {
    "schema_version": 1,
    "gate": "P4_GPS_OFF_FAST_LIO_EXTERNAL_NAV_CLOSED_LOOP",
    "status": "PASS",
    "mission_status": mission["status"],
    "mission_elapsed_s": mission["elapsed_s"],
    "mission_checks": mission["checks"],
    "verified_parameters": mission["verified_parameters"],
    "external_nav": mission["external_nav"],
    "localization_status": evaluation["status"],
    "localization_metrics": evaluation["metrics"],
    "rosbag_present": metadata.is_file(),
    "ground_truth_isolated": True,
    "external_assets_unchanged": "PASS:" in isolation,
    "generated_at_utc": datetime.now(timezone.utc).isoformat(),
}
(run / "summary.json").write_text(
    json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
)
print(json.dumps(summary, ensure_ascii=False, indent=2))
PY

echo "PASS: P4 GPS-off FAST-LIO ExternalNav closed loop completed."
echo "Results: ${run_dir}"
