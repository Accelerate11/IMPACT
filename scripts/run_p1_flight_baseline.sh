#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
workspace_root="$(cd -- "${script_dir}/.." && pwd -P)"
duration_s=600
hover_s=20
requested_run_dir=""

usage() {
  echo "Usage: $0 [--duration SECONDS] [--hover SECONDS] [--run-dir PATH]" >&2
}

while (($#)); do
  case "$1" in
    --duration) duration_s="$2"; shift 2 ;;
    --hover) hover_s="$2"; shift 2 ;;
    --run-dir) requested_run_dir="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) usage; exit 2 ;;
  esac
done
[[ "${duration_s}" =~ ^[1-9][0-9]*$ ]] || { echo "Invalid duration" >&2; exit 2; }
[[ "${hover_s}" =~ ^[0-9]+([.][0-9]+)?$ ]] || { echo "Invalid hover duration" >&2; exit 2; }
((duration_s >= 90)) || { echo "P1 duration must be at least 90 seconds." >&2; exit 2; }
hover_parameter="${hover_s}"
[[ "${hover_parameter}" == *.* ]] || hover_parameter="${hover_parameter}.0"

for required in \
  "${workspace_root}/xq_install/setup.bash" \
  "${workspace_root}/scripts/audit_external_assets.sh" \
  "/home/accelerate/ardupilot/build/sitl/bin/arducopter" \
  "/home/accelerate/ardupilot_gazebo/build/libArduPilotPlugin.so"; do
  [[ -e "${required}" ]] || { echo "Missing dependency: ${required}" >&2; exit 2; }
done

# Instance 0 is the locally installed model's fixed interface. Never evict an
# existing user of either port; fail before changing any process state.
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
  run_dir="${workspace_root}/runs/p1_${timestamp}_$$"
fi
case "${run_dir}" in
  "${workspace_root}"/runs/*) ;;
  *) echo "Run directory must stay below ${workspace_root}/runs." >&2; exit 2 ;;
esac
mkdir -p -- "${run_dir}/ros_logs"

unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
unset GZ_SIM_RESOURCE_PATH IGN_GAZEBO_RESOURCE_PATH SDF_PATH
unset RMW_IMPLEMENTATION FASTRTPS_DEFAULT_PROFILES_FILE CYCLONEDDS_URI
set +u
source /opt/ros/humble/setup.bash
source "${workspace_root}/xq_install/setup.bash"
set -u

export ROS_DOMAIN_ID=$((32 + (10#$(date +%S) + $$) % 70))
export ROS_LOCALHOST_ONLY=1
export ROS2CLI_NO_DAEMON=1
export GZ_PARTITION="xq_p1_${USER:-wsl}_${timestamp}_$$"
export ROS_LOG_DIR="${run_dir}/ros_logs"
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe
export GALLIUM_DRIVER=llvmpipe
export EGL_PLATFORM=surfaceless
export QT_QPA_PLATFORM=offscreen
export GZ_SIM_RESOURCE_PATH="${workspace_root}/xq_install/xq_gz_assets/share/xq_gz_assets/models:/home/accelerate/ardupilot_gazebo/models:/home/accelerate/ardupilot_gazebo/worlds"
export IGN_GAZEBO_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="/home/accelerate/ardupilot_gazebo/build"

world="${workspace_root}/xq_install/xq_gz_assets/share/xq_gz_assets/worlds/xq_p1_ardupilot_empty.sdf"
mission_result="${run_dir}/mission-result.json"
[[ -f "${world}" ]] || { echo "Installed P1 world is missing; rebuild first." >&2; exit 2; }

cat >"${run_dir}/run.env" <<EOF
run_started_utc=${timestamp}
duration_s=${duration_s}
hover_s=${hover_s}
ros_domain_id=${ROS_DOMAIN_ID}
gz_partition=${GZ_PARTITION}
world=${world}
mavros_namespace=/uav1/mavros
sitl_tcp_port=5760
gazebo_fdm_udp_port=9002
EOF

sha256sum \
  "${world}" \
  /home/accelerate/ardupilot/build/sitl/bin/arducopter \
  /home/accelerate/ardupilot/Tools/autotest/default_params/copter.parm \
  /home/accelerate/ardupilot/Tools/autotest/default_params/gazebo-iris.parm \
  /home/accelerate/ardupilot_gazebo/build/libArduPilotPlugin.so \
  /home/accelerate/ardupilot_gazebo/models/iris_with_ardupilot/model.sdf \
  /home/accelerate/ardupilot_gazebo/models/iris_with_standoffs/model.sdf \
  >"${run_dir}/runtime-dependencies.sha256"

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
  if [[ "${pgid}" != "${pid}" ]]; then
    echo "Failed to isolate process group for ${label}." >&2
    return 4
  fi
  pids+=("${pid}")
  labels+=("${label}")
  echo "${pid}" >"${run_dir}/${label}.pid"
}

stop_groups() {
  local index pid pgid round
  for ((index=${#pids[@]}-1; index>=0; index--)); do
    pid="${pids[index]}"
    pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d '[:space:]' || true)"
    [[ "${pgid}" == "${pid}" ]] || continue
    kill -INT -- "-${pgid}" 2>/dev/null || true
  done
  for round in 1 2 3 4 5 6 7 8; do
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

run_started_seconds=${SECONDS}
pushd /home/accelerate/ardupilot >/dev/null
start_group sitl "${run_dir}/sitl.log" \
  /home/accelerate/ardupilot/build/sitl/bin/arducopter \
  -S --model JSON --speedup 1 --slave 0 \
  --defaults Tools/autotest/default_params/copter.parm,Tools/autotest/default_params/gazebo-iris.parm \
  --sim-address=127.0.0.1 -I0
popd >/dev/null
wait_log "${run_dir}/sitl.log" "SERIAL0 on TCP port 5760" 30 "SITL MAVLink listener"

start_group mavros "${run_dir}/mavros.log" \
  ros2 launch mavros apm.launch \
  fcu_url:=tcp://127.0.0.1:5760 namespace:=uav1/mavros
wait_log "${run_dir}/sitl.log" "Loaded defaults" 45 "ArduPilot boot"

start_group gazebo "${run_dir}/gazebo.log" gz sim -r -s -v 3 "${world}"
wait_log "${run_dir}/sitl.log" "JSON received" 75 "SITL-Gazebo JSON link"
wait_log "${run_dir}/mavros.log" "Got HEARTBEAT" 45 "MAVROS heartbeat"

ros2 service list >"${run_dir}/ros-services.txt" 2>&1

start_group rosbag "${run_dir}/rosbag.log" \
  ros2 bag record -o "${run_dir}/rosbag" \
  /uav1/mavros/state /uav1/mavros/local_position/odom \
  /uav1/mavros/imu/data /uav1/mavros/global_position/global

start_group mission "${run_dir}/mission.log" \
  ros2 run xq_autonomy xq_p1_flight_baseline --ros-args \
  -p mavros_prefix:=/uav1/mavros \
  -p takeoff_altitude_m:=2.0 \
  -p hover_duration_s:="${hover_parameter}" \
  -p mission_timeout_s:=150.0 \
  -p result_file:="${mission_result}"

mission_deadline=$((SECONDS + 165))
while [[ ! -s "${mission_result}" ]] && ((SECONDS < mission_deadline)); do
  for index in "${!pids[@]}"; do
    if ! kill -0 "${pids[index]}" 2>/dev/null; then
      echo "Process exited early: ${labels[index]}" >&2
      exit 5
    fi
  done
  sleep 1
done
[[ -s "${mission_result}" ]] || { echo "Mission result was not produced." >&2; exit 6; }
mission_status="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["status"])' "${mission_result}")"
[[ "${mission_status}" == "PASS" ]] || { cat "${mission_result}" >&2; exit 7; }

# P1's formal gate is continuous process/link stability. Keep the isolated
# stack alive until the requested wall duration even though flight is complete.
while ((SECONDS - run_started_seconds < duration_s)); do
  for index in 0 1 2; do
    if ! kill -0 "${pids[index]}" 2>/dev/null; then
      echo "Core process exited during stability dwell: ${labels[index]}" >&2
      exit 8
    fi
  done
  sleep 1
done

timeout 10 ros2 topic echo --once /uav1/mavros/state \
  >"${run_dir}/final-state.txt" 2>&1
grep -q 'connected: true' "${run_dir}/final-state.txt" || {
  echo "MAVROS was not connected at the end of the stability dwell." >&2
  exit 9
}

stop_groups
finish_audit
python3 - "${run_dir}" "${duration_s}" <<'PY'
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

run = Path(sys.argv[1])
mission = json.loads((run / "mission-result.json").read_text(encoding="utf-8"))
summary = {
    "schema_version": 1,
    "gate": "P1_ARDUPILOT_GAZEBO_MAVROS_BASELINE",
    "status": "PASS",
    "continuous_wall_duration_s": int(sys.argv[2]),
    "mission_status": mission["status"],
    "mission_elapsed_s": mission["elapsed_s"],
    "rosbag_present": (run / "rosbag" / "metadata.yaml").is_file(),
    "external_assets_unchanged": "PASS:" in (run / "isolation-audit.txt").read_text(),
    "generated_at_utc": datetime.now(timezone.utc).isoformat(),
}
(run / "summary.json").write_text(
    json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
)
print(json.dumps(summary, ensure_ascii=False, indent=2))
PY

echo "PASS: P1 flight baseline and stability dwell completed."
echo "Results: ${run_dir}"
