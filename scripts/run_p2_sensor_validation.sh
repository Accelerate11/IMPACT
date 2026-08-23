#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
workspace_root="$(cd -- "${script_dir}/.." && pwd -P)"
duration_s=600
requested_result_dir=""

usage() {
  echo "Usage: $0 [--duration SECONDS] [--result-dir PATH]" >&2
}

while (($#)); do
  case "$1" in
    --duration) duration_s="$2"; shift 2 ;;
    --result-dir) requested_result_dir="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) usage; exit 2 ;;
  esac
done
[[ "${duration_s}" =~ ^[1-9][0-9]*$ ]] || { echo "Invalid duration" >&2; exit 2; }
((duration_s >= 60)) || { echo "P2 duration must be at least 60 seconds." >&2; exit 2; }

for required in \
  "${workspace_root}/xq_install/setup.bash" \
  "${workspace_root}/xq_install/.xq_build_manifest.json" \
  "${workspace_root}/scripts/audit_external_assets.sh"; do
  [[ -f "${required}" ]] || { echo "Missing dependency: ${required}" >&2; exit 2; }
done

timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
results_root="${workspace_root}/experiments/results/sensor_validation"
if [[ -n "${requested_result_dir}" ]]; then
  result_dir="$(realpath -m -- "${requested_result_dir}")"
else
  result_dir="${results_root}/p2_${timestamp}_$$"
fi
case "${result_dir}" in
  "${results_root}"/*) ;;
  *) echo "Result directory must stay below ${results_root}." >&2; exit 2 ;;
esac
mkdir -p -- "${result_dir}/configuration" "${result_dir}/ros_logs"

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
export GZ_PARTITION="xq_p2_${USER:-wsl}_${timestamp}_$$"
export ROS_LOG_DIR="${result_dir}/ros_logs"
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe
export GALLIUM_DRIVER=llvmpipe
export EGL_PLATFORM=surfaceless
export QT_QPA_PLATFORM=offscreen

install_root="${workspace_root}/xq_install"
world="${install_root}/xq_gz_assets/share/xq_gz_assets/worlds/xq_indoor_office.sdf"
model="${install_root}/xq_gz_assets/share/xq_gz_assets/models/xq_iris_mid360/model.sdf"
bridge_config="${install_root}/xq_gz_bridge/share/xq_gz_bridge/config/p2_mid360.yaml"
launch_file="${install_root}/xq_sim_bringup/share/xq_sim_bringup/launch/xq_p2_sensors.launch.py"
validation_file="${result_dir}/sensor-validation.json"
for required in "${world}" "${model}" "${bridge_config}" "${launch_file}"; do
  [[ -f "${required}" ]] || { echo "Missing installed P2 input: ${required}" >&2; exit 2; }
done

cp -- "${world}" "${result_dir}/configuration/world.sdf"
cp -- "${model}" "${result_dir}/configuration/model.sdf"
cp -- "${bridge_config}" "${result_dir}/configuration/p2_mid360.yaml"
cp -- "${launch_file}" "${result_dir}/configuration/xq_p2_sensors.launch.py"
cp -- "${install_root}/.xq_build_manifest.json" "${result_dir}/configuration/build-manifest.json"
sha256sum "${result_dir}"/configuration/* >"${result_dir}/configuration.sha256"
cat >"${result_dir}/run.env" <<EOF
run_started_utc=${timestamp}
minimum_duration_s=${duration_s}
ros_domain_id=${ROS_DOMAIN_ID}
gz_partition=${GZ_PARTITION}
lidar_topic=/livox/lidar
imu_topic=/livox/imu
lidar_frame=livox_frame
imu_frame=livox_imu
EOF

before_audit="${result_dir}/external-assets.before.sha256"
after_audit="${result_dir}/external-assets.after.sha256"
bash "${script_dir}/audit_external_assets.sh" snapshot "${before_audit}" >/dev/null

declare -a pids=()
declare -a labels=()
audit_done=false

start_group() {
  local label="$1" log_file="$2"
  shift 2
  setsid "$@" >"${log_file}" 2>&1 < /dev/null &
  local pid=$!
  sleep 0.25
  local pgid
  pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d '[:space:]')"
  [[ "${pgid}" == "${pid}" ]] || {
    echo "Could not isolate ${label} process group." >&2
    return 4
  }
  pids+=("${pid}")
  labels+=("${label}")
  echo "${pid}" >"${result_dir}/${label}.pid"
}

stop_groups() {
  local index pid pgid round alive
  for ((index=${#pids[@]}-1; index>=0; index--)); do
    pid="${pids[index]}"
    pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d '[:space:]' || true)"
    [[ "${pgid}" == "${pid}" ]] && kill -INT -- "-${pgid}" 2>/dev/null || true
  done
  for round in 1 2 3 4 5 6 7 8 9 10; do
    alive=false
    for pid in "${pids[@]}"; do kill -0 "${pid}" 2>/dev/null && alive=true; done
    [[ "${alive}" == false ]] && break
    sleep 1
  done
  for pid in "${pids[@]}"; do
    pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d '[:space:]' || true)"
    [[ "${pgid}" == "${pid}" ]] && kill -TERM -- "-${pgid}" 2>/dev/null || true
  done
  for round in 1 2 3 4 5; do
    alive=false
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
  [[ "${audit_done}" == false ]] || return 0
  bash "${script_dir}/audit_external_assets.sh" snapshot "${after_audit}" >/dev/null
  bash "${script_dir}/audit_external_assets.sh" compare \
    "${before_audit}" "${after_audit}" >"${result_dir}/isolation-audit.txt" 2>&1
  audit_done=true
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

duration_parameter="${duration_s}.0"
start_group launch "${result_dir}/launch.log" \
  ros2 launch xq_sim_bringup xq_p2_sensors.launch.py \
  result_file:="${validation_file}" minimum_duration_s:="${duration_parameter}"

topic_deadline=$((SECONDS + 90))
while ((SECONDS < topic_deadline)); do
  if [[ -s "${validation_file}" ]] && grep -q '"count": [1-9]' "${validation_file}"; then
    break
  fi
  kill -0 "${pids[0]}" 2>/dev/null || {
    echo "P2 launch exited before sensor readiness." >&2
    tail -n 120 "${result_dir}/launch.log" >&2 || true
    exit 5
  }
  sleep 1
done
[[ -s "${validation_file}" ]] || { echo "P2 validator produced no report." >&2; exit 6; }

start_group rosbag "${result_dir}/rosbag.log" \
  ros2 bag record -o "${result_dir}/rosbag" \
  --compression-mode file --compression-format zstd \
  --compression-threads 2 --max-bag-duration 60 \
  /livox/lidar /livox/imu /tf_static /clock

validation_deadline=$((SECONDS + duration_s + 150))
status="IN_PROGRESS"
while ((SECONDS < validation_deadline)); do
  kill -0 "${pids[0]}" 2>/dev/null || {
    echo "P2 launch exited during validation." >&2
    tail -n 120 "${result_dir}/launch.log" >&2 || true
    exit 7
  }
  status="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["status"])' \
    "${validation_file}" 2>/dev/null || echo IN_PROGRESS)"
  [[ "${status}" == "FAIL" ]] && {
    cat "${validation_file}" >&2
    exit 8
  }
  [[ "${status}" == "PASS" ]] && break
  sleep 1
done
[[ "${status}" == "PASS" ]] || { echo "P2 did not reach PASS before deadline." >&2; exit 9; }

stop_groups
finish_audit
[[ -f "${result_dir}/rosbag/metadata.yaml" ]] || {
  echo "rosbag metadata is missing." >&2
  exit 10
}
python3 - "${result_dir}" <<'PY'
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

root = Path(sys.argv[1])
validation = json.loads((root / "sensor-validation.json").read_text(encoding="utf-8"))
summary = {
    "schema_version": 1,
    "gate": "P2_MID360_SENSOR_VALIDATION",
    "status": validation["status"],
    "capture_wall_duration_s": validation["capture_wall_duration_s"],
    "lidar_rate_hz": validation["lidar"]["rate_hz"],
    "imu_rate_hz": validation["imu"]["rate_hz"],
    "lidar_messages": validation["lidar"]["count"],
    "imu_messages": validation["imu"]["count"],
    "minimum_points_per_frame": validation["lidar"]["min_points_per_frame"],
    "checks": validation["checks"],
    "rosbag_present": (root / "rosbag" / "metadata.yaml").is_file(),
    "external_assets_unchanged": "PASS:" in (root / "isolation-audit.txt").read_text(),
    "generated_at_utc": datetime.now(timezone.utc).isoformat(),
}
(root / "summary.json").write_text(
    json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
)
print(json.dumps(summary, ensure_ascii=False, indent=2))
PY

echo "PASS: P2 Mid-360-like sensor Gate completed."
echo "Results: ${result_dir}"
