#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
workspace_root="$(cd -- "${script_dir}/.." && pwd -P)"
timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
default_result_dir="${workspace_root}/experiments/results/impact_p9/p9_${timestamp}_$$"
result_dir="$(realpath -m -- "${1:-${default_result_dir}}")"
case "${result_dir}" in
  "${workspace_root}"/experiments/results/impact_p9/*) ;;
  *) echo "P9 result directory must stay under this workspace's impact_p9 results." >&2; exit 2 ;;
esac
result_file="${result_dir}/p9-gate-result.json"
calibration="${workspace_root}/evidence/P7/p7-calibration.json"
world="${workspace_root}/xq_install/xq_gz_assets/share/xq_gz_assets/worlds/xq_p9_integrity_gate.sdf"

[[ -f "${calibration}" ]] || { echo "P7 calibration missing: ${calibration}" >&2; exit 2; }
[[ -f "${world}" ]] || { echo "P9 Gazebo world missing: ${world}" >&2; exit 2; }
mkdir -p -- "${result_dir}/ros_logs"

unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
unset GZ_SIM_RESOURCE_PATH IGN_GAZEBO_RESOURCE_PATH SDF_PATH
set +u
source /opt/ros/humble/setup.bash
source "${workspace_root}/xq_install/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-109}"
export ROS_LOCALHOST_ONLY=1
export ROS2CLI_NO_DAEMON=1
export ROS_LOG_DIR="${result_dir}/ros_logs"
export GZ_PARTITION="xq_p9_${USER:-wsl}_${timestamp}_$$"
export GZ_SIM_RESOURCE_PATH="${workspace_root}/xq_install/xq_gz_assets/share/xq_gz_assets/models"
export IGN_GAZEBO_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH}"
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe
export GALLIUM_DRIVER=llvmpipe
export EGL_PLATFORM=surfaceless
export QT_QPA_PLATFORM=offscreen

declare -a managed_pids=()
cleanup() {
  local status=$? pid pgid
  trap - EXIT INT TERM
  set +e
  for pid in "${managed_pids[@]}"; do
    pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d '[:space:]')"
    [[ -n "${pgid}" ]] && kill -INT -- "-${pgid}" 2>/dev/null
  done
  sleep 1
  for pid in "${managed_pids[@]}"; do
    pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d '[:space:]')"
    [[ -n "${pgid}" ]] && kill -TERM -- "-${pgid}" 2>/dev/null
  done
  for pid in "${managed_pids[@]}"; do wait "${pid}" 2>/dev/null; done
  exit "${status}"
}
trap cleanup EXIT INT TERM

start_group() {
  local log_file="$1"
  shift
  setsid "$@" >"${log_file}" 2>&1 < /dev/null &
  managed_pids+=("$!")
}

bash "${script_dir}/audit_external_assets.sh" snapshot "${result_dir}/external-assets.before.sha256" >/dev/null
calibration_sha="$(sha256sum "${calibration}" | cut -d ' ' -f 1)"
sha256sum "${calibration}" >"${result_dir}/p7-calibration.sha256"
sha256sum "${world}" >"${result_dir}/p9-world.sha256"
gz sdf -k "${world}" >"${result_dir}/sdf-validation.txt"
{
  echo "run_started_utc=${timestamp}"
  echo "ros_domain_id=${ROS_DOMAIN_ID}"
  echo "gz_partition=${GZ_PARTITION}"
  echo "world=${world}"
  echo "p7_calibration=${calibration}"
  echo "p7_calibration_sha256=${calibration_sha}"
  echo "margin_reserve_m=0.10"
  echo "hard_constraint=true"
  echo "weighted_cost=false"
  echo "ground_truth_subscribed=false"
} >"${result_dir}/run.env"

start_group "${result_dir}/gazebo.log" gz sim -r -s --headless-rendering -v 3 \
  --record-path "${result_dir}/gz_record" --record-period 0.05 "${world}"
deadline=$((SECONDS + 30))
until gz service -l 2>/dev/null | grep -Fq '/world/xq_p9_integrity_gate/control'; do
  kill -0 "${managed_pids[0]}" 2>/dev/null || { echo "Gazebo exited early." >&2; exit 3; }
  ((SECONDS < deadline)) || { echo "Gazebo world did not become ready." >&2; exit 4; }
  sleep 0.25
done

start_group "${result_dir}/rosbag.log" ros2 bag record \
  --compression-mode file --compression-format zstd -o "${result_dir}/rosbag" \
  /planning/candidate_bspline /planning/bspline /planning/trajectory_certified \
  /xq/p5/cloud_map /xq/p9/scenario /integrity/directional /integrity/alert_limit \
  /integrity/alert_limit_profile \
  /integrity/alert_limit_debug /integrity/margin /integrity/margin_debug \
  /planning/certification_debug

start_group "${result_dir}/p9-stack.log" ros2 launch xq_sim_bringup \
  xq_p9_integrity_margin_gate.launch.py result_file:="${result_file}" \
  calibration_file:="${calibration}" calibration_sha256:="${calibration_sha}"

timeout 20 bash -c 'until ros2 node list 2>/dev/null | grep -qx /xq_p9_integrity_margin; do sleep 0.2; done'
ros2 node info /xq_p9_integrity_margin >"${result_dir}/margin-node-graph.txt"
ros2 node info /xq_p9_trajectory_gate >"${result_dir}/gate-node-graph.txt"
if grep -Fq '/xq/eval/' "${result_dir}/margin-node-graph.txt" "${result_dir}/gate-node-graph.txt"; then
  echo "Ground Truth leaked into the P9 algorithm graph." >&2
  exit 8
fi

deadline=$((SECONDS + 25))
status=""
while ((SECONDS < deadline)); do
  if [[ -s "${result_file}" ]]; then
    status="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["status"])' "${result_file}")"
    [[ "${status}" == PASS ]] && break
  fi
  sleep 0.25
done
[[ "${status}" == PASS ]] || { cat "${result_file}" 2>/dev/null >&2 || true; exit 9; }
sleep 1

# Stop recorders before checking their artifacts.  cleanup remains responsible
# for all remaining process groups and is safe if these groups already exited.
for index in 1 2 0; do
  pid="${managed_pids[${index}]}"
  pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d '[:space:]')"
  [[ -n "${pgid}" ]] && kill -INT -- "-${pgid}" 2>/dev/null || true
  wait "${pid}" 2>/dev/null || true
done

[[ -f "${result_dir}/rosbag/metadata.yaml" ]] || { echo "P9 rosbag metadata missing." >&2; exit 10; }
[[ -f "${result_dir}/gz_record/state.tlog" ]] || { echo "P9 Gazebo recording missing." >&2; exit 11; }
bash "${script_dir}/audit_external_assets.sh" snapshot "${result_dir}/external-assets.after.sha256" >/dev/null
bash "${script_dir}/audit_external_assets.sh" compare \
  "${result_dir}/external-assets.before.sha256" "${result_dir}/external-assets.after.sha256" \
  | tee "${result_dir}/isolation-audit.txt"

python3 - "${result_dir}" "${calibration_sha}" <<'PY'
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

root = Path(sys.argv[1])
gate = json.loads((root / "p9-gate-result.json").read_text(encoding="utf-8"))
summary = {
    "schema_version": 1,
    "gate": "P9_INTEGRITY_MARGIN",
    "status": "PASS",
    "integrity_margin": gate,
    "p7_calibration_sha256": sys.argv[2],
    "rosbag_present": (root / "rosbag" / "metadata.yaml").is_file(),
    "gazebo_recording_present": (root / "gz_record" / "state.tlog").is_file(),
    "open_top_world": True,
    "walls_preserved": True,
    "ground_truth_isolated": True,
    "external_assets_unchanged": "PASS:" in (root / "isolation-audit.txt").read_text(encoding="utf-8"),
    "hard_constraint": True,
    "weighted_cost": False,
    "generated_at_utc": datetime.now(timezone.utc).isoformat(),
}
(root / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
print(json.dumps(summary, indent=2))
PY

echo "PASS: P9 Integrity Margin hard-constraint Gate completed."
echo "Results: ${result_dir}"
