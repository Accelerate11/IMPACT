#!/usr/bin/env bash
# R0: reproducible regression of the existing GPS-off ArduPilot flight chain.
# This gate does not claim that P15 has been connected to ArduPilot.
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
workspace_root="$(cd -- "${script_dir}/.." && pwd -P)"
timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
run_dir="${workspace_root}/experiments/results/research_baseline/r0_${timestamp}_$$"
p4_dir="${workspace_root}/experiments/results/external_nav/research_r0_${timestamp}_$$"
mkdir -p "${run_dir}"
stage=build_freshness
flight_pid=""

finish() {
  local status=$?
  trap - EXIT INT TERM
  # P4 owns separate process groups. Forward termination to its runner and
  # let that runner reap its own groups before recording the failed stage.
  if [[ -n "${flight_pid}" ]] && kill -0 "${flight_pid}" 2>/dev/null; then
    kill -TERM "${flight_pid}" 2>/dev/null || true
    wait "${flight_pid}" || true
  fi
  python3 - "${run_dir}" "${p4_dir}" "${workspace_root}" "${stage}" "${status}" <<'PY'
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

run, p4, workspace = map(Path, sys.argv[1:4])
stage, code = sys.argv[4], int(sys.argv[5])
summary = {
    "schema_version": 1,
    "gate": "R0_ISOLATED_ARDUPILOT_REGRESSION",
    "status": "PASS" if code == 0 and stage == "complete" else "FAIL",
    "last_stage": stage,
    "exit_code": code,
    "workspace_root": str(workspace),
    "p4_run_dir": str(p4),
    "generated_at_utc": datetime.now(timezone.utc).isoformat(),
    "scope": "Existing P4 GPS-off ExternalNav flight regression; not P15/AP integration",
}
for name in ("python-tests.log", "bridge-test-results.log", "build-freshness.log"):
    path = run / name
    if path.is_file():
        summary[name.removesuffix(".log")] = path.read_text(errors="replace").strip()
p4_summary = p4 / "summary.json"
if p4_summary.is_file():
    summary["flight"] = json.loads(p4_summary.read_text())
(run / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n")
print(json.dumps(summary, ensure_ascii=False, indent=2))
PY
  echo "Research baseline results: ${run_dir}"
  exit "${status}"
}
trap finish EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

python3 "${script_dir}/check_build_manifest.py" --workspace-root "${workspace_root}" \
  > "${run_dir}/build-freshness.log" 2>&1

unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
unset GZ_SIM_RESOURCE_PATH IGN_GAZEBO_RESOURCE_PATH SDF_PATH
set +u
source /opt/ros/humble/setup.bash
source "${workspace_root}/xq_install/setup.bash"
set -u
export PYTHONNOUSERSITE=1
export PYTHONDONTWRITEBYTECODE=1
export ROS_LOCALHOST_ONLY=1
export ROS2CLI_NO_DAEMON=1
export PYTHONPATH="${workspace_root}/src/xq_autonomy:${workspace_root}/src/impact_fault_injection:${PYTHONPATH:-}"
cd "${workspace_root}"

for package in xq_autonomy impact_fault_injection xq_fast_lio xq_gz_bridge xq_sim_bringup ego_planner; do
  prefix="$(ros2 pkg prefix "${package}")"
  [[ "${prefix}" == "${workspace_root}/xq_install/${package}" ]] || {
    echo "Unexpected package prefix for ${package}: ${prefix}" >&2
    exit 2
  }
  printf '%s\t%s\n' "${package}" "${prefix}" >> "${run_dir}/ros-package-prefixes.tsv"
done

sha256sum \
  "${script_dir}/run_research_baseline.sh" \
  "${script_dir}/check_build_manifest.py" \
  "${script_dir}/run_p4_external_nav.sh" \
  > "${run_dir}/runner-sources.sha256"
cp "${workspace_root}/xq_install/.xq_build_manifest.json" "${run_dir}/build-manifest.json"

stage=python_regression
echo "R0: running Python regressions."
python3 -m pytest -q -p no:cacheprovider src/xq_autonomy/test src/impact_fault_injection/test \
  > "${run_dir}/python-tests.log" 2>&1

stage=bridge_regression
echo "R0: running Gazebo bridge conversion and shutdown regressions."
colcon --log-base "${workspace_root}/xq_test_log" test \
  --build-base "${workspace_root}/xq_build" \
  --install-base "${workspace_root}/xq_install" \
  --packages-select xq_gz_bridge \
  --event-handlers console_direct+ \
  > "${run_dir}/bridge-tests.log" 2>&1
colcon test-result --test-result-base "${workspace_root}/xq_build/xq_gz_bridge" --verbose \
  > "${run_dir}/bridge-test-results.log" 2>&1

stage=ardupilot_flight
echo "R0: starting the GPS-off ArduPilot takeoff, square-flight and landing regression."
bash "${script_dir}/run_p4_external_nav.sh" --run-dir "${p4_dir}" \
  > "${run_dir}/flight-runner.log" 2>&1 &
flight_pid=$!
wait "${flight_pid}"
flight_pid=""
stage=post_flight_freshness
python3 "${script_dir}/check_build_manifest.py" --workspace-root "${workspace_root}" \
  > "${run_dir}/post-flight-build-freshness.log" 2>&1
stage=complete
