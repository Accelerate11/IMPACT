#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
workspace_root="$(cd -- "${script_dir}/.." && pwd -P)"
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
set +u
source /opt/ros/humble/setup.bash
source "${workspace_root}/xq_install/setup.bash"
set -u
export CMAKE_PREFIX_PATH="${workspace_root}/livo_deps:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="${workspace_root}/livo_deps/lib:${LD_LIBRARY_PATH:-}"
export CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 OMP_NUM_THREADS=2
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
attempt_dir="${workspace_root}/experiments/results/research_multisensor/joint_livo_build_$(date -u +%Y%m%dT%H%M%SZ)_$$"
mkdir -p -- "$(dirname -- "${attempt_dir}")"
mkdir -- "${attempt_dir}"
printf '%s\n' "${attempt_dir}"
cd -- "${workspace_root}"
python3 scripts/check_joint_livo_build_manifest.py --workspace-root "${workspace_root}" \
  --mode prepare --record-dir "${attempt_dir}" > "${attempt_dir}/prepare.log" 2>&1
set +e
colcon --log-base "${workspace_root}/joint_livo_log" build \
  --base-paths "${workspace_root}/src/xq_fast_livo2" \
  --build-base "${workspace_root}/joint_livo_build" \
  --install-base "${workspace_root}/joint_livo_install" \
  --packages-select xq_fast_livo2 --executor sequential \
  --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release \
  > "${attempt_dir}/build.log" 2>&1
build_status=$?
set -e
printf '%s\n' "${build_status}" > "${attempt_dir}/compiler-exit-code.txt"
python3 scripts/check_joint_livo_build_manifest.py --workspace-root "${workspace_root}" \
  --mode audit --record-dir "${attempt_dir}" > "${attempt_dir}/after-build-audit.log" 2>&1
if [[ "${build_status}" -ne 0 ]]; then
  printf '%s\n' 'FAIL: compiler/colcon failed; original attempt retained' >&2
  exit "${build_status}"
fi
python3 scripts/check_joint_livo_build_manifest.py --workspace-root "${workspace_root}" \
  --mode write --record-dir "${attempt_dir}" > "${attempt_dir}/manifest.log" 2>&1
python3 scripts/check_joint_livo_build_manifest.py --workspace-root "${workspace_root}" \
  --mode check > "${attempt_dir}/freshness.log" 2>&1
printf '%s\n' 'PASS' > "${attempt_dir}/status.txt"
printf '%s\n' 'PASS: joint_livo_install is bound to sources/dependencies/runtime; old LIVO prefixes unchanged'
