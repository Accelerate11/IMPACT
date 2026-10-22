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
export CMAKE_BUILD_PARALLEL_LEVEL=1
export MAKEFLAGS=-j1
export OMP_NUM_THREADS=2
cd "${workspace_root}"
python3 scripts/check_livo_build_manifest.py --workspace-root "${workspace_root}" --mode prepare
colcon --log-base "${workspace_root}/livo_log" build \
  --base-paths "${workspace_root}/src/xq_fast_livo2" \
  --build-base "${workspace_root}/livo_build" \
  --install-base "${workspace_root}/livo_install" \
  --packages-select xq_fast_livo2 --executor sequential \
  --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release
python3 scripts/check_livo_build_manifest.py --workspace-root "${workspace_root}" --mode write
