#!/usr/bin/env bash
set -euo pipefail
ulimit -c 0
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
set +u
source /opt/ros/humble/setup.bash
source /home/accelerate/impact_research_ws/xq_install/setup.bash
set -u
ap471_script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
exec python3 "$ap471_script_dir/run_ap471_extnav_probe.py" "$@"
