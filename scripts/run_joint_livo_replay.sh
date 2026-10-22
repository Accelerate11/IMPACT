#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
workspace_root="$(cd -- "${script_dir}/.." && pwd -P)"
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
unset GZ_SIM_RESOURCE_PATH IGN_GAZEBO_RESOURCE_PATH SDF_PATH
set +u
source /opt/ros/humble/setup.bash
source "${workspace_root}/xq_install/setup.bash"
source "${workspace_root}/joint_livo_install/setup.bash"
set -u
export LD_LIBRARY_PATH="${workspace_root}/livo_deps/lib:${LD_LIBRARY_PATH:-}"
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
unset ROS_DISCOVERY_SERVER CYCLONEDDS_URI FASTDDS_DEFAULT_PROFILES_FILE FASTRTPS_DEFAULT_PROFILES_FILE
export ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS2CLI_NO_DAEMON=1 OMP_NUM_THREADS=2
export ROS_DOMAIN_ID="${IMPACT_JOINT_ROS_DOMAIN:-$((120 + $$ % 70))}"
cd "${workspace_root}"
exec python3 scripts/run_joint_livo_replay.py --workspace-root "${workspace_root}" "$@"
