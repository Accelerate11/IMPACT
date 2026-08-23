#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
workspace_root="$(cd -- "${script_dir}/.." && pwd -P)"
timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
run_dir="${workspace_root}/experiments/results/impact_p9/p9_gazebo_${timestamp}_$$"
echo "Running the P9 hard Gate headless, then opening Gazebo and RViz together."
bash "${script_dir}/run_p9_integrity_margin.sh" "${run_dir}"
exec bash "${script_dir}/view_p9_combined.sh" "${run_dir}"
