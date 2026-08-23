#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
workspace_root="$(cd -- "${script_dir}/.." && pwd -P)"
timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
run_dir="${workspace_root}/experiments/results/impact_p8/p8_gazebo_${timestamp}_$$"

echo "P8 live Gate is starting headless; this can take up to about 7.5 minutes."
echo "Gazebo and RViz will open together only after the Gate reports PASS."
echo "For immediate playback of the frozen PASS result, use:"
echo "  bash scripts/view_p8_combined.sh"

bash "${script_dir}/run_p5_baseline.sh" \
  --phase8 --gazebo-record --run-dir "${run_dir}" "$@"
echo "P8 Gate PASS. Opening Gazebo and RViz replay windows."
exec bash "${script_dir}/view_p8_combined.sh" "${run_dir}"
