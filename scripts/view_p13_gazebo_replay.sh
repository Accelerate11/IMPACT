#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
workspace_root="$(cd -- "${script_dir}/.." && pwd -P)"
default_run="$(for candidate in $(ls -td "${workspace_root}"/experiments/results/impact_p13/gate_* 2>/dev/null); do
  [[ -f "${candidate}/p13-gate-result.json" ]] || continue
  [[ "$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["status"])' "${candidate}/p13-gate-result.json")" == PASS ]] && { echo "${candidate}"; break; }
done)"
run_dir="$(realpath -e -- "${1:-${default_run}}")"
echo "P13 Gazebo replay uses the accepted high_200ms recording."
bash "${script_dir}/view_p12_gazebo_replay.sh" "${run_dir}/high_200ms"
