#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
workspace_root="$(cd -- "${script_dir}/.." && pwd -P)"
timestamp="$(date -u +%Y%m%dT%H%M%SZ)"

export XQ_CANDIDATE_GENERATION_MODE="lattice"
export XQ_CANDIDATE_METRIC_SOURCE="online_map"
export XQ_LATTICE_LATERAL_LEVELS="${XQ_LATTICE_LATERAL_LEVELS:-5}"
export XQ_LATTICE_VERTICAL_LEVELS="${XQ_LATTICE_VERTICAL_LEVELS:-2}"
export XQ_TASK_PROGRESS_WEIGHT="${XQ_TASK_PROGRESS_WEIGHT:-0.85}"
export XQ_TASK_MAP_AGE_TIME_CONSTANT_S="${XQ_TASK_MAP_AGE_TIME_CONSTANT_S:-20.0}"
# A 24 m outbound task plus a conservative straight-line return reserve cannot
# be evaluated against the legacy 32-unit one-way proxy.  The research budget
# retains the hard energy check while making the physical return term real.
export XQ_RESEARCH_ENERGY_REMAINING="${XQ_RESEARCH_ENERGY_REMAINING:-70.0}"
export XQ_UTILITY_INDIFFERENCE_BAND="${XQ_UTILITY_INDIFFERENCE_BAND:-0.05}"
export XQ_DYNAMIC_PATH_QUERY_MODE="${XQ_DYNAMIC_PATH_QUERY_MODE:-active_trajectory}"
export XQ_MINIMUM_DYNAMIC_CLUSTER_POINTS="${XQ_MINIMUM_DYNAMIC_CLUSTER_POINTS:-5}"
export XQ_DYNAMIC_CLUSTER_RADIUS_M="${XQ_DYNAMIC_CLUSTER_RADIUS_M:-0.45}"
export XQ_TERMINAL_EXTENSION_MODE="${XQ_TERMINAL_EXTENSION_MODE:-progress_watchdog}"
export XQ_RUNTIME_INTEGRITY_GUARD_MODE="${XQ_RUNTIME_INTEGRITY_GUARD_MODE:-current_margin_replan}"
export XQ_RUNTIME_INTEGRITY_MARGIN_M="${XQ_RUNTIME_INTEGRITY_MARGIN_M:-0.12}"
export XQ_COMPLEX_THRESHOLDS="${XQ_COMPLEX_THRESHOLDS:-${workspace_root}/config/p15_research_thresholds.json}"
export XQ_COMPLEX_ANALYZER="${XQ_COMPLEX_ANALYZER:-${workspace_root}/scripts/analyze_p15_research_comparison.py}"

result="${1:-${workspace_root}/experiments/results/impact_complex_comparison/p15_research_${timestamp}_$$}"
exec bash "${script_dir}/run_complex_algorithm_comparison.sh" "${result}"
