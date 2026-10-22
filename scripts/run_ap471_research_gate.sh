#!/usr/bin/env bash
# Deliberately separate from historical AP flight runners.
set -euo pipefail
ulimit -c 0
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
ap471_script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
exec python3 "$ap471_script_dir/run_ap471_research_gate.py" "$@"
