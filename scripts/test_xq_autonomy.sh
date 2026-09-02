#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="${XQ_WORKSPACE_ROOT:-$(cd -- "${SCRIPT_DIR}/.." && pwd)}"

source /opt/ros/humble/setup.bash
if [[ -f "${REPO_ROOT}/xq_install/setup.bash" ]]; then
  source "${REPO_ROOT}/xq_install/setup.bash"
fi
set -u

export PYTHONPATH="${REPO_ROOT}/src/xq_autonomy${PYTHONPATH:+:${PYTHONPATH}}"
cd "${REPO_ROOT}"
python3 -m pytest -q src/xq_autonomy/test src/impact_fault_injection/test
