#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
export XQ_COMPLEX_WORLD="xq_complex_3d_warehouse.sdf"
export XQ_COMPLEX_LATERAL_OFFSET="0.70"
export XQ_COMPLEX_ENABLE_VERTICAL="true"
export XQ_COMPLEX_VERTICAL_OFFSET="0.70"
exec bash "${script_dir}/run_complex_live_visualization.sh"
