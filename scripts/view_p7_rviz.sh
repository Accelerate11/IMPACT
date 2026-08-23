#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
XQ_P7_REPLAY=1 exec bash "${script_dir}/view_p5_rviz.sh" "$@"
