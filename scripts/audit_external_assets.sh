#!/usr/bin/env bash
set -euo pipefail
export LC_ALL=C

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
workspace_root="$(cd -- "${script_dir}/.." && pwd -P)"

readonly external_asset_dirs=(
  "/home/accelerate/cuadc_ws/src/uav_slam_sim/worlds"
  "/home/accelerate/cuadc_ws/src/uav_slam_sim/models"
  "/home/accelerate/ardupilot_gazebo/worlds"
  "/home/accelerate/ardupilot_gazebo/models"
)

usage() {
  echo "Usage: $0 snapshot <workspace-output-file>" >&2
  echo "       $0 compare <before-file> <after-file>" >&2
}

inside_workspace() {
  local candidate
  candidate="$(realpath -m -- "$1")"
  [[ "${candidate}" == "${workspace_root}"/runs/* ]] ||
    [[ "${candidate}" == "${workspace_root}"/experiments/results/sensor_validation/* ]] ||
    [[ "${candidate}" == "${workspace_root}"/experiments/results/localization/* ]] ||
    [[ "${candidate}" == "${workspace_root}"/experiments/results/external_nav/* ]] ||
    [[ "${candidate}" == "${workspace_root}"/experiments/results/baseline_v1/* ]] ||
    [[ "${candidate}" == "${workspace_root}"/experiments/results/impact_p6/* ]] ||
    [[ "${candidate}" == "${workspace_root}"/experiments/results/impact_p7/* ]] ||
    [[ "${candidate}" == "${workspace_root}"/experiments/results/impact_p8/* ]]
}

snapshot() {
  local output_file output_abs output_parent directory file link_target
  output_file="$1"
  if ! inside_workspace "${output_file}"; then
    echo "Snapshot output must stay below an approved XQ result directory." >&2
    exit 2
  fi
  output_abs="$(realpath -m -- "${output_file}")"
  output_parent="$(dirname -- "${output_abs}")"
  mkdir -p -- "${output_parent}"

  {
    echo "# XQ external asset audit v1"
    for directory in "${external_asset_dirs[@]}"; do
      echo "DIRECTORY ${directory}"
      if [[ ! -d "${directory}" ]]; then
        echo "MISSING ${directory}"
        continue
      fi
      while IFS= read -r -d '' file; do
        sha256sum -- "${file}"
      done < <(find "${directory}" -xdev -type f -print0 | sort -z)
      while IFS= read -r -d '' file; do
        link_target="$(readlink -- "${file}")"
        printf 'SYMLINK %s -> %s\n' "${file}" "${link_target}"
      done < <(find "${directory}" -xdev -type l -print0 | sort -z)
    done
  } > "${output_abs}"
  sha256sum -- "${output_abs}"
}

compare_snapshots() {
  local before after
  before="$(realpath -e -- "$1")"
  after="$(realpath -e -- "$2")"
  if ! inside_workspace "${before}" || ! inside_workspace "${after}"; then
    echo "Both snapshots must stay below an approved XQ result directory." >&2
    exit 2
  fi
  if cmp -s -- "${before}" "${after}"; then
    echo "PASS: pre-existing Gazebo map/model assets are byte-for-byte unchanged."
    return 0
  fi
  echo "FAIL: a pre-existing Gazebo map/model asset changed." >&2
  diff -u -- "${before}" "${after}" >&2 || true
  return 3
}

case "${1:-}" in
  snapshot)
    [[ $# -eq 2 ]] || { usage; exit 2; }
    snapshot "$2"
    ;;
  compare)
    [[ $# -eq 3 ]] || { usage; exit 2; }
    compare_snapshots "$2" "$3"
    ;;
  *)
    usage
    exit 2
    ;;
esac
