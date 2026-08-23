#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
workspace_root="$(cd -- "${script_dir}/.." && pwd -P)"
default_run="$(ls -td "${workspace_root}"/experiments/results/impact_p10/visual_* | head -1)"
run_dir="$(realpath -e -- "${1:-${default_run}}")"
record_dir="${run_dir}/gz_record"
[[ -f "${record_dir}/state.tlog" ]] || { echo "P10 Gazebo recording missing." >&2; exit 2; }

set +u
source /opt/ros/humble/setup.bash
source "${workspace_root}/xq_install/setup.bash"
set -u
export GZ_PARTITION="xq_p10_replay_${USER:-wsl}_$(date -u +%Y%m%dT%H%M%SZ)_$$"
export GZ_SIM_RESOURCE_PATH="${workspace_root}/xq_install/xq_gz_assets/share/xq_gz_assets/models"
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe
export GALLIUM_DRIVER=llvmpipe

setsid gz sim -s --headless-rendering -v 3 --playback "${record_dir}" \
  >"${run_dir}/gazebo-replay-server.log" 2>&1 < /dev/null & server_pid=$!
gui_pid=""
cleanup() {
  local status=$?; trap - EXIT INT TERM; set +e
  [[ -n "${gui_pid}" ]] && kill -INT -- "-${gui_pid}" 2>/dev/null || true
  kill -INT -- "-${server_pid}" 2>/dev/null || true
  sleep 1
  [[ -n "${gui_pid}" ]] && kill -TERM -- "-${gui_pid}" 2>/dev/null || true
  kill -TERM -- "-${server_pid}" 2>/dev/null || true
  exit "${status}"
}
trap cleanup EXIT INT TERM

deadline=$((SECONDS + 30))
world_name=""
until [[ -n "${world_name}" ]]; do
  services="$(gz service -l 2>/dev/null)"
  if grep -Fq '/world/xq_p10_long_corridor/control' <<<"${services}"; then
    world_name=xq_p10_long_corridor
  elif grep -Fq '/world/default/control' <<<"${services}"; then
    # Gazebo playback reconstructs the recorded entities in a default wrapper world.
    world_name=default
  fi
  kill -0 "${server_pid}" 2>/dev/null || { echo "P10 Gazebo replay server exited." >&2; exit 3; }
  ((SECONDS < deadline)) || { echo "P10 replay world did not become ready." >&2; exit 4; }
  [[ -n "${world_name}" ]] || sleep 0.25
done
setsid env -u QT_QPA_PLATFORM -u EGL_PLATFORM nice -n 10 gz sim -g -v 3 \
  >"${run_dir}/gazebo-replay-gui.log" 2>&1 & gui_pid=$!
sleep 4
kill -0 "${gui_pid}" 2>/dev/null || { echo "P10 Gazebo GUI exited during startup." >&2; exit 5; }
camera_response="$(gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera \
  --reptype gz.msgs.Boolean --timeout 5000 \
  --req 'pose: {position: {x: -8.0, y: 0.0, z: 17.0}, orientation: {x: 0.0, y: 0.70710678, z: 0.0, w: 0.70710678}} projection_type: "perspective"')"
grep -Fq 'data: true' <<<"${camera_response}" || { echo "P10 Gazebo camera setup failed." >&2; exit 6; }
gz service -s "/world/${world_name}/control" --reqtype gz.msgs.WorldControl \
  --reptype gz.msgs.Boolean --timeout 5000 --req 'pause: false' >/dev/null
echo "Gazebo P10 replay: open top, walls preserved, Minimum-Excitation flight."
wait "${gui_pid}" || true
