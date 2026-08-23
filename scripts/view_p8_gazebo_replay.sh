#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
workspace_root="$(cd -- "${script_dir}/.." && pwd -P)"
run_dir="$(realpath -e -- "${1:?Usage: $0 P8_RUN_DIR}")"
record_dir="${run_dir}/gz_record"
shell_model="${workspace_root}/xq_install/xq_gz_assets/share/xq_gz_assets/models/xq_p5_replay_shell/model.sdf"
[[ -d "${record_dir}" ]] || { echo "Gazebo recording missing: ${record_dir}" >&2; exit 2; }
[[ -f "${shell_model}" ]] || { echo "Replay shell missing: ${shell_model}" >&2; exit 2; }

unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
unset GZ_SIM_RESOURCE_PATH IGN_GAZEBO_RESOURCE_PATH SDF_PATH
set +u
source /opt/ros/humble/setup.bash
source "${workspace_root}/xq_install/setup.bash"
set -u

export GZ_PARTITION="xq_p8_replay_${USER:-wsl}_$(date -u +%Y%m%dT%H%M%SZ)_$$"
export GZ_SIM_RESOURCE_PATH="${workspace_root}/xq_install/xq_gz_assets/share/xq_gz_assets/models:/home/accelerate/ardupilot_gazebo/models:/home/accelerate/ardupilot_gazebo/worlds"
export IGN_GAZEBO_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH}"
export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/accelerate/ardupilot_gazebo/build
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe
export GALLIUM_DRIVER=llvmpipe
export EGL_PLATFORM=surfaceless
export QT_QPA_PLATFORM=offscreen

server_log="${run_dir}/gazebo-replay-server.log"
gui_log="${run_dir}/gazebo-replay-gui.log"
setsid gz sim -s --headless-rendering -v 3 --playback "${record_dir}" >"${server_log}" 2>&1 < /dev/null &
server_pid=$!
gui_pid=""
cleanup() {
  local status=$?; trap - EXIT INT TERM; set +e
  if [[ -n "${gui_pid}" ]]; then
    kill -INT -- -"${gui_pid}" 2>/dev/null || true
    sleep 1
    kill -TERM -- -"${gui_pid}" 2>/dev/null || true
  fi
  kill -INT -- -"${server_pid}" 2>/dev/null || true
  sleep 2
  kill -TERM -- -"${server_pid}" 2>/dev/null || true
  wait "${server_pid}" 2>/dev/null || true
  exit "${status}"
}
trap cleanup EXIT INT TERM

deadline=$((SECONDS + 30))
world_name=""
until [[ -n "${world_name}" ]]; do
  services="$(gz service -l)"
  if grep -Fq '/world/xq_p5_structured_room/control' <<<"${services}"; then
    world_name=xq_p5_structured_room
  elif grep -Fq '/world/default/control' <<<"${services}"; then
    world_name=default
  fi
  kill -0 "${server_pid}" 2>/dev/null || { echo "Gazebo replay server exited; see ${server_log}" >&2; exit 3; }
  ((SECONDS < deadline)) || { echo "Gazebo replay world did not become ready." >&2; exit 4; }
  [[ -n "${world_name}" ]] || sleep 0.5
done

scene_response="$(
  gz service -s "/world/${world_name}/scene/info" \
    --reqtype gz.msgs.Empty --reptype gz.msgs.Scene \
    --timeout 5000 --req ''
)"
ceiling_node=xq_office_shell::xq_ceiling_link
if ! grep -Fq 'name: "xq_shell_link"' <<<"${scene_response}"; then
  spawn_response="$(
    gz service -s "/world/${world_name}/create" \
      --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean \
      --timeout 5000 --req "sdf_filename: '${shell_model}'"
  )"
  grep -Fq 'data: true' <<<"${spawn_response}" || {
    echo "Gazebo replay shell could not be spawned." >&2
    exit 5
  }
  ceiling_node=xq_p5_replay_shell::xq_ceiling_link
fi

setsid env -u QT_QPA_PLATFORM -u EGL_PLATFORM nice -n 10 gz sim -g -v 3 >"${gui_log}" 2>&1 &
gui_pid=$!
sleep 4
kill -0 "${gui_pid}" 2>/dev/null || { echo "Gazebo replay GUI exited; see ${gui_log}" >&2; exit 5; }
bash "${script_dir}/configure_gazebo_view.sh" \
  "${run_dir}/gazebo-replay-view.txt" "${world_name}" \
  "${ceiling_node}"

gz service -s "/world/${world_name}/control" \
  --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean \
  --timeout 5000 --req 'pause: false' >/dev/null

echo "Gazebo replay is running. Close the Gazebo window to finish."
wait "${gui_pid}" || true
