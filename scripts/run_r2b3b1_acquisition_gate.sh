#!/usr/bin/env bash
# R2B3B1: new range/noise acquisition alongside the existing short AP SIL gate.
# Derived from the retained R1/R2 runner; old inputs and results are never reused as new data.
set -euo pipefail
ulimit -c 0
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
workspace_root="$(cd -- "${script_dir}/.." && pwd -P)"
trial=normal
renderer=gpu
multisensor=true
while (($#)); do
  case "$1" in

    --renderer) renderer="$2"; shift 2 ;;

    *) echo "Usage: $0 [--renderer gpu|software]" >&2; exit 2 ;;
  esac
done
[[ "${renderer}" == gpu || "${renderer}" == software ]] || exit 2
[[ "${multisensor}" != true || "${trial}" == normal ]] || exit 2
timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
run_dir="${workspace_root}/experiments/results/research_ap/r1_${trial}_${timestamp}_$$"
if [[ "${multisensor}" == true ]]; then
  run_dir="${workspace_root}/experiments/results/research_multisensor/r2b3b1_${timestamp}_$$"
fi
display_label="R1 ${trial}"
if [[ "${multisensor}" == true ]]; then display_label='R2B3B1 range/noise'; fi
mkdir -p "${run_dir}/configuration" "${run_dir}/ros_logs" "${run_dir}/sitl_runtime"
stage=preflight
paper_baseline=/mnt/d/CODEX/impact/evidence/PAPER_PRESERVATION/paper_baseline_20261020T092036Z
protected_helper=/mnt/d/CODEX/impact/evidence/RESEARCH_R2/r2_suite_20261018T165613Z_320/archive_research_r2.py
protected_baseline=/mnt/d/CODEX/impact/runs/research_bootstrap_20261018/protected.before.json
audit_preserved() {
  python3 "$paper_baseline/verify_paper_baseline.py" --baseline "$paper_baseline" \
    --allow-new-prefix evidence/RESEARCH_R2B3B0 --allow-new-prefix evidence/RESEARCH_R2B3B1 \
    > "$run_dir/paper-baseline.$1.json" || return $?
  python3 /mnt/d/CODEX/impact/runs/verify_r2b3b1_preserved.py > "$run_dir/extra-preservation.$1.json" || return $?
  python3 - "$protected_helper" "$protected_baseline" "$run_dir" "$1" <<'PY'
import json,runpy,sys
from pathlib import Path
api=runpy.run_path(sys.argv[1])
current,result=api['compare_protected'](api['read_object'](Path(sys.argv[2])))
(Path(sys.argv[3])/('protected-audit.'+sys.argv[4]+'.json')).write_text(json.dumps(result,indent=2)+'\n')
if result['status']!='PASS':raise SystemExit('Protected files changed')
PY
}
audit_preserved before

owned_domain=false
declare -a pids=()
declare -a labels=()
stop_groups() {
  local pid round alive index code
  local -A term_forced=() kill_forced=()
  # Launchers forward SIGINT to their children. Signal each owned leader first
  # to avoid delivering the same interrupt twice during ROS shutdown.
  for pid in "${pids[@]}"; do kill -INT "${pid}" 2>/dev/null || true; done
  for round in {1..10}; do
    alive=false
    for pid in "${pids[@]}"; do pgrep -g "${pid}" >/dev/null && alive=true; done
    [[ "${alive}" == false ]] && break
    sleep 1
  done
  for pid in "${pids[@]}"; do
    if pgrep -g "${pid}" >/dev/null; then term_forced[${pid}]=true; kill -TERM -- "-${pid}" 2>/dev/null || true; fi
  done
  for round in {1..5}; do
    alive=false
    for pid in "${pids[@]}"; do pgrep -g "${pid}" >/dev/null && alive=true; done
    [[ "${alive}" == false ]] && break
    sleep 1
  done
  for pid in "${pids[@]}"; do
    if pgrep -g "${pid}" >/dev/null; then kill_forced[${pid}]=true; kill -KILL -- "-${pid}" 2>/dev/null || true; fi
  done
  for index in "${!pids[@]}"; do
    pid="${pids[$index]}"
    if wait "${pid}" 2>/dev/null; then code=0; else code=$?; fi
    printf '{"label":"%s","pid":%d,"exit_code":%d,"term_forced":%s,"kill_forced":%s}\n' \
      "${labels[$index]}" "${pid}" "${code}" "${term_forced[$pid]:-false}" "${kill_forced[$pid]:-false}" \
      >> "${run_dir}/process-shutdown.jsonl"
  done
  pids=()
  labels=()
}
cleanup() {
  local status=$?
  trap - EXIT INT TERM
  set +e
  stop_groups
  python3 - "${run_dir}" <<'PY'
import json, sys
from pathlib import Path
run=Path(sys.argv[1]); path=run/'process-shutdown.jsonl'
events=[json.loads(line) for line in path.read_text().splitlines()] if path.exists() else []
issues=[item for item in events if item['exit_code'] not in (0,130) or item['term_forced'] or item['kill_forced']]
(run/'shutdown-result.json').write_text(json.dumps({
    'status':'ISSUES' if issues else 'PASS' if events else 'NOT_STARTED',
    'scope':'Post-mission environment cleanup, separate from flight-interface acceptance',
    'processes':events,'issues':issues},indent=2)+'\n')
PY
  if [[ -f "${run_dir}/external-assets.before.sha256" ]]; then
    bash "${script_dir}/audit_external_assets.sh" snapshot "${run_dir}/external-assets.after.sha256" >/dev/null
    bash "${script_dir}/audit_external_assets.sh" compare \
      "${run_dir}/external-assets.before.sha256" "${run_dir}/external-assets.after.sha256" \
      > "${run_dir}/isolation-audit.txt" 2>&1
    [[ $? == 0 ]] || status=12
  fi
  audit_preserved after > "$run_dir/preservation-cleanup.log" 2>&1
  [[ $? == 0 ]] || status=14
  # This run chooses its own DDS domain and starts with no daemon in it.
  if [[ "${owned_domain}" == true ]]; then timeout 5 ros2 daemon stop > "${run_dir}/daemon-stop.log" 2>&1; fi
  python3 - "${run_dir}" "${stage}" "${status}" <<'PY'
import json, sys
from datetime import datetime, timezone
from pathlib import Path
run, stage, code = Path(sys.argv[1]), sys.argv[2], int(sys.argv[3])
result = {'status': 'PASS' if code == 0 and stage == 'complete' else 'FAIL',
          'last_stage': stage, 'exit_code': code,
          'generated_at_utc': datetime.now(timezone.utc).isoformat()}
(run / 'runner-result.json').write_text(json.dumps(result, indent=2) + '\n')
print(json.dumps({**result, 'run_dir': str(run)}))
PY
  exit "${status}"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
start_group() {
  local label="$1"
  shift
  # A noninteractive shell backgrounds jobs with SIGINT ignored. Reset it
  # before exec so owned SITL/Gazebo processes can honor graceful cleanup.
  setsid env --default-signal=INT,TERM "$@" > "${run_dir}/${label}.log" 2>&1 < /dev/null &
  local pid=$!
  pids+=("${pid}")
  labels+=("${label}")
  echo "${pid}" > "${run_dir}/${label}.pid"
  sleep 0.2
  [[ "$(ps -o pgid= -p "${pid}" | tr -d '[:space:]')" == "${pid}" ]]
}
wait_log() {
  local file="$1" pattern="$2" seconds="$3" deadline=$((SECONDS + $3))
  while ((SECONDS < deadline)); do
    grep -Fq -- "${pattern}" "${file}" 2>/dev/null && return 0
    sleep 0.5
  done
  echo "Timeout waiting for ${pattern}; see ${file}" >&2
  return 1
}
python3 "${script_dir}/check_build_manifest.py" --workspace-root "${workspace_root}" > "${run_dir}/build-freshness.log"
if ss -H -ltnp | grep -Eq '(:|\])5760[[:space:]]'; then echo 'TCP 5760 occupied' >&2; exit 3; fi
if ss -H -lunp | grep -Eq '(:|\])9002[[:space:]]'; then echo 'UDP 9002 occupied' >&2; exit 3; fi
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
unset GZ_SIM_RESOURCE_PATH IGN_GAZEBO_RESOURCE_PATH SDF_PATH
unset LIBGL_ALWAYS_SOFTWARE MESA_LOADER_DRIVER_OVERRIDE GALLIUM_DRIVER EGL_PLATFORM QT_QPA_PLATFORM
set +u
source /opt/ros/humble/setup.bash
source "${workspace_root}/xq_install/setup.bash"
set -u
unset ROS_DISCOVERY_SERVER CYCLONEDDS_URI FASTDDS_DEFAULT_PROFILES_FILE FASTRTPS_DEFAULT_PROFILES_FILE
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 ROS_LOCALHOST_ONLY=0 ROS2CLI_NO_DAEMON=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp OMP_NUM_THREADS=2
export ROS_DOMAIN_ID=$((140 + ($$ % 45)))
owned_domain=true
python3 "$script_dir/prepare_livo_transport.py" --initial-peer-range 64 --output-dir "$run_dir/configuration" > "$run_dir/transport-setup.log"
export FASTRTPS_DEFAULT_PROFILES_FILE="$run_dir/configuration/fastdds-replay.xml"
export GZ_PARTITION="impact_r2b3b1_${timestamp}_$$"
export ROS_LOG_DIR="${run_dir}/ros_logs"
export GZ_SIM_RESOURCE_PATH="${workspace_root}/xq_install/xq_gz_assets/share/xq_gz_assets/models:/home/accelerate/ardupilot_gazebo/models:/home/accelerate/ardupilot_gazebo/worlds"
export IGN_GAZEBO_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH}" SDF_PATH="${GZ_SIM_RESOURCE_PATH}"
export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/accelerate/ardupilot_gazebo/build
world_name=xq_p4_external_nav
if [[ "${multisensor}" == true ]]; then world_name=xq_r2b3_range; fi
world="${workspace_root}/xq_install/xq_gz_assets/share/xq_gz_assets/worlds/${world_name}.sdf"
params="${workspace_root}/xq_install/xq_autonomy/share/xq_autonomy/config/xq_p4_extnav.parm"
config="${run_dir}/configuration/research_ap_gate.json"
cp "${workspace_root}/config/research_ap_gate.json" "${config}"
cp "${workspace_root}/evidence/P7/p7-calibration.json" "${run_dir}/configuration/p7-calibration.json"
cp "${workspace_root}/xq_install/.xq_build_manifest.json" "${run_dir}/configuration/build-manifest.json"
for file in src/xq_autonomy/xq_autonomy/ap_execution.py src/xq_autonomy/xq_autonomy/ap_execution_node.py \
  src/xq_autonomy/xq_autonomy/ap_research_mission_node.py src/xq_autonomy/xq_autonomy/ap_research_evaluator_node.py \
  src/xq_autonomy/xq_autonomy/p11_flight_controller_node.py src/xq_sim_bringup/launch/xq_ap_research.launch.py \
  scripts/run_ap_research_gate.sh scripts/wait_ap_research_ready.py scripts/analyze_ap_research_gate.py \
  scripts/audit_external_assets.sh; do
  cp "${workspace_root}/${file}" "${run_dir}/configuration/$(basename "${file}")"
done
sha256sum "${run_dir}"/configuration/* > "${run_dir}/configuration.sha256"
if [[ "${multisensor}" == true ]]; then
  for file in config/research_multisensor_contract.json config/research_multisensor_gate.json \
    scripts/analyze_multisensor_contract.py scripts/run_multisensor_fault_matrix.py \
    src/xq_autonomy/xq_autonomy/multisensor_contract.py src/xq_autonomy/xq_autonomy/downward_flow.py \
    src/xq_autonomy/xq_autonomy/downward_flow_node.py src/xq_autonomy/xq_autonomy/multisensor_monitor_node.py \
    src/xq_autonomy/xq_autonomy/p4_external_nav_node.py src/xq_sim_bringup/launch/xq_r2_contract.launch.py \
    src/xq_gz_bridge/src/r2_camera_bridge_node.cpp src/xq_gz_bridge/src/camera_conversions.cpp \
    src/xq_gz_assets/models/xq_iris_multisensor_ardupilot/model.sdf; do
    cp "${workspace_root}/${file}" "${run_dir}/configuration/$(basename "${file}")"
  done
  cp "${world}" "${run_dir}/configuration/${world_name}.sdf"
  sha256sum "${run_dir}"/configuration/* > "${run_dir}/configuration.sha256"
fi
cp "$workspace_root/config/research_range_acquisition_gate.json" "$run_dir/configuration/"
for file in scripts/run_r2b3b1_acquisition_gate.sh scripts/audit_range_acquisition.py scripts/prepare_livo_transport.py \
  scripts/test/test_range_acquisition_audit.py config/research_range_acquisition_gate.json \
  src/xq_gz_bridge/src/r2_camera_bridge_node.cpp src/xq_gz_bridge/src/r2_range_bridge_node.cpp \
  src/xq_gz_bridge/src/range_conversions.cpp src/xq_gz_bridge/include/xq_gz_bridge/range_conversions.hpp \
  src/xq_gz_bridge/test/test_range_conversions.cpp src/xq_gz_bridge/CMakeLists.txt \
  src/xq_gz_assets/models/xq_iris_multisensor_range_ardupilot/model.sdf \
  src/xq_gz_assets/models/xq_iris_multisensor_range_ardupilot/model.config \
  src/xq_gz_assets/worlds/xq_r2b3_range.sdf src/xq_sim_bringup/launch/xq_r2b3_range.launch.py \
  src/xq_sim_bringup/launch/xq_r2_contract.launch.py; do
  mkdir -p "$run_dir/sources/$(dirname -- "$file")"
  cp "$workspace_root/$file" "$run_dir/sources/$file"
done
find "$run_dir/sources" -type f -print0 | sort -z | xargs -0 sha256sum > "$run_dir/source-snapshot.sha256"
sha256sum "${run_dir}"/configuration/* > "${run_dir}/configuration.sha256"
sha256sum "${world}" "${params}" /home/accelerate/ardupilot/build/sitl/bin/arducopter \
  /home/accelerate/ardupilot_gazebo/build/libArduPilotPlugin.so > "${run_dir}/runtime-dependencies.sha256"
bash "${script_dir}/audit_external_assets.sh" snapshot "${run_dir}/external-assets.before.sha256" >/dev/null
declare -a render_args=()
if [[ "${renderer}" == gpu ]]; then
  export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
  glxinfo -B > "${run_dir}/renderer-probe.txt"
  grep -q 'D3D12 (NVIDIA' "${run_dir}/renderer-probe.txt" || { echo 'Requested NVIDIA renderer unavailable' >&2; exit 4; }
else
  export LIBGL_ALWAYS_SOFTWARE=1 MESA_LOADER_DRIVER_OVERRIDE=llvmpipe GALLIUM_DRIVER=llvmpipe
  export EGL_PLATFORM=surfaceless QT_QPA_PLATFORM=offscreen
  render_args=(--headless-rendering)
fi
replay=false
[[ "${trial}" == replay ]] && replay=true
printf 'trial=%s\nrenderer=%s\nros_domain_id=%s\ngz_partition=%s\nworld=%s\n' \
  "${trial}" "${renderer}" "${ROS_DOMAIN_ID}" "${GZ_PARTITION}" "${world}" > "${run_dir}/run.env"
echo "${display_label}: starting AP/Gazebo dynamics; results ${run_dir}"
stage=simulation_startup
pushd "${run_dir}/sitl_runtime" >/dev/null
start_group sitl /home/accelerate/ardupilot/build/sitl/bin/arducopter \
  -S --model JSON --speedup 1 --slave 0 --wipe \
  --defaults "/home/accelerate/ardupilot/Tools/autotest/default_params/copter.parm,/home/accelerate/ardupilot/Tools/autotest/default_params/gazebo-iris.parm,${params}" \
  --sim-address=127.0.0.1 -I0
popd >/dev/null
wait_log "${run_dir}/sitl.log" 'SERIAL0 on TCP port 5760' 30
start_group mavros ros2 launch mavros apm.launch fcu_url:=tcp://127.0.0.1:5760 namespace:=uav1/mavros
wait_log "${run_dir}/sitl.log" 'Loaded defaults' 45
start_group gazebo gz sim -r -s "${render_args[@]}" -v 3 "${world}"
wait_log "${run_dir}/sitl.log" 'JSON received' 90
wait_log "${run_dir}/mavros.log" 'Got HEARTBEAT' 60
frame_deadline=$((SECONDS + 45))
frame_ready=false
while ((SECONDS < frame_deadline)); do
  if timeout 6 ros2 param get /uav1/mavros/setpoint_velocity mav_frame \
      > "${run_dir}/mavros-velocity-frame.txt" 2>&1 && \
      grep -q 'String value is: LOCAL_NED' "${run_dir}/mavros-velocity-frame.txt"; then
    frame_ready=true
    break
  fi
  sleep 1
done
[[ "${frame_ready}" == true ]] || { echo 'MAVROS velocity frame did not verify' >&2; exit 5; }
declare -a extra_bag_topics=()
declare -a recording_options=(-s sqlite3 --compression-mode none --storage-preset-profile resilient \
  --max-bag-size 268435456 --max-cache-size 104857600)
# Humble 0.15.16 can leave zstd frames incomplete at exact 128 KiB EOF
# boundaries. Keep durable, split raw databases for future research runs.
if [[ "${multisensor}" == true ]]; then
  extra_bag_topics=(/impact/r2/front/image_raw /impact/r2/front/camera_info \
    /impact/r2/down/image_raw /impact/r2/down/camera_info /impact/r2/flow/measurement \
    /impact/r2/flow/status /impact/r2/camera_bridge/status /impact/r2/calibration_metadata \
    /impact/r2b3/down/range /impact/r2b3/eval/range_reference /impact/r2b3/imu \
    /impact/r2b3/native/noisy_scan /impact/r2b3/eval/reference_scan \
    /impact/r2b3/noise_metadata /impact/r2b3/range_bridge/status \
    /impact/r2b3/native/range_source)
fi
start_group rosbag ros2 bag record "${recording_options[@]}" -o "${run_dir}/rosbag" \
  /clock /livox/lidar /livox/imu /localization/odom /localization/geometry /cloud_registered \
  /xq/eval/p4/ground_truth /uav1/mavros/odometry/out /uav1/mavros/local_position/odom \
  /uav1/mavros/state /uav1/mavros/setpoint_velocity/cmd_vel /uav1/mavros/setpoint_position/local \
  /xq/p4/extnav/status /impact/ap/mission_status /impact/ap/execution_enabled /impact/ap/execution_status \
  /impact/research/velocity_cmd /impact/research/command_provenance /xq/p11/flight_status /xq/p12/flight_status \
  /xq/p13/flight_status /integrity/exploration_decision /integrity/exploration_debug /integrity/directional \
  /integrity/information_map /mapping/p12/status /mapping/p12/static_voxels /mapping/p12/dynamic_voxels \
  /planning/p11/frontier_candidates /planning/p11/frontier_candidate_set /planning/p11/selected_bspline \
  /integrity/p13/latency_trace /tf /tf_static "${extra_bag_topics[@]}"
if [[ "${multisensor}" == true ]]; then
  start_group multisensor ros2 launch xq_sim_bringup xq_r2b3_range.launch.py \
    config_file:="${run_dir}/configuration/research_multisensor_contract.json" \
    result_file:="${run_dir}/multisensor-result.json"
fi
start_group base ros2 launch xq_sim_bringup xq_ap_research.launch.py stage:=base config_file:="${config}" \
  mission_result_file:="${run_dir}/mission-result.json" evaluation_result_file:="${run_dir}/evaluation-result.json" \
  replay_test:="${replay}" world_name:="${world_name}"
stage=takeoff_and_hover
python3 "${script_dir}/wait_ap_research_ready.py" --stage hover --timeout 120 --output "${run_dir}/hover-readiness.json" > "${run_dir}/hover-readiness.log" 2>&1
echo "${display_label}: hover confirmed; warming the online maps."
start_group mapping ros2 launch xq_sim_bringup xq_ap_research.launch.py stage:=mapping config_file:="${config}"
stage=mapping_readiness
python3 "${script_dir}/wait_ap_research_ready.py" --stage mapping --timeout 70 --output "${run_dir}/readiness.json" > "${run_dir}/readiness.log" 2>&1
echo "${display_label}: maps ready; starting integrity-constrained research planning."
start_group planning ros2 launch xq_sim_bringup xq_ap_research.launch.py stage:=planning config_file:="${config}" \
  calibration_file:="${run_dir}/configuration/p7-calibration.json"
stage=research_flight
timeout 10 ros2 topic info /xq/eval/p4/ground_truth -v > "${run_dir}/ground-truth-topic-graph.txt"
timeout 10 ros2 topic info /impact/unused_legacy_velocity -v > "${run_dir}/unused-legacy-topic-graph.txt"
timeout 10 ros2 topic info /uav1/mavros/setpoint_velocity/cmd_vel -v > "${run_dir}/ap-command-topic-graph.txt"
deadline=$((SECONDS + 180))
while [[ ! -s "${run_dir}/mission-result.json" ]] && ((SECONDS < deadline)); do sleep 1; done
[[ -s "${run_dir}/mission-result.json" ]] || { echo 'No mission result by runner deadline' >&2; exit 8; }
sleep 2
stage=active_process_audit
python3 - "${run_dir}" "${pids[@]}" <<'PY'
import json, os, re, sys
from pathlib import Path
run = Path(sys.argv[1])
alive = []
for pid in map(int, sys.argv[2:]):
    try:
        os.kill(pid, 0)
        alive.append(True)
    except ProcessLookupError:
        alive.append(False)
matches = {}
for name in ('base.log', 'mapping.log', 'planning.log'):
    matches[name] = [line for line in (run / name).read_text(errors='replace').splitlines()
                     if re.search(r'Traceback|process has died|\[ERROR\]', line)]
if (run / 'multisensor.log').exists():
    matches['multisensor.log'] = [line for line in (run / 'multisensor.log').read_text(errors='replace').splitlines()
                                 if re.search(r'Traceback|process has died|\[ERROR\]', line)]
result = {'status': 'PASS' if all(alive) and not any(matches.values()) else 'FAIL',
          'processes_alive': all(alive), 'runtime_errors': matches}
(run / 'active-node-log-audit.json').write_text(json.dumps(result, indent=2) + '\n')
if result['status'] != 'PASS':
    raise SystemExit('An active process exited or a runtime node error was recorded')
PY
for topic in /impact/r2b3/eval/range_reference /impact/r2b3/eval/reference_scan \
  /impact/r2b3/native/range_source /impact/r2b3/down/range /impact/r2b3/imu; do
  name="${topic//\//_}"
  timeout 10 ros2 topic info "$topic" -v > "$run_dir/topic-graph${name}.txt"
done
stop_groups
python3 - "$run_dir" <<'PY'
import json,sys
from pathlib import Path
run=Path(sys.argv[1])
events=[json.loads(line) for line in (run/'process-shutdown.jsonl').read_text().splitlines()]
issues=[e for e in events if e['exit_code'] not in (0,130) or e['term_forced'] or e['kill_forced']]
(run/'shutdown-result.json').write_text(json.dumps(dict(status='ISSUES' if issues else 'PASS',
    scope='Processes stopped before closed-bag audit; cleanup issues are separate from flight acceptance',
    processes=events,issues=issues),indent=2)+'\n')
# Closure exists now; this explicitly is not a successful completed experiment.
(run/'runner-result.json').write_text(json.dumps(dict(status='POSTFLIGHT_PENDING',
    last_stage='closed_bag_analysis_pending',experiment_complete=False),indent=2)+'\n')
PY
python3 - "${run_dir}" <<'PY'
import json, sys, yaml
from pathlib import Path
run = Path(sys.argv[1])
path = run / 'rosbag/metadata.yaml'
try:
    metadata = yaml.safe_load(path.read_text())['rosbag2_bagfile_information']
    count = int(metadata['message_count'])
    topics = {item['topic_metadata']['name']: int(item['message_count'])
              for item in metadata['topics_with_message_count']}
    result = {'status': 'PASS' if count > 0 else 'FAIL', 'message_count': count, 'topics': topics}
except (OSError, ValueError, KeyError, TypeError) as exc:
    result = {'status': 'FAIL', 'reason': str(exc), 'message_count': 0, 'topics': {}}
(run / 'bag-audit.json').write_text(json.dumps(result, indent=2) + '\n')
PY
python3 "${script_dir}/check_build_manifest.py" --workspace-root "${workspace_root}" > "${run_dir}/post-flight-build-freshness.log"
stage=analysis
declare -a analysis_args=()
[[ "${trial}" == replay ]] && analysis_args=(--replay-test)
python3 "${script_dir}/analyze_ap_research_gate.py" --run-dir "${run_dir}" "${analysis_args[@]}"
if [[ "${multisensor}" == true ]]; then
  python3 "${script_dir}/run_multisensor_fault_matrix.py" --run-dir "${run_dir}"
  python3 "${script_dir}/analyze_multisensor_contract.py" --run-dir "${run_dir}"
  python3 "${script_dir}/audit_multisensor_bag.py" --run-dir "${run_dir}"
  python3 "${script_dir}/audit_range_acquisition.py" --run-dir "${run_dir}" \
    --protocol "$run_dir/configuration/research_range_acquisition_gate.json"
fi
sha256sum -c "$run_dir/source-snapshot.sha256" > "$run_dir/source-snapshot.after.log"
sha256sum -c "$run_dir/configuration.sha256" > "$run_dir/configuration.after.log"
python3 - "$run_dir" "$workspace_root" <<'PY'
import hashlib,sys
from pathlib import Path
run,workspace=map(Path,sys.argv[1:])
for p in (run/'sources').rglob('*'):
    if p.is_file():
        assert hashlib.sha256(p.read_bytes()).digest()==hashlib.sha256((workspace/p.relative_to(run/'sources')).read_bytes()).digest(),str(p)
print('Live source/snapshot identity PASS')
PY
stage=complete
