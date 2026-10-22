#!/usr/bin/env bash
# R2B2: two arms on the same recorded acquisition sequence, off all AP paths.
set -euo pipefail
ulimit -c 0
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
workspace_root="$(cd -- "${script_dir}/.." && pwd -P)"
stamp="$(date -u +%Y%m%dT%H%M%SZ)"
run="${workspace_root}/experiments/results/research_multisensor/r2b2_replay_${stamp}_$$"
input="${workspace_root}/experiments/results/research_multisensor/r2b2_input_v2_20261019"
mkdir -p "${run}/configuration" "${run}/sources"
stage=preflight
publisher_pid=""
mapper_pid=""
finish() {
  local code=$?
  trap - EXIT INT TERM
  set +e
  for pid in "${publisher_pid}" "${mapper_pid}"; do
    [[ -z "$pid" ]] || kill -INT "$pid" 2>/dev/null
  done
  for round in {1..30}; do
    local alive=false
    for pid in "${publisher_pid}" "${mapper_pid}"; do [[ -z "$pid" ]] || { kill -0 "$pid" 2>/dev/null && alive=true; }; done
    [[ "$alive" == false ]] && break
    sleep 0.1
  done
  for pid in "${publisher_pid}" "${mapper_pid}"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -TERM "$pid"
      # Let timeout's --kill-after=5s finish before a final bounded group kill.
      for attempt in {1..60}; do kill -0 "$pid" 2>/dev/null || break; sleep 0.1; done
      if kill -0 "$pid" 2>/dev/null; then
        local process_group
        process_group="$(ps -o pgid= -p "$pid" | tr -d ' ')"
        if [[ "$process_group" == "$pid" ]]; then
          kill -KILL -- "-${process_group}" 2>/dev/null
        else
          kill -KILL "$pid" 2>/dev/null
          printf 'Unverified child group for timeout PID %s\n' "$pid" >> "${run}/cleanup-issues.log"
        fi
      fi
      wait "$pid"
    fi
  done
  python3 - "${run}" "${stage}" "${code}" <<'PY'
import json, sys
from datetime import datetime, timezone
from pathlib import Path
run,stage,code=Path(sys.argv[1]),sys.argv[2],int(sys.argv[3])
result=dict(status='PASS' if stage=='complete' and code==0 else 'FAIL',last_stage=stage,exit_code=code,
            generated_at_utc=datetime.now(timezone.utc).isoformat(),run_dir=str(run),
            scope='Offline shared-state LIVO replay only; no AP/control or hardware integration')
(run/'runner-result.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result))
PY
  exit "$code"
}
trap finish EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
unset GZ_SIM_RESOURCE_PATH IGN_GAZEBO_RESOURCE_PATH SDF_PATH
set +u
source /opt/ros/humble/setup.bash
source "${workspace_root}/xq_install/setup.bash"
source "${workspace_root}/livo_install/setup.bash"
set -u
export LD_LIBRARY_PATH="${workspace_root}/livo_deps/lib:${LD_LIBRARY_PATH:-}"
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
unset ROS_DISCOVERY_SERVER CYCLONEDDS_URI FASTDDS_DEFAULT_PROFILES_FILE FASTRTPS_DEFAULT_PROFILES_FILE
export ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS2CLI_NO_DAEMON=1 OMP_NUM_THREADS=2
export ROS_DOMAIN_ID=$((120 + $$ % 70))
export ROS_LOG_DIR="${run}/ros_logs"
cd "${workspace_root}"
python3 scripts/prepare_livo_transport.py --output-dir "${run}/configuration" > "${run}/transport-setup.log"
export FASTRTPS_DEFAULT_PROFILES_FILE="${run}/configuration/fastdds-replay.xml"
cp config/research_livo_replay_gate.json "${run}/configuration/"
cp src/xq_fast_livo2/config/r2_snapshot.yaml "${run}/configuration/"
cp "${input}/sensor-index.json" "${input}/source-stamps.json" "${input}/truth.jsonl" "${run}/"
cp "${workspace_root}/livo_deps/dependency-manifest.json" "${run}/configuration/"
cp "${workspace_root}/xq_install/.xq_build_manifest.json" "${run}/configuration/"
cp "${workspace_root}/livo_install/.impact-build-manifest.json" "${run}/configuration/"
cp -r src/xq_fast_livo2 "${run}/sources/"
cp scripts/replay_livo_inputs.py scripts/analyze_livo_replay.py scripts/run_livo_replay_gate.sh \
  scripts/audit_livo_replay_artifacts.py scripts/wait_livo_processing.py \
  scripts/audit_livo_processing.py scripts/check_livo_build_manifest.py \
  scripts/build_livo_isolated.sh scripts/prepare_livo_transport.py "${run}/sources/"
sha256sum "${workspace_root}/livo_install/xq_fast_livo2/lib/xq_fast_livo2/xq_livo_mapping" \
  "${workspace_root}/livo_install/xq_fast_livo2/lib/libxq_livo_core.a" > "${run}/binary.sha256"
python3 scripts/check_build_manifest.py --workspace-root "${workspace_root}" > "${run}/build-freshness.before.log"
python3 scripts/check_livo_build_manifest.py --workspace-root "${workspace_root}" > "${run}/livo-build-freshness.before.log"
python3 - "${run}" <<'PY'
import hashlib,json,sys
from pathlib import Path
run=Path(sys.argv[1]); source=json.loads((run/'sensor-index.json').read_text())
protocol=json.loads((run/'configuration/research_livo_replay_gate.json').read_text())
if source['camera_keep_every_n']!=protocol['camera_keep_every_n'] or source['bag']!=protocol['source_bag']:
    raise SystemExit('input index differs from frozen replay protocol')
stamps={key: [row['stamp_ns'] for row in source['entries'] if row['sensor']==key]
        for key in ('front','lidar','imu')}
if any(not values or any(b<=a for a,b in zip(values,values[1:])) for values in stamps.values()):
    raise SystemExit('nonmonotone completion inputs')
eligible=[value for value in stamps['front'] if stamps['lidar'][0]<value<=min(stamps['lidar'][-1],stamps['imu'][-1])]
if not eligible: raise SystemExit('no fully covered image epoch')
contract=dict(expected_lidar_messages=len(stamps['lidar']),expected_imu_messages=len(stamps['imu']),
              expected_image_messages=len(stamps['front']),last_eligible_image_stamp_ns=eligible[-1],
              last_lidar_stamp_ns=stamps['lidar'][-1],last_imu_stamp_ns=stamps['imu'][-1])
(run/'configuration/replay-completion-contract.json').write_text(json.dumps(contract,indent=2)+'\n')
(run/'configuration/completion-arguments.txt').write_text(''.join('research.'+key+':='+str(value)+'\n' for key,value in contract.items()))
frozen=Path('/mnt/d/CODEX/impact/evidence/RESEARCH_R2/r2_suite_20261018T165613Z_320/r2_contract_20261018T165306Z_447/camera-bag-audit.json')
expected=json.loads(frozen.read_text())['original_files_before']
bag=Path(source['bag']).resolve()
if {Path(item['file']).name for item in source['raw_files']} != {name for name in expected if name.endswith('.db3')}:
    raise SystemExit('index does not bind every original database')
for item in source['raw_files']:
    path=Path(item['file']).resolve()
    if path.parent!=bag or expected.get(path.name)!=dict(bytes=item['size_bytes'],sha256=item['sha256']):
        raise SystemExit('indexed database not bound to frozen bag')
records=[]
for name,item in expected.items():
    path=Path(source['bag'])/name
    h=hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''): h.update(block)
    actual=dict(bytes=path.stat().st_size,sha256=h.hexdigest())
    records.append(dict(path=str(path),**actual))
    if actual!=item: raise SystemExit('source bag changed: '+name)
(run/'source-bag.before.json').write_text(json.dumps(records,indent=2)+'\n')
PY
mapfile -t completion_parameters < "${run}/configuration/completion-arguments.txt"
completion_arguments=()
for parameter in "${completion_parameters[@]}"; do completion_arguments+=(-p "$parameter"); done
for arm in arm_livo arm_no_visual; do
  stage="replay_${arm}"
  mkdir -p "${run}/${arm}"
  enabled=true
  [[ "${arm}" != arm_no_visual ]] || enabled=false
  timeout --signal=INT --kill-after=5s 300s python3 scripts/replay_livo_inputs.py publish --index "${run}/sensor-index.json" \
    --output "${run}/${arm}" --rate 0.5 > "${run}/${arm}/publisher.log" 2>&1 &
  publisher_pid=$!
  for attempt in {1..600}; do
    [[ -f "${run}/${arm}/parameter-ready.json" ]] && break
    kill -0 "${publisher_pid}" 2>/dev/null || { wait "${publisher_pid}"; exit 8; }
    sleep 0.05
  done
  [[ -f "${run}/${arm}/parameter-ready.json" ]] || { printf 'Publisher startup did not reach READY\n' >&2; exit 8; }
  timeout --signal=INT --kill-after=5s 300s "${workspace_root}/livo_install/xq_fast_livo2/lib/xq_fast_livo2/xq_livo_mapping" \
    --ros-args --params-file "${run}/configuration/r2_snapshot.yaml" \
    -p "research.visual_update_enabled:=${enabled}" \
    -p "research.run_dir:=${run}/${arm}" \
    -p "research.run_file:=${run}/${arm}/visual-updates.jsonl" \
    "${completion_arguments[@]}" \
    > "${run}/${arm}/mapping.log" 2>&1 &
  mapper_pid=$!
  wait "${publisher_pid}"
  publisher_pid=""
  python3 scripts/wait_livo_processing.py --run-dir "${run}/${arm}" --mapper-pid "${mapper_pid}" \
    --timeout-s 30 > "${run}/${arm}/processing-wait.log" 2>&1
  kill -INT "${mapper_pid}"
  wait "${mapper_pid}"
  mapper_pid=""
done
stage=evaluation
python3 scripts/analyze_livo_replay.py --run-dir "${run}" --protocol "${run}/configuration/research_livo_replay_gate.json" > "${run}/evaluation.log" 2>&1
stage=postflight_audit
python3 scripts/audit_livo_replay_artifacts.py --run-dir "${run}" > "${run}/artifact-audit.log" 2>&1
sha256sum -c "${run}/binary.sha256" > "${run}/binary-freshness.after.log"
python3 scripts/check_build_manifest.py --workspace-root "${workspace_root}" > "${run}/build-freshness.after.log"
python3 scripts/check_livo_build_manifest.py --workspace-root "${workspace_root}" > "${run}/livo-build-freshness.after.log"
stage=complete
