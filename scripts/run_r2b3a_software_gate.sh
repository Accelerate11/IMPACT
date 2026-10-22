#!/usr/bin/env bash
# Flow preconditions only: no flow state injection, ROS node or new AP flight.
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
workspace_root="$(cd -- "${script_dir}/.." && pwd -P)"
[[ "$workspace_root" == /home/accelerate/impact_research_ws ]] || { echo 'Unexpected research workspace' >&2; exit 2; }
run="${workspace_root}/experiments/results/research_multisensor/r2b3a_$(date -u +%Y%m%dT%H%M%SZ)_$$"
canonical=/mnt/d/CODEX/impact
protected_helper="${canonical}/evidence/RESEARCH_R2/r2_suite_20261018T165613Z_320/archive_research_r2.py"
protected_baseline="${canonical}/runs/research_bootstrap_20261018/protected.before.json"
mkdir -p "${run}/configuration" "${run}/sources"
stage=preflight

audit_protected() {
  python3 - "$protected_helper" "$protected_baseline" "$run" "$1" <<'PY'
import hashlib,json,runpy,sys
from pathlib import Path
helper,baseline,run=map(Path,sys.argv[1:4]); label=sys.argv[4]
api=runpy.run_path(str(helper)); current,result=api['compare_protected'](api['read_object'](baseline))
result['helper_sha256']=hashlib.sha256(helper.read_bytes()).hexdigest()
(run/('protected.'+label+'.json')).write_text(json.dumps(current,indent=2)+'\n')
(run/('protected-audit.'+label+'.json')).write_text(json.dumps(result,indent=2)+'\n')
if result['status']!='PASS': raise SystemExit('Protected original files changed')
PY
}

verify_sources() {
  python3 - "$run" "$workspace_root" <<'PY'
import hashlib,sys
from pathlib import Path
run,workspace=map(Path,sys.argv[1:])
for root in ('sources','configuration'):
    for path in (run/root).rglob('*'):
        if not path.is_file() or path.name=='protected_archive_helper.py': continue
        other=workspace/path.relative_to(run/root) if root=='sources' else workspace/'config'/path.name
        if hashlib.sha256(path.read_bytes()).digest()!=hashlib.sha256(other.read_bytes()).digest():
            raise SystemExit('Running source/config differs from snapshot: '+str(other))
print('Source/configuration identity PASS')
PY
}

finish() {
  local code=$?
  trap - EXIT INT TERM
  set +e
  audit_protected after > "$run/protected-audit.after.log" 2>&1
  [[ $? == 0 ]] || code=12
  bash "$script_dir/audit_external_assets.sh" snapshot "$run/external-assets.after.sha256" >/dev/null 2>&1
  bash "$script_dir/audit_external_assets.sh" compare "$run/external-assets.before.sha256" "$run/external-assets.after.sha256" > "$run/isolation-audit.txt" 2>&1
  [[ $? == 0 ]] || code=13
  python3 - "$run" "$stage" "$code" <<'PY'
import json,re,sys
from datetime import datetime,timezone
from pathlib import Path
run,stage,code=Path(sys.argv[1]),sys.argv[2],int(sys.argv[3])
result=dict(schema_version=1,phase='R2B3A',status='PASS' if code==0 and stage=='complete' else 'FAIL',
            last_stage=stage,exit_code=code,run_dir=str(run),generated_at_utc=datetime.now(timezone.utc).isoformat(),
            hardware_readiness='NOT_READY',actual_flow_state_updates=0,full_common_state_flow_fusion_validated=False,
            new_ap_flight_tested=False,statistical_integrity_tested=False,
            scope='Recorded simulation flow geometry/provenance and a single-window synthetic correlated Gaussian update; not full flow fusion')
for name,relative in [('flow_geometry','flow-geometry/replay-result.json'),('correlation_study','correlated-study/study-result.json'),
                      ('independent_geometry_audit','flow-geometry/audit-result.json'),
                      ('independent_study_audit','correlated-study/audit-result.json'),
                      ('protected_before','protected-audit.before.json'),('protected_after','protected-audit.after.json')]:
    path=run/relative
    if path.exists(): result[name]=json.loads(path.read_text())
log=run/'python-tests.log'
if log.exists():
    matches=re.findall(r'(\d+) passed',log.read_text())
    result['python_tests_passed']=int(matches[-1]) if matches else None
(run/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
print(json.dumps({'status':result['status'],'run_dir':str(run),'last_stage':stage,'python_tests_passed':result.get('python_tests_passed')}))
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
set -u
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 ROS2CLI_NO_DAEMON=1
cd "$workspace_root"
cp config/research_flow_precondition_gate.json config/research_hardware_plan.json "$run/configuration/"
cp "$protected_helper" "$run/sources/protected_archive_helper.py"
for relative in \
  src/xq_autonomy/xq_autonomy/correlated_update.py \
  src/xq_autonomy/xq_autonomy/flow_geometry.py \
  src/xq_autonomy/xq_autonomy/downward_flow.py \
  src/xq_autonomy/test/test_correlated_update.py \
  src/xq_autonomy/test/test_flow_geometry.py \
  scripts/run_correlated_flow_study.py scripts/audit_correlated_flow_study.py \
  scripts/prepare_flow_geometry_replay.py scripts/audit_flow_geometry_replay.py scripts/run_r2b3a_software_gate.sh \
  scripts/test/test_correlated_flow_study_audit.py scripts/test/test_flow_geometry_replay_audit.py \
  scripts/test/test_prepare_flow_geometry_replay.py \
  scripts/build_isolated.sh scripts/build_livo_isolated.sh scripts/check_build_manifest.py \
  scripts/check_livo_build_manifest.py scripts/run_livo_regression.sh scripts/audit_external_assets.sh; do
  mkdir -p "$run/sources/$(dirname -- "$relative")"
  cp "$relative" "$run/sources/$relative"
done
find "$run/sources" "$run/configuration" -type f -print0 | sort -z | xargs -0 sha256sum > "$run/source-configuration.sha256"
verify_sources > "$run/source-identity.before.log"
audit_protected before > "$run/protected-audit.before.log" 2>&1
bash "$script_dir/audit_external_assets.sh" snapshot "$run/external-assets.before.sha256" >/dev/null
python3 - "$workspace_root" "$canonical" "$run" <<'PY'
import hashlib,json,sys
from pathlib import Path
workspace,canonical,run=map(Path,sys.argv[1:])
previous=canonical/'evidence/RESEARCH_R2B2/r2b2_suite_20261019T141249Z/r2b2_replay_20261019T140630Z_1564/sources/xq_fast_livo2'
def tree(root): return {str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in root.rglob('*') if p.is_file()}
current=tree(workspace/'src/xq_fast_livo2'); expected=tree(previous)
result=dict(status='PASS' if current==expected else 'FAIL',source_files=len(current),cpp_common_state_flow_changes=False,source_sha256=current)
(run/'livo-source-preserved.json').write_text(json.dumps(result,indent=2)+'\n')
if result['status']!='PASS': raise SystemExit('R2B3A must not silently change the established LIVO estimator')
PY
stage=build
bash "$script_dir/build_isolated.sh" > "$run/build.log" 2>&1
cp xq_install/.xq_build_manifest.json "$run/base-build-manifest.json"
# Rebind the unchanged LIVO source/binary to the newly built base prefix.
bash "$script_dir/build_livo_isolated.sh" > "$run/livo-build.log" 2>&1
cp livo_install/.impact-build-manifest.json "$run/livo-build-manifest.json"
set +u
source "$workspace_root/xq_install/setup.bash"
set -u
export PYTHONPATH="$workspace_root/src/xq_autonomy:$workspace_root/src/impact_fault_injection:${PYTHONPATH:-}"
python3 - "$run" <<'PY'
import inspect,json,platform,sys
from pathlib import Path
import cv2,numpy,pytest
import xq_autonomy.correlated_update as update
import xq_autonomy.flow_geometry as geometry
result=dict(python=platform.python_version(),numpy=numpy.__version__,opencv=cv2.__version__,pytest=pytest.__version__,
            geometry_source=inspect.getfile(geometry),correlated_update_source=inspect.getfile(update),python_no_user_site=True)
Path(sys.argv[1],'environment.json').write_text(json.dumps(result,indent=2)+'\n')
PY
stage=regression
bash "$script_dir/run_livo_regression.sh" > "$run/python-tests.log" 2>&1
stage=flow_geometry_replay
python3 "$script_dir/prepare_flow_geometry_replay.py" --protocol "$run/configuration/research_flow_precondition_gate.json" \
  --output "$run/flow-geometry" --canonical-root "$canonical" > "$run/flow-geometry.log" 2>&1
stage=independent_geometry_audit
python3 "$script_dir/audit_flow_geometry_replay.py" --replay-dir "$run/flow-geometry" \
  --protocol "$run/configuration/research_flow_precondition_gate.json" > "$run/independent-geometry-audit.log" 2>&1
stage=correlated_study
python3 "$script_dir/run_correlated_flow_study.py" --protocol "$run/configuration/research_flow_precondition_gate.json" \
  --output "$run/correlated-study" > "$run/correlated-study.log" 2>&1
stage=independent_study_audit
python3 "$script_dir/audit_correlated_flow_study.py" --study-dir "$run/correlated-study" \
  --protocol "$run/configuration/research_flow_precondition_gate.json" > "$run/independent-study-audit.log" 2>&1
stage=freshness
python3 "$script_dir/check_build_manifest.py" --workspace-root "$workspace_root" > "$run/build-freshness.after.log"
python3 "$script_dir/check_livo_build_manifest.py" --workspace-root "$workspace_root" > "$run/livo-build-freshness.after.log"
sha256sum -c "$run/source-configuration.sha256" > "$run/source-configuration.after.log"
verify_sources > "$run/source-identity.after.log"
stage=complete
