#!/usr/bin/env bash
# R2B1: serial reception and shared-scale mathematical prerequisites only.
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
workspace_root="$(cd -- "${script_dir}/.." && pwd -P)"
timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
run_dir="${workspace_root}/experiments/results/research_multisensor/r2b1_software_${timestamp}_$$"
protected_helper="/mnt/d/CODEX/impact/evidence/RESEARCH_R2/r2_suite_20261018T165613Z_320/archive_research_r2.py"
protected_baseline="/mnt/d/CODEX/impact/runs/research_bootstrap_20261018/protected.before.json"
mkdir -p "${run_dir}/configuration" "${run_dir}/sources"
stage=preflight

audit_protected() {
  python3 - "${protected_helper}" "${protected_baseline}" "${run_dir}" "$1" <<'PY'
import hashlib, json, runpy, sys
from pathlib import Path
helper, baseline, run = map(Path, sys.argv[1:4])
label = sys.argv[4]
api = runpy.run_path(str(helper))
original = api['read_object'](baseline)
current, result = api['compare_protected'](original)
result['helper_sha256'] = hashlib.sha256(helper.read_bytes()).hexdigest()
(run / ('protected.' + label + '.json')).write_text(json.dumps(current, indent=2) + '\n')
(run / ('protected-audit.' + label + '.json')).write_text(json.dumps(result, indent=2) + '\n')
if result['status'] != 'PASS':
    raise SystemExit('original protected files changed')
PY
}

finish() {
  local code=$?
  trap - EXIT INT TERM
  set +e
  audit_protected after > "${run_dir}/protected-audit.after.log" 2>&1
  [[ $? == 0 ]] || code=12
  bash "${script_dir}/audit_external_assets.sh" snapshot "${run_dir}/external-assets.after.sha256" >/dev/null 2>&1
  bash "${script_dir}/audit_external_assets.sh" compare "${run_dir}/external-assets.before.sha256" "${run_dir}/external-assets.after.sha256" \
    > "${run_dir}/isolation-audit.txt" 2>&1
  [[ $? == 0 ]] || code=13
  python3 - "${run_dir}" "${stage}" "${code}" <<'PY'
import json, sys
from datetime import datetime, timezone
from pathlib import Path
run, stage, code = Path(sys.argv[1]), sys.argv[2], int(sys.argv[3])
summary = dict(schema_version=1, phase='R2B1',
               status='PASS' if code == 0 and stage == 'complete' else 'FAIL',
               last_stage=stage, exit_code=code, run_dir=str(run),
               generated_at_utc=datetime.now(timezone.utc).isoformat(),
               hardware_readiness='NOT_READY', common_state_fusion_tested=False,
               ap_optical_flow_ingestion_tested=False, new_ap_flight_tested=False,
               statistical_integrity_tested=False,
               scope='Software-only synthetic serial PTY/replay and Gaussian shared-scale moment verification; not hardware or fusion validation')
for name, path in [('pty', 'pty/result.json'), ('shared_scale', 'shared-scale/result.json'),
                   ('protected_before', 'protected-audit.before.json'), ('protected_after', 'protected-audit.after.json')]:
    file = run / path
    if file.exists():
        summary[name] = json.loads(file.read_text())
(run / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
print(json.dumps({'status': summary['status'], 'run_dir': str(run), 'last_stage': stage}))
PY
  exit "${code}"
}
trap finish EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
unset GZ_SIM_RESOURCE_PATH IGN_GAZEBO_RESOURCE_PATH SDF_PATH
set +u
source /opt/ros/humble/setup.bash
set -u
export PYTHONNOUSERSITE=1
export PYTHONDONTWRITEBYTECODE=1
export ROS_LOCALHOST_ONLY=1
export ROS2CLI_NO_DAEMON=1
cd "${workspace_root}"
cp "${workspace_root}/config/research_shared_scale_study.json" "${run_dir}/configuration/"
cp "${workspace_root}/config/research_hardware_plan.json" "${run_dir}/configuration/"
cp "${protected_helper}" "${run_dir}/sources/protected_archive_helper.py"
for relative in \
  src/xq_autonomy/xq_autonomy/micolink.py \
  src/xq_autonomy/xq_autonomy/scale_uncertainty.py \
  src/xq_autonomy/test/test_micolink.py \
  src/xq_autonomy/test/test_scale_uncertainty.py \
  scripts/capture_micolink.py scripts/run_micolink_pty_gate.py \
  scripts/run_shared_scale_study.py scripts/run_r2b1_software_gate.sh \
  scripts/check_build_manifest.py scripts/build_isolated.sh \
  scripts/test/test_capture_micolink.py; do
  mkdir -p "${run_dir}/sources/$(dirname -- "${relative}")"
  cp "${workspace_root}/${relative}" "${run_dir}/sources/${relative}"
done
find "${run_dir}/sources" "${run_dir}/configuration" -type f -print0 | sort -z | xargs -0 sha256sum > "${run_dir}/source-configuration.sha256"
audit_protected before > "${run_dir}/protected-audit.before.log" 2>&1
bash "${script_dir}/audit_external_assets.sh" snapshot "${run_dir}/external-assets.before.sha256" >/dev/null

stage=build
bash "${script_dir}/build_isolated.sh" > "${run_dir}/build.log" 2>&1
cp "${workspace_root}/xq_install/.xq_build_manifest.json" "${run_dir}/build-manifest.json"
set +u
source "${workspace_root}/xq_install/setup.bash"
set -u
export PYTHONPATH="${workspace_root}/src/xq_autonomy:${workspace_root}/src/impact_fault_injection:${PYTHONPATH:-}"
python3 - "${run_dir}" <<'PY'
import json, platform, sys
from pathlib import Path
import cv2, numpy, pytest, matplotlib
Path(sys.argv[1], 'environment.json').write_text(json.dumps(dict(
    python=platform.python_version(), numpy=numpy.__version__, opencv=cv2.__version__,
    pytest=pytest.__version__, matplotlib=matplotlib.__version__,
    python_no_user_site=True, python_path=sys.path), indent=2) + '\n')
PY
stage=regression
python3 -m pytest -q -p no:cacheprovider src/xq_autonomy/test src/impact_fault_injection/test scripts/test \
  > "${run_dir}/python-tests.log" 2>&1
stage=serial_pty
python3 "${script_dir}/run_micolink_pty_gate.py" --output "${run_dir}/pty" > "${run_dir}/pty.log" 2>&1
stage=shared_scale_study
python3 "${script_dir}/run_shared_scale_study.py" \
  --config "${run_dir}/configuration/research_shared_scale_study.json" \
  --output "${run_dir}/shared-scale" > "${run_dir}/shared-scale.log" 2>&1
stage=freshness
python3 "${script_dir}/check_build_manifest.py" --workspace-root "${workspace_root}" > "${run_dir}/build-freshness.log" 2>&1
stage=complete
