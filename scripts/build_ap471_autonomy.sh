#!/usr/bin/env bash
# Isolated package-only build. The historical xq_install is a read-only underlay.
set -euo pipefail
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
readonly ap471_ws=/home/accelerate/impact_research_ws
readonly ap471_id="$(date -u +%Y%m%dT%H%M%SZ)_$$"
export AP471_AUTONOMY_RUN="$ap471_ws/experiments/results/research_multisensor/ap471_autonomy_build_$ap471_id"
export AP471_AUTONOMY_SOURCE="$ap471_ws/external/ap471_autonomy_$ap471_id/xq_autonomy"
export AP471_AUTONOMY_INSTALL="$ap471_ws/ap471_autonomy_install_$ap471_id"
readonly ap471_build="$ap471_ws/ap471_autonomy_build_$ap471_id"
readonly ap471_log="$ap471_ws/ap471_autonomy_log_$ap471_id"
mkdir "$AP471_AUTONOMY_RUN" "$(dirname "$AP471_AUTONOMY_SOURCE")"
cp "${BASH_SOURCE[0]}" "$AP471_AUTONOMY_RUN/build_ap471_autonomy.sh"
ap471_stage=freeze
ap471_result=FAIL
finish_ap471_autonomy() {
    local ap471_exit=$?
    trap - EXIT
    export AP471_AUTONOMY_STAGE="$ap471_stage" AP471_AUTONOMY_RESULT="$ap471_result" AP471_AUTONOMY_EXIT="$ap471_exit"
    python3 - <<'PY'
import json,os
from pathlib import Path
run=Path(os.environ['AP471_AUTONOMY_RUN'])
value={name.lower().removeprefix('ap471_autonomy_'):os.environ[name] for name in (
    'AP471_AUTONOMY_RUN','AP471_AUTONOMY_SOURCE','AP471_AUTONOMY_INSTALL',
    'AP471_AUTONOMY_STAGE','AP471_AUTONOMY_RESULT','AP471_AUTONOMY_EXIT')}
with (run/'status.json').open('x') as out:
    json.dump(value,out,indent=2);out.write('\n')
print(json.dumps(value))
PY
    exit "$ap471_exit"
}
trap finish_ap471_autonomy EXIT
printf 'AP471_AUTONOMY_RUN=%s\nAP471_AUTONOMY_INSTALL=%s\n' "$AP471_AUTONOMY_RUN" "$AP471_AUTONOMY_INSTALL"
python3 - <<'PY'
import hashlib,json,os,shutil
from pathlib import Path
canonical=Path('/mnt/d/CODEX/impact/src/xq_autonomy')
run=Path(os.environ['AP471_AUTONOMY_RUN'])
def inventory(root):
    return {str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(root.rglob('*'))
            if p.is_file() and not any(x in p.parts for x in ('__pycache__','.pytest_cache')) and p.suffix!='.pyc'}
shutil.copytree(canonical,Path(os.environ['AP471_AUTONOMY_SOURCE']),ignore=shutil.ignore_patterns('__pycache__','.pytest_cache','*.pyc'))
with (run/'inputs-before.json').open('x') as out:
    json.dump({'canonical':inventory(canonical),'underlay':inventory(Path('/home/accelerate/impact_research_ws/xq_install'))},out,indent=2);out.write('\n')
PY
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
set +u
source /opt/ros/humble/setup.bash
source "$ap471_ws/xq_install/setup.bash"
set -u
ap471_stage=build
colcon --log-base "$ap471_log" build --base-paths "$AP471_AUTONOMY_SOURCE" \
    --build-base "$ap471_build" --install-base "$AP471_AUTONOMY_INSTALL" \
    --packages-select xq_autonomy --event-handlers console_direct+ > "$AP471_AUTONOMY_RUN/build.log" 2>&1
set +u
source "$AP471_AUTONOMY_INSTALL/setup.bash"
set -u
ap471_stage=tests
python3 -m pytest -q /mnt/d/CODEX/impact/src/xq_autonomy/test > "$AP471_AUTONOMY_RUN/tests.log" 2>&1
ap471_stage=verify
python3 - <<'PY'
import hashlib,json,os
from pathlib import Path
canonical=Path('/mnt/d/CODEX/impact/src/xq_autonomy')
run=Path(os.environ['AP471_AUTONOMY_RUN']);install=Path(os.environ['AP471_AUTONOMY_INSTALL'])
def inventory(root):
    return {str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(root.rglob('*'))
            if p.is_file() and not any(x in p.parts for x in ('__pycache__','.pytest_cache')) and p.suffix!='.pyc'}
before=json.loads((run/'inputs-before.json').read_text())
after={'canonical':inventory(canonical),'underlay':inventory(Path('/home/accelerate/impact_research_ws/xq_install'))}
with (run/'inputs-after.json').open('x') as out:
    json.dump(after,out,indent=2);out.write('\n')
if before!=after:
    raise SystemExit('Canonical sources or read-only xq_install underlay changed during build')
site=install/'xq_autonomy/lib/python3.10/site-packages/xq_autonomy'
bound={}
for relative,digest in before['canonical'].items():
    if relative.startswith('xq_autonomy/') and relative.endswith('.py'):
        actual=site/relative.removeprefix('xq_autonomy/')
        if not actual.is_file() or hashlib.sha256(actual.read_bytes()).hexdigest()!=digest:
            raise SystemExit('Installed module differs from canonical source: '+relative)
        bound[relative]={'sha256':digest,'installed_path':str(actual)}
manifest={'schema_version':1,'status':'PASS','scope':'AP471 xq_autonomy-only isolated build; not full workspace manifest',
          'firmware_commit':'dbe792162d06cab66c3475fd5556bf7a120f119e',
          'canonical_source_sha256':before['canonical'],'installed_modules':bound,
          'underlay_file_set_and_sha256_unchanged':True,'build_run':str(run)}
with (install/'.xq_build_manifest.json').open('x') as out:
    json.dump(manifest,out,indent=2);out.write('\n')
with (run/'build-manifest.json').open('x') as out:
    json.dump(manifest,out,indent=2);out.write('\n')
print(json.dumps({'status':'PASS','installed_module_count':len(bound),'underlay_file_count':len(before['underlay'])}))
PY
ap471_stage=complete
ap471_result=PASS
