#!/usr/bin/env bash
# Prepare an isolated, pinned SITL build. Never writes the historical AP tree.
set -euo pipefail
umask 022
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
readonly ap471_workspace=/home/accelerate/impact_research_ws
readonly ap471_tag=Copter-4.7.1
readonly ap471_commit=dbe792162d06cab66c3475fd5556bf7a120f119e
readonly ap471_id="$(date -u +%Y%m%dT%H%M%SZ)_$$"
readonly ap471_run="$ap471_workspace/experiments/results/research_multisensor/ap471_prepare_$ap471_id"
readonly ap471_external="$ap471_workspace/external/ap471_$ap471_id"
ap471_reuse_source=''
if (($#)); then
    [[ $# == 2 && "$1" == --reuse-source ]] || { echo 'Usage: prepare_ap471_research.sh [--reuse-source PINNED_SOURCE]' >&2; exit 2; }
    ap471_reuse_source="$(realpath -- "$2")"
    [[ "$ap471_reuse_source" == "$ap471_workspace"/external/ap471_*/source ]] || { echo 'Refusing source outside isolated AP 4.7.1 directories' >&2; exit 2; }
fi
readonly ap471_source="${ap471_reuse_source:-$ap471_external/source}"
readonly ap471_python="$ap471_external/python/bin/python3"
readonly ap471_build="$ap471_workspace/build/ap471_$ap471_id"
readonly ap471_binary="$ap471_build/sitl/bin/arducopter"
mkdir -p "$ap471_workspace/experiments/results/research_multisensor" "$ap471_workspace/external" "$ap471_workspace/build"
mkdir "$ap471_run" "$ap471_external" "$ap471_build"
cp -- "${BASH_SOURCE[0]}" "$ap471_run/prepare_ap471_research.sh"
ap471_stage=preflight
ap471_result=FAIL
export AP471_RUN="$ap471_run" AP471_SOURCE="$ap471_source" AP471_BUILD="$ap471_build" AP471_BINARY="$ap471_binary" AP471_COMMIT="$ap471_commit" AP471_TAG="$ap471_tag" AP471_PYTHON="$ap471_python"
finish_ap471() {
    local ap471_exit=$?
    trap - EXIT
    export AP471_STAGE="$ap471_stage" AP471_RESULT="$ap471_result" AP471_EXIT="$ap471_exit"
    python3 - <<'PY'
import hashlib, json, os
from pathlib import Path
from datetime import datetime, timezone
run = Path(os.environ['AP471_RUN'])
record = {k.lower().removeprefix('ap471_'): os.environ[k] for k in (
    'AP471_SOURCE', 'AP471_BUILD', 'AP471_BINARY', 'AP471_COMMIT', 'AP471_TAG',
    'AP471_STAGE', 'AP471_RESULT', 'AP471_EXIT', 'AP471_PYTHON')}
record.update(closed_at_utc=datetime.now(timezone.utc).isoformat(),
              purpose='isolated SITL compilation only; no firmware flashing or flight',
              historical_ap_tree='/home/accelerate/ardupilot', historical_ap_tree_written=False)
binary = Path(record['binary'])
if binary.is_file():
    record['binary_size'] = binary.stat().st_size
    record['binary_sha256'] = hashlib.file_digest(binary.open('rb'), 'sha256').hexdigest() if hasattr(hashlib, 'file_digest') else hashlib.sha256(binary.read_bytes()).hexdigest()
for name, value in (('status.json', record),):
    with (run/name).open('x') as out:
        json.dump(value, out, indent=2, allow_nan=False); out.write('\n')
print(json.dumps(record, separators=(',', ':')))
PY
    exit "$ap471_exit"
}
trap finish_ap471 EXIT
printf 'AP471_RUN=%s\nAP471_SOURCE=%s\nAP471_BUILD=%s\n' "$ap471_run" "$ap471_source" "$ap471_build"
{
    uname -a
    cat /proc/sys/kernel/random/boot_id
    python3 --version
    g++ --version
    git --version
    command -v python3 g++ git
    free -h
    df -h "$ap471_workspace"
    python3 -m pip freeze
} > "$ap471_run/environment.log" 2>&1
ap471_stage=resolve_tag
git ls-remote https://github.com/ArduPilot/ardupilot.git "refs/tags/$ap471_tag" "refs/tags/$ap471_tag^{}" > "$ap471_run/remote-tag.log" 2>&1
grep -Fx "$ap471_commit"$'\t'"refs/tags/$ap471_tag" "$ap471_run/remote-tag.log"
ap471_stage=clone
if [[ -z "$ap471_reuse_source" ]]; then
    git clone --depth 1 --branch "$ap471_tag" --single-branch https://github.com/ArduPilot/ardupilot.git "$ap471_source" > "$ap471_run/clone.log" 2>&1
else
    printf 'Reusing isolated pinned source: %s\n' "$ap471_source" > "$ap471_run/clone.log"
    test -z "$(git -C "$ap471_source" status --porcelain=v1 --untracked-files=all)"
fi
test "$(git -C "$ap471_source" rev-parse HEAD)" = "$ap471_commit"
git -C "$ap471_source" show --no-patch --format=fuller HEAD > "$ap471_run/source-commit.log"
cp "$ap471_source/ArduCopter/version.h" "$ap471_run/version.h"
grep -Fx '#define THISFIRMWARE "ArduCopter V4.7.1"' "$ap471_run/version.h"
ap471_stage=submodules
git -C "$ap471_source" submodule update --init --recursive --depth 1 --jobs 2 > "$ap471_run/submodules.log" 2>&1
git -C "$ap471_source" submodule status --recursive > "$ap471_run/submodule-status.log"
if grep -Eq '^[+-U]' "$ap471_run/submodule-status.log"; then
    printf 'Submodule checkout does not match pinned superproject\n' >&2
    exit 1
fi
ap471_stage=private_python
python3 -m venv "$ap471_external/python" > "$ap471_run/private-python.log" 2>&1
"$ap471_python" -m pip install --disable-pip-version-check pexpect==4.9.0 ptyprocess==0.7.0 empy==3.3.4 future==1.0.0 lxml==6.1.1 pymavlink==2.4.49 fastcrc==0.3.6 pyserial==3.5 > "$ap471_run/python-dependencies.log" 2>&1
"$ap471_python" -m pip freeze > "$ap471_run/python-freeze.txt"
ap471_stage=configure
(cd "$ap471_source"; "$ap471_python" ./waf configure --board sitl --out "$ap471_build") > "$ap471_run/configure.log" 2>&1
ap471_stage=build
(cd "$ap471_source"; "$ap471_python" ./waf copter -j2) > "$ap471_run/build.log" 2>&1
test -x "$ap471_binary"
ap471_stage=version_check
python3 - "$ap471_binary" "$ap471_run/binary-version.log" <<'PY'
import re, sys
from pathlib import Path
data = Path(sys.argv[1]).read_bytes()
versions = [match.decode('ascii') for match in re.findall(rb'ArduCopter V4\.7\.1[^\x00\r\n]*', data)]
if not versions:
    raise SystemExit('Pinned ArduCopter version absent from compiled ELF')
with Path(sys.argv[2]).open('x') as out:
    out.write('\n'.join(versions)+'\n')
print('\n'.join(versions))
PY
(cd "$ap471_run"; "$ap471_binary" --help) > "$ap471_run/binary-help.log" 2>&1
sha256sum "$ap471_binary" > "$ap471_run/binary-sha256.txt"
git -C "$ap471_source" status --porcelain=v1 --untracked-files=all > "$ap471_run/source-status.log"
test ! -s "$ap471_run/source-status.log"
python3 - <<'PY'
import hashlib,json,os
from pathlib import Path
run=Path(os.environ['AP471_RUN'])
files=[]
for path in sorted(run.iterdir()):
    if path.is_file():
        files.append({'path':path.name,'size':path.stat().st_size,'sha256':hashlib.sha256(path.read_bytes()).hexdigest()})
with (run/'artifact-manifest.json').open('x') as out:
    json.dump({'schema_version':1,'files':files,'status_json_written_after_manifest':True},out,indent=2);out.write('\n')
PY
ap471_stage=complete
ap471_result=PASS
