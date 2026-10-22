#!/usr/bin/env bash
# Build only bundled sources in a fresh external runtime. Never source old overlays.
set -euo pipefail
root=""
runtime_root=""
jobs=1
while (($#)); do
  case "$1" in
    --root) root="$2"; shift 2;;
    --runtime-root) runtime_root="$2"; shift 2;;
    --jobs) jobs="$2"; shift 2;;
    *) echo "Usage: $0 --root BUNDLE --runtime-root NEW_RUNTIME_PARENT [--jobs 1]" >&2; exit 2;;
  esac
done
[[ -n "$root" && -n "$runtime_root" && "$jobs" =~ ^[1-9][0-9]*$ ]] || { echo "Missing/invalid arguments" >&2; exit 2; }
root="$(realpath -e -- "$root")"
runtime_root="$(realpath -m -- "$runtime_root")"
[[ -d "$root/src" && "$runtime_root" != "$root" && "$runtime_root" != "$root/"* ]] || { echo "Runtime must be external to bundle" >&2; exit 2; }
mkdir -p -- "$runtime_root"
attempt="$runtime_root/build_$(date -u +%Y%m%dT%H%M%SZ)_$$"
mkdir -- "$attempt"
export IMPACT_BUILD_ROOT="$root" IMPACT_BUILD_ATTEMPT="$attempt" IMPACT_BUILD_JOBS="$jobs"
echo "$attempt"
finish_build() {
  local code=$?
  if [[ ! -f "$attempt/build-result.json" ]]; then
    IMPACT_BUILD_EXIT="$code" python3 -c 'import json,os;from pathlib import Path;p=Path(os.environ["IMPACT_BUILD_ATTEMPT"])/"build-result.json";p.open("x").write(json.dumps({"schema_version":1,"status":"FAIL","exit_code":int(os.environ["IMPACT_BUILD_EXIT"]),"scope":"Build harness stopped before normal result; inspect retained logs."},indent=2)+"\n")'
  fi
}
trap finish_build EXIT
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
unset GZ_SIM_RESOURCE_PATH IGN_GAZEBO_RESOURCE_PATH SDF_PATH GZ_SIM_SYSTEM_PLUGIN_PATH
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
export MAKEFLAGS="-j$jobs" CMAKE_BUILD_PARALLEL_LEVEL="$jobs"
set +u
source /opt/ros/humble/setup.bash
set -u
python3 - <<'PY'
from pathlib import Path
import hashlib,json,os,platform,subprocess,sys
root=Path(os.environ["IMPACT_BUILD_ROOT"]); attempt=Path(os.environ["IMPACT_BUILD_ATTEMPT"])
def sha(p):
 h=hashlib.sha256()
 with p.open("rb") as f:
  for b in iter(lambda:f.read(1048576),b""):h.update(b)
 return h.hexdigest()
def snapshot():
 out={}
 for p in sorted((root/"src").rglob("*")):
  if any(t in {"__pycache__", ".git", ".pytest_cache"} for t in p.parts):continue
  if p.is_symlink():raise ValueError("Source alias: "+str(p))
  if p.is_file():out[p.relative_to(root).as_posix()]={"bytes":p.stat().st_size,"sha256":sha(p)}
 return out
def write(name,value):(attempt/name).write_text(json.dumps(value,indent=2,allow_nan=False)+"\n")
before=snapshot();write("sources.before.json",before)
environment={k:v for k,v in os.environ.items() if k in {"AMENT_PREFIX_PATH","CMAKE_PREFIX_PATH","COLCON_PREFIX_PATH","LD_LIBRARY_PATH","PYTHONPATH","ROS_DISTRO","ROS_VERSION","MAKEFLAGS","CMAKE_BUILD_PARALLEL_LEVEL","PYTHONNOUSERSITE"}}
write("environment.json",{"platform":platform.platform(),"machine":platform.machine(),"environment":environment,"root":str(root),"jobs":int(os.environ["IMPACT_BUILD_JOBS"]),"livo_excluded":True,"old_workspace_overlays_allowed":False})
listing=subprocess.run(["colcon","--log-base",str(attempt/"list-log"),"list","--base-paths",str(root/"src"),"--names-only"],cwd=attempt,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
(attempt/"colcon-list.log").write_text(listing.stdout)
names=listing.stdout.split()
expected={"impact_fault_injection","xq_autonomy","bspline_opt","ego_planner","path_searching","plan_env","quadrotor_msgs","traj_utils","xq_fast_lio","xq_gz_assets","xq_gz_bridge","xq_livox_interfaces","xq_sim_bringup","xq_sim_interfaces"}
selected=set(names)-{"xq_fast_livo2"}
if listing.returncode or selected!=expected:raise ValueError("Unexpected colcon source package set: "+repr(names))
command=["colcon","--log-base",str(attempt/"colcon-log"),"build","--base-paths",str(root/"src"),"--packages-ignore","xq_fast_livo2","--build-base",str(attempt/"build"),"--install-base",str(attempt/"install"),"--merge-install","--executor","sequential","--event-handlers","console_direct+","--cmake-args","-DCMAKE_BUILD_TYPE=Release","-DBUILD_TESTING=OFF"]
write("command.json",command)
with (attempt/"build.log").open("x") as log:
 result=subprocess.run(command,cwd=attempt,stdout=log,stderr=subprocess.STDOUT)
after=snapshot();write("sources.after.json",after)
installed=sorted(p.name for p in (attempt/"install/share/ament_index/resource_index/packages").glob("*")) if (attempt/"install").exists() else []
checks={"colcon_exit_zero":result.returncode==0,"all_14_selected_packages_installed":expected.issubset(installed),"source_file_sets_and_sha256_unchanged":before==after,"install_setup_present":(attempt/"install/setup.bash").is_file()}
write("build-result.json",{"schema_version":1,"status":"PASS" if all(checks.values()) else "FAIL","checks":checks,"exit_code":result.returncode,"selected_packages":sorted(expected),"installed_packages":installed,"root":str(root),"attempt":str(attempt),"install":str(attempt/"install"),"scope":"Fresh bundled canonical 14 ROS packages; LIVO excluded for separate Sophus/vikit build. Build success is not runtime or flight validation."})
print(json.dumps({"status":"PASS" if all(checks.values()) else "FAIL","attempt":str(attempt),"install":str(attempt/"install"),"exit_code":result.returncode}),flush=True)
sys.exit(0 if all(checks.values()) else 1)
PY

