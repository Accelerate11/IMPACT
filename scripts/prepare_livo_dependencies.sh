#!/usr/bin/env bash
# Rebuild pinned ROS2-port dependencies in an isolated, user-owned prefix.
set -Eeuo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repository_root="$(cd -- "${script_dir}/.." && pwd)"
workspace_root=/home/accelerate/impact_research_ws
jobs=2
while (($#)); do
  case "$1" in
    --workspace-root) workspace_root="$2"; shift 2 ;;
    --jobs) jobs="$2"; shift 2 ;;
    *) echo "Usage: $0 [--workspace-root ROOT] [--jobs 1|2]" >&2; exit 2 ;;
  esac
done
[[ "$jobs" == 1 || "$jobs" == 2 ]] || { echo 'jobs must be 1 or 2' >&2; exit 2; }
workspace_root="$(realpath -e -- "$workspace_root")"
[[ "$workspace_root" == /home/accelerate/impact_research_ws ]] || {
  echo 'Refusing a workspace outside the explicitly authorized isolated research root' >&2; exit 2;
}
prefix="${workspace_root}/livo_deps"
[[ ! -L "$prefix" ]] || { echo 'Refusing a symlink install prefix' >&2; exit 2; }
if [[ -d "$prefix" && ! -f "$prefix/.impact-livo-dependencies-owned" ]]; then
  echo 'Existing prefix has no ownership marker; refusing to overwrite' >&2; exit 2
fi
utc="$(date -u +%Y%m%dT%H%M%SZ)"
attempt="${workspace_root}/third_party/deps-${utc}-$$"
log_dir="${repository_root}/runs/research_r2b2_deps_${utc}_$$"
mkdir -p -- "$attempt" "$log_dir" "$prefix"
touch "$prefix/.impact-livo-dependencies-owned"
exec > >(tee "${log_dir}/build.log") 2>&1
echo "Dependency attempt: $attempt"
echo "Install prefix: $prefix"
echo "Logs: $log_dir"
export IMPACT_DEP_ATTEMPT="$attempt" IMPACT_DEP_PREFIX="$prefix" IMPACT_DEP_LOG_DIR="$log_dir"
export IMPACT_DEP_REPOSITORY="$repository_root"
export IMPACT_DEP_SCRIPT_START_SHA256="$(sha256sum "${script_dir}/prepare_livo_dependencies.sh" | cut -d ' ' -f 1)"
python3 - <<'PY'
import json,os
from pathlib import Path
prefix=Path(os.environ['IMPACT_DEP_PREFIX'])
(prefix/'dependency-manifest.json').write_text(json.dumps({
    'schema_version':1,'status':'BUILDING','attempt':os.environ['IMPACT_DEP_ATTEMPT'],
    'log_dir':os.environ['IMPACT_DEP_LOG_DIR'],
    'script_sha256':os.environ['IMPACT_DEP_SCRIPT_START_SHA256']},indent=2)+'\n')
PY
finish() {
  local code=$?
  trap - EXIT
  export IMPACT_DEP_EXIT_CODE="$code"
  python3 - <<'PY'
import hashlib,json,os,subprocess
from pathlib import Path
attempt=Path(os.environ['IMPACT_DEP_ATTEMPT'])
prefix=Path(os.environ['IMPACT_DEP_PREFIX'])
repo=Path(os.environ['IMPACT_DEP_REPOSITORY'])
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
sources={}
for name in ('Sophus','rpg_vikit'):
    path=attempt/name
    if (path/'.git').is_dir():
        sources[name]={'head':subprocess.run(['git','-C',str(path),'rev-parse','HEAD'],capture_output=True,text=True).stdout.strip(),
                       'origin':subprocess.run(['git','-C',str(path),'remote','get-url','origin'],capture_output=True,text=True).stdout.strip(),
                       'status':subprocess.run(['git','-C',str(path),'status','--porcelain'],capture_output=True,text=True).stdout.strip()}
patches={str(p.relative_to(attempt/'wrapper')):digest(p) for p in sorted((attempt/'wrapper').rglob('*')) if p.is_file()}
installed={}
for path in sorted(prefix.rglob('*')):
    if path.name=='dependency-manifest.json' or not path.is_file():continue
    relative=str(path.relative_to(prefix))
    installed[relative]={'bytes':path.stat().st_size,'sha256':digest(path)}
code=int(os.environ['IMPACT_DEP_EXIT_CODE'])
result={'schema_version':1,'status':'PASS' if code==0 else 'FAIL','exit_code':code,'attempt':str(attempt),'install_prefix':str(prefix),
        'sources':sources,'wrapper_and_patch_sha256':patches,'script_sha256':os.environ['IMPACT_DEP_SCRIPT_START_SHA256'],
        'installed_files':installed,
        'install_scope':'isolated user prefix only; no sudo install; no original workspace environment',
        'targets':['Sophus::Sophus','vikit_common::vikit_common','vikit_ros::vikit_ros']}
text=json.dumps(result,indent=2)+'\n'
(Path(os.environ['IMPACT_DEP_LOG_DIR'])/'result.json').write_text(text)
(prefix/'dependency-manifest.json').write_text(text)
print(json.dumps({'status':result['status'],'result':str(Path(os.environ['IMPACT_DEP_LOG_DIR'])/'result.json')}))
PY
  exit "$code"
}
trap finish EXIT

# Do not inherit overlays or library paths from either project workspace.
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
set +u
source /opt/ros/humble/setup.bash
set -u
export PYTHONDONTWRITEBYTECODE=1
export CMAKE_PREFIX_PATH="${prefix}:${CMAKE_PREFIX_PATH:-}"

clone_pin() {
  local url="$1" name="$2" commit="$3"
  local cache used_cache=false
  # Reuse only Git objects with this exact upstream origin and pinned commit.
  # --no-hardlinks keeps the new clone independent of the earlier attempt.
  for cache in "${workspace_root}"/third_party/deps-*/"${name}"; do
    [[ "$cache" != "${attempt}/${name}" && -d "$cache/.git" ]] || continue
    [[ "$(git -C "$cache" remote get-url origin 2>/dev/null)" == "$url" ]] || continue
    git -C "$cache" cat-file -e "${commit}^{commit}" 2>/dev/null || continue
    echo "Cloning verified local Git objects: $cache ($commit)"
    git clone --no-checkout --no-hardlinks "$cache" "${attempt}/${name}"
    git -C "${attempt}/${name}" remote set-url origin "$url"
    used_cache=true
    break
  done
  if [[ "$used_cache" == false ]]; then
    git -c http.lowSpeedLimit=1 -c http.lowSpeedTime=30 clone --no-checkout "$url" "${attempt}/${name}"
  fi
  git -C "${attempt}/${name}" checkout --detach "$commit"
  [[ "$(git -C "${attempt}/${name}" rev-parse HEAD)" == "$commit" ]]
}
clone_pin https://github.com/strasdat/Sophus.git Sophus de0f8d3d92bf776271e16de56d1803940ebccab9
clone_pin https://github.com/Robotic-Developer-Road/rpg_vikit.git rpg_vikit 4b7abc838f5d2ca9137f70f122eaaeff9eaf0f50

common_cmake=(-DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$prefix"
  -DCMAKE_EXPORT_NO_PACKAGE_REGISTRY=ON -DCMAKE_EXPORT_PACKAGE_REGISTRY=OFF
  -DCMAKE_FIND_USE_PACKAGE_REGISTRY=OFF -DCMAKE_FIND_USE_SYSTEM_PACKAGE_REGISTRY=OFF)
cmake -S "${attempt}/Sophus" -B "${attempt}/build-Sophus" "${common_cmake[@]}" \
  -DSOPHUS_INSTALL=ON -DBUILD_SOPHUS_TESTS=OFF -DBUILD_SOPHUS_EXAMPLES=OFF
cmake --build "${attempt}/build-Sophus" --parallel "$jobs"
cmake --install "${attempt}/build-Sophus"
install -D -m 644 "${attempt}/Sophus/LICENSE.txt" \
  "${prefix}/share/impact_livo_dependencies/licenses/Sophus-LICENSE.txt"
install -D -m 644 "${attempt}/rpg_vikit/README.md" \
  "${prefix}/share/impact_livo_dependencies/licenses/rpg_vikit-README.upstream.md"

# Wrapper and explicit patches fix build/export and bounded parameter discovery;
# numerical camera/estimator source stays pinned.
cp -a -- "${repository_root}/config/livo_dependency_patches" "${attempt}/wrapper"
for patch in "${attempt}/wrapper"/*.patch; do
  [[ -f "$patch" ]] || continue
  git -C "${attempt}/rpg_vikit" apply --check "$patch"
  git -C "${attempt}/rpg_vikit" apply "$patch"
done
cmake -S "${attempt}/wrapper" -B "${attempt}/build-vikit" "${common_cmake[@]}" \
  -DVIKIT_SOURCE_DIR="${attempt}/rpg_vikit" -DSophus_DIR="${prefix}/share/sophus/cmake"
cmake --build "${attempt}/build-vikit" --parallel "$jobs"
cmake --install "${attempt}/build-vikit"

# Independent consumer verifies exports and actually loads a calibrated Pinhole
# through a deliberately delayed ROS2 parameter server on an isolated domain.
# It also checks missing-service timeout and wrong-type failure; no hardware.
mkdir -p "${attempt}/consumer"
cat > "${attempt}/consumer/CMakeLists.txt" <<'CMAKE'
cmake_minimum_required(VERSION 3.16)
project(impact_dependency_probe LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
find_package(Sophus 1.22.10 EXACT REQUIRED CONFIG)
find_package(vikit_common REQUIRED CONFIG)
find_package(vikit_ros REQUIRED CONFIG)
if(NOT EXISTS "${vikit_common_INCLUDE_DIRS}/vikit/pinhole_camera.h")
  message(FATAL_ERROR "Invalid installed vikit_common include directory")
endif()
if(NOT EXISTS "${vikit_ros_INCLUDE_DIRS}/vikit/camera_loader.h")
  message(FATAL_ERROR "Invalid installed vikit_ros include directory")
endif()
add_executable(dependency_probe probe.cpp)
target_link_libraries(dependency_probe PRIVATE Sophus::Sophus vikit_common::vikit_common vikit_ros::vikit_ros)
CMAKE
cp "${attempt}/wrapper/camera_loader_probe.cpp" "${attempt}/consumer/probe.cpp"
cmake -S "${attempt}/consumer" -B "${attempt}/build-consumer" "${common_cmake[@]}" \
  -DSophus_DIR="${prefix}/share/sophus/cmake" \
  -Dvikit_common_DIR="${prefix}/share/vikit_common/cmake" \
  -Dvikit_ros_DIR="${prefix}/share/vikit_ros/cmake"
cmake --build "${attempt}/build-consumer" --parallel "$jobs"
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=$((210 + $$ % 20))
"${attempt}/build-consumer/dependency_probe" delayed
"${attempt}/build-consumer/dependency_probe" missing
"${attempt}/build-consumer/dependency_probe" wrong-type
diff -qr -- "${repository_root}/config/livo_dependency_patches" "${attempt}/wrapper"
echo 'Installed dependency consumer probe PASS'
