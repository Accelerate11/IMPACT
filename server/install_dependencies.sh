#!/usr/bin/env bash
# Ubuntu 22.04 / amd64 only. Default is a read-only dependency check.
set -euo pipefail
mode=--check
profile=full
usage() {
  printf '%s\n' 'Usage: bash server/install_dependencies.sh [--check|--install] [--profile offline|full]'
}
while (($#)); do
  case "$1" in
    --check|--install) mode="$1"; shift;;
    --profile)
      [[ $# -ge 2 ]] || { usage >&2; exit 2; }
      profile="$2"; shift 2;;
    --help|-h) usage; exit 0;;
    *) usage >&2; exit 2;;
  esac
done
[[ "$profile" == offline || "$profile" == full ]] || { usage >&2; exit 2; }
source /etc/os-release
if [[ "${ID:-}" != ubuntu || "${VERSION_ID:-}" != 22.04 || "$(uname -m)" != x86_64 ]]; then
  printf '%s\n' 'Requires Ubuntu 22.04 x86_64, system Python 3.10.' >&2
  exit 2
fi
if [[ "$mode" == --check ]]; then
  IMPACT_DEPENDENCY_PROFILE="$profile" /usr/bin/python3 - <<'PY'
import importlib.util,json,os,shutil,subprocess,sys
from pathlib import Path
profile=os.environ['IMPACT_DEPENDENCY_PROFILE']
checks={'python310':sys.version_info[:2]==(3,10),'Eigen3':Path('/usr/include/eigen3/Eigen/Core').is_file(),'g++':shutil.which('g++') is not None}
checks.update({name:importlib.util.find_spec(name) is not None for name in ('numpy','matplotlib')})
if profile=='full':
    checks['ros_humble']=Path('/opt/ros/humble/setup.bash').is_file()
    checks.update({name:shutil.which(name) is not None for name in ('cmake','git','colcon','gz','ip')})
    checks.update({name:importlib.util.find_spec(name) is not None for name in ('cv2','pytest','yaml')})
    checks['fmt']=Path('/usr/include/fmt/format.h').is_file()
    checks['RapidJSON']=Path('/usr/include/rapidjson/document.h').is_file()
    checks['gstreamer_devel']=shutil.which('pkg-config') is not None and subprocess.run(['pkg-config','--exists','gstreamer-1.0','gstreamer-app-1.0'],check=False).returncode==0
print(json.dumps({'status':'PASS' if all(checks.values()) else 'MISSING_DEPENDENCIES','profile':profile,'checks':checks,'note':'Presence check only. offline needs no ROS/Gazebo/resources; full requires separate build and runtime checks.'},indent=2))
sys.exit(0 if all(checks.values()) else 1)
PY
  exit
fi
if [[ "$EUID" -ne 0 ]]; then
  exec sudo -- bash "$0" --install --profile "$profile"
fi
export DEBIAN_FRONTEND=noninteractive
# Bound stalled network/lock waits; these are inactivity/retry settings, not
# a promise about the total package installation duration.
task_apt_options=(-o Acquire::Retries=2 -o Acquire::http::Timeout=30 -o Acquire::https::Timeout=30 -o Dpkg::Lock::Timeout=120)
printf 'Installing dependency profile %s (APT network timeout 30s; retries 2).\n' "$profile"
apt-get "${task_apt_options[@]}" update
if [[ "$profile" == offline ]]; then
  apt-get "${task_apt_options[@]}" install -y --no-install-recommends g++ libeigen3-dev python3 python3-numpy python3-matplotlib
  printf '%s\n' 'Offline study dependencies installed. Run --check --profile offline.'
  exit
fi
apt-get "${task_apt_options[@]}" install -y --no-install-recommends python3 ca-certificates curl gnupg lsb-release software-properties-common locales
add-apt-repository -y universe
locale-gen en_US.UTF-8
export LANG=en_US.UTF-8
# Official ROS apt-source package; the exact downloaded version is printed.
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  task_tmp="$(mktemp -d /tmp/impact-apt.XXXXXX)"
  trap 'rm -f -- "$task_tmp/ros-apt-release.json" "$task_tmp/ros2-apt-source.deb"; rmdir -- "$task_tmp" 2>/dev/null || true' EXIT
  curl --fail --location --retry 3 https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest -o "$task_tmp/ros-apt-release.json"
  task_ros_apt_version="$(/usr/bin/python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["tag_name"])' "$task_tmp/ros-apt-release.json")"
  [[ "$task_ros_apt_version" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]] || { printf '%s\n' 'Unexpected official ros-apt-source release name'; exit 2; }
  printf 'Official ros-apt-source release: %s\n' "$task_ros_apt_version"
  curl --fail --location --retry 3 "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${task_ros_apt_version}/ros2-apt-source_${task_ros_apt_version}.jammy_all.deb" -o "$task_tmp/ros2-apt-source.deb"
  dpkg -i "$task_tmp/ros2-apt-source.deb"
fi
if [[ ! -f /etc/apt/sources.list.d/gazebo-stable.list ]]; then
  curl --fail --location --retry 3 https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
  printf '%s\n' 'deb [arch=amd64 signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main' > /etc/apt/sources.list.d/gazebo-stable.list
fi
apt-get "${task_apt_options[@]}" update
apt-get "${task_apt_options[@]}" install -y --no-install-recommends \
  build-essential cmake git pkg-config gawk ccache ninja-build iproute2 \
  python3-dev python3-venv python3-pip python3-colcon-common-extensions python3-rosdep \
  python3-numpy python3-scipy python3-matplotlib python3-opencv python3-yaml python3-pytest \
  python3-pytest-cov python3-empy python3-setuptools python3-lxml python3-pexpect \
  libeigen3-dev libpcl-dev libopencv-dev libyaml-cpp-dev libboost-all-dev libfmt-dev \
  rapidjson-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  libgeographiclib-dev geographiclib-tools libgtest-dev \
  libegl1-mesa libegl1-mesa-dev libgl1-mesa-dri mesa-utils \
  ros-humble-ros-base ros-humble-ament-cmake-gtest ros-humble-ament-cmake-pytest \
  ros-humble-rmw-fastrtps-cpp ros-humble-rosbag2-storage-default-plugins \
  ros-humble-cv-bridge ros-humble-image-transport ros-humble-pcl-conversions \
  ros-humble-tf2-ros ros-humble-tf2-geometry-msgs ros-humble-message-filters ros-humble-geographic-msgs \
  ros-humble-mavros ros-humble-mavros-extras ros-humble-mavros-msgs \
  gz-harmonic libgz-sim8-dev libgz-transport13-dev libgz-msgs10-dev \
  libgz-rendering8-dev libgz-sensors8-dev libsdformat14-dev
# MAVROS requires a geoid grid; install only the grid actually used.
if [[ ! -f /usr/share/GeographicLib/geoids/egm96-5.pgm ]]; then
  geographiclib-get-geoids egm96-5
fi
printf '%s\n' 'Dependencies installed. Re-run --check; do not apt-upgrade while a bound experiment is running.'
