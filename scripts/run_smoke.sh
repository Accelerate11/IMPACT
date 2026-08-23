#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
workspace_root="$(cd -- "${script_dir}/.." && pwd -P)"
duration=""
with_faults=false
headless=true
seed=20260820
requested_run_dir=""
require_algorithm_pass=false

usage() {
  cat <<'EOF'
Usage: run_smoke.sh [options]
  --duration SECONDS   Wall-clock run duration (default: 30, or 120 with faults)
  --with-faults        Enable the deterministic fault schedule
  --gui                Launch the Gazebo GUI instead of headless mode
  --seed INTEGER       Reproducibility seed (default: 20260820)
  --run-dir PATH       Result directory; must be below this workspace's runs/
  --require-algorithm-pass
                       Exit non-zero for any gated SIMULATED_FAIL or
                       INSUFFICIENT_EVIDENCE result (integration/isolation
                       status is still reported independently)
EOF
}

while (($#)); do
  case "$1" in
    --duration)
      [[ $# -ge 2 ]] || { usage >&2; exit 2; }
      duration="$2"
      shift 2
      ;;
    --with-faults)
      with_faults=true
      shift
      ;;
    --gui)
      headless=false
      shift
      ;;
    --seed)
      [[ $# -ge 2 ]] || { usage >&2; exit 2; }
      seed="$2"
      shift 2
      ;;
    --run-dir)
      [[ $# -ge 2 ]] || { usage >&2; exit 2; }
      requested_run_dir="$2"
      shift 2
      ;;
    --require-algorithm-pass)
      require_algorithm_pass=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ -z "${duration}" ]]; then
  if [[ "${with_faults}" == true ]]; then duration=120; else duration=30; fi
fi
[[ "${duration}" =~ ^[1-9][0-9]*$ ]] || { echo "Duration must be a positive integer." >&2; exit 2; }
[[ "${seed}" =~ ^[0-9]+$ ]] || { echo "Seed must be a non-negative integer." >&2; exit 2; }

if [[ ! -f "${workspace_root}/xq_install/setup.bash" ]]; then
  echo "Missing ${workspace_root}/xq_install/setup.bash; run scripts/build_isolated.sh first." >&2
  exit 2
fi
build_manifest="${workspace_root}/xq_install/.xq_build_manifest.json"
if [[ ! -f "${build_manifest}" ]]; then
  echo "Missing ${build_manifest}; rebuild with scripts/build_isolated.sh before running." >&2
  exit 2
fi
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble was not found at /opt/ros/humble." >&2
  exit 2
fi

# Recompute both trees before sourcing the overlay.  Merely finding
# xq_install/setup.bash is not evidence that the install matches current
# sources, nor that installed files have remained unchanged.
python3 - "${workspace_root}" "${build_manifest}" <<'PY'
from __future__ import annotations

import hashlib
import json
import os
import sys
from pathlib import Path


workspace = Path(sys.argv[1]).resolve()
manifest_path = Path(sys.argv[2]).resolve()


def file_digest(path: Path) -> str:
    digest = hashlib.sha256()
    if path.is_symlink():
        digest.update(b"SYMLINK\0")
        digest.update(os.readlink(path).encode("utf-8", errors="surrogateescape"))
    else:
        with path.open("rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
    return digest.hexdigest()


def collect(root: Path, excluded: set[Path] | None = None) -> list[dict[str, object]]:
    excluded = excluded or set()
    ignored_directories = {"__pycache__", ".pytest_cache"}
    records: list[dict[str, object]] = []
    for path in sorted(root.rglob("*"), key=lambda item: item.as_posix()):
        if (
            path in excluded
            or any(part in ignored_directories for part in path.parts)
            or path.suffix in {".pyc", ".pyo"}
        ):
            continue
        if not (path.is_file() or path.is_symlink()):
            continue
        records.append(
            {
                "path": path.relative_to(root).as_posix(),
                "sha256": file_digest(path),
                "size_bytes": int(path.lstat().st_size),
                "kind": "symlink" if path.is_symlink() else "file",
            }
        )
    return records


def tree_digest(records: list[dict[str, object]]) -> str:
    canonical = json.dumps(records, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


try:
    expected = json.loads(manifest_path.read_text(encoding="utf-8"))
except (OSError, json.JSONDecodeError) as exc:
    raise SystemExit(f"Invalid build manifest {manifest_path}: {exc}")
if expected.get("contract") != "xq_isolated_build_manifest":
    raise SystemExit("Build manifest has an unknown contract; rebuild the isolated workspace.")

source_files = collect(workspace / "src")
install_files = collect(workspace / "xq_install", {manifest_path})
actual_source = tree_digest(source_files)
actual_install = tree_digest(install_files)
problems = []
if actual_source != expected.get("source_tree_sha256"):
    problems.append(
        f"source tree is stale (built={expected.get('source_tree_sha256')}, current={actual_source})"
    )
if actual_install != expected.get("install_tree_sha256"):
    problems.append(
        f"install tree changed (built={expected.get('install_tree_sha256')}, current={actual_install})"
    )
if source_files != expected.get("source_files"):
    problems.append("source file manifest differs")
if install_files != expected.get("install_files"):
    problems.append("installed file manifest differs")
if problems:
    raise SystemExit(
        "Refusing to run an out-of-date or modified xq_install:\n  - "
        + "\n  - ".join(problems)
        + "\nRun scripts/build_isolated.sh."
    )
print(f"Build freshness PASS: source={actual_source}, install={actual_install}")
PY

timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
if [[ -n "${requested_run_dir}" ]]; then
  run_dir="$(realpath -m -- "${requested_run_dir}")"
else
  run_dir="${workspace_root}/runs/smoke_${timestamp}_$$"
fi
case "${run_dir}" in
  "${workspace_root}"/runs/*) ;;
  *)
    echo "Result directory must stay below ${workspace_root}/runs." >&2
    exit 2
    ;;
esac
mkdir -p -- "${run_dir}"
mkdir -p -- "${run_dir}/ros_logs"

# Clear inherited overlays and resource paths before sourcing only the base ROS
# installation and this isolated workspace.
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
unset GZ_SIM_RESOURCE_PATH IGN_GAZEBO_RESOURCE_PATH SDF_PATH
unset RMW_IMPLEMENTATION FASTRTPS_DEFAULT_PROFILES_FILE CYCLONEDDS_URI
set +u
source /opt/ros/humble/setup.bash
source "${workspace_root}/xq_install/setup.bash"
set -u
export ROS_LOG_DIR="${run_dir}/ros_logs"

nonce="$(date +%s%N)_$$"

# ROS 2 recommends domain IDs 0--101 on Linux.  Keep project runs in the
# narrower 32--101 band and inspect existing process environments plus
# explicit ros2-daemon arguments before selecting a candidate.  This scan is
# read-only and never signals another project's process.
select_ros_domain_id() {
  python3 - "${nonce}" <<'PY'
from __future__ import annotations

import hashlib
import re
import sys
from pathlib import Path


used: set[int] = set()
for process_dir in Path("/proc").glob("[0-9]*"):
    try:
        environment = (process_dir / "environ").read_bytes().split(b"\0")
    except (OSError, PermissionError):
        environment = []
    for item in environment:
        if not item.startswith(b"ROS_DOMAIN_ID="):
            continue
        try:
            value = int(item.split(b"=", 1)[1])
        except ValueError:
            continue
        if 32 <= value <= 101:
            used.add(value)

    try:
        arguments = [
            item.decode("utf-8", errors="replace")
            for item in (process_dir / "cmdline").read_bytes().split(b"\0")
            if item
        ]
    except (OSError, PermissionError):
        arguments = []
    for index, argument in enumerate(arguments):
        value_text = None
        if argument == "--ros-domain-id" and index + 1 < len(arguments):
            value_text = arguments[index + 1]
        else:
            match = re.fullmatch(r"--ros-domain-id=(\d+)", argument)
            if match:
                value_text = match.group(1)
        if value_text is not None:
            try:
                value = int(value_text)
            except ValueError:
                continue
            if 32 <= value <= 101:
                used.add(value)

candidates = list(range(32, 102))
offset = int(hashlib.sha256(sys.argv[1].encode("utf-8")).hexdigest(), 16) % len(candidates)
ordered = candidates[offset:] + candidates[:offset]
for candidate in ordered:
    if candidate not in used:
        print(candidate)
        break
else:
    raise SystemExit("No unused ROS_DOMAIN_ID is available in the safe 32--101 range.")
PY
}

if ! selected_domain="$(select_ros_domain_id)"; then
  echo "Refusing to launch without an unused ROS_DOMAIN_ID in 32--101." >&2
  exit 2
fi
export ROS_DOMAIN_ID="${selected_domain}"
export GZ_PARTITION="xq_${USER:-wsl}_${timestamp}_$$_${nonce}"
export ROS_LOCALHOST_ONLY=1

# Gazebo Harmonic's headless GPU lidar uses EGL with Mesa llvmpipe in WSL.
if [[ "${headless}" == true ]]; then
  export LIBGL_ALWAYS_SOFTWARE=1
  export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe
  export GALLIUM_DRIVER=llvmpipe
  export EGL_PLATFORM=surfaceless
  export QT_QPA_PLATFORM=offscreen
fi

installed_world_file="${workspace_root}/xq_install/xq_gz_assets/share/xq_gz_assets/worlds/xq_indoor_office.sdf"
installed_model_file="${workspace_root}/xq_install/xq_gz_assets/share/xq_gz_assets/models/xq_iris_mid360/model.sdf"
installed_model_config="${workspace_root}/xq_install/xq_gz_assets/share/xq_gz_assets/models/xq_iris_mid360/model.config"
installed_bridge_config="${workspace_root}/xq_install/xq_gz_bridge/share/xq_gz_bridge/config/bridge.yaml"
installed_stack_config="${workspace_root}/xq_install/xq_autonomy/share/xq_autonomy/config/stack.yaml"
installed_fault_schedule="${workspace_root}/xq_install/xq_autonomy/share/xq_autonomy/config/fault_schedule.json"
installed_launch_file="${workspace_root}/xq_install/xq_sim_bringup/share/xq_sim_bringup/launch/xq_sil.launch.py"

for required_file in \
  "${installed_world_file}" \
  "${installed_model_file}" \
  "${installed_model_config}" \
  "${installed_bridge_config}" \
  "${installed_stack_config}" \
  "${installed_fault_schedule}" \
  "${installed_launch_file}"; do
  [[ -f "${required_file}" ]] || {
    echo "Installed XQ runtime input is missing: ${required_file}" >&2
    exit 2
  }
done

# Freeze every material runtime input before launch.  Gazebo and ROS nodes are
# passed the snapshots for world/stack/bridge/fault inputs, so the recorded
# bytes are the bytes actually used by this run.  Model and launch snapshots
# prove the installed dependencies that resolved the run.
configuration_dir="${run_dir}/configuration"
configuration_manifest="${run_dir}/configuration-manifest.json"
python3 - \
  "${workspace_root}" \
  "${configuration_dir}" \
  "${configuration_manifest}" \
  "${build_manifest}" \
  "${with_faults}" \
  "${seed}" \
  "${duration}" \
  "${ROS_DOMAIN_ID}" \
  "${GZ_PARTITION}" \
  "${headless}" \
  "${require_algorithm_pass}" <<'PY'
from __future__ import annotations

import hashlib
import json
import shutil
import sys
from datetime import datetime, timezone
from pathlib import Path


workspace = Path(sys.argv[1]).resolve()
snapshot_dir = Path(sys.argv[2]).resolve()
manifest_path = Path(sys.argv[3]).resolve()
build_manifest = Path(sys.argv[4]).resolve()
faults_enabled = sys.argv[5].lower() == "true"
seed = int(sys.argv[6])
duration_s = int(sys.argv[7])
ros_domain_id = int(sys.argv[8])
gz_partition = sys.argv[9]
headless = sys.argv[10].lower() == "true"
require_algorithm_pass = sys.argv[11].lower() == "true"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


install = workspace / "xq_install"
specifications = [
    (
        "world",
        install / "xq_gz_assets/share/xq_gz_assets/worlds/xq_indoor_office.sdf",
        "world.sdf",
        True,
        "world_file",
    ),
    (
        "vehicle_sensor_model",
        install / "xq_gz_assets/share/xq_gz_assets/models/xq_iris_mid360/model.sdf",
        "model.sdf",
        True,
        "Gazebo model://xq_iris_mid360 resolution",
    ),
    (
        "vehicle_sensor_model_metadata",
        install / "xq_gz_assets/share/xq_gz_assets/models/xq_iris_mid360/model.config",
        "model.config",
        True,
        "Gazebo model metadata",
    ),
    (
        "bridge_parameters",
        install / "xq_gz_bridge/share/xq_gz_bridge/config/bridge.yaml",
        "bridge.yaml",
        True,
        "bridge_config",
    ),
    (
        "stack_parameters",
        install / "xq_autonomy/share/xq_autonomy/config/stack.yaml",
        "stack.yaml",
        True,
        "stack_config",
    ),
    (
        "fault_schedule",
        install / "xq_autonomy/share/xq_autonomy/config/fault_schedule.json",
        "fault_schedule.json",
        faults_enabled,
        "fault_schedule",
    ),
    (
        "launch_description",
        install / "xq_sim_bringup/share/xq_sim_bringup/launch/xq_sil.launch.py",
        "xq_sil.launch.py",
        True,
        "ros2 launch resolved installed file",
    ),
    (
        "isolated_build_manifest",
        build_manifest,
        "xq_build_manifest.json",
        True,
        "pre-launch freshness contract",
    ),
]

snapshot_dir.mkdir(parents=True, exist_ok=False)
records = []
for logical_name, source, snapshot_name, active, use in specifications:
    snapshot = snapshot_dir / snapshot_name
    shutil.copy2(source, snapshot)
    source_hash = sha256(source)
    snapshot_hash = sha256(snapshot)
    if source_hash != snapshot_hash:
        raise SystemExit(f"Configuration snapshot hash mismatch: {source}")
    records.append(
        {
            "logical_name": logical_name,
            "active_for_this_run": bool(active),
            "runtime_use": use,
            "installed_path": str(source),
            "snapshot_path": str(snapshot.relative_to(manifest_path.parent)),
            "sha256": snapshot_hash,
            "size_bytes": snapshot.stat().st_size,
        }
    )

build_data = json.loads(build_manifest.read_text(encoding="utf-8"))
manifest = {
    "schema_version": 1,
    "contract": "xq_runtime_configuration_evidence",
    "generated_at_utc": datetime.now(timezone.utc).isoformat(),
    "fault_injection_enabled": faults_enabled,
    "seed": seed,
    "requested_wall_duration_s": duration_s,
    "ros_domain_id": ros_domain_id,
    "gz_partition": gz_partition,
    "ros_log_dir": str(manifest_path.parent / "ros_logs"),
    "launch_arguments": {
        "headless": headless,
        "inject_faults": faults_enabled,
        "start_gazebo": True,
        "start_bridge": True,
        "start_network_relay": True,
        "start_stack": True,
        "start_metrics": True,
    },
    "require_algorithm_pass": require_algorithm_pass,
    "source_tree_sha256": build_data.get("source_tree_sha256"),
    "install_tree_sha256": build_data.get("install_tree_sha256"),
    "files": records,
}
manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
PY

world_file="${configuration_dir}/world.sdf"
bridge_config="${configuration_dir}/bridge.yaml"
stack_config="${configuration_dir}/stack.yaml"
fault_schedule="${configuration_dir}/fault_schedule.json"
world_sha256="$(sha256sum -- "${world_file}" | awk '{print $1}')"
spec_sha256="UNSET"
if [[ -f "${workspace_root}/SOURCE_SPEC_SHA256" ]]; then
  read -r spec_sha256 _ < "${workspace_root}/SOURCE_SPEC_SHA256" || true
  if [[ ! "${spec_sha256}" =~ ^[0-9a-fA-F]{64}$ ]]; then
    echo "Ignoring malformed SOURCE_SPEC_SHA256." >&2
    spec_sha256="UNSET"
  fi
fi

before_audit="${run_dir}/external-assets.before.sha256"
after_audit="${run_dir}/external-assets.after.sha256"
audit_report="${run_dir}/isolation-audit.txt"
bash "${script_dir}/audit_external_assets.sh" snapshot "${before_audit}" >/dev/null

launch_pid=""
launch_pgid=""
audit_finished=false

stop_launch_group() {
  local signal round
  [[ -n "${launch_pid}" ]] || return 0
  if kill -0 "${launch_pid}" 2>/dev/null; then
    # Let ros2 launch perform one orderly shutdown first.  Only if its own PID
    # remains alive do we escalate to the verified private process group.
    kill -INT "${launch_pid}" 2>/dev/null || true
    for round in 1 2 3 4 5 6 7 8; do
      kill -0 "${launch_pid}" 2>/dev/null || break
      sleep 1
    done
  fi
  if kill -0 "${launch_pid}" 2>/dev/null; then
    if [[ -z "${launch_pgid}" ]] || [[ "${launch_pgid}" != "${launch_pid}" ]]; then
      echo "Refusing escalation because the recorded PGID is not launch PID." >&2
    else
      kill -TERM -- "-${launch_pgid}" 2>/dev/null || true
      for round in 1 2 3 4 5; do
        kill -0 "${launch_pid}" 2>/dev/null || break
        sleep 1
      done
      if kill -0 "${launch_pid}" 2>/dev/null; then
        kill -KILL -- "-${launch_pgid}" 2>/dev/null || true
      fi
    fi
  fi
  wait "${launch_pid}" 2>/dev/null || true

  # If launch exited early, reap any surviving child in its verified private
  # group.  The negative PID never targets processes outside this run.
  if [[ -n "${launch_pgid}" ]] && [[ "${launch_pgid}" == "${launch_pid}" ]] &&
     kill -0 -- "-${launch_pgid}" 2>/dev/null; then
    for signal in TERM KILL; do
      kill -"${signal}" -- "-${launch_pgid}" 2>/dev/null || true
      for round in 1 2 3 4 5; do
        kill -0 -- "-${launch_pgid}" 2>/dev/null || break 2
        sleep 1
      done
    done
  fi
  launch_pid=""
}

finish_audit() {
  [[ "${audit_finished}" == false ]] || return 0
  bash "${script_dir}/audit_external_assets.sh" snapshot "${after_audit}" >/dev/null
  if bash "${script_dir}/audit_external_assets.sh" compare "${before_audit}" "${after_audit}" >"${audit_report}" 2>&1; then
    audit_finished=true
    return 0
  fi
  cat -- "${audit_report}" >&2
  audit_finished=true
  return 3
}

cleanup() {
  local original_status=$?
  trap - EXIT INT TERM
  set +e
  stop_launch_group
  finish_audit
  local audit_status=$?
  if ((audit_status != 0)); then exit "${audit_status}"; fi
  exit "${original_status}"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

echo "Starting isolated XQ SIL for ${duration}s"
echo "  ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "  GZ_PARTITION=${GZ_PARTITION}"
echo "  results=${run_dir}"

setsid ros2 launch xq_sim_bringup xq_sil.launch.py \
  headless:="${headless}" \
  inject_faults:="${with_faults}" \
  world_file:="${world_file}" \
  bridge_config:="${bridge_config}" \
  stack_config:="${stack_config}" \
  fault_schedule:="${fault_schedule}" \
  run_dir:="${run_dir}" \
  seed:="${seed}" \
  ros_domain_id:="${ROS_DOMAIN_ID}" \
  gz_partition:="${GZ_PARTITION}" \
  spec_sha256:="${spec_sha256}" \
  world_sha256:="${world_sha256}" \
  configuration_manifest_path:="${configuration_manifest}" \
  >"${run_dir}/launch.log" 2>&1 &
launch_pid=$!
sleep 1
launch_pgid="$(ps -o pgid= -p "${launch_pid}" | tr -d '[:space:]')"
if [[ "${launch_pgid}" != "${launch_pid}" ]]; then
  echo "Could not establish an isolated launch process group." >&2
  exit 3
fi

deadline=$((SECONDS + duration))
exited_early=false
while ((SECONDS < deadline)); do
  if ! kill -0 "${launch_pid}" 2>/dev/null; then
    exited_early=true
    break
  fi
  sleep 1
done

stop_launch_group
finish_audit

if [[ "${exited_early}" == true ]]; then
  echo "The launch exited before the requested duration." >&2
  tail -n 60 -- "${run_dir}/launch.log" >&2 || true
  exit 4
fi
if [[ ! -s "${run_dir}/metrics.json" ]]; then
  echo "The metrics node did not produce metrics.json." >&2
  tail -n 60 -- "${run_dir}/launch.log" >&2 || true
  exit 5
fi
if grep -Eq 'Traceback|\[ERROR\].*process has died' "${run_dir}/launch.log"; then
  echo "A launched process reported an unhandled exception or crash." >&2
  tail -n 80 -- "${run_dir}/launch.log" >&2 || true
  exit 6
fi

echo "PASS: integration processes completed and the isolation audit is unchanged."
echo "Metrics: ${run_dir}/metrics.json"
echo "Report:  ${run_dir}/report.md"
echo "Config:  ${configuration_manifest}"
validation_contract_status=0
python3 - "${run_dir}/metrics.json" "${with_faults}" <<'PY' || validation_contract_status=$?
import json
import sys

with open(sys.argv[1], encoding="utf-8") as stream:
    metrics = json.load(stream)

if metrics.get("configuration_evidence", {}).get("status") != "CAPTURED":
    print("FAIL: runtime configuration snapshots were not hash-verified.", file=sys.stderr)
    raise SystemExit(8)

with_faults = sys.argv[2].lower() == "true"
if with_faults:
    faults = metrics.get("fault_events", {})
    coverage = faults.get("schedule_coverage", {})
    responses = faults.get("responses", [])
    windows_complete = bool(responses) and all(
        bool(item.get("window", {}).get("complete")) for item in responses
    )
    failed_responses = [
        item for item in responses if item.get("status") != "SIMULATED_PASS"
    ]
    if (
        coverage.get("status") != "COMPLETE"
        or not windows_complete
        or failed_responses
    ):
        print(
            "FAIL: requested F1-F8 proxy fault acceptance did not pass: "
            f"coverage={coverage.get('status')}, "
            f"windows_complete={windows_complete}, "
            f"failed_responses={len(failed_responses)}",
            file=sys.stderr,
        )
        if coverage.get("status") != "COMPLETE":
            print(
                "  schedule coverage details: "
                f"planned={coverage.get('planned_fault_ids', [])!r} "
                f"missing={coverage.get('missing_fault_ids', [])!r} "
                f"unexpected={coverage.get('unexpected_fault_ids', [])!r}",
                file=sys.stderr,
            )
        for response in failed_responses:
            fault_id = response.get("fault", {}).get("fault_id", "UNKNOWN")
            print(
                f"  {fault_id}: {response.get('status', 'MISSING_STATUS')}",
                file=sys.stderr,
            )
            for check in response.get("expected_checks", []):
                if check.get("status") == "SIMULATED_PASS":
                    continue
                print(
                    f"    - {check.get('name')}: {check.get('status')} "
                    f"expected={check.get('expected')!r} "
                    f"observed={check.get('observed')!r}",
                    file=sys.stderr,
                )
        print(
            "These checks are Gazebo SIL/proxy evidence only; failure or pass "
            "is not a formal hardware/flight acceptance verdict.",
            file=sys.stderr,
        )
        raise SystemExit(9)
    print(
        "PASS: all configured F1-F8 response checks passed at the "
        "Gazebo SIL/proxy layer (not formal hardware/flight acceptance)."
    )
else:
    print("PASS: runtime configuration snapshots are hash-verified.")
PY
if ((validation_contract_status != 0)); then
  exit "${validation_contract_status}"
fi
algorithm_gate_status=0
python3 - "${run_dir}/metrics.json" "${require_algorithm_pass}" <<'PY' || algorithm_gate_status=$?
import json
import sys

with open(sys.argv[1], encoding="utf-8") as stream:
    metrics = json.load(stream)

require_pass = sys.argv[2].lower() == "true"
gate = metrics.get("algorithm_gate", {})
checks = gate.get("checks", [])
failed = [
    item.get("metric", "UNKNOWN")
    for item in checks
    if item.get("status") != "SIMULATED_PASS"
]
print("Algorithm metric statuses (separate from the integration PASS):")
for item in checks:
    print(f"  {item.get('metric', 'UNKNOWN')}: {item.get('status', 'MISSING')}")
print(f"  R6_dynamic_obstacle_replan: {metrics.get('replanning', {}).get('status', 'MISSING')}")
if failed:
    print("WARNING: gated simulated algorithm evidence failed or is insufficient: " + ", ".join(failed))
    if require_pass:
        print("FAIL: --require-algorithm-pass was requested.", file=sys.stderr)
        raise SystemExit(7)
elif not checks:
    print("WARNING: metrics.json has no algorithm_gate checks.")
    if require_pass:
        raise SystemExit(7)
else:
    print("PASS: all gated simulated algorithm checks passed.")
PY
tail -n 20 -- "${run_dir}/launch.log" || true
if ((algorithm_gate_status != 0)); then
  exit "${algorithm_gate_status}"
fi
