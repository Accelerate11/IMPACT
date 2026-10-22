#!/usr/bin/env python3
"""Isolated no-AP Gazebo/ROS sensor and algorithm-proxy smoke for the server bundle."""
from __future__ import annotations

import argparse
from collections import defaultdict
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import shlex
import signal
import struct
import subprocess
import sys
import time


def write(path, value):
    with path.open("x", encoding="utf-8") as handle:
        json.dump(value, handle, indent=2, allow_nan=False)
        handle.write("\n")


def digest(path):
    h = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1048576), b""):
            h.update(block)
    return h.hexdigest()


def fingerprint(root):
    result = {}
    for family in ("src", "third_party/ardupilot_gazebo/models", "third_party/ardupilot_gazebo/worlds"):
        base = root / family
        if not base.is_dir() or base.is_symlink():
            raise ValueError("Missing/untrusted input tree: " + str(base))
        for path in sorted(base.rglob("*")):
            if any(part in {"__pycache__", ".git", ".pytest_cache"} for part in path.parts):
                continue
            if path.is_symlink():
                raise ValueError("Input aliases forbidden: " + str(path))
            if path.is_file():
                result[path.relative_to(root).as_posix()] = {"bytes": path.stat().st_size, "sha256": digest(path)}
    return result


def used_domains():
    found = set()
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            values = (entry / "environ").read_bytes().split(b"\0")
            for value in values:
                if value.startswith(b"ROS_DOMAIN_ID="):
                    found.add(int(value.split(b"=", 1)[1]))
        except (OSError, ValueError):
            continue
    return found


def proc_table():
    result = {}
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            text = (entry / "stat").read_text()
            fields = text[text.rfind(")") + 2:].split()
            result[int(entry.name)] = (int(fields[1]), fields[19], fields[0])
        except (OSError, ValueError, IndexError):
            continue
    return result


def stop_owned(process):
    table = proc_table()
    owned = {process.pid}
    changed = True
    while changed:
        previous = len(owned)
        owned.update(pid for pid, (parent, _, _) in table.items() if parent in owned)
        changed = len(owned) != previous
    identities = {pid: table[pid][1] for pid in owned if pid in table}
    escalations = []
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            pass
    def alive():
        current = proc_table()
        return [pid for pid, start in identities.items()
                if pid in current and current[pid][1] == start and current[pid][2] != "Z"]
    for sig, delay in ((signal.SIGTERM, 8), (signal.SIGKILL, 3)):
        pending = alive()
        if not pending:
            break
        escalations.append({"signal": sig.name, "owned_pids": pending})
        for pid in pending:
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pass
        deadline = time.monotonic() + delay
        while alive() and time.monotonic() < deadline:
            time.sleep(0.1)
    try:
        process.wait(timeout=1)
    except subprocess.TimeoutExpired:
        pass
    return {"returncode": process.poll(), "escalations": escalations, "owned_remaining": alive(),
            "scope": "Only tracked descendants with unchanged /proc start identities; no global pkill."}


def shell_command(install, command):
    shell = 'set -e; set +u; source /opt/ros/humble/setup.bash; source "$1/local_setup.bash"; shift; exec "$@"'
    return ["bash", "-c", shell, "impact_server", str(install), *command]


def observer(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from ament_index_python.packages import get_package_prefix
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Imu, PointCloud2
    from nav_msgs.msg import Odometry, OccupancyGrid
    from xq_sim_interfaces.msg import HealthStatus
    output = args.observer
    rclpy.init()
    node = Node("impact_server_sensor_observer")
    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
    specs = {
        "clock": ("/clock", Clock), "lidar": ("/xq/agent_01/sensors/lidar/points", PointCloud2),
        "imu": ("/xq/agent_01/sensors/imu", Imu), "odom": ("/xq/agent_01/localization/odom", Odometry),
        "map": ("/xq/agent_01/map/nav", OccupancyGrid), "health": ("/xq/agent_01/health", HealthStatus),
    }
    prefixes = {name: get_package_prefix(name) for name in ("xq_autonomy", "xq_gz_bridge", "xq_gz_assets", "xq_sim_bringup", "xq_sim_interfaces")}
    seen = defaultdict(int)
    stats = {key: {"count": 0, "first_stamp_ns": None, "last_stamp_ns": None, "nonincreasing": 0, "invalid_payloads": 0, "sampled_finite_nonzero_xyz": 0} for key in specs}
    window_start = None
    startup_counts = {}
    ledger = (output / "sensor-observations.jsonl").open("x")
    def receive(key, msg):
        nonlocal window_start
        seen[key] += 1
        if window_start is None:
            return
        item = stats[key]
        stamp = None
        if key == "clock":
            stamp = msg.clock.sec * 1_000_000_000 + msg.clock.nanosec
        elif hasattr(msg, "header"):
            stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        item["count"] += 1
        if stamp is not None:
            if item["last_stamp_ns"] is not None and stamp <= item["last_stamp_ns"]:
                item["nonincreasing"] += 1
            if item["first_stamp_ns"] is None:
                item["first_stamp_ns"] = stamp
            item["last_stamp_ns"] = stamp
        valid = True
        if key == "imu":
            valid = stamp > 0 and msg.header.frame_id == "xq_imu_link" and all(math.isfinite(v) for v in (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z, msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z))
        elif key == "lidar":
            valid = stamp > 0 and msg.width * msg.height > 0 and msg.point_step > 0 and len(msg.data) >= msg.row_step * msg.height and msg.header.frame_id == "xq_mid360_link"
            fields = {field.name: field for field in msg.fields}
            valid = valid and all(name in fields and fields[name].datatype == 7 and fields[name].count == 1 and 0 <= fields[name].offset <= msg.point_step - 4 for name in ("x", "y", "z"))
            finite = 0
            if valid:
                endian = ">f" if msg.is_bigendian else "<f"
                count = msg.width * msg.height
                for index in range(0, count, max(1, count // 64)):
                    offset = (index // msg.width) * msg.row_step + (index % msg.width) * msg.point_step
                    xyz = [struct.unpack_from(endian, msg.data, offset + fields[name].offset)[0] for name in ("x", "y", "z")]
                    finite += int(all(math.isfinite(value) for value in xyz) and sum(value * value for value in xyz) > 1e-4)
                item["sampled_finite_nonzero_xyz"] += finite
            valid = valid and finite > 0
        elif key == "odom":
            valid = all(math.isfinite(v) for v in (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z))
        if not valid:
            item["invalid_payloads"] += 1
        ledger.write(json.dumps({"kind": "sensor_observation", "topic": specs[key][0], "monotonic_ns": time.monotonic_ns(), "stamp_ns": stamp, "valid_payload": valid}) + "\n")
    subscriptions = [node.create_subscription(kind, topic, lambda msg, key=key: receive(key, msg), qos) for key, (topic, kind) in specs.items()]
    write(output / "observer-ready.json", {"prefixes": prefixes, "pid": os.getpid()})
    started = time.monotonic()
    while not (output / "launch-started.json").exists() and time.monotonic() - started < 30:
        rclpy.spin_once(node, timeout_sec=0.05)
    startup = time.monotonic()
    while time.monotonic() - startup < args.startup_timeout + args.duration:
        rclpy.spin_once(node, timeout_sec=0.02)
        if window_start is None and all(seen[key] for key in ("clock", "lidar", "imu", "odom")):
            window_start = time.monotonic()
            startup_counts = dict(seen)
        if window_start is not None and time.monotonic() - window_start >= args.duration:
            break
        if window_start is None and time.monotonic() - startup >= args.startup_timeout:
            break
    elapsed = time.monotonic() - window_start if window_start is not None else 0.0
    names = node.get_node_names_and_namespaces()
    graph = []
    for name, namespace in names:
        try:
            topics = node.get_subscriber_names_and_types_by_node(name, namespace)
            graph.append({"name": name, "namespace": namespace, "subscriptions": topics})
        except Exception as error:
            graph.append({"name": name, "namespace": namespace, "error": str(error)})
    ledger.close()
    truth_leaks = [entry for entry in graph if entry["name"] == "xq_stack_node"
                   and any(topic.startswith("/xq/eval/") for topic, _ in entry.get("subscriptions", []))]
    checks = {
        "complete_wall_observation_window": elapsed >= args.duration,
        "clock_observed": stats["clock"]["count"] >= 10,
        "lidar_observed": stats["lidar"]["count"] >= 10,
        "imu_observed": stats["imu"]["count"] >= 100,
        "proxy_odometry_observed": stats["odom"]["count"] >= 10,
        "proxy_map_and_health_observed": stats["map"]["count"] > 0 and stats["health"]["count"] > 0,
        "sensor_headers_strictly_increasing": all(stats[key]["nonincreasing"] == 0 for key in ("lidar", "imu")),
        "sensor_payloads_valid": all(stats[key]["invalid_payloads"] == 0 for key in ("lidar", "imu", "odom")),
        "simulation_clock_advances": stats["clock"]["last_stamp_ns"] is not None and stats["clock"]["last_stamp_ns"] > stats["clock"]["first_stamp_ns"],
        "algorithm_node_present_and_truth_isolated": any(entry["name"] == "xq_stack_node" for entry in graph) and not truth_leaks,
        "no_mavros_or_AP_mission_nodes": not any("mavros" in name.lower() or "mission" in name.lower() or "ap_execution" in name.lower() for name, _ in names),
        "only_fresh_build_prefixes": all(Path(prefix).resolve() == args.install.resolve() for prefix in prefixes.values()),
    }
    report = {"schema_version": 1, "status": "PASS" if all(checks.values()) else "FAIL", "checks": checks,
              "duration_wall_s": elapsed, "startup_wall_s": (window_start or time.monotonic()) - startup,
              "streams": stats, "startup_counts": startup_counts, "total_received_counts": dict(seen), "prefixes": prefixes, "graph": graph,
              "scope": "LiDAR/IMU and legacy 2D algorithm proxies in no-AP VelocityControl scene. Not FAST-LIO/LIVO execution, camera/range, joint-flow fusion, flight dynamics or mission validation."}
    write(output / "observer-result.json", report)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if report["status"] == "PASS" else 1


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path)
    parser.add_argument("--runtime-root", type=Path)
    parser.add_argument("--build-attempt", type=Path)
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--startup-timeout", type=float, default=60.0)
    parser.add_argument("--renderer", choices=("software", "auto"), default="software")
    parser.add_argument("--ros-domain-id", type=int)
    parser.add_argument("--observer", type=Path, help=argparse.SUPPRESS)
    parser.add_argument("--install", type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if args.observer:
        return observer(args)
    if not args.root or not args.runtime_root or not 1 <= args.duration <= 600:
        parser.error("--root, --runtime-root and duration in [1,600] are required")
    root = args.root.resolve()
    runtime = args.runtime_root.absolute()
    if runtime == root or root in runtime.parents or any(p.is_symlink() for p in (runtime, *runtime.parents)):
        parser.error("Runtime must be a real directory outside bundle")
    runtime.mkdir(parents=True, exist_ok=True)
    attempts = sorted(runtime.glob("build_*"))
    build = args.build_attempt.resolve() if args.build_attempt else (attempts[-1] if attempts else None)
    if build is None:
        parser.error("No build attempt; run build_workspace.sh first")
    build_result = json.loads((build / "build-result.json").read_text())
    if build_result.get("status") != "PASS" or Path(build_result["root"]).resolve() != root:
        parser.error("Selected/latest build did not PASS for this bundle; no fallback")
    install = build / "install"
    out = runtime / ("sim_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "_" + str(os.getpid()))
    out.mkdir()
    print(str(out), flush=True)
    before = fingerprint(root)
    write(out / "inputs.before.json", before)
    built_sources = json.loads((build / "sources.after.json").read_text())
    current_sources = {key: value for key, value in before.items() if key.startswith("src/")}
    if current_sources != built_sources:
        write(out / "result.json", {"schema_version": 1, "status": "FAIL", "stage": "preflight",
                                   "error": "Current bundle source set/SHA differs from selected build; rebuild required"})
        return 1
    resources = {}
    for source in sorted((root / "src/xq_gz_assets").rglob("*")):
        relative = source.relative_to(root / "src/xq_gz_assets")
        if source.is_file() and relative.parts[0] in ("models", "worlds"):
            resources["share/xq_gz_assets/" + relative.as_posix()] = source
    resources.update({"share/xq_sim_bringup/launch/xq_sil.launch.py": root / "src/xq_sim_bringup/launch/xq_sil.launch.py",
                      "share/xq_gz_bridge/config/bridge.yaml": root / "src/xq_gz_bridge/config/bridge.yaml",
                      "lib/python3.10/site-packages/xq_autonomy/stack_node.py": root / "src/xq_autonomy/xq_autonomy/stack_node.py"})
    resource_identity = {name: {"source_sha256": digest(source),
                               "installed_sha256": digest(install / name) if (install / name).is_file() else None}
                         for name, source in resources.items()}
    write(out / "installed-resource-identity.json", resource_identity)
    if not all(item["source_sha256"] == item["installed_sha256"] for item in resource_identity.values()):
        write(out / "result.json", {"schema_version": 1, "status": "FAIL", "stage": "preflight",
                                   "error": "Installed model/world/launch/bridge/proxy source identity mismatch"})
        return 1
    excluded = used_domains()
    choices = [value for value in range(32, 102) if value not in excluded]
    domain = args.ros_domain_id if args.ros_domain_id is not None else (choices[time.time_ns() % len(choices)] if choices else None)
    if domain not in choices:
        raise ValueError("Requested domain is occupied/unavailable")
    partition = "impact_server_" + str(os.getpid()) + "_" + str(time.time_ns())
    env = dict(os.environ)
    for key in ("AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH", "COLCON_PREFIX_PATH", "LD_LIBRARY_PATH", "PYTHONPATH", "ROS_DISTRO", "GZ_SIM_RESOURCE_PATH", "IGN_GAZEBO_RESOURCE_PATH", "SDF_PATH", "GZ_SIM_SYSTEM_PLUGIN_PATH", "DISPLAY", "WAYLAND_DISPLAY", "LD_PRELOAD", "FASTRTPS_DEFAULT_PROFILES_FILE", "FASTDDS_DEFAULT_PROFILES_FILE", "ROS_DISCOVERY_SERVER", "CYCLONEDDS_URI"):
        env.pop(key, None)
    for name in ("ros-logs", "cache", "config"):
        (out / name).mkdir()
    env.update({"ROS_LOG_DIR": str(out / "ros-logs"), "XDG_CACHE_HOME": str(out / "cache"), "XDG_CONFIG_HOME": str(out / "config"), "ROS_DOMAIN_ID": str(domain), "GZ_PARTITION": partition, "ROS_LOCALHOST_ONLY": "1", "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp", "ROS2CLI_DISABLE_DAEMON": "1", "PYTHONNOUSERSITE": "1", "PYTHONDONTWRITEBYTECODE": "1",
                "GZ_SIM_RESOURCE_PATH": str(install / "share/xq_gz_assets/models") + ":" + str(root / "third_party/ardupilot_gazebo/models"),
                "GZ_SIM_SYSTEM_PLUGIN_PATH": str(install / "lib")})
    if args.renderer == "software":
        env.update({"LIBGL_ALWAYS_SOFTWARE": "1", "GALLIUM_DRIVER": "llvmpipe", "MESA_LOADER_DRIVER_OVERRIDE": "llvmpipe", "EGL_PLATFORM": "surfaceless"})
    commands = {
        "observer": shell_command(install, ["python3", str(Path(__file__).resolve()), "--observer", str(out), "--install", str(install), "--duration", str(args.duration), "--startup-timeout", str(args.startup_timeout)]),
        "launch": shell_command(install, ["ros2", "launch", "xq_sim_bringup", "xq_sil.launch.py", "headless:=true", "start_metrics:=false", "start_network_relay:=false", "inject_faults:=false", "start_stack:=true", "run_dir:=" + str(out / "proxy-results"), "ros_domain_id:=" + str(domain), "gz_partition:=" + partition]),
    }
    write(out / "run-configuration.json", {"root": str(root), "runtime": str(runtime), "build": str(build), "install": str(install), "domain": domain, "gz_partition": partition, "renderer": args.renderer, "duration_s": args.duration, "commands": commands, "AP_started": False, "arming_messages_authorized": False, "environment": {key: value for key, value in env.items() if key.startswith(("ROS_", "GZ_", "LIBGL", "GALLIUM", "MESA", "EGL", "PYTHON"))}})
    launched = {}
    logs = []
    shutdown = {}
    error = None
    try:
        for key in ("observer", "launch"):
            if key == "launch":
                deadline = time.monotonic() + 20
                while not (out / "observer-ready.json").exists() and launched["observer"].poll() is None and time.monotonic() < deadline:
                    time.sleep(0.1)
                if not (out / "observer-ready.json").is_file():
                    raise RuntimeError("Observer did not become ready")
            log = (out / (key + ".log")).open("x")
            logs.append(log)
            launched[key] = subprocess.Popen(commands[key], cwd=out, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            if key == "launch":
                write(out / "launch-started.json", {"monotonic_ns": time.monotonic_ns(), "launch_pid": launched[key].pid})
        deadline = time.monotonic() + args.startup_timeout + args.duration + 15
        while launched["observer"].poll() is None:
            if launched["launch"].poll() is not None:
                raise RuntimeError("ROS launch exited before observation completed")
            if time.monotonic() > deadline:
                raise RuntimeError("Observer deadline exceeded")
            time.sleep(0.2)
    except BaseException as caught:
        error = type(caught).__name__ + ": " + str(caught)
    finally:
        for key in ("observer", "launch"):
            if key in launched:
                shutdown[key] = stop_owned(launched[key])
        for log in logs:
            log.close()
    after = fingerprint(root)
    write(out / "inputs.after.json", after)
    observed = json.loads((out / "observer-result.json").read_text()) if (out / "observer-result.json").exists() else {}
    checks = {"observer_PASS": observed.get("status") == "PASS", "no_runner_exception": error is None,
              "installed_resources_match_bundled_canonical_sources": True,
              "input_file_sets_and_SHA_unchanged": before == after,
              "all_owned_processes_stopped": all(not item["owned_remaining"] for item in shutdown.values())}
    report = {"schema_version": 1, "status": "PASS" if all(checks.values()) else "FAIL", "checks": checks, "error": error,
              "observer": observed, "shutdown": shutdown, "run_dir": str(out), "build_attempt": str(build),
              "script_sha256": digest(Path(__file__)), "scope": "Fresh portable no-AP Gazebo LiDAR/IMU + legacy 2D algorithm-proxy smoke only. No armed command, MAVROS, physical AP flight, camera/range or new joint estimator validation."}
    write(out / "result.json", report)
    print(json.dumps({"status": report["status"], "run_dir": str(out), "counts": {key: value["count"] for key, value in observed.get("streams", {}).items()}, "error": error}), flush=True)
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())

