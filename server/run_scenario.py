#!/usr/bin/env python3
"""Portable historical 3D FAST-LIO/integrity/dynamic-map scenarios, never AP flight.

No canonical algorithm, launch, threshold or calibration file is rewritten.
Short smoke evidence is explicitly separate from completed mission evaluation.
"""
from __future__ import annotations
import argparse
from collections import defaultdict
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

from run_headless_sim import digest, fingerprint, shell_command, stop_owned, used_domains, write

PROFILES = {
    "warehouse": ("complex_comparison_thresholds.json", "xq_complex_warehouse.sdf"),
    "warehouse_3d": ("complex_3d_thresholds.json", "xq_complex_3d_warehouse.sdf"),
    "bidirectional": ("complex_bidirectional_thresholds.json", "xq_complex_bidirectional_warehouse.sdf"),
    "compositional": ("complex_compositional_thresholds.json", "xq_complex_compositional_warehouse.sdf"),
    "p15_lattice": ("p15_research_thresholds.json", "xq_complex_compositional_warehouse.sdf"),
    "p14_fault": ("p14_gate_thresholds.json", "xq_p12_dynamic_obstacle.sdf"),
}
SCOPE = "Historical native FAST-LIO + integrity/candidate/dynamic-map/latency stack with a velocity-controlled algorithm carrier. No AP/MAVROS/ARM, new joint-flow estimator, camera/range fusion, EGO executable execution or hardware validation."


def observe(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from ament_index_python.packages import get_package_prefix
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Imu, PointCloud2
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from xq_sim_interfaces.msg import DirectionalIntegrity, IntegrityExplorationDecision, FaultEvent
    out = args.observe
    rclpy.init()
    node = Node("impact_server_advanced_observer")
    sensor = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
    reliable = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
    latched = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    streams = {
        "clock": ("/clock", Clock, sensor),
        "lidar": ("/livox/lidar", PointCloud2, sensor),
        "imu": ("/livox/imu", Imu, sensor),
        "native_odom": ("/localization/odom", Odometry, reliable),
        "integrity": ("/integrity/directional", DirectionalIntegrity, reliable),
        "static_map": ("/mapping/p12/static_voxels", PointCloud2, latched),
        "decision": ("/integrity/exploration_decision", IntegrityExplorationDecision, latched),
        "status": ("/xq/p13/flight_status", String, latched),
        "fault": ("/impact/fault_event", FaultEvent, latched),
    }
    seen = defaultdict(int)
    stats = {name: {"count": 0, "first_stamp_ns": None, "last_stamp_ns": None, "nonincreasing": 0} for name in streams}
    window = None
    startup_counts = {}
    last_status = {}
    first_position = last_position = None
    maximum_estimated_displacement_m = 0.0
    with (out / "observations.jsonl").open("x") as ledger:
        def callback(name, msg):
            nonlocal last_status, first_position, last_position, maximum_estimated_displacement_m
            seen[name] += 1
            stamp = None
            if name == "clock":
                stamp = msg.clock.sec * 1_000_000_000 + msg.clock.nanosec
            elif hasattr(msg, "header"):
                stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            event = {"kind": name, "phase": "observation" if window is not None else "startup",
                     "monotonic_ns": time.monotonic_ns(), "stamp_ns": stamp}
            if name == "status":
                event["raw"] = msg.data
                try:
                    parsed = json.loads(msg.data)
                    last_status = {key: parsed.get(key) for key in ("phase", "finished", "current_position_x_m", "decisions_applied", "latency_sample_count", "latency_p99_ms")}
                except (ValueError, TypeError):
                    last_status = {"parse_error": True}
            if name == "native_odom":
                position = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
                if all(math.isfinite(value) for value in position):
                    first_position = position if first_position is None else first_position
                    last_position = position
                    maximum_estimated_displacement_m = max(maximum_estimated_displacement_m, math.dist(first_position, position))
                event["estimated_position_xyz"] = position if all(math.isfinite(value) for value in position) else None
            if name == "decision":
                event.update(valid=bool(msg.valid), selected_trajectory_id=int(msg.selected_trajectory_id))
            if name == "fault":
                event.update(fault_id=msg.fault_id, target=msg.target_module, severity=msg.severity, duration_s=msg.duration_s, seed=msg.seed)
            ledger.write(json.dumps(event, allow_nan=False) + "\n")
            if window is not None:
                item = stats[name]
                item["count"] += 1
                if stamp is not None:
                    if item["last_stamp_ns"] is not None and stamp <= item["last_stamp_ns"]:
                        item["nonincreasing"] += 1
                    if item["first_stamp_ns"] is None:
                        item["first_stamp_ns"] = stamp
                    item["last_stamp_ns"] = stamp
        subscriptions = [node.create_subscription(kind, topic, lambda msg, name=name: callback(name, msg), qos) for name, (topic, kind, qos) in streams.items()]
        prefixes = {name: get_package_prefix(name) for name in ("xq_fast_lio", "xq_autonomy", "xq_gz_bridge", "xq_gz_assets", "xq_sim_bringup")}
        write(out / "observer-ready.json", {"pid": os.getpid(), "prefixes": prefixes})
        startup = time.monotonic()
        while not (out / "launch-started.json").exists() and time.monotonic() - startup < 30:
            rclpy.spin_once(node, timeout_sec=0.05)
        startup = time.monotonic()
        required = ("clock", "lidar", "imu", "native_odom", "integrity", "static_map", "decision", "status")
        while True:
            rclpy.spin_once(node, timeout_sec=0.02)
            now = time.monotonic()
            if window is None and all(seen[name] for name in required):
                window = now
                startup_counts = dict(seen)
            if window is not None and now - window >= args.duration:
                break
            if window is None and now - startup > args.startup_timeout:
                break
    elapsed = time.monotonic() - window if window is not None else 0.0
    graph = []
    for name, namespace in node.get_node_names_and_namespaces():
        try:
            graph.append({"name": name, "namespace": namespace,
                          "subscriptions": node.get_subscriber_names_and_types_by_node(name, namespace)})
        except Exception as error:
            graph.append({"name": name, "namespace": namespace, "error": str(error)})
    algorithm_nodes = [item for item in graph if "evaluator" not in item["name"] and item["name"] != "impact_server_advanced_observer"]
    truth_leaks = [item["name"] for item in algorithm_nodes
                   if any(topic.startswith("/xq/eval/") for topic, _ in item.get("subscriptions", []))]
    checks = {
        "complete_wall_observation_window": elapsed >= args.duration,
        "all_native_and_advanced_topics_observed": all(seen[name] > 0 for name in required),
        "native_odom_and_integrity_continue_during_window": stats["native_odom"]["count"] >= 5 and stats["integrity"]["count"] >= 5,
        "lidar_and_IMU_observed_during_window": stats["lidar"]["count"] >= 10 and stats["imu"]["count"] >= 100,
        "native_FAST_LIO_node_present": any(item["name"] == "xq_fast_lio" for item in graph),
        "no_algorithm_truth_subscription": not truth_leaks,
        "no_AP_or_MAVROS_nodes": not any("mavros" in item["name"].lower() or "ap_execution" in item["name"].lower() for item in graph),
        "all_package_prefixes_are_fresh_install": all(Path(path).resolve() == args.install.resolve() for path in prefixes.values()),
    }
    result = {"schema_version": 1, "status": "PASS" if all(checks.values()) else "FAIL", "checks": checks,
              "wall_observation_s": elapsed, "startup_wall_s": (window or time.monotonic()) - startup,
              "startup_counts": startup_counts, "streams": stats, "total_received": dict(seen),
              "last_controller_status": last_status, "first_estimated_position_xyz": first_position,
              "last_estimated_position_xyz": last_position, "maximum_estimated_displacement_m": maximum_estimated_displacement_m,
              "fault_observation": "OBSERVED" if seen["fault"] else "NOT_OBSERVED",
              "truth_leaks": truth_leaks, "graph": graph, "prefixes": prefixes, "scope": SCOPE}
    write(out / "observer-result.json", result)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if result["status"] == "PASS" else 1


def make_launch(root, install, out, args, calibration):
    threshold_name, world_name = PROFILES[args.scenario]
    profile = json.loads((root / "config" / threshold_name).read_text())
    params = {"world_file": str(install / "share/xq_gz_assets/worlds" / world_name),
              "calibration_file": str(calibration), "calibration_sha256": digest(calibration),
              "mission_distance_m": profile.get("mission_distance_m", 24.0)}
    if args.scenario == "p14_fault":
        if args.fault_file:
            source = args.fault_file.resolve()
            raw = source.read_bytes()
            schedule = json.loads(raw)
            (out / "configuration/fault-schedule.json").write_bytes(raw)
            fault_origin = {"file": str(source), "sha256": digest(source), "policy": "Provided schedule bytes and all explicit seeds retained."}
        elif args.fault:
            schedule = {"schema_version": 1, "seed": args.seed, "faults": [{"id": "server_single", "type": args.fault,
                        "start_time_s": args.fault_start, "duration_s": args.fault_duration, "severity": args.fault_severity}]}
            write(out / "configuration/fault-schedule.json", schedule)
            fault_origin = {"policy": "Explicit server single fault; seed from CLI."}
        else:
            source = root / "config/p14_matrix_schedule.json"
            (out / "configuration/fault-schedule.json").write_bytes(source.read_bytes())
            schedule = json.loads(source.read_text())
            fault_origin = {"file": str(source), "sha256": digest(source), "policy": "Original matrix schedule and seeds retained; --seed controls Gazebo RNG only."}
        if not schedule.get("faults"):
            raise ValueError("P14 requires an explicit nonempty fault schedule")
        params.update(trial="matrix", schedule_file=str(out / "configuration/fault-schedule.json"),
                      thresholds_file=str(root / "config/p14_gate_thresholds.json"),
                      p12_thresholds_file=str(root / "config/p12_gate_thresholds.json"),
                      p13_thresholds_file=str(root / "config/p13_gate_thresholds.json"),
                      p12_result_file=str(out / "mission-p12.json"), p13_result_file=str(out / "mission-p13.json"),
                      p14_result_file=str(out / "mission-p14.json"), planner_delay_ms=args.planner_delay_ms)
        launch_name = "xq_p14_fault_injection.launch.py"
    else:
        fault_origin = {"policy": "No injected sensor faults in this scenario"}
        params.update(headless_rendering=True, thresholds_file=str(root / "config/p13_gate_thresholds.json"),
                      p12_thresholds_file=str(root / "config/complex_dynamic_thresholds.json"),
                      p12_result_file=str(out / "mission-p12.json"), p13_result_file=str(out / "mission-p13.json"),
                      flight_variant=args.variant, enable_p11_evaluator=True, p11_result_file=str(out / "mission-p11.json"),
                      planner_delay_ms=args.planner_delay_ms,
                      latency_profile="low_50ms" if args.planner_delay_ms == 50 else "high_200ms" if args.planner_delay_ms == 200 else "server_custom",
                      lateral_candidate_shape="challenge_then_center", obstacle_park_y_m=6.6,
                      geometric_clearance_m=0.82, fixed_buffer_m=0.58, protection_level_m=0.10,
                      required_margin_m=0.06, maximum_speed_mps=0.42, maximum_acceleration_mps2=0.8,
                      rejected_candidate_retry_s=1.0, maximum_candidate_retries=60,
                      dynamic_path_query_mode="forward_axis", minimum_dynamic_cluster_points=1)
        keys = ("lateral_offset_m", "enable_vertical_candidate", "enable_diagonal_vertical_candidates",
                "vertical_offset_m", "integrity_information_memory_horizon_s", "integrity_information_memory_max_frames",
                "segment_goal_tolerance_m", "post_dynamic_static_confirmation_s", "reversible_static_ttl_s",
                "maximum_rays", "obstacle_enter_start_s", "obstacle_enter_end_s", "obstacle_leave_start_s", "obstacle_leave_end_s")
        params.update({key: profile[key] for key in keys if key in profile})
        if args.scenario == "p15_lattice":
            params.update(candidate_generation_mode="lattice", candidate_metric_source="online_map",
                          lattice_lateral_levels=5, lattice_vertical_levels=2, research_energy_remaining=70.0,
                          utility_indifference_band=0.05, dynamic_path_query_mode="active_trajectory",
                          minimum_dynamic_cluster_points=5, dynamic_cluster_radius_m=0.45,
                          terminal_extension_mode="progress_watchdog", runtime_integrity_guard_mode="current_margin_replan",
                          runtime_integrity_margin_m=0.12)
        launch_name = "xq_p13_flight.launch.py"
    write(out / "configuration/scenario-profile.json", {"scenario": args.scenario, "original_profile": profile,
          "profile_source_sha256": digest(root / "config" / threshold_name), "launch_parameters": params,
          "fault_origin": fault_origin})
    command = ["ros2", "launch", "xq_sim_bringup", launch_name]
    for name, value in params.items():
        command.append(name + ":=" + (str(value).lower() if type(value) is bool else str(value)))
    return launch_name, command


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path)
    parser.add_argument("--runtime-root", type=Path)
    parser.add_argument("--build-attempt", type=Path)
    parser.add_argument("--scenario", choices=tuple(PROFILES), default="p15_lattice")
    parser.add_argument("--variant", choices=("information_only", "integrity_constrained"), default="integrity_constrained")
    parser.add_argument("--seed", type=int, default=501)
    parser.add_argument("--duration", type=float, default=30.0, help="Wall observation seconds after all required topics appear")
    parser.add_argument("--startup-timeout", type=float, default=90.0)
    parser.add_argument("--planner-delay-ms", type=float, default=50.0)
    parser.add_argument("--renderer", choices=("software", "auto"), default="software")
    parser.add_argument("--calibration-file", type=Path)
    parser.add_argument("--fault-file", type=Path)
    parser.add_argument("--fault", choices=("lidar_dropout", "imu_dropout", "odom_delay", "planner_delay", "camera_failure", "20_percent_packet_loss", "cpu_load", "timestamp_jitter", "localization_covariance_inflation", "low_battery"))
    parser.add_argument("--fault-start", type=float, default=20.0)
    parser.add_argument("--fault-duration", type=float, default=2.0)
    parser.add_argument("--fault-severity", type=float, default=1.0)
    parser.add_argument("--require-mission-complete", action="store_true")
    parser.add_argument("--observe", type=Path, help=argparse.SUPPRESS)
    parser.add_argument("--install", type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if args.observe:
        return observe(args)
    if not args.root or not args.runtime_root or not 1 <= args.duration <= 3600 or not 0 <= args.seed < 2**32 or not 0 <= args.planner_delay_ms <= 2000:
        parser.error("Required roots, duration1..3600, uint32 seed and delay0..2000")
    if (args.fault or args.fault_file) and args.scenario != "p14_fault":
        parser.error("Fault options require explicit p14_fault profile")
    if args.scenario == "p14_fault" and args.variant != "integrity_constrained":
        parser.error("p14_fault supports only the historical integrity_constrained controller")
    if args.fault and args.fault_file:
        parser.error("Select one fault file or single fault")
    if args.fault_start < 0 or args.fault_duration <= 0 or not math.isfinite(args.fault_severity):
        parser.error("Invalid fault interval/severity")
    root, runtime = args.root.resolve(), args.runtime_root.absolute()
    if runtime == root or root in runtime.parents or any(path.is_symlink() for path in (runtime, *runtime.parents)):
        parser.error("Use a real external runtime root")
    runtime.mkdir(parents=True, exist_ok=True)
    candidates = sorted(runtime.glob("build_*"))
    build = args.build_attempt.resolve() if args.build_attempt else candidates[-1] if candidates else None
    if build is None:
        parser.error("No fresh base build found")
    built = json.loads((build / "build-result.json").read_text())
    if built.get("status") != "PASS" or Path(built["root"]).resolve() != root:
        parser.error("Selected/latest build not PASS for this bundle")
    install = build / "install"
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out = runtime / ("scenario_" + args.scenario + "_" + stamp + "_" + str(os.getpid()))
    out.mkdir()
    (out / "configuration").mkdir()
    print(str(out), flush=True)
    before = fingerprint(root)
    if {key: value for key, value in before.items() if key.startswith("src/")} != json.loads((build / "sources.after.json").read_text()):
        raise ValueError("Current canonical source does not match selected new build")
    write(out / "inputs.before.json", before)
    calibration_source = (args.calibration_file or Path(__file__).parent / "calibration/p7-calibration.json").resolve()
    calibration = out / "configuration/p7-calibration.json"
    calibration.write_bytes(calibration_source.read_bytes())
    if args.calibration_file is None:
        provenance = json.loads(calibration_source.with_name("PROVENANCE.json").read_text())
        allowed = {(provenance["bytes"], provenance["sha256"]),
                   (provenance["linux_lf_bytes"], provenance["linux_lf_sha256"])}
        if (calibration.stat().st_size, digest(calibration)) not in allowed:
            raise ValueError("Bundled historical calibration identity is neither original nor explicit Linux LF")
    frozen = json.loads(calibration.read_text())
    if not frozen.get("train_only") or frozen.get("test_data_used", True):
        raise ValueError("Only unchanged train-only old FAST-LIO calibration is accepted")
    launch_name, launch = make_launch(root, install, out, args, calibration)
    identity = {}
    for name in (launch_name,):
        source = root / "src/xq_sim_bringup/launch" / name
        target = install / "share/xq_sim_bringup/launch" / name
        identity[str(target)] = {"source": digest(source), "installed": digest(target)}
    for relative in ("xq_fast_lio/config/xq_p3.yaml", "xq_gz_bridge/config/p3_fast_lio.yaml"):
        source, target = root / "src" / relative, install / "share" / relative
        identity[str(target)] = {"source": digest(source), "installed": digest(target)}
    write(out / "installed-launch-config-identity.json", identity)
    if any(item["source"] != item["installed"] for item in identity.values()):
        raise ValueError("Installed launch/native configuration differs from canonical source")
    occupied = used_domains()
    choices = [value for value in range(32, 102) if value not in occupied]
    if not choices:
        raise ValueError("No unused visible ROS domain")
    domain = choices[time.time_ns() % len(choices)]
    partition = "impact_advanced_" + str(os.getpid()) + "_" + str(time.time_ns())
    env = dict(os.environ)
    for key in ("AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH", "COLCON_PREFIX_PATH", "LD_LIBRARY_PATH", "PYTHONPATH", "ROS_DISTRO", "ROS_DISCOVERY_SERVER", "CYCLONEDDS_URI", "FASTRTPS_DEFAULT_PROFILES_FILE", "FASTDDS_DEFAULT_PROFILES_FILE", "GZ_SIM_RESOURCE_PATH", "GZ_SIM_SYSTEM_PLUGIN_PATH", "IGN_GAZEBO_RESOURCE_PATH", "SDF_PATH", "DISPLAY", "WAYLAND_DISPLAY", "LD_PRELOAD"):
        env.pop(key, None)
    for name in ("bin", "ros-logs", "cache", "xdg-config"):
        (out / name).mkdir()
    real_gz = Path("/usr/bin/gz")
    shim = out / "bin/gz"
    shim.write_text("#!/usr/bin/python3\nimport os,sys,json\na=sys.argv[1:]\nif a and a[0]=='sim': a=a[:1]+['--seed'," + repr(str(args.seed)) + "]+a[1:]\nwith open(" + repr(str(out / "gz-command.jsonl")) + ",'a') as f:f.write(json.dumps(a)+'\\n')\nos.execv(" + repr(str(real_gz)) + ",[" + repr(str(real_gz)) + "]+a)\n")
    shim.chmod(0o755)
    # Match the separately validated base launcher: FastDDS builtin SHM/UDP,
    # localhost-only ROS and no inherited or generated transport override.
    env.update(ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY="1", RMW_IMPLEMENTATION="rmw_fastrtps_cpp",
               ROS2CLI_DISABLE_DAEMON="1", PYTHONNOUSERSITE="1", PYTHONDONTWRITEBYTECODE="1",
               GZ_PARTITION=partition, GZ_SIM_SYSTEM_PLUGIN_PATH=str(install / "lib"), ROS_LOG_DIR=str(out / "ros-logs"),
               XDG_CACHE_HOME=str(out / "cache"), XDG_CONFIG_HOME=str(out / "xdg-config"),
               PATH=str(out / "bin") + ":" + env.get("PATH", "/usr/bin:/bin"))
    if args.renderer == "software":
        env.update(LIBGL_ALWAYS_SOFTWARE="1", GALLIUM_DRIVER="llvmpipe", MESA_LOADER_DRIVER_OVERRIDE="llvmpipe", EGL_PLATFORM="surfaceless")
    commands = {
        "observer": shell_command(install, ["python3", str(Path(__file__).resolve()), "--observe", str(out), "--install", str(install), "--duration", str(args.duration), "--startup-timeout", str(args.startup_timeout)]),
        "launch": shell_command(install, launch),
    }
    write(out / "configuration.json", {"scenario": args.scenario, "variant": args.variant, "seed": args.seed,
          "seed_scope": "Actual gz sim --seed inserted by owned runtime shim; original bridge conditioning seed remains20260822; explicit fault schedule seeds retained.",
          "root": str(root), "build": str(build), "duration_wall_s": args.duration, "domain": domain, "partition": partition,
          "commands": commands, "gazebo_shim_sha256": digest(shim), "real_gz": str(real_gz), "real_gz_sha256": digest(real_gz),
          "DDS_scope": "Same as validated base: ROS_LOCALHOST_ONLY=1, FastDDS builtin SHM/UDP; inherited profiles cleared and no custom profile; no route/WSL/firewall/system changes",
          "calibration_original": str(calibration_source), "calibration_sha256": digest(calibration_source),
          "old_calibration_valid_for_new_joint_estimator": False, "scope": SCOPE})
    children, logs, shutdown = {}, [], {}
    error = None
    try:
        for name in ("observer", "launch"):
            if name == "launch":
                deadline = time.monotonic() + 20
                while not (out / "observer-ready.json").exists() and children["observer"].poll() is None and time.monotonic() < deadline:
                    time.sleep(.1)
                if not (out / "observer-ready.json").exists():
                    raise RuntimeError("Advanced observer failed to become ready")
            handle = (out / (name + ".log")).open("x")
            logs.append(handle)
            children[name] = subprocess.Popen(commands[name], env=env, cwd=out, stdout=handle, stderr=subprocess.STDOUT, start_new_session=True)
            if name == "launch":
                write(out / "launch-started.json", {"pid": children[name].pid, "monotonic_ns": time.monotonic_ns()})
        deadline = time.monotonic() + args.startup_timeout + args.duration + 20
        while children["observer"].poll() is None:
            if children["launch"].poll() is not None:
                raise RuntimeError("Advanced launch exited before observation")
            if time.monotonic() > deadline:
                raise TimeoutError("Advanced observation deadline")
            time.sleep(.2)
    except BaseException as caught:
        error = type(caught).__name__ + ": " + str(caught)
    finally:
        for name in ("observer", "launch"):
            if name in children:
                shutdown[name] = stop_owned(children[name])
        for handle in logs:
            handle.close()
    after = fingerprint(root)
    write(out / "inputs.after.json", after)
    observed = json.loads((out / "observer-result.json").read_text()) if (out / "observer-result.json").exists() else {}
    missions = {path.name: json.loads(path.read_text()) for path in out.glob("mission-*.json")}
    expected = ("mission-p12.json", "mission-p13.json", "mission-p14.json") if args.scenario == "p14_fault" else ("mission-p11.json", "mission-p12.json", "mission-p13.json")
    mission_pass = all(missions.get(name, {}).get("status") == "PASS" for name in expected)
    gz_calls = [json.loads(line) for line in (out / "gz-command.jsonl").read_text().splitlines()] if (out / "gz-command.jsonl").exists() else []
    checks = {"observer_PASS": observed.get("status") == "PASS", "no_runner_exception": error is None,
              "inputs_unchanged": before == after, "calibration_input_unchanged": digest(calibration_source) == digest(calibration),
              "actual_gazebo_seed_forwarded": any(call[:3] == ["sim", "--seed", str(args.seed)] for call in gz_calls),
              "all_owned_processes_stopped": all(not item["owned_remaining"] for item in shutdown.values())}
    if args.require_mission_complete:
        checks["all_original_mission_evaluators_PASS"] = mission_pass
    result = {"schema_version": 1, "status": "PASS" if all(checks.values()) else "FAIL", "acceptance_mode": "MISSION" if args.require_mission_complete else "SMOKE",
              "checks": checks, "error": error, "observer": observed, "shutdown": shutdown,
              "mission_status": "PASS" if mission_pass else "INCOMPLETE_OR_FAILED", "original_mission_results": missions,
              "mission_completion_required": args.require_mission_complete, "scope": SCOPE,
              "new_joint_estimator_integrated": False, "AP_flight_validated": False,
              "script_sha256": digest(Path(__file__)), "shared_helper_sha256": digest(Path(__file__).with_name("run_headless_sim.py"))}
    write(out / "result.json", result)
    print(json.dumps({"status": result["status"], "acceptance_mode": result["acceptance_mode"], "mission_status": result["mission_status"],
                      "run_dir": str(out), "counts": {name: item["count"] for name, item in observed.get("streams", {}).items()}, "error": error}), flush=True)
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())

