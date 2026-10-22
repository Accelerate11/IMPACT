#!/usr/bin/env python3
"""Compile a behavioral fixture against the actual isolated native LIVO library.

The fixture exercises real ImuProcess::Process2, rather than a copied numerical
model. Full five-sensor replay and AP acceptance remain separate obligations.
"""
from __future__ import annotations
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess


def record(path):
    data = path.read_bytes()
    return {"bytes": len(data), "sha256": hashlib.sha256(data).hexdigest()}


def execute(arguments, cwd, logfile):
    result = subprocess.run(arguments, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    logfile.write_text(result.stdout)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace-root", type=Path, required=True)
    args = parser.parse_args(argv)
    workspace = args.workspace_root.resolve()
    output = workspace / "experiments/results/research_multisensor" / (
        "native_empty_lio_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "_" + str(os.getpid()))
    output.mkdir()
    print(output, flush=True)
    build = workspace / "joint_livo_build/xq_fast_livo2"
    manifest = workspace / "joint_livo_install/.impact-joint-build-manifest.json"
    fixture = workspace / "src/xq_fast_livo2/test/test_research_empty_lio.cpp"
    flags_path = build / "CMakeFiles/xq_livo_core.dir/flags.make"
    link_path = build / "CMakeFiles/xq_livo_mapping.dir/link.txt"
    archive = build / "libxq_livo_core.a"
    paths = {"fixture.cpp": fixture, "build-manifest.json": manifest,
             "flags.make": flags_path, "link.txt": link_path, "native-library.a": archive,
             "runner.py": Path(__file__).resolve()}
    before = {name: record(path) for name, path in paths.items()}
    for name, path in paths.items():
        if name != "native-library.a":
            shutil.copyfile(path, output / name)
    check_command = ["python3", str(workspace / "scripts/check_joint_livo_build_manifest.py"),
                     "--workspace-root", str(workspace), "--mode", "check"]
    checked = execute(check_command, workspace, output / "freshness.before.log")
    if checked.returncode:
        raise ValueError("native library is not a fresh source-bound build")
    make = {}
    for line in flags_path.read_text().splitlines():
        if " = " in line:
            name, value = line.split(" = ", 1)
            make[name] = shlex.split(value)
    object_file = output / "fixture.o"
    compile_command = ["/usr/bin/c++", *make["CXX_DEFINES"], *make["CXX_INCLUDES"],
                       *make["CXX_FLAGS"], "-O0", "-g0", "-c", str(output / "fixture.cpp"), "-o", str(object_file)]
    compiled = execute(compile_command, build, output / "compile.log")
    link_command = shlex.split(link_path.read_text())
    main_objects = [i for i, word in enumerate(link_command) if word.endswith("/src/main.cpp.o")]
    if len(main_objects) != 1:
        raise ValueError("native executable link command does not have one main object")
    link_command[main_objects[0]] = str(object_file)
    binary = output / "test_research_empty_lio"
    link_command[link_command.index("-o") + 1] = str(binary)
    linked = execute(link_command, build, output / "link.log") if compiled.returncode == 0 else None
    tested = execute([str(binary)], output, output / "test.log") if linked is not None and linked.returncode == 0 else None
    after = {name: record(path) for name, path in paths.items()}
    final_check = execute(check_command, workspace, output / "freshness.after.log")
    matched = re.search(r"^PASS (\d+) checks;", tested.stdout if tested else "", re.MULTILINE)
    passed = (compiled.returncode == 0 and linked is not None and linked.returncode == 0 and
              tested is not None and tested.returncode == 0 and matched is not None and
              before == after and final_check.returncode == 0)
    report = {
        "schema_version": 1, "status": "PASS" if passed else "FAIL",
        "scope": "actual_native_ImuProcess_behavior_not_complete_sensor_replay",
        "inputs_before": before, "inputs_after": after,
        "source_build_inputs_and_native_library_sha256_unchanged": before == after,
        "compile_command": compile_command, "link_command": link_command,
        "compile_exit_code": compiled.returncode,
        "link_exit_code": linked.returncode if linked else None,
        "test_exit_code": tested.returncode if tested else None,
        "assertion_checks": int(matched.group(1)) if matched else None,
        "binary": record(binary) if binary.exists() else None,
        "behaviors": ["empty_LIO_advances_IMU_nominal_covariance_and_epoch",
                      "empty_LIO_does_not_fabricate_lidar_points",
                      "same_stamp_VIO_does_not_repeat_integration_or_process_noise",
                      "aggregate_F_Q_reconstruct_actual_native_covariance",
                      "next_real_cloud_matches_nonmissing_cloud_IMU_control"],
        "bootstrap_definition": "stationary_initialization_nominal_defined_at_measurement_epoch_no_prior_integration_claim",
        "full_native_replay_validated": False, "closed_loop_AP_validated": False,
        "hardware_validated": False,
    }
    (output / "result.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(json.dumps({"status": report["status"], "assertion_checks": report["assertion_checks"], "output": str(output)}))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
