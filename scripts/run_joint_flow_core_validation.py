#!/usr/bin/env python3
"""Freeze, compile and run the Eigen-only nonlinear joint-flow core checks.

This artifact validates mathematical/core contracts.  It is explicitly not a
native ROS replay, AP closed-loop test, paper performance result or hardware test.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys


def digest(path: Path) -> dict:
    data = path.read_bytes()
    return {"bytes": len(data), "sha256": hashlib.sha256(data).hexdigest()}


def run(command: list[str], log: Path) -> subprocess.CompletedProcess:
    result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    log.write_text(result.stdout, encoding="utf-8")
    return result


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args(argv)
    workspace = args.workspace_root.resolve()
    now = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    output = args.output_dir or workspace / "experiments/results/research_multisensor" / f"joint_flow_core_{now}_{os.getpid()}"
    output = output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.mkdir()
    print(output, flush=True)
    package = workspace / "src/xq_fast_livo2"
    sources = {"include/research_joint_flow.h": package / "include/research_joint_flow.h",
               "test/test_research_joint_flow.cpp": package / "test/test_research_joint_flow.cpp",
               "runner/run_joint_flow_core_validation.py": Path(__file__).resolve()}
    before = {name: digest(path) for name, path in sources.items()}
    for name, path in sources.items():
        target = output / "sources" / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(path, target)
    eigen = Path("/usr/include/eigen3/Eigen")
    eigen_before = {str(path.relative_to(eigen)): digest(path) for path in sorted(eigen.rglob("*")) if path.is_file()}
    (output / "eigen-inputs.before.json").write_text(json.dumps(eigen_before, indent=2) + "\n")
    compiler = shutil.which("g++")
    if compiler is None:
        raise ValueError("g++ unavailable")
    compiler_version = subprocess.check_output([compiler, "--version"], text=True).strip()
    frontend = Path(subprocess.check_output([compiler, "-print-prog-name=cc1plus"], text=True).strip()).resolve()
    toolchain = {"compiler_path": compiler, "compiler_launcher": digest(Path(compiler).resolve()),
                 "compiler_version": compiler_version,
                 "frontend": {"path": str(frontend), **digest(frontend)},
                 "eigen_version": subprocess.check_output(["dpkg-query", "-W", "-f=${Version}", "libeigen3-dev"], text=True)}
    binary = output / "test_research_joint_flow"
    compile_command = [compiler, "-std=c++17", "-O0", "-g0", "-Wall", "-Wextra", "-Werror",
                       "-I/usr/include/eigen3", "-I" + str(output / "sources/include"),
                       str(output / "sources/test/test_research_joint_flow.cpp"), "-o", str(binary)]
    compiled = run(compile_command, output / "compile.log")
    tested = run([str(binary)], output / "test.log") if compiled.returncode == 0 else None
    after = {name: digest(path) for name, path in sources.items()}
    eigen_after = {str(path.relative_to(eigen)): digest(path) for path in sorted(eigen.rglob("*")) if path.is_file()}
    matched = re.search(r"^PASS (\d+) checks;", tested.stdout if tested else "", re.MULTILINE)
    successful = compiled.returncode == 0 and tested is not None and tested.returncode == 0 and matched is not None
    source_unchanged = before == after and eigen_before == eigen_after
    report = {
        "schema_version": 1, "status": "PASS" if successful and source_unchanged else "FAIL",
        "scope": "Eigen_only_core_mathematical_contracts_not_native_replay",
        "started_utc": now, "source_inputs_before": before, "source_inputs_after": after,
        "all_source_and_eigen_file_sets_and_sha256_unchanged": source_unchanged,
        "toolchain": toolchain, "compile_command": compile_command,
        "compile_exit_code": compiled.returncode,
        "test_exit_code": tested.returncode if tested else None,
        "assertion_checks": int(matched.group(1)) if matched else None,
        "binary": digest(binary) if binary.exists() else None,
        "independent_math_scope": [
            "analytic_28D_two_view_and_initial_pixel_Jacobians_vs_central_difference",
            "analytic_28D_ray_plane_range_Jacobian_vs_independent_central_difference",
            "right_SO3_reset_Jacobian_vs_independent_rotation_composition",
            "45000_seeded_range_plane_joint_augmentation_Monte_Carlo_covariance_samples",
            "native_conditional_extension_delta_P_and_Pgyro_vs_direct_28D_Joseph_EKF",
            "actual_native_nominal_and_covariance_changes_with_K_residual_reconstruction",
            "preupdate_NIS_degeneracy_unknown_covariance_and_stale_diagnostics_rejection",
            "PSD_gyro_component_decomposition_and_tag_inheritance_across_pairs",
        ],
        "native_ROS_replay_validated": False, "closed_loop_AP_validated": False,
        "hardware_validated": False, "algorithm_superiority_demonstrated": False,
        "limitations": [
            "native_fixed_map_and_independent_native_observation_approximation",
            "single_local_plane_with_declared_slope_prior_and_bounded_footprint",
            "pixel_covariance_is_explicit_input_not_empirical_sensor_calibration",
            "unit_assertion_count_is_not_estimation_or_paper_acceptance",
        ],
    }
    (output / "result.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    (output / "eigen-inputs.after.json").write_text(json.dumps(eigen_after, indent=2) + "\n")
    print(json.dumps({"status": report["status"], "assertion_checks": report["assertion_checks"], "output": str(output)}))
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
