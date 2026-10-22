#!/usr/bin/env python3
"""Bind the separate joint-LIVO build to complete sources and dependencies.

This proves build identity, not estimator correctness or simulation acceptance.
No old livo_install or livo_build file is written by this module.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess


def record(path: Path) -> dict:
    data = path.read_bytes()
    result = {"bytes": len(data), "sha256": hashlib.sha256(data).hexdigest()}
    if path.is_symlink():
        result["symlink"] = os.readlink(path)
    return result


def tree(root: Path) -> dict:
    if not root.is_dir():
        raise ValueError(f"required tree missing: {root}")
    result = {}
    for path in sorted(root.rglob("*")):
        if "__pycache__" in path.parts or path.suffix == ".pyc":
            continue
        if path.is_symlink() and path.is_dir():
            raise ValueError(f"unbound directory symlink: {path}")
        if path.is_file():
            result[str(path.relative_to(root))] = record(path)
        elif path.is_symlink():
            raise ValueError(f"dangling dependency/source symlink: {path}")
    return result


def command(arguments: list[str]) -> str:
    return subprocess.run(arguments, check=True, text=True, stdout=subprocess.PIPE,
                          stderr=subprocess.STDOUT).stdout.strip()


def inputs(workspace: Path) -> dict:
    prefix = workspace / "livo_deps"
    dependency_manifest = prefix / "dependency-manifest.json"
    dependency = json.loads(dependency_manifest.read_text())
    if dependency.get("status") != "PASS":
        raise ValueError("dependency manifest is not PASS")
    for name, expected in dependency["installed_files"].items():
        actual = record(prefix / name)
        if {"bytes": actual["bytes"], "sha256": actual["sha256"]} != expected:
            raise ValueError("dependency manifest mismatch: " + name)
    tools = {}
    for name in ("g++", "cmake", "python3", "colcon"):
        executable = shutil.which(name)
        if executable is None:
            raise ValueError("required build tool unavailable: " + name)
        resolved = Path(executable).resolve()
        tools[name] = {"path": str(resolved), **record(resolved)}
    tools["g++"]["version"] = command(["g++", "--version"])
    frontend = Path(command(["g++", "-print-prog-name=cc1plus"])).resolve()
    tools["g++"]["frontend"] = {"path": str(frontend), **record(frontend)}
    tools["cmake"]["version"] = command(["cmake", "--version"])
    tools["python3"]["version"] = command(["python3", "--version"])
    packages = command(["dpkg-query", "-W", "-f=${Package}\t${Version}\t${Architecture}\n"])
    return {
        "source_files": tree(workspace / "src/xq_fast_livo2"),
        "dependencies_manifest": record(dependency_manifest),
        "dependency_installed_files": tree(prefix),
        "base_build_manifest": record(workspace / "xq_install/.xq_build_manifest.json"),
        "base_installed_files": tree(workspace / "xq_install"),
        "build_script": record(workspace / "scripts/build_joint_livo_isolated.sh"),
        "manifest_script": record(workspace / "scripts/check_joint_livo_build_manifest.py"),
        "toolchain": tools,
        "installed_package_database_sha256": hashlib.sha256(packages.encode()).hexdigest(),
        "environment": {key: os.environ.get(key, "") for key in (
            "AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH", "COLCON_PREFIX_PATH",
            "LD_LIBRARY_PATH", "CC", "CXX", "CFLAGS", "CXXFLAGS", "LDFLAGS")},
        "scope": "separate_joint_livo_install_no_hardware_or_algorithm_acceptance_claim",
    }


def protected(workspace: Path) -> dict:
    return {name: tree(workspace / name) for name in ("livo_install", "livo_build")}


def runtime_files(binary: Path) -> dict:
    output = command(["ldd", str(binary)])
    if "not found" in output:
        raise ValueError("installed binary has an unresolved dependency")
    paths = set()
    for line in output.splitlines():
        for word in line.split():
            if word.startswith("/"):
                path = Path(word)
                if path.is_file():
                    paths.add(path.resolve())
    if not paths:
        raise ValueError("ldd did not identify linked runtime files")
    return {str(path): record(path) for path in sorted(paths)}


def write_new(path: Path, content: dict) -> None:
    with path.open("x", encoding="utf-8") as stream:
        json.dump(content, stream, indent=2, allow_nan=False)
        stream.write("\n")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace-root", type=Path, required=True)
    parser.add_argument("--mode", choices=("prepare", "audit", "write", "check"), default="check")
    parser.add_argument("--record-dir", type=Path)
    args = parser.parse_args(argv)
    workspace = args.workspace_root.resolve()
    current = inputs(workspace)
    manifest_path = workspace / "joint_livo_install/.impact-joint-build-manifest.json"
    if args.mode == "prepare":
        if args.record_dir is None or not args.record_dir.is_dir():
            raise ValueError("prepare requires an existing exclusive attempt directory")
        write_new(args.record_dir / "inputs.before.json", current)
        write_new(args.record_dir / "protected.before.json", protected(workspace))
    elif args.mode in ("audit", "write"):
        if args.record_dir is None:
            raise ValueError("write requires --record-dir")
        before = json.loads((args.record_dir / "inputs.before.json").read_text())
        protected_before = json.loads((args.record_dir / "protected.before.json").read_text())
        after_path = args.record_dir / "inputs.after.json"
        if after_path.exists():
            if json.loads(after_path.read_text()) != current:
                raise ValueError("inputs changed after the completed build audit")
        else:
            write_new(after_path, current)
        protected_after = protected(workspace)
        protected_path = args.record_dir / "protected.after.json"
        if protected_path.exists():
            if json.loads(protected_path.read_text()) != protected_after:
                raise ValueError("protected prefixes changed after the completed build audit")
        else:
            write_new(protected_path, protected_after)
        if before != current:
            raise ValueError("joint-LIVO sources/dependencies/toolchain changed during compilation")
        if protected_before != protected_after:
            raise ValueError("old livo_install/livo_build changed during isolated build")
        if args.mode == "write":
            binary = workspace / "joint_livo_install/xq_fast_livo2/lib/xq_fast_livo2/xq_livo_mapping"
            manifest = {
            "schema_version": 1, "status": "PASS", "workspace_root": str(workspace),
            "inputs": current,
            "installed_files": tree(workspace / "joint_livo_install/xq_fast_livo2"),
            "binary": {"path": str(binary), **record(binary)},
            "runtime_libraries": runtime_files(binary),
            "protected_old_livo_install_and_build_unchanged": True,
            }
            write_new(args.record_dir / "build-manifest.json", manifest)
            manifest_path.parent.mkdir(parents=True, exist_ok=True)
            temporary = manifest_path.with_suffix(".json.tmp")
            temporary.write_text(json.dumps(manifest, indent=2, allow_nan=False) + "\n")
            temporary.replace(manifest_path)
    else:
        manifest = json.loads(manifest_path.read_text())
        binary = Path(manifest["binary"]["path"])
        # Runtime setup may prepend the new install prefix.  Build-time
        # environment is strictly equal before/after compilation; later checks
        # bind immutable inputs and the actually resolved runtime library files.
        expected_identity = {key: value for key, value in manifest["inputs"].items() if key != "environment"}
        current_identity = {key: value for key, value in current.items() if key != "environment"}
        if (manifest.get("status") != "PASS" or manifest["workspace_root"] != str(workspace)
                or expected_identity != current_identity
                or manifest["installed_files"] != tree(workspace / "joint_livo_install/xq_fast_livo2")
                or manifest["binary"] != {"path": str(binary), **record(binary)}
                or manifest["runtime_libraries"] != runtime_files(binary)):
            raise ValueError("joint-LIVO build source/dependency/binary mismatch")
    print(json.dumps({"status": "PASS", "mode": args.mode,
                      "source_files": len(current["source_files"]),
                      "manifest": str(manifest_path)}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
