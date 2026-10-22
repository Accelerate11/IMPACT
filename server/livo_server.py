#!/usr/bin/env python3
"""Portable deployment plumbing around the unchanged IMPACT joint-LIVO tools.

No filtering, residual, covariance, gate, input sample or experiment mode is
modified here. Build a fresh native workspace instead of relocating a binary.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import io
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import tarfile

PINS = {
    "Sophus": {"commit": "de0f8d3d92bf776271e16de56d1803940ebccab9",
               "url": "https://github.com/strasdat/Sophus.git"},
    "rpg_vikit": {"commit": "4b7abc838f5d2ca9137f70f122eaaeff9eaf0f50",
                  "url": "https://github.com/Robotic-Developer-Road/rpg_vikit.git"},
}
SCRIPTS = (
    "build_joint_livo_isolated.sh", "run_joint_livo_replay.sh",
    "run_joint_livo_replay.py", "check_joint_livo_build_manifest.py",
    "replay_joint_livo_inputs.py", "prepare_joint_livo_selection.py",
    "prepare_livo_transport.py", "analyze_joint_livo_replay.py",
    "analyze_livo_replay.py",
)
MODES = ("native", "joint_no_flow", "full", "zero_gyro", "zero_plane")


def default_package_root():
    for candidate in Path(__file__).resolve().parents:
        if (candidate / "src/xq_fast_livo2").is_dir() and (candidate / "scripts").is_dir():
            return candidate
    raise ValueError("Cannot locate the package root; provide --root")


def wrapper_relative(package):
    if (package / "server/livo_server.py").is_file():
        return "server"
    return "packaging/server"


def record(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    value = {"bytes": path.stat().st_size, "sha256": digest.hexdigest()}
    if path.is_symlink():
        value["symlink"] = os.readlink(path)
    return value


def tree(path, exclude=()):
    if not path.is_dir():
        raise ValueError("Missing required directory: " + str(path))
    result = {}
    for entry in sorted(path.rglob("*")):
        relative = entry.relative_to(path).as_posix()
        if any(part in (".git", "__pycache__", ".pytest_cache") for part in entry.parts):
            continue
        if relative in exclude or entry.suffix in (".pyc", ".pyo"):
            continue
        if entry.is_symlink() and entry.is_dir():
            raise ValueError("Directory symlinks are not portable source identity: " + str(entry))
        if entry.is_file():
            result[relative] = record(entry)
        elif entry.is_symlink():
            raise ValueError("Dangling symlink: " + str(entry))
    return result


def write_new(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write("\n")


def call(arguments, log, env=None):
    log.parent.mkdir(parents=True, exist_ok=True)
    with log.open("x", encoding="utf-8") as stream:
        stream.write(json.dumps({"command": [str(x) for x in arguments]}) + "\n")
        stream.flush()
        subprocess.run([str(x) for x in arguments], stdout=stream,
                       stderr=subprocess.STDOUT, env=env, check=True)


def source_identity(package):
    result = {}
    for relative in ("src/xq_fast_livo2", "src/xq_livox_interfaces",
                     "config/livo_dependency_patches"):
        for name, value in tree(package / relative).items():
            result[relative + "/" + name] = value
    for name in SCRIPTS:
        result["scripts/" + name] = record(package / "scripts" / name)
    wrapper = wrapper_relative(package)
    for relative in ("config/research_joint_livo_replay.json",
                     wrapper + "/livo_server.py",
                     wrapper + "/build_livo_server.sh",
                     wrapper + "/run_livo_server.sh"):
        result[relative] = record(package / relative)
    return result


def export_vendor(args):
    output = args.output.resolve()
    if output.exists():
        raise ValueError("Vendor output must be a new directory")
    output.mkdir(parents=True)
    repositories = {"Sophus": args.sophus_repository, "rpg_vikit": args.vikit_repository}
    manifest = {"schema_version": 1, "status": "PASS", "sources": {}}
    for name, pin in PINS.items():
        repository = repositories[name].resolve()
        origin = subprocess.check_output(["git", "-C", str(repository), "remote", "get-url", "origin"], text=True).strip()
        if origin != pin["url"]:
            raise ValueError("Unexpected vendor origin: " + origin)
        # git archive reads the pinned object, ignoring patched/dirty worktrees.
        blob = subprocess.check_output(["git", "-C", str(repository), "archive", pin["commit"]])
        destination = output / name
        destination.mkdir()
        with tarfile.open(fileobj=io.BytesIO(blob), mode="r:") as archive:
            for member in archive.getmembers():
                path = destination / member.name
                resolved = path.resolve()
                if destination != resolved and destination not in resolved.parents:
                    raise ValueError("Vendor archive path escapes output")
                if member.isdir():
                    path.mkdir(parents=True, exist_ok=True)
                elif member.isfile():
                    path.parent.mkdir(parents=True, exist_ok=True)
                    with archive.extractfile(member) as stream:
                        path.write_bytes(stream.read())
                else:
                    raise ValueError("Vendor archive contains a nonregular member")
        manifest["sources"][name] = dict(pin, files=tree(destination))
    write_new(output / "vendor-manifest.json", manifest)
    print(json.dumps({"status": "PASS", "vendor_root": str(output)}))


def verify_vendor(vendor):
    manifest_path = vendor / "vendor-manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("status") != "PASS" or set(manifest.get("sources", {})) != set(PINS):
        raise ValueError("Invalid vendor manifest")
    for name, pin in PINS.items():
        declared = manifest["sources"][name]
        if any(declared.get(key) != value for key, value in pin.items()):
            raise ValueError("Vendor revision/origin mismatch: " + name)
        if declared["files"] != tree(vendor / name):
            raise ValueError("Vendor source set/bytes/SHA256 mismatch: " + name)
    return record(manifest_path)


def stage(package, runtime):
    if runtime.exists() or runtime.is_symlink():
        raise ValueError("Use a new runtime root; existing attempts are preserved: " + str(runtime))
    before = source_identity(package)
    runtime.mkdir(parents=True)
    workspace = runtime / "workspace"
    workspace.mkdir()
    for relative in ("src/xq_fast_livo2", "src/xq_livox_interfaces",
                     "config/livo_dependency_patches"):
        shutil.copytree(package / relative, workspace / relative,
                        ignore=shutil.ignore_patterns("__pycache__", "*.pyc", ".git"))
    (workspace / "scripts").mkdir()
    for name in SCRIPTS:
        shutil.copy2(package / "scripts" / name, workspace / "scripts" / name)
    shutil.copy2(package / "config/research_joint_livo_replay.json", workspace / "config")
    # The original isolated checker protects these trees. On a new server they
    # are genuinely empty, not placeholders for an omitted historical build.
    for name in ("livo_build", "livo_install"):
        (workspace / name).mkdir()
    if before != source_identity(package):
        raise ValueError("Package source changed while staging")
    copied = {name: record(workspace / name) for name in before
              if not name.startswith(("packaging/", "server/"))}
    if copied != {name: value for name, value in before.items()
                  if not name.startswith(("packaging/", "server/"))}:
        raise ValueError("Staged source bytes differ from the package")
    write_new(runtime / "stage-manifest.json", {
        "schema_version": 1, "status": "PASS", "package_root": str(package),
        "workspace_root": str(workspace), "sources": before,
        "staged_source_files": copied,
        "scope": "Unchanged native algorithms and original replay/build tools; new deployment-only wrapper.",
    })
    return workspace


def check_stage(package, runtime):
    manifest = json.loads((runtime / "stage-manifest.json").read_text())
    workspace = runtime / "workspace"
    if (manifest.get("status") != "PASS" or manifest["package_root"] != str(package)
            or manifest["workspace_root"] != str(workspace)
            or manifest["sources"] != source_identity(package)):
        raise ValueError("Package changed after staging; use a fresh runtime root")
    if manifest["staged_source_files"] != {name: record(workspace / name)
                                           for name in manifest["staged_source_files"]}:
        raise ValueError("Staged source changed after copying")
    # Exact file sets are checked as well as each originally copied file.
    current = {}
    for relative in ("src/xq_fast_livo2", "src/xq_livox_interfaces",
                     "config/livo_dependency_patches"):
        current.update({relative + "/" + name: value for name, value in tree(workspace / relative).items()})
    expected = {name: value for name, value in manifest["staged_source_files"].items()
                if name.startswith(("src/", "config/livo_dependency_patches/"))}
    if current != expected:
        raise ValueError("Staged source file set changed")
    result_path = runtime / "server-build-result.json"
    if result_path.is_file():
        built = json.loads(result_path.read_text())
        if (built.get("status") != "PASS"
                or verify_vendor(Path(built["vendor_root"])) != built["vendor_manifest"]):
            raise ValueError("Dependency source identity changed after the server build")
    return workspace


def build_dependencies(package, runtime, vendor, jobs):
    vendor_record = verify_vendor(vendor)
    workspace = runtime / "workspace"
    attempt = runtime / "dependency-build"
    attempt.mkdir()
    prefix = workspace / "livo_deps"
    prefix.mkdir()
    (prefix / ".impact-livo-dependencies-owned").touch()
    for name in PINS:
        shutil.copytree(vendor / name, attempt / name)
    wrapper = attempt / "wrapper"
    shutil.copytree(package / "config/livo_dependency_patches", wrapper)
    patch_environment = os.environ.copy()
    # A source package can itself live inside a larger Git repository. Prevent
    # git apply from discovering that unrelated parent when applying to a clean
    # git-archive export with no .git directory of its own.
    patch_environment["GIT_CEILING_DIRECTORIES"] = str(attempt)
    for patch in sorted(wrapper.glob("*.patch")):
        for extra in (["--check"], []):
            call(["git", "-C", attempt / "rpg_vikit", "apply", *extra, patch],
                 attempt / (patch.stem + ("-check.log" if extra else "-apply.log")), patch_environment)
    env = os.environ.copy()
    env["CMAKE_PREFIX_PATH"] = str(prefix) + ":" + env.get("CMAKE_PREFIX_PATH", "")
    env["LD_LIBRARY_PATH"] = str(prefix / "lib") + ":" + env.get("LD_LIBRARY_PATH", "")
    common = ["-DCMAKE_BUILD_TYPE=Release", "-DCMAKE_INSTALL_PREFIX=" + str(prefix),
              "-DCMAKE_EXPORT_NO_PACKAGE_REGISTRY=ON", "-DCMAKE_EXPORT_PACKAGE_REGISTRY=OFF",
              "-DCMAKE_FIND_USE_PACKAGE_REGISTRY=OFF", "-DCMAKE_FIND_USE_SYSTEM_PACKAGE_REGISTRY=OFF"]
    targets = [
        ("Sophus", attempt / "Sophus", ["-DSOPHUS_INSTALL=ON", "-DBUILD_SOPHUS_TESTS=OFF", "-DBUILD_SOPHUS_EXAMPLES=OFF"]),
        ("vikit", wrapper, ["-DVIKIT_SOURCE_DIR=" + str(attempt / "rpg_vikit"),
                            "-DSophus_DIR=" + str(prefix / "share/sophus/cmake")]),
    ]
    for name, source, options in targets:
        build = attempt / ("build-" + name)
        call(["cmake", "-S", source, "-B", build, *common, *options], attempt / (name + "-configure.log"), env)
        call(["cmake", "--build", build, "--parallel", str(jobs)], attempt / (name + "-build.log"), env)
        call(["cmake", "--install", build], attempt / (name + "-install.log"), env)
    licenses = prefix / "share/impact_livo_dependencies/licenses"
    licenses.mkdir(parents=True, exist_ok=True)
    shutil.copy2(attempt / "Sophus/LICENSE.txt", licenses / "Sophus-LICENSE.txt")
    shutil.copy2(attempt / "rpg_vikit/README.md", licenses / "rpg_vikit-README.upstream.md")
    consumer = attempt / "consumer"
    consumer.mkdir()
    (consumer / "CMakeLists.txt").write_text('''cmake_minimum_required(VERSION 3.16)
project(impact_server_dependency_probe LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
find_package(Sophus 1.22.10 EXACT REQUIRED CONFIG)
find_package(vikit_common REQUIRED CONFIG)
find_package(vikit_ros REQUIRED CONFIG)
add_executable(dependency_probe probe.cpp)
target_link_libraries(dependency_probe PRIVATE Sophus::Sophus vikit_common::vikit_common vikit_ros::vikit_ros)
''', encoding="utf-8")
    shutil.copy2(wrapper / "camera_loader_probe.cpp", consumer / "probe.cpp")
    call(["cmake", "-S", consumer, "-B", attempt / "build-consumer", *common], attempt / "probe-configure.log", env)
    call(["cmake", "--build", attempt / "build-consumer", "--parallel", str(jobs)], attempt / "probe-build.log", env)
    env.update(ROS_LOCALHOST_ONLY="1", ROS_DOMAIN_ID=str(210 + os.getpid() % 20))
    for mode in ("delayed", "missing", "wrong-type"):
        call([attempt / "build-consumer/dependency_probe", mode], attempt / ("probe-" + mode + ".log"), env)
    if record(vendor / "vendor-manifest.json") != vendor_record:
        raise ValueError("Vendor manifest changed during compilation")
    verify_vendor(vendor)
    write_new(prefix / "dependency-manifest.json", {
        "schema_version": 1, "status": "PASS", "sources": PINS,
        "vendor_manifest": vendor_record,
        "wrapper_and_patch_sha256": tree(wrapper),
        # Match the unchanged original checker's dependency-manifest schema.
        # Its separate full prefix tree also binds symlink targets, if present.
        "installed_files": {name: {"bytes": value["bytes"], "sha256": value["sha256"]}
                            for name, value in tree(prefix).items()},
        "consumer_probes": ["delayed", "missing", "wrong-type"],
        "scope": "Server-native dependency rebuild; no numerical algorithm changes.",
    })


def build(args, package, runtime):
    if platform.system() != "Linux" or platform.machine() not in ("x86_64", "amd64"):
        raise ValueError("Native replay build target is Ubuntu 22.04 x86_64 Linux")
    if not Path("/opt/ros/humble/setup.bash").is_file() or not os.environ.get("AMENT_PREFIX_PATH"):
        raise ValueError("Invoke build_livo_server.sh so ROS 2 Humble is sourced")
    vendor = (args.vendor_root or package / "third_party/livo").resolve()
    verify_vendor(vendor)
    workspace = stage(package, runtime)
    logs = runtime / "logs"
    command = ["colcon", "--log-base", workspace / "xq_log", "build",
               "--base-paths", workspace / "src/xq_livox_interfaces",
               "--build-base", workspace / "xq_build", "--install-base", workspace / "xq_install",
               "--executor", "sequential", "--event-handlers", "console_direct+",
               "--cmake-args", "-DCMAKE_BUILD_TYPE=Release"]
    call(command, logs / "livox-message-build.log")
    check_stage(package, runtime)
    write_new(workspace / "xq_install/.xq_build_manifest.json", {
        "schema_version": 1, "status": "PASS", "contract": "server_minimal_livox_interface_build",
        "scope": "Only xq_livox_interfaces was built; not the full Gazebo/bringup workspace.",
        "source_files": tree(workspace / "src/xq_livox_interfaces"),
        "installed_files": tree(workspace / "xq_install"),
        "command": [str(x) for x in command],
    })
    build_dependencies(package, runtime, vendor, args.jobs)
    call(["bash", workspace / "scripts/build_joint_livo_isolated.sh"], logs / "native-joint-build.log")
    check_stage(package, runtime)
    write_new(runtime / "server-build-result.json", {
        "schema_version": 1, "status": "PASS", "utc": datetime.now(timezone.utc).isoformat(),
        "stage_manifest": record(runtime / "stage-manifest.json"),
        "vendor_root": str(vendor), "vendor_manifest": verify_vendor(vendor),
        "native_manifest": record(workspace / "joint_livo_install/.impact-joint-build-manifest.json"),
        "binary": record(workspace / "joint_livo_install/xq_fast_livo2/lib/xq_fast_livo2/xq_livo_mapping"),
        "original_build_and_manifest_checker_executed_unchanged": True,
        "hardware_or_accuracy_acceptance": False,
    })
    print(json.dumps({"status": "PASS", "runtime_root": str(runtime)}))


def run_action(args, package, runtime):
    workspace = check_stage(package, runtime)
    if args.command == "check":
        command = ["bash", "-c", 'set -e; set +u; source /opt/ros/humble/setup.bash; source "$1/xq_install/setup.bash"; source "$1/joint_livo_install/setup.bash"; export LD_LIBRARY_PATH="$1/livo_deps/lib:${LD_LIBRARY_PATH:-}"; exec python3 "$1/scripts/check_joint_livo_build_manifest.py" --workspace-root "$1"', "impact-check", str(workspace)]
        subprocess.run(command, check=True)
    elif args.command == "prepare":
        bag = (args.bag or package / "data/b1/rosbag").resolve()
        output = (args.output or runtime / "inputs/original").resolve()
        subprocess.run(["python3", str(workspace / "scripts/replay_joint_livo_inputs.py"), "prepare",
                        "--bag", str(bag), "--output", str(output)], check=True)
    elif args.command == "replay":
        index = (args.index or runtime / "inputs/original/sensor-index.json").resolve()
        command = ["bash", str(workspace / "scripts/run_joint_livo_replay.sh"),
                   "--index", str(index), "--output", str(args.output.resolve())]
        if args.modes:
            command.extend(["--modes", *args.modes])
        subprocess.run(command, check=True)
    elif args.command == "evaluate":
        truth = (args.truth or runtime / "inputs/original/evaluator/truth.jsonl").resolve()
        subprocess.run(["python3", str(workspace / "scripts/analyze_joint_livo_replay.py"),
                        "--run", str(args.run.resolve()), "--truth", str(truth),
                        "--output", str(args.output.resolve())], check=True)
    check_stage(package, runtime)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    vendor = commands.add_parser("vendor-export", help="Export clean pinned Git objects, not a patched worktree")
    vendor.add_argument("--sophus-repository", type=Path, required=True)
    vendor.add_argument("--vikit-repository", type=Path, required=True)
    vendor.add_argument("--output", type=Path, required=True)
    verify = commands.add_parser("vendor-verify")
    verify.add_argument("--vendor-root", type=Path, required=True)
    for name in ("build", "check", "prepare", "replay", "evaluate"):
        command = commands.add_parser(name)
        command.add_argument("--root", "--package-root", dest="package_root", type=Path,
                             default=default_package_root())
        command.add_argument("--runtime-root", type=Path)
        if name == "build":
            command.add_argument("--vendor-root", type=Path)
            command.add_argument("--jobs", type=int, choices=(1, 2), default=1)
        if name == "prepare":
            command.add_argument("--bag", type=Path)
            command.add_argument("--output", type=Path)
        if name == "replay":
            command.add_argument("--index", type=Path)
            command.add_argument("--output", type=Path, required=True)
            command.add_argument("--modes", choices=MODES, nargs="+")
        if name == "evaluate":
            command.add_argument("--run", type=Path, required=True)
            command.add_argument("--truth", type=Path)
            command.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    if args.command == "vendor-export":
        export_vendor(args)
    elif args.command == "vendor-verify":
        print(json.dumps({"status": "PASS", "manifest": verify_vendor(args.vendor_root.resolve())}))
    else:
        package = args.package_root.resolve()
        runtime = (args.runtime_root or package / "server_runtime/livo").resolve()
        if args.command == "build":
            build(args, package, runtime)
        else:
            run_action(args, package, runtime)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, KeyError, subprocess.CalledProcessError) as error:
        print(json.dumps({"status": "FAIL", "error": type(error).__name__ + ": " + str(error)}))
        raise SystemExit(1)
