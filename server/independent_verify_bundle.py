#!/usr/bin/env python3
"""Read-only, standard-library audit of an IMPACT server bundle.

This checks source/data/asset identity and coverage, not numerical correctness,
successful compilation, simulator performance, or flight safety. No SQLite is
opened and no ROS, AP, or Gazebo process is started.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path, PurePosixPath
import re
import sys
import xml.etree.ElementTree as ET


def strict_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("Duplicate JSON key: " + key)
        result[key] = value
    return result


def load_json(path):
    return json.loads(path.read_text(encoding="utf-8"), object_pairs_hook=strict_object)


def digest(path):
    hasher = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            hasher.update(block)
    return hasher.hexdigest()


def relative_path(text):
    value = PurePosixPath(text)
    if not text or value.is_absolute() or ".." in value.parts or "\\" in text:
        raise ValueError("Unsafe relative manifest path: " + repr(text))
    return value


def safe_file(root, relative):
    relative = relative_path(relative)
    root = root.absolute()
    candidate = root.joinpath(*relative.parts)
    if any(path.is_symlink() for path in (root, *root.parents)):
        raise ValueError("Aliased root is forbidden: " + str(root))
    current = root
    for part in relative.parts:
        current = current / part
        if current.is_symlink():
            raise ValueError("Symlink is forbidden: " + str(current))
    if not candidate.is_file():
        raise ValueError("Missing regular file: " + str(candidate))
    return candidate


def check_record(root, record, path_key="path"):
    try:
        path = safe_file(root, record[path_key])
        size = path.stat().st_size
        actual = digest(path)
        identity = "original" if (size, actual) == (record["bytes"], record["sha256"]) else None
        if identity is None and "linux_lf_bytes" in record and "linux_lf_sha256" in record:
            if (size, actual) == (record["linux_lf_bytes"], record["linux_lf_sha256"]):
                identity = "explicit_linux_lf"
        return {"path": record[path_key], "passed": identity is not None,
                "accepted_identity": identity, "bytes": size, "sha256": actual}
    except (KeyError, OSError, ValueError) as error:
        return {"path": record.get(path_key), "passed": False, "error": str(error)}


def validate_manifest(manifest):
    if manifest.get("schema_version") != 1 or manifest.get("kind") != "IMPACT_SERVER_ALGORITHM_COVERAGE":
        raise ValueError("Unsupported coverage manifest schema")
    paths = []
    for record in manifest["source_files"] + manifest["serial_prototype"]["source_files"]:
        relative_path(record["path"])
        if type(record["bytes"]) is not int or record["bytes"] < 0:
            raise ValueError("Invalid manifest file size")
        if re.fullmatch("[0-9a-f]{64}", record["sha256"]) is None:
            raise ValueError("Invalid manifest SHA256")
        if "linux_lf_bytes" in record or "linux_lf_sha256" in record:
            if (type(record.get("linux_lf_bytes")) is not int or record["linux_lf_bytes"] < 0
                    or not isinstance(record.get("linux_lf_sha256"), str)
                    or re.fullmatch("[0-9a-f]{64}", record["linux_lf_sha256"]) is None):
                raise ValueError("Invalid explicit Linux LF identity")
    for record in manifest["source_files"]:
        paths.append(record["path"])
    if len(paths) != len(set(paths)):
        raise ValueError("Duplicate source file identity")
    if len(manifest["top_level_components"]) != 10 or len(manifest["ros_packages"]) != 15:
        raise ValueError("Expected ten top-level components and fifteen ROS packages")
    for group in manifest["algorithm_groups"]:
        if not group["required_paths"] or any(path not in paths for path in group["required_paths"]):
            raise ValueError("Coverage group references an unindexed source: " + group["id"])


def absolute_references(root, external, records):
    """Expose host-specific references without pretending to rewrite programs.

    A missing historical audit input is distinct from an unresolved required
    runtime model, which the asset checks already fail. These findings remain
    visible even when all immutable-source checks pass.
    """
    prefixes = {
        "/home/accelerate/impact_research_ws": root,
        "/home/accelerate/ardupilot_gazebo": external,
        "/home/accelerate/ardupilot": root / "third_party/ardupilot",
        "/mnt/d/CODEX/impact": root,
    }
    pattern = re.compile(r"/(?:home/accelerate|mnt/d/CODEX/impact)[A-Za-z0-9_./+@-]*")
    result = []
    for record in records:
        if not record["passed"] or Path(record["path"]).suffix not in {".py", ".sh", ".json", ".yaml", ".yml"}:
            continue
        path = safe_file(root, record["path"])
        for number, line in enumerate(path.read_text(encoding="utf-8", errors="replace").splitlines(), 1):
            for original in dict.fromkeys(pattern.findall(line)):
                replacement = None
                for prefix, base in prefixes.items():
                    if original == prefix or original.startswith(prefix + "/"):
                        replacement = base / original[len(prefix):].lstrip("/")
                        break
                result.append({"path": record["path"], "line": number, "original_reference": original,
                               "suggested_relocated_reference": str(replacement) if replacement is not None else None,
                               "relocated_reference_exists": replacement.exists() if replacement is not None else False,
                               "classification": "host_reference_requires_reviewed_launcher_not_automatic_rewrite"})
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, required=True, help="Bundle root containing src/scripts/config")
    parser.add_argument("--coverage-manifest", type=Path, default=Path(__file__).with_name("coverage_manifest.json"))
    parser.add_argument("--external-assets", type=Path, help="ardupilot_gazebo snapshot root (with models/worlds)")
    parser.add_argument("--raw-bag", type=Path, help="B1 original bag directory; default data/b1/rosbag")
    parser.add_argument("--prototype-dir", type=Path, help="Serial prototype source root; default prototypes/serial_position")
    parser.add_argument("--skip-rawdata", action="store_true", help="Report a limited source-only audit without checking 1.5GB raw data")
    parser.add_argument("--output", type=Path, help="Exclusive new JSON report; never overwritten")
    args = parser.parse_args(argv)
    if args.output is not None and (args.output.exists() or args.output.is_symlink()):
        parser.error("Output already exists; use a new path")
    root = args.workspace.absolute()
    manifest = load_json(args.coverage_manifest)
    validate_manifest(manifest)
    checks = {}
    records = [check_record(root, item) for item in manifest["source_files"]]
    checks["all_597_source_files_size_and_sha256"] = len(records) == 597 and all(item["passed"] for item in records)
    actual_source_set, source_aliases = set(), []
    exclusions = set(manifest["source_exclusions"])
    for family in ("src", "scripts", "config"):
        base = root / family
        if base.is_symlink():
            source_aliases.append(family)
            continue
        for path in base.rglob("*"):
            relative = path.relative_to(root)
            if any(part in exclusions for part in relative.parts):
                continue
            if path.is_symlink():
                source_aliases.append(relative.as_posix())
            elif path.is_file():
                actual_source_set.add(relative.as_posix())
    expected_source_set = {item["path"] for item in manifest["source_files"]}
    checks["source_exact_file_set_without_aliases"] = not source_aliases and actual_source_set == expected_source_set
    packages = []
    for item in manifest["ros_packages"]:
        try:
            path = safe_file(root, item["path"])
            node = ET.parse(path).getroot()
            passed = node.findtext("name") == item["name"] and [x.text for x in node.findall("license")] == item["license"]
            packages.append({"name": item["name"], "passed": passed})
        except (OSError, ValueError, ET.ParseError) as error:
            packages.append({"name": item["name"], "passed": False, "error": str(error)})
    checks["all_15_ros_package_names_and_license_declarations"] = all(item["passed"] for item in packages)
    groups = []
    indexed = {item["path"]: item["passed"] for item in records}
    for group in manifest["algorithm_groups"]:
        groups.append({"id": group["id"], "passed": all(indexed.get(path, False) for path in group["required_paths"]),
                       "experimental_scope": group["experimental_scope"]})
    checks["every_declared_algorithm_group_present_and_identical"] = all(item["passed"] for item in groups)
    prototype = args.prototype_dir or root / "prototypes/serial_position"
    prototype_records = [check_record(prototype, {**item, "path": Path(item["path"]).name})
                         for item in manifest["serial_prototype"]["source_files"]]
    checks["serial_prototype_four_sources_present_and_identical"] = len(prototype_records) == 4 and all(item["passed"] for item in prototype_records)

    external = args.external_assets or root / "third_party/ardupilot_gazebo"
    asset_records = []
    asset_set_checks = {}
    for family in manifest["external_assets"]:
        if not family["required_for_bundle"]:
            continue
        subdir = family["bundle_subdir"]
        expected = {item["relative_path"] for item in family["files"]}
        target = external / subdir
        actual, aliases = set(), []
        if target.is_dir() and not target.is_symlink():
            for path in target.rglob("*"):
                if path.is_symlink():
                    aliases.append(str(path.relative_to(target)))
                elif path.is_file():
                    actual.add(path.relative_to(target).as_posix())
        asset_set_checks[subdir] = not aliases and actual == expected
        for item in family["files"]:
            asset_records.append(check_record(target, item, "relative_path"))
    checks["external_models_worlds_exact_file_sets_no_aliases"] = bool(asset_set_checks) and all(asset_set_checks.values())
    checks["external_models_worlds_sizes_sha256"] = bool(asset_records) and all(item["passed"] for item in asset_records)
    license_record = check_record(external, manifest["external_asset_license"])
    checks["external_asset_license_identity"] = license_record["passed"]

    model_roots = [root / "src/xq_gz_assets/models", external / "models"]
    sdf_results = []
    for item in manifest["sdf_assets"]:
        try:
            path = safe_file(root, item["path"])
            node = ET.parse(path).getroot()
            unresolved = []
            for uri in node.findall(".//uri"):
                text = uri.text or ""
                if text.startswith("model://"):
                    rest = text[len("model://"):]
                    candidate = relative_path(rest)
                    if not any((folder / candidate.as_posix()).exists() for folder in model_roots):
                        unresolved.append(text)
                elif text.startswith(("http://", "https://", "file://", "/")):
                    unresolved.append(text)
            sdf_results.append({"path": item["path"], "passed": not unresolved, "unresolved_uris": unresolved})
        except (ValueError, OSError, ET.ParseError) as error:
            sdf_results.append({"path": item["path"], "passed": False, "error": str(error)})
    for name in ("iris_with_ardupilot", "iris_with_standoffs"):
        try:
            path = safe_file(external, "models/" + name + "/model.sdf")
            node = ET.parse(path).getroot()
            missing = [uri.text for uri in node.findall(".//uri")
                       if uri.text and uri.text.startswith("model://")
                       and not any((folder / relative_path(uri.text[8:]).as_posix()).exists() for folder in model_roots)]
            sdf_results.append({"path": "third_party/ardupilot_gazebo/models/" + name + "/model.sdf",
                                "passed": not missing, "unresolved_uris": missing})
        except (ValueError, OSError, ET.ParseError) as error:
            sdf_results.append({"path": name, "passed": False, "error": str(error)})
    checks["all_sdf_xml_and_model_uri_dependencies_resolve"] = all(item["passed"] for item in sdf_results)

    raw_records = []
    raw_set = None
    if not args.skip_rawdata:
        raw = args.raw_bag or root / "data/b1/rosbag"
        expected = set(manifest["raw_replay_data"]["source_files"])
        actual, aliases = set(), []
        if raw.is_dir() and not raw.is_symlink():
            for path in raw.iterdir():
                if path.is_symlink() or not path.is_file():
                    aliases.append(path.name)
                else:
                    actual.add(path.name)
        raw_set = not aliases and actual == expected
        for name, item in manifest["raw_replay_data"]["source_files"].items():
            raw_records.append(check_record(raw, {"path": name, **item}))
        checks["original_B1_bag_exact_file_set_without_sqlite_sidecars"] = raw_set
        checks["original_B1_bag_all_bytes_sha256"] = all(item["passed"] for item in raw_records)
    status = "PASS" if all(checks.values()) else "FAIL"
    report = {
        "schema_version": 1, "status": status, "kind": "STATIC_SERVER_BUNDLE_COVERAGE_AUDIT",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(), "workspace": str(root),
        "coverage_manifest_sha256": digest(args.coverage_manifest),
        "checks": checks, "failed_checks": [name for name, passed in checks.items() if not passed],
        "source_files": records, "ros_packages": packages, "algorithm_groups": groups,
        "source_file_set_difference": {"unexpected": sorted(actual_source_set - expected_source_set),
                                       "missing": sorted(expected_source_set - actual_source_set), "aliases": source_aliases},
        "prototype_sources": prototype_records, "external_asset_file_sets": asset_set_checks,
        "external_asset_license": license_record,
        "external_asset_files": asset_records, "sdf": sdf_results, "raw_files": raw_records,
        "rawdata_checked": not args.skip_rawdata,
        "absolute_dependency_references": absolute_references(root, external, records),
        "portable_entrypoints_verified": False,
        "portability_findings": manifest["portability_findings"],
        "runtime_or_compilation_validated": False, "algorithm_superiority_validated": False,
        "scope": "Read-only static file/algorithm coverage audit. Historical absolute paths still require declared launch adapters; PASS is not proof that every entrypoint runs. GPU sensor rendering, dependencies and runtime remain separate gates.",
    }
    if args.output is not None:
        with args.output.open("x", encoding="utf-8") as handle:
            json.dump(report, handle, indent=2, ensure_ascii=False, allow_nan=False)
            handle.write("\n")
    print(json.dumps({"status": status, "failed_checks": report["failed_checks"],
                      "source_files": len(records), "algorithm_groups": len(groups),
                      "rawdata_checked": report["rawdata_checked"],
                      "output": str(args.output) if args.output else None}))
    return 0 if status == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())

