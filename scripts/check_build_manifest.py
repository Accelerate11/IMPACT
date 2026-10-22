#!/usr/bin/env python3
"""Verify an isolated XQ build without changing its manifest or rebuilding it."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
from pathlib import Path


def file_digest(path: Path) -> str:
    """Match build_isolated.sh and run_smoke.sh, including symlink records."""
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


def check_build_manifest(workspace_root: Path) -> tuple[str, str]:
    workspace = workspace_root.resolve()
    source_root = workspace / "src"
    install_root = workspace / "xq_install"
    manifest_path = install_root / ".xq_build_manifest.json"
    try:
        expected = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        raise ValueError(f"Invalid build manifest {manifest_path}: {exc}") from exc
    if not isinstance(expected, dict):
        raise ValueError(f"Invalid build manifest {manifest_path}: expected a JSON object")
    if expected.get("contract") != "xq_isolated_build_manifest":
        raise ValueError("Build manifest has an unknown contract")

    # Check roots before traversing either tree. A copied manifest must not
    # certify another workspace's build, even when its file hashes are equal.
    for field, current in (
        ("workspace_root", workspace),
        ("source_root", source_root),
        ("install_root", install_root),
    ):
        declared = expected.get(field)
        if (
            not isinstance(declared, str)
            or not Path(declared).is_absolute()
            or Path(declared).resolve() != current
        ):
            raise ValueError(
                f"Build manifest {field} mismatch: built={declared!r}, current={current}"
            )
        if not current.is_dir():
            raise ValueError(f"Build directory is missing: {current}")

    source_files = collect(source_root)
    install_files = collect(install_root, {manifest_path})
    actual_source = tree_digest(source_files)
    actual_install = tree_digest(install_files)
    problems: list[str] = []
    if actual_source != expected.get("source_tree_sha256"):
        problems.append(
            f"source tree is stale (built={expected.get('source_tree_sha256')}, "
            f"current={actual_source})"
        )
    if actual_install != expected.get("install_tree_sha256"):
        problems.append(
            f"install tree changed (built={expected.get('install_tree_sha256')}, "
            f"current={actual_install})"
        )
    if source_files != expected.get("source_files"):
        problems.append("source file manifest differs")
    if install_files != expected.get("install_files"):
        problems.append("installed file manifest differs")
    if problems:
        raise ValueError("Refusing an out-of-date or modified xq_install:\n  - " + "\n  - ".join(problems))
    return actual_source, actual_install


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--workspace-root",
        type=Path,
        default=Path(__file__).resolve().parent.parent,
        help="workspace to check (default: parent of this script's directory)",
    )
    args = parser.parse_args(argv)
    try:
        source_hash, install_hash = check_build_manifest(args.workspace_root)
    except (OSError, ValueError) as exc:
        print(f"Build freshness FAIL: {exc}", file=sys.stderr)
        print("Rebuild explicitly with scripts/build_isolated.sh; no files were changed.", file=sys.stderr)
        return 1
    print(f"Build freshness PASS: source={source_hash}, install={install_hash}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
