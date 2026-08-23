#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
workspace_root="$(cd -- "${script_dir}/.." && pwd -P)"

if [[ ! -f "${workspace_root}/src/xq_sim_bringup/package.xml" ]] ||
   [[ ! -f "${workspace_root}/src/xq_gz_assets/package.xml" ]]; then
  echo "Refusing to build: ${workspace_root} is not the XQ isolated workspace." >&2
  exit 2
fi
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble was not found at /opt/ros/humble." >&2
  exit 2
fi
if ! grep -qi microsoft /proc/version 2>/dev/null; then
  echo "This build helper is intended for the configured WSL environment." >&2
  exit 2
fi

# Do not inherit overlays or Gazebo model paths from other projects.
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
unset GZ_SIM_RESOURCE_PATH IGN_GAZEBO_RESOURCE_PATH SDF_PATH
set +u
source /opt/ros/humble/setup.bash
set -u

cd -- "${workspace_root}"
colcon --log-base "${workspace_root}/xq_log" build \
  --base-paths "${workspace_root}/src" \
  --build-base "${workspace_root}/xq_build" \
  --install-base "${workspace_root}/xq_install" \
  --packages-up-to xq_sim_bringup \
  --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

# Record the exact source and installed trees that this successful build
# produced.  run_smoke.sh recomputes both hashes and refuses to launch an
# installation that no longer matches this manifest.  Python caches are
# excluded because importing an installed module may create them after build.
python3 - "${workspace_root}" <<'PY'
from __future__ import annotations

import hashlib
import json
import os
import sys
from datetime import datetime, timezone
from pathlib import Path


workspace = Path(sys.argv[1]).resolve()
source_root = workspace / "src"
install_root = workspace / "xq_install"
manifest_path = install_root / ".xq_build_manifest.json"


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


source_files = collect(source_root)
install_files = collect(install_root, {manifest_path})
manifest = {
    "schema_version": 1,
    "contract": "xq_isolated_build_manifest",
    "generated_at_utc": datetime.now(timezone.utc).isoformat(),
    "workspace_root": str(workspace),
    "source_root": str(source_root),
    "install_root": str(install_root),
    "source_tree_sha256": tree_digest(source_files),
    "install_tree_sha256": tree_digest(install_files),
    "source_files": source_files,
    "install_files": install_files,
}
temporary = manifest_path.with_suffix(".json.tmp")
temporary.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
temporary.replace(manifest_path)
print(f"Source tree SHA-256:  {manifest['source_tree_sha256']}")
print(f"Install tree SHA-256: {manifest['install_tree_sha256']}")
print(f"Build manifest:        {manifest_path}")
PY

echo "Built only the XQ workspace at ${workspace_root}/xq_install."
