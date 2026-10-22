"""CLI regressions for build freshness and workspace isolation."""

from __future__ import annotations

import hashlib
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "check_build_manifest.py"


class BuildManifestTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.workspace = Path(self.temporary.name).resolve()
        (self.workspace / "src").mkdir()
        (self.workspace / "xq_install").mkdir()
        self.source = self.workspace / "src" / "node.py"
        self.installed = self.workspace / "xq_install" / "node.py"
        self.source.write_bytes(b"source v1\n")
        self.installed.write_bytes(b"installed v1\n")
        self.manifest_path = self.workspace / "xq_install" / ".xq_build_manifest.json"
        # Construct the legacy manifest format from known fixture records,
        # without using the checker to generate its own expected answers.
        source_records = [self.record("node.py", b"source v1\n")]
        install_records = [self.record("node.py", b"installed v1\n")]
        self.manifest = {
            "schema_version": 1,
            "contract": "xq_isolated_build_manifest",
            "workspace_root": str(self.workspace),
            "source_root": str(self.workspace / "src"),
            "install_root": str(self.workspace / "xq_install"),
            "source_tree_sha256": self.tree_hash(source_records),
            "install_tree_sha256": self.tree_hash(install_records),
            "source_files": source_records,
            "install_files": install_records,
        }
        self.write_manifest()

    @staticmethod
    def record(name: str, contents: bytes) -> dict[str, object]:
        return {
            "path": name,
            "sha256": hashlib.sha256(contents).hexdigest(),
            "size_bytes": len(contents),
            "kind": "file",
        }

    @staticmethod
    def tree_hash(records: list[dict[str, object]]) -> str:
        encoded = json.dumps(records, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
        return hashlib.sha256(encoded.encode("utf-8")).hexdigest()

    def write_manifest(self) -> None:
        self.manifest_path.write_text(json.dumps(self.manifest), encoding="utf-8")

    def run_check(self) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [sys.executable, "-B", str(SCRIPT), "--workspace-root", str(self.workspace)],
            capture_output=True,
            text=True,
            check=False,
        )

    def test_legacy_manifest_passes_without_writing_files(self) -> None:
        before = {
            path.relative_to(self.workspace): path.read_bytes()
            for path in self.workspace.rglob("*") if path.is_file()
        }
        result = self.run_check()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("Build freshness PASS", result.stdout)
        after = {
            path.relative_to(self.workspace): path.read_bytes()
            for path in self.workspace.rglob("*") if path.is_file()
        }
        self.assertEqual(before, after)

    def test_changed_source_is_rejected_without_updating_manifest(self) -> None:
        original_manifest = self.manifest_path.read_bytes()
        self.source.write_bytes(b"source v2\n")
        result = self.run_check()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("source tree is stale", result.stderr)
        self.assertEqual(self.manifest_path.read_bytes(), original_manifest)

    def test_changed_installation_is_rejected(self) -> None:
        self.installed.write_bytes(b"installed v2\n")
        result = self.run_check()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("install tree changed", result.stderr)

    def test_each_mismatched_root_is_rejected_even_with_matching_hashes(self) -> None:
        for field in ("workspace_root", "source_root", "install_root"):
            with self.subTest(field=field):
                original = self.manifest[field]
                self.manifest[field] = str(self.workspace / "old_workspace")
                self.write_manifest()
                result = self.run_check()
                self.assertNotEqual(result.returncode, 0)
                self.assertIn(f"{field} mismatch", result.stderr)
                self.manifest[field] = original

    def test_python_caches_are_ignored_like_the_legacy_checker(self) -> None:
        for root in (self.workspace / "src", self.workspace / "xq_install"):
            (root / "__pycache__").mkdir()
            (root / "__pycache__" / "node.cpython.pyc").write_bytes(b"cache")
            (root / "node.pyo").write_bytes(b"cache")
        result = self.run_check()
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_missing_manifest_fails_without_building(self) -> None:
        self.manifest_path.unlink()
        result = self.run_check()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("Invalid build manifest", result.stderr)
        self.assertFalse(self.manifest_path.exists())


if __name__ == "__main__":
    unittest.main()
