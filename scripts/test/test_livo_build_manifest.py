"""Reject stale binaries and source/dependency changes across an actual build."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / 'check_livo_build_manifest.py'


def command(workspace, mode):
    return subprocess.run([sys.executable, str(SCRIPT), '--workspace-root', str(workspace),
                           '--mode', mode], capture_output=True, text=True)


@pytest.fixture
def workspace(tmp_path):
    files = {'src/xq_fast_livo2/src/mapper.cpp': 'source-v1',
             'scripts/build_livo_isolated.sh': 'build-v1',
             'xq_install/.xq_build_manifest.json': '{}',
             'livo_deps/lib/vikit.so': 'dependency-v1',
             'livo_install/xq_fast_livo2/lib/mapper': 'binary-v1'}
    for name, value in files.items():
        path = tmp_path / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(value)
    data = b'dependency-v1'
    (tmp_path / 'livo_deps/dependency-manifest.json').write_text(json.dumps({
        'status': 'PASS', 'installed_files': {'lib/vikit.so': {
            'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest()}}}))
    return tmp_path


def test_completed_build_reproduces(workspace):
    for mode in ('prepare', 'write', 'check'):
        assert command(workspace, mode).returncode == 0


def test_sources_cannot_change_during_build(workspace):
    assert command(workspace, 'prepare').returncode == 0
    (workspace / 'src/xq_fast_livo2/src/mapper.cpp').write_text('source-v2')
    assert command(workspace, 'write').returncode != 0
    assert not (workspace / 'livo_install/.impact-build-manifest.json').exists()


@pytest.mark.parametrize('name', ['src/xq_fast_livo2/src/mapper.cpp',
                                  'livo_deps/lib/vikit.so',
                                  'livo_install/xq_fast_livo2/lib/mapper'])
def test_stale_or_modified_build_rejected(workspace, name):
    assert command(workspace, 'prepare').returncode == 0
    assert command(workspace, 'write').returncode == 0
    (workspace / name).write_text('changed-after-build')
    assert command(workspace, 'check').returncode != 0
