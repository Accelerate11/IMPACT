#!/usr/bin/env python3
"""Bind a successful LIVO build to its exact sources and installed dependencies."""
import argparse
import hashlib
import json
from pathlib import Path


def record(path):
    return {'bytes': path.stat().st_size, 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}


def tree(root):
    return {str(path.relative_to(root)): record(path) for path in sorted(root.rglob('*'))
            if path.is_file() and '__pycache__' not in path.parts and path.suffix != '.pyc'}


def inputs(workspace):
    prefix = workspace / 'livo_deps'
    manifest = json.loads((prefix / 'dependency-manifest.json').read_text())
    if manifest['status'] != 'PASS':
        raise ValueError('dependencies are not a completed build')
    for name, expected in manifest['installed_files'].items():
        if record(prefix / name) != expected:
            raise ValueError('installed dependency changed: ' + name)
    return {'source_files': tree(workspace / 'src/xq_fast_livo2'),
            'dependencies_manifest': record(prefix / 'dependency-manifest.json'),
            'base_build_manifest': record(workspace / 'xq_install/.xq_build_manifest.json'),
            'build_script': record(workspace / 'scripts/build_livo_isolated.sh')}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace-root', type=Path, required=True)
    parser.add_argument('--mode', choices=('prepare', 'write', 'check'), default='check')
    args = parser.parse_args()
    workspace = args.workspace_root.resolve()
    previous = workspace / 'livo_build/.impact-build-inputs.json'
    manifest_path = workspace / 'livo_install/.impact-build-manifest.json'
    current = inputs(workspace)
    if args.mode == 'prepare':
        previous.parent.mkdir(parents=True, exist_ok=True)
        previous.write_text(json.dumps(current, indent=2) + '\n')
    elif args.mode == 'write':
        if json.loads(previous.read_text()) != current:
            raise ValueError('LIVO inputs changed during compilation')
        manifest_path.write_text(json.dumps({'schema_version': 1, 'status': 'PASS',
            'workspace_root': str(workspace), 'inputs': current,
            'installed_files': tree(workspace / 'livo_install/xq_fast_livo2')}, indent=2) + '\n')
    else:
        expected = json.loads(manifest_path.read_text())
        if (expected['status'] != 'PASS' or expected['workspace_root'] != str(workspace)
                or expected['inputs'] != current
                or expected['installed_files'] != tree(workspace / 'livo_install/xq_fast_livo2')):
            raise ValueError('LIVO source/dependency/installed build mismatch')
    print(json.dumps({'status': 'PASS', 'mode': args.mode,
                      'source_files': len(current['source_files'])}))


if __name__ == '__main__':
    main()
