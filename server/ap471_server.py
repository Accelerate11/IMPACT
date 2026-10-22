#!/usr/bin/env python3
"""Portable AP 4.7.1 preparation and UNARMED server checks; no flight runner.

Only fresh paths below --workspace are written. Repository / supplied source
trees are read-only inputs. There is deliberately no ARM / mission subcommand.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import platform
import re
import shlex
import shutil
import signal
import socket
import subprocess
import sys
import uuid

sys.dont_write_bytecode = True

COMMIT = 'dbe792162d06cab66c3475fd5556bf7a120f119e'
TAG = 'Copter-4.7.1'
PIP_REQUIREMENTS = ('pexpect==4.9.0', 'ptyprocess==0.7.0', 'empy==3.3.4',
    'future==1.0.0', 'lxml==6.1.1', 'pymavlink==2.4.49', 'fastcrc==0.3.6', 'pyserial==3.5')
SKIP = {'.git', 'build', '__pycache__', '.pytest_cache'}


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as handle:
        for block in iter(lambda: handle.read(1048576), b''):
            digest.update(block)
    return digest.hexdigest()


def write(path, value):
    path = Path(path)
    with path.open('x', encoding='utf-8') as handle:
        json.dump(value, handle, indent=2, ensure_ascii=False, allow_nan=False)
        handle.write('\n')


def inventory(root, skip=SKIP):
    root = Path(root).resolve(strict=True)
    files = {}
    for path in sorted(root.rglob('*')):
        relative = path.relative_to(root)
        if any(part in skip-{'build'} for part in relative.parts) or (
                'build' in skip and relative.parts[0]=='build') or path.suffix == '.pyc':
            continue
        if path.is_symlink():
            target=os.readlink(path)
            if Path(target).is_absolute() or not path.resolve(strict=True).is_relative_to(root):
                raise ValueError('Non-self-contained input symlink: '+str(path))
            data=os.fsencode(target)
            files[str(relative)]={'kind':'symlink','link_target':target,
                'bytes':len(data),'sha256':hashlib.sha256(data).hexdigest(),
                'resolved_relative':str(path.resolve().relative_to(root))}
            continue
        if path.is_file():
            files[str(relative)] = {'bytes': path.stat().st_size, 'sha256': sha(path)}
    return files


def clean_env():
    env = dict(os.environ)
    env.update(PYTHONNOUSERSITE='1', PYTHONDONTWRITEBYTECODE='1', GIT_OPTIONAL_LOCKS='0')
    return env


def command(argv, *, cwd=None, env=None, log=None, timeout=None):
    argv = list(map(str, argv))
    if log is None:
        return subprocess.check_output(argv, cwd=cwd, env=env or clean_env(),
            text=True, stderr=subprocess.STDOUT, timeout=timeout)
    with Path(log).open('xb') as handle:
        process = subprocess.Popen(argv, cwd=cwd, env=env or clean_env(),
            stdout=handle, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            code = process.wait(timeout=timeout)
        except BaseException:
            stop_group(process)
            raise
    if code:
        raise RuntimeError(f'Command exited {code}; see {log}')


def check_platform():
    release = dict(line.split('=', 1) for line in Path('/etc/os-release').read_text().splitlines()
        if '=' in line)
    if platform.system() != 'Linux' or platform.machine() not in ('x86_64', 'amd64'):
        raise ValueError('This package targets Linux x86_64 only')
    if release.get('ID', '').strip('"') != 'ubuntu' or release.get('VERSION_ID', '').strip('"') != '22.04':
        raise ValueError('This package targets Ubuntu 22.04 only')
    if sys.version_info[:2] != (3, 10):
        raise ValueError('Use Ubuntu system Python 3.10; ROS Humble Python ABI must match')


def git_proof(root, require_ap=False, require_self_contained=False):
    root = Path(root)
    if not (root/'.git').exists():
        if require_ap:
            raise ValueError('AP source needs complete Git metadata and initialized submodules')
        return {'git_commit': None, 'revision_status': 'UNKNOWN_SOURCE_SNAPSHOT'}
    if require_self_contained:
        gitdir=Path(command(['git','-C',root,'rev-parse','--absolute-git-dir']).strip()).resolve(strict=True)
        common=Path(command(['git','-C',root,'rev-parse','--path-format=absolute','--git-common-dir']).strip()).resolve(strict=True)
        if not (root/'.git').is_dir() or not gitdir.is_relative_to(root.resolve()) or not common.is_relative_to(root.resolve()):
            raise ValueError('Build input Git metadata escapes source; export standalone repositories first')
    head = command(['git', '-C', root, 'rev-parse', 'HEAD']).strip()
    if require_ap and head != COMMIT:
        raise ValueError('AP source differs from pinned official Copter-4.7.1 commit')
    dirty = command(['git', '-C', root, 'status', '--porcelain=v1', '--untracked-files=all'])
    submodules = command(['git', '-C', root, 'submodule', 'status', '--recursive'])
    if dirty or any(line.startswith(('-', '+', 'U')) for line in submodules.splitlines()):
        raise ValueError('Source/submodules are dirty, missing or do not match superproject')
    alternates = root/'.git/objects/info/alternates'
    if alternates.exists() and alternates.read_text().strip():
        raise ValueError('Git object alternates are not self-contained; export a standalone source tree')
    return {'git_commit': head, 'submodules': submodules,
            'revision_status': 'PINNED_GIT_CHECKOUT'}


def snapshot_copy(source, dest, *, keep_git=False):
    source=Path(source).resolve(strict=True)
    excluded = {'__pycache__', '.pytest_cache', '*.pyc'}
    if not keep_git:
        excluded.add('.git')
    for path in Path(source).rglob('*'):
        if path.is_symlink() and (Path(os.readlink(path)).is_absolute() or
                not path.resolve(strict=True).is_relative_to(Path(source).resolve())):
            raise ValueError('Source snapshot contains a non-relocatable symlink: '+str(path))
    ordinary_ignore=shutil.ignore_patterns(*excluded)
    def ignore(directory,names):
        result=set(ordinary_ignore(directory,names))
        # ChibiOS / CrashDebug contain tracked source directories named build.
        # Only the top-level historical build output is an excluded artifact.
        if Path(directory).resolve()==source and 'build' in names:
            result.add('build')
        return result
    shutil.copytree(source, dest, symlinks=True, ignore=ignore)


class Run:
    def __init__(self, args, kind):
        self.repo = args.repo.resolve(strict=True)
        self.ws = args.workspace.resolve()
        if self.ws == self.repo or (self.ws.is_relative_to(self.repo) and
                not self.ws.is_relative_to(self.repo/'server_runtime')):
            raise ValueError('Inside a bundle, --workspace must be ROOT/server_runtime or its child')
        self.ws.mkdir(parents=True, exist_ok=True)
        self.id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+uuid.uuid4().hex[:8]
        self.root = self.ws/'results'/f'ap471_{kind}_{self.id}'
        self.root.mkdir(parents=True, exist_ok=False)
        self.record = {'schema_version': 1, 'status': 'FAIL', 'result': 'FAIL',
            'stage': 'initial', 'kind': kind, 'commit': COMMIT, 'run': str(self.root),
            'repo': str(self.repo), 'workspace': str(self.ws),
            'hardware_accessed': False, 'armed_trial': False,
            'flight_validated': False, 'fusion_validated': False}
        write(self.root/'request.json', {k: str(v) if isinstance(v, Path) else v
            for k, v in vars(args).items()})
        shutil.copyfile(Path(__file__), self.root/Path(__file__).name)
        print(json.dumps({'event': 'run_created', 'run': str(self.root)}), flush=True)

    def success(self, **fields):
        self.record.update(status='PASS', result='PASS', stage='complete', **fields)

    def close(self, error=None):
        if error is not None:
            self.record.update(status='FAIL', result='FAIL', error=repr(error))
        self.record['closed_at_utc'] = datetime.now(timezone.utc).isoformat()
        write(self.root/'status.json', self.record)
        write(self.root/'artifact-manifest.json', {'files': inventory(self.root),
            'self_excluded': 'artifact-manifest.json'})
        print(json.dumps(self.record, ensure_ascii=False), flush=True)


def prepare(args, run):
    run.record['stage'] = 'source'
    external = run.ws/'external'/f'ap471_{run.id}'
    external.mkdir(parents=True, exist_ok=False)
    source = external/'source'
    vendored = run.repo/'third_party/ardupilot'
    selected_source = args.source or (vendored if vendored.is_dir() else None)
    if selected_source:
        original = selected_source.resolve(strict=True)
        before = inventory(original)
        original_proof = git_proof(original, require_ap=True, require_self_contained=True)
        snapshot_copy(original, source, keep_git=True)
        if inventory(original) != before:
            raise RuntimeError('Read-only original source changed while copying')
        write(run.root/'source-origin.json', {'path': str(original), 'proof': original_proof, 'files': before})
    else:
        command(['git', 'clone', '--depth', '1', '--single-branch', '--branch', TAG,
            'https://github.com/ArduPilot/ardupilot.git', source], log=run.root/'clone.log')
        command(['git', '-C', source, 'submodule', 'update', '--init', '--recursive', '--depth', '1',
            '--jobs', str(min(args.jobs, 4))], log=run.root/'submodules.log')
    proof = git_proof(source, require_ap=True, require_self_contained=True)
    (run.root/'submodule-status.log').write_text(proof['submodules'])
    header = source/'ArduCopter/version.h'
    if '#define THISFIRMWARE "ArduCopter V4.7.1"' not in header.read_text():
        raise ValueError('AP version.h identity mismatch')
    shutil.copyfile(header, run.root/'version.h')
    write(run.root/'source-files.json', inventory(source))
    python = external/'python/bin/python3'
    command([sys.executable, '-m', 'venv', external/'python'], log=run.root/'venv.log')
    pip = [python, '-m', 'pip', 'install', '--disable-pip-version-check']
    if args.wheelhouse:
        pip += ['--no-index', '--find-links', args.wheelhouse.resolve(strict=True)]
    command([*pip, *PIP_REQUIREMENTS], log=run.root/'python-dependencies.log')
    (run.root/'python-freeze.txt').write_text(command([python, '-m', 'pip', 'freeze']))
    build = run.ws/'build'/f'ap471_{run.id}'
    build.mkdir(parents=True, exist_ok=False)
    run.record.update(stage='build', source=str(source), python=str(python), build=str(build))
    command([python, './waf', 'configure', '--board', 'sitl', '--out', build], cwd=source,
        log=run.root/'configure.log')
    command([python, './waf', 'copter', '-j'+str(args.jobs)], cwd=source, log=run.root/'build.log')
    binary = build/'sitl/bin/arducopter'
    if not binary.is_file() or not os.access(binary, os.X_OK):
        raise RuntimeError('No executable SITL copter produced')
    if b'ArduCopter V4.7.1' not in binary.read_bytes():
        raise ValueError('Compiled ELF has no expected ArduCopter version')
    if git_proof(source, require_ap=True, require_self_contained=True) != proof:
        raise RuntimeError('Pinned source/submodule identity changed during build')
    write(run.root/'toolchain.json', {name: command(argv).strip() for name, argv in {
        'uname': ['uname', '-a'], 'compiler': ['g++', '--version'],
        'python': [python, '--version'], 'git': ['git', '--version']}.items()})
    run.success(source=str(source), python=str(python), build=str(build), binary=str(binary),
        binary_sha256=sha(binary), binary_size=binary.stat().st_size, tag=TAG,
        scope='New-server pinned x86_64 SITL build only; no hardware or flight')


def load_preparation(path):
    path = path.resolve(strict=True)
    proof = json.loads((path/'status.json').read_text())
    if proof.get('result') != 'PASS' or proof.get('stage') != 'complete' or proof.get('commit') != COMMIT:
        raise ValueError('A completed AP471 preparation PASS is required')
    source, binary = (Path(proof[key]).resolve(strict=True) for key in ('source', 'binary'))
    python = Path(proof['python']).absolute()  # Do not resolve the venv launcher symlink.
    if not python.is_file() or sha(binary) != proof.get('binary_sha256'):
        raise ValueError('Recorded Python launcher/binary absent or binary SHA changed')
    if git_proof(source, require_ap=True, require_self_contained=True)['submodules'] != (path/'submodule-status.log').read_text():
        raise ValueError('Pinned AP submodules changed')
    return proof, source, binary, python


def stop_group(process):
    used = []
    for sig, delay in ((signal.SIGINT, 8), (signal.SIGTERM, 3), (signal.SIGKILL, 3)):
        if process.poll() is not None:
            break
        os.killpg(process.pid, sig)
        if sig != signal.SIGINT:
            used.append(signal.Signals(sig).name)
        try:
            process.wait(timeout=delay)
        except subprocess.TimeoutExpired:
            pass
    return {'exit_code': process.poll(), 'forced_signals': used}


def smoke(args, run):
    proof, source, binary, python = load_preparation(args.preparation)
    helper = run.repo/'scripts/run_ap471_research_gate.py'
    config_path = run.repo/'config/research_ap471_gate.json'
    spec = importlib.util.spec_from_file_location('ap471_original_smoke', helper)
    api = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(api)
    config = json.loads(config_path.read_text())
    api.validate_config(config)
    tracked = [binary, source/'ArduCopter/version.h', helper, config_path, args.preparation/'status.json']
    before = {str(p): sha(p) for p in tracked}
    write(run.root/'dependencies.before.json', before)
    shutil.copyfile(config_path, run.root/'config.json')
    params = run.root/'required.parm'
    params.write_text(''.join(f'{k} {v}\n' for k, v in config['required_parameters'].items()))
    runtime = run.root/'sitl_runtime'
    runtime.mkdir()
    # AP471 parses arguments sequentially: -I offsets the default 5760 value.
    # Place the explicit --base-port AFTER -I so it is always absolute, even
    # when the requested explicit value itself is 5760.
    port = args.base_port
    for candidate in (port, port+2, port+3):
        with socket.socket() as check:
            check.bind(('127.0.0.1', candidate))
    argv = [binary, '-S', '--model', 'quad', '--speedup', '1', '--slave', '0', '--wipe',
        '--defaults', str(source/'Tools/autotest/default_params/copter.parm')+','+str(params),
        '-I', str(args.instance), '--base-port', str(args.base_port),
        '--home', '-35.363261,149.165230,584,353', '--sim-address', '127.0.0.1']
    write(run.root/'launch-plan.json', {'command': list(map(str, argv)), 'cwd': str(runtime),
        'port': port, 'flight_commands_allowed': False, 'model': 'quad',
        'scope': 'Internal-dynamics disarmed firmware identity/parameter smoke; not Gazebo/ExternalNav'})
    process = None
    run.record['stage'] = 'unarmed_smoke'
    try:
        with (run.root/'sitl.log').open('xb') as log:
            process = subprocess.Popen(list(map(str, argv)), cwd=runtime, env=clean_env(),
                stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            command([python, helper, '--config', run.root/'config.json', '--_observe-run', run.root,
                '--_port', str(port)], log=run.root/'observer.log',
                timeout=2*config['startup_timeout_s']+config['startup_parameter_settle_s']+
                    config['parameter_timeout_s']+config['observation_s']+15)
            if process.poll() is not None:
                raise RuntimeError('SITL exited before smoke completion')
    finally:
        shutdown = stop_group(process) if process is not None else {}
        write(run.root/'shutdown.json', shutdown)
        after = {str(p): sha(p) if p.is_file() else None for p in tracked}
        write(run.root/'dependencies.after.json', after)
        if after != before:
            raise RuntimeError('Read-only smoke dependencies changed')
        if shutdown and (shutdown['forced_signals'] or shutdown['exit_code'] not in (0, 130, -signal.SIGINT)):
            raise RuntimeError('SITL shutdown issue: '+repr(shutdown))
    result = json.loads((run.root/'smoke-result.json').read_text())
    if result['status'] != 'PASS' or result['armed_seen']:
        raise RuntimeError('Disarmed smoke did not pass')
    run.success(binary=str(binary), binary_sha256=sha(binary),
        scope='Disarmed firmware identity/complete parameter readback only; no ROS/Gazebo or fusion',
        parameter_readback=result['parameter_readback'], heartbeat_count=result['heartbeat_count'])


def ros_command(argv, underlay, ros_setup, *, overlay=None):
    paths = [ros_setup, underlay/'setup.bash']
    if overlay:
        paths.append(overlay/'setup.bash')
    for path in paths:
        if not path.is_file():
            raise ValueError('Missing ROS/underlay setup: '+str(path))
    script = 'set -e; set +u; '
    for i in range(len(paths)):
        script += 'source "$1"; shift; '
    script += 'exec "$@"'
    return ['bash', '--noprofile', '--norc', '-c', script, 'ap471_server', *paths, *argv]


def build_autonomy(args, run):
    source = run.repo/'src/xq_autonomy'
    underlay = args.underlay.resolve(strict=True)
    before = {'canonical': inventory(source), 'underlay': inventory(underlay, skip={'.git','__pycache__','.pytest_cache'})}
    frozen = run.ws/'sources'/f'ap471_autonomy_{run.id}'/'xq_autonomy'
    frozen.parent.mkdir(parents=True, exist_ok=False)
    snapshot_copy(source, frozen)
    build = run.ws/'build'/f'ap471_autonomy_{run.id}'
    install = run.ws/'install'/f'ap471_autonomy_{run.id}'
    logs = run.ws/'logs'/f'ap471_autonomy_{run.id}'
    write(run.root/'inputs-before.json', before)
    env = clean_env()
    for key in ('AMENT_PREFIX_PATH','CMAKE_PREFIX_PATH','COLCON_PREFIX_PATH','PYTHONPATH','LD_LIBRARY_PATH'):
        env.pop(key, None)
    run.record['stage'] = 'isolated_autonomy_build'
    command(ros_command(['colcon','--log-base',logs,'build','--base-paths',frozen,
        '--build-base',build,'--install-base',install,'--packages-select','xq_autonomy',
        '--event-handlers','console_direct+'], underlay, args.ros_setup), env=env, log=run.root/'build.log')
    command(ros_command(['python3','-m','pytest','-q',frozen/'test'],underlay,args.ros_setup,overlay=install),
        env=env, log=run.root/'tests.log')
    after = {'canonical': inventory(source), 'underlay': inventory(underlay, skip={'.git','__pycache__','.pytest_cache'})}
    write(run.root/'inputs-after.json', after)
    if before != after:
        raise RuntimeError('Canonical source or read-only underlay changed')
    locations = list(install.glob('xq_autonomy/lib/python*/site-packages/xq_autonomy'))
    if len(locations) != 1:
        raise ValueError('Expected exactly one isolated installed Python module directory')
    bound = {}
    for rel,row in before['canonical'].items():
        if rel.startswith('xq_autonomy/') and rel.endswith('.py'):
            actual = locations[0]/rel.removeprefix('xq_autonomy/')
            if not actual.is_file() or sha(actual) != row['sha256']:
                raise RuntimeError('Installed source mismatch: '+rel)
            bound[rel] = {'sha256': row['sha256'], 'installed_path': str(actual)}
    manifest = {'schema_version':1,'status':'PASS','scope':'New-server isolated xq_autonomy only',
        'firmware_commit':COMMIT,'canonical_source_sha256':{k:v['sha256'] for k,v in before['canonical'].items()},
        'installed_modules':bound,'underlay_file_set_and_sha256_unchanged':True,
        'underlay_files':before['underlay'],'install_files_excluding_this_manifest':inventory(install),
        'build_run':str(run.root),'source_snapshot':str(frozen)}
    write(install/'.xq_build_manifest.json', manifest)
    write(run.root/'build-manifest.json', manifest)
    run.success(install_prefix=str(install), source_snapshot=str(frozen), installed_modules=len(bound),
        scope='New-server isolated autonomy build + tests; no AP/Gazebo execution')


def build_plugin(args, run):
    original = args.plugin_source.resolve(strict=True)
    before = inventory(original)
    provenance = git_proof(original)
    if not (original/'LICENSE.md').is_file():
        raise ValueError('Vendored plugin must retain LICENSE.md')
    frozen = run.ws/'sources'/f'ap471_gazebo_{run.id}'
    frozen.parent.mkdir(parents=True, exist_ok=True)
    snapshot_copy(original, frozen)
    build = run.ws/'build'/f'ap471_gazebo_{run.id}'
    env = clean_env(); env['GZ_VERSION'] = 'harmonic'
    write(run.root/'plugin-origin.json', {'path':str(original),'provenance':provenance,'files':before})
    run.record['stage'] = 'plugin_build'
    command(['cmake','-S',frozen,'-B',build,'-DCMAKE_BUILD_TYPE=RelWithDebInfo'],
        env=env, log=run.root/'configure.log')
    command(['cmake','--build',build,'--parallel',str(args.jobs)],env=env,log=run.root/'build.log')
    binary = build/'libArduPilotPlugin.so'
    if not binary.is_file():
        raise RuntimeError('Gazebo AP plugin was not produced')
    if inventory(original) != before:
        raise RuntimeError('Read-only plugin origin changed')
    write(run.root/'plugin-build-manifest.json', {'source':str(frozen),'source_files':inventory(frozen),
        'provenance':provenance,'binary':str(binary),'binary_sha256':sha(binary),
        'build_files':inventory(build,skip={'.git','__pycache__'})})
    run.success(plugin_source=str(frozen), plugin_build=str(build), binary=str(binary),
        binary_sha256=sha(binary), plugin_provenance=provenance,
        scope='Harmonic ArduPilot plugin compilation only; not camera/EGL/transport validation')


def headless_env(args):
    plugin = args.plugin_source.resolve(strict=True)
    build = args.plugin_build.resolve(strict=True)
    assets = args.assets.resolve(strict=True)
    for item in (build/'libArduPilotPlugin.so',plugin/'models/iris_with_ardupilot/model.sdf',
            plugin/'models/iris_with_standoffs/model.sdf',assets/'models',assets/'worlds'):
        if not item.exists():
            raise ValueError('Missing resource: '+str(item))
    values={'GZ_VERSION':'harmonic','GZ_SIM_SYSTEM_PLUGIN_PATH':str(build),
        'GZ_SIM_RESOURCE_PATH':':'.join(map(str,[assets/'models',assets/'worlds',plugin/'models',plugin/'worlds'])),
        'ROS2CLI_NO_DAEMON':'1','PYTHONNOUSERSITE':'1','PYTHONDONTWRITEBYTECODE':'1'}
    print('unset MESA_D3D12_DEFAULT_ADAPTER_NAME DISPLAY')
    for key,value in values.items():
        print('export '+key+'='+shlex.quote(value))
    print('# Run gz sim -s -r --headless-rendering WORLD.sdf; this command does not start AP or ARM.')


def plan_gazebo(args,run):
    _,source,binary,ap_python=load_preparation(args.preparation)
    ap_sites=[p for p in (ap_python.parent.parent/'lib').glob('python*/site-packages')
        if (p/'pymavlink/__init__.py').is_file()]
    if len(ap_sites)!=1:raise ValueError('Expected one private prepared pymavlink site')
    plugin=args.plugin_source.resolve(strict=True)
    build=args.plugin_build.resolve(strict=True)
    underlay=args.underlay.resolve(strict=True)
    assets=(args.assets or run.repo/'src/xq_gz_assets').resolve(strict=True)
    world=assets/'worlds/xq_r2b3_range.sdf'
    for path in (plugin/'LICENSE.md',plugin/'models/iris_with_ardupilot/model.sdf',
            plugin/'models/iris_with_standoffs/model.sdf',build/'libArduPilotPlugin.so',
            underlay/'setup.bash',world):
        if not path.is_file():raise ValueError('Missing planned dependency: '+str(path))
    config=run.repo/'config/research_ap471_gate.json'
    spec=importlib.util.spec_from_file_location('ap471_plan_config',run.repo/'scripts/run_ap471_research_gate.py')
    api=importlib.util.module_from_spec(spec);spec.loader.exec_module(api)
    declaration=json.loads(config.read_text());api.validate_config(declaration)
    params=declaration['required_parameters']
    required=run.root/'required.parm'
    required.write_text(''.join(f'{key} {value}\n' for key,value in params.items()))
    runtime=run.root/'sitl_runtime';runtime.mkdir()
    environment={'GZ_VERSION':'harmonic','GZ_PARTITION':'impact_ap471_'+run.id,
        'GZ_SIM_SYSTEM_PLUGIN_PATH':str(build),
        'GZ_SIM_RESOURCE_PATH':':'.join(map(str,[assets/'models',assets/'worlds',plugin/'models',plugin/'worlds'])),
        'ROS_DOMAIN_ID':str(args.ros_domain_id),'ROS2CLI_NO_DAEMON':'1',
        'PYTHONNOUSERSITE':'1','PYTHONDONTWRITEBYTECODE':'1'}
    commands={
        'sitl':[str(binary),'-S','--model','JSON','--speedup','1','--slave','0','--wipe',
            '--defaults',','.join(map(str,[source/'Tools/autotest/default_params/copter.parm',
                source/'Tools/autotest/default_params/gazebo-iris.parm',required])),
            '-I0','--base-port','6760','--sim-address','127.0.0.1','--sim-port-in','9003','--sim-port-out','9002'],
        'unarmed_wire_guard':[str(ap_python),str(run.repo/'scripts/probe_mavlink_tcp.py'),
            '--output-dir',str(run.root/'wire'),'--target-port','6760','--listen-port','6761',
            '--pymavlink-site',str(ap_sites[0]),
            '--guard-unarmed','--allow-fcu-startup-ascii','--allow-sitl-irlock-bootstrap-line','--max-runtime-s','240'],
        'mavros':['ros2','launch','mavros','apm.launch','fcu_url:=tcp://127.0.0.1:6761','namespace:=uav1/mavros'],
        'gazebo':['gz','sim','-r','-s','--headless-rendering','-v','3',str(world)],
        'bridge':['ros2','run','xq_gz_bridge','xq_gz_bridge_node','--ros-args','--params-file',
            str(run.repo/'src/xq_gz_bridge/config/p4_external_nav.yaml'),'-p','publish_ground_truth:=false']}
    dependencies={str(path):sha(path) for path in [binary,build/'libArduPilotPlugin.so',world,config,
        run.repo/'scripts/probe_mavlink_tcp.py',run.repo/'src/xq_gz_bridge/config/p4_external_nav.yaml']}
    write(run.root/'launch-plan.json',{'execution_status':'NOT_EXECUTED','flight_commands_allowed':False,
        'parameters':params,'environment':environment,'unset_environment':['DISPLAY','MESA_D3D12_DEFAULT_ADAPTER_NAME'],
        'commands':commands,'sitl_cwd':str(runtime),'ros_source':[str(args.ros_setup),str(underlay/'setup.bash')],
        'dependencies':dependencies,'plugin_source_files':inventory(plugin),'assets_files':inventory(assets),
        'underlay_files':inventory(underlay,skip={'.git','__pycache__','.pytest_cache'}),
        'fixed_udp_ports':[9002,9003],'tcp_ports':[6760,6761,6762,6763],
        'limits':['Plan generation is not a Gazebo/EGL/MAVROS execution PASS.',
            'No estimator or AP research mission is started by this plan.',
            'Wire guard must precede MAVROS; no direct bypass to SITL telemetry.',
            'The supplied Iris model fixes UDP9002; do not run two such worlds concurrently.']})
    run.success(execution_status='NOT_EXECUTED',scope='Portable unarmed JSON/Gazebo launch plan only',
        launch_plan=str(run.root/'launch-plan.json'))


def audit(args):
    report={'schema_version':1,'firmware_commit':COMMIT,'flight_commands_allowed':False,
        'platform':platform.platform(),'python':sys.version,'tools':{k:shutil.which(k)
        for k in ('git','g++','cmake','colcon','gz','ros2','ss')},
        'egm96_5_present':Path('/usr/share/GeographicLib/geoids/egm96-5.pgm').is_file(),
        'ros_humble_setup_present':args.ros_setup.is_file(),
        'scope':'Read-only dependency inventory; presence is not a functional or rendering PASS'}
    packages=['ros-humble-mavros','ros-humble-mavros-extras','ros-humble-mavros-msgs',
        'gz-sim8-cli','libgz-sim8','libgz-sim8-dev','libgz-transport13','libgz-msgs10']
    for name in packages:
        try:
            report.setdefault('installed_packages',{})[name]=command(['dpkg-query','-W','-f=${Version}',name]).strip()
        except subprocess.CalledProcessError:
            report.setdefault('installed_packages',{})[name]=None
    print(json.dumps(report,indent=2))


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    script_path=Path(__file__).resolve()
    default_root=script_path.parents[2] if script_path.parent.parent.name=='packaging' else script_path.parents[1]
    parser.add_argument('--root','--repo',dest='repo',type=Path,default=default_root)
    parser.add_argument('--workspace',type=Path,default=None)
    parser.add_argument('--ros-setup',type=Path,default=Path('/opt/ros/humble/setup.bash'))
    sub=parser.add_subparsers(dest='action',required=True)
    sub.add_parser('audit',help='Read-only platform/package inventory')
    p=sub.add_parser('prepare',help='Compile pinned official AP in fresh workspace paths')
    p.add_argument('--source',type=Path,help='Optional complete vendored git source, copied before build')
    p.add_argument('--wheelhouse',type=Path);p.add_argument('--jobs',type=int,default=2)
    p=sub.add_parser('smoke',help='UNARMED internal-quad identity and parameter smoke only')
    p.add_argument('--preparation',type=Path,required=True)
    p.add_argument('--base-port',type=int,default=5760);p.add_argument('--instance',type=int,default=20)
    p=sub.add_parser('build-autonomy',help='Build frozen autonomy snapshot over explicit ROS underlay')
    p.add_argument('--underlay',type=Path,required=True)
    p=sub.add_parser('build-plugin',help='Build vendored Harmonic plugin in fresh paths')
    p.add_argument('--plugin-source',type=Path,required=True);p.add_argument('--jobs',type=int,default=2)
    p=sub.add_parser('headless-env',help='Print shell-quoted resource environment; launch nothing')
    p.add_argument('--plugin-source',type=Path,required=True)
    p.add_argument('--plugin-build',type=Path,required=True)
    p.add_argument('--assets',type=Path,required=True)
    p=sub.add_parser('plan-gazebo',help='Freeze portable guarded JSON/MAVROS/Gazebo commands; execute nothing')
    p.add_argument('--preparation',type=Path,required=True)
    p.add_argument('--plugin-source',type=Path,required=True)
    p.add_argument('--plugin-build',type=Path,required=True)
    p.add_argument('--underlay',type=Path,required=True)
    p.add_argument('--assets',type=Path)
    p.add_argument('--ros-domain-id',type=int,default=120)
    args=parser.parse_args(argv)
    if args.workspace is None:
        args.workspace=Path(os.environ.get('IMPACT_SERVER_WORKSPACE',str(args.repo/'server_runtime')))
    check_platform()
    if getattr(args,'jobs',2) not in range(1,65):parser.error('--jobs must be in 1..64')
    if args.action=='smoke' and (not 0<=args.instance<=100 or not 1024<=args.base_port<=64000):
        parser.error('Invalid isolated telemetry port range')
    if args.action=='plan-gazebo' and not 0<=args.ros_domain_id<=232:
        parser.error('Invalid ROS_DOMAIN_ID')
    if args.action=='audit':audit(args);return 0
    if args.action=='headless-env':headless_env(args);return 0
    run=Run(args,args.action)
    error=None
    def interrupted(sig,frame):raise KeyboardInterrupt(signal.Signals(sig).name)
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,interrupted)
    try:
        {'prepare':prepare,'smoke':smoke,'build-autonomy':build_autonomy,'build-plugin':build_plugin,
            'plan-gazebo':plan_gazebo}[args.action](args,run)
    except BaseException as exc:
        error=exc
    finally:
        run.close(error)
    return 0 if run.record['status']=='PASS' else 1


if __name__=='__main__':
    raise SystemExit(main())
