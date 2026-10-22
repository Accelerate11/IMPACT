#!/usr/bin/env python3
"""Export pinned AP plus every recursive submodule as self-contained local Git clones.

Reads the original repository; writes only a NEW destination and adjacent
manifest. No hardlinks, worktree gitdir files, object alternates or old builds.
This works without the network when the original submodules are complete.
"""
from __future__ import annotations
import argparse
from datetime import datetime, timezone
import json
import hashlib
import os
from pathlib import Path
import subprocess
import sys

sys.dont_write_bytecode = True
from ap471_server import COMMIT, git_proof, sha, write


def git(root, *args, allow_missing=False):
    result = subprocess.run(['git','-C',str(root),*map(str,args)],text=True,
        stdout=subprocess.PIPE,stderr=subprocess.PIPE,
        env=__import__('os').environ | {'GIT_OPTIONAL_LOCKS':'0'})
    if result.returncode and not allow_missing:
        raise RuntimeError('git '+repr(args)+': '+result.stderr)
    return result.stdout.strip()


def clone(origin, dest, expected, upstream, records, relative='.'):
    if git(origin,'rev-parse','HEAD') != expected:
        raise ValueError('Original submodule identity mismatch: '+str(origin))
    if git(origin,'status','--porcelain=v1','--untracked-files=all'):
        raise ValueError('Original source is dirty: '+str(origin))
    dest.parent.mkdir(parents=True,exist_ok=True)
    result=subprocess.run(['git','clone','--no-hardlinks','--no-checkout','--local',str(origin),str(dest)],
        text=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
    if result.returncode:
        raise RuntimeError('Local standalone clone failed: '+result.stderr)
    git(dest,'checkout','--detach',expected)
    if upstream:
        git(dest,'remote','set-url','origin',upstream)
    else:
        git(dest,'remote','remove','origin')
    gitdir=Path(git(dest,'rev-parse','--absolute-git-dir')).resolve(strict=True)
    common=Path(git(dest,'rev-parse','--path-format=absolute','--git-common-dir')).resolve(strict=True)
    if not (dest/'.git').is_dir() or not gitdir.is_relative_to(dest.resolve()) or not common.is_relative_to(dest.resolve()):
        raise ValueError('Exported git metadata escapes its standalone repository')
    alternates=gitdir/'objects/info/alternates'
    if alternates.exists() and alternates.read_text().strip():
        raise ValueError('Export unexpectedly retains borrowed Git objects')
    records.append({'path':relative,'commit':expected,'upstream':upstream,
        'shallow':git(dest,'rev-parse','--is-shallow-repository')=='true',
        'gitdir_is_self_contained':True})
    modules=origin/'.gitmodules'
    if modules.is_file():
        rows=git(origin,'config','-f',modules,'--get-regexp',r'^submodule\..*\.path$',allow_missing=True)
        git(dest,'submodule','init')
        for row in rows.splitlines():
            key, child_relative=row.split(None,1)
            child_path=Path(child_relative)
            if child_path.is_absolute() or '..' in child_path.parts:
                raise ValueError('Unsafe submodule path')
            tree=git(origin,'ls-tree','HEAD','--',child_relative)
            descriptor=tree.split('\t',1)[0].split()
            if len(descriptor)!=3 or descriptor[:2]!=['160000','commit']:
                raise ValueError('Not a pinned Git submodule: '+child_relative)
            name=key[len('submodule.'):-len('.path')]
            url=git(origin,'config','-f',modules,'--get','submodule.'+name+'.url',allow_missing=True)
            child_origin=origin/child_relative
            known_url=git(child_origin,'remote','get-url','origin',allow_missing=True)
            if known_url.startswith(('https://','http://','git://','ssh://','git@')):
                url=known_url
            clone(child_origin,dest/child_relative,descriptor[2],url,records,
                child_relative if relative=='.' else relative+'/'+child_relative)
    git(dest,'fsck','--full','--no-reflogs')


def tracked_inventory(root):
    root=root.resolve(strict=True)
    tracked=set(filter(None,git(root,'ls-files','--recurse-submodules','-z').split('\x00')))
    files={}
    # First record link objects without following them. Git may legitimately
    # track a directory alias, e.g. pymavlink's node_modules/jspack.
    for relative in sorted(tracked):
        path=root/relative
        if path.is_symlink():
            target=os.readlink(path)
            resolved=path.resolve(strict=True)
            if Path(target).is_absolute() or not resolved.is_relative_to(root):
                raise ValueError('Tracked link escapes exported repository closure: '+relative)
            data=os.fsencode(target)
            files[relative]={'kind':'symlink','bytes':len(data),
                'sha256':hashlib.sha256(data).hexdigest(),'link_target':target,
                'resolved_relative':str(resolved.relative_to(root))}
        elif path.is_file():
            files[relative]={'kind':'file','bytes':path.stat().st_size,'sha256':sha(path)}
        else:
            raise ValueError('Missing/non-self-contained tracked source: '+relative)
    for relative,row in files.items():
        if row['kind']!='symlink':continue
        resolved=root/row['resolved_relative']
        if resolved.is_file():
            if row['resolved_relative'] not in tracked:
                raise ValueError('Link target file is not in tracked export closure: '+relative)
            row.update(target_kind='file',target_sha256=sha(resolved),
                target_sha256_definition='SHA256 of target file bytes')
        elif resolved.is_dir():
            prefix=row['resolved_relative'].rstrip('/')+'/'
            entries={name[len(prefix):]:{k:v for k,v in value.items()
                if k not in ('target_kind','target_sha256','target_sha256_definition')}
                for name,value in files.items() if name.startswith(prefix)}
            if not entries:
                raise ValueError('Link target directory is not in tracked export closure: '+relative)
            blob=json.dumps(entries,sort_keys=True,separators=(',',':'),ensure_ascii=True).encode()
            row.update(target_kind='directory',target_sha256=hashlib.sha256(blob).hexdigest(),
                target_sha256_definition='SHA256 of canonical JSON manifest of tracked target entries')
        else:
            raise ValueError('Unsupported link target: '+relative)
    return files


def pins(status):
    return [line.split()[:2] for line in status.splitlines()]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=Path,required=True)
    p.add_argument('--destination',type=Path,required=True)
    p.add_argument('--manifest',type=Path)
    args=p.parse_args()
    source=args.source.resolve(strict=True);dest=args.destination.absolute()
    manifest=args.manifest or dest.with_name(dest.name+'_export_manifest.json')
    if dest.exists() or manifest.exists():
        raise ValueError('Destination/manifest must be new; failed exports are retained')
    if dest.resolve().is_relative_to(source):
        raise ValueError('Export destination cannot be inside original source')
    original=git_proof(source,require_ap=True)
    original_files=tracked_inventory(source)
    records=[]
    clone(source,dest,COMMIT,'https://github.com/ArduPilot/ardupilot.git',records)
    exported=git_proof(dest,require_ap=True)
    if exported['git_commit']!=original['git_commit'] or pins(exported['submodules'])!=pins(original['submodules']):
        raise ValueError('Exported commit/submodule set differs from original')
    exported_files=tracked_inventory(dest)
    if exported_files!=original_files or tracked_inventory(source)!=original_files:
        raise ValueError('Exported tracked source differs or read-only original changed')
    if not (dest/'COPYING.txt').is_file():
        raise ValueError('GPL license missing')
    value={'schema_version':1,'status':'PASS','commit':COMMIT,
        'created_at_utc':datetime.now(timezone.utc).isoformat(),
        'source':str(source),'destination':str(dest),'repositories':records,
        'source_files':exported_files,'submodule_status':exported['submodules'],
        'build_artifacts_exported':False,'object_hardlinks_or_alternates_used':False,
        'original_file_set_and_sha_unchanged':True}
    write(manifest,value)
    print(json.dumps({'status':'PASS','destination':str(dest),'manifest':str(manifest),
        'repositories':len(records),'source_files':len(exported_files)}))


if __name__=='__main__':
    main()
