#!/usr/bin/env python3
"""Fetch and verify immutable resources, keeping normal Git clones small."""
import argparse
import hashlib
import json
import os
from pathlib import Path,PurePosixPath
import shutil
import tarfile
import tempfile
import urllib.request


def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as handle:
        for block in iter(lambda:handle.read(8*1024*1024),b''):
            h.update(block)
    return h.hexdigest()


def check_tree(root,manifest):
    failures=[]
    for row in manifest['files']:
        path=root/row['path']
        if row['kind']=='symlink':
            valid=path.is_symlink() and os.readlink(path)==row['link_target'] and path.resolve(strict=True).is_relative_to(root.resolve())
        else:
            valid=path.is_file() and not path.is_symlink() and path.stat().st_size==row['bytes'] and sha(path)==row['sha256']
        if not valid:
            failures.append(row['path'])
    if failures:
        raise ValueError('Resource fingerprint mismatch: '+', '.join(failures[:10]))


def authenticated_manifest(path,cfg):
    # Bind every installed-resource verification to this release, including
    # the already-installed fast path. An empty or stale inventory cannot pass.
    if not path.is_file() or path.is_symlink() or sha(path)!=cfg['manifest_sha256']:
        raise ValueError('Resource manifest differs from the committed release identity')
    manifest=json.loads(path.read_text())
    if (manifest.get('schema_version')!=1 or manifest.get('mode')!='resources'
            or len(manifest.get('files',[]))!=cfg['file_count']
            or manifest.get('regular_file_bytes')!=cfg['regular_file_bytes']):
        raise ValueError('Resource manifest schema/count/size mismatch')
    return manifest


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    script=Path(__file__).resolve()
    default=script.parents[2] if script.parent.parent.name=='packaging' else script.parents[1]
    parser.add_argument('--root',type=Path,default=default)
    parser.add_argument('--archive',type=Path,help='Use a transferred local archive instead of downloading')
    parser.add_argument('--verify-only',action='store_true')
    args=parser.parse_args()
    root=args.root.resolve()
    cfg=json.loads(script.with_name('resources.json').read_text())
    provenance=root/'provenance/resources_manifest.json'
    if provenance.is_file():
        check_tree(root,authenticated_manifest(provenance,cfg))
        print(json.dumps({'status':'PASS','scope':'Installed resource files verified; no download or overwrite','root':str(root)}))
        return 0
    if args.verify_only:
        raise ValueError('Resources have not been installed; run without --verify-only')
    if any((root/n).exists() for n in cfg['top_level_directories']):
        raise ValueError('Existing data/third_party without verified install manifest. Use a new checkout or validate your explicit resources; existing directories will not be overwritten.')
    cache=root/'server_runtime/downloads'
    cache.mkdir(parents=True,exist_ok=True)
    archive=(args.archive or cache/cfg['asset']).resolve()
    if args.archive is None and not archive.is_file():
        partial=archive.with_suffix(archive.suffix+'.partial')
        if cfg.get('parts'):
            for number,part in enumerate(cfg['parts'],1):
                piece=cache/part['asset']
                if not (piece.is_file() and piece.stat().st_size==part['bytes'] and sha(piece)==part['sha256']):
                    print('Downloading part %d/%d: %s'%(number,len(cfg['parts']),part['asset']),flush=True)
                    pending=piece.with_name(piece.name+'.partial')
                    request=urllib.request.Request(part['url'],headers={'User-Agent':'IMPACT-resource-fetch/1'})
                    with urllib.request.urlopen(request,timeout=90) as response,pending.open('wb') as output:
                        shutil.copyfileobj(response,output,1024*1024)
                    if pending.stat().st_size!=part['bytes'] or sha(pending)!=part['sha256']:
                        raise ValueError('Part checksum mismatch: '+part['asset'])
                    pending.replace(piece)
            with partial.open('wb') as output:
                for part in cfg['parts']:
                    with (cache/part['asset']).open('rb') as incoming:
                        shutil.copyfileobj(incoming,output,8*1024*1024)
        else:
            print('Downloading '+cfg['url'],flush=True)
            request=urllib.request.Request(cfg['url'],headers={'User-Agent':'IMPACT-resource-fetch/1'})
            with urllib.request.urlopen(request,timeout=90) as response,partial.open('wb') as output:
                shutil.copyfileobj(response,output,8*1024*1024)
        if partial.stat().st_size!=cfg['bytes'] or sha(partial)!=cfg['sha256']:
            raise ValueError('Downloaded archive SHA-256/size mismatch; partial file retained')
        partial.rename(archive)
    if not archive.is_file() or archive.stat().st_size!=cfg['bytes'] or sha(archive)!=cfg['sha256']:
        raise ValueError('Archive size or SHA-256 differs from the committed release descriptor')
    with tempfile.TemporaryDirectory(prefix='extract-',dir=cache) as temporary:
        stage=Path(temporary)
        with tarfile.open(archive,'r:gz') as packed:
            members=packed.getmembers()
            for member in members:
                name=PurePosixPath(member.name)
                if name.is_absolute() or '..' in name.parts or (name.parts[0] not in cfg['top_level_directories'] and member.name!='RESOURCE_MANIFEST.json'):
                    raise ValueError('Unexpected archive path: '+member.name)
                destination=stage/member.name
                if not destination.resolve().is_relative_to(stage):
                    raise ValueError('Archive path escapes extraction directory')
                if member.islnk() or member.isdev() or member.isfifo():
                    raise ValueError('Unsupported archive entry: '+member.name)
                if member.issym():
                    link=PurePosixPath(member.linkname)
                    if link.is_absolute() or not (destination.parent/member.linkname).resolve().is_relative_to(stage):
                        raise ValueError('Archive link escapes extraction directory')
            # Archive is authenticated by the committed SHA; paths and links above
            # are validated explicitly for system Python 3.10 tarfile.
            for member in members:
                destination=stage/member.name
                if not destination.parent.resolve().is_relative_to(stage):
                    raise ValueError('Archive traversal through a symlink')
                packed.extract(member,path=stage,set_attrs=True)
        manifest=authenticated_manifest(stage/'RESOURCE_MANIFEST.json',cfg)
        check_tree(stage,manifest)
        for folder in cfg['top_level_directories']:
            (stage/folder).rename(root/folder)
        provenance.parent.mkdir(parents=True,exist_ok=True)
        (stage/'RESOURCE_MANIFEST.json').rename(provenance)
    print(json.dumps({'status':'PASS','files':cfg['file_count'],'root':str(root),'archive_sha256':cfg['sha256']}))
    return 0


if __name__=='__main__':
    raise SystemExit(main())
