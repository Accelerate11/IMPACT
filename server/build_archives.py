#!/usr/bin/env python3
"""Build streaming, self-contained server archives and SHA-256 inventories."""
import argparse
from datetime import datetime,timezone
import hashlib
import io
import json
import os
from pathlib import Path
import tarfile


def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda:f.read(8*1024*1024),b''):
            h.update(b)
    return h.hexdigest()


def record(root,path):
    row={'path':path.relative_to(root).as_posix()}
    if path.is_symlink():
        target=os.readlink(path)
        assert not Path(target).is_absolute() and path.resolve(strict=True).is_relative_to(root)
        data=os.fsencode(target)
        row.update(kind='symlink',link_target=target,bytes=len(data),sha256=hashlib.sha256(data).hexdigest())
    else:
        row.update(kind='file',bytes=path.stat().st_size,sha256=sha(path))
    return row


def portable(info):
    info.uid=info.gid=0
    info.uname=info.gname=''
    info.mode=0o755 if info.isdir() else (0o777 if info.issym() else (0o755 if info.mode&0o111 else 0o644))
    return info


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--root',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--mode',choices=['resources','full','evidence'],required=True)
    a=p.parse_args()
    root=a.root.resolve()
    out=a.output.resolve()
    out.parent.mkdir(parents=True,exist_ok=True)
    if out.exists():
        raise SystemExit('Refusing to overwrite sealed archive: '+str(out))
    roots=[root/n for n in ('data','third_party')] if a.mode=='resources' else ([root/'evidence'] if a.mode=='evidence' else [x for x in root.iterdir() if x.name not in ('server_runtime','.git','__pycache__','.pytest_cache')])
    paths=[]
    for selected in roots:
        if selected.is_symlink() or selected.is_file():
            paths.append(selected)
        else:
            for here,dirs,files in os.walk(selected,followlinks=False):
                for name in sorted(dirs+files):
                    path=Path(here)/name
                    if path.is_symlink() or path.is_file():
                        paths.append(path)
        print('Inventory: '+selected.name,flush=True)
    rows=[record(root,x) for x in sorted(paths)]
    manifest={'schema_version':1,'mode':a.mode,'created_at_utc':datetime.now(timezone.utc).isoformat(),'root_layout':'Paths relative to the server workspace root','files':rows,'regular_file_bytes':sum(x['bytes'] for x in rows if x['kind']=='file')}
    payload=(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n').encode()
    internal_name='RESOURCE_MANIFEST.json' if a.mode=='resources' else 'BUNDLE_MANIFEST.json'
    with tarfile.open(out,'x:gz',compresslevel=3,format=tarfile.PAX_FORMAT,dereference=False) as archive:
        for selected in roots:
            archive.add(selected,arcname=selected.relative_to(root).as_posix(),recursive=True,filter=portable)
            print('Archived: '+selected.name,flush=True)
        info=tarfile.TarInfo(internal_name)
        info.size=len(payload)
        info.mode=0o644
        archive.addfile(info,io.BytesIO(payload))
    digest=sha(out)
    out.with_name(out.name+'.sha256').write_text(digest+'  '+out.name+'\n')
    out.with_name(out.name+'.manifest.json').write_bytes(payload)
    result={'status':'PASS','mode':a.mode,'path':str(out),'bytes':out.stat().st_size,'sha256':digest,'source_files':len(rows),'regular_source_bytes':manifest['regular_file_bytes'],'self_contained_symlinks':sum(r['kind']=='symlink' for r in rows)}
    out.with_name(out.name+'.result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result),flush=True)


if __name__=='__main__':
    main()
