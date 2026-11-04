"""Export source and explicitly selected model assets, excluding local credentials/data."""
import argparse
import json
from pathlib import Path
import shutil
import subprocess
import sys

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent))
from model_adapters import get_adapter, REGISTRY
from osmo_transport import file_hash


def git(repo,*args):
    return subprocess.check_output(['git','-C',str(repo),*args])


def export(repo,destination,additional=()):
    names=set(p for p in git(repo,'ls-files','--recurse-submodules','-z').decode().split('\0') if p)
    names.update(str(p.relative_to(repo)) for p in additional)
    records={}
    for name in sorted(names):
        # A git-tracked source export, not docker commit or an unrestricted directory copy.
        path=repo/name
        if not path.is_file() and not path.is_symlink():continue
        if '.git' in Path(name).parts or name=='.env' or 'ws2_runtime' in Path(name).parts:continue
        target=destination/name;target.parent.mkdir(parents=True,exist_ok=True)
        if path.is_symlink():
            import os
            link=os.readlink(path)
            (path.parent/link).resolve().relative_to(repo.resolve())
            if Path(link).is_absolute():raise ValueError('Absolute source symlink: '+str(path))
            target.symlink_to(link)
            records[name]={'symlink':link}
        else:
            shutil.copy2(path,target)
            records[name]=file_hash(path)
    return {'head':git(repo,'rev-parse','HEAD').decode().strip(),
            'dirty':bool(git(repo,'status','--porcelain').strip()),
            'submodules':git(repo,'submodule','status','--recursive').decode().splitlines(),
            'files':records}


def prepare(planner,output,torch_cache=None):
    adapter=get_adapter(planner);air=HERE.parents[2]
    output=Path(output).resolve()
    if output.exists():raise ValueError('Choose a new empty build-context path')
    # Missing weights must fail before producing an apparently ready image context.
    weights=[]
    for pattern in adapter.weight_patterns:
        matches=list(adapter.repository.glob(pattern))
        if not matches:raise ValueError('Restore model weights first: '+pattern)
        weights+=matches
    if adapter.torch_cache:
        if torch_cache is None:raise ValueError('Pass --torch-cache with the populated MonoNav torch cache')
        torch_cache=Path(torch_cache)
        if not (torch_cache/'hub/checkpoints/ZoeD_M12_N.pt').is_file():raise ValueError('Missing ZoeDepth checkpoint')
        if not list((torch_cache/'hub').glob('*MiDaS*/hubconf.py')):raise ValueError('Missing cached MiDaS source')
    output.mkdir(parents=True)
    additions=list(HERE.parent.glob('osmo_*.py'))+[HERE.parent/'runtime_process.py']
    additions += [p for p in HERE.rglob('*') if p.is_file() and '__pycache__' not in p.parts]
    manifest={'schema_version':1,'planner':planner,
              'inference_device':'cuda' if planner=='mononav' else 'cpu','repositories':{}}
    manifest['repositories']['AirStack']=export(air,output/'workspace/AirStack',additions)
    manifest['repositories'][planner]=export(adapter.repository,output/'workspace'/adapter.repository.name,weights)
    if torch_cache:
        # Only torch's model/source hub cache, never arbitrary user-home files.
        shutil.copytree(torch_cache/'hub',output/'workspace/torch/hub',ignore=shutil.ignore_patterns('__pycache__','*.pyc','.git'))
        manifest['torch_hub_sha256']={str(p.relative_to(output/'workspace/torch')):file_hash(p)
                                    for p in (output/'workspace/torch').rglob('*') if p.is_file()}
    (output/'workspace/AirStack/image_manifest.json').write_text(json.dumps(manifest,indent=2))
    shutil.copy2(HERE/'Dockerfile',output/'Dockerfile')
    shutil.copytree(HERE/'requirements',output/'requirements')
    (output/'.dockerignore').write_text('**/.git\n**/__pycache__\n**/*.pyc\n')
    return manifest


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--planner',required=True,choices=sorted(REGISTRY))
    p.add_argument('--output',required=True,type=Path)
    p.add_argument('--torch-cache',type=Path)
    a=p.parse_args();m=prepare(a.planner,a.output,a.torch_cache)
    print(json.dumps({'context':str(a.output),'planner':m['planner'],
                      'source_heads':{k:v['head'] for k,v in m['repositories'].items()}},indent=2))
