"""Prepare only the required per-house fracture caches; no Isaac process."""
import argparse
import json
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from disaster import quake_suburban_bake as bake


def prepare(report_path,cache,manifest,ids=None):
    with open(report_path) as f:
        houses=json.load(f)['houses']
    result=[]; seen=set()
    for h in houses:
        if ids and h['id'] not in ids:
            continue
        key,_=bake.cache_key(h)
        if key in seen:
            continue
        seen.add(key)
        print('[quake_prepare] '+h['id']+' '+h['mode'],flush=True)
        path,meta=bake.bake_house(h,cache)
        result.append(dict(path=path,sidecar=str(Path(path).with_suffix('.json')),
                           id=h['id'],ready=meta['physics_ready']))
    with open(manifest,'w') as f:
        json.dump(result,f,indent=2)
    print('[quake_prepare] PREPARE_COMPLETE '+manifest,flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--report',required=True)
    p.add_argument('--cache',required=True)
    p.add_argument('--manifest',required=True)
    p.add_argument('--ids',nargs='*')
    a=p.parse_args()
    prepare(a.report,a.cache,a.manifest,a.ids)
