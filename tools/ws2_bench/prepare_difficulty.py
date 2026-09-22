"""Generate nested Office density tiers without launching Isaac Sim.

Counts mean additional props; original Office walls/furniture remain. A coarse
2-D connectivity check is a geometry filter, not proof a learned planner passes.
"""
import argparse,json,math,tempfile
from collections import deque
from pathlib import Path
from pxr import Usd,UsdGeom
from office_variants import build_variant,open_source,bounds

COUNTS={'easy':1,'medium':3,'hard':5}
PROTECTED=[[[-.8,-4.8,-.1],[.8,-3.2,2.2]],[[-.8,3.2,-.1],[.8,4.8,2.2]]]

def obstacle_bounds(source):
    stage=open_source(source);cache=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render'])
    occupied={};floors=[]
    for p in stage.GetDefaultPrim().GetChildren():
        if p.GetName().startswith(('SM_Floor','SM_Ceiling')):
            b=bounds(cache,p)
            if p.GetName().startswith('SM_Floor') and b and abs(b[1][2])<.03:floors.append(b)
            continue
        for item in p.GetChildren() if p.GetName()=='SM_Buildings' else [p]:
            b=bounds(cache,item)
            if b:occupied[str(item.GetPath())]=b
    return occupied,floors

def route_exists(occupied,floors):
    """Default spawn→8m goal, at1.2m altitude,0.35m horizontal radius."""
    step=.2;radius=.35
    boxes=[b for b in occupied if b[0][2]<1.4 and b[1][2]>.9]
    blocked=set()
    for i in range(-30,31):
        for j in range(-25,51):
            x,y=i*step,j*step
            if not any(b[0][0]<=x<=b[1][0] and b[0][1]<=y<=b[1][1] for b in floors) or any(b[0][0]-radius<=x<=b[1][0]+radius and b[0][1]-radius<=y<=b[1][1]+radius for b in boxes):blocked.add((i,j))
    start=(0,-20);goal=(0,20);pending=deque([start]);seen={start}
    if start in blocked or goal in blocked:return False
    while pending:
        x,y=pending.popleft()
        if (x,y)==goal:return True
        for n in [(x+1,y),(x-1,y),(x,y+1),(x,y-1)]:
            if -30<=n[0]<=30 and -25<=n[1]<=50 and n not in blocked and n not in seen:
                seen.add(n);pending.append(n)
    return False

def operations():
    ops=[{'action':'move','source':'/Root/SM_Plant7_463','delta_range_m':[[-3,-3,0],[-1,0,0]]},
         {'action':'duplicate','source':'/Root/SM_Plant8','delta_range_m':[[-6,1,0],[-2,4,0]]},
         {'action':'duplicate','source':'/Root/SM_ColumnA13','delta_range_m':[[-5,-23,0],[0,-20,0]]}]
    keys=['move','plant','column']
    # Displacements place additional props in the lobby and connected corridor;
    # support/static-obstacle checks use the actual supplied Office stage.
    for i in range(2,6):
        ops.extend([{'action':'duplicate','source':'/Root/SM_Plant8','delta_range_m':[[-9,3,0],[1,13,0]]},
                    {'action':'duplicate','source':'/Root/SM_ColumnA13','delta_range_m':[[-7,-26,0],[3,-16,0]]}])
        keys.extend([f'plant_{i}',f'column_{i}'])
    return keys,ops

def generate(source):
    base,floors=obstacle_bounds(source);keys,ops=operations();catalog={name:[] for name in COUNTS}
    for seed in range(8):
        for attempt in range(64):
            cfg={'seed':seed+1009*attempt,'clearance_m':.15,'protected_regions_m':PROTECTED,'operations':ops}
            try:
                with tempfile.TemporaryDirectory() as directory:
                    manifest=build_variant(source,cfg,Path(directory)/'scene.usda')
                occupied=dict(base)
                for change in manifest['changes']:occupied[change['target']]=change['after_bounds_m']
                if not route_exists(occupied.values(),floors):continue
                break
            except ValueError:continue
        else:raise ValueError(f'No valid hard layout for seed{seed}; widen placement region or reduce density')
        for name,count in COUNTS.items():
            changes=manifest['changes'][:1+2*count]
            entry={key:change['delta_m'] for key,change in zip(keys,changes)}
            entry.update(seed=seed,generation_seed=cfg['seed'],difficulty=name,
                         added_counts={'plants':count,'columns':count},source_sha256=manifest['source_sha256'],
                         validation='USD bounds: no overlaps, floor support, protected spawn/8m-goal; coarse 0.35m-radius route exists. Learned-planner success unverified.',
                         protected_regions_source_m=PROTECTED,
                         added_bounds_source_m={key:change['after_bounds_m'] for key,change in zip(keys,changes)})
            catalog[name].append(entry)
        print(f'seed{seed}: nested1/3/5pairs, generation seed{cfg["seed"]}',flush=True)
    return catalog

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('office_usd',type=Path)
    p.add_argument('--output',type=Path,default=Path(__file__).with_name('layouts.json'))
    a=p.parse_args();catalog=json.loads(a.output.read_text()) if a.output.exists() else {}
    catalog.update(generate(a.office_usd))
    a.output.write_text(json.dumps(catalog,indent=2)+'\n')
