#!/usr/bin/env python3
"""Headless review/freeze of a CPU-assembled suburban-earthquake cell."""
import collections
import json
import math
import os
import sys
import time

from isaacsim import SimulationApp

_gpu=os.environ.get('ISAAC_SIM_ACTIVE_GPU','').strip()
_args=['--/renderer/multiGpu/enabled=false',
       '--/renderer/raytracingMotion/enabled=false',
       '--/rtx/raytracing/fractionalCutoutOpacity=true',
       '--/rtx/pathtracing/fractionalCutoutOpacity=true']
_launch={'headless':True,'extra_args':_args}
if _gpu:
    _launch.update(active_gpu=int(_gpu),physics_gpu=int(_gpu))
app=SimulationApp(_launch)

import omni.kit.app
import omni.usd

REPO=os.path.abspath(os.path.join(os.path.dirname(__file__),'../../..'))
sys.path.insert(0,REPO+'/scene_gen')
sys.path.insert(0,REPO+'/simulation/isaac-sim/utils')
from disaster import freeze,quake_export
import snapshots_rp as snapshots

INPUT=os.environ['REVIEW_INPUT']
OUT=os.environ['FREEZE_OUT']
NAME=os.environ['FREEZE_NAME']
SNAPS=os.environ.get('SNAP_DIR') or OUT+'/snaps'
CONFIG=os.environ.get('SCENE_CONFIG','suburb_earthquake_1000_l1')
SEED=int(os.environ.get('QUAKE_SEED','11'))
MAG=float(os.environ.get('MAGNITUDE','0'))


def _capture(stage,path,eye,target,focal=28.,subframes=12):
    snapshots.place_camera(stage,eye,target,focal_mm=focal)
    return bool(snapshots.snapshot(stage,path,res=(1280,1280),
                                   subframes=subframes,
                                   target=(target[0],target[1])))


def _review(stage,report):
    os.makedirs(SNAPS,exist_ok=True)
    xmin,ymin,xmax,ymax=(float(v) for v in report['region'])
    cx,cy=(xmin+xmax)/2.,(ymin+ymax)/2.
    span=max(xmax-xmin,ymax-ymin)
    rows=[]

    def shot(name,eye,target,focal=28.,kind='scene'):
        path=os.path.join(SNAPS,name+'.png')
        ok=_capture(stage,path,eye,target,focal)
        rows.append(dict(name=name,path=os.path.relpath(path,SNAPS),
                         ok=ok,kind=kind,eye=[round(v,3) for v in eye],
                         target=[round(v,3) for v in target]))
        return ok

    shot('plate_top',(cx,cy,.72*span),(cx,cy,0.),18.,'whole_scene')
    distance=.72*span
    height=.46*span
    for az in (45.,135.,225.,315.):
        a=math.radians(az)
        shot('plate_obl_%03d'%az,
             (cx+distance*math.cos(a),cy+distance*math.sin(a),height),
             (cx,cy,min(25.,.035*span)),24.,'whole_scene')

    # A bounded, deterministic cross-section of damaged houses. Every mode
    # is represented before the remaining slots are filled across the plate.
    damaged=[h for h in report['houses'] if h.get('mode')!='pristine']
    limit=max(6,int(os.environ.get('SUBURB_QUAKE_REVIEW_BUILDINGS','24')))
    chosen=[]
    for mode in ('collapse','soft_storey','partial_collapse','foundation','racked'):
        group=[h for h in damaged if h.get('mode')==mode]
        if group:
            chosen.append(group[len(group)//2])
    remaining=[h for h in damaged if h not in chosen]
    need=max(0,limit-len(chosen))
    if need and remaining:
        for i in range(need):
            row=remaining[int(round(i*(len(remaining)-1)/max(1,need-1)))]
            if row not in chosen:
                chosen.append(row)
    for i,h in enumerate(chosen[:limit]):
        x,y=float(h['x']),float(h['y'])
        target=(x,y,min(float(h.get('H',10.)),4.5))
        shot('damage_%02d_%s_top'%(i,h['mode']),(x,y,42.),target,
             32.,'damaged_building')
        a=math.radians(225.)
        shot('damage_%02d_%s_obl'%(i,h['mode']),
             (x+30.*math.cos(a),y+30.*math.sin(a),18.),target,
             35.,'damaged_building')

    for person in report['people']:
        target=tuple(float(v) for v in person['review_target'])
        eyes=person.get('review_eyes') or ()
        if len(eyes)!=2:
            raise RuntimeError('casualty lacks two verified review eyes: '+
                               str(person.get('id')))
        for view,eye in enumerate(eyes,1):
            shot('people/%s_view%d'%(person['id'],view),
                 tuple(float(v) for v in eye),target,28.,'casualty')
    return rows


def main():
    report_path=os.path.join(INPUT,'scene_report.json')
    with open(report_path) as fh:
        report=json.load(fh)
    source=report.get('scene_path') or os.path.join(INPUT,'review_scene.usdc')
    if not os.path.isfile(source):
        candidates=[os.path.join(INPUT,q) for q in os.listdir(INPUT)
                    if q.startswith('review_scene.usd')]
        if len(candidates)!=1:
            raise RuntimeError('expected one CPU review stage, found '+repr(candidates))
        source=candidates[0]
    os.makedirs(OUT,exist_ok=True)
    os.makedirs(SNAPS,exist_ok=True)
    print('[suburban_quake_freeze] opening '+source,flush=True)
    omni.usd.get_context().open_stage(source)
    stage=None
    for _ in range(80):
        app.update()
        stage=omni.usd.get_context().get_stage()
        if stage is not None and stage.GetPrimAtPath('/World').IsValid():
            break
    if stage is None or not stage.GetPrimAtPath('/World').IsValid():
        raise RuntimeError('could not open '+source)
    for _ in range(40):
        app.update()

    states=collections.Counter(p.get('state') for p in report['people'])
    people_report=dict(interior_casualties=states.get('interior_casualty',0),
                       rubble_casualties=states.get('rubble_casualty',0),
                       generic_humans_deactivated=0,cover_pieces=0,
                       underfilled=max(0,int(report.get('people_requested',0))-
                                       len(report['people'])))
    people_doc=quake_export.people_document(report['people'],people_report)
    tally=collections.Counter(h.get('mode') for h in report['houses'])
    stats=dict(records=report['houses'],buildings=len(report['houses']),
               tally=dict(tally),tilted=tally.get('foundation',0)+
               tally.get('racked',0)+tally.get('soft_storey',0),missing=0)
    region=report['region']
    quake_export.write_sidecars(
        stage,OUT,stats,report.get('vehicles',()),people_doc,1.,CONFIG,SEED,
        'suburban_house_cache',None,
        [float(region[2])-float(region[0]),
         float(region[3])-float(region[1])],magnitude=MAG,people_variant=1)
    # Keep the exact lightweight decision records beside the scored GT.
    for name in ('scene_report.json','damage_plan.json'):
        src=os.path.join(INPUT,name)
        if os.path.isfile(src):
            import shutil
            shutil.copy2(src,os.path.join(OUT,name))

    started=time.time()
    views=_review(stage,report)
    successful=sum(bool(v['ok']) for v in views)
    review=dict(schema='airstack.suburban-earthquake-review/1',
                expected_views=len(views),successful_views=successful,
                failed=[v['path'] for v in views if not v['ok']],
                png_count=successful,views=views,
                coverage=dict(whole_scene='nadir plus four bearings',
                              damaged_buildings=sum(v['kind']=='damaged_building'
                                                    for v in views)//2,
                              casualties=len(report['people']),
                              casualty_views='two verified drone views each'),
                elapsed_s=time.time()-started)
    review['ok']=successful==len(views) and not review['failed']
    with open(os.path.join(OUT,'review_manifest.json'),'w') as fh:
        json.dump(review,fh,indent=1)
    print('[suburban_quake_freeze] REVIEW GATE %s: %d/%d' %
          ('OK' if review['ok'] else 'FAILED',successful,len(views)),flush=True)
    if not review['ok']:
        raise RuntimeError('suburban earthquake extensive review gate failed')

    info=freeze.export_scene(OUT,NAME,collect=False)
    freeze.report(info)
    with open(os.path.join(OUT,'freeze_report.json'),'w') as fh:
        json.dump(info,fh,indent=1)
    if not info.get('portable_ok'):
        raise RuntimeError('suburban earthquake portability gate failed')
    print('[suburban_quake_freeze] FREEZE DONE '+
          os.path.join(OUT,NAME+'.usd'),flush=True)


try:
    main()
except BaseException:
    import traceback
    traceback.print_exc()
    sys.stdout.flush(); sys.stderr.flush()
    os._exit(1)
else:
    sys.stdout.flush(); sys.stderr.flush()
    # Kit 5.1 can hang in synthetic-data shutdown after a large capture set.
    # The final report and USD are already durable at this boundary.
    os._exit(0)
