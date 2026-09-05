"""Settle one prepared suburban earthquake house at a time; never a city."""
import json
import os
import random
import sys
from pathlib import Path
from isaacsim import SimulationApp
_gpu=os.environ.get('ISAAC_SIM_ACTIVE_GPU','').strip()
_launch={'headless':True,'extra_args':['--/renderer/multiGpu/enabled=false']}
if _gpu:
    _launch.update(active_gpu=int(_gpu),physics_gpu=int(_gpu))
app=SimulationApp(_launch)
import omni.usd
from pxr import Sdf,Usd,UsdGeom,UsdPhysics
from isaacsim.core.api import SimulationContext
REPO=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(REPO/'scene_gen'))
from disaster import settle


def main():
    manifest=os.environ['QUAKE_BAKE_WORK']
    with open(manifest) as f:
        work=json.load(f)
    completed=0
    use_gpu=os.environ.get('QUAKE_SETTLE_GPU','0').strip().lower() in ('1','true','yes')
    print('[quake_bake] physics backend '+('GPU' if use_gpu else 'CPU'),flush=True)
    for row in work:
        with open(row['sidecar']) as f:
            meta=json.load(f)
        if meta['physics_ready']:
            continue
        print('[quake_bake] START '+row['id']+' '+row['path'],flush=True)
        SimulationContext.clear_instance()
        omni.usd.get_context().open_stage(row['path'])
        stage=omni.usd.get_context().get_stage()
        for _ in range(4):
            app.update()
        # Decomposing every thin sheet costs far more than solving its fall.
        # Use measured PCA thickness to reserve decomposition for folded
        # sections, rather than every panel whose long edge exceeds 1.2 m.
        import numpy as np
        from disaster.quake_suburban_interactions import mesh_points
        approx={}
        for path in meta['loose_paths']:
            pts=np.asarray(mesh_points(stage.GetPrimAtPath(path)))
            centre=pts.mean(axis=0)
            _,axes=np.linalg.eigh((pts-centre).T@(pts-centre))
            spans=np.sort(np.ptp((pts-centre)@axes,axis=0))
            if spans[0]>.32 and spans[2]>1.5:
                approx[path]='convexDecomposition'
        print('[quake_bake] COLLIDERS '+str(len(meta['loose_paths']))+' bodies; '+str(len(approx))+' folded sections',flush=True)
        result=settle.run(stage,meta['loose_paths'],meta['static_paths'],
            steps=720,max_steps=1800,quiet_steps=120,kick=.12,
            rng=random.Random(meta['seed']),bake_result=True,gpu=use_gpu,
            max_speed=4.,density=420.,ccd=True,ground_plane_z=0.,floor_z=0.,
            approx_map=approx,decomp_limits={'max_hulls':4,'voxel_resolution':20000},
            converge=True,strict=False,rest_v2=True,
            # Keep the live simulation running when only a late fragment is
            # still moving: three continuations, each one third of the
            # original settle budget.  These remain configurable for an
            # unusually stubborn asset without rebuilding its fracture.
            retry_passes=int(os.environ.get('SETTLE_RETRY_PASSES','3')),
            retry_fraction=float(os.environ.get(
                'SETTLE_RETRY_FRACTION','0.333333333333')))
        if not result.get('driver','').startswith('SimulationContext'):
            raise RuntimeError('No explicit physics driver: '+str(result))
        # As in the shipped urban-fire and urban-quake bakers, never export a
        # chip frozen in mid-flight.  The live simulation has already had all
        # three continuation passes.  Removing one or two pathological shards
        # costs no visible damage detail; accepting a larger failed pile is
        # forbidden and remains a hard error.
        moving=list(result.get('still_moving_paths') or
                    result.get('still_moving_examples') or [])
        deactivated=[]
        for path in moving:
            prim=stage.GetPrimAtPath(Sdf.Path(str(path)))
            if prim and prim.IsValid() and prim.IsActive():
                prim.SetActive(False)
                deactivated.append(str(path))
        from disaster.quake_suburban_bake import (
            accept_bounded_grade_cull,accept_bounded_unsettled_cull,
            validate_settled_export)
        cull_limit=int(os.environ.get('QUAKE_MAX_UNSETTLED_CULL','2'))
        accept_bounded_unsettled_cull(
            result,len(deactivated),max_count=cull_limit)
        if deactivated:
            print('[quake_bake] DEACTIVATED_UNSETTLED '+
                  str(len(deactivated))+' '+','.join(deactivated),flush=True)

        # A deterministic thin shard can tunnel farther than the bounded
        # post-clamp repair allows even after every live continuation.  Do
        # what the urban quake/fire bakers already do for a pathological chip:
        # omit at most the remaining tiny cull budget, never accept a deep
        # teleport and never discard a real failed pile.
        grade_limit=float(os.environ.get('QUAKE_MAX_GRADE_REPAIR_M','.30'))
        grade_count=int(result.get('below_grade_pts') or 0)
        grade_examples=list(result.get('below_grade_examples') or [])
        grade_deactivated=[]
        grade_budget=max(0,cull_limit-len(deactivated))
        if (abs(float(result.get('below_grade_pts_worst') or 0.)) >
                grade_limit and 0 < grade_count <= grade_budget and
                len(grade_examples) >= grade_count):
            for path,_z in grade_examples[:grade_count]:
                prim=stage.GetPrimAtPath(Sdf.Path(str(path)))
                if prim and prim.IsValid() and prim.IsActive():
                    prim.SetActive(False)
                    grade_deactivated.append(str(path))
        accept_bounded_grade_cull(
            result,len(grade_deactivated),max_count=grade_budget,
            max_grade_repair_m=grade_limit)
        if grade_deactivated:
            print('[quake_bake] DEACTIVATED_BELOW_GRADE '+
                  str(len(grade_deactivated))+' '+
                  ','.join(grade_deactivated),flush=True)
        active_loose=[path for path in meta['loose_paths']
                      if stage.GetPrimAtPath(path).IsActive()]
        minimum_z=min(p[2] for path in active_loose
                      for p in mesh_points(stage.GetPrimAtPath(path)))
        exported=validate_settled_export(result,minimum_z)
        # The temporary /World physics scene lies outside the referenced
        # /House default prim. Disable every body in the exported house.
        for prim in list(stage.Traverse()):
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)
        meta['physics_bake']={k:v for k,v in result.items() if isinstance(v,(str,int,float,bool))}
        meta['physics_bake']['bodies']=len(meta['loose_paths'])
        meta['physics_bake']['deactivated_unsettled_paths']=deactivated
        meta['physics_bake']['deactivated_below_grade_paths']=grade_deactivated
        meta['physics_bake'].update(exported)
        meta['physics_bake']['pre_clamp_faults']=result.get('faults',[])
        from shapely.geometry import MultiPoint
        from disaster.quake_suburban_interactions import mesh_points
        meta['fracture_footprints']=[]
        for path in active_loose:
            points=mesh_points(stage.GetPrimAtPath(path))
            shape=MultiPoint([(p[0],p[1]) for p in points]).convex_hull
            if shape.geom_type=='Polygon':
                meta['fracture_footprints'].append(list(shape.exterior.coords))
        meta['physics_ready']=True
        stage.GetRootLayer().Save()
        with open(row['sidecar'],'w') as f:
            json.dump(meta,f,indent=2)
        completed+=1
        print('[quake_bake] DONE '+row['id']+' '+str(meta['physics_bake']),flush=True)
    print('[quake_bake] BAKE_COMPLETE '+str(completed),flush=True)


try:
    main()
except BaseException:
    import traceback
    traceback.print_exc()
    print('[quake_bake] BAKE_FAILED',flush=True)
    app.close()
    os._exit(1)
else:
    app.close()
