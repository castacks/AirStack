"""CPU/USD assembly of a suburban earthquake, without starting Kit."""
import argparse
import json
import os
import sys
import time
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from pxr import Gf,Sdf,Usd,UsdGeom,UsdLux,UsdPhysics,UsdShade
import scene_generator as sg
import suburb_scene as ss
from compile_disaster import load_scene_config
from disaster import quake_suburban as qs
from disaster import quake_suburban_bake as qb
from disaster import quake_suburban_ground as ground
from disaster import quake_suburban_interactions as interactions

PARENT='/World/stage/generated'
SPECIES={
    'Black_Oak':'tower/Assets/Vegetation/Black_Oak/Black_Oak.usd',
    'Shumard_Oak':'tower/Assets/Vegetation/Shumard_Oak/Shumard_Oak.usd',
    'Common_Apple':'tower/Assets/Vegetation/Common_Apple/Common_Apple.usd',
    'Douglas_Fir':'brownstone/Assets/Vegetation/Trees/Douglas_Fir.usd',
    'Largetooth_Aspen':'brownstone/Assets/Vegetation/Trees/Largetooth_Aspen.usd',
    'American_Beech':'brownstone/Assets/Vegetation/Trees/American_Beech.usd'}


def configure_sky(stage,cfg):
    """Apply the preset HDRI and dome settings, independent of scene damage."""
    light=UsdLux.DomeLight.Define(stage,'/World/DomeLight')
    light.CreateIntensityAttr(float(cfg.get('sky_intensity',3500.)))
    light.CreateExposureAttr(float(cfg.get('sky_exposure',0.)))
    light.CreateColorAttr(Gf.Vec3f(*cfg.get('sky_color',(1.,1.,1.))))
    sky=sg.resolve_sky(cfg)
    if sky and Path(sky).suffix.lower() in ('.hdr','.exr'):
        # A bound procedural sky material would override texture:file.
        UsdShade.MaterialBindingAPI.Apply(light.GetPrim()).UnbindAllBindings()
        light.CreateTextureFileAttr(Sdf.AssetPath(sky))
        light.CreateTextureFormatAttr('latlong')
    return sky


def house_keepout(houses,clearance=.6,stage=None):
    from shapely.geometry import Polygon,MultiPoint
    from shapely.ops import unary_union
    from detail import modular_house as mh
    polygons=[]
    for h in houses:
        if stage is not None:
            points=interactions.mesh_points(stage.GetPrimAtPath(h['prim']+'/house/structure'))
            if points:
                # Include the real leaning roof/walls, not just the upright
                # lot footprint now that foundation transforms really compose.
                polygons.append(MultiPoint([(p[0],p[1]) for p in points]).convex_hull)
        cells=mh.footprint(mh.STYLES[h['style']])
        ox=-(min(x for x,y in cells)+max(x for x,y in cells)+1)*2.5
        oy=-(min(y for x,y in cells)+max(y for x,y in cells)+1)*2.5
        for i,j in cells:
            polygons.append(Polygon([qs._world((i+a)*5+ox,(j+b)*5+oy,h)
                            for a,b in ((0,0),(1,0),(1,1),(0,1))]))
    return unary_union(polygons).buffer(clearance)


def tree_fits(stage,prim,keepout):
    from shapely.geometry import box
    bounds=UsdGeom.BBoxCache(0,['default','render']).ComputeWorldBound(prim).ComputeAlignedRange()
    if bounds.IsEmpty():
        raise RuntimeError('unresolved tree '+str(prim.GetPath()))
    lo,hi=bounds.GetMin(),bounds.GetMax()
    return not keepout.intersects(box(lo[0],lo[1],hi[0],hi[1]))


def foundation_ground(stage,houses,bounds):
    import random
    from disaster import quake_flow as qf
    mats=qf.materials(stage,PARENT)
    result=[]
    for h in houses:
        if h['mode']!='foundation':
            continue
        g=h['foundation_response']
        m=dict(cx=h['x'],cy=h['y'],yaw=h['yaw'],W=h['W'],D=h['D'],z0=0.,top=h['H'])
        result.append(qf._c_ground_response(stage,m,low_side=g['low'],drop_m=g['drop'],rise_m=g['rise'],
            parent=PARENT+'/quake_ground/foundation_'+h['id'],mats=mats,rng=random.Random(h['seed']+71),
            tag=h['id'],bounds=tuple(bounds),fissures=False,kerb=False,mudline=False,boils=False,
            ground_at=lambda x,y:'grass'))
    return result


def _report_house(house):
    """Keep the review/GT fields, not megabytes of per-fragment cache data."""
    omit={'cache','debris_specs','fracture_footprints','physics_bake',
          'collapse_zones','fracture_report'}
    return {k:v for k,v in house.items() if k not in omit}


def _vehicle_rows(placements):
    keep=('category','prim_path','usd','roll_deg','pitch_deg','yaw_deg',
          'heading_deg','axis_up','role')
    return [{k:p.get(k) for k in keep if p.get(k) is not None}
            for p in placements
            if str(p.get('category') or '').lower()
            in ('car','vehicle','truck','van') and p.get('prim_path')]


def build(out,config_name='suburb_earthquake_250',cache=None,people=True,
          previous_report=None,scene_format='usda'):
    started=time.time()
    out=os.path.abspath(out)
    os.makedirs(out,exist_ok=True)
    scene_format=str(scene_format).lower().lstrip('.')
    if scene_format not in ('usda','usdc'):
        raise ValueError('scene_format must be usda or usdc')
    scene_path=out+'/review_scene.'+scene_format
    if os.path.exists(scene_path):
        raise FileExistsError('Use a new revision directory; refusing to overwrite '+scene_path)
    stage=Usd.Stage.CreateNew(scene_path)
    UsdGeom.SetStageMetersPerUnit(stage,1.)
    UsdGeom.SetStageUpAxis(stage,'Z')
    stage.SetDefaultPrim(UsdGeom.Xform.Define(stage,'/World').GetPrim())
    physics=UsdPhysics.Scene.Define(stage,'/World/PhysicsScene')
    physics.CreateGravityDirectionAttr(Gf.Vec3f(0.,0.,-1.))
    physics.CreateGravityMagnitudeAttr(9.81)
    cfg=load_scene_config(config_name)
    cfg.setdefault('usds',{})['humans']=[]
    configure_sky(stage,cfg)
    info={}
    placements=ss.generate_suburb_on_stage(stage,cfg,parent_path=PARENT,
                        scene_scale_factor=1.,info_out=info,assembly=True)
    region=tuple(info['region'])
    expected_size=tuple(float(v) for v in cfg['layout']['region_m'])
    expected=(-expected_size[0]/2.,-expected_size[1]/2.,
              expected_size[0]/2.,expected_size[1]/2.)
    if any(abs(float(a)-float(b))>.01 for a,b in zip(region,expected)):
        raise ValueError('Suburban region disagrees with config: %r != %r' %
                         (region,expected))
    seed=int(cfg.get('seed',10))
    field=sg.make_damage_field(cfg['disaster']['field'],region)
    houses=qs.plan_houses(info['house_instances'],cfg,field,seed)
    cache=os.path.abspath(cache or out+'/house_cache')
    for i,h in enumerate(houses):
        qb.assemble_house(stage,h,PARENT,cache)
        print('[suburban_quake] cached house %d/%d %s'%(i+1,len(houses),h['mode']),flush=True)
    trees=[]; tree_changes=[]
    keepout=house_keepout(houses,stage=stage)
    for i,t in enumerate(info.get('tree_instances',[])):
        root=PARENT+'/trees/t_%04d'%i
        prim=UsdGeom.Xform.Define(stage,root).GetPrim()
        prim.GetReferences().AddReference(sg._join_asset_root('airstack://scene_gen/assets/aec/'+SPECIES[t['species']],''))
        xf=UsdGeom.Xformable(prim)
        xf.AddTranslateOp().Set(Gf.Vec3d(t['x'],t['y'],0.))
        xf.AddRotateZOp().Set(float(t['yaw']))
        xf.AddScaleOp().Set(Gf.Vec3f(.01))
        prim.SetInstanceable(True)
        chosen=t['species']
        if not tree_fits(stage,prim,keepout):
            chosen=None
            # Keep the planting station and natural asset scale. Prefer
            # modest crowns from the same existing library, never shrink an oak.
            for species in ('Common_Apple','Largetooth_Aspen','American_Beech','Douglas_Fir'):
                prim.GetReferences().ClearReferences()
                prim.GetReferences().AddReference(sg._join_asset_root('airstack://scene_gen/assets/aec/'+SPECIES[species],''))
                if tree_fits(stage,prim,keepout):
                    chosen=species
                    break
            tree_changes.append(dict(prim=root,from_species=t['species'],to_species=chosen))
        if chosen is None:
            stage.RemovePrim(root)
            continue
        trees.append(dict(t,prim=root,species=chosen))
    soil=cfg['disaster'].get('soft_soil')
    traces=ground.traces_for_scene(houses,soil,seed,region)
    print('[suburban_quake] authoring material-matched rupture surfaces',flush=True)
    ground_report=ground.author(stage,PARENT,traces,seed,region)
    ground_report['foundation_responses']=foundation_ground(stage,houses,region)
    print('[suburban_quake] applying local object interactions',flush=True)
    changes=interactions.author(stage,houses,placements,trees,traces,soil,seed,tree_keepout=keepout)
    blockers=[]
    bounds=UsdGeom.BBoxCache(0,['default','render'])
    for path in [t['prim'] for t in trees]+[p['prim_path'] for p in placements if p.get('prim_path')]:
        prim=stage.GetPrimAtPath(path)
        if not prim:
            continue
        box=bounds.ComputeWorldBound(prim).ComputeAlignedRange()
        if box.IsEmpty():
            continue
        lo,hi=box.GetMin(),box.GetMax()
        if hi[2]>.3:
            blockers.append((float((lo[0]+hi[0])/2),float((lo[1]+hi[1])/2),float(max(hi[0]-lo[0],hi[1]-lo[1])/2)))
    previous=[]
    if previous_report:
        with open(previous_report) as f:
            previous=json.load(f)['people']
    people_requested=int(cfg['earthquake_suburban'].get('people_total',12))
    survivors=qs.author_people(stage,houses,PARENT,blockers,
         total=people_requested,seed=seed,preferred=previous,
         bounds=region) if people else []
    if people and not survivors:
        raise RuntimeError('No casualties passed the geometry/sightline gates')
    # Referenced assets can carry dormant physics; explicitly disable every
    # body in this review. No timeline step is needed to make damage stand.
    disabled=0
    for prim in list(stage.Traverse()):
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)
            disabled+=1
    report_houses=[_report_house(h) for h in houses]
    report=dict(revision=qb.REVISION,region=region,houses=report_houses,trees=trees,
                people=survivors,ground=ground_report,interactions=changes,tree_clearance_changes=tree_changes,
                people_requested=people_requested if people else 0,
                vehicles=_vehicle_rows(placements),
                scene_path=os.path.basename(scene_path),
                settle=dict(global_steps=0,disabled_bodies=disabled,static_cached=True),
                elapsed_s=time.time()-started)
    for name,value in (('scene_report',report),
                       ('damage_plan',dict(region=region,houses=report_houses))):
        with open(out+'/'+name+'.json','w') as f:
            json.dump(value,f,indent=2,default=str)
    stage.GetRootLayer().Save()
    print('[suburban_quake] STATIC_BUILD_COMPLETE '+scene_path,flush=True)
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',required=True)
    parser.add_argument('--config',default='suburb_earthquake_250')
    parser.add_argument('--cache')
    parser.add_argument('--previous-report',help='Revalidate and retain previously accepted casualty positions first')
    parser.add_argument('--no-people',action='store_true',help='geometry diagnostic only, not a complete review')
    parser.add_argument('--scene-format',choices=('usda','usdc'),default='usda',
                        help='use binary usdc for large dataset cells')
    args=parser.parse_args()
    build(args.out,args.config,args.cache,not args.no_people,
          args.previous_report,args.scene_format)
