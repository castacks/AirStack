"""Cached earthquake damage: fractured kit surfaces mixed with timber stock.

No whole-neighbourhood physics. Each cache entry is authored at the origin and
its loose fragments settled separately; assembly references resting transforms
and only adapts placement interactions.
"""
import hashlib
import json
import math
import os
import random

REVISION = 'fractured_bearing_v5'
NORMAL = {'S': (0., -1.), 'E': (1., 0.), 'N': (0., 1.), 'W': (-1., 0.)}


def accept_bounded_unsettled_cull(result, deactivated, max_count=2):
    """Turn a tiny, explicitly removed mid-flight tail into a valid solve.

    Urban fire and urban earthquake already prefer an absent chip to debris
    frozen in the air.  Suburban quake used to reject the whole house (and
    therefore the whole three-level queue) when one persistent collision
    jitter survived every live continuation.  Preserve the pre-cull verdict
    in the report and accept only a complete, tightly bounded removal; a real
    non-converged pile still fails :func:`validate_settled_export`.
    """
    before = int(result.get('still_moving') or 0)
    if before <= 0:
        return True
    result['pre_deactivate_still_moving'] = before
    result['deactivated_unsettled'] = int(deactivated)
    result['pre_deactivate_still_moving_paths'] = list(
        result.get('still_moving_paths') or
        result.get('still_moving_examples') or [])
    if before > int(max_count) or int(deactivated) != before:
        return False
    result['still_moving'] = 0
    result['still_moving_paths'] = []
    result['still_moving_examples'] = []
    result['converged'] = True
    result['stop_reason'] = 'bounded_unsettled_deactivated'
    return True


def validate_settled_export(result, minimum_z, max_grade_repair_m=.30):
    """Check exported points after settle's final clamp, retaining its verdict.

    A tilted foundation intersects the grade plane. Permit a small, measured
    grade repair only; never accept an unconverged solve or a deep tunnel.
    The shared at_rest_ok value describes the PRE-clamp pose and is unchanged.
    """
    bad = [k for k in ('still_moving', 'no_collider', 'no_local_frame',
                      'rescue_failed', 'clamp_failed') if result.get(k)]
    if not result.get('converged') or bad:
        raise RuntimeError('Invalid house solve: '+str(bad or ['not converged']))
    # ``below_grade_pts_worst`` describes the pose *before* settle's final
    # point-based clamp.  A thin shard can tunnel a little over 20 cm through
    # the plane even with CCD, then be placed correctly back on grade.  Do not
    # discard an otherwise valid deterministic cache for that historical
    # value; gate the successful repair and the final exported points instead.
    # Keep a finite upper bound so a genuinely escaped body is still rejected.
    if abs(float(result.get('below_grade_pts_worst') or 0.)) > float(max_grade_repair_m):
        raise RuntimeError('House fragment requires excessive grade repair')
    if not math.isfinite(minimum_z) or minimum_z < -.0201:
        raise RuntimeError('Exported house fragment remains below grade: '+str(minimum_z))
    return {'post_clamp_verified': True, 'export_min_z': float(minimum_z)}


def restore_structure_frame(stage,path):
    from pxr import UsdGeom
    return UsdGeom.Xform.Define(stage,path)


def debris_specs(rec, removed_count):
    """Compact gravity apron, with the existing real-size stock and skins.

    Replaced shell material stays local. There is no wind trail, fire finish,
    or assumption that the missing material has disappeared from the lot.
    Pieces are seated bottom-up against preceding stock, not a fake mound.
    """
    from . import planks
    from detail import modular_house as mh
    from shapely.geometry import Polygon
    rng = random.Random(rec['seed']+201)
    full = rec['mode'] == 'collapse'
    nx,ny = NORMAL[rec['side']]
    tx,ty = -ny,nx
    half = (rec['D'] if nx == 0 else rec['W'])/2
    along = (rec['W'] if nx == 0 else rec['D'])/2
    # Face area is a visual material budget, not the volume of the hollow
    # building's bounding box. The old 460-piece cap erased most of a house.
    target_area=float(rec.get('removed_face_area_m2',removed_count*14.))*float(rec.get('stock_area_factor',2.0))
    mean_area=sum(v[0]*sum(v[1])/2*sum(v[2])/2 for v in planks.STOCK.values())
    n = min(3600, max(240, int(target_area/max(.1,mean_area))))
    skins = mh.palette_skins(rec.get('palette') or mh.STYLES[rec['style']]['palette'])
    specs, bounds = [], []
    for i in range(n):
        zones=rec.get('collapse_zones',[])
        if zones and rng.random()<.82:
            zone=rng.choices(zones,weights=[z['area'] for z in zones])[0]
            x=rng.uniform(zone['x0'],zone['x1'])+rng.gauss(0,.45)
            y=rng.uniform(zone['y0'],zone['y1'])+rng.gauss(0,.45)
        elif full:
            x = rng.uniform(-.46,.46)*rec['W'] + rng.gauss(0,.8)
            y = rng.uniform(-.46,.46)*rec['D'] + rng.gauss(0,.8)
        else:
            distance = half + rng.triangular(-4.8,4.2,-.8)
            lateral = rng.uniform(-.85,.85)*along
            x,y = nx*distance+tx*lateral, ny*distance+ty*lateral
        k,ln,wd,th = planks._piece(rng)
        s = planks._lay(x,y,k,ln,wd,th,rng.uniform(0,360),rng,tilt_p=0)
        # Most stock lies face-down; larger roof/wall pieces retain texture.
        s['pitch'] = s['roll'] = 0.
        s['z'] = th/2+.008
        if k in skins:
            s['skin'] = skins[k]
        pts,_ = planks._box(s)
        xs,ys = [p[0] for p in pts],[p[1] for p in pts]
        a = (min(xs),min(ys),max(xs),max(ys))
        footprint = Polygon([(p[0],p[1]) for p in pts[:4]])
        support = .008
        for b,other,top in bounds:
            overlap = max(0,min(a[2],b[2])-max(a[0],b[0]))*max(0,min(a[3],b[3])-max(a[1],b[1]))
            if overlap and footprint.intersection(other).area > .20*min(footprint.area,other.area):
                support = max(support,top)
        s['z'] = support+th/2-.003
        bounds.append((a,footprint,s['z']+th/2))
        specs.append(s)
    return specs


def module_failures(items, bounds, rec):
    """Conservative support closure using measured module extents.

    A module's PIVOT does not say which bays it spans. In particular, a
    5 m floor tile or 10 m wall can bridge the selection boundary while its
    pivot survives. Whole unsupported modules join the collapsed material.
    """
    from shapely.geometry import box
    from shapely.ops import unary_union
    from .damage import _sub_of
    nx,ny=NORMAL[rec['side']]
    half=(rec['D'] if nx==0 else rec['W'])/2
    shapes=[box(b[0],b[1],b[3],b[4]) for b in bounds]
    roles=[_sub_of(q['category']) for q in items]
    levels=[float(q['z_m']) for q in items]
    broken=set()
    for i,(q,b) in enumerate(zip(items,bounds)):
        if roles[i]=='floor' and levels[i]<.1:
            continue
        edge=max(nx*x+ny*y for x in (b[0],b[3]) for y in (b[1],b[4]))
        if rec['mode']=='collapse' or (rec['mode']=='partial_collapse' and edge>half-5.0+.15):
            broken.add(i)
        elif rec['mode']=='soft_storey' and levels[i]<3.4 and roles[i] in ('wall','door','bay_roof') and edge>half-3:
            broken.add(i)
    initial=len(broken)
    if rec['mode'] not in ('partial_collapse','collapse'):
        return broken,0
    changed=True
    while changed:
        before=len(broken)
        for level in sorted(set(z for r,z in zip(roles,levels) if r=='floor' and z>.1)):
            floors=[i for i,r in enumerate(roles) if r=='floor' and abs(levels[i]-level)<.1 and i not in broken]
            if not floors:
                continue
            union=unary_union([shapes[i].buffer(.02) for i in floors])
            components=list(union.geoms) if hasattr(union,'geoms') else [union]
            for component in components:
                supports=[shapes[j].intersection(component) for j,r in enumerate(roles)
                          if r=='wall' and j not in broken and abs(levels[j]+3.5-level)<.15 and shapes[j].intersects(component)]
                hull=unary_union(supports).convex_hull
                # One surviving wall is not support for a 5 m cantilever.
                bearing=hull.buffer(.18)
                for i in floors:
                    if component.intersects(shapes[i]) and (hull.area<.25*component.area or
                            shapes[i].difference(bearing).area>.15*shapes[i].area):
                        broken.add(i)
        for i,role in enumerate(roles):
            if i in broken or levels[i]<.1 or role=='floor':
                continue
            if role in ('roof','bay_roof'):
                # A roof module spanning a failed wall cannot stay aloft.
                if any(j in broken and roles[j]=='wall' and 0<levels[i]-levels[j]<4.2 and
                       shapes[i].intersection(shapes[j]).area>.02 for j in range(len(items))):
                    broken.add(i)
            elif role in ('wall','door') and levels[i]>=3.4:
                below=[j for j,r in enumerate(roles) if r=='wall' and j not in broken and abs(levels[j]+3.5-levels[i])<.2]
                coverage=unary_union([shapes[j] for j in below]).intersection(shapes[i]).area
                floors=[j for j,r in enumerate(roles) if r=='floor' and j not in broken and abs(levels[j]-levels[i])<.2 and shapes[j].intersects(shapes[i])]
                if coverage<.55*shapes[i].area or not floors:
                    broken.add(i)
        changed=len(broken)!=before
    return broken,len(broken)-initial


def author_house(stage, rec, parent, ssf=1.):
    from pxr import Gf, UsdGeom
    from detail import modular_house as mh
    import scene_generator as sg
    from . import damage, planks, quake_flow as qf
    from .quake_suburban import local_xy
    if ssf != 1.:
        raise ValueError('suburban earthquake bake requires metre stages')
    root = parent+'/'+rec['id']
    shell = root+'/structure'
    UsdGeom.Xform.Define(stage,root)
    restore_structure_frame(stage,shell)
    rng = random.Random(rec['seed'])
    items = mh.build_building(rec['style'],rec['x'],rec['y'],rec['yaw'],rng,category='house')
    palette = rec.get('palette') or mh.STYLES[rec['style']]['palette']
    for q in items:
        q['palette'] = palette
    sg.apply_placements(stage,items,shell,ssf,instance_categories=set())
    # apply_placements defines its parent as Scope. Scope ignores transform
    # ops, even when their stored matrix reads back correctly. Restore Xform
    # AFTER placement, before authoring any foundation/ground movement.
    restore_structure_frame(stage,shell)
    mh.apply_palette(stage,items,shell)
    mode = rec['mode']
    nx,ny = NORMAL[rec['side']]
    half = (rec['D'] if nx == 0 else rec['W'])/2
    from .quake_suburban_interactions import mesh_points
    bounds=[]
    for q in items:
        pts=mesh_points(stage.GetPrimAtPath(q['prim_path']))
        if not pts:
            raise RuntimeError('unresolved house module '+q['prim_path'])
        xyz=[(*local_xy(p[0],p[1],rec),p[2]) for p in pts]
        bounds.append(tuple(min(p[k] for p in xyz) for k in range(3))+
                      tuple(max(p[k] for p in xyz) for k in range(3)))
    broken,extra=module_failures(items,bounds,rec)
    from . import quake_suburban_fragments as fragments
    loose, torn_static, fracture_report = fragments.author(stage,root,items,bounds,broken,rec)
    removed=[]; zones=[]; face_area=0.
    for i,q in enumerate(items):
        if i in broken:
            stage.GetPrimAtPath(q['prim_path']).SetActive(False)
            removed.append(q)
            b=bounds[i]; w,d=b[3]-b[0],b[4]-b[1]
            sub=damage._sub_of(q['category'])
            face_area+=max(w,d)*3.5 if sub=='wall' else w*d
            if sub in ('floor','roof'):
                zones.append(dict(x0=b[0],y0=b[1],x1=b[3],y1=b[4],area=w*d))
    rec.update(support_cascade_modules=extra,removed_face_area_m2=face_area,collapse_zones=zones,
               fracture_report=fracture_report,stock_area_factor=.95)
    rec['failure_sides'] = list('SENW') if mode in ('collapse','soft_storey') else [rec['side']] if removed else []
    rec['removed_modules'] = len(removed)
    rec['prim'] = root
    rec['reach_m'] = {side:4.5 for side in rec['failure_sides']}
    # Uneven bearing failure: the existing earthquake matrix deliberately
    # lowers one edge and raises the opposite edge about an interior pivot.
    if mode in ('foundation','racked','soft_storey'):
        m = dict(cx=rec['x'],cy=rec['y'],yaw=rec['yaw'],W=rec['W'],D=rec['D'],z0=0.,top=rec['H'])
        angle = rng.uniform(3.5,6.5) if mode != 'racked' else rng.uniform(.6,1.2)
        mat,geom = qf._c_tilt_matrix(m,rec['side'],angle,
            .32 if mode != 'racked' else .03,max_drop_m=1.35,
            min_rise_m=.18 if mode != 'racked' else None,max_rise_m=.9)
        UsdGeom.Xformable(stage.GetPrimAtPath(shell)).AddTransformOp().Set(mat)
        rec['foundation_response'] = geom
    if mode == 'soft_storey':
        # A compressed, racked residual storey supports the upper box; no
        # rigid upper floors are left suspended above removed supports.
        for q in items:
            p = stage.GetPrimAtPath(q['prim_path'])
            if not p.IsActive():
                continue
            xf = UsdGeom.Xformable(p)
            mat = xf.GetLocalTransformation()
            if q['z_m'] >= 3.4:
                t=mat.ExtractTranslation()
                mat.SetTranslateOnly(t+Gf.Vec3d(.5*nx,.5*ny,-1.5))
            elif damage._sub_of(q['category']) in ('wall','door','bay_roof'):
                deform=Gf.Matrix4d(1.)
                deform[2]=Gf.Vec4d(.5*nx/3.5,.5*ny/3.5,2./3.5,0.)
                mat=mat*deform
            xf.ClearXformOpOrder()
            xf.AddTransformOp().Set(mat)
    specs = debris_specs(rec,len(removed)) if removed else []
    mats = planks.materials(stage,root)
    skin_mats = {}
    for name in set(mh.palette_skins(palette).values()):
        tex,tint = mh.palette_texture(stage,root,name)
        if tex:
            skin_mats[name] = planks.skin_material(stage,root+'/DebrisLooks/'+name,tex,tint)
    # A cache bake is always centred; direct callers may request a world pose.
    from .quake_suburban import _world
    for s in specs:
        s['x'],s['y'] = _world(s['x'],s['y'],rec)
        s['yaw'] += rec['yaw']
    planks.build(stage,root+'/rubble',specs,mats,1.,skin_mats=skin_mats)
    rec['debris_specs'] = specs
    rec['loose_count'] = len(loose)
    rec['authored_debris_count'] = len(specs)
    static=[q['prim_path'] for q in items if stage.GetPrimAtPath(q['prim_path']).IsActive()]+torn_static
    from pxr import Usd
    rubble=stage.GetPrimAtPath(root+'/rubble')
    if rubble:
        static += [str(p.GetPath()) for p in Usd.PrimRange(rubble) if p.IsA(UsdGeom.Mesh)]
    return loose,static


def cache_key(rec):
    # Three deterministic variants per style/mode/side/palette. The cache
    # includes its recipe revision so old damage is never silently reused.
    spec = {k:rec.get(k) for k in ('style','palette','mode','side')}
    # Side never reaches geometry for an intact house or a full collapse:
    # pristine authors no failure, while collapse removes every module and
    # distributes rubble over the whole footprint. Keeping four hashes for
    # those identical results multiplied a 1 km cache for no visual benefit.
    if spec['mode'] in ('pristine','collapse'):
        spec['side'] = '*'
    spec.update(revision=REVISION,variant=int(rec['seed'])%3)
    return hashlib.sha256(json.dumps(spec,sort_keys=True).encode()).hexdigest()[:18],spec


def bake_house(rec, directory):
    from pxr import Usd, UsdGeom
    key,spec = cache_key(rec)
    os.makedirs(directory,exist_ok=True)
    path=os.path.join(directory,key+'.usda')
    sidecar=os.path.join(directory,key+'.json')
    if os.path.exists(path) and os.path.exists(sidecar):
        with open(sidecar) as f:
            return path,json.load(f)
    local=dict(rec,x=0.,y=0.,yaw=0.,yaw_deg=0.,id='house',seed=1000+spec['variant'])
    stage=Usd.Stage.CreateNew(path)
    UsdGeom.SetStageMetersPerUnit(stage,1.)
    UsdGeom.SetStageUpAxis(stage,'Z')
    stage.SetDefaultPrim(UsdGeom.Xform.Define(stage,'/House').GetPrim())
    loose,static=author_house(stage,local,'/House')
    local.update(loose_paths=loose,static_paths=static,physics_ready=not loose)
    local.update(cache_spec=spec,cache_key=key)
    stage.GetRootLayer().Save()
    with open(sidecar,'w') as f:
        json.dump(local,f,indent=2)
    return path,local


def assemble_house(stage, rec, parent, directory):
    from pxr import Gf, UsdGeom
    path,meta=bake_house(rec,directory)
    if not meta.get('physics_ready',False):
        raise RuntimeError('House cache needs isolated physics bake before assembly: '+path)
    root=parent+'/'+rec['id']
    prim=UsdGeom.Xform.Define(stage,root).GetPrim()
    prim.GetReferences().AddReference(path)
    xf=UsdGeom.Xformable(prim)
    xf.AddTranslateOp().Set(Gf.Vec3d(rec['x'],rec['y'],0.))
    xf.AddRotateZOp().Set(float(rec['yaw']))
    rec.update(prim=root,cache=path,cache_key=meta['cache_key'])
    for k in ('failure_sides','reach_m','removed_modules','foundation_response','authored_debris_count',
              'support_cascade_modules','removed_face_area_m2','collapse_zones','fracture_report','physics_bake','fracture_footprints'):
        if k in meta:
            rec[k]=meta[k]
    # Compact placement-space interaction footprint; no physics is enabled.
    rec['debris_specs']=meta['debris_specs']
    return root
