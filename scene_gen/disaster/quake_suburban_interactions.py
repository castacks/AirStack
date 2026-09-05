"""Placement-only responses to the same ruptures used to author the ground.

Cached house rubble is never simulated again. Only affected placements get
USD overrides; source cache files and unaffected placements remain unchanged.
"""
import math
import random
import zlib

from .quake_suburban_ground import fault_sample


def support_plane(x, y, width, depth, traces, yaw=0.):
    """Fit surviving ground contacts, not the bottom of an open fissure.

    A rigid footprint spanning a crack bridges it. The plane is raised to
    the highest sampled contact after fitting, avoiding penetration of the
    upthrown lip. A small object wholly inside an opening can drop into it.
    """
    import numpy as np
    c,s=math.cos(math.radians(yaw)),math.sin(math.radians(yaw))
    samples=[]
    for u in (-width/2,0.,width/2):
        for v in (-depth/2,0.,depth/2):
            dx,dy=c*u-s*v,s*u+c*v
            q=fault_sample(x+dx,y+dy,traces)
            samples.append((dx,dy,q))
    contacts=[p for p in samples if not p[2]['opening']]
    if len(contacts)<3:
        contacts=samples
    a=np.array([[u,v,1.] for u,v,q in contacts])
    z=np.array([q['z'] for u,v,q in contacts])
    gx,gy,dz=np.linalg.lstsq(a,z,rcond=None)[0]
    # A rigid foundation/footplate bridges abrupt offsets; don't convert a
    # 40 cm discontinuity under a tiny post into a 90-degree ground normal.
    slope=math.hypot(gx,gy)
    if slope>math.tan(math.radians(22)):
        scale=math.tan(math.radians(22))/slope
        gx*=scale; gy*=scale
    dz=max(q['z']-gx*u-gy*v for u,v,q in contacts)
    nearest=max(samples,key=lambda p:abs(p[2]['z']))[2]
    return dict(gx=float(gx),gy=float(gy),dz=float(dz),
                tilt_deg=math.degrees(math.atan(math.hypot(gx,gy))),
                fault_id=nearest['id'],affected=any(abs(p[2]['z'])>.003 for p in samples))


def plane_matrix(x,y,z,plane):
    from pxr import Gf
    n=Gf.Vec3d(-plane['gx'],-plane['gy'],1.).GetNormalized()
    r=Gf.Matrix4d(1.).SetRotate(Gf.Rotation(Gf.Vec3d(0,0,1),n))
    return (Gf.Matrix4d(1.).SetTranslate(Gf.Vec3d(-x,-y,-z))*r*
            Gf.Matrix4d(1.).SetTranslate(Gf.Vec3d(x,y,z+plane['dz'])))


def world_delta(prim, delta):
    """Compose in world space, preserving an existing parent yaw and scale."""
    from pxr import UsdGeom
    cache=UsdGeom.XformCache()
    parent=cache.GetParentToWorldTransform(prim)
    xf=UsdGeom.Xformable(prim)
    mat=xf.GetLocalTransformation()*parent*delta*parent.GetInverse()
    xf.ClearXformOpOrder()
    xf.AddTransformOp(opSuffix='quakeResponse').Set(mat)


def mesh_points(prim):
    from pxr import Gf,Usd,UsdGeom
    cache=UsdGeom.XformCache()
    points=[]
    for p in Usd.PrimRange(prim,Usd.TraverseInstanceProxies()):
        if not p.IsA(UsdGeom.Mesh):
            continue
        pts=UsdGeom.Mesh(p).GetPointsAttr().Get()
        if pts is None:
            continue
        mat=cache.GetLocalToWorldTransform(p)
        points.extend(tuple(mat.Transform(Gf.Vec3d(*v))) for v in pts)
    return points


def collapse_footprints(houses):
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    from . import planks
    from .quake_suburban import _world
    result=[]
    for h in houses:
        shapes=[]
        for spec in h.get('debris_specs',[]):
            pts,_=planks._box(spec)
            shapes.append(Polygon([_world(p[0],p[1],h) for p in pts[:4]]))
        for polygon in h.get('fracture_footprints',[]):
            shapes.append(Polygon([_world(p[0],p[1],h) for p in polygon]))
        if shapes:
            result.append((h,unary_union(shapes)))
    return result


def adapt_rubble(stage, house, traces):
    """Override only affected 8-point stock pieces in the composed cache."""
    from pxr import Gf,Usd,UsdGeom,Vt
    root=stage.GetPrimAtPath(house['prim']+'/house/rubble')
    if not root:
        return 0
    changed=0
    cache=UsdGeom.XformCache()
    for prim in Usd.PrimRange(root):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        mesh=UsdGeom.Mesh(prim)
        pts=list(mesh.GetPointsAttr().Get() or [])
        normals=list(mesh.GetNormalsAttr().Get() or [])
        mat=cache.GetLocalToWorldTransform(prim); inv=mat.GetInverse()
        dirty=False
        for start in range(0,len(pts),8):
            block=[mat.Transform(Gf.Vec3d(*p)) for p in pts[start:start+8]]
            if len(block)!=8:
                raise ValueError('quake rubble no longer uses 8-point stock')
            x=sum(p[0] for p in block)/8; y=sum(p[1] for p in block)/8
            width=max(p[0] for p in block)-min(p[0] for p in block)
            depth=max(p[1] for p in block)-min(p[1] for p in block)
            plane=support_plane(x,y,width,depth,traces)
            if not plane['affected']:
                continue
            delta=plane_matrix(x,y,min(p[2] for p in block),plane)
            local=mat*delta*inv
            for j,p in enumerate(pts[start:start+8]):
                pts[start+j]=Gf.Vec3f(*local.Transform(Gf.Vec3d(*p)))
            for j in range(start//8*24,min(start//8*24+24,len(normals))):
                normals[j]=Gf.Vec3f(*local.TransformDir(Gf.Vec3d(*normals[j])).GetNormalized())
            changed+=1; dirty=True
        if dirty:
            mesh.GetPointsAttr().Set(Vt.Vec3fArray(pts))
            mesh.GetNormalsAttr().Set(Vt.Vec3fArray(normals))
            mesh.GetExtentAttr().Set(UsdGeom.PointBased(mesh).ComputeExtent(pts))
    return changed


ROOT_BALL_SOIL_MATERIAL = "/World/stage/generated/QuakeLooks/soil"


def bind_root_ball_materials(stage, material_path=ROOT_BALL_SOIL_MATERIAL):
    """Bind generated uprooted-tree root masses to the quake soil material.

    Early frozen suburban-quake scenes created these meshes without a binding
    or display colour. Keep this idempotent so the freeze launcher can repair
    an already-assembled scene without repeating its CPU build.
    """
    from pxr import Usd, UsdGeom, UsdShade

    material = UsdShade.Material.Get(stage, material_path)
    if not material or not material.GetPrim().IsValid():
        return 0
    rebound = 0
    for prim in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()):
        if not (prim.IsA(UsdGeom.Mesh)
                and prim.GetName().endswith("_root_ball")):
            continue
        bound = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial(
            materialPurpose=UsdShade.Tokens.full)[0]
        if bound and bound.GetPrim().IsValid():
            continue
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(
            material, UsdShade.Tokens.strongerThanDescendants)
        rebound += 1
    return rebound


def author(stage,houses,placements,trees,traces,soil,seed=10,tree_keepout=None):
    from pxr import Gf,UsdGeom
    from shapely.geometry import MultiPoint,box
    from . import vegetation
    from .quake_suburban import soft_weight
    changes=[]
    for h in houses:
        plane=support_plane(h['x'],h['y'],h['W'],h['D'],traces,h['yaw'])
        if plane['affected']:
            # The shell has its own uneven foundation failure. Add local
            # ground movement to it, never rotate the entire rubble apron.
            shell=stage.GetPrimAtPath(h['prim']+'/house/structure')
            world_delta(shell,plane_matrix(h['x'],h['y'],0.,plane))
            changes.append(dict(prim=str(shell.GetPath()),kind='house',cause='ground',**plane))
        count=adapt_rubble(stage,h,traces)
        # Real, independently settled house fragments are a separate
        # population from the 8-vertex authored stock. Move each rigidly;
        # never feed an irregular mesh to the stock's eight-point loop.
        fractured=stage.GetPrimAtPath(h['prim']+'/house/fractured')
        if fractured:
            from pxr import Usd
            for piece in Usd.PrimRange(fractured):
                if not piece.IsA(UsdGeom.Mesh):
                    continue
                points=mesh_points(piece)
                if not points:
                    continue
                lo=[min(p[k] for p in points) for k in range(3)]
                hi=[max(p[k] for p in points) for k in range(3)]
                x,y=(lo[0]+hi[0])/2,(lo[1]+hi[1])/2
                response=support_plane(x,y,hi[0]-lo[0],hi[1]-lo[1],traces)
                if response['affected']:
                    world_delta(piece,plane_matrix(x,y,lo[2],response))
                    count+=1
        if count:
            changes.append(dict(prim=h['prim'],kind='rubble',cause='ground',pieces=count))
    footprints=collapse_footprints(houses)
    for p in placements:
        path=p.get('prim_path')
        prim=stage.GetPrimAtPath(path) if path else None
        if not prim:
            continue
        kind=str(p.get('prop_kind') or p.get('category',''))
        if any(k in kind for k in ('human','ground','crosswalk','sidewalk')):
            continue
        rng=random.Random(seed+zlib.crc32(path.encode()))
        bounds=UsdGeom.BBoxCache(0,['default','render']).ComputeWorldBound(prim).ComputeAlignedRange()
        if bounds.IsEmpty():
            continue
        lo,hi=bounds.GetMin(),bounds.GetMax()
        x,y=float((lo[0]+hi[0])/2),float((lo[1]+hi[1])/2)
        width,depth=max(.15,hi[0]-lo[0]),max(.15,hi[1]-lo[1])
        plane=support_plane(x,y,width,depth,traces)
        candidates=[h for h,g in footprints if g.intersects(box(lo[0],lo[1],hi[0],hi[1]))]
        hit=None
        if candidates and any(k in kind for k in ('fence','sign','bench','table','streetlight','trash')):
            points=mesh_points(prim)
            shape=MultiPoint([(p[0],p[1]) for p in points]).convex_hull if points else None
            hit=next((h for h,g in footprints if shape is not None and h in candidates and g.intersects(shape)),None)
        fragile=any(k in kind for k in ('sign','streetlight','trash','bench','table'))
        shaking=fragile and rng.random()<.10
        topple=bool(hit or shaking or (fragile and plane['affected']))
        if not topple and not plane['affected']:
            continue
        cause='collapse' if hit else 'ground' if plane['affected'] else 'shaking'
        if topple:
            q=fault_sample(x,y,traces)
            dx,dy=(x-hit['x'],y-hit['y']) if hit else q['downhill'] if plane['affected'] else (rng.uniform(-1,1),rng.uniform(-1,1))
            axis=Gf.Vec3d(-dy,dx,0.).GetNormalized()
            angle=rng.uniform(72,88)
            delta=(Gf.Matrix4d(1.).SetTranslate(Gf.Vec3d(-x,-y,-lo[2]))*
                   Gf.Matrix4d(1.).SetRotate(Gf.Rotation(axis,angle))*
                   Gf.Matrix4d(1.).SetTranslate(Gf.Vec3d(x,y,lo[2])))
            world_delta(prim,delta)
            points=mesh_points(prim)
            if points:
                # Seat against the actual displacement under each vertex;
                # original nominal ground height stays in the same frame.
                shift=max(float(lo[2])+fault_sample(v[0],v[1],traces)['z']-v[2] for v in points)
                world_delta(prim,Gf.Matrix4d(1.).SetTranslate(Gf.Vec3d(0,0,shift)))
            plane['topple_deg']=angle
        else:
            world_delta(prim,plane_matrix(x,y,float(lo[2]),plane))
        changes.append(dict(prim=path,kind=kind,cause=cause,house_id=hit['id'] if hit else None,**plane))
    for t in trees:
        x,y=t['x'],t['y']; path=t['prim']
        tree_prim=stage.GetPrimAtPath(path)
        tree_xf=UsdGeom.Xformable(tree_prim)
        original_ops=tree_xf.GetOrderedXformOps()
        original_values=[op.Get() for op in original_ops]
        def restore_tree():
            tree_xf.SetXformOpOrder(original_ops)
            for op,value in zip(original_ops,original_values):
                op.Set(value)
        def clears_houses():
            if tree_keepout is None:
                return True
            b=UsdGeom.BBoxCache(0,['default','render']).ComputeWorldBound(tree_prim).ComputeAlignedRange()
            return not tree_keepout.intersects(box(b.GetMin()[0],b.GetMin()[1],b.GetMax()[0],b.GetMax()[1]))
        rng=random.Random(seed+zlib.crc32(path.encode()))
        q=fault_sample(x,y,traces)
        plane=support_plane(x,y,.8,.8,traces)
        weak=soft_weight(x,y,soil)
        uproot=t['species']!='Black_Oak' and (q['weight']>.15 or rng.random()<.20*weak)
        if uproot:
            dx,dy=q['downhill'] if q['weight']>.01 else (math.cos(rng.uniform(0,6.28)),math.sin(rng.uniform(0,6.28)))
            az=math.degrees(math.atan2(dy,dx))
            lift=.8+max(0.,q['z'])
            lean=vegetation.tip_tree(stage,path,78.,azimuth_deg=az-t['yaw'],
                  lift_m=lift,seat_band=(-.15,.05),lean_min_deg=40.)
            if not clears_houses():
                # Do not plant a valid tree and then swing its crown through
                # an intact house without modelling that impact.
                restore_tree()
                uproot=False
            else:
                a=math.radians(lean); b=math.radians(az)
                vegetation.root_ball(stage,path+'_root_ball',(x,y),lift,
                     (math.sin(a)*math.cos(b),math.sin(a)*math.sin(b),math.cos(a)),
                     .65,rng,mat_prim_path=ROOT_BALL_SOIL_MATERIAL)
                changes.append(dict(prim=path,kind='tree',cause='root_failure',
                                    fault_id=q['id'],tilt_deg=lean,lift_m=lift))
        if not uproot and plane['affected']:
            world_delta(stage.GetPrimAtPath(path),plane_matrix(x,y,0.,plane))
            if not clears_houses():
                restore_tree()
                world_delta(tree_prim,Gf.Matrix4d(1.).SetTranslate(Gf.Vec3d(0,0,plane['dz'])))
                plane.update(gx=0.,gy=0.,tilt_deg=0.,clearance_limited=True)
            changes.append(dict(prim=path,kind='tree',cause='ground',**plane))
    # The material is supplied at creation above. This also makes the function
    # repair-safe if a caller composes an older generated root ball first.
    bind_root_ball_materials(stage)
    return changes
