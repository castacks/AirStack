"""Actual house skins fractured into irregular survivors and gravity debris.

Reuses the fire fracture partition/judge pattern and its UV projection. Cuts
open kit surfaces WITHOUT spanning their openings with fabricated thick caps.
Only detached cells are probabilistically thinned; surviving bearings are not.
Loose fragments must be settled per house before this cache may be assembled.
"""
import math
import numpy as np


def surface_cells(mesh, rng, n=30, depth=0):
    from . import fracture as fr
    seeds=fr._seeds(mesh,n,rng,mode='uniform')
    result=[]
    for i,p in enumerate(seeds):
        piece=mesh
        distances=np.linalg.norm(seeds-p,axis=1)
        for j in np.argsort(distances):
            if j==i or distances[j]<1e-8:
                continue
            if piece is None or not len(piece.faces):
                break
            if distances[j]/2>np.linalg.norm(piece.vertices-p,axis=1).max():
                break
            piece=fr.slice_plane(piece,(p-seeds[j])/distances[j],(p+seeds[j])/2,cap=False)
        if piece is None or not len(piece.faces) or piece.area<.018:
            continue
        if max(piece.extents)>3.4 and depth<2:
            result.extend(surface_cells(piece,rng,12,depth+1))
        else:
            result.append(piece)
    if not result:
        raise RuntimeError('Real house surface fracture produced no cells')
    return result


def source_skin(stage,path,parent):
    """Read the dominant opaque source skin, including floors and sloped roofs."""
    from pxr import Usd,UsdGeom,UsdShade
    from . import fire_collapse as fc, urban_fire as uf, soot_bake as sb
    ctx=dict(stage=stage,parent=parent)
    cache=UsdGeom.XformCache()
    candidates=[]
    for prim in Usd.PrimRange(stage.GetPrimAtPath(path),Usd.TraverseInstanceProxies()):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        arr=uf._mesh_arrays(prim)
        if arr is None:
            continue
        uvname,_=fc._uv_primvar(prim)
        if not uvname:
            continue
        M=np.asarray(cache.GetLocalToWorldTransform(prim))
        pts=arr['points']@M[:3,:3]+M[3,:3]
        groups=[(list(s.GetIndicesAttr().Get()),s.GetPrim()) for s in
                UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(prim))]
        for ids,target in groups or [(None,prim)]:
            tri,_,slots=sb.triangles(arr['counts'],arr['indices'],ids)
            if not len(tri):
                continue
            mat=UsdShade.MaterialBindingAPI(target).ComputeBoundMaterial(materialPurpose=UsdShade.Tokens.full)[0]
            if not mat:
                continue
            mp=str(mat.GetPath()); tex=fc._basecolor_url(mat)
            if fc.facade_class(mp,tex) in ('glass','fake'):
                continue
            corners=pts[tri]
            area=np.linalg.norm(np.cross(corners[:,1]-corners[:,0],corners[:,2]-corners[:,0]),axis=1).sum()/2
            uv=sb._corner_uv(tri,slots,arr['uv'],arr['interp'],arr['uv_indices'])
            candidates.append(dict(area=area,tris=corners,uv=uv,uvname=uvname,
                uvspan=tuple(np.ptp(uv.reshape(-1,2),axis=0)),mat=mp,tex=tex,glass=False))
    if not candidates:
        raise RuntimeError('No original opaque skin found: '+path)
    skin=max(candidates,key=lambda s:s['area'])
    skin['mat']=fc.rehome_material(ctx,path,skin)
    return ctx,skin


def keep_grounded_cells(cells, candidates, base_z, tolerance=.045):
    """A ragged standing remnant must connect to the original bottom edge.

    Only used for ground-storey wall remnants, never to pin a loose fragment
    over a vanished floor. Surface-cell boxes are in the unrotated house frame.
    """
    keep={i for i in candidates if cells[i].bounds[0,2]<=base_z+tolerance}
    while True:
        new={i for i in candidates if i not in keep and any(
            np.all(cells[i].bounds[0]<=cells[j].bounds[1]+tolerance) and
            np.all(cells[j].bounds[0]<=cells[i].bounds[1]+tolerance) for j in keep)}
        if not new:
            return keep
        keep.update(new)


def author(stage,root,items,bounds,broken,rec):
    from pxr import Gf,UsdGeom,UsdShade
    from shapely.geometry import box,Point
    from shapely.ops import unary_union
    from . import fracture as fr,fire_collapse as fc,planks,damage
    from .quake_suburban_bake import NORMAL
    mode=rec['mode']
    report=dict(source_modules=0,retained_fragments=0,loose_fragments=0,consumed_fragments=0,
                torn_modules=[],bottom_fractures=0)
    if mode not in ('partial_collapse','collapse','soft_storey','foundation'):
        return [],[],report
    if rec['x'] or rec['y'] or rec['yaw']:
        raise ValueError('Fracture cache must be authored at the origin')
    # No runtime installer, and no SimulationApp required to cut geometry.
    if fr._vtk() is None:
        raise RuntimeError('VTK is required for the bounded surface cuts')
    nx,ny=NORMAL[rec['side']]
    half=(rec['W'] if nx else rec['D'])/2
    shapes=[box(b[0],b[1],b[3],b[4]) for b in bounds]
    failure=unary_union([shapes[i] for i in broken])
    loose=[]; statics=[]
    core=planks.materials(stage,root)['stud']
    for i,q in enumerate(items):
        role=damage._sub_of(q['category']); b=bounds[i]
        level=float(q['z_m'])
        if role=='floor' and level<.1:
            continue
        foundation=(mode=='foundation' and role in ('wall','door') and level<.1 and
            max(nx*x+ny*y for x in (b[0],b[3]) for y in (b[1],b[4]))>half-5.1)
        edge=(mode=='partial_collapse' and i not in broken and role in ('wall','floor','roof','bay_roof') and
              not failure.is_empty and shapes[i].distance(failure)<.3)
        if i not in broken and not foundation and not edge:
            continue
        path=q['prim_path']
        mesh=fr.prim_to_mesh(stage,path)
        if mesh is None:
            raise RuntimeError('Unreadable fracture source '+path)
        ctx,skin=source_skin(stage,path,root)
        rng=np.random.default_rng(rec['seed']+701+i*1009)
        cells=surface_cells(mesh,rng,n=min(75,max(20,int(mesh.area/1.8))))
        phase=float(rng.uniform(0,6.28))
        def detached(c):
            p=c.centroid
            along=-ny*p[0]+nx*p[1]
            wobble=.20*math.sin(along*2.4+phase)+.10*math.sin(along*5.7-phase)
            if foundation:
                return p[2]<.75+wobble
            if edge:
                return failure.distance(Point(p[0],p[1]))<.65+wobble
            # Some ground wall fragments remain as low, ragged stubs, as in
            # suburban fire. Upper fragments never independently stay aloft.
            if mode=='partial_collapse' and role=='wall' and level<.1:
                return p[2]>.65+wobble
            return True
        keep={k for k,c in enumerate(cells) if not detached(c)}
        if role=='wall' and level<.1 and not foundation:
            keep=keep_grounded_cells(cells,keep,b[2])
        dropped=[k for k in range(len(cells)) if k not in keep]
        # Fire's large-biased consumption pattern, but no fire material:
        # omitted surface fragments are represented by the authored stock.
        ordered=sorted(dropped,key=lambda k:-cells[k].area)
        count=int(.40*len(ordered))
        pool=ordered[:min(len(ordered),max(count,int(count*1.6)))]
        consumed=set(int(k) for k in rng.choice(pool,count,replace=False)) if count else set()
        for k,c in enumerate(cells):
            if k in consumed:
                continue
            standing=k in keep
            # Refuse the giant spanning fragments the first review rejected.
            if not standing and max(c.extents)>3.5:
                consumed.add(k)
                continue
            branch=root+'/structure/torn' if standing else root+'/fractured'
            dst=branch+'/m%03d_f%03d'%(i,k)
            fr._write_mesh(stage,dst,c)
            UsdGeom.Mesh.Get(stage,dst).CreateDoubleSidedAttr(True)
            UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(dst)).Bind(core)
            if fc.skin_fragment(ctx,skin,dst)!='uv':
                raise RuntimeError('Source UVs were not carried onto '+dst)
            if standing:
                statics.append(dst)
            else:
                # Start real fragments already off balance so disconnected
                # wall cells cannot remain an interlocked vertical mosaic.
                xf=UsdGeom.Xformable(stage.GetPrimAtPath(dst))
                xf.AddRotateXYZOp().Set(Gf.Vec3f(float(rng.uniform(-24,24)),float(rng.uniform(-24,24)),float(rng.uniform(-14,14))))
                loose.append(dst)
        stage.GetPrimAtPath(path).SetActive(False)
        report['source_modules']+=1
        report['consumed_fragments']+=len(consumed)
        if edge or foundation:
            report['torn_modules'].append(path)
        report['bottom_fractures']+=int(foundation)
    report.update(retained_fragments=len(statics),loose_fragments=len(loose))
    print('[suburban_quake] actual fracture '+str(report),flush=True)
    return loose,statics,report
