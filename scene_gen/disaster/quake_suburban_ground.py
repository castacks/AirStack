"""Material-matched fractured surfaces and their shared displacement field.

Geometry and object responses consume the same trace records. No global
quake-flow constants or urban material routing are changed.
"""
import math
import random


DEFAULT_BOUNDS = (-125.0, -125.0, 125.0, 125.0)


def traces_for_scene(houses, soil, seed=10, bounds=DEFAULT_BOUNDS):
    rng=random.Random(seed+404)
    traces=[]
    # M5.5 compiles a geometrically described soft-soil zone with rate 0:
    # it is useful provenance, but must not author liquefaction ruptures when
    # the magnitude-duration gate says liquefaction is absent.
    if soil and float(soil.get('rate',0.)) > 0.:
        cx,cy=soil['center']
        heading=math.radians(soil.get('angle_deg',0))
        for off in (-.45,0.,.45):
            length=soil['rx_m']*rng.uniform(1.15,1.6)
            x=cx-math.cos(heading)*length/2-math.sin(heading)*off*soil['ry_m']
            y=cy-math.sin(heading)*length/2+math.cos(heading)*off*soil['ry_m']
            traces.append(_trace(x,y,heading,length,rng,len(traces),bounds))
    for h in houses:
        if h['mode'] != 'foundation':
            continue
        from .quake_suburban import _world
        # Bearing failure propagates outward from an existing footprint.
        x,y=_world(h['W']/2+.6,h['D']/2+.6,h)
        heading=math.radians(h['yaw']+rng.uniform(5,75))
        traces.append(_trace(x,y,heading,rng.uniform(8,16),rng,len(traces),bounds))
    return traces


def _trace(x,y,heading,length,rng,index,bounds=DEFAULT_BOUNDS):
    xmin,ymin,xmax,ymax=map(float,bounds)
    points=[(x,y)]
    for _ in range(max(2,int(length/2.))):
        heading+=rng.uniform(-.12,.12)
        x+=2*math.cos(heading)
        y+=2*math.sin(heading)
        # Keep the tapered endpoint just inside the authored plate.  The old
        # 124 m literal was the same one-metre inset for a 250 m review.
        if not (xmin+1.<x<xmax-1. and ymin+1.<y<ymax-1.):
            break
        points.append((x,y))
    return dict(id='rupture_%02d'%index,points=points,width=rng.uniform(.3,.65),
                band=rng.uniform(2.4,3.6),heave=rng.uniform(.18,.42),
                drop=rng.uniform(.12,.3),depth=rng.uniform(.65,1.1))


def fault_sample(x,y,traces):
    """Signed elevation, nearest crack distance and down-slope direction."""
    best=None
    for tr in traces:
        pts=tr['points']
        for i,(a,b) in enumerate(zip(pts,pts[1:])):
            vx,vy=b[0]-a[0],b[1]-a[1]
            length=math.hypot(vx,vy)
            if length<1e-8:
                continue
            t=max(0.,min(1.,((x-a[0])*vx+(y-a[1])*vy)/(length*length)))
            px,py=a[0]+t*vx,a[1]+t*vy
            distance=math.hypot(x-px,y-py)
            if best is not None and distance>=best['distance']:
                continue
            nx,ny=-vy/length,vx/length
            signed=(x-px)*nx+(y-py)*ny
            taper=math.sin(math.pi*(i+t)/max(1,len(pts)-1))**.5
            half_width=tr['width']/2*taper
            edge=max(0.,distance-half_width)
            weight=max(0.,1.-edge/max(.001,tr['band']*taper))**1.5*taper
            z=(tr['heave'] if signed>=0 else -tr['drop'])*weight
            if distance<half_width:
                z=-tr['depth']*taper
            best=dict(z=z,distance=distance,signed=signed,normal=(nx,ny),
                      opening=distance<half_width,
                      downhill=(-nx,-ny),weight=weight,id=tr['id'])
    return best or dict(z=0.,distance=1e9,signed=0.,normal=(0.,1.),
                        downhill=(0.,-1.),weight=0.,id=None,opening=False)


def _polys(geometry):
    if geometry.is_empty:
        return []
    if geometry.geom_type=='Polygon':
        return [geometry]
    return [p for part in geometry.geoms for p in _polys(part)] if hasattr(geometry,'geoms') else []


def trace_envelope(trace,opening=False):
    """Feather the cut itself to a point; no rectangular endpoint or border."""
    from shapely.geometry import LineString,Point
    from shapely.ops import unary_union
    line=LineString(trace['points'])
    n=max(4,int(line.length/(min(.15,trace['width']/3) if opening else .6)))
    radius=trace['width']/2+(0. if opening else trace['band'])
    discs=[]
    for i in range(1,n):
        t=i/n
        r=radius*math.sin(math.pi*t)**.5
        p=line.interpolate(t,normalized=True)
        discs.append(Point(p.x,p.y).buffer(r,quad_segs=8))
    return unary_union(discs)


def _mesh(stage,path,geom,z_at,material,uv_coeff):
    import numpy as np
    from shapely.ops import triangulate
    from pxr import Gf,Sdf,UsdGeom,UsdShade,Vt
    points,indices,normals,uv=[] ,[],[],[]
    for polygon in _polys(geom):
        for tri in triangulate(polygon):
            if not polygon.covers(tri) or tri.area<1e-7:
                continue
            xy=list(tri.exterior.coords)[:3]
            xyz=[(x,y,float(z_at(x,y))) for x,y in xy]
            normal=np.cross(np.subtract(xyz[1],xyz[0]),np.subtract(xyz[2],xyz[0]))
            if normal[2]<0:
                xyz.reverse(); xy.reverse(); normal=-normal
            normal/=max(1e-12,np.linalg.norm(normal))
            offset=len(points)
            points.extend(xyz)
            indices.extend((offset,offset+1,offset+2))
            normals.extend([tuple(normal)]*3)
            uv.extend(tuple(np.array([x,y,1.])@uv_coeff) for x,y in xy)
    mesh=UsdGeom.Mesh.Define(stage,path)
    mesh.GetPointsAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*p) for p in points]))
    mesh.GetFaceVertexCountsAttr().Set([3]*(len(indices)//3))
    mesh.GetFaceVertexIndicesAttr().Set(indices)
    mesh.GetNormalsAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*n) for n in normals]))
    mesh.SetNormalsInterpolation('vertex')
    mesh.GetSubdivisionSchemeAttr().Set('none')
    UsdGeom.PrimvarsAPI(mesh).CreatePrimvar('st',Sdf.ValueTypeNames.TexCoord2fArray,'vertex').Set(
        Vt.Vec2fArray([Gf.Vec2f(*p) for p in uv]))
    if points:
        arr=np.array(points)
        mesh.GetExtentAttr().Set([Gf.Vec3f(*arr.min(0)),Gf.Vec3f(*arr.max(0))])
    else:
        mesh.GetExtentAttr().Set([Gf.Vec3f(0.),Gf.Vec3f(0.)])
    if material:
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(UsdShade.Material(stage.GetPrimAtPath(material)))
    return mesh.GetPrim()


def read_surfaces(stage,root):
    """Actual authored ground polygons, material bindings AND UV transforms."""
    import numpy as np
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    from pxr import Usd,UsdGeom,UsdShade
    result=[]
    xf=UsdGeom.XformCache()
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        m=UsdGeom.Mesh(prim)
        pts=m.GetPointsAttr().Get()
        if pts is None or len(pts)<3:
            continue
        matrix=xf.GetLocalToWorldTransform(prim)
        xyz=np.array([tuple(matrix.Transform(p)) for p in pts])
        if np.ptp(xyz[:,2])>.002:
            continue
        idx=list(m.GetFaceVertexIndicesAttr().Get() or [])
        counts=list(m.GetFaceVertexCountsAttr().Get() or [])
        polygons=[]; offset=0
        for n in counts:
            p=Polygon(xyz[idx[offset:offset+n],:2])
            offset+=n
            if p.is_valid and p.area>1e-7:
                polygons.append(p)
        if not polygons:
            continue
        st=UsdGeom.PrimvarsAPI(m).GetPrimvar('st')
        tex=st.ComputeFlattened() if st else None
        design=np.column_stack((xyz[:,:2],np.ones(len(xyz))))
        if tex is not None and len(tex)==len(xyz):
            coeff=np.linalg.lstsq(design,np.array(tex),rcond=None)[0]
        elif tex is not None and len(tex)==len(idx):
            coeff=np.linalg.lstsq(design[idx],np.array(tex),rcond=None)[0]
        else:
            coeff=np.array([[.25,0],[0,.25],[0,0]])
        mat=UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
        result.append(dict(path=str(prim.GetPath()),geometry=unary_union(polygons),
            z=float(xyz[0,2]),material=str(mat.GetPath()) if mat else None,uv=coeff,
            color=list(m.GetDisplayColorAttr().Get() or []),
            opacity=list(m.GetDisplayOpacityAttr().Get() or [])))
    return sorted(result,key=lambda r:-r['z'])


def author(stage,parent,traces,seed=10,bounds=DEFAULT_BOUNDS):
    """Cut and replace bands with raised/broken pieces of THAT ground.

    Jittered Voronoi pavement/turf polygons are geometry replacement, not
    overlaid generic damaged-asphalt textures. Original material and UV scale
    continue over the pieces. Both below-grade soil and open seams are real.
    """
    import numpy as np
    from scipy.spatial import Voronoi
    from shapely.geometry import LineString,Polygon,box
    from shapely.ops import unary_union
    from pxr import UsdGeom
    from . import quake_flow as qf
    xmin,ymin,xmax,ymax=map(float,bounds)
    plate=box(xmin,ymin,xmax,ymax)
    lines=[LineString(t['points']) for t in traces if len(t['points'])>=2]
    if not lines:
        return dict(traces=traces,pieces=0)
    valid=[t for t in traces if len(t['points'])>=2]
    band=unary_union([trace_envelope(t) for t in valid]).intersection(plate)
    opening=unary_union([trace_envelope(t,opening=True) for t in valid])
    surfaces=read_surfaces(stage,parent+'/ground')
    if not surfaces:
        raise RuntimeError('no actual ground surfaces/materials to fracture')
    scope=parent+'/quake_ground'
    UsdGeom.Scope.Define(stage,scope)
    rng=np.random.default_rng(seed+810)
    # Seed lattice only indexes cells; jitter changes their borders and size.
    sites=[]
    from shapely.geometry import Point
    padded=band.buffer(4.)
    px0,py0,px1,py1=padded.bounds
    for x in np.arange(max(xmin-5.,px0),min(xmax+5.,px1)+1.6,1.6):
        for y in np.arange(max(ymin-5.,py0),min(ymax+5.,py1)+1.6,1.6):
            if padded.contains(Point(x,y)):
                sites.append((x+rng.uniform(-.65,.65),y+rng.uniform(-.65,.65)))
    far=max(xmax-xmin,ymax-ymin)*2.+10.
    sites.extend([(xmin-far,ymin-far),(xmin-far,ymax+far),
                  (xmax+far,ymax+far),(xmax+far,ymin-far)])
    vor=Voronoi(np.array(sites))
    cells=[]
    for rid in vor.point_region[:-4]:
        region=vor.regions[rid]
        if not region or -1 in region:
            continue
        p=Polygon(vor.vertices[region]).intersection(band).difference(opening)
        if p.area>.03:
            cells.append(p.buffer(-.025,join_style=2))
    covered=Polygon()
    made=0
    material_counts={}
    mats=qf.materials(stage,parent)
    soil=qf._c_look_at(stage,parent,mats,'soil',tag='suburban')
    side_points,side_faces=[],[]
    for surface in surfaces:
        area=surface['geometry'].intersection(band)
        if area.is_empty:
            continue
        # Preserve the original geometry/material outside the disturbed band.
        remain=surface['geometry'].difference(band)
        _mesh(stage,surface['path'],remain,lambda x,y,z=surface['z']:z,
              surface['material'],surface['uv'])
        visible=area.difference(covered)
        covered=covered.union(area)
        if visible.area<.02:
            continue
        for cell in cells:
            patch=cell.intersection(visible)
            if patch.area<.025:
                continue
            z_at=lambda x,y,z=surface['z']:z+fault_sample(x,y,traces)['z']
            prim=_mesh(stage,scope+'/surface_piece_%04d'%made,patch,z_at,
                       surface['material'],surface['uv'])
            # Road paint uses displayColor rather than a bound material.
            # Preserve that too, otherwise the markings turn default grey.
            if surface['color']:
                UsdGeom.Mesh(prim).CreateDisplayColorAttr([surface['color'][0]])
            if surface['opacity']:
                UsdGeom.Mesh(prim).CreateDisplayOpacityAttr([surface['opacity'][0]])
            # Actual turf/asphalt thickness, not paper-thin floating tiles.
            # Paint is carried by the surface below, not its own thick slab.
            if surface['z']<.19:
                for poly in _polys(patch):
                    ring=list(poly.exterior.coords)
                    for a,b in zip(ring,ring[1:]):
                        za,zb=z_at(*a),z_at(*b)
                        n=len(side_points)
                        side_points.extend([(a[0],a[1],za),(b[0],b[1],zb),
                            (b[0],b[1],zb-.16),(a[0],a[1],za-.16)])
                        side_faces.append((n,n+1,n+2,n+3))
            made+=1
            key=surface['material'] or 'unbound'
            material_counts[key]=material_counts.get(key,0)+1
    # A dark exposed subsoil bed is visible through the seams/open rupture.
    qf._c_geom_mesh(dict(stage=stage,parent=scope,tag='suburban',bounds=tuple(bounds),authored=[]),
                    'broken_edges',side_points,side_faces,soil)
    _mesh(stage,scope+'/exposed_subsoil',band,
          lambda x,y:min(-.12,fault_sample(x,y,traces)['z']-.12),
          str(soil.GetPath()),np.array([[.6,0],[0,.6],[0,0]]))
    return dict(traces=traces,pieces=made,material_counts=material_counts,
                disturbed_area_m2=band.area,source_surfaces=len(surfaces))
