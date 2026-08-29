"""Generic visual damage for semantic-free standalone building assets.

The source building remains the identity shell.  This module infers a regular
structural bay/storey grid from its measured bounds and authors damage cues in
front of, behind, and around that shell.  Recipes are disaster-neutral: an
earthquake, fire, blast, wind, or dataset author chooses the recipe and owns
its probability/severity model.

Visible break profiles are stepped or notched on a storey/bay grid.  We do not
Voronoi-fracture the source: the CityDemo monoliths have too few source faces
for that to produce anything except conspicuous triangular plates.
"""

from dataclasses import dataclass
import math
import random


RECIPES = ("corner_loss", "roof_collapse", "soft_storey",
           "mid_storey", "partial_collapse")


@dataclass(frozen=True)
class Descriptor:
    width: float
    depth: float
    height: float
    base_z: float = 0.0
    storey_m: float = 3.4
    construction: str = "rc"

    @property
    def storeys(self):
        return max(2, int(round(self.height / self.storey_m)))

    @property
    def pitch(self):
        return self.height / self.storeys


def stepped_profile(desc, recipe):
    """Return a front-elevation damage polygon in local (x,z) metres."""
    w, h, p = desc.width, desc.height, desc.pitch
    if recipe == "corner_loss":
        # A staircase follows bays and courses; no long diagonal triangle.
        return [(0.10*w, 0.18*h), (0.50*w, 0.18*h), (0.50*w, 0.72*h),
                (0.37*w, 0.72*h), (0.37*w, 0.84*h), (0.22*w, 0.84*h),
                (0.22*w, 0.94*h), (0.10*w, 0.94*h)]
    if recipe == "roof_collapse":
        return [(-0.38*w, h-1.55*p), (0.30*w, h-1.55*p),
                (0.30*w, h-0.92*p), (0.18*w, h-0.92*p),
                (0.18*w, h-0.42*p), (-0.38*w, h-0.42*p)]
    if recipe == "soft_storey":
        return [(-0.49*w, 0.08*p), (0.49*w, 0.08*p),
                (0.49*w, 0.92*p), (-0.49*w, 0.92*p)]
    if recipe == "mid_storey":
        z = max(1.5*p, 0.42*h)
        return [(-0.49*w, z), (0.49*w, z), (0.49*w, z+0.72*p),
                (0.20*w, z+0.72*p), (0.20*w, z+0.88*p),
                (-0.18*w, z+0.88*p), (-0.18*w, z+0.70*p),
                (-0.49*w, z+0.70*p)]
    if recipe == "partial_collapse":
        return [(0.00*w, 0.06*h), (0.49*w, 0.06*h), (0.49*w, 0.94*h),
                (0.30*w, 0.94*h), (0.30*w, 0.82*h), (0.17*w, 0.82*h),
                (0.17*w, 0.63*h), (0.05*w, 0.63*h), (0.05*w, 0.38*h),
                (0.00*w, 0.38*h)]
    raise ValueError("unknown monolith damage recipe: " + str(recipe))


def rubble_specs(desc, recipe, seed=0):
    """CPU-only deterministic rubble layout; every piece is exactly seated."""
    rng = random.Random(seed)
    severity = {"roof_collapse": 0.45, "soft_storey": 0.38,
                "mid_storey": 0.50, "corner_loss": 0.62,
                "partial_collapse": 1.0}[recipe]
    n = int(max(55, min(360, desc.width * desc.height * 0.030 * severity)))
    out = []
    side_bias = 0.70 if recipe in ("corner_loss", "partial_collapse") else 0.0
    for i in range(n):
        # Mostly 15-55 cm, with a small structural tail.  Pieces are compact,
        # never thin triangular façade shards.
        u = rng.random()
        s = 0.14 + 0.52 * (u ** 2.1)
        if i < max(3, n // 18):
            kind = "slab" if i % 2 == 0 else "beam"
            sx = rng.uniform(1.2, 3.4) if kind == "slab" else rng.uniform(1.4, 3.8)
            sy = rng.uniform(0.7, 1.8) if kind == "slab" else rng.uniform(0.18, 0.35)
            sz = rng.uniform(0.14, 0.28) if kind == "slab" else rng.uniform(0.18, 0.32)
        else:
            kind = "chunk"
            sx, sy, sz = s*rng.uniform(0.75, 1.35), s*rng.uniform(0.7, 1.3), s*rng.uniform(0.55, 1.05)
        x = rng.uniform(-0.52, 0.52) * desc.width + side_bias * desc.width * rng.random()
        y = -desc.depth/2.0 - rng.uniform(0.15, max(2.0, 0.10*desc.height))
        out.append(dict(kind=kind, x=x, y=y, z=desc.base_z + sz/2.0,
                        size=(sx, sy, sz), yaw=rng.uniform(-35, 35)))
    return out


def validate(desc, recipe, seed=0):
    issues = []
    poly = stepped_profile(desc, recipe)
    if len(poly) < 4:
        issues.append("damage profile has fewer than four vertices")
    if any(not (-0.51*desc.width <= x <= 0.51*desc.width and
                   desc.base_z <= z <= desc.base_z+desc.height+1e-6)
           for x, z in poly):
        issues.append("damage profile escapes facade")
    # No three-point polygon is allowed: triangles are exactly the visual cue
    # this system is designed to avoid.
    if len(poly) == 3:
        issues.append("triangular damage profile")
    rubble = rubble_specs(desc, recipe, seed)
    if any(abs(q["z"] - (desc.base_z + q["size"][2]/2.0)) > 1e-9 for q in rubble):
        issues.append("unseated rubble")
    if any(min(q["size"]) <= 0.0 for q in rubble):
        issues.append("non-positive rubble dimension")
    return issues


def _material(stage, path, color, roughness=0.8, metallic=0.0):
    from pxr import Gf, Sdf, UsdShade
    mat = UsdShade.Material.Define(stage, Sdf.Path(path))
    sh = UsdShade.Shader.Define(stage, Sdf.Path(path + "/Shader"))
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(roughness))
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(float(metallic))
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def _box(stage, path, center, size, mat, yaw=0.0):
    from pxr import Gf, Sdf, UsdGeom, UsdShade
    cube = UsdGeom.Cube.Define(stage, Sdf.Path(path))
    cube.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(cube)
    xf.AddTranslateOp().Set(Gf.Vec3d(*center))
    xf.AddRotateZOp().Set(float(yaw))
    xf.AddScaleOp().Set(Gf.Vec3f(*size))
    UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(mat)
    return path


def _polygon(stage, path, points, mat):
    from pxr import Gf, Sdf, UsdGeom, UsdShade
    mesh = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
    mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in points])
    mesh.CreateFaceVertexCountsAttr([len(points)])
    mesh.CreateFaceVertexIndicesAttr(list(range(len(points))))
    mesh.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
    return path


def _bary2(p, a, b, c):
    """Barycentric coordinates of 2-D *p* in triangle a,b,c."""
    den = (b[1]-c[1])*(a[0]-c[0]) + (c[0]-b[0])*(a[1]-c[1])
    if abs(den) < 1e-12:
        return (1.0, 0.0, 0.0)
    u = ((b[1]-c[1])*(p[0]-c[0]) + (c[0]-b[0])*(p[1]-c[1]))/den
    v = ((c[1]-a[1])*(p[0]-c[0]) + (a[0]-c[0])*(p[1]-c[1]))/den
    return (u, v, 1.0-u-v)


def cut_shell(stage, source_root, desc, recipe):
    """Cut actual façade/roof faces while retaining the source UV mapping.

    Each source n-gon is triangulated, clipped in its dominant elevation plane
    with GEOS/Shapely, and rewritten as n-gons. New position and face-varying
    UV values are barycentrically interpolated from the source triangle. The
    visible boundary follows :func:`stepped_profile`; triangulation stays an
    implementation detail inside the surviving textured surface.
    """
    from pxr import Gf, Usd, UsdGeom
    from shapely.geometry import Polygon

    root = stage.GetPrimAtPath(source_root)
    if not root:
        return {"meshes": 0, "faces_removed": 0}
    profile = stepped_profile(desc, recipe)
    cutter_front = Polygon(profile)
    # Roof cut is deliberately a notched six-sided plan, not a rectangular
    # black plate and not a triangular hole.
    rw, rd = 0.58*desc.width, 0.55*desc.depth
    cutter_roof = Polygon([(-.48*rw, -.50*rd), (.48*rw, -.50*rd),
                           (.48*rw, .12*rd), (.30*rw, .12*rd),
                           (.30*rw, .50*rd), (-.48*rw, .50*rd)])
    xc = UsdGeom.XformCache(Usd.TimeCode.Default())
    root_inv = xc.GetLocalToWorldTransform(root).GetInverse()
    changed = removed = 0
    for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        mesh = UsdGeom.Mesh(prim)
        pts = list(mesh.GetPointsAttr().Get() or [])
        counts = list(mesh.GetFaceVertexCountsAttr().Get() or [])
        indices = list(mesh.GetFaceVertexIndicesAttr().Get() or [])
        if not pts or not counts:
            continue
        m = xc.GetLocalToWorldTransform(prim) * root_inv
        rpts = [m.Transform(Gf.Vec3d(float(q[0]), float(q[1]), float(q[2]))) for q in pts]
        uvp = UsdGeom.PrimvarsAPI(prim).GetPrimvar("uv0")
        uvvals = list(uvp.Get() or []) if uvp else []
        uvidx = list(uvp.GetIndices() or []) if uvp and uvp.IsIndexed() else []
        if uvp and str(uvp.GetInterpolation()) != "faceVarying":
            uvvals, uvidx = [], []
        new_pts, new_counts, new_idx, new_uv = [], [], [], []
        cursor = 0
        touched = False
        for count in counts:
            fidx = indices[cursor:cursor+count]
            fuv = []
            if uvvals:
                for k in range(cursor, cursor+count):
                    fuv.append(uvvals[uvidx[k] if uvidx else k])
            cursor += count
            rp = [rpts[k] for k in fidx]
            if len(rp) < 3:
                continue
            # Dominant normal and boundary proximity select only the front
            # elevation, plus the roof for the roof recipe.
            a0, b0, c0 = rp[0], rp[1], rp[2]
            n = Gf.Cross(b0-a0, c0-a0)
            front = abs(n[1]) >= max(abs(n[0]), abs(n[2])) and sum(q[1] for q in rp)/len(rp) < -0.35*desc.depth
            roof = recipe == "roof_collapse" and abs(n[2]) >= max(abs(n[0]), abs(n[1])) and sum(q[2] for q in rp)/len(rp) > 0.82*desc.height
            cutter = cutter_front if front else (cutter_roof if roof else None)
            projection = (lambda q: (float(q[0]), float(q[2]))) if front else (lambda q: (float(q[0]), float(q[1])))
            for j in range(1, len(fidx)-1):
                tri_i = (0, j, j+1)
                tri2 = [projection(rp[k]) for k in tri_i]
                tri_poly = Polygon(tri2)
                pieces = [tri_poly] if cutter is None or not tri_poly.intersects(cutter) else []
                if cutter is not None and tri_poly.intersects(cutter):
                    touched = True
                    diff = tri_poly.difference(cutter)
                    if diff.is_empty:
                        removed += 1
                    elif diff.geom_type == "Polygon":
                        pieces = [diff]
                    else:
                        # `.geoms` ONLY EXISTS ON A COLLECTION. A triangle that
                        # meets the cutter along an edge differences to a
                        # LineString — not empty, not a Polygon, and with no
                        # `.geoms` — so this raised `'LineString' object has no
                        # attribute 'geoms'` and took the whole building with
                        # it (downtowncity Building_12, 2026-08-29). A
                        # degenerate result has no area to keep, so an empty
                        # piece list is the right answer for it.
                        pieces = [g for g in getattr(diff, "geoms", ())
                                  if g.geom_type == "Polygon"]
                for piece in pieces:
                    coords = list(piece.exterior.coords)[:-1]
                    if len(coords) < 3 or piece.area < 1e-7:
                        continue
                    base = len(new_pts)
                    oa, ob, oc = (pts[fidx[k]] for k in tri_i)
                    ua = fuv[tri_i[0]] if fuv else None
                    ub = fuv[tri_i[1]] if fuv else None
                    uc = fuv[tri_i[2]] if fuv else None
                    for xy in coords:
                        w = _bary2(xy, tri2[0], tri2[1], tri2[2])
                        new_pts.append(Gf.Vec3f(*(float(w[0]*oa[k]+w[1]*ob[k]+w[2]*oc[k]) for k in range(3))))
                        if fuv:
                            new_uv.append(Gf.Vec2f(float(w[0]*ua[0]+w[1]*ub[0]+w[2]*uc[0]),
                                                  float(w[0]*ua[1]+w[1]*ub[1]+w[2]*uc[1])))
                    new_counts.append(len(coords))
                    new_idx.extend(range(base, base+len(coords)))
        if touched:
            mesh.GetPointsAttr().Set(new_pts)
            mesh.GetFaceVertexCountsAttr().Set(new_counts)
            mesh.GetFaceVertexIndicesAttr().Set(new_idx)
            mesh.GetNormalsAttr().Clear()
            if uvvals:
                uvp.Set(new_uv)
                uvp.SetIndices([])
                uvp.SetInterpolation(UsdGeom.Tokens.faceVarying)
            changed += 1
    return {"meshes": changed, "faces_removed": removed}


def author(stage, parent, desc, recipe, seed=0, source_root=None):
    """Author one generic recipe around a yaw-zero monolith at the origin."""
    from pxr import Sdf, UsdGeom
    if validate(desc, recipe, seed):
        raise ValueError("; ".join(validate(desc, recipe, seed)))
    UsdGeom.Scope.Define(stage, Sdf.Path(parent))
    looks = parent + "/Looks"
    mats = {
        "void": _material(stage, looks+"/void", (0.012, 0.014, 0.016), 0.98),
        "concrete": _material(stage, looks+"/concrete", (0.31, 0.29, 0.27), 0.92),
        "core": _material(stage, looks+"/core", (0.22, 0.19, 0.16), 0.96),
        "dust": _material(stage, looks+"/dust", (0.40, 0.37, 0.32), 1.0),
        "steel": _material(stage, looks+"/steel", (0.18, 0.11, 0.075), 0.72, 0.35),
    }
    poly = stepped_profile(desc, recipe)
    cut = cut_shell(stage, source_root, desc, recipe) if source_root else {"meshes": 0, "faces_removed": 0}
    # The recess is behind the real clipped shell. It supplies room depth and
    # prevents the far façade from shining through the opening.
    front_y = -desc.depth/2.0 + 0.22
    _polygon(stage, parent+"/damage_void",
             [(x, front_y, z) for x, z in poly], mats["void"])
    zlo, zhi = min(z for _x, z in poly), max(z for _x, z in poly)
    xlo, xhi = min(x for x, _z in poly), max(x for x, _z in poly)

    # Storey slabs visible through the recess. Their front edges are irregular
    # lengths, but every slab is quadrilateral/box geometry.
    for i in range(1, desc.storeys):
        z = desc.base_z + i*desc.pitch
        if zlo+0.15 < z < zhi-0.10:
            span = (xhi-xlo) * (0.70 + 0.22*math.sin(i*1.73))
            cx = (xlo+xhi)/2.0 + 0.08*(xhi-xlo)*math.sin(i*2.31)
            _box(stage, parent+"/slab_{:02d}".format(i),
                 (cx, front_y-0.10, z), (span, 0.75, 0.18), mats["concrete"],
                 yaw=2.0*math.sin(i*1.1) if recipe != "soft_storey" else 0.0)

    # Columns make frame damage legible. Some are shortened/buckled according
    # to recipe; no unsupported pristine rectangle is left projecting.
    ncol = max(3, int(round((xhi-xlo)/5.0))+1)
    for j in range(ncol):
        x = xlo + (xhi-xlo)*(j+0.5)/ncol
        h = max(0.6, zhi-zlo)
        if recipe in ("soft_storey", "mid_storey"):
            h *= 0.62 + 0.22*((j+1)%2)
        _box(stage, parent+"/column_{:02d}".format(j),
             (x, front_y-0.06, zlo+h/2.0), (0.34, 0.34, h), mats["core"],
             yaw=(-6.0 if j%2 else 5.0) if recipe in ("soft_storey", "mid_storey") else 0.0)

    # Roof hole gets a top-facing occluder and a sagged slab below it.
    if recipe == "roof_collapse":
        rw, rd = 0.58*desc.width, 0.55*desc.depth
        _box(stage, parent+"/roof_void", (0.02*desc.width, 0.0, desc.height-0.20),
             (0.92*rw, 0.88*rd, 0.07), mats["void"], yaw=0.0)
        _box(stage, parent+"/roof_sag", (0.02*desc.width, 0.05*desc.depth,
                                         desc.height-0.62*desc.pitch),
             (0.72*rw, 0.65*rd, 0.20), mats["concrete"], yaw=-5.0)

    # Ground rubble: compact chunks plus a few slabs/beams. All pieces are
    # arithmetically seated; the later world-point audit verifies composition.
    for i, q in enumerate(rubble_specs(desc, recipe, seed)):
        mat = mats["steel"] if q["kind"] == "beam" else (mats["concrete"] if q["kind"] == "slab" else mats["dust"])
        _box(stage, parent+"/debris_{:04d}".format(i),
             (q["x"], q["y"], q["z"]), q["size"], mat, q["yaw"])
    return {"recipe": recipe, "profile": poly, "cut": cut,
            "rubble_count": len(rubble_specs(desc, recipe, seed))}
