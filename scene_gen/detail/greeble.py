"""greeble — give a flat monolith real, connected window reveals.

WHY GEOMETRY AND NOT A MATERIAL
--------------------------------
The 112 standalone building assets are photographs on extruded solids. The
material route for adding depth to them is dead in this renderer, and that was
measured rather than assumed (`facade_displace_bench_launch_script.py`: four
panels of the same wall under a raking light):

  * OmniPBR has NO displacement input in this build — `grep -c displacement
    /isaac-sim/kit/mdl/core/Base/OmniPBR.mdl` returns 0, so the usual "enable
    Displacement on the material" step has no such control here.
  * OmniSurface HAS the inputs (`geometry_displacement_image`, `_scale`) and
    RTX Real-Time does not tessellate for them: the displaced panel rendered
    identically to the flat one — on a 200 x 200 grid, so there was plenty to
    move — while the modelled column was the only one with depth.
  * A normal map shades correctly and does nothing at a SILHOUETTE, which is
    exactly where a drone camera sees a façade.

So the depth has to be geometry. The question is only how little is needed.

THE FLOATING-WINDOW BUG, AND THE CONSTRUCTION THAT FIXES IT
------------------------------------------------------------
The first cut built each bay as four strips (cill, head, two mullions) with a
pane set back behind them, and the windows "seem floating instead of connected
to the outside wall part" (user, 2026-08-28). That is a modelling error, not a
shading one: four strips with a gap between them is a GRID WITH HOLES, so
nothing joins the outer face to the recessed pane and the reveal looks through
to daylight.

A real reveal is not a hole with a pane behind it. It is a POCKET — the wall
stays continuous and the window is the part of it that sits further back:

    outer face   bands and piers, FULL wall thickness
    reveal       the INNER SIDES of those pieces; free, because they are solid
    pane         one continuous panel per storey, set back by `reveal_m`

Nothing is a hole, so nothing can float.

NONE OF THESE BUILDINGS IS A BOX, AND THAT DECIDES THE ALGORITHM
-----------------------------------------------------------------
Measured over all 112 by points-per-mesh: 0 % box-like, 58 % simple stepped
(13-40), 18 % complex (41-200), 24 % detailed (>200). They are L-plans,
T-plans, setbacks and multi-lobed towers, each as ONE mesh.

So a bounding box — or the min/max of the geometry across a height band — is
not the wall line, it is the ENVELOPE. On `tower_03_0015` (two towers over a
podium) that spanned the gap between the towers and drew bands through open
air: horizontal slabs sticking out into space, which is what the second
attempt produced.

`storey_outline` slices the mesh at each storey and returns the plan OUTLINE,
so bands and piers just run along its edges. The construction then stops
caring how complicated the plan is, and an L-plan costs the same as a
rectangle.

PRIM COUNT
----------
Per storey the bands, piers and panes of EVERY edge are merged into three
meshes, so a storey is three prims whatever its plan.
"""

import math

BAY_M = 3.6            # bay pitch
STOREY_M = 3.6         # floor to floor
REVEAL_M = 0.42        # how far the pane sits behind the outer face
WALL_T = 0.55          # depth of the bands and piers
MULLION_W = 0.42       # width of a pier between windows
BAND_H = 0.85          # height of the spandrel band under each window


def part_trimesh(stage, part):
    """One part as a trimesh, in world space."""
    import numpy as np
    import trimesh
    from pxr import UsdGeom
    pr = stage.GetPrimAtPath(part["path"])
    if not pr or not pr.IsValid():
        return None
    m = UsdGeom.Mesh(pr)
    pts = m.GetPointsAttr().Get()
    counts = list(m.GetFaceVertexCountsAttr().Get() or [])
    idx = list(m.GetFaceVertexIndicesAttr().Get() or [])
    if pts is None or not counts:
        return None
    v = np.asarray(pts, dtype=float)
    M = np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(pr),
                   dtype=float)
    v = (np.c_[v, np.ones(len(v))] @ M)[:, :3]
    tris, k = [], 0
    for c in counts:
        for j in range(1, c - 1):
            tris.append([idx[k], idx[k + j], idx[k + j + 1]])
        k += c
    if not tris:
        return None
    return trimesh.Trimesh(vertices=v, faces=np.asarray(tris, dtype=np.int64),
                           process=False)


def storey_outline(mesh, z, min_edge=1.6):
    """The plan outline at height `z`, as [(p0, p1, outward, length)].

    A CROSS-SECTION, NOT A BOUNDING INTERVAL — see the module docstring.
    """
    import numpy as np
    if mesh is None:
        return []
    try:
        sec = mesh.section(plane_origin=(0.0, 0.0, float(z)),
                           plane_normal=(0.0, 0.0, 1.0))
        loops = sec.discrete if sec is not None else []
    except Exception:
        return []
    out = []
    for loop in loops:
        L = np.asarray(loop, dtype=float)
        if len(L) < 3:
            continue
        xy = L[:, :2]
        c = xy.mean(axis=0)
        for a, b in zip(xy[:-1], xy[1:]):
            d = b - a
            n = float(np.hypot(d[0], d[1]))
            if n < min_edge:
                continue
            # outward is the edge turned 90 deg, disambiguated against the
            # loop centroid so a concave edge still points out of the solid
            ox, oy = d[1] / n, -d[0] / n
            if float(np.dot(np.array([ox, oy]), 0.5 * (a + b) - c)) < 0:
                ox, oy = -ox, -oy
            out.append((a, b, (ox, oy), n))
    return out


def _mesh(stage, path, boxes, mat):
    """Author many yawed boxes as ONE mesh.

    Merging is what keeps this affordable: a storey's bands, piers and panes
    are three prims however many edges its plan has. Same argument
    `quake_flow._heap` makes for its chunks and `planks` for its boards.
    """
    from pxr import Gf, Sdf, UsdGeom, UsdShade, Vt
    pts, counts, idx = [], [], []
    for (cx, cy, cz, sx, sy, sz, yaw) in boxes:
        a = math.radians(yaw)
        ca, sa = math.cos(a), math.sin(a)
        b = len(pts)
        hx, hy, hz = sx / 2.0, sy / 2.0, sz / 2.0
        for dz in (-hz, hz):
            for dx, dy in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)):
                pts.append(Gf.Vec3f(float(cx + dx * ca - dy * sa),
                                    float(cy + dx * sa + dy * ca),
                                    float(cz + dz)))
        for f in ((0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4),
                  (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)):
            idx.extend([b + q for q in f])
            counts.append(4)
    if not pts:
        return None
    me = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
    me.CreatePointsAttr(Vt.Vec3fArray(pts))
    me.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
    me.CreateFaceVertexIndicesAttr(Vt.IntArray(idx))
    me.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    lo = [min(q[k] for q in pts) for k in range(3)]
    hi = [max(q[k] for q in pts) for k in range(3)]
    me.CreateExtentAttr([Gf.Vec3f(*map(float, lo)), Gf.Vec3f(*map(float, hi))])
    if mat is not None:
        UsdShade.MaterialBindingAPI(me.GetPrim()).Bind(mat)
    return path


def greeble_part(stage, root, part, mats, mesh=None, bay_m=BAY_M,
                 storey_m=STOREY_M, reveal_m=REVEAL_M, wall_t=WALL_T,
                 mullion_w=MULLION_W, band_h=BAND_H, tag="g"):
    """Bands, piers and recessed panes along the plan outline, per storey."""
    z0, z1 = part["z0"], part["z1"]
    H = z1 - z0
    if H < 2.5 or mesh is None:
        return []
    n_st = max(1, int(round(H / storey_m)))
    sh = H / n_st
    made, k = [], [0]

    def _p(kind):
        k[0] += 1
        return "{0}/{1}_{2}_{3}".format(root, tag, kind, k[0])

    for j in range(n_st):
        zlo, zhi = z0 + j * sh, z0 + (j + 1) * sh
        # SAMPLE INSIDE THE STOREY, not on its floor: a slice exactly on a
        # setback lands on the discontinuity and may return either footprint.
        edges = storey_outline(mesh, zlo + sh * 0.5)
        if not edges:
            continue
        bands, piers, panes = [], [], []
        bh = min(band_h, sh * 0.42)
        hh = min(band_h * 0.55, sh * 0.20)
        pd = max(0.06, wall_t - reveal_m)
        for (a, b, (ox, oy), L) in edges:
            mx, my = 0.5 * (a[0] + b[0]), 0.5 * (a[1] + b[1])
            ax, ay = (b[0] - a[0]) / L, (b[1] - a[1]) / L
            yaw = math.degrees(math.atan2(ay, ax))
            # INWARD by half the thickness, so the skin sits ON the wall and
            # never grows the building's footprint
            cx = mx - ox * (wall_t / 2.0)
            cy = my - oy * (wall_t / 2.0)
            bands.append((cx, cy, zlo + bh / 2.0, L, wall_t, bh, yaw))
            bands.append((cx, cy, zhi - hh / 2.0, L, wall_t, hh, yaw))
            px = mx - ox * (reveal_m + pd / 2.0)
            py = my - oy * (reveal_m + pd / 2.0)
            panes.append((px, py, 0.5 * (zlo + zhi), L - 0.04, pd,
                          sh - 0.04, yaw))
            n_bay = max(1, int(round(L / bay_m)))
            bw = L / n_bay
            for i in range(n_bay + 1):
                u = -L / 2.0 + i * bw
                u = min(max(u, -L / 2.0 + mullion_w / 2.0),
                        L / 2.0 - mullion_w / 2.0)
                piers.append((cx + ax * u, cy + ay * u, 0.5 * (zlo + zhi),
                              mullion_w, wall_t, sh, yaw))
        for kind, boxes, mat in (("pane", panes, mats.get("glass")),
                                 ("band", bands, mats.get("wall")),
                                 ("pier", piers, mats.get("wall"))):
            pth = _mesh(stage, _p(kind), boxes, mat)
            if pth:
                made.append(pth)
    return made


def greeble_parts(stage, root, parts, mats, **kw):
    """Greeble every WALL part of a monolith. Returns [paths].

    Roof parts are skipped: a roof has no windows, and running bands round
    one puts a grid of piers across the deck.
    """
    from pxr import Sdf, UsdGeom
    UsdGeom.Scope.Define(stage, Sdf.Path(root))
    made = []
    for n, part in enumerate(parts):
        if part.get("kind") == "roof":
            continue
        if (part["z1"] - part["z0"]) < 2.5:
            continue                      # a cornice band, not a storey
        made += greeble_part(stage, root, part, mats,
                             mesh=part_trimesh(stage, part),
                             tag="p{0}".format(n), **kw)
    return made


def materials(stage, parent, wall_rgb=(0.055, 0.052, 0.048),
              glass_rgb=(0.010, 0.013, 0.014)):
    """Concrete for the bands and piers, dark glass for the pane.

    LINEAR ALBEDO. `damage._pbr` writes these into OmniPBR's
    `diffuse_color_constant` and the renderer encodes to sRGB, so
    screen ~= linear ** 0.42: 0.055 is a mid concrete grey and 0.30 is white.
    The repo has been caught by this twice.
    """
    from pxr import UsdShade
    from disaster import damage
    out = {}
    for key, rgb, rough in (("wall", wall_rgb, 0.92),
                            ("glass", glass_rgb, 0.16)):
        p = "{0}/GreebleLooks/{1}".format(parent, key)
        m = UsdShade.Material.Get(stage, p)
        if not m:
            m = damage._pbr(stage, p, rgb, rough)
        out[key] = m
    return out
