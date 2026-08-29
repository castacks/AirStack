"""gac_storey_slice — cut ONE merged building into clean per-storey pieces.

WHY NOT THE CENTROID BINNING IN `gac_slice`
--------------------------------------------
That one assigns each whole triangle to a cell by its centroid and never cuts
a triangle. At a 4 m cell against a mesh whose triangles are up to a metre
across, the boundary is therefore ragged by up to a triangle — which rendered
as a SAWTOOTH edge and diagonal steps rather than a clean storey line (user,
2026-08-29: "this is cutting it up into triangles or diagonal pieces instead
of per story rectangular pieces"). Real slicing has to CLIP the triangles.

THE TOOL, WHICH IS ALREADY IN THE REPO
---------------------------------------
`disaster/fracture.py` installs VTK specifically as its "C++ plane clipping
backend" and wraps `vtkClipPolyData` in `slice_plane`. That is the right
primitive and it is already a dependency. What it does NOT do is carry the
attributes: `fracture._to_vtk` builds a polydata of points and polys only, so
a slice comes back with correct geometry and no UVs and no material
assignment — which is the other half of what a sliced kit needs.

So this module puts the attributes INTO the polydata, where `vtkClipPolyData`
handles them for free:

  * UVs go in as POINT DATA. VTK interpolates point data at every vertex it
    creates on the cut line, which is exactly the barycentric UV rebuild
    `monolith_damage.cut_shell` hand-rolls with Shapely.
  * The material index goes in as CELL DATA. VTK copies cell data to the
    output cells, so every surviving triangle still knows which of the
    asset's materials it belongs to and the GeomSubsets can be rebuilt.

DE-INDEXED ON THE WAY IN. USD stores UVs `faceVarying` (one per face-corner)
while VTK point data is one per POINT, so a shared vertex with two different
UVs cannot be represented. Splitting every triangle into its own three
vertices makes the mapping exact; it triples the vertex count, which at
270k triangles is fine and is what the renderer does anyway.
"""

import os

# a cut plane this close to the top or bottom of the mesh yields a sliver
EDGE_EPS_M = 0.05
# how far the ring partition's OUTER limits sit beyond the mesh — see `ring`
OUTER_PAD_M = 0.05


def read_mesh(stage, src_path, verbose=True):
    """(points, tris, uvs, matid, material list) for every Mesh under `src`.

    De-indexed: `points`, `uvs` are per face-corner, `tris` indexes them 3 at
    a time, `matid` is one entry per triangle.
    """
    import numpy as np
    from pxr import Sdf, Usd, UsdGeom, UsdShade

    root = stage.GetPrimAtPath(src_path)
    if not root or not root.IsValid():
        return None
    xc = UsdGeom.XformCache()
    root_inv = xc.GetLocalToWorldTransform(root).GetInverse()
    P, UV, MID = [], [], []
    mats, mat_ix = [], {}
    for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        me = UsdGeom.Mesh(prim)
        pts = me.GetPointsAttr().Get()
        counts = me.GetFaceVertexCountsAttr().Get()
        fvi = me.GetFaceVertexIndicesAttr().Get()
        if not pts or not counts or not fvi:
            continue
        M = np.asarray(xc.GetLocalToWorldTransform(prim) * root_inv, dtype=float)
        V = np.asarray(pts, dtype=float)
        V = (np.c_[V, np.ones(len(V))] @ M)[:, :3]
        counts = np.asarray(counts, dtype=np.int64)
        fvi = np.asarray(fvi, dtype=np.int64)
        start = np.concatenate([[0], np.cumsum(counts)[:-1]])
        # the first faceVarying texCoord set; GAC calls it `st`
        uvv = uvi = None
        for pv in UsdGeom.PrimvarsAPI(prim).GetPrimvars():
            if str(pv.GetTypeName()) not in ("texCoord2f[]", "float2[]"):
                continue
            if str(pv.GetInterpolation()) != "faceVarying":
                continue
            vals = pv.Get()
            if vals is None or not len(vals):
                continue
            uvv = np.asarray([(q[0], q[1]) for q in vals], dtype=float)
            uvi = np.asarray(pv.GetIndices() or [], dtype=np.int64)
            break
        # face -> material index.
        #
        # THE MESH-LEVEL BINDING IS NOT OPTIONAL. Packs differ in where they
        # bind: GreatAmericanCity puts one material on each of 14 GeomSubsets
        # of a single mesh, while the AEC brownstones have 307 MESHES, ZERO
        # subsets, and a material on each mesh (measured). Harvesting only
        # from subsets therefore found nothing on a brownstone, every face
        # fell through to index 0, and the whole building came back wearing
        # one arbitrary material — the lost-textures report. The mesh's own
        # binding is the default for any face no subset claims.
        fm = {}
        mb = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
        dflt = 0
        if mb and mb.GetPrim().IsValid():
            k = str(mb.GetPrim().GetPath())
            if k not in mat_ix:
                mat_ix[k] = len(mats)
                mats.append(mb)
            dflt = mat_ix[k]
        elif not mats:
            mat_ix[""] = 0
            mats.append(None)
        for sub in UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(prim)):
            mp = UsdShade.MaterialBindingAPI(sub.GetPrim()).ComputeBoundMaterial()[0]
            key = str(mp.GetPrim().GetPath()) if mp and mp.GetPrim().IsValid() else ""
            if key not in mat_ix:
                mat_ix[key] = len(mats)
                mats.append(mp)
            for f in (sub.GetIndicesAttr().Get() or []):
                fm[int(f)] = mat_ix[key]
        for f in range(len(counts)):
            n = int(counts[f])
            base = int(start[f])
            mi = fm.get(f, dflt)
            # fan-triangulate the n-gon, de-indexing as we go
            for t in range(1, n - 1):
                for k in (0, t, t + 1):
                    P.append(V[fvi[base + k]])
                    if uvv is not None:
                        j = base + k
                        UV.append(uvv[uvi[j] if len(uvi) else j])
                    else:
                        UV.append((0.0, 0.0))
                MID.append(mi)
    if not P:
        return None
    P = np.asarray(P, dtype=float)
    out = {"P": P, "UV": np.asarray(UV, dtype=float),
           "MID": np.asarray(MID, dtype=np.int32),
           "tris": np.arange(len(P), dtype=np.int64).reshape(-1, 3),
           "mats": mats}
    if verbose:
        print("[storey_slice] read {0}: {1} tri(s), {2} material(s), "
              "uv={3}".format(src_path.rsplit("/", 1)[-1], len(out["tris"]),
                              len(mats), "yes" if len(UV) else "no"))
    return out


def _to_vtk(m):
    """Polydata carrying UVs as point data and the material id as cell data."""
    import numpy as np
    import vtk
    from vtk.util import numpy_support as ns

    pd = vtk.vtkPolyData()
    pts = vtk.vtkPoints()
    pts.SetData(ns.numpy_to_vtk(np.ascontiguousarray(m["P"]), deep=True))
    pd.SetPoints(pts)
    f = m["tris"]
    cells = np.hstack([np.full((len(f), 1), 3, dtype=np.int64), f]).ravel()
    ca = vtk.vtkCellArray()
    ca.SetCells(len(f), ns.numpy_to_vtkIdTypeArray(
        np.ascontiguousarray(cells), deep=True))
    pd.SetPolys(ca)
    uv = ns.numpy_to_vtk(np.ascontiguousarray(m["UV"]), deep=True)
    uv.SetName("st")
    pd.GetPointData().SetTCoords(uv)           # INTERPOLATED across the cut
    mid = ns.numpy_to_vtk(np.ascontiguousarray(m["MID"]), deep=True,
                          array_type=vtk.VTK_INT)
    mid.SetName("mid")
    pd.GetCellData().AddArray(mid)             # COPIED to the output cells
    return pd


def _from_vtk(pd):
    import numpy as np
    import vtk
    from vtk.util import numpy_support as ns

    tf = vtk.vtkTriangleFilter()
    tf.SetInputData(pd)
    tf.Update()
    out = tf.GetOutput()
    if out.GetNumberOfPoints() == 0 or out.GetNumberOfPolys() == 0:
        return None
    P = ns.vtk_to_numpy(out.GetPoints().GetData())
    conn = ns.vtk_to_numpy(out.GetPolys().GetConnectivityArray())
    tris = conn.reshape(-1, 3)
    tc = out.GetPointData().GetTCoords()
    UV = (ns.vtk_to_numpy(tc) if tc is not None
          else np.zeros((len(P), 2), dtype=float))
    ma = out.GetCellData().GetArray("mid")
    MID = (ns.vtk_to_numpy(ma).astype(np.int32) if ma is not None
           else np.zeros(len(tris), dtype=np.int32))
    return {"P": np.asarray(P, dtype=float), "UV": np.asarray(UV, dtype=float),
            "tris": np.asarray(tris, dtype=np.int64), "MID": MID}


def clip(m, normal, origin):
    """Keep the half of `m` that `normal` points AWAY from (below the plane).

    `vtkClipPolyData` keeps the side where the implicit function is positive;
    a vtkPlane is positive on the normal side, so `InsideOutOn` keeps the
    other. Attributes ride along: point data is interpolated at the new cut
    vertices, cell data is copied.
    """
    import vtk

    pl = vtk.vtkPlane()
    pl.SetOrigin(*[float(q) for q in origin])
    pl.SetNormal(*[float(q) for q in normal])
    cl = vtk.vtkClipPolyData()
    cl.SetInputData(_to_vtk(m) if "mats" in m or "tris" in m else m)
    cl.SetClipFunction(pl)
    cl.InsideOutOn()
    cl.Update()
    return _from_vtk(cl.GetOutput())


# A storey height to divide by when the asset's own windows cannot supply one.
# The median across the seven GAC buildings whose grid IS measurable is 3.98 m
# and the spread is 3.56-4.10, so this is that stock's own number rather than a
# generic office figure.
TARGET_STOREY_M = 3.95


def regular_grid(bbox, target=TARGET_STOREY_M, name="", verbose=True):
    """A grid from the BOUNDS alone, for a building with no readable windows.

    A MEASURED WINDOW LATTICE IS AN OPTIMISATION, NOT A PRECONDITION (user,
    2026-08-29: "it kinda doesn't matter if you can split it up into a grid
    like structure that's fine. Even for the glass buildings"). Most of this
    library cannot supply one — MEASURED (`tools/gac_grid_sweep.py`) only 10
    of 31 GAC buildings have a punched-window lattice to lock onto; the rest
    are curtain-walled or continuously banded, including the two tallest
    towers. Refusing to slice those would put the whole tall end of the
    library out of reach for no benefit, because a glass tower has no window
    rows to avoid cutting in the first place.

    Divides the height into a WHOLE number of storeys nearest `target`, so the
    storeys come out even and there is no sliver band left at the top — which
    a fixed 3.4 m divisor does leave, and which then shows up as one
    anomalously thin piece per building.
    """
    (x0, y0, z0), (x1, y1, z1) = bbox
    H = float(z1 - z0)
    n = max(1, int(round(H / float(target))))
    h = H / n
    lines = [z0 + h * k for k in range(n + 1)]
    g = {"bbox": bbox, "storey_h": h, "storeys": lines, "bays": {},
         "confidence": 0.0, "measured": False,
         "W": float(x1 - x0), "D": float(y1 - y0), "H": H, "z0": float(z0)}
    if verbose:
        print("[storey_slice] {0}: regular grid, {1:.1f} m / {2} = {3:.2f} m "
              "storeys (no window lattice)".format(name or "?", H, n, h))
    return g


def grid_for(stage, src, bbox, wins, name="", target=TARGET_STOREY_M,
             verbose=True):
    """The best grid available: the asset's own windows, else a regular one.

    Returns `(grid, measured)`. `measured` is what a caller should report —
    a regular grid is a legitimate result here, not a failure, but the two
    must not be confused when judging a slice.
    """
    from detail import gac_slice as gsl

    if wins:
        g = gsl.measure_grid(wins, bbox, verbose=False, name=name)
        if g.get("confidence", 0.0) >= gsl.MIN_CONFIDENCE and g.get("storeys"):
            g["measured"] = True
            if verbose:
                print("[storey_slice] {0}: measured grid, {1:.2f} m storeys "
                      "x{2} (confidence {3:.2f})".format(
                          name or "?", g["storey_h"], len(g["storeys"]),
                          g["confidence"]))
            return g, True
    return regular_grid(bbox, target, name, verbose), False


def cut_lines(g, offset=0.5, verbose=True):
    """Floor lines placed BETWEEN the window rows instead of through them.

    THE GRID IS PHASE-LOCKED TO THE WINDOW CENTRES, which is what makes it
    measurable — `gac_slice.storey_period` finds the period by maximising the
    circular mean of the window centres' phase, so the lattice it returns
    passes through the middle of every window row. Cutting on it therefore
    slices every window in half (user, 2026-08-29: "the stories are directly
    splitting the windows").

    A real floor line is in the SPANDREL, the blind band between the head of
    one window and the cill of the one above. That is half a period away from
    the window centres, so the cut lattice is the window lattice shifted by
    `offset` of a period. Returns the shifted lines inside the mesh.
    """
    # NO WINDOWS, NO MEASURED GRID. The AEC brownstones carry no glass subset
    # at all (`tools/openings_probe.py`), so `measure_grid` has nothing to lock
    # a period to. A stated fallback is honest; silently cutting on a period
    # that was never measured is not, so the caller is told which it got.
    per = float(g.get("storey_h") or 0.0)
    if per <= 0.0 or g.get("confidence", 0.0) < 0.0:
        per = float(g.get("fallback_storey_h") or 3.4)
    (_x0, _y0, z0), (_x1, _y1, z1) = g["bbox"]
    out = [z + offset * per for z in g["storeys"]]
    out = [z for z in out if z0 + 0.05 < z < z1 - 0.05]
    if verbose:
        print("[storey_slice] cut lines shifted {0:.0%} of a {1:.2f} m storey "
              "off the window rows: {2} line(s)".format(offset, per, len(out)))
    return out


def window_clearance(cut_z, win_centres, win_h=1.1):
    """Smallest gap between any cut line and any window centre, in metres.

    The acceptance test for `cut_lines`: a cut nearer than half a window
    height to a centre goes through the glass. Measured window height on this
    stock is ~1.1 m (`tools/openings_probe.py`: median island 0.93 x 1.11 m).
    Returns (worst clearance, number of windows a cut passes through).
    """
    zs = [z for pts in win_centres.values() for _u, z in pts]
    if not zs or not cut_z:
        return None, 0
    worst = min(min(abs(z - c) for c in cut_z) for z in zs)
    hit = sum(1 for z in zs if min(abs(z - c) for c in cut_z) < 0.5 * win_h)
    return worst, hit


def storeys(m, floors, verbose=True):
    """Cut `m` into one piece per storey band with two horizontal planes each.

    Returns [(z_lo, z_hi, piece)] for the bands that survived.
    """
    # THE LATTICE DOES NOT REACH THE ENDS OF THE MESH, and both ends have to
    # be closed or their geometry is silently dropped. `gac_slice.lattice`
    # returns the floor lines INSIDE [z0, z1], so the first line normally sits
    # above the mesh bottom (the shopfront band, which on SM_Building_01 is
    # most of a storey) and the last sits below the top (the cornice).
    # Measured before this: the bands summed to 4.03% less area than the
    # source, all of it the base and the crown.
    zs = list(floors)
    bot = float(m["P"][:, 2].min())
    top = float(m["P"][:, 2].max())
    if not zs or zs[0] > bot + EDGE_EPS_M:
        zs = [bot] + zs
    if not zs or zs[-1] < top - EDGE_EPS_M:
        zs = zs + [top]
    out = []
    for i in range(len(zs) - 1):
        lo, hi = zs[i], zs[i + 1]
        band = clip(m, (0.0, 0.0, 1.0), (0.0, 0.0, hi))     # keep below hi
        if band is None:
            continue
        band = clip(band, (0.0, 0.0, -1.0), (0.0, 0.0, lo))  # keep above lo
        if band is None or not len(band["tris"]):
            continue
        out.append((lo, hi, band))
    if verbose:
        print("[storey_slice] {0} band(s) from {1} floor line(s): {2}".format(
            len(out), len(zs), ", ".join("%.1f-%.1f" % (a, b)
                                         for a, b, _p in out[:6])))
    return out


# How a bay is subdivided across the face. NOT EQUAL THIRDS (user,
# 2026-08-29: "you can make the middle one larger and side 2 the smaller
# parts instead of equal thirds") — a façade bay is a wide opening panel
# between two narrower piers, so the middle piece is the wide one. Fractions
# of the bay, summing to 1.
BAY_SPLITS = (0.26, 0.48, 0.26)


def ring(band, bbox, leg, bays, splits=BAY_SPLITS, verbose=False):
    """Split one storey band into corner + middle pieces around its ring.

    THE PARTITION IS A 3x3 GRID IN PLAN, WHICH MAKES IT EXACT. The plan is cut
    at `x0+leg`, `x1-leg`, `y0+leg`, `y1-leg`; the four outer corners are the
    CORNER pieces, the four edge strips are the runs (each subdivided into
    bays), and the middle cell is the interior CORE. Nine regions, no overlap
    and no gap — which matters because any overlap duplicates geometry into two
    pieces and any gap drops it on the floor.

    That is `detail/urban_building.py`'s own grammar: "each side is a RUN of
    façade modules between two CORNER pieces". `leg` is the corner leg, the
    same quantity the kit's `_c(piece, "SE", 1.0)` declares.

    Returns [(role, side, bay, piece)].
    """
    # PAD THE OUTER BOUNDS. The building's own skin lies EXACTLY on x0/x1/
    # y0/y1, and a triangle lying exactly in a clip plane has the implicit
    # function zero at all three vertices — vtkClipPolyData's behaviour there
    # is ambiguous and it drops them. That silently loses the outer façade,
    # which is the one surface this whole exercise is about. Only the INTERIOR
    # cuts (xa/xb/ya/yb) need to be exact; the outer limits just have to
    # contain the mesh.
    (x0, y0, _z0), (x1, y1, _z1) = bbox
    x0, y0 = x0 - OUTER_PAD_M, y0 - OUTER_PAD_M
    x1, y1 = x1 + OUTER_PAD_M, y1 + OUTER_PAD_M
    xa, xb = x0 + leg + OUTER_PAD_M, x1 - leg - OUTER_PAD_M
    ya, yb = y0 + leg + OUTER_PAD_M, y1 - leg - OUTER_PAD_M
    if xb <= xa or yb <= ya:            # too small to have a middle at all
        return [("corner", "-", 0, band)]

    def box(m, lo_x, hi_x, lo_y, hi_y):
        """The part of `m` inside an axis-aligned plan rectangle."""
        p = m
        for nrm, org in (((1.0, 0.0, 0.0), (hi_x, 0.0, 0.0)),
                         ((-1.0, 0.0, 0.0), (lo_x, 0.0, 0.0)),
                         ((0.0, 1.0, 0.0), (0.0, hi_y, 0.0)),
                         ((0.0, -1.0, 0.0), (0.0, lo_y, 0.0))):
            if p is None or not len(p["tris"]):
                return None
            p = clip(p, nrm, org)
        return p if (p is not None and len(p["tris"])) else None

    out = []
    # --- the four corners -------------------------------------------------
    for side, lo_x, hi_x, lo_y, hi_y in (("SW", x0, xa, y0, ya),
                                         ("SE", xb, x1, y0, ya),
                                         ("NW", x0, xa, yb, y1),
                                         ("NE", xb, x1, yb, y1)):
        p = box(band, lo_x, hi_x, lo_y, hi_y)
        if p is not None:
            out.append(("corner", side, 0, p))
    # --- the four runs, each divided into bays ----------------------------
    runs = (("S", xa, xb, y0, ya, "x"), ("N", xa, xb, yb, y1, "x"),
            ("W", x0, xa, ya, yb, "y"), ("E", xb, x1, ya, yb, "y"))
    for side, lo_x, hi_x, lo_y, hi_y, axis in runs:
        span = (hi_x - lo_x) if axis == "x" else (hi_y - lo_y)
        n = max(1, int(round(span / max(0.5, bays))))
        fr = list(splits) if splits else [1.0]
        tot = float(sum(fr)) or 1.0
        for k in range(n):
            base = k / float(n)
            acc = 0.0
            for j, f in enumerate(fr):
                t0 = base + (acc / tot) / n
                acc += f
                t1 = base + (acc / tot) / n
                if axis == "x":
                    p = box(band, lo_x + t0 * span, lo_x + t1 * span,
                            lo_y, hi_y)
                else:
                    p = box(band, lo_x, hi_x, lo_y + t0 * span,
                            lo_y + t1 * span)
                if p is not None:
                    # the wide middle sub-panel is the opening; the two
                    # narrow ones either side are the piers
                    role = "wall" if j == len(fr) // 2 else "pier"
                    out.append((role, side, k * len(fr) + j, p))
    # --- the interior -----------------------------------------------------
    core = box(band, xa, xb, ya, yb)
    if core is not None:
        out.append(("core", "-", 0, core))
    if verbose:
        by = {}
        for role, _s, _b, _p in out:
            by[role] = by.get(role, 0) + 1
        print("[storey_slice]   ring -> {0}".format(
            "  ".join("{0}={1}".format(k, v) for k, v in sorted(by.items()))))
    return out


# role -> the `sub` token `quake_flow._sub_and_mass` parses out of a category.
# `pier` and `core` both map to `storey`: a pier IS a wall element (the narrow
# panel between openings) and the interior core is treated as one so that
# nothing in the building is invisible to the element walk.
_ROLE_SUB = {"wall": "storey", "pier": "storey", "core": "storey",
             "corner": "storey_corner", "parapet": "parapet",
             "parapet_corner": "parapet_corner", "roof": "roof"}


# THE UPSTAND'S HEIGHT. Mirrors the exact number `gac_slice.register_style`
# already assumes for a parapet band's `h` (`max(0.8, 0.35 * storey_h)`) so
# that once `_fix_advertised_bands` (below) corrects that band to the
# MEASURED height of what this function actually cut, the correction is
# small rather than a contradiction of the assumption the rest of the spec
# was built on.
PARAPET_FRAC = 0.35
PARAPET_MIN_M = 0.8


def roof_and_parapet(band, bbox, leg, bays, storey_h, splits=BAY_SPLITS,
                     verbose=False):
    """Split the TOPMOST storey band into a roof deck and a parapet upstand.

    WHY THIS EXISTS. `ring()` alone never produces a `roof` or `parapet`
    piece — it only ever returns `corner` / `wall` / `pier` / `core` — so
    `gac_slice.register_style` was advertising a `parapet` band with NOTHING
    behind it, and every `urban_fire` recipe that looks for `role="roof"` or
    `role="parapet"` elements on a sliced building found none. That is the
    traced cause of the mismatched-roof report (user, 2026-08-29, on
    `row1_gac.png`): a downstream roof-authoring path meant for a building
    with real roof/parapet elements falls through to a WORLD-BBOX behaviour
    that was fixed for kit buildings on 2026-08-28 and never had a sliced
    building's own geometry to draw on instead.

    THE SPLIT IS JUST ANOTHER RING. The top band already contains the real
    roof deck and whatever stands above it — a merged mesh does not label
    the two — so a second horizontal cut at `hi - par_h` (an upstand height
    below the mesh's own apex, not a plane borrowed from anywhere else)
    divides it into a lower piece that is still an ordinary storey wall and
    an upper piece. `ring()`ing that upper piece on the SAME plan partition
    gives the answer directly: its four corners and runs ARE the parapet
    upstand around the edge of the roof, and whatever lands in the
    untouched middle cell IS the flat deck, because that is exactly what a
    merged mesh puts there. No new geometry is invented — the same
    triangles that used to come back labelled `wall`/`corner`/`core` for
    this one band now come back labelled `parapet`/`parapet_corner`/`roof`
    instead, off the SAME clip machinery `ring()` already proved.

    MEASURED: most of this stock's top band does NOT have room for a second
    cut. `cut_lines`'s topmost spandrel line already sits close to the
    mesh's own apex on 4 of the first 5 buildings probed (`SM_Building_01`:
    a 0.-something-metre remainder; `_04`, `_09` likewise), because the
    coping above the last row of windows is architecturally thin to start
    with — there is no separate ordinary storey wall hiding above the last
    window row, the coping/parapet more or less IS the whole of what is left
    once `storeys()` reaches the mesh's apex. Demanding room for BOTH a wall
    zone AND an upstand before calling anything `roof`/`parapet` (the first
    version of this function) left every one of those buildings advertising
    nothing, for a reason that had nothing to do with them lacking a roof —
    only with the split being too greedy. So: when the whole band is no
    taller than an upstand-plus-a-minimal-wall would need, there is no wall
    zone to carve out of it at all — `ring()` the WHOLE band once, at its own
    height, and relabel by position. The lower-split path below still runs
    on the buildings tall enough in the top band to actually have a last
    storey wall standing under their coping (`SM_Building_24`, the taller
    curtain-wall towers).

    AN INTERIOR CUT, SO NO `OUTER_PAD_M`. `z_split` (when the split path
    runs) is `hi` (the mesh's own apex, an OUTER limit `ring()`'s plan cuts
    do pad against) minus an upstand height derived from the MEASURED
    storey pitch — not from anything in the mesh itself — so it lands off
    any vertex or edge by construction, the same reasoning that lets
    `storeys()`'s spandrel cuts run exact. Only a plane deliberately placed
    ON the mesh's own skin (the x0/x1/y0/y1 `ring()` pads for) risks the
    zero-everywhere triangle.

    `split_ok` in the return is `True` whenever the band came back labelled
    `roof`/`parapet`/`parapet_corner` at all (whether or not the wall zone
    was separated out) and `False` only on the genuine clip failure this
    function cannot recover from — a caller doing partition-integrity math
    on a plain `ring()` in that case needs to know it got one.

    Returns `([(role, side, bay, piece)], split_ok)`.
    """
    (x0, y0, lo), (x1, y1, hi) = bbox
    relabel = {"corner": "parapet_corner", "wall": "parapet",
               "pier": "parapet", "core": "roof"}

    def _whole_band_as_roof():
        out = []
        for role, side, k, piece in ring(band, bbox, leg, bays,
                                         splits=splits, verbose=verbose):
            out.append((relabel.get(role, role), side, k, piece))
        return out

    par_h = max(PARAPET_MIN_M, PARAPET_FRAC * float(storey_h or 0.0))
    # A THIRD of a storey is the minimum that still reads as an actual wall
    # zone, not a sliver; EDGE_EPS_M is `storeys()`'s own sliver guard.
    min_wall = max(EDGE_EPS_M, 0.3 * float(storey_h or 0.0))
    if hi - lo <= par_h + min_wall:
        out = _whole_band_as_roof()
        if verbose:
            by = {}
            for role, _s, _k, _p in out:
                by[role] = by.get(role, 0) + 1
            print("[storey_slice]   top band {0:.2f} m too thin for a "
                  "separate wall zone; whole band -> {1}".format(
                      hi - lo, "  ".join("{0}={1}".format(k, v)
                                        for k, v in sorted(by.items()))))
        return out, True
    z_split = hi - par_h
    lower = clip(band, (0.0, 0.0, 1.0), (0.0, 0.0, z_split))    # keep below
    upper = clip(band, (0.0, 0.0, -1.0), (0.0, 0.0, z_split))   # keep above
    if (lower is None or not len(lower["tris"])
            or upper is None or not len(upper["tris"])):
        # the exact-coincidence edge case (see the module docstring's
        # OUTER_PAD_M note) — vanishingly unlikely at an arbitrary interior
        # height, but if it happens the honest answer is the whole band, not
        # a silently dropped one
        return _whole_band_as_roof(), False
    out = list(ring(lower, bbox, leg, bays, splits=splits, verbose=verbose))
    for role, side, k, piece in ring(upper, bbox, leg, bays, splits=splits,
                                     verbose=verbose):
        out.append((relabel.get(role, role), side, k, piece))
    if verbose:
        by = {}
        for role, _s, _k, _p in out:
            by[role] = by.get(role, 0) + 1
        print("[storey_slice]   roof/parapet split {0:.2f} m below the apex "
              "-> {1}".format(par_h, "  ".join(
                  "{0}={1}".format(k, v) for k, v in sorted(by.items()))))
    return out, True


def as_placements(stage, cells, scope, style, mats, verbose=True):
    """Write ring cells as prims and return kit-shaped PLACEMENT dicts.

    THIS IS THE BRIDGE between the slicer and the damage ladder, and it is a
    short function because `quake_flow.classify` reads only a placement dict —
    `category`, `x_m`/`y_m`/`z_m`, `yaw_deg`, and `prim_path` once the prim
    exists. Express the cells that way and `describe`, `fit_interior` and every
    `urban_fire` recipe work on a sliced whole-asset building with NO change to
    any of them. That is the whole point of cutting on the kit's own grammar
    rather than on some grid of our own.

    The anchor is the piece's own (centroid x, centroid y, min z), which is
    what `gac_slice.register_style` assumes when it writes the piece's
    `PIECES` entry as `(sx, sy, sz, -sx/2, -sy/2, 0)`.
    """
    import numpy as np
    from pxr import Sdf, UsdGeom

    UsdGeom.Scope.Define(stage, Sdf.Path(scope))
    out = []
    for j, (storey, role, side, bay, piece) in enumerate(cells):
        nm = "{0}_{1}_{2}_{3:02d}_{4:04d}".format(
            role, side.replace("-", "x"), bay, storey, j)
        path = "{0}/{1}".format(scope, nm)
        if not write_piece(stage, path, piece, mats):
            continue
        P = piece["P"]
        c = P.mean(axis=0)
        mn, mx = P.min(axis=0), P.max(axis=0)
        out.append({
            "category": "bld_{0}_{1}".format(style, _ROLE_SUB.get(role, "storey")),
            "usd": "slice://{0}".format(nm),
            "x_m": float(c[0]), "y_m": float(c[1]), "z_m": float(mn[2]),
            "yaw_deg": 0.0, "scale": 1.0, "prim_path": path,
            "_size": (float(mx[0] - mn[0]), float(mx[1] - mn[1]),
                      float(mx[2] - mn[2])),
            "_role": role, "_side": side, "_storey": storey, "_bay": bay})
    if verbose:
        by = {}
        for p in out:
            by[p["_role"]] = by.get(p["_role"], 0) + 1
        print("[storey_slice] {0} placement(s) for style {1}: {2}".format(
            len(out), style,
            "  ".join("{0}={1}".format(k, v) for k, v in sorted(by.items()))))
    return out


def slice_to_kit(stage, src, cell, style, target=TARGET_STOREY_M,
                 offset=0.5, verbose=True):
    """Slice a placed asset into kit placements, ready for `burn_building`.

    One call: measure the grid (or fall back to a regular one), cut the
    storeys clear of the windows, ring each band, write the pieces, register
    the synthetic style so `quake_flow._mass_specs` has real dimensions, and
    hide the merged original.

    Returns `(placements, grid, measured)`.
    """
    from detail import gac_slice as gsl
    from pxr import UsdGeom

    wins, bbox = gsl.window_centres(stage, src)
    g, measured = grid_for(stage, src, bbox, wins, name=style,
                           target=target, verbose=verbose)
    m = read_mesh(stage, src, verbose=False)
    if m is None:
        return [], g, measured
    lines = cut_lines(g, offset, verbose=False)
    bands = storeys(m, lines, verbose=False)
    leg = max(1.2, 0.6 * ((g["bays"].get("E") or {}).get("pitch") or 3.5))
    bay = (g["bays"].get("E") or {}).get("pitch") or 3.5
    cells = []
    n_bands = len(bands)
    roofed = False
    for i, (lo, hi, band) in enumerate(bands):
        bb = ((bbox[0][0], bbox[0][1], lo), (bbox[1][0], bbox[1][1], hi))
        # ONLY THE TOPMOST BAND CAN HOLD THE ROOF. Every band below it is a
        # storey sandwiched between two floor lines and is, honestly, just a
        # wall — `roof_and_parapet` only has anything to find in the one band
        # that runs up to the mesh's own apex.
        if i == n_bands - 1:
            pieces, roofed = roof_and_parapet(band, bb, leg, bay,
                                              g["storey_h"], verbose=verbose)
        else:
            pieces = ring(band, bb, leg, bay)
        for role, side, k, piece in pieces:
            cells.append((i, role, side, k, piece))
    pls = as_placements(stage, cells, cell + "/pieces", style, m["mats"],
                        verbose=verbose)
    spec = gsl.register_style(g, style, pieces_of=pls)
    _fix_advertised_bands(spec, pls, style, verbose=verbose)
    UsdGeom.Imageable(stage.GetPrimAtPath(src)).MakeInvisible()
    if verbose and not roofed:
        print("[storey_slice] {0}: top band could not be labelled roof/"
              "parapet at all (clip failure) — check {1}".format(style, src))
    return pls, g, measured


def _fix_advertised_bands(spec, pls, style, verbose=True):
    """Correct `ub.STYLES[style]["bands"]` to match what THIS asset actually
    cut, instead of what `gac_slice.register_style` has to assume.

    `register_style` (in `gac_slice.py`, which this module does not own)
    unconditionally writes a `parapet` band into the spec it installs,
    because at the time it was written the ring-only slicer could never
    produce a `parapet`/`parapet_corner` piece and there was nothing to
    check the claim against — that mismatch (a band advertised in
    `ub.STYLES[name]["bands"]` with no placement backing it) is the traced
    cause of the mismatched-roof report this fix addresses. `roof_and_parapet`
    backs it on every asset measured so far (it only ever fails to on the
    rare vtkClipPolyData exact-coincidence edge case its own docstring
    covers), but this correction still runs unconditionally rather than
    trusting that: the advertised band has to be corrected per building,
    here, to what THIS asset's own pieces actually contain, or the one
    building that DOES hit that edge case goes back to advertising a band
    nothing backs:

      * no `parapet`/`parapet_corner` piece survived the cut -> the band is
        REMOVED rather than left advertising a height for nothing;
      * one did -> the band's `h` is overwritten with the MEASURED extent of
        the pieces that were actually produced, rather than
        `register_style`'s `0.35 * storey_h` guess.

    `_mass_specs` skips `parapet` bands entirely when it sums storey heights
    (`if band.get("parapet"): continue`), and nothing downstream reads a
    band's `h` outside that sum for a sliced building's `spec["bands"]`
    (measured: `footprint`, `height`, `_mass_specs` are the only readers,
    and only `height` and `_mass_specs` look at `h`, both skipping parapet
    bands) — so today this correction is a consistency fix with no live
    effect on the fire/quake recipes, which read `_els(role=...)` off the
    PLACEMENTS instead. It is here anyway because a band the spec claims and
    the placements do not have is exactly the bug this whole change closes,
    and the next reader of `ub.STYLES[name]["bands"]` should not have to
    rediscover that the two can disagree.

    THE CLEANER FIX BELONGS IN `gac_slice.py`: `register_style` should take
    the measured parapet extent (or `None`) as an argument from its caller
    instead of deriving `0.35 * storey_h` unconditionally and always
    writing a band for it — see the accompanying report for what that
    signature change would look like.
    """
    bands = spec.get("bands") or []
    if not bands:
        return
    par = [p for p in pls if p.get("_role") in ("parapet", "parapet_corner")]
    if not par:
        spec["bands"] = [b for b in bands if not b.get("parapet")]
        if verbose:
            print("[storey_slice] {0}: no parapet piece survived the cut; "
                  "removed the advertised (unbacked) parapet band".format(
                      style))
        return
    lo = min(p["z_m"] for p in par)
    hi = max(p["z_m"] + p["_size"][2] for p in par)
    real_h = max(0.05, hi - lo)
    for b in bands:
        if b.get("parapet"):
            b["h"] = real_h


def write_piece(stage, path, piece, mats):
    """One clipped fragment as a USD Mesh with `st` and per-material subsets."""
    import numpy as np
    from pxr import Sdf, UsdGeom, UsdShade, Vt

    if piece is None or not len(piece["tris"]):
        return None
    me = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
    me.CreatePointsAttr(Vt.Vec3fArray(
        [tuple(map(float, q)) for q in piece["P"]]))
    me.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(piece["tris"])))
    me.CreateFaceVertexIndicesAttr(Vt.IntArray(
        [int(i) for i in piece["tris"].ravel()]))
    pv = UsdGeom.PrimvarsAPI(me).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying)
    pv.Set(Vt.Vec2fArray([tuple(map(float, piece["UV"][i]))
                          for i in piece["tris"].ravel()]))
    for mi in sorted(set(int(q) for q in piece["MID"])):
        faces = [int(k) for k, q in enumerate(piece["MID"]) if int(q) == mi]
        if not faces or mi >= len(mats) or mats[mi] is None:
            continue
        # familyName MUST be `materialBind`.
        # `CreateGeomSubset(geom, name, elementType, indices)` leaves the
        # family EMPTY, and a renderer only consults the `materialBind` family
        # when resolving per-face materials — so subsets in family "" are
        # ignored, the whole mesh falls back to one material, and a sliced
        # brick building with windows renders as a FLAT BROWN BOX. The cut
        # geometry is perfect while that happens, which is what makes it look
        # like a UV or slicing bug instead of a schema one. The source asset's
        # own subsets are all `family='materialBind'` (measured).
        sub = UsdGeom.Subset.CreateGeomSubset(
            me, "mat_{0}".format(mi), UsdGeom.Tokens.face, Vt.IntArray(faces),
            UsdShade.Tokens.materialBind, UsdGeom.Tokens.partition)
        # APPLY the schema before binding — the non-applied constructor writes
        # a relationship USD warns about and the renderer ignores.
        UsdShade.MaterialBindingAPI.Apply(sub.GetPrim()).Bind(mats[mi])
    return path
