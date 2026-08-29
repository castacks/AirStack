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
        # face -> material index
        fm = {}
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
            mi = fm.get(f, 0)
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
