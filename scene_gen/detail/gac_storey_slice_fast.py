"""Experimental fast partitions for gac_storey_slice.

This module is intentionally not imported by the production pipeline.  The
experimental fast-fire launcher monkey-patches only ``storeys`` and ``ring``
for one process.  Geometry stays in VTK while a region is partitioned; every
plane is evaluated once with vtkClipPolyData's complementary output, and the
known-good non-merging locator preserves coincident decal UVs.
"""

import numpy as np

from detail import gac_storey_slice as base

_read_mesh_original = base.read_mesh
_to_vtk_original = base._to_vtk


def read_mesh(stage, src_path, verbose=True):
    """Production mesh reader plus de-indexed, transformed source normals."""
    from pxr import Usd, UsdGeom

    out = _read_mesh_original(stage, src_path, verbose=verbose)
    if out is None:
        return None
    root = stage.GetPrimAtPath(src_path)
    xc = UsdGeom.XformCache()
    root_inv = xc.GetLocalToWorldTransform(root).GetInverse()
    normals = []
    for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        me = UsdGeom.Mesh(prim)
        pts = me.GetPointsAttr().Get()
        counts = me.GetFaceVertexCountsAttr().Get()
        fvi = me.GetFaceVertexIndicesAttr().Get()
        if not pts or not counts or not fvi:
            continue
        V = np.asarray(pts, dtype=float)
        counts = np.asarray(counts, dtype=np.int64)
        fvi = np.asarray(fvi, dtype=np.int64)
        starts = np.concatenate([[0], np.cumsum(counts)[:-1]])
        authored = me.GetNormalsAttr().Get()
        interp = str(me.GetNormalsInterpolation())
        authored = (np.asarray(authored, dtype=float)
                    if authored is not None and len(authored) else None)
        M = np.asarray(xc.GetLocalToWorldTransform(prim) * root_inv, dtype=float)
        A = M[:3, :3]
        for f, count in enumerate(counts):
            n = int(count)
            b = int(starts[f])
            for t in range(1, n - 1):
                corners = (0, t, t + 1)
                if authored is None:
                    q = V[fvi[[b + k for k in corners]]]
                    fn = np.cross(q[1] - q[0], q[2] - q[0])
                    srcn = (fn, fn, fn)
                elif interp in ("vertex", "varying"):
                    srcn = [authored[fvi[b + k]] for k in corners]
                elif interp == "uniform":
                    srcn = (authored[f],) * 3
                else:  # faceVarying (the GAC path), indexed like corners
                    srcn = [authored[b + k] for k in corners]
                for n0 in srcn:
                    nw = np.asarray(n0, dtype=float) @ np.linalg.inv(A)
                    ln = np.linalg.norm(nw)
                    normals.append(nw / ln if ln > 1e-12 else (0.0, 0.0, 1.0))
    if len(normals) != len(out["P"]):
        raise RuntimeError("normal stream mismatch: %d normals for %d points" %
                           (len(normals), len(out["P"])))
    out["N"] = np.asarray(normals, dtype=np.float32)
    return out


def _to_vtk(m):
    """Production VTK conversion with normals carried as point data."""
    from vtk.util import numpy_support as ns

    pd = _to_vtk_original(m)
    if "N" in m:
        arr = ns.numpy_to_vtk(np.ascontiguousarray(m["N"]), deep=True)
        arr.SetName("Normals")
        pd.GetPointData().SetNormals(arr)
    return pd


def _split(pd, axis, value):
    import vtk

    plane = vtk.vtkPlane()
    origin = [0.0, 0.0, 0.0]
    normal = [0.0, 0.0, 0.0]
    origin[axis] = float(value)
    normal[axis] = 1.0
    plane.SetOrigin(*origin)
    plane.SetNormal(*normal)
    clip = vtk.vtkClipPolyData()
    clip.SetInputData(pd)
    clip.SetClipFunction(plane)
    clip.InsideOutOn()                 # output 0 is below the ascending cut
    clip.SetLocator(vtk.vtkNonMergingPointLocator())
    clip.GenerateClippedOutputOn()
    clip.Update()
    below = vtk.vtkPolyData()
    above = vtk.vtkPolyData()
    below.ShallowCopy(clip.GetOutput())
    above.ShallowCopy(clip.GetClippedOutput())
    return below, above


def _sweep(pd, axis, cuts):
    out, rest = [], pd
    for value in cuts:
        below, rest = _split(rest, axis, value)
        out.append(below)
        if not rest.GetNumberOfCells():
            break
    out.append(rest)
    return out


def _compact(pd):
    """VTK polydata -> mesh dict, dropping unused points but never merging."""
    import vtk
    from vtk.util import numpy_support as ns

    if pd is None or not pd.GetNumberOfCells():
        return None
    tri = vtk.vtkTriangleFilter()
    tri.SetInputData(pd)
    tri.Update()
    out = tri.GetOutput()
    if not out.GetNumberOfPolys():
        return None
    points = ns.vtk_to_numpy(out.GetPoints().GetData())
    conn = ns.vtk_to_numpy(out.GetPolys().GetConnectivityArray())
    tris = conn.reshape(-1, 3)
    used, inv = np.unique(tris.ravel(), return_inverse=True)
    tc = out.GetPointData().GetTCoords()
    uv = (ns.vtk_to_numpy(tc) if tc is not None
          else np.zeros((len(points), 2), dtype=np.float32))
    mid_a = out.GetCellData().GetArray("mid")
    mid = (ns.vtk_to_numpy(mid_a).astype(np.int32, copy=False)
           if mid_a is not None else np.zeros(len(tris), dtype=np.int32))
    na = out.GetPointData().GetNormals()
    normals = (ns.vtk_to_numpy(na) if na is not None else None)
    result = {"P": np.asarray(points[used], dtype=float),
            "UV": np.asarray(uv[used], dtype=float),
            "tris": inv.reshape(-1, 3).astype(np.int64, copy=False),
            "MID": mid}
    if normals is not None:
        result["N"] = np.asarray(normals[used], dtype=np.float32)
    return result


def write_piece(stage, path, piece, mats, role=None):
    """Production-equivalent schema authored through bulk NumPy buffers."""
    from pxr import Sdf, UsdGeom, UsdShade, Vt

    if piece is None or not len(piece["tris"]):
        return None
    tris = np.ascontiguousarray(piece["tris"], dtype=np.int32)
    points = np.ascontiguousarray(piece["P"], dtype=np.float32)
    uv = np.ascontiguousarray(piece["UV"][tris.reshape(-1)], dtype=np.float32)
    mids = np.asarray(piece["MID"])
    me = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
    # These are final render triangles, not a subdivision control cage.
    # Leaving USD's Catmull-Clark default bends each clipped triangle and
    # produces the black/white triangular facade wedges seen in Isaac.
    me.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    me.CreateDoubleSidedAttr(True)
    me.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(points))
    me.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(
        np.full(len(tris), 3, dtype=np.int32)))
    me.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(tris.reshape(-1)))
    pv = UsdGeom.PrimvarsAPI(me).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying)
    pv.Set(Vt.Vec2fArray.FromNumpy(uv))
    if "N" in piece:
        normals = np.ascontiguousarray(piece["N"][tris.reshape(-1)],
                                       dtype=np.float32)
        me.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(normals))
        me.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
    scope = path.rsplit("/", 1)[0]
    fallback = base._role_fallback_material(stage, scope, role or "wall")
    UsdShade.MaterialBindingAPI.Apply(me.GetPrim()).Bind(fallback)
    for mi in sorted(set(int(q) for q in mids)):
        faces = np.flatnonzero(mids == mi).astype(np.int32, copy=False)
        if not len(faces) or mi >= len(mats) or mats[mi] is None:
            continue
        sub = UsdGeom.Subset.CreateGeomSubset(
            me, "mat_{0}".format(mi), UsdGeom.Tokens.face,
            Vt.IntArray.FromNumpy(np.ascontiguousarray(faces)),
            UsdShade.Tokens.materialBind, UsdGeom.Tokens.partition)
        bind_mat = base._selfcontained_like(stage, scope, mats[mi]) or mats[mi]
        UsdShade.MaterialBindingAPI.Apply(sub.GetPrim()).Bind(bind_mat)
    return path


def storeys(m, floors, verbose=True):
    bot = float(m["P"][:, 2].min())
    top = float(m["P"][:, 2].max())
    zs = base._closed_lines(floors, bot, top)
    pd = base._to_vtk(m)
    bands = _sweep(pd, 2, zs[1:-1])
    result = []
    for i, band in enumerate(bands):
        piece = _compact(band)
        if piece is not None and len(piece["tris"]):
            result.append((zs[i], zs[i + 1], piece))
    if verbose:
        print("[fast_slice] %d band(s), %d horizontal planes (one sweep)"
              % (len(result), max(0, len(zs) - 2)))
    return result


def _subcuts(lo, hi, pitch, splits):
    span = hi - lo
    n = max(1, int(round(span / max(0.5, pitch))))
    fractions = list(splits) if splits else [1.0]
    total = float(sum(fractions)) or 1.0
    cuts, labels = [], []
    for k in range(n):
        acc = 0.0
        for j, frac in enumerate(fractions):
            acc += frac
            labels.append((k, j, len(fractions)))
            if not (k == n - 1 and j == len(fractions) - 1):
                cuts.append(lo + span * ((k + acc / total) / n))
    return cuts, labels


def _append_run(out, pd, axis, lo, hi, pitch, splits, side):
    cuts, labels = _subcuts(lo, hi, pitch, splits)
    cells = _sweep(pd, axis, cuts)
    for cell, (k, j, nf) in zip(cells, labels):
        p = _compact(cell)
        if p is not None and len(p["tris"]):
            role = "wall" if j == nf // 2 else "pier"
            out.append((role, side, k * nf + j, p))


def ring(band, bbox, leg, bays, splits=base.BAY_SPLITS, verbose=False,
         hot_sides=None):
    """The production 3x3 ring grammar, expressed as partition sweeps."""
    (x0, y0, _), (x1, y1, _) = bbox
    xa, xb = x0 + leg, x1 - leg
    ya, yb = y0 + leg, y1 - leg
    if xb <= xa or yb <= ya:
        return [("corner", "-", 0, band)]

    pd = base._to_vtk(band)
    left, middle, right = _sweep(pd, 0, (xa, xb))
    out = []

    # Left and right columns: corner / W|E run / corner.
    for col, side, low_corner, high_corner in (
            (left, "W", "SW", "NW"), (right, "E", "SE", "NE")):
        low, run, high = _sweep(col, 1, (ya, yb))
        for cp, cname in ((low, low_corner), (high, high_corner)):
            p = _compact(cp)
            if p is not None and len(p["tris"]):
                out.append(("corner", cname, 0, p))
        if hot_sides is not None and side not in hot_sides:
            p = _compact(run)
            if p is not None and len(p["tris"]):
                out.append(("wall", side, 0, p))
        else:
            _append_run(out, run, 1, ya, yb, bays, splits, side)

    # Middle column: S run / core / N run.
    south, core, north = _sweep(middle, 1, (ya, yb))
    for run, side in ((south, "S"), (north, "N")):
        if hot_sides is not None and side not in hot_sides:
            p = _compact(run)
            if p is not None and len(p["tris"]):
                out.append(("wall", side, 0, p))
        else:
            _append_run(out, run, 0, xa, xb, bays, splits, side)
    p = _compact(core)
    if p is not None and len(p["tris"]):
        out.append(("core", "-", 0, p))
    if verbose:
        print("[fast_slice] ring -> %d piece(s), sweep partition" % len(out))
    return out


def install():
    """Install only into this process; no production source is modified."""
    base.storeys = storeys
    base.ring = ring
    base.write_piece = write_piece
    base.read_mesh = read_mesh
    base._to_vtk = _to_vtk
    # Keep the experiment isolated while tightening GAC's per-window face
    # test.  The shipped function matches only a triangle centroid; a large
    # facade triangle can therefore land in a window island and be rebound
    # to transparent/white glass.  Require the whole projected triangle to
    # fit that same island.  Executing a private copy changes this process's
    # module object only; disaster/gac_fire.py remains untouched.
    import inspect
    import textwrap
    from disaster import gac_fire
    src = textwrap.dedent(inspect.getsource(gac_fire.damage_windows))
    bind_needle = """                cpath = str(cur.GetPrim().GetPath())\n                if cpath in orig_of:\n"""
    bind_replacement = """                cpath = str(cur.GetPrim().GetPath())\n                # Asset-specific correctness guard: this is the concrete\n                # surround despite the misleading `Win` token.\n                if \"win_concrete\" in cpath.lower():\n                    continue\n                if cpath in orig_of:\n"""
    needle = """                    island = _match_island(side, u, z, rects)\n                    if island is None:\n"""
    replacement = """                    island = _match_island(side, u, z, rects)\n                    if island is not None:\n                        _q = P[vids]\n                        _pu = _q[:, 0] if side in (\"S\", \"N\") else _q[:, 1]\n                        _pz = _q[:, 2]\n                        _u0, _u1, _z0, _z1 = island\n                        _pad = 0.15\n                        _fits = (float(_pu.min()) >= _u0 - _pad and\n                                 float(_pu.max()) <= _u1 + _pad and\n                                 float(_pz.min()) >= _z0 - _pad and\n                                 float(_pz.max()) <= _z1 + _pad and\n                                 float(_pu.max() - _pu.min()) <= 1.25 * (_u1 - _u0) and\n                                 float(_pz.max() - _pz.min()) <= 1.25 * (_z1 - _z0))\n                        if not _fits:\n                            island = None\n                    if island is None:\n"""
    if needle not in src or bind_needle not in src:
        raise RuntimeError("could not install experimental whole-window guard")
    src = src.replace(bind_needle, bind_replacement).replace(needle, replacement)
    exec(compile(src,
                 "<experimental whole-window guard>", "exec"),
         gac_fire.__dict__)
    gac_fire.darken_glass = gac_fire.damage_windows
    # SM_Building_30's concrete window surround is unfortunately named
    # M_Building_30_Win_Concrete.  The generic name heuristic treats "Win"
    # as glazing and punches individual facade triangles out of it.
    gac_fire.PACKS["gac"].setdefault("glazing_material_deny", set()).add(
        "win_concrete")
    def _artifact_safe_windows(stage, ctx, pls, rects=None, mass=None,
                               sooted=None, glass_tex=None, deny_mat=None):
        ctx.setdefault("notes", []).append(
            "windows: triangle-level burnout disabled by experimental "
            "artifact-safe pipeline; original glazing retained")
        return 0
    # burn_gac resolves this name from gac_fire's module globals at call time.
    gac_fire.damage_windows = _artifact_safe_windows
    gac_fire.darken_glass = _artifact_safe_windows
    print("[fast_slice] whole-window containment guard installed; "
          "triangle-level window burnout disabled")
    return base
