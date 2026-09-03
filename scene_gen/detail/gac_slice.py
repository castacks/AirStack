"""gac_slice — cut a MERGED building mesh into kit-shaped pieces so the fire
ladder can damage it.

THE IDEA, WHICH IS THE USER'S
------------------------------
`detail/urban_building.py` assembles a building as BANDS x SIDES x CORNERS
then a roof: each elevation of each storey is a run of façade modules between
two corner pieces. `disaster/quake_flow.describe` reads that back out into an
element table, and every urban-fire recipe walks it. A merged whole-asset mesh
has no elements, so it gets a flat tint and nothing else.

But a merged mesh can be CUT on the same grammar (user, 2026-08-29: "since
it's 1 mesh but can be split up. Can you split it up by roof and then per story
into corner, multiple middle pieces? That's similar to how
moderncityenvironment is assembled right?"). It is — and GreatAmericanCity is
built on the same module grid as the kit.

THE GRID IS MEASURED, NEVER ASSUMED
------------------------------------
MEASURED (`tools/gac_grid_probe.py`, 2026-08-29) from the window centres of
the glass subset:

    SM_Building_01   13 storey lines, rise 4.00 m (sd 0.27)   bay 4.04 m (sd 0.00)
    SM_Building_04   12 storey lines, rise 4.09 m (sd 0.18)   bay 4.09 m (sd 0.08)

A 4.0 m storey on a 4.04 m bay, with essentially no deviation over 55 m — and
that is the same module `ModernCityEnvironment01` is authored on (its own
notes: "5 m / 4 m module"). `urban_fire.MONO_STOREY_M` assumed 3.4 m, which is
0.6 m per floor: over thirteen floors it is nearly two storeys of drift, and
every cut would have landed through the glazing. Hence `storey_period`, which
recovers the period from the data instead.

Not every asset is as clean. `SM_Building_24` glazes in BANDS rather than
punched openings and naive clustering reports 1.48 m (sd 0.46) — which is a
sub-feature, not a storey. `storey_period` scores candidate periods against
the whole distribution rather than trusting neighbour spacings, which is what
makes that case recoverable; `confidence` reports when it is not, and a
low-confidence asset should be left un-sliced rather than cut wrong.

WHOLE FACES, NOT CLIPPED ONES
------------------------------
Each triangle is assigned to a cell by its CENTROID and kept whole. That means
no polygon clipping, no barycentric UV rebuild, and no `cut_shell`-class
dependency on Shapely — and at 270k triangles against a 4 m cell there are
hundreds of triangles per cell, so the boundary is accurate to a triangle.
The cell edges come out slightly ragged, which is what a burnt break should
look like anyway (`damage-must-look-natural`: "ragged, disturbed, natural
breaks", not "very square/rectangular cutoffs").
"""

import math
import os

# `win` IS IN THE LIST BECAUSE THE PACK ABBREVIATES.
# GreatAmericanCity names some window materials `M_Building_NN_Win_Inst` —
# which contains "win" but NOT "window", so the original four-token filter
# matched nothing on them and `measure_grid` reported "fewer than 6 windows
# found". MEASURED (`tools/gac_grid_sweep.py`, 2026-08-29): only 7 of the 31
# buildings were measurable under the old list; adding "win" recovers
# thousands of glass faces on six more (05, 06_Small, 15, 19, 26, 30) and
# three of those then measure cleanly at confidence 0.55-0.65 with ZERO
# windows crossed by a cut.
#
# A frame matching "win" rather than the glass itself is fine here: this list
# is used to LOCATE the window grid, and a frame sits at the same place the
# glass does. It is deliberately NOT shared with
# `urban_fire._MONO_GLASS_TEX`, which classifies a part as glazing for
# shading and would be wrong to widen.
GLASS_TEX = ("glass", "window", "curtain", "glazing", "win",
             # PAINTED WINDOWS ARE WINDOWS TOO. 10 of the 12 GreatAmericanCity
             # towers (SM_Building_10/12/13/18/21/22/23/25/27/28) carry no glass
             # mesh at all: their glazing bands are opaque faces bound to
             # `M_Fake_Interior_0N`, `M_Fake_Light`, `M_Images_*_Off_Light` — a
             # background-LOD decal — so nothing found a window, no storey
             # grid was measured from them and no fire was ever planned on a
             # tower (probe sweep, 2026-08-30). Those faces are the openings.
             "fake_interior", "fake_light", "off_light")
# ...and an AWNING is not a window: "win" is in "awning", and the ground-floor
# canopies of SM_Building_02/04 were being counted as glazing (and blacked
# out on the band) because of it.
GLASS_TEX_NOT = ("awning",)


def is_glazing(tex_name, glass_tex=None, mat_name=None):
    """Does this base-map file name belong to a window / curtain-wall /
    painted-window material? Keyword match on the basename, minus the
    `GLASS_TEX_NOT` false friends.

    `mat_name`, when given, is the bound MATERIAL PRIM's own name, tested
    with the same token list as a SECOND chance. A downtowncity block binds
    its real glazing to materials called `Glass_window`, `glass`,
    `Window_003/4/5` and `rollershutter_window_01_001` that carry NO diffuse
    texture at all (MEASURED, `_plans/dtc_buildings.json`: 2-3 of every
    asset's glazing materials are untextured, and `Window_00N`'s map is a
    `ChatGPT Image ....png` whose name says nothing) — so a texture-only
    test finds one curtain-wall material per building and misses the
    punched windows entirely. It is the same test `tools/dtc_catalogue.py`
    already scored the catalogue's `glazing_by_side` with.

    A NO-OP ON GreatAmericanCity, BY MEASUREMENT, which is what keeps the
    GAC path frozen: every GAC material prim is named `UnrealMaterial`
    (`tools/_dtc_reg_probe.py`, 2026-08-30 — the same fact
    `rehome_materials` special-cases below), and `is_glazing` of that is
    False. Callers that pass nothing here get byte-identical behaviour:
    with `mat_name=None` the loop below sees exactly one candidate, and an
    `awning` texture still returns False rather than falling through.
    """
    toks = glass_tex or GLASS_TEX
    for cand in (tex_name, mat_name):
        low = str(cand or "").rsplit("/", 1)[-1].lower()
        if not low or any(n in low for n in GLASS_TEX_NOT):
            continue
        if any(g in low for g in toks):
            return True
    return False
# Candidate floor-to-floor heights, metres. Real buildings sit inside this;
# outside it a "period" is a window mullion or a whole massing band.
PERIOD_RANGE = (2.6, 5.4)
PERIOD_STEP = 0.02
# Below this the grid was not recovered and the asset must not be sliced.
MIN_CONFIDENCE = 0.55
SIDES = ("S", "N", "W", "E")


# ---------------------------------------------------------------------------
# The grid
# ---------------------------------------------------------------------------
def storey_period(zs, lo=None, hi=None):
    """(period, phase, confidence) of the strongest lattice in `zs`.

    A PERIODOGRAM, NOT A NEIGHBOUR SPACING. Taking `median(diff(sorted(z)))`
    is what reports 1.48 m on a banded façade: it measures the gap between
    whatever two features happen to be adjacent, so any sub-feature halves it
    and any missing row doubles it. Scoring each candidate period against
    EVERY window instead is immune to both — a mullion adds a consistent
    offset that still lands on the same lattice, and a missing row costs
    nothing.

    Confidence is 1 - 2*<|frac - 0.5|> mapped so that a perfect lattice is 1.0
    and a uniform scatter is 0.0.
    """
    if len(zs) < 6:
        return None, 0.0, 0.0
    lo = lo or PERIOD_RANGE[0]
    hi = hi or PERIOD_RANGE[1]
    best = (None, 0.0, -1.0)
    z0 = min(zs)
    n = int(round((hi - lo) / PERIOD_STEP)) + 1
    for k in range(n):
        p = lo + k * PERIOD_STEP
        # circular mean of the phase, which is how you average an angle
        sx = sy = 0.0
        for z in zs:
            a = 2.0 * math.pi * ((z - z0) / p % 1.0)
            sx += math.cos(a)
            sy += math.sin(a)
        r = math.hypot(sx, sy) / len(zs)          # 0 scattered .. 1 locked
        if r > best[2]:
            phase = (math.atan2(sy, sx) / (2.0 * math.pi)) % 1.0
            best = (p, (z0 + phase * p), r)
    return best[0], best[1], best[2]


def lattice(period, phase, lo, hi):
    """Every lattice line of `period` through [lo, hi]."""
    if not period or period <= 0:
        return []
    k0 = math.floor((lo - phase) / period)
    out = []
    k = k0
    while True:
        v = phase + k * period
        if v > hi + 1e-6:
            break
        if v >= lo - 1e-6:
            out.append(v)
        k += 1
    return out


def measure_grid(win_centres, bbox, verbose=True, name=""):
    """The storey/bay grid of one building from its window centres.

    `win_centres` is `{side: [(u, z), ...]}` in the building's own frame, `u`
    being the along-face coordinate. Returns a dict with `storey_h`,
    `storeys` (z of each floor line), `bay` per side, and `confidence`.
    """
    (x0, y0, z0), (x1, y1, z1) = bbox
    allz = [z for pts in win_centres.values() for _u, z in pts]
    per, phase, conf = storey_period(allz)
    if per is None:
        return {"confidence": 0.0, "why": "fewer than 6 windows found"}
    floors = lattice(per, phase, z0, z1)
    bays = {}
    for side, pts in win_centres.items():
        us = [u for u, _z in pts]
        if len(us) < 4:
            bays[side] = None
            continue
        span = (x1 - x0) if side in ("S", "N") else (y1 - y0)
        bp, bph, bconf = storey_period(us, 2.0, min(9.0, max(2.5, span / 2.0)))
        bays[side] = {"pitch": bp, "phase": bph, "confidence": bconf}
    g = {"storey_h": per, "storeys": floors, "bays": bays,
         "confidence": conf, "bbox": bbox,
         "W": x1 - x0, "D": y1 - y0, "H": z1 - z0, "z0": z0}
    if verbose:
        bl = ", ".join("{0}={1}".format(
            s, "-" if not b or not b["pitch"] else "%.2f m" % b["pitch"])
            for s, b in sorted(bays.items()))
        print("[gac_slice] {0}: {1:.1f} x {2:.1f} x {3:.1f} m, storey {4:.2f} m "
              "x{5} (confidence {6:.2f}), bays {7}".format(
                  name or "?", g["W"], g["D"], g["H"], per, len(floors),
                  conf, bl))
    return g


def window_centres(stage, prim_path, glass_tex=GLASS_TEX):
    """`({side: [(u, z)]}, bbox)` from the glass subsets of a PLACED building.

    The same read `tools/gac_slice_probe.py` validated the estimator with, in
    the module so a launcher does not reimplement it and drift from it.
    `u` is the along-face coordinate in the prim's own frame, which is what
    `measure_grid` bins the bay lattice on.
    """
    import numpy as np
    from pxr import Sdf, Usd, UsdGeom, UsdShade

    root = stage.GetPrimAtPath(prim_path)
    if not root or not root.IsValid():
        return {}, None
    xc = UsdGeom.XformCache()
    root_inv = xc.GetLocalToWorldTransform(root).GetInverse()

    def _tex(p):
        """(diffuse basename, MATERIAL PRIM NAME) of `p`'s bound material.

        The second half is what a downtowncity block needs: its window
        materials carry no diffuse map, so the name is the only evidence
        there is (see `is_glazing`). Empty on GAC in the sense that
        matters — every GAC material prim is called `UnrealMaterial`.
        """
        mat = UsdShade.MaterialBindingAPI(p).ComputeBoundMaterial()[0]
        if not mat or not mat.GetPrim().IsValid():
            return "", ""
        mname = mat.GetPrim().GetName()
        for c in Usd.PrimRange(mat.GetPrim()):
            sh = UsdShade.Shader(c)
            if not sh or sh.GetIdAttr().Get() != "UsdPreviewSurface":
                continue
            d = sh.GetInput("diffuseColor")
            if d is not None and d.HasConnectedSource():
                ts = UsdShade.Shader(d.GetConnectedSource()[0].GetPrim())
                f = ts.GetInput("file")
                v = f.Get() if f else None
                if isinstance(v, Sdf.AssetPath) and v.path:
                    return v.path.rsplit("/", 1)[-1], mname
            break
        return "", mname

    wins, lo, hi = {}, None, None
    for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        me = UsdGeom.Mesh(prim)
        pts = me.GetPointsAttr().Get()
        if pts is None or not len(pts):
            continue
        M = np.asarray(xc.GetLocalToWorldTransform(prim) * root_inv, dtype=float)
        P = np.asarray(pts, dtype=float)
        P = (np.c_[P, np.ones(len(P))] @ M)[:, :3]
        mn, mx = P.min(axis=0), P.max(axis=0)
        lo = mn if lo is None else np.minimum(lo, mn)
        hi = mx if hi is None else np.maximum(hi, mx)
        counts = np.asarray(me.GetFaceVertexCountsAttr().Get() or [], dtype=np.int64)
        fvi = np.asarray(me.GetFaceVertexIndicesAttr().Get() or [], dtype=np.int64)
        if not len(counts) or len(fvi) != int(counts.sum()):
            continue
        start = np.concatenate([[0], np.cumsum(counts)[:-1]])
        for sub in UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(prim)):
            _t, _mn = _tex(sub.GetPrim())
            if not is_glazing(_t, glass_tex, mat_name=_mn):
                continue
            for f in (sub.GetIndicesAttr().Get() or []):
                f = int(f)
                if f >= len(counts):
                    continue
                V = P[fvi[start[f]:start[f] + counts[f]]]
                cen = V.mean(axis=0)
                n = np.cross(V[1] - V[0], V[2] - V[0])
                ln = float(np.linalg.norm(n))
                if ln < 1e-12:
                    continue
                side = _side_of(*(n / ln))
                if side is None:
                    continue
                u = cen[1] if side in ("E", "W") else cen[0]
                wins.setdefault(side, []).append((float(u), float(cen[2])))
    bbox = (tuple(map(float, lo)), tuple(map(float, hi))) if lo is not None else None
    return wins, bbox


# ---------------------------------------------------------------------------
# The cut
# ---------------------------------------------------------------------------
def _side_of(nx, ny, nz):
    """Which elevation a face normal belongs to, or None for roof/floor."""
    if abs(nz) >= max(abs(nx), abs(ny)):
        return None
    return ("E" if nx > 0 else "W") if abs(nx) >= abs(ny) else ("N" if ny > 0 else "S")


def cell_of(cen, nrm, g, corner_m):
    """(role, side, storey, bay) for one face, or None to leave it merged.

    `corner_m` is how far from an edge of the elevation counts as the CORNER
    piece — the kit's corner legs are 1-8 m, so this is the same idea.
    """
    (x0, y0, z0), (x1, y1, z1) = g["bbox"]
    side = _side_of(*nrm)
    top = g["storeys"][-1] if g["storeys"] else z1
    if side is None:
        if cen[2] >= top - 0.5 * g["storey_h"]:
            return ("roof", "-", -1, 0)
        return None                     # interior floor/ceiling: leave merged
    # storey index from the measured lattice
    st = 0
    for i, z in enumerate(g["storeys"]):
        if cen[2] >= z - 1e-6:
            st = i
    # parapet: above the top floor line
    if cen[2] >= top - 1e-6:
        role = "parapet"
    else:
        role = "wall"
    u = cen[1] if side in ("E", "W") else cen[0]
    ulo = (y0 if side in ("E", "W") else x0)
    uhi = (y1 if side in ("E", "W") else x1)
    if (u - ulo) <= corner_m or (uhi - u) <= corner_m:
        role = "corner" if role == "wall" else "parapet_corner"
    b = g["bays"].get(side) or {}
    pitch = b.get("pitch") or g["storey_h"]
    bay = int((u - ulo) // max(0.5, pitch))
    return (role, side, st, bay)


def slice_building(stage, src_path, dst_scope, g, style, corner_m=None,
                   verbose=True):
    """Cut the mesh at `src_path` into one prim per cell under `dst_scope`.

    Returns the kit-shaped PLACEMENT list — `category`, `usd`, `x_m`, `y_m`,
    `z_m`, `yaw_deg`, `prim_path` — which is the entire contract
    `quake_flow.classify` reads. Nothing downstream needs to know the building
    was ever a single mesh.
    """
    import numpy as np
    from pxr import Sdf, UsdGeom, UsdShade, Vt

    corner_m = corner_m if corner_m is not None else max(1.5, 0.5 * g["storey_h"])
    src = stage.GetPrimAtPath(src_path)
    if not src or not src.IsValid():
        raise ValueError("no prim at {0}".format(src_path))
    meshes = [p for p in src.GetChildren() if p.IsA(UsdGeom.Mesh)]
    if src.IsA(UsdGeom.Mesh):
        meshes = [src]
    if not meshes:
        from pxr import Usd
        meshes = [p for p in Usd.PrimRange(src) if p.IsA(UsdGeom.Mesh)]
    UsdGeom.Scope.Define(stage, Sdf.Path(dst_scope))
    xc = UsdGeom.XformCache()
    root_inv = xc.GetLocalToWorldTransform(src).GetInverse()

    cells = {}
    mats_of = {}
    for prim in meshes:
        me = UsdGeom.Mesh(prim)
        pts = me.GetPointsAttr().Get()
        counts = me.GetFaceVertexCountsAttr().Get()
        fvi = me.GetFaceVertexIndicesAttr().Get()
        if not pts or not counts or not fvi:
            continue
        M = np.asarray(xc.GetLocalToWorldTransform(prim) * root_inv, dtype=float)
        P = np.asarray(pts, dtype=float)
        P = (np.c_[P, np.ones(len(P))] @ M)[:, :3]
        counts = np.asarray(counts, dtype=np.int64)
        fvi = np.asarray(fvi, dtype=np.int64)
        start = np.concatenate([[0], np.cumsum(counts)[:-1]])
        # EVERY faceVarying UV SET, BY DISCOVERY, NOT BY NAME.
        # Hardcoding "uv0" is what made the first sliced buildings render as
        # ONE FLAT BROWN: GreatAmericanCity names its UVs `st` (measured,
        # `tools/gac_uv_probe.py` — and SM_Building_04/_24 also carry
        # st1/st2/st3), so the lookup returned nothing, the pieces were
        # written with no UVs at all, and every face sampled a single texel.
        # Geometry, materials and bindings were all correct, which is what
        # made it look like a material bug rather than a UV one.
        # `disaster/monolith_damage.cut_shell` still has this same hardcoding.
        uvsets = []
        for pv in UsdGeom.PrimvarsAPI(prim).GetPrimvars():
            if str(pv.GetTypeName()) not in ("texCoord2f[]", "float2[]"):
                continue
            if str(pv.GetInterpolation()) != "faceVarying":
                continue
            vals = pv.Get()
            if vals is None or not len(vals):
                continue
            uvsets.append((pv.GetPrimvarName(), list(vals),
                           list(pv.GetIndices() or []) if pv.IsIndexed() else []))
        # face -> bound material, via the subsets
        face_mat = {}
        for sub in UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(prim)):
            mp = UsdShade.MaterialBindingAPI(sub.GetPrim()).ComputeBoundMaterial()[0]
            key = str(mp.GetPrim().GetPath()) if mp and mp.GetPrim().IsValid() else ""
            mats_of[key] = mp
            for f in (sub.GetIndicesAttr().Get() or []):
                face_mat[int(f)] = key
        default_mat = ""
        mb = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
        if mb and mb.GetPrim().IsValid():
            default_mat = str(mb.GetPrim().GetPath())
            mats_of[default_mat] = mb

        for f in range(len(counts)):
            idx = fvi[start[f]:start[f] + counts[f]]
            V = P[idx]
            cen = V.mean(axis=0)
            n = np.cross(V[1] - V[0], V[2] - V[0])
            ln = float(np.linalg.norm(n))
            if ln < 1e-12:
                continue
            key = cell_of(cen, n / ln, g, corner_m)
            if key is None:
                key = ("core", "-", -2, 0)
            rec = cells.setdefault(key, {"P": [], "counts": [], "idx": [],
                                         "uv": {}, "sub": {}})
            base = len(rec["P"])
            for k, vi in enumerate(idx):
                rec["P"].append(tuple(P[vi]))
                j = start[f] + k
                for nm_uv, vals, inds in uvsets:
                    rec["uv"].setdefault(nm_uv, []).append(
                        tuple(vals[inds[j] if inds else j]))
            rec["counts"].append(int(counts[f]))
            rec["idx"].extend(range(base, base + int(counts[f])))
            rec["sub"].setdefault(face_mat.get(f, default_mat), []).append(
                len(rec["counts"]) - 1)

    # hand the caller the source materials so it can re-home them
    slice_building.last_materials = dict(mats_of)
    placements, n_prim = [], 0
    for (role, side, st, bay), rec in sorted(cells.items()):
        if not rec["counts"]:
            continue
        nm = "{0}_{1}_{2}_{3}".format(role, side, st, bay).replace("-", "x")
        path = "{0}/{1}".format(dst_scope, nm)
        m = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
        m.CreatePointsAttr(Vt.Vec3fArray([tuple(map(float, q)) for q in rec["P"]]))
        m.CreateFaceVertexCountsAttr(Vt.IntArray(rec["counts"]))
        m.CreateFaceVertexIndicesAttr(Vt.IntArray(rec["idx"]))
        for nm_uv, vals in rec["uv"].items():
            pv = UsdGeom.PrimvarsAPI(m).CreatePrimvar(
                nm_uv, Sdf.ValueTypeNames.TexCoord2fArray,
                UsdGeom.Tokens.faceVarying)
            pv.Set(Vt.Vec2fArray([tuple(map(float, q)) for q in vals]))
        # ONE SUBSET PER ORIGINAL MATERIAL. Re-binding the whole cell to one
        # material is what turns a sliced brick building into a sliced grey
        # building; the identity of the art has to survive the cut.
        for key, faces in rec["sub"].items():
            mp = mats_of.get(key)
            if mp is None or not faces:
                continue
            sub = UsdGeom.Subset.CreateGeomSubset(
                m, "mat_{0}".format(abs(hash(key)) % 100000),
                UsdGeom.Tokens.face, Vt.IntArray(sorted(faces)))
            UsdShade.MaterialBindingAPI(sub.GetPrim()).Bind(mp)
        import numpy as np
        C = np.asarray(rec["P"], dtype=float)
        cx, cy = float(C[:, 0].mean()), float(C[:, 1].mean())
        cz = float(C[:, 2].min())
        sz = C.max(axis=0) - C.min(axis=0)
        placements.append({
            "category": "bld_{0}_{1}".format(style, _sub_for(role)),
            "usd": "gacslice://{0}".format(nm),
            "x_m": cx, "y_m": cy, "z_m": cz, "yaw_deg": 0.0,
            "scale": 1.0, "prim_path": path,
            "_size": (float(sz[0]), float(sz[1]), float(sz[2])),
            "_role": role, "_side": side, "_storey": st, "_bay": bay})
        n_prim += 1
    if verbose:
        by = {}
        for p in placements:
            by[p["_role"]] = by.get(p["_role"], 0) + 1
        print("[gac_slice] {0} -> {1} piece(s): {2}".format(
            src_path.rsplit("/", 1)[-1], n_prim,
            "  ".join("{0}={1}".format(k, v) for k, v in sorted(by.items()))))
    return placements


def _material_source(mat_prim):
    """(layer identifier, prim path) of the material's OWN usd file.

    MEASURED (`tools/gac_mat_probe.py`): every GreatAmericanCity material is a
    separate file — `/SM_Building_01/LOD0/Section0/UnrealMaterial` has specs in
    both `Meshes/SM_Building_01.usd` and `Materials/M_Images_Inst.usd`. So a
    sliced piece can own its material by REFERENCING that file, rather than
    depending on the merged original staying on the stage.

    THE SIDECAR IS A GAC HABIT, NOT A RULE. A downtowncity block
    (`assets/downtowncity/Amar_Tower.usdc`) has NO `Materials/*_Inst.usd`:
    its 33-88 materials are authored INLINE in the one `.usdc` beside the
    mesh. Returning `(None, None)` for those made `fire_bake.
    rehome_for_export` count every one of them as FAILED, which keeps
    `<cell>/src` in the export (`src_kept: true`) — the whole merged tower,
    invisible, in every bake. The asset's own layer is a perfectly good home
    for the material: a fresh prim referencing `(that layer, that material's
    path in it)` composes the material and nothing else, exactly as the
    `_Inst.usd` reference does, and the sliced pieces stop depending on the
    SUBTREE while still depending on the FILE (which they always did).

    So: the `Materials/` sidecar still wins wherever there is one — the GAC
    path returns the identical pair it always did, because that branch
    `return`s before the fallback is ever consulted — and everything else
    falls back to the strongest NON-ANONYMOUS layer carrying a spec for the
    prim, which for a merged asset is the asset file itself. Anonymous
    layers (the stage's own root/session layer, where `piece_material_like`
    writes its soot override) are skipped: referencing those would be
    circular and would not survive the export.
    """
    fallback = (None, None)
    for spec in mat_prim.GetPrimStack():
        ident = str(spec.layer.identifier)
        if "/Materials/" in ident or ident.endswith("_Inst.usd"):
            return ident, spec.path
        if fallback[0] is None and not spec.layer.anonymous:
            fallback = (ident, spec.path)
    return fallback


def rehome_materials(stage, mat_prims, dst_looks, verbose=True):
    """Give the sliced pieces their OWN materials under `dst_looks`.

    WITHOUT THIS THE PIECES ARE NOT A KIT. They bind to Material prims that
    live inside the merged source's subtree, so they only render while that
    source is still composed — and deactivating it (the obvious thing to do
    once the pieces exist) turns every piece WHITE with its geometry and its
    UVs perfectly intact. That failure is silent and looks like a texture bug.

    Returns `{source material path: UsdShade.Material}`.
    """
    from pxr import Sdf, UsdGeom, UsdShade

    UsdGeom.Scope.Define(stage, Sdf.Path(dst_looks))
    out, n_ref, n_copy = {}, 0, 0
    for key, mp in mat_prims.items():
        if mp is None or not mp.GetPrim().IsValid():
            continue
        name = mp.GetPrim().GetName()
        if name in ("UnrealMaterial", "material"):
            # every GAC section names its material the same thing; take the
            # file stem instead or they all collapse onto one prim
            ident, _pp = _material_source(mp.GetPrim())
            if ident:
                name = os.path.basename(ident).rsplit(".", 1)[0]
        path = "{0}/{1}".format(dst_looks, name)
        if stage.GetPrimAtPath(Sdf.Path(path)).IsValid():
            out[key] = UsdShade.Material.Get(stage, path)
            continue
        ident, pp = _material_source(mp.GetPrim())
        newp = None
        if ident:
            newp = stage.DefinePrim(Sdf.Path(path))
            if pp is not None:
                newp.GetReferences().AddReference(ident, Sdf.Path(pp))
            else:
                newp.GetReferences().AddReference(ident)
            n_ref += 1
        else:
            # inline material: copy the composed spec into the session layer
            newp = stage.DefinePrim(Sdf.Path(path))
            src_spec = mp.GetPrim().GetPrimStack()[0]
            try:
                Sdf.CopySpec(src_spec.layer, src_spec.path,
                             stage.GetEditTarget().GetLayer(), Sdf.Path(path))
                n_copy += 1
            except Exception:
                newp = None
        if newp is not None and UsdShade.Material.Get(stage, path):
            out[key] = UsdShade.Material.Get(stage, path)
    if verbose:
        print("[gac_slice] materials re-homed to {0}: {1} referenced, "
              "{2} copied, {3} bound".format(dst_looks, n_ref, n_copy, len(out)))
    return out


def explode(stage, placements, out_m=1.5, up_m=1.0, verbose=True):
    """Push every piece off the assembly so the cut can be SEEN.

    A sliced building that still stands assembled looks exactly like the
    building it was cut from — which is the one thing an inspection view must
    not do. Each piece moves outward along its own elevation's normal and up
    by its storey, so the result reads as an exploded assembly drawing.
    """
    from pxr import Gf, UsdGeom

    nrm = {"S": (0.0, -1.0), "N": (0.0, 1.0),
           "E": (1.0, 0.0), "W": (-1.0, 0.0), "-": (0.0, 0.0)}
    n = 0
    for p in placements:
        prim = stage.GetPrimAtPath(p["prim_path"])
        if not prim or not prim.IsValid():
            continue
        side, st = p.get("_side", "-"), max(0, int(p.get("_storey", 0)))
        ox, oy = nrm.get(side, (0.0, 0.0))
        dz = up_m * st
        if p.get("_role") == "roof":
            dz = up_m * (st + 3.0) if st >= 0 else up_m * 3.0
        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(ox * out_m, oy * out_m, dz))
        n += 1
    if verbose:
        print("[gac_slice] exploded {0} piece(s) by {1:.1f} m out, "
              "{2:.1f} m per storey".format(n, out_m, up_m))
    return n


def _sub_for(role):
    """Element role -> the `sub` token `quake_flow._sub_and_mass` expects."""
    return {"wall": "storey", "corner": "storey_corner",
            "parapet": "parapet", "parapet_corner": "parapet_corner",
            "roof": "roof", "core": "storey"}.get(role, "storey")


# ---------------------------------------------------------------------------
# Making the sliced building look like a STYLE to the rest of the pipeline
# ---------------------------------------------------------------------------
def register_style(g, name, pieces_of=None, family="01"):
    """Register a synthetic `urban_building` style for a sliced building.

    `quake_flow.describe` does `ub.STYLES[style]` and `_mass_specs` derives the
    mass box from that spec — so a sliced building needs a spec whose
    footprint and bands are ITS OWN measured ones, or every recipe would place
    art against a kit building's dimensions instead. This builds that spec from
    the measured grid and installs it, along with per-piece sizes in
    `ub.PIECES` so `classify` reads the real storey height rather than its
    3.0 m fallback.

    THE STOREY COUNT IS FILTERED THE SAME WAY `gac_fire.mass_from_grid`
    FILTERS ITS OWN `levels`, and for the same reason: `grid_for`'s raw
    `g["storeys"]` can carry one mark within 0.5 m of the bbox top — the
    parapet coping on a measured lattice that happens to phase-lock this
    close, or (on a `regular_grid` fallback, whose lines run `z0` to `z1`
    inclusive) the bbox top ITSELF, always exactly `z1`. `mass_from_grid`
    already treats that mark as "not a floor" for `fire["top"]`/
    `fire["n_storeys"]`; this function used to count it anyway, so the
    runtime storey table `_mass_specs` builds from `ub.STYLES[name]` (what
    `quake_flow.describe` — fire AND the sliced earthquake ladder both go
    through it — actually hands every recipe) carried ONE MORE entry than
    fire planning's own count, a phantom storey sitting at/above the real
    roof. `r_expose_interior`'s catch floor could walk onto it and land ON
    the parapet instead of one storey below the deck (SM_Building_11 F4,
    user review 2026-08-31: "roof seems to have a bunch of debris ...
    building can't just have debris on its roof for no reason"); worse,
    `r_fire_collapse`'s own `n_lv = len(m["levels"])` fed `s0` and the
    heap-height formula's `m["top"]` from the SAME inflated table, so a
    building that genuinely DID collapse still piled heap chips and
    fit-out column tops above its real, collapsed deck (SM_Building_26 F5,
    user review 2026-08-31: "this building has debris on the roof and
    pillars sticking out, i thought we were fixing that").

    Filtering here, at the one place that BUILDS the runtime table, fixes
    every reader downstream at once and needs no per-recipe deck-height
    guard to compensate for it. Silently skipped when `g` carries no
    `"bbox"` (test/tool fixtures that hand-build a grid with no bbox key —
    `tests/test_quake_sliced.py`'s `_fixture`, `tools/quake_gac_probe.py`
    — a synthetic grid built storey-by-storey already has no phantom mark
    to filter, and the production callers, `gac_storey_slice.grid_for`'s
    `measure_grid`/`regular_grid`, always set `bbox`).
    """
    from detail import urban_building as ub

    per = g["storey_h"]
    storeys = g.get("storeys") or []
    bbox = g.get("bbox")
    if bbox is not None:
        (_x0, _y0, z0), (_x1, _y1, z1) = bbox
        filtered = sorted(z for z in storeys if z0 - 1e-6 <= z < z1 - 0.5)
        if not filtered or filtered[0] > z0 + 0.5:
            filtered = [float(z0)] + filtered
        storeys = filtered
    # The measured floor marks are not necessarily a uniform lattice.  Keep
    # them on the synthetic style instead of later reconstructing N copies of
    # one median `storey_h` (SM_Building_02: that reconstruction put the roof
    # 4.8 m too high and created protruding storey-10 columns).
    z_base = float(bbox[0][2]) if bbox is not None else 0.0
    roof_piece_z = [float(p.get("z_m", 0.0)) for p in (pieces_of or [])
                    if p.get("_role") == "roof"]
    parapet_piece_z = [float(p.get("z_m", 0.0)) for p in (pieces_of or [])
                       if p.get("_role") in ("parapet", "parapet_corner")]
    deck_z = (min(roof_piece_z) if roof_piece_z else
              (min(parapet_piece_z) if parapet_piece_z else None))
    if deck_z is not None:
        storeys = [z for z in storeys if z < deck_z - 0.25]
    envelope_roles = ("wall", "pier", "corner")
    envelope_storeys = [int(p.get("_storey", 0)) for p in (pieces_of or [])
                        if p.get("_role") in envelope_roles]
    if envelope_storeys:
        # A measured cut can sit below the roof yet bound only the dedicated
        # roof/parapet band. It is not another occupiable floor. Keep exactly
        # the starts backed by an envelope band.
        storeys = storeys[:max(envelope_storeys) + 1]
    n = max(1, len(storeys))
    module = None
    for b in (g.get("bays") or {}).values():
        if b and b.get("pitch"):
            module = b["pitch"] if module is None else min(module, b["pitch"])
    module = module or per
    leg = max(1.0, min(3.0, 0.5 * module))
    nx = max(1, int(round((g["W"] - 2 * leg) / module)))
    ny = max(1, int(round((g["D"] - 2 * leg) / module)))
    spec = {"bays": (nx, ny), "note": "sliced from a merged mesh",
            "family": family,      # `quake_flow.FAMILY_TYPE` -> construction type
            "measured_levels_m": [float(z) - z_base for z in storeys],
            "measured_deck_m": ((float(deck_z) - z_base)
                                if deck_z is not None else None),
            "bands": [{"sub": "storey", "h": per, "module": module,
                       "repeat": n, "walls": [], "corner": None},
                      {"sub": "parapet", "h": max(0.8, 0.35 * per),
                       "module": module, "parapet": True, "walls": [],
                       "corner": None}]}
    ub.STYLES[name] = spec
    # A `PIECES` ENTRY IS A SIX-TUPLE, NOT A SIZE.
    # `(sx, sy, sz, xmin, ymin, zmin)` — `urban_building.py:260` unpacks all
    # six (`sx, sy, _sz, xmin, ymin, _zmin = PIECES[name]`). Registering a
    # 3-tuple here parsed fine, survived an F1 building whose recipes never
    # reached that line, and then failed on the next one with "not enough
    # values to unpack (expected 6, got 3)" — a good example of why the
    # cheapest severity is the worst thing to smoke-test on.
    #
    # `slice_building` anchors each piece at its own (centroid_x, centroid_y,
    # min_z), so the piece-local bbox minimum is (-sx/2, -sy/2, 0).
    for p in (pieces_of or []):
        sx, sy, sz = p["_size"]
        ub.PIECES.setdefault(_piece_key(p),
                             (sx, sy, sz, -0.5 * sx, -0.5 * sy, 0.0))
    return spec


def _piece_key(p):
    return str(p.get("usd", "")).rsplit("/", 1)[-1].rsplit(".", 1)[0]


def check(verbose=True):
    """Host-side: the grid maths, with no USD in sight."""
    bad = []
    # a perfect 4 m lattice must be recovered exactly
    zs = [2.0 + 4.0 * k for k in range(13)]
    per, phase, conf = storey_period(zs)
    if per is None or abs(per - 4.0) > 0.05:
        bad.append("clean 4 m lattice recovered as {0}".format(per))
    if conf < 0.95:
        bad.append("clean lattice confidence only {0:.2f}".format(conf))
    # ... and must survive a mullion at each mid-height, which is exactly what
    # breaks a median-of-differences estimate (it reports 2.0 m, not 4.0)
    noisy = sorted(zs + [z + 2.0 for z in zs[:-1]])
    import statistics
    med = statistics.median([b - a for a, b in zip(noisy, noisy[1:])])
    per2, _ph, conf2 = storey_period([z for z in zs])
    if abs(med - 2.0) > 0.2:
        bad.append("fixture wrong: median spacing {0}".format(med))
    if per2 is None or abs(per2 - 4.0) > 0.05:
        bad.append("periodogram lost the 4 m period ({0})".format(per2))
    # a scatter must NOT produce a confident period
    import random
    rng = random.Random(3)
    sc = [rng.uniform(0.0, 50.0) for _ in range(60)]
    _p, _ph, c3 = storey_period(sc)
    if c3 > MIN_CONFIDENCE:
        bad.append("random scatter scored confidence {0:.2f}".format(c3))
    # lattice() spans the range
    ls = lattice(4.0, 2.0, 0.0, 50.0)
    if len(ls) != 13 or abs(ls[0] - 2.0) > 1e-9:
        bad.append("lattice() gave {0} line(s) starting {1}".format(len(ls), ls[:1]))
    # side classification
    if _side_of(1, 0, 0) != "E" or _side_of(0, -1, 0) != "S" or _side_of(0, 0, 1):
        bad.append("_side_of misclassifies")
    if _sub_for("corner") != "storey_corner" or _sub_for("roof") != "roof":
        bad.append("_sub_for does not match quake_flow's tokens")
    if verbose:
        print("[gac_slice] check {0}".format("ok" if not bad else "FAILED"))
        for b in bad:
            print("  " + b)
    return bad


if __name__ == "__main__":
    raise SystemExit(1 if check() else 0)
