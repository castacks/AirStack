"""gac_fire — a MERGED whole-asset building (GreatAmericanCity) through the
fire ladder, with its soot planned and baked BEFORE the slice.

WHY BEFORE THE SLICE. `urban_fire.burn_building` on a kit building bakes the
soot skin into every module's own base map through that module's UVs
(`_bind_soot` / `soot_bake`) — one baked map per sooted module, which is right
for a kit whose modules each carry their own small atlas. A GAC building is
the opposite: ONE mesh, ~14 material subsets, a few dozen textures shared by
the whole façade, then sliced into hundreds of pieces that all sample the
same atlases. Baking per piece would write hundreds of copies of the same
2K maps. So (user, 2026-08-30: "we have the 1 png that's on the building and
we can set up the soot pattern before splitting it up"):

    measure the windows and the storey grid on the MERGED asset
      -> a mass box + a fire plan (`urban_fire.plan_fire`) + fire EVENTS from
         the asset's own window islands (`soot_plume.plan_events`)
      -> the soot skin (`soot_plume.skin`) round that box
      -> baked ONCE into each material's atlas through the merged mesh's UVs
         (`soot_bake.uv_position_map` on the de-indexed mesh `read_mesh`
         already produces for the slicer; the side of every texel is the
         nearest wall line, so one bake covers all four elevations)
      -> a copy of each material with only its diffuse map swapped
         (`soot_plume.piece_material_like`)
      -> THEN `gac_storey_slice.slice_to_kit` (or a baked kit via
         `kit_bake.load_kit`), every piece subset rebound to the sooted copy
         of whatever it was bound to
      -> `burn_building(..., fire=, events=, openings_fn=, soot_prebaked=True)`
         runs the rest of the ladder — windows, gutting, roof, collapse,
         flames from the SAME events — and skips its own per-piece soot.

The same list of events drives the stain and the flames, exactly as on a kit
building; the only difference is WHERE the bake happens.

WHAT THE WINDOWS ARE. `gac_slice.window_centres` returns one point per glass
FACE; a window is an island of those. `window_rects` groups the glass faces
of each elevation into islands by grid-hashed union-find on their bboxes and
returns (u0, u1, z0, z1) per island in the asset's frame — these are the
openings the events vent through, in the record shape `soot_plume` and
`urban_fire._flame_sources` expect (a synthetic wall frame per side, so
`quake_flow._b_face_pt` places the Flow emitters on the real façade plane).

GLASS. The slicer cannot address a window, so `r_window_burnout` does
nothing on a GAC piece (it says so and returns, `soot_prebaked` being a
set/frozenset). `damage_windows` (`darken_glass` is now an alias) treats the
merged asset's own measured window ISLANDS instead, per window rather than
per whole material subset: a hot-side window in the fire's own band is
burnt out to a real see-through hole; one storey above the band, and any
band-or-above storey on a side that neighbours a hot one, is crazed —
sooted, dark, semi-transparent, still glazed; everything else keeps its
glass. `burn_gac` tops up its own `fit_storeys` so there is a gutted floor
and its fallen debris behind a hole that is now something you can see
through, not just an opaque void tone.
"""

import hashlib
import math
import os

import numpy as np

GAC_DIR = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"
           "GreatAmericanCity/Assets/Game/GreatAmericanCity/Meshes/")
GAC_SCALE = 0.01                 # the pack is authored in centimetres
ISLAND_CELL_M = 0.30             # grid hash cell for window-island grouping
BAKE_PX_MIN, BAKE_PX_MAX = 1024, 2048
# A texel is SHARED when faces more than this far apart in height both
# sample it — the atlas tiles up the building, and a pre-slice bake would put
# one storey's soot on every storey that reuses the texel. Such atlases are
# baked per PIECE after the slice instead (the kit path).
SHARED_TEXEL_M = 2.0
SHARED_FRAC_MAX = 0.08


# ---------------------------------------------------------------------------
# Placing the merged asset
# ---------------------------------------------------------------------------
def place_source(stage, cell, usd, scale=GAC_SCALE):
    """Reference the merged asset under `cell/src`, centred in plan on the
    cell with its base at the cell's z. Returns the holder path or None.
    (The same seat `gac_kit_launch_script.place_source` uses.)"""
    from pxr import Gf, Sdf, Usd, UsdGeom

    holder = cell + "/src"
    UsdGeom.Xform.Define(stage, Sdf.Path(holder))
    kid = stage.DefinePrim(Sdf.Path(holder + "/asset"))
    kid.GetReferences().AddReference(usd)
    stage.Load(Sdf.Path(holder))
    xf = UsdGeom.Xformable(kid)
    xf.ClearXformOpOrder()
    tr = xf.AddTranslateOp()
    if abs(scale - 1.0) > 1e-9:
        xf.AddScaleOp().Set(Gf.Vec3f(scale, scale, scale))
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    r = cache.ComputeWorldBound(stage.GetPrimAtPath(holder)).ComputeAlignedRange()
    if r.IsEmpty():
        return None
    mn, mx = r.GetMin(), r.GetMax()
    c = UsdGeom.XformCache().GetLocalToWorldTransform(
        stage.GetPrimAtPath(cell)).ExtractTranslation()
    tr.Set(Gf.Vec3d(-(0.5 * (mn[0] + mx[0]) - c[0]),
                    -(0.5 * (mn[1] + mx[1]) - c[1]), -(mn[2] - c[2])))
    return holder


# ---------------------------------------------------------------------------
# Windows as islands
# ---------------------------------------------------------------------------
def window_rects(stage, src, glass_tex=None, planes=None):
    """`{side: [(u0, u1, z0, z1), ...]}` of the glass ISLANDS on each
    elevation, in the asset holder's frame (`u` = x on S/N, y on E/W — the
    same `u` `gac_slice.window_centres` reports). Islands are glass faces
    whose bboxes share a 0.3 m grid cell, joined by union-find.

    `planes`, if given a dict, is filled in place with `{side: plane_coord}`
    — the real façade plane per elevation (x for E/W, y for S/N), measured as
    the MEDIAN outward coordinate of the glass faces on that side plus 0.15 m
    outward (the glass sits recessed into the wall opening by roughly that
    much). The asset's overall bbox face is the wrong plane to hang a flame
    or a frame origin off: it is the outer extent of whatever sticks out
    furthest (cornices, canopies, signage), which can be 1-3 m proud of the
    real wall the windows sit in."""
    from detail import gac_slice as gsl
    from pxr import Usd, UsdGeom, UsdShade

    glass_tex = glass_tex or gsl.GLASS_TEX
    root = stage.GetPrimAtPath(src)
    if not root or not root.IsValid():
        return {}
    xc = UsdGeom.XformCache()
    root_inv = xc.GetLocalToWorldTransform(root).GetInverse()
    # outward-normal sign per side, matching `quake_flow._b_face_pt`'s
    # convention (E/N outward is +x/+y, S/W outward is -y/-x): the plane
    # measured from the glass is pushed 0.15 m further OUT, away from the
    # building centre, to land on the wall face rather than the pane.
    out_sign = {"E": 1.0, "N": 1.0, "S": -1.0, "W": -1.0}
    glass_recess_m = 0.15

    def _tex(p):
        mat = UsdShade.MaterialBindingAPI(p).ComputeBoundMaterial()[0]
        if not mat or not mat.GetPrim().IsValid():
            return ""
        sp, inp, url = _diffuse_of(mat.GetPrim())
        return (url or "").rsplit("/", 1)[-1]

    # A PANE IS ON THE ELEVATION IT STANDS IN, NOT THE ONE ITS NORMAL POINTS
    # TO. Glass is double-sided in these assets: every pane has a back face
    # with the opposite normal, and classifying by normal (`gsl._side_of`)
    # put that back face on the OPPOSITE elevation as a phantom window — a
    # mirror image of the real E windows filed under W (SM_Building_02's
    # "W" plane measured 11.27 m, its real E plane 11.63 m; insets of up to
    # 57 m on the phantom sides, `tools/gac_plane_probe.py`, 2026-08-30).
    # `prepare` then picked E/W as the burning sides by island count and
    # planned events, soot and flames on a blank W wall. Two passes: gather
    # every vertical glass face with the asset's extent, file each by the
    # bbox face its centroid is NEAREST, take the median plane per side,
    # and keep only the faces within `plane_tol_m` of that plane (interior
    # partitions and atrium glass are not façade).
    plane_tol_m = 1.5
    faces = []        # (V, side_by_normal_ok)
    lo = np.full(3, np.inf)
    hi = np.full(3, -np.inf)
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
        lo = np.minimum(lo, P.min(0))
        hi = np.maximum(hi, P.max(0))
        counts = np.asarray(me.GetFaceVertexCountsAttr().Get() or [], dtype=np.int64)
        fvi = np.asarray(me.GetFaceVertexIndicesAttr().Get() or [], dtype=np.int64)
        if not len(counts) or len(fvi) != int(counts.sum()):
            continue
        start = np.concatenate([[0], np.cumsum(counts)[:-1]])
        for sub in UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(prim)):
            if not gsl.is_glazing(_tex(sub.GetPrim()), glass_tex):
                continue
            for f in (sub.GetIndicesAttr().Get() or []):
                f = int(f)
                if f >= len(counts):
                    continue
                V = P[fvi[start[f]:start[f] + counts[f]]]
                n = np.cross(V[1] - V[0], V[2] - V[0])
                ln = float(np.linalg.norm(n))
                if ln < 1e-12:
                    continue
                n = n / ln
                if abs(n[2]) >= max(abs(n[0]), abs(n[1])):
                    continue          # roof light / floor glass
                faces.append(V)
    if not faces or not np.all(np.isfinite(lo)):
        return {}
    cen = np.array([V.mean(0) for V in faces])
    # distance of each face centroid to the four bbox faces: S, E, N, W
    dist = np.stack([cen[:, 1] - lo[1], hi[0] - cen[:, 0],
                     hi[1] - cen[:, 1], cen[:, 0] - lo[0]], axis=1)
    side_ix = np.argmin(dist, axis=1)
    ring = ("S", "E", "N", "W")
    boxes = {}   # side -> list of (u0, u1, z0, z1)
    plane_vals = {}
    for k, side in enumerate(ring):
        sel = np.nonzero(side_ix == k)[0]
        if not len(sel):
            continue
        axis = 0 if side in ("E", "W") else 1
        oc = cen[sel, axis]
        plane = float(np.median(oc))
        keep = sel[np.abs(oc - plane) <= plane_tol_m]
        if not len(keep):
            continue
        plane_vals[side] = [float(v) for v in cen[keep, axis]]
        for i in keep:
            V = faces[i]
            uu = V[:, 1] if side in ("E", "W") else V[:, 0]
            boxes.setdefault(side, []).append(
                (float(uu.min()), float(uu.max()),
                 float(V[:, 2].min()), float(V[:, 2].max())))
    out = {}
    for side, bl in boxes.items():
        out[side] = _islands(bl)
    if planes is not None:
        for side, vals in plane_vals.items():
            planes[side] = float(np.median(vals)) + out_sign[side] * glass_recess_m
    return out


def _islands(boxes, cell=ISLAND_CELL_M):
    """Union-find over boxes that share a grid cell -> merged bboxes."""
    n = len(boxes)
    if n == 0:
        return []
    parent = list(range(n))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[rb] = ra

    owner = {}
    for i, (u0, u1, z0, z1) in enumerate(boxes):
        c0, c1 = int(math.floor(u0 / cell)), int(math.floor(u1 / cell))
        r0, r1 = int(math.floor(z0 / cell)), int(math.floor(z1 / cell))
        for c in range(c0, c1 + 1):
            for r in range(r0, r1 + 1):
                k = (c, r)
                j = owner.get(k)
                if j is None:
                    owner[k] = i
                else:
                    union(i, j)
    agg = {}
    for i, (u0, u1, z0, z1) in enumerate(boxes):
        r = find(i)
        a = agg.get(r)
        if a is None:
            agg[r] = [u0, u1, z0, z1]
        else:
            a[0], a[1] = min(a[0], u0), max(a[1], u1)
            a[2], a[3] = min(a[2], z0), max(a[3], z1)
    rects = [tuple(v) for v in agg.values()
             if (v[1] - v[0]) >= 0.25 and (v[3] - v[2]) >= 0.25]
    rects.sort(key=lambda r: (r[2], r[0]))
    return rects


# ---------------------------------------------------------------------------
# The mass box and the openings provider
# ---------------------------------------------------------------------------
def mass_from_grid(g, bbox, mesh=None):
    """A `quake_flow`-shaped mass dict for the merged asset, in the CELL
    frame (the asset is centred on the cell by `place_source`).

    `top` is the bbox z1 -- the highest point ANYTHING on the roof reaches,
    which is the parapet coping, not the deck. `deck_z` is the real roof
    surface everything should sit ON (row-2 review, 2026-08-30: "floating
    roof props and also floating debris"); every roof consumer downstream
    reads `m.get("deck_z", m["top"])` so the kit path (whose masses carry no
    `deck_z`) is untouched.

    MEASUREMENT, when `mesh` (`gac_storey_slice.read_mesh`'s de-indexed dict:
    `P` per face-corner, `tris` = `arange(len(P)).reshape(-1, 3)`) is given:
    among its upward-facing triangles (normal z > 0.8) whose centroid falls
    in the top 15% of the building height, bin z by triangle AREA in 0.2 m
    steps.

    THE GLOBAL BIGGEST BIN IS THE WRONG BIN. A GAC mesh's "fake interior"
    carries a visible floor plate at EVERY storey, so the top-15% window on a
    tall building holds several near-identical big-area bins spaced one
    storey apart, not just one (measured, `tools/gac_deck_probe.py`
    diagnostics, 2026-08-30: SM_Building_12 repeats a ~1724 m2 slab bin every
    ~4.6 m for four storeys inside the window). Taking the single largest bin
    in the whole window is a coin flip across those repeats -- it landed on
    one two storeys down and reported a 14.9 m parapet, obvious nonsense for
    a coping. What is reliable: walking DOWN from the bbox top, the first bin
    that is actually roof-sized -- area >= 20% of the plan footprint (`W *
    D`) -- IS the deck, because a thin coping/parapet ring (perimeter x wall
    thickness) never gets close to that fraction of the full plan. Bins
    thinner than that, from the top down to the deck bin, are the parapet.
    Falls back to the single largest bin in the window if nothing clears the
    threshold (an irregular roof with no flat majority), and to `top` itself
    when `mesh` is None or nothing in it qualifies at all; `deck_note`
    records which case fired, for `tools/gac_deck_probe.py`.
    """
    (x0, y0, z0), (x1, y1, z1) = bbox
    floors = list(g.get("storeys") or [])
    levels = sorted(z for z in floors if z0 - 1e-6 <= z < z1 - 0.5)
    if not levels or levels[0] > z0 + 0.5:
        levels = [z0] + levels
    deck_z, deck_note = float(z1), "no mesh given, falling back to top"
    P = mesh.get("P") if mesh is not None else None
    tris = mesh.get("tris") if mesh is not None else None
    if P is not None and tris is not None and len(tris):
        V0, V1, V2 = P[tris[:, 0]], P[tris[:, 1]], P[tris[:, 2]]
        cross = np.cross(V1 - V0, V2 - V0)
        ln = np.linalg.norm(cross, axis=1)
        ok = ln > 1e-9
        nz = np.zeros(len(tris))
        nz[ok] = cross[ok, 2] / ln[ok]
        area = 0.5 * ln
        zc = (V0[:, 2] + V1[:, 2] + V2[:, 2]) / 3.0
        H = float(z1 - z0)
        band = float(z1) - 0.15 * H
        up = ok & (nz > 0.8) & (zc >= band)
        if up.any():
            zc_u, area_u = zc[up], area[up]
            bw = 0.2
            b0 = float(zc_u.min())
            idx = np.floor((zc_u - b0) / bw).astype(int)
            nbin = int(idx.max()) + 1
            bin_area = np.zeros(nbin)
            bin_zw = np.zeros(nbin)
            np.add.at(bin_area, idx, area_u)
            np.add.at(bin_zw, idx, area_u * zc_u)
            nz_bins = np.nonzero(bin_area > 1e-6)[0]
            if len(nz_bins):
                thresh = 0.2 * float(x1 - x0) * float(y1 - y0)
                order = nz_bins[::-1]          # highest z first
                deck_bin = None
                for b in order:
                    if bin_area[b] >= thresh:
                        deck_bin = int(b)
                        break
                top_bin = int(order[0])
                if deck_bin is None:
                    # NOTHING IN THE WINDOW READS AS A FULL PLATEAU -- an
                    # irregular roof (a lantern, a sawtooth). Fall back to
                    # the old rule (biggest bin in the window) rather than
                    # leave `deck_z` at the bbox top.
                    deck_bin = int(nz_bins[np.argmax(bin_area[nz_bins])])
                    deck_z = float(bin_zw[deck_bin] / bin_area[deck_bin])
                    deck_note = ("no bin reached the {0:.0f} m2 plateau "
                                "threshold (20% of the {1:.0f} x {2:.0f} m "
                                "footprint); used the largest available bin "
                                "instead".format(thresh, x1 - x0, y1 - y0))
                elif deck_bin == top_bin:
                    deck_z = float(bin_zw[deck_bin] / bin_area[deck_bin])
                    deck_note = "no separate parapet -- top bin is already a full plateau"
                else:
                    deck_z = float(bin_zw[deck_bin] / bin_area[deck_bin])
                    par_z = float(bin_zw[top_bin] / bin_area[top_bin])
                    deck_note = ("parapet coping excluded (its bin at "
                                "z={0:.2f}, {1:.1f} m2 < {2:.0f} m2 "
                                "threshold)".format(par_z, bin_area[top_bin],
                                                    thresh))
            else:
                deck_note = "no area in top-15% band after binning (shouldn't happen)"
        else:
            deck_note = "no upward face (normal z > 0.8) in top 15% of height"
    elif mesh is not None:
        deck_note = "mesh had no usable P/tris"
    return {"tag": "main", "cx": 0.5 * (x0 + x1), "cy": 0.5 * (y0 + y1),
            "yaw": 0.0, "W": float(x1 - x0), "D": float(y1 - y0),
            "z0": float(z0), "top": float(z1), "deck_z": deck_z,
            "deck_note": deck_note, "levels": levels,
            "module": float(((g.get("bays") or {}).get("E") or {}).get("pitch")
                            or 4.0),
            "spec": {"bands": []}}


def side_frame(m, side, planes=None):
    """A `quake_flow._piece_frame`-shaped wall frame spanning a whole
    elevation, so `_b_face_pt(fr, u, v, out)` lands on that façade plane.
    `u` runs the way `soot_plume.side_u` counts it.

    `planes`, if given, is the `{side: plane_coord}` dict `window_rects`
    fills — the measured façade plane, in place of the mass BBOX face (which
    is the outer extent of whatever protrudes furthest and can be 1-3 m
    proud of the real wall). Only the frame origin's OUTWARD coordinate
    (x for E/W, y for S/N) moves; the `u`-origin coordinate and the yaw are
    unchanged, so `soot_plume.side_u`'s convention still holds."""
    W, D, cx, cy = m["W"], m["D"], m["cx"], m["cy"]
    H = float(m["top"] - m["z0"])
    pl = planes or {}
    if side == "S":
        return (cx - W / 2.0, pl.get("S", cy - D / 2.0), 0.0, W, H, -0.02, False)
    if side == "E":
        return (pl.get("E", cx + W / 2.0), cy - D / 2.0, math.pi / 2.0, D, H, -0.02, False)
    if side == "N":
        return (cx + W / 2.0, pl.get("N", cy + D / 2.0), math.pi, W, H, -0.02, False)
    return (pl.get("W", cx - W / 2.0), cy + D / 2.0, 1.5 * math.pi, D, H, -0.02, False)


def openings_provider(rects, m, planes=None):
    """`(ctx, mass, side, storey) -> [opening records]` over the measured
    window islands: each record carries `span` (u0, u1, z_sill, z_head) in
    `soot_plume.side_u`'s convention plus the fields `_flame_sources` reads.

    `planes`, if given, is `window_rects`' measured `{side: plane_coord}`
    dict, forwarded to `side_frame` so every opening's `fr` sits on the real
    façade plane instead of the mass bbox face."""
    W, D, cx, cy = m["W"], m["D"], m["cx"], m["cy"]
    levels = list(m["levels"])
    frames = {s: side_frame(m, s, planes) for s in ("S", "E", "N", "W")}
    by = {}
    for side, rl in rects.items():
        for (u0, u1, z0, z1) in rl:
            if side == "S":
                a, b = u0 - (cx - W / 2.0), u1 - (cx - W / 2.0)
            elif side == "E":
                a, b = u0 - (cy - D / 2.0), u1 - (cy - D / 2.0)
            elif side == "N":
                a, b = (cx + W / 2.0) - u1, (cx + W / 2.0) - u0
            else:
                a, b = (cy + D / 2.0) - u1, (cy + D / 2.0) - u0
            zc = 0.5 * (z0 + z1)
            st = 0
            for i, lv in enumerate(levels):
                if zc >= lv - 0.05:
                    st = i
            e = {"mass": "main", "x": 0.5 * (u0 + u1), "y": zc, "z": z0,
                 "storey": st, "side": side, "role": "wall",
                 "name": "gac_window", "p": {}, "dead": False}
            by.setdefault((side, st), []).append({
                "fr": frames[side], "ua": a, "ub": b, "va": z0, "vb": z1,
                "hua": a, "hub": b, "hva": z0, "hvb": z1, "out": -0.05,
                "e": e, "m": m, "side": side, "storey": st, "mass": "main",
                "span": (min(a, b), max(a, b), float(z0), float(z1))})

    def provider(ctx, mtag, side, storey):
        return list(by.get((side, storey), []))

    provider.count = sum(len(v) for v in by.values())
    return provider


# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------
def _diffuse_of(mat_prim):
    """(shader path, input name, texture url) of the map feeding a
    UsdPreviewSurface's diffuseColor — the GAC materials' layout — falling
    back to `soot_plume.find_basecolor`'s name heuristics."""
    from pxr import Sdf, Usd, UsdShade
    from . import soot_plume as spl

    if not mat_prim or not mat_prim.IsValid():
        return None, None, None
    for c in Usd.PrimRange(mat_prim):
        sh = UsdShade.Shader(c)
        if not sh or sh.GetIdAttr().Get() != "UsdPreviewSurface":
            continue
        d = sh.GetInput("diffuseColor")
        if d is not None and d.HasConnectedSource():
            ts = UsdShade.Shader(d.GetConnectedSource()[0].GetPrim())
            f = ts.GetInput("file") if ts else None
            v = f.Get() if f else None
            if isinstance(v, Sdf.AssetPath) and (v.resolvedPath or v.path):
                return ts.GetPrim().GetPath(), "file", (v.resolvedPath or v.path)
        break
    return spl.find_basecolor(mat_prim)


def _sample_skin_any_side(sk, m, pts):
    """Skin RGBA at cell-frame points on ANY elevation: each point's side is
    the wall line it is nearest, then `soot_plume.side_u` on that side."""
    from . import soot_bake as sb

    W, D = float(m["W"]), float(m["D"])
    lx = pts[:, 0] - float(m["cx"])
    ly = pts[:, 1] - float(m["cy"])
    dist = np.stack([np.abs(ly + D / 2.0), np.abs(lx - W / 2.0),
                     np.abs(ly - D / 2.0), np.abs(lx + W / 2.0)], axis=1)
    side_ix = np.argmin(dist, axis=1)            # 0 S, 1 E, 2 N, 3 W
    out = np.zeros((len(pts), 4), dtype=np.float32)
    for k, side in enumerate(("S", "E", "N", "W")):
        sel = side_ix == k
        if not sel.any():
            continue
        out[sel] = sb.sample_skin(sk, side, m, pts[sel])
    return out


def bake_atlases(stage, cell, mesh, sk, m, out_dir, verbose=True):
    """Bake the skin into every textured material of the merged mesh, once
    per texture, through the mesh's own UVs. Returns
    `{key: UsdShade.Material}` where `key` is both the source material's
    prim path AND its texture url (so pieces can be rebound by either)."""
    from PIL import Image
    from . import soot_bake as sb, soot_plume as spl

    P, UV, MID, mats = mesh["P"], mesh["UV"], mesh["MID"], mesh["mats"]
    n_tri = len(MID)
    counts = np.full(n_tri, 3, dtype=np.int64)
    indices = np.arange(3 * n_tri, dtype=np.int64)
    os.makedirs(out_dir, exist_ok=True)
    sooted, by_tex, stats = {}, {}, {"baked": 0, "clean": 0, "notex": 0}
    for k, mat in enumerate(mats):
        if mat is None:
            continue
        mp = mat.GetPrim()
        sh_path, inp, tex = _diffuse_of(mp)
        if not tex:
            stats["notex"] += 1
            continue
        if tex in by_tex:
            sooted[str(mp.GetPath())] = by_tex[tex]
            sooted["_png"][str(mp.GetPath())] = sooted["_png"][tex]
            continue
        face_ids = np.nonzero(MID == k)[0]
        if len(face_ids) == 0:
            continue
        base = spl._read_rgb(tex, max_px=BAKE_PX_MAX)
        if base is None:
            stats["notex"] += 1
            continue
        px = int(max(BAKE_PX_MIN, min(BAKE_PX_MAX, max(base.shape[0], base.shape[1]))))
        pos, mask = sb.uv_position_map(P, counts, indices, UV, "vertex", None,
                                       face_ids=face_ids.tolist(), px=px)
        if not mask.any():
            continue
        # IS THIS ATLAS TILED UP THE BUILDING? Rasterise the same faces in
        # the opposite order: a texel that two faces at different heights
        # both cover comes back with two different positions. Cheap (a
        # second pass at a quarter of the resolution) and decisive.
        pq = max(256, px // 4)
        pa, ma = sb.uv_position_map(P, counts, indices, UV, "vertex", None,
                                    face_ids=face_ids.tolist(), px=pq)
        pb, mb = sb.uv_position_map(P, counts, indices, UV, "vertex", None,
                                    face_ids=face_ids[::-1].tolist(), px=pq)
        both = ma & mb
        shared = 0.0
        if both.any():
            dz = np.abs(pa[both][:, 2] - pb[both][:, 2])
            shared = float((dz > SHARED_TEXEL_M).mean())
        if shared > SHARED_FRAC_MAX:
            stats["tiled"] = stats.get("tiled", 0) + 1
            sooted.setdefault("_tiled", set()).add(tex)
            sooted["_tiled"].add(str(mp.GetPath()))
            if verbose:
                print("[gac_fire]   atlas {0}: TILED ({1:.0%} of its texels serve "
                      "faces >{2:.0f} m apart) -> baked per piece after the "
                      "slice".format(tex.rsplit("/", 1)[-1], shared,
                                     SHARED_TEXEL_M))
            continue
        pts = pos[mask].astype(np.float64)
        rgba = _sample_skin_any_side(sk, m, pts)
        if float(rgba[:, 3].max()) < 0.03:
            stats["clean"] += 1
            continue
        base = np.asarray(base, dtype=np.float32)
        if base.ndim == 2:
            base = np.repeat(base[..., None], 3, axis=2)
        base = base[..., :3]
        byi = np.linspace(0, base.shape[0] - 1, px).astype(int)
        bxi = np.linspace(0, base.shape[1] - 1, px).astype(int)
        out = base[byi][:, bxi].copy()
        a = rgba[:, 3:4]
        b = out[mask]
        grey = b.mean(axis=1, keepdims=True)
        desat = b * (1.0 - spl.DESAT * a) + grey * (spl.DESAT * a)
        out[mask] = np.clip(desat * (1.0 - a) + rgba[:, :3] * a, 0.0, 1.0)
        digest = hashlib.md5(np.round(out * 255.0).astype(np.uint8).tobytes()
                             ).hexdigest()[:16]
        png = os.path.join(out_dir, "gacsoot_{0}.png".format(digest))
        if not os.path.exists(png):
            Image.fromarray((np.clip(out, 0, 1) * 255.0 + 0.5)
                            .astype(np.uint8)).save(png)
        mpath = "{0}/SootLooks/m{1}".format(cell, len(by_tex))
        new = spl.piece_material_like(stage, mpath, mp, sh_path, inp, png)
        if new is None:
            new = spl.piece_material(stage, mpath, png)
        by_tex[tex] = new
        sooted[str(mp.GetPath())] = new
        sooted[tex] = new
        sooted.setdefault("_png", {})[tex] = png
        sooted["_png"][str(mp.GetPath())] = png
        stats["baked"] += 1
        if verbose:
            print("[gac_fire]   atlas {0}: {1} tri(s), {2} px, soot on "
                  "{3:.0%} of its texels -> {4}".format(
                      tex.rsplit("/", 1)[-1], len(face_ids), px,
                      float((rgba[:, 3] > 0.1).mean()), os.path.basename(png)))
    if verbose:
        print("[gac_fire] {0} atlas(es) baked, {1} untouched by the fire, "
              "{2} tiled (left to the per-piece bake), {3} material(s) "
              "without a diffuse map".format(
                  stats["baked"], stats["clean"], stats.get("tiled", 0),
                  stats["notex"]))
    return sooted


def rebind_sooted(stage, pls, sooted):
    """Every piece subset bound to a material that has a sooted copy is
    rebound to the copy (matched by material path, then by texture url —
    a baked kit's rehomed materials share the texture, not the path)."""
    from pxr import Usd, UsdGeom, UsdShade

    n = 0
    for p in pls:
        prim = stage.GetPrimAtPath(p["prim_path"]) if p.get("prim_path") else None
        if not prim or not prim.IsValid():
            continue
        for mesh in Usd.PrimRange(prim):
            if not mesh.IsA(UsdGeom.Mesh):
                continue
            subs = list(UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(mesh)))
            for t in ([s.GetPrim() for s in subs] or [mesh]):
                cur = UsdShade.MaterialBindingAPI(t).ComputeBoundMaterial()[0]
                if not cur or not cur.GetPrim().IsValid():
                    continue
                new = sooted.get(str(cur.GetPrim().GetPath()))
                if new is None:
                    _sp, _inp, tex = _diffuse_of(cur.GetPrim())
                    new = sooted.get(tex) if tex else None
                if new is None or isinstance(new, dict):
                    continue
                UsdShade.MaterialBindingAPI(t).Bind(new)
                n += 1
    return n


def uf_side_neighbours(side):
    ring = ("S", "E", "N", "W")
    i = ring.index(side) if side in ring else 0
    return (ring[i - 1], ring[(i + 1) % 4])


_SIDES4 = ("S", "E", "N", "W")


def _piece_sides(side_tag):
    """Every cardinal side of `_SIDES4` a piece's `_side` tag touches.

    MEASURED (`tools/gac_burn_probe.py`'s window census, 2026-08-30), not
    assumed: `gac_storey_slice.as_placements` writes THREE different shapes
    into `p["_side"]`, and the first pass here only handled one of them —
    every corner and every merged-band piece on SM_Building_03 F1 kept its
    plain glazing untouched, all the way through the fire's own origin
    storey, because none of them ever passed the old `side.split("x")` test:

      * a plain run: one of `_SIDES4` already ("S", "E", "N", "W");
      * a RING corner (`_cut`'s own `("SW", ...), ("SE", ...), ("NW", ...),
        ("NE", ...)`): two compass letters, no separator at all — the
        `_darken_glass_legacy` comment's "'SxE'" describes a DIFFERENT
        corner convention (`urban_building`'s kit pieces) that never
        applied to a GAC region cut;
      * a MERGED band (`slice_to_kit`'s `merged_lower`/`merged_upper`,
        everything below the fire's origin or above its `region["top"]`
        collapsed to one piece): the literal string `"x"` — the WHOLE
        perimeter in one mesh, touching every side at once.

    A tag this function does not recognise is treated the same as the
    merged-band case (every side) rather than excluded — the OUTER filter
    this feeds is only ever a cheap skip ahead of the real, per-FACE side
    test (nearest wall line, inside `damage_windows`'s own loop), so the
    failure mode of guessing too WIDE here is a few extra faces checked and
    discarded; guessing too NARROW is a window silently left untreated,
    which is the bug this function exists to stop happening again."""
    t = str(side_tag or "")
    if t in _SIDES4:
        return {t}
    parts = [c for c in t.replace("-", "x").split("x") if c]
    hit = set(c for c in parts if c in _SIDES4) if len(parts) > 1 else set()
    if not hit:
        hit = set(c for c in t if c in _SIDES4)   # compass corner, e.g. "SW"
    return hit or set(_SIDES4)                     # unrecognised -> every side


def _storey_of(z, levels):
    """Storey index of world height `z` on a `levels` list (storey i's floor
    at `levels[i]`) — the convention `openings_provider` counts by."""
    st = 0
    for i, lv in enumerate(levels):
        if z >= lv - 0.05:
            st = i
    return st


def _match_island(side, u, z, rects, pad=0.15):
    """The window ISLAND (u0, u1, z0, z1) on `side` that (u, z) falls in —
    `pad` metres of slack for a face centroid that sits a little inside the
    frame rather than dead centre on the glass — or None."""
    for box in (rects or {}).get(side, ()):
        u0, u1, z0, z1 = box
        if u0 - pad <= u <= u1 + pad and z0 - pad <= z <= z1 + pad:
            return box
    return None


def _window_materials(stage, cell):
    """The two GAC-only window looks `damage_windows` binds.

    Both use the `enable_opacity` / `opacity_threshold=0` / `opacity_mode=0`
    OmniPBR recipe `urban_fire.materials`'s glass-deposit bands already use:
    a genuine per-pixel cutout on a bench that passes `--/rtx/raytracing/
    fractionalCutoutOpacity` (`gac_fire_bench_launch_script.KIT_ARGS` does),
    a binary cutout — opaque at 1, a real hole at exactly 0, no fractional
    blend — everywhere else (`wall_overlay`'s own note on the same flag).
    `burnt` is authored at exactly 0 for that reason: a fully burnt-out pane
    is meant to read as a hole on EITHER renderer state, not only the one
    with the flag on."""
    from pxr import Sdf, UsdShade
    from . import damage as dmg

    scope = cell + "/GacWinLooks"

    def _mk(name, rgb, rough, opacity):
        path = scope + "/" + name
        m = UsdShade.Material.Get(stage, path)
        if m and m.GetPrim().IsValid():
            return m
        m = dmg._pbr(stage, path, rgb, rough)
        sh = UsdShade.Shader.Get(stage, m.GetPath().AppendChild("Shader"))
        if sh:
            sh.CreateInput("enable_opacity", Sdf.ValueTypeNames.Bool).Set(True)
            sh.CreateInput("opacity_constant",
                          Sdf.ValueTypeNames.Float).Set(float(opacity))
            sh.CreateInput("opacity_threshold",
                          Sdf.ValueTypeNames.Float).Set(0.0)
            sh.CreateInput("opacity_mode", Sdf.ValueTypeNames.Int).Set(0)
        return m

    return {
        # fully cut: a burnt-out pane is a hole, not a dark pane
        "burnt": _mk("burnt", (0.008, 0.007, 0.006), 0.90, 0.0),
        # sooted, cracked, still glazed: dark and half gone, not a hole
        "crazed": _mk("crazed", (0.026, 0.029, 0.029), 0.78, 0.6),
    }


def _darken_glass_legacy(stage, ctx, pls, sooted=None, glass_tex=None):
    """`darken_glass`'s ORIGINAL behaviour, kept verbatim for any call site
    that has not been updated to pass `damage_windows` the measured window
    islands (`rects`/`mass`): the whole glass subset of every piece on the
    burning storeys/sides -> the void tone, with no per-window split."""
    from detail import gac_slice as gsl
    from pxr import Usd, UsdGeom, UsdShade

    glass_tex = glass_tex or gsl.GLASS_TEX
    band = set(int(s) for s in ctx["fire"]["storeys"])
    hot = set(ctx["fire"]["sides"])
    for sd in list(hot):
        hot.update(uf_side_neighbours(sd))
    mat = ctx["mats"]["void"]
    back = {}
    for k, v in ((sooted or {}).get("_png") or {}).items():
        if "/" in k and not k.startswith("/World") and "://" in k:
            back[v] = k          # baked png -> original texture url
    # ...and the per-piece copies `urban_fire._bind_soot` made for the tiled
    # atlases: `ctx["soot_mats"]` maps (original material path, png) -> copy
    orig_of = {}
    for key, cm in (ctx.get("soot_mats") or {}).items():
        try:
            orig_of[str(cm.GetPrim().GetPath())] = key[0]
        except Exception:
            continue
    n = 0
    for p in pls:
        if p.get("_storey") not in band or p.get("dead"):
            continue
        side = str(p.get("_side", ""))
        # a corner piece is named for both of its sides ("SxE"); a run for one
        if not any(sd in ctx["fire"]["sides"] for sd in side.split("x")):
            continue
        prim = stage.GetPrimAtPath(p["prim_path"]) if p.get("prim_path") else None
        if not prim or not prim.IsValid() or not prim.IsActive():
            continue
        for mesh in Usd.PrimRange(prim):
            if not mesh.IsA(UsdGeom.Mesh):
                continue
            for s in UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(mesh)):
                cur = UsdShade.MaterialBindingAPI(s.GetPrim()).ComputeBoundMaterial()[0]
                if not cur or not cur.GetPrim().IsValid():
                    continue
                cpath = str(cur.GetPrim().GetPath())
                if cpath in orig_of:
                    op = stage.GetPrimAtPath(orig_of[cpath])
                    _sp, _inp, tex = _diffuse_of(op) if op and op.IsValid() else (None, None, None)
                else:
                    _sp, _inp, tex = _diffuse_of(cur.GetPrim())
                tex = back.get(tex, tex)
                if tex and gsl.is_glazing(tex, glass_tex):
                    UsdShade.MaterialBindingAPI(s.GetPrim()).Bind(mat)
                    n += 1
    return n


def damage_windows(stage, ctx, pls, rects=None, mass=None, sooted=None,
                   glass_tex=None):
    """Per-WINDOW glazing treatment on the burning elevations of a sliced
    GAC building — a burnt-out pane is now a real hole, not a blacked-out
    subset (`darken_glass`'s old, whole-subset behaviour: see
    `_darken_glass_legacy`, still reachable with the old call signature).

    Three states, decided per WINDOW ISLAND (`rects`, `window_rects`'
    measured (u0, u1, z0, z1) boxes, matched to each face by its own
    centroid) rather than per material subset — a subset is typically ONE
    material for a whole elevation, and a corner piece's single "glass"
    subset covers windows on BOTH of its faces, so binding the whole subset
    one way put a hot-side window's treatment on its cold-side neighbour too
    (the exact "rectangles" / "random black windows on a cold elevation"
    failure `darken_glass` already had to stop once, 2026-08-30):

      * BURNT OUT (`mats["burnt"]`, opacity 0, a real hole) — a hot-side
        window whose storey is in the fire's own band.
      * CRAZED / SMOKED (`mats["crazed"]`, opacity ~0.6, dark, still glazed)
        — a hot-side window one storey above the band, or any window in the
        band or one above it on a side that NEIGHBOURS a hot one (the same
        corner bleed `uf_side_neighbours` already modelled for the old
        whole-subset bind).
      * left alone everywhere else: whatever the sooted-atlas bake already
        bound it to, or the asset's own glass.

    A subset that ends up split has its face list SHRUNK to the faces that
    keep their existing binding, and one new partition-family GeomSubset
    per treatment is added alongside it with the reassigned faces — faces
    are exclusive to one subset, so the shrink is what keeps the partition
    honest.

    `rects`/`mass` are `prepare()`'s own `pre["rects"]`/`pre["mass"]` — the
    caller passes them explicitly because `ctx` does not carry them yet
    (`ctx["gac"]` is only assembled by `burn_gac` AFTER this returns).
    Either omitted (any call site still on `darken_glass`'s original
    signature) falls back to `_darken_glass_legacy`, unchanged."""
    if rects is None or mass is None:
        return _darken_glass_legacy(stage, ctx, pls, sooted=sooted,
                                    glass_tex=glass_tex)

    from detail import gac_slice as gsl
    from pxr import Usd, UsdGeom, UsdShade, Vt

    glass_tex = glass_tex or gsl.GLASS_TEX
    fire = ctx["fire"]
    W, D = float(mass["W"]), float(mass["D"])
    cx, cy = float(mass["cx"]), float(mass["cy"])
    levels = list(mass["levels"])
    n_st = len(levels)
    band = set(int(s) for s in fire["storeys"])
    top_band = int(fire["top"])
    above_st = top_band + 1 if top_band + 1 < n_st else None
    hot_sides = set(fire["sides"])
    bleed_sides = set()
    for sd in hot_sides:
        bleed_sides.update(uf_side_neighbours(sd))
    bleed_sides -= hot_sides
    relevant_sides = hot_sides | bleed_sides

    mats = _window_materials(stage, ctx["parent"])
    back = {}
    for k, v in ((sooted or {}).get("_png") or {}).items():
        if "/" in k and not k.startswith("/World") and "://" in k:
            back[v] = k
    orig_of = {}
    for key, cm in (ctx.get("soot_mats") or {}).items():
        try:
            orig_of[str(cm.GetPrim().GetPath())] = key[0]
        except Exception:
            continue

    n_burnt = n_crazed = 0
    for p in pls:
        if p.get("dead"):
            continue
        sides_here = _piece_sides(p.get("_side", ""))
        if not (sides_here & relevant_sides):
            continue
        prim = stage.GetPrimAtPath(p["prim_path"]) if p.get("prim_path") else None
        if not prim or not prim.IsValid() or not prim.IsActive():
            continue
        for meshp in Usd.PrimRange(prim):
            if not meshp.IsA(UsdGeom.Mesh):
                continue
            me = UsdGeom.Mesh(meshp)
            pts = me.GetPointsAttr().Get()
            if not pts:
                continue
            P = np.asarray(pts, dtype=np.float64)
            counts = np.asarray(me.GetFaceVertexCountsAttr().Get() or [],
                                dtype=np.int64)
            fvi = np.asarray(me.GetFaceVertexIndicesAttr().Get() or [],
                             dtype=np.int64)
            if not len(counts) or len(fvi) != int(counts.sum()):
                continue
            start = np.concatenate([[0], np.cumsum(counts)[:-1]])
            for sub in list(UsdGeom.Subset.GetAllGeomSubsets(
                    UsdGeom.Imageable(meshp))):
                cur = UsdShade.MaterialBindingAPI(
                    sub.GetPrim()).ComputeBoundMaterial()[0]
                if not cur or not cur.GetPrim().IsValid():
                    continue
                cpath = str(cur.GetPrim().GetPath())
                if cpath in orig_of:
                    op = stage.GetPrimAtPath(orig_of[cpath])
                    _sp, _inp, tex = (_diffuse_of(op)
                                     if op and op.IsValid() else
                                     (None, None, None))
                else:
                    _sp, _inp, tex = _diffuse_of(cur.GetPrim())
                tex = back.get(tex, tex)
                if not tex or not gsl.is_glazing(tex, glass_tex):
                    continue
                idx = [int(f) for f in (sub.GetIndicesAttr().Get() or [])]
                if not idx:
                    continue
                burnt_f, crazed_f, keep_f = [], [], []
                for f in idx:
                    if f >= len(counts):
                        keep_f.append(f)
                        continue
                    vids = fvi[start[f]:start[f] + counts[f]]
                    cen = P[vids].mean(axis=0)
                    dist = (abs(cen[1] - cy + D / 2.0),
                           abs(cen[0] - cx - W / 2.0),
                           abs(cen[1] - cy - D / 2.0),
                           abs(cen[0] - cx + W / 2.0))
                    side = _SIDES4[int(np.argmin(dist))]
                    if side not in relevant_sides:
                        keep_f.append(f)
                        continue
                    u = cen[0] if side in ("S", "N") else cen[1]
                    z = float(cen[2])
                    island = _match_island(side, u, z, rects)
                    st = _storey_of(0.5 * (island[2] + island[3])
                                    if island else z, levels)
                    hot = side in hot_sides
                    bleed = side in bleed_sides
                    if hot and st in band:
                        burnt_f.append(f)
                    elif ((hot and above_st is not None and st == above_st)
                         or (bleed and (st in band or st == above_st))):
                        crazed_f.append(f)
                    else:
                        keep_f.append(f)
                if not burnt_f and not crazed_f:
                    continue
                if keep_f:
                    sub.GetIndicesAttr().Set(Vt.IntArray(keep_f))
                else:
                    sub.GetPrim().SetActive(False)
                base_name = sub.GetPrim().GetName()
                if burnt_f:
                    bs = UsdGeom.Subset.CreateGeomSubset(
                        me, base_name + "_burnt", UsdGeom.Tokens.face,
                        Vt.IntArray(burnt_f), UsdShade.Tokens.materialBind,
                        UsdGeom.Tokens.partition)
                    UsdShade.MaterialBindingAPI.Apply(
                        bs.GetPrim()).Bind(mats["burnt"])
                    n_burnt += len(burnt_f)
                if crazed_f:
                    cs = UsdGeom.Subset.CreateGeomSubset(
                        me, base_name + "_crazed", UsdGeom.Tokens.face,
                        Vt.IntArray(crazed_f), UsdShade.Tokens.materialBind,
                        UsdGeom.Tokens.partition)
                    UsdShade.MaterialBindingAPI.Apply(
                        cs.GetPrim()).Bind(mats["crazed"])
                    n_crazed += len(crazed_f)
    ctx["notes"].append(
        "windows: {0} burnt out (see-through), {1} crazed/smoked -- "
        "per-window, not per-subset".format(n_burnt, n_crazed))
    return n_burnt + n_crazed


darken_glass = damage_windows


# ---------------------------------------------------------------------------
# The whole thing
# ---------------------------------------------------------------------------
def prepare(stage, cell, name, level, rng, tag, origin=None, sides=None,
            out_dir=None, verbose=True):
    """Everything up to (not including) the slice: place the asset, measure
    it, plan the fire and its events from the window islands, rasterise the
    skin, bake it into the atlases. Returns a dict the slice/burn tail and
    the offline tools (`tools/gac_fire_probe.py`, `tools/soot_elevation.py
    --gac`) both consume: src, grid, measured, rects, mass, info, btype,
    fire, provider, events, skin, mesh (None after the bake), sooted, planes
    (the measured per-side façade plane `window_rects` filled — see
    `side_frame`)."""
    from detail import gac_slice as gsl, gac_storey_slice as gss
    from . import soot_plume as spl, urban_fire as uf

    style = "gac_" + name
    src = place_source(stage, cell, GAC_DIR + name + ".usd", GAC_SCALE)
    if not src:
        raise RuntimeError("{0}: nothing composed".format(name))
    wins, bbox = gsl.window_centres(stage, src)
    g, measured = gss.grid_for(stage, src, bbox, wins, name=name, verbose=verbose)
    planes = {}
    rects = window_rects(stage, src, planes=planes)
    # THE MESH IS READ HERE, BEFORE THE MASS BOX, NOT AFTER. `mass_from_grid`
    # measures the real roof-deck height (`deck_z`) off this same array's
    # upward faces -- everything that sits "on the roof" downstream
    # (`dress_roof_urban`, `_deck_slab`, `_rafter_teeth`, `r_roof_scorch`)
    # reads `m.get("deck_z", m["top"])` in place of the bbox top, which is
    # the parapet coping and can be well over a metre proud of the real deck
    # (row-2 review, 2026-08-30: "floating roof props and also floating
    # debris"). The bake below still runs on this same `mesh`; nothing else
    # about it changes.
    mesh = gss.read_mesh(stage, src, verbose=False)
    m = mass_from_grid(g, bbox, mesh=mesh)
    n_st = len(m["levels"])
    btype = "urm" if m["top"] - m["z0"] <= 25.0 else "rc"
    info = {"style": style, "family": "01", "type": btype, "x": 0.0, "y": 0.0,
            "yaw": 0.0, "masses": {"main": m}, "elements": [],
            "H": m["top"] - m["z0"]}
    if origin is None:
        origin = max(0, min(n_st - 1, int(round(0.25 * (n_st - 1)))))
    if sides is None:
        # THE FIRE VENTS WHERE THE WINDOWS ARE. A GAC asset carries its
        # glazing on one or two elevations and blank party walls elsewhere
        # (`SM_Building_02`: 36 islands each on E and W, none on S/N), so a
        # side drawn at random is a blank wall half the time and the
        # building gets no events at all. Rank the elevations by island
        # count and take as many as the level's plan wants.
        ranked = sorted(rects.keys(), key=lambda sd: -len(rects[sd]))
        ranked = [sd for sd in ranked if rects[sd]] or ["S"]
        n_side = 1 if level in ("F1", "F2") else (2 if level == "F3" else
                                                  min(len(ranked), rng.randint(2, 4)))
        sides = tuple(ranked[:max(1, n_side)])
    fire = uf.plan_fire(info, level, rng, origin=origin, sides=sides)
    # `deck_z` HAS TO RIDE ON `fire`, NOT ON `m`. `burn_building` never sees
    # this `info`/`mass` dict again -- it rebuilds its own `ctx["info"]` from
    # `quake_flow.describe(style, placements, ...)`, which measures a fresh
    # mass box off the SLICED PIECES through `_mass_specs`/`register_style`
    # and knows nothing about `deck_z` (measured, `_roof_seat_probe.py`:
    # SM_Building_03 F1's `dress_roof_urban` read `m.get("deck_z", ...)` off
    # that rebuilt mass and silently fell back to `top` every time). `fire`,
    # by contrast, IS the same dict object `burn_building` copies verbatim
    # into `ctx["fire"]` (`fire=fire` all the way through `burn_gac`), so
    # stashing it here is the one channel that actually reaches the roof
    # recipes. The kit path never sets this key (`plan_fire`'s own return
    # never does), so `ctx["fire"].get("deck_z", m["top"])` there is
    # unchanged.
    fire["deck_z"] = m.get("deck_z")
    fire["deck_note"] = m.get("deck_note")
    provider = openings_provider(rects, m, planes=planes)
    ctx0 = {"info": info, "fire": fire, "rng": rng, "tag": tag,
            "soot_openings": provider}
    events = spl.plan_events(ctx0, uf._severity)
    heavy = 1.0
    for rname, kw in uf.LADDER.get(btype, {}).get(level, []):
        if rname == "smoke_stain":
            heavy = float((kw or {}).get("heavy", 1.0))
    sk = spl.skin(ctx0, events, np.random.default_rng(spl.event_seed(ctx0) ^ 0x5EED),
                  finish=fire.get("finish") or "char", glass=False,
                  duration_scale=heavy)
    if verbose:
        print("[gac_fire] {0} {1}: {2:.0f} x {3:.0f} x {4:.0f} m, {5} storey(s), "
              "{6} window island(s), origin st{7} band {8}-{9} on {10}, {11}"
              .format(name, level, m["W"], m["D"], m["top"] - m["z0"], n_st,
                      provider.count, fire["origin"], fire["storeys"][0],
                      fire["top"], "/".join(fire["sides"]),
                      spl.summarise(events)))
    sooted = {}
    if mesh is not None and events:
        sooted = bake_atlases(stage, cell, mesh, sk, m,
                              out_dir or spl.OUT_DIR, verbose=verbose)
    return {"name": name, "style": style, "src": src, "grid": g,
            "measured": measured, "rects": rects, "mass": m, "info": info,
            "btype": btype, "fire": fire, "provider": provider,
            "events": events, "skin": sk, "mesh": mesh, "sooted": sooted,
            "heavy": heavy, "planes": planes}


def burn_gac(stage, cell, name, level, rng, nrng, mats, tag, flow_root=None,
             mat_cache=None, ssf=1.0, origin=None, sides=None,
             use_baked_kit=True, out_dir=None, verbose=True):
    """Place GAC asset `name` under `cell`, plan its fire, bake its soot into
    its atlases, slice it, and run the ladder. Returns `burn_building`'s ctx
    with `ctx["gac"]` = {grid, events, n_pieces, n_atlases, n_rebound, ...}.

    `cell` must already exist (an Xform the caller positioned); everything
    is authored in the cell's frame, exactly as the kit benches do.
    """
    from detail import gac_slice as gsl, gac_storey_slice as gss, kit_bake as kb
    from detail import urban_building as ub
    from . import urban_fire as uf

    pre = prepare(stage, cell, name, level, rng, tag, origin=origin,
                  sides=sides, out_dir=out_dir, verbose=verbose)
    pre["mesh"] = None                      # the largest array; not needed now
    src, style, fire, events, provider, sooted = (
        pre["src"], pre["style"], pre["fire"], pre["events"],
        pre["provider"], pre["sooted"])
    # NAMES/N_ST, MOVED UP FROM THE FIT-OUT SECTION BELOW (which still uses
    # both). Needed here first: whether the recipe list can ever touch the
    # roof decides how far UP the region cut has to ring — see `region=`
    # below and `gac_storey_slice.plan_slice_budget`'s own `region["top"]`
    # docstring.
    names = set(n for n, _kw in uf.LADDER.get(pre["btype"], {}).get(level, []))
    n_st = len(pre["mass"]["levels"])
    # the kit
    if use_baked_kit and kb.have_kit(name):
        pls, g2, meas2 = kb.load_kit(stage, cell, name, ssf)
        if style not in ub.STYLES:
            gsl.register_style(g2, style, pieces_of=pls)
        from pxr import UsdGeom
        UsdGeom.Imageable(stage.GetPrimAtPath(src)).MakeInvisible()
    else:
        # RING ONLY AS FAR UP AS SOMETHING CAN REACH. `fire["top"]` is the
        # highest storey the fire itself involves; a recipe that needs the
        # real roof regardless (a burn-through hole, either collapse) still
        # gets it, so `_deck_slab`/`r_roof_burnthrough`/`r_fire_collapse`
        # never find a merged wall piece where they expect a `role="roof"`
        # one. Nothing else in the ladder looks past `fire["top"]`, so
        # everything above it — the parapet/roof band included — collapses
        # to one piece instead of being ringed for no reader.
        roof_needed = (bool(fire.get("roof"))
                      or bool(names & {"roof_burnthrough", "fire_collapse"}))
        top = (n_st - 1) if roof_needed else int(fire["top"])
        pls, g2, meas2 = gss.slice_to_kit(
            stage, src, cell, style, verbose=verbose,
            region={"origin": fire["origin"], "top": top,
                    "sides": fire["sides"]})
    n_rebound = rebind_sooted(stage, pls, sooted) if sooted else 0
    if verbose:
        print("[gac_fire] {0}: {1} piece(s), {2} subset(s) rebound to sooted "
              "atlases".format(name, len(pls), n_rebound))
    # every subset already on a sooted copy is done; the subsets of TILED
    # atlases (and any material the pre-bake could not read) still go through
    # the kit's per-piece bake, with this building's own skin
    prebaked = set(str(v.GetPrim().GetPath()) for k, v in sooted.items()
                   if k not in ("_png", "_tiled") and hasattr(v, "GetPrim"))
    # ONLY THE STOREYS SOMEBODY CAN SEE INTO GET A FIT-OUT. A GAC shell has
    # no openings the slicer can empty — its burnt windows go to an opaque
    # void tone — so props behind an intact façade are invisible and cost
    # build time and prims for nothing ("on the next launch I don't want
    # props that nobody can see", user 2026-08-30). The interior shows only
    # where the shell is opened: through a burnt-through roof (the top
    # three storeys, `burn_building`'s own rule) and where a collapse takes
    # the shell away (`fire_collapse`: the top two; `partial_collapse`: the
    # band on the lost elevation). `names`/`n_st` were computed above, before
    # the slice, because the region cut needed them first.
    fit_storeys = set()
    if fire.get("roof") and names & {"roof_burnthrough", "fire_collapse"}:
        fit_storeys |= set(range(max(0, n_st - 3), n_st))
    if "fire_collapse" in names:
        fit_storeys |= set(range(max(0, n_st - 2), n_st))
    if "partial_collapse" in names:
        # NOT `fire["storeys"]` — THE STOREYS THE COLLAPSE ACTUALLY OPENS.
        # The burning band runs from the fire's origin to the top of the
        # block (SM_Building_05 F5c: 15 storeys, band 4-18, 2,500+ static
        # fit-out prims), but `partial_collapse` only takes the shell away
        # from its OWN failure line up, and everything under that line is
        # behind an intact elevation exactly like the rest of the building —
        # the "props nobody can see" this whole block exists to avoid.
        # `plan_partial_collapse` is pure arithmetic on the same `fire` plan
        # and the same measured mass box, so the failure line can be worked
        # out HERE, before the slice, without authoring anything: `_kill`
        # walks `info["elements"]`, which `prepare` leaves EMPTY, so the
        # module-budget trim (the only step that can RAISE `s0`) cannot fire
        # and the line comes back at or below the one the recipe will use.
        # That makes this a superset of the storeys the recipe opens, never a
        # subset. Two storeys below the lowest lost one are kept as well: the
        # floor the wreckage lands on and the one under it are both in view
        # through the hole.
        from . import fire_collapse as fcol
        kw_pc = {}
        for _rn, _kw in uf.LADDER.get(pre["btype"], {}).get(level, []):
            if _rn == "partial_collapse":
                kw_pc = dict(_kw or {})
        try:
            _pc = fcol.plan_partial_collapse(
                {"info": pre["info"], "fire": fire, "tag": tag}, **kw_pc)
            _lo = max(0, int(_pc["s0"]) - 2)
        except Exception as exc:                          # pragma: no cover
            print("[gac_fire] partial-collapse fit-out plan failed ({0}); "
                  "falling back to the whole burning band".format(exc))
            _lo = min(int(st) for st in fire["storeys"])
        fit_storeys |= set(range(_lo, n_st))
    # THE BAND ITSELF, ON EVERY NON-COLLAPSE LEVEL, CAPPED AT FOUR STOREYS.
    # `damage_windows` now burns a hot-side band window out to a real HOLE
    # instead of an opaque void tone, so an F1-F4 building that never
    # reaches the roof and never collapses used to show that hole onto
    # NOTHING — `fit_storeys` stayed empty for every one of those levels,
    # which stripped `gut_interior`/`expose_interior` out of the recipe list
    # below even where `LADDER` calls for both (user review, 2026-08-30: the
    # floor rubble the MCE kit path has never reached a GAC building).
    # Capped, because `quake_flow.fit_interior`'s own per-storey cost (a
    # furniture pool AND, on an `rc` frame, a full column grid, both sized
    # off the WHOLE building footprint rather than one kit module) is not
    # cheap on a merged asset. The LOWEST four of the band, nearest
    # `origin`: a band tall enough to reach the roof is already covered up
    # there by the roof/collapse rules above, so this rule's job is the part
    # of the chimney those rules miss.
    if level in ("F1", "F2", "F3", "F4"):
        fit_storeys |= set(sorted(int(st) for st in fire["storeys"])[:4])
    # ...and with nothing open, `expose_interior`'s beams, catch floor and
    # floor rubble "behind the openings" are behind opaque glass too: run
    # the ladder without it (a recipe LIST is what `burn_building` takes in
    # place of a level name; the fire plan itself is handed in as `fire=`)
    recipes = list(uf.LADDER.get(pre["btype"], {}).get(level, []))
    if not fit_storeys:
        recipes = [(n, kw) for n, kw in recipes
                   if n not in ("expose_interior", "gut_interior")]
    elif level in ("F1", "F2", "F3", "F4"):
        # F1's OWN LADDER HAS NEITHER RECIPE (a kit F1 is smoke damage over
        # an intact façade — nothing to see behind it). The GAC F1 band now
        # sees behind its glazing too, so top them up here rather than in
        # the shared `LADDER`, which the kit path stays frozen against.
        have = set(n for n, _kw in recipes)
        if "gut_interior" not in have:
            recipes = recipes + [("gut_interior", {"frac": 0.4})]
        if "expose_interior" not in have:
            recipes = recipes + [("expose_interior", {})]
    ctx = uf.burn_building(stage, cell, style, pls, 0.0, 0.0, 0.0, recipes, rng,
                           nrng, mats, tag, flow_root=flow_root,
                           origin=fire["origin"], sides=fire["sides"],
                           mat_cache=mat_cache, events=events,
                           openings_fn=provider, soot_prebaked=prebaked,
                           fire=fire, skin=pre["skin"], fit_storeys=fit_storeys)
    n_glass = damage_windows(stage, ctx, pls, rects=pre["rects"],
                             mass=pre["mass"], sooted=sooted)
    n_atlas = len(set(id(v) for k, v in sooted.items() if k != "_png"))
    ctx["gac"] = {"grid": pre["grid"], "events": events, "n_pieces": len(pls),
                  "n_atlases": n_atlas, "n_rebound": n_rebound,
                  "n_glass": n_glass, "mass": pre["mass"], "rects": pre["rects"],
                  "skin": pre["skin"]}
    ctx["notes"].append("gac: {0} piece(s), {1} sooted atlas(es), {2} subset(s) "
                        "rebound, {3} window(s) burnt out / crazed on the band, "
                        "fit-out on {4} storey(s) ({5})"
                        .format(len(pls), n_atlas, n_rebound, n_glass,
                                len(fit_storeys),
                                "visible through the roof / collapse" if fit_storeys
                                else "nothing is open, none authored"))
    return ctx
