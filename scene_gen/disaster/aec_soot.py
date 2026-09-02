#!/usr/bin/env python
"""aec_soot.py — REAL soot on a part-addressable AEC building, through the
same pipeline MCE and GAC use: one physics-driven soot field for the whole
facade, then cut per part by where that part actually sits, composited into
that part's own base map through its OWN UVs, bound as that part's own look.

    import aec_soot
    aec_soot.soot_building(stage, "/World/b0", level="F3")

WHY THIS FILE EXISTS, AND WHAT IT REPLACES (user, 2026-09-02): *"Do what we
did for MCE. Create a pattern to go over the whole facade. Then assemble the
actual building pieces so they fit on the facade and then cut out the sooted
overlay and apply. We have an entire pipeline for realistic soot generation."*

An earlier version of this module darkened each part's texture by a per-height
factor. That is NOT the pipeline, and it failed in two ways the user named
immediately: ramping inside a TILING texture repeated the gradient on every
tile ("it's like a repeating pattern"), and blending toward a greyscale mean
desaturated the brick ("black and white instead of base colour"). Neither can
happen here, because the soot is sampled in WORLD space from one continuous
canvas and composited through each part's real UVs.

THE CHAIN (identical to `gac_fire.prepare`, different measurement source):

    Floors prims       -> storey grid          (the asset TELLS us its floors)
    Windows prims      -> per-elevation rects  (real openings, not islands
                          guessed from glass faces — this asset is
                          part-addressable, so the windows are named)
    mass_from_grid     -> quake_flow-shaped mass box
    openings_provider  -> opening records per (side, storey)
    plan_fire          -> which storeys/elevations burn, from where up
    plan_events        -> flame/smoulder/out per contiguous opening run
    skin               -> ONE unwrapped S|E|N|W soot canvas (EN 1991-1-2
                          flame, Heskestad plume, Riahi-Beyler deposition)
    uv_position_map +  -> per part: its texels' world positions, sampled
    bake_module           against that canvas, composited into ITS base map
    Bind                  as that part's own look

DE-INSTANCE FIRST. USD refuses edits inside an instance proxy ("authoring to
an instancing prototype is not allowed") and a bind onto a proxy silently
resolves to the prototype — which is why an earlier attempt reported "320
materials rebuilt" and changed nothing on screen.
"""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
for _p in (os.path.normpath(os.path.join(_HERE, "..", "tools")),
           os.path.normpath(os.path.join(_HERE, "..")), _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)

SOOT_OUT = os.environ.get("AEC_SOOT_OUT", "/isaac-sim/.cache/aec_soot_tex")
BAKE_PX = int(os.environ.get("AEC_SOOT_PX", "512"))

#: Geometry categories that carry facade soot. Interior-only categories are
#: skipped: soot on a ceiling nobody sees is texture memory for nothing.
FACADE_CATS = ("Structural_Framing", "Walls", "Roofs", "Doors", "Stairs",
               "Railings", "Generic_Models", "Specialty_Equipment")


def _cat_of(prim_path):
    parts = str(prim_path).split("/")
    if "Geometry" in parts:
        i = parts.index("Geometry")
        if i + 1 < len(parts):
            return parts[i + 1]
    return "?"


def _unit_of(prim_path):
    for q in str(prim_path).split("/"):
        if q.startswith("Reference_Brownstone02_"):
            return q
    return "?"


def measure(stage, root_path):
    """`(bbox, storey_z, window_rects)` read from the asset's OWN prims."""
    from pxr import Usd, UsdGeom, Gf
    root = stage.GetPrimAtPath(root_path)
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    whole = bc.ComputeWorldBound(root).ComputeAlignedRange()
    mn, mx = whole.GetMin(), whole.GetMax()
    cx, cy = 0.5 * (mn[0] + mx[0]), 0.5 * (mn[1] + mx[1])
    W, D = mx[0] - mn[0], mx[1] - mn[1]

    floors, wins = [], {"S": [], "E": [], "N": [], "W": []}
    for prim in Usd.PrimRange(root):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        cat = _cat_of(prim.GetPath())
        if cat not in ("Floors", "Windows"):
            continue
        r = bc.ComputeWorldBound(prim).ComputeAlignedRange()
        if r.IsEmpty():
            continue
        a, b = r.GetMin(), r.GetMax()
        if cat == "Floors":
            floors.append(0.5 * (a[2] + b[2]))
            continue
        # WHICH ELEVATION does this window face? The nearest wall line.
        d = {"S": abs(0.5 * (a[1] + b[1]) - mn[1]),
             "N": abs(mx[1] - 0.5 * (a[1] + b[1])),
             "W": abs(0.5 * (a[0] + b[0]) - mn[0]),
             "E": abs(mx[0] - 0.5 * (a[0] + b[0]))}
        side = min(d, key=d.get)
        u0, u1 = ((a[0] - cx, b[0] - cx) if side in ("S", "N")
                  else (a[1] - cy, b[1] - cy))
        wins[side].append((float(min(u0, u1)), float(max(u0, u1)),
                           float(a[2] - mn[2]), float(b[2] - mn[2])))
    # storeys: cluster floor heights to 0.5 m
    zs = sorted(set(round((z - mn[2]) * 2.0) / 2.0 for z in floors))
    keep = []
    for z in zs:
        if not keep or z - keep[-1] > 1.5:
            keep.append(z)
    return ((mn[0], mn[1], mn[2], mx[0], mx[1], mx[2]), keep, wins,
            (cx, cy, W, D))


def soot_building(stage, root_path, level="F3", origin_storey=None,
                  sides=None, seed=7, verbose=True):
    """Soot every facade part under `root_path` from ONE physics soot field.

    Returns `(parts_bound, looks_made, n_events)`.
    """
    import numpy as np
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade

    import soot_plume as spl
    import soot_bake as sbk
    import urban_fire as uf
    import gac_fire as gf
    import mdl_to_preview as m2p

    root = stage.GetPrimAtPath(root_path)
    if not root or not root.IsValid():
        return (0, 0, 0)

    un = 0
    for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
        if prim.IsInstanceable():
            prim.SetInstanceable(False)
            un += 1

    bbox, storey_z, wins, (cx, cy, W, D) = measure(stage, root_path)
    n_st = max(2, len(storey_z))
    z0, z1 = bbox[2], bbox[5]
    m = {"tag": "main", "x": cx, "y": cy, "W": W, "D": D, "yaw": 0.0,
         "base": z0, "top": z1, "deck_z": z1,
         "levels": [z0 + z for z in storey_z] or [z0],
         "n": n_st, "spec": {}}
    info = {"masses": {"main": m}, "type": "urm", "elements": [],
            "n_storeys": n_st}
    if origin_storey is None:
        origin_storey = max(0, int(round(0.25 * (n_st - 1))))
    if sides is None:
        sides = [s for s in ("S", "E", "N", "W") if wins.get(s)][:2] or ["S"]
    fire = {"mass": "main", "origin": int(origin_storey), "sides": list(sides),
            "storeys": list(range(int(origin_storey), n_st)),
            "top": n_st - 1, "level": level, "n_storeys": n_st,
            "state": uf.ACTIVE.get(level, "flame"),
            "finish": uf.FINISH.get(level, "char"),
            "deck_z": z1, "roof": False}

    provider = gf.openings_provider(wins, m)
    ctx0 = {"info": info, "fire": fire, "rng": __import__("random").Random(seed),
            "tag": "main", "soot_openings": provider}
    events = spl.plan_events(ctx0, uf._severity)
    if not events:
        if verbose:
            print("[aec_soot] NO EVENTS — no openings inside the burning band")
        return (0, 0, 0)
    nrng = np.random.default_rng(spl.event_seed(ctx0) ^ 0x5EED)
    sk = spl.skin(ctx0, events, nrng, finish=fire["finish"])
    if verbose:
        print("[aec_soot] skin: {0} event(s) over sides {1}, origin storey "
              "{2}/{3}".format(len(events), ",".join(sides), origin_storey,
                               n_st - 1))

    os.makedirs(SOOT_OUT, exist_ok=True)
    looks = stage.DefinePrim(Sdf.Path(root_path).AppendChild("SootLooks"),
                             "Scope")
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    posmap = {}
    bound = made = 0

    for prim in Usd.PrimRange(root):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        if _cat_of(prim.GetPath()) not in FACADE_CATS:
            continue
        r = bc.ComputeWorldBound(prim).ComputeAlignedRange()
        if r.IsEmpty():
            continue
        c = r.GetMidpoint()
        d = {"S": abs(c[1] - bbox[1]), "N": abs(bbox[4] - c[1]),
             "W": abs(c[0] - bbox[0]), "E": abs(bbox[3] - c[0])}
        side = min(d, key=d.get)
        if side not in sides:
            continue                    # only elevations that actually vent
        src_mat, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
        if not src_mat:
            continue
        tex = _textures_of(src_mat, m2p)
        if not tex.get("base"):
            continue
        me = UsdGeom.Mesh(prim)
        key = str(prim.GetPath())
        pm = posmap.get(key)
        if pm is None:
            try:
                pts = np.asarray(me.GetPointsAttr().Get(), dtype=np.float64)
                fvc = np.asarray(me.GetFaceVertexCountsAttr().Get())
                fvi = np.asarray(me.GetFaceVertexIndicesAttr().Get())
                uv = None
                for pv in UsdGeom.PrimvarsAPI(prim).GetPrimvars():
                    if str(pv.GetTypeName()) in ("texCoord2f[]", "float2[]"):
                        uv = np.asarray([(q[0], q[1]) for q in (pv.Get() or [])])
                        if len(uv):
                            break
                if uv is None or not len(uv):
                    posmap[key] = False
                    continue
                pm = sbk.uv_position_map(pts, fvc, fvi, uv, px=BAKE_PX)
                posmap[key] = pm
            except Exception:
                posmap[key] = False
                continue
        if pm is False:
            continue
        pos, mask = pm[0], pm[1]
        xform = np.asarray(
            UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
                Usd.TimeCode.Default()), dtype=np.float64)
        try:
            base_rgb = _load_rgb(tex["base"], np)
            out = sbk.bake_module(sk, side, m, xform, pos, mask, base_rgb,
                                  px=BAKE_PX)
        except Exception as exc:
            if verbose and made == 0:
                print("[aec_soot] bake_module failed on {0}: {1}"
                      .format(prim.GetName(), exc))
            continue
        if out is None:
            continue
        png = os.path.join(SOOT_OUT, "{0}.png".format(
            abs(hash((key, tex["base"]))) % (10 ** 12)))
        _save_rgb(out, png, np)
        mat = UsdShade.Material.Define(
            stage, looks.GetPath().AppendChild("m{0}".format(made)))
        m2p.rebuild(stage, mat.GetPrim(),
                    {"base": png, "normal": tex.get("normal"),
                     "orm": tex.get("orm")})
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat)
        made += 1
        bound += 1

    if verbose:
        print("[aec_soot] de-instanced {0}; {1} facade part(s) sooted through "
              "their own UVs from {2} event(s)".format(un, bound, len(events)))
    return (bound, made, len(events))


def _textures_of(src_mat, m2p):
    from pxr import UsdShade
    shader = None
    for ctx in ("mdl", ""):
        try:
            s = (UsdShade.Material(src_mat.GetPrim()).ComputeSurfaceSource(ctx)[0]
                 if ctx else
                 UsdShade.Material(src_mat.GetPrim()).ComputeSurfaceSource()[0])
        except Exception:
            s = None
        if s:
            shader = s
            break
    if shader is None:
        return {}
    a = shader.GetSourceAsset("mdl")
    if not a:
        return {}
    mdl = getattr(a, "resolvedPath", "") or a.path
    return m2p.module_textures(mdl) if mdl else {}


def _load_rgb(path, np):
    from PIL import Image
    return np.asarray(Image.open(path).convert("RGB")).astype("float32") / 255.0


def _save_rgb(arr, path, np):
    from PIL import Image
    a = arr
    if a.ndim == 3 and a.shape[2] > 3:
        a = a[..., :3]
    Image.fromarray((np.clip(a, 0, 1) * 255.0 + 0.5).astype("uint8")).save(path)
