#!/usr/bin/env python
"""aec_burn.py -- soot and interior char on an AEC brownstone ROW, by name.

    import aec_burn
    aec_burn.burn_row(stage, "/World/b0", level="F3", units=(2, 3), seed=7)

WHAT THIS ASSET IS (measured 2026-09-02, `Reference_Brownstone5Row`):
`Reference_Brownstone02_0N/Brownstone02_{Instanced,Referenced}/.../Geometry/
<Category>/<Type>/<mesh>` -- 1535 named meshes in five separately addressable
row-house units, 6.65 m wide along the row (world Y), 21 m deep (world X),
four storeys. Per unit: ONE `Walls_Exterior/Walls_ExteriorFacade` mesh (1382
triangles, front AND rear wall, inner and outer faces), 80 Windows meshes,
19 Doors, 14 Floors, 8 Ceilings, 99 Structural_Framing, 22 Lighting_Fixtures,
plus stairs, railings, roofs, casework. Everything has a name, and the
categories that are not labelled (`Railings_588755_0`, `Stairs_441396_18939`)
are told apart from the labelled ones by LOCATION only.

WHY THE SOOT IS A CONFORMAL LAYER AND NOT A BAKE INTO THE MAP. The brick
`st` on the facade mesh spans ~15 repeats in both axes: the 4K brick map
TILES every ~1 m of wall. `soot_bake.uv_position_map` folds every repeat
onto one texel period, so baking the physics skin THROUGH those UVs writes
every storey's soot onto the same tile -- the "repeating pattern" the user
rejected on 2026-09-02, by construction. The alternatives:

  * re-UV the wall into a unique atlas and re-project brick + soot into it:
    costs 50-100x the brick's texel density, and two of the five units are
    vMaterials `Facade_Brick_Red_Clinker_*` -- a PROCEDURAL blend of
    `clinker_diff.jpg` with paint/leak/grime masks that no single PNG holds
    (the raw MDL renders exactly as shipped; a converted copy would not);
  * a translucent quad per elevation (`gac_fire.overlay_soot`): "a rectangle
    placed on the side of the house ... the wrong size", because a quad spans
    the mass BBOX (the stoop and rear yard put that 3.5 m proud of the real
    wall plane) and has none of the wall's relief.

So: every exterior triangle of the burning units is DUPLICATED, pushed
`STANDOFF_M` along its own normal, given `st` from its WORLD position on the
skin's unwrapped S|E|N|W canvas (`soot_plume.perimeter_offsets` /
`side_u`, exactly the addressing `soot_bake.sample_skin` uses), merged into
ONE mesh per fire and bound to `wall_overlay.overlay_material_textured`
(diffuse = the skin's RGB, opacity = its alpha, two SEPARATE PNGs). The
brick MDL underneath is never touched. Bay windows, stoops, cornices, fire
escapes and window reveals all carry the soot at their own position, and
the pattern is continuous across parts because it was never per part.
Needs `/rtx/raytracing/fractionalCutoutOpacity` (both forms -- see the
`build-urban-fire-scenes` skill, bug 4).

Roof faces (|n_z| > 0.7 at or above the deck) sample the canvas at
`deck_z - ROOF_K * d_inward`: walking in from the burning parapet reads the
wall DOWN from its eaves, so the deck carries the plume's fringe for a few
metres and then cleans up, instead of taking the eaves' soot uniformly.

THE FIRE IS PER UNIT. The mass box `soot_plume` unwraps is the burning
units' own bbox, so the neighbours stay clean, party wall by party wall --
which is how a rowhouse fire reads. Openings are the named Windows prims
(no glass-island guessing), the storey grid is the Floors prims, and
`urban_fire.plan_fire` draws the origin/band exactly as it does for a kit
building. Sides are the ones that actually have windows in the burning
units (front and rear for a mid-row unit, plus the end wall for an end
unit) -- never a blank party wall.

INTERIORS "can all just be completely charred" (user). Parts between the
front and rear wall planes (measured off the Windows: they sit IN the wall)
whose storey is in the fire band are rebound to a flat char; the facade
wall's own INNER faces in those storeys get an opaque char layer the same
way the soot layer is built; emissive light planes go dark. That needs the
burning units DE-INSTANCED (`SetInstanceable(False)` on
`Brownstone02_Instanced`) -- a bind onto an instance proxy silently
resolves to the prototype. Only the burning units are de-instanced; the
soot layer itself reads proxies and needs nothing.

Nothing is removed and nothing moves. Windows out, roof holes, collapse are
later passes.
"""
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
for _p in (os.path.normpath(os.path.join(_HERE, "..")), _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)

try:                                      # package import (disaster.aec_burn)
    from . import soot_plume as spl
    from . import urban_fire as uf
    from . import gac_fire as gf
    from . import wall_overlay as wov
    from . import soot_bake as sbk
except ImportError:                       # bare-script / sys.path import
    import soot_plume as spl              # noqa: F401
    import urban_fire as uf
    import gac_fire as gf
    import wall_overlay as wov
    import soot_bake as sbk

#: metres the soot layer stands off the real surface. Ray-traced, so this
#: only has to clear coincidence, never a z-buffer.
STANDOFF_M = 0.015
#: the roof strip: how far (m) from the parapet the roof texture extends,
#: and the e-folding distance of the fringe's alpha over it. Reading the
#: wall DOWN from the eaves instead (the first cut) blackened a 21 m deep
#: deck edge to edge on an F3, because every row it read was in the band
#: (aec_burn1 top view, 2026-09-02); the plume clears the roof, only its
#: fringe lands on it.
ROOF_REACH_M = 6.0
ROOF_DECAY_M = 1.6
#: metres of wall under the parapet whose soot the roof fringe is drawn from
EAVES_BAND_M = 1.0
#: a window island narrower/shorter than this is trim, not an opening
WIN_MIN_M = 0.3
#: how close (m) a part's bbox may come to a wall plane and still count as
#: interior; anything at or through the plane is exterior (windows, doors,
#: wall packs), anything beyond it is exterior (stoop, yard, fire escape).
PLANE_TOL_M = 0.3
#: roughness of the soot layer -- matte; the brick's own gloss shows where
#: alpha is low
OVERLAY_ROUGH = 0.92
#: flat char for interiors (`urban_fire._FLAT["char_concrete"]`, linear)
CHAR_RGB = (0.0175, 0.0170, 0.0165)
CHAR_ROUGH = 0.97

OUT_DIR_DEFAULT = "/isaac-sim/.cache/aec_burn_tex"

#: categories that are interior by construction whatever their position
#: (`Walls` is NOT here: the one `Basic_Wall` per unit is the areaway
#: retaining wall in front of the stoop, measured at x = -7.8)
INTERIOR_CATS = ("Floors", "Ceilings", "Casework")
#: categories that never take the char even when inside the planes (the
#: windows ARE the openings, in the wall plane; the facade wall is handled
#: face by face). Doors go through the position test: a front door is in
#: the plane and takes the soot layer, an interior door chars.
NEVER_CHAR = ("Windows", "Walls_Exterior")


# ---------------------------------------------------------------------------
# reading the asset
# ---------------------------------------------------------------------------
def _cat_of(path):
    parts = str(path).split("/")
    if "Geometry" in parts:
        i = parts.index("Geometry")
        if i + 1 < len(parts):
            return parts[i + 1]
    return "?"


def _unit_of(path, root_path):
    rel = str(path)[len(str(root_path)):].strip("/").split("/")
    # the first path element under the row's own reference prim that names
    # a unit: `Reference_Brownstone02_03`
    for q in rel:
        if q.startswith("Reference_") and q.rsplit("_", 1)[-1].isdigit():
            return q
    return None


def _unit_index(unit_name):
    return int(unit_name.rsplit("_", 1)[-1])


def _mat_name(prim):
    from pxr import UsdShade
    mat, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
    return mat.GetPath().name if mat else ""


def _mesh_tris_world(prim, mpu, xf_cache):
    """`(V, N)`: world-METRE triangle corners (T, 3, 3) and unit normals
    (T, 3) of one mesh, winding-corrected for a mirrored transform or a
    left-handed mesh. None for an empty mesh."""
    from pxr import UsdGeom
    me = UsdGeom.Mesh(prim)
    pts = me.GetPointsAttr().Get()
    if not pts:
        return None
    P = np.asarray(pts, dtype=np.float64)
    fvc = np.asarray(me.GetFaceVertexCountsAttr().Get() or [], dtype=np.int64)
    fvi = np.asarray(me.GetFaceVertexIndicesAttr().Get() or [], dtype=np.int64)
    if fvc.size == 0 or fvi.size == 0:
        return None
    tri, _f, _s = sbk.triangles(fvc, fvi)
    if tri.shape[0] == 0:
        return None
    M = np.asarray(xf_cache.GetLocalToWorldTransform(prim), dtype=np.float64)
    W = (P @ M[:3, :3] + M[3, :3]) * float(mpu)
    V = W[tri]                                        # (T, 3, 3)
    N = np.cross(V[:, 1] - V[:, 0], V[:, 2] - V[:, 0])
    L = np.linalg.norm(N, axis=1)
    ok = L > 1e-12
    V, N, L = V[ok], N[ok], L[ok]
    if V.shape[0] == 0:
        return None
    N = N / L[:, None]
    flip = np.linalg.det(M[:3, :3]) < 0.0
    try:
        if (me.GetOrientationAttr().Get() or "rightHanded") == "leftHanded":
            flip = not flip
    except Exception:
        pass
    if flip:
        N = -N
    return V, N


def measure_row(stage, root_path, verbose=True):
    """Everything the plan needs, read off the asset's own prims, in world
    METRES: the units (bbox, instanceable prim, meshes), the storey levels,
    the window islands per unit per side, the wall planes and the deck."""
    from pxr import Usd, UsdGeom
    root = stage.GetPrimAtPath(root_path)
    if not root or not root.IsValid():
        raise ValueError("no prim at {0}".format(root_path))
    mpu = float(UsdGeom.GetStageMetersPerUnit(stage)) or 1.0
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])

    units = {}
    for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        u = _unit_of(prim.GetPath(), root_path)
        if u is None:
            continue
        r = bc.ComputeWorldBound(prim).ComputeAlignedRange()
        if r.IsEmpty():
            continue
        a, b = r.GetMin(), r.GetMax()
        bb = (a[0] * mpu, a[1] * mpu, a[2] * mpu, b[0] * mpu, b[1] * mpu, b[2] * mpu)
        rec = units.setdefault(u, {"name": u, "meshes": [], "bbox": None,
                                   "inst": None})
        rec["meshes"].append({"prim": prim, "path": str(prim.GetPath()),
                              "cat": _cat_of(prim.GetPath()), "bbox": bb,
                              "mat": _mat_name(prim),
                              "name": prim.GetName()})
        o = rec["bbox"]
        rec["bbox"] = bb if o is None else (min(o[0], bb[0]), min(o[1], bb[1]),
                                            min(o[2], bb[2]), max(o[3], bb[3]),
                                            max(o[4], bb[4]), max(o[5], bb[5]))
    if not units:
        raise ValueError("no unit meshes under {0}".format(root_path))
    # the instanceable prim of each unit (what has to be de-instanced)
    for prim in Usd.PrimRange(root):
        if prim.IsInstanceable():
            u = _unit_of(prim.GetPath(), root_path)
            if u in units:
                units[u]["inst"] = prim

    # row axis: the axis along which unit centres spread
    cs = np.asarray([[0.5 * (r["bbox"][0] + r["bbox"][3]),
                      0.5 * (r["bbox"][1] + r["bbox"][4])] for r in units.values()])
    axis = int(np.argmax(cs.max(axis=0) - cs.min(axis=0))) if len(cs) > 1 else 1
    perp = 1 - axis
    order = sorted(units.values(), key=lambda r: _unit_index(r["name"]))

    for rec in order:
        bb = rec["bbox"]
        rec["idx"] = _unit_index(rec["name"])
        rec["cx"], rec["cy"] = 0.5 * (bb[0] + bb[3]), 0.5 * (bb[1] + bb[4])
        # WINDOWS: side by the THINNEST bbox axis (a window sits in its wall,
        # so it is thin across the wall), never by nearest bbox line -- on a
        # 6.65 m unit most front windows are nearer a party wall than the
        # front bbox line.
        boxes = {"S": [], "E": [], "N": [], "W": []}
        planes = {"lo": [], "hi": []}        # wall planes across the row
        for mrec in rec["meshes"]:
            if mrec["cat"] != "Windows":
                continue
            x0, y0, z0, x1, y1, z1 = mrec["bbox"]
            side = _side_by_shape(mrec["bbox"], rec["cx"], rec["cy"], perp)
            if side in ("W", "E"):       # a wall facing +-x
                boxes[side].append((y0, y1, z0, z1))
                if perp == 0:
                    (planes["lo"] if side == "W" else planes["hi"]).append(0.5 * (x0 + x1))
            else:
                boxes[side].append((x0, x1, z0, z1))
                if perp == 1:
                    (planes["lo"] if side == "S" else planes["hi"]).append(0.5 * (y0 + y1))
        rec["windows"] = {s: _islands(v) for s, v in boxes.items()}
        rec["plane_lo"] = float(np.median(planes["lo"])) if planes["lo"] else bb[perp] + 3.0
        rec["plane_hi"] = float(np.median(planes["hi"])) if planes["hi"] else bb[3 + perp] - 3.0
        # THE DECK is the roof membrane with the largest footprint, not the
        # lowest one: a rear-extension roof at storey height read as the
        # deck and dropped the top storey from the grid (probe, 2026-09-02).
        roofs = [(abs((m["bbox"][3] - m["bbox"][0]) * (m["bbox"][4] - m["bbox"][1])),
                  m["bbox"][2]) for m in rec["meshes"] if m["cat"].startswith("Roofs")]
        rec["deck_z"] = float(max(roofs)[1]) if roofs else bb[5] - 1.0
        # STOREYS from the Floors' top faces, clustered 1.5 m apart; the
        # basement/areaway slabs within a metre of the base are the base
        # storey, not a storey of their own.
        floors = sorted(set(round(m["bbox"][5], 2) for m in rec["meshes"]
                            if m["cat"] == "Floors"))
        lv = []
        for z in floors:
            if z > bb[2] + 1.0 and z < rec["deck_z"] - 1.5 and (not lv or z - lv[-1] > 1.5):
                lv.append(z)
        rec["levels"] = [bb[2]] + lv

    meas = {"mpu": mpu, "root": root_path, "units": order, "axis": axis,
            "perp": perp, "bc": bc}
    if verbose:
        print("[aec_burn] {0} unit(s), row along {1}; per unit: {2}".format(
            len(order), "xy"[axis],
            "; ".join("{0}: {1} win(s) {2}, {3} storey(s) {4}".format(
                r["idx"], sum(len(v) for v in r["windows"].values()),
                {s: len(v) for s, v in r["windows"].items() if v},
                len(r["levels"]), [round(z, 1) for z in r["levels"]])
                for r in order)))
    return meas


def _islands(boxes, pad=0.12):
    """Union overlapping (u0, u1, z0, z1) boxes into window islands."""
    n = len(boxes)
    if n == 0:
        return []
    parent = list(range(n))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for i in range(n):
        a = boxes[i]
        for j in range(i + 1, n):
            b = boxes[j]
            if (a[0] - pad <= b[1] and b[0] - pad <= a[1]
                    and a[2] - pad <= b[3] and b[2] - pad <= a[3]):
                ri, rj = find(i), find(j)
                if ri != rj:
                    parent[ri] = rj
    groups = {}
    for i in range(n):
        groups.setdefault(find(i), []).append(boxes[i])
    out = []
    for g in groups.values():
        u0 = min(b[0] for b in g); u1 = max(b[1] for b in g)
        z0 = min(b[2] for b in g); z1 = max(b[3] for b in g)
        if (u1 - u0) >= WIN_MIN_M and (z1 - z0) >= WIN_MIN_M:
            out.append((float(u0), float(u1), float(z0), float(z1)))
    out.sort()
    return out


# ---------------------------------------------------------------------------
# the plan: mass box, openings, plan_fire, events, skin
# ---------------------------------------------------------------------------
def default_units(n):
    """Which units burn when the caller does not say: the middle two of a
    row of four or more, everything on a shorter row."""
    if n <= 2:
        return tuple(range(1, n + 1))
    a = (n + 1) // 2
    return (a, a + 1) if a + 1 <= n else (a,)


def pick_units(n, rng):
    """A SEEDED choice for a city bake: one or two contiguous units (two
    70 % of the time on a row of three or more), anywhere along the row,
    so ten burning rows in a city are not ten identical middle pairs."""
    if n <= 2:
        return tuple(range(1, n + 1))
    k = 2 if rng.random() < 0.7 else 1
    start = rng.randint(1, n - k + 1)
    return tuple(range(start, start + k))


def parse_units(text):
    """`"2,3"` / `"2-3"` / `"4"` -> a tuple of 1-based unit indices, or None."""
    t = (text or "").strip().replace("/", ",").replace("-", ",")
    if not t:
        return None
    return tuple(int(q) for q in t.split(",") if q.strip())


def plan_row(meas, level="F3", units=None, seed=7, origin=None, sides=None,
             verbose=True):
    """The fire plan for `units` (1-based indices, contiguous) of the row:
    mass box, storey grid, openings provider, `urban_fire.plan_fire`,
    `soot_plume.plan_events`, `soot_plume.skin`."""
    import random
    n = len(meas["units"])
    units = tuple(int(u) for u in (units or default_units(n)))
    burning = [r for r in meas["units"] if r["idx"] in units]
    if not burning:
        raise ValueError("units {0} not in row of {1}".format(units, n))
    bb = [min(r["bbox"][i] for r in burning) for i in range(3)] + \
         [max(r["bbox"][i] for r in burning) for i in range(3, 6)]
    x0, y0, z0, x1, y1, z1 = bb
    # storey grid: the union of the burning units' floor levels, clustered
    lv_all = sorted(set(round(z, 1) for r in burning for z in r["levels"][1:]))
    levels = [float(z0)]
    for z in lv_all:
        if z - levels[-1] > 1.5:
            levels.append(float(z))
    deck_z = float(np.median([r["deck_z"] for r in burning]))
    m = {"tag": "main", "cx": 0.5 * (x0 + x1), "cy": 0.5 * (y0 + y1),
         "yaw": 0.0, "W": float(x1 - x0), "D": float(y1 - y0),
         "z0": float(z0), "top": float(z1), "deck_z": deck_z,
         "levels": levels, "module": 4.0, "spec": {"bands": []}}
    info = {"masses": {"main": m}, "type": "urm", "elements": [],
            "n_storeys": len(levels), "x": m["cx"], "y": m["cy"]}

    # openings, in openings_provider's ABSOLUTE convention (S/N: x, E/W: y)
    rects = {"S": [], "E": [], "N": [], "W": []}
    for r in burning:
        for s, isl in r["windows"].items():
            rects[s].extend(isl)
    counts = {s: len(v) for s, v in rects.items()}
    # most windows first; on a tie the STREET FRONT (the wall facing -X on
    # this asset, side "W", with the stoops and bays) beats the rear
    front = "W" if meas["perp"] == 0 else "S"
    cands = [s for s in sorted(counts, key=lambda k: (-counts[k], k != front))
             if counts[s] >= 2]
    if not cands:
        raise ValueError("no window islands on the burning units")
    rng = random.Random(seed)
    if sides is None:
        want = 1 if level in ("F1", "F2") else (2 if level == "F3" else len(cands))
        sides = tuple(cands[:max(1, want)])
    fire = uf.plan_fire(info, level, rng, origin=origin, sides=tuple(sides))
    # THE REAL FACADE PLANES for the opening frames: the mass bbox face is
    # the stoop / rear-yard extent, ~3.5 m proud of the wall, and a Flow
    # emitter placed on it would burn in the street. `side_frame(planes=)`
    # moves only the frame's outward coordinate (gac_fire's own mechanism).
    if meas["perp"] == 0:
        planes = {"W": float(np.mean([r["plane_lo"] for r in burning])),
                  "E": float(np.mean([r["plane_hi"] for r in burning]))}
    else:
        planes = {"S": float(np.mean([r["plane_lo"] for r in burning])),
                  "N": float(np.mean([r["plane_hi"] for r in burning]))}
    provider = gf.openings_provider(rects, m, planes=planes)
    root_name = str(meas["root"]).rsplit("/", 1)[-1]
    ctx = {"info": info, "fire": fire, "rng": rng,
           "tag": "{0}_u{1}".format(root_name, "_".join(str(u) for u in units)),
           "soot_openings": provider}
    events = spl.plan_events(ctx, uf._severity)
    sk = None
    if events:
        nrng = np.random.default_rng(spl.event_seed(ctx) ^ 0x5EED)
        sk = spl.skin(ctx, events, nrng, finish=fire["finish"] or "char")
    if verbose:
        print("[aec_burn] units {0} {1}: origin storey {2}, band {3} of {4}, "
              "sides {5} (windows {6}), {7} event(s){8}".format(
                  units, level, fire["origin"], fire["storeys"],
                  len(levels), "".join(fire["sides"]), counts, len(events),
                  "" if events else " -- NO EVENTS"))
        if sk is not None:
            a = sk["rgba"][..., 3]
            print("[aec_burn] skin {0}x{1} px at {2:.0f} px/m, alpha mean {3:.3f}, "
                  ">0.3 on {4:.1%} of the canvas".format(
                      a.shape[1], a.shape[0], sk["ppm"], float(a.mean()),
                      float((a > 0.3).mean())))
    return {"units": units, "burning": burning, "m": m, "info": info,
            "fire": fire, "ctx": ctx, "events": events, "sk": sk,
            "rects": rects, "levels": levels, "planes": planes}


def flames_row(stage, meas, plan, flow_root, verbose=True):
    """Flow flames and smoke from the SAME event list the soot came from --
    `urban_fire.r_flames` on a ctx shaped the way that recipe reads it.
    The launcher must have authored the Flow stack (`fire.setup_flow_stack`)
    and hands its root in. State follows the level (`urban_fire.ACTIVE`):
    F2/F3 flame, F4 smoulder, F5 residual, F1 nothing. Interior smoke seats
    on the storey of origin only -- with the collapse the floors above it
    are on the debris pile, not where they were planned."""
    import random
    fire = dict(plan["fire"])
    fire["events"] = plan["events"]
    ctx = {"stage": stage, "flow_root": flow_root, "fire": fire,
           "info": plan["info"], "tag": plan["ctx"]["tag"], "notes": [],
           "rng": random.Random(spl.event_seed(plan["ctx"]) ^ 0xF1A3),
           "fit": {"slabs": {("main", int(fire["origin"])): True}}}
    if not fire.get("events"):
        return []
    try:
        uf.r_flames(ctx)
    except Exception as exc:
        import traceback
        traceback.print_exc()
        ctx["notes"].append("flames FAILED: {0}".format(exc))
    if verbose:
        for n in ctx["notes"]:
            print("[aec_burn] " + n)
    return ctx["notes"]


# ---------------------------------------------------------------------------
# classification: exterior / interior, side, storey
# ---------------------------------------------------------------------------
def _storey_of(levels, z):
    st = 0
    for i, lv in enumerate(levels):
        if z >= lv - 0.05:
            st = i
    return st


def _is_interior(mrec, unit, perp, deck_z):
    """A part BETWEEN the wall planes (never at or through one), under the
    deck. Windows/doors sit in the planes, stoops/yards/fire escapes are
    beyond them, roof plant is above the deck: all exterior."""
    if mrec["cat"] in NEVER_CHAR:
        return False
    b = mrec["bbox"]
    lo, hi = b[perp], b[3 + perp]
    c = 0.5 * (lo + hi)
    cz = 0.5 * (b[2] + b[5])
    # a Floors/Ceilings/Casework part is interior WHEREVER it sits: the roof
    # build-up carries a white "ceiling" slab at 12.1 m, above the deck the
    # largest membrane reports, and testing the deck first left it exterior
    # -- a bright plate floating in every roofless shell (aec_show2 F4/F5
    # top views, 2026-09-02)
    if mrec["cat"] in INTERIOR_CATS:
        return True
    if cz >= deck_z - 0.05:
        return False
    return (unit["plane_lo"] + PLANE_TOL_M <= c <= unit["plane_hi"] - PLANE_TOL_M
            and lo >= unit["plane_lo"] - PLANE_TOL_M
            and hi <= unit["plane_hi"] + PLANE_TOL_M)


def _face_side(m, n, cxy, perp, end_tol=0.6):
    """Which canvas side a face samples. Primary: the nearer of the two
    long-wall lines (across the row). A face whose normal runs ALONG the row
    and which sits at an end of the mass box samples that end side."""
    lx, ly = cxy[0] - m["cx"], cxy[1] - m["cy"]
    W, D = m["W"], m["D"]
    dist = {"S": ly + D / 2.0, "N": D / 2.0 - ly, "W": lx + W / 2.0, "E": W / 2.0 - lx}
    if perp == 0:                      # walls face +-x: W/E primary, S/N ends
        prim = "W" if dist["W"] < dist["E"] else "E"
        if abs(n[1]) > 0.7:
            if dist["S"] < end_tol:
                return "S"
            if dist["N"] < end_tol:
                return "N"
        return prim
    prim = "S" if dist["S"] < dist["N"] else "N"
    if abs(n[0]) > 0.7:
        if dist["W"] < end_tol:
            return "W"
        if dist["E"] < end_tol:
            return "E"
    return prim


def _wall_dist(m, side, x, y):
    """Metres inward from `side`'s wall line (positive inside the box)."""
    lx, ly = x - m["cx"], y - m["cy"]
    return {"S": ly + m["D"] / 2.0, "N": m["D"] / 2.0 - ly,
            "W": lx + m["W"] / 2.0, "E": m["W"] / 2.0 - lx}[side]


def canvas_uv(sk, m, side, pts):
    """`st` (P, 2) for world-metre points on `side`: u along the unwrapped
    perimeter (`soot_plume.perimeter_offsets` + `side_u`), v up the wall
    (row 0 of the canvas is the top, so v = (z - z0) / H)."""
    per, H, z0 = sk["per"], sk["H"], sk["z0"]
    off = sk["offsets"][side]
    u = np.asarray([spl.side_u(m, side, float(p[0]), float(p[1])) for p in pts])
    z = pts[:, 2].astype(np.float64)
    return np.stack([(off + u) / per, np.clip((z - z0) / H, 0.0, 1.0)], axis=1)


def roof_uv(sk, m, side, pts):
    """`st` (P, 2) for roof points on the ROOF STRIP texture: u along the
    perimeter as on the wall, v = 1 at the parapet falling to 0 at
    `ROOF_REACH_M` inward (row 0 of the strip is the parapet)."""
    per = sk["per"]
    off = sk["offsets"][side]
    u = np.asarray([spl.side_u(m, side, float(p[0]), float(p[1])) for p in pts])
    d = np.asarray([_wall_dist(m, side, float(p[0]), float(p[1])) for p in pts])
    v = 1.0 - np.clip(np.maximum(0.0, d) / ROOF_REACH_M, 0.0, 1.0)
    return np.stack([(off + u) / per, v], axis=1)


def roof_canvas(sk, rows=96):
    """The roof strip RGBA (rows x w): the wall's own soot from the metre
    under the parapet (`EAVES_BAND_M`, cycled row by row so the mottle
    survives), its alpha decayed by `exp(-d / ROOF_DECAY_M)` with d the
    inward distance each row stands for."""
    rgba, ppm = sk["rgba"], sk["ppm"]
    kb = max(1, int(round(EAVES_BAND_M * ppm)))
    band = rgba[:kb]
    out = np.zeros((rows, rgba.shape[1], 4), dtype=np.float32)
    for r in range(rows):
        d = ROOF_REACH_M * r / max(1, rows - 1)
        out[r] = band[r % kb]
        out[r, :, 3] *= math.exp(-d / ROOF_DECAY_M)
    return out


# ---------------------------------------------------------------------------
# authoring
# ---------------------------------------------------------------------------
def _author_tris(stage, path, root_prim, mpu, xf_cache, V, N, uv, offset, mat):
    """One Mesh of disconnected triangles: `V` (T, 3, 3) world metres pushed
    `offset` along `N` (T, 3), `uv` (T, 3, 2) or None, authored in the ROOT
    prim's local frame so it rides with the building."""
    from pxr import Gf, Sdf, UsdGeom, UsdShade, Vt
    T = V.shape[0]
    P = V + N[:, None, :] * float(offset)
    Mw = np.asarray(xf_cache.GetLocalToWorldTransform(root_prim), dtype=np.float64)
    Minv = np.linalg.inv(Mw)
    Pw = (P / float(mpu)).reshape(-1, 3)
    Pl = Pw @ Minv[:3, :3] + Minv[3, :3]
    mesh = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
    mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(Pl.astype(np.float32)))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(T, 3, dtype=np.int32)))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(np.arange(3 * T, dtype=np.int32)))
    mesh.CreateDoubleSidedAttr(True)
    mesh.CreateSubdivisionSchemeAttr("none")
    lo, hi = Pl.min(axis=0), Pl.max(axis=0)
    mesh.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*[float(v) for v in lo]),
                                         Gf.Vec3f(*[float(v) for v in hi])]))
    if uv is not None:
        pv = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
            "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying)
        pv.Set(Vt.Vec2fArray.FromNumpy(uv.reshape(-1, 2).astype(np.float32)))
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
    return mesh


def _flat_material(stage, path, rgb, rough):
    from pxr import Gf, Sdf, UsdShade
    mat = UsdShade.Material.Define(stage, Sdf.Path(path))
    sh = UsdShade.Shader.Define(stage, Sdf.Path(path).AppendChild("PreviewSurface"))
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(rough))
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def author_row(stage, meas, plan, out_dir=None, verbose=True):
    """Lay the soot layer over the burning units' exterior, char their
    burning interiors. Returns a stats dict."""
    from pxr import Sdf, Usd, UsdGeom, UsdShade
    out_dir = out_dir or os.environ.get("AEC_BURN_OUT") or OUT_DIR_DEFAULT
    root_path = meas["root"]
    root = stage.GetPrimAtPath(root_path)
    mpu = meas["mpu"]
    perp = meas["perp"]
    xf = UsdGeom.XformCache(Usd.TimeCode.Default())
    m, fire, sk = plan["m"], plan["fire"], plan["sk"]
    band = set(int(s) for s in fire["storeys"])
    levels = plan["levels"]
    stats = {"tris_soot": 0, "tris_char": 0, "parts_soot": 0, "parts_char": 0,
             "deinstanced": 0, "skipped_empty": 0}
    if sk is None:
        if verbose:
            print("[aec_burn] no skin -- nothing authored")
        return stats

    looks = stage.DefinePrim(Sdf.Path(root_path).AppendChild("BurnLooks"), "Scope")
    geo = stage.DefinePrim(Sdf.Path(root_path).AppendChild("BurnLayer"), "Scope")
    dif, opa = gf._write_overlay_textures(sk["rgba"], out_dir, plan["ctx"]["tag"])
    soot_mat = wov.overlay_material_textured(
        stage, str(looks.GetPath().AppendChild("soot")), opa, dif, OVERLAY_ROUGH)
    rdif, ropa = gf._write_overlay_textures(roof_canvas(sk), out_dir,
                                            plan["ctx"]["tag"] + "_roof")
    roof_mat = wov.overlay_material_textured(
        stage, str(looks.GetPath().AppendChild("soot_roof")), ropa, rdif, OVERLAY_ROUGH)
    char_mat = _flat_material(stage, str(looks.GetPath().AppendChild("char")),
                              CHAR_RGB, CHAR_ROUGH)

    # 1. de-instance the burning units (interior binds need real prims)
    for unit in plan["burning"]:
        ip = unit.get("inst")
        if ip is not None and ip.IsInstanceable():
            ip.SetInstanceable(False)
            stats["deinstanced"] += 1
    # the prim objects captured while instanced are proxies; re-resolve
    if stats["deinstanced"]:
        for unit in plan["burning"]:
            for mrec in unit["meshes"]:
                mrec["prim"] = stage.GetPrimAtPath(mrec["path"])
        xf.Clear()

    Vs, Ns, UVs = [], [], []            # the soot layer (walls and everything else)
    Vr, Nr, UVr = [], [], []            # the roof strip
    Vc, Nc = [], []                     # the opaque char layer (wall inner faces)
    for unit in plan["burning"]:
        deck_z = unit["deck_z"]
        for mrec in unit["meshes"]:
            prim = mrec["prim"]
            if mrec.get("dead") or not prim or not prim.IsValid() or not prim.IsActive():
                continue
            interior = _is_interior(mrec, unit, perp, deck_z)
            b = mrec["bbox"]
            st = _storey_of(levels, 0.5 * (b[2] + b[5]))
            if interior:
                if st in band:
                    UsdShade.MaterialBindingAPI.Apply(prim).Bind(
                        char_mat, UsdShade.Tokens.strongerThanDescendants)
                    stats["parts_char"] += 1
                continue
            got = _mesh_tris_world(prim, mpu, xf)
            if got is None:
                stats["skipped_empty"] += 1
                continue
            V, N = got
            C = V.mean(axis=1)                          # (T, 3) centroids
            # the facade wall's INNER faces in a burning storey: opaque char
            if mrec["cat"] == "Walls_Exterior":
                # a face on the front half whose normal points to the rear
                # (or vice versa) faces INTO the house. Geometric, not
                # plane-relative: the front plane estimate sits between
                # the bays and the main wall face (probe, 2026-09-02).
                ax = perp
                mid = unit["cx"] if ax == 0 else unit["cy"]
                inward = (((C[:, ax] < mid) & (N[:, ax] > 0.7))
                          | ((C[:, ax] > mid) & (N[:, ax] < -0.7)))
                sts = np.asarray([_storey_of(levels, z) for z in C[:, 2]])
                in_band = np.isin(sts, list(band))
                ch = inward & in_band
                if ch.any():
                    Vc.append(V[ch]); Nc.append(N[ch])
                    stats["tris_char"] += int(ch.sum())
                keep = ~ch
                V, N, C = V[keep], N[keep], C[keep]
                if V.shape[0] == 0:
                    continue
            uv = np.zeros((V.shape[0], 3, 2), dtype=np.float64)
            roof = (np.abs(N[:, 2]) > 0.7) & (C[:, 2] > deck_z - 0.8)
            sides = np.asarray([_face_side(m, N[t], C[t], perp)
                                for t in range(V.shape[0])])
            for side in ("S", "E", "N", "W"):
                sel = sides == side
                if not sel.any():
                    continue
                pts = V[sel].reshape(-1, 3)
                if (sel & ~roof).any():
                    s2 = sel & ~roof
                    uv[s2] = canvas_uv(sk, m, side, V[s2].reshape(-1, 3)).reshape(-1, 3, 2)
                if (sel & roof).any():
                    s2 = sel & roof
                    uv[s2] = roof_uv(sk, m, side, V[s2].reshape(-1, 3)).reshape(-1, 3, 2)
            if (~roof).any():
                Vs.append(V[~roof]); Ns.append(N[~roof]); UVs.append(uv[~roof])
                stats["tris_soot"] += int((~roof).sum())
            if roof.any():
                Vr.append(V[roof]); Nr.append(N[roof]); UVr.append(uv[roof])
                stats["tris_roof"] = stats.get("tris_roof", 0) + int(roof.sum())
            stats["parts_soot"] += 1

    if Vs:
        _author_tris(stage, str(geo.GetPath().AppendChild("soot")), root, mpu, xf,
                     np.concatenate(Vs), np.concatenate(Ns), np.concatenate(UVs),
                     STANDOFF_M, soot_mat)
    if Vr:
        _author_tris(stage, str(geo.GetPath().AppendChild("soot_roof")), root, mpu, xf,
                     np.concatenate(Vr), np.concatenate(Nr), np.concatenate(UVr),
                     STANDOFF_M, roof_mat)
    if Vc:
        _author_tris(stage, str(geo.GetPath().AppendChild("char_walls")), root, mpu,
                     xf, np.concatenate(Vc), np.concatenate(Nc), None,
                     STANDOFF_M, char_mat)
    if verbose:
        print("[aec_burn] soot layer: {0} tri(s) from {1} exterior part(s) + "
              "{6} roof tri(s); char: {2} interior part(s) rebound + {3} wall "
              "inner tri(s); {4} unit(s) de-instanced; textures {5}".format(
                  stats["tris_soot"], stats["parts_soot"], stats["parts_char"],
                  stats["tris_char"], stats["deinstanced"], out_dir,
                  stats.get("tris_roof", 0)))
    return stats


def burn_row(stage, root_path, level="F3", units=None, seed=7, origin=None,
             sides=None, out_dir=None, verbose=True, damage=True,
             flow_root=None):
    """measure -> plan -> damage (parts taken away) -> author (soot + char)
    -> flames (Flow, when `flow_root` is given). Returns `(meas, plan,
    stats)`. The damage runs BEFORE the soot layer so the layer is cut from
    what is left, never from a wall that is gone."""
    meas = measure_row(stage, root_path, verbose=verbose)
    plan = plan_row(meas, level=level, units=units, seed=seed, origin=origin,
                    sides=sides, verbose=verbose)
    dstats = damage_row(stage, meas, plan, verbose=verbose) if damage else {}
    stats = author_row(stage, meas, plan, out_dir=out_dir, verbose=verbose)
    stats.update({"damage_" + k: v for k, v in dstats.items()})
    if flow_root:
        stats["flames"] = flames_row(stage, meas, plan, flow_root, verbose=verbose)
    return meas, plan, stats


# ---------------------------------------------------------------------------
# THE DAMAGE LADDER -- named parts taken away, by level
# ---------------------------------------------------------------------------
# What each level does to the burning units, in the fire band, on the burning
# elevations where a side applies. Cumulative: a level implies everything a
# lower one does. Mirrors `urban_fire.LADDER` in spirit (glass first, then
# the sashes, then the fit-out, then the roof, then a wall) with the vocabulary
# this asset actually has -- see the module docstring.
LADDER = {
    "F1": {"glass": "events"},
    "F2": {"glass": "band", "door_glass": True, "door_char": True},
    "F3": {"glass": "band", "door_glass": True, "door_char": True, "sash": True,
           "frame_char": True, "ceilings": 0.5, "int_doors": "char",
           "deck_lumber": True},
    "F4": {"glass": "band", "door_glass": True, "door_char": True, "sash": True,
           "frame_char": True, "ceilings": 1.0, "int_doors": "gone",
           "deck_lumber": True, "door_gone": True, "floors": True, "roof": True,
           "stairs": True, "casework": True},
}
LADDER["F4"]["collapse"] = True
LADDER["F5"] = dict(LADDER["F4"], wall=True)
LADDER["F5c"] = LADDER["F5"]
LADDER["F6"] = LADDER["F5"]

# ACTUAL COLLAPSE (user, 2026-09-02: "for more burnt out units there has to
# be actual collapse of structure"). With `collapse`, the floors, ceilings,
# roof deck, roof plant and stairs of the band are not merely deactivated:
# each is cut into `COLLAPSE_SPLIT` pieces (VTK box clips) and authored as
# a rigid-body candidate in an UNSCALED sibling scope (`<root>_debris`,
# world metres, local points about the centroid + a translate op -- the
# `_cyl` lesson: the body's origin is its shape), so `settle.run` drops
# them into the shell and they pile on the storey of origin; the F5 wall
# strips are authored where they stood with an outward velocity and the
# bricks outside the face, and fall onto a low static mound. The caller
# (the launcher) runs `disaster.settle.run` on `stats["loose"]` /
# `stats["static"]` / `stats["velocity"]`; a bare-USD probe simply leaves
# the bodies where they were authored.
COLLAPSE_SPLIT = 3
ROOF_SPLIT = 4
DEBRIS_SUFFIX = "_debris"
#: outward velocity of a lost wall strip: rotates about its foot, top leads
THROW_BASE, THROW_TOP = 0.4, 2.4

#: F2+: the share of in-band windows on a burning side that lose their glass
#: is `GLASS_BASE + severity`, so the compartment of origin's neighbours
#: go and the storeys the plume only stained keep some panes
GLASS_BASE = 0.35
#: the F5 wall loss: which storeys (from the top down) and how much of the
#: elevation's width at the top, stepping in below (`fire_collapse`'s own
#: PROFILE_FOOT staircase, applied to this asset's two-storey loss)
WALL_LOSE_STOREYS = 2
WALL_SPAN_FRAC = (0.62, 0.86)
WALL_PROFILE_FOOT = 0.55
#: how far past the window plane the wall-loss box reaches into the house
WALL_THROUGH_M = 1.3
#: rubble: the mound's depth outward from the wall foot and its height
RUBBLE_DEPTH_M = (2.4, 3.4)
RUBBLE_H_M = (0.9, 1.4)
RUBBLE_BRICKS = 360
RUBBLE_CHUNKS = 30
#: the lost wall as SLABS? Off (user, 2026-09-02: "it'll be fine as long as
#: you place some bricks only"): settled strips read as big grey concrete
#: panels lying in the street (aec_show6). With physics the wall becomes
#: bricks; the hand-laid strips stay only on the no-physics probe path.
WALL_STRIP_BODIES = False
#: brick-sized bricks (user, 2026-09-02: "the bricks in the rubble are too
#: big. Have smaller ones there and make the bricks in the rubble also look
#: burnt"): a real brick, a small chunk of two or three still mortared
#: together, and a charred share; all darker than the clean wall
BRICK_SIZE_M = (0.20, 0.095, 0.06)
CHUNK_SIZE_M = (0.34, 0.20, 0.14)
#: flat tones (linear) -- burnt aluminium/steel; the mound is charred brick
#: (a burnt collapse is black rubble with brick showing through, no dust),
#: the loose bricks a burnt, darker brick than the clean wall
BURNT_METAL_RGB = (0.030, 0.026, 0.022)
RUBBLE_MOUND_RGB = (0.040, 0.026, 0.020)
RUBBLE_BRICK_RGB = (0.055, 0.030, 0.024)
#: the lost wall's strips: how wide, how far outside the face they start
#: (clear of the kept wall's collider -- spawned IN the wall they wedged in
#: the hole and never fell, "wallslab_05_2", 2026-09-02) and how far they
#: lean before the settle takes over
STRIP_W_M = 1.4
STRIP_STANDOFF_M = 0.45
STRIP_LEAN_DEG = 18.0

_WOOD_MATS = ("Wood_", "Fabric_", "Paint_", "Wood")


def _win_id(name):
    """`Windows_438338_18439438338_36282` -> `438338`."""
    q = name.split("_")
    return q[1] if len(q) > 1 else name


def _side_by_shape(bb, cx, cy, perp):
    """S/E/N/W of a part that sits IN a wall, by its THINNEST bbox axis --
    never by nearest bbox line (most front windows on a 6.65 m unit are
    nearer a party wall than the front line). A part whose plan aspect is
    not clearly thin one way (a 45-degree bay-window pane: dx ~ dy) goes to
    the wall ACROSS the row it is nearer, because a diagonal pane is a bay
    on the street/rear elevation, never an end wall -- six such panes per
    unit landed on the party-wall sides and dragged those blank sides into
    an F4 plan (aec_show2, 2026-09-02)."""
    dx, dy = bb[3] - bb[0], bb[4] - bb[1]
    mx, my = 0.5 * (bb[0] + bb[3]), 0.5 * (bb[1] + bb[4])
    if dx < 0.77 * dy:
        return "W" if mx < cx else "E"
    if dy < 0.77 * dx:
        return "S" if my < cy else "N"
    if perp == 0:
        return "W" if mx < cx else "E"
    return "S" if my < cy else "N"


def _part_side(bb, unit, perp=0):
    return _side_by_shape(bb, unit["cx"], unit["cy"], perp)


def _u_span(m, side, bb):
    """(u0, u1) of a bbox along `side` in `soot_plume.side_u` metres."""
    xs = (bb[0], bb[3]) if side in ("S", "N") else (0.5 * (bb[0] + bb[3]),) * 2
    ys = (0.5 * (bb[1] + bb[4]),) * 2 if side in ("S", "N") else (bb[1], bb[4])
    us = [spl.side_u(m, side, x, y) for x, y in zip(xs, ys)]
    return min(us), max(us)


def _kill(stage, mrec, stats, key):
    prim = mrec["prim"]
    if prim and prim.IsValid() and prim.IsActive():
        prim.SetActive(False)
    mrec["dead"] = True
    stats[key] = stats.get(key, 0) + 1


def _debris_scope(stage, root_path):
    """The UNSCALED sibling scope the rigid-body candidates live in."""
    from pxr import Sdf
    p = Sdf.Path(root_path)
    sc = p.GetParentPath().AppendChild(p.name + DEBRIS_SUFFIX)
    stage.DefinePrim(sc, "Scope")
    return str(sc)


def _author_body(stage, path, V, mat, uv=None):
    """One rigid-body candidate from disconnected triangles `V` (T, 3, 3) in
    world METRES: local points about the centroid, a translate op carrying
    the centroid, optional faceVarying `uv` (T, 3, 2). Returns
    `(path, centroid)`."""
    from pxr import Gf, Sdf, UsdGeom, UsdShade, Vt
    V = np.asarray(V, dtype=np.float64)
    T = V.shape[0]
    c = V.reshape(-1, 3).mean(axis=0)
    L = (V - c).reshape(-1, 3)
    mesh = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
    mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(L.astype(np.float32)))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(T, 3, dtype=np.int32)))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(np.arange(3 * T, dtype=np.int32)))
    mesh.CreateDoubleSidedAttr(True)
    mesh.CreateSubdivisionSchemeAttr("none")
    lo, hi = L.min(axis=0), L.max(axis=0)
    mesh.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*[float(v) for v in lo]),
                                         Gf.Vec3f(*[float(v) for v in hi])]))
    if uv is not None:
        pv = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
            "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying)
        pv.Set(Vt.Vec2fArray.FromNumpy(np.asarray(uv).reshape(-1, 2).astype(np.float32)))
    xf = UsdGeom.Xformable(mesh)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(float(c[0]), float(c[1]), float(c[2])))
    if mat is not None:
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
    return str(mesh.GetPath()), c


def _drop_pieces(stage, meas, mrec, scope, mat, stats, key, tag, n_split=1,
                 axis=0):
    """Cut a part into `n_split` pieces along `axis` and author each as a
    rigid-body candidate where it stood; the original goes inactive."""
    from pxr import Usd, UsdGeom
    xf = UsdGeom.XformCache(Usd.TimeCode.Default())
    md = _mesh_dict_world(mrec["prim"], meas["mpu"], xf)
    if md is None or not len(md["tris"]):
        _kill(stage, mrec, stats, key)
        return 0
    pieces = [md]
    if n_split > 1:
        P = np.asarray(md["P"])
        lo_a, hi_a = float(P[:, axis].min()), float(P[:, axis].max())
        cuts = np.linspace(lo_a, hi_a, n_split + 1)
        pieces = []
        for i in range(n_split):
            lo = [-1e9, -1e9, -1e9]; hi = [1e9, 1e9, 1e9]
            lo[axis], hi[axis] = cuts[i], cuts[i + 1]
            try:
                pc = _clip_box(md, lo, hi, keep_outside=False)
            except Exception:
                pc = None
            if pc is not None and len(pc["tris"]):
                pieces.append(pc)
        if not pieces:
            pieces = [md]
    k = 0
    for pc in pieces:
        V = np.asarray(pc["P"])[np.asarray(pc["tris"])]
        if V.shape[0] < 4:
            continue
        path, _c = _author_body(stage, "{0}/{1}_{2}".format(scope, tag, k), V, mat)
        stats.setdefault("loose", []).append(path)
        k += 1
    _kill(stage, mrec, stats, key)
    return k


def _char(mrec, mat, stats, key):
    from pxr import UsdShade
    prim = mrec["prim"]
    if prim and prim.IsValid():
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(
            mat, UsdShade.Tokens.strongerThanDescendants)
        mrec["charred"] = True
        stats[key] = stats.get(key, 0) + 1


def damage_row(stage, meas, plan, verbose=True):
    """Take the named parts away, level by level. Returns a stats dict.
    Marks every removed record `dead` so `author_row` never soots it, and
    swaps the facade wall record for the clipped copy when a wall is lost."""
    import random
    from pxr import Sdf, UsdShade
    fire, m, ctx = plan["fire"], plan["m"], plan["ctx"]
    level = fire["level"]
    spec = LADDER.get(level) or {}
    stats = {}
    if not spec:
        return stats
    band = set(int(s) for s in fire["storeys"])
    origin = int(fire["origin"])
    sides = set(fire["sides"])
    levels = plan["levels"]
    perp = meas["perp"]
    rng = random.Random(spl.event_seed(ctx) ^ 0xDA3A)
    root_path = meas["root"]

    # everything below authors on the units' own prims: de-instance first
    for unit in plan["burning"]:
        ip = unit.get("inst")
        if ip is not None and ip.IsInstanceable():
            ip.SetInstanceable(False)
            stats["deinstanced"] = stats.get("deinstanced", 0) + 1
    for unit in plan["burning"]:
        for mrec in unit["meshes"]:
            mrec["prim"] = stage.GetPrimAtPath(mrec["path"])

    looks = stage.DefinePrim(Sdf.Path(root_path).AppendChild("BurnLooks"), "Scope")
    metal_mat = _flat_material(stage, str(looks.GetPath().AppendChild("burnt_metal")),
                               BURNT_METAL_RGB, 0.85)
    char_mat = _flat_material(stage, str(looks.GetPath().AppendChild("char")),
                              CHAR_RGB, CHAR_ROUGH)
    collapse = bool(spec.get("collapse"))
    stats["loose"], stats["static"], stats["velocity"] = [], [], {}
    scope = _debris_scope(stage, root_path) if collapse else None
    along = 1 - perp

    for unit in plan["burning"]:
        deck_z = unit["deck_z"]
        utag = unit["name"].rsplit("_", 1)[-1]
        ev_by = {}
        for ev in plan["events"]:
            ev_by.setdefault((ev["side"], int(ev["storey"])), []).append(ev)

        # --- windows: glass, then sash, then the frame ---------------------
        groups = {}
        for mrec in unit["meshes"]:
            if mrec["cat"] == "Windows":
                groups.setdefault(_win_id(mrec["name"]), []).append(mrec)
        for wid, ms in groups.items():
            bb = [min(r["bbox"][i] for r in ms) for i in range(3)] + \
                 [max(r["bbox"][i] for r in ms) for i in range(3, 6)]
            side = _part_side(bb, unit, perp)
            st = _storey_of(levels, 0.5 * (bb[2] + bb[5]))
            if st not in band or side not in sides:
                continue
            if spec["glass"] == "events":
                u0, u1 = _u_span(m, side, bb)
                hit = any(o["span"][0] - 0.2 <= u1 and u0 <= o["span"][1] + 0.2
                          for ev in ev_by.get((side, st), []) for o in ev["ops"])
            else:
                hit = rng.random() < min(1.0, GLASS_BASE + uf._severity(ctx, st))
            if not hit:
                continue
            alu = [r for r in ms if not (r["mat"].startswith("Clear_Glass") or r["mat"] == "")]
            trim = max(alu, key=lambda r: (r["bbox"][3] - r["bbox"][0]) * (r["bbox"][5] - r["bbox"][2])
                       + (r["bbox"][4] - r["bbox"][1]) * (r["bbox"][5] - r["bbox"][2])) if alu else None
            for r in ms:
                if r["mat"].startswith("Clear_Glass") or r["mat"] == "":
                    _kill(stage, r, stats, "glass")
                elif r is not trim and spec.get("sash"):
                    _kill(stage, r, stats, "sash")
                elif spec.get("frame_char"):
                    _char(r, metal_mat, stats, "frame_char")

        for mrec in unit["meshes"]:
            if mrec.get("dead") or mrec["cat"] == "Windows":
                continue
            b = mrec["bbox"]
            st = _storey_of(levels, 0.5 * (b[2] + b[5]))
            cat, mat = mrec["cat"], mrec["mat"] or ""
            interior = _is_interior(mrec, unit, perp, deck_z)
            wood = any(mat.startswith(w) for w in _WOOD_MATS)
            glassy = mat.startswith("Clear_Glass") or mat == "Glass"

            # --- doors ----------------------------------------------------
            if cat == "Doors":
                if st not in band:
                    continue
                if interior:
                    if spec.get("int_doors") == "gone":
                        _kill(stage, mrec, stats, "int_door")
                    elif spec.get("int_doors") == "char":
                        _char(mrec, char_mat, stats, "int_door_char")
                    continue
                if _part_side(b, unit, perp) not in sides:
                    continue
                if glassy and spec.get("door_glass"):
                    _kill(stage, mrec, stats, "door_glass")
                elif wood and spec.get("door_gone"):
                    _kill(stage, mrec, stats, "door")
                elif spec.get("door_char"):
                    _char(mrec, metal_mat if not wood else char_mat, stats, "door_char")
                continue

            # --- the roof and what stands on it ---------------------------
            roof_gone = spec.get("roof") and fire.get("roof")
            nm = "{0}_{1}".format(utag, mrec["name"][:24])
            if cat.startswith("Roofs"):
                if roof_gone and b[2] >= deck_z - 0.5:
                    if collapse:
                        _drop_pieces(stage, meas, mrec, scope, char_mat, stats, "roof",
                                     "roof_" + nm, n_split=ROOF_SPLIT, axis=along)
                    else:
                        _kill(stage, mrec, stats, "roof")
                continue
            if 0.5 * (b[2] + b[5]) >= deck_z - 0.05 and cat in (
                    "Mechanical_Equipment", "Specialty_Equipment") or (
                    (cat.startswith("Railings") or cat.startswith("Top_Rails"))
                    and b[2] >= deck_z - 0.05):
                if roof_gone:
                    if collapse:
                        _drop_pieces(stage, meas, mrec, scope, metal_mat, stats,
                                     "roof_plant", "plant_" + nm)
                    else:
                        _kill(stage, mrec, stats, "roof_plant")
                continue

            # --- the fit-out ---------------------------------------------
            if interior:
                # the roof build-up's own ceiling slab goes with the roof
                if cat == "Ceilings" and roof_gone and b[5] >= deck_z - 1.0:
                    if collapse:
                        _drop_pieces(stage, meas, mrec, scope, char_mat, stats,
                                     "roof_ceiling", "rceil_" + nm,
                                     n_split=COLLAPSE_SPLIT, axis=perp)
                    else:
                        _kill(stage, mrec, stats, "roof_ceiling")
                    continue
                if st not in band:
                    continue
                if cat == "Floors":
                    if spec.get("floors") and st > origin and wood:
                        if collapse:
                            _drop_pieces(stage, meas, mrec, scope, char_mat, stats,
                                         "floor", "floor_" + nm,
                                         n_split=COLLAPSE_SPLIT, axis=perp)
                        else:
                            _kill(stage, mrec, stats, "floor")
                elif cat == "Ceilings":
                    p = float(spec.get("ceilings", 0.0))
                    if p > 0 and (p >= 1.0 or rng.random() < p):
                        if collapse:
                            _drop_pieces(stage, meas, mrec, scope, char_mat, stats,
                                         "ceiling", "ceil_" + nm,
                                         n_split=COLLAPSE_SPLIT, axis=perp)
                        else:
                            _kill(stage, mrec, stats, "ceiling")
                elif cat.startswith(("Stairs", "Runs", "Supports")):
                    if spec.get("stairs") and wood:
                        if collapse:
                            _drop_pieces(stage, meas, mrec, scope, char_mat, stats,
                                         "stair", "stair_" + nm)
                        else:
                            _kill(stage, mrec, stats, "stair")
                elif cat == "Casework" and spec.get("casework"):
                    _kill(stage, mrec, stats, "casework")
                continue

            # --- the rear deck (exterior timber on the burning side) ------
            if cat == "Structural_Framing" and st in band:
                side = "E" if 0.5 * (b[0] + b[3]) > unit["cx"] else "W"
                if side in sides:
                    if wood and spec.get("deck_lumber"):
                        _kill(stage, mrec, stats, "deck_lumber")
                    elif not wood and spec.get("frame_char"):
                        _char(mrec, metal_mat, stats, "deck_steel_char")
            elif cat == "Structural_Columns" and st in band and spec.get("frame_char"):
                _char(mrec, metal_mat, stats, "column_char")

        # --- F5: one burning elevation lost from the failure line up -----
        if spec.get("wall"):
            try:
                _lose_wall(stage, meas, plan, unit, rng, stats, verbose=verbose,
                           scope=scope)
            except Exception as exc:
                import traceback
                traceback.print_exc()
                print("[aec_burn] wall loss FAILED on {0}: {1}".format(unit["name"], exc))

        # --- what the bodies land on: the shell, the origin floor, the stoop
        if collapse:
            for mrec in unit["meshes"]:
                if mrec.get("dead"):
                    continue
                b = mrec["bbox"]
                st = _storey_of(levels, 0.5 * (b[2] + b[5]))
                c = mrec["cat"]
                if (c in ("Walls_Exterior", "Walls", "Structural_Columns")
                        or c.startswith(("Stairs", "Runs"))
                        or (c == "Floors" and st <= origin)):
                    stats["static"].append(mrec["path"])

    if verbose:
        summary = {k: (len(v) if isinstance(v, (list, dict)) else v)
                   for k, v in stats.items()}
        print("[aec_burn] damage {0}: {1}".format(
            level, ", ".join("{0} {1}".format(k, v) for k, v in sorted(summary.items()))))
    return stats


# ---------------------------------------------------------------------------
# F5: the wall loss (VTK box clips on the facade mesh) and its rubble
# ---------------------------------------------------------------------------
def _mesh_dict_world(prim, mpu, xf_cache):
    """`gac_storey_slice`-shaped dict (`P`, `tris`, `UV`, `MID`) of one mesh,
    DE-INDEXED, in world METRES, UVs looked up by the primvar's own rule."""
    from pxr import UsdGeom
    me = UsdGeom.Mesh(prim)
    pts = np.asarray(me.GetPointsAttr().Get() or [], dtype=np.float64)
    fvc = np.asarray(me.GetFaceVertexCountsAttr().Get() or [], dtype=np.int64)
    fvi = np.asarray(me.GetFaceVertexIndicesAttr().Get() or [], dtype=np.int64)
    if not len(pts) or not len(fvc):
        return None
    tri, _f, slot = sbk.triangles(fvc, fvi)
    M = np.asarray(xf_cache.GetLocalToWorldTransform(prim), dtype=np.float64)
    W = (pts @ M[:3, :3] + M[3, :3]) * float(mpu)
    uv = np.zeros((tri.shape[0], 3, 2), dtype=np.float64)
    for pv in UsdGeom.PrimvarsAPI(prim).GetPrimvars():
        if str(pv.GetTypeName()) not in ("texCoord2f[]", "float2[]"):
            continue
        vals = pv.Get()
        if not vals:
            continue
        arr = np.asarray([(q[0], q[1]) for q in vals], dtype=np.float64)
        idx = np.asarray(pv.GetIndices() or [], dtype=np.int64) if pv.IsIndexed() else None
        uv = sbk._corner_uv(tri, slot, arr, str(pv.GetInterpolation()), idx)
        break
    T = tri.shape[0]
    return {"P": W[tri].reshape(-1, 3), "tris": np.arange(3 * T).reshape(-1, 3),
            "UV": np.asarray(uv).reshape(-1, 2), "MID": np.zeros(T, dtype=np.int32)}


def _clip_implicit(md, implicit, keep_outside=True):
    """Clip a mesh dict by any VTK implicit function.

    VTK implicit functions are negative on their inside.  The normal path
    therefore keeps the positive/outside half; ``keep_outside=False`` keeps
    the negative/inside half.  Keeping this small wrapper beside
    :func:`_clip_box` lets non-fire damage use a rounded or oblique bite
    without reimplementing the UV-carrying VTK conversion.
    """
    import vtk
    from detail import gac_storey_slice as gss
    if md is None or not len(md["tris"]):
        return None
    cl = vtk.vtkClipPolyData()
    cl.SetInputData(gss._to_vtk(md))
    cl.SetClipFunction(implicit)
    if not keep_outside:
        cl.InsideOutOn()
    cl.SetLocator(vtk.vtkNonMergingPointLocator())
    cl.Update()
    return gss._from_vtk(cl.GetOutput())


def _clip_box(md, lo, hi, keep_outside=True):
    """`vtkBox` clip of a mesh dict: the part OUTSIDE (default) or INSIDE
    `[lo, hi]` (world metres), UVs interpolated across the cut."""
    import vtk
    box = vtk.vtkBox()
    box.SetBounds(float(lo[0]), float(hi[0]), float(lo[1]), float(hi[1]),
                  float(lo[2]), float(hi[2]))
    return _clip_implicit(md, box, keep_outside=keep_outside)


def _write_world_piece(stage, path, root_prim, mpu, xf_cache, md, mat):
    """A mesh dict in world metres authored in the root's local frame with
    its `st` and one material."""
    from pxr import Gf, Sdf, UsdGeom, UsdShade, Vt
    Mw = np.asarray(xf_cache.GetLocalToWorldTransform(root_prim), dtype=np.float64)
    Minv = np.linalg.inv(Mw)
    Pl = (np.asarray(md["P"], dtype=np.float64) / float(mpu)) @ Minv[:3, :3] + Minv[3, :3]
    tris = np.asarray(md["tris"], dtype=np.int64)
    mesh = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
    mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(Pl.astype(np.float32)))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(tris), 3, dtype=np.int32)))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(tris.ravel().astype(np.int32)))
    mesh.CreateDoubleSidedAttr(True)
    mesh.CreateSubdivisionSchemeAttr("none")
    lo, hi = Pl.min(axis=0), Pl.max(axis=0)
    mesh.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*[float(v) for v in lo]),
                                         Gf.Vec3f(*[float(v) for v in hi])]))
    uv = np.asarray(md["UV"], dtype=np.float64)[tris.ravel()]
    pv = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying)
    pv.Set(Vt.Vec2fArray.FromNumpy(uv.astype(np.float32)))
    if mat is not None:
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
    return mesh


def _lose_wall(stage, meas, plan, unit, rng, stats, verbose=True, scope=None,
               author_rubble=True, layer_name="BurnLayer",
               looks_name="BurnLooks"):
    """Cut the top `WALL_LOSE_STOREYS` storeys of one burning elevation out
    of the unit's facade mesh on a two-step staircase. With `scope` (the
    debris scope, i.e. a physics settle will follow) the cut-out strips are
    authored WHERE THEY STOOD as rigid bodies with an outward velocity and
    the bricks just outside the face, over a low static mound; without one
    the strips are laid by hand on a taller mound (no physics available).

    ``author_rubble=False`` stops after replacing the source façade with its
    UV-preserving remainder.  Earthquake uses that geometry-only mode and
    feeds the removed mass to its shared rubble-v2 planner; fire retains the
    historical default, including its charred bricks and settle bodies.
    ``layer_name``/``looks_name`` keep the authored namespaces honest for
    that shared use.  All defaults are the shipped fire behaviour.
    """
    from pxr import Sdf, Usd, UsdGeom, UsdShade
    from detail import gac_storey_slice as gss
    fire, m, levels = plan["fire"], plan["m"], plan["levels"]
    perp, mpu = meas["perp"], meas["mpu"]
    root = stage.GetPrimAtPath(meas["root"])
    xf = UsdGeom.XformCache(Usd.TimeCode.Default())
    wall = next((r for r in unit["meshes"] if r["cat"] == "Walls_Exterior"
                 and not r.get("dead")), None)
    if wall is None:
        return
    side = fire["sides"][0]
    if side not in (("W", "E") if perp == 0 else ("S", "N")):
        side = "W" if perp == 0 else "S"
    n = len(levels)
    f0 = max(int(fire["origin"]), n - WALL_LOSE_STOREYS)
    bb = unit["bbox"]
    plane = unit["plane_lo"] if side in ("W", "S") else unit["plane_hi"]
    # the box reaches from beyond the bays to THROUGH the wall: the window
    # plane sits mid-reveal, the main wall's outer skin is ~0.5 m inboard of
    # it and its inner face ~0.35 m further. Stopping at +0.7 cut the outer
    # skin only and left the inner face standing in the hole, so from the
    # street the "collapse" read as a pale wall with window holes
    # (aec_show2 F5, 2026-09-02). WALL_THROUGH_M clears the inner face; only
    # the facade mesh is clipped, so the fit-out behind it is untouched.
    if side in ("W", "S"):
        a_lo, a_hi = bb[perp] - 0.5, plane + WALL_THROUGH_M
    else:
        a_lo, a_hi = plane - WALL_THROUGH_M, bb[3 + perp] + 0.5
    along = 1 - perp
    L0, L1 = bb[along], bb[3 + along]
    width = L1 - L0
    span = rng.uniform(*WALL_SPAN_FRAC) * width
    c = rng.uniform(L0 + span / 2.0 + 0.3, L1 - span / 2.0 - 0.3) if span + 0.6 < width \
        else 0.5 * (L0 + L1)
    boxes = []
    for k, st in enumerate(range(n - 1, f0 - 1, -1)):
        z0 = levels[st] + (0.0 if k else 0.0)
        z1 = (levels[st + 1] if st + 1 < n else unit["bbox"][5] + 1.0)
        if k == 0:
            z0 = levels[st] + 0.15
        frac = 1.0 if k == 0 else WALL_PROFILE_FOOT ** k
        s = span * frac
        cc = c + rng.uniform(-0.15, 0.15) * (span - s)
        lo = [0.0, 0.0, z0]; hi = [0.0, 0.0, z1]
        lo[perp], hi[perp] = a_lo, a_hi
        lo[along], hi[along] = cc - s / 2.0, cc + s / 2.0
        boxes.append((lo, hi))
    # EVERYTHING SET IN THE LOST WALL GOES WITH IT: the window trims (glass
    # and sashes are already gone), the doors, the wall packs — left where
    # they were they hang in the hole as "floating window borders"
    # (`/World/aec3_F5/BurnLayer/soot`, user 2026-09-02) and so do their
    # soot-layer copies. With physics they are rigid bodies with the wall's
    # own outward push and the settle drops them with the bricks; without,
    # they are simply deactivated. Marked dead either way, so `author_row`
    # never soots them.
    looks = stage.DefinePrim(Sdf.Path(meas["root"]).AppendChild(looks_name), "Scope")
    metal_mat = _flat_material(stage, str(looks.GetPath().AppendChild("burnt_metal")),
                               BURNT_METAL_RGB, 0.85)
    out_sign0 = -1.0 if side in ("W", "S") else 1.0
    n_parts = 0
    for mrec in unit["meshes"]:
        if mrec.get("dead") or mrec["cat"] not in (
                "Windows", "Doors", "Generic_Models", "Lighting_Fixtures"):
            continue
        b = mrec["bbox"]
        cc = (0.5 * (b[0] + b[3]), 0.5 * (b[1] + b[4]), 0.5 * (b[2] + b[5]))
        inside = any(all(lo[k] - 0.3 <= cc[k] <= hi[k] + 0.3 for k in range(3))
                     for lo, hi in boxes)
        if not inside:
            continue
        if scope:
            before = len(stats.get("loose") or [])
            _drop_pieces(stage, meas, mrec, scope, metal_mat, stats, "wall_part",
                         "wpart_{0}_{1}".format(unit["name"].rsplit("_", 1)[-1], mrec["name"][:20]))
            for path in (stats.get("loose") or [])[before:]:
                v = [0.0, 0.0, 0.0]
                v[perp] = out_sign0 * rng.uniform(0.4, 1.6)
                stats.setdefault("velocity", {})[path] = (v[0], v[1], 0.0)
        else:
            _kill(stage, mrec, stats, "wall_part")
        n_parts += 1
    md = _mesh_dict_world(wall["prim"], mpu, xf)
    if md is None:
        return
    kept, removed = md, []
    for lo, hi in boxes:
        piece = _clip_box(kept, lo, hi, keep_outside=False)
        if piece is not None and len(piece["tris"]):
            removed.append(piece)
        kept = _clip_box(kept, lo, hi, keep_outside=True)
        if kept is None:
            break
    if not removed:
        return
    src_mat, _ = UsdShade.MaterialBindingAPI(wall["prim"]).ComputeBoundMaterial()
    layer = stage.DefinePrim(Sdf.Path(meas["root"]).AppendChild(layer_name), "Scope")
    layer_scope = str(layer.GetPath())
    # LIVE STAGE: bind the source material prim itself. The export clone
    # (`gac_storey_slice._selfcontained_like`, a re-authored MDL sourceAsset)
    # is what survives a per-building export, but a re-authored MDL module
    # path renders WHITE in this Kit (build-urban-fire-scenes skill,
    # 2026-09-02 late); on the composed stage the original prim is right
    # there and renders brick. The export path is the open item.
    bind = src_mat if src_mat else None
    tag = unit["name"].rsplit("_", 1)[-1]
    if kept is not None and len(kept["tris"]):
        new = _write_world_piece(stage, layer_scope + "/wall_{0}".format(tag), root, mpu, xf,
                                 kept, bind)
        wall["prim"].SetActive(False)
        wall["prim"] = new.GetPrim()
        wall["path"] = str(new.GetPath())
        stats["wall_kept_tris"] = stats.get("wall_kept_tris", 0) + int(len(kept["tris"]))
    n_rem = sum(len(p["tris"]) for p in removed)
    stats["wall_lost_tris"] = stats.get("wall_lost_tris", 0) + int(n_rem)

    result = {"side": side, "boxes": boxes, "kept_tris":
              int(len(kept["tris"])) if kept is not None else 0,
              "lost_tris": int(n_rem), "parts_removed": int(n_parts)}
    if not author_rubble:
        if verbose:
            print("[aec_burn] {0}: {1} elevation partially lost ({2} tri(s) "
                  "cut, {3} kept); rubble delegated to caller".format(
                      unit["name"], side, n_rem, result["kept_tris"]))
        return result

    # --- the rubble: a mound outside the wall foot, wall slabs on it -----
    out_sign = -1.0 if side in ("W", "S") else 1.0
    foot = plane + out_sign * 0.3
    depth = rng.uniform(*RUBBLE_DEPTH_M)
    hmax = rng.uniform(*RUBBLE_H_M) * (0.55 if scope else 1.0)
    z_ground = float(levels[0]) if levels[0] > -0.3 else 0.0
    # a unit whose stoop stands in front: pile past it
    if side in ("W",) and perp == 0:
        foot = min(foot, bb[0] + 0.2) if out_sign < 0 else foot
    cx_m = foot + out_sign * depth * 0.5
    ca = c
    looks = stage.DefinePrim(Sdf.Path(meas["root"]).AppendChild(looks_name), "Scope")
    mound_mat = _flat_material(stage, str(looks.GetPath().AppendChild("rubble_mound")),
                               RUBBLE_MOUND_RGB, 0.97)
    brick_mat = _flat_material(stage, str(looks.GetPath().AppendChild("rubble_brick")),
                               RUBBLE_BRICK_RGB, 0.95)
    char_mat = _flat_material(stage, str(looks.GetPath().AppendChild("char")),
                              CHAR_RGB, CHAR_ROUGH)
    half_a = span * 0.55 + 1.0
    # NO MOUND with physics (user, 2026-09-02: "the F5 rubble looks weird. I
    # don't think it's needed. Once you have buildings collapse it'll be
    # fine as long as you place some bricks only") -- the bodies pile up on
    # the street by themselves. The authored mound stays only for the
    # no-physics (bare USD) path, where nothing can fall.
    if not scope:
        V, N = _mound_tris(rng, cx_m, ca, perp, depth, half_a, hmax, z_ground)
        _author_tris(stage, layer_scope + "/rubble_{0}".format(tag), root, mpu, xf,
                     V, N, None, 0.0, mound_mat)
    z_top = unit["bbox"][5]
    z_f = levels[f0]
    out_vec = np.zeros(3); out_vec[perp] = out_sign
    P_all = np.asarray(md["P"], dtype=np.float64)

    def _outer(a0, a1, z0, z1):
        """The facade's OUTERMOST coordinate across the row within an
        along/height window -- a bay stands ~0.6 m proud of the window
        plane, and a body spawned inside its collider never falls
        (aec_show5: strips and bricks hanging on the bays)."""
        sel = ((P_all[:, along] >= a0 - 0.3) & (P_all[:, along] <= a1 + 0.3)
               & (P_all[:, 2] >= z0 - 0.3) & (P_all[:, 2] <= z1 + 0.3))
        if not sel.any():
            return plane
        return float(P_all[sel, perp].min() if out_sign < 0 else P_all[sel, perp].max())

    if scope:
        # PHYSICS: bricks and chunks as bodies just outside the lost face,
        # spread over the lost storeys, a modest outward push -- they fall
        # onto the mound and the street and pile up on their own
        nb = 0
        for kind, count, size, mat in (
                ("brick", RUBBLE_BRICKS // 2, BRICK_SIZE_M, brick_mat),
                ("chunk", RUBBLE_CHUNKS, CHUNK_SIZE_M, brick_mat),
                ("charred", RUBBLE_BRICKS // 2, BRICK_SIZE_M, char_mat)):
            for i in range(count):
                p = [0.0, 0.0, 0.0]
                p[along] = ca + rng.uniform(-span / 2.0, span / 2.0)
                p[2] = rng.uniform(z_f + 0.3, z_top - 0.3)
                face = _outer(p[along] - 0.3, p[along] + 0.3, p[2] - 0.5, p[2] + 0.5)
                p[perp] = face + out_sign * (0.35 + rng.uniform(0.0, 0.9))
                sz = tuple(s * rng.uniform(0.85, 1.15) for s in size)
                vb, _n = _box_tris(p, sz, rng.uniform(0, 360), rng.uniform(-35, 35))
                path, c = _author_body(stage, "{0}/{1}_{2}_{3}".format(scope, kind, tag, nb),
                                       vb, mat)
                v = out_vec * rng.uniform(0.3, 1.4)
                stats["velocity"][path] = (float(v[0]), float(v[1]), 0.0)
                stats["loose"].append(path)
                nb += 1
    else:
        # NO PHYSICS: bricks and chunks strewn on the mound, densest near its
        # crown, a share charred so the pile reads as burnt rubble
        for kind, count, size, mat in (
                ("bricks", RUBBLE_BRICKS, (0.21, 0.10, 0.07), brick_mat),
                ("chunks", RUBBLE_CHUNKS, (0.55, 0.32, 0.24), brick_mat),
                ("charred", RUBBLE_BRICKS // 2, (0.22, 0.11, 0.08), char_mat)):
            Vb, Nb = [], []
            for i in range(count):
                r = rng.random() ** 0.7
                th = rng.uniform(0, 2 * math.pi)
                ea = half_a * r * math.cos(th)
                ep = depth * 0.5 * r * math.sin(th)
                p = [0.0, 0.0, 0.0]
                p[along] = ca + ea
                p[perp] = cx_m + ep
                zh = _mound_h(hmax, depth * 0.5, half_a, ep, ea)
                sz = tuple(s * rng.uniform(0.7, 1.3) for s in size)
                p[2] = z_ground + max(0.0, zh) + 0.35 * sz[2]
                vb, nb2 = _box_tris(p, sz, rng.uniform(0, 360), rng.uniform(-35, 35))
                Vb.append(vb); Nb.append(nb2)
            _author_tris(stage, layer_scope + "/rubble_{0}_{1}".format(kind, tag), root,
                         mpu, xf, np.concatenate(Vb), np.concatenate(Nb), None, 0.0, mat)
    # the fallen wall: each removed piece split into strips (and, with
    # physics, per storey too, so a fragment is a storey tall, not two)
    k = 0
    for piece in (removed if (WALL_STRIP_BODIES or not scope) else []):
        P = np.asarray(piece["P"])
        lo_a, hi_a = P[:, along].min(), P[:, along].max()
        nstrip = max(2, min(6, int(round((hi_a - lo_a) / (STRIP_W_M if scope else 1.6)))))
        cuts = np.linspace(lo_a, hi_a, nstrip + 1)
        z_lo, z_hi = float(P[:, 2].min()), float(P[:, 2].max())
        zcuts = [z_lo - 0.1, z_hi + 0.1]
        if scope:
            zcuts = [z_lo - 0.1] + [lv for lv in levels if z_lo + 0.5 < lv < z_hi - 0.5] + [z_hi + 0.1]
        for i in range(nstrip):
          for j in range(len(zcuts) - 1):
            lo = [-1e9, -1e9, -1e9]; hi = [1e9, 1e9, 1e9]
            lo[along], hi[along] = cuts[i], cuts[i + 1]
            lo[2], hi[2] = zcuts[j], zcuts[j + 1]
            strip = _clip_box(piece, lo, hi, keep_outside=False)
            if strip is None or not len(strip["tris"]):
                continue
            Q = np.asarray(strip["P"], dtype=np.float64)
            if scope:
                # PHYSICS: the strip starts just OUTSIDE the face it came
                # from, leaning out about its own foot, with the outward
                # velocity of a wall rotating about its foot (top leads);
                # the settle does the rest. Spawned IN the wall it sat inside
                # the kept wall's collider and never fell.
                tri = np.asarray(strip["tris"])
                zb = Q[:, 2].min()
                pivot = np.zeros(3)
                pivot[along] = Q[:, along].mean(); pivot[perp] = plane; pivot[2] = zb
                ang = math.radians(STRIP_LEAN_DEG) * (1.0 if out_sign < 0 else -1.0)
                R = _rot_about_axis(along, ang)
                Q2 = (Q - pivot) @ R.T + pivot
                # clear of the facade's outermost face in this window (a
                # bay), then the standoff
                face = _outer(float(Q[:, along].min()), float(Q[:, along].max()),
                              float(Q[:, 2].min()), float(Q[:, 2].max()))
                inner = float(Q2[:, perp].max() if out_sign < 0 else Q2[:, perp].min())
                shift = abs(inner - face) + STRIP_STANDOFF_M
                Q2[:, perp] += out_sign * shift
                Vq = Q2[tri]
                uvq = np.asarray(strip["UV"])[tri]
                path, c = _author_body(
                    stage, "{0}/wallslab_{1}_{2}".format(scope, tag, k), Vq, bind, uvq)
                frac = (float(c[2]) - z_f) / max(0.5, z_top - z_f)
                speed = THROW_BASE + (THROW_TOP - THROW_BASE) * max(0.0, min(1.0, frac))
                v = out_vec * speed
                stats["velocity"][path] = (float(v[0]), float(v[1]), 0.0)
                stats["loose"].append(path)
                k += 1
                continue
            zb = Q[:, 2].min()
            pivot = np.zeros(3); pivot[along] = Q[:, along].mean(); pivot[perp] = plane; pivot[2] = zb
            ang = math.radians(rng.uniform(72.0, 105.0)) * (1.0 if out_sign < 0 else -1.0)
            # rotate about the ALONG axis at the wall foot, so the top of the
            # strip swings outward and down
            R = _rot_about_axis(along, ang)
            Q2 = (Q - pivot) @ R.T + pivot
            yaw = math.radians(rng.uniform(-14.0, 14.0))
            Rz = _rot_about_axis(2, yaw)
            cen = Q2.mean(axis=0)
            Q2 = (Q2 - cen) @ Rz.T + cen
            # land it on the mound
            ea = cen[along] - ca
            ep = cen[perp] - cx_m
            zs = z_ground + max(0.0, _mound_h(hmax, depth * 0.5, half_a, ep, ea))
            Q2[:, 2] += (zs - Q2[:, 2].min()) + rng.uniform(0.02, 0.25)
            Q2[:, perp] += out_sign * rng.uniform(0.2, 1.2)
            strip = dict(strip); strip["P"] = Q2
            _write_world_piece(stage, layer_scope + "/wallslab_{0}_{1}".format(tag, k), root,
                               mpu, xf, strip, bind)
            k += 1
    stats["wall_slabs"] = stats.get("wall_slabs", 0) + k
    if verbose:
        print("[aec_burn] {0}: {1} elevation lost from storey {2} up ({3} tri(s) cut, "
              "{4} kept), {5} slab(s) on a {6:.1f} x {7:.1f} m mound".format(
                  unit["name"], side, f0, n_rem, len(kept["tris"]) if kept else 0, k,
                  span * 1.1 + 2.0, depth))
    result["wall_slabs"] = int(k)
    return result


def _rot_about_axis(axis, ang):
    c, s = math.cos(ang), math.sin(ang)
    if axis == 0:
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=np.float64)
    if axis == 1:
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=np.float64)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float64)


def _mound_h(hmax, rp, ra, ep, ea):
    """Height of a half-ellipsoid mound at offsets (ep across, ea along)."""
    q = (ep / max(1e-6, rp)) ** 2 + (ea / max(1e-6, ra)) ** 2
    return hmax * math.sqrt(max(0.0, 1.0 - q))


def _mound_tris(rng, cx_perp, c_along, perp, depth, half_along, hmax, z_ground,
                n_r=10, n_t=28):
    """Disconnected triangles of a noisy half-ellipsoid mound, world metres."""
    along = 1 - perp
    rp, ra = depth * 0.5, half_along
    pts = []
    for i in range(n_r + 1):
        ring = []
        r = i / float(n_r)
        for j in range(n_t):
            th = 2 * math.pi * j / n_t
            wob = 1.0 + 0.10 * math.sin(3 * th + rng.random()) + 0.05 * (rng.random() - 0.5)
            ep, ea = rp * r * math.cos(th) * wob, ra * r * math.sin(th) * wob
            z = _mound_h(hmax, rp, ra, ep, ea) * (1.0 + 0.12 * (rng.random() - 0.5))
            p = [0.0, 0.0, z_ground + max(0.0, z) - 0.05]
            p[perp] = cx_perp + ep
            p[along] = c_along + ea
            ring.append(p)
        pts.append(ring)
    V = []
    for i in range(n_r):
        for j in range(n_t):
            a, b = pts[i][j], pts[i][(j + 1) % n_t]
            c2, d = pts[i + 1][j], pts[i + 1][(j + 1) % n_t]
            if i == 0:
                V.append([pts[0][0], c2, d])
            else:
                V.append([a, c2, d]); V.append([a, d, b])
    V = np.asarray(V, dtype=np.float64)
    N = np.cross(V[:, 1] - V[:, 0], V[:, 2] - V[:, 0])
    L = np.linalg.norm(N, axis=1); ok = L > 1e-12
    V, N = V[ok], N[ok] / L[ok][:, None]
    flip = N[:, 2] < 0
    V[flip] = V[flip][:, ::-1]
    N[flip] = -N[flip]
    return V, N


def _box_tris(centre, size, yaw_deg, tilt_deg):
    """12 disconnected triangles of a box, world metres."""
    sx, sy, sz = [0.5 * s for s in size]
    c = np.asarray(centre, dtype=np.float64)
    corners = np.array([[x, y, z] for x in (-sx, sx) for y in (-sy, sy) for z in (-sz, sz)])
    R = _rot_about_axis(2, math.radians(yaw_deg)) @ _rot_about_axis(0, math.radians(tilt_deg))
    corners = corners @ R.T + c
    faces = [(0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4), (2, 6, 7, 3), (0, 4, 6, 2), (1, 3, 7, 5)]
    V = []
    for q in faces:
        V.append([corners[q[0]], corners[q[1]], corners[q[2]]])
        V.append([corners[q[0]], corners[q[2]], corners[q[3]]])
    V = np.asarray(V, dtype=np.float64)
    N = np.cross(V[:, 1] - V[:, 0], V[:, 2] - V[:, 0])
    L = np.linalg.norm(N, axis=1)
    N = N / np.maximum(L, 1e-12)[:, None]
    # outward: away from the box centre
    C = V.mean(axis=1)
    flip = np.einsum("ij,ij->i", N, C - c) < 0
    N[flip] = -N[flip]
    V[flip] = V[flip][:, ::-1]
    return V, N


# ---------------------------------------------------------------------------
# the 2D oracle
# ---------------------------------------------------------------------------
def preview_png(meas, plan, out_path, wall_tris=True):
    """The skin canvas with the window islands and (optionally) the facade
    wall's outer triangles drawn in canvas space -- if the wall outlines
    frame the window rectangles, the mapping is right; if the plume roots
    sit on the rectangles, the events are right."""
    from PIL import Image, ImageDraw
    from pxr import Usd, UsdGeom
    sk, m = plan["sk"], plan["m"]
    if sk is None:
        return None
    rgba = sk["rgba"]
    h, w = rgba.shape[0], rgba.shape[1]
    # composite over a brick-ish ground so alpha reads
    bg = np.asarray([0.42, 0.27, 0.22], dtype=np.float32)
    a = rgba[..., 3:4]
    rgb = bg[None, None, :] * (1.0 - a) + rgba[..., :3] * a
    # linear -> sRGB-ish so it reads on screen like the render will
    rgb = np.clip(rgb, 0, 1) ** (1.0 / 2.2)
    im = Image.fromarray((rgb * 255.0 + 0.5).astype(np.uint8), "RGB")
    dr = ImageDraw.Draw(im)
    ppm, per, H, z0, off = sk["ppm"], sk["per"], sk["H"], sk["z0"], sk["offsets"]

    def col_of(side, u):
        return int(((off[side] + u) * ppm) % w)

    def row_of(z):
        return int((H - (z - z0)) * ppm)

    for side in ("S", "E", "N", "W"):
        x = col_of(side, 0.0)
        dr.line([(x, 0), (x, h)], fill=(255, 255, 0), width=2)
        dr.text((x + 4, 4), side, fill=(255, 255, 0))
    for ev in plan["events"]:
        c0, c1 = col_of(ev["side"], ev["u0"]), col_of(ev["side"], ev["u1"])
        r0, r1 = row_of(ev["z_head"]), row_of(ev["z_sill"])
        colr = {"flame": (255, 90, 0), "smoulder": (255, 200, 0),
                "out": (120, 200, 255), "stain": (200, 200, 200)}.get(ev["state"], (255, 255, 255))
        dr.rectangle([min(c0, c1), r0, max(c0, c1), r1], outline=colr, width=2)
    # every window island of the burning units, thin white
    for side, isl in plan["rects"].items():
        for (u0, u1, za, zb) in isl:
            if side in ("S", "N"):
                a0, a1 = u0 - (m["cx"] - m["W"] / 2.0), u1 - (m["cx"] - m["W"] / 2.0)
                if side == "N":
                    a0, a1 = (m["cx"] + m["W"] / 2.0) - u1, (m["cx"] + m["W"] / 2.0) - u0
            else:
                a0, a1 = u0 - (m["cy"] - m["D"] / 2.0), u1 - (m["cy"] - m["D"] / 2.0)
                if side == "W":
                    a0, a1 = (m["cy"] + m["D"] / 2.0) - u1, (m["cy"] + m["D"] / 2.0) - u0
            dr.rectangle([col_of(side, a0), row_of(zb), col_of(side, a1), row_of(za)],
                         outline=(255, 255, 255), width=1)
    if wall_tris:
        xf = UsdGeom.XformCache(Usd.TimeCode.Default())
        perp = meas["perp"]
        for unit in plan["burning"]:
            for mrec in unit["meshes"]:
                if mrec["cat"] != "Walls_Exterior":
                    continue
                got = _mesh_tris_world(mrec["prim"], meas["mpu"], xf)
                if got is None:
                    continue
                V, N = got
                C = V.mean(axis=1)
                for t in range(V.shape[0]):
                    if abs(N[t, 2]) > 0.7:
                        continue
                    side = _face_side(m, N[t], C[t], perp)
                    uv = canvas_uv(sk, m, side, V[t])
                    pts = [(int(u * w) % w, int((1.0 - v) * h)) for u, v in uv]
                    if max(p[0] for p in pts) - min(p[0] for p in pts) > w / 2:
                        continue     # seam straddler, do not draw across
                    dr.polygon(pts, outline=(0, 255, 120))
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    im.save(out_path)
    return out_path
