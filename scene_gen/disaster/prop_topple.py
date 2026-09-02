"""prop_topple.py — earthquake damage for kerbside props: `streetlight`,
`traffic_light`, `sign` and `street_tree` (live scene review, 2026-09-01,
user: "let's also work on props damage. Make street lights, trees, signals
all fall over if they're in damage range (I can see some of this is
happening already)").

WHAT WAS ALREADY HAPPENING, AND WHY IT ISN'T ENOUGH. `disaster.quake.
_clear_under_heaps` already tips/leans/removes street furniture, but only
within a DG4/DG5 BUILDING'S OWN RUBBLE-PILE REACH (a few metres past that
one wall) — a prop two blocks from any collapse, in the same strong-shaking
radius, is never touched by it. This module is the complementary,
FIELD-DRIVEN pass: every placed prop of the four target categories reads the
SAME damage-field intensity a building's own grade draw reads
(`quake.assemble`'s `field(x, y) * grade_scale`), independent of whether a
specific neighbour collapsed. `_clear_under_heaps` still runs first (it is
the more specific, geometrically-grounded mechanism for the pile itself);
this pass explicitly SKIPS anything already off-vertical (see
`_already_tilted`) so the two never compound into a double-rotated prop.

THREE-TIER TOPPLE, BY LOCAL INTENSITY. Below `partial_thresh`: untouched.
`partial_thresh` to `full_thresh`: a partial lean (10-30 deg). At or above
`full_thresh`: a full fall (~85-95 deg) for a pole; a tree instead draws
`tree_full_share` (default 0.35) of the time for a full windthrow and the
rest of the time still only a STRONG lean (25-40 deg) — the brief's own
"prefer lean + some fully windthrown-style falls": a rooted tree resists
being laid flat by wind/shaking far more than a slip-jointed sign or a
pole on a footing does, so even at high intensity most trees just lean
hard rather than uproot.

THE FALL AZIMUTH is drawn per prop (`_draw_azimuth`): 60% of the time
(when the scene names an epicentre, `disaster.epicenter`) it points AWAY
from the epicentre, jittered +-20 deg so a row of poles does not all fall
in lockstep; otherwise (or with no epicentre at all) it is uniform random.
Either way it is then walked through up to four candidate bearings (drawn
azimuth, +180, +90, -90) until one clears every building's footprint
(`_fall_clear`) — see "BUILDING CLEARANCE" below. A prop for which none of
the four is clear is left untouched and counted "refused".

BUILDING CLEARANCE, CHEAP. `records` (`quake.assemble`'s own per-building
damage ledger: x, y, W, D, prim — the same list `_clear_under_heaps` reads)
gives every building's footprint; `quake.in_reach` (already pure, already
exercised by `test_quake_heap_clearance.py`) answers "is this point inside
this footprint, plus a margin" without needing a stage query. The prop's
own reach is estimated from ONE bbox query (`_prop_height_stage`) times
sin(fall angle) — a stick approximation, not a swept-volume test, which is
the "cheap" the brief asks for. Checked at the fall's midpoint and tip.

AUTHORING REUSES `disaster.street_furniture.apply_street_furniture_pose`
FOR ALL FOUR CATEGORIES, INCLUDING THE TREE. That function is disaster-
agnostic by its own docstring (built for the hurricane pipeline, explicitly
meant to be called by other launchers) and already carries the two fixes
that matter here: the pre-pose ground is MEASURED from the standing item's
own mesh points (`bake.world_point_bounds`, never `UsdGeom.BBoxCache` —
that cache's AABB-of-an-AABB is what floats/buries fallen debris
elsewhere in this repo), and the rotation composes place-then-lean in the
correct (left-operand-first) order.

`vegetation.tip_tree` — the tornado pipeline's own whole-tree windthrow,
and the precedent this module was pointed at — was DELIBERATELY NOT called
directly. It rebuilds a tree's xform ops by reading a bare `rotateZ` key
off the placement; `city_detail.py` street trees (like every other
`apply_placements` prop) are authored with a combined `rotateXYZ` op
instead (`scene_generator.apply_placements`: "we keep ops as translate ->
rotateXYZ -> scale"), so `tip_tree` would silently find no yaw to preserve
and drop the tree's placed orientation. `apply_street_furniture_pose`
reads `rotateXYZ` (`vals.get("rotateXYZ")`) and has no tree-specific
assumption to violate, so it is the correct idiom to reuse here even
though its name says "street furniture" — the mechanics (rotate the whole
referenced prim about its own local origin, about an axis built from the
fall azimuth, then re-seat by measured ground) are exactly what a rigid
tree topple needs too. The root-plate mound `vegetation.root_plate`
authors for a tornado windthrow is NOT added here — out of this pass's
scope; see the module's own docstring if a future pass wants it.

ENV KNOBS (task brief: "threshold + shares tunable"):
    EQ_PROP_TOPPLE                 1/0, default on
    EQ_PROP_TOPPLE_FULL            full-fall intensity threshold (0.55)
    EQ_PROP_TOPPLE_PARTIAL         partial-lean intensity threshold (0.30)
    EQ_PROP_TOPPLE_TREE_FULL_SHARE share of high-intensity trees that fully
                                    windthrow rather than just lean hard (0.35)
    EQ_PROP_TOPPLE_MARGIN_M        building-footprint clearance margin (0.4 m)

PURE vs STAGE-TOUCHING, same split discipline `quake.py`'s own heap-
clearance half holds (module docstring there: "PURE — no pxr, no
scene_generator"). `_prop_kind`, `_draw_azimuth`, `decide_prop`,
`_building_footprints_stage` and `_fall_clear` need no `pxr` import
anywhere in their call chain (`quake.in_reach`, which `_fall_clear` calls,
is pure too) and are exercised directly on the host. `topple_props` and
its small stage helpers (`_prop_height_stage`, `_already_tilted`,
`_apply_extra_sink`) are the pxr-touching half.
"""

import math
import os

_POLE_CATS = frozenset({"streetlight", "traffic_light", "sign"})
_TREE_CATS = frozenset({"street_tree", "tree"})
_TARGET_CATS = _POLE_CATS | _TREE_CATS

# ---------------------------------------------------------------------------
# knobs (module-level defaults; env-overridden in `topple_props`)
# ---------------------------------------------------------------------------
FULL_THRESH_DEFAULT = 0.55
PARTIAL_THRESH_DEFAULT = 0.30
TREE_FULL_SHARE_DEFAULT = 0.35
BUILDING_MARGIN_M_DEFAULT = 0.4

AWAY_FROM_EPI_P = 0.6
AZIMUTH_JITTER_DEG = 20.0

PARTIAL_DEG = (10.0, 30.0)          # pole partial lean; also the tree mid-tier lean
FULL_DEG = (85.0, 95.0)             # pole full fall
TREE_FULL_DEG = (80.0, 95.0)        # tree full windthrow
TREE_STRONG_LEAN_DEG = (25.0, 40.0)  # tree denied full windthrow at high intensity
FULL_SINK_M = (0.04, 0.14)          # extra dig-in on top of the measured re-seat


def prop_topple_enabled():
    """`EQ_PROP_TOPPLE` — default ON (task brief: "EQ_PROP_TOPPLE=1 default
    on"). Read live, not cached at import time, so a test can flip it."""
    return os.environ.get("EQ_PROP_TOPPLE", "1").strip().lower() not in (
        "0", "false", "no")


def _prop_kind(category):
    """"tree" | "pole" | None — the vocabulary `decide_prop` keys on. Only
    the FOUR named categories are in scope (task brief: "NOT benches/
    planters/hydrants unless trivially includable" — they are not, here)."""
    if category in _TREE_CATS:
        return "tree"
    if category in _POLE_CATS:
        return "pole"
    return None


def _draw_azimuth(x, y, epicenter, rng, away_p=AWAY_FROM_EPI_P,
                  jitter_deg=AZIMUTH_JITTER_DEG):
    """Degrees 0..360, the PRIMARY candidate fall bearing for one prop at
    (x, y). `epicenter` is `(ex, ey)` in the same metre frame as (x, y), or
    None. Away-from-epicentre `away_p` of the time (jittered so a ring of
    props round the epicentre does not fall in perfect radial lockstep),
    uniform random otherwise — "azimuth away from/random vs epicentre",
    per the brief."""
    if epicenter is not None and rng.random() < away_p:
        ex, ey = float(epicenter[0]), float(epicenter[1])
        dx, dy = float(x) - ex, float(y) - ey
        base = (math.degrees(math.atan2(dy, dx)) if (dx or dy)
                else rng.uniform(0.0, 360.0))
        return (base + rng.uniform(-jitter_deg, jitter_deg)) % 360.0
    return rng.uniform(0.0, 360.0)


def decide_prop(category, intensity, rng, x=0.0, y=0.0, epicenter=None, *,
                full_thresh=FULL_THRESH_DEFAULT,
                partial_thresh=PARTIAL_THRESH_DEFAULT,
                tree_full_share=TREE_FULL_SHARE_DEFAULT):
    """One topple decision for a single prop, PURE (no pxr, no stage).

    Returns None (untouched — below `partial_thresh`, or not a target
    category) or a dict:

        {"kind": "pole"|"tree", "tier": "full"|"partial",
         "deg": <unsigned lean, degrees>, "azimuth_deg": <primary bearing>}

    `deg`/`azimuth_deg` are the topple's PRIMARY draw; `topple_props` may
    still swap in a flipped/rotated bearing if the primary one would put
    the prop through a building (see `_fall_clear`) — the degree and tier
    drawn here do not change when that happens, only the bearing does.
    """
    kind = _prop_kind(category)
    if kind is None or float(intensity) < float(partial_thresh):
        return None
    az = _draw_azimuth(x, y, epicenter, rng)
    if float(intensity) < float(full_thresh):
        deg = rng.uniform(*PARTIAL_DEG)
        return {"kind": kind, "tier": "partial", "deg": deg, "azimuth_deg": az}
    if kind == "tree":
        if rng.random() < float(tree_full_share):
            deg = rng.uniform(*TREE_FULL_DEG)
            return {"kind": kind, "tier": "full", "deg": deg, "azimuth_deg": az}
        deg = rng.uniform(*TREE_STRONG_LEAN_DEG)
        return {"kind": kind, "tier": "partial", "deg": deg, "azimuth_deg": az}
    deg = rng.uniform(*FULL_DEG)
    return {"kind": kind, "tier": "full", "deg": deg, "azimuth_deg": az}


def _building_footprints_stage(records, placements_by_prim, ssf):
    """[(cx, cy, W, D, yaw_deg)], all in STAGE units, one per building
    `records` entry (`quake.assemble`'s own per-building ledger: x/y/W/D in
    METRES, `prim` the building's own prim path). `yaw_deg` is looked up off
    the matching placement — `records` itself never carries it."""
    out = []
    for r in records or ():
        p = placements_by_prim.get(r.get("prim")) or {}
        out.append((float(r.get("x", 0.0)) * ssf, float(r.get("y", 0.0)) * ssf,
                    max(1.0, float(r.get("W", 20.0))) * ssf,
                    max(1.0, float(r.get("D", 20.0))) * ssf,
                    float(p.get("yaw_deg", 0.0))))
    return out


def _fall_clear(px_stage, py_stage, azimuth_deg, reach_stage, buildings,
                margin_stage):
    """True if a prop at (px_stage, py_stage), falling toward
    `azimuth_deg` a stage-unit `reach_stage`, keeps its midpoint AND tip
    clear of every building footprint (each expanded by `margin_stage`).
    Cheap: a stick approximation (the prop's own bbox height x sin(lean)),
    not a swept OBB — the brief's own "cheap footprint check"."""
    from .quake import in_reach

    a = math.radians(float(azimuth_deg))
    ux, uy = math.cos(a), math.sin(a)
    for frac in (0.5, 1.0):
        sx = px_stage + reach_stage * frac * ux
        sy = py_stage + reach_stage * frac * uy
        for (cx, cy, W, D, yaw) in buildings:
            inside, _d, _s = in_reach(cx, cy, W, D, yaw, sx, sy, margin_stage)
            if inside:
                return False
    return True


def _clear_azimuth(px_stage, py_stage, primary_az, reach_stage, buildings,
                   margin_stage):
    """The first of (primary, +180, +90, -90) that clears every building,
    or None if all four intersect. Four candidates, not an exhaustive
    search — "cheap", per the brief; a prop hemmed in on all four principal
    bearings is refused rather than swept over a finer angle grid."""
    for delta in (0.0, 180.0, 90.0, -90.0):
        az = (primary_az + delta) % 360.0
        if _fall_clear(px_stage, py_stage, az, reach_stage, buildings,
                       margin_stage):
            return az
    return None


# ---------------------------------------------------------------------------
# stage-touching half
# ---------------------------------------------------------------------------
def _prop_height_stage(prim):
    """World-space bbox height (stage units) of the STANDING prop — a cheap
    stand-in for its reach once toppled (`height * sin(lean)`), not a
    seating measurement (that is `apply_street_furniture_pose`'s own job,
    done properly with points, not this bbox)."""
    from pxr import Usd, UsdGeom

    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    rng_ = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    if rng_.IsEmpty():
        return 0.0
    lo, hi = rng_.GetMin(), rng_.GetMax()
    return max(0.0, float(hi[2]) - float(lo[2]))


def _already_tilted(prim, eps_deg=3.0):
    """True if `prim`'s CURRENT local transform already carries a
    meaningful tilt off vertical — i.e. something upstream (typically
    `quake._clear_under_heaps`'s own tip/lean, for the same pole
    categories this pass targets) already toppled it. A plain
    `apply_placements` prop only ever carries a yaw (rotation about world
    Z), so its local "up" stays exactly (0, 0, 1); anything already tipped
    reads a real angle here. Guards against double-toppling regardless of
    which upstream pass did it, and regardless of call order."""
    from pxr import Gf, UsdGeom

    local = UsdGeom.XformCache().GetLocalTransformation(prim)[0]
    up = local.TransformDir(Gf.Vec3d(0.0, 0.0, 1.0))
    n = up.GetLength()
    if n < 1e-9:
        return False
    z = max(-1.0, min(1.0, up[2] / n))
    return math.degrees(math.acos(z)) > eps_deg


def _apply_extra_sink(prim, sink_m, ssf):
    """Nudge the ALREADY-RE-SEATED prim's translate down `sink_m` more
    metres — "small sink so the base digs in" (task brief), on top of
    `apply_street_furniture_pose`'s own correct ground re-seat rather than
    instead of it."""
    if sink_m <= 0.0:
        return
    from pxr import Gf, UsdGeom

    xf = UsdGeom.Xformable(prim)
    for op in xf.GetOrderedXformOps():
        if op.GetOpName().split(":")[-1] == "translate":
            v = op.Get()
            if v is not None:
                op.Set(Gf.Vec3d(float(v[0]), float(v[1]),
                                float(v[2]) - float(sink_m) * float(ssf)))
            break


def topple_props(stage, config, placements, records, field, grade_scale, rng,
                 ssf, epicenter=None, bounds=None, verbose=True):
    """Topple every placed streetlight/traffic_light/sign/street_tree whose
    local damage-field intensity clears `partial_thresh`. Called from
    `quake.assemble`, AFTER `_clear_under_heaps` (so `_already_tilted`
    correctly skips whatever that pass already touched) and after
    `records` is fully populated (so building footprints are known).

    Returns `{category: {"placed", "toppled", "leaned", "refused"}}`, one
    entry per category actually seen in `placements` (pre-seeded with the
    four target categories at zero so a category with nothing placed still
    reports).
    """
    stats = {cat: {"placed": 0, "toppled": 0, "leaned": 0, "refused": 0}
             for cat in ("streetlight", "traffic_light", "sign", "street_tree")}
    if not prop_topple_enabled():
        if verbose:
            print("[quake] prop topple: disabled (EQ_PROP_TOPPLE=0)")
        return stats

    from . import street_furniture as sf

    dis = config.get("disaster") or {}
    epi = epicenter
    if epi is None:
        e = dis.get("epicenter")
        if e is not None:
            try:
                epi = (float(e[0]), float(e[1]))
            except (TypeError, ValueError, IndexError):
                epi = None

    full_thresh = float(os.environ.get("EQ_PROP_TOPPLE_FULL", FULL_THRESH_DEFAULT))
    partial_thresh = float(os.environ.get("EQ_PROP_TOPPLE_PARTIAL",
                                          PARTIAL_THRESH_DEFAULT))
    tree_full_share = float(os.environ.get("EQ_PROP_TOPPLE_TREE_FULL_SHARE",
                                           TREE_FULL_SHARE_DEFAULT))
    margin_m = float(os.environ.get("EQ_PROP_TOPPLE_MARGIN_M",
                                    BUILDING_MARGIN_M_DEFAULT))

    by_prim = {p.get("prim_path"): p for p in placements if p.get("prim_path")}
    buildings = _building_footprints_stage(records, by_prim, ssf)
    margin_stage = margin_m * ssf

    for p in placements:
        cat = p.get("category")
        kind = _prop_kind(cat)
        if kind is None:
            continue
        path = p.get("prim_path")
        if not path:
            continue
        s = stats.setdefault(cat, {"placed": 0, "toppled": 0, "leaned": 0,
                                   "refused": 0})
        s["placed"] += 1

        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid():
            continue
        try:
            if _already_tilted(prim):
                continue
        except Exception:
            pass

        x, y = float(p.get("x_m", 0.0)), float(p.get("y_m", 0.0))
        px, py = x * ssf, y * ssf
        if bounds is not None:
            bx0, by0, bx1, by1 = bounds
            if not (bx0 <= px <= bx1 and by0 <= py <= by1):
                continue

        inten = float(field(x, y)) * float(grade_scale)
        dec = decide_prop(cat, inten, rng, x=x, y=y, epicenter=epi,
                          full_thresh=full_thresh, partial_thresh=partial_thresh,
                          tree_full_share=tree_full_share)
        if dec is None:
            continue

        try:
            h_stage = _prop_height_stage(prim)
        except Exception:
            h_stage = 6.0 * ssf
        reach_stage = h_stage * math.sin(math.radians(min(90.0, abs(dec["deg"]))))

        az = _clear_azimuth(px, py, dec["azimuth_deg"], reach_stage, buildings,
                            margin_stage)
        if az is None:
            s["refused"] += 1
            continue

        try:
            action = "flat" if dec["tier"] == "full" else "leaning"
            decision = {"x": x, "y": y, "category": cat, "action": action,
                       "lean_deg": dec["deg"], "azimuth_deg": az, "slide_m": 0.0}
            changed = sf.apply_street_furniture_pose(stage, path, decision, ssf=ssf)
            if changed and dec["tier"] == "full":
                _apply_extra_sink(prim, rng.uniform(*FULL_SINK_M), ssf)
        except Exception as exc:
            if verbose:
                print("[quake] prop topple: {0} on {1} failed ({2}), skipped"
                      .format(dec["tier"], path, exc))
            continue

        if dec["tier"] == "full":
            s["toppled"] += 1
        else:
            s["leaned"] += 1

    if verbose:
        seen = {k: v for k, v in stats.items() if v["placed"]}
        tot_p = sum(v["placed"] for v in seen.values())
        tot_t = sum(v["toppled"] for v in seen.values())
        tot_l = sum(v["leaned"] for v in seen.values())
        tot_r = sum(v["refused"] for v in seen.values())
        print("[quake] prop topple: {0} placed, {1} toppled, {2} leaned, "
              "{3} refused (building clearance): {4}".format(
                  tot_p, tot_t, tot_l, tot_r,
                  ", ".join("{0} {1}/{2}/{3}/{4}".format(
                      k, v["placed"], v["toppled"], v["leaned"], v["refused"])
                           for k, v in sorted(seen.items()))))
    return stats
