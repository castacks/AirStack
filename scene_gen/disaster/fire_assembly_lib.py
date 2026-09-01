"""fire_assembly_lib — the parts of `fire_assembly_launch_script.py` that a
SECOND launcher needs, lifted out so it can import them.

WHY THIS MODULE EXISTS. `fire_assembly_launch_script.py` builds a
`SimulationApp` AT IMPORT (line 107 of the original) — `import
fire_assembly_launch_script` from another launcher would start a second Kit
app inside the first, which is the segfault `downtown_quake_launch_script.py`
already documents ("a second Kit app in one process is a segfault inside the
first second (measured)"). The city launcher
(`urban_fire_city_launch_script.py`, `urban_fire_city_plan.md` work item #7)
has to re-place the Flow emitters on a BAKE exactly the way the row launcher
does, so the alternative to this module is a second copy of `place_fire` —
the one function in the pipeline that must not drift, because it is the only
place `urban_fire._flame_sources` is called against a bake rather than
against a live building.

WHAT IS IN HERE, AND WHAT IS NOT. Everything here was MOVED, not rewritten:
`vram_mb`, `resolve_bakes`, `order_bakes`, `build_ground_and_light`,
`_sphere_source`, `place_fire`, and the fire-facing review-camera arithmetic
(`fire_view_params`) that used to live inline in the row launcher's capture
block. The three module-level knobs those functions used to close over —
`FA_GLOB`, `FA_ORDER`, `FA_SMOKE` — became ordinary parameters
(`pattern=`, `order=`, `smoke=`) and the log prefix became `prefix="fa"`, so
the row launcher passes its own env-derived values and gets byte-identical
behaviour. Nothing else changed; no logic was touched.

What is NOT here: `_env` (both launchers need it BEFORE `SimulationApp`,
i.e. before `scene_gen` is on `sys.path`), `KIT_ARGS` (same reason — it is a
literal in each launcher on purpose, see the row launcher's own comment) and
`_bbox` (three lines, and each launcher measures different prims).
"""

import glob as _glob
import json
import math
import os
import random

from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdShade, Vt

from . import fire as fx
from . import soot_plume as spl
from . import urban_fire as uf


# ---------------------------------------------------------------------------
# Finding the bakes
# ---------------------------------------------------------------------------
def vram_mb(tag, prefix="fa"):
    """Print and return the card's used VRAM (MiB) at a named stage.

    MEASURE AS YOU GO. The first assembled row filled a 16 GB card (14.3 GB)
    and the 1 km x 1 km scene this feeds has to fit a 5090 (32 GB) / RTX PRO
    5000 (48 GB) with every other component running — "at the current rate
    those would also have been exceeded" (user, 2026-08-30). Four readings
    per run — empty stage, geometry composed, Flow up, after the captures —
    give the per-building content cost and the Flow cost separately, which is
    what a projection needs.

    `prefix` only names the printing launcher (`fa` the row, `fc` the city);
    the row launcher's default keeps its lines exactly as they were.
    """
    import subprocess
    try:
        out = subprocess.run(["nvidia-smi", "--query-gpu=memory.used,memory.total",
                              "--format=csv,noheader,nounits"],
                             capture_output=True, text=True, timeout=10).stdout
        used, total = [float(x) for x in out.strip().split("\n")[0].split(",")[:2]]
    except Exception as exc:                        # no nvidia-smi in the image?
        print("[{0}] VRAM {1}: unavailable ({2})".format(prefix, tag, exc))
        return None
    print("[{0}] VRAM {1}: {2:.0f} / {3:.0f} MiB".format(prefix, tag, used, total))
    return used


def resolve_bakes(spec, pattern="*.usd"):
    """`FA_BAKES` -> an ordered list of `(usd, json)` pairs.

    A directory is globbed; a comma list is taken as given. A `.usd` with no
    sidecar is still ASSEMBLED (its geometry is complete) but gets no
    emitters and says so — that is a bake whose export succeeded and whose
    sidecar write did not, and dropping the building would hide it.
    """
    out = []
    for item in [q.strip() for q in str(spec).split(",") if q.strip()]:
        if os.path.isdir(item):
            found = sorted(_glob.glob(os.path.join(item, pattern)))
        else:
            found = [item]
        for u in found:
            j = os.path.splitext(u)[0] + ".json"
            out.append((u, j if os.path.exists(j) else ""))
    seen, uniq = set(), []
    for u, j in out:
        if u in seen:
            continue
        seen.add(u)
        uniq.append((u, j))
    return uniq


def order_bakes(rows, order=()):
    """Column order: `FA_ORDER` first if given, then each sidecar's own
    `index` (the manifest position it was baked from), then the file name."""
    if order:
        rank = {s: i for i, s in enumerate(order)}
        return sorted(rows, key=lambda r: (
            rank.get(os.path.splitext(os.path.basename(r["usd"]))[0], 10 ** 6),
            r["doc"].get("index", 0) if r["doc"] else 0,
            os.path.basename(r["usd"])))
    return sorted(rows, key=lambda r: (
        r["doc"].get("index", 0) if r["doc"] else 0,
        os.path.basename(r["usd"])))


# ---------------------------------------------------------------------------
# Ground and light — the benches' own seat
# ---------------------------------------------------------------------------
def build_ground_and_light(stage, span, prefix="fa"):
    """Pavement-grey ground and a LOW warm key (25 deg).

    Copied from `urban_fire_bench` / `gac_fire_bench` on purpose: char is
    0.15 on screen and a spall scar 0.44, so under a flat overhead key the
    whole elevation crushes to black and none of the plume structure
    survives. A raking sun separates the tongues and the scars — and it is
    also what a drone flies in.

    NOT used by the CITY launcher: a generated city brings its own ground
    (`apply_ground_planes`) and its own sky (`add_sky`/`resolve_sky`), and a
    second 400 m quad at z=0 would z-fight the road surface.
    """
    import scene_generator as sg

    e = max(400.0, span * 1.4)
    ground = UsdGeom.Mesh.Define(stage, Sdf.Path("/World/ground"))
    ground.CreatePointsAttr([Gf.Vec3f(-e, -e, 0.0), Gf.Vec3f(e, -e, 0.0),
                             Gf.Vec3f(e, e, 0.0), Gf.Vec3f(-e, e, 0.0)])
    ground.CreateFaceVertexCountsAttr([4])
    ground.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    ground.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
    ground.CreateDisplayColorAttr([Gf.Vec3f(0.30, 0.30, 0.29)])
    ground.CreateExtentAttr([Gf.Vec3f(-e, -e, 0.0), Gf.Vec3f(e, e, 0.0)])
    try:
        mp = stage.DefinePrim(Sdf.Path("/World/Looks/pavement"))
        mp.GetReferences().AddReference(sg._join_asset_root(
            "airstack://scene_gen/assets/materials/megascans/Road_Asphalt.usda", ""))
        mp.Load()
        m = UsdShade.Material.Get(stage, "/World/Looks/pavement")
        if m:
            UsdShade.MaterialBindingAPI(ground.GetPrim()).Bind(m)
    except Exception as exc:
        print("[{0}] ground material unavailable: {1}".format(prefix, exc))
    dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/domeLight"))
    dome.CreateIntensityAttr(700.0)
    dome.CreateColorAttr(Gf.Vec3f(0.72, 0.76, 0.86))
    key = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/keyLight"))
    key.CreateIntensityAttr(3200.0)
    key.CreateAngleAttr(0.9)
    key.CreateColorAttr(Gf.Vec3f(1.0, 0.94, 0.86))
    key.AddRotateXYZOp().Set(Gf.Vec3f(-25.0, 0.0, 28.0))


def bbox(stage, path):
    """World-aligned `[x0, y0, z0, x1, y1, z1]` of `path`, or `None`.

    `useExtentsHint=False` on purpose — see `fix-floating-debris`: an
    extents hint is authored data and can be stale, which is exactly how
    airborne wood once audited as clean.
    """
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                           [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
                           useExtentsHint=False)
    prim = stage.GetPrimAtPath(Sdf.Path(path))
    if not prim or not prim.IsValid():
        return None
    r = bc.ComputeWorldBound(prim).ComputeAlignedRange()
    if r.IsEmpty():
        return None
    lo, hi = r.GetMin(), r.GetMax()
    return [float(lo[0]), float(lo[1]), float(lo[2]),
            float(hi[0]), float(hi[1]), float(hi[2])]


# ---------------------------------------------------------------------------
# The fire, put back
# ---------------------------------------------------------------------------
def _sphere_source(stage, path, seat, state, scale, vel, dx, dy, top_z, *,
                   radius_scale=1.0):
    """One smoke-only `FlowEmitterSphere` at a recorded seat.

    The seat came out of the bake already clamped to the settled geometry;
    `top_z` here is what THIS stage measures on the referenced building, so a
    reference that composed differently (or not at all) cannot leave a plume
    in mid-air.

    `radius_scale` is `place_fire`'s `smoke_scale` knob (2026-08-31, "increase
    amount of smoke" — user review of the live 500 m fire city): it widens
    the seat's own recorded radius. Keyword-only and additive on purpose —
    `_sphere_source`'s positional signature is frozen by
    `test_urban_fire_city_launch.EXPECTED_SIGNATURES` (an `ast`-level check
    against `fn.args.args`, which does not see keyword-only parameters), so
    every existing caller that does not pass it keeps byte-identical output.
    """
    prim = fx._flow_create(stage, path, "FlowEmitterSphere")
    if not prim or not prim.IsValid():
        return 0
    z = float(seat["z"])
    if top_z is not None:
        z = min(z, float(top_z) - 0.4)
    fx._set(prim, "layer", Sdf.ValueTypeNames.Int, int(fx.FLOW_LAYER))
    fx._set(prim, "position", Sdf.ValueTypeNames.Float3,
            Gf.Vec3f(float(seat["x"]) + dx, float(seat["y"]) + dy, z))
    fx._set(prim, "radius", Sdf.ValueTypeNames.Float,
            float(seat.get("radius", 1.2)) * float(radius_scale))
    fx._set(prim, "radiusIsWorldSpace", Sdf.ValueTypeNames.Bool, True)
    fx._set(prim, "coupleRateSmoke", Sdf.ValueTypeNames.Float, 2.0)
    fx._set(prim, "velocity", Sdf.ValueTypeNames.Float3, Gf.Vec3f(*vel))
    fx._set(prim, "velocityIsWorldSpace", Sdf.ValueTypeNames.Bool, True)
    fx.set_emission(prim, state, scale=float(seat.get("scale", 1.0)) * scale)
    return 1


# ---------------------------------------------------------------------------
# EMITTER POSITIONS SPREAD ACROSS SIDES AND OPENINGS (2026-08-31)
# ---------------------------------------------------------------------------
# "Live fire seems to mostly only stay on 1 side of the building... also more
# positions on each side" (user review of the live 500 m fire city). THE BUG:
# every per-opening budget loop in `place_fire` read `for ev in <events of one
# state>: for op in ev["ops"]: if n_open >= cap: break` — a bare `break` only
# escapes the INNER loop, so once the cap is spent the outer loop still walks
# every remaining event, immediately re-breaks on its own first opening, and
# never gives them anything. `soot_plume.plan_events` emits one EVENT per
# (side, storey) run, so "whichever event sorts first" is "whichever side's
# run was planned first" — the fire never got past it, regardless of how many
# sides the PLAN actually carried.
#
# THE FIX DOES NOT CHANGE THE TOTAL SELECTED. `_round_robin` (and the two
# selectors built on it below) visit the exact same flat set of candidates
# the old code did, only re-ordered, and every caller still stops at the same
# cap — so the emitter-budget accounting downstream
# (`urban_fire_city_launch_script.emitter_estimate`, which counts
# `min(cap, total candidates)` without caring which ones) stays correct with
# no change on its side. Applied to every event-ranked branch that could
# otherwise cluster on one side: the flame-opening budget, F4's own
# reduced-intensity top-up, the active-fire vent-smoke picks, the burnt-out
# smoulder-smoke picks, and the F1 wisp pair — an F5/F5c building's fire is
# ENTIRELY these smoke picks (soot_plume never plans a `"flame"`-state event
# once the building has reached "residual"), so leaving them out would have
# left exactly the buildings the review screenshots showed still one-sided.
def _spread_order(n):
    """Indices `0..n-1` visited so every PREFIX samples roughly evenly across
    the range — recursive bisection, level order: the middle first, then the
    middle of each half, and so on — instead of left-to-right.

    A `soot_plume` flame RUN is a physically contiguous strip of openings;
    taking its first `k` left-to-right lights only one end of it. A budget
    that only fits 2 of a 6-opening run should show two spread-out windows,
    not the leftmost pair.
    """
    if n <= 0:
        return []
    order = []
    queue = [(0, n - 1)]
    while queue:
        lo, hi = queue.pop(0)
        if lo > hi:
            continue
        mid = (lo + hi) // 2
        order.append(mid)
        queue.append((lo, mid - 1))
        queue.append((mid + 1, hi))
    return order


def _round_robin(groups, weights=None):
    """Flatten a list of lists round-robin: item 0 of every group in turn,
    then item 1 of every group that still has one, and so on. Each group's
    own internal order is preserved — only the INTERLEAVING changes.

    `weights` (2026-08-31, street-facing bias — see `street_side_ranks`),
    when given, is a per-group REPEAT COUNT consumed within one round before
    the round moves to the next group: a group with weight 2 gives up two of
    its own items for every one item every weight-1 group gives up, so it
    empties roughly twice as fast without ever starving another group to
    zero (every group still gets exactly one turn per round, weight >= 1).
    `None` (every caller before 2026-08-31) is exactly weight 1 for every
    group — the ORIGINAL shared-index interleave, byte-identical output:
    each group's own index only ever advances by the same one step per round
    a shared index would have, so a caller that never passes `weights` sees
    no change at all.
    """
    out = []
    idx = [0] * len(groups)
    active = True
    while active:
        active = False
        for gi, g in enumerate(groups):
            w = 1 if not weights else max(1, int(weights[gi]))
            for _ in range(w):
                if idx[gi] < len(g):
                    out.append(g[idx[gi]])
                    idx[gi] += 1
                    active = True
    return out


def _group_by_side(events):
    """`(sides in first-seen order, {side: [event, ...]})` — the grouping
    both round-robin selectors below share."""
    by_side, order_sides = {}, []
    for ev in events:
        side = ev.get("side", "S")
        if side not in by_side:
            by_side[side] = []
            order_sides.append(side)
        by_side[side].append(ev)
    return order_sides, by_side


def _side_weights(order_sides, street_bias_side, bias_weight):
    """Per-`order_sides`-index weight for `_round_robin`: `bias_weight` (an
    int repeat count, floored at 1) for `street_bias_side` if it is one of
    `order_sides`, 1 for every other side. `None` (no bias) when
    `street_bias_side` is falsy or not among `order_sides` — an unbiased
    caller's `_round_robin` call then stays on its exact old, unweighted
    path (see `_round_robin`'s own docstring).

    2026-08-31, SECOND REVIEW ("street facing fires all seem to be in the
    top left. I want it randomized"): this used to compute the boosted side
    itself, as the single ARGMAX of a `{side: score}` dict — deterministic,
    so every building whose open side happened to face the same compass
    direction (common along one axis of a generated grid; `street_side_
    score`'s neighbour-clearance term also returns the identical 200 m
    sentinel for "nothing within 200 m", so ties across a whole district are
    the NORM, not an edge case) boosted that SAME direction, and a whole
    district read as venting one way. `choose_street_side` now makes that
    call ONCE, per building, seeded off something that varies building to
    building — this function only APPLIES whichever side it chose; it is no
    longer where the choice is made, which is what lets a caller reuse the
    exact same choice across every round-robin call for one building
    (flame, smoke, wisp) without importing extra state.
    """
    if not street_bias_side or street_bias_side not in order_sides:
        return None
    w = max(1, int(bias_weight))
    return [w if s == street_bias_side else 1 for s in order_sides]


def _flame_selection_order(evs_of_state, street_bias_side=None, bias_weight=2):
    """`(ev, op)` pairs across one building's events of a single fire STATE
    (all `"flame"`, or all `"smoulder"`), ordered so a `max_open`-limited
    budget lights more than one side and more than one spot on each side.

    ROUND-ROBIN ACROSS SIDES FIRST: events are grouped by `ev["side"]`, one
    queue per side in first-seen order. ROUND-ROBIN ACROSS STOREYS WITHIN A
    SIDE SECOND: a side with several events (one per burning storey) cycles
    through them rather than draining the first storey's run before moving
    to the next. Openings WITHIN one event are visited in `_spread_order`
    rather than left-to-right, so a partially-lit run samples across its own
    span too. Fully deterministic given the same bake (sides by first
    appearance, events within a side by `id`), so a repeat run against the
    same sidecar and the same `max_emitters` selects the exact same
    openings.

    `street_bias_side`/`bias_weight` (2026-08-31, "I want more of the
    actual 'fire' elements on buildings, especially on street facing
    sides"; RANDOMISED per building as of the same day's second review —
    see `choose_street_side`/`_side_weights`): when `street_bias_side` names
    one of `evs_of_state`'s own sides, it gets `bias_weight` turns per round
    instead of 1 — MORE of a budget-limited selection lands there — while
    every other side still gets its one turn per round, so a building whose
    venting wraps a corner never goes back to showing fire on only one side
    (the ORIGINAL round-robin fix, above, still holds as a floor).
    """
    order_sides, by_side = _group_by_side(evs_of_state)
    side_seqs = []
    for side in order_sides:
        evs_here = sorted(by_side[side], key=lambda e: e["id"])
        event_seqs = [[(ev, ev["ops"][k]) for k in _spread_order(len(ev["ops"]))]
                     for ev in evs_here]
        side_seqs.append(_round_robin(event_seqs))
    return _round_robin(side_seqs, _side_weights(order_sides, street_bias_side,
                                                 bias_weight))


def _select_events_by_side(events_in_priority_order, budget,
                           street_bias_side=None, bias_weight=2):
    """The first `budget` events out of `events_in_priority_order`,
    round-robin across `ev["side"]` rather than taken in the caller's raw
    (usually storey- or id-ranked) order — the same one-sided-cluster fix as
    `_flame_selection_order`, for the F1-wisp pick that selects whole
    EVENTS (one source each) instead of individual openings. Each side's own
    internal ranking is exactly the order the caller passed in.

    `street_bias_side`/`bias_weight` — see `_flame_selection_order`: the
    same street-facing bias, applied to a whole-event pick instead of a
    per-opening one.
    """
    order_sides, by_side = _group_by_side(events_in_priority_order)
    flat = _round_robin([by_side[s] for s in order_sides],
                        _side_weights(order_sides, street_bias_side,
                                     bias_weight))
    return flat[:max(0, int(budget))]


# ---------------------------------------------------------------------------
# STREET-FACING BIAS — deriving it from the CITY DUMP (2026-08-31)
# ---------------------------------------------------------------------------
# "I want more of the actual 'fire' elements on buildings, especially on
# street facing sides so it's visible" (user, 2026-08-31 review). Which of a
# burning building's `sides` opens onto a real street is not new information
# — `tools/fire_city_union.py`'s `street_facing_metrics` already scores a
# manifest against this exact metric, `disaster.urban_fire_spread.
# street_side_score` — but that tool needs the layout's block rects (to
# score the SECOND term, "how far to the edge of the block") on top of the
# dump's own placement geometry, and this launcher never loads the
# layout's `_typology_of` rects from a file (`generate_scene_on_stage`
# builds a fresh one every run, it does not read one back). NEIGHBOUR
# CLEARANCE ALONE — `street_side_score(..., block_rect=None)`, "score on
# neighbour clearance alone" per its own docstring — needs nothing but
# positions, and is already that metric's dominant term (a side with a nice
# open block edge but a building 2 m away still reads as boxed in). The city
# placements dump (`FC_DUMP`, `urban_fire_city_launch_script.
# dump_city_placements`'s own schema) already has every placement's real
# measured `x_m`/`y_m`/`W`/`D`/`yaw_deg` — exactly `street_side_score`'s
# other required input — so this is a genuine reuse, not a reimplementation.
def load_dump_positions(path):
    """`{i: {"x", "y", "W", "D", "yaw"}}` for every placement recorded in a
    `fire_city_placements_dump.v1` file, keyed by the SAME global index
    (`house_<cell>_<i>`) a fire manifest record's own `i` names.

    Every neighbour counts here, burnable or not — an unburnable landmark
    still blocks a street view exactly as well as a burnable one does, and
    `street_side_score(block_rect=None)` only ever wants geometry — so this
    is a flat read of the dump's `placements` list, no typology/burnable
    gating (contrast `tools/fire_city_union.py`'s `_burnable_geometry`,
    which needs that gating for a DIFFERENT purpose — deciding which
    buildings the fire itself could have spread through).

    `{}` on a missing file, a bad schema, or any parse error — a caller that
    cannot get a street bias falls back to the original unweighted
    round-robin (`street_side_ranks` below returns `{}` too), it does not
    fail the build over a missing/stale dump path.
    """
    try:
        with open(path) as fh:
            doc = json.load(fh)
        out = {}
        for p in doc.get("placements") or []:
            i = p.get("i")
            if i is None:
                continue
            out[int(i)] = {"x": float(p["x_m"]), "y": float(p["y_m"]),
                           "W": float(p.get("W") or 1.0),
                           "D": float(p.get("D") or 1.0),
                           "H": float(p.get("H") or 0.0),
                           "yaw": float(p.get("yaw_deg") or 0.0)}
        return out
    except Exception:
        return {}


def street_side_ranks(positions, i, sides):
    """`{side: score}` for every `side` in `sides` — higher means more
    likely to open onto a street than onto a neighbour's wall, per
    `disaster.urban_fire_spread.street_side_score(..., block_rect=None)`
    (neighbour clearance only — see the section docstring above).

    `{}` if `i` is not in `positions` or `sides` is empty, so a caller
    (`_side_weights`) sees the SAME "no bias" signal a missing/stale dump
    gives — never a fabricated zero score for every side, which would look
    exactly like "no side is street-facing" instead of "unknown".
    """
    b = positions.get(int(i)) if positions else None
    if not b or not sides:
        return {}
    from . import urban_fire_spread as ufs

    others = [v for k, v in positions.items() if k != int(i)]
    return {s: ufs.street_side_score(b, s, others) for s in sides}


#: `street_side_score`'s own neighbour-clearance term returns exactly
#: `urban_fire_spread.STREET_SCORE_CAP_M` (200.0) for "no neighbour within
#: 200 m at all" on ANY compass direction that happens to be true for — no
#: real city block on this plate is 200 m from its nearest neighbour, so
#: that sentinel is not a genuine distance, it is "unmeasurably open". Left
#: unclamped it would swamp every side's own honest (much smaller) opening
#: in a proportional weighting; clamped here to something an actually
#: generous block edge on `city_138` could plausibly score.
STREET_BIAS_SCORE_CEILING = 60.0


def choose_street_side(street_rank, seed):
    """ONE side out of `street_rank`, chosen at RANDOM — weighted by score
    (clamped at `STREET_BIAS_SCORE_CEILING` first, so the 200 m sentinel
    above cannot dominate the draw), never the deterministic argmax.

    `seed` MUST be something STABLE per building — the manifest record's
    own global dump index, or its bake `stem` — and must NEVER be the row's
    enumerate position in whatever list this run happens to be iterating;
    `random.Random` accepts any hashable, so a formatted string
    (`"{FA_SEED}-{stem}-street"`) is as good a seed as an int.

    2026-08-31, SECOND REVIEW: "street facing fires all seem to be in the
    top left. I want it randomized." MEASURED CAUSE — the OLD code
    (`_side_weights` computing its own argmax) always picked the single
    highest-scoring side, and because `street_side_score`'s neighbour-
    clearance term returns the SAME 200.0 sentinel for "nothing within
    200 m" regardless of WHICH compass direction is open, every building in
    a district whose open side happens to face, say, north (a common
    pattern along one axis of a generated grid — city blocks share an
    orientation) boosted north, and a whole district read as venting one
    way ("top left" in the reviewed capture). A per-building WEIGHTED
    RANDOM CHOICE, seeded off something that varies building to building,
    breaks that correlation — two buildings with the IDENTICAL score
    pattern (a common case: both capped at the ceiling on the same set of
    sides) now draw independently, spreading the boosted bearing across the
    whole district — while still favouring a genuinely more-open side over
    a boxed-in one on average, because the boxed-in side's weight is zero.

    `None` when `street_rank` is falsy or every side scores <= 0 (nothing
    to prefer — the same "no bias" signal `_side_weights` always used).
    """
    positive = {s: min(float(v), STREET_BIAS_SCORE_CEILING)
               for s, v in (street_rank or {}).items() if v > 0.0}
    if not positive:
        return None
    sides = sorted(positive.keys())            # stable order before drawing
    weights = [positive[s] for s in sides]
    return random.Random(seed).choices(sides, weights=weights, k=1)[0]


#: the two ladder levels that actually drop the roof deck: F5c
#: (`fire_collapse`'s own elevation/corner-loss level) and F6 (the GAC
#: full-collapse label). Every OTHER level, however badly burnt, still has
#: a standing roof to smoke off of — F5/"residual" is a burnt-OUT shell,
#: not a collapsed one.
ROOF_COLLAPSE_LEVELS = ("F5c", "F6")
#: `top_z - deck_z` below this, metres, reads as "no parapet/roof band left
#: standing above the deck" — see `roof_has_collapsed`. Every intact-roof
#: bake in `city_138` measured a 3.3-11.9 m gap here (the parapet coping),
#: so this margin is well clear of that range without being so loose it
#: would ever call a genuinely intact roof collapsed.
ROOF_COLLAPSE_MARGIN_M = 1.5


def roof_has_collapsed(doc):
    """Best-effort "is there still a roof deck to smoke off of", from
    EXISTING sidecar fields only — no new one (a new field would force
    re-baking every building this runs against).

    `doc["fire"]["level"]` in `ROOF_COLLAPSE_LEVELS` is checked first and is
    exact for BOTH bake families. Failing that, a GAC bake also carries a
    measured `deck_z` (the real roof SURFACE, `gac_fire.mass_from_grid`,
    distinct from the parapet-coping bbox top) and the baker's own measured
    `top_z` (the SETTLED geometry's composed top, `fire_bake`'s "SMOKE ON
    WHAT IS LEFT, NOT ON WHAT WAS PLANNED" pass) — if the gap between them
    has closed to near zero the parapet/roof band above the deck is no
    longer standing, a real (if indirect) collapse signal a kit bake cannot
    give (`fire_bake.mass_to_json`: "`None` for a kit mass, which never
    carries one" — kit buildings fall through to the level check alone,
    which is exact for the two levels that use it).
    """
    f = doc.get("fire") or {}
    if f.get("level") in ROOF_COLLAPSE_LEVELS:
        return True
    deck_z = f.get("deck_z")
    if deck_z is None:
        mtag = f.get("mass") or "main"
        deck_z = ((doc.get("masses") or {}).get(mtag) or {}).get("deck_z")
    top_z = doc.get("top_z")
    if deck_z is not None and top_z is not None:
        return (float(top_z) - float(deck_z)) < ROOF_COLLAPSE_MARGIN_M
    return False


#: "more actual places of fire on each building" (2026-08-31, second
#: review). At least this many DISTINCT `(side, storey)` groups get a flame
#: source when the building's own live "flame"-state events don't already
#: span that many — see the extra-cluster loop in `place_fire`.
FLAME_MIN_CLUSTERS = 3
#: cap on how many extra (dimmer, `"out"`-event-sourced) picks the
#: cluster-diversity top-up may add, so a building with lots of budget
#: headroom does not have this alone double its flame count.
FLAME_EXTRA_MAX = 3

# ---------------------------------------------------------------------------
# SIZE-SCALED ALLOCATION (2026-08-31, third review, item 4): "If the smaller
# building can have 5-7 fire windows, the bigger one should have more." A
# 3-4 storey shop and a 28-storey tower were being priced by the SAME
# opening budget (`FA_EMITTERS`) modulated only by a coarse >=12-storey
# floor (`min(16, n_storeys // 2)`) — a 25-storey building and a 13-storey
# one landed on nearly the same count. `flame_window_target`/
# `smoke_window_target` replace that with a straight line in `n_storeys`,
# clamped at both ends: floored so even a tiny building still reads as
# genuinely on fire, ceilinged so a supertall tower cannot alone blow the
# global emitter budget. Opt-in (`place_fire`'s `flame_size_scaling`/
# `smoke_size_scaling`, both `False` by default) — the row/bench launcher,
# which never passes either, keeps its exact original formula.
FLAME_WINDOWS_BASE = 3.0
FLAME_WINDOWS_PER_STOREY = 1.0
FLAME_WINDOWS_MIN = 5
FLAME_WINDOWS_MAX = 30
SMOKE_WINDOWS_BASE = 2.0
SMOKE_WINDOWS_PER_STOREY = 0.5
SMOKE_WINDOWS_MIN = 3
SMOKE_WINDOWS_MAX = 16


def _clamp_round(base, per_storey, n_storeys, lo, hi):
    n = max(0, int(n_storeys or 0))
    return max(lo, min(hi, int(round(base + per_storey * n))))


def flame_window_target(n_storeys):
    """How many flame-carrying openings a building of this height "should"
    have, before any budget rationing — `clamp(round(FLAME_WINDOWS_BASE +
    FLAME_WINDOWS_PER_STOREY * n_storeys), FLAME_WINDOWS_MIN,
    FLAME_WINDOWS_MAX)`. A 3-storey shop lands at 6, a 25+-storey tower
    clamps at the 30 ceiling."""
    return _clamp_round(FLAME_WINDOWS_BASE, FLAME_WINDOWS_PER_STOREY,
                        n_storeys, FLAME_WINDOWS_MIN, FLAME_WINDOWS_MAX)


def smoke_window_target(n_storeys):
    """The window-smoke counterpart of `flame_window_target` — smaller
    base/slope/ceiling, since smoke is a secondary read next to the fire
    itself, but scaling "the same way" per the review."""
    return _clamp_round(SMOKE_WINDOWS_BASE, SMOKE_WINDOWS_PER_STOREY,
                        n_storeys, SMOKE_WINDOWS_MIN, SMOKE_WINDOWS_MAX)


# ---------------------------------------------------------------------------
# RESIDUAL FLAME POCKETS (2026-08-31, third review, item 5) — the user's
# headline case: a 134 m, 28-storey RESIDUAL (F5) tower with 75 baked events
# (72 "out", 3 "smoulder") showed ZERO flame, because only `state ==
# "flame"` (F2/F3) ever reached the primary loop and the F4-only top-up
# never covered F5/F5c/F6 at all. "The user wants fire on its windows" —
# reduced, SCATTERED, single-window pockets (never a contiguous run, which
# reads as an active compartment fire and belongs to F2/F3) across the
# burnt band. `RESIDUAL_FLAME_SCALE` dims each pocket below even the F4
# top-up's own 0.6 factor: these are embers in a shell that has already
# burned through, not a fire still eating fuel.
RESIDUAL_FLAME_SCALE = 0.55
#: a residual target below this many openings does not read as "pockets of
#: fire" at all, however small the building.
RESIDUAL_FLAME_MIN = 2


def _scattered_selection_order(events, street_bias_side=None, bias_weight=2):
    """`(ev, op)` pairs, AT MOST ONE PER EVENT — the residual-pocket
    counterpart to `_flame_selection_order`.

    An `soot_plume` EVENT is, by construction, one physically CONTIGUOUS
    run at one `(mass, side, storey)` — taking more than one opening from
    the same event is indistinguishable from an active compartment fire
    (`_flame_selection_order`'s own job). Taking AT MOST one — its own
    `_spread_order`-picked middle opening, so a lone pocket sits mid-run
    rather than jammed against either raw edge — guarantees no two chosen
    windows ever share a run, which is what keeps a scattered allocation
    from reading as a second (or third) contiguous fire. Round-robins
    across sides (and the caller's already-CHOSEN `street_bias_side`, see
    `choose_street_side`) exactly like every other selector here, so a
    building with events on several elevations gets pockets spread across
    them, not all on whichever side's events happened to come first.
    """
    order_sides, by_side = _group_by_side(events)
    side_seqs = []
    for side in order_sides:
        evs_here = sorted(by_side[side], key=lambda e: e["id"])
        one_per_event = [(ev, ev["ops"][_spread_order(len(ev["ops"]))[0]])
                         for ev in evs_here if ev.get("ops")]
        side_seqs.append(one_per_event)
    return _round_robin(side_seqs, _side_weights(order_sides, street_bias_side,
                                                 bias_weight))


# ---------------------------------------------------------------------------
# CONTACT SNAP — the fire must touch the building (2026-08-31)
# ---------------------------------------------------------------------------
# "some of the fires seem to be floating outside the building... surely
# there's a test to see if the fire is touching the building and we make it
# go inside the building by some amount" (user, re:
# `gac_SM_Building_11_F4_o18_SNW_s374/bake/bake/g6/pieces/wall_x_0_00_0000`).
#
# MEASURED CAUSE. `gac_fire.window_rects` only fills `planes[side]` — the
# real, measured façade plane — from actual GLASS faces. A building with NO
# real glazing on a side (or, like SM_Building_11, none anywhere) never gets
# an entry, and `gac_fire.side_frame(m, side, planes)` then falls back to the
# MASS BBOX face (`cx - W/2`, etc.) for that side's whole opening frame —
# including every SYNTHETIC bay-window `openings_provider` invents to cover a
# starved band (see the skill's "event starvation" note). The mass box is
# measured from the merged asset's overall extent (`gac_fire.mass_from_grid`)
# and is the outer envelope of whatever protrudes furthest — a parapet, a
# cornice, a stepped-back upper massing, an L-shaped notch the box just spans
# — so a bbox-face frame can sit metres off the real wall. Every downstream
# consumer (`urban_fire._flame_sources`, the interior/roof plume seats are
# UNAFFECTED — they come from measured floor slabs, not this) trusts the
# frame's plane completely, so the fire hangs in the air exactly that far off
# the building.
#
# THE FIX IS AT ASSEMBLY TIME, NOT AT BAKE TIME — no re-bake, and it also
# repairs every bake already on disk. `snap_events_to_geometry` tests each
# opening's assumed wall point against the building's own COMPOSED geometry
# (real triangles, never `UsdGeom.BBoxCache` — see `fix-floating-debris`) and
# corrects `op["out"]` — the one scalar `_flame_sources` already adds
# `FLAME_OUT` to — so every downstream consumer (flame, window-jet smoke, the
# residual pockets) is fixed by construction, with zero change to
# `urban_fire._flame_sources` itself.
#: an opening's assumed wall point within this of real geometry, along its
#: own outward normal, is already "touching" — left untouched (a genuine
#: real-glazing building measures ~0 here; see the module docstring for the
#: healthy-control expectation).
SNAP_TOL_M = 0.3
#: how far past the found real surface a snapped opening's plane is pulled,
#: back INTO the building — "go inside the building by some amount" (user).
SNAP_INSET_M = 0.2
#: how far in front of (outward from) / behind (inward of) the assumed wall
#: plane the contact ray searches before giving up. `_in` reaches toward the
#: building's own interior — deep enough to cross a genuinely inset façade —
#: `_out` reaches away from it, in case the mass box UNDERSHOT the real wall
#: (a canopy, a bay window) rather than overshot it.
SNAP_REACH_IN_M = 3.0
SNAP_REACH_OUT_M = 6.0


def bake_geometry_root(holder, doc):
    """Where a bake's own geometry composes once referenced under a fresh
    holder (`urban_fire_city_launch_script.place_holder`: the reference
    always lands on `<holder>/bake`, a prim with no path of its own inside
    the referenced file).

    Every bake this codebase writes has `default_prim="/World"` and
    `root="/World/bake"` (`fire_bake.sidecar`) — `root` is always a CHILD of
    `default_prim` — so referencing the file onto `<holder>/bake` composes
    that child's own subtree AT `<holder>/bake` + (root's path relative to
    `default_prim`), i.e. `<holder>/bake/bake` in practice. Computed from the
    sidecar's own fields rather than hardcoded, so a bake with a different
    `root` still resolves correctly.

    `holder` falsy — a caller diagnosing/verifying a bake FILE opened
    directly as its own stage, where `default_prim` IS the stage's real root
    and there is no reference indirection to account for — returns
    `doc["root"]` unchanged.
    """
    root = doc.get("root") or "/World/bake"
    if not holder:
        return root
    default_prim = doc.get("default_prim") or "/World"
    rel = root[len(default_prim):] if root.startswith(default_prim) else root
    return holder + "/bake" + rel


def _gather_world_triangles(stage, root):
    """World-space `(V, F)` — an (N, 3) point array and an (M, 3) int64
    triangle-index array — for every ACTIVE, VISIBLE Mesh under `root`.

    POINTS-BASED, NEVER `UsdGeom.BBoxCache` (see `fix-floating-debris`: a
    bounding-box test is exactly the blind spot that let airborne wood audit
    as clean). This is `fire_bake._judge_candidates`'s own triangle-soup
    recipe, minus the per-candidate ownership bookkeeping that pass needs and
    a plain contact test does not. Invisible geometry is skipped for the same
    reason `_judge_candidates` skips it: at assembly/diagnosis time a
    referenced bake's own hidden `<cell>/src` (or, in a live city stage, the
    hidden intact cell behind it) would otherwise read as real support.
    `(None, None)` if `root` does not exist or carries no usable mesh.
    """
    import numpy as np
    from pxr import Usd

    from . import soot_bake as sb

    rp = stage.GetPrimAtPath(Sdf.Path(root))
    if not rp or not rp.IsValid():
        return None, None
    xf = UsdGeom.XformCache()
    all_pts, all_tris, npts = [], [], 0
    for p in Usd.PrimRange(rp):
        if not p.IsA(UsdGeom.Mesh) or not p.IsActive():
            continue
        try:
            if UsdGeom.Imageable(p).ComputeVisibility() == UsdGeom.Tokens.invisible:
                continue
        except Exception:
            pass
        mesh = UsdGeom.Mesh(p)
        pts = mesh.GetPointsAttr().Get()
        if not pts:
            continue
        counts = mesh.GetFaceVertexCountsAttr().Get()
        idx = mesh.GetFaceVertexIndicesAttr().Get()
        if not counts or not idx:
            continue
        M = np.array(xf.GetLocalToWorldTransform(p), dtype=float)
        W = np.asarray(pts, dtype=float) @ M[:3, :3] + M[3, :3]
        tri, _tf, _ts = sb.triangles(np.asarray(counts), np.asarray(idx))
        if len(tri):
            all_tris.append(tri + npts)
        all_pts.append(W)
        npts += len(W)
    if not all_pts or not all_tris:
        return None, None
    return np.vstack(all_pts), np.vstack(all_tris)


def build_contact_locator(stage, geom_root, verbose=False):
    """One `vtkStaticCellLocator` over every triangle composed under
    `geom_root` — built ONCE per building. The caller (`place_fire`, via
    `snap_events_to_geometry`) calls this exactly once and reuses it for
    every opening's contact test, which is what keeps ~900 emitters
    citywide cheap at launch (`fire_bake._judge_candidates` measured 0.05 s
    to build the same locator type over the LARGEST bake's 414,538
    triangles — a per-building cost this pays once, not per emitter).
    `None` if `vtk` is unavailable or the subtree has no usable geometry.
    """
    from . import fracture

    if not fracture.ensure_vtk(verbose=verbose):
        if verbose:
            print("[fa] contact snap: vtk unavailable — skipping "
                  "({0})".format(geom_root))
        return None

    import numpy as np
    import vtk
    from vtk.util import numpy_support as ns

    V, F = _gather_world_triangles(stage, geom_root)
    if V is None or not len(V) or F is None or not len(F):
        if verbose:
            print("[fa] contact snap: no geometry under {0}".format(geom_root))
        return None
    vpts = vtk.vtkPoints()
    vpts.SetData(ns.numpy_to_vtk(np.ascontiguousarray(V), deep=True))
    ca = vtk.vtkCellArray()
    cells = np.hstack(
        [np.full((len(F), 1), 3, dtype=np.int64), F]).ravel()
    ca.ImportLegacyFormat(ns.numpy_to_vtkIdTypeArray(cells, deep=True))
    pd = vtk.vtkPolyData()
    pd.SetPoints(vpts)
    pd.SetPolys(ca)
    locator = vtk.vtkStaticCellLocator()
    locator.SetDataSet(pd)
    locator.BuildLocator()
    return locator


def _opening_test_ray(op):
    """`(point, outward)` for one opening: the WORLD point its assumed wall
    plane sits at (`quake_flow._b_face_pt(op["fr"], u, v, 0.0)` at the
    opening's own mid-`u`/mid-`v` — `out=0.0`, not `op["out"]`, because the
    -0.05 m recess baked into every `op["out"]` is noise next to what this
    test is checking for) and the unit outward normal
    (`quake_flow._outward(op["m"], op["side"])`) `contact_offset` searches
    along.
    """
    from . import quake_flow as qf

    hu0, hu1 = op.get("hua", op["ua"]), op.get("hub", op["ub"])
    hv0, hv1 = op.get("hva", op["va"]), op.get("hvb", op["vb"])
    u = 0.5 * (float(hu0) + float(hu1))
    v = 0.5 * (float(hv0) + float(hv1))
    x, y, z = qf._b_face_pt(op["fr"], u, v, 0.0)
    ox, oy = qf._outward(op["m"], op["side"])
    return (x, y, z), (float(ox), float(oy), 0.0)


def contact_offset(locator, op, reach_in_m=SNAP_REACH_IN_M,
                   reach_out_m=SNAP_REACH_OUT_M):
    """Signed distance, in metres along `op`'s OWN outward normal, from its
    assumed wall plane to the nearest real surface `locator` finds crossing
    the line through it — positive if the real wall is further OUT than
    assumed (the measured-plane bug: a mass-box face inset from the true
    façade), negative if it is further IN (the mass box undershot). `None`
    if nothing crosses within `reach_in_m` inward / `reach_out_m` outward of
    the assumed plane — a phantom opening (an L-notch the mass box spans,
    with no real wall anywhere near where a synthetic grid put a window).

    Every hit along the ray is considered, not just the first — the closest
    one TO THE ASSUMED PLANE, so a ray that has to cross the building's own
    far wall to reach `reach_out_m` (a thin tower, a short `reach_in_m`
    miss) is not fooled by a hit on the wrong side of the building.
    """
    import vtk

    (px, py, pz), (ox, oy, oz) = _opening_test_ray(op)
    p0 = (px - ox * reach_in_m, py - oy * reach_in_m, pz - oz * reach_in_m)
    p1 = (px + ox * reach_out_m, py + oy * reach_out_m, pz + oz * reach_out_m)
    ipts, icells = vtk.vtkPoints(), vtk.vtkIdList()
    locator.IntersectWithLine(p0, p1, 1e-6, ipts, icells)
    n = ipts.GetNumberOfPoints()
    if n == 0:
        return None
    best = None
    for i in range(n):
        hx, hy, hz = ipts.GetPoint(i)
        t = (hx - px) * ox + (hy - py) * oy + (hz - pz) * oz
        if best is None or abs(t) < abs(best):
            best = t
    return best


def snap_events_to_geometry(stage, geom_root, events, tol_m=SNAP_TOL_M,
                            inset_m=SNAP_INSET_M,
                            reach_in_m=SNAP_REACH_IN_M,
                            reach_out_m=SNAP_REACH_OUT_M, verbose=False):
    """Contact-test and correct every opening in `events` against the
    building's own composed geometry under `geom_root` — ONE
    `vtkStaticCellLocator` built here for the whole building
    (`build_contact_locator`), reused for every opening.

    Per opening: `contact_offset` measures `t`, the signed offset (metres,
    along the opening's own outward normal) from its assumed wall plane to
    the nearest real surface.

      * `t is None` (nothing within reach — a phantom off an L-notch): the
        opening is DROPPED (removed from its event's `ops`).
      * `abs(t) <= tol_m` (already touching, within noise): left COMPLETELY
        UNTOUCHED — this is what keeps a healthy real-glazing building's
        openings byte-identical (measured plane, `t` ~ 0 by construction).
      * otherwise: SNAPPED — a fresh `op` dict with `op["out"] = t -
        inset_m`, so `_flame_sources`'s own `op["out"] + FLAME_OUT` now
        measures from the REAL wall, pulled `inset_m` further inside it
        ("go inside the building by some amount", user) before the usual
        proud-of-the-wall push is added back on top.

    Non-mutating on its input: returns a NEW `events` list (each event a
    shallow copy with its own, possibly-shrunk `ops` list; an unmodified
    `op` is the SAME dict the caller passed in, a snapped one is a copy) —
    so a caller that reuses the same `events` object across two `place_fire`
    calls (every determinism test in `test_fire_emitter_distribution.py`
    does exactly this) never sees the first call's correction bleed into the
    second.

    Returns `(events, stats)` — `events` unchanged (same object) and
    `stats = {"tested": 0, "ok": 0, "snapped": 0, "dropped": 0,
    "worst_offset_m": 0.0, "locator": False}` when `geom_root` is falsy or
    no locator could be built (vtk unavailable, or the subtree is empty) —
    the caller (`place_fire`) then proceeds exactly as it always has, no
    snap applied, no opening dropped.
    """
    stats = {"tested": 0, "ok": 0, "snapped": 0, "dropped": 0,
             "worst_offset_m": 0.0, "locator": False}
    if not events or not geom_root:
        return events, stats
    locator = build_contact_locator(stage, geom_root, verbose=verbose)
    stats["locator"] = locator is not None
    if locator is None:
        return events, stats
    out_events = []
    for ev in events:
        new_ev = dict(ev)
        kept = []
        for op in ev.get("ops") or []:
            stats["tested"] += 1
            t = contact_offset(locator, op, reach_in_m, reach_out_m)
            if t is None:
                stats["dropped"] += 1
                continue
            stats["worst_offset_m"] = max(stats["worst_offset_m"], abs(t))
            if abs(t) <= tol_m:
                stats["ok"] += 1
                kept.append(op)
                continue
            new_op = dict(op)
            new_op["out"] = float(t) - float(inset_m)
            stats["snapped"] += 1
            kept.append(new_op)
        new_ev["ops"] = kept
        out_events.append(new_ev)
    if verbose:
        print("[fa] contact snap {0}: {1} tested, {2} ok, {3} snapped, "
              "{4} dropped, worst pre-fix offset {5:.2f} m".format(
                  geom_root, stats["tested"], stats["ok"], stats["snapped"],
                  stats["dropped"], stats["worst_offset_m"]))
    return out_events, stats


# ---------------------------------------------------------------------------
# NO FIRE AT THE EXTREME TOP UNLESS THE WINDOWS ARE REAL (2026-08-31)
# ---------------------------------------------------------------------------
# "avoid fires at the extreme top of buildings cause a lot of them don't
# have windows there and it just looks weird. So try to do 2-3rd last story
# as the max unless we're 100% sure about windows on the top floor" (user).
# "100% sure" maps exactly onto real vs. SYNTHETIC openings: a kit event
# comes from `quake_flow.describe`'s measured window table and a real-
# glazing GAC event from `gac_fire.window_rects`' measured islands — both
# sure, top floor included, unchanged. A synthetic one
# (`gac_fire._synthetic_side_rects`, the painted-window fallback — see the
# `SM_Building_11` case this whole session started from) is never sure by
# construction — `gac_fire.openings_provider` already keeps its OWN grid off
# the top `gac_fire.SYN_TOP_EXCLUDE_STOREYS` storeys at BAKE time; this is
# the ASSEMBLY-side backstop for a bake baked before that existed.
def is_synthetic_op(op):
    """Is this opening's data INVENTED (a `gac_fire._synthetic_side_rects`
    bay-window grid) rather than measured?

    `op["e"]["synthetic"]` when the sidecar carries it (every GAC bake since
    this fix — `fire_bake._E_KEYS`). Falls back to the pre-existing
    `"gac_window_synth"` vs `"gac_window"` `e["name"]` convention for a bake
    that predates the field — every bake in `city_138` today. A kit opening
    sets neither and always reads real."""
    e = op.get("e") or {}
    if "synthetic" in e:
        return bool(e.get("synthetic"))
    return str(e.get("name") or "").endswith("_synth")


def drop_top_storey_synthetic(events, n_storeys, top_exclude=None):
    """Remove every SYNTHETIC opening (`is_synthetic_op`) whose storey is
    above `gac_fire.max_synthetic_storey(n_storeys, top_exclude)` from
    `events` — a real/measured opening at any storey, including the top
    one, is never touched.

    Runs as a pre-pass over the FULL `events` list, exactly like
    `snap_events_to_geometry` (and typically right beside it) — every
    downstream consumer (the primary flame loop, the F4 top-up, the
    cluster-diversity top-up, the residual pockets, BOTH window-jet-smoke
    mechanisms) draws from these same, now-filtered `ev["ops"]`, so the
    fix reaches all of them for free with no per-branch code: "flame
    emitters never go on [an excluded] opening... window-jet smoke: apply
    the same rule to synthetic events only" falls out of filtering the
    shared data once.

    Non-mutating, same shape as `snap_events_to_geometry`: returns a NEW
    `events` list. `top_exclude` defaults to `gac_fire.SYN_TOP_EXCLUDE_
    STOREYS` (the same constant the bake-time synthesis itself now honours,
    so the assembly-side backstop agrees with what a fresh bake would have
    done). `(events, stats)` — `stats = {"tested", "dropped", "max_allowed"}`
    — `"tested"` counts only SYNTHETIC openings (a real one is never even
    checked against the storey cap), `"max_allowed"` is `None` when
    `n_storeys` is falsy (nothing to test against; `events` returned as
    given).
    """
    from . import gac_fire as gf

    stats = {"tested": 0, "dropped": 0, "max_allowed": None}
    if not events or not n_storeys:
        return events, stats
    top_exclude = (gf.SYN_TOP_EXCLUDE_STOREYS if top_exclude is None
                  else top_exclude)
    max_allowed = gf.max_synthetic_storey(int(n_storeys), top_exclude)
    stats["max_allowed"] = max_allowed
    out_events = []
    for ev in events:
        new_ev = dict(ev)
        kept = []
        for op in ev.get("ops") or []:
            if is_synthetic_op(op):
                stats["tested"] += 1
                st = int(op.get("storey", ev.get("storey", 0)))
                if st > max_allowed:
                    stats["dropped"] += 1
                    continue
            kept.append(op)
        new_ev["ops"] = kept
        out_events.append(new_ev)
    return out_events, stats


# ---------------------------------------------------------------------------
# SMOKE COMPLEMENTS THE FLAME VERTICALLY (2026-08-31)
# ---------------------------------------------------------------------------
# "have more smoke on lower floors so it looks like those have been burnt
# out if you're not putting fire there" (user). The side-smoke picks
# (`out_ranked`'s event-level middle-opening pick, or the opening-level
# window-jet pick) used to rank candidates by `(-storey, id)` — HIGHEST
# storey first, the same direction the flame budget itself already favours
# (a climbing fire's own hottest, most-recently-lit band). Two mechanisms
# stacking on the SAME few high storeys is what read as "everything is
# happening up there and the lower floors show nothing" — exactly backwards
# from a real burnt-out building, which smokes from EVERY compartment fire
# has already passed through, low ones included.
#
# `lit_groups` — the SAME `(side, storey)` set the flame loops already build
# (`place_fire`'s own primary loop, the F4 top-up, the residual pockets, the
# cluster top-up ALL now add to it, not just the cluster top-up as before)
# — is what "already shows a flame source" means here. A candidate NOT in
# it outranks one that is; within each of those two groups, LOWER storey
# outranks higher. AT LEAST ONE NEAR THE FLAME: a build with candidates on
# both sides of the split still needs at least one smoke jet where the fire
# actually is ("a flame storey with zero smoke looks wrong too") — the last
# of the capped picks is swapped for the first LIT candidate in this same
# ranked order when every pick so far landed on a not-yet-lit storey and at
# least one lit candidate exists. A swap, never an addition — the total
# picked never exceeds the caller's own budget, so the emitter-budget
# accounting is unchanged.
def _vertical_priority_key(lit_groups):
    return lambda e: (int((e.get("side"), e["storey"]) in lit_groups),
                      e["storey"], e["id"])


def _pick_smoke_openings(evs_of_state, lit_groups, cap, street_bias_side=None,
                         bias_weight=2):
    """`(ev, op)` pairs, AT MOST `cap` — the window-jet (opening-level)
    smoke pick, ranked not-yet-lit-and-lower-storey first (see the section
    docstring above), round-robin across sides same as every other
    selector here, with the at-least-one-near-the-flame guarantee applied
    after capping."""
    order_sides, by_side = _group_by_side(evs_of_state)
    key = _vertical_priority_key(lit_groups)
    side_seqs = []
    for side in order_sides:
        evs_here = sorted(by_side[side], key=key)
        event_seqs = [[(ev, ev["ops"][k]) for k in _spread_order(len(ev["ops"]))]
                     for ev in evs_here]
        side_seqs.append(_round_robin(event_seqs))
    ranked = _round_robin(side_seqs, _side_weights(order_sides, street_bias_side,
                                                    bias_weight))

    def lit(pair):
        ev, _op = pair
        return (ev.get("side"), ev["storey"]) in lit_groups

    picked = ranked[:max(0, int(cap))]
    if picked and not any(lit(p) for p in picked):
        for p in ranked[len(picked):]:
            if lit(p):
                picked = picked[:-1] + [p]
                break
    return picked


def _pick_smoke_events(events_in_priority_order, lit_groups, budget,
                       street_bias_side=None, bias_weight=2):
    """Event-level counterpart of `_pick_smoke_openings`, for the
    `smoke_window_jets=False` (one source at an event's own middle opening)
    mechanism — same vertical-priority ranking, same round-robin-across-
    sides, same at-least-one-near-the-flame guarantee, `budget` EVENTS
    instead of `cap` OPENINGS."""
    order_sides, by_side = _group_by_side(events_in_priority_order)
    key = _vertical_priority_key(lit_groups)
    ranked_sides = [sorted(by_side[s], key=key) for s in order_sides]
    ranked = _round_robin(ranked_sides, _side_weights(order_sides, street_bias_side,
                                                       bias_weight))

    def lit(ev):
        return (ev.get("side"), ev["storey"]) in lit_groups

    picked = ranked[:max(0, int(budget))]
    if picked and not any(lit(e) for e in picked):
        for e in ranked[len(picked):]:
            if lit(e):
                picked = picked[:-1] + [e]
                break
    return picked


def place_fire(stage, root, doc, masses, events, tag, rng, top_z,
               dx, dy, scale=1.0, max_emitters=9, smoke=True, *,
               smoke_scale=1.0, street_bias_side=None, street_bias_weight=2,
               side_smoke_flame_max=None, side_smoke_nonflame_max=None,
               roof_cap_intact=None, roof_cap_collapsed=None,
               flame_min_clusters=None, flame_extra_max=FLAME_EXTRA_MAX,
               flame_size_scaling=False, smoke_size_scaling=False,
               smoke_window_jets=False, residual_flame_frac=0.0,
               smoke_vertical_bias=False,
               geom_root=None, snap_tol_m=SNAP_TOL_M,
               snap_inset_m=SNAP_INSET_M, snap_reach_in_m=SNAP_REACH_IN_M,
               snap_reach_out_m=SNAP_REACH_OUT_M):
    """`urban_fire.r_flames`, re-run against a BAKED building.

    Same four parts, same order — the difference is only where the inputs
    come from:

      1. FLAME events -> `FLAME_PER_OPENING` sheet sources across each
         opening's head, at most `max_emitters` openings. The opening records
         are the bake's own (`fire_bake.op_from_json`), so this is
         `urban_fire._flame_sources` verbatim, not a re-implementation. When
         `flame_min_clusters` is given and the live "flame"-state events
         alone span fewer than that many distinct `(side, storey)` groups, a
         SECOND pass (dimmer, from the building's own "out" events — see
         below) tops up to `flame_min_clusters` groups, budget permitting —
         "more actual places of fire on each building" (2026-08-31, second
         review), not just more openings within the one hottest run.
      2. SIDE SMOKE — window-SHEET `FlowEmitterBox` sources at individual
         opening HEADS (`uf._flame_sources`, the exact same primitive and
         facade-plane placement flames use, not a sphere): on an ACTIVE
         fire, from the compartments that have already burnt OUT, highest-
         storey-first among candidate EVENTS but picked at the OPENING
         level (`_flame_selection_order`, not `_select_events_by_side`) up
         to `side_smoke_flame_max` distinct openings. On a burnt-out one,
         its SMOULDER events' own openings FIRST (still visibly active),
         then — 2026-08-31, "let's have it from the sides as well... F4
         buildings especially should pour smoke from openings" — openings
         from its OWN burnt-OUT ("out"-state) events fill whatever budget
         the smoulder openings did not, up to `side_smoke_nonflame_max`.
         `soot_plume.plan_events` records dozens of "out" events per burnt-
         out building (measured: 28-114 on `city_138`'s F4/F5 GAC
         buildings) against 1-3 "smoulder" ones, so this is a large,
         previously-untapped pool of real openings, not a reach. PICKING AT
         THE OPENING LEVEL (not the event level) is the 2026-08-31 second-
         review fix for "I see more smoke but not really side smoke, they
         should be coming out of windows similar to the fire" — event-level
         picking authored exactly ONE source at the MIDDLE opening of
         whichever event was chosen, however many openings that event
         actually had, which read as one ambient puff per compartment
         rather than several window jets; opening-level picking (the same
         round-robin/`_spread_order` machinery flames already use) can
         author several, each seated at its OWN opening's head with the
         SAME outward+upward velocity flames get, so it visibly pours out
         of a window and rises up the wall instead of drifting as a blob.
      3. The interior smoke, seated on the sidecar's recorded floor-slab
         SEATS rather than from `ctx["fit"]`, because the fit-out is
         geometry inside a referenced file by now.
      4. The roof plume, gated on `fire["roof"]`, likewise from the seats —
         capped at `roof_cap_collapsed` (default 2, unchanged) once
         `roof_has_collapsed(doc)` is true, `roof_cap_intact` (default 2,
         unchanged) otherwise. A caller that wants SIDE smoke to dominate
         an intact roof (2026-08-31 review: "unless the roof is collapsed
         there should be more smoke coming from sides than top") passes a
         smaller `roof_cap_intact` — trading a roof-plume slot for a
         window one, not just adding — while `roof_cap_collapsed` stays at
         its old count: a collapsed roof is a hole with nothing directional
         about it, and it earned every one of those sources honestly.

    An event whose openings belong to a module a collapse killed is skipped:
    `e["dead"]` was serialised AFTER the ladder ran, so a flame never floats
    where a wall used to be.

    `_wall_vents` — `r_flames`' fallback for a band with no openings at all —
    is NOT reproduced: it walks `ctx["info"]["elements"]`, which a bake does
    not carry. A building that would have needed it says so and gets its
    roof/interior smoke only.

    `smoke` is the row launcher's `FA_SMOKE`, now an argument rather than a
    module global so the city launcher can pass its own.

    `smoke_scale` (keyword-only, default 1.0, meant to carry `FA_SMOKE_SCALE`)
    is the "increase amount of smoke" knob from the 2026-08-31 review. It
    does NOT add emitters on its own — every smoke/interior/roof/wisp source
    below still gets exactly the count `side_smoke_*`/`roof_cap_*` say, so
    the global emitter budget's estimate stays exact — it multiplies the two
    HONEST density levers already in the Flow authoring: the emission
    `scale` fed to `fire.set_emission` (the `"smoke"` field of `fire.
    STATE_EMISSION`, i.e. the actual smoke density a voxel injects) for
    every smoke-only source, and the seat `radius` of the seated sphere
    plumes (`_sphere_source`'s `radius_scale`) for the interior and roof
    plumes specifically — a wider sphere is a wider column, not just a
    denser one. Flame sources (`state="flame"`) are untouched by design: the
    original ask was more SMOKE, not a brighter fire (this call's own new
    `max_emitters`/street bias/cluster knobs are the separate, later ask
    for more FIRE).

    `side_smoke_flame_max`/`side_smoke_nonflame_max` (keyword-only,
    default `None`) fall back to the ORIGINAL fixed budgets
    (`uf.SMOKE_EXTRA_MAX`, `spl.SMOULDER_EVENTS_MAX` — 3 each) when not
    given, so a caller that never opts in (the row/bench launcher) sees
    byte-identical output. `roof_cap_intact`/`roof_cap_collapsed` fall back
    to 2 (the original fixed count) the same way. `flame_min_clusters`
    defaults to `None`, which disables the cluster-diversity top-up
    entirely — the row/bench launcher, which never passes it, is unaffected.

    `flame_size_scaling`/`smoke_size_scaling` (keyword-only, both `False`
    by default — 2026-08-31, third review, "if the smaller building can
    have 5-7 fire windows, the bigger one should have more"): when
    `flame_size_scaling` is `True`, `max_open` (the flame-opening ceiling)
    becomes `min(max_emitters, flame_window_target(n_storeys))` instead of
    the original `max(max_emitters, min(16, n_storeys // 2))` — the
    allocator's own `max_emitters` step still rations WHICH buildings get
    funded at all, but the CEILING each step counts toward is now the
    building's own height-scaled target, so a short building plateaus early
    (freeing budget) while a tower keeps growing across more allocator
    rounds. `smoke_size_scaling` does the same for the two `side_smoke_*`
    caps, via `smoke_window_target`. `False` for either (every caller before
    2026-08-31) is the exact original formula.

    `smoke_window_jets` (keyword-only, `False` by default) selects between
    the two smoke-picking MECHANISMS part 2 above describes — event-level
    (`False`, the ORIGINAL: one source at whichever event's own middle
    opening) or opening-level (`True`: several sources, one per opening,
    spread the same way flames are). It is independent of `smoke_size_
    scaling` (that controls the CAP; this controls what each unit of the
    cap actually looks like) so a caller can opt into either without the
    other, though the city launcher opts into both together.

    `residual_flame_frac` (keyword-only, default `0.0` — same review, the
    HEADLINE case: a 28-storey RESIDUAL/F5 tower with 75 baked events
    showed zero flame, because only `state == "flame"` ever reached the
    primary loop): `0.0` restores exactly TODAY's behaviour — an F4
    ("smoulder") building still gets the ORIGINAL reduced-intensity top-up
    below, F5/F5c/F6 ("residual") gets none. Any positive fraction REPLACES
    that F4 top-up (for F4 too, not just F5+) with a single unified
    mechanism covering every non-"flame" state: `round(residual_flame_frac
    * flame_window_target(n_storeys))` (floored at `RESIDUAL_FLAME_MIN`)
    SCATTERED single-window pockets (`_scattered_selection_order` — at most
    one opening per event, so two picks never share a contiguous run, which
    would read as an active F2/F3 fire rather than embers in a burnt-out
    shell), preferring the building's own SMOULDER-event openings first,
    then its "out"-event openings to fill the rest — the same
    `street_bias_side` every other selection here uses, so the pockets
    favour the street-facing elevation too.

    `street_bias_side`/`street_bias_weight` (keyword-only; `street_bias_side`
    a single side letter — the caller's ALREADY-CHOSEN boosted side, see
    `choose_street_side` — `None` by default) bias EVERY round-robin
    selection below — the flame openings, the F4 reduced-intensity top-up,
    the cluster-diversity top-up, the vent-smoke picks (both branches), and
    the F1 wisp pair — toward that one side, at `street_bias_weight`
    (default 2) turns per round instead of 1 for every other side (see
    `_side_weights`). `None` (every caller before 2026-08-31, and any
    building `choose_street_side` found nothing to prefer for) keeps the
    exact original unweighted interleave. THIS FUNCTION NEVER CHOOSES THE
    SIDE ITSELF — `street_bias_side` must already be one value, decided
    ONCE per building by the caller (`choose_street_side`, seeded off a
    STABLE per-building identity), so every round-robin call below for the
    same building biases the SAME side; recomputing an argmax in here on
    every call is what produced the "boosted side is always whichever one
    happens to score highest" clustering the second review flagged.

    OPENINGS AND EVENTS ARE OTHERWISE PICKED ROUND-ROBIN ACROSS SIDES, not
    in raw event order — see `_flame_selection_order`/`_select_events_by_
    side` above this function. "Live fire seems to mostly only stay on 1
    side of the building... also more positions on each side" (2026-08-30
    review): the old per-branch loops walked events in whatever order they
    were planned in and a `break` that only escaped one nested loop meant
    the first side's events silently ate the whole budget. The COUNT
    selected per branch does not change because of this ordering by itself
    (only the street bias and the cluster top-up change a count, and only by
    trading roof for side or spending `flame_extra_max` explicitly, never by
    silently inflating the global total) — so the round-robin itself remains
    purely a distribution fix layered under the budgets, not a second budget
    mechanism.

    `geom_root`/`snap_tol_m`/`snap_inset_m`/`snap_reach_in_m`/
    `snap_reach_out_m` (keyword-only, `geom_root` `None` by default — the
    2026-08-31 "fires floating outside the building" fix, see the
    `snap_events_to_geometry` section above this function): when `geom_root`
    names the building's own composed geometry (`bake_geometry_root(holder,
    doc)` for a real assembly; the bake's own `doc["root"]` for a bare-USD
    diagnosis/verification run), EVERY opening in `events` is contact-tested
    against it and corrected — SNAPPED to the real wall (inset `snap_inset_m`
    further in) if it was more than `snap_tol_m` off, DROPPED if no real
    surface is within reach at all — before any flame/smoke/residual
    selection runs, so the fix reaches flames, window-jet side smoke and the
    residual pockets alike (they all draw from these same, now-corrected,
    events). `None` (every caller before this fix, including the row/bench
    launcher and every existing test) skips the whole pass — `events` is
    used exactly as given, byte-identical to before. The returned dict's
    `"snap"` key is `snap_events_to_geometry`'s own stats dict either way
    (all-zero, `"locator": False` when `geom_root` is `None`).

    `drop_top_storey_synthetic` (2026-08-31, "avoid fires at the extreme top
    of buildings... unless we're 100% sure about windows on the top floor")
    runs UNCONDITIONALLY, right after the snap pass — it only ever removes a
    SYNTHETIC opening (`is_synthetic_op`) above the top-storey cap, so a kit
    building (never synthetic) or a real-glazing GAC opening (also never
    synthetic, top floor included) is untouched regardless of caller. The
    returned dict does not carry its stats — call it directly for that (the
    39-building probe does, for its per-building table column).

    `smoke_vertical_bias` (keyword-only, `False` by default — 2026-08-31,
    "have more smoke on lower floors so it looks like those have been burnt
    out if you're not putting fire there"): `True` replaces BOTH side-smoke
    selectors' `(-storey, id)`-ranked pool (`out_ranked`, the ORIGINAL
    "highest storey first" order) with `_pick_smoke_openings`/`_pick_smoke_
    events` — not-yet-lit-and-lower-storey-first, `lit_groups` sourced from
    every flame source authored above this point (the primary loop, the F4
    top-up, the residual pockets, the cluster top-up), with the same
    at-least-one-near-the-flame guarantee those functions document. Applies
    to BOTH the `is_flame` branch's `out`-event pool and the non-flame
    branch's smoulder-then-out pools. `False` (every caller before this fix)
    keeps the exact original ranking and pick order.
    """
    snap_stats = {"tested": 0, "ok": 0, "snapped": 0, "dropped": 0,
                 "worst_offset_m": 0.0, "locator": False}
    if geom_root:
        events, snap_stats = snap_events_to_geometry(
            stage, geom_root, events, tol_m=snap_tol_m,
            inset_m=snap_inset_m, reach_in_m=snap_reach_in_m,
            reach_out_m=snap_reach_out_m)
    f = doc["fire"]
    # NO FIRE AT THE EXTREME TOP UNLESS THE WINDOWS ARE REAL — see the
    # `drop_top_storey_synthetic` section above. Runs unconditionally (not
    # opt-in): it only ever touches a SYNTHETIC opening, which a kit
    # building's events never contain, so a caller that never sees a GAC
    # bake with a starved band is unaffected either way.
    n_st = int(f.get("n_storeys") or 0)
    events, top_stats = drop_top_storey_synthetic(events, n_st)
    state = f.get("state")
    # A LEVEL WITH NO ACTIVE STATE CAN STILL SMOULDER. F1 has `ACTIVE=None`
    # by design, but on the GAC path `soot_plume.plan_events` now gives F1
    # one `smoulder` event (user, second row: "B0 has almost no damage") —
    # place a smoke wisp on it, no flames.
    wisp_only = False
    if not state:
        if any(ev.get("state") == "smoulder" and ev.get("ops")
               for ev in (events or [])):
            state, wisp_only = "smoulder", True
        else:
            return {"flame": 0, "smoke": 0, "interior": 0, "roof": 0,
                    "note": "level {0} has no active state — no emitters"
                            .format(f.get("level")), "snap": snap_stats,
                    "synthetic_top": top_stats}
    ctx = {"stage": stage, "rng": rng, "tag": tag, "notes": [],
           "flow_root": root, "info": {"masses": masses},
           "fire": {"origin": int(f["origin"]),
                    "storeys": [int(s) for s in f["storeys"]],
                    "top": int(f["top"]), "sides": tuple(f["sides"]),
                    "n_storeys": int(f["n_storeys"]), "mass": f["mass"],
                    "roof": bool(f["roof"]), "level": f["level"],
                    "state": state}}
    UsdGeom.Xform.Define(stage, Sdf.Path(root + "/emitters"))

    def live(ev):
        return all(not (o.get("e") or {}).get("dead") for o in ev["ops"])

    evs = [ev for ev in events if live(ev) and ev.get("ops")]
    is_flame = state == "flame"
    n_flame = n_open = 0
    # A TALL BUILDING BURNS IN MORE WINDOWS. Six openings read as a fire on
    # a 12-storey block and as a candle on a 37-storey tower ("the taller
    # buildings need more fire/flames, looks weird otherwise", user
    # 2026-08-30): from 12 storeys up the opening budget grows with the
    # height, capped at 16. SUPERSEDED BY `flame_window_target` when
    # `flame_size_scaling` is on (2026-08-31, third review) — `min` instead
    # of `max` here on purpose: `max_emitters` is still the allocator's own
    # per-round funding step, so a building plateaus at its OWN smaller
    # target once the step outgrows it, leaving the freed budget for a
    # taller building's target to keep climbing across later rounds.
    if flame_size_scaling:
        max_open = min(max_emitters, flame_window_target(n_st))
    else:
        max_open = (max(max_emitters, min(16, n_st // 2)) if n_st >= 12
                    else max_emitters)
    lit_groups = set()
    for ev, op in _flame_selection_order(
            [e for e in evs if e["state"] == "flame"],
            street_bias_side=street_bias_side, bias_weight=street_bias_weight):
        if n_open >= max_open:
            break
        n_flame += uf._flame_sources(
            ctx, root, op, "flame", scale,
            "e{0}_{1}".format(ev["id"], n_open), uf.FLAME_PER_OPENING)
        lit_groups.add((ev.get("side"), ev.get("storey")))
        n_open += 1
    residual_on = bool(residual_flame_frac) and float(residual_flame_frac) > 0.0
    if state == "smoulder" and not wisp_only and not residual_on:
        # F4: the fire is DYING, not dead — a few openings are still alight
        # at reduced intensity, the rest smoulder. Without this an F4
        # building showed soot and a roof hole and no fire at all.
        # SUPERSEDED BY THE RESIDUAL-FLAME BLOCK BELOW once
        # `residual_flame_frac` is on (2026-08-31, third review) — that
        # block covers F4 too, with the SAME scattered/single-window
        # treatment F5/F5c/F6 get, so this stays the exact `residual_
        # flame_frac == 0.0` fallback ("0 restores today's behavior").
        cap = max(2, max_open // 2)
        for ev, op in _flame_selection_order(
                [e for e in evs if e["state"] == "smoulder"],
                street_bias_side=street_bias_side,
                bias_weight=street_bias_weight):
            if n_open >= cap:
                break
            n_flame += uf._flame_sources(
                ctx, root, op, "flame", scale * 0.6,
                "s{0}_{1}".format(ev["id"], n_open),
                max(1, uf.FLAME_PER_OPENING - 1))
            lit_groups.add((ev.get("side"), ev.get("storey")))
            n_open += 1
    # RESIDUAL FLAME POCKETS (2026-08-31, third review, item 5 — the
    # headline case): every state that is NOT actively "flame" (F4's
    # "smoulder" AND F5/F5c/F6's "residual") gets a REDUCED, SCATTERED
    # allocation of single-window flame pockets — "the user wants fire on
    # its windows" for a 134 m residual tower that, before this, authored
    # NONE. Target = `residual_flame_frac` of what a FLAME-state building of
    # this SAME height would get (`flame_window_target`, independent of
    # whether `flame_size_scaling` itself is on for this call — the
    # fraction is always taken against the size formula), floored at
    # `RESIDUAL_FLAME_MIN`. SMOULDER-event openings first (still visibly
    # active), "out"-event openings fill the rest — the same priority the
    # side-smoke picks below use, and the same reason: dozens of "out"
    # events exist against 1-3 "smoulder" ones. `_scattered_selection_order`
    # (AT MOST ONE OPENING PER EVENT) is what keeps these from reading as a
    # contiguous F2/F3-style compartment fire.
    if not is_flame and not wisp_only and residual_on:
        residual_target = max(RESIDUAL_FLAME_MIN,
                              int(round(float(residual_flame_frac)
                                        * flame_window_target(n_st))))
        n_residual = 0
        sm_pockets = [e for e in evs if e["state"] == "smoulder"]
        for ev, op in _scattered_selection_order(
                sm_pockets, street_bias_side=street_bias_side,
                bias_weight=street_bias_weight):
            if n_residual >= residual_target:
                break
            n_flame += uf._flame_sources(
                ctx, root, op, "flame", scale * RESIDUAL_FLAME_SCALE,
                "r{0}_{1}".format(ev["id"], n_residual), 1)
            lit_groups.add((ev.get("side"), ev.get("storey")))
            n_residual += 1
            n_open += 1
        if n_residual < residual_target:
            out_pockets = [e for e in evs if e["state"] == "out"]
            for ev, op in _scattered_selection_order(
                    out_pockets, street_bias_side=street_bias_side,
                    bias_weight=street_bias_weight):
                if n_residual >= residual_target:
                    break
                n_flame += uf._flame_sources(
                    ctx, root, op, "flame", scale * RESIDUAL_FLAME_SCALE,
                    "r{0}_{1}".format(ev["id"], n_residual), 1)
                lit_groups.add((ev.get("side"), ev.get("storey")))
                n_residual += 1
                n_open += 1
    # MULTIPLE FLAME CLUSTERS (2026-08-31, second review: "more actual
    # places of fire on each building" — flames were concentrating in one
    # hottest contiguous run because a currently-active fire (`is_flame`)
    # only ever has "flame"-state events on the ONE (or two) `(side,
    # storey)` band(s) it is CURRENTLY at — a climbing fire, by design,
    # never marks a passed-through compartment "flame" again once it has
    # moved on (that compartment becomes an "out" event instead). So a
    # building whose plan only ever lit one band literally has no SECOND
    # cluster of "flame"-state data to round-robin into, no matter how the
    # selection is ordered. This taps the SAME "out" event pool the smoke
    # picks below already use, but for a few DIMMER flame accents (echoing
    # the F4 top-up's own `scale * 0.6` / `FLAME_PER_OPENING - 1` recipe) at
    # `(side, storey)` groups the primary loop above never lit — a
    # currently-flaming building really has been alight in every one of its
    # own "out" compartments at some point, so a residual glow there is
    # truthful, not invented. Opt-in only (`flame_min_clusters` not `None`)
    # — the row/bench launcher, which never passes it, is byte-identical.
    if is_flame and flame_min_clusters is not None:
        extra_budget = min(max(0, max_open - n_open), max(0, int(flame_extra_max)))
        if len(lit_groups) < int(flame_min_clusters) and extra_budget > 0:
            extra_pool = [e for e in evs if e["state"] == "out"]
            n_extra = 0
            for ev, op in _flame_selection_order(
                    extra_pool, street_bias_side=street_bias_side,
                    bias_weight=street_bias_weight):
                if n_extra >= extra_budget:
                    break
                n_flame += uf._flame_sources(
                    ctx, root, op, "flame", scale * 0.6,
                    "x{0}_{1}".format(ev["id"], n_extra),
                    max(1, uf.FLAME_PER_OPENING - 1))
                lit_groups.add((ev.get("side"), ev.get("storey")))
                n_open += 1
                n_extra += 1
    note = ""
    if is_flame and n_open == 0:
        note = ("no live flame event with openings — `_wall_vents` is not "
                "available from a bake, so this building shows smoke only")

    n_smoke = 0
    if not smoke:
        # flames only (FA_SMOKE=0): no vent smoke, no interior or roof plumes
        return {"flame": n_flame, "smoke": 0, "interior": 0, "roof": 0,
                "openings": n_open, "state": state,
                "note": note or "smoke off (FA_SMOKE=0)", "snap": snap_stats,
                "synthetic_top": top_stats}
    if wisp_only:
        # one wisp per smouldering window, half strength, nothing else —
        # round-robin by side even at a budget of 2, so a wisp-stage building
        # smouldering on two sides shows one wisp on each rather than both on
        # whichever side's events happen to sort first.
        n_smoke = 0
        for ev in _select_events_by_side(
                [e for e in evs if e["state"] == "smoulder"], 2,
                street_bias_side=street_bias_side,
                bias_weight=street_bias_weight):
            op = ev["ops"][len(ev["ops"]) // 2]
            n_smoke += uf._flame_sources(ctx, root, op, "smoulder",
                                         scale * 0.5 * smoke_scale,
                                         "wisp{0}".format(ev["id"]), 1)
        return {"flame": 0, "smoke": n_smoke, "interior": 0, "roof": 0,
                "openings": 0, "state": "wisp", "note": "F1 wisp",
                "snap": snap_stats, "synthetic_top": top_stats}
    side_smoke_flame_cap = (smoke_window_target(n_st) if smoke_size_scaling
                            else (uf.SMOKE_EXTRA_MAX
                                  if side_smoke_flame_max is None
                                  else max(0, int(side_smoke_flame_max))))
    side_smoke_nonflame_cap = (smoke_window_target(n_st) if smoke_size_scaling
                               else (spl.SMOULDER_EVENTS_MAX
                                     if side_smoke_nonflame_max is None
                                     else max(0, int(side_smoke_nonflame_max))))
    if is_flame:
        # F5/F5c never reach this branch — `soot_plume` stops planning
        # "flame"-state events once the building is "residual" — but a still-
        # actively-flaming building (F2/F3) can have "out" events on more
        # than one side, and the plain storey/id sort below used to hand the
        # whole budget to whichever side had the tallest one.
        out_ranked = sorted([e for e in evs if e["state"] == "out"],
                            key=lambda e: (-e["storey"], e["id"]))
        if smoke_window_jets:
            # OPENING-LEVEL, WINDOW-SHEET PICKS (2026-08-31, second review:
            # "I see more smoke but not really side smoke, they should be
            # coming out of windows similar to the fire") — see this
            # function's own docstring, part 2, for the full rationale.
            n_side = 0
            if smoke_vertical_bias:
                picks = _pick_smoke_openings(
                    out_ranked, lit_groups, side_smoke_flame_cap,
                    street_bias_side=street_bias_side,
                    bias_weight=street_bias_weight)
            else:
                picks = list(_flame_selection_order(
                    out_ranked, street_bias_side=street_bias_side,
                    bias_weight=street_bias_weight))[:side_smoke_flame_cap]
            for ev, op in picks:
                n_smoke += uf._flame_sources(
                    ctx, root, op, "smoke", scale * smoke_scale,
                    "sm{0}_{1}".format(ev["id"], n_side), 1)
                n_side += 1
        else:
            # ORIGINAL event-level pick (one source at the event's own
            # MIDDLE opening) — kept exactly as it was for a caller that
            # never opts into `smoke_window_jets` (the row/bench launcher).
            picked = (_pick_smoke_events(
                          out_ranked, lit_groups, side_smoke_flame_cap,
                          street_bias_side=street_bias_side,
                          bias_weight=street_bias_weight)
                      if smoke_vertical_bias else
                      _select_events_by_side(
                          out_ranked, side_smoke_flame_cap,
                          street_bias_side=street_bias_side,
                          bias_weight=street_bias_weight))
            for ev in picked:
                op = ev["ops"][len(ev["ops"]) // 2]
                n_smoke += uf._flame_sources(ctx, root, op, "smoke",
                                             scale * smoke_scale,
                                             "sm{0}".format(ev["id"]), 1)
    else:
        # An F5/F5c's fire IS this branch — no flame-state events exist for
        # it at all, so the same one-sided-cluster bug here was the whole
        # story for those buildings, not just a corner case of it.
        #
        # SMOULDER FIRST, "OUT" TO FILL THE REST — but ONLY when a caller
        # explicitly opted into a bigger budget (`side_smoke_nonflame_max`
        # not `None`, or `smoke_size_scaling`). A burnt-out building has
        # only 1-3 SMOULDER events (still visibly active) against dozens of
        # "out" ones (compartments that already burned through — a real
        # opening, just not still glowing) — 2026-08-31: "unless the roof is
        # collapsed there should be more smoke coming from sides than
        # top... need more smoke from all buildings that are on fire".
        # Tapping the "out" pool is what turns a 1-3-source trickle into a
        # real vented elevation without inventing any new event data — but
        # gating it behind the explicit opt-in is what keeps a caller that
        # never customises this argument (the row/bench launcher) on the
        # byte-identical ORIGINAL selection (smoulder events only,
        # `spl.SMOULDER_EVENTS_MAX`), exactly as this function's own
        # docstring promises.
        sm = [e for e in evs if e["state"] == "smoulder"]
        out_fill_enabled = (side_smoke_nonflame_max is not None
                           or smoke_size_scaling)
        if smoke_window_jets:
            # OPENING-LEVEL, WINDOW-SHEET PICKS — same rationale as the
            # `is_flame` branch above.
            n_side = 0
            if smoke_vertical_bias:
                picks = _pick_smoke_openings(
                    sm, lit_groups, side_smoke_nonflame_cap,
                    street_bias_side=street_bias_side,
                    bias_weight=street_bias_weight)
            else:
                picks = list(_flame_selection_order(
                    sm, street_bias_side=street_bias_side,
                    bias_weight=street_bias_weight))[:side_smoke_nonflame_cap]
            for ev, op in picks:
                n_smoke += uf._flame_sources(
                    ctx, root, op, state, scale * smoke_scale,
                    "sm{0}_{1}".format(ev["id"], n_side), 1)
                n_side += 1
            if out_fill_enabled and n_side < side_smoke_nonflame_cap:
                out_ranked = sorted([e for e in evs if e["state"] == "out"],
                                    key=lambda e: (-e["storey"], e["id"]))
                remaining = side_smoke_nonflame_cap - n_side
                if smoke_vertical_bias:
                    fill_picks = _pick_smoke_openings(
                        out_ranked, lit_groups, remaining,
                        street_bias_side=street_bias_side,
                        bias_weight=street_bias_weight)
                else:
                    fill_picks = list(_flame_selection_order(
                        out_ranked, street_bias_side=street_bias_side,
                        bias_weight=street_bias_weight))[:remaining]
                for ev, op in fill_picks:
                    n_smoke += uf._flame_sources(
                        ctx, root, op, state, scale * smoke_scale,
                        "sm{0}_{1}".format(ev["id"], n_side), 1)
                    n_side += 1
        else:
            picked = (_pick_smoke_events(
                          sm, lit_groups, side_smoke_nonflame_cap,
                          street_bias_side=street_bias_side,
                          bias_weight=street_bias_weight)
                      if smoke_vertical_bias else
                      _select_events_by_side(
                          sm, side_smoke_nonflame_cap,
                          street_bias_side=street_bias_side,
                          bias_weight=street_bias_weight))
            if out_fill_enabled and len(picked) < side_smoke_nonflame_cap:
                used = {ev["id"] for ev in picked}
                out_pool = [e for e in evs
                           if e["state"] == "out" and e["id"] not in used]
                out_ranked = sorted(out_pool,
                                    key=lambda e: (-e["storey"], e["id"]))
                remaining = side_smoke_nonflame_cap - len(picked)
                fill = (_pick_smoke_events(
                            out_ranked, lit_groups, remaining,
                            street_bias_side=street_bias_side,
                            bias_weight=street_bias_weight)
                        if smoke_vertical_bias else
                        _select_events_by_side(
                            out_ranked, remaining,
                            street_bias_side=street_bias_side,
                            bias_weight=street_bias_weight))
                picked = picked + fill
            for ev in picked:
                op = ev["ops"][len(ev["ops"]) // 2]
                # `state` (the building's own visual state — "smoulder" or
                # "residual"), not `ev["state"]`: an "out" event's own state
                # has no entry in `fire.STATE_EMISSION`, and `set_emission`
                # would silently author a DISABLED emitter (dead weight, no
                # smoke) if it were passed straight through.
                n_smoke += uf._flame_sources(ctx, root, op, state,
                                             scale * smoke_scale,
                                             "sm{0}".format(ev["id"]), 1)

    seats = doc.get("seats") or {}
    n_int = 0
    # INTERIOR SMOKE, NORMALLY BURNT-OUT ONLY — but a building with
    # LITERALLY NOTHING ELSE (`n_flame == n_smoke == 0`: no live flame
    # event, no side-smoke pick either) still has to look like it caught
    # fire, not like an untouched building standing next to the soot skin
    # baked into its own walls. This is a real, observed shape — a GAC
    # building whose `soot_plume.plan_events` planned ZERO events at all
    # (`gac_SM_Building_29_F3_o7_ENW_s808` in `city_138`; the "starved-
    # events trap" section of the skill documents a related but different
    # symptom of the same island-shape filter) — every one of the 32
    # burning buildings must show SOME smoke, and this is the one case
    # that otherwise authors nothing whatsoever. LAST RESORT ONLY: gated on
    # both being exactly zero, so it never fires on a building that already
    # has flame or side smoke to show.
    if not is_flame or (n_flame == 0 and n_smoke == 0):
        int_state = state if not is_flame else "smoke"
        for k, seat in enumerate(seats.get("interior") or []):
            n_int += _sphere_source(
                stage, "{0}/emitters/{1}_int_{2}".format(root, tag, k),
                seat, int_state, scale * smoke_scale, (0.0, 0.0, 1.6), dx, dy,
                top_z, radius_scale=smoke_scale)
    n_roof = 0
    collapsed = False
    if f.get("roof"):
        collapsed = roof_has_collapsed(doc)
        roof_cap = (roof_cap_collapsed if collapsed else roof_cap_intact)
        roof_cap = 2 if roof_cap is None else max(0, int(roof_cap))
        for k, seat in enumerate((seats.get("roof") or [])[:roof_cap]):
            n_roof += _sphere_source(
                stage, "{0}/emitters/{1}_roof{2}".format(root, tag, k),
                seat, "smoulder" if state == "flame" else state,
                scale * smoke_scale, (0.6, 0.2, 3.2), dx, dy, top_z,
                radius_scale=smoke_scale)
    return {"flame": n_flame, "smoke": n_smoke, "interior": n_int,
            "roof": n_roof, "openings": n_open, "state": state, "note": note,
            "roof_collapsed": collapsed, "snap": snap_stats,
            "synthetic_top": top_stats}


# ---------------------------------------------------------------------------
# THE REVIEW CAMERA FACES THE FIRE
# ---------------------------------------------------------------------------
def fire_view_params(doc, masses, box):
    """`views_around` arguments for one damaged building — was the arithmetic
    inline in `fire_assembly_launch_script`'s capture loop, moved here word
    for word so the city launcher frames a burning building the same way.

    `box` is the building's measured world bbox
    (`[x0, y0, z0, x1, y1, z1]`); `doc`/`masses` its sidecar and rehydrated
    mass boxes.

      * `top_h` — TOP-VIEW HEIGHT FROM THE BUILDING'S OWN MEASURED SIZE: at
        18 mm on the 20.955 mm aperture, 0.5 x the horizontal FOV is 1.164,
        so this is the standoff that fits the footprint and clears the roof
        — `gac_fire_bench`'s own arithmetic.
      * `azimuth_deg` — THE REVIEW CAMERA FACES THE FIRE. The default
        oblique looks from the south-west whatever burns; a building alight
        on E showed its blank back wall in every capture (fire_row3).
        Bearing = the burning sides' outward directions summed.
      * `aim_h` — the middle of the burning band, camera a little above it.
    """
    W, D, H = box[3] - box[0], box[4] - box[1], box[5] - box[2]
    top_h = max(W, D) / 1.164 * 1.45 + H
    # 1.7x on H (was a shared 1.3x): at 1.3x a 60 m+ tower's facade
    # overflows the frame and the camera reads as "clipped into the wall"
    # (d31_apartment_tall_F5_obl, city_v3 review). Width-dominant buildings
    # keep the old 1.3x arithmetic exactly.
    obl_dist = max(50.0, 1.3 * max(W, D), 1.7 * H)
    obl_h = max(18.0, 0.4 * H)
    fd = (doc or {}).get("fire") or {}
    vec = {"E": (1, 0), "N": (0, 1), "W": (-1, 0), "S": (0, -1)}
    vx = sum(vec.get(sd, (0, 0))[0] for sd in (fd.get("sides") or []))
    vy = sum(vec.get(sd, (0, 0))[1] for sd in (fd.get("sides") or []))
    az = math.degrees(math.atan2(vy, vx)) if (vx or vy) else 225.0
    aim_h = 1.0
    try:
        lv = ((masses or {}).get(fd.get("mass") or "main")
              or list((masses or {}).values())[0])["levels"]
        sts = [int(q) for q in (fd.get("storeys") or [])]
        if sts:
            aim_h = 0.5 * (lv[min(sts[0], len(lv) - 1)]
                           + lv[min(sts[-1], len(lv) - 1)]) + 1.5
    except Exception:
        pass
    obl_h = max(obl_h, aim_h + 0.3 * obl_dist)
    return {"top_h": top_h, "obl_dist": obl_dist, "obl_h": obl_h,
            "azimuth_deg": az, "aim_h": aim_h}


def _point_in_footprint(px, py, o, margin=0.0):
    """Is the world point inside obstacle `o`'s yaw-rotated footprint
    (`{"x","y","W","D","yaw"}`, metres/degrees), grown by `margin`?"""
    dx, dy = px - o["x"], py - o["y"]
    a = math.radians(-(o.get("yaw") or 0.0))
    lx = dx * math.cos(a) - dy * math.sin(a)
    ly = dx * math.sin(a) + dy * math.cos(a)
    return (abs(lx) <= o["W"] / 2.0 + margin
            and abs(ly) <= o["D"] / 2.0 + margin)


def clear_oblique(vp, x, y, obstacles, margin_m=3.0, step_m=12.0, tries=8):
    """Push a `fire_view_params` oblique camera OUT along its own azimuth —
    and, only as a last resort, UP over the blocker — until the camera point
    sits inside no neighbour's footprint.

    `fire_view_params`' standoff arithmetic was written on benches with
    empty ground around every row; in the packed 500 m city a 50-100 m
    standoff lands inside a neighbouring building often enough that 3 of 32
    `city_v3` obliques were photographs of the inside of a wall
    (d19/d31/d49). `obstacles` is `load_dump_positions`' value list (every
    placement, `H` included) minus the subject itself; an obstacle shorter
    than the camera (plus 1 m clearance) never blocks. The bench launchers
    pass no obstacles and are untouched. Returns an adjusted COPY of `vp`
    and the number of push-out steps taken (0 = original framing kept).
    """
    vp = dict(vp)

    def solve(az_deg):
        """(dist, height, cost) once the camera at this bearing is pushed
        out of every footprint and raised over the sightline."""
        a = math.radians(az_deg)
        ux, uy = math.cos(a), math.sin(a)

        def blockers(d, h):
            px, py = x + d * ux, y + d * uy
            return [o for o in obstacles
                    if (o.get("H") or 1e9) + 1.0 > h
                    and _point_in_footprint(px, py, o, margin_m)]

        d, h = vp["obl_dist"], vp["obl_h"]
        for _ in range(tries):
            if not blockers(d, h):
                break
            d += step_m
        else:
            blk = blockers(d, h)
            if blk:
                h = max(h, max((o.get("H") or 0.0) for o in blk) + 6.0)
        h = raise_over_sightline(x, y, vp["aim_h"], d, az_deg, h, obstacles)
        return d, h

    # A camera OUTSIDE every footprint can still photograph the back of a
    # neighbour (city_v4: d6/d24/wave nose-to-wall against a tower), and in
    # a downtown, clearing a sightline by HEIGHT alone climbs over 200 m
    # towers and degrades the oblique into a second top view. So scan
    # bearings around the fire-facing azimuth (+-45 deg keeps a burning
    # side in frame), pay a small penalty per degree of swing, and keep
    # whichever needs the least final eye height.
    base = vp["azimuth_deg"]
    best = None
    for off in (0, 15, -15, 30, -30, 45, -45):
        d, h = solve(base + off)
        cost = h + 0.22 * abs(off)
        if best is None or cost < best[0]:
            best = (cost, base + off, d, h)
    _, az, d, h = best
    moved = int(abs(az - base) > 0.1) + int(d > vp["obl_dist"] + 0.1) \
        + int(h > vp["obl_h"] + 0.5)
    vp["azimuth_deg"], vp["obl_dist"], vp["obl_h"] = az, d, h
    return vp, moved


def raise_over_sightline(x, y, aim_h, dist, azimuth_deg, h, obstacles,
                         clearance_m=4.0, n_samples=24, h_cap=340.0,
                         margin_m=1.5):
    """The eye height needed so the straight line from the camera
    (`dist` out along `azimuth_deg`, height `h`) to the aim point
    (`x, y, aim_h`) passes OVER every obstacle footprint it crosses.

    Pure 2D-plus-height: the segment is sampled at `n_samples` points; a
    sample inside an obstacle whose H exceeds the line's height there
    blocks, and the eye must rise to `aim_h + (H + clearance - aim_h)/s`
    (s = the sample's fraction of the way from aim to eye — a blocker NEAR
    THE CAMERA forces almost no rise, one near the subject forces a lot,
    which is why raising beats pushing for a canyon). Iterated to a fixed
    point (a higher eye tilts the line over new candidates only, never
    into them, so two passes settle it); capped at `h_cap` — past that a
    top-down exists and reads better anyway. Returns the (possibly
    unchanged) eye height.
    """
    a = math.radians(azimuth_deg)
    ux, uy = math.cos(a), math.sin(a)
    for _ in range(3):
        need = h
        for k in range(1, n_samples):
            s = k / float(n_samples)          # 0 = aim point, 1 = eye
            px, py = x + s * dist * ux, y + s * dist * uy
            lz = aim_h + s * (h - aim_h)
            for o in obstacles:
                top = o.get("H") or 0.0
                if top <= lz:
                    continue
                if _point_in_footprint(px, py, o, margin_m):
                    need = max(need,
                               aim_h + (top + clearance_m - aim_h) / s)
        if need <= h + 0.5 or h >= h_cap:
            break
        h = min(need, h_cap)
    return h


# ---------------------------------------------------------------------------
# SCORCHED VEGETATION NEAR A BURNING BUILDING (2026-08-31)
# ---------------------------------------------------------------------------
# "if debris falls on trees, etc they can catch on fire so make them
# scorched... i don't really need debris there" (user, 2026-08-31) — no wood
# debris, no felling, no fracture: `vegetation.py`'s `burn_tree` pipeline
# (survey -> defoliate -> scorch_foliage/char_bole -> wood_debris/topple) is
# the WILDFIRE model, built for scenes where the tree IS the disaster and
# priced accordingly (per-tree MDL texture synthesis, new debris meshes). At
# CITY SCALE, with the tree only a bystander to a structure fire, the user's
# own instruction is the simpler bar: "a flat dark bind on leaf materials is
# acceptable". So this reuses exactly two things from `vegetation.py` — its
# `survey()` classification (which mesh/PointInstancer prototype is leaf vs.
# wood; it already knows a PointInstancer's material lives on its PROTOTYPE,
# which may be a subtree this pass would otherwise walk right past) and
# `_kind`/`_bound`'s MDL-aware material typing — and binds ONE flat charred
# material per role, built once for the whole city from `damage._pbr` with
# the same char/scorch RGB tones `urban_fire`'s own `_burn_set` draws from.
# Trees outside a fire's reach are never touched, so they "stay green" by
# construction (nothing here scans or rebinds them).
VEG_RADIUS_MULT = {"F2": 1.2, "F3": 1.3, "F4": 1.5, "F5": 1.7,
                   "F5c": 1.9, "F6": 2.1}
#: a squat building still reaches its own kerb trees — `1.2 x H` on an 6 m
#: two-storey shopfront is 7.2 m, inside a single street-tree bay.
VEG_RADIUS_FLOOR_M = 8.0


def veg_scorch_radius_m(level, height_m):
    """Vegetation scorch reach for one damaged building, in metres.

    0.0 below F2 — "level>=F2" per the brief: F0/F1 barely mark the shell,
    let alone anything standing near it, and F1 already carries its own
    wisp-only fire state (`fire_assembly_lib.place_fire`'s `wisp_only`).
    F2 upward scales `VEG_RADIUS_MULT` x the building's own MEASURED height
    (its composed bbox, so a partial collapse's shorter remaining shell
    reaches less far than its original height would have), floored at
    `VEG_RADIUS_FLOOR_M` so a short building is not given a radius smaller
    than the gap to its own street trees.
    """
    mult = VEG_RADIUS_MULT.get(level)
    if not mult:
        return 0.0
    return max(VEG_RADIUS_FLOOR_M, mult * max(0.0, float(height_m)))


def _dist_point_to_box_xy(x, y, box):
    """Nearest-point distance, in the XY plane, from `(x, y)` to an
    axis-aligned footprint `box = [x0, y0, z0, x1, y1, z1]` (this module's
    own `bbox()` shape) — 0.0 when the point is inside or on the footprint,
    so a tree standing inside a building's own settled street debris is
    never treated as "far from the fire"."""
    x0, y0, x1, y1 = float(box[0]), float(box[1]), float(box[3]), float(box[4])
    dx = max(x0 - x, 0.0, x - x1)
    dy = max(y0 - y, 0.0, y - y1)
    return math.hypot(dx, dy)


def vegetation_scorch_targets(fuels, buildings):
    """`{prim_path: {"dist_m", "radius_m", "i", "level"}}` for every fuel
    placement within its nearest qualifying building's `veg_scorch_radius_m`.

    `fuels` is `fire.select_fuels(placements)`'s own `(x_m, y_m, prim_path)`
    triples (tree/plant/shrub/hedge/bush, matched by the SAME substring rule
    the wildfire spread solver uses, so "greenery" here means exactly what
    it means everywhere else in this codebase). `buildings` is
    `[{"i", "box", "level"}, ...]`, one entry per placed bake; an entry
    missing a box or level (no bake composed, or F0/F1) contributes nothing.

    A fuel in reach of more than one fire keeps the call with the larger
    MARGIN (radius minus distance) rather than the nearest fire outright —
    deterministic, order-independent, and biased toward whichever fire most
    clearly reaches it rather than whichever building merely happens to be
    closest.
    """
    out = {}
    for x, y, path in fuels:
        if not path:
            continue
        best = None
        for b in buildings:
            box = b.get("box")
            level = b.get("level")
            if not box or not level:
                continue
            h = float(box[5]) - float(box[2])
            r = veg_scorch_radius_m(level, h)
            if r <= 0.0:
                continue
            d = _dist_point_to_box_xy(float(x), float(y), box)
            if d > r:
                continue
            margin = r - d
            if best is None or margin > best[0]:
                best = (margin, d, r, b.get("i"), level)
        if best is not None:
            out[path] = {"dist_m": best[1], "radius_m": best[2],
                        "i": best[3], "level": best[4]}
    return out


def scorch_materials(stage, root):
    """The two flat charred materials every scorched tree binds — built ONCE
    for the whole city under `<root>/VegLooks`, so the prim cost of this pass
    is 2 materials total, not 2 per tree.

    `damage._pbr` with no texture (a flat OmniPBR, exactly the "flat dark
    bind" the brief allows) at `damage`'s own char/scorch RGB tones — the
    same tones `urban_fire._burn_set` draws its char/scorch/ash set from, so
    a scorched street tree reads as the same fire that damaged the building
    behind it rather than a separately-invented palette.
    """
    from . import damage

    leaf = damage._pbr(stage, root + "/VegLooks/leaf_char",
                       damage._CHAR_RGB, 0.95)
    trunk = damage._pbr(stage, root + "/VegLooks/trunk_char",
                        damage._SCORCH_RGB, 0.9)
    return {"leaf": leaf, "trunk": trunk}


def _bind_at_subset_granularity(prim, mat):
    """Bind `mat` at whatever granularity this prim's OWN material already
    lives at — its GeomSubsets if it has any, the prim itself otherwise.

    Mirrors `vegetation._bound`'s read side exactly, so an override binding
    authored here can never be masked by a more specific existing one — the
    same rule `urban_fire._bind_subsets` documents for a façade: "a whole-
    module bind is fatal ... the next pass's ordinary per-subset bind is
    then silently ignored". `_bound`/`_kind` are what CHOSE `mat` for this
    prim in the first place, so binding anywhere else would rebind the wrong
    granularity relative to what was just read.
    """
    subs = list(UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(prim)))
    targets = [s.GetPrim() for s in subs] or [prim]
    for t in targets:
        UsdShade.MaterialBindingAPI(t).Bind(mat)
    return len(targets)


def _bind_prototypes(stage, pi_path, mat):
    """Rebind every Mesh under a PointInstancer's PROTOTYPE subtree(s).

    The instancer prim itself carries no drawable geometry — the prototype
    does, and `vegetation.survey` already worked out which prototypes back
    THIS instancer's `leaf_pi`/`wood_pi` entry, so this only has to walk
    `GetPrototypesRel()` and bind what it finds.
    """
    pi_prim = stage.GetPrimAtPath(pi_path)
    if not pi_prim or not pi_prim.IsValid():
        return 0
    n = 0
    for t in UsdGeom.PointInstancer(pi_prim).GetPrototypesRel().GetTargets():
        proto = stage.GetPrimAtPath(t)
        if not proto or not proto.IsValid():
            continue
        for q in Usd.PrimRange(proto):
            if not q.IsA(UsdGeom.Mesh):
                continue
            n += _bind_at_subset_granularity(q, mat)
    return n


def apply_vegetation_scorch(stage, target_paths, root):
    """Bind the charred leaf/trunk materials onto every tree at `target_paths`.

    Runs `vegetation.survey` on each (the SAME classification the wildfire
    pipeline trusts: direct "bole" meshes plus `leaf_pi`/`wood_pi`
    PointInstancer entries), then rebinds bole meshes at their own subset
    granularity and instancer prototypes whole. `survey` already refuses an
    `IsInstance()` tree with a printed warning and an empty result, so a
    city built with `instanceable=true` street trees (it is not, today —
    `apply_placements` references each placement onto its own prim path) is
    a silent no-op here rather than a crash.
    """
    from . import vegetation as veg

    mats = scorch_materials(stage, root)
    n_leaf = n_trunk = n_trees = 0
    for path in target_paths:
        # AN INSTANCED TREE MUST BE DE-INSTANCED BEFORE THE REBIND. The 500 m
        # city now places vegetation `instanceable` (66,590 placements of 87
        # USDs OOM-killed composition twice, 2026-08-31), and USD forbids
        # edits inside an instance — `survey` would refuse it as a silent
        # no-op. Only the scorch TARGETS pay the de-instancing cost: a
        # handful of trees near the fires, while the green thousands stay
        # shared prototypes.
        _p = stage.GetPrimAtPath(path)
        if _p and _p.IsValid() and _p.IsInstanceable():
            _p.SetInstanceable(False)
        info = veg.survey(stage, path)
        touched = False
        for mesh_path, _mat_path, _tex, kind in info.get("bole", []):
            prim = stage.GetPrimAtPath(mesh_path)
            if not prim or not prim.IsValid():
                continue
            mat = mats["leaf"] if kind == "leaf" else mats["trunk"]
            _bind_at_subset_granularity(prim, mat)
            touched = True
            if kind == "leaf":
                n_leaf += 1
            else:
                n_trunk += 1
        for pi_path, _n in info.get("leaf_pi", []):
            k = _bind_prototypes(stage, pi_path, mats["leaf"])
            n_leaf += k
            touched = touched or k > 0
        for pi_path, _n in info.get("wood_pi", []):
            k = _bind_prototypes(stage, pi_path, mats["trunk"])
            n_trunk += k
            touched = touched or k > 0
        if touched:
            n_trees += 1
    return {"trees": n_trees, "leaf_binds": n_leaf, "trunk_binds": n_trunk}


def scorch_vegetation_pass(stage, placements, placed_rows, root):
    """The full scorched-vegetation pass, end to end.

    `placements` is the city's FULL placement list (`FireCityApp.placements`
    — `fire.select_fuels` does its own category filtering); `placed_rows` is
    `FireCityApp.placed`, each a bake row carrying `i`/`bbox`/`doc`.
    """
    buildings = [{"i": r.get("i"), "box": r.get("bbox"),
                 "level": ((r.get("doc") or {}).get("fire") or {})
                 .get("level")}
                for r in placed_rows]
    fuels = fx.select_fuels(placements)
    targets = vegetation_scorch_targets(fuels, buildings)
    stats = apply_vegetation_scorch(stage, list(targets.keys()), root)
    stats["fuels_total"] = len(fuels)
    stats["scorched"] = len(targets)
    stats["targets"] = targets
    return stats


# ---------------------------------------------------------------------------
# FIRE-SIDE DEBRIS APRON for a NON-COLLAPSE burning building (2026-08-31)
# ---------------------------------------------------------------------------
# "for the ones without a partial or full collapse, I want smaller debris
# particles on the sides that are on fire" (user, 2026-08-31) — F1..F5 only;
# F5c/F6 already drop an authored collapse heap (`fire_collapse`) and this
# apron is not meant to compete with (or duplicate) that rubble.
#
# ONE MESH PER BUILDING, NOT ONE PRIM PER LUMP. `quake_flow._a_lump` authors
# one `UsdGeom.Mesh` PER CALL — right for a collapse heap's few hundred
# chunks on ONE building, wrong here: with a global scatter over a whole
# city (up to `APRON_MAX_PER_SIDE` lumps per venting side, several sides per
# building, ~26 buildings) that idiom would cost hundreds of prims for
# geometry that never needs independent transforms or physics (the brief:
# "no debris needs authoring on them [trees]... Instanced or merged geometry
# preferred... keep the prim count bounded"). `_lump_points` reproduces
# `_a_lump`'s exact per-lump box/jitter/rotation math (so the LOOK is
# unchanged) but returns its 8 points already offset to a world seat instead
# of authoring a Mesh with a translate op, so `author_merged_lumps` can pack
# every lump for a building into ONE Mesh's points/faces — 1 prim (plus up
# to 2 material GeomSubsets) per building regardless of lump count.
APRON_LEVELS = ("F1", "F2", "F3", "F4", "F5")
# DENSITY, ROUND 2 (2026-08-31, same day): "the minor debris that is supposed
# to look like it's fallen off from the burning building onto the floor needs
# to increase" (user). The round-1 table above was tuned only against the
# gate/prim-budget math, never against how it actually reads at review
# distance on a real 39-building manifest — roughly DOUBLED here (F5 17->34)
# and `APRON_MAX_PER_SIDE` raised to match, so a wide GAC/downtowncity
# elevation (15-70 m — the case the round-1 comment on `r_street_debris`
# above already flagged as under-served at the OLD cap) actually gets more
# lumps instead of saturating the same ceiling. Cost is still ONE merged
# mesh per building regardless of count (see `author_merged_lumps`), so this
# is cheap: doubling the lump count roughly doubles one building's triangle
# count, not its prim count. `scale` (below) is the live-tunable multiplier
# on TOP of this table — `FA_APRON_SCALE` at the launcher.
APRON_DENSITY = {"F1": 8, "F2": 12, "F3": 18, "F4": 26, "F5": 34}
#: the wall length the base `APRON_DENSITY` count is tuned for
APRON_REF_SPAN_M = 14.0
#: bounds the merged mesh's triangle count even on a long block face
APRON_MAX_PER_SIDE = 70
#: a lump's nominal size, metres — "glass-scale" charred debris, not a
#: collapse chunk (`quake_flow._heap`'s own chunks run 1-3 m)
APRON_SIZE_M = (0.10, 0.28)
#: how far out from the wall face the scatter sits, onto the sidewalk.
#: Widened alongside the density bump (was 0.25-1.3 m) so a chip can land a
#: little further out toward the kerb instead of every one hugging the wall
#: — "a modest scatter reaching into the street is fine". Still nowhere near
#: a road: `layout/city_layout.py`'s own road-hierarchy offset math keeps
#: "5 m for the sidewalk a real block still keeps between its building line
#: and the kerb" BEFORE any lane width is added, so a 2.2 m max setback
#: leaves a >=2.8 m margin even on the narrowest modelled frontage, before
#: counting the kerb-to-centreline distance of the road itself at all (see
#: `test_apron_setback_stays_off_the_road` in `tests/test_fire_city_dressing.py`).
APRON_SETBACK_M = (0.3, 2.2)
#: kept clear of the corners (a fraction of the wall's own span), so a lump
#: never lands past the return wall this side does not own
APRON_INSET_FRAC = 0.08
#: ALWAYS THE DARK END — `urban_fire._debris_mat`'s own rule and ratio for
#: anything lying on the ground rather than on a visible elevation.
APRON_CHAR_P = 0.72
#: `layout/city_layout.py`'s own documented clearance — the sidewalk a real
#: block keeps between a building's own line and the kerb, before any lane
#: width. `APRON_SETBACK_M`'s upper bound must stay well under this (see the
#: comment on `APRON_SETBACK_M`, and the test that pins it down).
APRON_ROAD_CLEARANCE_M = 5.0


def apron_count(level, span_m, scale=1.0):
    """How many small debris lumps one venting SIDE gets.

    Density scales with LEVEL (F1 a few wisps of char, F5 a real scatter)
    and with the wall's own length, referenced to `APRON_REF_SPAN_M` so a
    long block face is not as sparse as a narrow row-house front. Floored at
    2 (once a level qualifies at all, there is always SOMETHING to see) and
    capped at `APRON_MAX_PER_SIDE`.

    `scale` is the live knob (`FA_APRON_SCALE` at the launcher) for tuning
    the whole apron up or down post-hoc without editing `APRON_DENSITY`
    itself — applied AFTER the span scaling, BEFORE the floor/cap, so a
    scale under 1 still cannot suppress a qualifying level entirely (the
    floor of 2 is scale-independent: "once a level qualifies at all, there
    is always something to see" holds regardless of the knob).
    """
    base = APRON_DENSITY.get(level)
    if not base:
        return 0
    n = int(round(base * max(0.6, float(span_m) / APRON_REF_SPAN_M)
                 * max(0.0, float(scale))))
    return max(2, min(APRON_MAX_PER_SIDE, n))


def apron_points_for_side(m, side, level, rng, scale=1.0):
    """`[(x, y, size_m), ...]` world-space seats along one venting side's
    base wall line, offset outward onto the ground.

    `m` must already be in the building's CITY frame (see `world_masses` —
    `quake_flow._to_world`/`_outward`/`_p_wall_point` all read `m["cx"]`/
    `["cy"]`/`["yaw"]`, which are the bake-LOCAL origin until something
    applies the cell's placement). Z is deliberately not decided here: it is
    the caller's concern (the building's own MEASURED ground contact, e.g.
    this module's own `bbox()`), not this mass's analytic `z0`, because
    `fire_bake.place` never touches `z0` (a yaw about the vertical axis does
    not move height, and the city cell's own dz is applied by the holder's
    translate, not by the mass).
    """
    from . import quake_flow as qf

    span = qf._a_side_span(m, side)
    n = apron_count(level, span, scale=scale)
    if n <= 0:
        return []
    ox, oy = qf._outward(m, side)
    lo, hi = span * APRON_INSET_FRAC, span * (1.0 - APRON_INSET_FRAC)
    if hi <= lo:
        lo, hi = 0.0, span
    out = []
    for _ in range(n):
        t = rng.uniform(lo, hi)
        lx, ly = qf._p_wall_point(m, side, t)
        wx, wy = qf._to_world(m, lx, ly)
        d = rng.uniform(*APRON_SETBACK_M)
        s = rng.uniform(*APRON_SIZE_M)
        out.append((wx + ox * d, wy + oy * d, s))
    return out


def world_masses(masses, x, y, yaw_deg):
    """A DEEP COPY of `masses`, rotated+translated to its city cell.

    Exactly `fire_bake.place`'s own transform, run on a COPY: a caller that
    only needs a building's WORLD-SPACE wall lines (this apron pass) must
    not mutate the ORIGINAL `masses` dict, because `put_the_fire_back` (the
    city launcher's own step 4) later calls `fire_bake.place` on that same
    original once it decides the building's Flow emitter allocation —
    calling `place` on the same dict twice would rotate+translate it AGAIN,
    silently doubling the transform for whichever pass ran second.
    """
    import copy

    from . import fire_bake as fb

    out = copy.deepcopy(masses)
    fb.place(out, [], None, x, y, yaw_deg)
    return out


_LUMP_FACES = ((0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4),
              (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7))


def _lump_points(cx, cy, cz, s, rng, jitter=0.3):
    """One `quake_flow._a_lump`-shaped box's 8 points, already offset to a
    WORLD seat `(cx, cy, cz)`.

    Reproduces `_a_lump`'s box/jitter/three-axis-rotation math verbatim (the
    look must not change) but returns points instead of authoring a Mesh
    with a translate op, so `author_merged_lumps` can pack many lumps into
    one Mesh's own point array.
    """
    hx = s * 0.5
    hy = s * rng.uniform(0.45, 1.0) * 0.5
    hz = s * rng.uniform(0.35, 0.85) * 0.5
    ya, pa, ra = (rng.uniform(0.0, 2.0 * math.pi), rng.uniform(-0.9, 0.9),
                  rng.uniform(-0.9, 0.9))
    cya, sya = math.cos(ya), math.sin(ya)
    cp, sp = math.cos(pa), math.sin(pa)
    cr, sr = math.cos(ra), math.sin(ra)
    pts = []
    for dz in (-hz, hz):
        for dx, dy in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)):
            x = dx * (1.0 + rng.uniform(-jitter, jitter))
            y = dy * (1.0 + rng.uniform(-jitter, jitter))
            z = dz * (1.0 + rng.uniform(-jitter, jitter))
            y, z = y * cp - z * sp, y * sp + z * cp
            x, z = x * cr - z * sr, x * sr + z * cr
            x, y = x * cya - y * sya, x * sya + y * cya
            pts.append((cx + x, cy + y, cz + z))
    return pts


def author_merged_lumps(stage, path, seats, rng, mat_char=None,
                        mat_scorch=None, char_p=APRON_CHAR_P):
    """One Mesh prim carrying every lump in `seats` (`[(x, y, z, size), ...]`).

    Each lump is independently assigned char or scorch (weighted `char_p`,
    `urban_fire._debris_mat`'s own ratio) via TWO `materialBind` GeomSubsets
    — `UsdShade.Tokens.materialBind` / `UsdGeom.Tokens.partition` are BOTH
    required on `CreateGeomSubset`, or the subset is silently ignored by the
    renderer and the whole mesh falls back to one material (the exact trap
    `surge.py`'s `build_ponding` already documents and works around). Falls
    back to a WHOLE-mesh bind (whichever material was given) so a caller
    that wants the apron in one tone can pass only `mat_char`.

    Returns `(prim, n_lumps)`; `(None, 0)` for an empty scatter — no prim is
    authored for a building with nothing to place.
    """
    if not seats:
        return None, 0
    points, counts, indices = [], [], []
    char_faces, scorch_faces = [], []
    for x, y, z, s in seats:
        base = len(points)
        points.extend(_lump_points(x, y, z, s, rng))
        face_start = len(counts)
        for f in _LUMP_FACES:
            counts.append(4)
            indices.extend(base + k for k in f)
        faces = list(range(face_start, len(counts)))
        (char_faces if rng.random() < char_p else scorch_faces).extend(faces)
    me = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
    me.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in points]))
    me.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
    me.CreateFaceVertexIndicesAttr(Vt.IntArray(indices))
    me.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    lo = [min(p[k] for p in points) for k in range(3)]
    hi = [max(p[k] for p in points) for k in range(3)]
    me.CreateExtentAttr([Gf.Vec3f(*lo), Gf.Vec3f(*hi)])
    fallback = mat_char if mat_char is not None else mat_scorch
    if fallback is not None:
        UsdShade.MaterialBindingAPI.Apply(me.GetPrim()).Bind(fallback)
    if mat_char is not None and char_faces:
        sub = UsdGeom.Subset.CreateGeomSubset(
            me, "apronChar", UsdGeom.Tokens.face, Vt.IntArray(char_faces),
            UsdShade.Tokens.materialBind, UsdGeom.Tokens.partition)
        UsdShade.MaterialBindingAPI.Apply(sub.GetPrim()).Bind(mat_char)
    if mat_scorch is not None and scorch_faces:
        sub = UsdGeom.Subset.CreateGeomSubset(
            me, "apronScorch", UsdGeom.Tokens.face, Vt.IntArray(scorch_faces),
            UsdShade.Tokens.materialBind, UsdGeom.Tokens.partition)
        UsdShade.MaterialBindingAPI.Apply(sub.GetPrim()).Bind(mat_scorch)
    return me.GetPrim(), len(seats)


def apron_debris_materials(stage, root):
    """The two flat debris tones the apron draws from — built ONCE for the
    whole city under `<root>/DebrisLooks`. ALWAYS THE DARK END: same rule
    and RGB tones `urban_fire._debris_mat`/`damage._pbr` use for anything
    lying on the ground rather than on a visible elevation."""
    from . import damage

    char = damage._pbr(stage, root + "/DebrisLooks/apron_char",
                       damage._CHAR_RGB, 0.92)
    scorch = damage._pbr(stage, root + "/DebrisLooks/apron_scorch",
                         damage._SCORCH_RGB, 0.88)
    return {"char": char, "scorch": scorch}


def build_fire_apron(stage, root, r, rng, scale=1.0):
    """The debris apron for one placed bake, or a no-op note for one that
    does not qualify.

    `r` is one `FireCityApp.placed` row (`i`/`stem`/`x`/`y`/`yaw`/`bbox`/
    `doc`/`masses`). Gated on `doc["fire"]["level"]` in `APRON_LEVELS`
    (F1..F5 — no F5c/F6, which already drop a collapse heap) and on having
    at least one recorded venting side (`doc["fire"]["sides"]`); a building
    failing either gate gets `"prim": None` and a `"note"` saying why.

    `scale` is `apron_count`'s own live density multiplier, passed straight
    through (`FA_APRON_SCALE` at the launcher).
    """
    doc = r.get("doc") or {}
    f = doc.get("fire") or {}
    level = f.get("level")
    if level not in APRON_LEVELS:
        return {"prim": None, "n": 0, "sides": (), "level": level,
                "note": "level {0!r} not F1-F5 (collapse or unburnt)"
                        .format(level)}
    sides = tuple(f.get("sides") or ())
    if not sides:
        return {"prim": None, "n": 0, "sides": (), "level": level,
                "note": "no venting sides recorded"}
    tag = f.get("mass") or "main"
    masses = r.get("masses") or {}
    world = world_masses(masses, float(r.get("x", 0.0)),
                         float(r.get("y", 0.0)), float(r.get("yaw", 0.0)))
    m = world.get(tag) or (list(world.values())[0] if world else None)
    if not m:
        return {"prim": None, "n": 0, "sides": sides, "level": level,
                "note": "mass {0!r} not found in this bake's sidecar"
                        .format(tag)}
    box = r.get("bbox")
    z0 = float(box[2]) if box else float(m.get("z0", 0.0))
    seats = []
    for side in sides:
        for x, y, s in apron_points_for_side(m, side, level, rng, scale=scale):
            seats.append((x, y, z0, s))
    mats = apron_debris_materials(stage, root)
    path = "{0}/apron/{1}".format(root, r.get("stem", "d{0}".format(r.get("i", 0))))
    prim, n = author_merged_lumps(stage, path, seats, rng,
                                  mat_char=mats["char"],
                                  mat_scorch=mats["scorch"])
    return {"prim": str(prim.GetPath()) if prim else None, "n": n,
            "sides": sides, "level": level}


def fire_apron_pass(stage, root, placed_rows, seed=7, scale=1.0):
    """The full fire-side debris apron pass over every placed bake.

    One `build_fire_apron` call per row, with a per-building rng so the
    scatter is stable given a stable `FA_SEED` without sharing draws across
    buildings (the same discipline `place_fire`'s own per-building
    `random.Random(SEED + 31 * r["i"])` follows). `scale` is the density
    multiplier (`FA_APRON_SCALE`), applied identically to every building.
    """
    out = []
    for r in placed_rows:
        rng = random.Random(int(seed) + 97 * int(r.get("i") or 0))
        res = build_fire_apron(stage, root, r, rng, scale=scale)
        res["i"] = r.get("i")
        res["stem"] = r.get("stem")
        out.append(res)
    return out
