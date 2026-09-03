"""tornado_urban — the URBAN tornado ladder for SLICED whole-asset buildings.

`quake_sliced` damages the SAME piece grid (`quake_flow.describe`'s element
table over a GAC/downtowncity slice or a real kit) for an EARTHQUAKE. This
module damages it for a TORNADO, and the two are deliberately separate
modules rather than a flag on one, because the physics do not share a
vocabulary:

  * an earthquake shakes the WHOLE building, roughly evenly on every face,
    and its severity is a property of the STRUCTURE (how the frame or the
    masonry responds); a tornado hits ONE SIDE — the windward face and its
    corners take almost everything, the leeward face takes almost nothing,
    and severity is a property of WIND PRESSURE + DEBRIS IMPACT, which is a
    function of which way a wall faces relative to a bearing, not of what
    the wall is made of;
  * an earthquake's ladder runs to total collapse (`pancake`,
    `masonry_collapse`) for the buildings that can take it; a tornado
    ladder NEVER collapses a structure and NEVER empties a storey — the
    user's constraint (`_plans/urban_tornado_plan.md` §0): "no completely
    demolished skyscrapers... kinda like the tornado hit the side of it."
    The record agrees (Lubbock 1970, Fort Worth 2000, Joplin 2011: gutted
    envelopes, standing frames) and this ladder has no state past T4 for
    exactly that reason — there is deliberately no T5;
  * every removed piece here has a DIRECTION: it lands downwind of where it
    stood, not in a heap at the building's own foot. `quake_sliced`'s pile
    specs (`_pile`, `disaster.quake_rubble`) are a dome over the footprint;
    this module's debris ledger (`_ledger_removed`) is a projectile model —
    release height, drag-scaled reach, a bearing plus gaussian scatter, and
    a push-out so a fragment never lands inside its own source building.

WHAT THIS MODULE REUSES, AND WHY THAT IS SAFE
-----------------------------------------------
`quake_sliced` already solved "how do you damage a piece grid with no
fracture" (removal on the grid, TOOTHED so a lost region never reads as a
rectangular module cut-out; rigid displacement of whole pieces about their
own bottom outer edge for a macroblock). That is grid arithmetic, not
earthquake-specific, so this module imports it directly rather than copying
it: `quake_sliced._Grid` (the same side/storey/bay index over the same
placement dicts), `_apply_region` (the exact toothing draw), `_disp` /
`rigid_matrix` / `apply_rigid` (the same displacement-spec contract, so a
macroblock or a hanging panel here is checkable with the identical matrix
code `quake_sliced`'s own tests use), `is_opening` / `is_pier` / `bay_no` /
`sub_ix` (the slicer's own bay grammar), `_CORNER_SIDES` / `_CORNER_END` /
`_SIDE_NORMAL` / `SIDES` (the side vocabulary). What is NOT reused:
`quake_sliced._pick_sides` (an earthquake building has no preferred face; a
tornado building's face is picked by the WIND, never by a shuffle) and
`quake_sliced._pile` / the rubble pile machinery (a tornado's debris is
thrown, not mounded).

WHAT THIS MODULE DELIBERATELY NEVER DOES
------------------------------------------
No collapse recipe of any kind (no `pancake`, `masonry_collapse`,
`storey_collapse`, `soft_storey`, `tilt_sink`, `overturn` — none of
`quake_sliced`'s foundation family either, since a tornado does not touch a
building's footing). No storey is ever left with zero standing pieces. No
`core` or `roof` piece is ever removed, except the ONE named exception in
the record: a lowrise URM building at very high intensity can lose its top
storey's exterior walls and shed its roof piece as debris while everything
below still stands (`top_storey_loss`) — that piece is deliberately ledgered
as debris, not silently dropped. No ground-storey pier is ever removed on a
TOWER (it reads as a collapse in an aerial frame even though nothing else
did). No out-of-plane peel above midrise (`HEIGHT_CAPS[...]["out_of_plane"]`)
— a curtain-wall highrise does not lose a macroblock of skin the way a
lowrise brick front does.

THE SHAPE OF THE CODE
----------------------
    level_for_intensity(i, rng, jitter=0.06) -> "T0".."T4"
        The EF-proxy cut table, `tornado._ladder`'s own jitter idiom.
    height_class_for(H, typology=None) -> "lowrise"/"midrise"/"highrise"/"tower"
        Drives the HEIGHT_CAPS a `_guard` and a post-hoc cap enforce.
    side_weights(info, wind, rng=None) -> {"S":.., ..., "SW":.., ...}
        Wind-loading weight per elevation and per corner. Every recipe below
        reads this instead of shuffling sides.
    plan_damage(info, elements, level, btype, rng, wind,
                height_class=None, intensity=None) -> plan
        PURE. No stage, no pxr — arithmetic on the element table exactly as
        `quake_sliced.plan_damage` is pure, and checked the same way (no
        USD anywhere above `apply_plan`, which is `disaster/
        tornado_urban_usd.py`'s job, NOT this module's).

THE PLAN SCHEMA (JSON-serialisable throughout; `_plans/urban_tornado_plan.md`
§2.8 is the contract this mirrors — every number here is a plain float,
every path a string, every collection a list, sorted where order would
otherwise vary run to run):

    schema, level, btype, style, H, height_class      what was asked for
    wind        {bearing_deg, speed_frac, cross_frac, over}
    side_weights {S/E/N/W/SW/SE/NW/NE: 0..1}
    recipes     [[name, kwargs], ...]      the GUARDED list actually run
    removed     [prim_path, ...]           deactivate these
    displaced   {prim_path: disp_spec}     `quake_sliced.rigid_matrix` spec;
                                           hanging panels, top-wall macroblocks
    glass       [prim_path, ...]           panes -> void material
    glass_bands [{...}]                    per-recipe glass record
    regions     [{recipe, side/corner, storeys, bays, cells}]
    roof_props  "sweep" | "keep"
    debris      [{kind, size:[l,w,t], x, y, z, z_lift, yaw_deg, tilt_deg,
                  material, from: prim_path, stacked,
                  source_tex, source_tex_name, tone, shade}, ...]
                 ROUND 3 (§8 R8) ADDS `z_lift`/`stacked` — NEVER a rename of
                 `z` (still always `0.0`, street grade, unchanged): `z_lift`
                 is a berm fragment's extra height WITHIN its own wall-base
                 heap (0 for every ballistic/downwind fragment), `stacked`
                 is `z_lift > 0.0` and doubles as the berm-membership flag.
                 See `_deposit_berm`'s docstring for why `z_lift` is authored
                 but not yet CONSUMED (`tornado_urban_usd.build_debris`'s
                 `_seat_z` flattens every fragment to grade today — verified,
                 not assumed; a future builder pass adds `z_lift` into its
                 own `ground_z`).
                 ROUND 3b (§8e F3) ADDS `source_tex`/`source_tex_name` —
                 ANOTHER pure addition, `material` untouched: the removed
                 piece's own bound cladding texture (`_tex_url`/`_tex_name`,
                 stamped onto the placement dict by `tornado_urban_usd.
                 annotate_surface` before `plan_damage` ever runs), copied
                 onto every FAÇADE fragment (`kind` in `panel`/`block`/
                 `coping` — a `wall`/`pier`/`corner`/`parapet` piece) it
                 sheds, berm and ballistic alike. Both are the empty string
                 `""` when the piece carried no texture OR when `kind` is
                 `membrane`/`metal` (a ROOF-shed piece — "a torn membrane is
                 roofing, not façade") OR `glass` (its own void look) — see
                 `_FACADE_TEX_KINDS` and `_ledger_removed`'s own comment.
                 `tornado_urban_usd.debris_material` is the consumer: one
                 triplanar material per distinct `source_tex_name`, tinted
                 by the fragment's own class bucket, so the debris at a
                 grey-stone brownstone's own foot is grey stone and not a
                 flat class-average colour.
                 ROUND 4 (v6 review) ADDS `tone`/`shade` — two more pure
                 additions. `tone` is the SOURCE BUILDING's masonry colour
                 token (`_KIT_TONE`/`_tone_for`: `"stone"`, `"tan"`, or
                 `""`), stamped on BLOCKY fragments only, and it exists
                 because a kit facade map is an ATLAS that cannot be tiled
                 onto debris (`tornado_urban_usd._tiling_safe`) — without
                 it a WHITE-STONE building's rubble takes the generic red
                 brick class map, which the v6 bench measured on B1/B3.
                 `shade` (0..`_DEBRIS_SHADES`-1) is the per-mesh tone-jitter
                 band, deterministic on (source piece, fragment index) via
                 `_stable_shade`, never a draw. Both are consumed by
                 `tornado_urban_usd.build_debris`/`debris_material`, which
                 group by them; `""`/`0` reproduce the pre-round look and
                 prim paths exactly.
    notes       [str, ...]
    stats       {n_pieces, n_removed, removed_frac (BY FAÇADE AREA — see
                 `_facade_area_of`), removed_count_frac (by piece count, for
                 reference only, NOT what the height-class cap enforces),
                 n_glass, n_displaced, n_debris, debris_volume_m3,
                 source_volume_m3 (removed pieces' own volume — an
                 approximate, not exact, budget for `debris_volume_m3`; see
                 `_frag_thickness_and_count`), max_removed_storey,
                 removed_by_side, n_glass_shards, glass_shards_thinned,
                 n_struct_debris, struct_debris_thinned, debris_thinned,
                 n_glass_candidates, glass_measured, n_berm, n_berm_kept,
                 berm_share_level, region}

WHERE THIS DEVIATES FROM `_plans/urban_tornado_plan.md`, AND WHY
------------------------------------------------------------------
Every deviation below keeps an invariant the plan states as a MUST (never
demolish a skyscraper, deterministic given a seed, JSON-safe) rather than
inventing a new one; see this module's own docstrings on `side_weights` and
`height_class_for` for the two spots this matters, and the tests /
`tests/test_tornado_urban.py`'s own docstring for what was measured against
a real baked kit to confirm the reading was safe.
"""

import math
import os as _os
import zlib as _zlib

from . import quake_flow as qf
from . import quake_sliced as qs
from . import tornado as trn

# ---------------------------------------------------------------------------
# ROUND 3 (2026-09-01) — THE PLATE REGION CLAMP
# ---------------------------------------------------------------------------
# `_plans/urban_tornado_plan.md` §8, R8: "fragments clamp to the plate region
# (the floating-off-plate defect)". A tall building's own release height `cz`
# can push a raw ballistic `reach` (capped at `REACH_MAX_H * H`, still tens
# of metres on a tower) past the edge of whatever crop window the caller
# actually built ground for — the fragment then floats over nothing once the
# plate itself is a fixed-size island rather than an infinite ground plane.
# `plan_damage` gains an OPTIONAL `region=(x0, y0, x1, y1)` kwarg (world
# metres) so a caller who KNOWS the plate's own bounds can pass them through
# unchanged; every fragment landing point (structural, glass, berm alike) is
# then clamped to `region` shrunk by `_REGION_MARGIN_M` on every side, so a
# fragment never lands within spitting distance of the plate's own edge
# either. `region=None` (the default) is a no-op — existing callers /
# `tools/tornado_urban_probe.py`'s positional-then-kwarg call shape are
# unaffected. `TU_PLATE_REGION` is the SAME idea `wreck_urban`/`wreck_kit`
# need without a call-site edit (those two files belong to another stream
# this round): set the env var to `"x0,y0,x1,y1"` and every `plan_damage`
# call in the process that does not pass `region` explicitly picks it up —
# read ONCE at import time (matching `GLASS_SHARDS_MAX_PER_BUILDING` /
# `DEBRIS_MAX_PER_BUILDING`'s own `_os.environ.get(...)` idiom), not
# per-call, so a launcher sets it once before importing this module.
_REGION_MARGIN_M = 2.0


def _region_from_env():
    raw = _os.environ.get("TU_PLATE_REGION")
    if not raw:
        return None
    try:
        parts = [float(v) for v in raw.replace(" ", "").split(",")]
    except ValueError:
        return None
    if len(parts) != 4:
        return None
    return tuple(parts)


TU_PLATE_REGION = _region_from_env()


def _clamp_to_region(wx, wy, region):
    """`(wx, wy)` pulled inside `region` shrunk by `_REGION_MARGIN_M` on
    every side. `region=None` is a no-op (returns the point unchanged) —
    the common case until a caller actually threads one through. A region
    narrower than `2 * _REGION_MARGIN_M` on an axis collapses that axis to
    its own midpoint rather than producing an inverted (lo > hi) clamp
    range."""
    if not region:
        return wx, wy
    x0, y0, x1, y1 = region
    lo_x, hi_x = min(x0, x1) + _REGION_MARGIN_M, max(x0, x1) - _REGION_MARGIN_M
    lo_y, hi_y = min(y0, y1) + _REGION_MARGIN_M, max(y0, y1) - _REGION_MARGIN_M
    if lo_x > hi_x:
        lo_x = hi_x = (x0 + x1) / 2.0
    if lo_y > hi_y:
        lo_y = hi_y = (y0 + y1) / 2.0
    return min(hi_x, max(lo_x, float(wx))), min(hi_y, max(lo_y, float(wy)))


# ---------------------------------------------------------------------------
# LEVELS — the EF-proxy cut table, `tornado._ladder`'s own jitter idiom
# ---------------------------------------------------------------------------
LEVELS = ("T0", "T1", "T2", "T3", "T4")

# `_plans/urban_tornado_plan.md` §2.6's DESIGN cuts, WIDENED once against the
# three real city manifests: re-binning their intensities against the
# original (0.32/0.52/0.72) cuts showed T3 dominating every table (the
# corridor is ~280 m wide over 30-90 m building footprints, so most of a
# building's own footprint sits well inside the field, not near an edge --
# it is the FIELD's shape doing that, not the cuts being wrong for a single
# building). Widening T1/T2 modestly costs nothing at the top of the ladder
# (level 3's T4 count is unchanged, 54 buildings) and fixes the real defect:
# level 1 (an EF2 track, `peak=0.75`) was reaching T4 by rounding error (6
# buildings there that should never have crossed 0.72 at all, only jitter
# pushed a `peak`-adjacent building over). `_ladder`'s convention is
# "< limit -> this name", so T4 is everything at or above 0.74 and the
# final tuple's limit only needs to be > 1.0.
_URBAN_CUTS = ((0.10, "T0"), (0.36, "T1"), (0.56, "T2"), (0.74, "T3"), (1.01, "T4"))


def level_for_intensity(i, rng, jitter=0.06):
    """The urban tornado level for intensity `i`, jittered per building.

    Same reason `tornado.house_level_for_intensity` jitters: without it every
    level boundary is a contour line parallel to the track, which reads as a
    gradient map rather than as damage (`tornado._ladder`'s own docstring).
    """
    return trn._ladder(_URBAN_CUTS, i, rng, jitter)


# ---------------------------------------------------------------------------
# HEIGHT CLASS — §2.3 / §2.6's four bands, HARD caps on removal
# ---------------------------------------------------------------------------
HEIGHT_CLASSES = ("lowrise", "midrise", "highrise", "tower")

# §2.6's table, verbatim: max removed façade pieces (a FRACTION of the
# building's own piece count — glass is a material change and is never
# capped by this table), max chunk storeys at T4, and whether a urm building
# is allowed the T4 `out_of_plane_top` windward-wall peel at all.
HEIGHT_CAPS = {
    "lowrise":  {"max_removed_frac": 0.35, "max_chunk_storeys": 2, "out_of_plane": True},
    "midrise":  {"max_removed_frac": 0.25, "max_chunk_storeys": 3, "out_of_plane": True},
    "highrise": {"max_removed_frac": 0.15, "max_chunk_storeys": 3, "out_of_plane": False},
    "tower":    {"max_removed_frac": 0.10, "max_chunk_storeys": 2, "out_of_plane": False},
}

_LOWRISE_MAX_M = 18.0
_MIDRISE_MAX_M = 45.0
_HIGHRISE_MAX_M = 100.0


def height_class_for(H, typology=None):
    """"lowrise"/"midrise"/"highrise"/"tower" for a building of height `H`.

    TYPOLOGY FIRST, HEIGHT FALLBACK — per §2.3's own wording. But there is a
    real mismatch worth writing down rather than papering over: the brief
    says to reuse `urban_fire_spread.height_class` / `TYPOLOGY_HEIGHT_CLASS`
    for the typology branch, and that table is a THREE-class system ("low" /
    "mid_high" / "skyscraper") built for a different question (fire spread
    collapse eligibility), not this ladder's FOUR bands. Its own measured
    typology medians straddle this ladder's cuts on both remaining classes —
    "mid_high" covers `midrise` (median 36 m) AND `brick_midrise` (median
    58 m, already past this ladder's 45 m cut) in one bucket, and
    "skyscraper" covers `tower` (median 78 m, BELOW this ladder's 100 m
    tower cut) AND `highrise` (median 165 m) in the other — so there is no
    lossless map from three classes onto four.

    The one case that IS unambiguous is the low end: every typology
    `TYPOLOGY_HEIGHT_CLASS` calls "low" (`rowhouse` 14.5 m, `lowrise`
    13.5 m median) sits well under this ladder's 18 m cut, so a typology
    that resolves to "low" is trusted outright. Everything else falls
    through to the measured `H` — always exact for a sliced building (it is
    the piece grid's own top), never a district median — rather than
    guessing which of two bands a "mid_high"/"skyscraper" typology means.
    This is the safety-first reading: `H` decides caps precisely instead of
    a typology bucket occasionally handing a 90 m building the loose
    `midrise` cap (0.25, out-of-plane allowed) when the height-exact
    `highrise` cap (0.15, no out-of-plane) is the one the record supports.
    """
    if typology:
        try:
            from . import urban_fire_spread as ufs
            cls3 = ufs.height_class(typology=typology)
            if cls3 == ufs.HEIGHT_CLASS_LOW:
                return "lowrise"
        except Exception:
            pass
    h = float(H or 0.0)
    if h < _LOWRISE_MAX_M:
        return "lowrise"
    if h < _MIDRISE_MAX_M:
        return "midrise"
    if h < _HIGHRISE_MAX_M:
        return "highrise"
    return "tower"


# ---------------------------------------------------------------------------
# WIND-SIDE WEIGHTS — §2.5
# ---------------------------------------------------------------------------
_OVER_WEIGHT = (0.70, 1.00)
_CORNER_BONUS = 0.15


def side_weights(info, wind, rng=None):
    """{"S":.., "E":.., "N":.., "W":.., "SW":.., "SE":.., "NW":.., "NE":..}.

    §2.5's formula exactly, off `quake_flow._outward(m, side)` for
    `m = info["masses"]["main"]`. One addition beyond the letter of the
    brief: `wind["over"]`'s branch is specified as "uniform(0.7, 1.0) PER
    SIDE" — that is a random draw, and the brief's own two-argument
    signature (`side_weights(info, wind)`) has nowhere to put an RNG while
    staying reproducible from a seed. `rng` is added here as an OPTIONAL
    third argument so `plan_damage` (which owns the seeded RNG) can pass it
    through and the whole plan stays byte-identical for a byte-identical
    seed; called with just `(info, wind)` — the documented shape — it still
    returns a valid answer, the FIXED midpoint of the (0.7, 1.0) band,
    rather than raising or silently going nondeterministic.
    """
    m = info["masses"]["main"]
    over = bool(wind.get("over", False))
    out = {}
    if over:
        mid = float(sum(_OVER_WEIGHT)) / 2.0
        for sd in qs.SIDES:
            out[sd] = float(rng.uniform(*_OVER_WEIGHT)) if rng is not None else mid
    else:
        brg = math.radians(float(wind.get("bearing_deg", 0.0)))
        dx, dy = math.cos(brg), math.sin(brg)
        for sd in qs.SIDES:
            nx, ny = qf._outward(m, sd)
            dot = nx * dx + ny * dy
            windward = max(0.0, -dot)
            leeward = max(0.0, dot) * 0.35
            side_w = (1.0 - abs(dot)) * 0.55
            out[sd] = float(min(1.0, max(0.0, windward + leeward + side_w)))
    for cn, (sa, sb) in qs._CORNER_SIDES.items():
        out[cn] = float(min(1.0, max(0.0, (out[sa] + out[sb]) / 2.0 + _CORNER_BONUS)))
    return out


def _rank_sides(weights):
    """The four elevations, windward first. A fixed tie-break (side name)
    keeps this deterministic when two sides land on the same weight."""
    return sorted(qs.SIDES, key=lambda s: (-weights.get(s, 0.0), s))


# ---------------------------------------------------------------------------
# THE PLANNER'S CONTEXT
# ---------------------------------------------------------------------------
def _glass_measured(g):
    """Did `tornado_urban_usd.annotate_glazing` stamp this grid's placements
    with `_glass_faces` before `describe`? A GRID-WIDE flag, not per-piece —
    checked ONCE so `t_glass_loss` and the debris ledger agree on which
    path a whole building takes, matching the discovery that motivated
    this: on a real slice the glazing sits in PIER/core pieces (not the
    `wall`/opening role a synthetic fixture puts it on), so a per-piece
    fallback would silently mix the two vocabularies on one building."""
    return any("_glass_faces" in (e.get("p") or {}) for e in g.els)


def _n_glass_candidates(g, measured):
    """How many pieces on the whole building COULD carry a voided pane —
    for `stats["n_glass_candidates"]`, independent of which recipe ran or
    how much of the building it touched."""
    n = 0
    for e in g.els:
        p = e.get("p") or {}
        if p.get("_side") not in qs.SIDES:
            continue
        if measured:
            if p.get("_role") in ("core", "roof"):
                continue
            if float(p.get("_glass_faces") or 0) > 0:
                n += 1
        elif qs.is_opening(p, g.n_sub):
            n += 1
    return n


def _pctx(info, elements, btype, rng, plan, wind, weights, height_class, intensity):
    """`pctx["g"]` is a real `quake_sliced._Grid` and `pctx["plan"]` carries
    the same `removed` / `_removed_set` keys `quake_sliced._apply_region` /
    `_remove` / `_unremove` read and write — so those three functions (the
    toothed-removal and rigid-displacement mechanics this module explicitly
    reuses rather than re-implementing) can be called DIRECTLY on this pctx.
    """
    g = qs._Grid(info, elements)
    measured = _glass_measured(g)
    return {"info": info, "g": g, "btype": btype, "rng": rng, "plan": plan,
            "H": float(info.get("H") or 0.0), "n_sub": g.n_sub,
            "wind": wind, "side_weights": weights,
            "height_class": height_class, "intensity": float(intensity),
            "glass_measured": measured,
            "n_glass_candidates": _n_glass_candidates(g, measured)}


def _note(pctx, text):
    pctx["plan"]["notes"].append(text)


# ---------------------------------------------------------------------------
# RECIPES — each is `t_<name>(pctx, **kw)`, PURE
# ---------------------------------------------------------------------------
def _n_bays_for(nb, frac, span=None):
    """How many of a side's `nb` bays a fraction asks for — ROUND 4 (D2).

    `max(1, int(round(nb * frac)))` was the old expression everywhere, and
    on a COARSE real face it under-delivers by a whole bay. Measured on the
    live container probe (`SM_Building_02`, the bench's A1-A3 cell):
    `g.n_bays["S"] == 3` (the S run is `pier_S_0` / `wall_S_1` /
    `pier_S_2` / ... — six runs, three bays), so a T3 cladding band drawn
    at `bay_frac = (0.40, 0.70)` rounded to **ONE** bay: one third of the
    windward face, three storeys of eleven, `removed_frac` 0.074 against a
    0.25 midrise cap — and the rendered bench cell read undamaged, which is
    exactly the user's "I don't see any building damages".

    The fix is not a bigger fraction (the ladder's fractions are sourced —
    R2 §8.6) but honest rounding against the fraction's own RANGE: round to
    the nearest bay, then clamp into `[ceil(nb*lo), floor(nb*hi)]` so what
    lands is inside the band the recipe asked for rather than one rounding
    step below its floor. On a 3-bay face at 40-70 % that is 2 bays; on a
    1-bay face it is still 1 (nothing to widen into); on a 10-bay face at
    40-70 % it is unchanged from the old expression.

    Same bug CLASS as bug 4 in `build-urban-tornado-scenes`'s catalogue
    ("piece-count caps are meaningless on a coarse grid") — a real slice's
    bay count is small and integer, so every share expressed over it has to
    be rounded deliberately.

    `span` is the `(lo, hi)` the caller drew `frac` from; omitted, the
    clamp degrades to the plain nearest-bay round.
    """
    import math as _math
    nb = int(nb)
    if nb <= 0:
        return 0
    n = int(round(nb * float(frac)))
    if span:
        lo, hi = float(span[0]), float(span[1])
        n_lo = int(_math.ceil(nb * lo - 1e-9))
        n_hi = int(_math.floor(nb * hi + 1e-9))
        if n_hi >= n_lo:
            n = min(max(n, n_lo), n_hi)
    return max(1, min(nb, n))


def _glass_frac_ref(pctx):
    """The MEDIAN `_glass_frac` over every glazed piece on this building —
    the per-asset reference `t_glass_loss` normalises against, computed
    once per plan and cached on the pctx.

    ROUND 4 (D2). `_glass_frac` is `annotate_glazing`'s glazing FACES over
    the piece's TOTAL faces, which is a mesh-COMPLEXITY ratio, not a
    coverage one: a curtain-wall piece whose pane is two triangles inside a
    mullion frame of several hundred scores ~0.02 while the pane covers
    most of the piece's visible area. Round 3 used it as a straight
    probability multiplier (`rng.random() < f * gf`), and on the real
    `SM_Building_24` slice that voided **2 panes out of 99 candidates** at
    T3 where the rc_glass ladder promises 65-90 % of the windward face —
    the A4 bench cell's "glass counts of 1-2 are invisible".

    Normalising by the building's own median makes the TYPICAL glazed piece
    break at the recipe's own fraction and keeps the field's intent (a
    piece with much less glazing than its neighbours breaks proportionally
    less), on any asset, without a per-asset constant.
    """
    got = pctx.get("_glass_frac_ref")
    if got is not None:
        return got
    vals = []
    for e in pctx["g"].els:
        p = e.get("p") or {}
        if p.get("_role") in ("core", "roof"):
            continue
        if float(p.get("_glass_faces") or 0) <= 0:
            continue
        vals.append(max(0.0, min(1.0, float(p.get("_glass_frac") or 0.0))))
    vals.sort()
    ref = vals[len(vals) // 2] if vals else 0.0
    ref = max(1e-3, float(ref))
    pctx["_glass_frac_ref"] = ref
    return ref


def t_glass_loss(pctx, base_frac=(0.05, 0.15), n_sides=1):
    """Glass out of the windward face(s), per-side fraction = `base_frac`
    scaled by that side's wind weight — the twin of `quake_sliced.
    s_glass_loss`, with the wind ranking standing in for `_pick_sides`.

    The band of storeys is CONTIGUOUS, as `s_glass_loss` draws it, but
    biased UPWARD: a tornado's pressure and debris-strike loading both rise
    with height over the lowest ~10 storeys of a building (more open sky,
    more time for wind-borne debris to have accelerated), so the band's
    start is drawn from a skewed distribution that favours the top of the
    building rather than a uniform one.

    TWO CANDIDATE-PICKING PATHS, chosen ONCE per plan (`pctx["glass_
    measured"]`, §the module's `_glass_measured`): a REAL slice's glazing
    does not sit where a synthetic fixture's does. On `SM_Building_02` (the
    lead's live probe) every `wall` piece carries only the blind `WallBack`
    material and the actual glazing is 18 PIER + 9 core pieces — so the
    role-based `is_opening` pick below (the FALLBACK path, and the only
    path a synthetic fixture with no `_glass_faces` annotation ever takes)
    finds zero panes to void on a building shaped like that.

    MEASURED PATH (`tornado_urban_usd.annotate_glazing` has stamped every
    placement with `_glass_faces` / `_glass_frac` before `describe`): the
    candidate pool for a side is every piece there of ANY role except
    `core`/`roof` with `_glass_faces > 0` — piers and corners included, not
    only `wall` — and each piece's own per-draw probability is additionally
    scaled by its `_glass_frac` (a pier that is 30% glazed is voided a
    third as often, at the same side/storey draw, as a fully-glazed piece).
    """
    rng, g = pctx["rng"], pctx["g"]
    sts = sorted(g.storeys)
    if not sts:
        return
    weights = pctx["side_weights"]
    lo, hi = base_frac
    ranked = _rank_sides(weights)[: max(1, int(n_sides))]
    measured = pctx.get("glass_measured", False)
    picked_total, bands = [], []
    for sd in ranked:
        w = weights.get(sd, 0.0)
        if w <= 0.0:
            continue
        nb = g.n_bays.get(sd, 0)
        if nb <= 0:
            continue
        f = rng.uniform(lo, hi) * w
        n_band = max(1, int(round(len(sts) * (0.30 + 0.45 * min(1.0, f / max(hi, 1e-6))))))
        n_band = min(len(sts), n_band)
        max_i0 = max(0, len(sts) - n_band)
        # ROUND 5 (user review 2026-09-02, items 3/4): SPREAD THE GLASS BAND
        # DOWN THE FACE, do not park it at the top. Round 4 drew the band
        # START `max_i0 - round(max_i0 * u ** 1.6)` — median ~0.67 of the
        # range, i.e. the upper storeys — on the theory that a tornado's
        # debris strike rises with height. The review's verdict ("damage is
        # towards the top not low") is that on these mid-rises the WHOLE
        # windward wall's glazing goes, so the visible band must start
        # low/middle and run up. Same mild low bias as `t_cladding_band`
        # (`u ** 1.3`, median ~0.4 of the range).
        if max_i0 > 0:
            i0 = int(round(max_i0 * (rng.random() ** 1.3)))
        else:
            i0 = 0
        band_st = sts[i0:i0 + n_band]
        if measured:
            cand = [e for st in band_st for b in sorted(g.sides.get(sd, ()))
                    for e in g.at((sd, st, b))
                    if (e.get("p") or {}).get("_role") not in ("core", "roof")
                    and float((e.get("p") or {}).get("_glass_faces") or 0) > 0]
            hit = []
            gf_ref = _glass_frac_ref(pctx)
            for e in cand:
                p = qs._path(e)
                gf = max(0.0, min(1.0, float((e.get("p") or {}).get("_glass_frac") or 0.0)))
                # ROUND 4 (D2): NORMALISED, not raw. `_glass_frac` is a
                # mesh-complexity ratio (glazing faces / all faces), so
                # `f * gf` voided 2 of 99 candidates on the real
                # SM_Building_24 slice at a recipe fraction of 0.65-0.90.
                # Relative to the building's own median glazed piece, the
                # typical pane breaks at `f` and a barely-glazed piece
                # still breaks proportionally less. See `_glass_frac_ref`.
                if p and rng.random() < f * min(1.0, gf / gf_ref):
                    hit.append(p)
            if cand and not hit and f > 0.0:
                wts = [max(1e-6, float((e.get("p") or {}).get("_glass_frac") or 0.0))
                      for e in cand]
                pick = rng.choices(cand, weights=wts, k=1)[0]
                if qs._path(pick):
                    hit = [qs._path(pick)]
        else:
            cand = [e for st in band_st for b in sorted(g.sides.get(sd, ()))
                    for e in g.at((sd, st, b))
                    if qs.is_opening(e.get("p") or {}, g.n_sub)]
            hit = [qs._path(e) for e in cand if qs._path(e) and rng.random() < f]
            if cand and not hit and f > 0.0:
                hit = [qs._path(cand[rng.randrange(len(cand))])]
        picked_total += [q for q in hit if q]
        bands.append({"side": sd, "weight": float(w), "frac": float(f),
                      "storeys": [int(s) for s in band_st], "n": len(hit)})
    plan = pctx["plan"]
    known = set(plan["glass"])
    plan["glass"] += [q for q in picked_total if q not in known]
    plan["glass_bands"].append({"recipe": "glass_loss", "bands": bands,
                                "measured": bool(measured)})
    _note(pctx, "glass_loss: {0} pane(s) voided over {1} side(s), wind-weighted "
                "({2}){3}".format(
                    len(picked_total), len(bands),
                    "/".join(b["side"] for b in bands) or "-",
                    " [measured _glass_faces/_glass_frac path, {0} glass "
                    "candidate(s) on the building]".format(
                        pctx.get("n_glass_candidates", 0)) if measured else ""))


def t_roof_props_sweep(pctx):
    """Rooftop plant (and, by the same flag, light fixtures/awnings at the
    lighter levels) marked for removal at apply time — `plan["roof_props"]`,
    consumed by `tornado_urban_usd.apply_plan` the way `quake_sliced.
    _sweep_roof_props_sliced` consumes `plan["ground"]`-adjacent flags."""
    pctx["plan"]["roof_props"] = "sweep"
    _note(pctx, "roof_props_sweep: rooftop plant/fixtures marked for removal")


def t_parapet_fall(pctx, n_sides=1, frac=(0.30, 0.60)):
    """Parapet / coping off the windward side and its weighted flank(s).

    Structurally `quake_sliced.s_parapet_fall`'s body (including its
    fallback for a slice with no distinct parapet role at all — the top
    band is used instead, `roles=None`), with the side list driven by wind
    rank instead of `_pick_sides`, and the flank sides' fraction additionally
    scaled by their own (lower) weight.
    """
    rng, g = pctx["rng"], pctx["g"]
    lo, hi = frac
    weights = pctx["side_weights"]
    par = g.role_pieces(("parapet", "parapet_corner"))
    st_par = g.top
    if par:
        st_par = max(int((e.get("p") or {}).get("_storey", g.top)) for e in par)
    ranked = _rank_sides(weights)[: max(1, int(n_sides))]
    total_lost, sides_done = 0, []
    for rank, sd in enumerate(ranked):
        w = weights.get(sd, 0.0)
        if rank > 0 and w <= 0.0:
            continue
        nb = g.n_bays.get(sd, 0)
        if nb <= 0:
            continue
        f = rng.uniform(lo, hi) * (1.0 if rank == 0 else max(0.15, w))
        # ROUND 4 (D2): `_n_bays_for`, not `int(round(...))` — see that
        # helper for the measurement. The flank sides' `f` is already
        # weight-scaled below the recipe's own range, so only the windward
        # side (rank 0) gets the range clamp.
        run = qs._bay_run(rng, nb, _n_bays_for(
            nb, f, (lo, hi) if rank == 0 else None))
        region = {(sd, st_par, b) for b in run
                  if any((e.get("p") or {}).get("_role") == "parapet" or not par
                         for e in g.at((sd, st_par, b)))}
        if not region:
            continue
        res = qs._apply_region(pctx, region,
                               corners=[(c, st_par) for c in qs._CORNER_SIDES],
                               tag="t_parapet_fall",
                               roles=("parapet", "parapet_corner") if par else None)
        total_lost += len(res["lost"]) + len(res["corners"])
        sides_done.append(sd)
    _note(pctx, "parapet_fall: {0} piece(s) off {1}".format(
        total_lost, "/".join(sides_done) or "-"))


def t_panel_loss(pctx, frac=(0.05, 0.10), top_frac=(1.0 / 3.0)):
    """A scatter of windward OPENING (glazed) panels out — the T2 signature
    for both a curtain wall's spandrels and a frame's infill (the same
    piece-grid vocabulary as `quake_sliced.s_infill_fail`, restricted here
    to the wind-picked windward face).

    ROUND 5 (user review 2026-09-02, items 3/4): the scatter runs the FULL
    HEIGHT of the windward face, not just its top third. The review's A1
    verdict was "damage at the top not the bottom"; a tornado's windward
    glazing goes low and middle at least as much as high, so the candidate
    pool is now every windward glazed panel (the `_glass_frac`-driven ledger
    and the height-class cap still bound how much actually breaks). `top_frac`
    is kept in the signature for the ladder tables that pass it, but no longer
    clips the pool to the top of the building.
    """
    rng, g = pctx["rng"], pctx["g"]
    sts = sorted(g.storeys)
    if not sts:
        return
    weights = pctx["side_weights"]
    sd = _rank_sides(weights)[0]
    w = weights.get(sd, 0.0)
    lo, hi = frac
    f = rng.uniform(lo, hi) * max(0.3, w)
    cand = [e for b in sorted(g.sides.get(sd, ())) for st in sts
            for e in g.at((sd, st, b)) if qs.is_opening(e.get("p") or {}, g.n_sub)]
    hit = [e for e in cand if rng.random() < f]
    if cand and not hit and f > 0.0:
        hit = [cand[rng.randrange(len(cand))]]
    qs._remove(pctx, hit, why="t_panel_loss")
    _note(pctx, "panel_loss: {0} opening panel(s) out across the full height "
                "of {1}".format(len(hit), sd))


def t_cladding_band(pctx, storeys=(2, 4), bay_frac=(0.40, 0.70), keep_pier=None):
    """ONE contiguous TOOTHED region on the windward face — 2-4 storeys x
    40-70 % of that side's own bays, upper-half biased — via `quake_sliced.
    _apply_region` directly (the exact toothing draw the earthquake ladder
    uses, unmodified).

    A ONE-BAY windward side on a tower/highrise (a coarse real slice's
    regular-grid fallback, `SM_Building_13`'s own shape in the container
    probe) is capped to 2 storeys: a full-width 3-4 storey band on a side
    with nothing to tooth INTO is a horizontal SLICE of the building, not a
    chunk of façade.
    """
    rng, g = pctx["rng"], pctx["g"]
    sts = sorted(g.storeys)
    if not sts:
        return
    weights = pctx["side_weights"]
    sd = _rank_sides(weights)[0]
    nb = g.n_bays.get(sd, 0)
    if nb <= 0:
        return
    hc = pctx.get("height_class")
    st_hi = int(storeys[1])
    st_lo = int(storeys[0])
    if nb == 1 and hc in ("tower", "highrise") and st_hi > 2:
        st_hi = 2
        _note(pctx, "cladding_band: 1-bay side ({0}) on a {1} building -- "
                    "storeys capped to 2 (a full-width band would be a "
                    "slice, not a chunk)".format(sd, hc))
    elif nb <= 2 and hc in ("lowrise", "midrise") and st_hi > st_lo:
        # ROUND 4 (D2): A NARROW FACE EXPRESSES THE BAND VERTICALLY.
        # `g.n_bays["S"]` is **2** on the real SM_Building_02 slice (six
        # runs, `n_sub` 3), so the horizontal share is quantised to 50 % or
        # 100 % of the face and the recipe can only reach for its own size
        # in storeys. A uniform 2-4 draw then landed 3 storeys of eleven —
        # 13 % of the windward face, `removed_frac` 0.074 against the 0.25
        # midrise cap — and the A2/A3 bench cells read undamaged. Drawing
        # from the TOP of the recipe's own storey range on a <= 2-bay face
        # is the mirror of the 1-bay tower cap above (same argument, other
        # direction) and stays inside both the recipe's range and the
        # height-class area cap `_cap_removed_frac` enforces afterwards.
        st_lo = max(st_lo, st_hi - 1)
        _note(pctx, "cladding_band: {0}-bay side ({1}) on a {2} building -- "
                    "storeys drawn from {3}-{4} (a narrow face can only "
                    "reach for the band's size vertically)".format(
                        nb, sd, hc, st_lo, st_hi))
    n_st = rng.randint(st_lo, max(st_lo, st_hi))
    n_st = max(1, min(len(sts), n_st))
    max_i0 = max(0, len(sts) - n_st)
    # ROUND 5 (user review 2026-09-02, items 3/4): BIAS THE BAND LOW, NOT
    # HIGH. "A tornado loads the whole windward face" — the review saw the
    # A1/A2 removal concentrated at the parapet band ("damage is towards the
    # top not low"). The round-4 draw, `randint(max_i0 // 2, max_i0)`,
    # ANCHORED the band in the UPPER HALF; a tornado's wind pressure and
    # debris strike load the lower and middle storeys of a windward wall at
    # least as hard (the ground-storey structure itself is never removed —
    # that guard is elsewhere — but the storeys just above it are exactly
    # where a cladding band should sit). Draw the start with a mild LOW bias
    # (`u ** 1.3`, median ~0.4 of the range) so the band lands lower/middle
    # and spreads up, never parked at the coping.
    i0 = int(round(max_i0 * (rng.random() ** 1.3))) if max_i0 > 0 else 0
    band_st = sts[i0:i0 + n_st]
    frac = rng.uniform(*bay_frac)
    # ROUND 4 (D2): `_n_bays_for` — `int(round(nb * frac))` turned a
    # 3-bay windward face at 40-70 % into ONE bay on the real
    # SM_Building_02 slice, which is why the A2/A3 bench cells read
    # undamaged. See that helper for the numbers.
    want = _n_bays_for(nb, frac, bay_frac)
    run = qs._bay_run(rng, nb, want)
    region = {(sd, st, b) for st in band_st for b in run if (sd, st, b) in g.runs}
    if not region:
        return
    res = qs._apply_region(pctx, region, corners=[], tag="t_cladding_band",
                           keep_pier=keep_pier)
    pctx["plan"]["regions"].append({
        "recipe": "cladding_band", "side": sd,
        "storeys": [int(s) for s in band_st], "bays": [int(b) for b in run],
        "cells": sorted(list(c) for c in region)})
    _note(pctx, "cladding_band: {0} storey(s) x {1} bay(s) off {2}, {3} "
                "piece(s) down, {4} boundary pier(s) left standing".format(
                    len(band_st), len(run), sd, len(res["lost"]),
                    len(res["kept_piers"])))


def t_hanging_panels(pctx, n=(1, 3), deg=(25.0, 70.0)):
    """1-3 panels along the boundary of what has already been lost on the
    windward face, pitched outward 25-70 deg about their own bottom OUTER
    edge, STILL ATTACHED (`displaced`, never `removed`).

    The pivot/axis/sign are `quake_sliced.s_out_of_plane`'s macroblock
    geometry exactly — pivot at the piece's own bottom outer edge (local
    centre pushed out by half its own depth, at its own base z), rotation
    axis along the wall, positive `deg` swinging the TOP of the piece
    further from the mass centre (`quake_sliced._out_dist` growing) and
    down — the sign `test_every_macroblock_lands_in_the_street` exists to
    catch. That is what keeps a hanging panel swinging INTO THE STREET
    rather than folding back over the still-standing wall behind it.
    """
    rng, g = pctx["rng"], pctx["g"]
    weights = pctx["side_weights"]
    sd = _rank_sides(weights)[0]
    present = set(g.runs)
    removed_set = pctx["plan"]["_removed_set"]
    lost_here = set()
    for key in g.cells(side=sd):
        for e in g.at(key):
            if qs._path(e) in removed_set:
                lost_here.add(key)
                break
    boundary_intact = qs._adjacent_bays(lost_here, present) if lost_here else []
    cand = [e for key in boundary_intact for e in g.at(key)
            if qs.is_opening(e.get("p") or {}, g.n_sub) and qs._path(e)
            and qs._path(e) not in removed_set]
    if not cand:
        sts = sorted(g.storeys)
        upper = sts[len(sts) // 2:] if sts else []
        cand = [e for st in upper for b in sorted(g.sides.get(sd, ()))
                for e in g.at((sd, st, b))
                if qs.is_opening(e.get("p") or {}, g.n_sub) and qs._path(e)
                and qs._path(e) not in removed_set]
    if not cand:
        return
    rng.shuffle(cand)
    n_take = rng.randint(int(n[0]), min(int(n[1]), len(cand)))
    chosen = cand[:max(0, n_take)]
    if not chosen:
        return
    m = g.mass_of(chosen[0])
    ox, oy = qf._outward(m, sd)
    ax, ay = -oy, ox
    lnx, lny = qs._SIDE_NORMAL[sd]
    for e in chosen:
        sx, sy, sz = qs._size(e)
        depth = sy if sd in ("S", "N") else sx
        plx = e["lx"] + lnx * depth / 2.0
        ply = e["ly"] + lny * depth / 2.0
        px, py = qf._to_world(m, plx, ply)
        d = rng.uniform(*deg)
        pctx["plan"]["displaced"][qs._path(e)] = qs._disp(
            pivot=(px, py, e["z"]), axis=(ax, ay, 0.0), deg=d,
            why="hanging panel off {0}, still attached, pitched {1:.0f} deg "
                "into the street".format(sd, d))
    _note(pctx, "hanging_panels: {0} panel(s) pitched outward on {1}".format(
        len(chosen), sd))


def t_chunk(pctx, keep_pier=None, corner_top_storeys=None,
            max_storeys=None, max_bays_per_side=None):
    """ONE structural chunk on the windward corner — 1-2 bays each side of
    the corner (2-4 total), `max_chunk_storeys` (§2.6, by height class) tall.

    "Region top >= 0.6 H" is §2.6's CONSTRAINT, not an instruction to always
    anchor at the roof: a chunk fixed to the top storey every time is a
    repetitive notch across a corridor, and the record has mid-height loss
    too (Fort Worth 2000's Bank One Tower: worst envelope damage on the SW
    face and W chamfer, not the parapet). So the region's own TOP storey is
    drawn anywhere in the upper 40 % of the building (`st_hi`, roof
    included) and the chunk hangs `k` storeys below that — never the ground
    storey (`st_lo = max(1, ...)`, `corner_fail`'s own rule).

    `corner_top_storeys` restricts which of the lost storeys carry a corner
    piece — the rc_glass skin-only chunk (§2.6: "corner pieces only in the
    top two storeys") — counted from the REGION's own top (`st_hi`), not
    from the building's.
    """
    rng, g = pctx["rng"], pctx["g"]
    if not g.storeys:
        return
    height_class = pctx["height_class"]
    max_st = max(1, int(HEIGHT_CAPS.get(height_class, HEIGHT_CAPS["midrise"])
                        ["max_chunk_storeys"]))
    if max_storeys is not None:
        max_st = min(max_st, max(1, int(max_storeys)))
    # Corner selection reads only `side_weights` (no rng), so resolving it
    # before drawing `k`/`st_hi` changes nothing about the RNG draw
    # sequence -- it only lets the one-bay-side check below adjust `max_st`
    # before `k` is drawn from it.
    weights = pctx["side_weights"]
    corners_ranked = sorted(qs._CORNER_SIDES, key=lambda c: (-weights.get(c, 0.0), c))
    cn = corners_ranked[0]
    sa, sb = qs._CORNER_SIDES[cn]
    one_bay = g.n_bays.get(sa, 0) == 1 or g.n_bays.get(sb, 0) == 1
    if one_bay and height_class in ("tower", "highrise") and max_st > 2:
        max_st = 2
        _note(pctx, "chunk: 1-bay side at the {0} corner on a {1} building "
                    "-- storeys capped to 2 (a full-width band would be a "
                    "slice, not a chunk)".format(cn, height_class))
    k = rng.randint(1, max_st)
    # ROUND 2 (stream K's dw_terrace measurement): "region top >= 0.6 H" is
    # a HEIGHT constraint, and drawing the anchor by STOREY INDEX assumed
    # bands are uniform — true for GAC/dtc slices and most kit styles,
    # FALSE for `dw_terrace`, whose 1.2 m trim band owns a full storey
    # index between two 6 m window bands: index 2 of 3 satisfied the old
    # `randint(ceil(0.6 * g.top), g.top)` while sitting at 31 % of H
    # (measured: 13 of 19 chunk draws violated the constraint over 40
    # seeds). The eligible set is now built from each storey's own BASE
    # ELEVATION against 0.6 * H — `m["levels"][st]` where the storey has a
    # level, the mass top for a band above the last one (a parapet) — and
    # the anchor drawn uniformly from it. On uniform bands this reproduces
    # the old index rule's range exactly.
    m_main = (pctx["info"].get("masses") or {}).get("main") or {}
    m_levels = list(m_main.get("levels") or [])
    m_top = float(m_main.get("top") or pctx["H"])

    def _st_base_z(st):
        return float(m_levels[st]) if 0 <= st < len(m_levels) else m_top

    eligible = [st for st in sorted(g.storeys)
                if st >= 1 and _st_base_z(st) >= 0.6 * pctx["H"] - 1e-6]
    if not eligible:
        eligible = [g.top]
    st_hi = eligible[rng.randrange(len(eligible))]
    st_lo = max(1, st_hi - k + 1)
    lost_st = [st for st in range(st_lo, st_hi + 1) if st in g.storeys]
    if not lost_st:
        return
    region = set()
    for sd in (sa, sb):
        nb = g.n_bays.get(sd, 0)
        if nb <= 0:
            continue
        end = qs._CORNER_END[(sd, cn)]
        bay_cap = 2 if max_bays_per_side is None else max(1, int(max_bays_per_side))
        want = min(nb, rng.randint(1, bay_cap))
        run = qs._bay_run(rng, nb, want, end=end)
        for st in lost_st:
            for b in run:
                if (sd, st, b) in g.runs:
                    region.add((sd, st, b))
    if not region:
        return
    corner_st = lost_st if corner_top_storeys is None else lost_st[-int(corner_top_storeys):]
    res = qs._apply_region(pctx, region, corners=[(cn, st) for st in corner_st],
                           tag="t_chunk", keep_pier=keep_pier)
    pctx["plan"]["regions"].append({
        "recipe": "chunk", "corner": cn, "sides": [sa, sb],
        "storeys": [int(s) for s in lost_st],
        "cells": sorted(list(c) for c in region)})
    _note(pctx, "chunk: {0} corner, storeys {1}-{2} ({3} of {4}), {5} "
                "piece(s) down (incl. {6} corner piece(s)), {7} boundary "
                "pier(s) left standing".format(
                    cn, lost_st[0], lost_st[-1], len(lost_st), max_st,
                    len(res["lost"]) + len(res["corners"]), len(res["corners"]),
                    len(res["kept_piers"])))


def t_out_of_plane_top(pctx):
    """URM ONLY, lowrise/midrise ONLY (`HEIGHT_CAPS[...]["out_of_plane"]`):
    the windward wall of the top 1-2 storeys peels off as macroblocks, the
    single directional failure this ladder allows (`quake_sliced.
    s_out_of_plane`'s DG4 mechanism, `from_storey = top - (n_st - 1)`).

    Never storey 0 (`from_storey = max(1, ...)`), and — unlike the
    earthquake recipe, which stops a bay short of the full elevation so the
    toothing has an intact bay to wander into — this one CAN take the whole
    windward width at the top, because a tornado's own record shows the top
    of a low building peeling off across its full face (Joplin, Moore); the
    boundary below the peeled band is still toothed by `_apply_region`.
    """
    if pctx["btype"] != "urm":
        return
    height_class = pctx["height_class"]
    if not HEIGHT_CAPS.get(height_class, {}).get("out_of_plane", False):
        _note(pctx, "out_of_plane_top: refused on a {0} building -- no "
                    "windward top-wall peel above midrise".format(height_class))
        return
    rng, g = pctx["rng"], pctx["g"]
    if not g.storeys:
        return
    n_st = rng.randint(1, 2)
    from_storey = max(1, g.top - n_st + 1)
    sts = sorted(s for s in g.storeys if s >= from_storey)
    if not sts:
        return
    weights = pctx["side_weights"]
    sd = _rank_sides(weights)[0]
    nb = g.n_bays.get(sd, 0)
    if nb <= 0:
        return
    run = sorted(g.sides.get(sd, ()))
    region = {(sd, st, b) for st in sts for b in run if (sd, st, b) in g.runs}
    if not region:
        return
    cand = [e for st in sts for b in run for e in g.at((sd, st, b))
            if qs.is_opening(e.get("p") or {}, g.n_sub) and qs._path(e)]
    rng.shuffle(cand)
    n_mb = rng.randint(1, max(1, min(3, len(cand)))) if cand else 0
    macros = cand[:n_mb]
    res = qs._apply_region(pctx, region, corners=[], tag="t_out_of_plane_top")
    m = g.mass_of(macros[0]) if macros else pctx["info"]["masses"]["main"]
    ox, oy = qf._outward(m, sd)
    ax, ay = -oy, ox
    lnx, lny = qs._SIDE_NORMAL[sd]
    plan = pctx["plan"]
    for e in macros:
        qs._unremove(pctx, qs._path(e))
        sx, sy, sz = qs._size(e)
        depth = sy if sd in ("S", "N") else sx
        plx = e["lx"] + lnx * depth / 2.0
        ply = e["ly"] + lny * depth / 2.0
        px, py = qf._to_world(m, plx, ply)
        deg = rng.uniform(*qs.MACRO_DEG)
        plan["displaced"][qs._path(e)] = qs._disp(
            pivot=(px, py, e["z"]), axis=(ax, ay, 0.0), deg=deg,
            why="tornado top-wall macroblock off {0} at storey {1}".format(
                sd, e["storey"]))
        plan["macroblocks"] = plan.get("macroblocks") or []
        plan["macroblocks"].append({"path": qs._path(e), "side": sd,
                                    "storey": int(e["storey"]), "deg": float(deg),
                                    "pivot": [float(px), float(py), float(e["z"])],
                                    "size": [sx, sy, sz]})
    plan["regions"].append({"recipe": "out_of_plane_top", "side": sd,
                            "storeys": [int(s) for s in sts],
                            "bays": [int(b) for b in run],
                            "cells": sorted(list(c) for c in region)})
    _note(pctx, "out_of_plane_top: {0} storey(s) of {1}'s windward wall off, "
                "{2} macroblock(s), {3} boundary pier(s) left standing".format(
                    len(sts), sd, len(macros), len(res["kept_piers"])))


def t_top_storey_loss(pctx):
    """LOWRISE URM ONLY, `intensity >= 0.85` ONLY: the top storey's exterior
    wall comes off 2-3 sides and the roof piece is ledgered as debris — the
    ONE named exception to "never remove `role in (core, roof)`" (§2.6). The
    building is STILL STANDING: every floor below the top, and the top
    storey's own core/interior, are untouched.
    """
    if pctx["btype"] != "urm" or pctx["height_class"] != "lowrise":
        return
    i = float(pctx.get("intensity") or 0.0)
    if i < 0.85:
        return
    rng, g = pctx["rng"], pctx["g"]
    if not g.storeys:
        return
    st_top = g.top
    weights = pctx["side_weights"]
    ranked = _rank_sides(weights)
    n_sides = rng.randint(2, min(3, len(ranked)))
    sides_on = ranked[:n_sides]
    region = set()
    for sd in sides_on:
        for b in sorted(g.sides.get(sd, ())):
            if (sd, st_top, b) in g.runs:
                region.add((sd, st_top, b))
    if not region:
        return
    corners = [(c, st_top) for c in qs._CORNER_SIDES
              if set(qs._CORNER_SIDES[c]) <= set(sides_on)]
    res = qs._apply_region(pctx, region, corners=corners, tag="t_top_storey_loss")
    plan = pctx["plan"]
    n_roof = 0
    for e in g.role_pieces(("roof",)):
        p = qs._path(e)
        if p and p not in plan["_removed_set"]:
            plan["_removed_set"].add(p)
            plan["removed"].append(p)
            n_roof += 1
    plan["regions"].append({"recipe": "top_storey_loss", "sides": list(sides_on),
                            "storeys": [int(st_top)],
                            "cells": sorted(list(c) for c in region)})
    _note(pctx, "top_storey_loss: top storey exterior wall off {0} ({1} "
                "piece(s)), {2} roof piece(s) down as debris, building still "
                "standing".format("/".join(sides_on),
                                  len(res["lost"]) + len(res["corners"]), n_roof))


# ---------------------------------------------------------------------------
# T4 EXTREME — the REAL urban collapse classes (`_plans/urban_tornado_plan.md`
# §8c, R11, user: "we have industrial buildings and brownstones. Those could
# collapse right ... So show those"). `t_facade_collapse` below is the
# brownstone/URM-lowrise half; the industrial tilt-up/light-roof half lives
# in the SELF-CONTAINED `disaster/tornado_collapse.py` (no piece grid at
# all — a shed's own footprint, not a slice), gated in `disaster/
# tornado_city.py`'s `damageable()`.
# ---------------------------------------------------------------------------
_FACADE_COLLAPSE_MIN_I = 0.82
# ROUND 3b (lead): 4 -> 5. The class exemplar ITSELF was five storeys —
# Waco 1953's R.T. Dennis Building ("originally 3 storeys, expanded to 5",
# _plans/urban_tornado_research.md §0.3) — and the kit `brownstone_row`
# style measures 5 real storeys and was refusing the one recipe built to
# show its class (stream C3's own honest probe finding).
_FACADE_COLLAPSE_MAX_STOREYS = 5
_FACADE_COLLAPSE_LEAN_DEG = (55.0, 80.0)
_FACADE_COLLAPSE_N_LEAN = (1, 2)
# THE DEBRIS PATH THIS RECIPE ACTUALLY BUILDS AGAINST — probed live, not
# assumed: stream DB's berm mechanism (this module's own "DEBRIS" section
# below, `_ledger_removed`/`_deposit_berm`/`_BERM_SHARE`/`_BERM_OUT_M`/
# `_BERM_H_PER_PIECE_M`/`_BERM_H_MAX_M` — out of THIS recipe's editable
# region) landed WHILE this recipe was being written (a mid-write `git
# diff` on this file showed it — see the shared-file discipline note in
# this stream's own `_plans/urban_tornado_C3_notes.md`). It needs NO hook
# from this recipe at all: it keys purely on a removed piece's own `_side`
# (`_FACADE_SIDES` — every main side plus every corner) and the PLAN's
# `level` (`_BERM_SHARE["T4"] = 0.75` — the highest share of any level), so
# every piece `t_facade_collapse` removes already gets the 75% berm split
# automatically, with the height profile (`min(_BERM_H_MAX_M, 0.15 *
# removed_by_side[sd])`) driven by HOW MANY pieces came off that one side —
# a full-height, every-storey facade loss removes far more pieces on `sd`
# than any other T4 recipe manages alone, so it is the one recipe in this
# ladder that reliably DRIVES `berm_h` to its 1.2 m ceiling, i.e. the
# "DEEP stacked berm" §8c asks for falls out of the shared mechanism's own
# per-side piece count, not out of anything special this recipe does. The
# one number §8c names that the landed mechanism does NOT match exactly is
# the reach: `_BERM_OUT_M` is `(0.3, 4.0)` m, not this recipe's own "0-6 m"
# — a DB knob this recipe has no business widening (out of the editable
# region), noted here rather than silently claimed as met. The roof piece
# this recipe also ledgers stays OUT of the berm split regardless (a
# `role == "roof"` piece carries no `_side`, so `_FACADE_SIDES` never
# matches it — `_ledger_removed`'s own comment: a shed roof sheet is the
# LOFTED class, never a wall-base heap).


def t_facade_collapse(pctx, n_lean=_FACADE_COLLAPSE_N_LEAN,
                      lean_deg=_FACADE_COLLAPSE_LEAN_DEG):
    """URM LOWRISE ONLY, <= 5 storeys, `intensity >= 0.82`: the ONLY urban
    collapse this ladder allows on a sliced building — the Waco Dennis
    Building / Nashville brick-lowrise state (`_plans/urban_tornado_
    research.md` §0.3, §6, §8.2). The street/windward facade's pieces come
    down across EVERY storey INCLUDING GROUND — the one recipe in this
    module allowed to touch a ground-storey run at all (every other T4
    recipe's own storey draw is biased away from it — `t_chunk`'s
    `eligible` set explicitly excludes storey 0, `t_cladding_band`'s band
    is upper-half biased) — while the building keeps standing on its other
    three elevations, its floors and its core.

    ELIGIBILITY (checked here AND, for `btype`/`height_class` only, a
    second time in `_guard` — the same defence-in-depth `t_out_of_plane_
    top`/`t_top_storey_loss` already practise, since `_guard` has neither
    the element table nor the intensity to check the storey-count/
    intensity gates itself): `btype == "urm"`, `height_class ==
    "lowrise"`, `<= 5` storeys, `i >= 0.82`. Refuses (no-ops, notes why)
    otherwise — the 0.82 floor is Waco's own lesson (§0.3: true masonry
    bearing-wall construction needed NEAR-F5 conditions to fully collapse,
    a steel-framed neighbour on the SAME block essentially untouched; the
    record's OTHER low-rise total-collapse population — tilt-up/CMU-light-
    roof construction — fails far earlier and far more often by a
    connection defect, not a wall-material failure, but that is a
    DIFFERENT construction class this recipe does not model: it is
    `tornado_collapse.py`'s `industrial` class instead, §8.2).

    THE STREET-OR-WINDWARD FACADE — ONE DEVIATION FROM THE LETTER OF §8c:
    the brief says "pick the higher side_weight of the two candidates"
    (street-facing vs. windward). This pure per-building planner
    (`plan_damage`'s own signature: `info`, `elements`, `level`, `btype`,
    `rng`, `wind` — no layout/neighbour context ever reaches it) has no way
    to know which elevation faces the STREET; that is a property of the
    block layout (`urban_fire_spread.street_side_score` needs every OTHER
    building on the block, which never reaches this module). The only
    candidate this function CAN resolve is the windward face
    (`_rank_sides(weights)[0]`, the same rank-0 pick `t_cladding_band`/
    `t_chunk`/`t_out_of_plane_top` already use) — and since "pick the
    higher weight of the two candidates" is, by definition, always the
    top-ranked side anyway, collapsing "street-or-windward" onto
    "windward" loses nothing a real two-candidate comparison would have
    picked differently; it only loses the (structurally unavailable) case
    where the street side is NOT the windward side. Documented here rather
    than silently approximated.

    ALWAYS EXACTLY ONE ELEVATION — trivially "never > 2 of 4" (§8c's own
    guard table): no corners are added to the region (`corners=[]`), so the
    two elevations neighbouring the failed one, and the leeward one, are
    all untouched by this recipe — 3 of 4 elevations survive it intact,
    well inside the ">= 2 untouched" the building-must-stand invariant
    requires.

    RECLAIMS THE SIDE FROM AN EARLIER T4 RECIPE. `t_hanging_panels` and
    `t_out_of_plane_top` both pick their own side the exact same way
    (`_rank_sides(weights)[0]`) and, since `facade_collapse` is
    deliberately LAST in `LADDER_T["urm"]["T4"]` (so it has final say), may
    already have DISPLACED — not removed — 1-3 pieces there by the time
    this recipe runs. A piece cannot be simultaneously `removed`
    (deactivated) and `displaced` (kept, rigidly transformed) without
    `tornado_urban_usd.apply_plan` doing something undefined with it, so
    any `plan["displaced"]` entry belonging to the chosen side is dropped
    BEFORE this recipe's own `_apply_region` runs — those pieces fall back
    to "untouched" and are cleanly swept into the full-side removal below
    (or, for the 1-2 pieces this recipe itself keeps standing, re-displaced
    under its own 55-80 deg lean spec). A different side's displacements
    (a `chunk`/`hanging_panels` draw that happened to land elsewhere) are
    never touched.

    THE CAP EXEMPTION (§8c, named here and enforced in `_cap_removed_frac`):
    the lowrise `max_removed_frac` cap (0.35, `HEIGHT_CAPS["lowrise"]`)
    exists to stop an ACCIDENTAL over-gutted plan — several ordinary T3/T4
    recipes stacking past a sane fraction by chance — not to forbid the one
    state this whole recipe exists to show: a real, single-elevation total
    facade loss is comfortably >= 0.35 of a building's own facade area BY
    CONSTRUCTION (one full elevation of four, floor to roof, is already
    ~0.25 on a square building before T4's other recipes add anything on
    top of it). `_cap_removed_frac` carries a NAMED carve-out for exactly
    this recipe's own `plan["regions"]` entries (`recipe ==
    "facade_collapse"`, never trimmed back down by that pass) and notes the
    carve-out in `plan["notes"]` whenever it actually mattered (the plan
    would otherwise have landed over the class cap).
    """
    if pctx["btype"] != "urm" or pctx["height_class"] != "lowrise":
        _note(pctx, "facade_collapse: refused -- lowrise urm only, this "
                    "building is {0} {1}".format(pctx["height_class"], pctx["btype"]))
        return
    g = pctx["g"]
    if not g.storeys:
        return
    n_storeys = len(g.storeys)
    if n_storeys > _FACADE_COLLAPSE_MAX_STOREYS:
        _note(pctx, "facade_collapse: refused -- {0} storeys > the {1}-storey "
                    "lowrise cap (Sec8c)".format(n_storeys,
                                                 _FACADE_COLLAPSE_MAX_STOREYS))
        return
    i = float(pctx.get("intensity") or 0.0)
    if i < _FACADE_COLLAPSE_MIN_I:
        _note(pctx, "facade_collapse: refused -- intensity {0:.2f} < {1:.2f} "
                    "(Waco/Nashville needed near-F5 conditions for a true "
                    "URM bearing-wall total collapse, R2 Sec0.3/Sec6)".format(
                        i, _FACADE_COLLAPSE_MIN_I))
        return

    rng = pctx["rng"]
    weights = pctx["side_weights"]
    sd = _rank_sides(weights)[0]
    nb = g.n_bays.get(sd, 0)
    if nb <= 0:
        _note(pctx, "facade_collapse: refused -- no bays on the windward "
                    "side {0}".format(sd))
        return

    # Reclaim `sd` from an earlier T4 recipe's own displacement (see
    # docstring) -- never touches a DIFFERENT side's entries.
    plan = pctx["plan"]
    idx_all = {qs._path(e): e for e in g.els}
    reclaimed = 0
    for p in list(plan["displaced"]):
        e2 = idx_all.get(p)
        if e2 is not None and (e2.get("p") or {}).get("_side") == sd:
            del plan["displaced"][p]
            reclaimed += 1

    sts = sorted(g.storeys)
    run = sorted(g.sides.get(sd, ()))
    region = {(sd, st, b) for st in sts for b in run if (sd, st, b) in g.runs}
    if not region:
        return
    has_ground = 0 in sts
    res = qs._apply_region(pctx, region, corners=[], tag="t_facade_collapse")
    lost_paths = sorted(set(res["lost"]) | set(res["corners"]))

    # 1-2 surviving pieces stay standing, pitched outward as leaning
    # macroblocks -- the SAME `_disp` mechanics `t_hanging_panels`/
    # `t_out_of_plane_top` use (pivot at the piece's own bottom outer edge,
    # axis along the wall, positive `deg` swinging the top away from the
    # mass centre and down, into the street).
    cand = [idx_all[p] for p in lost_paths
            if p in idx_all and qs.is_opening((idx_all[p].get("p") or {}), g.n_sub)]
    rng.shuffle(cand)
    n_take = rng.randint(int(n_lean[0]), min(int(n_lean[1]), len(cand))) if cand else 0
    leaning = cand[:max(0, n_take)]
    if leaning:
        m = g.mass_of(leaning[0])
        ox, oy = qf._outward(m, sd)
        ax, ay = -oy, ox
        lnx, lny = qs._SIDE_NORMAL[sd]
        for e in leaning:
            qs._unremove(pctx, qs._path(e))
            sx, sy, sz = qs._size(e)
            depth = sy if sd in ("S", "N") else sx
            plx = e["lx"] + lnx * depth / 2.0
            ply = e["ly"] + lny * depth / 2.0
            px, py = qf._to_world(m, plx, ply)
            deg = rng.uniform(*lean_deg)
            plan["displaced"][qs._path(e)] = qs._disp(
                pivot=(px, py, e["z"]), axis=(ax, ay, 0.0), deg=deg,
                why="facade_collapse: leaning macroblock off {0}, still "
                    "standing, pitched {1:.0f} deg outward -- one of the "
                    "recipe's 1-2 survivors".format(sd, deg))
            plan["macroblocks"] = plan.get("macroblocks") or []
            plan["macroblocks"].append({
                "path": qs._path(e), "side": sd, "recipe": "facade_collapse",
                "storey": int(e.get("storey", (e.get("p") or {}).get("_storey", 0))),
                "deg": float(deg), "pivot": [float(px), float(py), float(e["z"])],
                "size": [sx, sy, sz]})

    # the roof is marked SHED (the roof stream skips its own peel pass on
    # it) and its piece(s) ledgered as debris -- the SAME named exception
    # `t_top_storey_loss` already uses for "never role in (core, roof)".
    plan["roof_shed"] = True
    n_roof = 0
    for e in g.role_pieces(("roof",)):
        p = qs._path(e)
        if p and p not in plan["_removed_set"]:
            plan["_removed_set"].add(p)
            plan["removed"].append(p)
            n_roof += 1

    plan["regions"].append({
        "recipe": "facade_collapse", "side": sd,
        "storeys": [int(s) for s in sts], "bays": [int(b) for b in run],
        "cells": sorted(list(c) for c in region)})
    # "untouched BY THIS RECIPE", not a claim about the whole plan -- an
    # earlier T4 recipe (`chunk`'s own corner draw, say) may independently
    # touch a few pieces of an adjacent side too; this recipe's own region
    # never does (`corners=[]` above), and that is the ONLY thing this note
    # is entitled to say (§"Labels follow the data" -- the module's own
    # round-2 bug 12).
    other_sides = "/".join(s for s in qs.SIDES if s != sd)
    _note(pctx, "facade_collapse: {0}'s street/windward facade off across "
                "{1} storey(s){2} ({3} piece(s) incl. {4} corner), {5} "
                "leaning macroblock(s), roof shed as debris ({6} piece(s)), "
                "{7} reclaimed displacement(s) from an earlier T4 recipe -- "
                "this recipe's own region never touches the other 3 "
                "elevations ({8}), building STANDS".format(
                    sd, len(sts),
                    " (incl. the GROUND storey -- the one recipe this "
                    "ladder allows to touch it)" if has_ground else "",
                    len(res["lost"]), len(res["corners"]), len(leaning),
                    n_roof, reclaimed, other_sides))


RECIPES_T = {
    "glass_loss": t_glass_loss,
    "roof_props_sweep": t_roof_props_sweep,
    "parapet_fall": t_parapet_fall,
    "panel_loss": t_panel_loss,
    "cladding_band": t_cladding_band,
    "hanging_panels": t_hanging_panels,
    "chunk": t_chunk,
    "out_of_plane_top": t_out_of_plane_top,
    "top_storey_loss": t_top_storey_loss,
    "facade_collapse": t_facade_collapse,
}


# ---------------------------------------------------------------------------
# THE LADDER — LADDER_T[btype][level] -> [(recipe, kwargs), ...]
# ---------------------------------------------------------------------------
# `rc_glass`'s base glass fractions are set HIGHER than urm/rc at every
# level ("GLASS dominates at every level", §2.6's per-type note) — the
# opposite of `quake_sliced.LADDER_S`, whose rc_glass DG1 glass share is
# SMALLER than urm/rc's (a curtain-wall tower moves less in an earthquake).
# Wind pressure and windborne debris are the opposite comparison: a pane is
# the single most wind-vulnerable element on any of these three
# construction types, and it is ALL a curtain wall has on its skin.
LADDER_T = {
    "urm": {
        "T0": [],
        "T1": [
            ("glass_loss", {"base_frac": (0.05, 0.15), "n_sides": 1}),
            ("roof_props_sweep", {}),
        ],
        "T2": [
            ("glass_loss", {"base_frac": (0.20, 0.45), "n_sides": 2}),
            ("parapet_fall", {"n_sides": 2, "frac": (0.30, 0.60)}),
            ("panel_loss", {"frac": (0.05, 0.10)}),
            ("roof_props_sweep", {}),
        ],
        "T3": [
            ("cladding_band", {"storeys": (2, 4), "bay_frac": (0.40, 0.70)}),
            ("glass_loss", {"base_frac": (0.50, 0.80), "n_sides": 4}),
            ("parapet_fall", {"n_sides": 3, "frac": (0.30, 0.60)}),
            ("roof_props_sweep", {}),
            ("hanging_panels", {"n": (1, 3), "deg": (25.0, 70.0)}),
        ],
        "T4": [
            ("cladding_band", {"storeys": (2, 4), "bay_frac": (0.40, 0.70)}),
            ("glass_loss", {"base_frac": (0.75, 0.95), "n_sides": 4}),
            ("parapet_fall", {"n_sides": 3, "frac": (0.30, 0.60)}),
            ("roof_props_sweep", {}),
            ("hanging_panels", {"n": (1, 3), "deg": (25.0, 70.0)}),
            ("chunk", {}),
            ("out_of_plane_top", {}),
            ("top_storey_loss", {}),
            # LAST, deliberately (`t_facade_collapse`'s own docstring,
            # "RECLAIMS THE SIDE FROM AN EARLIER T4 RECIPE"): self-gated to
            # <= 4 storeys and i >= 0.82, so on most urm lowrise buildings
            # this is a no-op behind every recipe above it; when it DOES
            # fire, it has final say over whichever elevation it and
            # `out_of_plane_top`/`hanging_panels` both rank windward.
            ("facade_collapse", {}),
        ],
    },
    "rc": {
        "T0": [],
        "T1": [
            ("glass_loss", {"base_frac": (0.05, 0.15), "n_sides": 1}),
            ("roof_props_sweep", {}),
        ],
        "T2": [
            ("panel_loss", {"frac": (0.20, 0.35)}),
            ("glass_loss", {"base_frac": (0.20, 0.45), "n_sides": 2}),
            ("parapet_fall", {"n_sides": 2, "frac": (0.30, 0.60)}),
            ("roof_props_sweep", {}),
        ],
        "T3": [
            ("cladding_band", {"storeys": (2, 4), "bay_frac": (0.40, 0.70),
                               "keep_pier": (0.45, 0.65)}),
            ("panel_loss", {"frac": (0.20, 0.35)}),
            ("glass_loss", {"base_frac": (0.50, 0.80), "n_sides": 4}),
            ("parapet_fall", {"n_sides": 3, "frac": (0.30, 0.60)}),
            ("roof_props_sweep", {}),
            ("hanging_panels", {"n": (1, 3), "deg": (25.0, 70.0)}),
        ],
        "T4": [
            ("cladding_band", {"storeys": (2, 4), "bay_frac": (0.40, 0.70),
                               "keep_pier": (0.45, 0.65)}),
            ("panel_loss", {"frac": (0.20, 0.35)}),
            ("glass_loss", {"base_frac": (0.50, 0.80), "n_sides": 4}),
            ("parapet_fall", {"n_sides": 3, "frac": (0.30, 0.60)}),
            ("roof_props_sweep", {}),
            ("hanging_panels", {"n": (1, 3), "deg": (25.0, 70.0)}),
            ("chunk", {"keep_pier": (0.45, 0.65)}),
        ],
    },
    "rc_glass": {
        "T0": [],
        # T1's glass share is SMALLER than the naive "glass dominates at
        # every level" reading above would suggest, not larger: R2's
        # research (`_plans/urban_tornado_research.md` §8.1, the WISE 2006
        # DI 18/19 damage-of-degree tables) puts the first curtain-wall
        # glass DOD at EXP 101 mph -- the EF1/EF2 BOUNDARY, i.e. the TOP of
        # T1's own intensity band, not its floor -- while rooftop items
        # (DOD 2/3, 83-92 mph) are squarely in range and stay on
        # `roof_props_sweep`. "A few panes, corner-weighted" is what a
        # building at the LOW end of T1 actually shows; T2 and up are
        # unchanged (Fort Worth 2000's EF1-EF2 glass cascade is a T2
        # phenomenon and the T2 base 0.35-0.60 already models it).
        "T1": [
            ("glass_loss", {"base_frac": (0.02, 0.06), "n_sides": 1}),
            ("roof_props_sweep", {}),
        ],
        "T2": [
            ("glass_loss", {"base_frac": (0.35, 0.60), "n_sides": 2}),
            ("parapet_fall", {"n_sides": 2, "frac": (0.30, 0.60)}),
            ("panel_loss", {"frac": (0.05, 0.10)}),
            ("roof_props_sweep", {}),
        ],
        "T3": [
            ("cladding_band", {"storeys": (2, 4), "bay_frac": (0.40, 0.70)}),
            ("glass_loss", {"base_frac": (0.65, 0.90), "n_sides": 4}),
            ("parapet_fall", {"n_sides": 3, "frac": (0.30, 0.60)}),
            ("roof_props_sweep", {}),
            ("hanging_panels", {"n": (1, 3), "deg": (25.0, 70.0)}),
        ],
        "T4": [
            ("cladding_band", {"storeys": (2, 4), "bay_frac": (0.40, 0.70)}),
            ("glass_loss", {"base_frac": (0.65, 0.90), "n_sides": 4}),
            ("parapet_fall", {"n_sides": 3, "frac": (0.30, 0.60)}),
            ("roof_props_sweep", {}),
            ("hanging_panels", {"n": (1, 3), "deg": (25.0, 70.0)}),
            # skin only (openings + piers of the region -- the region can
            # never contain a `core` piece to begin with, since `core`
            # placements carry no side and never enter `g.runs`); corner
            # pieces only in the top two storeys; NEVER a macroblock.
            ("chunk", {"corner_top_storeys": 2}),
        ],
    },
}


def _guard(recs, btype, info, height_class):
    """Refuse what §2.6's height-class table forbids outright, and say so.

    Three rules, all HARD caps rather than taste: no windward top-wall peel
    above midrise (`HEIGHT_CAPS[...]["out_of_plane"]`), `top_storey_loss` is
    lowrise-urm-only, and `facade_collapse` is lowrise-urm-only too (its
    other two eligibility gates — <= 4 storeys, intensity >= 0.82 — need the
    element table and the intensity respectively, neither of which reaches
    this function, so those two live inside `t_facade_collapse` itself; see
    that recipe's own "ELIGIBILITY" docstring paragraph). All three recipes
    already no-op defensively if called anyway (defence in depth for a
    future ladder edit that adds one to the wrong bucket by mistake), so
    refusing them here changes nothing about correctness — it changes the
    PLAN's own `recipes` record, so a reader of the plan sees the refusal
    instead of a recipe that silently did nothing.
    """
    H = float(info.get("H") or 0.0)
    tall = height_class in ("highrise", "tower")
    severe_local_bite = any(name == "chunk" for name, _kw in recs)
    out, notes = [], []
    for name, kw in recs:
        kw = dict(kw or {})
        # A whole intact semantic panel is never acceptable visible tornado
        # debris.  Panel cells remain useful to locate damage, but the visible
        # result must go through the fracture/tear path below.
        if name in ("panel_loss", "hanging_panels"):
            notes.append("guard: {0} refused -- intact rectangular facade "
                         "cells are localization metadata, not tornado "
                         "fracture geometry".format(name))
            continue
        # T4 used to stack a broad cladding band and a corner chunk.  That
        # creates two independent missing rectangles and reads as generalized
        # collapse.  At the severe level the single boundary-local chunk is
        # the structural envelope event; its perimeter is fractured later.
        if severe_local_bite and name == "cladding_band":
            notes.append("guard: cladding_band replaced by the single T4 "
                         "localized ragged edge bite")
            continue
        # A tornado is primarily an ENVELOPE event on a tall building.  The
        # old generic ladder could combine a multi-storey cladding rectangle,
        # scattered whole-panel removal, hanging rectangular slice cells and
        # a corner chunk on the same tower.  The result read as structural
        # collapse even though the frame remained numerically present.  Keep
        # tall RC/steel masses standing: glass_loss supplies the characteristic
        # full-height shattered curtain wall; parapet/roof plant may go; only
        # the most severe row (the one that contains `chunk`) gets ONE shallow,
        # one-bay-per-face ragged edge bite.  `_author_tears` gives that bite
        # its fractured perimeter, exact source material and exact source UV.
        if tall and name == "cladding_band":
            notes.append(
                "guard: {0} refused on a {1} building ({2:.0f} m) -- tall "
                "buildings retain their structural shell; tornado evidence "
                "is glazing/roof loss plus at most one localized ragged edge "
                "bite".format(name, height_class, H))
            continue
        if height_class != "lowrise" and name == "chunk":
            kw["max_storeys"] = 2
            kw["max_bays_per_side"] = 1
            notes.append(
                "guard: non-low-rise chunk localized to <=2 storeys and "
                "one edge bay per face; the frame and remaining shell stand")
        if name == "out_of_plane_top":
            if height_class != "lowrise" or not HEIGHT_CAPS.get(
                    height_class, {}).get("out_of_plane", False):
                notes.append(
                    "guard: out_of_plane_top refused on a {0} building "
                    "({1:.0f} m) -- whole-width top-wall peel is limited "
                    "to vulnerable low-rise masonry".format(height_class, H))
                continue
            if btype != "urm":
                notes.append("guard: out_of_plane_top refused -- urm-only "
                             "recipe, this building is {0}".format(btype))
                continue
        if name == "top_storey_loss" and (height_class != "lowrise" or btype != "urm"):
            notes.append("guard: top_storey_loss refused -- lowrise urm "
                         "only, this building is {0} {1}".format(
                             height_class, btype))
            continue
        if name == "facade_collapse" and (height_class != "lowrise" or btype != "urm"):
            notes.append(
                "guard: facade_collapse refused -- lowrise urm only, this "
                "building is {0} {1} (the storey-count and intensity gates "
                "of Sec8c are checked inside the recipe itself -- this "
                "guard has neither the element table nor the intensity "
                "here)".format(height_class, btype))
            continue
        out.append((name, kw))
    return out, notes


# ---------------------------------------------------------------------------
# POST-HOC CAP ENFORCEMENT — §2.6's caps as a second, independent gate
# ---------------------------------------------------------------------------
def _cell_of(e, n_sub):
    p = e.get("p") or {}
    sd = p.get("_side")
    if sd not in qs.SIDES:
        return None
    return (sd, int(p.get("_storey", 0)), qs.bay_no(p, n_sub))


def _facade_area_of(e):
    """A piece's own FAÇADE area — the two LARGEST dims of its `_size`
    multiplied, since a wall/pier/parapet piece's smallest dim is its own
    thickness, not part of the face an aerial camera reads. This is what
    `removed_frac` is measured against (below), not a piece COUNT: a real
    slice's coarse grid (`SM_Building_13`'s regular-grid fallback, one bay
    per side) can hide a 40 x 12 m hole behind a `removed_frac` of 0.04 by
    count when it is three whole wall runs."""
    sx, sy, sz = qs._size(e)
    a, b, _c = sorted((sx, sy, sz), reverse=True)
    return a * b


def _total_facade_area(g):
    return sum(_facade_area_of(e) for e in g.els
              if (e.get("p") or {}).get("_side") in qs.SIDES) or 1e-9


def _cell_dist_from_anchor(region_info, cell):
    """How far a cell sits from the region's own anchor (its corner, or its
    own bay midpoint when it has none) -- used to un-remove the FARTHEST
    pieces first when a plan is over the height-class cap, so what survives
    is the part of the region closest to the corner/anchor it grew from."""
    sd, st, b = cell
    cells = [tuple(c) for c in (region_info.get("cells") or [])]
    sts = [c[1] for c in cells] or [st]
    st_top = max(sts)
    d_st = st_top - st
    corner = region_info.get("corner")
    bays_here = sorted(set(c[2] for c in cells if c[0] == sd)) or [b]
    if corner:
        end = qs._CORNER_END.get((sd, corner))
        if end == "lo":
            d_b = b - bays_here[0]
        elif end == "hi":
            d_b = bays_here[-1] - b
        else:
            d_b = 0
    else:
        mid = bays_here[len(bays_here) // 2]
        d_b = abs(b - mid)
    return d_st + d_b


def _enforce_ground_floor_rule(pctx, plan, height_class):
    """Never a ground-storey pier removed on a TOWER (§2.6's closing
    paragraph, verbatim) -- it reads as a collapse in an aerial frame even
    though nothing else about the building did."""
    if height_class != "tower":
        return
    g = pctx["g"]
    idx = {qs._path(e): e for e in g.els}
    restored = []
    for p in list(plan["removed"]):
        e = idx.get(p)
        if not e:
            continue
        pp = e.get("p") or {}
        if int(pp.get("_storey", -1)) == 0 and qs.is_pier(pp, g.n_sub):
            plan["_removed_set"].discard(p)
            restored.append(p)
    if restored:
        rs = set(restored)
        plan["removed"] = [q for q in plan["removed"] if q not in rs]
        _note(pctx, "guard: {0} ground-storey pier(s) restored on a tower "
                    "-- never a ground-storey pier".format(len(restored)))


def _cap_removed_frac(pctx, plan, height_class):
    """If `removed_frac` — measured by FAÇADE AREA, not piece count (see
    `_facade_area_of`) — is still over the height class's cap once every
    recipe has run, un-remove pieces from the boundary of the LARGEST
    region -- farthest from its corner/anchor first -- until it is not.
    `core` and `roof` are never touched here (roof stays gone if
    `top_storey_loss` put it there; it is not a "removal" mistake to walk
    back, it is the one named exception).

    NAMED CARVE-OUT (Sec8c, `t_facade_collapse`'s own docstring): a
    `facade_collapse` region is NEVER a trim candidate below. The lowrise
    cap exists to stop an ACCIDENTAL over-gutted plan — several ordinary
    T3/T4 recipes stacking past a sane fraction by chance — not to forbid
    the one state that recipe exists to show: a real, documented single-
    elevation total facade loss (Waco/Nashville) is comfortably over the
    0.35 lowrise cap BY CONSTRUCTION (one full elevation of four, floor to
    roof, is already ~0.25 before anything else in the plan adds to it).
    Every OTHER region on the same plan still competes for trimming exactly
    as before; only `facade_collapse`'s own cells are exempt."""
    g = pctx["g"]
    cap = HEIGHT_CAPS.get(height_class, HEIGHT_CAPS["midrise"])["max_removed_frac"]
    idx = {qs._path(e): e for e in g.els}
    total_area = _total_facade_area(g)
    removed_area = sum(
        _facade_area_of(idx[p]) for p in plan["removed"]
        if p in idx and (idx[p].get("p") or {}).get("_side") in qs.SIDES)

    if removed_area / total_area <= cap:
        return
    idx_by_cell = {}
    for e in g.els:
        key = _cell_of(e, g.n_sub)
        if key is not None:
            idx_by_cell.setdefault(key, []).append(e)
    has_facade_collapse = any(r.get("recipe") == "facade_collapse"
                              for r in plan["regions"])
    regions = sorted((r for r in plan["regions"] if r.get("cells")
                      and r.get("recipe") != "facade_collapse"),
                     key=lambda r: -len(r["cells"]))
    restored_total = []
    for region in regions:
        if removed_area / total_area <= cap:
            break
        cells = [tuple(c) for c in region["cells"]]
        ranked_cells = sorted(cells, key=lambda c: -_cell_dist_from_anchor(region, c))
        for cell in ranked_cells:
            if removed_area / total_area <= cap:
                break
            for e in idx_by_cell.get(cell, []):
                p = qs._path(e)
                role = (e.get("p") or {}).get("_role")
                if p in plan["_removed_set"] and role not in ("core", "roof"):
                    plan["_removed_set"].discard(p)
                    restored_total.append(p)
                    if (e.get("p") or {}).get("_side") in qs.SIDES:
                        removed_area -= _facade_area_of(e)
    if restored_total:
        rs = set(restored_total)
        plan["removed"] = [q for q in plan["removed"] if q not in rs]
        _note(pctx, "cap: {0} piece(s) restored to bring removed_frac under "
                    "the {1} class cap ({2:.2f})".format(
                        len(restored_total), height_class, cap))
    if has_facade_collapse and removed_area / total_area > cap:
        _note(pctx, "cap: facade_collapse region exempted from the {0} "
                    "lowrise area cap ({1:.2f}, removed_frac {2:.2f}) -- the "
                    "cap exists to stop accidental gutting, not the "
                    "documented Waco/Nashville full-facade-loss state (R2 "
                    "Sec0.3/Sec6/Sec8c)".format(
                        height_class, cap, removed_area / total_area))


def _enforce_no_empty_storey(pctx, plan):
    """A storey is never left with zero standing pieces -- the last safety
    net, after the cap pass may have trimmed a region down."""
    g = pctx["g"]
    by_storey = {}
    for e in g.els:
        p = e.get("p") or {}
        by_storey.setdefault(int(p.get("_storey", 0)), []).append(e)
    restored = []
    for _st, els in by_storey.items():
        paths = [qs._path(e) for e in els if qs._path(e)]
        if paths and all(p in plan["_removed_set"] for p in paths):
            piers = [e for e in els if qs.is_pier(e.get("p") or {}, g.n_sub)]
            pick = piers[:2] if piers else els[:2]
            for e in pick:
                p = qs._path(e)
                if p in plan["_removed_set"]:
                    plan["_removed_set"].discard(p)
                    restored.append(p)
    if restored:
        rs = set(restored)
        plan["removed"] = [q for q in plan["removed"] if q not in rs]
        _note(pctx, "guard: {0} piece(s) restored -- a storey must never be "
                    "completely emptied".format(len(restored)))


# ---------------------------------------------------------------------------
# ROUND 3b F2a (§8e) -- RAGGED TEARS, reusing `quake_sliced._plan_tears`
# (pure geometry, `fire_collapse.plan_edges` under it) unchanged. See
# `_finalise`'s own call-site comment for WHERE this runs and why; this is
# WHAT it needs adapted, and both adaptations stay in THIS module (`quake_
# sliced.py` / `fire_collapse.py` are not touched anywhere in this file):
#
#   1. `_plan_tears` reads `plan["panels"]` unconditionally (`moved =
#      set(plan["displaced"]) | set(p for p, _s in plan["panels"])`) --
#      this ladder has no pile/panel mechanism of its own, so the schema
#      gained an always-empty `"panels"` field (see `plan_damage`'s dict
#      literal) purely so that line does not `KeyError`.
#   2. `fire_collapse.plan_edges`'s own `live()` filter -- which SURVIVING
#      piece is even ELIGIBLE to be torn -- keys off the RAW, pre-adapt
#      `e["role"]` (`fire_collapse.SHELL_ROLES`, which includes
#      "balcony"), not this ladder's own ADAPTED `p["_role"]` (`tornado_
#      kit.adapt`'s "core" catch-all for portico/pediment/ornament/
#      balcony decoration -- "never an addressable side/bay cell", that
#      module's own words). A balcony can therefore surface as a torn
#      NEIGHBOUR even though it is this ladder's own untouchable
#      decoration class; `_cap_tears` drops it, the same way `_plan_
#      tears` already drops a "core" piece on the KILLED side (it does
#      not on the torn/LIVE side -- that is this gap).
#   3. `_plan_tears` has no `budget`/cap ARGUMENT of its own (`QS_MAX_
#      TEARS` is a `quake_sliced`-owned module constant, applied inside
#      its own per-mass loop) -- `TU_MAX_TEARS` is this module's OWN,
#      independently settable env knob, defaulting to `quake_sliced.
#      QS_MAX_TEARS`'s CURRENT value, applied as a post-hoc trim of the
#      STABLE order `_plan_tears` already returns.
# ---------------------------------------------------------------------------
TU_MAX_TEARS = int(_os.environ.get("TU_MAX_TEARS", str(qs.QS_MAX_TEARS))
                   or qs.QS_MAX_TEARS)


def _cap_tears(jobs, cap, border=None):
    """See the section comment above for both things this does and why
    neither is a change to `quake_sliced._plan_tears` itself: drop any job
    whose torn piece is this ladder's own "core" decoration class, then
    keep only the first `cap` non-dropped jobs (STABLE order), marking
    the rest `dropped=True` -- the exact field `_author_tears`
    (`disaster/tornado_urban_usd.py`, stream FX1's apply-side half) already
    reads to skip a job.

    ROUND 4 (D2, v6 review) -- `border` IS NEVER DROPPED. The user's "some
    of the walls still have even breaks" is this cap: a job on a piece that
    actually borders a hole is what makes that hole ragged, and dropping it
    leaves the slicer's own straight piece boundary showing. Border jobs
    (`_hole_border_paths`) are kept unconditionally and do not count against
    `cap`; the cap now bounds only the DECORATIVE tears — pieces near the
    damage but not on a hole edge — so the authored total stays bounded by
    the hole perimeter, which the height-class area caps already bound.
    `border` None (or `TU_TEAR_BORDER=0`) is the round-3 flat cap."""
    border = set(border or ()) if TU_TEAR_BORDER_ON else set()
    kept = 0
    n_border_kept = 0
    for j in jobs:
        if j.get("dropped"):
            continue
        e = j.get("el")
        name = str((e or {}).get("name") or "").lower()
        sliced_shell_core = name.startswith(("core_x", "core_y"))
        if (e is not None and (e.get("p") or {}).get("_role") == "core"
                and not sliced_shell_core):
            j["dropped"] = True
            continue
        if qs._path(e) in border:
            j["border"] = True
            n_border_kept += 1
            continue
        if kept >= int(cap):
            j["dropped"] = True
        else:
            kept += 1
    return jobs


# ---------------------------------------------------------------------------
# ROUND 4 (D2, v6 review) -- THE SLICED SUPPORT POST-PASS: "no piece stands
# on air".
#
# The user, on the v6 lit bench: "/World/tornado_bench/A3/cell/pieces/
# wall_N_1_10_0204 -- some walls like this are still floating". Measured:
# that piece is storey 10 of an 11-storey slice and every piece in its own
# bay column below it had been removed by `t_out_of_plane_top` + `t_chunk`,
# so it survived hanging in the air over a hole.
#
# Stream K built this rule for the KIT path (`tornado_kit._unsupported` /
# `_support_closure`, inside `kit_guard`), and `tornado_urban` grew only the
# ROOF half of it (`_shed_unsupported_roof`). The SLICED path had nothing
# for walls. This is K's vocabulary re-stated on the sliced grid -- same
# `_Grid` cell addressing (`g.at((side, storey, bay))`, `g.corners`,
# `qs.bay_no`), same two-clause run rule, same closure -- and NOT an import
# of it: `tornado_kit` imports THIS module (`import ... tornado_urban as tu`),
# so depending on it here would be a cycle.
#
# ONE RULE IS DELIBERATELY NOT PORTED: K's `core` ORNAMENT test. On the kit
# grid `core` is `tornado_kit.adapt`'s catch-all for portico / pediment /
# ornament / balcony decoration; on a SLICED grid `core` is the building's
# own interior core (11 pieces on SM_Building_02, one per storey, spanning
# the footprint), which no recipe ever removes and which nothing else is
# holding up. Applying the ornament radius test to those would gut the
# building from the inside out. `core` is never unsupported here.
# ---------------------------------------------------------------------------
_SUPPORT_MAX_PASSES = 12
#: `TU_SUPPORT=0` restores round-3 behaviour (no wall support pass) -- the
#: A/B switch the container probe uses to report before/after.
TU_SUPPORT_ON = _os.environ.get("TU_SUPPORT", "1") not in ("0", "", "false")
#: ROUND 5 (user review 2026-09-02, item 5/8: "lots of these floating walls
#: on A3, A4"). The grid support rule (`_column_dead` via `_cell_below`)
#: treats a bay column with NOTHING in it below as SUPPORTED — deliberately,
#: so a non-uniform storey table (the parapet band owns a storey index the
#: walls do not) is not mistaken for a hole. But that same leniency lets a
#: piece with genuinely nothing under it hang in the air. These two
#: tolerances gate a GEOMETRIC fallback used ONLY in that "grid sees nothing
#: below" case: a live piece counts as holding `e` up when their world-XY
#: footprints overlap by at least `_GEOM_SUPPORT_OVERLAP_M` on both axes and
#: its top reaches within `_GEOM_SUPPORT_GAP_M` of `e`'s base. A piece is
#: therefore shed only when NOTHING — neither the grid column nor any real
#: geometry — is beneath it, so a legitimate setback/cantilever (which has a
#: wall directly under it in world space even when the bay INDEX shifts
#: storey to storey) is never touched.
_GEOM_SUPPORT_GAP_M = 0.6
_GEOM_SUPPORT_OVERLAP_M = 0.3
#: The geometric floater fallback is only trusted on a REGULAR grid. TWO
#: gates, both measured on the real kits:
#:   * STOREY-VALUE DENSITY. A real slice carries ~1.3 distinct `_storey`
#:     values per metre of height (SM_Building_02: 51 values over 38.7 m);
#:     the degenerate SM_Building_09 carries ~25 (1488 over 58.9 m) — bogus
#:     per-piece storey values that make `_cell_below` meaningless and the
#:     geometric fallback cascade to 75 % removed. Cap at `_PER_M` values per
#:     metre (floor `_FLOOR`), which sits an order of magnitude above every
#:     real building and an order below the degenerate one.
#:   * EMPTY-BELOW FRACTION, kept as a secondary guard.
#: When either fails, the round-3 leniency ("nothing below -> supported") is
#: kept exactly as before this round.
_GEOM_REGULAR_MAX_EMPTY = 0.4
_GEOM_MAX_STOREYS_PER_M = 4.0
_GEOM_MAX_STOREYS_FLOOR = 60


def _cell_el(g, side, storey, bay):
    """A representative placement at `(side, storey, bay)` for the geometric
    support test — `None` when the cell is empty."""
    got = g.at((side, storey, bay))
    return got[0] if got else None


def _geom_support_below(g, e, removed):
    """Is any LIVE piece physically beneath `e`'s footprint, close enough to
    hold it up? World-XY bbox overlap of at least `_GEOM_SUPPORT_OVERLAP_M`
    on both axes with a piece whose top sits within `_GEOM_SUPPORT_GAP_M` of
    `e`'s base and whose base is below `e`'s (genuinely under it). Catches
    the real support the grid's bay-index walk misses; first supporter wins
    (early exit)."""
    if e is None:
        return False
    ex, ey, ez = float(e.get("x", 0.0)), float(e.get("y", 0.0)), float(e.get("z", 0.0))
    sx, sy, _sz = (float(v) for v in qs._size(e))
    ex0, ex1 = ex - sx / 2.0, ex + sx / 2.0
    ey0, ey1 = ey - sy / 2.0, ey + sy / 2.0
    for e2 in g.els:
        if e2 is e:
            continue
        q = qs._path(e2)
        if not q or q in removed:
            continue
        # A supporter is VERTICAL STRUCTURE, not the central core (which
        # spans the whole footprint and would "hold up" every perimeter
        # wall above it — the leniency this pass exists to remove) and not a
        # roof/slab.
        if (e2.get("p") or {}).get("_role") in ("core", "roof"):
            continue
        z2 = float(e2.get("z", 0.0))
        s2x, s2y, s2z = (float(v) for v in qs._size(e2))
        top2 = z2 + s2z
        if top2 < ez - _GEOM_SUPPORT_GAP_M:      # too low to reach the base
            continue
        if z2 >= ez - 1e-6:                       # at or above e, not below it
            continue
        x2, y2 = float(e2.get("x", 0.0)), float(e2.get("y", 0.0))
        ox = min(ex1, x2 + s2x / 2.0) - max(ex0, x2 - s2x / 2.0)
        oy = min(ey1, y2 + s2y / 2.0) - max(ey0, y2 - s2y / 2.0)
        if ox >= _GEOM_SUPPORT_OVERLAP_M and oy >= _GEOM_SUPPORT_OVERLAP_M:
            return True
    return False


def _cell_below(g, side, storey, bay):
    """The pieces DIRECTLY under `(side, storey, bay)` — the nearest storey
    below that actually carries a piece in that bay column, not blindly
    `storey - 1`. `tornado_kit._cell_below`'s rule: a slice's storey table
    is not uniform either (SM_Building_02's parapet band owns storey 11
    while the wall runs stop at 10), and a bay empty at one index is not
    evidence of a hole. `[]` when nothing at all sits under this cell."""
    for s in range(int(storey) - 1, -1, -1):
        got = g.at((side, s, bay))
        if got:
            return got
    return []


def _corner_below(g, corner, storey):
    """`_cell_below` for a corner column (`_Grid.corners`)."""
    for s in range(int(storey) - 1, -1, -1):
        got = g.corners.get((corner, s))
        if got:
            return got
    return []


def _column_dead(g, side, storey, bay, removed):
    """Is the whole bay column under `(side, storey, bay)` gone?

    ROUND 5 (item 5/8, floating walls): when the GRID sees nothing in this
    bay column below (`_cell_below` empty), do not conclude "supported"
    outright — that leniency is what left `wall_N_1_10_0204` hanging. Fall
    back to a GEOMETRIC test: the column is dead (nothing holds this cell up)
    unless a live piece is physically beneath the cell's own piece
    (`_geom_support_below`). A regular building's every upper piece has a
    wall directly under it in world space, so this only ever fires "dead"
    on a genuine floater.

    THE GEOMETRIC FALLBACK IS GATED to REGULAR grids (`g._tu_geom_support`,
    set once by `_shed_unsupported_walls`). On a DEGENERATE slice
    (SM_Building_09: pieces spread over ~1488 bogus `_storey` values, so
    `_cell_below` is empty for nearly every piece) the fallback would fire on
    everything and its inevitable few false-negatives cascade through the
    closure to gut the building — measured 75 % removed on that fixture. When
    the flag is absent/False the round-3 leniency ("nothing below ->
    supported") is kept, exactly as before this round.
    """
    below = _cell_below(g, side, storey, bay)
    if not below:
        if getattr(g, "_tu_geom_support", False):
            return not _geom_support_below(g, _cell_el(g, side, storey, bay),
                                           removed)
        return False
    return all(qs._path(e) in removed for e in below)


def _unsupported(g, e, removed):
    """Does element `e` stand on air, given the `removed` path set?

    * `wall` / `pier` on a main side — unsupported when its OWN bay column
      below is entirely gone AND it has no live NEIGHBOUR to span to (an
      adjacent bay at its own storey that both survives and has a live
      column of its own). The second clause is what keeps `quake_sliced.
      _apply_region`'s TOOTHING working: a kept boundary pier inside a lost
      band is standing on the bay next to it, which is the whole point of
      toothing, while `wall_N_1_10_0204` — a whole bay column gone under it
      and its neighbours gone too — still fails.
    * `corner` / `parapet_corner` — the column test ONLY. A corner is the
      END of both its runs; there is no bay beyond it to span to.
    * `parapet` — the run rule, same as a wall. `_shed_unsupported_roof`
      owns the "its wall BAND emptied" case and this owns "its own column
      went"; both only ever add, and they agree.
    * `core` — never (see the section comment).
    * `roof` — NOT decided here; `_shed_unsupported_roof` owns roof tiles
      and slabs and is re-run after this pass.
    """
    p = e.get("p") or {}
    role = p.get("_role")
    if role in ("roof", "core"):
        return False
    side = p.get("_side")
    storey = int(p.get("_storey", 0))

    if side in qs.SIDES:
        if storey <= 0:
            return False
        bay = qs.bay_no(p, g.n_sub)
        if not _column_dead(g, side, storey, bay, removed):
            return False
        if role in ("corner", "parapet_corner"):
            return True
        for nb in (bay - 1, bay + 1):
            for e2 in g.at((side, storey, nb)):
                if qs._path(e2) in removed:
                    continue
                if not _column_dead(g, side, storey, nb, removed):
                    return False
        return True

    if side in qs._CORNER_SIDES:
        if storey <= 0:
            return False
        below = _corner_below(g, side, storey)
        if not below:
            return False
        return all(qs._path(e2) in removed for e2 in below)

    return False


def _support_closure(g, seeds, protect=()):
    """`seeds` grown until nothing standing is unsupported. Returns
    `(removed_set, shed_paths)`, `shed_paths` in discovery order so the
    caller can ledger exactly what the SUPPORT rule took as opposed to what
    a recipe took. `tornado_kit._support_closure`'s body on this grid.

    `protect` — the plan's DISPLACED pieces — is never shed. This is where
    this pass diverges from `tornado_kit.kit_guard`, which DEMOTES an
    unsupported displaced piece to removed (its step 5). On the sliced
    ladder a displaced piece is not standing on air: `t_hanging_panels` and
    `t_out_of_plane_top` pitch it about its own BOTTOM OUTER EDGE and its
    own docstring says so ("STILL ATTACHED (`displaced`, never
    `removed`)"), and `t_facade_collapse`'s leaning macroblocks lean out of
    an elevation whose lower storeys that same recipe just took — by
    construction their column below is gone, and demoting them would delete
    the one state Sec8c's carve-out exists to show. They are attached at
    the pivot, so they are not candidates; they still count as LIVE for
    everything else's column test, exactly as in `kit_guard`."""
    removed = set(seeds)
    protect = set(protect)
    shed = []
    for _ in range(_SUPPORT_MAX_PASSES):
        newly = []
        for e in g.els:
            path = qs._path(e)
            if not path or path in removed or path in protect:
                continue
            if _unsupported(g, e, removed):
                newly.append(path)
        if not newly:
            break
        removed.update(newly)
        shed.extend(newly)
    return removed, shed


def _unsupported_survivors(g, removed, protect=()):
    """THE AUDIT — every SURVIVING piece that still stands on air, given a
    finished plan's removal set. Must be empty after `_finalise`; the
    container probe and `test_no_sliced_piece_stands_on_air` both assert
    that, and it is the one number the user's `wall_N_1_10_0204` note is
    about."""
    removed = set(removed)
    protect = set(protect)
    out = []
    for e in g.els:
        path = qs._path(e)
        if not path or path in removed or path in protect:
            continue
        if _unsupported(g, e, removed):
            out.append(path)
    return out


def _seed_keep_score(e, weights, top):
    """How much a RECIPE removal belongs in a tornado's damage zone —
    `tornado_kit._keep_score`'s shape on the sliced grid. The seed trade
    below hands back the LOWEST scores first, so what survives a trade is
    the coping, the top storeys, the windward face and the windward
    corner."""
    p = e.get("p") or {}
    role, side = p.get("_role"), p.get("_side")
    storey = int(p.get("_storey", 0))
    score = 0.0
    if role in ("parapet", "parapet_corner"):
        score += 4.0
    if role in ("corner",):
        score += 0.5
    score += 2.0 * float(weights.get(side, 0.0) or 0.0)
    score += 1.5 * (storey / float(max(1, top)))
    return score


def _shed_unsupported_walls(pctx, plan, height_class):
    """THE PASS. Demote every surviving wall/pier/corner/parapet piece that
    stands on air to `removed` — ledgered as debris like any other removal
    (the ledger and every removal-derived stat are RE-RUN downstream in
    `_finalise`, never patched).

    THE SEED TRADE, and why it both terminates and keeps the cap. The
    closure only ever ADDS, so it can push a plan that `_cap_removed_frac`
    just brought under the height-class area cap back over it — and those
    additions are NOT restorable by that function, which only ever hands
    back cells belonging to a `plan["regions"]` record. So when the closure
    overshoots, this hands back the least-wanted SEED (a RECIPE removal,
    lowest `_seed_keep_score` first) and re-derives the closure from the
    smaller seed set. `_support_closure` is monotone in its seeds — fewer
    seeds can only mean fewer sheds — so each trade strictly reduces the
    result and the loop is bounded by the seed count. The alternative,
    letting the closure win and noting the drift (which is the call
    `tornado_kit.kit_guard` makes), leaves a plan over a cap that R2 Sec8.6
    calls "strongly supported, do not loosen"; trading a seed keeps BOTH
    invariants, at the cost of one recipe removal.

    Returns the number of pieces the support rule took.
    """
    if not TU_SUPPORT_ON:
        _note(pctx, "support: DISABLED (TU_SUPPORT=0) -- round-3 behaviour")
        return 0
    g = pctx["g"]
    # ROUND 5 (item 5/8): decide ONCE whether this grid is REGULAR enough to
    # trust the geometric "nothing below -> floater" fallback (`_column_dead`
    # reads `g._tu_geom_support`). On a regular slice a wall/pier at storey>0
    # almost always has a grid piece in its own bay column below, so a
    # `_cell_below`-empty piece is a real signal; on a DEGENERATE slice
    # (SM_Building_09's ~1488 bogus `_storey` values) `_cell_below` is empty
    # for nearly everything and the fallback would gut the building. Gate on
    # the empty-below FRACTION over the perimeter pieces at storey>0.
    _perim = [e for e in g.els
              if (e.get("p") or {}).get("_role") in ("wall", "pier", "corner")
              and int((e.get("p") or {}).get("_storey", 0)) > 0
              and (e.get("p") or {}).get("_side") in qs.SIDES]
    _empty_below = sum(
        1 for e in _perim
        if not _cell_below(g, (e.get("p") or {}).get("_side"),
                           int((e.get("p") or {}).get("_storey", 0)),
                           qs.bay_no(e.get("p") or {}, g.n_sub)))
    _frac_empty = (_empty_below / float(len(_perim))) if _perim else 1.0
    _H = max(1.0, float(pctx.get("H") or 0.0))
    _storey_cap = max(_GEOM_MAX_STOREYS_FLOOR, _GEOM_MAX_STOREYS_PER_M * _H)
    _n_storeys = len(g.storeys)
    _regular = (bool(_perim) and _frac_empty <= _GEOM_REGULAR_MAX_EMPTY
                and _n_storeys <= _storey_cap)
    g._tu_geom_support = _regular
    _note(pctx, "support: geometric floater fallback {0} "
                "(empty-below {1:.2f} <= {2:.2f}? ; {3} storey-value(s) <= "
                "{4:.0f}?) over {5} perimeter piece(s)".format(
                    "ON" if _regular else "OFF (degenerate grid)",
                    _frac_empty, _GEOM_REGULAR_MAX_EMPTY, _n_storeys,
                    _storey_cap, len(_perim)))
    protect = set(plan.get("displaced") or ())
    # ...AND THE TOOTHING'S OWN KEPT PIERS (stream K's collision report,
    # verified by neutering: `test_toothing_no_boundary_row_is_all_or_
    # nothing` failed with "every boundary pier removed" and passed with
    # this pass off). `quake_sliced._apply_region` deliberately RETAINS a
    # boundary pier inside a lost band -- that is the anti-"rectangular
    # module cut-out" mechanism the whole ladder is built on, and it is
    # exactly the shape the column test flags: its own bay column is gone
    # under it by construction. A kept pier is load-bearing BY DESIGN
    # INTENT; the ragged edge is the point. The plan does not carry
    # `res["kept_piers"]` (the recipes only quote the count in a note), so
    # it is re-derived here from the region records: a SURVIVING piece
    # whose grid cell belongs to a region the ladder emptied is, by
    # definition, a piece toothing chose to leave standing.
    region_cells = set()
    for r in plan.get("regions") or ():
        for c in r.get("cells") or ():
            region_cells.add(tuple(c))
    if region_cells:
        for e in g.els:
            q = qs._path(e)
            if not q or q in plan["_removed_set"]:
                continue
            if _cell_of(e, g.n_sub) in region_cells:
                protect.add(q)
    cap = HEIGHT_CAPS.get(height_class, HEIGHT_CAPS["midrise"])["max_removed_frac"]
    total_area = _total_facade_area(g) or 1.0
    idx = {qs._path(e): e for e in g.els}
    weights = plan.get("side_weights") or {}
    top = max([int((e.get("p") or {}).get("_storey", 0)) for e in g.els] or [1])

    def _frac(rm):
        return sum(_facade_area_of(idx[q]) for q in rm
                   if q in idx
                   and (idx[q].get("p") or {}).get("_side") in qs.SIDES) / total_area

    seeds = set(plan["_removed_set"])
    n_traded = 0
    removed, shed = _support_closure(g, seeds, protect)
    for _ in range(len(seeds) + 4):
        if _frac(removed) <= cap or not seeds:
            break
        worst = min(seeds, key=lambda q: _seed_keep_score(
            idx.get(q) or {}, weights, top))
        seeds.discard(worst)
        n_traded += 1
        removed, shed = _support_closure(g, seeds, protect)

    plan["_removed_set"] = set(removed)
    plan["removed"] = sorted(removed)
    plan["_support_protect"] = sorted(protect)
    plan["support_shed"] = sorted(shed)
    if shed:
        _note(pctx, "support: {0} piece(s) shed that would have stood on air "
                    "(own bay column below entirely gone and no live "
                    "neighbour bay to span to; a corner has no bay to span "
                    "to at all){1}".format(
                        len(shed),
                        "" if not n_traded else
                        "; {0} recipe removal(s) traded back to stay under "
                        "the {1} area cap {2:.2f}".format(
                            n_traded, height_class, cap)))
    return len(shed)


# ---------------------------------------------------------------------------
# ROUND 4 (D2, v6 review) -- TEAR COVERAGE: no hole edge left square.
#
# The other half of the same user note: "some of the walls still have even
# breaks". Measured on the T4 probe: `quake_sliced._plan_tears` hands
# `fire_collapse.plan_edges` a budget of `QS_MAX_TEARS` (40) PER MASS and
# `_cap_tears` then capped the kept jobs at `TU_MAX_TEARS` (the same 40), so
# on a plan that opens more hole perimeter than that — every T4 — the pieces
# past the budget got no tear job at all and their break stayed on the
# slicer's own straight piece boundary.
#
# Two changes, both here (neither `quake_sliced` nor `fire_collapse` is
# touched): the PLANNING budget is raised for the duration of this module's
# own `_plan_tears` call, and `_cap_tears` never drops a job on a piece that
# actually borders a hole. The cap still bounds the DECORATIVE tears (a
# piece near a hole but not on its edge); the border set is bounded by the
# hole perimeter, which is bounded by the height-class area caps.
# ---------------------------------------------------------------------------
#: What `quake_sliced._plan_tears` is allowed to PLAN (it reads
#: `qs.QS_MAX_TEARS` at call time as its `fire_collapse.plan_edges` budget).
#: Raised only for the duration of the call below and restored in a
#: `finally`, so no other consumer of that module sees a different value.
TU_TEAR_PLAN_BUDGET = int(_os.environ.get("TU_TEAR_PLAN_BUDGET", "240") or 240)
#: How big a gap between two footprints still counts as TOUCHING when
#: `fire_collapse.plan_edges` decides whether a surviving piece is on a
#: hole's edge. `quake_sliced.QS_TEAR_TOL_M` is 0.6 m, tuned on modelled KIT
#: modules that butt; a SLICED piece's footprint is the bbox of a region cut
#: and consecutive cells DO NOT BUTT (`plan_edges`'s own EDGE_GAP_FRAC note),
#: so on this path the 0.6 m left real hole-edge neighbours unclassified —
#: measured, see `_plan_tears_wide`.
TU_TEAR_TOL_M = float(_os.environ.get("TU_TEAR_TOL_M", "0.75") or 0.75)
#: `TU_TEAR_BORDER=0` restores round-3 behaviour (a flat cap that can leave
#: a hole edge untorn) -- the A/B switch for the probe.
TU_TEAR_BORDER_ON = _os.environ.get(
    "TU_TEAR_BORDER", "1") not in ("0", "", "false")


def _plan_tears_wide(pctx, plan):
    """`quake_sliced._plan_tears` with the planning budget raised to
    `TU_TEAR_PLAN_BUDGET` for the duration of the call.

    `_plan_tears` passes `budget=max(0, QS_MAX_TEARS - len(out))` into
    `fire_collapse.plan_edges`, reading that module global at CALL time, and
    it is documented as an env-overridable knob. Raising it here (and
    restoring it in a `finally`) is the only way to plan more border tears
    without editing `quake_sliced.py`, which this stream does not own. The
    number of jobs actually AUTHORED is still bounded by `_cap_tears`."""
    old_budget, old_tol = qs.QS_MAX_TEARS, qs.QS_TEAR_TOL_M
    # `fire_collapse.plan_edges` reads the top-level `storey`; tornado's
    # region grid and `_hole_border_paths` use the authoritative sliced
    # `_storey`. Align them only for this borrowed planning call. Keeping the
    # two indices divergent left real hole-border pieces with no job, hence a
    # perfectly straight storey-cell silhouette. Restore every value below so
    # the shared element table and the fire/quake paths are untouched.
    saved_storeys = []
    for e in pctx["info"].get("elements") or ():
        p = e.get("p") or {}
        if "_storey" in p:
            saved_storeys.append((e, "storey" in e, e.get("storey")))
            e["storey"] = int(p["_storey"])
    try:
        qs.QS_MAX_TEARS = max(old_budget, TU_TEAR_PLAN_BUDGET)
        if TU_TEAR_TOL_M > 0:
            qs.QS_TEAR_TOL_M = TU_TEAR_TOL_M
        return qs._plan_tears(pctx, plan)
    finally:
        qs.QS_MAX_TEARS, qs.QS_TEAR_TOL_M = old_budget, old_tol
        for e, existed, value in saved_storeys:
            if existed:
                e["storey"] = value
            else:
                e.pop("storey", None)


def _border_over(e_live, m=None):
    """`plan_edges`'s own `over = min(1.2, 0.3 * w)` — how much two pieces
    one storey apart must OVERLAP along the wall before either is the
    other's hole edge, measured on the SURVIVING piece's own width. A flat
    threshold was looser than that on a narrow pier (0.51 m of overlap
    against a 3.07 m pier reads as an edge at 0.30 and does not at 0.92),
    and every piece the two rules disagreed about was a piece the audit
    then reported as an untorn border."""
    a0, a1 = _along_lo_hi(e_live, None, m)
    return min(1.2, 0.3 * max(0.3, a1 - a0))
_BORDER_TOL_M = 0.75      # `quake_sliced.QS_TEAR_TOL_M`'s 0.6 plus a little:
                          # a sliced piece's footprint is the bbox of a
                          # region cut and consecutive cells do not butt
                          # (`fire_collapse.plan_edges`'s own EDGE_GAP_FRAC
                          # note), so "touching" needs slack.


def _touch_xy(e1, e2, tol=_BORDER_TOL_M):
    """Do these two pieces' footprints touch in PLAN? Half-extent sum plus
    `tol` on each axis, from `quake_sliced._size` and `describe`'s own
    `lx`/`ly`. Used for the CORNER links only — a run-to-run neighbour is
    decided by the grid's own bay/storey indices, which are exact."""
    sx1, sy1, _ = qs._size(e1)
    sx2, sy2, _ = qs._size(e2)
    dx = abs(float(e1.get("lx", 0.0)) - float(e2.get("lx", 0.0)))
    dy = abs(float(e1.get("ly", 0.0)) - float(e2.get("ly", 0.0)))
    return (dx <= (sx1 + sx2) / 2.0 + tol) and (dy <= (sy1 + sy2) / 2.0 + tol)


def _corner_tear_jobs(pctx, plan, already):
    """Tear jobs for the CORNER pieces beside a hole — the ones
    `fire_collapse.plan_edges` structurally cannot reach.

    THE GAP, measured on the T3/T4 probes: `quake_sliced._plan_tears` builds
    its `edge_plan["sides"]` from the removed pieces' sides FILTERED TO
    `qs.SIDES` (the four main elevations), and `plan_edges`'s own `live()`
    then enumerates `e["side"] == sd` for those sides only. A sliced
    `corner` / `parapet_corner` piece carries `side` "SW"/"SE"/"NW"/"NE",
    so it is in neither the lost set nor the `return`-class adjoining set
    and NO job is ever emitted for it. 22 of 54 hole-border pieces on the
    SM_Building_02 T3 plan were corner pieces butting the removed band's
    own bay (`corner_SW_*` at x -14.0..-11.8 against `pier_S_0_*` starting
    at -11.8 — they share an edge exactly), each keeping the slicer's own
    square break. That is the user's "some of the walls still have even
    breaks", on the windward corner where a tornado chunk always is.

    THE FIX, without editing `fire_collapse` or `quake_sliced`: call
    `plan_edges` a SECOND time per lost side with a SHADOW element list —
    copies of the corner elements whose `side` is rewritten to that lost
    main side. That is geometrically the right reading (a SW corner IS the
    low-x end of the S wall line, which is exactly what `el_span`'s S
    convention measures) and it makes them enumerable. The copies carry the
    SAME `p` dict, so `_tears_to_json` records the real prim path and
    `_author_tears` re-resolves the real element at apply time; the job's
    recorded `side` is the LOST side, which is the direction the hole is on
    and the one `_tear_perimeter` wants for `quake_flow._outward`.

    A separate `_tear_rng` instance is used, so the run-piece draws in the
    first pass are untouched and the corner cuts are still deterministic.
    """
    from . import fire_collapse as fc
    info = pctx["info"]
    removed = set(plan["_removed_set"])
    moved = set(plan.get("displaced") or ()) | {q for q, _s in plan["panels"]}
    scope = plan.get("tear_scope") or {}
    if not scope:
        return []
    prng = qs._tear_rng(info, plan)
    out = []
    seen = set(already)
    for mass in sorted(scope):
        sc = scope[mass] or {}
        m = info["masses"].get(mass) or info["masses"]["main"]
        for sd in sc.get("sides") or ():
            kill = [e for e in info["elements"]
                    if (e.get("mass") or "main") == mass
                    and e.get("side") == sd and qs._path(e) in removed]
            if not kill:
                continue
            shadow = []
            for e in info["elements"]:
                cs = (e.get("p") or {}).get("_side")
                if cs not in qs._CORNER_SIDES or sd not in qs._CORNER_SIDES[cs]:
                    continue
                q = qs._path(e)
                if not q or q in removed or q in moved or q in seen:
                    continue
                if (e.get("mass") or "main") != mass:
                    continue
                c = dict(e)
                c["side"] = sd
                shadow.append(c)
            if not shadow:
                continue
            ctx2 = {"info": dict(info, elements=list(kill) + shadow)}
            edge_plan = {"mass": mass, "sides": (sd,),
                         "storeys": sorted(sc.get("storeys") or ()),
                         "pad_m": qs.TEAR_PAD_M, "kill": kill}
            for j in fc.plan_edges(ctx2, edge_plan, m, prng,
                                   tol=(TU_TEAR_TOL_M or qs.QS_TEAR_TOL_M),
                                   budget=TU_TEAR_PLAN_BUDGET):
                q = qs._path(j.get("el") or {})
                if not q or q in seen:
                    continue
                j["mass"] = mass
                j["dropped"] = bool(j.get("dropped"))
                seen.add(q)
                out.append(j)
    return out


def _sliced_core_tear_jobs(pctx, plan, already):
    """Tear exposed ``core_x/core_y`` shell strips beside a removed cell.

    These are genuine sliced source geometry, not kit ornament.  Their role
    is ``core`` because they have no single facade side, so the normal edge
    planner cannot enumerate them and leaves a factory-square end visible.
    Temporarily project each touching strip onto the removed elevation and
    reuse the same edge planner/judges as ordinary wall panels.
    """
    from . import fire_collapse as fc
    info = pctx["info"]
    removed = set(plan["_removed_set"])
    scope = plan.get("tear_scope") or {}
    seen = set(already)
    out = []
    prng = qs._tear_rng(info, plan)
    for mass in sorted(scope):
        m = info["masses"].get(mass) or info["masses"]["main"]
        for sd in (scope[mass] or {}).get("sides") or ():
            kill = [e for e in info["elements"]
                    if (e.get("mass") or "main") == mass
                    and e.get("side") == sd and qs._path(e) in removed]
            if not kill:
                continue
            shadow = []
            for e in info["elements"]:
                q = qs._path(e)
                name = str(e.get("name") or "").lower()
                if (not name.startswith(("core_x", "core_y")) or not q or
                        q in removed or q in seen or
                        (e.get("mass") or "main") != mass):
                    continue
                if not any(abs(int(e.get("storey", 0)) -
                               int(dead.get("storey", 0))) <= 1 and
                           _touch_xy(e, dead) for dead in kill):
                    continue
                c = dict(e)
                c["role"], c["side"] = "wall", sd
                shadow.append(c)
            if not shadow:
                continue
            ctx2 = {"info": dict(info, elements=list(kill) + shadow)}
            edge_plan = {"mass": mass, "sides": (sd,),
                         "storeys": sorted((scope[mass] or {}).get("storeys") or ()),
                         "pad_m": qs.TEAR_PAD_M, "kill": kill}
            for j in fc.plan_edges(ctx2, edge_plan, m, prng,
                                   tol=(TU_TEAR_TOL_M or qs.QS_TEAR_TOL_M),
                                   budget=TU_TEAR_PLAN_BUDGET):
                q = qs._path(j.get("el") or {})
                if not q or q in seen:
                    continue
                j["mass"], j["dropped"] = mass, bool(j.get("dropped"))
                seen.add(q)
                out.append(j)
    return out


def _along_lo_hi(e, side=None, m=None):
    """A piece's span along an ELEVATION's own axis. With a mass frame this
    is `fire_collapse.el_span` ITSELF — the very function `plan_edges` uses
    to decide its `left`/`right`/`below`/`above` classes, so the audit and
    the pass that fills it cannot disagree at the margin (they did: a
    half-`_size` span put two pieces 0.5 m into each other where `el_span`'s
    measured width put them 0.1 m apart, and every such piece surfaced as a
    phantom "untorn border"). Without one it falls back to the placement's
    own centre and `_size`, which is all a host-side fixture has.

    `side` defaults to the piece's own; a CORNER piece has no single axis of
    its own, so a caller comparing one against a run piece passes the RUN's
    side — the same rewrite `_corner_tear_jobs` hands `plan_edges`."""
    if side is None:
        side = (e.get("p") or {}).get("_side") or e.get("side")
    if m is not None:
        try:
            from . import fire_collapse as fc
            t0, t1 = fc.el_span(m, e if e.get("side") == side
                                else dict(e, side=side))
            return float(t0), float(t1)
        except Exception:                                   # noqa: BLE001
            pass
    sx, sy, _sz = qs._size(e)
    if side in ("S", "N"):
        c, half = float(e.get("lx", 0.0)), sx / 2.0
    else:
        c, half = float(e.get("ly", 0.0)), sy / 2.0
    return c - half, c + half


def _along_overlap(e1, e2, side=None, m=None):
    """How much two pieces overlap along an elevation — `plan_edges`'s own
    `min(b, t1) - max(a, t0)` test, which is what separates a piece UNDER A
    HOLE from a piece under the kept pier beside it (and a corner under the
    hole from one diagonally past its end)."""
    a0, a1 = _along_lo_hi(e1, side, m)
    b0, b1 = _along_lo_hi(e2, side, m)
    return min(a1, b1) - max(a0, b0)


def _hole_border_paths(g, removed, moved=(), masses=None):
    """Every SURVIVING piece that touches a hole — `fire_collapse.
    plan_edges`'s own adjacency, restated on this grid.

    THE DEFINITION MATTERS, and getting it wrong twice is what this
    docstring is for. Measured on the real SM_Building_02 T4 plan:

      * an INDEX rule (same side, one storey, one BAY) claimed 24 pieces
        that no hole is near. A `_Grid` bay holds `n_sub` runs (3 on this
        asset), so "same bay" spans up to three pieces wide, and the
        toothing deliberately keeps a boundary pier inside a lost band —
        `pier_S_2_04` is in the band's own bay and directly under the pier
        that SURVIVED, not under the hole.
      * an INDEX rule for the corners claimed all seven storeys of the SW
        corner against a band at the other end of the S run.

    So both links are geometric, and split by relation exactly as
    `plan_edges` splits its classes:

      same storey  -> ABUTTING along the wall is the edge (`left`/`right`).
      one storey   -> the spans must OVERLAP (`below`/`above`), not merely
                      touch at a corner; a diagonal neighbour is not an
                      edge, which is `plan_edges`'s own `over` test.
      corners      -> footprint proximity (`_touch_xy`), since a corner is
                      at ONE end of each of its two runs.

    This set says which of `plan_edges`' jobs the cap may never drop, and
    counts what a plan left square (`stats["n_border_untorn"]`)."""
    removed = set(removed)
    # A DISPLACED piece is never a tear candidate — `quake_sliced.
    # _plan_tears` drops any job on a piece in `plan["displaced"]`
    # ("`_break_split`ing a prim that is about to be relocated tears a dead
    # reference out from under the move"), so counting one as an untorn
    # border would report a defect the pipeline is right to have.
    moved = set(moved)
    border = set()

    def _mass(e):
        return (masses or {}).get(e.get("mass") or "main") if masses else None

    def _add(els, against=None, mode="touch", axis=None):
        for e2 in els or ():
            q = qs._path(e2)
            if not q or q in removed or q in moved:
                continue
            # Roof-edge trim is shed to the debris ledger, never fractured
            # in place (`tornado_urban_usd._author_tears`). It is therefore
            # not a visible wall-hole border and must not make this coverage
            # audit report a knowingly unsupported tear job.
            pp2 = e2.get("p") or {}
            words = " ".join((str(pp2.get("_role") or e2.get("role") or ""),
                              str(e2.get("name") or ""))).lower()
            if any(k in words for k in
                   ("parapet", "cornice", "coping", "roof", "ledge")):
                continue
            if against is not None:
                m = _mass(e2)
                if mode == "overlap":
                    if _along_overlap(against, e2, axis, m) <= _border_over(e2, m):
                        continue
                elif not _touch_xy(against, e2):
                    continue
            border.add(q)

    for e in g.els:
        path = qs._path(e)
        if not path or path not in removed:
            continue
        p = e.get("p") or {}
        side = p.get("_side")
        storey = int(p.get("_storey", 0))
        if side in qs.SIDES:
            bay = qs.bay_no(p, g.n_sub)
            for ds in (-1, 0, 1):
                for db in (-1, 0, 1):
                    if ds == 0 and db == 0:
                        continue
                    _add(g.at((side, storey + ds, bay + db)), against=e,
                         mode="touch" if ds == 0 else "overlap", axis=side)
            for cs, sides_of in qs._CORNER_SIDES.items():
                if side not in sides_of:
                    continue
                # SAME RELATION SPLIT AS THE RUNS, measured on the RUN's own
                # axis: a corner beside the hole is an edge, a corner one
                # storey up or down and past the hole's end is not.
                for ds in (-1, 0, 1):
                    _add(g.corners.get((cs, storey + ds)), against=e,
                         mode="touch" if ds == 0 else "overlap", axis=side)
        elif side in qs._CORNER_SIDES:
            for ds in (-1, 0, 1):
                if ds:
                    _add(g.corners.get((side, storey + ds)))
            for sd in qs._CORNER_SIDES[side]:
                for ds in (-1, 0, 1):
                    for b in sorted(g.sides.get(sd, ()) or ()):
                        _add(g.at((sd, storey + ds, b)), against=e,
                             mode="touch" if ds == 0 else "overlap", axis=sd)
    return border


def _fill_missing_border_tears(pctx, plan, border, jobs):
    """Give every geometric hole border a shallow cut toward its nearest
    removed neighbour when the borrowed edge classifier emitted no job."""
    seen = {qs._path(j.get("el") or {}) for j in jobs}
    missing = sorted(set(border) - seen)
    if not missing:
        return jobs
    info = pctx["info"]
    idx = {qs._path(e): e for e in info["elements"]}
    dead = [e for e in info["elements"]
            if qs._path(e) in plan["_removed_set"]]
    for path in missing:
        e = idx.get(path)
        if not e:
            continue
        pp = e.get("p") or {}
        side = pp.get("_side")
        if side not in qs.SIDES:
            continue
        mass = e.get("mass") or "main"
        st = int(pp.get("_storey", e.get("storey", 0)))
        candidates = [d for d in dead
                      if (d.get("mass") or "main") == mass
                      and (d.get("p") or {}).get("_side") == side
                      and abs(int((d.get("p") or {}).get(
                          "_storey", d.get("storey", 0))) - st) <= 1]
        if not candidates:
            continue
        d = min(candidates, key=lambda q:
                abs(float(q.get("lx", 0.0)) - float(e.get("lx", 0.0))) +
                abs(float(q.get("ly", 0.0)) - float(e.get("ly", 0.0))) +
                3.0 * abs(int((q.get("p") or {}).get(
                    "_storey", q.get("storey", 0))) - st))
        dst = int((d.get("p") or {}).get("_storey", d.get("storey", 0)))
        if dst != st:
            z0 = float(e.get("z", 0.0))
            h = max(0.3, float(e.get("h", qs._size(e)[2])))
            above = dst > st
            pen = min(0.45, 0.18 * h)
            cut = {"cls": "below" if above else "above", "kind": "z",
                   "pen": pen, "line": z0 + (h - pen if above else pen),
                   "loose_above": above, "amp": min(0.22, 0.7 * pen)}
        else:
            m = info["masses"].get(mass) or info["masses"]["main"]
            a0, a1 = _along_lo_hi(e, side, m)
            b0, b1 = _along_lo_hi(d, side, m)
            higher = (b0 + b1) > (a0 + a1)
            pen = min(0.45, 0.18 * max(0.3, a1 - a0))
            cut = {"cls": "left" if higher else "right", "kind": "v",
                   "pen": pen, "line": a1 - pen if higher else a0 + pen,
                   "loose_hi": higher, "amp": min(0.22, 0.7 * pen)}
        jobs.append({"el": e, "side": side, "storey": st, "mass": mass,
                     "classes": [cut["cls"]], "cuts": [cut],
                     "dropped": False, "border": True})
    return jobs


# ---------------------------------------------------------------------------
# ROUND 3b F1 (§8e) -- THE ROOF SUPPORT POST-PASS (user, on the first lit
# bench: "floating roofs, missing chunks from buildings ... Look at the
# various skills on how to damage buildings"). Generalises the fire
# skill's own collapse-before-art ordering rule ("anything that takes a
# wall away must run before the passes that author art ON walls",
# `.agents/skills/build-urban-fire-scenes/SKILL.md`) and `fix-floating-
# debris`'s support test (a piece is supported when something ELSE'S OWN
# TOP lands in its vertical span, never by its own advertised seat) to a
# PLANNER post-pass -- this ladder authors removal and (via `disaster/
# tornado_roof.py`) roof art from the SAME plan, not two ordered stage-
# side steps, so the post-pass has to run inside the planner, after every
# recipe (and every cap/restore guard) has had its say.
# ---------------------------------------------------------------------------
ROOF_SHED_FRAC = 0.35        # sliced/slab: the plan brief's own "~35%" --
                             # share of the top storey's own wall/pier/
                             # corner pieces gone before a SLAB roof piece
                             # (one deck spanning most of the footprint)
                             # sheds whole
_ROOF_TILE_FRAC = 0.25       # a roof piece covering more than this share of
                             # its OWN mass's footprint area (W * D) is a
                             # SLAB (a real GAC/downtowncity deck --
                             # `detail/gac_storey_slice.roof_and_parapet`
                             # always collapses one storey band to ONE
                             # piece -- or a kit style whose single roof
                             # tile already spans the whole footprint, e.g.
                             # `urban_building`'s 23.08 m "SM_Roof") --
                             # tested by the AREA rule (`ROOF_SHED_FRAC`)
                             # above. Anything smaller is a discrete TILE
                             # (a kit style's 4-8 m roof-module grid,
                             # `urban_building._roof`'s own nx*ny tiling)
                             # -- tested by LOCAL bay-column support
                             # below. Deliberately NOT a kit/sliced
                             # provenance flag: this piece-geometry split
                             # already tells the two apart, on BOTH
                             # sources, with no `tornado_kit.py` adapter
                             # field needed.
_ROOF_SUPPORT_RADIUS_M = 6.0  # a TILE roof piece or a parapet/parapet_
                              # corner piece within this planar distance
                              # of a surviving top-storey wall/pier/corner
                              # piece counts as supported by it -- bigger
                              # than half the widest measured kit roof-
                              # tile pitch (8 m), so a tile whose own
                              # centroid sits over its own bay column
                              # always reaches the piece under it


def _local_support(g, plan, mass, lx, ly, storey, radius=_ROOF_SUPPORT_RADIUS_M):
    """True when a wall/pier/corner piece of `mass` at `storey`, within
    `radius` of local point `(lx, ly)`, SURVIVES (is not in
    `plan["_removed_set"]`). A tile/parapet piece with none nearby AT ALL
    is treated as supported -- an interior roof tile, say, is never near
    a wall this ladder ever removes (it never touches floor/interior
    structure), so "nothing nearby" is not evidence of a hole, only of
    being far from any wall."""
    near = False
    for e in g.els:
        if (e.get("mass") or "main") != mass:
            continue
        p = e.get("p") or {}
        if p.get("_role") not in ("wall", "pier", "corner"):
            continue
        if int(p.get("_storey", 0)) != storey:
            continue
        dx = float(e.get("lx", 0.0)) - lx
        dy = float(e.get("ly", 0.0)) - ly
        if (dx * dx + dy * dy) ** 0.5 <= radius:
            near = True
            if qs._path(e) not in plan["_removed_set"]:
                return True
    return not near


def _wall_band_storey(g, mass, storey):
    """The highest storey `<= storey` that has ANY wall/pier/corner piece
    of `mass` -- the real wall band a roof/parapet piece at `storey` sits
    above, needed because `gac_storey_slice.roof_and_parapet`'s own
    "whole band" fallback (its own docstring: "MEASURED: most of this
    stock's top band does NOT have room for a second cut... ring() the
    WHOLE band once... and relabel by position") relabels the ENTIRE
    topmost storey band to `parapet`/`parapet_corner`/`roof` on the
    majority of real GAC buildings measured, leaving that exact storey
    index with ZERO `wall`/`pier`/`corner` pieces at all -- a bare
    `_local_support(..., storey=p["_storey"])` call would then find
    NOTHING nearby, ever, and `_local_support`'s own "nothing nearby
    means supported" rule would leave every parapet permanently
    un-sheddable on exactly this (common) construction. `urban_building`'s
    OWN kit parapet band, by contrast, does NOT raise `z` past itself
    (`build_building`'s own comment: "a band marked parapet does not
    raise the roof, which sits at its base") so a KIT parapet's `_storey`
    already equals its own wall band's -- MEASURED (`walkup`, seed 7):
    storey 6 carries `wall`/`corner`/`parapet`/`parapet_corner`/`roof` all
    at once. This function returns `storey` unchanged in that case (the
    `<=` search finds it immediately) and only walks downward on the
    sliced whole-band case. `storey` itself (not `0`) when the mass
    carries no wall/pier/corner piece at any storey at all -- refuses to
    invent a false floor rather than guess."""
    candidates = sorted({int((e.get("p") or {}).get("_storey", 0))
                        for e in g.els
                        if (e.get("mass") or "main") == mass
                        and (e.get("p") or {}).get("_role") in ("wall", "pier", "corner")
                        and int((e.get("p") or {}).get("_storey", 0)) <= storey})
    return candidates[-1] if candidates else storey


def _top_storey_removed_frac(g, plan, mass, storey):
    """Fraction of `mass`'s own wall/pier/corner pieces at `storey` that
    are in `plan["_removed_set"]` -- the SLAB roof-shed test's own metric.
    `0.0` (never sheds) when the mass carries no such piece at that
    storey at all."""
    els = [e for e in g.els
          if (e.get("mass") or "main") == mass
          and int((e.get("p") or {}).get("_storey", 0)) == storey
          and (e.get("p") or {}).get("_role") in ("wall", "pier", "corner")]
    if not els:
        return 0.0
    removed = sum(1 for e in els if qs._path(e) in plan["_removed_set"])
    return removed / float(len(els))


def _shed_unsupported_roof(pctx, plan):
    """Walk every `roof`/`parapet`/`parapet_corner` element and take down
    anything left standing over an emptied span. Returns `(n_roof,
    n_parapet)`.

    PARAPET / PARAPET_CORNER: each piece's own local `_local_support`
    test against ITS OWN (mass, storey) -- "a parapet piece above a lost
    top-storey cell always goes with it" (plan brief, verbatim).

    ROOF: split per mass into SLAB pieces (tested by `ROOF_SHED_FRAC`,
    `_top_storey_removed_frac`) and TILE pieces (tested individually by
    `_local_support`) -- see `_ROOF_TILE_FRAC`'s own comment for why this
    split needs no kit/sliced provenance flag. Every roof piece of a mass
    that ends up shed (by either test) sets `plan["roof_shed"] = True`.

    THE `roof_shed` SIGNAL, NORMALISED. Verified by reading `disaster/
    tornado_roof.py`'s `_roof_already_shed`: it does NOT read `plan[
    "roof_shed"]` at all -- it checks `plan["regions"]` for an entry whose
    "recipe" is literally `"top_storey_loss"` (`t_top_storey_loss`'s own
    region tag). The plan brief's own phrasing ("a `roof_shed` flag, which
    `tornado_roof` already honours") does not match the code as it
    stands: `t_facade_collapse` (an existing T4 recipe, R11/stream C3,
    outside this pass's own editable region) already sets `plan[
    "roof_shed"] = True` and ledgers its roof piece(s) correctly but tags
    its OWN region `"facade_collapse"`, which `_roof_already_shed` does
    NOT match -- so `tornado_roof` would still draw a synthetic peel
    patch/lip/sheets/coping/props on a roof plane whose real piece is
    already gone, on exactly the showcased bench cell (B1,
    `brownstone_row` T4) the user was looking at when they reported
    "floating roofs". Fixed here, ADDITIVELY (no existing field renamed,
    `t_facade_collapse`'s own body untouched -- it is not this pass's
    file region): any plan carrying `roof_shed=True` (from this pass or
    an earlier recipe) gets a `top_storey_loss`-tagged region appended if
    it does not already have one, so `tornado_roof.py` -- stream RF's
    file, not edited here -- reads the SAME signal regardless of which
    recipe actually shed the roof.
    """
    g = pctx["g"]
    masses = pctx["info"]["masses"]
    n_roof = n_parapet = 0
    # A piece can be REMOVED, DISPLACED or SURVIVING -- never two of them
    # (`quake_sliced._finalise`'s own rule, `_plan_tears`'s `moved` set
    # mirrors it). `t_facade_collapse` (an earlier T4 recipe) can leave a
    # parapet piece `displaced` (one of its own 1-2 leaning macroblock
    # survivors -- its own candidate draw is bay-position-based, `qs.
    # is_opening`, with no role check, the same role-blindness `test_
    # toothing_no_boundary_row_is_all_or_nothing` measured elsewhere, so a
    # parapet CAN be picked). This pass must never re-fate an ALREADY-
    # displaced piece as removed too.
    moved = set(plan.get("displaced") or {})

    def _shed(e):
        p = qs._path(e)
        if not p or p in plan["_removed_set"] or p in moved:
            return False
        plan["_removed_set"].add(p)
        plan["removed"].append(p)
        return True

    # -- parapet / parapet_corner -------------------------------------------
    for e in g.role_pieces(("parapet", "parapet_corner")):
        if qs._path(e) in moved:
            continue
        p = e.get("p") or {}
        mass = e.get("mass") or "main"
        storey = _wall_band_storey(g, mass, int(p.get("_storey", 0)))
        lx, ly = float(e.get("lx", 0.0)), float(e.get("ly", 0.0))
        if not _local_support(g, plan, mass, lx, ly, storey):
            if _shed(e):
                n_parapet += 1

    # -- roof: SLAB (area test) vs TILE (local support) ---------------------
    by_mass = {}
    for e in g.role_pieces(("roof",)):
        by_mass.setdefault(e.get("mass") or "main", []).append(e)
    shed_mass = set()
    for mass, pieces in sorted(by_mass.items()):
        m = masses.get(mass) or masses.get("main") or {}
        footprint = max(1.0, float(m.get("W", 0.0)) * float(m.get("D", 0.0)))
        slabs, tiles = [], []
        for e in pieces:
            sx, sy, _sz = qs._size(e)
            (slabs if (sx * sy) / footprint > _ROOF_TILE_FRAC
             else tiles).append(e)
        if slabs:
            raw_storey = max(int((e.get("p") or {}).get("_storey", 0))
                            for e in slabs)
            storey = _wall_band_storey(g, mass, raw_storey)
            frac = _top_storey_removed_frac(g, plan, mass, storey)
            if frac > ROOF_SHED_FRAC:
                for e in slabs:
                    if _shed(e):
                        n_roof += 1
                shed_mass.add(mass)
                _note(pctx, "roof: {0:.0f}% of {1}'s top-storey wall/pier/"
                            "corner pieces gone -- {2} roof slab piece(s) "
                            "shed as debris".format(100.0 * frac, mass,
                                                    len(slabs)))
        for e in tiles:
            p = e.get("p") or {}
            storey = _wall_band_storey(g, mass, int(p.get("_storey", 0)))
            lx, ly = float(e.get("lx", 0.0)), float(e.get("ly", 0.0))
            if not _local_support(g, plan, mass, lx, ly, storey):
                if _shed(e):
                    n_roof += 1
        if pieces and all(qs._path(e) in plan["_removed_set"]
                          for e in pieces):
            shed_mass.add(mass)

    if shed_mass:
        plan["roof_shed"] = True
    if plan.get("roof_shed") and not any(
            r.get("recipe") == "top_storey_loss" for r in plan["regions"]):
        plan["regions"].append({"recipe": "top_storey_loss", "sides": [],
                                "storeys": [], "cells": []})

    if n_roof or n_parapet:
        _note(pctx, "roof support: {0} roof piece(s), {1} parapet piece(s) "
                    "shed -- no roof/parapet element stands over an "
                    "emptied top-storey span".format(n_roof, n_parapet))
    return n_roof, n_parapet


# ---------------------------------------------------------------------------
# DEBRIS — §2.7's projectile deposition model
# ---------------------------------------------------------------------------
G_ACCEL = 9.80665
# DRAG COEFFICIENT BY FRAGMENT CLASS. Round 1's values (panel 0.55 / block
# 0.35 / coping 0.30 / glass 0.80 / deck 0.70) treated every class as a
# massless ballistic projectile -- a dropped coping block ended up 400-745 m
# from the building in the container probe (a REAL slice, `SM_Building_13`,
# T4). Physically wrong: only sheet goods (deck panels, glass shards) carry
# far on wind alone -- a dense masonry/concrete block hits its own terminal
# velocity within about a metre of falling and drifts only a small fraction
# of (wind speed x fall time) before it lands, so `block`/`coping`'s
# coefficients are an order smaller than `panel`'s, which is itself smaller
# than `deck`/`glass` (true sheet aerodynamics).
#
# R5 (round 2, 2026-09-01 -- urban ground-evidence review): `deck` is RETIRED
# as a kind. It used to name every roof-shed fragment regardless of
# construction type and, worse, was the one route by which a fragment could
# fall through to `tornado_urban_usd.debris_material`'s old
# `elif bucket == "deck": mat = planks.wood_material(...)` branch -- pale
# sawn timber on a masonry/curtain-wall city, the exact "suburb debris
# copied straight onto downtown" defect the user called out ("There
# shouldn't be all this wood debris everywhere ... you can't just copy it
# directly"). `_kind_of` now splits a roof piece into `membrane` (urm/rc:
# built-up/modified-bitumen flat roofing, the dominant real covering on
# masonry/concrete-frame stock) or `metal` (rc_glass: rooftop mechanical
# decking / standing-seam plant housings on a curtain-wall tower) -- see
# `_kind_of`'s own docstring. `metal` keeps roughly `panel`'s coefficient
# (a stiffer sheet than felt, but still a sheet, not a block); `membrane`
# keeps the old `deck` value (0.45 -- sheet goods fly the farthest of the
# structural classes, Fort Worth 2000's roofing-material throws).
_C_KIND = {"panel": 0.22, "block": 0.08, "coping": 0.08, "glass": 0.35,
          "membrane": 0.45, "metal": 0.30}
# A HARD CEILING on top of the coefficient, because a `sqrt(z)` ballistic
# term with no drag term still overshoots for a very tall release (there is
# no "the wind stopped pushing it" in this model otherwise). Fraction of the
# building's OWN height, floored so a short building's cap is not itself
# absurdly tight.
REACH_MAX_H = 1.5           # panel / block / coping / membrane / metal
REACH_MAX_H_GLASS = 0.8     # glass carries less far, relatively, than a sheet
REACH_FLOOR_M = 6.0
# ROUND 4 (D3) — THE ABSOLUTE REACH CEILING, in metres, on top of the two
# height-relative caps above.
#
# MEASURED, not guessed. The container probe (`tools/tornado_urban_probe.py
# SM_Building_02 T4 7 35`, a REAL 42.7 m slice) put its debris bbox at
# x [-17.9, 73.8] y [-11.0, 63.5] around a 25.3 x 10.9 m footprint —
# fragments 74 m from the cell origin, because `REACH_MAX_H * H` is 64 m on
# this building alone. On the round-3 bench that is off the plate: the A and
# C rows sit 70 m from the origin with the plate edge at +-120 m, i.e. 50 m
# of clearance, so the A row threw fragments onto the void past the north
# edge — the "fragments landed past the plate edge plus floating clusters at
# the border" defect in `bench_overview.png` (a fragment past the plate is
# seated at z~0 with no ground under it, which is what reads as floating).
#
# The region clamp (`_clamp_to_region`, above) was the round-3 answer and is
# still the right one where a caller KNOWS its plate — but it is opt-in and
# NOTHING in the tree sets `TU_PLATE_REGION` or passes `region=` (grepped:
# the only hits are this module, its tests and the skill), so it has never
# fired in a bench or a city run; the probe's own `stats["region"]` is
# `null`. This ceiling is the half that needs no caller at all.
#
# 30 m is also the RESEARCH number, not just the plate's: `_plans/
# urban_tornado_research.md` §5's NEAR-FIELD table is 15-20 m (roofing
# gravel 15 m, a precast curb chunk 20 m, Fort Worth 2000 / Joplin 2011) and
# §5 says in its own words "do not calibrate against the mile-scale
# numbers; they are a different transport mechanism". The LOFTED population
# (roofing sheets 800 m+) is deliberately not modelled: at that range the
# material is nowhere near its own building and reads as litter, which is
# the note the round-3 review already made about the corridor.
REACH_ABS_MAX_M = 30.0
_FILL = (0.6, 0.9)
# (length range, width range) a fragment's two free dimensions are drawn
# from. The THIRD dimension (thickness) is no longer derived from a target
# volume — round 1 did that, and an under-sized share (a thin sliver of a
# tiny volume budget with a wide/long draw) produced ~600 m "slabs" once a
# large piece's volume was spread across a couple of skinny fragments. A
# fragment's thickness is now the SOURCE PIECE's own thickness instead
# (`min(sx, sy)` of its `_size`, clamped) — see `_frag_thickness_and_count`.
#
# R5: `block`/`coping` are SQUATTER than round 1 shipped -- both ranges are
# now the SAME on both axes, which bounds the worst-case independent draw's
# aspect ratio to `hi/lo` by construction (0.72/0.30 = 2.40 for block,
# 0.50/0.22 = 2.27 for coping, both under the ~2.5 ceiling) instead of the
# old 1.0/0.3 = 3.33 and 1.2/0.2 = 6.0 -- a masonry LUMP reads as rubble
# from 60-90 m only if it is not noticeably longer than it is wide; a 6:1
# coping fragment is a plank with a different material name on it, which is
# exactly the suburb-lumber signature this round exists to remove.
#
# ROUND 4 (D3): `panel` IS NOW BLOCKY TOO, and this is the fragment class
# that actually shipped the lumber read. R5 left it elongated on the
# argument that "a spalled wall/cladding panel genuinely is rectangular" and
# never bounded the ratio -- measured on the container probe's own plan
# (`SM_Building_02` T4 s7, 81 `panel` fragments): aspect p50 2.46, p90 6.72,
# MAX 11.96, with 40% of them over 3:1. A 2.5 x 0.15 m box lying on asphalt
# under a brick building is a plank whatever its material hint says, and the
# user's round-3 verdict was exactly that ("masonry fragments read as
# scattered wooden planks"). The range is now 0.50-1.60 x 0.35-1.10 -- still
# rectangular, no longer a sliver -- and `_dims_for` clamps EVERY blocky-kind
# draw to `_MAX_ASPECT` so the bound holds by construction rather than by
# arithmetic on the table (block/coping already satisfy it; the clamp is a
# no-op for them and documents the intent).
#
# `membrane`/`metal` (the retired `deck` kind's two replacements, `_kind_of`)
# are the ONLY sheet classes left and keep their sheet-sized plan range: they
# are roof covering, they are the one thing that genuinely flies as a sheet,
# and they are dark grey/metal, never timber-coloured.
_MAX_ASPECT = 3.0
#: Kinds whose fragments must read as LUMPS, not boards -- every façade kind
#: `_kind_of` derives (wall -> panel, pier/corner -> block, parapet/
#: parapet_corner -> coping). The sheet kinds (`membrane`/`metal`) and
#: `glass` (sized by its own pane loop, never through `_dims_for`) are
#: deliberately outside it.
_BLOCKY_KINDS = frozenset({"panel", "block", "coping"})
_DIMS = {
    "panel": ((0.50, 1.60), (0.35, 1.10)),
    "block": ((0.30, 0.72), (0.30, 0.72)),
    "coping": ((0.22, 0.50), (0.22, 0.50)),
    "membrane": ((1.0, 3.0), (0.5, 2.0)),
    "metal": ((1.0, 3.0), (0.5, 2.0)),
}
# ROUND 4 (D3), the other half of "thickness real": the blocky (masonry)
# kinds get a 0.12 m FLOOR so a chunk of wall is a chunk and not a wafer,
# and the two SHEET kinds get a sheet's ceiling. `membrane` measured 0.400 m
# thick at the MEDIAN on the probe plan -- thickness is the source piece's
# own `min(sx, sy)` and a roof slab's plan dimension is metres, so every
# membrane fragment pinned to the old shared 0.40 ceiling and a "torn
# built-up roof sheet" came out a 3 x 2 x 0.4 m slab. Built-up/modified-
# bitumen roofing is a few centimetres; standing-seam or deck metal thinner
# still.
_THICK_RANGE = {"panel": (0.12, 0.40), "block": (0.12, 0.40),
                "coping": (0.12, 0.40), "membrane": (0.02, 0.08),
                "metal": (0.01, 0.06)}
# ROUND 4 (v6 lit-bench review, "the B rubble is still not fixed ... the kit
# berms render as uniform light-grey IDENTICAL boxes"). `_DIMS` above is one
# narrow range per kind, so every fragment of a class comes out within a
# factor of ~2.4 of every other one and a berm reads as a gravel bed. The
# real thing is BIMODAL: mostly small spall with a MINORITY of large units
# that fell whole -- `_plans/urban_tornado_research.md` §4's own Fort Worth
# 2000 note, "a spalled EIFS/cladding panel or a tilt-up wall panel falls as
# a unit near where it broke off". `_SLAB_SHARE` of every blocky fragment is
# drawn from this second table instead. Aspect stays inside `_MAX_ASPECT`
# BY CONSTRUCTION here too (panel 2.20/0.80 = 2.75, block 1.60/0.70 = 2.29,
# coping 1.20/0.45 = 2.67), so the clamp in `_dims_for` is a no-op on a slab
# draw and the "no planks" invariant holds for both halves of the mix.
_SLAB_SHARE = 0.20
_DIMS_SLAB = {
    "panel": ((1.20, 2.20), (0.80, 1.60)),
    "block": ((0.90, 1.60), (0.70, 1.30)),
    "coping": ((0.60, 1.20), (0.45, 0.90)),
}
# ROUND 4 (v6) — THE MASONRY TONE, and the reason a hint table is the right
# answer here rather than the building's own map.
#
# The debris class branches carry ONE masonry look, `brick` (Brick_Wall_Worn,
# measured linear mean 0.213/0.109/0.071 -- a red-brown BRICK map). That is
# right for the A row's red-brick GAC stock and it is what the v6 review
# APPROVED there, so it stays untouched. It is wrong under B1/B3, whose kit
# facades are WHITE STONE, and under B2, which is tan: a white building
# shedding red brick is the same "material that isn't this building's"
# defect the review has now called out twice.
#
# Why not just bind the building's own facade map? Because it is an ATLAS --
# `_tiling_safe`/`_TILING_SAFE_TOKENS` in `tornado_urban_usd` documents the
# measurement: `M_MBuilding03_Facades` is a packed sheet of windows, doors
# and trim, and a triplanar world projection of it paints a random crop on
# every fragment (that IS the "uniform light-grey identical boxes" in
# B1_obl/B3_obl -- an atlas averaged over a 0.4 m cube). The map cannot be
# reused; only its TONE can, and a tone is one token, not a texture.
#
# So: a per-STYLE tone token, stamped on the fragment by the ledger and
# resolved to (rubble map, tint) by `tornado_urban_usd._TONE_LOOK`. Styles
# absent from this table stamp NOTHING and take the class branch exactly as
# before -- every sliced (A-row) building, and any kit style not listed.
_KIT_TONE = {
    "brownstone_row": "stone",   # B1: white/cream stone facade
    "walkup": "stone",           # B3: same white stone kit stock
    "dw_terrace": "tan",         # B2: tan/buff brick-and-stucco terrace
}
#: How many TONE-JITTERED variants of a masonry debris material a berm is
#: split across. `build_debris` groups by (kind, label, tone, shade) so this
#: multiplies the mesh count for the blocky kinds ONLY (3 groups -> 9 on a
#: typical building; glass/membrane/metal stay at one each). Set to 1 to
#: turn the jitter off entirely.
_DEBRIS_SHADES = 3


def _stable_shade(source_path):
    """A stable 0..`_DEBRIS_SHADES`-1 offset for a source piece, so two
    pieces do not start their fragments in the same tone band (which would
    put every low-index fragment -- the ones `_thin_fragments` keeps first
    -- in one shade). `zlib.crc32` rather than `hash()`: `hash()` on a str
    is salted per process, and a plan must be identical run to run."""
    return int(_zlib.crc32(str(source_path).encode("utf-8")) % 997)


def _tone_for(style):
    """The masonry tone token for a building `style`, or `""` for one this
    table does not name (which is every sliced building -- their class
    branch is the approved A-row look and must not move)."""
    return _KIT_TONE.get(str(style or "").strip().lower(), "")
_MAT_BY_BTYPE = {
    ("urm", "panel"): "brick", ("urm", "block"): "brick",
    ("rc", "panel"): "concrete_panel", ("rc", "block"): "concrete_panel",
    ("rc_glass", "panel"): "metal", ("rc_glass", "block"): "metal",
}
# A structural piece's fragment COUNT absorbs its volume instead of the
# fragments' own dimensions (which stay bounded, see `_DIMS`/`_THICK_RANGE`)
# -- `_frag_thickness_and_count`'s own docstring. Capped per piece so one
# enormous wall run does not author hundreds of boxes for itself.
N_MAX_PER_PIECE = 40

# A tall rc_glass building's own glass count (§2.7: 1 shard per 0.35 m2 of
# a broken pane, capped 60 per PANE) is uncapped per BUILDING -- a single
# midrise T4 plan measured 1,400+ glass fragments, and a corridor of a
# hundred damaged buildings would author on the order of 150k shard boxes
# for what reads, from 60 m, as a glass carpet (the assembly's own job,
# `_plans/urban_tornado_plan.md` §2.9: "ONE merged mesh per (kind,
# material) class" -- a mesh IS one draw call regardless of vertex count,
# but 150k individually-placed boxes is still a planning-time and JSON-size
# problem no later merge undoes). This is a per-BUILDING ceiling, applied
# once every pane has been ledgered, by a DETERMINISTIC stride-based thin
# (never a random subsample, so the plan stays byte-identical per seed) —
# see `_thin_fragments`. `DEBRIS_MAX_PER_BUILDING` is the same idea for the
# non-glass (structural) fragments, added alongside the fragment-count fix
# above for the same reason: a coarse real slice (few, huge pieces) with
# `N_MAX_PER_PIECE` shards each can still add up per building.
GLASS_SHARDS_MAX_PER_BUILDING = int(
    _os.environ.get("TU_GLASS_SHARDS_MAX", "400") or 400)
# R5: 600 -> 400. The corridor was reading as generally littered rather than
# struck once the ground-evidence field (`disaster.tornado_urban_ground`)
# added its OWN street-level debris on top of every building's own ledger;
# a lower per-building ceiling keeps the two fields from double-covering the
# same asphalt. `GLASS_SHARDS_MAX_PER_BUILDING` is untouched -- glass carpets
# are the one debris signature the review asked for MORE of, not less.
DEBRIS_MAX_PER_BUILDING = int(_os.environ.get("TU_DEBRIS_MAX", "400") or 400)


def _thin_fragments(some_frags, cap):
    """Deterministically thin `some_frags` (any single class -- glass shards
    or the structural panel/block/coping/membrane/metal fragments, kept separate so
    one class's cap cannot starve the other) to EXACTLY `min(cap,
    len(some_frags))` fragments — one guaranteed fragment per SOURCE
    (`f["from"]`), then the remaining budget spent ROUND-ROBIN across
    sources (column 0 of every source's own remaining fragments, in
    original order, before column 1 of any source) rather than a random
    draw, so a re-plan of the same seed produces the same thinned set and
    no source's tail is favoured over another's. Every source keeps at
    least one fragment, unless `cap` itself is smaller than the number of
    sources, in which case that is the one remaining slack: some sources
    get none rather than none getting the guarantee.

    Returns `(kept, n_before_thin, was_thinned)`.
    """
    n_before = len(some_frags)
    if cap <= 0 or n_before <= cap:
        return some_frags, n_before, False
    by_src, order = {}, []
    for f in some_frags:
        src = f["from"]
        if src not in by_src:
            by_src[src] = []
            order.append(src)
        by_src[src].append(f)
    n_src = len(order)
    if n_src >= cap:
        return [by_src[src][0] for src in order[:cap]], n_before, True
    remaining_budget = cap - n_src
    out = [by_src[src][0] for src in order]
    col = 1
    while len(out) - n_src < remaining_budget:
        added = False
        for src in order:
            if len(out) - n_src >= remaining_budget:
                break
            frs = by_src[src]
            if col < len(frs):
                out.append(frs[col])
                added = True
        if not added:
            break
        col += 1
    return out, n_before, True


def _kind_of(pp, btype=None):
    """A removed piece's debris KIND. `btype` (added R5) decides which of
    the two roof-shed kinds a `role == "roof"` piece becomes -- see the
    `_C_KIND` comment above for the reach-coefficient reasoning and the
    module docstring's R5 note for why `deck` (the single kind this branch
    used to return, unconditionally) is retired: it was the one path by
    which a fragment could fall through to `planks.wood_material` in
    `tornado_urban_usd.debris_material`, and a masonry/curtain-wall city has
    no business shedding pale sawn timber.

    `urm`/`rc` (masonry-bearing-wall and concrete-frame) roofs are almost
    always a built-up or modified-bitumen MEMBRANE in the real record --
    the flat, dark, gravel- or cap-sheet covering on this stock's low/
    mid-rise fabric. `rc_glass` (curtain-wall tower) roofs instead shed
    rooftop mechanical decking and plant housings, which read as METAL. A
    single building only ever has one `btype`, so this never mixes the two
    kinds on one roof -- it decides which ONE of them that roof's shed
    material is.
    """
    role = pp.get("_role")
    if role in ("parapet", "parapet_corner"):
        return "coping"
    if role == "roof":
        return "metal" if btype == "rc_glass" else "membrane"
    if role == "wall":
        return "panel"
    if role in ("pier", "corner"):
        return "block"
    return "block"


#: Tokens that would route a debris fragment onto a MASONRY (brick/stone)
#: look — the one thing a GLASS building must never shed (user review
#: 2026-09-02, item 10: "If it's a glass building then why is there brick
#: debris on the floor?").
_MASONRY_TOKENS = ("brick", "stone", "masonry", "coping")


def _material_hint(pp, kind, btype):
    mat = pp.get("material") or pp.get("_material")
    # ROUND 5 (user review 2026-09-02, item 10): THE DEBRIS CLASS FOLLOWS THE
    # CONSTRUCTION TYPE. A curtain-wall (`rc_glass`) building sheds GLASS and
    # METAL/MULLION debris, never brick — so its coping is a metal cap, not a
    # masonry one, and a stray masonry-named `material` string on one of its
    # pieces is not allowed to route that piece's rubble onto the brick map.
    # (`_MAT_BY_BTYPE` already resolves its wall/pier panels to `metal`; this
    # closes the coping and the raw-material leaks.)
    if btype == "rc_glass":
        if kind in ("coping", "membrane", "metal"):
            return "metal"
        if mat and not any(t in str(mat).lower() for t in _MASONRY_TOKENS):
            return str(mat)
        return _MAT_BY_BTYPE.get((btype, kind), "metal")
    if mat:
        return str(mat)
    if kind == "coping":
        return "coping"
    if kind == "membrane":
        return "membrane"
    if kind == "metal":
        return "metal"
    return _MAT_BY_BTYPE.get((btype, kind), "concrete_panel")


# ROUND 3b (§8e F3, stream FX2) — which debris KINDS may inherit their
# source piece's own cladding texture (`_tex_url`/`_tex_name`, stamped by
# `tornado_urban_usd.annotate_surface`). `panel` (role `wall`), `block`
# (role `pier`/`corner`, and `_kind_of`'s own unmatched-role fallback) and
# `coping` (role `parapet`/`parapet_corner`) are every kind `_kind_of` ever
# derives from a FAÇADE role — the wall the debris actually broke off of.
# `membrane` and `metal` are the two ROOF-shed kinds (`_kind_of`'s
# `role == "roof"` branch, by `btype`) and are deliberately EXCLUDED even
# when the source piece happens to carry a texture: "Glass and membrane
# fragments keep their class looks (a torn membrane is roofing, not
# façade)" — a roof deck/membrane's own bound map is not a reliable stand-
# in for "what colour was this building's roofing", the same reason
# `annotate_surface` is never asked to resolve one. Note `metal` here is
# the KIND string (a roof-shed piece) and is NOT the same thing as the
# `metal` MATERIAL HINT `_material_hint` returns for a `panel`/`block`
# piece on an `rc_glass` (curtain-wall) building via `_MAT_BY_BTYPE` — that
# piece's KIND is still `panel`/`block`, so it passes this gate and DOES
# inherit its own mullion/spandrel-panel texture when one is bound; only a
# piece whose KIND is the string `"metal"` (always a roof deck) is held
# back. `glass` never reaches this gate at all — it is deposited from
# `plan["glass"]` by a separate loop below that never passes a texture in.
_FACADE_TEX_KINDS = frozenset({"panel", "block", "coping"})


def _lognormal(rng, sigma):
    return math.exp(rng.gauss(0.0, sigma))


def _mean_frag_area(kind):
    """Expected plan area of ONE fragment of `kind` -- the divisor that
    turns a piece's face area into a fragment COUNT
    (`_frag_thickness_and_count`). ROUND 4 (v6 review) it has to account
    for the SLAB MIX (`_DIMS_SLAB`/`_SLAB_SHARE`, below): a berm whose
    fragments are 20% large chunks has a larger mean fragment, so the same
    piece yields FEWER of them -- if this stayed the small-only mean the
    count would over-shoot the source volume by the slab share."""
    (l_lo, l_hi), (w_lo, w_hi) = _DIMS.get(kind, ((0.3, 1.0), (0.3, 1.0)))
    small = ((l_lo + l_hi) / 2.0) * ((w_lo + w_hi) / 2.0)
    slab_dims = _DIMS_SLAB.get(kind)
    if not slab_dims:
        return small
    (sl_lo, sl_hi), (sw_lo, sw_hi) = slab_dims
    slab = ((sl_lo + sl_hi) / 2.0) * ((sw_lo + sw_hi) / 2.0)
    return (1.0 - _SLAB_SHARE) * small + _SLAB_SHARE * slab


def _piece_face_area(size):
    a, b, _c = sorted((float(size[0]), float(size[1]), float(size[2])),
                      reverse=True)
    return a * b


def _frag_thickness_and_count(kind, size, fill):
    """`(thickness, n)` for a piece's own structural fragments.

    THICKNESS IS THE SOURCE PIECE'S OWN, never derived from a fragment's
    target volume (round 1's bug: an under-sized volume share with a wide
    random length/width draw could force a many-metres-thick "fragment").
    `min(sx, sy)` of the piece's own `_size` is the closest this vocabulary
    has to "how thick was the wall/pier/coping" — a slicer piece's Z extent
    is a storey height, not a material thickness — clamped to a masonry/
    concrete-panel-sane range.

    COUNT absorbs the volume instead: `piece_face_area / mean_frag_area`,
    scaled by `fill`, capped at `N_MAX_PER_PIECE` — so a coarse real slice's
    one giant wall run becomes a bounded NUMBER of plausibly-sized
    fragments rather than a handful of oversized ones. This makes volume
    conservation approximate rather than exact (the `N_MAX_PER_PIECE` cap
    can leave a huge piece's fragments well under its own volume — see
    `_ledger_removed`'s `debris_volume_m3` / `source_volume_m3` stats,
    which report both rather than pretending the ratio is always 1).
    """
    sx, sy, _sz = size
    lo, hi = _THICK_RANGE.get(kind, (0.05, 0.40))
    thick = min(hi, max(lo, min(float(sx), float(sy))))
    face_area = _piece_face_area(size)
    mean_area = max(1e-6, _mean_frag_area(kind))
    n = int(round(fill * face_area / mean_area))
    n = max(1, min(N_MAX_PER_PIECE, n))
    return thick, n


def _dims_for(kind, rng, thickness):
    """`[length, width, thickness]` for one fragment.

    ROUND 4 (D3) — THE ASPECT CLAMP. Two independent uniform draws over the
    `_DIMS` ranges can always produce a sliver (`l_hi / w_lo`: 11.96 was the
    worst `panel` on the probe plan). For a BLOCKY kind the two draws are
    kept -- so the rng stream, and therefore every seed's plan, advances
    exactly as before -- and the RESULT is clamped to `_MAX_ASPECT`: the
    short axis is pulled up into range first, and if the range itself cannot
    honour the bound the LONG axis is shortened instead, so
    `max/min <= _MAX_ASPECT` holds unconditionally rather than as a property
    of whichever numbers happen to be in the table. Sheet kinds
    (`membrane`/`metal`) are untouched -- a torn roof sheet IS long and thin.
    """
    table = _DIMS
    if kind in _DIMS_SLAB and rng.random() < _SLAB_SHARE:
        # ROUND 4 (v6 review, "the berm is identical cubes"): the SLAB
        # DRAW. A real wall breach drops a few large units -- a whole
        # spalled cladding panel, a corner block, a run of coping -- among
        # the small spall, and it is that SIZE CONTRAST that reads as
        # rubble from 60-90 m rather than as a gravel bed of identical
        # boxes. One extra `rng.random()` per fragment; `_mean_frag_area`
        # above already accounts for the mix so the fragment COUNT drops
        # to match rather than the pile gaining volume.
        table = _DIMS_SLAB
    (l_lo, l_hi), (w_lo, w_hi) = table.get(kind, ((0.3, 1.0), (0.3, 1.0)))
    length = rng.uniform(l_lo, l_hi)
    width = rng.uniform(w_lo, w_hi)
    if kind in _BLOCKY_KINDS:
        width = min(w_hi, max(width, length / _MAX_ASPECT))
        length = min(l_hi, max(length, width / _MAX_ASPECT))
        if max(length, width) > _MAX_ASPECT * min(length, width):
            # the ranges could not meet the bound (never with the table
            # above; defensive so a future range edit cannot reintroduce a
            # plank silently) -- shorten the long axis.
            if length > width:
                length = _MAX_ASPECT * width
            else:
                width = _MAX_ASPECT * length
    return [float(length), float(width), float(thickness)]


def _dir_to_local(m, dx, dy):
    a = math.radians(-m["yaw"])
    return (dx * math.cos(a) - dy * math.sin(a), dx * math.sin(a) + dy * math.cos(a))


def _exit_t(lx, ly, dlx, dly, w2, d2):
    ts = []
    if abs(dlx) > 1e-9:
        for edge in (w2, -w2):
            t = (edge - lx) / dlx
            if t > 1e-9 and abs(ly + t * dly) <= d2 + 1e-6:
                ts.append(t)
    if abs(dly) > 1e-9:
        for edge in (d2, -d2):
            t = (edge - ly) / dly
            if t > 1e-9 and abs(lx + t * dlx) <= w2 + 1e-6:
                ts.append(t)
    return min(ts) if ts else 0.0


def _push_out_of_footprint(m, wx, wy, bearing_rad):
    """A landing point inside the SOURCE building's own footprint pushed out
    to the façade line + 1.5 m, continuing along the SAME bearing the
    fragment was already travelling (§2.7, verbatim) -- not to the nearest
    edge, which is a different (and un-specified) rule."""
    lx, ly = qf._to_local(m, wx, wy)
    w2, d2 = m["W"] / 2.0, m["D"] / 2.0
    if abs(lx) <= w2 and abs(ly) <= d2:
        dlx, dly = _dir_to_local(m, math.cos(bearing_rad), math.sin(bearing_rad))
        t = _exit_t(lx, ly, dlx, dly, w2, d2)
        lx = lx + (t + 1.5) * dlx
        ly = ly + (t + 1.5) * dly
        return qf._to_world(m, lx, ly)
    return wx, wy


# ---------------------------------------------------------------------------
# ROUND 3 (2026-09-01) — THE RUBBLE BERM, `_plans/urban_tornado_plan.md` §8
# R8: "DEBRIS MUST CONCENTRATE"
# ---------------------------------------------------------------------------
# User verdict on the round-2 bench: "it looks like wooden planks scattered
# everywhere rather than actual street props." Round 2 fixed the MATERIAL
# (no more `deck`/wood, R5) but left the SPATIAL pattern untouched — every
# removed piece's fragments were 100% ballistic, thrown downwind, so a
# façade that lost ten pieces produced ten little scatter fans rather than
# the one thing every real downtown-tornado survey documents at a struck
# building's own base: a RUBBLE BERM. `_plans/urban_tornado_research.md`
# §4 ("A CHUNK OUT OF THE SIDE"): a spalled EIFS/cladding panel or a
# tilt-up wall panel falls as a unit near where it broke off (Fort Worth
# 2000's Sweet Shop Factory: 24-ft wall panels "consequently fell" once
# their own roof-truss connection failed — gravity, not sustained wind
# transport, moves THIS material); §5 ("DEBRIS ON THE STREET")'s own
# NEAR-FIELD table (roofing gravel 15 m, a precast curb chunk 20 m, both
# Fort Worth 2000 / Joplin 2011) is the same close-in regime, set against
# its own LOFTED table (roofing sheets 800 m+, "gained 100-150 ft of
# elevation") for the minority that does travel — §5's own words: "do not
# calibrate against the mile-scale numbers; they are a different transport
# mechanism." This section is the missing SPATIAL half of that finding: a
# majority share of every removed FAÇADE piece's fragments (glass excepted
# — §2.9, "glass keeps its own near-façade behaviour" already, via its own
# low `_C_KIND`) now lands in a heap hugging the piece's OWN wall base
# rather than joining the downwind ballistic fan every fragment used to.
#
# `_BERM_SHARE[level]` — level-graded: more of a T4 breach's material never
# leaves the foot of the wall than a T2's does (the wind that could carry
# debris further has itself only just crossed the DOD needed to remove a
# structural chunk at all). T0/T1 remove no façade piece at all
# (`LADDER_T[...]["T1"]` is glass + roof-props only), so no level below T2
# is a key here; `.get(level, _BERM_SHARE["T3"])` is the fallback a future
# level would hit rather than crashing.
_BERM_SHARE = {"T2": 0.55, "T3": 0.65, "T4": 0.75}
# 0.3-4.0 m out from the wall along its own outward normal — §8's own
# numbers, verbatim: a piece's own broken-off-and-fell heap sits hard
# against the wall it came from, an order of magnitude closer than a
# multi-storey release's downwind ballistic reach (tens of metres, capped
# at `REACH_MAX_H * H`).
_BERM_OUT_M = (0.3, 4.0)
# Along-wall spread = the piece's own bay width x 1.4 — wide enough that a
# heap reads as a CONTINUOUS run along a damaged bay, not a single pile
# exactly under one piece's centreline; narrow enough that neighbouring
# bays' own heaps still read as separate concentrations rather than one
# blurred stripe the full length of the side.
_BERM_ALONG_MULT = 1.4
# Height profile: `berm_h = min(_BERM_H_MAX_M, _BERM_H_PER_PIECE_M *
# pieces_lost_on_that_side)` — one wall panel's own heap barely rises
# (0.15 m, a single course of debris); a windward side that lost eight
# pieces piles to the 1.2 m ceiling (knee-to-waist, the practical upper
# bound for a loose masonry/panel heap before it would read as a standing
# wall remnant rather than rubble).
_BERM_H_PER_PIECE_M = 0.15
_BERM_H_MAX_M = 1.2
# Sides a berm can form against — the four main elevations plus the four
# corners (`quake_sliced._CORNER_SIDES`). A `role == "roof"` piece (the
# lowrise `top_storey_loss` exception, §2.6) carries no `_side` at all and
# stays 100% in the downwind ballistic population below — a shed roof
# sheet is exactly the LOFTED/travels-far class per §5, never a wall-base
# heap.
_FACADE_SIDES = frozenset(qs.SIDES) | frozenset(qs._CORNER_SIDES)


def _outward_any(m, sd):
    """`qf._outward(m, sd)` for a main side; for a corner (`"SW"` etc.) the
    NORMALISED SUM of its two adjacent sides' outward normals — the
    diagonal direction a corner's own rubble heap bulges into, matching
    `side_weights`' own corner treatment (mean of the two adjacent sides)
    rather than inventing a second corner-geometry convention. Falls back
    to a fixed direction only for a `sd` that is neither (should not
    happen for a façade-side piece; defensive, not reachable from
    `_ledger_removed`'s own `_FACADE_SIDES` gate)."""
    if sd in qs.SIDES:
        return qf._outward(m, sd)
    sides = qs._CORNER_SIDES.get(sd)
    if not sides:
        return 1.0, 0.0
    sa, sb = sides
    oxa, oya = qf._outward(m, sa)
    oxb, oyb = qf._outward(m, sb)
    ox, oy = oxa + oxb, oya + oyb
    n = math.hypot(ox, oy)
    if n < 1e-9:
        return oxa, oya
    return ox / n, oy / n


def _along_dim(sd, size):
    """The piece's own along-wall dimension — `size[0]` for S/N
    (`tornado_kit._extents`'s `(along, thick, sz)` convention), `size[1]`
    for E/W (the swapped convention on those two sides), the larger of the
    two raw dims for a corner (no along/thick swap exists for a
    corner/roof/other piece — `_extents`'s own docstring, the same
    reasoning `_facade_area_of` already uses for its two-largest-dims
    pick)."""
    sx, sy, _sz = size
    if sd in ("S", "N"):
        return sx
    if sd in ("E", "W"):
        return sy
    return max(sx, sy)


def _ledger_removed(pctx, plan, wind, intensity, region=None):
    """§2.7's deposition model over every REMOVED piece (structural debris:
    panel/block/coping/membrane/metal, fragment COUNT absorbing that piece's own
    volume rather than fragment size — `_frag_thickness_and_count`) plus
    every VOIDED pane (`plan["glass"]`: glass shards, sized by the pane's
    own glazed AREA per §2.7's "1 per 0.35 m2" rule rather than by a volume
    budget -- a pane is a few millimetres thick and "conserve a shard's
    volume against the pane's own paper-thin one" is not a meaningful
    constraint the way it is for a masonry block; area is what the brief's
    own formula is keyed to). Both classes get their own per-building
    thinning pass (`GLASS_SHARDS_MAX_PER_BUILDING` /
    `DEBRIS_MAX_PER_BUILDING`) so one class's cap cannot starve the other.

    ROUND 3: every façade-side removed piece's fragments split BERM
    (`_BERM_SHARE[level]`, heaped against the piece's own wall base —
    `_deposit_berm`, the section above) vs. the REMAINING share, which
    still runs the original downwind ballistic model (`_deposit`,
    unchanged). Every landing point, berm or ballistic, glass or
    structural, is clamped to `region` (`_clamp_to_region` — a no-op when
    `region` is `None`, the default `plan_damage` passes down unless a
    caller or `TU_PLATE_REGION` supplies one).

    Returns a dict — `plan_damage`'s `_finalise` unpacks it into
    `plan["debris"]` and the matching `stats` fields.
    """
    g = pctx["g"]
    rng = pctx["rng"]
    btype = pctx["btype"]
    H = max(1.0, pctx["H"])
    idx = {qs._path(e): e for e in g.els}
    bearing = math.radians(float(wind.get("bearing_deg", 0.0)))
    speed = 25.0 + 70.0 * max(0.0, min(1.0, float(intensity)))
    berm_share_level = _BERM_SHARE.get(str(plan.get("level")),
                                       _BERM_SHARE["T3"])
    struct_frags = []
    glass_frags = []

    def _deposit(kind, mat, size, m, cx, cy, cz, source_path, target,
                source_tex="", source_tex_name="", tone="", shade=0):
        theta = bearing + math.radians(rng.gauss(0.0, 18.0))
        C = _C_KIND.get(kind, 0.2)
        raw_reach = (C * speed * math.sqrt(max(0.0, 2.0 * cz / G_ACCEL))
                    * _lognormal(rng, 0.35))
        cap_frac = REACH_MAX_H_GLASS if kind == "glass" else REACH_MAX_H
        # ROUND 4 (D3): the height-relative cap AND the absolute ceiling --
        # `REACH_MAX_H * H` alone is 64 m on a 42.7 m building and 180 m on
        # a tower, which walks debris off the bench plate (and, in a city,
        # onto blocks the building never touched). `REACH_FLOOR_M` still
        # wins on a very short building, exactly as before.
        reach_cap = max(REACH_FLOOR_M, min(cap_frac * H, REACH_ABS_MAX_M))
        reach = min(raw_reach, reach_cap)
        sigma = 0.12 * reach + 0.04 * cz
        lat = rng.gauss(0.0, max(0.01, sigma))
        bx, by = math.cos(theta), math.sin(theta)
        px_, py_ = -by, bx
        wx = cx + reach * bx + lat * px_
        wy = cy + reach * by + lat * py_
        wx, wy = _push_out_of_footprint(m, wx, wy, bearing)
        wx, wy = _clamp_to_region(wx, wy, region)
        tilt = rng.uniform(5.0, 25.0) if rng.random() < 0.15 else rng.uniform(0.0, 3.0)
        target.append({
            "kind": kind, "size": [float(q) for q in size],
            "x": float(wx), "y": float(wy), "z": 0.0, "z_lift": 0.0,
            "yaw_deg": float(rng.uniform(0.0, 360.0)), "tilt_deg": float(tilt),
            "material": mat, "from": source_path, "stacked": False,
            "tone": str(tone or ""), "shade": int(shade or 0),
            "source_tex": str(source_tex or ""),
            "source_tex_name": str(source_tex_name or "")})

    def _deposit_berm(kind, mat, size, m, sd, cx, cy, bay_w, berm_h,
                      source_path, target, source_tex="", source_tex_name="",
                      tone="", shade=0):
        """The berm's own landing model — position at the wall line under
        the piece's own bay run, `_BERM_OUT_M` out along the outward
        normal, an along-wall offset within `bay_w * _BERM_ALONG_MULT`,
        and a `z_lift` drawn up to `berm_h` (the height PROFILE — see the
        module-level comment above `_BERM_H_PER_PIECE_M` — fragments may
        stack within the heap rather than every one lying at grade).

        `z_lift` IS NOT CONSUMED BY `tornado_urban_usd.build_debris` TODAY
        — verified directly: `build_debris`'s per-fragment loop reads
        `size`/`x`/`y`/`yaw_deg`/`tilt_deg` off each `frag` dict and passes
        a Z it computes ITSELF from `_seat_z(t, w, tilt_deg,
        ground_z=ground_z)`, which never looks at `frag["z"]` (already a
        no-op placeholder, always `0.0`, before this round) or any other
        per-fragment key. So `_seat_z` FLATTENS every fragment to grade
        regardless of what this function draws — stacking does not
        compose with face-seating YET. `z_lift` is authored into the plan
        anyway (an ADDED field, `"z"` itself untouched, per the round's own
        rule: coordinate through the plan schema, never rename) so a
        future `build_debris` pass can add it into its own `_seat_z` call
        (`ground_z=ground_z + frag.get("z_lift", 0.0)`) without another
        planner change — this function's job stops at authoring the
        number correctly; wiring the builder to honour it is
        `tornado_urban_usd.py`'s (another stream owns that file this
        round). `stacked` is `True` whenever the draw is not exactly
        `0.0` — in practice every berm fragment (a continuous
        `Uniform(0, berm_h)` draw has a measure-zero chance of landing on
        `0.0` exactly) and no ballistic fragment (`_deposit` above hard-
        codes `"z_lift": 0.0`), so the flag doubles as an unambiguous
        berm-membership marker for a consumer that only wants the flag.
        """
        ox, oy = _outward_any(m, sd)
        alx, aly = -oy, ox
        out_dist = rng.uniform(*_BERM_OUT_M)
        spread = max(0.2, float(bay_w)) * _BERM_ALONG_MULT
        along_off = rng.uniform(-spread / 2.0, spread / 2.0)
        wx = cx + ox * out_dist + alx * along_off
        wy = cy + oy * out_dist + aly * along_off
        # SAFETY NET, corners only in practice: a main side's outward
        # normal is exactly one local axis, so `out_dist` alone always
        # clears that axis's bound (a piece's own centroid sits within
        # half its own thickness of the wall line) regardless of
        # `along_off`. A corner's `_outward_any` direction is diagonal, so
        # a small `out_dist` draw CAN still leave the point inside the
        # OBB on a deep corner piece — `_exit_t`/`_dir_to_local` (the same
        # exit-the-box math `_push_out_of_footprint` uses, along THIS
        # point's own outward direction rather than the wind bearing) push
        # it the rest of the way out, +0.3 m past the wall line, same as
        # this function's own `_BERM_OUT_M` floor.
        lx, ly = qf._to_local(m, wx, wy)
        w2, d2 = m["W"] / 2.0, m["D"] / 2.0
        if abs(lx) <= w2 and abs(ly) <= d2:
            dlx, dly = _dir_to_local(m, ox, oy)
            t = _exit_t(lx, ly, dlx, dly, w2, d2)
            lx, ly = lx + (t + 0.3) * dlx, ly + (t + 0.3) * dly
            wx, wy = qf._to_world(m, lx, ly)
        wx, wy = _clamp_to_region(wx, wy, region)
        # ROUND 4 (D3) — A HEAP IS A WEDGE, AND ITS MASS SITS LOW.
        # `rng.uniform(0.0, berm_h)` put as many fragments in the top third
        # of a 1.2 m heap as in the bottom third and put them the same
        # height up 4 m OUT from the wall as hard against it -- a fragment
        # lifted 1.0 m at the toe of the heap has nothing under it and is a
        # floater, which is half of what "floating clusters" reads as. Two
        # shape terms, ONE rng draw (same stream cost as the `uniform` it
        # replaces, so a seed's plan advances identically): a linear WEDGE
        # from full height at the wall to 0.15 at the outer edge of
        # `_BERM_OUT_M`, and an exponent that biases the draw low (median
        # 0.5 ** 1.6 = 0.33 of the local ceiling, not 0.5). Still strictly
        # inside `(0, berm_h]`, which is what the plan's own contract and
        # `stacked` depend on.
        out_lo, out_hi = _BERM_OUT_M
        wedge = 1.0 - 0.85 * ((out_dist - out_lo) / max(1e-6, out_hi - out_lo))
        wedge = min(1.0, max(0.15, wedge))
        z_lift = max(0.0, berm_h) * wedge * (rng.random() ** 1.6)
        tilt = rng.uniform(5.0, 25.0) if rng.random() < 0.15 else rng.uniform(0.0, 3.0)
        target.append({
            "kind": kind, "size": [float(q) for q in size],
            "x": float(wx), "y": float(wy), "z": 0.0, "z_lift": float(z_lift),
            "yaw_deg": float(rng.uniform(0.0, 360.0)), "tilt_deg": float(tilt),
            "material": mat, "from": source_path,
            "stacked": bool(z_lift > 0.0),
            "tone": str(tone or ""), "shade": int(shade or 0),
            "source_tex": str(source_tex or ""),
            "source_tex_name": str(source_tex_name or "")})

    source_volume = 0.0
    removed_by_side = {}
    for p in sorted(plan["removed"]):
        e = idx.get(p)
        if e is None:
            continue
        sd0 = (e.get("p") or {}).get("_side")
        if sd0 in _FACADE_SIDES:
            removed_by_side[sd0] = removed_by_side.get(sd0, 0) + 1

    # ROUND 4 (v6 review): the building's own masonry TONE, resolved once
    # per plan from its style (`_KIT_TONE`/`_tone_for`) and stamped on every
    # BLOCKY fragment. `""` for every style the table does not name -- which
    # is every sliced (A-row) building, whose class-branch look the review
    # approved and which must not move.
    tone = _tone_for(plan.get("style"))
    # Sliced core strips and trim subsets often carry WallBack/beam atlases.
    # They are structural pieces, but those pixels are not the exterior wall
    # stock. Find real exterior donors once and borrow the closest course for
    # any removed piece whose own stamp is not a usable facade surface.
    facade_donors = []
    want = (("brick", "stone", "stucco") if btype == "urm" else
            ("concrete",))
    for de in g.els:
        dp = de.get("p") or {}
        du = str(dp.get("_tex_url") or "")
        dn = str(dp.get("_tex_name") or "")
        words = (du + " " + dn).lower()
        if (dp.get("_side") in _FACADE_SIDES and du and
                any(q in words for q in want) and
                not any(q in words for q in ("wallback", "beam_trim"))):
            facade_donors.append(de)

    def _facade_source(e, pp):
        url = str(pp.get("_tex_url") or "")
        name = str(pp.get("_tex_name") or "")
        words = (url + " " + name).lower()
        if (url and any(q in words for q in want) and
                not any(q in words for q in ("wallback", "beam_trim"))):
            return url, name
        if not facade_donors:
            return url, name
        sd = pp.get("_side")
        st = int(pp.get("_storey", e.get("storey", 0)))
        donor = min(facade_donors, key=lambda d: (
            0 if (d.get("p") or {}).get("_side") == sd else 1,
            abs(int((d.get("p") or {}).get("_storey",
                                           d.get("storey", 0))) - st)))
        dp = donor.get("p") or {}
        return str(dp.get("_tex_url") or ""), str(dp.get("_tex_name") or "")

    n_berm_total = 0
    for p in sorted(plan["removed"]):
        e = idx.get(p)
        if e is None:
            continue
        pp = e.get("p") or {}
        kind = _kind_of(pp, btype)
        m = g.mass_of(e)
        size = qs._size(e)
        sx, sy, sz = size
        source_volume += sx * sy * sz
        cx, cy = e["x"], e["y"]
        cz = e["z"] + sz / 2.0
        fill = rng.uniform(*_FILL)
        thick, n = _frag_thickness_and_count(kind, size, fill)
        mat = _material_hint(pp, kind, btype)
        # ROUND 3b (§8e F3): the piece's own resolved cladding texture
        # (`annotate_surface`'s stamp), gated to FAÇADE kinds only — see
        # `_FACADE_TEX_KINDS`'s own comment. `""`/`""` (never `None`, so a
        # downstream `.get("source_tex")` always reads a string) for a roof
        # kind, an untextured piece, or a piece annotate_surface never saw.
        if kind in _FACADE_TEX_KINDS:
            tex_url, tex_name = _facade_source(e, pp)
        else:
            tex_url = ""
            tex_name = ""
        sd = pp.get("_side")
        is_facade = sd in _FACADE_SIDES
        n_berm = 0
        bay_w = berm_h = 0.0
        if is_facade:
            n_berm = max(0, min(n, int(round(n * berm_share_level))))
            bay_w = _along_dim(sd, size)
            berm_h = min(_BERM_H_MAX_M,
                        _BERM_H_PER_PIECE_M * removed_by_side.get(sd, 1))
        for i in range(n):
            frag_size = _dims_for(kind, rng, thick)
            # INTERLEAVED, not grouped: `_thin_fragments`'s per-source
            # round-robin keeps a source's fragments in COLUMN order (its
            # own index-0 first, then index-1 of every source, ...), so a
            # piece whose fragments were [berm, berm, ..., ballistic,
            # ballistic, ...] would have its berm share survive thinning
            # at a MUCH higher rate than `berm_share_level` (every low
            # column is berm) -- measured, not assumed: a grouped ordering
            # on a T4/tower fixture with `DEBRIS_MAX_PER_BUILDING` binding
            # pushed the post-thin berm fraction to 0.94 against a target
            # of 0.75. Spacing the `n_berm` berm slots evenly across the
            # piece's own `n` fragments (the same "distribute k marks over
            # n slots" rule a Bresenham line uses) keeps EVERY column-
            # prefix of a piece's own list close to the SAME berm
            # fraction, so `n_berm / n_struct_debris` still tracks
            # `_BERM_SHARE[level]` even when the per-building cap binds.
            is_berm = (n_berm > 0
                      and ((i + 1) * n_berm) // n != (i * n_berm) // n)
            # The per-mesh tone JITTER band this fragment belongs to.
            # DETERMINISTIC on (source piece, fragment index) rather than a
            # draw, so it costs no rng and a re-plan of the same seed puts
            # every fragment in the same band; `_thin_fragments`' own
            # round-robin then keeps the three bands evenly represented
            # after thinning instead of favouring one.
            shade = (i + _stable_shade(p)) % max(1, _DEBRIS_SHADES) \
                if kind in _BLOCKY_KINDS else 0
            if is_berm:
                _deposit_berm(kind, mat, frag_size, m, sd, cx, cy, bay_w,
                              berm_h, p, struct_frags,
                              source_tex=tex_url, source_tex_name=tex_name,
                              tone=(tone if kind in _BLOCKY_KINDS else ""),
                              shade=shade)
                n_berm_total += 1
            else:
                _deposit(kind, mat, frag_size, m, cx, cy, cz, p, struct_frags,
                         source_tex=tex_url, source_tex_name=tex_name,
                         tone=(tone if kind in _BLOCKY_KINDS else ""),
                         shade=shade)

    glass_measured = pctx.get("glass_measured", False)
    for p in sorted(plan["glass"]):
        e = idx.get(p)
        if e is None:
            continue
        pp = e.get("p") or {}
        sd = pp.get("_side")
        sx, sy, sz = qs._size(e)
        area = (sx * sz) if sd in ("S", "N") else (sy * sz)
        # MEASURED: a piece's actual glazed area is `_glass_frac` of its own
        # face -- a pier that is 30% glass sheds 30% of a full pane's
        # shards, not a full pane's worth off its whole (mostly opaque)
        # face. FALLBACK (no `_glass_faces` anywhere on this grid): the
        # piece's full face, unchanged from round 1.
        if glass_measured:
            gf = max(0.0, min(1.0, float(pp.get("_glass_frac") or 0.0)))
            area *= gf
        n = int(min(60, max(1, round(area / 0.35))))
        m = g.mass_of(e)
        cx, cy = e["x"], e["y"]
        cz = e["z"] + sz / 2.0
        for _i in range(n):
            size = [rng.uniform(0.15, 0.7), rng.uniform(0.15, 0.7),
                   rng.uniform(0.006, 0.014)]
            _deposit("glass", "glass", size, m, cx, cy, cz, p, glass_frags)

    kept_struct, n_struct_before, struct_thinned = _thin_fragments(
        struct_frags, DEBRIS_MAX_PER_BUILDING)
    kept_glass, n_glass_before, glass_thinned = _thin_fragments(
        glass_frags, GLASS_SHARDS_MAX_PER_BUILDING)
    # `"stacked"` doubles as the berm-membership flag (see `_deposit_berm`'s
    # own docstring) -- a ballistic fragment always carries `"z_lift": 0.0`
    # exactly, a berm fragment's continuous draw is >0.0 with probability 1
    # in practice, so counting `stacked` fragments post-thin is an exact
    # count of how many of the SURVIVING fragments are berm, not just of
    # how many were authored (`n_berm_total`, pre-thin).
    n_berm_kept = sum(1 for f in kept_struct if f.get("stacked"))
    return {
        "frags": kept_struct + kept_glass,
        "source_volume_m3": source_volume,
        "n_struct": len(kept_struct), "n_struct_before": n_struct_before,
        "struct_thinned": struct_thinned,
        "n_glass_shards": len(kept_glass), "n_glass_shards_before": n_glass_before,
        "glass_shards_thinned": glass_thinned,
        "n_berm": n_berm_total, "n_berm_kept": n_berm_kept,
        "berm_share_level": berm_share_level,
    }


# ---------------------------------------------------------------------------
# THE PLAN
# ---------------------------------------------------------------------------
def plan_damage(info, elements, level, btype, rng, wind, height_class=None,
                intensity=None, region=None):
    """Everything a tornado does to one building's envelope, decided with NO
    USD access at all. See the module docstring for the schema. Signature
    matches `tools/tornado_urban_probe.py`'s own call
    (`plan_damage(info, elements, LEVEL, btype, rng, wind,
    height_class=..., intensity=...)`) unchanged — `region` is a new
    KEYWORD-only addition at the end, so every existing positional/keyword
    call site (this module's own `_run_one_recipe`/`_plan` test helpers
    included) still works with no edit.

    `region=(x0, y0, x1, y1)` — the plate's own world-metre bounds, ROUND 3
    §8 R8's "the floating-off-plate defect": every fragment this building's
    ledger authors clamps to `region` shrunk by `_REGION_MARGIN_M`
    (`_ledger_removed` -> `_clamp_to_region`). `region=None` (the default)
    falls back to `TU_PLATE_REGION` (module-level, read once from the
    `TU_PLATE_REGION` env var at import time as `"x0,y0,x1,y1"`) so
    `wreck_urban`/`wreck_kit` (another stream's files this round) can set
    the env var once, before this module is imported, and every
    `plan_damage` call in the process picks up the SAME region with no
    call-site edit at all; a caller who DOES pass `region` explicitly
    always wins over the env fallback.
    """
    wind = {"bearing_deg": float((wind or {}).get("bearing_deg", 0.0)),
            "speed_frac": float((wind or {}).get("speed_frac", 0.0)),
            "cross_frac": float((wind or {}).get("cross_frac", 0.0)),
            "over": bool((wind or {}).get("over", False))}
    if height_class is None:
        height_class = height_class_for(info.get("H"))
    if intensity is None:
        intensity = (wind or {}).get("intensity", wind.get("speed_frac", 0.0))
    intensity = float(intensity)
    if region is None:
        # ROUND 4 (D3): the module-level value first (a test monkeypatches
        # THAT, and it is what an import-time env set produces), then a LIVE
        # re-read of the env. Round 3 read the env once at import and nothing
        # else, which made the whole clamp unreachable for the two callers it
        # was written for: `wreck_urban`/`wreck_kit` import this module at
        # module-import time, long before a launcher knows its plate, and a
        # BENCH needs a DIFFERENT region per cell anyway (a plan is authored
        # in the cell's own local frame -- `wreck_urban` calls `describe` at
        # x=y=yaw=0 -- so the plate region for a cell at (30, 70) is
        # `PLATE - (30, 70)`, not `PLATE`). Reading per call lets a caller
        # set `TU_PLATE_REGION` immediately before each cell. Nothing in the
        # tree sets it yet (grepped) -- `REACH_ABS_MAX_M` is the half that
        # bounds the field with no caller at all.
        region = TU_PLATE_REGION if TU_PLATE_REGION is not None \
            else _region_from_env()

    recs_raw = list(LADDER_T.get(btype, LADDER_T["urm"]).get(level, []))
    recs, guard_notes = _guard(recs_raw, btype, info, height_class)
    for name, _kw in recs:
        if name not in RECIPES_T:
            raise KeyError("unknown urban tornado recipe {0!r}".format(name))

    plan = {
        "schema": "tornado_urban_plan.v1",
        "level": str(level), "btype": str(btype), "style": info.get("style"),
        "H": float(info.get("H") or 0.0), "height_class": height_class,
        "wind": wind, "recipes": [[n, dict(kw or {})] for n, kw in recs],
        "removed": [], "displaced": {}, "glass": [], "glass_bands": [],
        "macroblocks": [], "regions": [], "roof_props": "keep", "debris": [],
        "notes": list(guard_notes), "stats": {},
        # ROUND 3b (§8e F2a, stream FX1) -- `panels` is ADDED PURELY so
        # `quake_sliced._plan_tears`/`_author_tears` (reused unchanged, see
        # `_finalise` below) can read `plan["panels"]` unconditionally
        # without a `KeyError` -- this ladder never lays a piece on a pile
        # the way `quake_sliced`'s own `_pile` mechanism does, so the list
        # is always empty; `roof_shed` (bool, default False) and `tears`/
        # `tear_scope` (the tear job list / the (mass -> side/storey) scope
        # `_plan_tears` itself populates) are the other two additions this
        # round makes to the schema -- see `_shed_unsupported_roof` and the
        # module docstring's schema section.
        "panels": [], "roof_shed": False, "tears": [], "tear_scope": {},
        "_removed_set": set(),
    }
    g = qs._Grid(info, elements)
    weights = side_weights(info, wind, rng)
    plan["side_weights"] = {k: float(v) for k, v in weights.items()}
    pctx = _pctx(info, elements, btype, rng, plan, wind, weights, height_class,
                intensity)
    for name, kw in recs:
        RECIPES_T[name](pctx, **(kw or {}))
    return _finalise(pctx, plan, height_class, wind, intensity, region=region)


def _finalise(pctx, plan, height_class, wind, intensity, region=None):
    g = pctx["g"]
    _enforce_ground_floor_rule(pctx, plan, height_class)
    _cap_removed_frac(pctx, plan, height_class)
    _enforce_no_empty_storey(pctx, plan)

    # ROUND 3b F2a (§8e) -- ragged tears, T3+ only. Called HERE (after every
    # cap/restore pass above has had its say), NOT at `quake_sliced.plan_
    # damage`'s own call site (right after its recipe loop, before its
    # `_finalise`): THIS module's `_finalise` can un-remove pieces
    # (`_cap_removed_frac`, `_enforce_no_empty_storey`), which
    # `quake_sliced`'s own `_finalise` never does, so planning a tear
    # against the removed-set BEFORE those restores could ragged a wall
    # that ends up whole again. Deliberately BEFORE `_shed_unsupported_
    # roof` below (F1): a shed roof/parapet piece's own `el_span` --
    # especially a SLICED building's single deck, which spans nearly the
    # whole footprint -- would plan spurious "left"/"right"/"below" tears
    # against wall bays that are merely BELOW the missing roof, not beside
    # a hole (see `_cap_tears`'s own note for the other reason this is not
    # `quake_sliced._plan_tears` called with no adaptation at all).
    # ROUND 4 (D2, v6 review) -- THE WALL SUPPORT PASS, before the tears so
    # the holes it opens get torn edges like any other hole, and after every
    # cap/restore pass above (it interleaves with `_cap_removed_frac` itself
    # -- see its own docstring for why that terminates).
    n_support_shed = _shed_unsupported_walls(pctx, plan, height_class)

    raw_tears = []
    border = set()
    n_border_no_job = 0
    if str(plan.get("level")) in ("T3", "T4"):
        border = _hole_border_paths(
            g, plan["_removed_set"],
            moved=set(plan["displaced"]) | {q for q, _s in plan["panels"]},
            masses=pctx["info"].get("masses"))
        # `TU_TEAR_BORDER=0` is the round-3 state end to end -- the flat
        # `QS_MAX_TEARS` planning budget, no corner pass, and a cap that can
        # drop a border job -- so the container probe's A/B measures the
        # whole tear-coverage change, not just its last step.
        raw_tears = (_plan_tears_wide(pctx, plan) if TU_TEAR_BORDER_ON
                     else qs._plan_tears(pctx, plan))
        if TU_TEAR_BORDER_ON:
            # ...then the CORNER pieces `plan_edges` structurally cannot
            # reach (see `_corner_tear_jobs`): every hole on a sliced
            # building has two of them and they were the biggest untorn
            # share in the audit.
            raw_tears += _corner_tear_jobs(
                pctx, plan, {qs._path(j.get("el") or {}) for j in raw_tears})
            raw_tears = _fill_missing_border_tears(
                pctx, plan, border, raw_tears)
        raw_tears = _cap_tears(raw_tears, TU_MAX_TEARS, border=border)
    n_tears = sum(1 for j in raw_tears if not j.get("dropped"))
    n_tears_dropped = sum(1 for j in raw_tears if j.get("dropped"))
    torn = {qs._path(j.get("el")) for j in raw_tears if not j.get("dropped")}
    has_job = {qs._path(j.get("el")) for j in raw_tears}
    # TWO DIFFERENT FAILURES, and only one of them is this module's.
    #   `n_border_untorn` — a hole-border piece `fire_collapse.plan_edges`
    #       DID emit a job for, that something here then dropped. That is
    #       the cap, it is where the user's "even breaks" came from, and
    #       `_cap_tears`'s border carve-out is why it is 0.
    #   `n_border_no_job` — `plan_edges` emitted nothing for the piece at
    #       all. Its `left`/`right`/`below`/`above` tests key on
    #       `describe`'s own `e["storey"]` while this grid keys on the
    #       placement's `_storey`, and the two do not agree piece-for-piece
    #       on every asset (the parapet band is the documented case), so a
    #       few genuine edge neighbours are invisible to it. NOT fixable
    #       from this module (`plan_edges` is `fire_collapse`'s), so it is
    #       counted and reported rather than hidden.
    n_border_no_job = len(border - has_job)
    n_border_untorn = len((border & has_job) - torn)
    if n_border_no_job:
        _note(pctx, "tears: {0} of {1} hole-border piece(s) got no tear job "
                    "at all (fire_collapse.plan_edges emitted none -- its "
                    "class tests key on describe's storey, this grid on "
                    "_storey)".format(n_border_no_job, len(border)))
    if n_border_untorn:
        _note(pctx, "tears: {0} hole-border piece(s) had a job and lost it "
                    "-- the cap must never drop a border job".format(
                        n_border_untorn))
    plan["tears"] = qs._tears_to_json(raw_tears)

    # ROUND 3b F1 (§8e) -- the roof/parapet support post-pass. AFTER tears
    # (see above) so its own removals never seed a tear job; still counts
    # as "after every cap" per the plan brief, since the caps above never
    # touch `role in (roof, parapet, parapet_corner)` in the first place.
    n_roof_shed, n_parapet_shed = _shed_unsupported_roof(pctx, plan)

    ledger = _ledger_removed(pctx, plan, wind, intensity, region=region)
    plan["debris"] = ledger["frags"]
    plan.pop("_removed_set", None)

    removed = sorted(set(plan["removed"]))
    plan["removed"] = removed
    plan["glass"] = sorted(set(plan["glass"]))
    idx = {qs._path(e): e for e in g.els}
    per_side = {}
    max_storey = -1
    removed_area = 0.0
    for p in removed:
        e = idx.get(p)
        if not e:
            continue
        pp = e.get("p") or {}
        st = int(pp.get("_storey", 0))
        max_storey = max(max_storey, st)
        sd = pp.get("_side")
        if sd in qs.SIDES:
            per_side[sd] = per_side.get(sd, 0) + 1
            removed_area += _facade_area_of(e)
    total_area = _total_facade_area(g)
    debris_volume = sum(f["size"][0] * f["size"][1] * f["size"][2]
                        for f in plan["debris"])
    n_debris_thinned = ledger["struct_thinned"] or ledger["glass_shards_thinned"]
    plan["stats"] = {
        "n_pieces": len(g.els), "n_removed": len(removed),
        # BY FAÇADE AREA — the metric `_cap_removed_frac` enforces (a
        # coarse real slice's few, huge pieces make a piece-count fraction
        # meaningless: see `_facade_area_of`'s own docstring).
        "removed_frac": float(removed_area / total_area),
        "removed_count_frac": (len(removed) / float(len(g.els)) if g.els else 0.0),
        "n_glass": len(plan["glass"]), "n_displaced": len(plan["displaced"]),
        "n_debris": len(plan["debris"]),
        "debris_volume_m3": float(debris_volume),
        "source_volume_m3": float(ledger["source_volume_m3"]),
        "max_removed_storey": int(max_storey) if max_storey >= 0 else None,
        "removed_by_side": dict(sorted(per_side.items())),
        "n_glass_shards": int(ledger["n_glass_shards"]),
        "glass_shards_thinned": bool(ledger["glass_shards_thinned"]),
        "n_struct_debris": int(ledger["n_struct"]),
        "struct_debris_thinned": bool(ledger["struct_thinned"]),
        "debris_thinned": bool(n_debris_thinned),
        "n_glass_candidates": int(pctx.get("n_glass_candidates", 0)),
        "glass_measured": bool(pctx.get("glass_measured", False)),
        # ROUND 3 §8 R8 — the berm share (see `_ledger_removed`'s own
        # docstring): `n_berm`/`n_berm_kept` are STRUCTURAL-fragment counts
        # only (glass never berms, §2.9), before/after the per-building
        # thinning pass; `berm_share_level` is the `_BERM_SHARE[level]`
        # value this plan actually used (so a test or a review print does
        # not need to re-derive it from `plan["level"]`).
        "n_berm": int(ledger["n_berm"]), "n_berm_kept": int(ledger["n_berm_kept"]),
        "berm_share_level": float(ledger["berm_share_level"]),
        "region": list(region) if region else None,
        # ROUND 3b (§8e F1/F2a) additions -- see `_shed_unsupported_roof`
        # and the tear block above `_ledger_removed`'s call.
        "n_tears": int(n_tears), "n_tears_dropped": int(n_tears_dropped),
        "n_roof_shed": int(n_roof_shed), "n_parapet_shed": int(n_parapet_shed),
        # ROUND 4 (D2, v6 review) -- the two invariants the user's note
        # turned into numbers: pieces the SUPPORT rule took because they
        # stood on air, and hole-border pieces still carrying a square
        # break. Both must read 0 for `n_unsupported` on a re-audit (see
        # `_unsupported_survivors`) and 0 here.
        "n_support_shed": int(n_support_shed),
        "n_hole_border": int(len(border)),
        "n_border_untorn": int(n_border_untorn),
        "n_border_no_job": int(n_border_no_job),
    }
    if ledger["glass_shards_thinned"]:
        plan["notes"].append(
            "glass shards thinned from {0} to {1} (cap {2}, one guaranteed "
            "per broken pane)".format(ledger["n_glass_shards_before"],
                                      ledger["n_glass_shards"],
                                      GLASS_SHARDS_MAX_PER_BUILDING))
    if ledger["struct_thinned"]:
        plan["notes"].append(
            "structural debris thinned from {0} to {1} (cap {2}, one "
            "guaranteed per removed piece)".format(
                ledger["n_struct_before"], ledger["n_struct"],
                DEBRIS_MAX_PER_BUILDING))
    return plan
