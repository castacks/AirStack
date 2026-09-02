"""tornado_roof — the ROOF DAMAGE SYSTEM for the urban tornado ladder
(`_plans/urban_tornado_plan.md` §8, ROUND 3, stream RF).

WHY THIS EXISTS
----------------
Round 2's bench review (the lead, after looking at the 500 m GUI scene):
"right composition and track, wrong read ... no roof damage, removals read
as cut-outs." The aerial signature of an urban tornado is ROOFS first, not
façades — an overhead camera at 60-90 m sees roof PLANES, not wall
elevations, and `disaster/tornado_urban.py`'s own ladder (T0..T4) never
touches a roof at all except the one named exception, `t_top_storey_loss`
(lowrise urm, i >= 0.85, which sheds the whole roof PIECE as debris). Every
other building in a scene built from that ladder alone keeps a pristine
roof regardless of level — exactly the defect the user's own review named.

The research backs the priority. The EF-scale's own damage-of-degree
tables (`_plans/urban_tornado_research.md` §1, WISE 2006/pdftotext-verified
against the primary EFScale.pdf) put **roof-covering loss as the SECOND
rung on every mid/high-rise building type's own ladder** — DI 18 (mid-rise)
DOD 2 "loss of roof covering (<20%)" at EXP 83 mph, DI 19 (high-rise) DOD 2
at 86 mph, both BELOW the first curtain-wall glazing DOD (DI 19 DOD 4,
101 mph) and below any parapet/coping DOD. Roof covering is the FIRST
thing an EF scale assessor looks for after "threshold of visible damage" —
it should be the FIRST thing this codebase's own ladder shows, and until
this module it showed nothing. The hurricane research's flat-roof section
(`_plans/hurricane_research.md` §2.2, FEMA P-2181 FS 3.3.2) gives the
MECHANISM this module renders: "the initiating failure is the PERIMETER
METAL, not the field of the roof" — gravel stop / coping lifts at a corner
or windward edge, the membrane peels back progressively from THAT edge (the
rolled-back-carpet look), insulation boards blow off exposing the deck.
§2.3 gives the second signature: rooftop mechanical equipment loses access
panels/casing first (~85-100 mph), slides and separates from its curb next
(~100-120), is fully ejected only above that — "displaced rooftop equipment
can puncture and tear roof membranes," so a unit that moved leaves a torn
gash and a dragged trail, not a clean gap.

WHAT THIS MODULE IS, AND IS NOT
----------------------------------
This is a SECOND damage pass over the SAME element table
`tornado_urban.plan_damage` (the façade ladder) already walked, producing
its OWN plan (`tornado_roof_plan.v1`) that a SEPARATE apply step consumes.
It is not a recipe inside `tornado_urban.LADDER_T` and does not touch that
module's `RECIPES_T` table, for the same reason the plan brief keeps it a
separate stream (RF) with separate files: the façade ladder's invariant —
same seed, byte-identical plan — must hold with or without this module
ever being called, and the cleanest way to guarantee that is to give this
pass its OWN rng stream that the façade planner never sees or advances
(see `roof_seed`/`ROOF_SEED_XOR` below).

It authors GENERIC roof furniture, not real placed prims. `tornado_urban_
usd.apply_plan`'s own "ROOF PROPS" step already has a `plan["roof_props"]
== "sweep"` branch that looks for `ctx["roof_plant"]`/`ctx["roof_fixed"]`
(paths `quake_flow.dress_roof` would have placed) — and that function is
never called for a tornado building (`wreck_urban`'s own docstring: "This
function does NOT call `quake_flow.dress_roof`... roof-furniture PLACEMENT
for the urban-tornado ladder is not yet decided (a later round's job)"),
so those two ctx keys are simply absent and the existing step is a
documented no-op. This module does not wire in `dress_roof` (an
earthquake-shaped placement needing `mats["tank_wood"]`/`mats[
"plant_metal"]` this ladder's `mats` dict is not guaranteed to carry — the
skill's own "known gaps" #5). Instead `_draw_props` synthesises its OWN
small anchor points across the roof plane (count ~ roof area / 350 m2, the
FEMA record's own "roughly one unit per few hundred square metres" rooftop
plant density) and represents them as plain tinted boxes — toppled in
place, or swept with 1-3 authored lying on their side downwind. When a
later round wires real `dress_roof`-shaped placement into the tornado
ladder, this synthetic layer is the thing to retire in favour of moving
REAL prop prims.

THE ROOF PLANE. `info["masses"]["main"]` (`m`) gives the building's own
footprint box: `m["W"] x m["D"]` centred at `(m["cx"], m["cy"])`, yawed by
`m["yaw"]`, its top at `m["top"]` — exactly the box `quake_flow._mass_specs`
builds and every façade recipe already reads. The roof PLANE this module
damages is that box inset by `ROOF_INSET_M` (0.4 m, an ordinary parapet/
coping thickness) on every side — the coping strip itself lives on the
OUTER 0.4 m this inset removes, consistent with "coping lifts first, then
the membrane peels back from where the coping used to be."

THE SEPARATE RNG STREAM. The brief's own suggestion — "a SEPARATE seeded
rng stream: `random.Random(rec_seed ^ 0xR00F)`-style" — assumes an integer
per-building seed is sitting at the hook call site. It is not: `wreck_urban`/
`wreck_kit` carry `rng` and `nrng` as already-constructed generators (a
`random.Random` and a `numpy.random.Generator` respectively — `nrng`'s API
does not match this module's `random.Random`-shaped draws, and consuming
either one to derive a roof seed would perturb the exact state sequence the
façade planner is entitled to expect unchanged) plus a `tag` STRING. `
roof_seed(tag)` below hashes that string with `zlib.crc32` (stable across
processes, unlike Python's own randomized `hash()`) and XORs it with
`ROOF_SEED_XOR` — deterministic given `tag`, and constructed from
information the hook already has without touching `rng`'s or `nrng`'s own
state at all. `tests/test_tornado_roof.py`'s own façade-plan-unchanged test
asserts this holds: a façade plan built with `rng = random.Random(seed)`
and NO call to this module is byte-identical, key for key, to one built
with the identical fresh `random.Random(seed)` immediately followed by a
`plan_roof(..., rng=random.Random(roof_seed(tag)))` call on a wholly
separate `random.Random` object.

THE PLAN SCHEMA (`tornado_roof_plan.v2`, JSON-serialisable, plain floats,
sorted collections where order would otherwise vary run to run):

    schema, level, btype, height_class, H
    wind            the same {bearing_deg, speed_frac, cross_frac, over}
                    dict the façade plan carries
    roof            {cx, cy, yaw, W, D, top, inset, rect_local}
    windward_side   "S"/"E"/"N"/"W" -- the single side every element below
                    is anchored against (§2.5's own ranking, drawn with
                    THIS module's own rng, never the façade's)
    side_weights    the same §2.5 weights (recomputed on this module's own
                    rng draw when `wind["over"]`; identical arithmetic
                    either way)
    skipped         bool -- True when the façade plan already shed the roof
                    piece (`top_storey_loss`) or the building carries no
                    usable mass box
    skip_reason     str or None
    patches         [] — ALWAYS EMPTY, at every level, with no knob to
                    refill it. The membrane-PEEL population is RETIRED
                    (§ "ROUND 4" below); the key survives only so a reader
                    written against v1 (`tools/tornado_roof_probe.py`
                    iterates it) parses a v2 plan without an edit.
    tear            {side, along_start_m, run_m, depth_m, p0_local,
                     p1_local} or a zero stub — WHERE the membrane let go:
                    the release line the sheets came off. RECORD ONLY,
                    `apply_roof` authors no geometry from it.
    sheets          [frag, ...]  the torn membrane/deck SHEETS the tear
                    threw downwind (§ "SHED MEMBRANE SHEETS" below)
    scour           {side, frac, rect_local, z, tint} or a zero-frac stub
    coping          {side, target_frac, already_removed_frac, tone,
                     piece_removed: [prim_path,...], boxes: [frag,...]}
                    — `tone` is the building's masonry token
                    (`tornado_urban._tone_for`, `""` for a style the table
                    does not name), stamped onto every fallen-coping frag
                    so `build_debris` routes it to the same pale-stone /
                    tan rubble look the façade ladder's own coping takes
                    instead of the generic red-brown brick class map
    props           {action, n_units, topple_frac, sweep_frac, n_topple,
                     n_thrown, topple: [frag,...], thrown: [frag,...]}
    notes           [str, ...]
    stats           {roof_area_m2, n_sheets, n_coping_pieces,
                     n_coping_boxes, n_topple, n_thrown_props, scour_frac}
                    — `n_patches`/`patch_coverage_frac` are GONE, not
                    zeroed: nothing peels any more, and reporting a peel
                    coverage of 0.0 would still be advertising a feature
                    this module no longer has.

A `frag` dict (the plan's own `sheets`, coping's `boxes`, props'
`topple`/`thrown`) is shaped exactly like a `tornado_urban.plan_damage` debris
record — `{kind, material, size:[l,w,t], x, y, z, yaw_deg, tilt_deg}` in
WORLD (cell-local, per this codebase's own convention — `wreck_urban`/
`wreck_kit` always call `quake_flow.describe` at the cell origin, so "world"
and "cell-local" coincide) — so `apply_roof` hands every one of them
straight to `tornado_urban_usd.build_debris`, the SAME merged-mesh-per-
class, `_seat_z`-bedded, `faceVarying`-normalled machinery the façade
ladder's own street debris already uses. That machinery takes ONE
`ground_z` per call, but a roof fragment can legitimately land on either
the roof PLANE or the street below it (a torn sheet blown clean off the
edge) — `_author_fragments_grouped` partitions by each fragment's own `z`
and calls `build_debris` once per distinct baseline, tagging `kind` with the
baseline so the two groups never collide on one merged-mesh prim path (the
MATERIAL bucket — the only thing that decides the LOOK — is keyed off
`material` alone, so both groups still share one look).

ROUND 4: THE MEMBRANE PEEL IS RETIRED. The peel patches this module
shipped in round 3 — one EXPOSED-SUBSTRATE quad per patch plus a curled
membrane LIP prism along its downwind boundary — are GONE, at every level,
with no env knob to bring them back. The user's own call on the round-3
bench, looking at `/World/tornado_bench/B3/cell/tornado_roof/
patch_02_substrate`: "idk what this is but it looks weird and unnatural
... I don't think we can make it look good in isaac-sim — don't do it."

They were right about the read, and the read was not a tint bug. A patch
was authored as ONE axis-aligned rectangle covering half a roof (the
probe's own two T3 buildings measured 0.61 and 0.53 of the inset roof
area, in 2 and 3 patches), sitting 0.015 m PROUD of the roof plane —
enough for the renderer to throw a hard drop shadow off two dead-straight
edges — and, since round 4's `_roof_material` fix (`diffuse_tint` is the
multiply slot over a bound map; `diffuse_color_constant` is only the
map-failed fallback), drawn BRIGHTER than the membrane field around it.
A real peeled flat roof reads the opposite way on every count: DARKER than
the surviving membrane, irregular in outline (wind tears membrane in
tongues off the windward edge, not in rectangles), FLUSH or slightly
recessed, and bunched into a roll at the tear line. Fixing all four inside
an axis-aligned-quad-plus-triangular-prism vocabulary means building a new
geometry system (per-tongue outlines, a recessed deck cut, a swept roll) —
not what a 60-90 m aerial read is worth. So: nothing is drawn on the roof
PLANE any more except the one dark, flat gravel-scour band.

WHAT SURVIVED. The peel's GROUND EVIDENCE — the torn membrane sheets it
threw downwind — is the half of the signature that always read correctly
(thin, small, scattered, seated flat by `build_debris`), so it is kept and
now drawn DIRECTLY off the windward edge rather than off a patch that no
longer exists (`_draw_shed_sheets`). Everything else this module does is
untouched: the gravel-scour band, the coping strip (piece removal plus
fallen ground evidence), the rooftop-prop topple/sweep, and the
`top_storey_loss` handshake that skips the whole pass.

SHED MEMBRANE SHEETS, PER LEVEL (`_SHEET_COUNT`). A tear RUN is drawn
along the windward edge — `_TEAR_RUN_FRAC` of that edge's own length,
snapped to one END of it (never centred, matching the record's own
"failure initiates at the edge/corner metal"), reaching `_TEAR_DEPTH_M`
in from the wall line — and `_SHEET_COUNT[level]` sheets are released from
the inner side of that run at roof height, carried along the wind bearing
by the same §2.7 `_deposit` model the coping boxes use (`tornado_urban.
_C_KIND["membrane"]`/`REACH_MAX_H`/`_lognormal`), landing either still on
the roof plane or past the building's own edge at grade. The run itself
authors NO geometry at all; it is recorded in the plan (`tear`) purely so
a reader can see where the sheets came from, and `apply_roof` never draws
it.

COPING STRIP. §2.6's own `t_parapet_fall` (T2+) already removes 30-60 % of
a building's windward parapet PIECES as part of the façade ladder — this
module's `_draw_coping` reads the façade plan's own `removed` list (when
handed one via the optional `facade_plan` kwarg) and only tops UP to its
own, slightly wider, per-level target (`_COPING_FRAC`, T1 already
non-zero — matching "roof-covering DODs are the earliest rungs", ahead of
`t_parapet_fall`'s own T2 floor) rather than double-removing pieces the
façade pass already took. Independently of whether real parapet/
parapet_corner pieces exist in the element table at all (a kit style with
no modelled parapet band has none — `tornado_kit`'s own "NOT SUPPORTED"
list does not even cover this case, it simply has zero `role == "parapet"`
elements), `_draw_coping` ALWAYS authors a handful of small fallen-coping
BOXES along the TRUE wall edge line (not the inset roof rect — coping
falls off the building's own outer edge) using the same low-`C_kind`
deposition the façade ladder already gives a parapet fragment, so a kit
building missing the piece-removal half of this signature still carries
the ground-evidence half.

ROOF PROPS. See "WHAT THIS MODULE IS, AND IS NOT" above for why these are
synthesised anchors rather than real placed prims. `_PROPS_TABLE` encodes
T1 mostly TOPPLING (a unit tips over near where it stood — casing/panel
loss and curb separation, ~85-120 mph, FEMA's own band) and T2+ mostly
SWEEPING (units gone, ejected out of frame) with 1-3 of the swept units
kept VISIBLE as authored boxes lying on their side, thrown to the
downwind portion of the roof — "displaced rooftop equipment... punctures
and tears roof membranes", so the visible remnant belongs downwind of
where it started, not centred.

HOOKS. `tornado_urban_usd.wreck_urban` and `tornado_kit.wreck_kit` each
call `plan_roof` + `apply_roof` immediately after their own façade
`apply_plan` call, passing the SAME `info`/`elements`/`level`/`wind` the
façade pass used and this module's own `roof_seed(tag)`-derived rng — see
those two functions' own docstrings for the exact call site (marked
`# R7 HOOK` in both) and `_plans/urban_tornado_W3RF_notes.md` for why the
edit is a two-line addition, not a rewrite, in each file.
"""

import math
import zlib

from pxr import Gf, Sdf, UsdGeom, UsdShade, Vt

from . import damage
from . import quake_flow as qf
from . import quake_sliced as qs
from . import tornado_urban as tu

# ---------------------------------------------------------------------------
# THE SEPARATE RNG STREAM
# ---------------------------------------------------------------------------
# Not literal hex "R00F" (not a valid hex token) -- a memorable stand-in
# for it. The only property that matters is that this XOR mixes `tag`'s own
# CRC well away from any other seed derivation in this codebase (none of
# which XOR against this constant), so two different subsystems seeding off
# the same `tag` string never collide.
ROOF_SEED_XOR = 0xA00F


def roof_seed(tag):
    """A stable (`zlib.crc32`, not Python's own randomized `hash()`)
    integer seed for this module's OWN `random.Random`, derived from `tag`
    alone -- never from `rng`'s or `nrng`'s state, so calling this and
    building a fresh `random.Random(roof_seed(tag))` never advances (and is
    never affected by) the façade planner's own rng sequence. See the
    module docstring's "THE SEPARATE RNG STREAM" section."""
    return zlib.crc32(str(tag).encode("utf-8")) ^ ROOF_SEED_XOR


# ---------------------------------------------------------------------------
# THE ROOF PLANE
# ---------------------------------------------------------------------------
ROOF_INSET_M = 0.4          # parapet/coping thickness the roof plane insets by
_ROOF_Z_OFFSET = 0.015      # the scour band's own lift off the roof plane
                            # — §2.9's own "0.015 m above the roof plane":
                            # thin enough to read as a material change
                            # rather than a step, thick enough not to
                            # z-fight the roof it lies on

LEVELS = ("T0", "T1", "T2", "T3", "T4")

# level -> (n_sheets_lo, n_sheets_hi): how many torn membrane/deck SHEETS
# the windward edge threw downwind. Round 3 drew these per PEEL PATCH (2-6
# each across 0-3 patches, i.e. 0-6 / 2-12 / 4-18 / 4-18 by level); with
# the patches retired the counts are stated directly, banded to land where
# the old product did and monotone in level by construction.
_SHEET_COUNT = {"T0": (0, 0), "T1": (0, 3), "T2": (2, 7), "T3": (5, 12),
                "T4": (7, 16)}

# (lo, hi) metres: how far in from the windward wall line the membrane tore
# before it let go, clamped to `_TEAR_DEPTH_MAX_FRAC` of the roof's own
# depth. NOT a coverage fraction — nothing is drawn over this area any
# more; it only sets the line the sheets were released from.
_TEAR_DEPTH_M = (1.0, 4.0)
_TEAR_DEPTH_MAX_FRAC = 0.6
# (lo, hi) fraction of the windward edge's own length the tear ran along.
_TEAR_RUN_FRAC = (0.35, 0.90)

# level -> (scour_lo, scour_hi): the fraction of the windward edge's own
# length the gravel-scour tint band runs along. Starts at T1 (roof-covering
# DODs are the EARLIEST rungs, R2 research §1 -- DI18/19 DOD2 sits BELOW
# every glazing/parapet DOD), grows through T4.
_SCOUR_FRAC = {"T0": (0.0, 0.0), "T1": (0.05, 0.15), "T2": (0.15, 0.30),
              "T3": (0.30, 0.50), "T4": (0.45, 0.70)}

# level -> (coping_lo, coping_hi): the TARGET fraction of the windward
# parapet edge's own length this module wants stripped, topped up over
# whatever `t_parapet_fall` (façade, T2+ only) already took. Non-zero at T1
# for the same "earliest rung" reason `_SCOUR_FRAC` is.
_COPING_FRAC = {"T0": (0.0, 0.0), "T1": (0.10, 0.25), "T2": (0.30, 0.50),
               "T3": (0.45, 0.65), "T4": (0.60, 0.80)}

# level -> (topple_lo, topple_hi, sweep_lo, sweep_hi, action, thrown_lo,
# thrown_hi). FEMA's own bands (module docstring, §2.3): casing/panel loss
# (rendered here as TOPPLE, a unit tipped near its own anchor) ~85-100 mph
# dominates at T1; curb separation and full ejection (SWEEP) dominate from
# T2 up, 1-3 of the swept units kept visible as thrown boxes.
_PROPS_TABLE = {
    "T0": (0.0, 0.0, 0.0, 0.0, "topple", 0, 0),
    "T1": (0.30, 0.60, 0.0, 0.15, "topple", 0, 1),
    "T2": (0.15, 0.35, 0.45, 0.70, "sweep", 1, 2),
    "T3": (0.05, 0.20, 0.60, 0.85, "sweep", 1, 3),
    "T4": (0.0, 0.15, 0.70, 0.95, "sweep", 2, 3),
}


def _roof_already_shed(facade_plan):
    """True when the FAÇADE plan already ledgered `top_storey_loss` --
    §2.6's ONE named exception where a `role == "roof"` piece is removed
    and shed as debris. `plan_roof` must never author a sheet/scour/
    coping/prop pass on a roof plane the façade pass has already
    physically taken off the building (nothing is left up there)."""
    if not facade_plan:
        return False
    for r in (facade_plan.get("regions") or ()):
        if r.get("recipe") == "top_storey_loss":
            return True
    return False


def _roof_rect_local(m, inset=ROOF_INSET_M):
    """(x0, y0, x1, y1) -- the roof PLANE in the mass's own local frame,
    the footprint box inset by `inset` on every side (the coping/parapet
    thickness the outer ring is left to)."""
    hw = max(0.5, float(m.get("W", 0.0)) / 2.0 - inset)
    hd = max(0.5, float(m.get("D", 0.0)) / 2.0 - inset)
    return (-hw, -hd, hw, hd)


def _side_frame(side, rect):
    """The `along`/`depth` axes for peeling `side` inward off `rect`.

    `along` runs along the wall line (the axis a bay index would run
    along); `depth` runs INTO the roof, away from that wall line -- 0 at
    the edge, growing toward the roof's own centre. `depth_max` is the
    full inward extent available on that axis, so a scour band or a
    tear run can be clamped to never cross the whole roof.
    """
    x0, y0, x1, y1 = rect
    if side == "S":
        return {"along_axis": "x", "along_lo": x0, "along_hi": x1,
                "edge_val": y0, "depth_sign": 1.0, "depth_max": (y1 - y0)}
    if side == "N":
        return {"along_axis": "x", "along_lo": x0, "along_hi": x1,
                "edge_val": y1, "depth_sign": -1.0, "depth_max": (y1 - y0)}
    if side == "W":
        return {"along_axis": "y", "along_lo": y0, "along_hi": y1,
                "edge_val": x0, "depth_sign": 1.0, "depth_max": (x1 - x0)}
    return {"along_axis": "y", "along_lo": y0, "along_hi": y1,     # "E"
            "edge_val": x1, "depth_sign": -1.0, "depth_max": (x1 - x0)}


def _band_rect(frame, along_start, along_len, depth_len):
    """(x0, y0, x1, y1) local, from `frame`'s own along/depth axes -- the
    gravel-scour band's own footprint (its only caller since the peel
    patches that shared it were retired)."""
    a0 = frame["along_lo"] + along_start
    a1 = a0 + along_len
    e0 = frame["edge_val"]
    e1 = e0 + frame["depth_sign"] * depth_len
    if frame["along_axis"] == "x":
        x0, x1 = a0, a1
        y0, y1 = (e0, e1) if e0 <= e1 else (e1, e0)
    else:
        y0, y1 = a0, a1
        x0, x1 = (e0, e1) if e0 <= e1 else (e1, e0)
    return (x0, y0, x1, y1)


def _inner_edge_line(frame, along_start, along_len, depth_len):
    """The line `depth_len` in from the windward wall, spanning
    `along_len` from `along_start` -- the INNER side of the tear run,
    where the membrane let go and the shed sheets were released from.
    Returns (p0, p1) local points."""
    a0 = frame["along_lo"] + along_start
    a1 = a0 + along_len
    inner = frame["edge_val"] + frame["depth_sign"] * depth_len
    if frame["along_axis"] == "x":
        return (a0, inner), (a1, inner)
    return (inner, a0), (inner, a1)


def _wall_edge_line_local(m, side):
    """(p0, p1) local -- the TRUE footprint wall line for `side` (not the
    inset roof rect): where coping actually overhangs the street."""
    hw, hd = float(m.get("W", 0.0)) / 2.0, float(m.get("D", 0.0)) / 2.0
    if side == "S":
        return (-hw, -hd), (hw, -hd)
    if side == "N":
        return (-hw, hd), (hw, hd)
    if side == "W":
        return (-hw, -hd), (-hw, hd)
    return (hw, -hd), (hw, hd)          # "E"


def _touching_corners(side):
    return [cn for cn, (a, b) in qs._CORNER_SIDES.items() if side in (a, b)]


# ---------------------------------------------------------------------------
# DEBRIS PHYSICS -- reusing `tornado_urban`'s own §2.7 constants/helpers so
# a roof fragment's reach is drawn from the SAME model the façade ladder's
# own shed-roof-piece debris already uses, not a second, competing formula.
# ---------------------------------------------------------------------------
def _deposit(rng, m, lx0, ly0, release_h, wind, kind, material, size,
             c_kind, tone=""):
    """One fragment released at local `(lx0, ly0)` at height `release_h`,
    carried along `wind["bearing_deg"]` with §2.7's own gaussian scatter and
    lognormal-drag reach. Returns a `tornado_urban`-shaped debris dict in
    WORLD (cell-local) coordinates, `z` set to the roof plane when the
    landing point is still inside the building's own footprint and to grade
    (0.0) when it carries past the edge -- "deposited downwind on the roof
    AND past it" (§8's own wording for the torn-sheet class). Used for the
    shed membrane sheets and the coping strip's fallen boxes -- NOT for
    roof props, whose "stay on the roof, downwind" placement (§8: "throw 1-3
    onto the roof surface downwind") is a direct biased-anchor draw
    (`_downwind_point`) rather than a wind-reach-then-clip pass: a swept
    unit's resting point is decided by where the wind pushed it across the
    roof deck, not by a ballistic throw with a fall height.
    """
    bearing = math.radians(float(wind.get("bearing_deg", 0.0)))
    theta = bearing + math.radians(rng.gauss(0.0, 18.0))
    speed = 25.0 + 70.0 * max(0.0, min(1.0, float(wind.get("speed_frac", 0.5))))
    raw_reach = (c_kind * speed * math.sqrt(max(0.0, 2.0 * max(0.0, release_h)
                                                / tu.G_ACCEL))
                * tu._lognormal(rng, 0.35))
    reach_cap = max(tu.REACH_FLOOR_M, tu.REACH_MAX_H * max(1.0, release_h))
    reach = min(raw_reach, reach_cap)
    sigma = 0.12 * reach + 0.04 * max(0.0, release_h)
    lat = rng.gauss(0.0, max(0.01, sigma))
    bx, by = math.cos(theta), math.sin(theta)
    px_, py_ = -by, bx
    wx0, wy0 = qf._to_world(m, lx0, ly0)
    wx = wx0 + reach * bx + lat * px_
    wy = wy0 + reach * by + lat * py_

    llx, lly = qf._to_local(m, wx, wy)
    on_roof = (abs(llx) <= float(m.get("W", 0.0)) / 2.0
              and abs(lly) <= float(m.get("D", 0.0)) / 2.0)
    z = release_h if on_roof else 0.0

    tilt = rng.uniform(5.0, 25.0) if rng.random() < 0.15 else rng.uniform(0.0, 3.0)
    return {"kind": str(kind), "material": str(material),
            "size": [float(q) for q in size], "x": float(wx), "y": float(wy),
            "z": float(z), "yaw_deg": float(rng.uniform(0.0, 360.0)),
            "tilt_deg": float(tilt), "tone": str(tone or "")}


def _downwind_point(rng, rect, bearing_deg, bias):
    """A local (lx, ly) inside `rect`, biased toward the DOWNWIND portion
    (`bias in (0, 1]`, higher = stronger skew via a max-of-N draw) -- where
    a swept roof unit's visible remnant comes to rest, and where a toppled
    unit's own anchor is drawn from (bias 0 there -- unbiased, since a
    unit that merely tips over stays wherever it originally stood)."""
    x0, y0, x1, y1 = rect
    bx, by = math.cos(math.radians(bearing_deg)), math.sin(math.radians(bearing_deg))
    n_draw = 1 + int(round(3 * max(0.0, min(1.0, bias))))
    t = max(rng.random() for _ in range(n_draw))
    cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    half = 0.5 * math.hypot(x1 - x0, y1 - y0)
    px_, py_ = -by, bx
    lat = rng.uniform(-0.4, 0.4) * min(x1 - x0, y1 - y0)
    lx = cx + (t * 2.0 - 1.0) * half * bx * 0.8 + lat * px_
    ly = cy + (t * 2.0 - 1.0) * half * by * 0.8 + lat * py_
    return min(x1, max(x0, lx)), min(y1, max(y0, ly))


# ---------------------------------------------------------------------------
# SHED MEMBRANE SHEETS
#
# All that is left of the retired peel population (module docstring,
# "ROUND 4"): the sheets it threw downwind. NOTHING here authors geometry
# on the roof plane -- the tear run is a plan RECORD that sets the release
# line and nothing else.
# ---------------------------------------------------------------------------
def _zero_tear(side):
    return {"side": side, "along_start_m": 0.0, "run_m": 0.0, "depth_m": 0.0,
            "p0_local": None, "p1_local": None}


def _draw_shed_sheets(rng, level, windward, rect, roof_top, m, wind):
    """`(tear, sheets)` -- the tear RUN along the windward edge and the
    torn membrane/deck sheets released from its inner side.

    The run is `_TEAR_RUN_FRAC` of the windward edge's own length, snapped
    to ONE END of that edge (never centred: the record's own "failure
    initiates at the edge/corner metal"), reaching `_TEAR_DEPTH_M` inward
    but never past `_TEAR_DEPTH_MAX_FRAC` of the roof's own depth. Each
    sheet is carried by the same `_deposit` model the coping boxes use, so
    it lands either still on the roof plane or past the building's own
    edge at grade.
    """
    lo_n, hi_n = _SHEET_COUNT.get(level, (0, 0))
    n = rng.randint(lo_n, hi_n) if hi_n > 0 else 0
    if n <= 0:
        return _zero_tear(windward), []

    frame = _side_frame(windward, rect)
    along_total = max(1e-6, frame["along_hi"] - frame["along_lo"])
    depth_max = max(1e-6, frame["depth_max"])
    run = min(along_total, along_total * rng.uniform(*_TEAR_RUN_FRAC))
    end = rng.choice(("lo", "hi"))
    a0 = 0.0 if end == "lo" else max(0.0, along_total - run)
    depth = max(0.5, min(depth_max * _TEAR_DEPTH_MAX_FRAC,
                         rng.uniform(*_TEAR_DEPTH_M)))
    p0_l, p1_l = _inner_edge_line(frame, a0, run, depth)

    sheets = []
    for _s in range(n):
        t = rng.random()
        slx = p0_l[0] + t * (p1_l[0] - p0_l[0])
        sly = p0_l[1] + t * (p1_l[1] - p0_l[1])
        size = (rng.uniform(1.0, 3.0), rng.uniform(0.5, 2.0),
               rng.uniform(0.006, 0.02))
        sheets.append(_deposit(rng, m, slx, sly, roof_top, wind,
                               "roof_sheet", "membrane", size,
                               tu._C_KIND["membrane"]))

    tear = {"side": windward, "along_start_m": float(a0), "run_m": float(run),
            "depth_m": float(depth),
            "p0_local": [float(p0_l[0]), float(p0_l[1])],
            "p1_local": [float(p1_l[0]), float(p1_l[1])]}
    return tear, sheets


# ---------------------------------------------------------------------------
# GRAVEL SCOUR
# ---------------------------------------------------------------------------
def _draw_scour(rng, level, windward, rect, roof_top):
    lo, hi = _SCOUR_FRAC.get(level, (0.0, 0.0))
    if hi <= 0.0:
        return {"side": windward, "frac": 0.0, "rect_local": None, "z": None,
               "tint": None}
    frac = rng.uniform(lo, hi)
    frame = _side_frame(windward, rect)
    along_total = max(1e-6, frame["along_hi"] - frame["along_lo"])
    depth_len = max(0.8, min(frame["depth_max"] * 0.9, 1.0 + 3.5 * frac))
    rx0, ry0, rx1, ry1 = _band_rect(frame, 0.0, along_total, depth_len)
    # gravel-ballast-stripped tone: darker than the intact roof, revealing
    # the dark felt/tar beneath (§ hurricane research 2.2's own sequence)
    tint = (0.30, 0.28, 0.26)
    return {"side": windward, "frac": float(frac),
            "rect_local": [float(rx0), float(ry0), float(rx1), float(ry1)],
            "z": float(roof_top + _ROOF_Z_OFFSET), "tint": list(tint)}


# ---------------------------------------------------------------------------
# COPING STRIP
# ---------------------------------------------------------------------------
def _draw_coping(rng, level, windward, info, elements, m, rect, roof_top,
                 wind, facade_plan):
    # ROUND 4 (stream D's B-rubble fix, extended here): the building's own
    # masonry TONE token, resolved from its style exactly the way the façade
    # ledger resolves it (`tornado_urban._KIT_TONE`/`_tone_for`). Fallen
    # coping classifies into `tornado_urban_usd._classify`'s `brick` bucket
    # (its material string is the literal "coping"), which is TONEABLE -- so
    # without this the white-stone B-row kits (`brownstone_row`, `walkup`)
    # got the generic red-brown brick rubble map on their coping while the
    # façade ladder's own coping fragments, three prims away on the same
    # building, correctly bound the pale `stone` look. `""` for every style
    # the table does not name (every sliced A-row building), which leaves
    # those bindings exactly where the review approved them.
    tone = tu._tone_for(info.get("style"))
    result = {"side": windward, "target_frac": 0.0, "already_removed_frac": 0.0,
              "tone": str(tone or ""), "piece_removed": [], "boxes": []}
    lo, hi = _COPING_FRAC.get(level, (0.0, 0.0))
    if hi <= 0.0:
        return result
    target = rng.uniform(lo, hi)
    result["target_frac"] = float(target)

    g = qs._Grid(info, elements)
    touching = (windward,) + tuple(_touching_corners(windward))
    par = [e for e in g.role_pieces(("parapet", "parapet_corner"))
          if (e.get("p") or {}).get("_side") in touching]
    par_paths = sorted({qs._path(e) for e in par if qs._path(e)})
    already = set((facade_plan or {}).get("removed") or ())
    n_par = len(par_paths)
    n_already = sum(1 for p in par_paths if p in already)
    result["already_removed_frac"] = (n_already / n_par) if n_par else 0.0

    remaining = max(0.0, target - result["already_removed_frac"])
    if n_par and remaining > 0.0:
        avail = sorted(p for p in par_paths if p not in already)
        want = min(len(avail), max(0, int(round(remaining * n_par))))
        picked = list(avail)
        rng.shuffle(picked)
        result["piece_removed"] = sorted(picked[:want])

    # ALWAYS author fallen-coping ground evidence, real pieces or not (see
    # the module docstring's "COPING STRIP" section) -- released along the
    # TRUE wall edge (not the inset roof rect), dense/low-C so it lands
    # close to the wall base, mirroring the façade ladder's own `coping`
    # fragment physics (`tu._C_KIND["coping"]`).
    p0, p1 = _wall_edge_line_local(m, windward)
    n_boxes = max(0, int(round(target * rng.uniform(3.0, 7.0))))
    boxes = []
    for _b in range(n_boxes):
        t = rng.random()
        lx = p0[0] + t * (p1[0] - p0[0])
        ly = p0[1] + t * (p1[1] - p0[1])
        size = (rng.uniform(0.22, 0.50), rng.uniform(0.22, 0.50),
               rng.uniform(0.15, 0.35))
        boxes.append(_deposit(rng, m, lx, ly, roof_top, wind, "coping",
                              "coping", size, tu._C_KIND["coping"],
                              tone=tone))
    result["boxes"] = boxes
    return result


# ---------------------------------------------------------------------------
# ROOF PROPS
# ---------------------------------------------------------------------------
def _draw_props(rng, level, rect, roof_top, wind, m):
    row = _PROPS_TABLE.get(level, _PROPS_TABLE["T0"])
    t_lo, t_hi, s_lo, s_hi, action, th_lo, th_hi = row
    x0, y0, x1, y1 = rect
    roof_area = max(1.0, (x1 - x0) * (y1 - y0))
    n_units = max(0, int(round(roof_area / 350.0)) + rng.randint(-1, 1))
    if level == "T0" or n_units <= 0:
        return {"action": "none", "n_units": 0, "topple_frac": 0.0,
                "sweep_frac": 0.0, "n_topple": 0, "n_thrown": 0,
                "topple": [], "thrown": []}

    topple_frac = rng.uniform(t_lo, t_hi)
    sweep_frac = rng.uniform(s_lo, s_hi)
    n_topple = int(round(n_units * topple_frac))
    n_sweep = int(round(n_units * sweep_frac))
    n_thrown = min(n_sweep, rng.randint(th_lo, th_hi)) if th_hi > 0 else 0

    inset2 = 1.5
    anchor_rect = (x0 + inset2, y0 + inset2, x1 - inset2, y1 - inset2)
    if anchor_rect[0] >= anchor_rect[2] or anchor_rect[1] >= anchor_rect[3]:
        anchor_rect = rect
    bearing = float(wind.get("bearing_deg", 0.0))

    topple = []
    for _u in range(max(0, n_topple)):
        lx, ly = _downwind_point(rng, anchor_rect, bearing, bias=0.0)
        wx, wy = qf._to_world(m, lx, ly)
        size = (rng.uniform(0.9, 1.8), rng.uniform(0.9, 1.8),
               rng.uniform(0.5, 1.0))
        topple.append({
            "kind": "roof_prop", "material": "metal",
            "size": [float(q) for q in size], "x": float(wx), "y": float(wy),
            "z": float(roof_top), "yaw_deg": float(rng.uniform(0.0, 360.0)),
            "tilt_deg": float(rng.uniform(40.0, 75.0))})

    thrown = []
    for _u in range(max(0, n_thrown)):
        lx, ly = _downwind_point(rng, rect, bearing, bias=0.8)
        wx, wy = qf._to_world(m, lx, ly)
        size = (rng.uniform(0.9, 1.8), rng.uniform(0.9, 1.8),
               rng.uniform(0.5, 1.0))
        thrown.append({
            "kind": "roof_prop", "material": "metal",
            "size": [float(q) for q in size], "x": float(wx), "y": float(wy),
            "z": float(roof_top), "yaw_deg": float(rng.uniform(0.0, 360.0)),
            "tilt_deg": float(rng.uniform(75.0, 100.0))})   # lying on its side

    return {"action": action, "n_units": int(n_units),
            "topple_frac": float(topple_frac), "sweep_frac": float(sweep_frac),
            "n_topple": len(topple), "n_thrown": len(thrown),
            "topple": topple, "thrown": thrown}


# ---------------------------------------------------------------------------
# STATS
# ---------------------------------------------------------------------------
def _empty_stats():
    """`n_patches`/`patch_coverage_frac` are deliberately ABSENT, not
    zeroed -- see the module docstring's schema note."""
    return {"roof_area_m2": 0.0, "n_sheets": 0, "n_coping_pieces": 0,
            "n_coping_boxes": 0, "n_topple": 0, "n_thrown_props": 0,
            "scour_frac": 0.0}


def _compute_stats(roof_plan, rect):
    x0, y0, x1, y1 = rect
    roof_area = max(1e-9, (x1 - x0) * (y1 - y0))
    coping = roof_plan.get("coping") or {}
    props = roof_plan.get("props") or {}
    scour = roof_plan.get("scour") or {}
    return {
        "roof_area_m2": float(roof_area),
        "n_sheets": len(roof_plan.get("sheets") or ()),
        "n_coping_pieces": len(coping.get("piece_removed") or ()),
        "n_coping_boxes": len(coping.get("boxes") or ()),
        "n_topple": int(props.get("n_topple") or 0),
        "n_thrown_props": int(props.get("n_thrown") or 0),
        "scour_frac": float(scour.get("frac") or 0.0),
    }


# ---------------------------------------------------------------------------
# THE PLAN
# ---------------------------------------------------------------------------
def plan_roof(info, elements, level, wind, rng, height_class, intensity,
             facade_plan=None):
    """Everything the roof-damage pass decides, with NO USD access at all
    -- see the module docstring for the schema. `facade_plan` is an
    OPTIONAL addition to the brief's own signature (deviation, documented
    in `_plans/urban_tornado_W3RF_notes.md`): needed to (a) refuse a roof
    already shed by the façade's own `top_storey_loss`, and (b) top up
    (never double-remove) the façade's own windward parapet loss rather
    than compete with it. `btype` is read from `info["type"]` (the same
    field `wreck_urban`/`wreck_kit` stamp onto `info` before calling
    `tornado_urban.plan_damage`), not a separate parameter -- the brief's
    own literal signature has no `btype` slot even though "draw per btype"
    needs one; reading `info["type"]` costs nothing new at the call site
    and keeps the signature exactly as given.
    """
    level = str(level or "T0")
    wind = {"bearing_deg": float((wind or {}).get("bearing_deg", 0.0)),
            "speed_frac": float((wind or {}).get("speed_frac", 0.0)),
            "cross_frac": float((wind or {}).get("cross_frac", 0.0)),
            "over": bool((wind or {}).get("over", False))}
    btype = str(info.get("type") or "rc")
    m = dict((info.get("masses") or {}).get("main") or {})
    H = float(info.get("H") or m.get("top") or 0.0)

    roof_plan = {
        "schema": "tornado_roof_plan.v2", "level": level, "btype": btype,
        "height_class": str(height_class or ""), "H": H, "wind": wind,
        "roof": {"cx": float(m.get("cx", 0.0)), "cy": float(m.get("cy", 0.0)),
                "yaw": float(m.get("yaw", 0.0)), "W": float(m.get("W", 0.0)),
                "D": float(m.get("D", 0.0)), "top": float(m.get("top", H)),
                "inset": ROOF_INSET_M, "rect_local": None},
        "windward_side": None, "side_weights": {},
        "skipped": False, "skip_reason": None,
        # `patches` is a permanently-empty v1 compatibility key: the
        # membrane-peel population is retired and nothing refills it.
        "patches": [], "tear": None, "sheets": [],
        "scour": None, "coping": None, "props": None,
        "notes": [], "stats": {},
    }

    if _roof_already_shed(facade_plan):
        roof_plan["skipped"] = True
        roof_plan["skip_reason"] = (
            "roof already shed by top_storey_loss -- no additional roof "
            "damage authored on a building whose roof piece is already "
            "ledgered as facade debris")
        roof_plan["notes"].append(roof_plan["skip_reason"])
        roof_plan["stats"] = _empty_stats()
        return roof_plan

    if float(m.get("W", 0.0)) <= 0.0 or float(m.get("D", 0.0)) <= 0.0:
        roof_plan["skipped"] = True
        roof_plan["skip_reason"] = ("no usable roof mass on "
                                    "info[\"masses\"][\"main\"]")
        roof_plan["notes"].append(roof_plan["skip_reason"])
        roof_plan["stats"] = _empty_stats()
        return roof_plan

    weights = tu.side_weights(info, wind, rng)
    roof_plan["side_weights"] = {k: float(v) for k, v in weights.items()}
    windward = tu._rank_sides(weights)[0]
    roof_plan["windward_side"] = windward

    rect = _roof_rect_local(m)
    roof_plan["roof"]["rect_local"] = [float(q) for q in rect]
    roof_top = float(m.get("top", H))

    roof_plan["tear"], roof_plan["sheets"] = _draw_shed_sheets(
        rng, level, windward, rect, roof_top, m, wind)
    roof_plan["scour"] = _draw_scour(rng, level, windward, rect, roof_top)
    roof_plan["coping"] = _draw_coping(rng, level, windward, info, elements, m,
                                       rect, roof_top, wind, facade_plan)
    roof_plan["props"] = _draw_props(rng, level, rect, roof_top, wind, m)

    roof_plan["notes"].append(
        "roof: {0} shed sheet(s) off {1}, scour {2:.2f}, coping target "
        "{3:.2f} ({4} piece(s) + {5} box(es)), props {6} ({7} topple / "
        "{8} thrown)"
        .format(len(roof_plan["sheets"]), windward,
                roof_plan["scour"]["frac"], roof_plan["coping"]["target_frac"],
                len(roof_plan["coping"]["piece_removed"]),
                len(roof_plan["coping"]["boxes"]),
                roof_plan["props"]["action"], roof_plan["props"]["n_topple"],
                roof_plan["props"]["n_thrown"]))
    roof_plan["stats"] = _compute_stats(roof_plan, rect)
    return roof_plan


# ===========================================================================
# APPLY -- pxr from here down
# ===========================================================================
def _roof_material(stage, ctx, key, rgb, roughness):
    """One material per roof LOOK KEY, cached in `ctx["mats"]` under a
    `"tornado_roof:"` namespace so it never collides with the façade
    ladder's own `"tornado_debris:"`-keyed materials in the same dict.

    FLAT COLOUR ONLY, and only one key ships today (`scour`). The single
    TEXTURED roof look this module ever had was the retired peel's
    concrete-deck substrate, and it carried round 4's own trap with it:
    per the OmniPBR MDL, `diffuse_color_constant` (what `damage._pbr`
    writes from its `tint` argument) is only the map-failed FALLBACK and
    `diffuse_tint` is the multiplier over a resolved map, so passing the
    class rgb as the "tint" multiplied the texture down to ~0.4 of itself
    on every patch. If a textured roof look is ever added back here, bind
    the map, set `diffuse_tint` near-neutral on the shader, and leave the
    class rgb as the constant -- do NOT assume `_pbr(tint=rgb)` tints.
    """
    mats = ctx.setdefault("mats", {})
    cache_key = "tornado_roof:" + str(key)
    got = mats.get(cache_key)
    if got is not None:
        return got
    parent = ctx.get("parent") or "/World"
    path = "{0}/TornadoRoofLooks/{1}".format(parent, key)
    mat = damage._pbr(stage, path, rgb, roughness)
    mats[cache_key] = mat
    return mat


def _author_quad(stage, path, rect_local, z, m, mat):
    """A single upward-facing quad, `rect_local` (roof-frame) converted to
    world (cell-local) corners via `quake_flow._to_world`, faceVarying
    normals (+Z) -- the gravel-scour band is the only thing this module
    still draws on the roof plane, viewed from directly above where a
    rooftop is always read."""
    x0, y0, x1, y1 = rect_local
    corners_l = ((x0, y0), (x1, y0), (x1, y1), (x0, y1))
    pts = [Gf.Vec3f(*qf._to_world(m, lx, ly), z) for lx, ly in corners_l]
    m_msh = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
    m_msh.CreatePointsAttr(Vt.Vec3fArray(pts))
    m_msh.CreateFaceVertexCountsAttr(Vt.IntArray([4]))
    m_msh.CreateFaceVertexIndicesAttr(Vt.IntArray([0, 1, 2, 3]))
    m_msh.CreateNormalsAttr(Vt.Vec3fArray([Gf.Vec3f(0, 0, 1)] * 4))
    m_msh.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
    m_msh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    xs, ys, zs = [p[0] for p in pts], [p[1] for p in pts], [p[2] for p in pts]
    m_msh.CreateExtentAttr([Gf.Vec3f(min(xs), min(ys), min(zs)),
                            Gf.Vec3f(max(xs), max(ys), max(zs))])
    if mat is not None:
        UsdShade.MaterialBindingAPI.Apply(m_msh.GetPrim()).Bind(mat)
    return path


def _author_fragments_grouped(stage, parent, fragments, ctx):
    """`tornado_urban_usd.build_debris` takes ONE `ground_z` per call, but
    this module's own fragments legitimately land on either the roof plane
    or the street below it (§ module docstring). Group by each fragment's
    own `z`, call `build_debris` once per distinct baseline, and SUFFIX
    `kind` with the baseline so the two groups' merged meshes never collide
    on one prim path (the debris LOOK stays governed by `material` alone,
    unaffected by this suffix -- `tornado_urban_usd._classify` never reads
    `kind`)."""
    from . import tornado_urban_usd as tuu

    if not fragments:
        return []
    groups = {}
    for f in fragments:
        z = round(float(f.get("z", 0.0)), 3)
        groups.setdefault(z, []).append(f)
    made = []
    for z in sorted(groups):
        tagged = []
        for f in groups[z]:
            g = dict(f)
            g["kind"] = "{0}_z{1:g}".format(f.get("kind", "frag"), z)
            tagged.append(g)
        made += tuu.build_debris(stage, parent, tagged, ctx, ground_z=z)
    return made


def apply_roof(stage, ctx, roof_plan, verbose=True):
    """Author `roof_plan` (§ this module's own schema) onto a live stage.
    Tolerant of a `skipped` or otherwise empty plan -- returns a zeroed
    counts dict rather than raising. `ctx` is the SAME dict `tornado_urban_
    usd.apply_plan`/`tornado_kit.wreck_kit` already carry (`stage`,
    `parent`, `mats`, `static_extra`, `notes`, ...); this function only
    ADDS to `ctx["mats"]`/`ctx["static_extra"]`/`ctx["notes"]`, it never
    replaces them.
    """
    roof_plan = roof_plan or {}
    ctx.setdefault("mats", {})
    ctx.setdefault("static_extra", [])
    ctx.setdefault("notes", [])
    # No `n_patch_quads`/`n_lips` key: this pass authors neither, at any
    # level, and a permanently-zero counter is a claim that it might.
    counts = {"n_sheet_meshes": 0, "n_scour_quads": 0, "n_coping_removed": 0,
             "n_coping_missing": 0, "n_coping_boxes_mesh": 0,
             "n_props_mesh": 0}

    if roof_plan.get("skipped"):
        reason = roof_plan.get("skip_reason") or "roof plan skipped"
        if reason not in ctx["notes"]:
            ctx["notes"].append("roof: " + reason)
        if verbose:
            print("[tornado_roof] skipped: {0}".format(reason))
        return counts

    parent = ctx.get("parent") or "/World"
    roof = roof_plan.get("roof") or {}
    m = {"cx": roof.get("cx", 0.0), "cy": roof.get("cy", 0.0),
        "yaw": roof.get("yaw", 0.0), "W": roof.get("W", 0.0),
        "D": roof.get("D", 0.0)}
    root = "{0}/tornado_roof".format(parent)
    UsdGeom.Scope.Define(stage, Sdf.Path(root))

    # 1) SHED MEMBRANE SHEETS -- debris only. NO substrate quad and NO
    #    membrane lip is authored here at any level (module docstring,
    #    "ROUND 4"); a v1 plan's own nested `patches[*]` are deliberately
    #    IGNORED rather than replayed, so an old cached plan re-applied
    #    through this function still puts nothing on the roof plane.
    sheets = list(roof_plan.get("sheets") or ())
    if sheets:
        made = _author_fragments_grouped(stage, parent, sheets, ctx)
        ctx["static_extra"].extend(made)
        counts["n_sheet_meshes"] = len(made)

    # 2) GRAVEL SCOUR -- one tinted band quad, the only roof-plane art
    scour = roof_plan.get("scour") or {}
    if scour.get("rect_local") and float(scour.get("frac") or 0.0) > 0.0:
        scour_mat = _roof_material(
            stage, ctx, "scour",
            tuple(scour.get("tint") or (0.3, 0.28, 0.26)), 0.85)
        spath = "{0}/scour".format(root)
        _author_quad(stage, spath, scour["rect_local"], float(scour["z"]),
                    m, scour_mat)
        ctx["static_extra"].append(spath)
        counts["n_scour_quads"] += 1

    # 3) COPING STRIP -- deactivate real pieces, always author fallen debris
    coping = roof_plan.get("coping") or {}
    n_rm, n_miss = 0, 0
    for path in sorted(coping.get("piece_removed") or ()):
        prim = stage.GetPrimAtPath(path) if path else None
        if not prim or not prim.IsValid():
            n_miss += 1
            continue
        if qf._deactivate(stage, path):
            n_rm += 1
    counts["n_coping_removed"] = n_rm
    counts["n_coping_missing"] = n_miss
    boxes = coping.get("boxes") or ()
    if boxes:
        made = _author_fragments_grouped(stage, parent, boxes, ctx)
        ctx["static_extra"].extend(made)
        counts["n_coping_boxes_mesh"] = len(made)

    # 4) ROOF PROPS -- toppled (near-anchor) + thrown (downwind, on-side)
    props = roof_plan.get("props") or {}
    prop_frags = list(props.get("topple") or ()) + list(props.get("thrown") or ())
    if prop_frags:
        made = _author_fragments_grouped(stage, parent, prop_frags, ctx)
        ctx["static_extra"].extend(made)
        counts["n_props_mesh"] = len(made)

    for line in roof_plan.get("notes") or ():
        full = "roof: " + line
        if full not in ctx["notes"]:
            ctx["notes"].append(full)

    if verbose:
        print("[tornado_roof] {0}: {1} shed-sheet mesh(es), {2} scour "
              "quad(s), {3} coping piece(s) removed (+{4} missing)/{5} box "
              "mesh(es), {6} prop mesh(es)".format(
                  roof_plan.get("level") or "plan", counts["n_sheet_meshes"],
                  counts["n_scour_quads"], counts["n_coping_removed"],
                  counts["n_coping_missing"], counts["n_coping_boxes_mesh"],
                  counts["n_props_mesh"]))
    return counts
