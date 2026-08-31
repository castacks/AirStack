"""fire_collapse — PART of a burnt-out shell has come down.

WHAT THIS IS, AND WHY `urban_fire.r_fire_collapse` IS NOT IT
------------------------------------------------------------
`urban_fire.r_fire_collapse` (F5/F6) is the Windsor / Plasco case: the burnt-out
TOP one or two storeys drop into the floors below and the lower frame, never
heated, stands. Looked at from the street the building still has four walls —
what changed is its skyline.

The user asked for the other half of the fire-collapse vocabulary
(2026-08-30): "I want some partial collapse buildings for fire in all sets ...
spawn the modern city env with 1 partial collapsed building." That is a
building that has lost part of its SHELL — one burnt elevation lying in the
street with the floors behind it sagged and dropped, or a corner of the block
gone from the fire floor up — with the rest of it still standing and the
interior on show through the hole.

The two are complementary and both are real; this module owns the second and
does not touch the first.

WHY IT STILL HAS TO READ AS A FIRE
-----------------------------------
An urban partial collapse is the earthquake pipeline's home ground
(`quake_flow.r_out_of_plane`, `r_corner_fail`, `monolith_damage.
partial_collapse`), and reusing its LOOK is the fastest way to build the wrong
disaster in one step. Three things separate the two, and every one of them is
enforced here:

  * **The rubble is black and wet, not dusty.** A quake heap's signature IS
    its dust plume — `quake_flow._heap`'s default `HEAP_MIX` is mortar dust
    over pale brick, and `_a_dustify` greys every fragment on top of that.
    Fire rubble has no fines standing on it: it has been hosed. Everything
    authored or rebound here draws from `urban_fire`'s own palette
    (`_debris_mat` -> char/scorch, `mats["soot"|"char_concrete"|"calcined"]`),
    and every helper borrowed from `quake_flow` has its output rebound
    afterwards (the same `before`/`made` diff `urban_fire.r_roof_burnthrough`
    uses around `quake_flow.r_roof_hole`).
  * **The failure starts where the FIRE was, not where the drift was.** The
    lost region is drawn from `ctx["fire"]["sides"]` and `ctx["fire"]
    ["origin"]`: nothing below the origin storey is touched, so the clean
    masonry band under a black stripe — the single strongest "this is a
    building fire" cue in `urban_fire`'s whole ladder — survives a collapse
    that takes everything above it.
  * **The break line is a STAIRCASE that widens upward**, not a rectangle and
    not a long diagonal. `monolith_damage`'s own note ("a staircase follows
    bays and courses; no long diagonal triangle", and its `validate()` refuses
    a three-point profile) is the rule; here the span of the lost region grows
    per storey from `PROFILE_FOOT` of its full width at the failure line to
    all of it at the top, so what is left standing is a notch on the module
    grid with a ragged edge, not a module-aligned hole.

MODES
-----
`elevation`  ONE burning elevation is gone from the failure line up. The wall
             fell OUTWARD (a peel rotating about its foot: fragments high on
             the wall get the most outward speed, exactly as
             `quake_flow.r_out_of_plane` does it), so the windrow of charred
             masonry is in the STREET, outside the wall line, and a smaller
             heap is inside where the inner leaf and the floor edges came
             down.
`corner`     the corner two burning elevations share is gone from the failure
             line up, taking a run of each adjoining elevation with it. The
             V-notch in the roofline, but blackened and with no dust.
`auto`       corner when the fire is venting on two elevations that share one
             and the building's own seed says so, else elevation.

THE RNG CONTRACT
----------------
`soot_plume`'s note applies with full force here: every recipe in
`urban_fire.LADDER` draws from ONE `random.Random` in ladder order, so a
recipe that draws a different NUMBER of values than it did yesterday moves
every later recipe's outcome ("for building 5 it looks like the roof is
floating even though previously it wasn't", user 2026-08-30). A recipe that
can be inserted anywhere in a ladder — which is exactly what this one is for —
must therefore not disturb that sequence at all.

So this module draws from ITS OWN generators, seeded off
`soot_plume.event_seed(ctx) ^ FIRE_COLLAPSE_SEED_XOR`, and INSTALLS them on
the ctx for the duration of the authoring (`_own_rng`), because the
`quake_flow` helpers it calls (`_heap`, `_ragged_slabs`, `_ragged_neighbours`,
`r_droop`) read `ctx["rng"]` / `ctx["nrng"]` off the ctx and cannot be handed
a generator any other way. The shared generators are restored in a `finally`,
so `r_partial_collapse` consumes exactly ZERO draws from them. `_break` and
`fracture.fracture_prim` get the private pair explicitly.

**`fracture_prim` wants a numpy `Generator`, never a `random.Random`** — it
calls `rng.permutation`, and the mismatch crashed a run on 2026-08-30 (the
roof-lid fix in `r_fire_collapse`, recorded in `build-urban-fire-scenes`).
`_own_rng` therefore carries BOTH: `random.Random(seed)` and
`numpy.random.default_rng(seed)`.

TESTED WITHOUT A STAGE
----------------------
Every decision this recipe makes — which elements die, which elevations are
left completely alone, where the two heaps go — is computed by
`plan_partial_collapse`, which touches no `pxr` and no stage. `scene_gen/tests/
test_fire_collapse.py` builds real kit buildings host-side
(`urban_building.build_building` + `quake_flow.describe`), plans a real fire,
and asserts on that plan. `r_partial_collapse` is only the USD authoring of a
plan that has already been checked.
"""

import math
import os
import random

# THE LEVEL HAS TO EXIST IN `soot_plume`'S OWN TABLE OR THE BUILDING DOES NOT
# BURN AT ALL. `soot_plume.plan_events` opens with
#
#     if level not in DURATION_S: return []       # F0: nothing vented
#
# so a level name that table has never heard of produces NO fire events —
# which means no soot skin, no glass film and no Flow emitters. A partially
# collapsed shell with pristine cladding is a demolition site, and the failure
# is silent: `burn_building` still runs the whole ladder and still reports
# success. `soot_plume.py` is owned elsewhere this session, so `F5c` is
# REGISTERED into its table from here rather than edited into it — same burn
# duration as F5 (a ~65 minute fully-developed fire), which is what F5c is.
#
# Guarded because `soot_plume` imports numpy at module scope and this module
# is imported from `urban_fire.RECIPES`; a host without numpy must still be
# able to import the ladder.
FIRE_LEVEL = "F5c"
try:
    from . import soot_plume as _spl
    _spl.DURATION_S.setdefault(FIRE_LEVEL, _spl.DURATION_S["F5"])
except Exception as _exc:                                    # pragma: no cover
    _spl = None
    print("[fire_collapse] WARNING: could not register {0} with soot_plume "
          "({1}); an {0} building will have NO fire events, so no soot and "
          "no flames".format(FIRE_LEVEL, _exc))


# ---------------------------------------------------------------------------
# Knobs
# ---------------------------------------------------------------------------
# The private seed. XOR'd into `soot_plume.event_seed(ctx)` so this recipe's
# draws are stable per building and independent of both the shared ladder rng
# and the events' own generator.
FIRE_COLLAPSE_SEED_XOR = 0xC011

# Roles that can be taken away. Same set `urban_fire.r_fire_collapse` sweeps,
# plus `balcony` (a cantilever off a wall that is no longer there has nothing
# to hang from).
SHELL_ROLES = ("wall", "corner", "parapet", "parapet_corner", "balcony")

# THE STAIRCASE. Share of the region's full width that is lost at the FAILURE
# LINE; it ramps to 1.0 at the top of the mass. 0.55 gives roughly two module
# steps over a five-storey band, which is what reads as a notch rather than a
# rectangle at 30 m. Setting this to 1.0 gives a rectangular hole, which is
# the artefact `monolith_damage.stepped_profile` exists to avoid.
PROFILE_FOOT = 0.55

# Share of the elevation lost at the TOP, drawn per building. Most of it: a
# wall that failed out of plane took the bays either side of the failure with
# it. Not ALL of it — a fire collapse that leaves no stub of the elevation
# standing at either end reads as a demolition, and the stub is what tells a
# viewer how tall the wall was.
SPAN_FRAC = (0.62, 0.86)
# ...and for the corner mode, per adjoining side, measured FROM the corner in
# MODULES, not as a share of the side. A share is what `SPAN_FRAC` is for and
# it does not describe a corner: 0.45 of `block_residential`'s 88 m south
# elevation is FORTY-ONE METRES of wall, which is half the building and 183
# modules to fracture (measured host-side before this was bounded).
# `quake_flow.r_corner_fail` makes the same call for the same reason and
# writes it as `max(4.0, module) * 1.6`; the fraction is kept only as a
# ceiling so the notch cannot eat a small building whole.
CORNER_REACH_MODULES = (1.3, 2.3)
CORNER_REACH_MAX_FRAC = 0.40

# HOW TALL THE LOST STRIP MAY BE. "From the origin storey up" is right on the
# 3-7 storey masonry stock this signature comes from — a wall that failed out
# of plane on a burnt-out block goes from the fire floor to the parapet. It is
# NOT right on a twenty-storey slab: a 60 m strip of elevation on the ground
# is a bombing, and the fire-collapse record has nothing like it (the Windsor
# tower and Plasco both lost their perimeter over the upper storeys with the
# frame below intact). So the band is capped from the TOP DOWN and the
# failure line rises to meet it — never the other way, because the failure
# line may never sink below the fire's own origin.
MAX_FALL_STOREYS = 6
# A LAST-RESORT BUDGET on how many kit modules may be fractured. Each one is
# a `_break` into 8-13 pieces plus a solidify, so this is the recipe's whole
# cost. Over budget, the failure line is raised a storey at a time (which is
# also the cheapest thing to look at: the notch gets shallower, not narrower,
# so the staircase profile survives).
MAX_MODULES = 110

# The windrow in the street. `depth_m` is its height at the wall line and
# `spread_frac` x the mass height is how far out it reaches (`quake_flow._heap`
# with `fill=False`). A whole elevation of masonry is a deeper windrow than a
# parapet: `r_out_of_plane` uses 1.0-1.9 m for exactly this event.
OUT_DEPTH_M = (1.15, 2.0)
OUT_SPREAD = (0.20, 0.34)
CORNER_DEPTH_M = (0.75, 1.45)
# The heap INSIDE, where the inner leaf and the broken floor edges landed. It
# is smaller than the one outside — the wall went out, not in — and it is what
# stops the exposed ground floor reading as a swept stage set.
IN_DEPTH_M = (2.2, 4.2)
IN_LUMP = (0.16, 0.62)

# How deep into the plan the "lost region" reaches, as a share of the mass's
# other dimension. This is what restricts the position sweep: everything
# static above the failure line AND inside this strip goes to the solver, and
# everything else — the far half of the roof deck, the plant on it, the walls
# on the untouched elevations — stays exactly where it was. Without the plan
# restriction the sweep is `r_fire_collapse`'s, which drops the whole roof.
REGION_DEPTH_FRAC = 0.26
REGION_DEPTH_MIN_M = 2.6

# Fracture: pieces per killed module, and how much of a kit shell's foil is
# consumed. Same shape as `r_fire_collapse`'s; the mass of the pile is the
# authored windrow, not the shells.
BREAK_PIECES = (8, 13)
BREAK_CONSUME = 0.26

# A wall rotating about its foot: the top leads. `quake_flow.r_out_of_plane`
# uses 0.4 + 2.4 z/H m/s and the bench settles with `max_speed=6.0`, so this
# never saturates the cap.
THROW_BASE = 0.40
THROW_TOP = 2.40

# ---------------------------------------------------------------------------
# THE PERIMETER OF THE HOLE (review round 4, 2026-08-30)
# ---------------------------------------------------------------------------
# The user, looking at `commercial_mid` F5c on the MCE bench: "some parts of
# it seem like they were directly cut off from the actual prims and therefore
# look like sharp straight or rectangular cuts". Every one of those lines is a
# KIT MODULE SEAM that no pass tore:
#
#   * the horizontal seam under each STAIRCASE STEP. The span widens by a
#     module or two per storey, so the module that survives at storey `s`
#     under a module that died at `s + 1` keeps a dead level top edge — and
#     there is one of those at every step of the profile, which is the whole
#     height of the notch.
#   * the horizontal seam UNDER the failure line. `_p_ragged_courses` was
#     called only inside the `reaches_end` branch, so on the common mid-wall
#     span nothing at all touched the storey below the hole.
#   * the vertical seams at the lower storeys of an edge that reaches the
#     corner at the TOP. `reaches_end` is evaluated at `top_storey` only, and
#     `_tear_edge` skipped that whole edge for every storey when it was true,
#     so the narrow lower part of the staircase kept factory ends.
#   * the return wall's near end when the loss stops one bay short of the
#     corner at some storeys and reaches it at others.
#
# The replacement (`plan_edges` -> `_tear_perimeter`) does not ask where the
# span was drawn at all. It asks, per SURVIVING module, "does a dead module
# touch me, and on which of my four edges" — so every edge of the hole is torn
# by construction, whatever shape the hole is, and the answer is checkable
# host-side with no stage (`test_fire_collapse.py`).
#
# How far into a surviving module the tear reaches, as a share of the module's
# own width (a vertical edge) or its own height (a horizontal one). The FAR
# portion always stays: `_break_split` returns statics as well as loose, and a
# module torn end to end is a module that was killed, not one that was torn.
EDGE_PEN = (0.25, 0.60)
# ...but a module whose BOTTOM edge is the hole is standing on what is left of
# its own foot. Take 0.6 of that and the storeys over it read as floating —
# the failure this whole recipe exists to avoid at the other end of the scale.
EDGE_PEN_ABOVE = (0.25, 0.40)
# Amplitude of the wandering break line, in metres, clamped to a third of the
# penetration so the line stays inside the module it is cutting.
EDGE_AMP_MAX = 0.55
EDGE_AMP_FRAC = 0.34
# Pieces per torn module, and the refinement budget. `_p_ragged_courses` uses
# `refine_max=4` on a wall band for the same reason: there are a lot of them.
EDGE_PIECES = (8, 11)
EDGE_REFINE_MAX = 5
# A return-wall module is "at the hole" when its NEAR end is within one module
# plus this much of the lost wall line — `quake_flow._ragged_neighbours` uses
# `depth_bays * max(4, module) + 1.0` for the same test.
RETURN_REACH_PAD_M = 1.0
# A LAST-RESORT BUDGET, the same shape as `MAX_MODULES`. Each edge tear is a
# `_break_split` (8-11 cells + refinement), so this is what the round-4 edge
# work costs on top of the kill loop. Over budget the jobs nearest the hole
# are kept and the count of what was dropped is reported — an under-torn edge
# is a bug, so it must be visible and not silent.
MAX_EDGE_MODULES = 200

# HOW BIG A GAP STILL COUNTS AS "TOUCHING", ON A SLICED BUILDING ONLY
# (review fire_dtc3, 2026-08-30).
#
# `plan_edges`' adjacency test is `-0.25 w <= (dead_lo - live_hi) <= tol` with
# `tol` at 0.6 m, which is exactly right for a KIT wall: `urban_building.
# PIECES` gives a module its modelled panel extent and consecutive modules
# butt — measured on `apartment` F5c, every neighbour of the hole sits
# 0.07-0.09 m from it (`tools/tear_edge_probe.py`, table B).
#
# A SLICED piece is not a panel. It is the BOUNDING BOX OF A REGION CUT
# (`gac_storey_slice._ring` -> `slice_to_kit`, whose placement carries
# `_size` = the cut cell's world extent and whose `PIECES` row
# `gac_slice.register_style` writes as `(sx, sy, sz, -sx/2, -sy/2, 0)`), so
# it swallows whatever sills, cornices, reveals and balcony returns fall in
# that cell — and consecutive cells' boxes therefore do NOT butt. Measured on
# `gac_SM_Building_02_F5c_s193`, side S:
#
#     pier_S_0   [ 0.82,  3.89]      wall_S_1 starts  4.08   gap  0.19
#     wall_S_1   [ 4.08,  9.75]      pier_S_2 starts  9.23   OVERLAP 0.52
#     pier_S_2   [ 9.23, 12.30]      pier_S_3 starts 12.34   gap  0.04
#     pier_S_3   [12.34, 15.42]      wall_S_4 starts 15.80   gap  0.38
#                                    ...at storey 8: 16.40   gap  0.98
#
# The same two columns gap by 0.38 m at one storey and 0.98 m at the next, so
# a fixed 0.6 m tolerance tears the boundary piece at storey 7 and silently
# misses its twin at storey 8. That is `pier_S_3_09_0102` — one of the two
# prims the user named — left with a factory slice seam on the lip of the
# hole. So on a sliced piece the tolerance is scaled by the piece's OWN
# width (a 3.07 m pier gets 1.38 m) and capped, and on a kit module nothing
# changes at all: `is_sliced` is False, `tol` stays 0.6 m, and the frozen MCE
# ladder draws exactly what it drew yesterday.
EDGE_GAP_FRAC = 0.45
EDGE_GAP_MAX_M = 1.6

# ---------------------------------------------------------------------------
# THE BURN ZONE (same review): "there are parts of the surface that look
# pristine. Any parts directly near where the building collapsed (up, left,
# down, right, anything) would have been flamed and scorched".
# ---------------------------------------------------------------------------
# A collapse opens the compartment: the fire that was venting through windows
# is now venting through a hole the size of the wall, and everything round its
# lip has had flame directly on it. That is a SKIN job, not a per-module bind
# — bug 5 of `build-urban-fire-scenes` is exactly the failure mode of a flat
# dark bind per module ("a rectangle BY CONSTRUCTION, however good the texture
# on it is"). `r_partial_collapse` writes the zone into `ctx["fire"]
# ["burn_zone"]` in `soot_plume.side_u` coordinates and `soot_plume.skin`
# raises the deposit's alpha inside it with a soft, noise-wandered edge.
#
# How far past the hole the flame reached, in metres. Sideways is about one
# module; up is the plume, which climbs; down is the little that rolls under
# the lip.
BURN_ZONE_PAD_U = 4.0
BURN_ZONE_PAD_UP = 3.0
BURN_ZONE_PAD_DOWN = 1.5


def _f(name, default):
    """An override from the environment, for a bench run that wants to push
    one number without a code change."""
    v = (os.environ.get(name) or "").strip()
    try:
        return float(v) if v else float(default)
    except ValueError:
        return float(default)


# ---------------------------------------------------------------------------
# Pure geometry — no pxr, no stage, no ctx["stage"]
# ---------------------------------------------------------------------------
_LOCAL_NORMAL = {"S": (0.0, -1.0), "N": (0.0, 1.0),
                 "E": (1.0, 0.0), "W": (-1.0, 0.0)}


def side_length(m, side):
    """Metres along the mass's `side` wall."""
    return float(m["W"] if side in ("S", "N") else m["D"])


def side_depth(m, side):
    """Metres across the mass, perpendicular to `side`."""
    return float(m["D"] if side in ("S", "N") else m["W"])


def outward_of(m, side, lx, ly):
    """Signed metres beyond the mass's `side` wall line, in the mass's own
    local frame: POSITIVE is outside the building (in the street), negative
    is inside it.

    This is the whole test behind "the heap is where a wall that fell outward
    lands". `quake_flow._heap(fill=False)` places its windrow chunks at
    `-D/2 - d` for the S side and the mirror of that for the others, so a
    correctly-planned outside heap has a strictly positive value here and the
    inside heap a strictly negative one.
    """
    W, D = float(m["W"]), float(m["D"])
    if side == "S":
        return -(D / 2.0) - float(ly)
    if side == "N":
        return float(ly) - D / 2.0
    if side == "W":
        return -(W / 2.0) - float(lx)
    return float(lx) - W / 2.0


def along_of(m, side, lx, ly):
    """Metres along the `side` wall from its low end, 0..side_length."""
    return ((float(lx) + m["W"] / 2.0) if side in ("S", "N")
            else (float(ly) + m["D"] / 2.0))


def wall_point(m, side, t, out_m=0.0):
    """Local (lx, ly) `t` metres along the `side` wall line, displaced
    `out_m` metres OUTWARD (negative = into the building)."""
    from . import quake_flow as qf
    lx, ly = qf._p_wall_point(m, side, float(t))
    nx, ny = _LOCAL_NORMAL[side]
    return (lx + nx * float(out_m), ly + ny * float(out_m))


def shared_corner(sides):
    """The corner two elevations share, e.g. ('S', 'E') -> 'SE'. None when
    they are opposite (there is no corner between S and N) or there is only
    one of them."""
    ns = [s for s in sides if s in ("S", "N")]
    ew = [s for s in sides if s in ("E", "W")]
    if ns and ew:
        return ns[0] + ew[0]
    return None


def corner_sides(corner):
    return (("S" if "S" in corner else "N"), ("E" if "E" in corner else "W"))


def corner_at_high_end(side, corner):
    """Is `corner` at the HIGH end of `side`'s along-coordinate?

    `along_of` runs with +x on the S/N walls and +y on the E/W walls, so the
    E end is high on S/N and the N end is high on E/W — the same convention
    `quake_flow.r_corner_fail` states for its own windrow spans.
    """
    return (("E" in corner) if side in ("S", "N") else ("N" in corner))


def corner_of_end(side, low_end):
    """The building corner at one END of `side`'s along-coordinate.

    `along_of` runs with +x on S/N and +y on E/W (`quake_flow._p_el_t`'s own
    convention), so the LOW end of S is its west end and the low end of E is
    its south end. This is `corner_at_high_end` read the other way round, and
    the two are checked against each other in the tests.
    """
    if side in ("S", "N"):
        return side + ("W" if low_end else "E")
    return ("S" if low_end else "N") + side


def other_side(corner, side):
    """The OTHER elevation that meets `side` at `corner`."""
    a, b = corner_sides(corner)
    return b if a == side else a


def is_sliced(e):
    """Is this element a piece a WHOLE-ASSET building was sliced into?

    A GreatAmericanCity / downtowncity block reaches the fire ladder as kit
    placements cut out of one merged mesh (`detail/gac_storey_slice.
    slice_to_kit`, `detail/gac_slice.slice_building`), and those two are the
    only producers in the tree that stamp `_role` on a placement and address
    it with a `slice://` / `gacslice://` url. Everything else — the
    ModernCityEnvironment kit, the AEC brownstones — is a modelled module
    with a real `urban_building.PIECES` row.

    The distinction matters exactly once, in `plan_edges`: a modelled module's
    footprint is its panel and butts its neighbour, a sliced piece's footprint
    is the bbox of a region cut and does not. See `EDGE_GAP_FRAC`.
    """
    p = e.get("p") or {}
    if p.get("_role"):
        return True
    return str(p.get("usd", "")).startswith(("slice://", "gacslice://"))


def edge_gap_tol(e, w, tol):
    """How far a dead module may sit from this one and still be "against" it.

    `tol` on a modelled kit module (they butt); scaled by the piece's own
    width, capped, on a sliced one (their bboxes do not). fire_dtc3.
    """
    if not is_sliced(e):
        return float(tol)
    return float(max(tol, min(EDGE_GAP_MAX_M, EDGE_GAP_FRAC * float(w))))


def el_footprint(m, e):
    """The four corners of a placed module's PLAN footprint, in the mass's
    local frame.

    MEASURED FROM `urban_building.PIECES`, not from `quake_flow._piece_frame`.
    The frame is a LINE — an origin, a yaw and a width — which describes a
    flat wall panel exactly and a CORNER BLOCK not at all: a 6 x 6 m
    `SkyscraperCorner_B` placed at yaw 90 on the south wall projects onto that
    wall as a single POINT through the frame, so it looked like a zero-width
    module that touched nothing, and every adjacency test against it silently
    returned "no neighbour" (measured, `highrise_step` storey 8: the corner
    block at t = 25.0 .. 25.0 on a 25 m wall). Same for a `dw` pillar, whose
    frame axis runs INTO the building. The measured bbox has no such blind
    spot — it is what the piece actually occupies.
    """
    from detail import urban_building as ub
    from . import quake_flow as qf

    meas = ub.PIECES.get(e["name"])
    a = math.radians(float(e.get("yaw", 0.0)))
    ca, sa = math.cos(a), math.sin(a)
    if meas:
        sx, sy, _sz, xmin, ymin, _zmin = meas
        box = ((xmin, ymin), (xmin + sx, ymin),
               (xmin + sx, ymin + sy), (xmin, ymin + sy))
    else:                                                     # pragma: no cover
        w = max(4.0, float(m.get("module") or 4.0))
        box = ((0.0, 0.0), (w, 0.0), (w, 0.4), (0.0, 0.4))
    out = []
    for px, py in box:
        wx = float(e["x"]) + ca * px - sa * py
        wy = float(e["y"]) + sa * px + ca * py
        out.append(qf._to_local(m, wx, wy))
    return out


def el_span(m, e, side=None):
    """(t0, t1) of a placed module in metres along `side` (its own by
    default), from that wall's low end — the module's whole footprint, both
    ends projected. See `el_footprint` for why this is not `_p_el_t` plus a
    width."""
    sd = side or e["side"]
    ts = [along_of(m, sd, lx, ly) for lx, ly in el_footprint(m, e)]
    return min(ts), max(ts)


def el_z_span(m, e):
    """(z_bottom, z_top) of a placed module in world metres."""
    from detail import urban_building as ub

    za = float(e.get("z", m["z0"]))
    meas = ub.PIECES.get(e["name"])
    if meas and float(meas[2]) > 0.05:
        return za, za + float(meas[2])
    lv = list(m["levels"])
    st = int(e.get("storey", 0))
    zb = float(lv[st + 1]) if st + 1 < len(lv) else float(m["top"])
    return za, max(za + 0.3, zb)


def el_near_far(m, e, side):
    """(near, far) distance in metres of a module's footprint from the `side`
    wall line, measured INTO the building. Used to find the return-wall bays
    at a corner the loss reaches: on a wall perpendicular to `side` the
    distance grows away from the corner, so "near" IS "at the corner"."""
    ds = [-outward_of(m, side, lx, ly) for lx, ly in el_footprint(m, e)]
    return min(ds), max(ds)


def _u_of_t(m, side, t):
    """`along_of` metres -> `soot_plume.side_u` metres.

    The two agree on S and E and are MIRRORED on N and W: `side_u` unwraps the
    perimeter counter-clockwise (S -> E -> N -> W) while `along_of` always runs
    with +x / +y. Getting this wrong puts the burn zone on the far end of the
    wall, which is the one failure a picture would not obviously show.
    """
    L = side_length(m, side)
    return float(t) if side in ("S", "E") else float(L - t)


def burn_zone_rects(plan, m, pad_u=None, pad_up=None, pad_down=None):
    """The `(side, u0, u1, z0, z1)` rectangles, in `soot_plume.side_u`
    coordinates, of everything the collapse put flame on.

    One rectangle PER STOREY per lost elevation, so the zone follows the
    staircase instead of boxing it — a single rectangle over the whole notch
    would put heavy soot on the corner of wall the profile deliberately left
    standing. Plus the return face of any corner the loss actually reaches.
    """
    pad_u = BURN_ZONE_PAD_U if pad_u is None else float(pad_u)
    pad_up = BURN_ZONE_PAD_UP if pad_up is None else float(pad_up)
    pad_down = BURN_ZONE_PAD_DOWN if pad_down is None else float(pad_down)
    lv = list(m["levels"])
    top = float(m["top"])

    def z_of(s):
        za = float(lv[s]) if s < len(lv) else top
        zb = float(lv[s + 1]) if s + 1 < len(lv) else top
        return za, max(za + 0.5, zb)

    rects = []
    z_lo_all, z_hi_all = z_of(plan["storeys"][0])[0], z_of(plan["top_storey"])[1]
    for sd in plan["sides"]:
        L = side_length(m, sd)
        for s in plan["storeys"]:
            t0, t1 = plan["span"][(sd, min(s, plan["top_storey"]))]
            za, zb = z_of(s)
            a = max(0.0, t0 - pad_u)
            b = min(L, t1 + pad_u)
            u0, u1 = sorted((_u_of_t(m, sd, a), _u_of_t(m, sd, b)))
            rects.append((sd, float(u0), float(u1),
                          float(za - pad_down), float(zb + pad_up)))
        # THE RETURN FACE. A wall that peeled off at a corner took the flame
        # round it; the first bay of the adjoining elevation is inside the
        # opening as far as the fire is concerned.
        for low_end, reached in zip((True, False), plan["reaches_end"][sd]):
            if not reached:
                continue
            c = corner_of_end(sd, low_end)
            sd2 = other_side(c, sd)
            L2 = side_length(m, sd2)
            if corner_at_high_end(sd2, c):
                a2, b2 = max(0.0, L2 - pad_u), L2
            else:
                a2, b2 = 0.0, min(L2, pad_u)
            u0, u1 = sorted((_u_of_t(m, sd2, a2), _u_of_t(m, sd2, b2)))
            rects.append((sd2, float(u0), float(u1),
                          float(z_lo_all - pad_down), float(z_hi_all + pad_up)))
    return rects


def _private_rng(ctx):
    """`(random.Random, seed)` for this recipe, from the building itself."""
    seed = None
    if _spl is not None:
        try:
            seed = _spl.event_seed(ctx) ^ FIRE_COLLAPSE_SEED_XOR
        except Exception:
            seed = None
    if seed is None:                                          # pragma: no cover
        f = ctx.get("fire") or {}
        key = "{0}|{1}|{2}|{3}".format(ctx.get("tag", ""), f.get("level"),
                                       f.get("origin"),
                                       "".join(f.get("sides", ())))
        seed = abs(hash(key)) ^ FIRE_COLLAPSE_SEED_XOR
    return random.Random(seed), seed


# ---------------------------------------------------------------------------
# THE PLAN — every decision, host-side
# ---------------------------------------------------------------------------
def plan_partial_collapse(ctx, mode="elevation", side=None, corner=None,
                          from_storey=None, span_frac=None, depth_m=None,
                          drop_slabs=1, mass=None, max_storeys=None,
                          max_modules=None):
    """Decide what comes down, and where it lands. No USD.

    Returns a dict:

      mode        "elevation" | "corner"
      mass        mass tag the failure is in (the fire's own mass)
      sides       the elevation(s) lost — a 1-tuple, or the corner's two
      keep_sides  the elevations that are NOT touched at all
      corner      "SE" etc, or None
      s0          the failure line: the lowest storey that comes down. NEVER
                  below `ctx["fire"]["origin"]` — the clean band under the
                  fire is the signature and a collapse does not eat into it.
      storeys     [s0 .. top of the mass]
      span        {(side, storey): (t0_m, t1_m)} — the staircase, in metres
                  along that wall from its low end
      kill        the element records that come away (they are NOT marked
                  dead here; `r_partial_collapse` does that as it breaks them)
      region      {side: (t0_m, t1_m, depth_m)} — the plan footprint of the
                  loss, used to restrict the position sweep
      heaps       [{where, side, along_m, centre_local, centre_world, ...}]
                  `where` is "outside" (the street windrow) or "inside"
      drop        [(mass, storey)] — fit-out slabs dropped whole
      cut_z       world z above which a static prim inside `region` falls
      seed        the private seed everything above was drawn from
    """
    from . import quake_flow as qf

    f = ctx["fire"]
    mtag = mass or f["mass"]
    m = ctx["info"]["masses"].get(mtag) or ctx["info"]["masses"]["main"]
    n_lv = max(1, len(m["levels"]))
    prng, seed = _private_rng(ctx)
    fire_sides = list(f.get("sides") or ("S",))

    # ---- 1. which elevation(s) -------------------------------------------
    if mode == "auto":
        c = shared_corner(fire_sides)
        mode = "corner" if (c and prng.random() < 0.45) else "elevation"
    if mode not in ("elevation", "corner"):
        raise ValueError("fire_collapse: unknown mode " + repr(mode))
    if mode == "corner":
        corner = corner or shared_corner(fire_sides)
        if not corner:
            # The fire is venting on one elevation (or on two opposite ones).
            # A corner still fails at the end of the burning wall — take the
            # neighbour the fire has bled round (`urban_fire._side_neighbors`,
            # the one-hop rule) rather than inventing a cold corner.
            from . import urban_fire as uf
            base = side or fire_sides[0]
            nb = uf._side_neighbors(base)
            corner = shared_corner((base, nb[prng.randrange(2)]))
        sides = corner_sides(corner)
    else:
        # THE ELEVATION THE BENCH PHOTOGRAPHS. `urban_fire_bench`'s `face` /
        # `close` cameras are aimed at `fire["sides"][0]`, so putting the
        # collapse anywhere else guarantees the one view that could show it
        # looks at an intact wall.
        sides = (side or fire_sides[0],)
        corner = None
    keep_sides = tuple(s for s in ("S", "E", "N", "W") if s not in sides)

    # ---- 2. the failure line ---------------------------------------------
    # NOTHING BELOW THE FIRE'S ORIGIN, EVER. Everything else here is a
    # preference; this is the invariant.
    floor = int(f["origin"])
    if from_storey is not None:
        s0 = max(floor, int(from_storey))
    else:
        # A wall that failed at grade takes the building with it, which is
        # the F5/F6 case and not this one — so on anything with three or more
        # storeys the peel starts at the first floor at the lowest, the same
        # `from_storey=1` default `quake_flow.r_out_of_plane` uses.
        s0 = max(floor, 1 if n_lv >= 3 else 0)
    s0 = max(0, min(s0, n_lv - 1))
    # THE BAND IS CAPPED FROM THE TOP DOWN. See `MAX_FALL_STOREYS`.
    n_max = int(max_storeys) if max_storeys else int(MAX_FALL_STOREYS)
    s0 = max(s0, n_lv - max(1, n_max))
    s0 = max(0, min(s0, n_lv - 1))
    # ...but a one-storey loss on a tall block is a hole, not a collapse.
    if n_lv - s0 < 2 and s0 > floor:
        s0 -= 1
    top_i = max(s0, n_lv - 1)

    # ---- 3. the staircase, and 4. what comes away -------------------------
    base = (float(span_frac) if span_frac is not None
            else prng.uniform(*SPAN_FRAC))
    reach_m = (prng.uniform(*CORNER_REACH_MODULES)
               * max(4.0, float(m.get("module") or 4.0)))
    centres = {}
    for sd in sides:
        L = side_length(m, sd)
        # centred, wandering off centre a little so a row of buildings does
        # not lose the middle of every wall. Anchored at the corner instead
        # in corner mode.
        centres[sd] = (0.5 * L + prng.uniform(-0.14, 0.14) * L
                       if mode == "elevation" else None)

    # Half a module of tolerance on each end: a kit piece is pivoted at one
    # end (`quake_flow._p_el_t` returns exactly that end), so a piece whose
    # pivot sits just outside the span can still have most of its body
    # inside it. `_ragged_neighbours` then tears the near ends of whatever
    # survives, so the vertical edges of the hole are not module seams.
    pad = 0.5 * max(4.0, float(m.get("module") or 4.0))

    def _spans(lo_storey):
        out = {}
        for sd in sides:
            L = side_length(m, sd)
            at_hi = corner_at_high_end(sd, corner) if corner else None
            for s in range(lo_storey, n_lv):
                k = (0.0 if top_i == lo_storey
                     else (s - lo_storey) / float(top_i - lo_storey))
                ramp = PROFILE_FOOT + (1.0 - PROFILE_FOOT) * k
                if mode == "elevation":
                    w = min(L, base * ramp * L)
                    c = centres[sd]
                    t0 = max(0.0, min(c - w / 2.0, L - w))
                    t1 = min(L, t0 + w)
                else:
                    w = min(reach_m * ramp, CORNER_REACH_MAX_FRAC * L)
                    t0, t1 = ((L - w, L) if at_hi else (0.0, w))
                out[(sd, s)] = (float(t0), float(t1))
        return out

    def _kill(lo_storey, sp):
        out = []
        for e in qf._els(ctx, mass=mtag, role=SHELL_ROLES):
            if e["side"] not in sides:
                continue
            s = int(e["storey"])
            if s < lo_storey:
                continue
            lo, hi = sp[(e["side"], min(s, top_i))]
            t = qf._p_el_t(m, e["side"], e)
            if lo - pad <= t <= hi + pad:
                out.append(e)
        return out

    budget = int(max_modules) if max_modules else int(MAX_MODULES)
    span = _spans(s0)
    kill = _kill(s0, span)
    trimmed = 0
    while len(kill) > budget and s0 < top_i and s0 + 1 <= top_i - 1:
        s0 += 1
        trimmed += 1
        span = _spans(s0)
        kill = _kill(s0, span)
    storeys = list(range(s0, n_lv))

    # DOES THE LOSS REACH A CORNER? It decides which machinery rags which
    # edge, and getting it wrong tears a return wall next to an intact bay.
    # `quake_flow._ragged_neighbours` only ever touches the OTHER two
    # elevations (it skips `e["side"] == side` by construction), which is
    # right when a whole elevation has gone — `r_out_of_plane`'s case — and
    # wrong for a span that stops in the middle of the wall, where both
    # vertical edges of the hole are on the LOST elevation itself.
    reaches_end = {}
    for sd in sides:
        L = side_length(m, sd)
        lo, hi = span[(sd, top_i)]
        reaches_end[sd] = (lo <= pad, hi >= L - pad)

    # ---- 5. the plan footprint of the loss --------------------------------
    dep_default = max(REGION_DEPTH_MIN_M,
                      REGION_DEPTH_FRAC * min(float(m["W"]), float(m["D"])))
    region = {}
    for sd in sides:
        lo, hi = span[(sd, top_i)]
        region[sd] = (float(lo), float(hi),
                      float(depth_m) if depth_m is not None else dep_default)

    # ---- 6. the two heaps --------------------------------------------------
    Hm = max(3.0, float(m["top"]) - float(m["z0"]))
    heaps = []
    for sd in sides:
        lo, hi = span[(sd, top_i)]
        mid = 0.5 * (lo + hi)
        spread = prng.uniform(*OUT_SPREAD)
        depth = prng.uniform(*(OUT_DEPTH_M if mode == "elevation"
                               else CORNER_DEPTH_M))
        reach = max(1.5, spread * Hm)
        # `_heap`'s windrow draws its distance from the wall as |gauss(0,
        # 0.45 reach)| + 0.15, so the mass of it sits about 0.45 reach out.
        lx, ly = wall_point(m, sd, mid, out_m=0.45 * reach)
        wx, wy = qf._to_world(m, lx, ly)
        L = side_length(m, sd)
        heaps.append({
            "where": "outside", "side": sd,
            "along_m": (lo, hi),
            # `_heap` wants FRACTIONS of the side, -0.5..0.5
            "along": (lo / L - 0.5, hi / L - 0.5),
            "spread_frac": spread, "depth_m": depth,
            "base_z": float(m["z0"]),
            "centre_local": (lx, ly), "centre_world": (wx, wy),
            "outward_m": outward_of(m, sd, lx, ly),
        })
    # ONE inside heap, on the widest of the lost runs. Smaller: the wall went
    # out. What is inside is the inner leaf, the plaster and the broken ends
    # of the floors that used to bear on it.
    sd_in = max(sides, key=lambda q: span[(q, top_i)][1] - span[(q, top_i)][0])
    lo, hi = span[(sd_in, top_i)]
    inner = prng.uniform(*IN_DEPTH_M)
    lx, ly = wall_point(m, sd_in, 0.5 * (lo + hi), out_m=-0.45 * inner)
    wx, wy = qf._to_world(m, lx, ly)
    heaps.append({
        "where": "inside", "side": sd_in, "along_m": (lo, hi),
        "depth_m": inner, "base_z": float(m["z0"]),
        "centre_local": (lx, ly), "centre_world": (wx, wy),
        "outward_m": outward_of(m, sd_in, lx, ly),
    })

    # ---- 7. the floors behind it -------------------------------------------
    # The topmost slab in the band lost its bearing wall on this side and came
    # down onto the one below; the rest of the band keeps its slab and loses a
    # ragged strip along the open edge (`_ragged_slabs`) and droops into it
    # (`r_droop`). Dropping every slab in the band is `r_fire_collapse`'s job
    # at F5 and reads as the whole building letting go.
    # ...and ONLY in elevation mode. A corner that failed takes the CORNER
    # of each slab (which `_ragged_slabs` breaks along the open edge), not
    # the whole floor plate: sending a 88 x 16 m slab to the solver because
    # a 7 m corner let go is `r_fire_collapse` wearing this recipe's name.
    n_drop = max(0, int(drop_slabs)) if mode == "elevation" else 0
    drop = ([(mtag, s) for s in storeys[-n_drop:]]
            if (n_drop and len(storeys) >= 2) else [])

    cut_z = float(m["levels"][s0]) - 0.4
    plan = {"mode": mode, "mass": mtag, "sides": tuple(sides),
            "keep_sides": keep_sides, "corner": corner, "s0": int(s0),
            "storeys": storeys, "span": span, "span_frac": base,
            "kill": kill, "region": region, "heaps": heaps, "drop": drop,
            "cut_z": cut_z, "seed": seed, "n_levels": n_lv,
            "top_storey": top_i, "trimmed_storeys": trimmed,
            "corner_reach_m": reach_m, "reaches_end": reaches_end,
            "pad_m": pad}
    # WHAT THE FIRE PUT FLAME ON WHEN THE WALL WENT. Pure geometry, so it is
    # part of the plan and not of the authoring; `r_partial_collapse` hands it
    # to `ctx["fire"]["burn_zone"]` and `soot_plume.skin` reads it there.
    plan["burn_zone"] = burn_zone_rects(plan, m)
    return plan


def region_side(plan, m, lx, ly, pad=1.0):
    """WHICH lost elevation's plan footprint the local point is in, or None.

    Same test `in_region` answers yes/no to; the caller that has to throw a
    piece needs to know which wall it was behind, so the outward direction
    it is thrown in is the one the wall actually fell in.
    """
    for sd, (lo, hi, dep) in plan["region"].items():
        t = along_of(m, sd, lx, ly)
        d_in = -outward_of(m, sd, lx, ly)          # metres INSIDE the wall
        if lo - pad <= t <= hi + pad and -1.5 <= d_in <= dep + pad:
            return sd
    return None


def in_region(plan, m, lx, ly, pad=1.0):
    """Is the local point inside the plan footprint of the loss?

    Used by the position sweep. `-1.5` on the inner bound lets a piece that
    is already proud of the wall (a cornice, a sooted overlay quad) count as
    part of the lost strip rather than as street furniture.
    """
    return region_side(plan, m, lx, ly, pad=pad) is not None


EDGE_CLASSES = ("above", "below", "left", "right", "return")


def plan_edges(ctx, plan, m, prng, tol=0.6, budget=None):
    """Every SURVIVING module that touches the hole, and which of its own
    edges the hole is on. Pure geometry: no `pxr`, no stage, no fracture.

    THIS IS THE ANSWER TO "SHARP STRAIGHT OR RECTANGULAR CUTS". The old code
    asked where the SPAN was drawn and tore one bay at each of its two
    vertical edges (`_tear_edge`) plus, on a corner loss only, the return
    walls and the courses under the failure line. Everything else round the
    hole kept a factory edge: the horizontal seam under every staircase step,
    the seam under the failure line on a mid-wall span, and the vertical seams
    at the lower storeys of an edge that only reaches the corner at the top.

    This asks the opposite question — one per surviving module, "is a dead
    module against me, and on which side" — so the hole's whole perimeter is
    covered whatever shape it has:

      left    a dead module begins where this one ENDS (the hole is at higher
              t): the tear takes this module's high end
      right   a dead module ends where this one BEGINS: its low end goes
      below   a dead module sits directly OVER it: its top edge goes. This is
              the staircase tread and the course under the failure line, and
              it is the single biggest source of straight lines in the notch.
      above   a dead module sits directly UNDER it: its foot goes — but only
              0.25-0.40 of it (`EDGE_PEN_ABOVE`), because that foot is what is
              holding whatever is over it and a building must not read as
              floating.
      return  the module is on the ADJOINING elevation, within one bay of the
              lost wall line, at a storey where the loss actually reaches that
              corner. Its near end goes.

    A module can be in several classes at once (the corner of a staircase step
    is `left` and `below` together); the caller unions their judges and breaks
    it ONCE, because `_break_split` deactivates the source prim.

    `tol` is how big a gap between two footprints still counts as touching.
    It is a CONSTANT on a modelled kit module and SCALED BY THE PIECE on a
    sliced one (`edge_gap_tol`), because a sliced piece's footprint is the
    bbox of a region cut and consecutive cells do not butt — the fire_dtc3
    miss, see `EDGE_GAP_FRAC`.

    Returns a list of job dicts, sorted stably, each with numeric break lines
    already drawn from `prng` — so the geometry is assertable host-side and
    `_tear_perimeter` is only the USD authoring of it.
    """
    killed = set(id(e) for e in plan["kill"])
    mtag = plan["mass"]
    lost = set(plan["sides"])
    dead = {}
    for e in plan["kill"]:
        dead.setdefault((e["side"], int(e["storey"])), []).append(el_span(m, e))

    def live(side=None, storey=None):
        for e in ctx["info"]["elements"]:
            if e["mass"] != mtag or e["role"] not in SHELL_ROLES:
                continue
            if e.get("dead") or id(e) in killed:
                continue
            if side is not None and e["side"] != side:
                continue
            if storey is not None and int(e["storey"]) != storey:
                continue
            yield e

    jobs = {}

    def _job(e):
        j = jobs.get(id(e))
        if j is None:
            t0, t1 = el_span(m, e)
            za, zb = el_z_span(m, e)
            j = {"el": e, "name": e.get("name"), "side": e["side"],
                 "storey": int(e["storey"]), "t0": t0, "t1": t1,
                 "za": za, "zb": zb, "w": max(0.3, t1 - t0),
                 "h": max(0.3, zb - za), "classes": [], "cuts": []}
            jobs[id(e)] = j
        return j

    # ---- the lost elevation itself: left / right / below / above ---------
    for sd in lost:
        for e in live(side=sd):
            s = int(e["storey"])
            j = None
            t0, t1 = el_span(m, e)
            w = max(0.3, t1 - t0)
            etol = edge_gap_tol(e, w, tol)
            for a, b in dead.get((sd, s), ()):
                if -0.25 * w <= (a - t1) <= etol:         # hole at higher t
                    j = j or _job(e)
                    if "left" not in j["classes"]:
                        j["classes"].append("left")
                if -0.25 * w <= (t0 - b) <= etol:         # hole at lower t
                    j = j or _job(e)
                    if "right" not in j["classes"]:
                        j["classes"].append("right")
            over = min(1.2, 0.3 * w)
            for key, cls in ((s + 1, "below"), (s - 1, "above")):
                for a, b in dead.get((sd, key), ()):
                    if min(b, t1) - max(a, t0) > over:
                        j = j or _job(e)
                        if cls not in j["classes"]:
                            j["classes"].append(cls)

    # ---- the return walls, at a corner the loss actually reaches ---------
    # PER STOREY, not from `reaches_end` (which is the top storey's answer):
    # the staircase means a span can reach the corner at the top and stop two
    # bays short at the failure line, and the bays in between are exactly the
    # ones that kept a straight end.
    for sd in lost:
        L = side_length(m, sd)
        reach = float(m["D"] if sd in ("S", "N") else m["W"])
        for s in plan["storeys"]:
            iv = dead.get((sd, s)) or []
            if not iv:
                continue
            ends = ((True, min(a for a, _b in iv) <= plan["pad_m"]),
                    (False, max(b for _a, b in iv) >= L - plan["pad_m"]))
            for low_end, hit in ends:
                if not hit:
                    continue
                sd2 = other_side(corner_of_end(sd, low_end), sd)
                if sd2 in lost:
                    continue        # it is a lost elevation; it has its own edges
                for e in live(side=sd2, storey=s):
                    near, _far = el_near_far(m, e, sd)
                    ew = max(0.3, el_span(m, e)[1] - el_span(m, e)[0])
                    if near > ew + RETURN_REACH_PAD_M:
                        continue
                    j = _job(e)
                    if "return" not in j["classes"]:
                        j["classes"].append("return")
                    j.setdefault("toward", []).append(
                        {"side": sd, "near": float(max(0.0, near)),
                         "reach": reach})

    # ---- draw the break lines -------------------------------------------
    out = []
    for j in sorted(jobs.values(), key=lambda q: (q["side"], q["storey"],
                                                  q["t0"], q["name"] or "")):
        j["classes"] = [c for c in EDGE_CLASSES if c in j["classes"]]
        for cls in j["classes"]:
            if cls in ("left", "right"):
                pen = prng.uniform(*EDGE_PEN) * j["w"]
                line = (j["t1"] - pen) if cls == "left" else (j["t0"] + pen)
                j["cuts"].append({"cls": cls, "kind": "v", "pen": pen,
                                  "line": line, "loose_hi": cls == "left",
                                  "amp": min(EDGE_AMP_MAX,
                                             EDGE_AMP_FRAC * pen)})
            elif cls in ("below", "above"):
                lo, hi = (EDGE_PEN if cls == "below" else EDGE_PEN_ABOVE)
                pen = prng.uniform(lo, hi) * j["h"]
                line = (j["zb"] - pen) if cls == "below" else (j["za"] + pen)
                j["cuts"].append({"cls": cls, "kind": "z", "pen": pen,
                                  "line": line, "loose_above": cls == "below",
                                  "amp": min(EDGE_AMP_MAX,
                                             EDGE_AMP_FRAC * pen)})
            else:
                for tw in j.get("toward", ()):
                    pen = prng.uniform(*EDGE_PEN) * j["w"]
                    frac = min(0.9, (tw["near"] + pen) / max(1e-6, tw["reach"]))
                    j["cuts"].append({"cls": cls, "kind": "t", "pen": pen,
                                      "side": tw["side"], "frac": frac,
                                      "near": tw["near"]})
        out.append(j)

    lim = int(MAX_EDGE_MODULES if budget is None else budget)
    if len(out) > lim:
        rank = {c: i for i, c in enumerate(("left", "right", "below", "above",
                                            "return"))}
        out.sort(key=lambda q: (min(rank[c] for c in q["classes"]),
                                q["storey"], q["side"], q["t0"]))
        for j in out[lim:]:
            j["dropped"] = True
    return out


def edge_census(jobs):
    """{class: [n_neighbours, n_torn]} — what the probe prints and what the
    tests assert 100 % on. A module in two classes counts in both."""
    out = dict((c, [0, 0]) for c in EDGE_CLASSES)
    for j in jobs:
        for c in j["classes"]:
            out[c][0] += 1
            if j.get("torn"):
                out[c][1] += 1
    return out


def describe(plan):
    """One line, for `ctx["notes"]` and for a host-side dry run."""
    heaps = ", ".join("{0} heap on {1} at {2:+.1f} m of the wall line".format(
        h["where"], h["side"], h["outward_m"]) for h in plan["heaps"])
    width = ("{0:.0f} % of the elevation at the top".format(
        100.0 * plan["span_frac"]) if plan["mode"] == "elevation"
        else "{0:.1f} m of each wall at the top".format(plan["corner_reach_m"]))
    return ("partial collapse ({0}{1}): {2} lost from storey {3} up "
            "({4} storey(s){5}, {6}), {7} module(s) taken, {8} slab(s) "
            "dropped whole; {9}".format(
                plan["mode"],
                ", corner " + plan["corner"] if plan["corner"] else "",
                "/".join(plan["sides"]), plan["s0"], len(plan["storeys"]),
                ", band trimmed {0} storey(s) to the module budget".format(
                    plan["trimmed_storeys"]) if plan["trimmed_storeys"] else "",
                width, len(plan["kill"]), len(plan["drop"]), heaps))


# ---------------------------------------------------------------------------
# The rng swap
# ---------------------------------------------------------------------------
class _own_rng(object):
    """Install this recipe's private generators on the ctx for a block.

    `quake_flow._heap`, `_ragged_slabs`, `_ragged_neighbours`, `r_droop` and
    `urban_fire._debris_mat` / `_joist_stubs` all read `ctx["rng"]` (and
    `ctx["nrng"]`) off the ctx and take no generator argument. Handing them a
    private pair by swapping the ctx entries — and restoring in a `finally` —
    is what lets this recipe be inserted anywhere in a ladder without moving
    any other recipe's draws. See the module docstring.
    """

    def __init__(self, ctx, prng, pnrng):
        self.ctx, self.prng, self.pnrng = ctx, prng, pnrng
        self.old = None

    def __enter__(self):
        self.old = (self.ctx.get("rng"), self.ctx.get("nrng"))
        self.ctx["rng"] = self.prng
        self.ctx["nrng"] = self.pnrng
        return self

    def __exit__(self, *exc):
        self.ctx["rng"], self.ctx["nrng"] = self.old
        return False


def _snapshot(ctx):
    return (set(ctx.get("loose") or ()), set(ctx.get("static_extra") or ()),
            set(ctx.get("authored") or ()))


def _new_since(ctx, snap):
    """(new_loose, new_static) since `_snapshot`, in a stable order."""
    lo0, st0, au0 = snap
    seen = set()
    new_lo, new_st = [], []
    for p in (ctx.get("loose") or ()):
        if p not in lo0 and p not in seen:
            seen.add(p)
            new_lo.append(p)
    for p in list(ctx.get("static_extra") or ()) + list(ctx.get("authored") or ()):
        if p not in st0 and p not in au0 and p not in seen:
            seen.add(p)
            new_st.append(p)
    return new_lo, new_st


def cut_subset(stage, path):
    """The `materialBind` GeomSubset holding a fragment's CUT faces, or None.

    `quake_flow._break` / `_break_split` bind the piece's OWN CLADDING (the
    module's texture, or on the GAC path the sooted atlas `gac_fire.
    rebind_sooted` already put on it) at PRIM level, and then
    `quake_flow._t_core_bind` puts every face this pipeline invented — the
    cut faces, the back, the reveals — into a `materialBind` GeomSubset named
    `core` and binds the brick/concrete core to that. So the two halves of a
    broken piece are already separated at authoring time and nobody had to
    keep a list: the subset is the break, the rest is the façade.

    `fracture.face_subset` returns None when the split is degenerate — a
    fragment entirely off the back of the wall, which has no façade to keep
    — and `_t_core_bind` does not run at all on a piece with no measurable
    thickness. Both cases give None here and the caller falls back to
    binding the whole prim, which is what every one of these binds did
    before.
    """
    from pxr import UsdGeom

    pr = stage.GetPrimAtPath(path) if path else None
    if not pr or not pr.IsValid():
        return None
    for s in UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(pr)):
        if s.GetPrim().GetName() == "core":
            return str(s.GetPrim().GetPath())
    return None


# `quake_flow._clad_material` authors the piece's own cladding at
# `<parent>/QuakeLooks/clad_<n>` — a triplanar around the texture the module
# was bound to when it was broken, which on the GAC path is the SOOTED atlas
# `gac_fire.rebind_sooted` had already put on it. It is the one material a
# fragment can carry that is worth keeping; everything else `_break` /
# `_break_split` can put on a prim is an invented core (mortar, dark
# concrete, plaster) drawn because the piece had no readable texture at all.
CLAD_PREFIX = "/QuakeLooks/clad_"


def facade_material(stage, path):
    """The piece's OWN cladding material if its prim binds one, else None.

    THE GUARD ON `cut_only`. A fragment with no readable source texture is
    bound an invented core material instead (`quake_flow._chunk_material` /
    `_mat_fn`), and keeping THAT on the outward faces of a burnt shell puts a
    pale mortar-grey patch on it — the "parts of the surface look pristine"
    half of the same review. There is only a façade to save when there is a
    façade material on the prim.
    """
    from pxr import UsdShade

    pr = stage.GetPrimAtPath(path) if path else None
    if not pr or not pr.IsValid():
        return None
    try:
        m = UsdShade.MaterialBindingAPI(pr).GetDirectBinding().GetMaterial()
    except Exception:
        return None
    if not m or not m.GetPrim().IsValid():
        return None
    return m if CLAD_PREFIX in str(m.GetPrim().GetPath()) else None


# ---------------------------------------------------------------------------
# THE TEAR'S OWN SKIN (review fire_dtc3, 2026-08-30)
# ---------------------------------------------------------------------------
# "the ragged break pieces that DO exist ... are a completely diff
# texture/color from the wall they extend ... make them look like extensions
# of the wall it's attached to" (user, fire_dtc3 bench, building b6 =
# `gac_SM_Building_02_F5c_s193`).
#
# MEASURED ON THAT BAKE, `tools/frag_facade_probe.py`:
#
#   /World/bake/g6/pieces/pier_S_3_09_0102              st[84]
#       sub mat_7 -> M_Building_01_WallBack_Inst_2cb1197b
#                    diffuseColor = tex:sootbake_71a526c32141f479.png
#   /World/bake/g6/brk_g6_pier_S_3_08_0086/frag_000     NO-UV
#       prim      -> clad_0        (a WORLD TRIPLANAR of that same atlas)
#       sub core  -> Char_2
#
# Two facts, and together they are the whole complaint:
#
#   * A FRAGMENT HAS NO UVs AT ALL. `fracture.prim_to_mesh` reads points,
#     counts and indices and nothing else; `fracture._write_mesh` writes
#     points, counts, indices, normals and extent and no `primvars:st`. So the
#     piece's own material — which is a UV-mapped ATLAS — could not be bound
#     to a fragment as things stood: every texel lookup would land on one
#     pixel.
#   * WHICH IS WHY `quake_flow._clad_material` EXISTS, and why it is a
#     WORLD-SPACE TRIPLANAR (`damage._pbr(project_uvw=True)`). A triplanar is
#     right for a TILEABLE map and wrong for a UNIQUE UNWRAP: a
#     `sootbake_*.png` / `gacsoot_*.png` is the piece's own atlas, so
#     projecting it through world coordinates at 0.45 repeats per metre
#     smears whatever happens to sit at those atlas coordinates — a window
#     reveal, roof felt, the sky patch in the corner of the sheet — across the
#     break. That is the "completely diff texture/color", and before this
#     change it was on 376 of 957 static fragments on the GAC building and
#     335 of 636 on the MCE kit one (`tools/tear_edge_probe.py`, table C).
#
# So: give the fragment REAL UVs and bind the parent's OWN material, the
# sooted copy `gac_fire.rebind_sooted` / `urban_fire._bind_soot` already put
# on the piece. That is exact, not an approximation — the fragment's outward
# faces ARE the parent's front surface, because `fracture.solidify`'s EXTRUDE
# branch "keeps the FRONT surface" and puts every invented face behind it. So
# for each fragment triangle there is a parent triangle it came off, and that
# triangle's own UV plane, extended to the fragment's three corners, is the
# mapping. The CUT faces are untouched: they are in the `core` GeomSubset and
# `_refire` still writes the char there — which is the whole point, and is
# what the previous round got right.
#
# THE PARENT MUST BE READ BEFORE IT IS FRACTURED. `_break_split` deactivates
# the source prim, and a deactivated prim has no points; `_tear_perimeter`
# therefore measures the façade first and hands the result forward.
#
# A façade test, the same one `fracture.face_subset` uses to decide which
# faces are NOT the façade, so the two agree on what "outward" means.
TEAR_OUT_COS = 0.30
# How far off its own parent triangle one fragment corner may extrapolate,
# in barycentric units. A corner is at most a cell's chew away from the
# surface it came off; anything past this is a fragment that is not really
# on the wall (a stray sliver), and clamping keeps its UVs inside the
# neighbourhood of the island instead of shooting across the atlas.
TEAR_BARY_CLAMP = 1.5
# Cost bound. The search is (fragment triangles) x (parent façade triangles);
# a GAC pier's façade is tens of triangles, but a curtain-wall cell can be
# thousands, so the biggest-area ones are kept and the rest dropped. Area,
# not a stride: the big triangles are the wall and the small ones are the
# window mullions.
TEAR_MAX_PARENT_TRIS = 2000
TEAR_CHUNK = 128
# The flat-tone fallback's roughness, and the sRGB -> linear exponent
# `damage._pbr` wants (`quake_flow.materials`: "LINEAR albedo (damage._pbr):
# screen grey ~ linear^0.42").
TEAR_TONE_ROUGH = 0.92
TEAR_TONE_GAMMA = 2.2
TEAR_TONE_PREFIX = "/FireLooks/tear_"


def _uv_primvar(prim):
    """(name, primvar) of a mesh's texture-coordinate primvar, or (None, None).

    Same test `urban_fire._mesh_arrays` makes — role first, then the four
    names this stock actually uses — but it also returns the NAME, because a
    fragment has to be given a primvar the parent's own material's
    `UsdPrimvarReader_float2` will read. Authoring `st` under a material whose
    reader asks for `st0` renders untextured, which is the same black
    rectangle by another route.
    """
    from pxr import UsdGeom

    for q in UsdGeom.PrimvarsAPI(prim).GetPrimvars():
        if (q.GetTypeName().role == "TextureCoordinate"
                or q.GetBaseName() in ("st", "uv", "UVMap", "st0")):
            return str(q.GetBaseName()), q
    return None, None


def facade_skin(ctx, path, out_xy):
    """The parent piece's FAÇADE surface, ready to re-skin its fragments.

    Returns `{"tris": (T,3,3) world corners, "uv": (T,3,2), "uvname",
    "mat": material path or None, "tex": base-colour url or None}` or None.

    ONE SUBSET, THE ONE WITH THE MOST OUTWARD AREA. A sliced GAC piece binds
    a material per source material (`gac_slice`: "ONE SUBSET PER ORIGINAL
    MATERIAL"), so its façade can be two or three of them — brick, a band of
    stone, the window frames. A fragment is ONE prim and takes ONE material,
    so the honest answer is the material that owns most of the wall; the
    alternative is authoring per-material subsets on every fragment, which
    would put a mullion material on a 0.3 m chip of brick as often as not.

    Must be called BEFORE `_break_split` deactivates `path`.
    """
    import numpy as np
    from pxr import Usd, UsdGeom, UsdShade
    from . import soot_bake as sb, urban_fire as uf

    stage = ctx["stage"]
    root = stage.GetPrimAtPath(path) if path else None
    if not root or not root.IsValid() or not root.IsActive():
        return None
    out = np.asarray((float(out_xy[0]), float(out_xy[1]), 0.0), dtype=float)
    ln_out = float(np.linalg.norm(out))
    if ln_out < 1e-9:
        return None
    out = out / ln_out
    xf = UsdGeom.XformCache()
    best = None
    for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        arr = uf._mesh_arrays(prim)
        if arr is None:                    # no texture coordinates: no carry
            continue
        uvname, _pv = _uv_primvar(prim)
        if not uvname:
            continue
        M = np.array(xf.GetLocalToWorldTransform(prim), dtype=float)
        Pw = arr["points"].astype(float) @ M[:3, :3] + M[3, :3]
        subs = list(UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(prim)))
        groups = []
        for s in subs:
            ids = s.GetIndicesAttr().Get()
            groups.append(([int(k) for k in ids] if ids else [], s.GetPrim()))
        if not groups:
            groups = [(None, prim)]
        for face_ids, target in groups:
            if face_ids is not None and not face_ids:
                continue
            tri, _tf, tslot = sb.triangles(arr["counts"], arr["indices"],
                                           face_ids)
            if not len(tri):
                continue
            C = Pw[tri]                                        # (T, 3, 3)
            nrm = np.cross(C[:, 1] - C[:, 0], C[:, 2] - C[:, 0])
            ln = np.linalg.norm(nrm, axis=1)
            keep = (ln > 1e-12) & ((nrm @ out) > TEAR_OUT_COS * ln)
            if not keep.any():
                continue
            area = 0.5 * float(ln[keep].sum())
            if best is not None and area <= best["area"]:
                continue
            uvc = sb._corner_uv(tri, tslot, arr["uv"], arr["interp"],
                                arr["uv_indices"])
            C, uvc, ln = C[keep], uvc[keep], ln[keep]
            if len(C) > TEAR_MAX_PARENT_TRIS:
                sel = np.argsort(-ln)[:TEAR_MAX_PARENT_TRIS]
                C, uvc = C[sel], uvc[sel]
            mat = UsdShade.MaterialBindingAPI(target).ComputeBoundMaterial()[0]
            mp = (str(mat.GetPrim().GetPath())
                  if mat and mat.GetPrim().IsValid() else None)
            best = {"area": area, "tris": C, "uv": uvc, "uvname": uvname,
                    "mat": mp, "tex": _basecolor_url(mat)}
    return best


def _basecolor_url(mat):
    """The base-colour texture a material feeds, as a url string, or None.

    `damage._basecolor_texture` reads a MATERIAL PRIM; this takes the handle
    `ComputeBoundMaterial` already returned, so the piece is only walked once.
    """
    from . import damage

    if not mat or not mat.GetPrim().IsValid():
        return None
    try:
        return damage._basecolor_texture(mat.GetPrim())
    except Exception:                                         # pragma: no cover
        return None


def _bary(A, B, C, n, nn, P):
    """Barycentric weights of `P` in triangle (A, B, C), EXTENDED off the
    plane: the three weights still sum to exactly 1 for any `P`, because
    cross(PB,PC) + cross(PC,PA) + cross(PA,PB) == n identically. Broadcasts —
    `(T,3)` triangles against `(k,1,3)` points gives `(k,T,3)` weights, and
    `(k,3)` against `(k,3)` gives `(k,3)`.
    """
    import numpy as np

    PA, PB, PC = A - P, B - P, C - P
    w0 = np.einsum("...i,...i->...", np.cross(PB, PC), n) / nn
    w1 = np.einsum("...i,...i->...", np.cross(PC, PA), n) / nn
    w2 = np.einsum("...i,...i->...", np.cross(PA, PB), n) / nn
    return np.stack([w0, w1, w2], axis=-1)


def project_uv(tris, uvs, corners, chunk=TEAR_CHUNK):
    """faceVarying UVs for `corners` (F,3,3), read off the parent's surface.

    PER FACE, NOT PER VERTEX. The parent's UVs are an ATLAS: two triangles
    that touch in 3-D can be on opposite sides of the sheet. Choosing a
    parent triangle per fragment VERTEX would therefore let one fragment
    triangle straddle a UV island and stretch the whole atlas across it — the
    smear this function exists to remove, back by another door. So the
    triangle is chosen ONCE per fragment face, from its centroid, and all
    three of that face's corners are read in THAT triangle's UV plane. The
    corners may sit slightly off it (the chew, the roughening, the solidify
    offset), so the weights are extrapolated rather than clamped to the
    triangle, and only bounded by `TEAR_BARY_CLAMP`.

    Returns (F,3,2) or None.
    """
    import numpy as np

    T = np.asarray(tris, dtype=float)
    U = np.asarray(uvs, dtype=float)
    if not len(T) or not len(corners):
        return None
    A, B, C = T[:, 0], T[:, 1], T[:, 2]
    n = np.cross(B - A, C - A)
    nn = np.einsum("ij,ij->i", n, n)
    ok = nn > 1e-18
    if not ok.any():
        return None
    A, B, C, n, nn, U = A[ok], B[ok], C[ok], n[ok], nn[ok], U[ok]
    corners = np.asarray(corners, dtype=float)
    cen = corners.mean(axis=1)
    out = np.zeros((len(corners), 3, 2), dtype=float)
    for s in range(0, len(corners), int(max(1, chunk))):
        e = min(len(corners), s + int(max(1, chunk)))
        q = cen[s:e][:, None, :]                               # (k,1,3)
        w = _bary(A[None], B[None], C[None], n[None], nn[None], q)
        wc = np.clip(w, 0.0, 1.0)
        wc = wc / np.maximum(wc.sum(axis=2, keepdims=True), 1e-12)
        near = (wc[..., 0:1] * A[None] + wc[..., 1:2] * B[None]
                + wc[..., 2:3] * C[None])                      # (k,T,3)
        pick = np.argmin(np.linalg.norm(near - q, axis=2), axis=1)
        Ap, Bp, Cp = A[pick], B[pick], C[pick]
        npk, nnp, Up = n[pick], nn[pick], U[pick]
        for j in range(3):
            wj = _bary(Ap, Bp, Cp, npk, nnp, corners[s:e, j])
            wj = np.clip(wj, -TEAR_BARY_CLAMP, TEAR_BARY_CLAMP)
            out[s:e, j] = (wj[:, 0:1] * Up[:, 0] + wj[:, 1:2] * Up[:, 1]
                           + wj[:, 2:3] * Up[:, 2])
    return out


def tone_material(ctx, sk):
    """A FLAT tone sampled from the parent's own façade map — the fallback
    for a piece whose UVs cannot be carried (no `primvars:st`, a degenerate
    façade, an unreadable atlas).

    SAMPLED OVER THE PIECE'S OWN UV BOX, not over the whole sheet: the map is
    an atlas of the whole building, so its global mean is roof felt and sky
    as much as it is wall. `soot_plume._read_rgb` is the reader the rest of
    the fire pipeline uses (it can reach Nucleus), and the mean is converted
    from screen to LINEAR because `damage._pbr` takes linear albedo.
    """
    import numpy as np
    from . import damage, soot_plume as spl

    tex = (sk or {}).get("tex")
    if not tex:
        return None
    cache = ctx.setdefault("tear_tone", {})
    uvbox = None
    if sk.get("uv") is not None and len(sk["uv"]):
        uv = np.asarray(sk["uv"], dtype=float).reshape(-1, 2)
        uvbox = (float(uv[:, 0].min()), float(uv[:, 0].max()),
                 float(uv[:, 1].min()), float(uv[:, 1].max()))
    key = (tex, None if uvbox is None
           else tuple(round(q, 2) for q in uvbox))
    mat = cache.get(key)
    if mat is not None:
        return mat or None
    img = spl._read_rgb(tex, max_px=512)
    if img is None:
        cache[key] = False
        return None
    img = np.asarray(img, dtype=np.float32)
    if img.ndim == 2:
        img = np.repeat(img[..., None], 3, axis=2)
    img = img[..., :3]
    if uvbox is not None:
        h, w = img.shape[0], img.shape[1]
        u0, u1, v0, v1 = uvbox
        # `soot_bake.uv_position_map`'s convention, which every bake in this
        # pipeline is written to: column = u * px, ROW 0 IS v = 1.
        c0 = int(max(0, min(w - 1, math.floor((u0 % 1.0) * w))))
        c1 = int(max(c0 + 1, min(w, math.ceil((u1 % 1.0 or 1.0) * w))))
        r0 = int(max(0, min(h - 1, math.floor((1.0 - (v1 % 1.0 or 1.0)) * h))))
        r1 = int(max(r0 + 1, min(h, math.ceil((1.0 - (v0 % 1.0)) * h))))
        crop = img[r0:r1, c0:c1]
        if crop.size:
            img = crop
    rgb = np.clip(img.reshape(-1, 3).mean(axis=0), 0.0, 1.0)
    lin = tuple(float(q) ** TEAR_TONE_GAMMA for q in rgb)
    mp = "{0}{1}{2}".format(ctx["parent"], TEAR_TONE_PREFIX,
                            len([k for k in cache if cache.get(k)]))
    mat = damage._pbr(ctx["stage"], mp, lin, TEAR_TONE_ROUGH)
    cache[key] = mat
    return mat


def skin_fragment(ctx, sk, frag_path):
    """Give ONE fragment the parent's UVs and the parent's own material.

    Returns "uv" (the parent's map, carried), "tone" (the flat sampled
    fallback) or None (nothing could be said about the parent, so the
    fragment keeps whatever `_break_split` bound).

    THE BIND IS WEAK ON PURPOSE (`quake_flow._bind`, not `_b_bind_over`).
    A `strongerThanDescendants` bind on the prim beats the bindings on its
    DESCENDANTS, and the `core` GeomSubset is a child prim — so binding the
    façade strongly here would take the char off the cut faces, which is the
    other half of the same review turned inside out.
    """
    from pxr import Sdf, UsdGeom, UsdShade, Vt
    import numpy as np
    from . import quake_flow as qf

    if not sk:
        return None
    stage = ctx["stage"]
    prim = stage.GetPrimAtPath(frag_path) if frag_path else None
    if not prim or not prim.IsValid():
        return None
    mesh = UsdGeom.Mesh(prim)
    pts = mesh.GetPointsAttr().Get()
    cnt = mesh.GetFaceVertexCountsAttr().Get()
    idx = mesh.GetFaceVertexIndicesAttr().Get()
    if not pts or not cnt or not idx:
        return None
    cnt = np.asarray(cnt, dtype=np.int64)
    if not cnt.size or int(cnt.min()) != 3 or int(cnt.max()) != 3:
        return None      # `_write_mesh` writes triangles; this is not one
    mat = (UsdShade.Material.Get(stage, sk["mat"]) if sk.get("mat") else None)
    how = "uv"
    if mat is None or not mat.GetPrim().IsValid():
        mat, how = tone_material(ctx, sk), "tone"
    if mat is None:
        return None
    if how == "uv":
        M = np.array(UsdGeom.XformCache().GetLocalToWorldTransform(prim),
                     dtype=float)
        P = np.asarray([[q[0], q[1], q[2]] for q in pts], dtype=float)
        W = P @ M[:3, :3] + M[3, :3]
        uv = project_uv(sk["tris"], sk["uv"],
                        W[np.asarray(idx, dtype=np.int64).reshape(-1, 3)])
        if uv is None:
            mat, how = tone_material(ctx, sk), "tone"
            if mat is None:
                return None
        else:
            pv = UsdGeom.PrimvarsAPI(prim).CreatePrimvar(
                sk["uvname"], Sdf.ValueTypeNames.TexCoord2fArray,
                UsdGeom.Tokens.faceVarying)
            pv.Set(Vt.Vec2fArray([(float(a), float(b))
                                  for a, b in uv.reshape(-1, 2)]))
    qf._bind(stage, frag_path, mat)
    ctx.setdefault("tear_faced", set()).add(frag_path)
    return how


def bind_break(ctx, path, mat, cut_only):
    """Bind `mat` over a broken piece.

    Returns "cut" (the char went on the cut faces and the façade was kept),
    "whole_nocore" (there was a façade to keep but the piece carries no
    `core` subset to put the char in — `_t_core_bind` only runs on a piece
    thick enough to solidify, so a clipped shell has none) or "whole".

    THE FAÇADE OF A PIECE THAT STAYS WHERE IT WAS IS STILL THE FAÇADE.
    Binding `_debris_mat` / `_burn_mat` over the whole prim
    (`strongerThanDescendants`) takes the cladding off the outward faces too,
    so a torn bay's standing half goes near-black while the module beside it
    keeps its sooted atlas and the seam between them reads as a hard dark
    rectangle — "the buildings still look like they make clean rectangular
    breaks ... the material of the broken/debris part is a much darker colour
    than the intact façade next to it" (user, review of the second row,
    2026-08-30). With `cut_only` the char goes on the `core` subset alone.

    Loose fragments are NOT cut-only: they tumbled into the heap and every
    face of them is in the pile, which is the one place the dark end belongs.

    ...AND A FRAGMENT `skin_fragment` HAS RE-SKINNED IS A FAÇADE TOO
    (fire_dtc3, 2026-08-30). It carries the parent piece's own UVs and the
    parent's own sooted material, which is not under `CLAD_PREFIX`, so
    `facade_material` says no about the one binding that is most emphatically
    yes. `ctx["tear_faced"]` is the register of those prims and it is
    consulted first; without it the very fix that put the wall back on the
    tear would be charred off again one call later, by `_refire`.
    """
    from . import quake_flow as qf

    faced = ctx.get("tear_faced") or ()
    if cut_only and (path in faced
                     or facade_material(ctx["stage"], path) is not None):
        sub = cut_subset(ctx["stage"], path)
        if sub:
            qf._b_bind_over(ctx["stage"], sub, mat)
            return "cut"
        if path in faced:
            # NO `core` SUBSET AND A CARRIED FAÇADE: `face_subset` found the
            # split degenerate (every face outward), so there are no cut
            # faces to char and charring the prim would take the wall off the
            # only faces there are. Keep it.
            return "kept"
        if isinstance(ctx.get("soot_prebaked"), (set, frozenset)):
            # A CLIPPED GAC SHELL HAS NO THICKNESS AND SO NO `core` SUBSET:
            # its only "cut faces" are the hairline edges of the tear. Charring
            # the whole fragment for their sake took the sooted atlas off the
            # standing half of 724 pieces on SM_Building_05 F5c — the dark
            # rectangles again. Keep the façade; the burn zone in the skin is
            # what darkens the wall round the hole.
            return "kept"
        qf._b_bind_over(ctx["stage"], path, mat)
        return "whole_nocore"
    qf._b_bind_over(ctx["stage"], path, mat)
    return "whole"


def _refire(ctx, loose_paths, static_paths):
    """Rebind quake-palette output to the fire palette.

    `_ragged_slabs` / `_ragged_neighbours` / `r_droop` bind `timber_dusty`,
    `concrete_dusty`, `brick_dusty` and `dust` — mortar fines over pale brick,
    which is the quake signature and is exactly what a fire rubble pile does
    NOT have. Same `before`/`made` diff `urban_fire.r_roof_burnthrough`
    already uses around `quake_flow.r_roof_hole`, and the same exemption for
    the reinforcement (`rebar_` / `sbar_`), which is steel and stays steel.

    Loose pieces are DEBRIS and take the dark char/scorch end (`_debris_mat`).
    Statics are what is still standing at the edge of the hole — a torn half
    bay, the surviving band of a floor slab — so they take the graded burn
    material `r_char_facade` stamps, not a flat tone: a flat dark bind over a
    hundred small cells reads as one black rectangle, which is bug 5 in
    `build-urban-fire-scenes` in miniature.

    ...AND ON A STATIC THAT BIND IS CUT-FACES-ONLY (`bind_break`). A torn
    bay's standing half has not moved: its outward faces are the same wall
    the module next door still shows, so they keep the cladding/sooted atlas
    `_break_split`'s `static_mat` put on them and only the `core` subset —
    the faces the fracture invented — takes the burn material. A fragment
    with no cladding to keep (no readable source texture, so `_break_split`
    drew an invented core material for it) is charred whole as before —
    `bind_break`'s own guard. Returns
    `(n_rebound, n_cut_only, n_facade_but_no_core_subset)`.
    """
    from . import quake_flow as qf
    from . import urban_fire as uf

    f = ctx["fire"]
    finish = f.get("finish") or "char"
    n = n_cut = n_nocore = 0
    # REINFORCEMENT IS STEEL, AND IT IS FIRE-BLACKENED STEEL: the quake
    # palette's `rebar` is a warm rust (0.30, 0.19, 0.13) that read as
    # scorched timber next to a black shell ("rods, probably structural.
    # They looked wooden", user review, 2026-08-30). Bars are rebound to the
    # fire palette's cool `steel`; joists and rafters are authored in it.
    steel = ctx["mats"].get("steel")
    for pth in loose_paths:
        name = str(pth).rsplit("/", 1)[-1]
        if name.startswith(("rebar", "sbar")):
            if steel is not None:
                qf._b_bind_over(ctx["stage"], pth, steel)
            continue
        qf._b_bind_over(ctx["stage"], pth, uf._debris_mat(ctx))
        n += 1
    for pth in static_paths:
        name = str(pth).rsplit("/", 1)[-1]
        if name.startswith(("rebar", "sbar")):
            if steel is not None:
                qf._b_bind_over(ctx["stage"], pth, steel)
            continue
        if name.startswith(("joist", "rafter")):
            continue
        how = bind_break(ctx, pth, uf._burn_mat(ctx, finish, 0.85),
                         cut_only=True)
        if how == "cut":
            n_cut += 1
        elif how == "whole_nocore":
            n_nocore += 1
        n += 1
    return n, n_cut, n_nocore


# ---------------------------------------------------------------------------
# THE RECIPE
# ---------------------------------------------------------------------------
def r_partial_collapse(ctx, mode="elevation", side=None, corner=None,
                       from_storey=None, span_frac=None, depth_m=None,
                       drop_slabs=1, heaps=True, ragged=True, throw=True,
                       mass=None):
    """PART of the burnt-out shell comes down; the rest of it stands.

    Runs BEFORE the passes that author art on walls — see the ordering note in
    `urban_fire.LADDER`'s F5 lists. `quake_flow._els` skips elements a recipe
    has marked `dead`, so a wall taken away after its soot has been baked
    leaves the soot standing in the sky ("a row of grey flags", uf_bench2
    dw_terrace, 2026-08-28).
    """
    import numpy as np
    from . import quake_flow as qf
    from . import urban_fire as uf

    stage, f = ctx["stage"], ctx["fire"]
    plan = plan_partial_collapse(ctx, mode=mode, side=side, corner=corner,
                                 from_storey=from_storey, span_frac=span_frac,
                                 depth_m=depth_m, drop_slabs=drop_slabs,
                                 mass=mass)
    mtag = plan["mass"]
    m = ctx["info"]["masses"].get(mtag) or ctx["info"]["masses"]["main"]
    prng = random.Random(plan["seed"])
    pnrng = np.random.default_rng(plan["seed"] & 0xFFFFFFFF)
    Hm = max(3.0, float(m["top"]) - float(m["z0"]))
    n_broke = n_art = n_frag = 0
    # WHAT THE BUILDING ALREADY HAD STANDING, before this recipe touched it.
    # The position sweep (step 5) may only look at THESE: everything this
    # recipe itself adds to `static_extra` is the part that STAYS UP — the
    # surviving cells of a torn bay (`_break_split`'s statics), the ragged
    # remainder of a floor slab, the windrow — and it is all inside the lost
    # region and above the failure line by construction, so an unfiltered
    # sweep would hand the standing half of every torn wall to the solver.
    # `r_fire_collapse` gets away without this only because `_break` with
    # `partial=None` returns no statics at all.
    pre_static = set(q for q in ctx.get("static_extra") or () if q)

    # WHAT THE COLLAPSE PUT FLAME ON. Set BEFORE any later pass reads it:
    # `r_smoke_stain` (and every other soot pass, through `_soot_skin`) builds
    # the skin once and caches it on the ctx, and this recipe runs ahead of all
    # of them in the F5c ladder. A stale skin from an earlier pass would have
    # no zone in it, so drop it if one is somehow there.
    f["burn_zone"] = plan["burn_zone"]
    ctx.pop("soot_skin", None)
    ctx["partial_collapse"] = plan
    # per-run, not cumulative: one ctx is one building, but a bench that
    # re-runs the recipe on the same ctx must not report yesterday's count.
    ctx.pop("_tear_skin", None)

    with _own_rng(ctx, prng, pnrng):
        # A BURNT-OUT SHELL HAS NO FLAME IN IT, whatever the event list says.
        # `soot_plume.plan_events` decides `burnt_out` from a literal tuple
        # ("F4", "F5", "F6") that does not know about F5c, so its
        # "the compartment of origin is always an event" fallback can label
        # one event `flame` on a building whose `urban_fire.ACTIVE` state is
        # `residual`. `r_flames` lights every `flame` event regardless of the
        # building state, so that would be a plume of fire on a cold ruin.
        # `ACTIVE[level]` is the authority; demote here, before any later
        # pass reads the list.
        if f.get("state") not in ("flame",):
            n_dem = 0
            for ev in (f.get("events") or ()):
                if ev.get("state") == "flame":
                    ev["state"] = "out"
                    n_dem += 1
            if n_dem:
                ctx["notes"].append(
                    "partial collapse: {0} flame event(s) demoted to burnt "
                    "out (state={1})".format(n_dem, f.get("state")))

        # ---- 1. the shell ------------------------------------------------
        # THE WALL FELL OUTWARD. It rotates about its foot, so the top leads
        # and the pieces land as a fan in the street rather than in a pile
        # against the plinth. `settle.prepare(velocity_map=...)` is the only
        # way to say that per body — a scene-wide `bias` is wind.
        for e in plan["kill"]:
            st, lo = qf._break(stage, ctx["parent"], e, ctx["tag"],
                               prng.randint(*BREAK_PIECES), prng, pnrng,
                               ctx["mats"], ctx["cache"], ctx["info"]["type"],
                               inner_p=0.4, rough=0.014,
                               consume=_f("UF_PC_CONSUME", BREAK_CONSUME),
                               style=ctx["info"]["style"])
            # `_b_bind_over`, NOT `_bind`. `_break` binds the module's own
            # cladding on the prim AND a core material on the broken-face
            # subsets, and an ordinary prim-level bind is
            # weakerThanDescendants — a third of a burnt terrace's rubble
            # stayed clean tan stonework that way (uf4, 2026-08-28).
            # ...but a STATIC out of `_break` has not moved, so it is a piece
            # of standing wall and its outward faces are still the façade:
            # `bind_break(cut_only=True)` puts the char on the `core` subset
            # alone. `partial=None` means `_break` returns no statics today,
            # so this is the rule stated where it belongs rather than a live
            # branch; the loose fragments below are all in the heap and take
            # the dark end whole, which is where it belongs.
            for pth in lo:
                bind_break(ctx, pth, uf._debris_mat(ctx), cut_only=False)
            for pth in st:
                bind_break(ctx, pth, uf._debris_mat(ctx), cut_only=True)
            if throw:
                ox, oy = qf._outward(m, e["side"])
                for pth in lo:
                    zf = min(1.0, max(0.0, (float(e["z"]) + float(e["h"]) * 0.5
                                            - float(m["z0"])) / Hm))
                    v = THROW_BASE + THROW_TOP * zf + prng.uniform(-0.2, 0.3)
                    ctx["velocity"][pth] = (ox * v, oy * v, 0.15 * v)
            ctx["loose"] += lo
            ctx["static_extra"] += st
            n_art += uf._drop_face_art(ctx, e)
            e["dead"] = True
            n_broke += 1
            n_frag += len(st) + len(lo)

        # ---- 2. THE WHOLE PERIMETER OF THE HOLE ---------------------------
        # A KIT SEAM IS A STRAIGHT LINE AND A FIRE DOES NOT CUT ONE. Every
        # surviving module that touches the loss — beside it, under it, over
        # it, and round the corner on the return — loses its near portion
        # along a wandering (masonry: course-stepped) line, so the hole's
        # outline is made of Voronoi cell boundaries and not of module joints.
        # See `plan_edges` for what the three passes this replaces missed.
        snap = _snapshot(ctx)
        n_edge = 0
        edge_jobs = []
        if ragged:
            edge_jobs = plan_edges(ctx, plan, m, prng)
            n_edge = _tear_perimeter(ctx, plan, m, prng, edge_jobs)

        # ---- 3. the floors behind it -------------------------------------
        # THE ROOF DECK HAS TO EXIST FIRST. `_ragged_slabs` calls
        # `_a_roofify`, which falls back to `_roof_box` — a faithful copy of
        # the kit tile's WORLD bbox, and `SM_Roof` is 23 x 23 m, wider than
        # several of these buildings. `urban_fire._deck_slab` authors ONE
        # slab clipped to the mass instead and registers it where
        # `_a_roofify` will find it; the F5c ladder runs `roof_burnthrough`
        # (which decks) before this, but the recipe must stand alone too.
        if not ctx.get("roof_slabs"):
            uf._deck_slab(ctx, mtag)
        if ragged:
            for sd in plan["sides"]:
                qf._ragged_slabs(ctx, mtag, sd, set(plan["storeys"]))
                # THE FLOORS AT THE OPENING HINGE DOWN — the USAR lean-to —
                # BUT ONLY WHEN A REAL LENGTH OF WALL HAS GONE.
                # `quake_flow._droop_strip` splits a strip 25-50 % of the
                # mass DEEP along the WHOLE of that side and rotates it about
                # its inner edge; behind a wall that is still standing (a
                # corner loss, or the stub either end of a partial span) that
                # is a floor tilting into a room for no visible reason, and
                # the far end of the strip hinges under masonry that never
                # moved. Gated on the loss covering most of the elevation,
                # which is exactly when the plate really has lost its
                # bearing along its whole length.
                lo_t, hi_t = plan["span"][(sd, plan["top_storey"])]
                if (hi_t - lo_t) >= 0.60 * side_length(m, sd):
                    qf.r_droop(ctx, mass=mtag, side=sd,
                               storeys=set(plan["storeys"]), p=0.35)
        new_lo, new_st = _new_since(ctx, snap)
        n_refire, n_refire_cut, n_refire_nocore = _refire(
            ctx, new_lo, new_st)

        # ---- 4. the slab that lost its bearing wall ----------------------
        n_slab = n_prop = 0
        fit = ctx.get("fit") or {}
        for key in plan["drop"]:
            slab = (fit.get("slabs") or {}).get(key)
            if not slab:
                continue
            pr = stage.GetPrimAtPath(slab)
            if pr and pr.IsValid() and pr.IsActive() and slab not in ctx["loose"]:
                ctx["loose"].append(slab)
                n_slab += 1
            # SAME ARGUMENT FOR THE FURNITURE ON IT. `fit_interior` seats
            # `props[(mtag, storey)]` directly on `slabs[(mtag, storey)]`, so
            # a prop `r_gut_interior` charred rather than deactivated is left
            # standing on a plate that is about to fall out from under it
            # (`r_fire_collapse` learned this the same way).
            for pp in (fit.get("props") or {}).get(key, ()):
                pr2 = stage.GetPrimAtPath(pp)
                if pr2 and pr2.IsValid() and pr2.IsActive() \
                        and pp not in ctx["loose"]:
                    ctx["loose"].append(pp)
                    n_prop += 1

        # ---- 4b. THE FIT-OUT INSIDE THE HOLE ------------------------------
        # THE POSITION SWEEP IN STEP 5 CANNOT SEE ANY OF THIS. `burn_building`
        # folds `ctx["fit"]["all"]` into `static_extra` ONCE, after every
        # recipe has run, so while this recipe is running the partitions, the
        # frame columns and the furniture are in no list the sweep walks —
        # and step 4 above only sends down the fit-out of the one or two
        # storeys whose whole slab drops. A partition standing on the strip
        # of floor `_ragged_slabs` has just broken away, behind the wall that
        # has just gone, is left upright in mid-air: "`part_main_6_1` and
        # `part_main_6_0` look like they're floating / staying upright when
        # they shouldn't" (user, review of the second row, 2026-08-30 —
        # commercial_mid F5c, storey 6, in the region the collapse took).
        #
        # So: anything of the fit-out on a KILLED storey whose own footprint
        # is inside `plan["region"]` goes to the solver with the fragments'
        # own outward throw and a downward kick (it let go, it was not
        # projected — the same sign `_tear_perimeter` gives the shards at the
        # lip). Everything on a surviving storey, and everything behind the
        # part of the elevation that still stands, is untouched: this is a
        # PARTIAL collapse and the rooms next door still have their walls.
        #
        # ...AND EVERYTHING ON A STOREY WHOSE WHOLE SLAB WENT DOWN IN STEP 4,
        # region or no region. That is the case the review actually caught:
        # `plan["drop"]` sends the topmost slab of the band to the solver as
        # ONE plate, so a partition anywhere on that floor — including the
        # far half of it, metres outside `plan["region"]` — is standing on
        # nothing the moment the plate moves. Step 4 already takes the props
        # off a dropped slab for exactly this reason and simply never learned
        # about the partitions and the columns.
        dropped = set(int(s) for (mt_d, s) in plan["drop"] if mt_d == mtag)
        n_fit = {"part": 0, "col": 0, "prop": 0}
        fit_now = ctx.get("fit") or {}
        killed = set(int(s) for s in plan["storeys"])
        cands = []
        for p in (fit_now.get("partitions") or ()):
            # `part_<mass>_<storey>_<k>`; the mass tag may itself contain
            # underscores, so index from the RIGHT.
            bits = str(p).rsplit("/", 1)[-1].split("_")
            if len(bits) < 4:
                continue
            try:
                s_, mt_ = int(bits[-2]), "_".join(bits[1:-2])
            except ValueError:
                continue
            cands.append((p, mt_, s_, "part"))
        for (mt_, s_), cols in (fit_now.get("columns") or {}).items():
            for p in (cols or ()):
                cands.append((p, mt_, int(s_), "col"))
        for (mt_, s_), props in (fit_now.get("props") or {}).items():
            for p in (props or ()):
                cands.append((p, mt_, int(s_), "prop"))
        # THE FOOTPRINT, NOT THE CENTRE — a partition is a 0.12 m sheet
        # running the SHORT way across the plan (`fit_interior`: `depth =
        # D * 0.35..0.7`), so one that frames into the lost wall has its END
        # in the region and its CENTRE several metres inside it. The centre
        # test step 5 uses is right for a compact static and would miss
        # exactly the prims the review named. Corners + centre of the world
        # bbox; a non-structural sheet whose floor and bearing wall have both
        # gone at one end does not stay upright.
        from pxr import Usd as _Usd, UsdGeom as _UsdGeom
        xf0 = _UsdGeom.XformCache()
        bc0 = _UsdGeom.BBoxCache(_Usd.TimeCode.Default(),
                                 [_UsdGeom.Tokens.default_])
        for p, mt_, s_, kind in cands:
            if mt_ != mtag or not p or p in ctx["loose"]:
                continue
            if s_ not in killed and s_ not in dropped:
                continue
            pr = stage.GetPrimAtPath(p)
            if not pr or not pr.IsValid() or not pr.IsActive():
                continue
            try:
                t = xf0.GetLocalToWorldTransform(pr).ExtractTranslation()
            except Exception:
                continue
            pts = [(float(t[0]), float(t[1]))]
            try:
                bx = bc0.ComputeWorldBound(pr).ComputeAlignedRange()
                if not bx.IsEmpty():
                    lo_b, hi_b = bx.GetMin(), bx.GetMax()
                    pts += [(float(lo_b[0]), float(lo_b[1])),
                            (float(hi_b[0]), float(lo_b[1])),
                            (float(hi_b[0]), float(hi_b[1])),
                            (float(lo_b[0]), float(hi_b[1]))]
            except Exception:
                pass
            sd = None
            for wx_, wy_ in pts:
                lx, ly = qf._to_local(m, wx_, wy_)
                sd = region_side(plan, m, lx, ly)
                if sd is not None:
                    break
            if sd is None and s_ not in dropped:
                continue
            # A piece on a dropped slab that is nowhere near the loss simply
            # falls; there is no wall for it to be thrown out through.
            side_out = sd if sd is not None else None
            ctx["loose"].append(p)
            ctx["static_extra"] = [q for q in ctx["static_extra"] if q != p]
            if throw and side_out is not None:
                ox, oy = qf._outward(m, side_out)
                zf = min(1.0, max(0.0, (float(t[2]) - float(m["z0"])) / Hm))
                v = THROW_BASE + THROW_TOP * zf + prng.uniform(-0.2, 0.3)
                ctx["velocity"][p] = (ox * v, oy * v, -0.15 * v)
            n_fit[kind] += 1

        # ---- 5. the position sweep, RESTRICTED IN PLAN --------------------
        # `r_fire_collapse`'s sweep is "everything static whose centre is
        # above the failure line", which is right when the whole top of the
        # block is coming down and catastrophic here: it would drop the far
        # half of the roof deck, its plant and the walls on three untouched
        # elevations. The same argument for using a POSITION test at all
        # still holds (the roof is `_roof_box` slabs, then `_split_strip`
        # remainders, then a deck slab, each registered in a different list
        # or in none, so a bookkeeping sweep misses one of them) — it just
        # needs a second predicate: above the failure line AND inside the
        # plan footprint of the loss.
        from pxr import UsdGeom
        xf = UsdGeom.XformCache()
        keep, moved = [], []
        for pth in list(ctx["static_extra"]):
            if pth not in pre_static:
                keep.append(pth)          # authored by THIS recipe: it stays
                continue
            pr = stage.GetPrimAtPath(pth) if pth else None
            if not pr or not pr.IsValid() or not pr.IsActive():
                keep.append(pth)
                continue
            try:
                t = xf.GetLocalToWorldTransform(pr).ExtractTranslation()
            except Exception:
                keep.append(pth)
                continue
            lx, ly = qf._to_local(m, float(t[0]), float(t[1]))
            if float(t[2]) > plan["cut_z"] and in_region(plan, m, lx, ly):
                moved.append(pth)
            else:
                keep.append(pth)
        ctx["static_extra"] = keep
        n_swept = 0
        for pth in moved:
            if pth not in ctx["loose"]:
                ctx["loose"].append(pth)
                n_swept += 1

        # ---- 6. the rubble ------------------------------------------------
        n_heap = 0
        if heaps:
            for h in plan["heaps"]:
                if h["where"] == "outside":
                    # NO DUST IN THE MIX — `_heap`'s own `HEAP_MIX` is mortar
                    # fines over pale brick, the quake signature. Fire rubble
                    # is black, wet from the hose, and has no fines standing
                    # on it.
                    made = qf._heap(
                        ctx, m, h["base_z"], 0.0, h["spread_frac"],
                        fill=False, sides=(h["side"],),
                        depth_m=h["depth_m"], along=h["along"],
                        tag="pcheap",
                        mat_fn=lambda: (
                            ctx["mats"]["char_concrete"] if prng.random() < 0.45
                            else ctx["mats"]["soot"] if prng.random() < 0.80
                            else ctx["mats"]["calcined"]))
                    n_heap += len(made)
                else:
                    n_heap += len(_inner_rubble(ctx, m, plan, h, prng))

        # ---- 7. the teeth at the break ------------------------------------
        # What did NOT fall: the bearing stubs still in the wall pocket where
        # a floor used to frame into the lost wall, and the bar/purlin ends
        # standing out of the deck where the roof edge went. This is what a
        # USAR photograph of a gutted shell has and a clean cut does not.
        for s in plan["storeys"]:
            uf._joist_stubs(ctx, mtag, s, n=prng.randint(2, 5))
        if plan["top_storey"] >= plan["n_levels"] - 1:
            uf._rafter_teeth(ctx, m, n=prng.randint(3, 7))

    cen = edge_census(edge_jobs)
    plan["edges"] = edge_jobs
    plan["edge_census"] = cen
    ctx["notes"].append(describe(plan))
    ctx["notes"].append(
        "partial collapse: {0} module(s) broken into {1} fragment(s), {2} "
        "stain(s) removed with them, {3} surviving module(s) at the edge of "
        "the hole torn on a wandering line, {4} piece(s) rebound off the "
        "quake palette ({9} on the CUT faces only with the façade kept, "
        "{10} with a façade but no core subset to put the char in), "
        "{5} slab(s) + {6} prop(s) dropped, {7} static prim(s) "
        "swept into the loss, {8} rubble chunk(s)".format(
            n_broke, n_frag, n_art, n_edge, n_refire, n_slab, n_prop,
            n_swept, n_heap, n_refire_cut, n_refire_nocore))
    ctx["notes"].append(
        "partial collapse fit-out in the hole: {0} partition(s), {1} "
        "column(s), {2} prop(s) sent down (killed storeys {3}-{4} inside the "
        "lost region)".format(n_fit["part"], n_fit["col"], n_fit["prop"],
                              plan["s0"], plan["top_storey"]))
    ctx["notes"].append(
        "partial collapse edges: " + ", ".join(
            "{0} {1}/{2}".format(c, cen[c][1], cen[c][0])
            for c in EDGE_CLASSES) +
        "; burn zone {0} rect(s) on {1}".format(
            len(plan["burn_zone"]),
            "/".join(sorted(set(r[0] for r in plan["burn_zone"])))))
    # THE TEAR SKIN (fire_dtc3, 2026-08-30). A static fragment counted under
    # `uv` carries the parent piece's own texture coordinates and the parent's
    # own (sooted) material, so it reads as the wall it broke off; `tone` is
    # the flat colour sampled from that wall's own map where the UVs could not
    # be carried; `none` is a fragment left on whatever `_break_split` bound,
    # which on a piece with a readable atlas is `_clad_material`'s world
    # triplanar — the smear the review named. `none` should be 0.
    sk = ctx.get("_tear_skin") or {}
    ctx["notes"].append(
        "partial collapse tear skin: {0} fragment(s) carry the parent wall's "
        "own UVs + material, {1} a tone sampled from its map, {2} left on the "
        "break palette; {3} torn piece(s) had a readable facade, {4} did "
        "not".format(sk.get("uv", 0), sk.get("tone", 0), sk.get("none", 0),
                     sk.get("pieces", 0), sk.get("no_facade", 0)))
    return plan


def edge_neighbour(ctx, plan, m, side, storey, low_edge):
    """The surviving bay next to one vertical edge of the hole, and where to
    crack it. Returns `(element, crack_t_metres)` or `(None, None)`.

    CALL IT AFTER THE KILL LOOP. It walks `quake_flow._els`, which skips
    elements a recipe has marked `dead` — that is how it knows which bay is
    still standing. Called on a fresh plan it will happily hand back a module
    that is about to be broken.

    THE CRACK GOES THROUGH THE MIDDLE OF THAT BAY, NOT ON THE SPAN'S OWN
    EDGE. Which modules die is quantised to the kit grid (it has to be — a
    module is the smallest thing `_break` can take), so the hole's real edge
    is ALWAYS a module seam however the span was drawn; cracking at the span
    edge can therefore shave a 16 cm sliver off the neighbour and leave the
    seam exactly where it was (measured on `commercial_mid` storey 6: span
    edge 4.84 m, module seam 5.00 m). Cracking a third to two thirds of the
    way INTO the surviving bay moves the visible edge off the grid, which is
    the same thing `_ragged_neighbours` does with its `keep` fraction.
    """
    from . import quake_flow as qf

    live = [e for e in qf._els(ctx, mass=plan["mass"], side=side,
                               role=("wall", "corner"))
            if int(e["storey"]) == storey]
    if not live:
        return None, None
    lo, hi = plan["span"][(side, min(storey, plan["top_storey"]))]
    edge_t = lo if low_edge else hi
    if low_edge:
        cand = [e for e in live if qf._p_el_t(m, side, e) <= edge_t]
        e = max(cand, key=lambda q: qf._p_el_t(m, side, q)) if cand else None
    else:
        cand = [e for e in live if qf._p_el_t(m, side, e) >= edge_t]
        e = min(cand, key=lambda q: qf._p_el_t(m, side, q)) if cand else None
    if e is None:
        return None, None
    fr = qf._piece_frame(e)
    w = float(fr[3]) if fr else max(4.0, float(m.get("module") or 4.0))
    t0 = qf._p_el_t(m, side, e)
    return e, (t0, t0 + w)


def _tear_perimeter(ctx, plan, m, prng, jobs):
    """Tear every surviving module that touches the hole. Returns the count.

    One `quake_flow._break_split` per module, with the UNION of that module's
    own cut judges — a module at the corner of a staircase step is `left` and
    `below` at once and `_break_split` deactivates its source prim, so it gets
    exactly one chance:

      v  `_p_vcrack_judge`   a wandering VERTICAL crack (masonry: quantised to
                             the bond, half a stretcher per riser), FEMA 306's
                             crack at a wall return
      z  `_p_zline_judge`    a HORIZONTAL break that staircases along the
                             courses (urm) or tears (rc) — the seam under a
                             staircase tread and under the failure line
      t  `_p_toward_judge`   the return wall's near end, cut on a line running
                             UP the wall at a fraction of the plan depth

    WHAT SURVIVES SURVIVES. `_break_split` returns (statics, loose): the far
    portion of a torn module stays exactly where it is, bound to the piece's
    own cladding (`static_mat`), and only the near portion comes away. A tear
    that leaves NO static is a module that was effectively killed and is
    reported as such (`no_static`), because the whole point of tearing rather
    than killing is that the wall is still there with a ragged end.

    ...AND WHAT STAYS KEEPS THE WALL ON IT (fire_dtc3, 2026-08-30). The
    façade of the piece is MEASURED before the split (`facade_skin` — the
    source prim is deactivated by `_break_split` and cannot be read after)
    and every STATIC fragment is then given the parent's own UVs and the
    parent's own sooted material (`skin_fragment`). Without that a static
    fragment carries `_clad_material`'s world triplanar of a UNIQUE atlas,
    which is a smear of unrelated parts of the sheet and is what the user saw:
    "the ragged break pieces that DO exist ... are a completely diff
    texture/color from the wall they extend". The LOOSE fragments are not
    skinned — they let go and fell, and the dark end is where they belong.

    Everything here draws from the PRIVATE `prng` and, through the ctx swap in
    `_own_rng`, from the private `nrng` — zero shared draws. `facade_skin` /
    `skin_fragment` draw NOTHING at all, which is what lets the tear skin be
    added to the frozen MCE kit path without moving one later outcome.
    """
    from . import damage, quake_flow as qf

    stage = ctx["stage"]
    btype = ctx["info"]["type"]
    pitch = qf._p_pitch(ctx)
    Hm = max(3.0, float(m["top"]) - float(m["z0"]))
    n = 0
    skin_stats = ctx.setdefault("_tear_skin",
                                {"uv": 0, "tone": 0, "none": 0, "pieces": 0,
                                 "no_facade": 0})
    for j in jobs:
        if j.get("dropped"):
            continue
        e = j["el"]
        if e.get("dead"):
            continue
        path = (e.get("p") or {}).get("prim_path")
        if not path:
            continue
        judges, vside = [], j["side"]
        for cut in j["cuts"]:
            if cut["kind"] == "v":
                judges.append(qf._p_vcrack_judge(
                    m, j["side"], cut["line"], prng, btype=btype,
                    amp=cut["amp"], pitch=pitch, loose_hi=cut["loose_hi"]))
            elif cut["kind"] == "z":
                judges.append(qf._p_zline_judge(
                    m, j["side"], cut["line"], prng, btype=btype,
                    amp=cut["amp"], loose_above=cut["loose_above"],
                    pitch=pitch))
            else:
                vside = cut["side"]
                judges.append(qf._p_toward_judge(
                    m, cut["side"], cut["frac"], prng, btype=btype,
                    z0=float(m["z0"]), span_z=Hm, pitch=pitch))
        if not judges:
            continue
        if len(judges) == 1:
            judge = judges[0]
        else:
            def judge(c, _js=tuple(judges)):
                for q in _js:
                    if q(c):
                        return True
                return False
        tex = damage.bound_texture(stage, path)
        # MEASURED BEFORE THE SPLIT. `_break_split` deactivates the source and
        # a deactivated prim has no points, so this is the last moment the
        # wall's own surface can be read.
        skin = facade_skin(ctx, path, e.get("out")
                           or (qf._outward(m, j["side"]) + (0.0,)))
        if skin is None:
            skin_stats["no_facade"] += 1
        else:
            skin_stats["pieces"] += 1
        st_p, lo_p = qf._break_split(
            ctx, path, prng.randint(*EDGE_PIECES), judge,
            qf._mat_fn(ctx, tex, 0.35),
            static_mat=(qf._clad_material(stage, ctx["parent"], ctx["cache"],
                                          tex) if tex else None),
            refine_max=EDGE_REFINE_MAX)
        if not st_p and not lo_p:
            j["failed"] = True
            continue
        # THE STANDING HALF IS STILL THAT WALL. Only the statics: a loose
        # fragment is in the heap.
        for q in st_p:
            how = skin_fragment(ctx, skin, q)
            skin_stats[how or "none"] += 1
        # The shards drop at the lip of the hole and join the heap under it:
        # they were never thrown, they let go, so this is a fraction of the
        # peeled wall's own `THROW_TOP` and it is biased downward.
        ox, oy = qf._outward(m, vside)
        for q in lo_p:
            v = prng.uniform(0.35, 1.30)
            ctx["velocity"][q] = (ox * v, oy * v, -0.15 * v)
        ctx["loose"] += lo_p
        ctx["static_extra"] += st_p
        e["dead"] = True
        j["torn"] = True
        j["n_static"] = len(st_p)
        j["n_loose"] = len(lo_p)
        j["no_static"] = not st_p
        n += 1
    return n


def _inner_rubble(ctx, m, plan, h, prng):
    """The smaller heap INSIDE the wall line.

    `quake_flow._heap(fill=False)` only ever places outward (its windrow `d`
    is drawn positive from the wall line) and `fill=True` is a dome over the
    WHOLE footprint, which is a total collapse. What is wanted here is a band
    of rubble a few metres deep just inside the opening: the inner leaf, the
    plaster, and the broken ends of the floors that bore on the wall that
    went. Authored with `_a_lump` the same way `r_expose_interior` authors
    what landed on a floor.
    """
    from . import quake_flow as qf
    from . import urban_fire as uf

    stage = ctx["stage"]
    side = h["side"]
    lo, hi = h["along_m"]
    L = side_length(m, side)
    dep = float(h["depth_m"])
    base = float(h["base_z"])
    span_m = max(1.0, hi - lo)
    n = int(max(24, min(420, span_m * dep * 2.4)))
    made = []
    for _ in range(n):
        t = prng.uniform(lo, hi)
        d_in = 0.35 + abs(prng.gauss(0.0, dep * 0.42))
        if d_in > dep:
            continue
        lx, ly = wall_point(m, side, min(L, max(0.0, t)), out_m=-d_in)
        wx, wy = qf._to_world(m, lx, ly)
        sz = IN_LUMP[0] + (IN_LUMP[1] - IN_LUMP[0]) * prng.random() ** 1.8
        # taper away from the wall: most of it is at the foot of the opening
        zmax = 0.9 * max(0.0, 1.0 - (d_in / dep) ** 1.3)
        r = prng.random()
        mat = (ctx["mats"]["char_concrete"] if r < 0.42 else
               ctx["mats"]["soot"] if r < 0.78 else
               uf._debris_mat(ctx))
        path = "{0}/pcin_{1}_{2}".format(ctx["parent"], ctx["tag"], qf._uid(ctx))
        qf._a_lump(stage, path, wx, wy,
                   base + prng.uniform(0.0, zmax) + sz * 0.15, sz, prng, mat,
                   jitter=0.45)
        ctx["authored"].append(path)
        ctx["static_extra"].append(path)
        made.append(path)
    return made


# ---------------------------------------------------------------------------
# Host-side self-check
# ---------------------------------------------------------------------------
def check(verbose=True, styles=("commercial_mid", "commercial", "apartment",
                                "block_residential", "dw_terrace")):
    """No stage, no pxr: plan a partial collapse on real kit buildings and
    assert the invariants the recipe is built around.

    Runs the same path `scene_gen/tests/test_fire_collapse.py` does; kept here
    so a launch script can gate on it the way it gates on `urban_fire.check`.
    """
    import numpy as np
    from detail import urban_building as ub
    from . import quake_flow as qf
    from . import urban_fire as uf

    bad = []
    for style in styles:
        for mode in ("elevation", "corner"):
            rng = random.Random(11)
            pls = ub.build_building(style, 0.0, 0.0, 0.0, rng)
            info = qf.describe(style, pls, 0.0, 0.0, 0.0)
            ctx = {"info": info, "rng": rng, "nrng": np.random.default_rng(11),
                   "notes": [], "tag": "chk"}
            ctx["fire"] = uf.plan_fire(info, FIRE_LEVEL, rng, origin=1,
                                       sides=("S", "E"))
            try:
                plan = plan_partial_collapse(ctx, mode=mode)
            except Exception as exc:
                bad.append("{0}/{1}: plan raised {2}".format(style, mode, exc))
                continue
            m = info["masses"][plan["mass"]]
            for e in plan["kill"]:
                if e["side"] not in plan["sides"]:
                    bad.append("{0}/{1}: killed {2} on {3}, not in {4}".format(
                        style, mode, e["name"], e["side"], plan["sides"]))
                    break
                if int(e["storey"]) < plan["s0"]:
                    bad.append("{0}/{1}: killed {2} at storey {3} < s0 {4}"
                               .format(style, mode, e["name"], e["storey"],
                                       plan["s0"]))
                    break
            if plan["s0"] < ctx["fire"]["origin"]:
                bad.append("{0}/{1}: failure line {2} below the fire origin {3}"
                           .format(style, mode, plan["s0"],
                                   ctx["fire"]["origin"]))
            if not plan["kill"]:
                bad.append("{0}/{1}: nothing to take away".format(style, mode))
            for h in plan["heaps"]:
                want = 1.0 if h["where"] == "outside" else -1.0
                if h["outward_m"] * want <= 0.0:
                    bad.append("{0}/{1}: {2} heap at {3:+.2f} m of the wall "
                               "line, wrong side".format(
                                   style, mode, h["where"], h["outward_m"]))
            kept = [e for e in info["elements"]
                    if e["side"] in plan["keep_sides"]
                    and e["role"] in SHELL_ROLES]
            lost_on_kept = [e for e in plan["kill"]
                            if e["side"] in plan["keep_sides"]]
            if lost_on_kept:
                bad.append("{0}/{1}: {2} module(s) taken off an untouched "
                           "elevation".format(style, mode, len(lost_on_kept)))
            if not kept:
                bad.append("{0}/{1}: no untouched elevation left".format(
                    style, mode))
    if verbose:
        print("[fire_collapse] check {0}".format("ok" if not bad else "FAILED"))
        for b in bad:
            print("  " + b)
    return bad


if __name__ == "__main__":                                    # pragma: no cover
    import sys
    sys.path.insert(0, os.path.normpath(
        os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")))
    raise SystemExit(1 if check() else 0)
