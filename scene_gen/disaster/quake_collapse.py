"""quake_collapse — the EARTHQUAKE collapse family, built on the urban-fire
partial-collapse primitives.

WHY THIS MODULE EXISTS
----------------------
The user, looking at the first 500 m M7.8 city on screen (2026-08-30):

    "How does urban fire do partial collapse damage to modern city
     environment. I want you to use that, it looks more realistic."
    "lots of material mismatches. damaged part and undamaged part look like
     completely diff materials, etc. I think urban fire fixes this (don't use
     the scorching from urban fire) only use the partial collapse mechanism
     and expand to total collapse, etc"
    "the debris you are forming has to be more realistic than what it is rn."

`disaster/fire_collapse.py` (level F5c) is the mechanism that reads right.
Nothing about it is a fire mechanism except its palette:

  1. `plan_partial_collapse` is a PURE PLANNER — no `pxr`, no stage — so
     every decision is arithmetic on the element table and every invariant is
     host-testable. `r_partial_collapse` is only the USD authoring of a plan
     that has already been checked.
  2. The kill list is quantised to the KIT MODULE GRID and shaped by a
     STAIRCASE that widens upward (`PROFILE_FOOT`), never a rectangle and
     never one long diagonal.
  3. Every SURVIVING module that touches a dead one is torn on the touching
     edge (`plan_edges` -> `_tear_perimeter`), so the outline of the hole is
     made of Voronoi cell boundaries and not of module joints. That is the
     whole answer to "sharp straight or rectangular cuts".
  4. Fragments come out of `quake_flow._break(mode="uniform")` AFTER
     `fracture.solidify`, so ~82 % of a fragment's faces keep the module's
     OWN cladding (`_clad_material`, triplanar) and only the `core` face
     subset — the faces the fracture invented — takes an interior material.
     That is the whole answer to "damaged part and undamaged part look like
     completely different materials".
  5. Per-fragment outward throw (`THROW_BASE + THROW_TOP * z/H`) instead of a
     scene-wide bias, so a wall that let go lands as a fan in the street.

This module is that mechanism with the earthquake's own failure modes on top
of it and NONE of the fire's semantics: no soot, no scorch, no `ctx["fire"]`,
no `soot_plume`, no `_refire`, no `bind_break` char palette, no `HEAP_MIX`,
no `_a_dustify`. Fragments keep the module cladding and fallen pieces get
`quake_flow._dust_loose` — a dusted COPY of each fragment's OWN material,
texture kept, tint applied. One material family per building, dusty where it
broke.

WHAT IT DOES **NOT** REPLACE
----------------------------
Every recipe in `quake_flow.RECIPES` is untouched, and `quake_flow.LADDER` is
untouched, so `EQ_LADDER=legacy` reproduces today byte-for-byte. The new modes
are registered under their own names (`qc_*`) and selected by a SECOND ladder
table, `LADDER_QC`, composed in `quake_flow` from `LADDER` plus this module's
`LADDER_OVERRIDES` (so the DG0-DG2 rows — glass, scars, rooftop plant — are
literally the same objects the legacy table holds and cannot drift).

THE MODES
---------
| mode | what comes away | who uses it |
|---|---|---|
| `elevation` | one WEAK elevation (the most-glazed face, or a caller's) from a failure line up, staircased; URM peels from the parapet down, RC drops its infill from a storey up | urm DG4, rc DG3 (`infill=True`) |
| `corner` | the corner two elevations share, a bay and a half of each, from the failure line up | urm DG3 (drawn) |
| `soft_storey` | a crush band at storey k on ALL FOUR sides, bitten into the storeys above and below so the band is not a clean slice; the block above drops and leans/racks | rc DG4, rc_glass DG4 |
| `mid_storey` | the same band at k >= 1, block above twisted in plan and offset | rc DG4 (drawn) |
| `pancake` | every storey's shell from s0 up, progressively wider; the slabs re-authored as a stack at a pitch | rc DG5 |
| `total` | every shell module including the roof and the parapet ring; a standing stub on the blind / party-wall sides | urm DG5 |
| `auto` | corner or elevation, drawn from the building's own private rng | urm DG3 |

THE RNG CONTRACT — THIS FAMILY CONSUMES ZERO SHARED DRAWS
---------------------------------------------------------
`wreck_building` runs one ladder in order off ONE `random.Random`, so a
recipe that draws a different NUMBER of values than it did yesterday moves
every LATER recipe's outcome. `fire_collapse` records the same rule for
`urban_fire.LADDER`. So, exactly as there:

* the generators are `random.Random(seed)` and `numpy.random.default_rng(seed)`
  with `seed` derived from the BUILDING (`zlib.crc32` of tag/style/type/pose,
  never `hash()` — `hash()` of a `str` is salted per process and would make a
  plan irreproducible between two runs of the same scene);
* `fire_collapse._own_rng` INSTALLS both on the ctx for the duration of the
  authoring, because `quake_flow._rubble`, `_a_slab_rim`, `_ragged_slabs`,
  `_disturb_interior`, `_dust_loose` and `r_droop` all read `ctx["rng"]` /
  `ctx["nrng"]` off the ctx and take no generator argument;
* the shared pair is restored in `__exit__`.

`plan_collapse` never touches the ctx at all: it takes its own generator and
returns. `tests/test_quake_collapse.py` advances the shared rng 50 times
between two plans of the same building and asserts they are identical.

TESTED WITHOUT A STAGE
----------------------
`plan_collapse` imports no `pxr` (and this module imports none at module
scope — every USD import is inside an authoring function), so the whole plan
is checkable host-side on REAL kit buildings:
`scene_gen/tests/test_quake_collapse.py`.

WHAT THE TESTS CANNOT SEE: whether the fan actually lands where the plan says,
whether the block above a soft storey reads as crushed rather than lifted,
whether `_dust_loose`'s tint is deep enough at 40 m. That needs a render.
"""

import os
import random
import zlib


# ---------------------------------------------------------------------------
# Knobs
# ---------------------------------------------------------------------------
# The private seed. XOR'd into the building's own crc32 so this family's draws
# are stable per building and independent of the shared ladder rng.
QC_SEED_XOR = 0x9CE1

# Roles a collapse may take away. Same set `fire_collapse.SHELL_ROLES` holds —
# `balcony` included, because a cantilever off a wall that is no longer there
# has nothing to hang from.
SHELL_ROLES = ("wall", "corner", "parapet", "parapet_corner", "balcony")
# ...and, unlike the fire's, a TOTAL collapse takes the roof with it. The fire
# ladder excludes `roof` deliberately (a burnt-out shell keeps its skyline);
# a URM building that has come down has no roof left standing anywhere.
ROOF_ROLES = ("roof",)
ALL_ROLES = SHELL_ROLES + ROOF_ROLES

# THE STAIRCASE, `fire_collapse`'s own: the share of the region's full width
# lost at the FAILURE LINE, ramping to 1.0 at the top of the mass. 0.55 gives
# roughly two module steps over a five-storey band. 1.0 is a rectangular hole,
# which is the artefact the profile exists to avoid.
PROFILE_FOOT = 0.55
# Share of the elevation lost at the TOP, drawn per building. Never all of it:
# a collapse that leaves no stub of wall standing at either end reads as a
# demolition, and the stub is what tells a viewer how tall the wall was.
SPAN_FRAC = (0.62, 0.86)
# Corner reach, per adjoining side, measured FROM the corner in MODULES — not
# as a share of the side. `fire_collapse` records the measurement: 0.45 of
# `block_residential`'s 88 m south elevation is FORTY-ONE METRES, which is
# half the building. `quake_flow.r_corner_fail` writes the same reach as
# `max(4, module) * 1.6` for the same reason.
CORNER_REACH_MODULES = (1.3, 2.3)
CORNER_REACH_MAX_FRAC = 0.40

# HOW TALL THE LOST STRIP MAY BE on the elevation/corner modes. A 60 m strip
# of elevation on the ground is a bombing. The band is capped from the TOP
# DOWN and the failure line rises to meet it.
MAX_FALL_STOREYS = 6
# A LAST-RESORT BUDGET on how many kit modules may be fractured. Each one is
# a `_break` into 8-13 pieces plus a solidify, so this is the recipe's whole
# cost. Over budget the failure line is raised a storey at a time (the notch
# gets shallower, not narrower, so the staircase profile survives) — and if
# it still does not fit the overrun is REPORTED, never silently dropped.
MAX_MODULES = 130
# ...and the same shape for the edge tears (each is a `_break_split`).
MAX_EDGE_MODULES = 200
# A band / total / pancake mode sweeps whole storeys and cannot raise a
# failure line to get under a budget: a total collapse that left half the
# building standing to save fracture time is not a total collapse. So for
# those modes this is a COST WARNING, not a limit — `plan["budget"]
# ["over_modules"]` reports the overrun and the note says so, and nothing is
# ever dropped. (`r_masonry_collapse` / `r_pancake` have no cap at all today,
# so an overrun here is a report of what the legacy recipes already spend.)
#
# MEASURED, host-side, at the 420 line: `block_stone` total 792 modules over
# 3 masses, `highrise_step` 350 over 2, `block_residential` 2,558 over 5 —
# and that last one is 1,194 on `wing3` alone, which is the number worth
# seeing in a bake log before wondering why one building took ten minutes.
MAX_MODULES_TOTAL = 420

# WHERE THE FAILURE LINE STARTS, per construction type. A URM elevation peels
# from the PARAPET DOWN — the top one or two storeys are the unrestrained
# ones (no diaphragm over them), which is why the reconnaissance photographs
# of out-of-plane failure are nearly all top-storey. An RC frame does not
# peel: it loses its INFILL, which starts at the first floor (the ground
# storey is the shopfront and is glass, not masonry).
URM_TOP_DOWN = (1, 2)
RC_FROM_STOREY = 1
# The corner mode reaches further down a frame than a masonry parapet: on RC
# the corner COLUMN zone is what failed, and it fails over several storeys.
CORNER_STOREYS = {"urm": (2, 3), "rc": (3, 4), "rc_glass": (2, 3)}

# THE BAND IS NOT A CLEAN SLICE. A crushed storey whose kill list is exactly
# `storey == k` leaves a dead-level line top and bottom, right on two slab
# lines — the "building lifted off with a pallet fork" artefact the first
# mid-storey bench showed and the one the round-4 review photographed again
# (`r4_commercial/4_commercial_soft_storey_ne.png`). So the band BITES into
# the storey above (the bottom of the block that came down with it) and, at a
# mid storey, into the one below.
BAND_BITE_UP = (0.14, 0.34)
BAND_BITE_DOWN = (0.08, 0.20)

# PANCAKE. The lowest storeys keep a run at each end (the frame did not go
# all at once); everything from `PANCAKE_RAMP_STOREYS` up goes whole.
# 0.62, MEASURED, not 0.74. A kit piece is pivoted at one END and the kill
# window carries half a module of tolerance either side, so on a 20 m wall a
# 0.74 run leaves 2.6 m at each end — less than one 4-5 m module, and NO
# module survives the bottom storey at all: the ramp is invisible and the
# whole pancake reads as a clean vertical slice again. 0.62 leaves 3.8 m,
# which is one module per end per side, which is what `plan_edges` then tears.
PANCAKE_FOOT = 0.62
PANCAKE_RAMP_STOREYS = 2
# `r_pancake`'s own stacking numbers, reused verbatim so the two look alike.
PANCAKE_PITCH = (0.55, 0.95)
PANCAKE_TILT = (-4.0, 4.0)
PANCAKE_JITTER = 0.6
PANCAKE_ROOF_TILT = (3.0, 9.0)

# THE STANDING STUB after a total collapse. Research (`eq_round4_rubble_
# research.md` sec5a, and the `STUB_KEEP_P` row of the skill's constants
# table): ~40-60 % of totally-collapsed buildings keep a stub, and where one
# survives it is the party wall / blind side — the wall that was braced by
# its neighbour. `r_masonry_collapse` keeps one on EVERY side, which is why
# its DG5 reads as a bathtub; here it is the blind sides only.
STUB_FRAC = (0.25, 0.62)
BLIND_GAP_M = 2.5
STUB_KEEP_P = 0.5
STUB_SIDES_FALLBACK = (0, 2)

# THROW. A wall rotating about its foot: the top leads (`r_out_of_plane`'s own
# 0.4 + 2.4 z/H profile, which the bench's `max_speed=6.0` never saturates).
THROW_BASE = 0.40
THROW_TOP = 2.40
# A TOTAL collapse drops nearly straight down — the outward fan belongs to a
# peel, not to a building falling into its own footprint (`r_masonry_collapse`
# makes the same correction for the same reason).
TOTAL_THROW_BASE = 0.20
TOTAL_THROW_TOP = 0.90
# A CRUSHED BAND IS NOT THROWN AT ALL. It is squeezed out sideways at walking
# pace under the mass above it; `r_soft_storey` uses 0.3-1.2 m/s.
BAND_THROW = (0.3, 1.2)
BAND_PUSH_M = (0.8, 2.2)

# FRACTURE. `mode="uniform"` and NOT `quake_flow._p_frac_kw` — that is the
# whole point of the round-5 rebuild. `_p_frac_kw` seeds the fracture on the
# masonry BOND (`mode="brick"`) or on a prism lattice (`mode="prism"`), and a
# lattice makes cuboids: "clean toy-brick prisms" is exactly what the user
# photographed in `r4_commercial/0_commercial_DG5_close.png`. The fire path
# takes uniform Voronoi cells and its rubble reads as rubble.
BREAK_PIECES = (8, 13)
BREAK_CONSUME = 0.26
BREAK_ROUGH = 0.014
# `_break` overrides this to 0.18 whenever the piece has a readable texture
# AND a core subset (see its own note) — which is the rule that keeps the
# cladding on ~82 % of the faces. It is passed here at the fire's own value so
# the fallback (a piece with no texture at all) matches the fire path too.
INNER_P = 0.4
# A total collapse sheds more, smaller, and thins harder: what lands on the
# CROWN of the pile is always on top, so the only lever that works is how many
# there are (`r_masonry_collapse`'s own note, and its own numbers).
TOTAL_PIECES = (18, 24)
TOTAL_CONSUME = 0.80
TOTAL_MAX_PIECE_M = 0.9
PANCAKE_PIECES = (15, 21)
PANCAKE_CONSUME = 0.55
PANCAKE_MAX_PIECE_M = 1.2
BAND_PIECES = (8, 12)
BAND_CONSUME = 0.45

# THE PILES. Every one of them goes through `quake_flow._rubble`, which is
# `EQ_RUBBLE`-gated (v2 = the round-4 mound/large-elements/instancer planner,
# v1 = the old `_heap` box crate). Nothing here ever calls `_heap` directly.
FAN_DEPTH_M = (1.0, 1.9)          # r_out_of_plane's own figures
FAN_SPREAD = (0.22, 0.40)
INFILL_DEPTH_M = (0.55, 1.00)     # dropped infill panels, not a fallen wall
CORNER_DEPTH_M = 0.8              # r_corner_fail's v2 value
CORNER_ALONG_PAD_M = 2.0
COLLAR_DEPTH_M = (0.5, 0.9)       # r_soft_storey's collar
DOME_CROWN_FRAC = 0.28            # FEMA's 0.33 air-space factor, as used today
DOME_SPREAD = (0.20, 0.34)
PANCAKE_CROWN_FRAC = 0.85
PANCAKE_SPREAD = (0.30, 0.45)
PANCAKE_LARGE_BUDGET = 4          # the stack carries what a pancake pile shows

# How deep into the plan the "lost region" reaches, as a share of the mass's
# other dimension. This is what restricts the position sweep on the
# elevation/corner modes: a static prim goes to the solver only when it is
# above the failure line AND inside this strip. Without it the sweep drops the
# far half of the roof deck and the plant on it.
REGION_DEPTH_FRAC = 0.26
REGION_DEPTH_MIN_M = 2.6

# THE TEETH AT THE BREAK. Bearing stubs still in the wall pocket where a floor
# framed into the lost wall. Steel on a frame, timber on masonry — the fire
# path authors these as `steel` posts unconditionally, which is right for a
# burnt shell and wrong for a collapsed brick block.
TEETH_N = (2, 5)
TEETH_LEN_M = (0.5, 1.8)
TEETH_SIZE_M = (0.16, 0.24)

# Which kit names read as an OPENING, for `weak_side`. A pattern match, not a
# measurement: `urban_building` carries no "has an opening" flag on a piece
# (that is a `quake_sliced` / GAC-slice concept), and `quake_flow.
# _pick_opening_panels` makes the same call the same way.
OPENING_PAT = ("facade", "window", "shop", "store", "door", "arch", "glass",
               "display", "entry")


def _f(name, default):
    """An override from the environment, for a bench run that wants to push
    one number without a code change."""
    v = (os.environ.get(name) or "").strip()
    try:
        return float(v) if v else float(default)
    except ValueError:
        return float(default)


MODES = ("elevation", "corner", "soft_storey", "mid_storey", "pancake",
         "total")


# ---------------------------------------------------------------------------
# The private generator
# ---------------------------------------------------------------------------
def plan_seed(ctx, mode, seed=None, mass=None):
    """The seed every draw in a plan comes from.

    `zlib.crc32` OF THE BUILDING, never `hash()`. `fire_collapse._private_rng`
    falls back to `abs(hash(key))` when `soot_plume` is missing, and `hash()`
    of a `str` is salted per PROCESS (`PYTHONHASHSEED`) — a plan drawn from it
    is not reproducible between two runs of the same scene, which is exactly
    what a baked archetype library needs it to be. crc32 is stable, cheap and
    has no import cost.

    The REQUESTED mode is part of the key, so `qc_elevation` and `qc_corner`
    on one building are different draws; the pose is part of it, so two
    buildings of one style in one city are different draws even if a caller
    hands them the same tag; and the MASS is part of it, so the four wings a
    total collapse brings down do not all draw the same stub sides and the
    same pile.
    """
    if seed is not None:
        return int(seed) & 0xFFFFFFFF
    info = ctx.get("info") or {}
    key = "|".join(str(q) for q in (
        ctx.get("tag", ""), info.get("style"), info.get("type"),
        info.get("x"), info.get("y"), info.get("yaw"), mode,
        mass or "main"))
    return (zlib.crc32(key.encode("utf-8")) ^ QC_SEED_XOR) & 0xFFFFFFFF


# ---------------------------------------------------------------------------
# Pure geometry helpers — no pxr, no stage
#
# The frame arithmetic (`side_length`, `outward_of`, `along_of`,
# `wall_point`, `el_span`, `el_z_span`, `el_footprint`, `in_region`,
# `region_side`, `corner_sides`, `corner_at_high_end`, `corner_of_end`,
# `other_side`, `shared_corner`) is `fire_collapse`'s and is IMPORTED, not
# copied: it is the same mass frame, and a second copy of `outward_of` with a
# sign flipped is the class of bug that puts a windrow inside a building.
# ---------------------------------------------------------------------------
def _fc():
    from . import fire_collapse as fc
    return fc


def _qf():
    from . import quake_flow as qf
    return qf


def blind_sides(ctx, given=None, prng=None):
    """The sides a collapsed building may keep a standing stub on.

    A stub survives where the wall was BRACED — the party wall of a terrace,
    the blind flank against the next block. `quake_flow.d_set_neighbours` /
    `ctx["neighbours"]` is where the assembly records that, so a neighbour
    closer than `BLIND_GAP_M` makes its side blind.

    With no neighbour registry at all (a bench building standing alone, a
    monolith the assembly never paired) there is no ground truth, so the
    research's own `STUB_KEEP_P` is used as a DRAW instead: about half of
    total collapses keep a stub, and when one is kept it is on one or two
    sides. `prng` is required for that branch; without one the answer is
    simply "no blind side", which keeps the function pure for a caller that
    only wants the registry's answer.
    """
    if given is not None:
        return tuple(s for s in ("S", "E", "N", "W") if s in tuple(given))
    qf = _qf()
    nbs = qf._d_nbs(ctx) if ctx.get("info") is not None else []
    out = sorted(set(n["side"] for n in (nbs or ())
                     if float(n.get("gap", 1e9)) <= BLIND_GAP_M))
    if out:
        return tuple(s for s in ("S", "E", "N", "W") if s in out)
    if prng is None:
        return ()
    if prng.random() >= STUB_KEEP_P:
        return ()
    n = prng.randint(*STUB_SIDES_FALLBACK)
    pool = ["S", "E", "N", "W"]
    prng.shuffle(pool)
    return tuple(s for s in ("S", "E", "N", "W") if s in pool[:max(1, n)])


def weak_side(ctx, mtag, prng, storeys=None):
    """The elevation a quake takes first: the one with the most OPENINGS.

    A wall's out-of-plane capacity is what is left of it after the windows are
    cut out, and the storefront elevation — a shopfront under four storeys of
    masonry — is the classic URM street-wall failure. There is no per-piece
    opening flag on this kit, so this is a name-pattern share (the same call
    `quake_flow._pick_opening_panels` makes), with a small bonus for the side
    that simply has more wall on it and a front bias to break a tie: `S` is
    the street elevation in `urban_building`'s own frame, and the bench
    cameras aim at it.

    Deterministic: the same building and the same generator give the same
    answer, and the number of draws taken depends only on the building.
    """
    qf = _qf()
    score = {}
    counts = {}
    for sd in ("S", "E", "N", "W"):
        els = [e for e in qf._els(ctx, mass=mtag, role=("wall", "corner"))
               if e["side"] == sd
               and (storeys is None or int(e["storey"]) in storeys)]
        counts[sd] = len(els)
        if not els:
            score[sd] = -1.0
            continue
        n_open = sum(1 for e in els
                     if any(k in (e.get("name") or "").lower()
                            for k in OPENING_PAT))
        score[sd] = n_open / float(len(els)) + 0.002 * len(els)
    best = max(score.values())
    tied = [sd for sd in ("S", "E", "N", "W") if score[sd] >= best - 1e-9]
    if len(tied) == 1:
        return tied[0]
    if "S" in tied and prng.random() < 0.55:
        return "S"
    return tied[prng.randrange(len(tied))]


def collapse_masses(info, mode, mass=None):
    """Which masses ONE call brings down.

    A TOTAL collapse and a PANCAKE take the whole building — `r_masonry_
    collapse` iterates `info["masses"].items()` and `r_pancake` iterates
    `[t for t in masses if t == mass or t.startswith(mass) or mass ==
    "main"]`, both for the same reason: `urban_building` builds a block as a
    main mass plus wings, and on the measured stock that is most of the tall
    stock — `block_residential` is 5 masses and 2,377 elements with only 181
    of them on `main`, `block_stone` is a 1-level base under two 8-storey
    wings. A "total collapse" that swept `main` alone would leave 584 modules
    of eight-storey wing standing on a heap.

    A PARTIAL mode (elevation, corner) and a BAND mode are single-mass by
    definition: one elevation of one mass peels, one storey of one mass
    crushes, and `_author_band` already carries every OTHER mass down with the
    block above (`r_soft_storey`'s own `mass == "main" and e["mass"] !=
    "main"` rule). An explicit `mass=` from the caller always wins.
    """
    mtag = mass or "main"
    if mtag not in info["masses"]:
        mtag = "main"
    if mode not in ("total", "pancake") or mass is not None:
        return [mtag]
    return [t for t in info["masses"]
            if t == mtag or t.startswith(mtag) or mtag == "main"]


def _spans_elevation(m, sides, s0, top_i, n_lv, base, centres, corner=None,
                     reach_m=None, foot=None):
    """The staircase, in metres along each wall from its low end.

    `fire_collapse.plan_partial_collapse`'s own profile: the width grows from
    `foot` of the full run at the failure line to all of it at the top, so
    what is left standing is a NOTCH on the module grid with a ragged edge.
    Anchored at the corner in corner mode, centred (wandering a little off
    centre) otherwise.
    """
    fc = _fc()
    # RESOLVED AT CALL TIME, not as a default argument: `foot=PROFILE_FOOT` in
    # the signature binds the module constant once, at import, so a bench (or
    # a test that flattens the profile to prove the staircase is real) could
    # set `quake_collapse.PROFILE_FOOT` and change nothing at all.
    foot = PROFILE_FOOT if foot is None else float(foot)
    out = {}
    for sd in sides:
        L = fc.side_length(m, sd)
        at_hi = fc.corner_at_high_end(sd, corner) if corner else None
        for s in range(s0, n_lv):
            k = (0.0 if top_i == s0
                 else (s - s0) / float(top_i - s0))
            ramp = foot + (1.0 - foot) * k
            if corner:
                w = min(reach_m * ramp, CORNER_REACH_MAX_FRAC * L)
                t0, t1 = ((L - w, L) if at_hi else (0.0, w))
            else:
                w = min(L, base * ramp * L)
                c = centres[sd]
                t0 = max(0.0, min(c - w / 2.0, L - w))
                t1 = min(L, t0 + w)
            out[(sd, s)] = (float(t0), float(t1))
    return out


def _kill_from_spans(ctx, m, mtag, spans, roles, pad, skip=()):
    """Every element of `mtag` whose pivot falls inside its own (side, storey)
    span. `skip` is a set of `id(element)` the caller has already claimed (the
    stub jobs), so one prim is never handed to two fracture calls —
    `_break_split` deactivates its source and the second call would fracture
    a dead prim.
    """
    qf = _qf()
    out = []
    for e in qf._els(ctx, mass=mtag, role=roles):
        if id(e) in skip:
            continue
        iv = spans.get((e["side"], int(e["storey"])))
        if not iv:
            continue
        lo, hi = iv
        t = qf._p_el_t(m, e["side"], e)
        if lo - pad <= t <= hi + pad:
            out.append(e)
    return out


def _heap_centres(m, side, along_m, out_m):
    """`{"local", "world", "outward_m"}` for one side of a pile, `out_m`
    metres OUTSIDE the wall line at the middle of its run.

    Every windrow/fan `quake_flow._rubble` authors is placed OUTWARD from the
    wall line by construction (`quake_rubble._side_axes` offsets by
    `+halfD + offset_m` and reaches further out still; v1's `_heap` draws its
    windrow distance positive). So a plan whose centre comes out NEGATIVE
    here has put the rubble inside the building, and the test asserts on
    exactly this number at four yaws.
    """
    fc, qf = _fc(), _qf()
    lo, hi = along_m
    lx, ly = fc.wall_point(m, side, 0.5 * (lo + hi), out_m=float(out_m))
    wx, wy = qf._to_world(m, lx, ly)
    return {"local": (lx, ly), "world": (wx, wy),
            "outward_m": fc.outward_of(m, side, lx, ly)}


# ---------------------------------------------------------------------------
# THE PLAN — every decision, host-side
# ---------------------------------------------------------------------------
def plan_collapse(ctx, mode="elevation", mass=None, side=None, corner=None,
                  storey=None, from_storey=None, span_frac=None, depth_m=None,
                  drop_slabs=1, max_storeys=None, max_modules=None,
                  max_edges=None, blind=None, infill=False, lean_side=None,
                  lean_deg=None, crush_m=None, twist_deg=None, offset_m=None,
                  pitch_m=None, stack_base_z=None, pile_base_z=None,
                  keep_stub=True, seed=None):
    """Decide what comes down, what tears, what falls and where it lands.
    NO USD, NO STAGE, NO SHARED DRAWS.

    Returns a dict. Every key is present in every mode; the ones that do not
    apply are empty (`{}` / `[]` / `None`), never absent, so a caller never
    has to branch on the mode to read the plan.

      mode          the RESOLVED mode: one of `MODES`. `"auto"` resolves to
                    `elevation` / `corner`; `soft_storey` with `storey=None`
                    resolves to `mid_storey` when the draw picks k >= 1.
      requested     what the caller asked for, before the resolution above.
      mass          mass tag the failure is in.
      btype         `urm` / `rc` / `rc_glass` — which recipe shape was used.
      sides         the elevation(s) that lose modules. One for `elevation`,
                    the corner's two for `corner`, all four for the band /
                    total / pancake modes.
      keep_sides    the elevations no module is taken off. Empty for the band
                    and total modes; they are still TORN at the band edges,
                    which is `tears`' business, not `kill`'s.
      corner        "SE" etc, or None.
      s0            the failure line: the lowest storey that comes down.
      storeys       every storey a span was drawn for, ascending.
      top_storey    the top of the mass (the staircase's own top).
      n_levels      len(mass["levels"]).
      band          [k] for `soft_storey` / `mid_storey`, else [].
      span          {(side, storey): (t0_m, t1_m)} — the staircase, in metres
                    along that wall from its LOW end.
      roles         the element roles this mode sweeps.
      kill          the element records that come away. NOT marked dead here;
                    `r_collapse` marks them as it breaks them.
      stub_jobs     [{el, side, storey, z_line, frac}] — the storey-0 wall
                    modules on a blind side that are TORN at `z_line` and left
                    standing as a stub instead of being killed.
      stub          {side: mean stub height, m} for the sides in `stub_jobs`.
      panels        element records kept WHOLE so the pile can lay them
                    half-buried on it (round-4 design table row 2: "1-3 wall
                    PANELS ... the building's own kit modules kept whole").
                    Never in `kill`, never in `tears`, and `EQ_RUBBLE=v2`
                    only — v1's `_heap` has no `panels` concept, and a module
                    kept whole for a pile that will never lay it is a wall
                    left standing in the middle of a total collapse.
      blind_sides   the sides `stub` was allowed to use (see `blind_sides`).
      tears         `fire_collapse.plan_edges` jobs: every SURVIVING module
                    that touches a dead one, with the class of its own edge
                    the hole is on (above/below/left/right/return) and the
                    numeric break line already drawn. This is what makes the
                    outline of the hole cell boundaries and not module joints.
      region        {side: (t0_m, t1_m, depth_m)} — the plan footprint of the
                    loss, used to restrict the position sweep.
      sweep         True when `r_collapse` should run that position sweep
                    (elevation / corner). The band / total / pancake modes
                    move what they move explicitly and must not sweep.
      heaps         [{kind, where, sides, along, along_m, depth_m, offset_m,
                     crown_m, spread_frac, elem_h_m, stub_h_m, base_z, budget,
                     tag, centres}] — one `quake_flow._rubble` call each.
                    `kind` is "dome" | "windrow" | "fan"; `centres` is
                    {side: {"local", "world", "outward_m"}} and is empty for a
                    dome (whose centre is the mass's own).
      drop          [(mass, storey)] — floor SLABS that go to the solver
                    WHOLE. `elevation` only: a slab that lost its bearing wall
                    on one side comes down as one plate. A pancake's slabs are
                    re-authored as `stack` and a total's are broken into
                    boards, so both leave this empty — a slab that is both
                    dropped and re-authored is one prim in two places.
      break_slabs   [(mass, storey)] — floor slabs broken into pieces instead
                    (`total`: a joisted timber floor comes apart into boards
                    and joists, and is then buried).
      fit_fall      [(mass, storey)] — storeys whose fit-out (partitions,
                    columns, and props unless `bury_props`) comes down
                    wherever it stands on that storey.
      fit_region_storeys
                    [storey] — storeys where the fit-out comes down only where
                    its FOOTPRINT reaches into `region` (the partial modes: a
                    partition behind the part of the wall that still stands is
                    in a room that still has its walls).
      bury_props    True when the building's contents end up UNDER the pile
                    (`quake_flow._a_bury_props`) rather than falling as loose
                    bodies — a total collapse and a pancake; an intact desk on
                    the crown of a DG5 heap is a round-2 review finding.
      teeth         True when bearing stubs are authored at the break. False
                    where the floor they would stand on is itself coming down.
      cut_z         world z above which a static prim inside `region` falls.
      crush         None, or the band mode's own geometry:
                    {storey, z_lo, z_hi, h_st, lean_side, geo, crush_m,
                     twist_deg, offset_m} where `geo` is EXACTLY what
                    `quake_flow._soft_storey_geometry` returned — the block
                    above is moved by `quake_flow._soft_storey_matrix(geo)`
                    and nothing here re-derives it.
      stack         None, or the pancake's slab stack:
                    {pitch_m, base_z, top_z, plates: [{storey, z, tilt_deg,
                     axis, jitter, sides, top}]}.
      roof          "kill" | "stack" | "ragged" — what happens to the roof.
      throw         (base, top) m/s for the per-fragment outward velocity.
      budget        {"modules", "edges", "trimmed_storeys", "over_modules",
                     "over_edges"} — what was spent and what did not fit.
      masses        [mass tag] — the masses this PLAN covers, always one.
                    `r_collapse` overwrites it on the first plan with the full
                    sweep (`collapse_masses`) and puts the other masses' plans
                    in `extra`, because a total collapse and a pancake take
                    the wings too.
      extra         [] on a plan; the other masses' plans on what `r_collapse`
                    returns.
      seed          the private seed everything above was drawn from.
      pad_m         half a module of tolerance on a span end (a kit piece is
                    pivoted at one end, so a piece whose pivot sits just
                    outside the span can still have most of its body inside).
      reaches_end   {side: (at_low_end, at_high_end)} at the top storey.
      infill        True when this is the RC infill variant of `elevation`.
    """
    fc = _fc()

    requested = mode
    info = ctx["info"]
    mtag = mass or "main"
    if mtag not in info["masses"]:
        mtag = "main"
    m = info["masses"][mtag]
    btype = info.get("type") or "urm"
    n_lv = max(1, len(m["levels"]))
    top_i = n_lv - 1
    seed = plan_seed(ctx, requested, seed, mass=mtag)
    prng = random.Random(seed)
    module_m = max(4.0, float(m.get("module") or 4.0))
    pad = 0.5 * module_m

    # ---- 0. resolve the mode --------------------------------------------
    if mode == "auto":
        # urm DG3: a corner or an elevation, drawn from the building's own
        # generator so the ladder's sequence is untouched either way.
        mode = "corner" if prng.random() < 0.55 else "elevation"
    if mode == "soft_storey" and storey is None:
        # rc DG4: `r_storey_collapse`'s own 2-in-3 split between a ground
        # soft storey (Northridge / Antakya) and a mid-storey crush (Kobe /
        # Mexico City), drawn privately.
        if n_lv >= 4 and prng.random() >= 0.66:
            mode = "mid_storey"
    if mode not in MODES:
        raise ValueError("quake_collapse: unknown mode " + repr(requested))

    plan = {"mode": mode, "requested": requested, "mass": mtag,
            "btype": btype, "seed": seed, "n_levels": n_lv,
            "top_storey": top_i, "pad_m": pad, "infill": bool(infill),
            "band": [], "stub_jobs": [], "stub": {}, "panels": [],
            "drop": [],
            "break_slabs": [], "fit_fall": [], "fit_region_storeys": [],
            "bury_props": False, "crush": None, "stack": None, "heaps": [],
            "region": {}, "sweep": False, "roof": "ragged", "teeth": False,
            "blind_sides": (), "extra": [],
            "throw": (THROW_BASE, THROW_TOP)}
    plan["masses"] = [mtag]

    # ---- 1. which elevations, which storeys, what dies -------------------
    if mode in ("elevation", "corner"):
        _plan_partial(ctx, plan, m, prng, mode, side, corner, from_storey,
                      span_frac, max_storeys, max_modules, drop_slabs,
                      infill, module_m)
    elif mode in ("soft_storey", "mid_storey"):
        _plan_band(ctx, plan, m, prng, mode, storey, lean_side, lean_deg,
                   crush_m, twist_deg, offset_m)
    elif mode == "pancake":
        _plan_pancake(ctx, plan, m, prng, from_storey, pitch_m, blind,
                      keep_stub, stack_base_z)
    else:
        _plan_total(ctx, plan, m, prng, blind, keep_stub, pile_base_z)

    # ---- 2. the plan footprint of the loss --------------------------------
    dep_default = max(REGION_DEPTH_MIN_M,
                      REGION_DEPTH_FRAC * min(float(m["W"]), float(m["D"])))
    dep = float(depth_m) if depth_m is not None else dep_default
    region = {}
    for sd in plan["sides"]:
        runs = [plan["span"][(sd, s)] for s in plan["storeys"]
                if (sd, s) in plan["span"]]
        if not runs:
            continue
        region[sd] = (float(min(a for a, _b in runs)),
                      float(max(b for _a, b in runs)), dep)
    plan["region"] = region

    # DOES THE LOSS REACH A CORNER? It decides which machinery rags which
    # edge; `plan_edges` asks the same question per storey, this is the top
    # storey's answer and is what a caller/test reads.
    reaches = {}
    for sd in plan["sides"]:
        L = fc.side_length(m, sd)
        iv = plan["span"].get((sd, plan["top_storey"]))
        if iv is None:
            runs = [plan["span"][(sd, s)] for s in plan["storeys"]
                    if (sd, s) in plan["span"]]
            iv = (min(a for a, _b in runs), max(b for _a, b in runs)) if runs \
                else (0.0, 0.0)
        reaches[sd] = (iv[0] <= pad, iv[1] >= L - pad)
    plan["reaches_end"] = reaches

    # ---- 3. THE WHOLE PERIMETER OF THE HOLE -------------------------------
    # A KIT SEAM IS A STRAIGHT LINE AND AN EARTHQUAKE DOES NOT CUT ONE.
    # `fire_collapse.plan_edges` asks, per SURVIVING module, "is a dead module
    # against me, and on which of my four edges" — so the hole's whole
    # perimeter is covered whatever shape the hole is. The kill list it is
    # given carries the STUB JOBS too: those modules are about to be broken by
    # a z-line split of their own and `_break_split` deactivates its source,
    # so a second job on the same prim would fracture a dead prim.
    #
    # ...and the ROOF is filtered out of it. A roof tile's `side` is whichever
    # wall its CENTRE happens to be nearest and its `storey` is the top one,
    # so leaving it in the dead table invents adjacencies on a wall it never
    # touched. `plan_edges`'s own `live()` filter already ignores roofs
    # (`fire_collapse.SHELL_ROLES` has no `roof`), so this only has to match
    # it on the dead side.
    edge_kill = [e for e in plan["kill"] if e["role"] in fc.SHELL_ROLES]
    edge_kill += [j["el"] for j in plan["stub_jobs"]]
    # ...AND THE PANELS. A module kept whole for the pile that also picks up a
    # tear job is `_break_split` into cells before `_lay_existing` ever sees
    # it, and the pile then lays a deactivated prim.
    edge_kill += list(plan["panels"])
    edge_plan = dict(plan)
    edge_plan["kill"] = edge_kill
    lim = int(MAX_EDGE_MODULES if max_edges is None else max_edges)
    tears = fc.plan_edges(ctx, edge_plan, m, prng, budget=lim)
    plan["tears"] = tears

    # THE BUDGET IS REPORTED, NEVER SILENTLY ENFORCED. `_plan_partial` raises
    # its failure line a storey at a time while it can (the notch gets
    # shallower, not narrower, so the staircase survives); a band / total /
    # pancake mode has no such lever — it sweeps whole storeys by definition —
    # so an overrun there has to be VISIBLE in the plan and in the note, and
    # an under-torn edge is a bug, not a saving.
    n_mod = len(plan["kill"])
    cap = int(plan.pop("_module_cap", MAX_MODULES))
    dropped_edges = len([j for j in tears if j.get("dropped")])
    plan["budget"] = {
        "modules": n_mod, "module_cap": cap,
        "edges": len(tears), "edge_cap": lim,
        "trimmed_storeys": int(plan.pop("_trimmed", 0)),
        "over_modules": max(0, n_mod - cap),
        "over_edges": dropped_edges,
    }
    return plan


# ---------------------------------------------------------------------------
# Per-mode planning
# ---------------------------------------------------------------------------
def _plan_partial(ctx, plan, m, prng, mode, side, corner, from_storey,
                  span_frac, max_storeys, max_modules, drop_slabs, infill,
                  module_m):
    """`elevation` and `corner`: a NOTCH in one or two elevations, staircased,
    with every other elevation untouched.

    THE FAILURE LINE IS A CONSTRUCTION-TYPE DECISION, not a fire origin. A URM
    elevation peels from the PARAPET DOWN: the top one or two storeys are the
    unrestrained ones and are what the reconnaissance photographs show going
    out of plane. An RC frame does not peel at all — it loses its INFILL, bay
    by bay, from the first floor up, and its ground storey is a shopfront made
    of glass rather than masonry.
    """
    fc = _fc()
    n_lv = plan["n_levels"]
    btype = plan["btype"]
    mtag = plan["mass"]
    top_i = plan["top_storey"]

    # ---- which elevation(s) ---------------------------------------------
    base_side = side or weak_side(ctx, mtag, prng)
    if mode == "corner":
        if not corner:
            nb = ("E", "W") if base_side in ("S", "N") else ("S", "N")
            corner = fc.shared_corner((base_side, nb[prng.randrange(2)]))
        sides = fc.corner_sides(corner)
    else:
        corner = None
        sides = (base_side,)
    plan["sides"] = tuple(sides)
    plan["corner"] = corner
    plan["keep_sides"] = tuple(s for s in ("S", "E", "N", "W")
                               if s not in sides)

    # ---- the failure line -------------------------------------------------
    if from_storey is not None:
        s0 = int(from_storey)
    elif mode == "corner":
        lo, hi = CORNER_STOREYS.get(btype, CORNER_STOREYS["urm"])
        s0 = n_lv - prng.randint(lo, hi)
    elif btype == "urm":
        s0 = n_lv - 1 - prng.randint(*URM_TOP_DOWN)
    else:
        s0 = RC_FROM_STOREY
    s0 = max(0, min(int(s0), n_lv - 1))
    # THE BAND IS CAPPED FROM THE TOP DOWN. A 60 m strip of elevation on the
    # ground is a bombing; the failure line rises to meet the cap.
    n_max = int(max_storeys) if max_storeys else int(
        3 if infill else MAX_FALL_STOREYS)
    s0 = max(s0, n_lv - max(1, n_max))
    s0 = max(0, min(s0, n_lv - 1))
    # ...but a one-storey loss on a tall block is a hole, not a collapse.
    if n_lv - s0 < 2 and s0 > 0:
        s0 -= 1

    base = (float(span_frac) if span_frac is not None
            else prng.uniform(*SPAN_FRAC))
    reach_m = prng.uniform(*CORNER_REACH_MODULES) * module_m
    centres = {}
    for sd in sides:
        L = fc.side_length(m, sd)
        centres[sd] = (None if mode == "corner"
                       else 0.5 * L + prng.uniform(-0.14, 0.14) * L)

    # AN RC INFILL LOSS TAKES PANELS, NOT THE FRAME. `corner` / `parapet` /
    # `balcony` pieces are the frame's own expression on the kit; taking them
    # away turns "the infill fell out of three bays" into "the corner of the
    # building is missing", which is a DG5 signature at DG3.
    roles = ("wall",) if infill else SHELL_ROLES
    plan["roles"] = roles

    cap = int(max_modules) if max_modules else int(MAX_MODULES)
    trimmed = 0
    while True:
        spans = _spans_elevation(m, sides, s0, top_i, n_lv, base, centres,
                                 corner=corner, reach_m=reach_m)
        kill = _kill_from_spans(ctx, m, mtag, spans, roles, plan["pad_m"])
        if len(kill) <= cap or s0 >= top_i or s0 + 1 > top_i - 1:
            break
        s0 += 1
        trimmed += 1
    plan["s0"] = int(s0)
    plan["span"] = spans
    plan["kill"] = kill
    plan["storeys"] = list(range(s0, n_lv))
    plan["span_frac"] = base
    plan["corner_reach_m"] = reach_m
    plan["_module_cap"] = cap
    plan["_trimmed"] = trimmed
    plan["sweep"] = True
    plan["roof"] = "ragged"
    plan["cut_z"] = float(m["levels"][s0]) - 0.4
    if infill:
        # dropped infill panels have no height to gather speed over
        plan["throw"] = (0.25, 0.9)

    # ---- the floors behind it ---------------------------------------------
    # The topmost slab in the band lost its bearing wall on this side and came
    # down onto the one below; the rest of the band keeps its slab and loses a
    # ragged strip along the open edge. Dropping every slab in the band is a
    # TOTAL collapse wearing this mode's name. And a corner that let go takes
    # the CORNER of each slab (`_ragged_slabs` breaks the open edge), never
    # the whole plate.
    n_drop = 0 if (mode == "corner" or infill) else max(0, int(drop_slabs))
    storeys = plan["storeys"]
    plan["drop"] = ([(mtag, s) for s in storeys[-n_drop:]]
                    if (n_drop and len(storeys) >= 2) else [])
    # EVERYTHING ON A STOREY WHOSE WHOLE PLATE WENT, region or no region: the
    # far half of that floor is standing on nothing the moment the plate
    # moves. Everything else only where its footprint reaches the loss.
    plan["fit_fall"] = list(plan["drop"])
    plan["fit_region_storeys"] = list(storeys)
    plan["teeth"] = True

    # ---- the fan in the street --------------------------------------------
    z_fail = (m["levels"][s0] if s0 < len(m["levels"]) else m["z0"])
    elem_h = max(1.0, float(m["top"]) - float(z_fail))
    for sd in sides:
        L = fc.side_length(m, sd)
        lo, hi = plan["span"][(sd, top_i)]
        if mode == "corner":
            # `r_corner_fail`'s own span: the last `reach + 2 m` of each of the
            # two sides that meet at the corner.
            w = min(0.5, (reach_m + CORNER_ALONG_PAD_M) / L)
            at_hi = fc.corner_at_high_end(sd, corner)
            along = (0.5 - w, 0.5) if at_hi else (-0.5, -0.5 + w)
            along_m = ((L - w * L, L) if at_hi else (0.0, w * L))
            depth = CORNER_DEPTH_M
            kind = "fan"
        elif infill:
            along = (lo / L - 0.5, hi / L - 0.5)
            along_m = (lo, hi)
            depth = prng.uniform(*INFILL_DEPTH_M)
            kind = "windrow"
        else:
            along = (lo / L - 0.5, hi / L - 0.5)
            along_m = (lo, hi)
            depth = prng.uniform(*FAN_DEPTH_M)
            kind = "fan"
        reach = max(1.5, 0.55 * elem_h)
        plan["heaps"].append({
            "kind": kind, "where": "outside", "sides": (sd,),
            "along": (float(along[0]), float(along[1])),
            "along_m": {sd: (float(along_m[0]), float(along_m[1]))},
            "depth_m": float(depth), "offset_m": 0.0, "crown_m": None,
            "spread_frac": prng.uniform(*FAN_SPREAD),
            "elem_h_m": float(elem_h), "stub_h_m": 0.0,
            "base_z": float(m["z0"]), "budget": None,
            "tag": "{0}_{1}".format(plan["mode"], sd),
            "centres": {sd: _heap_centres(m, sd, along_m, 0.45 * reach)},
        })


def _plan_band(ctx, plan, m, prng, mode, storey, lean_side, lean_deg, crush_m,
               twist_deg, offset_m):
    """`soft_storey` and `mid_storey`: ONE storey crushes on ALL FOUR sides
    and the block above drops onto it.

    THE BAND IS BITTEN, NOT SLICED. A kill list of exactly `storey == k`
    leaves a dead-level line top and bottom, both on a slab line — the
    "building lifted off with a pallet fork" artefact the round-4 review
    photographed (`r4_commercial/4_commercial_soft_storey_ne.png`: a clean
    rectangular gap under an otherwise pristine block). So the band takes a
    run of the storey ABOVE with it (the bottom of the block that came down)
    and, at a mid storey, a smaller run of the one BELOW; the ends of those
    runs are where `plan_edges` then puts left/right tears, and the seam under
    and over the band gets below/above tears everywhere else.

    THE MECHANISM IS `quake_flow._soft_storey_geometry` AND IS NOT
    RE-DERIVED HERE. It is two mechanisms drawn 60/40 (differential crush on a
    wedge, and a plumb sidesway rack), and the ORIGINAL code in this pipeline
    pivoted the wrong base edge and leaned buildings AWAY from the side it
    named — see "Soft-storey mechanics fix" in `build-earthquake-scenes`. The
    fix lives there; this plan carries its output verbatim in `crush["geo"]`
    and `r_collapse` hands that straight to `_soft_storey_matrix`.
    """
    fc, qf = _fc(), _qf()
    n_lv = plan["n_levels"]
    mtag = plan["mass"]
    lv = list(m["levels"])

    # A MID-STOREY CRUSH NEEDS A STOREY IN THE MIDDLE. `r_mid_storey` makes
    # the same call (`if n < 3: return r_soft_storey(storey=0)`) — on a one-
    # or two-level mass there is nothing between the ground and the roof, and
    # forcing k = 1 crushes the top storey, which is a roof collapse.
    if n_lv < 3 and storey is None:
        mode = "soft_storey"
        plan["mode"] = mode
    if storey is not None:
        k = int(storey)
    elif mode == "mid_storey":
        k = prng.randrange(1, max(2, n_lv - 1))
    else:
        k = 0
    k = max(0, min(k, n_lv - 1))
    if mode == "mid_storey" and k < 1:
        k = min(1, max(0, n_lv - 1))
    z_lo = float(lv[k])
    z_hi = float(lv[k + 1]) if k + 1 < len(lv) else float(m["top"])
    h_st = max(0.6, z_hi - z_lo)

    sides = ("S", "E", "N", "W")
    plan["sides"] = sides
    plan["keep_sides"] = ()
    plan["corner"] = None
    plan["band"] = [k]
    plan["s0"] = k
    plan["roles"] = SHELL_ROLES
    plan["roof"] = "ragged"
    plan["cut_z"] = z_lo
    plan["throw"] = None            # crushed, not thrown: see BAND_THROW
    plan["sweep"] = False

    spans = {}
    for sd in sides:
        L = fc.side_length(m, sd)
        spans[(sd, k)] = (0.0, L)
        # A ZERO-WIDTH BITE IS NOT A BITE. Recording `(t0, t0)` would put an
        # entry in `span` that kills nothing, reads as "the band is bitten"
        # to anything that walks the table, and hides the very artefact the
        # bite exists to prevent.
        if k + 1 < n_lv:
            w = prng.uniform(*BAND_BITE_UP) * L
            t0 = prng.uniform(0.0, max(0.0, L - w))
            if w > 1e-6:
                spans[(sd, k + 1)] = (float(t0), float(min(L, t0 + w)))
        if k >= 1:
            w = prng.uniform(*BAND_BITE_DOWN) * L
            t0 = prng.uniform(0.0, max(0.0, L - w))
            if w > 1e-6:
                spans[(sd, k - 1)] = (float(t0), float(min(L, t0 + w)))
    plan["span"] = spans
    plan["storeys"] = sorted(set(s for _sd, s in spans))
    cap = int(MAX_MODULES_TOTAL)
    plan["kill"] = _kill_from_spans(ctx, m, mtag, spans, SHELL_ROLES,
                                    plan["pad_m"])
    plan["_module_cap"] = cap
    plan["_trimmed"] = 0

    side_lean = lean_side or ("S", "E", "N", "W")[prng.randrange(4)]
    crush_frac = (float(crush_m) / h_st) if crush_m is not None else None
    if mode == "mid_storey":
        # `r_mid_storey`'s own numbers: a gentler tilt with a plan TWIST and
        # an offset, which is the one thing that tells a mid-storey crush from
        # a building that is merely shorter.
        lean = lean_deg if lean_deg is not None else prng.uniform(1.5, 4.0)
        twist = (twist_deg if twist_deg is not None
                 else prng.uniform(2.0, 8.0) * (1 if prng.random() < 0.5 else -1))
        off = offset_m if offset_m is not None else prng.uniform(0.3, 2.0)
    else:
        lean = lean_deg
        twist = twist_deg or 0.0
        off = offset_m or 0.0
    geo = qf._soft_storey_geometry(m, side_lean, h_st, z_lo, prng,
                                   lean_deg=lean, crush_frac=crush_frac)
    plan["crush"] = {
        "storey": k, "z_lo": z_lo, "z_hi": z_hi, "h_st": h_st,
        "lean_side": side_lean, "geo": geo,
        "crush_m": 0.5 * (geo["r_lean"] + geo["r_far"]),
        "twist_deg": float(twist), "offset_m": float(off),
    }
    # ONLY THE CRUSHED STOREY. `plan["storeys"]` also holds the bites at
    # k +- 1, and a partition at k + 1 is standing on the slab at k + 1 —
    # which is intact and rides down with the block. Dropping it would leave a
    # sheet of plasterboard in the gap the block just closed.
    plan["fit_fall"] = [(mtag, k)]
    plan["fit_region_storeys"] = []
    plan["drop"] = []
    plan["teeth"] = True

    # THE COLLAR: the crushed storey's material squeezed out round the
    # perimeter, 1-3 m wide (Northridge Meadows, Antakya). ONE `_rubble` call
    # with all four sides — `quake_rubble` builds one strip cell per side from
    # it, which is cheaper than four piles and is what `r_soft_storey` already
    # does. The per-side centres are carried so a test can check every one of
    # them is outside its own wall line at any yaw.
    depth = prng.uniform(*COLLAR_DEPTH_M) + plan["crush"]["crush_m"] * 0.25
    plan["heaps"].append({
        "kind": "windrow", "where": "outside", "sides": sides,
        "along": None, "along_m": dict(
            (sd, (0.0, fc.side_length(m, sd))) for sd in sides),
        "depth_m": float(depth), "offset_m": 0.0, "crown_m": None,
        "spread_frac": None, "elem_h_m": float(h_st), "stub_h_m": 0.0,
        "base_z": float(z_lo), "budget": None,
        "tag": "collar_{0}".format(k),
        "centres": dict(
            (sd, _heap_centres(m, sd, (0.0, fc.side_length(m, sd)),
                               0.45 * max(1.0, h_st)))
            for sd in sides),
    })


def _plan_stubs(ctx, plan, m, prng, blind, keep_stub, storey=0):
    """The storey-`storey` wall modules on a BLIND side, torn at a stub height
    instead of killed. Fills `stub_jobs` / `stub` / `blind_sides`.

    `r_masonry_collapse(keep_stub=True)` keeps a stub on EVERY side, and the
    result reads as a bathtub with rubble in it. The record is the opposite:
    where a stub survives a total collapse it is the PARTY WALL — the one the
    neighbour braced (research sec5a; the skill's `STUB_KEEP_P` row records
    that no code gated this before).
    """
    qf = _qf()
    bs = blind_sides(ctx, given=blind, prng=prng)
    plan["blind_sides"] = tuple(bs)
    if not keep_stub or not bs:
        return
    jobs, hs = [], {}
    # NO `prim_path` FILTER. Whether a placement has been written to the stage
    # yet is an AUTHORING fact, not a planning one — host-side (the tests,
    # `check()`, a dry run) no placement has a path at all, and filtering on it
    # here made the stub list silently empty in every offline check while
    # working in Kit. `_author_stubs` skips a pathless element, exactly as
    # `fire_collapse._tear_perimeter` does with its own jobs.
    for e in qf._els(ctx, mass=plan["mass"], role="wall", storey=int(storey)):
        if e["side"] not in bs:
            continue
        frac = prng.uniform(*STUB_FRAC)
        h = float(e.get("h") or 3.0)
        jobs.append({"el": e, "side": e["side"], "storey": int(storey),
                     "frac": float(frac), "h_m": float(h * frac),
                     "z_line": float(e["z"]) + h * frac})
        hs.setdefault(e["side"], []).append(h * frac)
    plan["stub_jobs"] = jobs
    plan["stub"] = dict((sd, sum(v) / len(v)) for sd, v in hs.items() if v)


def _plan_panels(ctx, plan, m, prng, storey=1, n=(1, 2)):
    """1-2 whole wall modules with openings, kept WHOLE for the pile to lay.

    `quake_flow._pick_opening_panels` does the same job for
    `r_masonry_collapse`, but it draws from `ctx["rng"]`, mutates the element
    table and needs a `prim_path` — three authoring concerns in a planning
    decision. This picks the ELEMENTS from the private generator and
    `_author_heaps` turns them into the `(prim_path, size)` pairs `_rubble`
    wants (`quake_rubble_usd._lay_existing` re-lays the prim itself onto the
    mound, so the module has to be kept out of BOTH the fracture loop and the
    tear list or it is destroyed before the pile gets it).

    There is no per-piece "has an opening" flag on this kit, so the pool is a
    name-pattern share with a fall-back to any wall module — the same call
    `_pick_opening_panels` makes the same way. Never on a blind side: that
    wall is where the stub is.
    """
    qf = _qf()
    if qf._RUBBLE_MODE != "v2":
        return []
    st = max(0, min(int(storey), plan["n_levels"] - 1))
    skip = set(id(j["el"]) for j in plan["stub_jobs"])
    pool = [e for e in qf._els(ctx, mass=plan["mass"], role="wall", storey=st)
            if id(e) not in skip
            and e["side"] not in plan.get("blind_sides", ())]
    named = [e for e in pool if any(k in (e.get("name") or "").lower()
                                    for k in OPENING_PAT)]
    pool = sorted(named or pool,
                  key=lambda q: (q["side"], round(q["lx"], 3),
                                 round(q["ly"], 3), q.get("name") or ""))
    if not pool:
        return []
    k = min(len(pool), 1 if len(pool) < 2 else prng.randint(*n))
    idx = list(range(len(pool)))
    prng.shuffle(idx)
    return [pool[i] for i in sorted(idx[:k])]


def _plan_total(ctx, plan, m, prng, blind, keep_stub, pile_base_z=None):
    """`total`: a URM building has come down into its own footprint.

    Everything the shell has goes — walls, corners, balconies, the parapet
    ring AND the roof, which is the one thing the fire's `SHELL_ROLES`
    deliberately excludes (a burnt-out shell keeps its skyline; a collapsed
    one has no skyline left). A stub survives on the blind sides only.
    """
    fc = _fc()
    n_lv = plan["n_levels"]
    mtag = plan["mass"]
    sides = ("S", "E", "N", "W")
    plan["sides"] = sides
    plan["keep_sides"] = ()
    plan["corner"] = None
    plan["s0"] = 0
    plan["roles"] = ALL_ROLES
    plan["roof"] = "kill"
    plan["sweep"] = False
    plan["throw"] = (TOTAL_THROW_BASE, TOTAL_THROW_TOP)
    plan["cut_z"] = float(m["z0"]) - 0.4

    _plan_stubs(ctx, plan, m, prng, blind, keep_stub, storey=0)
    plan["panels"] = _plan_panels(ctx, plan, m, prng,
                                  storey=min(1, n_lv - 1))
    skip = set(id(j["el"]) for j in plan["stub_jobs"])
    skip |= set(id(e) for e in plan["panels"])

    spans = {}
    for sd in sides:
        L = fc.side_length(m, sd)
        for s in range(0, n_lv):
            spans[(sd, s)] = (0.0, L)
    plan["span"] = spans
    plan["storeys"] = list(range(0, n_lv))
    plan["_module_cap"] = int(MAX_MODULES_TOTAL)
    plan["_trimmed"] = 0
    kill = _kill_from_spans(ctx, m, mtag, spans, SHELL_ROLES, plan["pad_m"],
                            skip=skip)
    # THE ROOF, UNCONDITIONALLY. A roof tile's `side` is whichever wall its
    # centre is nearest and its span test is meaningless, so it is swept by
    # ROLE and not by geometry.
    qf = _qf()
    kill += list(qf._els(ctx, mass=mtag, role=ROOF_ROLES))
    plan["kill"] = kill
    # A JOISTED FLOOR COMES APART INTO BOARDS, AND THEN IT IS BURIED. Dropping
    # a 22 x 18 m plate whole puts one sheet on the crown of the pile;
    # `r_masonry_collapse` learned that in round 2 ("the DG5 heaps read as a
    # LUMBER YARD") and breaks them instead, small and hard-thinned.
    plan["drop"] = []
    plan["break_slabs"] = [(mtag, s) for s in range(0, n_lv)]
    plan["fit_fall"] = [(mtag, s) for s in range(0, n_lv)]
    plan["fit_region_storeys"] = []
    plan["bury_props"] = True
    plan["teeth"] = False

    H = max(3.0, float(m["top"]) - float(m["z0"]))
    stub_h = (sum(plan["stub"].values()) / len(plan["stub"])
              if plan["stub"] else 0.0)
    fall = tuple(s for s in sides if s not in plan.get("blind_sides", ()))
    plan["heaps"].append({
        "kind": "dome", "where": "footprint",
        "sides": fall or ("S", "E"),
        "along": None, "along_m": {},
        "depth_m": None, "offset_m": 0.0,
        "crown_m": float(H * DOME_CROWN_FRAC),
        "spread_frac": prng.uniform(*DOME_SPREAD),
        "elem_h_m": None, "stub_h_m": float(stub_h),
        # ON THE GROUND, NOT AT THE WING'S OWN BASE. A wing's `m["z0"]` is
        # the MAIN mass's roof, so `r_masonry_collapse`'s per-mass
        # `_rubble(ctx, m, "dome", ...)` leaves a wing's pile floating four
        # metres up over the heap the main body just made (`block_stone`: a
        # 1-level base under two 8-storey wings). `_pile_mass` moves the base
        # and keeps `top - z0` — every derived height and reach — unchanged.
        "base_z": float(m["z0"] if pile_base_z is None else pile_base_z),
        "budget": None,
        "tag": "collapse_{0}".format(mtag), "centres": {},
        "panels": list(plan["panels"]),
    })


def _plan_pancake(ctx, plan, m, prng, from_storey, pitch_m, blind, keep_stub,
                  stack_base_z=None):
    """`pancake`: an RC frame comes down storey by storey and the SLABS stack.

    The shell is killed from `s0` up, progressively wider — the bottom two
    storeys keep a run at each end (the frame did not let go all at once), and
    everything above goes whole. The slabs are then RE-AUTHORED as a stack at
    a pitch rather than simulated: a stack of thin plates is exactly what
    PhysX does worst, and `r_pancake` learned that in round 1. The stacking
    numbers here ARE `r_pancake`'s.
    """
    fc = _fc()
    n_lv = plan["n_levels"]
    mtag = plan["mass"]
    sides = ("S", "E", "N", "W")
    s0 = max(0, min(int(from_storey or 0), n_lv - 1))
    plan["sides"] = sides
    plan["keep_sides"] = ()
    plan["corner"] = None
    plan["s0"] = s0
    plan["roles"] = SHELL_ROLES
    plan["roof"] = "stack"
    plan["sweep"] = False
    plan["throw"] = (0.30, 1.60)          # r_pancake's own façade shed
    plan["cut_z"] = float(m["levels"][s0]) - 0.4

    _plan_stubs(ctx, plan, m, prng, blind, keep_stub, storey=s0)
    plan["panels"] = _plan_panels(ctx, plan, m, prng,
                                  storey=min(s0 + 1, n_lv - 1))
    skip = set(id(j["el"]) for j in plan["stub_jobs"])
    skip |= set(id(e) for e in plan["panels"])

    spans = {}
    for sd in sides:
        L = fc.side_length(m, sd)
        for s in range(s0, n_lv):
            k = min(1.0, (s - s0) / float(max(1, PANCAKE_RAMP_STOREYS)))
            ramp = PANCAKE_FOOT + (1.0 - PANCAKE_FOOT) * k
            w = min(L, ramp * L)
            t0 = max(0.0, min(0.5 * L - w / 2.0, L - w))
            spans[(sd, s)] = (float(t0), float(min(L, t0 + w)))
    plan["span"] = spans
    plan["storeys"] = list(range(s0, n_lv))
    plan["_module_cap"] = int(MAX_MODULES_TOTAL)
    plan["_trimmed"] = 0
    plan["kill"] = _kill_from_spans(ctx, m, mtag, spans, SHELL_ROLES,
                                    plan["pad_m"], skip=skip)
    # THE SLABS ARE THE STACK. They are re-authored at a pitch below, so they
    # must not ALSO be handed to the solver: one prim in two places is the
    # double-transform class of bug the round-4 catalogue records twice.
    plan["drop"] = []
    plan["fit_fall"] = [(mtag, s) for s in range(s0, n_lv)]
    plan["fit_region_storeys"] = []
    plan["bury_props"] = True
    plan["teeth"] = False

    pitch = float(pitch_m) if pitch_m is not None else prng.uniform(*PANCAKE_PITCH)
    # A WING PANCAKES ONTO WHAT THE MAIN BODY LEFT, not onto the ground:
    # `r_pancake` puts a wing's stack base at the main stack's own top, which
    # is what a tower over a collapsed podium actually does. `r_collapse`
    # computes that height from the main plan and passes it in.
    base = float(m["z0"] if stack_base_z is None else stack_base_z)
    plates = []
    for i in range(1, n_lv + 1):
        top_plate = (i >= n_lv - 1)
        allside = ["S", "E", "N", "W"]
        prng.shuffle(allside)
        n_sides = ((3 + (1 if prng.random() < 0.5 else 0)) if top_plate
                   else (2 if prng.random() < 0.45 else 1))
        plates.append({
            "storey": i, "z": base + pitch * (i - 1 + 0.5),
            "tilt_deg": prng.uniform(*PANCAKE_TILT),
            "axis": ((1.0, 0.0, 0.0) if prng.random() < 0.5
                     else (0.0, 1.0, 0.0)),
            "jitter": (prng.uniform(-PANCAKE_JITTER, PANCAKE_JITTER),
                       prng.uniform(-PANCAKE_JITTER, PANCAKE_JITTER)),
            "sides": tuple(allside[:n_sides]), "top": bool(top_plate),
        })
    plan["stack"] = {"pitch_m": pitch, "base_z": base,
                     "top_z": base + pitch * n_lv + 0.15, "plates": plates,
                     "roof_tilt_deg": prng.uniform(*PANCAKE_ROOF_TILT)}

    fall = tuple(s for s in sides if s not in plan.get("blind_sides", ()))
    plan["heaps"].append({
        "kind": "dome", "where": "footprint",
        "sides": fall or ("S", "E"),
        "along": None, "along_m": {},
        "depth_m": None, "offset_m": 0.0,
        "crown_m": float(pitch * n_lv * PANCAKE_CROWN_FRAC),
        "spread_frac": prng.uniform(*PANCAKE_SPREAD),
        "elem_h_m": None, "stub_h_m": 0.0,
        "base_z": float(base), "budget": {"n_large": PANCAKE_LARGE_BUDGET},
        "tag": "pancake_{0}".format(mtag), "centres": {},
        "panels": list(plan["panels"]),
    })


# ---------------------------------------------------------------------------
# Reading a plan
# ---------------------------------------------------------------------------
def band_transform(plan):
    """The band modes' rigid displacement as pure numbers, for a host-side
    check: `(translate, pivot, axis, deg, twist_deg, offset_xy)`.

    `r_collapse` composes exactly this through `quake_flow.
    _soft_storey_matrix` + `_rot_about` + `_translate`; a test can apply it
    with numpy (the routing test's `_apply`) and assert both base edges of the
    block land on their own residual heights. Returns None outside the band
    modes.
    """
    c = plan.get("crush")
    if not c:
        return None
    geo = c["geo"]
    return {"translate": tuple(geo["translate"]), "pivot": geo.get("pivot"),
            "axis": tuple(geo["axis"]), "deg": float(geo.get("deg") or 0.0),
            "twist_deg": float(c.get("twist_deg") or 0.0),
            "offset_m": float(c.get("offset_m") or 0.0),
            "lean_side": c["lean_side"], "pivot_z": (c["z_lo"]
                                                     + c["crush_m"])}


def describe(plan):
    """One line, for `ctx["notes"]` and for a host-side dry run."""
    heaps = ", ".join("{0} {1} on {2}".format(
        h["kind"], h["where"],
        "/".join(h["sides"]) if h["sides"] else "the footprint")
        for h in plan["heaps"])
    extra = ""
    if plan["crush"]:
        c = plan["crush"]
        extra = (", storey {0} crushed to {1:.2f} m ({2}, lean {3} {4:.1f} "
                 "deg)".format(c["storey"], c["crush_m"], c["geo"]["mode"],
                               c["lean_side"], c["geo"]["lean_deg"]))
    elif plan["stack"]:
        extra = ", {0} slab(s) stacked at {1:.2f} m pitch".format(
            len(plan["stack"]["plates"]), plan["stack"]["pitch_m"])
    elif plan["stub"]:
        extra = ", stub kept on " + "/".join(
            "{0} {1:.2f} m".format(k, v) for k, v in sorted(plan["stub"].items()))
    return ("quake collapse ({0}{1}): {2} on {3} from storey {4} up, "
            "{5} module(s) taken, {6} surviving module(s) torn at the edge, "
            "{7} slab(s) dropped; {8}{9}".format(
                plan["mode"],
                " [infill]" if plan["infill"] else "",
                plan["btype"], "/".join(plan["sides"]), plan["s0"],
                len(plan["kill"]), len(plan["tears"]), len(plan["drop"]),
                heaps, extra))


# ---------------------------------------------------------------------------
# AUTHORING — everything below touches USD. Every `pxr` import is local.
# ---------------------------------------------------------------------------
def _teeth(ctx, plan, m, prng, storeys):
    """Bearing stubs still in the wall pocket where a floor framed into a wall
    that has gone, and the joist/purlin ends standing out of the break.

    STEEL ON A FRAME, TIMBER ON MASONRY. `urban_fire._joist_stubs` authors the
    same posts but binds `mats["steel"]` unconditionally and reads
    `ctx["fire"]` for its deck height — right for a burnt shell, wrong for a
    brick block that came down, and unusable here without dragging the fire
    ctx in. Same geometry, same reasoning (a POST, not a rod: a thin `_cyl`
    reads as a wire and PhysX cooks nothing for it), two materials.
    """
    qf = _qf()
    mtag = plan["mass"]
    btype = plan["btype"]
    mats = ctx["mats"]
    mat = mats.get("steel" if btype != "urm" else "timber") or mats["plaster"]
    fit = ctx.get("fit") or {}
    slabs = fit.get("slabs") or {}
    W = float(m["W"]) - 2 * qf.WALL_INSET
    D = float(m["D"]) - 2 * qf.WALL_INSET
    n_made = 0
    for s in storeys:
        if s < 1 or s >= len(m["levels"]):
            continue
        # A POST STANDS ON A FLOOR. Without a slab under it the post hangs in
        # clear air, which is the exact bug the fire path recorded on
        # `office_wide` F5c.
        if not slabs.get((mtag, s - 1)):
            continue
        z_slab = float(m["levels"][s]) - 0.18
        z_floor = float(m["levels"][s - 1])
        if z_slab - z_floor < 0.6:
            continue
        for _ in range(prng.randint(*TEETH_N)):
            L = prng.uniform(TEETH_LEN_M[0], min(TEETH_LEN_M[1],
                                                 z_slab - z_floor))
            if prng.random() < 0.5:
                lx = prng.uniform(-W / 2.0, W / 2.0)
                ly = (D / 2.0) * (1.0 if prng.random() < 0.5 else -1.0)
            else:
                ly = prng.uniform(-D / 2.0, D / 2.0)
                lx = (W / 2.0) * (1.0 if prng.random() < 0.5 else -1.0)
            wx, wy = qf._to_world(m, lx, ly)
            sz = prng.uniform(*TEETH_SIZE_M)
            path = "{0}/qcjoist_{1}_{2}".format(ctx["parent"], ctx["tag"],
                                                qf._uid(ctx))
            qf._box(ctx["stage"], path, wx, wy, z_floor + L / 2.0, sz, sz, L,
                    prng.uniform(-15.0, 15.0), mat)
            ctx["authored"].append(path)
            ctx["static_extra"].append(path)
            n_made += 1
    return n_made


def _author_kill(ctx, plan, m, prng, pnrng):
    """Fracture every module in the kill list and throw its fragments.

    `mode="uniform"` and NOT `quake_flow._p_frac_kw`: a bond/prism lattice
    makes cuboids, and cuboids are the "toy bricks" the round-5 review
    photographed. `inner_p` is the fire's, which `_break` itself lowers to
    0.18 whenever the piece has a texture and a core subset — so ~82 % of a
    fragment's faces keep the module's own cladding and only the invented cut
    faces take an interior material. Loose fragments then get `_dust_loose`:
    a dusted COPY of whatever each one is actually bound to, so a fallen piece
    is the same material as the wall it came out of, darker.
    """
    qf = _qf()
    stage = ctx["stage"]
    info = ctx["info"]
    mode = plan["mode"]
    Hm = max(3.0, float(m["top"]) - float(m["z0"]))
    n_broke = n_frag = 0
    band_k = plan["band"][0] if plan["band"] else None

    if mode == "total":
        pieces, consume, cap = TOTAL_PIECES, TOTAL_CONSUME, TOTAL_MAX_PIECE_M
    elif mode == "pancake":
        pieces, consume, cap = (PANCAKE_PIECES, PANCAKE_CONSUME,
                                PANCAKE_MAX_PIECE_M)
    elif band_k is not None:
        pieces, consume, cap = BAND_PIECES, BAND_CONSUME, None
    else:
        pieces, consume, cap = BREAK_PIECES, BREAK_CONSUME, None
    consume = _f("QC_CONSUME", consume)

    for e in plan["kill"]:
        if e.get("dead"):
            continue
        if e["role"] in ROOF_ROLES:
            # A MASONRY ROOF IS A TIMBER DECK and comes apart into boards, not
            # into concrete plates. `_break_box_like` swaps the kit tile for a
            # solid slab first (a kit roof tile is a zero-thickness quad) and
            # is the quake pipeline's own helper for it.
            made = qf._break_box_like(ctx, e, 52,
                                      timber=(plan["btype"] == "urm"),
                                      consume=0.85, dusty=True,
                                      max_piece_m=1.0)
            qf._a_lay_flat(ctx, made)
            ctx["loose"] += made
            e["dead"] = True
            n_broke += 1
            n_frag += len(made)
            continue
        st, lo = qf._break(stage, ctx["parent"], e, ctx["tag"],
                           prng.randint(*pieces), prng, pnrng, ctx["mats"],
                           ctx["cache"], info["type"], inner_p=INNER_P,
                           mode="uniform", rough=BREAK_ROUGH,
                           consume=consume, max_piece_m=cap,
                           style=info.get("style"))
        ox, oy = qf._outward(m, e["side"])
        if band_k is not None:
            # THE BAND IS CRUSHED, NOT THROWN. It is squeezed out sideways at
            # walking pace under the mass above it, and the fragments are
            # squashed down to the storey's residual height so the collar sits
            # in the gap rather than floating in it.
            geo = plan["crush"]["geo"]
            z_lo = plan["crush"]["z_lo"]
            h_st = plan["crush"]["h_st"]
            residual = qf._soft_storey_residual(
                geo, m, plan["crush"]["lean_side"], e["side"], e["lx"], e["ly"])
            qf._squash(stage, lo, z_lo, residual / h_st)
            push = prng.uniform(*BAND_PUSH_M)
            qf._transform_prims(stage, lo, qf._translate(ox * push,
                                                         oy * push, 0.0))
            for pth in lo:
                ctx["velocity"][pth] = (ox * prng.uniform(*BAND_THROW),
                                        oy * prng.uniform(*BAND_THROW), 0.0)
        else:
            base_v, top_v = plan["throw"]
            for pth in lo:
                zf = min(1.0, max(0.0, (float(e["z"]) + float(e["h"]) * 0.5
                                        - float(m["z0"])) / Hm))
                v = base_v + top_v * zf + prng.uniform(-0.2, 0.3)
                if mode == "total":
                    # nearly straight down; the outward fan belongs to a peel
                    ctx["velocity"][pth] = (ox * v * prng.uniform(0.3, 1.0),
                                            oy * v * prng.uniform(0.3, 1.0),
                                            -0.4 * prng.random())
                else:
                    ctx["velocity"][pth] = (ox * v, oy * v, 0.15 * v)
        if mode in ("total", "pancake"):
            qf._a_lay_flat(ctx, lo)
        qf._dust_loose(ctx, lo)
        ctx["loose"] += lo
        ctx["static_extra"] += st
        e["dead"] = True
        n_broke += 1
        n_frag += len(st) + len(lo)
    return n_broke, n_frag


def _author_stubs(ctx, plan, m, prng):
    """Tear the storey-0 wall modules on the blind sides at their stub height.

    `_break_split` with `quake_flow._p_zline_judge`: the break line STAIRCASES
    along the courses (urm) or tears (rc), which is the one edge a total
    collapse's heap leans against and therefore the most-looked-at line in the
    whole recipe (`r_masonry_collapse`'s own note). The surviving stub keeps
    the module's own cladding (`static_mat`); only what comes away is dusted.
    """
    from . import damage
    qf = _qf()
    stage = ctx["stage"]
    btype = plan["btype"]
    n = 0
    for j in plan["stub_jobs"]:
        e = j["el"]
        if e.get("dead"):
            continue
        path = (e.get("p") or {}).get("prim_path")
        if not path:
            continue
        tex = damage.bound_texture(stage, path)
        st, lo = qf._break_split(
            ctx, path, 10 + prng.randrange(4),
            qf._p_zline_judge(m, e["side"], j["z_line"], prng, btype=btype,
                              amp=float(e.get("h") or 3.0) * 0.30,
                              loose_above=True, pitch=qf._p_pitch(ctx)),
            qf._mat_fn(ctx, tex, 0.35),
            static_mat=(qf._clad_material(stage, ctx["parent"], ctx["cache"],
                                          tex) if tex else None))
        ox, oy = qf._outward(m, e["side"])
        for pth in lo:
            v = prng.uniform(0.2, 0.8)
            ctx["velocity"][pth] = (ox * v, oy * v, -0.2 * v)
        qf._dust_loose(ctx, lo)
        ctx["loose"] += lo
        ctx["static_extra"] += st
        e["dead"] = True
        n += 1
    return n


def _author_heaps(ctx, plan, m, prng):
    """Every pile, through `quake_flow._rubble` and nothing else.

    `_rubble` is `EQ_RUBBLE`-gated (v2 = the round-4 mound / large-element /
    instancer planner; v1 = the old `_heap` box crate) and is the ONE call
    site a routed recipe may use. Nothing here calls `_heap` — the round-4
    routing test asserts exactly that of every recipe it owns, and this family
    is written to pass the same sweep.
    """
    qf = _qf()
    n = 0
    for h in plan["heaps"]:
        mm = qf._pile_mass(m, h["base_z"])
        # THE WHOLE MODULES THE PILE LAYS. `quake_rubble_usd._lay_existing`
        # re-lays the prim itself (it does not copy it), so an element with no
        # `prim_path` — a host-side dry run — simply contributes nothing, and
        # one that IS laid is marked dead so `wreck_building`'s closing sweep
        # does not also file it as a standing wall.
        pan = []
        for e in (h.get("panels") or ()):
            path = (e.get("p") or {}).get("prim_path")
            if not path:
                continue
            pan.append((path, qf._module_size(m, e)))
            e["dead"] = True
        ret = qf._rubble(ctx, mm, h["kind"], sides=h["sides"],
                         along=h["along"], depth_m=h["depth_m"],
                         offset_m=h["offset_m"], stub_h_m=h["stub_h_m"],
                         elem_h_m=h["elem_h_m"], crown_m=h["crown_m"],
                         spread_frac=h["spread_frac"], budget=h["budget"],
                         panels=pan, tag=h["tag"])
        made = ret.get("all") or []
        n += len(made)
        if plan["crush"] and plan["crush"]["storey"] > 0:
            # a collar authored in the air beside a MID-storey band must fall
            # rather than hover (`r_soft_storey`'s own correction)
            keep = set(made)
            ctx["static_extra"] = [q for q in ctx["static_extra"]
                                   if q not in keep]
            ctx["loose"] += made
    return n


def _fall_fitout(ctx, plan, m, prng):
    """Send down the fit-out that has lost its floor or its bearing wall.

    Three populations, and each one was a review finding once:
      * the SLABS in `plan["drop"]` (and their props, which `fit_interior`
        seats directly on them — a prop left standing on a plate that is about
        to move is `r_fire_collapse`'s own lesson);
      * everything on a storey in `plan["fit_fall"]`, region or no region: a
        storey whose whole plate went, or whose columns crushed, has nothing
        under its partitions;
      * on a storey in `plan["fit_region_storeys"]`, anything whose FOOTPRINT
        reaches into `plan["region"]` — "part_main_6_1 ... look like they're
        floating" (user, fire review). The footprint, not the centre: a
        partition is a 0.12 m sheet running the short way across the plan, so
        one framing into the lost wall has its END in the region and its
        centre several metres inside.

    Props are skipped entirely when `plan["bury_props"]` is set: a total
    collapse and a pancake put the contents of the building UNDER the pile
    (`quake_flow._a_bury_props`, called by the authoring step that owns the
    pile), and a prop that is both buried and loose is one prim in two places.
    """
    from pxr import Usd, UsdGeom
    qf = _qf()
    stage = ctx["stage"]
    fc = _fc()
    mtag = plan["mass"]
    fit = ctx.get("fit") or {}
    n = {"slab": 0, "part": 0, "col": 0, "prop": 0}
    Hm = max(3.0, float(m["top"]) - float(m["z0"]))

    for key in plan["drop"]:
        slab = (fit.get("slabs") or {}).get(key)
        if not slab:
            continue
        pr = stage.GetPrimAtPath(slab)
        if pr and pr.IsValid() and pr.IsActive() and slab not in ctx["loose"]:
            ctx["loose"].append(slab)
            n["slab"] += 1

    fall_storeys = set(int(s) for (mt, s) in plan["fit_fall"] if mt == mtag)
    region_storeys = set(int(s) for s in plan["fit_region_storeys"])
    bury = bool(plan.get("bury_props"))
    cands = []
    for p in (fit.get("partitions") or ()):
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
    for (mt_, s_), cols in (fit.get("columns") or {}).items():
        for p in (cols or ()):
            cands.append((p, mt_, int(s_), "col"))
    if not bury:
        for (mt_, s_), props in (fit.get("props") or {}).items():
            for p in (props or ()):
                cands.append((p, mt_, int(s_), "prop"))

    xf = UsdGeom.XformCache()
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    for p, mt_, s_, kind in cands:
        if mt_ != mtag or not p or p in ctx["loose"]:
            continue
        if s_ not in fall_storeys and s_ not in region_storeys:
            continue
        pr = stage.GetPrimAtPath(p)
        if not pr or not pr.IsValid() or not pr.IsActive():
            continue
        try:
            t = xf.GetLocalToWorldTransform(pr).ExtractTranslation()
        except Exception:
            continue
        sd = None
        if plan["region"] and s_ in region_storeys:
            pts = [(float(t[0]), float(t[1]))]
            try:
                bx = bc.ComputeWorldBound(pr).ComputeAlignedRange()
                if not bx.IsEmpty():
                    lo_b, hi_b = bx.GetMin(), bx.GetMax()
                    pts += [(float(lo_b[0]), float(lo_b[1])),
                            (float(hi_b[0]), float(lo_b[1])),
                            (float(hi_b[0]), float(hi_b[1])),
                            (float(lo_b[0]), float(hi_b[1]))]
            except Exception:
                pass
            for wx_, wy_ in pts:
                lx, ly = qf._to_local(m, wx_, wy_)
                sd = fc.region_side(plan, m, lx, ly)
                if sd is not None:
                    break
        if sd is None and s_ not in fall_storeys:
            continue
        ctx["loose"].append(p)
        ctx["static_extra"] = [q for q in ctx["static_extra"] if q != p]
        if sd is not None and plan["throw"]:
            ox, oy = qf._outward(m, sd)
            zf = min(1.0, max(0.0, (float(t[2]) - float(m["z0"])) / Hm))
            v = plan["throw"][0] + plan["throw"][1] * zf
            ctx["velocity"][p] = (ox * v, oy * v, -0.15 * v)
        n[kind] += 1
    return n


def _author_floors(ctx, plan, m, prng):
    """Break the floor slabs a TOTAL collapse brings down, and bury what was
    standing on them.

    `r_masonry_collapse`'s own treatment, and its own numbers: a joisted
    timber floor comes apart into 54-70 plank cells with `consume` 0.88 at
    `consume_pool` 1.05 (so the thinning takes the BIGGEST pieces), bound to
    the dusty joist tint with 45 % of them in mortar dust instead. Round 2:
    "the DG5 heaps read as a LUMBER YARD" — 30-40 cells out of a 22 x 18 m
    deck are 3.5 x 1.2 m boards, five storeys of them cover the brick
    completely, and bound to the pale `timber` map they are the brightest
    thing in the shot.

    NOT `_dust_loose`. Every material this authors is already an `A_DEBRIS`
    flat colour, calibrated in LINEAR albedo; `diffuse_tint` MULTIPLIES, so
    tinting one a second time is the near-black bug the round-4 catalogue
    records for `_material_for_look` and round 2 records for
    `planks.wood_material`. `_dust_loose`'s own docstring makes the same
    exclusion for the same reason.
    """
    qf = _qf()
    stage = ctx["stage"]
    fit = ctx.get("fit") or {}
    mtag = plan["mass"]
    timber = (plan["btype"] == "urm")
    n = 0
    for key in plan["break_slabs"]:
        pth = (fit.get("slabs") or {}).get(key)
        if not pth:
            continue
        pr = stage.GetPrimAtPath(pth)
        if not pr or not pr.IsValid() or not pr.IsActive():
            continue
        made = qf._break_box(
            stage, pth, 54 + prng.randrange(16), prng, ctx["nrng"],
            qf._a_mat(ctx, "timber_dusty" if timber else "concrete_dusty"),
            qf._a_mat(ctx, "brick_dusty"), 0.45,
            mode=("plank" if timber else "prism"),
            aspect=((1.8, 3.6) if timber else None), consume=0.88,
            consume_pool=1.05, max_piece_m=1.0)
        qf._a_lay_flat(ctx, made, p=0.9)
        ctx["loose"] += made
        fit["all"] = [q for q in fit["all"] if q != pth]
        n += len(made)
    if plan.get("bury_props"):
        # UNDER the pile, not on it. A pancake's contents are under the slab
        # STACK, so the burial depth is the stack's own; a total collapse's
        # are under a heap of H/3 (FEMA's air-space factor).
        st = plan.get("stack")
        if st:
            base, depth, keep = (float(st["base_z"]),
                                 st["pitch_m"] * plan["n_levels"] * 0.6, 0.25)
        else:
            H = max(3.0, float(m["top"]) - float(m["z0"]))
            base, depth, keep = float(m["z0"]), H * DOME_CROWN_FRAC, 0.3
        for (pm, _i), props in (fit.get("props") or {}).items():
            if pm == mtag:
                qf._a_bury_props(ctx, props, base, depth, keep=keep)
    return n


def _author_band(ctx, plan, m, prng, snap_before_tears):
    """Move the block above a crushed storey.

    `quake_flow._soft_storey_matrix(plan["crush"]["geo"])` and NOTHING
    RE-DERIVED. The mid-storey signature (a plan twist and an offset) is
    composed on top exactly as `r_soft_storey` composes it.

    THE TORN STATICS OF THE STOREY ABOVE HAVE TO RIDE DOWN WITH IT. They are
    not `_els` any more (`_tear_perimeter` marks the module dead), so the only
    way to find them is a diff of what this recipe has authored: everything
    new in `static_extra` whose world z sits at or above the band's ceiling
    belongs to the block. Leave them behind and the seam grows a row of
    fragments hanging where the storey used to be.
    """
    from pxr import UsdGeom
    fc, qf = _fc(), _qf()
    stage = ctx["stage"]
    info = ctx["info"]
    fit = ctx.get("fit") or {}
    c = plan["crush"]
    mtag = plan["mass"]
    k, z_hi, z_lo = c["storey"], c["z_hi"], c["z_lo"]
    ox, oy = qf._outward(m, c["lean_side"])

    above = []
    for e in qf._els(ctx, mass=None):
        em = info["masses"].get(e["mass"]) or m
        if e["mass"] == mtag or e["mass"].startswith(mtag + "_") or (
                mtag == "main" and e["mass"] != "main"):
            if float(e["z"]) >= z_hi - 0.05 or (
                    e["role"] in ("roof",) and float(em["top"]) >= z_hi):
                above.append((e.get("p") or {}).get("prim_path"))
    for (mt, i), pth in (fit.get("slabs") or {}).items():
        if i > k or (mt != mtag and mt.startswith(mtag)) or (
                mtag == "main" and mt != "main"):
            above.append(pth)
    for (mt, i), cols in (fit.get("columns") or {}).items():
        if i > k or (mtag == "main" and mt != "main"):
            above.extend(cols)
    for pth in (fit.get("partitions") or ()):
        try:
            i = int(str(pth).rsplit("_", 2)[-2])
        except ValueError:
            i = 0
        if i > k:
            above.append(pth)
    for (mt, i), props in (fit.get("props") or {}).items():
        if i > k or (mtag == "main" and mt != "main"):
            above.extend(props)

    # the new statics the tears left, split by height
    _new_lo, new_st = fc._new_since(ctx, snap_before_tears)
    xf = UsdGeom.XformCache()
    for pth in new_st:
        pr = stage.GetPrimAtPath(pth) if pth else None
        if not pr or not pr.IsValid():
            continue
        try:
            t = xf.GetLocalToWorldTransform(pr).ExtractTranslation()
        except Exception:
            continue
        if float(t[2]) >= z_hi - 0.05:
            above.append(pth)

    seen, out = set(), []
    for a in above:
        if not a or a in seen or a in ctx["loose"]:
            continue
        seen.add(a)
        out.append(a)
    M = qf._soft_storey_matrix(c["geo"])
    if c["twist_deg"] or c["offset_m"]:
        M = M * qf._rot_about((m["cx"], m["cy"], z_lo + c["crush_m"]),
                              (0.0, 0.0, 1.0), float(c["twist_deg"])) \
            * qf._translate(ox * float(c["offset_m"]),
                            oy * float(c["offset_m"]), 0.0)
    qf._transform_prims(stage, out, M)
    ctx["static_extra"] += out
    return len(out)


def _author_stack(ctx, plan, m, prng):
    """Re-author the slabs of a pancaked frame as a STACK.

    `r_pancake`'s own arithmetic, kept: each plate loses one to four sides
    along a wandering line (a pancaked slab does not keep its formwork edge —
    the first pass left a deck of cards seen from the air), then translates to
    its pitch height with a small tilt and jitter, then takes the dark roof
    map (the fit-out binds slabs with a pale pavement map, which reads from
    the air as a sheet of bathroom tile on the crown of the pile).
    """
    from pxr import UsdShade
    qf = _qf()
    stage = ctx["stage"]
    info = ctx["info"]
    fit = ctx["fit"]
    mtag = plan["mass"]
    st_plan = plan["stack"]
    n_plates = 0
    stack = []
    for pl in st_plan["plates"]:
        key = (mtag, pl["storey"])
        pth = (fit.get("slabs") or {}).get(key)
        if pth is None:
            continue
        cur, sts = pth, []
        for sd in pl["sides"]:
            km = UsdShade.MaterialBindingAPI(
                stage.GetPrimAtPath(cur)).ComputeBoundMaterial()[0]
            d = prng.uniform(0.5, 1.6)
            rem, strip = qf._split_strip(ctx, cur, m, sd, d + 1.5,
                                         ctx["mats"]["concrete"])
            st, lo = qf._break_split(
                ctx, strip, 10 + prng.randrange(4),
                qf._p_edge_judge(m, sd, d, prng, btype=info["type"]),
                qf._mat_fn(ctx, None, 1.0),
                min_volume_frac=0.0008, static_mat=km if km else None,
                refine_max=6, max_loose_m=2.6, rough=qf.ROUGH_STRIP_M)
            cur, sts = rem, sts + st
            qf._dust_loose(ctx, lo)
            ctx["loose"] += lo
            ctx["authored"].append(rem)
        group = [cur] + sts
        fit["slabs"][key] = cur
        z = float(pl["z"])
        i = int(pl["storey"])
        n_lv = plan["n_levels"]
        self_z = (m["levels"][i] - qf.SLAB_T[info["type"]] / 2.0 if i < n_lv
                  else float(m["top"]) - qf.SLAB_T[info["type"]] / 2.0)
        jx, jy = pl["jitter"]
        M = qf._translate(jx, jy, z - self_z) * qf._rot_about(
            (m["cx"], m["cy"], z), pl["axis"], pl["tilt_deg"])
        qf._transform_prims(stage, group, M)
        for q in group:
            qf._bind(stage, q, qf._a_roof_mat(ctx))
        stack += group
        n_plates += 1
        for _ in range(2):
            qf._rebar_tuft(ctx, group[0], z + 0.1, n=2, length=(0.4, 1.0))
    ctx["static_extra"] += stack

    # the roof lands on top, tilted
    top_z = float(st_plan["top_z"])
    for e in list(qf._els(ctx, mass=mtag, role="roof")):
        pth = qf._roof_box(ctx, e)
        if not pth:
            continue
        M = qf._translate(prng.uniform(-0.8, 0.8), prng.uniform(-0.8, 0.8),
                          top_z - float(e["z"])) * qf._rot_about(
            (m["cx"], m["cy"], top_z),
            ((1.0, 0.0, 0.0) if prng.random() < 0.5 else (0.0, 1.0, 0.0)),
            float(st_plan["roof_tilt_deg"]))
        qf._transform_prims(stage, [pth], M)
        ctx["static_extra"].append(pth)
        e["dead"] = True
    # (the props were buried by `_author_floors`, which owns `bury_props`
    # for both total and pancake — burying them twice moves them twice)
    return n_plates


def _sweep_statics(ctx, plan, m, pre_static):
    """The position sweep, RESTRICTED IN PLAN (elevation / corner only).

    "Everything static above the failure line" is right when the whole top of
    a block is coming down and catastrophic here: it would drop the far half
    of the roof deck, its plant and the walls on three untouched elevations.
    The reason for a POSITION test at all still holds (the roof is `_roof_box`
    slabs, then `_split_strip` remainders, then whatever a recipe authored,
    each in a different list or in none), so the test gains a second
    predicate: above the failure line AND inside `plan["region"]` — and it
    only ever looks at statics that existed BEFORE this recipe ran, because
    everything the recipe itself adds is the part that STAYS UP.
    """
    from pxr import UsdGeom
    fc, qf = _fc(), _qf()
    stage = ctx["stage"]
    xf = UsdGeom.XformCache()
    keep, moved = [], []
    for pth in list(ctx["static_extra"]):
        if pth not in pre_static:
            keep.append(pth)
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
        if float(t[2]) > plan["cut_z"] and fc.in_region(plan, m, lx, ly):
            moved.append(pth)
        else:
            keep.append(pth)
    ctx["static_extra"] = keep
    n = 0
    for pth in moved:
        if pth not in ctx["loose"]:
            ctx["loose"].append(pth)
            n += 1
    return n


def r_collapse(ctx, mode="elevation", **kw):
    """THE RECIPE. Author `plan_collapse(ctx, mode, **kw)`.

    Order matters and every step is here for a reason already paid for once:

      1. the SHELL comes away first — `quake_flow._els` skips elements a
         recipe has marked dead, so a wall taken away after art has been
         authored on it leaves the art standing in the sky;
      2. the STUBS (total / pancake) are torn at their own height;
      3. the PERIMETER of the hole is torn, every surviving module that
         touches a dead one, on the edge the hole is on;
      4. the floors: the slab that lost its bearing wall drops whole, the rest
         lose a ragged strip and droop into the opening;
      5. the FIT-OUT that has lost its floor or its wall goes down;
      6. the band's block above moves / the pancake's slabs stack;
      7. the position sweep (elevation / corner only);
      8. the PILES, through `_rubble`;
      9. the teeth at the break.

    Everything draws from the PRIVATE generators, installed on the ctx for the
    duration (`fire_collapse._own_rng`) because `_rubble`, `_a_slab_rim`,
    `_ragged_slabs`, `_disturb_interior`, `r_droop` and `_dust_loose` all read
    `ctx["rng"]` off the ctx. Zero shared draws.

    Returns the FIRST mass's plan, with `plan["masses"]` naming every mass
    this call brought down and `plan["extra"]` holding the others' plans;
    `ctx["quake_collapse"]` is the whole list.
    """
    # EVERY MASS, for a total collapse and a pancake. See `collapse_masses`:
    # `urban_building` builds a block as a main mass plus wings, and on the
    # tall stock the wings ARE the building (`block_residential`: 181 of its
    # 2,377 elements are on `main`). A partial or band mode is single-mass by
    # definition and this loop runs once.
    tags = collapse_masses(ctx["info"], mode, kw.get("mass"))
    plans = []
    stack_top = None
    first = None
    for i, mt in enumerate(tags):
        kw2 = dict(kw)
        kw2["mass"] = mt
        if i and stack_top is not None:
            # a wing pancakes ONTO what the main body left (`r_pancake`)
            kw2.setdefault("stack_base_z", stack_top)
        if i and first is not None and first["mode"] == "total":
            # ...and every mass's heap sits on the GROUND, not on the wing's
            # own base, which is the main mass's roof
            kw2.setdefault("pile_base_z", first["heaps"][0]["base_z"])
        p_i = _author_one(ctx, mode, kw2)
        plans.append(p_i)
        if first is None:
            first = p_i
        if p_i["stack"] and stack_top is None:
            stack_top = (float(p_i["stack"]["base_z"])
                         + p_i["stack"]["pitch_m"] * p_i["n_levels"] + 0.4)
    plan = plans[0]
    plan["masses"] = list(tags)
    plan["extra"] = plans[1:]
    ctx["quake_collapse"] = plans
    return plan


def _author_one(ctx, mode, kw):
    """`r_collapse` for ONE mass — the whole authoring order, in one place so
    the multi-mass sweep above is a loop and not a second copy of it."""
    import numpy as np
    fc, qf = _fc(), _qf()

    plan = plan_collapse(ctx, mode=mode, **kw)
    mtag = plan["mass"]
    m = ctx["info"]["masses"].get(mtag) or ctx["info"]["masses"]["main"]
    prng = random.Random(plan["seed"])
    pnrng = np.random.default_rng(plan["seed"] & 0xFFFFFFFF)
    pre_static = set(q for q in (ctx.get("static_extra") or ()) if q)

    with fc._own_rng(ctx, prng, pnrng):
        # ---- 1/2. the shell and the stubs -------------------------------
        n_broke, n_frag = _author_kill(ctx, plan, m, prng, pnrng)
        n_stub = _author_stubs(ctx, plan, m, prng)

        # ---- 3. the whole perimeter of the hole -------------------------
        snap = fc._snapshot(ctx)
        n_edge = fc._tear_perimeter(ctx, plan, m, prng, plan["tears"])

        # ---- 4. the floors ----------------------------------------------
        n_slabrim = 0
        if plan["crush"]:
            # BOTH slabs the gap exposes: the one under the crushed storey
            # (it stays put, and takes rebar tufts) and the one the block
            # above sits on (it is about to be transformed, so no tufts).
            k = plan["crush"]["storey"]
            n_slabrim += len(qf._a_slab_rim(ctx, mtag, k, n_sides=2))
            n_slabrim += len(qf._a_slab_rim(ctx, mtag, k + 1, n_sides=2,
                                            bars=False))
        elif plan["mode"] in ("elevation", "corner") and not plan["infill"]:
            for sd in plan["sides"]:
                qf._ragged_slabs(ctx, mtag, sd, set(plan["storeys"]))
                lo_t, hi_t = plan["span"][(sd, plan["top_storey"])]
                # THE FLOORS AT THE OPENING HINGE DOWN — the USAR lean-to —
                # BUT ONLY WHEN A REAL LENGTH OF WALL HAS GONE. `_droop_strip`
                # splits a strip 25-50 % of the mass DEEP along the WHOLE of
                # that side; behind a wall that is still standing that is a
                # floor tilting into a room for no visible reason.
                if (hi_t - lo_t) >= 0.60 * fc.side_length(m, sd):
                    qf.r_droop(ctx, mass=mtag, side=sd,
                               storeys=set(plan["storeys"]), p=0.35)
            qf._disturb_interior(ctx, mtag, set(plan["storeys"]),
                                 side=plan["sides"][0])

        # ---- 5. the fit-out ---------------------------------------------
        n_fit = _fall_fitout(ctx, plan, m, prng)
        n_board = _author_floors(ctx, plan, m, prng) \
            if (plan["break_slabs"] or plan["bury_props"]) else 0

        # ---- 6. the block above / the slab stack ------------------------
        n_above = n_plates = 0
        if plan["crush"]:
            n_above = _author_band(ctx, plan, m, prng, snap)
        elif plan["stack"]:
            n_plates = _author_stack(ctx, plan, m, prng)

        # ---- 7. the position sweep --------------------------------------
        n_swept = _sweep_statics(ctx, plan, m, pre_static) if plan["sweep"] \
            else 0

        # ---- 8. the piles ------------------------------------------------
        n_heap = _author_heaps(ctx, plan, m, prng)

        # ---- 9. the teeth at the break -----------------------------------
        # ...and NOT where the floor they would stand on is itself coming
        # down: a total collapse and a pancake leave nothing for a bearing
        # stub to stand in, and a static post over a slab that has just gone
        # to the solver is a stick hanging in the sky (uf_r2j, 2026-08-28).
        n_teeth = _teeth(ctx, plan, m, prng, plan["storeys"]) \
            if plan["teeth"] else 0

        # THE DUST IS APPLIED AT EACH FRACTURE SITE, NOT IN ONE SWEEP HERE,
        # and that is deliberate. `_dust_loose` rebinds a dusted COPY of
        # whatever a prim is CURRENTLY bound to, so calling it twice on one
        # fragment tints an already-tinted copy — `diffuse_tint` multiplies,
        # and a second pass is the near-black bug the round-4 catalogue
        # records for `_material_for_look` and round 2 records for
        # `planks.wood_material`. It is also why the pieces `_ragged_slabs`,
        # `_a_slab_rim`, `r_droop`, `_disturb_interior` and `_author_floors`
        # author are left alone: every one of those is already an `A_DEBRIS`
        # flat colour calibrated in linear albedo. What IS dusted is exactly
        # what `_chunk_material` / `_mat_fn` can hand a PRISTINE palette entry
        # or a cladding photo — the shell fragments and the stub shards — and
        # that is the answer to "damaged part and undamaged part look like
        # completely diff materials". Nothing here calls `_a_dustify` (a
        # palette SWAP with no entry for `brick` or for a cladding photo at
        # all) or `HEAP_MIX`.

    cen = fc.edge_census(plan["tears"])
    plan["edge_census"] = cen
    ctx["notes"].append(describe(plan))
    ctx["notes"].append(
        "quake collapse: {0} module(s) broken into {1} fragment(s), {2} stub "
        "module(s) torn at their own height, {3} surviving module(s) torn at "
        "the edge of the hole, {4} slab rim piece(s), {5} static prim(s) "
        "swept into the loss, {6} prim(s) carried with the block above, {7} "
        "slab(s) stacked, {8} floor board(s), {9} rubble prim(s), {10} joist "
        "stub(s)".format(
            n_broke, n_frag, n_stub, n_edge, n_slabrim, n_swept, n_above,
            n_plates, n_board, n_heap, n_teeth))
    ctx["notes"].append(
        "quake collapse fit-out: {0} slab(s), {1} partition(s), {2} "
        "column(s), {3} prop(s) sent down".format(
            n_fit["slab"], n_fit["part"], n_fit["col"], n_fit["prop"]))
    ctx["notes"].append(
        "quake collapse edges: " + ", ".join(
            "{0} {1}/{2}".format(c, cen[c][1], cen[c][0])
            for c in fc.EDGE_CLASSES) +
        "; budget {0}/{1} module(s), {2}/{3} edge(s){4}".format(
            plan["budget"]["modules"], plan["budget"]["module_cap"],
            plan["budget"]["edges"], plan["budget"]["edge_cap"],
            ", OVER BUDGET" if (plan["budget"]["over_modules"]
                                or plan["budget"]["over_edges"]) else ""))
    return plan


# ---------------------------------------------------------------------------
# Registration
# ---------------------------------------------------------------------------
def _recipe(mode):
    """One `RECIPES` entry per mode. `mode` may still be overridden by a
    ladder entry or an `EQ_RECIPES` bench line, which is how `qc_auto` is
    expressible at all."""
    def _r(ctx, **kw):
        return r_collapse(ctx, mode=kw.pop("mode", mode), **kw)
    _r.__name__ = "r_qc_" + mode
    _r.__doc__ = "quake_collapse.r_collapse(mode={0!r}).".format(mode)
    return _r


RECIPES = {
    "qc_elevation": _recipe("elevation"),
    "qc_corner": _recipe("corner"),
    "qc_soft_storey": _recipe("soft_storey"),
    "qc_mid_storey": _recipe("mid_storey"),
    "qc_pancake": _recipe("pancake"),
    "qc_total": _recipe("total"),
    # THE SEVENTH NAME, and why it exists: a ladder table is STATIC, so
    # "urm DG3 -> a corner or an elevation, drawn" cannot be written as a
    # table entry — the draw has to live in a recipe. `quake_flow.
    # r_storey_collapse` is the same shape for the same reason (a soft storey
    # two times in three, a mid storey otherwise). The draw is made from the
    # BUILDING'S OWN private generator, so it takes no shared draw either.
    "qc_auto": _recipe("auto"),
}


# THE LADDER, as OVERRIDES rather than a second full table.
#
# `quake_flow` composes `LADDER_QC` by copying `LADDER` and applying these,
# so every row this table does not name — DG0-DG2, the glass slots, the
# rooftop plant, the foundation family — is literally the legacy row and
# cannot drift from it. What changes is only the grades where a collapse
# happens.
#
#   urm  DG3  corner or elevation, drawn      (was `corner_fail`)
#        DG4  an elevation from a storey up   (was `out_of_plane`)
#        DG5  total                           (was `masonry_collapse`)
#   rc   DG3  the infill of one elevation     (was `infill_fail` alone)
#        DG4  a soft storey, or a mid storey  (was `storey_collapse`)
#        DG5  pancake                         (was `pancake`)
#   rc_glass DG4  the podium's soft storey    (was `soft_storey`)
#        DG5  unchanged — `tilt_sink`. No curtain-wall tower has come down in
#             the reviewed record, and that is the one thing this family must
#             not change about the tower ladder.
LADDER_OVERRIDES = {
    "urm": {
        "DG3": [("parapet_fall", {"sides": 2, "frac": 0.8}),
                ("qc_auto", {}),
                ("roof_hole", {"frac": 0.25}),
                ("facade_scars", {"frac": 0.22}),
                ("rooftop_fail", {"frac": 0.7}),
                ("signage_fail", {}),
                ("storefront_glass", {"grade": 3})],
        "DG4": [("qc_elevation", {}),
                ("parapet_fall", {"sides": 3, "frac": 0.9}),
                ("roof_hole", {"frac": 0.35}),
                ("rooftop_fail", {"frac": 0.8}),
                ("facade_scars", {"frac": 0.18}),
                ("storefront_glass", {"grade": 4})],
        "DG5": [("qc_total", {})],
    },
    "rc": {
        "DG3": [("qc_elevation", {"infill": True}),
                ("balcony_fail", {"frac": 0.6}),
                ("rooftop_fail", {"frac": 0.7}),
                ("signage_fail", {}),
                ("parapet_fall", {"sides": 2, "frac": 0.6}),
                ("storefront_glass", {"grade": 3})],
        "DG4": [("balcony_fail", {"frac": 0.7}),
                ("storefront_glass", {"grade": 4}),
                ("qc_soft_storey", {}),
                ("rooftop_fail", {"frac": 0.8}),
                ("glass_follow", {})],
        "DG5": [("qc_pancake", {})],
    },
    "rc_glass": {
        "DG4": [("curtain_wall", {"grade": 4}),
                ("storefront_glass", {"grade": 4}),
                ("qc_soft_storey", {"storey": 0, "lean_deg": 2.5}),
                ("glass_follow", {})],
    },
}


# ---------------------------------------------------------------------------
# Host-side self-check
# ---------------------------------------------------------------------------
def check(verbose=True, styles=("commercial_mid", "commercial", "apartment",
                                "block_residential", "highrise_step",
                                "dw_terrace", "brownstone_row")):
    """No stage, no pxr: plan every mode on real kit buildings and assert the
    invariants the family is built around. Kept here so a launch script can
    gate on it the way it gates on `quake_flow.check`."""
    import numpy as np
    from detail import urban_building as ub
    from . import quake_flow as qf

    bad = []
    for name, recs in LADDER_OVERRIDES.items():
        for grade, lst in recs.items():
            for rname, _kw in lst:
                if rname not in RECIPES and rname not in qf.RECIPES:
                    bad.append("LADDER_OVERRIDES {0}/{1}: unknown recipe "
                               "{2}".format(name, grade, rname))
    for style in styles:
        for mode in MODES + ("auto",):
            rng = random.Random(11)
            pls = ub.build_building(style, 0.0, 0.0, 0.0, rng)
            info = qf.describe(style, pls, 0.0, 0.0, 0.0)
            ctx = {"info": info, "rng": rng,
                   "nrng": np.random.default_rng(11), "notes": [],
                   "tag": "chk_" + style}
            try:
                plan = plan_collapse(ctx, mode=mode)
            except Exception as exc:                       # pragma: no cover
                bad.append("{0}/{1}: plan raised {2!r}".format(style, mode,
                                                               exc))
                continue
            m = info["masses"][plan["mass"]]
            if not plan["kill"]:
                bad.append("{0}/{1}: nothing taken away".format(style, mode))
            for e in plan["kill"]:
                if e["side"] not in plan["sides"] and e["role"] not in ROOF_ROLES:
                    bad.append("{0}/{1}: killed {2} on {3}, not in {4}".format(
                        style, mode, e["name"], e["side"], plan["sides"]))
                    break
            if plan["mode"] in ("elevation", "corner") and not plan["keep_sides"]:
                bad.append("{0}/{1}: no untouched elevation left".format(
                    style, mode))
            for h in plan["heaps"]:
                for sd, c in (h["centres"] or {}).items():
                    if c["outward_m"] <= 0.0:
                        bad.append("{0}/{1}: {2} heap on {3} at {4:+.2f} m of "
                                   "the wall line, wrong side".format(
                                       style, mode, h["kind"], sd,
                                       c["outward_m"]))
            for sd in plan["stub"]:
                if sd not in plan.get("blind_sides", ()):
                    bad.append("{0}/{1}: stub on {2}, not a blind side".format(
                        style, mode, sd))
            _ = m
    if verbose:
        print("[quake_collapse] check {0}".format("ok" if not bad else
                                                  "FAILED"))
        for b in bad:
            print("  " + b)
    return bad


if __name__ == "__main__":                                    # pragma: no cover
    import sys
    sys.path.insert(0, os.path.normpath(
        os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")))
    raise SystemExit(1 if check() else 0)
