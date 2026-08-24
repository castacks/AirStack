"""quake — the earthquake pipeline, assembled from the parts that each win.

WHAT THIS IS
------------
Three ways to wreck a building grew up in this repo side by side, and each of
them is the best at exactly one thing:

    mesh_damage      the FIELD and the CUT. A scalar failure field over world
                     space decides where the building fails, seeds are drawn
                     against it with variable-radius Poisson thinning, and the
                     cut itself is a k-d tree lookup of the 24 nearest
                     bisectors per cell over a per-cell face subset — so it is
                     near-linear in cell count. It is also the only one of the
                     three that carries the source MATERIALS, UVs and geom
                     subsets onto the fragments.

    vtk_fracture     the SEPARATION. `shrink=0.97` insets every fragment about
                     its own centroid, which is what lets PhysX start from a
                     legal configuration instead of from pervasive contact.
                     Nothing else here had it, and without it a cut building
                     is a stable arch that sags and never falls.

    earthquake       the LOOK of the pile, via `solids.thicken` — but it pays
                     for it with a ray-cast visibility pass that took 358 s on
                     one tower against 33 s for the whole of mesh_damage.

    settle           the COLLAPSE, once `blast` gives the pieces somewhere to
                     go.

None of them wins on two. So this module picks the field and cut from
`mesh_damage`, the inset from `vtk_fracture` (now a `shrink` argument on
`fracture_to_stage` rather than a reimplementation), thickening from
`mesh_damage.solidify` rather than the ray-cast one, and hands the loose
fragments to `settle.run` with a radial blast.

WHY AN ORCHESTRATOR AND NOT A FOURTH IMPLEMENTATION
---------------------------------------------------
Because the repo has already paid for that mistake once. `mesh_damage`'s own
header records a second parallel vocabulary — `lean`, `pancake`, `crumble`,
`shockwave`, `punch_hole` — that was deleted precisely because it duplicated
the fracture running after it and did not compose. Every line below is a call
into an existing stage; there is no geometry code in this file, and a fix to
any stage reaches this pipeline for free.

THICKENING, AND WHAT "IF NECESSARY" MEANS
-----------------------------------------
Three separate things can make thickening the wrong move, and only one of them
is a judgement:

  * the asset set declared `solid: true` — its art already has material in the
    walls, so extruding it would double-wall the building. Declared, not
    detected: see `mesh_damage.solidify`.
  * the mesh is a thin detail — a mullion, a railing, a gutter. Handled inside
    `solidify` by the `max_span_frac` cap against the second-smallest bbox
    dimension, per mesh.
  * nothing on this building will ever come free, so there is nothing for the
    thickness to serve. `damage_building` probes the field first and dismisses
    the building before stage one.

So `wall_m` is passed unconditionally and the decision is left where the
knowledge is. What this module does add is a *report*: `thickened` says how
many meshes actually got volume, so a building that came out as paper is
visible instead of being inferred from the render.

MEASURED, ON THE ASSET THAT BROKE EVERYTHING ELSE
--------------------------------------------------
`ModernCityEnvironment/Collected_Building02` — 91 x 96 x 69 m, 287,430 faces,
471,324 points in ONE mesh, open shell. Earthquake at intensity 0.9, settled
with a 1500-step ceiling. `drop` is the mean vertical displacement, which is
the number that says whether a building collapsed:

    pipeline                        shatter   settle   drop      textured
    vtk_fracture (Krrish)             23 s     1.7 s   -22.1 m   no
    earthquake.shake (preview tool)  317 s     3.1 s    -4.8 m   yes
    mesh_damage alone                 76 s    39.0 s    -5.5 m   yes
    quake, no consume                 94 s    41.6 s    -6.7 m   yes
    quake, 32 cells (DEFAULT)        107 s     6.0 s   -17.3 m   yes
    quake, 64 cells + consume 0.6    156 s     4.3 s   -17.6 m   yes

Two things that sweep taught, both counter-intuitive enough to record:

  * **Cutting finer does not make a building collapse; it makes it explode.**
    At 160 cells with a 12 m/s blast the mean displacement came out POSITIVE
    (+2.9 m) and pieces travelled 121 m. Fragment count and blast speed are
    coupled — small pieces need a small push — which is why `BLAST` came down
    to 6 m/s when `consume` went in.
  * **`support` was a red herring.** It looks like the knob that decides how
    much of the building is wrecked, but at these intensities the field is
    over the threshold nearly everywhere: only 2,668 of 1.5M faces stayed
    uncut at 0.12, and 248 at 0.04. The tower left standing in every earlier
    render was not uncut source geometry, it was the fragments themselves,
    stacked and self-supporting.

The settle also gets *faster* as the damage gets better — 41.6 s to 6.0 s —
because a pile that actually falls reaches rest, while one that is jammed
grinds against itself for the whole step budget.

USAGE
-----
    from disaster import quake

    rep = quake.run(stage, stage.GetPrimAtPath("/World/Building"),
                    intensity=0.9, seed=0)
    print(rep["cells"], rep["loose"], rep["settle"]["drop_mean"])

`shatter()` alone is the pure-geometry half — no Isaac, no PhysX — for callers
that batch many buildings and settle them together (`tools/batch_damage.py`
does exactly that, and one settle over a grid is much cheaper than N).
"""

from collections import namedtuple

from . import levels as L
from . import mesh_damage as md

# ---------------------------------------------------------------------------
# THE LADDER RECIPES — what each RUNG means, with severity nowhere in sight
#
# This is the last hop of the chain and the only one that was missing:
#
#     severity           high-level config, scene-wide, continuous
#       -> field         `compile_earthquake` shapes a radial field around the
#                        epicentre; severity sets its reach, not its values
#       -> local damage  `levels.local_damage(field(x, y), severity)`
#       -> RUNG          `levels.level_at` quantises to a named kind of damage
#       -> RECIPE        <- here. How to actually break this building.
#
# SEVERITY MUST NOT APPEAR BELOW THIS LINE, and that is the whole point of the
# separation. A rung is a KIND of failure — a soft storey is a soft storey
# whether the quake was a 0.4 or a 0.9 — so the recipe that renders it is
# fixed, and severity's only job is deciding which buildings reach it. That is
# what lets Stage A bake ONE archetype per (type, level) and reuse the library
# at every severity, and it is what `tests/test_severity_shapes_only_the_field`
# holds the line on.
#
# `intensity` is therefore a property of the RUNG, not of the scene: it is the
# local field strength at which this kind of failure is rendered. Feeding the
# scene's own intensity in instead is what made `cracked` (midpoint 0.28) come
# out byte-identical to `pristine` — `damage_building` dismisses any building
# whose peak damage is under `release`, so the bottom two rungs of a five-rung
# ladder collapsed into one archetype.
#
# WHY `release` IS THE KNOB THAT SEPARATES THE RUNGS
# --------------------------------------------------
# It is the threshold a fragment's own mean damage must clear to come free, so
# it decides HOW MUCH of a cut building actually leaves — which is exactly the
# difference between the rung names. `cracked` sits just under the field's
# peak so only the very worst pieces drop and the rest stay anchored as visible
# cracks; `pancaked` sits low enough that nearly everything releases. The cut
# itself barely changes.
# ---------------------------------------------------------------------------

#: ONE MECHANISM. A named way a building fails, with the field shape that
#: produces it and the fracture thresholds that read it.
#:
#: `intensity`   how hard this mechanism drives its own field.
#: `release`     a fragment's own damage at which it comes free.
#: `collapse`    damage in the column BELOW a fragment at which it comes free
#:               for lack of support.
#: `fragment_m`  crack spacing in metres. A WORLD LENGTH — the same on a shed
#:               and on a tower, which is what makes rubble rubble.
#: `consume`     share of freed fragments pulverised away entirely.
#: `blast`       outward launch speed, m/s.
#: `asymmetry`   how much of the demand one side of the plan carries.
#: `shear_band`  how much of the plan the transition takes. Small is an edge.
#: `storey_band` height of the failing band as a fraction of the building,
#:               0 for "no band, fail on the height profile alone".
#: `storey_at`   where that band sits, 0 = ground floor.
Mech = namedtuple(
    "Mech",
    "intensity release collapse fragment_m consume blast asymmetry "
    "shear_band storey_band storey_at")

MECHANISMS = {
    # Fracture lines on a building that stays up. `release` just under the
    # field's peak, nothing consumed, nothing thrown.
    "crack": Mech(0.60, 0.56, 0.95, 2.2, 0.00, 0.0, 0.35, 1.00, 0.00, 0.00),
    # THE GROUND FLOOR GOES AND THE REST COMES DOWN ON IT. `storey_band`
    # confines the failure to a window at the base, so the mass above is never
    # cut — it loses its support instead, which `unsupported` detects and the
    # settle turns into a topple onto the wreckage. Without the band the
    # height profile merely makes a low failure LIKELY, and the rung came out
    # looking like a pancake with pieces hanging over it.
    "soft_storey":
        Mech(0.95, 0.34, 0.45, 1.8, 0.35, 0.0, 0.25, 1.00, 0.30, 0.06),
    # One side shears off full height and leaves the floor plates showing.
    # asymmetry 1.0 with a narrow band makes the boundary an edge: the far
    # side evaluates under `support`, is never cut, and keeps its own
    # textures. `blast` is nearly off — this side should DROP, not scatter.
    "shear_off":
        Mech(0.95, 0.42, 0.45, 2.0, 0.30, 0.5, 1.00, 0.22, 0.00, 0.00),
    # Everything releases and over half the mass is pulverised into the voids.
    "pancake":
        Mech(1.00, 0.30, 0.40, 1.6, 0.55, 1.5, 0.20, 1.00, 0.00, 0.00),
}

#: WHAT A RUNG IS: a list of ``(mechanism, share of the plan)``, worst first.
#:
#: A rung is a SEVERITY, not a mode. A real building at one severity shows
#: several kinds of failure at once — a wing pancakes, the wing beside it
#: loses its ground floor and leans onto the wreck, the far end merely cracks
#: — and forcing a rung to name exactly one of those is what made the ladder
#: non-monotonic to look at: `soft_storey` and `partial_collapse` were tuned
#: independently against different fields, and the gentler rung came out
#: looking worse than the harsher one.
#:
#: So severity decides HOW MANY mechanisms are in play and how much of the
#: plan the worst one takes; the rung name is just the label of that worst
#: one. `share` is the fraction of the footprint the mechanism claims, as a
#: wedge about a per-mechanism heading — see `mesh_damage.sector_mask`. A
#: share of 1.0 is the whole building and needs no mask.
RUNG_PLAN = {
    "cracked":          [("crack", 1.00)],
    "soft_storey":      [("soft_storey", 0.55), ("crack", 1.00)],
    "partial_collapse": [("shear_off", 0.45), ("soft_storey", 0.35),
                         ("crack", 1.00)],
    "pancaked":         [("pancake", 0.70), ("shear_off", 0.40),
                         ("soft_storey", 0.40)],
}


def plan_for(level):
    """The DOMINANT mechanism for a rung, or None for `pristine`/unknown.

    Unknown degrades to "nothing happened" rather than raising, for the reason
    `levels.NONE_LADDER` does: a typo should not fail a bake that has already
    cost an hour.
    """
    steps = RUNG_PLAN.get(str(level or "pristine"))
    return MECHANISMS[steps[0][0]] if steps else None


def field_for(level, seed=0):
    """``f(bounds) -> Failure`` for a rung: every mechanism, composed.

    A factory rather than a field because `solidify` moves the bounds the
    field is anchored to — see `mesh_damage.damage_building`.
    """
    steps = RUNG_PLAN.get(str(level or "pristine")) or []

    def build(bounds):
        parts = []
        for k, (name, share) in enumerate(steps):
            m = MECHANISMS[name]
            # A HEADING PER MECHANISM, spread around the plan so two of them
            # land on different sides instead of stacking on one. Seeded, so
            # the same building and rung reproduce.
            head = (137.5 * (k + 1) + 61.0 * (int(seed) % 7)) % 360.0
            f = md.field_earthquake(
                bounds, m.intensity, int(seed) + 13 * k,
                asymmetry=m.asymmetry, shear_band=m.shear_band,
                heading_deg=head, storey_band=m.storey_band,
                storey_at=m.storey_at)
            mask = None if share >= 0.999 else md.sector_mask(
                bounds, head, share)
            parts.append((f, mask))
        return md.compose("earthquake", parts,
                          char=0.06 * max((MECHANISMS[n].intensity
                                           for n, _ in steps), default=0.0))

    return build


#: Inset applied to loose fragments, as a fraction of their own size. 0.97 is
#: `vtk_fracture`'s long-standing value and is a hairline at building scale:
#: 1.5 cm on a 1 m chunk. Large enough that the solver starts legal, small
#: enough that the gap does not read as a gap.
SHRINK = 0.97

#: Metres of wall on a REFERENCE-sized building. A wall is about half a metre
#: thick on a bungalow and on a tower alike, which is why `solidify` takes an
#: absolute length — but the two do not read the same. On a 12 m house 0.5 m
#: is masonry; on a 96 m tower it is 0.5% of the span, a foil skin inside a
#: 10 m Voronoi cell, and the fragments come out looking hollow. So the wall
#: grows with the building, slowly (square root, not linear — a skyscraper has
#: thicker walls than a house, not eight times thicker) and to a ceiling.
WALL_M = 0.5
WALL_M_MAX = 2.0

#: Building radius `WALL_M` is quoted for.
REF_RADIUS_M = 12.0

#: TARGET RUBBLE SIZE, in metres, for every building in the library.
#:
#: A chunk of broken concrete is a metre or two across whether it fell off a
#: bungalow or off a tower, and the cell count is whatever that implies. The
#: pipeline used to state it the other way round — a cell BUDGET scaled by the
#: building's area and clamped — and the clamp is what made rubble size an
#: asset property: the 96 m tower wanted 1,640 cells, got 140, and
#: `fracture_seeds` did the only thing it can when the budget binds, which is
#: scale the spacing up until the seeds fit. Twenty-metre "rubble".
#:
#: Each mechanism sets its own `fragment_m` around this — finer where the
#: material was pulverised — and `cells_for` derives the count so the budget
#: does not bind first.
RUBBLE_M = 2.0

#: Compute ceiling on cells for ONE building. Not a design number: the cut is
#: near-linear in cells since `_fracture_hier`, but the SETTLE is not — every
#: cell is a rigid body with a cooked collider and PhysX cost climbs with the
#: contacts between them. Set from measurement; see `tools/quake_preview.py`.
MAX_CELLS_CAP = 1200

#: Outward launch speed at the structure's axis, m/s. Gravity alone cannot
#: separate a Voronoi tiling; see `settle.blast_velocities`. Kept modest
#: because `consume` now does most of the work of opening the pile: at 12 m/s
#: over 148 fragments the tower came apart UPWARD (+2.9 m mean, 121 m spread),
#: which is a detonation and not a collapse.
BLAST = 6.0

#: Share of the loose fragments that never reaches the simulation at all —
#: material pulverised and packed into the voids. See `consume`, and
#: `vtk_fracture.fracture_mesh`, which is where the idea and the large-piece
#: bias come from.
#:
#: This is the share at FULL intensity. `run` scales it by the local intensity
#: unless told otherwise, because void is a consequence of how hard the place
#: was hit: a `cracked` building has not pulverised half its mass, and applying
#: a flat fraction to the gentle rungs of the ladder deletes walls off a
#: building that is supposed to be standing.
CONSUME = 0.45

#: Ceiling, not a target — `settle.run` stops as soon as the pile is at rest
#: and reports how much of the budget it needed. Raised from 1500 with the
#: move to ~1,200-cell rubble: the budget is spent per BODY and a collapse now
#: has hundreds of them landing on each other in sequence.
STEPS = 4000


def wall_for(root_prim, wall_m=WALL_M, ref_m=REF_RADIUS_M, cap=WALL_M_MAX):
    """Wall thickness for this building's size. See `WALL_M`."""
    prims = md.mesh_prims(root_prim)
    b = md.bounds_of(prims) if prims else None
    if b is None or not b.radius:
        return float(wall_m)
    import math as _m
    scale = _m.sqrt(max(1.0, float(b.radius) / float(ref_m)))
    return float(min(float(cap), float(wall_m) * scale))


def cells_for(root_prim, fragment_m=RUBBLE_M, cap=None):
    """How many cells this building needs to break into *fragment_m* rubble.

    Derived from the building's own envelope area — facades plus roof, which
    is the surface a Voronoi cut actually tiles — divided by the area one
    fragment covers. So the COUNT varies with the building and the SIZE does
    not, which is the way round rubble works.

    `cap` is a compute ceiling, not a design choice, and it is deliberately
    high enough not to bind on the library's largest asset. When it does bind,
    `fracture_seeds` widens the spacing and the rubble gets coarser — that is
    the failure this parameterisation exists to make visible rather than
    silent, so it is reported by `shatter` as `cells_capped`.
    """
    prims = md.mesh_prims(root_prim)
    b = md.bounds_of(prims) if prims else None
    if b is None:
        return int(MAX_CELLS_CAP if cap is None else cap)
    w, d, h = (float(x) for x in b.dims)
    area = 2.0 * (w + d) * h + w * d
    want = int(round(area / max(float(fragment_m) ** 2, 1e-6)))
    top = int(MAX_CELLS_CAP if cap is None else cap)
    return int(max(24, min(top, want)))


def shatter(stage, root_prim, intensity, seed=0, wall_m=WALL_M,
            fragment_m=RUBBLE_M, max_cells=None, shrink=SHRINK, solid=False,
            max_edge_m=4.0, solid_kw=None, field_fn=None, field_kw=None,
            auto_cells=True, interior=True, **fracture_kw):
    """Field, thicken, cut. Returns `mesh_damage.damage_building`'s report.

    Pure geometry: numpy and pxr, no Kit and no PhysX, so this runs on the host
    or inside a worker process. The returned `loose` list is what `collapse`
    wants.

    `max_cells` defaults to whatever `fragment_m` rubble implies for this
    building (`cells_for`). Pass a number with `auto_cells=False` to take it
    literally.
    """
    n = cells_for(root_prim, fragment_m) if auto_cells or max_cells is None \
        else int(max_cells)
    w = wall_for(root_prim, wall_m) if auto_cells else float(wall_m)
    rep = md.damage_building(
        stage, root_prim, "earthquake", float(intensity), seed=int(seed),
        wall_m=float(w), solid=bool(solid), max_edge_m=float(max_edge_m),
        interior=bool(interior),
        field_fn=field_fn, solid_kw=solid_kw, field_kw=field_kw,
        fragment_m=float(fragment_m),
        max_cells=int(n), shrink=float(shrink), **fracture_kw)
    rep["cells_wanted"] = n
    rep["cells_capped"] = bool(n >= MAX_CELLS_CAP)
    return rep


def consume(stage, report, fraction=CONSUME, pool=1.25, seed=0):
    """Retire a share of the loose fragments. Returns how many went.

    VOID IS WHAT MAKES A COLLAPSE READ AS ONE, and it is the mechanism this
    pipeline was missing. A Voronoi fracture tiles the original volume exactly:
    every cell is supported by the cells beneath it, so a thickened tower cut
    into chunks is a dry-stone column and PhysX is right to leave it standing.
    Measured on the 96 m tower — 32 cells dropped 6.7 m and stopped, with 27 of
    30 bodies still grinding at the step ceiling.

    Cutting finer does not fix it, it inverts the failure: at 160 cells the
    same blast threw the pieces 121 m and the mean displacement came out
    POSITIVE (+2.9 m, i.e. upward). Fine rubble with nothing removed is not a
    pile, it is shrapnel.

    A real collapse loses material — pulverised into dust and packed into the
    voids — so a share of the fragments are simply never simulated.
    `vtk_fracture.fracture_mesh` has modelled this from the beginning and its
    reasoning is worth keeping: the draw is biased toward the LARGE pieces,
    because a surviving big panel is a wall that fell over, and taking those
    out is what leaves something that reads as debris. `pool` is the strength
    of that bias — candidates are the largest `fraction * pool` of the pile and
    the removals are drawn from among them, so 1.0 removes exactly the biggest
    and larger values let smaller ones into the draw.

    Fragments are DEACTIVATED, not removed: they live under a referenced layer
    where `RemovePrim` does not compose (the trap `delete_faces` documents),
    and deactivating is reversible if a caller wants the material back.
    """
    import random

    import numpy as np

    from pxr import Usd, UsdGeom

    loose = list(report.get("loose") or [])
    if not loose or fraction <= 0.0:
        return 0

    bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                           [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    size = {}
    for p in loose:
        prim = stage.GetPrimAtPath(p)
        if not prim or not prim.IsValid():
            continue
        r = bb.ComputeWorldBound(prim).ComputeAlignedRange()
        if r.IsEmpty():
            continue
        size[p] = float(np.prod(np.maximum(np.array(r.GetSize()), 1e-9)))
    if not size:
        return 0

    order = sorted(size, key=lambda p: -size[p])          # largest first
    n_drop = int(round(len(order) * float(fraction)))
    if n_drop <= 0:
        return 0
    cand = order[:min(len(order), max(n_drop, int(n_drop * float(pool))))]
    random.Random(int(seed)).shuffle(cand)
    gone = set(cand[:n_drop])

    for p in gone:
        prim = stage.GetPrimAtPath(p)
        if prim and prim.IsValid():
            prim.SetActive(False)
    report["loose"] = [p for p in loose if p not in gone]
    report["paths"] = [p for p in (report.get("paths") or []) if p not in gone]
    report["consumed"] = len(gone)
    return len(gone)


def collapse(stage, report, seed=0, blast=BLAST, steps=STEPS, kick=0.15,
             gpu=True, bake_result=True):
    """Drop what came free and freeze it. Needs Kit — `settle` imports carb.

    Anchored fragments are handed over as STATIC colliders rather than being
    ignored: they are what the loose pieces land on, and a pile that falls
    through the part of the building still standing is not a collapse.
    """
    import random

    from . import settle

    loose = list(report.get("loose") or [])
    if not loose:
        return {}
    anchored = [p for p in report.get("paths") or [] if p not in set(loose)]
    return settle.run(stage, loose, anchored, steps=int(steps),
                      kick=float(kick), blast=float(blast),
                      rng=random.Random(int(seed)), gpu=bool(gpu),
                      bake_result=bool(bake_result))


def run(stage, root_prim, intensity, seed=0, blast=BLAST, steps=STEPS,
        gpu=True, consume_frac=None, **kw):
    """`shatter` then `collapse`, with the settle report folded in.

    Returns the shatter report plus `settle`. `dismissed` (the field never
    reached the release threshold) and `cells == 0` (nothing came apart) both
    come back with an empty `settle`, which is the correct answer rather than
    an error: an undamaged building is a legitimate outcome of a low severity.
    """
    rep = shatter(stage, root_prim, intensity, seed=seed, **kw)
    # Between the cut and the settle, because a fragment that is never going
    # to exist should not be given a collider and a rigid body first.
    frac = (CONSUME * float(intensity)) if consume_frac is None \
        else float(consume_frac)
    consume(stage, rep, fraction=frac, seed=seed)
    rep["settle"] = collapse(stage, rep, seed=seed, blast=blast, steps=steps,
                             gpu=gpu)
    return rep


# ---------------------------------------------------------------------------
# The chain, end to end
# ---------------------------------------------------------------------------


def at_level(stage, root_prim, level, seed=0, settle_it=True, steps=STEPS,
             gpu=True, **override):
    """Break one building to a named RUNG. Severity is not an argument.

    A rung is a SET of mechanisms over regions of the plan (`RUNG_PLAN`), not
    one mode — see the note there. The composed field is built by `field_for`;
    the fracture thresholds come from the worst mechanism in the set, because
    a threshold is a property of the cut and there is only one cut.

    Returns the shatter report with `level` and (when settled) `settle` added.
    A rung with no plan returns a report that says so and touches nothing —
    `pristine` is a legitimate outcome, not an error.
    """
    steps_plan = RUNG_PLAN.get(str(level or "pristine"))
    if not steps_plan:
        return {"level": level, "field": None, "cells": 0, "paths": [],
                "loose": [], "thickened": 0, "dismissed": True}

    mechs = [MECHANISMS[n] for n, _ in steps_plan]
    top = mechs[0]
    kw = {
        # The most permissive thresholds in the set: a fragment that any
        # mechanism would have freed has to come free, and taking the maximum
        # would let the gentlest one veto the worst.
        "release": min(m.release for m in mechs),
        "collapse": min(m.collapse for m in mechs),
        # The finest spacing in the set — rubble is sized by the mechanism
        # that pulverised it, not averaged with the one that only cracked.
        "fragment_m": min(m.fragment_m for m in mechs),
        "consume": top.consume,
        "blast": top.blast,
        "intensity": top.intensity,
    }
    kw.update(override)

    rep = shatter(stage, root_prim, kw["intensity"], seed=seed,
                  fragment_m=kw["fragment_m"],
                  field_fn=field_for(level, seed),
                  release=kw["release"], collapse=kw["collapse"])
    rep["level"] = level
    rep["mechanisms"] = [n for n, _ in steps_plan]
    # Consume scales with the RUNG's own consume, not with any severity: how
    # much material a `pancaked` building loses is part of what pancaked means.
    consume(stage, rep, fraction=float(kw["consume"]), seed=seed)
    if settle_it:
        rep["settle"] = collapse(stage, rep, seed=seed,
                                 blast=float(kw["blast"]), steps=steps,
                                 gpu=gpu)
    return rep


def level_for(field, x, y, severity, kind=L.STRUCTURE):
    """The rung the building at ``(x, y)`` lands on. Thin pass-through.

    Exists so a caller driving the earthquake pipeline never has to remember
    that the composition is `level_at(local_damage(field(x, y), severity))`,
    and so the ordering lives in `levels.py` rather than being re-derived here.
    """
    return L.level_for("earthquake", field, x, y, severity, kind)


def damage_at(stage, root_prim, field, x, y, severity, seed=0, **kw):
    """severity + field + position -> rung -> wrecked building. The whole chain.

    The single entry point that closes the loop from a high-level config to
    what one building looks like:

        rung = level_for(field, x, y, severity)      # WHICH kind of damage
        at_level(stage, prim, rung.name)             # what that kind looks like

    Severity reaches the geometry ONLY through the choice of rung. Two scenes
    at severity 0.5 and 0.9 give the same building the same archetype whenever
    the field puts it on the same rung, which is what makes a severity sweep
    comparable — see `levels.local_damage`.
    """
    rung = level_for(field, x, y, severity)
    return at_level(stage, root_prim, rung.name, seed=seed, **kw)
