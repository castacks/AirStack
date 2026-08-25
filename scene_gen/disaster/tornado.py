"""tornado — THE tornado building-damage pipeline.

One script per disaster type, all of them calling `mesh_damage` as the API.
This is the tornado one; `disaster/quake.py` is the earthquake one and this
file mirrors it rung for rung, because the two disasters differ in what they
DO to a building and in nothing else about how the pipeline runs.
`mesh_damage` owns the geometry (fields, solidify, interior, the cutter, the
support graph); this file owns what a TORNADO means — which mechanisms are in
play at which rung, how big the debris is, how much of it leaves the lot, and
where it goes.

WHAT A TORNADO IS, AS A DIFFERENCE FROM AN EARTHQUAKE
------------------------------------------------------
Both leave a broken building, and from a drone at 100 m three things tell them
apart. All three are visible in the numbers below and nowhere else:

  * **Which end fails.** Shaking loads a building in shear and the demand is
    largest at the base, so a quake building settles onto its own footprint.
    Wind speed climbs with height and the load goes as its square, so a storm
    fails a building from the TOP down: the roof first, then the top storey
    behind it. `_field_wind`'s height `exponent` and `floor` are the whole of
    that, and they are the knobs the rungs below walk down: exponent 6.0 with
    no floor is a roof membrane and nothing under it, exponent 1.0 over a 0.75
    floor is the entire building failing at once.

  * **Where the material ends up.** A collapse drops what it consumes where it
    stood; a tornado carries it away. So `consume` is HIGH — the mass that
    left the lot is not on the lot, and modelling it as a pile of rubble at
    the foundation is the single most common way a rendered tornado reads as
    a bomb site. See `CONSUME` for why this is the opposite of the earthquake
    reading, and `THROW` for why the ejecta that goes with it is nonetheless
    SMALL.

  * **The shared bearing.** Every building on the track fails on the same face
    and throws its debris the same way. That is what makes a track read as a
    track, and it is why `field_for` takes ONE storm bearing and gives every
    mechanism the same one — where `quake.field_for` deliberately spreads its
    mechanisms around the plan on a golden angle, because ground motion has no
    bearing a whole street shares.

THE PATH OF DESTRUCTION IS NOT IN THIS FILE
--------------------------------------------
It is `field.py`'s ``kind: path``, which `compile_disaster.compile_tornado`
emits: a corridor of `width_m` across the region, easing to nothing over
`falloff_m`, with the rest of the city untouched. That field decides WHICH
buildings are hit and how hard, and `levels.LADDERS["tornado"]` quantises the
result to a rung. So "crushed in the direct path, roofs off further out" is
already stated by the composition and needs nothing here:

    on the centreline   local damage ~1.0  ->  `swept_clean`
    a third out         ~0.7               ->  `walls_breached`
    at the corridor lip ~0.3               ->  `roof_lost` / `roof_stripped`
    beyond the falloff  0.0                ->  `pristine`, and untouched

What this file owes that chain is the other half: that each of those rung
names renders as a distinct KIND of failure, in order, and that a viewer can
tell them apart. `RUNG_PLAN` is that promise.

SEVERITY IS NOT AN ARGUMENT ANYWHERE BELOW
-------------------------------------------
Same rule as `quake`, same reason. A rung is a kind of damage — a stripped
roof is a stripped roof whether the storm was an EF2 or an EF5 — so the recipe
that renders it is fixed and severity's only job is deciding which buildings
reach it. That is what lets Stage A bake ONE archetype per (type, level) and
reuse the library at every severity;
`tests/test_severity_shapes_only_the_field` holds the line on it.

USAGE
-----
    from disaster import tornado

    rep = tornado.at_level(stage, prim, "walls_breached", heading_deg=35.0)
    print(rep["cells"], rep["loose"], rep["settle"]["drop_mean"])

`shatter()` alone is the pure-geometry half — no Isaac, no PhysX — for callers
that batch many buildings and settle them together, which is what
`archetypes/bake.py` does.
"""

from collections import namedtuple

from . import levels as L
from . import mesh_damage as md

# ---------------------------------------------------------------------------
# THE LADDER RECIPES — what each RUNG means
#
#     severity           high-level config, scene-wide, continuous
#       -> field         `compile_tornado` lays a narrow PATH across the
#                        region; severity sets its width, not its values
#       -> local damage  `levels.local_damage(field(x, y), severity)`
#       -> RUNG          `levels.level_at` quantises to a named kind of damage
#       -> RECIPE        <- here. How to actually wreck this building.
# ---------------------------------------------------------------------------

#: ONE MECHANISM. A named way wind fails a building, with the field shape that
#: produces it and the fracture thresholds that read it.
#:
#: `intensity`   how hard this mechanism drives its own field.
#: `support`     damage at which material is worth cutting at all. Below it a
#:               face stays on the source prim with its own textures. A
#:               property of the DOMINANT mechanism, not the gentlest — see
#:               `at_level`.
#: `release`     a fragment's own damage at which it comes free.
#: `collapse`    damage in the column BELOW a fragment at which it comes free
#:               for lack of support.
#: `fragment_m`  debris spacing in metres. A WORLD LENGTH — the same on a shed
#:               and on a tower, which is what makes debris debris.
#: `consume`     share of freed fragments that leaves the lot entirely.
#: `blast`       outward launch speed at the settle, m/s.
#: `exponent`    height profile of the wind load: damage goes as
#:               (height fraction) ** exponent. LARGE is a roof and nothing
#:               else; near zero is the whole building at once.
#: `amp`         how far over the release threshold the profile is driven.
#: `floor`       how much of the load reaches the SILL. `exponent` alone is
#:               zero at the ground, so without this no rung can fail the
#:               bottom course of a wall — see `mesh_damage._field_wind`.
#: `windward`    share of the load carried by the face the storm strikes, on
#:               `_field_wind`'s reading — 0.5 fails one side only, 0 is
#:               plan-uniform, which is what a direct hit looks like.
#: `throw`       downwind displacement of what comes free, as a fraction of
#:               the building's own size.
#: `lift`        the same, upward.
Mech = namedtuple(
    "Mech",
    "intensity support release collapse fragment_m consume blast exponent "
    "amp floor windward throw lift")

MECHANISMS = {
    # THE ROOF COVERING, AND NOTHING UNDER IT. `release` sits just under the
    # field's peak so only the top course actually leaves — everything below
    # is cut and stays anchored, which is what a stripped roof over an intact
    # house looks like. Nothing is consumed: shingles and deck are exactly the
    # light material a storm carries away visibly, so it should be on the
    # ground downwind rather than deleted.
    #
    # `exponent` 2.6, NOT THE 6.0 THIS WAS WRITTEN WITH. 6.0 means "the top
    # sixth", which is a storey on a tower and the ridge line alone on a 6.4 m
    # bungalow — and this ladder is a suburban one. Measured: only 0.8% of the
    # Chase's 27k faces cleared `support`, too small a soup to yield a single
    # Voronoi cell, so `roof_stripped` rendered as PRISTINE on two of four
    # houses while working on the other two. A rung that is a coin flip on the
    # asset is worse than a rung that is wrong.
    #
    # WHERE THE ROOF ACTUALLY IS, measured rather than assumed: on these
    # bungalows the eaves are at t ~ 0.5 and the face count above them is thin
    # (26% of the Chase's faces above t=0.5 but 5.9% above 0.6; the Gilchrist,
    # which carries 89k faces of window and trim detail low down, has 7.0% and
    # 0.56%). Face count is not area, and a band picked on the assumption that
    # it is lands above the roof line. `support` 0.24 under exponent 2.6 opens
    # the cut at t ~ 0.52, which is the eaves, and `release` 0.52 frees from
    # t ~ 0.74, which is the covering over them.
    "peel":
        Mech(0.65, 0.24, 0.52, 0.95, 2.4, 0.00, 0.0, 2.6, 1.35, 0.00, 0.35,
             0.10, 0.00),
    # THE ROOF STRUCTURE GOES, AND THE TOP COURSE OF WALL WITH IT. Once the
    # deck is off, the top of the wall has lost the diaphragm that braced it,
    # so the failure reaches a storey down — `exponent` 2.2 against `peel`'s
    # 3.6, and a lower `release` so the rafters follow the covering.
    "roof_off":
        Mech(0.90, 0.18, 0.40, 0.62, 2.2, 0.20, 0.2, 1.8, 1.45, 0.00, 0.40,
             0.16, 0.03),
    # THE WINDWARD WALL RACKS AND FAILS FULL HEIGHT. The pressure is on one
    # face and the frame has no shear wall to give it back, so the failure
    # runs to the sill: a 0.35 `floor` under a mild profile, where the two
    # rungs above it have none. `windward` 0.50 confines it to the struck side
    # so the leeward wall evaluates under `support`, is never cut, and survives
    # as its own fully textured prim. That standing far wall beside an open
    # cross-section is what `walls_breached` means.
    "rack":
        Mech(0.95, 0.16, 0.34, 0.55, 1.8, 0.35, 0.3, 1.2, 1.35, 0.35, 0.50,
             0.40, 0.06),
    # DIRECT HIT: THE HOUSE IS CRUSHED AND TAKEN AWAY. Under the vortex there
    # is no sheltered face — the load arrives from every side in turn — so
    # `windward` drops to 0.15, and a 0.75 `floor` carries three quarters of
    # the load to the ground: the sill plate fails with the ridge and the whole
    # structure comes apart at once, rather than being peeled from the top.
    # `release` is low enough that nearly everything goes, the `throw` is the
    # largest in the table, and so is the `consume` — because what is left on
    # the slab afterwards is a bare foundation and a scatter of splinters, not
    # a building-shaped pile.
    #
    # `amp` 1.35 rather than the 1.8 that first cleared the threshold
    # everywhere: at 1.8 the field CLIPPED over half the building, and a
    # constant field is a field with no gradient for `fracture_seeds` to bias
    # against — every fragment came out the same size, which reads as a
    # tessellation rather than as debris. 1.35 still frees 100% of the
    # material and leaves the gust variation visible in the piece sizes.
    "sweep":
        Mech(1.00, 0.10, 0.26, 0.36, 1.4, 0.70, 0.5, 1.0, 1.35, 0.75, 0.15,
             0.60, 0.15),
}

#: WHAT A RUNG IS: a list of ``(mechanism, share of the plan)``, worst first.
#: The names are `levels.LADDERS["tornado"][STRUCTURE]` and nothing else may
#: appear here — an invented name bakes no art.
#:
#: A rung is a SEVERITY, not a mode: a real building in a storm shows several
#: at once — the windward wall is gone, the roof over the rest of it is open
#: to the sky, and the far corner has only lost its shingles. `share` is the
#: fraction of the footprint the mechanism claims, as a wedge about the
#: WINDWARD bearing (`field_for`), so the wedges nest concentrically around
#: the struck face and the damage grades off toward the lee. That nesting is
#: deliberate and is the difference from `quake.RUNG_PLAN`, whose mechanisms
#: are spread around the plan on a golden angle: an earthquake's failures are
#: unrelated to each other's position, a storm's are all downwind of the same
#: face.
RUNG_PLAN = {
    "roof_stripped":  [("peel", 1.00)],
    "roof_lost":      [("roof_off", 0.65), ("peel", 1.00)],
    "walls_breached": [("rack", 0.45), ("roof_off", 0.75), ("peel", 1.00)],
    # THE ONE RUNG THAT IS A SINGLE MECHANISM, and deliberately. Every other
    # rung is a composition because a partially-hit building fails in several
    # ways at once; a direct hit is not partial. `sweep` is already plan-uniform
    # (`windward` 0.15) and reaches the sill, so a wedge for it is a
    # contradiction — masking the lee 54 deg out of a share of 0.85 left the
    # far wall standing on the rung whose name says nothing is left, and the
    # gentler mechanisms under it were dominated by `compose`'s maximum
    # everywhere and contributed nothing but seeds.
    "swept_clean":    [("sweep", 1.00)],
}


def steps_for(level, plan=None):
    """The ``[(mechanism, share), ...]`` a rung means, or an explicit *plan*.

    `plan` is how a caller asks for a composition the ladder does not name —
    one mechanism on its own, or a pairing being looked at. It is checked
    against `MECHANISMS` here so a typo fails at the call rather than as an
    empty field an hour into a bake.
    """
    if plan is not None:
        out = [(str(n), float(sh)) for n, sh in plan]
        bad = [n for n, _ in out if n not in MECHANISMS]
        if bad:
            raise KeyError(f"no such mechanism: {bad}; "
                           f"have {sorted(MECHANISMS)}")
        return out
    return list(RUNG_PLAN.get(str(level or "pristine")) or [])


def plan_for(level, plan=None):
    """The DOMINANT mechanism for a rung, or None for `pristine`/unknown.

    Unknown degrades to "nothing happened" rather than raising, for the reason
    `levels.NONE_LADDER` does: a typo should not fail a bake that has already
    cost an hour.
    """
    steps = steps_for(level, plan)
    return MECHANISMS[steps[0][0]] if steps else None


def field_for(level, seed=0, heading_deg=None, plan=None):
    """``f(bounds) -> Failure`` for a rung: every mechanism, composed.

    A factory rather than a field because `solidify` moves the bounds the
    field is anchored to — see `mesh_damage.damage_building`.

    ONE BEARING FOR ALL OF THEM. `heading_deg` is the storm's, straight off
    `disaster.field.heading_deg`, and every mechanism is built on it and
    masked to a wedge centred on the face it strikes (heading + 180). Giving
    each mechanism its own bearing the way `quake.field_for` does would put
    the roof damage on one side and the wall failure on another, which is not
    a storm — and it would break the one property that makes a whole street
    read as a single track.
    """
    steps = steps_for(level, plan)
    # The bearing seeds off the building when the scene has none to give, so a
    # preview or a bake is reproducible; a real scene always supplies one.
    head = (float(heading_deg) if heading_deg is not None
            else (61.0 * (int(seed) % 7) + 35.0) % 360.0)
    windward = (head + 180.0) % 360.0

    def build(bounds):
        parts = []
        for k, (name, share) in enumerate(steps):
            m = MECHANISMS[name]
            f = md.field_tornado(
                bounds, m.intensity, int(seed) + 13 * k, heading_deg=head,
                exponent=m.exponent, amp=m.amp, floor=m.floor,
                windward=m.windward, throw=m.throw, lift=m.lift)
            mask = None if share >= 0.999 else md.sector_mask(
                bounds, windward, share)
            parts.append((f, mask))
        return md.compose("tornado", parts,
                          char=0.03 * max((MECHANISMS[n].intensity
                                           for n, _ in steps), default=0.0))

    return build


#: Inset applied to loose fragments, as a fraction of their own size. 0.97 is
#: `vtk_fracture`'s long-standing value and is a hairline at building scale:
#: 1.5 cm on a 1 m chunk. Large enough that the solver starts legal, small
#: enough that the gap does not read as a gap.
SHRINK = 0.97

#: OFF, AND THE MEASUREMENT THAT TURNED IT OFF IS WHY IT IS STILL HERE.
#:
#: A plank pile launches where a lump pile settles (`walls_breached`, same
#: house and seed: -1.1 m as masonry, +2.5 m as timber), and the obvious
#: suspect was the inset — `shrink` takes a fixed FRACTION off every axis, so
#: an anisotropic fragment gets a gap proportional to each axis's own extent
#: and almost none across the thin one, which is where its neighbours are.
#: `fracture_to_stage` grew `gap_m` to inset by an absolute distance instead.
#:
#: It did not help, and that is worth recording so nobody tries it twice:
#:
#:     Colbert walls_breached   proportional shrink 0.97   +5.0 m
#:                              absolute gap 3 cm         +12.8 m
#:                              absolute gap, floored     +12.8 m  (identical)
#:
#: Byte-identical with and without the floor means the clamp never bound, so
#: the sliver theory that motivated the floor was wrong too. Whatever launches
#: a plank pile, it is not the starting clearance. The live suspect is the
#: ejecta THROW: it displaces each fragment by an amount proportional to its
#: own damage, and a 4.8 m plank driven a metre past its neighbour overlaps
#: far more of it than a 2.4 m lump does — the same coupling `THROW` already
#: documents, made worse by length. Kept at 0.0 so the isotropic path is
#: untouched; `mesh_damage.gap_m` remains available for whoever tests that.
GAP_M = 0.0

#: WHAT A SUBURBAN HOUSE IS MADE OF. A tornado ladder is a suburban ladder
#: (see the note on the rungs), and a detached house in this library is a
#: timber frame — studs, sheathing, siding, rafters — not the masonry every
#: number in `mesh_damage` assumed before `MATERIALS` existed. It buys two
#: things: a 0.15 m wall instead of 0.5 m, and a plank-shaped fragment instead
#: of a lump.
#:
#: It is a DEFAULT and not a decision: the material belongs to the asset, so a
#: caller that knows better (an asset set declaring masonry row houses, the
#: urban library) passes its own and nothing here has to change.
MATERIAL = "timber"

#: TARGET DEBRIS SIZE, in metres, for every building in the library. Finer
#: than the earthquake's 2.0: a quake breaks masonry into chunks, a storm
#: splinters timber and tears sheet material, and the aftermath photographs as
#: a field of boards rather than a pile of slabs. Each mechanism sets its own
#: `fragment_m` around this and `mesh_damage.cells_for` derives the count, so
#: the SIZE is fixed and the count varies with the building.
DEBRIS_M = 1.6

#: Outward launch speed at the settle, m/s. Small, and deliberately smaller
#: than the earthquake's 6.0, because for a storm the separation is already
#: done: `Failure.ejecta` has translated every released fragment downwind by
#: `throw` before PhysX ever sees it, so the blast is only here to keep the
#: remaining contacts from starting jammed.
#:
#: MEASURED (four bungalows, `tools/damage_spread.py --set suburban`): 1.0 m/s
#: on `rack` over ~100 cells took the mean vertical displacement to +7.5 m and
#: 2.0 on `sweep` over ~220 cells to +19.8 m. Both are pieces going UP, which
#: is a detonation and not a storm. `quake`'s header records the same coupling
#: from the other end — fragment count and blast speed are coupled, and small
#: pieces need a small push — so these are now 0.0 to 0.5.
BLAST = 0.5

#: Share of the loose fragments that never reaches the simulation at all.
#:
#: THE OPPOSITE READING FROM THE EARTHQUAKE'S. There, consumed material is
#: pulverised into the voids of its own pile and the number is what makes a
#: collapse read as a collapse. Here it is material that LEFT — lofted, and
#: deposited somewhere the building no longer is. Both delete fragments; only
#: one of them is about the pile. The consequence is that a tornado's number
#: is much larger (0.70 on a direct hit against the quake's 0.45 at full
#: intensity) and that a swept building is supposed to end up with a bare
#: slab, which is the rung's name.
#:
#: The scatter that lands back in the scene is not this module's: it is
#: `disaster.debris` off `compile_tornado`'s `debris.path_*` block, which lays
#: the continuous band along the whole corridor rather than per building.
CONSUME = 0.45

#: WHY THE THROW IS A FRACTION OF THE BUILDING AND NOT A HUNDRED METRES
#: ---------------------------------------------------------------------
#: The obvious reading of a tornado — the debris is a long way downwind — was
#: in these numbers as `throw` up to 2.2x the building radius, and it is what
#: blew every heavy rung apart. `Failure.ejecta` displaces each fragment by an
#: amount proportional to ITS OWN damage, and the gust noise that modulates
#: damage varies on a ~5 m scale while a fragment is 1.4 m across. So
#: neighbouring cells differ in damage by 0.05-0.1, and at a 19 m throw that
#: is one to two metres of DIFFERENTIAL displacement between two pieces whose
#: shared face is 1.4 m wide. They are driven through each other before PhysX
#: starts, and it resolves the interpenetration by launching the pile: +19.8 m
#: mean on `swept_clean`.
#:
#: The throw is therefore what separates a fragment from its neighbours, not
#: what carries the wreckage to the next street. The wreckage that lands
#: elsewhere is `disaster.debris` off `compile_tornado`'s `debris.path_*`
#: block, which lays a continuous band along the whole corridor — a scene-level
#: pass with no interpenetration to worry about, and the right place for it.
#: THE THROW MUST ALSO SCALE WITH HOW MUCH OF THE BUILDING IS STILL THERE, and
#: that is the second half of the same measurement. A displacement pushes a
#: released fragment into whatever did NOT move, and on a light rung almost
#: everything did not move: `roof_stripped` frees three panels off a house
#: that is otherwise intact, a 2.1 m throw put them inside the roof they came
#: off, and PhysX ejected them upward — +4.8 m median, a roof hovering over an
#: undamaged house. So the gentle rungs get the SMALLEST throw (0.10) and the
#: swept one the largest (0.60), which is the opposite of the intuition that a
#: worse storm throws its debris further, and is right for the same reason: by
#: `swept_clean` there is nothing left standing to be thrown into.
#:
#: THE PROPER FIX IS A VELOCITY, NOT A DISPLACEMENT. A storm carries material
#: downwind over time, and PhysX integrates a velocity without ever placing two
#: solids in the same space. `settle.run` only offers a RADIAL `blast` today,
#: so there is no way to say "downwind" to it; adding a directional wind
#: velocity there would let `throw` go back to describing the real distance
#: instead of being clamped by the interpenetration it causes. Written down
#: rather than done because it is a change to `settle`, which the earthquake
#: pipeline shares. See TORNADO_STATE.md.

#: Ceiling, not a target — `settle.run` stops as soon as the pile is at rest
#: and reports how much of the budget it needed.
STEPS = 4000


def shatter(stage, root_prim, intensity, seed=0, wall_m=None,
            fragment_m=DEBRIS_M, max_cells=None, shrink=SHRINK, gap_m=GAP_M,
            solid=False,
            max_edge_m=4.0, solid_kw=None, field_fn=None, field_kw=None,
            auto_cells=True, interior=True, material=MATERIAL, grain=None,
            **fracture_kw):
    """Field, thicken, cut. Returns `mesh_damage.damage_building`'s report.

    Pure geometry: numpy and pxr, no Kit and no PhysX, so this runs on the host
    or inside a worker process. The returned `loose` list is what `collapse`
    wants.

    `max_cells` defaults to whatever `fragment_m` debris implies for this
    building (`mesh_damage.cells_for`). Pass a number with `auto_cells=False`
    to take it literally.

    WHAT THE BUILDING IS MADE OF is *material* (`mesh_damage.MATERIALS`), and
    it supplies two things this file has no business deciding: how thick the
    wall is and what shape a fragment of it comes out. Both are properties of
    the construction, not of the weather — a tornado and an earthquake break
    the same stud wall into the same planks — so they are looked up rather
    than tuned here, and an explicit `wall_m` or `grain` still wins.
    """
    mat = md.material(material)
    if wall_m is None:
        wall_m = mat.wall_m
    if grain is None:
        grain = mat.grain
    n = md.cells_for(root_prim, fragment_m) if auto_cells or max_cells is None \
        else int(max_cells)
    w = md.wall_for(root_prim, wall_m) if auto_cells else float(wall_m)
    rep = md.damage_building(
        stage, root_prim, "tornado", float(intensity), seed=int(seed),
        wall_m=float(w), solid=bool(solid), max_edge_m=float(max_edge_m),
        interior=bool(interior),
        field_fn=field_fn, solid_kw=solid_kw, field_kw=field_kw,
        fragment_m=float(fragment_m), grain=grain,
        max_cells=int(n), shrink=float(shrink), gap_m=float(gap_m),
        **fracture_kw)
    rep["cells_wanted"] = n
    rep["cells_capped"] = bool(n >= md.MAX_CELLS_CAP)
    return rep


def carry_off(stage, report, fraction=CONSUME, pool=1.25, seed=0):
    """Retire a share of the loose fragments — the mass the storm took away.

    `quake.consume` under another name and with the bias reversed for a
    reason. There the draw is weighted toward the LARGEST pieces, because a
    surviving big panel is a wall that fell over and removing it is what
    leaves something reading as debris. A storm sorts by weight instead: it
    lofts what it can lift, so the pieces that leave are the SMALL ones and
    the ones left behind are the heavy slabs and the foundation. Reversing the
    bias is the whole of that difference, and it is why a swept lot comes out
    as a few big pieces on bare ground rather than as a uniform thinning.

    `pool` is the strength of the bias — candidates are the smallest
    ``fraction * pool`` of the pile and the removals are drawn from among them,
    so 1.0 removes exactly the lightest and larger values let bigger ones into
    the draw.

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
    # NEVER THE SLABS. An orphaned slab is a whole uncut piece that lost its
    # load path (`fracture_to_stage`'s SUPPORT note) — gravity dropped it where
    # it stood, the wind did not carry it anywhere, so it is not part of the
    # population this draw is over. It is also the heaviest thing in the pile,
    # which is exactly what a storm leaves behind.
    slabs = set(report.get("slabs") or ())
    loose = [p for p in loose if p not in slabs]
    if not loose:
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

    order = sorted(size, key=lambda p: size[p])           # smallest first
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
    report["loose"] = [p for p in (report.get("loose") or [])
                       if p not in gone]
    report["paths"] = [p for p in (report.get("paths") or []) if p not in gone]
    report["consumed"] = len(gone)
    return len(gone)


def collapse(stage, report, seed=0, blast=BLAST, steps=STEPS, kick=0.15,
             gpu=True, bake_result=True):
    """Drop what came free and freeze it. Needs Kit — `settle` imports carb.

    Anchored fragments are handed over as STATIC colliders rather than being
    ignored: they are what the loose pieces land on, and debris that falls
    through the part of the house still standing is not debris.

    ORPHANED SLABS ARE THE EXCEPTION TO EVERY DEFAULT HERE. A slab is a whole
    uncut piece of the building that lost its support, so unlike a 1.5 m
    splinter it is neither convex nor small — a roof plane's convex hull is a
    solid wedge that lands balanced on an edge the building never had. They get
    a decomposition instead, which is worth the cook time when there are a
    handful of them and thousands of everything else. On this pipeline the slab
    is usually the roof deck, which is the one piece `roof_stripped` is about.
    """
    import random

    from . import settle

    loose = list(report.get("loose") or [])
    if not loose:
        return {}
    slabs = [p for p in report.get("slabs") or () if p in set(loose)]
    anchored = [p for p in report.get("paths") or [] if p not in set(loose)]
    return settle.run(stage, loose, anchored, steps=int(steps),
                      kick=float(kick), blast=float(blast),
                      approx_map={p: "convexDecomposition" for p in slabs},
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
    carry_off(stage, rep, fraction=frac, seed=seed)
    rep["settle"] = collapse(stage, rep, seed=seed, blast=blast, steps=steps,
                             gpu=gpu)
    return rep


# ---------------------------------------------------------------------------
# The chain, end to end
# ---------------------------------------------------------------------------


#: Recipe knobs `at_level` will let a caller override. Anything else it is
#: handed is a `shatter` argument and is passed straight through — the two
#: must not be one `**kwargs` bag, or `solid=True` gets silently swallowed
#: into the recipe and a solid asset is thickened anyway.
OVERRIDABLE = ("intensity", "support", "release", "collapse", "fragment_m",
               "consume", "blast")


def at_level(stage, root_prim, level, seed=0, settle_it=True, steps=STEPS,
             gpu=True, heading_deg=None, plan=None, material=MATERIAL,
             **kwargs):
    """Wreck one building to a named RUNG. Severity is not an argument.

    A rung is a SET of mechanisms over wedges of the plan (`RUNG_PLAN`), not
    one mode. The composed field is built by `field_for`; the fracture
    thresholds come from the worst mechanism in the set, because a threshold
    is a property of the cut and there is only one cut.

    `heading_deg` is the STORM's bearing and should come from the scene
    (`disaster.field.heading_deg`, which `mesh_damage.apply_to_stage` forwards
    for exactly this). Left None it is derived from the seed, so a bake or a
    preview is reproducible but a street of buildings would not share a
    bearing — which is the property a track is made of.

    Keyword arguments are split, not merged: the names in `OVERRIDABLE` nudge
    the recipe, everything else (`solid`, `wall_m`, `max_edge_m`, `solid_kw`)
    goes to `shatter` untouched.

    `plan` overrides `RUNG_PLAN` with an explicit ``[(mechanism, share), ...]``
    and makes *level* a label. Not a scene path — the ladder is what ships — it
    is how a mechanism is rendered on its own, which is the only way to see
    which of them is doing what.

    Returns the shatter report with `level` and (when settled) `settle` added.
    A rung with no plan returns a report that says so and touches nothing —
    `pristine` is a legitimate outcome, not an error.
    """
    override = {k: kwargs.pop(k) for k in list(kwargs) if k in OVERRIDABLE}
    steps_plan = steps_for(level, plan)
    if not steps_plan:
        return {"level": level, "field": None, "cells": 0, "paths": [],
                "loose": [], "slabs": [], "thickened": 0, "dismissed": True}

    mechs = [MECHANISMS[n] for n, _ in steps_plan]
    top = mechs[0]
    kw = {
        # The most permissive thresholds in the set: a fragment that any
        # mechanism would have freed has to come free, and taking the maximum
        # would let the gentlest one veto the worst.
        "release": min(m.release for m in mechs),
        "collapse": min(m.collapse for m in mechs),
        # The finest spacing in the set — debris is sized by the mechanism
        # that splintered it, not averaged with the one that only lifted
        # shingles.
        "fragment_m": min(m.fragment_m for m in mechs),
        # THE DOMINANT one's, not the most permissive: there is one cut, and
        # how much of the building it leaves whole is what the rung is named
        # after. Taking the minimum would let `peel` — which every rung below
        # the top carries as its background — cut the whole house on a rung
        # whose name says only the roof went.
        "support": top.support,
        "consume": top.consume,
        "blast": top.blast,
        "intensity": top.intensity,
    }
    kw.update(override)

    rep = shatter(stage, root_prim, kw["intensity"], seed=seed,
                  fragment_m=kw["fragment_m"],
                  field_fn=field_for(level, seed, heading_deg, plan),
                  material=material,
                  support=kw["support"], release=kw["release"],
                  collapse=kw["collapse"], **kwargs)
    rep["level"] = level
    rep["mechanisms"] = [n for n, _ in steps_plan]
    # How much mass the storm carried off is part of what the RUNG means, and
    # scales with nothing else: a `roof_stripped` house has not lost 70% of
    # itself no matter how bad the storm was elsewhere.
    carry_off(stage, rep, fraction=float(kw["consume"]), seed=seed)
    if settle_it:
        rep["settle"] = collapse(stage, rep, seed=seed,
                                 blast=float(kw["blast"]), steps=steps,
                                 gpu=gpu)
    return rep


def level_for(field, x, y, severity, kind=L.STRUCTURE):
    """The rung the building at ``(x, y)`` lands on. Thin pass-through.

    Exists so a caller driving the tornado pipeline never has to remember that
    the composition is `level_at(local_damage(field(x, y), severity))`, and so
    the ordering lives in `levels.py` rather than being re-derived here.
    """
    return L.level_for("tornado", field, x, y, severity, kind)


def damage_at(stage, root_prim, field, x, y, severity, seed=0, **kw):
    """severity + field + position -> rung -> wrecked building. The whole chain.

    The single entry point that closes the loop from a high-level config to
    what one building looks like:

        rung = level_for(field, x, y, severity)      # WHICH kind of damage
        at_level(stage, prim, rung.name)             # what that kind looks like

    With `field` the ``kind: path`` corridor, this is the path of destruction:
    the same call gives a house on the centreline `swept_clean` and one at the
    corridor's lip `roof_stripped`, and severity reaches the geometry ONLY
    through that choice — which is what makes a severity sweep comparable.
    """
    rung = level_for(field, x, y, severity)
    return at_level(stage, root_prim, rung.name, seed=seed, **kw)
