"""wind_flow stage — take one building apart with wind instead of fire.

This is `disaster.damage_flow` with the fire removed and the direction put in.
It exists as a separate module rather than as a flag on that one because the
two differ in every table and in three of the four fracture arguments, and a
`if tornado:` running through the middle of `damage_building` would be the
kind of change that quietly breaks the wildfire path six months from now.

WHAT ACTUALLY CHANGES, AND WHY
------------------------------
1. **Nothing is consumed.** `damage_flow` passes `consume=0.55,
   consume_pool=1.02` — over half the fragments are never authored, biased
   hard toward the LARGEST, because a third to a half of a timber building
   genuinely burns away and a surviving big panel reads as "a wall that fell
   over" rather than as debris. A tornado does the opposite: it CONSERVES
   material and DISPLACES it, and the big recognisable pieces — a run of
   sheathing, half a roof plane, a wall section lying flat — are exactly what
   makes a track read as a track. `consume` is 0 up to `leveled`.

   The one exception is `swept`, where a high consume IS correct and means
   something different: at EF4-5 the material really is gone from the lot,
   because it is a quarter-mile downwind. `disaster.planks` puts it there.

2. **The fragments are BOARDS.** `fracture.mode="plank"` seeds an anisotropic
   lattice, whose Voronoi cells are rectangular boxes. A wall blown apart
   delaminates along its fasteners into the sections the mill cut; it does not
   craze into the alligator grid that `mode="char"` produces, which is a
   property of charring specifically. `rough` drops with it — a snapped stud
   has crisp sawn faces and one splintered end, not a uniformly fibrous
   surface, and 0.045 on a thin board visibly bows it.

3. **The materials are the building's own, plus bare timber.** No soot, no
   char, no ash. `damage.bound_texture` reads what a module was clad in
   BEFORE it is fractured (a prim handle does not survive its own subtree
   being deactivated — a string does), and a plain `_pbr` around that texture
   gives a fragment that still looks like a piece of that house. A share of
   the pieces take `planks.wood_material` instead, because a wall that has
   come apart is showing its studs and the back of its sheathing, and that
   pale timber is most of what makes a debris field photograph light.

4. **Walls go before roofs are finished.** `damage_flow`'s ladder is
   roof-first at every level, which is right for a fire: it burns upward, the
   roof structure fails first, and the walls stand until they are undermined.
   Wind is lateral. The roof still goes first — uplift takes the covering and
   then the deck at speeds well below anything that touches a wall — but the
   wall count climbs much faster after that, because past about EF2 the
   failure is the whole windward wall going over, not a course off the top.

THE DIRECTION IS NOT IN THIS MODULE
-----------------------------------
Deliberately. `wreck_building` fractures and returns; where the pieces END UP
is `settle.run(bias=...)`, and the bias is a property of the SETTLE, which
runs once over a whole grid of archetypes rather than per building. Baking a
throw direction into a per-style archetype would also be wasted: the assembly
yaws each reference to the street, so an authored direction inside it comes
out pointing wherever that house happens to face. See the assembly launcher
for how the two are reconciled (the wrecked levels are yawed to the TRACK, the
recognisable ones to the street).
"""

import math

# n_walls, partial_p, seeds, cut_range, n_floors, roof_seeds, consume
#
# `roof_seeds` is separate from `seeds` and that is the whole shape of a wind
# failure: at `roof_stripped` the roof is in pieces and every wall is
# untouched, which has no analogue in the fire ladder (there is no level at
# which a fire has taken the roof and left the walls pristine — it went past
# them to get there).
# SEED COUNTS ARE DELIBERATELY MODEST, and the constraint is the SETTLE rather
# than the look. Past `partial_collapse` every wall in the house breaks (see
# `light_high` below), a kit house carries around twenty of them, and the
# archetype bake settles the WHOLE LIBRARY in one pass — so a seed count that
# reads fine on a single bench multiplies by 48 archetypes into tens of
# thousands of rigid bodies. From the altitude this scene is captured at, a
# levelled house at 300 fragments and one at 600 are the same picture, and the
# plank field adds another 140 boards on top of either.
BREAK_PLAN = {
    "pristine":         (0, 0.00, 0,  (0.00, 0.00), 0, 0,  0.00),
    # EF0-EF1. Covering and sheathing peeled off; structure and walls whole.
    "roof_stripped":    (0, 0.00, 0,  (0.00, 0.00), 0, 10, 0.00),
    # EF1-EF2. Roof structure down. One wall loses its top courses.
    "roof_collapsed":   (1, 0.95, 9,  (0.62, 0.84), 0, 12, 0.00),
    # EF2-EF3. Exterior walls failing; the top floor plate goes with them.
    "partial_collapse": (4, 0.55, 11, (0.30, 0.58), 1, 13, 0.00),
    # EF3-EF4. Everything down, material still on the lot.
    "leveled":          (7, 0.18, 13, (0.06, 0.24), 3, 14, 0.10),
    # EF4-EF5. Slab swept. The material is not destroyed, it is DOWNWIND —
    # `planks.scatter_from_wreck` is where it went, and this number is only
    # how much of it left the footprint.
    "swept":            (9, 0.05, 14, (0.02, 0.12), 5, 16, 0.62),
}

DAMAGED_LEVELS = ("roof_stripped", "roof_collapsed", "partial_collapse",
                  "leveled", "swept")

# Share of fragments bound to bare sawn timber rather than to the module's own
# cladding, per level. Climbing, because the further a building is taken apart
# the more of what you are looking at is its INSIDE — studs, joists, the
# unpainted back of sheathing — and that is pale where the outside is painted.
_BARE = {"roof_stripped": 0.30, "roof_collapsed": 0.42,
         "partial_collapse": 0.55, "leveled": 0.68, "swept": 0.72}


def cascade_supports(items, broken, radius_m=4.0, z_eps=0.5):
    """Close a break set under support — re-exported from `damage_flow`.

    Identical semantics and identical code path; wind does not change the fact
    that a wall with nothing under it comes down. Imported here rather than
    duplicated so a fix to one is a fix to both.
    """
    from .damage_flow import cascade_supports as _cs
    return _cs(items, broken, radius_m=radius_m, z_eps=z_eps)


def _debris_material(stage, parent, cache, texture, rng, bare_p, planks_mats):
    """A fragment's material: the module's own cladding, or bare timber.

    CACHED ON THE TEXTURE PATH. This is called once per fragment and a plate's
    worth of buildings runs to five figures of them; `damage.scorched_material`
    keeps a module-level cache for exactly this reason, and the plain path
    needs one just as much.
    """
    from . import damage

    if rng.random() < bare_p or not texture:
        # `planks.STOCK` keys are what the plank field itself is grouped by,
        # so a fragment and a loose board off the same house take the same
        # material and read as the same material.
        return planks_mats.get("stud") or planks_mats.get("board")
    m = cache.get(texture)
    if m is None:
        # 0.45 repeats per metre = one tile a bit over two metres, which is
        # `damage._pbr`'s own documented reference value for world-projected
        # debris. Fragments carry no UVs, so this MUST be the triplanar path.
        m = damage._pbr(stage, "{0}/WreckLooks/clad_{1}".format(
            parent, len(cache)), (1.0, 1.0, 1.0), 0.82,
            texture=texture, scale_uv=(0.45, 0.45))
        cache[texture] = m
    return m


def wreck_building(stage, parent, items, tag, level, rng, nrng, planks_mats,
                   mat_cache=None):
    """Blow one building's modules apart to `level`. Returns fragment paths.

    Same contract as `damage_flow.damage_building` — `items` is the building's
    module placements, the return is a flat list of newly authored fragment
    prim paths, and the STATIC STUBS COME BACK IN IT TOO. That last point is
    load-bearing and is inherited deliberately: once a module has been broken
    it is not attached to anything, so treating the surviving stub as static
    geometry pins it exactly where the cut left it and it hangs over whatever
    used to hold it up.

    `planks_mats` is `planks.materials(stage, parent)`; `mat_cache` is an
    optional dict shared across buildings so one cladding texture yields one
    material for the whole run rather than one per house.
    """
    from pxr import UsdShade
    from . import damage, fracture

    n_walls, partial_p, seeds, cut_range, n_floors, roof_seeds, consume = \
        BREAK_PLAN[level]
    frags = []
    if not seeds and not roof_seeds:
        return frags
    cache = mat_cache if mat_cache is not None else {}
    bare_p = _BARE.get(level, 0.5)

    roofs = [q for q in items
             if damage._sub_of(q.get("category")) in ("roof", "bay_roof")]
    walls = [q for q in items if damage._sub_of(q.get("category")) == "wall"]
    floors = [q for q in items if damage._sub_of(q.get("category")) == "floor"]

    # WHICH WALLS. `damage_flow` picks lowest-first because fire burns upward
    # and undermines from the bottom. Wind does not care about height — it
    # cares about EXPOSURE, and the walls that fail are the ones facing into
    # it. Without a heading in this module (see the header) the honest model
    # is a random draw over the whole set rather than a height-sorted one,
    # which at least does not encode the WRONG mechanism.
    pick = (rng.sample(walls, min(n_walls, len(walls)))
            if n_walls and walls else [])
    floors_sorted = sorted(floors, key=lambda q: -float(q.get("z_m", 0.0)))
    pick_f = floors_sorted[:min(n_floors, len(floors_sorted))]

    # "LIGHT" WALLS — the ones not hard-picked — normally lose only a course
    # off the top, which is what keeps a partly-collapsed house recognisable
    # as a house. AT `leveled` AND `swept` THAT DISTINCTION HAS TO COLLAPSE.
    # `n_walls` is 7-9 and a kit house can carry twenty; leaving the other
    # thirteen standing to three quarters of their height gives a "swept"
    # slab with most of its walls still up, which is the level's whole
    # premise inverted. Past `partial_collapse`, every wall breaks low.
    light = []
    if level in ("partial_collapse", "leveled", "swept"):
        light = [q for q in walls if q not in pick]
    light_high = (level == "partial_collapse")

    broken_ids = cascade_supports(
        items, {id(q) for q in pick + pick_f + roofs + light})
    light_ids = {id(q) for q in light}
    targets = [q for q in items if id(q) in broken_ids]

    for q in targets:
        path = q.get("prim_path")
        if not path:
            continue
        if is_immovable(q.get("category")):
            continue
        sub = damage._sub_of(q.get("category"))
        is_roof = sub in ("roof", "bay_roof")
        # The roof has its own seed count, because at `roof_stripped` it is
        # the only thing that broke and it has to carry the whole read: a
        # roof in four pieces is a roof that fell in, and what a wind does is
        # take it away in sheets.
        n_seed = roof_seeds if is_roof else seeds
        if not n_seed:
            continue
        out = "{0}/brk_{1}_{2}".format(parent, tag, path.rsplit("/", 1)[-1])
        src_tex = damage.bound_texture(stage, path)
        cut = ((0.72, 0.94) if (id(q) in light_ids and light_high)
               else cut_range)

        if sub == "wall" and ((id(q) in light_ids and light_high)
                              or rng.random() < partial_p):
            st, lo = fracture.fracture_partial(
                stage, path, out, n_pieces=n_seed, rng=nrng,
                cut_frac=rng.uniform(*cut), mode="plank",
                rough=0.010, consume=consume * 0.5)
            made = list(st) + list(lo)
        else:
            made = fracture.fracture_prim(
                stage, path, out, n_pieces=n_seed, rng=nrng,
                mode="plank", rough=0.010, verbose=False,
                consume=consume, consume_pool=1.6,
                # A SHEET OF SHEATHING IS THIN, and the default cull is a
                # fraction of the module's own bounding-box volume — which a
                # thin panel has very little of. 0.004 discarded exactly the
                # broken sheets that make this field read; 0.0008 keeps them.
                min_volume_frac=0.0008)
        frags.extend(made)
        for pth in made:
            pr = stage.GetPrimAtPath(pth)
            if not pr or not pr.IsValid():
                continue
            m = _debris_material(stage, parent, cache, src_tex, rng, bare_p,
                                 planks_mats)
            if m is not None:
                UsdShade.MaterialBindingAPI(pr).Bind(m)

    # DOORS AND WINDOWS GO EARLY AND THEY GO FIRST. A pressure differential
    # takes the openings before it takes the structure, which is why a house
    # at EF1 has its garage door in the back garden and its roof still on.
    # Deactivated rather than fractured, exactly as `damage_flow` does it —
    # what is left of them is in the plank field.
    if level != "roof_stripped":
        for q in items:
            if damage._sub_of(q.get("category")) not in ("door", "door_slot"):
                continue
            if is_immovable(q.get("category")):
                continue
            pth = q.get("prim_path")
            pr = stage.GetPrimAtPath(pth) if pth else None
            if pr and pr.IsValid() and pr.IsActive():
                pr.SetActive(False)
    return frags


# WHAT A TORNADO CANNOT MOVE, and it is NOT `damage.INCOMBUSTIBLE`.
#
# That tuple is a list of things that do not BURN, and half of it is wrong
# here in the most visible way possible: a `streetlight` and a `sign` come
# through a wildfire visibly untouched, and they are among the first things a
# tornado shears off and throws. Reusing the fire list would leave a corridor
# of levelled houses with every street sign standing perfectly upright in it.
#
# What survives a tornado is what is IN THE GROUND or full of water: the pool,
# the manhole, the storm drain, the slab. Matched as substrings, same as the
# fire list.
IMMOVABLE = ("pool", "water", "manhole", "storm_drain", "slab", "foundation")


def is_immovable(category):
    c = str(category or "").lower()
    return any(k in c for k in IMMOVABLE)


def throw_bias(heading_deg, curl_deg, speed_mps):
    """The `settle.prepare(bias=...)` vector for a track heading. m/s."""
    a = math.radians(float(heading_deg) + float(curl_deg))
    return (math.cos(a) * float(speed_mps),
            math.sin(a) * float(speed_mps),
            # A SMALL UPWARD COMPONENT, because the pieces have to CLEAR
            # things to travel. Given a purely horizontal launch every
            # fragment ploughs straight into whatever is next to it and stops
            # a metre from where it started, which reads as a collapse with a
            # limp. It is also physically the right story: the vertical
            # velocities in a tornado's core are what make it a tornado.
            0.35 * float(speed_mps))
