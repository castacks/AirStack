"""debris — the rubble a damaged building leaves on the ground around it.

WHY THIS IS NOT A PROP PASS ANY MORE
------------------------------------
Debris used to be scattered by `disaster_stage`, out of one pool, at a count
per building, after the damage was over. Three things were wrong with that and
they are the same thing three times over: **the debris did not know what
building it came off.**

  * MATERIAL. Every building on the block shed the same concrete chunks, so a
    brick rowhouse and a curtain-wall tower left identical rubble — while the
    fracture running a metre away was already cutting each of them as the
    material the pack declares (`mesh_damage.STRUCTURE_OF`). The pile and the
    wreck it came off disagreed.
  * VOLUME. A count is not an amount. `pieces_per_building: [14, 26]` places
    twenty pieces whether they are 0.3 m bricks or 3 m slabs, and it moved in
    two steps — destroyed shed all of it, damaged shed `damaged_debris_scale`
    of it — where the damage itself is continuous and already measured.
  * WHEN. It ran over the placement list, so the archetype library — which is
    where every wreck in a scene bigger than a block actually comes from — had
    none of it baked in.

So debris is planned HERE, from the damage report, and authored by whoever did
the damage: `mesh_damage.apply_to_stage` for a building cut live, and
`archetypes.bake` into the cell, where it settles with the fragments and is
merged into the exported archetype. `disaster_stage` calls the same planner for
the buildings no cutter reached — an authored-ruin swap, a tilt-and-sink
stand-in — because those are still damaged buildings and still owe the lot some
rubble; they just have no report to read, so their `fallen` comes from the fate
and the field instead.

THE AMOUNT IS A VOLUME, AND THE COUNT FALLS OUT OF IT
-----------------------------------------------------
`shed_m3_per_m` is how many cubic metres of loose rubble a building that came
down completely leaves for each metre of its own perimeter. Multiply by the
perimeter, scale by how much actually fell, and that is the budget; pieces are
drawn until it is spent, so a pool of bricks yields many and a pool of floor
slabs yields few — which is what the count knobs could never say.

PERIMETER, NOT AREA AND NOT VOLUME, because debris lands BESIDE a building.
Linear in the building's volume is what a collapse really does and it is
unusable: it gives a 90 m tower two hundred times the rubble of a bungalow,
when almost all of a tall building's mass lands inside its own footprint where
the fracture has already put it. Linear in the footprint AREA is the opposite
mistake — it starves the small buildings, which have proportionally the most
facade to spill from.

`spread_m` is not in the budget either, and that is deliberate. A windstorm
throws the same material further, not more of it; folding the reach into the
amount made a tornado shed six times an earthquake's rubble for no reason
except that it spread it wider.

Height enters weakly: the square root of the storey count, clamped at both
ends.
"""

import math

#: Ground spoil. Not a construction — nothing shed it, the collapse scraped it
#: up — so it appears in every structure's list below and is nobody's first
#: entry. It is also why a timber suburb has any piles at all: its houses are
#: the one thing in either asset set with no masonry anywhere in them, and
#: every mound in `suburban_nucleus` is either soil or brick.
GROUND = "earth"

#: What a structure may shed, BEST FIRST — an ordered preference, not a menu.
#: A break exposes the STRUCTURE, so the first entry is that structure's own
#: rubble and later ones are what a viewer will accept mixed into the pile;
#: `PRIMACY` makes the first dominate rather than merely appear. Each fallback
#: has a reason: a masonry wall has concrete lintels and floor plates in it, a
#: steel frame carries concrete deck and stair cores, a timber house has a
#: brick chimney and a tiled roof. Concrete does NOT fall back to brick — red
#: brick spilling out of a concrete tower is the exact mistake this table
#: exists to stop.
SHEDS = {
    "masonry": ("brick", "concrete", GROUND),
    "concrete": ("concrete", GROUND),
    "steel": ("steel", "concrete", GROUND),
    "timber": ("wood", GROUND, "brick"),
}

#: How much of its bounding box a debris asset is actually made of. A chunk of
#: rubble is a lump in a box; a mound is nearer half of one. Without this the
#: budget is spent about three times too fast and a severe scene comes out
#: sparser than a mild one did before.
OCCUPANCY = {"debris": 0.35, "debris_pile": 0.5}

#: Storeys the height factor is quoted against, and the range it may reach.
#: 12 m is four storeys — a low midrise, and the size most of both asset sets
#: actually is.
REF_HEIGHT_M = 12.0
HEIGHT_FACTOR = (0.7, 1.8)


# ---------------------------------------------------------------------------
# how much came down
# ---------------------------------------------------------------------------

def fallen(report) -> float:
    """The share of a cut building that came free, 0..1.

    THE PILE, NOT THE DIAL. `disaster_stage` used to decide this from the fate
    — 1.0 for a building it called destroyed, `damaged_debris_scale` for one it
    called damaged — which is a two-valued answer to a continuous question, and
    worse, an answer given before the cut ran. A building can be marked
    destroyed and hold together; one marked damaged can lose a whole wing.

    Slabs are excluded: an orphaned slab is a piece of the building that never
    failed, released whole because what held it up is gone, so nothing threw it
    and it is not debris (`mesh_damage.fracture_to_stage`).

    Consumed fragments ARE counted, and that is the point of counting them.
    `quake.consume` retires a share of the pile to model the material a real
    collapse pulverises into dust and packs into its own voids — so it is
    exactly the material that should be showing up around the base as fines.
    """
    cells = int((report or {}).get("cells") or 0)
    if cells <= 0:
        return 0.0
    loose = len((report or {}).get("loose") or ())
    slabs = len((report or {}).get("slabs") or ())
    consumed = int((report or {}).get("consumed") or 0)
    return min(1.0, max(0, loose - slabs + consumed) / float(cells))


# ---------------------------------------------------------------------------
# what there is to place
# ---------------------------------------------------------------------------

def pools(config, resolver) -> dict:
    """``{category: {material: [entry, ...]}}`` for this config's debris.

    An entry is ``{"usd", "scale", "axis_up", "fp", "vol_m3"}`` — everything a
    caller needs to place one and to know what placing it costs. Measured once
    per asset through the shared `SizeResolver`, so the on-disk measurement
    cache absorbs it and a severity sweep pays nothing.

    Memoised on the config OBJECT, the way `kinds._pack_materials` is: Stage A
    asks once per cell and a bake is hundreds of cells.
    """
    got = getattr(pools, "_cache", None)
    if got is not None and got[0] is config and got[1] is resolver:
        return got[2]

    from scene_generator import _normalize_usd_list, asset_materials

    scale = float(config.get("asset_scale", 1.0))
    root = str(config.get("asset_root", "") or "")
    materials = asset_materials(config)
    cfg = (config.get("usds") or {}).get("debris") or {}

    out = {}
    for cat, key in (("debris", "pieces"), ("debris_pile", "piles")):
        paths, sc_ovr, au_ovr, _yaw, _tags = _normalize_usd_list(
            cfg.get(key) or [], scale, root)
        by_material = {}
        for usd in paths:
            sc = float(sc_ovr.get(usd, scale))
            au = str(au_ovr.get(usd, "Z"))
            fp = resolver.get(usd, cat, scale=sc, axis_up=au)
            vol = (max(fp["sx"], 1e-3) * max(fp["sy"], 1e-3)
                   * max(fp.get("sz", 1.0), 1e-3) * OCCUPANCY[cat])
            by_material.setdefault(str(materials.get(usd, "")).lower(),
                                   []).append(
                {"usd": usd, "scale": sc, "axis_up": au, "fp": fp,
                 "vol_m3": vol})
        out[cat] = by_material
    pools._cache = (config, resolver, out)
    return out


#: How much likelier each material in `SHEDS` is than the one after it. The
#: order in `SHEDS` is a preference and not a menu: a brick rowhouse's rubble
#: should read as brick with some concrete in it, not as concrete with some
#: brick. Flat draws got that backwards on the urban set, where there are 26
#: concrete pieces against 12 brick ones — the 52 masonry buildings in a severe
#: scene shed 4,771 concrete and 172 brick, which is a concrete city.
PRIMACY = 0.35


def for_structure(by_material: dict, kind: str) -> list:
    """``[(weight, entries), ...]`` the debris a building may shed.

    *kind* is accepted and ignored — see the note below. `PRIMACY` still
    weights the groups so the draw is not flat, it just no longer picks WHICH
    groups on the structure's behalf.
    """
    # MATERIAL MATCHING IS OFF. `SHEDS` used to route a structure to the
    # debris its own fabric would produce — brick from masonry, plate from
    # steel — and the packs cannot support it: `urban_nucleus` has six steel
    # and three glass buildings and not one piece of steel or glazing debris,
    # so the fallback below was already doing most of the work. The result was
    # that a pack's material tags decided whether a building shed anything
    # recognisable, which is a property of the SURVEY, not of the collapse.
    # Every asset is now a candidate for every structure.
    groups = [e for e in by_material.values() if e]
    total = sum(PRIMACY ** i for i in range(len(groups)))
    return [(PRIMACY ** i / total, g) for i, g in enumerate(groups)]


# ---------------------------------------------------------------------------
# how much of it
# ---------------------------------------------------------------------------

#: How hard a piece's size pulls it toward the middle, and the limits on it.
#: The exponent is 1/3 because that turns a VOLUME ratio into a LINEAR one: a
#: piece with eight times the volume is twice the size and lands at half the
#: spread. The clamp stops a pool's extremes — a dust fleck, a whole floor
#: slab — from being flung to the horizon or nailed to the centre.
SPREAD_BY_SIZE = 1.0 / 3.0
SPREAD_CLAMP = (0.35, 1.6)

#: A MOUND'S SPREAD, as a fraction of a piece's. Mounds are the bulk of the
#: collapse — the fines the building ground itself into — and that material
#: does not travel: it slumps where the building stood. Pieces are what gets
#: thrown clear, so they keep the full spread and are the only thing that
#: should be landing well outside the footprint.
PILE_SPREAD_FRAC = 0.35

#: How wide the scatter band is, as a fraction of the wreck's half-extent.
#: Scatter lines the outline; this is how far to either side of that line it
#: strays before the per-piece size term widens or narrows it further.
SCATTER_BAND = 0.55

#: MOUNDS ARE DRAWN TO THE SIZE OF THE WRECK. The pile assets are modelled for
#: a house, and on a collapsed tower a scattering of house-sized mounds reads
#: as gravel — but the same 2x that fixes the tower makes a midrise's piles
#: bigger than the midrise. So the draw is scaled by the wreck's own
#: half-extent against this reference, which is about a house: a 16x23 m block
#: lands near 1.0 and a 42x36 m tower near 2.0.
#:
#: Volume goes as the cube, so a 2x mound is 8x the material and the budget
#: buys ~8x fewer of them. That is the intent on a tower — a few large piles
#: rather than many lost in the rubble — and it is why the reference matters:
#: at a flat 2x a small building got one enormous mound and nothing else.
PILE_REF_M = 10.0
PILE_SCALE_CLAMP = (0.6, 2.5)

#: THE BIGGEST MOUNDS ARE NOT FOR SMALL BUILDINGS. The pile pool spans a wide
#: range of asset sizes, and scaling a large one down is not the same as
#: picking a small one: the detail scales with it, so a shrunken mound of
#: boulders reads as a mound of gravel in the wrong place. A mound wider than
#: this fraction of the wreck's SHORT side is simply not drawn for it — a
#: 16x23 m block will not use anything over 4 m, a 42x36 m tower goes to 9.
#:
#: Measured against the urban pool, that is the line that matters: it has an
#: 8.0 m `huge_concrete_rubble_pile` and a 6.1 m `brick_debris_pile` at the top
#: and a 3.5 m `concrete_debris_elements` next, so a block draws from the 3.5 m
#: end and below while a tower keeps all eleven. At 0.45 the block's limit came
#: out 7.2 m, which excluded only the largest and left it using a 6.1 m mound
#: on a 16 m wreck.
PILE_MAX_FRAC = 0.25

#: Scatter carries the smaller half of the budget and is spread over a much
#: wider band than the mounds, so the same volume reads far sparser on the
#: ground. This buys back some of that density without touching the mounds.
SCATTER_DENSITY = 1.35


def _spread_factor(vol_m3: float, v_ref: float) -> float:
    """Multiplier on the placement sigma for a piece of *vol_m3*. Big -> small."""
    if vol_m3 <= 0.0 or v_ref <= 0.0:
        return 1.0
    lo, hi = SPREAD_CLAMP
    return min(hi, max(lo, (v_ref / vol_m3) ** SPREAD_BY_SIZE))


#: HOW MUCH RUBBLE A SQUARE METRE OF BUILDING LEAVES, at total collapse.
#:
#: The size heuristic is the SHELL AREA — `2*sx*sy + 2*(sx+sy)*h` — because
#: that is what a building is made of. A building is a box of air with a skin;
#: its footprint alone says nothing about a tower, and its bounding volume says
#: it is solid, which it is not.
#:
#: Calibrated so a 17x23x19 m block leaves about 115 m3 and a 50x40x66 m tower
#: about 790 m3 — a 7x ratio, which is their shell-area ratio, and the thing
#: that was missing when the budget came off fragment counts (the tower cut
#: into roughly as many pieces as the block, so it shed as little).
DEBRIS_M3_PER_M2 = 0.00494

#: AND IT SCALES FASTER THAN THE SKIN. Linear in shell area, a 66 m tower shed
#: 7x a 19 m block — its area ratio — and still read as under-dressed beside
#: it, because a tall building does not just have more skin, it drops that skin
#: from higher and further, so more of it ends up as loose rubble on the ground
#: rather than staying in the standing wreck. At 1.3 the same pair comes out
#: 12x apart, which is what the picture wants. `DEBRIS_M3_PER_M2` is
#: re-calibrated against it so the block is unchanged.
DEBRIS_SIZE_EXP = 1.3

#: WHAT EACH RUNG LEAVES, as a fraction of a total collapse. Hand-set, and
#: hand-set on purpose: the amount of rubble around a wreck is a look, and
#: deriving it from the cut — from consumed counts or fragment volumes — tied
#: it to how finely the cutter happened to work rather than to how hard the
#: building was hit. A rung is exactly the right place to say it.
RUNG_DEBRIS = {
    "pristine": 0.0,
    "cracked": 0.06,            # a facade sheds; the building is standing
    "soft_storey": 0.35,        # one floor gone, its contents on the pavement
    "partial_collapse": 0.55,
    "shear_off": 0.55,
    "pancaked": 1.0,
    "fallen": 1.0,
    "stump": 0.85,
}


def rung_share(rung) -> float:
    """The debris multiplier for a rung.

    A NUMBER IS TAKEN AS THE MULTIPLIER ITSELF, because not every caller has a
    rung to name. `disaster_stage` sheds debris for buildings the live cutter
    never reached — the ones that only tilt and sink — and what it has is a
    continuous share off the damage field, not a rung. Unknown NAMES shed as a
    collapse: better an over-dressed wreck than one on a clean lot.
    """
    if isinstance(rung, (int, float)) and not isinstance(rung, bool):
        return max(0.0, float(rung))
    return float(RUNG_DEBRIS.get(str(rung or "").lower(), 1.0))


# ---------------------------------------------------------------------------
# placing it against a wreck that has already fallen
#
# BOTH LIVE HERE so the archetype bake and the fast loop cannot drift: they
# were written in the loop first, and a baked archetype that rings the plan a
# building used to occupy — while the loop scatters along where the rubble
# actually landed — is the kind of difference nobody notices until a scene
# looks wrong for reasons that are not in the scene's own code.
# ---------------------------------------------------------------------------

def _bbox_cache():
    from pxr import Usd, UsdGeom
    return UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                             [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])


def settled_footprint(stage, paths, fallback):
    """Where the wreck actually ENDED UP: ``((sx, sy), (cx, cy))``.

    THE ORIGINAL OUTLINE IS THE WRONG LINE TO SCATTER ALONG once a building
    has come down. A tower's rubble does not stay inside its 50x40 m plan — it
    slumps and spreads — so debris lining the plan sits in a ring around a pile
    that is nowhere near it, which is exactly how it looked.

    Measured off the PERCENTILES of the settled pieces, not their bounding box:
    a handful of fragments get thrown a long way (the settle reports
    `spread_max` near its own cap on tall buildings), and one of those would
    otherwise define the outline the whole scatter follows. The 10th-90th band
    is the pile.
    """
    import numpy as np
    cache = _bbox_cache()
    pts = []
    for path in paths:
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid() or not prim.IsActive():
            continue
        rng_ = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        if rng_.IsEmpty():
            continue
        c = rng_.GetMidpoint()
        pts.append((float(c[0]), float(c[1])))
    if len(pts) < 8:
        return fallback
    a = np.asarray(pts)
    lo = np.percentile(a, 10, axis=0)
    hi = np.percentile(a, 90, axis=0)
    size = np.maximum(hi - lo, 1.0)
    mid = (hi + lo) / 2.0
    return (float(size[0]), float(size[1])), (float(mid[0]), float(mid[1]))


def pile_colliders(stage, statics, k):
    """Give each mound an invisible collider SMALLER than the mound.

    USD has no way to scale a collider away from the mesh it is built on,
    so the collider is a second prim: the same asset, the same placement,
    scaled by *k*, hidden, and the only one of the pair PhysX ever sees.
    The visible mound is left with no collision at all.

    RE-SEATED ON THE GROUND after scaling. The scale is about the prim's
    own origin, and a debris asset whose origin is not at its base would
    otherwise float — so the proxy is pushed back down by however far its
    world bottom rose.

    `convexHull`, not the exact triangle mesh `settle` gives static
    geometry: a mound is roughly convex, the hull cooks in a fraction of
    the time, and erring loose is the right way to err here — it errs
    toward things sinking in.
    """
    from pxr import Gf, Sdf, UsdGeom

    from disaster.settle import _apply_collider

    cache = _bbox_cache()
    out = []
    for path in statics:
        mound = stage.GetPrimAtPath(path)
        if not mound or not mound.IsValid():
            continue
        before = cache.ComputeWorldBound(mound).ComputeAlignedRange()
        proxy_path = Sdf.Path(str(path) + "_col")
        proxy = stage.DefinePrim(proxy_path, "Xform")
        refs = mound.GetMetadata("references")
        for ref in (refs.GetAddedOrExplicitItems() if refs else ()):
            proxy.GetReferences().AddReference(ref.assetPath, ref.primPath)
        m = UsdGeom.Xformable(mound).GetLocalTransformation()
        xf = UsdGeom.Xformable(proxy)
        xf.ClearXformOpOrder()
        op = xf.AddTransformOp()
        scale = Gf.Matrix4d().SetScale(Gf.Vec3d(k, k, k))
        op.Set(scale * m)
        after = cache.ComputeWorldBound(proxy).ComputeAlignedRange()
        if not before.IsEmpty() and not after.IsEmpty():
            dz = before.GetMin()[2] - after.GetMin()[2]
            op.Set(scale * m
                   * Gf.Matrix4d().SetTranslate(Gf.Vec3d(0.0, 0.0, dz)))
        UsdGeom.Imageable(proxy).MakeInvisible()
        _apply_collider(proxy, approximation="convexHull")
        out.append(str(proxy_path))
    return out

def budget_m3(footprint_m, height_m, rung, rules) -> tuple:
    """``(pieces_m3, piles_m3)`` of rubble a building sheds.

    SIZE x RUNG, and nothing else. *footprint_m* is ``(sx, sy)``; the shell
    area it implies with *height_m* is the size term, and `rung_share` is the
    hand-set multiplier for how hard this building was hit.
    """
    sx, sy = max(float(footprint_m[0]), 1.0), max(float(footprint_m[1]), 1.0)
    h = max(float(height_m or REF_HEIGHT_M), 1.0)
    per_m2 = float((rules or {}).get("m3_per_m2", DEBRIS_M3_PER_M2))
    pile_share = float((rules or {}).get("pile_share", 0.55))
    area = 2.0 * sx * sy + 2.0 * (sx + sy) * h
    exp = float((rules or {}).get("m3_size_exp", DEBRIS_SIZE_EXP))
    total = (area ** exp) * per_m2 * rung_share(rung)
    if total <= 0.0:
        return 0.0, 0.0
    return (total * (1.0 - pile_share) * SCATTER_DENSITY,
            total * pile_share)


# ---------------------------------------------------------------------------
# where it lands
# ---------------------------------------------------------------------------

def _fit_piles(groups, max_w, pile_scale):
    """Drop mound assets too wide for this wreck; renormalise the weights.

    Keeps the SMALLEST available mound when nothing fits, because a wreck with
    no pile at all is worse than one with a slightly oversized pile — the
    mounds are the fines, and a collapse without them reads as a pile of loose
    panels on clean ground.
    """
    def _width(e):
        f = e.get("fp") or {}
        return (max(float(f.get("sx", 0.0)), float(f.get("sy", 0.0)))
                * float(e.get("scale", 1.0)) * float(pile_scale))

    kept = [(w, [e for e in ents if _width(e) <= max_w]) for w, ents in groups]
    kept = [(w, es) for w, es in kept if es]
    if not kept:
        every = [e for _w, ents in groups for e in ents]
        if not every:
            return []
        kept = [(1.0, [min(every, key=_width)])]
    total = sum(w for w, _e in kept) or 1.0
    return [(w / total, es) for w, es in kept]


def _pick(groups, r):
    """One material group, drawn against `PRIMACY`."""
    acc = 0.0
    for w, entries in groups:
        acc += w
        if r <= acc:
            return entries
    return groups[-1][1]


def plan(config, resolver, *, kind, rung, centre, footprint_m, height_m,
         rules, rng, bias=None, keep=None) -> list:
    """Placement dicts for one building's debris. No stage, no `pxr`.

    *kind* is the STRUCTURE (`mesh_damage.MATERIALS`), *share* is `fallen`,
    *centre* is ``(x, y)`` in metres and *footprint_m* is ``(sx, sy)``.
    *bias* is an optional unit vector the piles lean toward — where a ruin
    overhangs, or downwind of a storm. *keep* is an optional
    ``(x, y) -> bool`` veto, which is how a caller keeps rubble out of an
    exclusion zone or off the edge of the region.

    Emits in the same shape `apply_placements` consumes, so the caller can
    author it directly and it survives into `targets.survey_from_placements` —
    debris is occupancy, and a casualty sampled without regard to it ends up
    inside a pile.
    """
    p_budget, l_budget = budget_m3(footprint_m, height_m, rung, rules)
    share = rung_share(rung)
    if p_budget <= 0.0 and l_budget <= 0.0:
        return []

    pool = pools(config, resolver)
    spread = float((rules or {}).get("spread_m", 7.0)) * (0.35 + 0.65 * share)
    cx, cy = float(centre[0]), float(centre[1])
    hw = max(float(footprint_m[0]), 1.0) / 2.0
    hh = max(float(footprint_m[1]), 1.0) / 2.0

    # GAUSSIAN ABOUT THE BUILDING'S OWN CENTRE, not an annulus outside it.
    # The ring this replaced put every piece beyond the footprint edge, which
    # draws a clean halo with a bare patch where the building stood — exactly
    # wrong for a collapse, where the densest rubble is on the footprint. A
    # normal distribution puts the most material where the building was and
    # thins out with distance, which is what a pile does.
    #
    # SIGMA PER AXIS, so an oblong building shed an oblong field: two sigma
    # reaches the old outer bound (half-extent + `spread_m`), so the field is
    # about as wide as before with the density moved inward.
    sx = (hw + spread) / 2.0
    sy = (hh + spread) / 2.0
    #: How far across the outline the scatter strays, before the size term.
    #: TIED TO THE WRECK, not to `spread_m`. At half of `spread_m` the band was
    #: ~3 m, which on a 42x36 m pile put every piece of scatter inside the
    #: rubble's own outline — invisible. A fraction of the wreck's half-extent
    #: scales with what it is lining, so a tower throws its skin proportionally
    #: as far as a shed does.
    band = max(SCATTER_BAND * (hw + hh) / 2.0, spread * 0.5, 1.0)
    #: How big a mound is drawn here — see `PILE_REF_M`.
    _lo, _hi = PILE_SCALE_CLAMP
    pile_scale = min(_hi, max(_lo, ((hw + hh) / 2.0) / PILE_REF_M))

    def _line(ang, f=1.0):
        """A point on the footprint's OUTLINE, jittered across it.

        SCATTER LINES THE BUILDING, it does not fill a disc around it. What
        leaves a collapse as loose pieces is the skin — the facade, the roof
        edge, the parapet — and it comes to rest against the wall it fell off.
        A centred Gaussian put as much of it in the middle of the plan as
        around the edge, which reads as a building that exploded rather than
        one that fell down.

        So the radius is the distance to the footprint edge at this angle, plus
        a band across it: negative lands the piece inside the outline, positive
        outside. `f` is the size term, so a small piece strays further from the
        line and a heavy one sits on it.
        """
        ca, sa = math.cos(ang), math.sin(ang)
        edge = min(hw / max(abs(ca), 1e-6), hh / max(abs(sa), 1e-6))
        r = max(0.0, edge + rng.gauss(0.0, band * f))
        return cx + ca * r, cy + sa * r

    def _gauss(ang, f=1.0, cat_f=1.0):
        """A point drawn about the centre, with the spread scaled by *f*.

        USED FOR MOUNDS. THE GAUSSIAN DESCRIBES VOLUME, NOT PIECES. Drawing every piece from the
        same normal already gives a Gaussian VOLUME density — each piece
        contributes its own volume at its own position, so the expected volume
        at a point is the pdf times the total. What it does not give is any
        relationship between how big a piece is and how far it went, and a
        collapse has a strong one: the heavy pieces stay where the building
        was and only the light stuff travels.

        So *f* narrows the draw for a big piece and widens it for a small one
        (see `_spread_factor`), which tilts size against radius while leaving
        the volume-weighted spread near where it was.
        """
        x = cx + rng.gauss(0.0, sx * f * cat_f)
        y = cy + rng.gauss(0.0, sy * f * cat_f)
        if bias is not None:
            x += math.cos(ang) * spread * 0.35
            y += math.sin(ang) * spread * 0.35
        return x, y

    out = []
    # NO DROP HEIGHT. The 0.4 m a piece used to start above the ground was
    # there to give the settle something to fall through, and it only ever
    # bought a bounce: a piece dropped from 0.4 m arrives with speed and
    # scatters off whatever it lands on. Seated ON the ground instead, the
    # settle has nothing to do but resolve the overlap — which is the part
    # that actually needs a solver, since the Gaussian places pieces without
    # any idea of what is already there.
    for cat, budget, lift in (("debris_pile", l_budget, 0.02),
                              ("debris", p_budget, 0.0)):
        groups = for_structure(pool.get(cat) or {}, kind)
        if not groups or budget <= 0.0:
            continue
        if cat == "debris_pile":
            groups = _fit_piles(groups, min(2.0 * hw, 2.0 * hh) * PILE_MAX_FRAC,
                                pile_scale)
            if not groups:
                continue
        # The piece this category calls average, so `_spread_factor` has
        # something to be relative to. Median, not mean: a pool with one huge
        # slab in it should not push everything else outward.
        _v = sorted(float(e.get("vol_m3") or 0.0)
                    for _w, entries in groups for e in entries
                    if float(e.get("vol_m3") or 0.0) > 0.0)
        v_ref = _v[len(_v) // 2] if _v else 1.0
        cat_f = (float((rules or {}).get("pile_spread_frac", PILE_SPREAD_FRAC))
                 if cat == "debris_pile" else 1.0)
        spent = 0.0
        # A cap on the draw, not on the amount: the budget is the amount. It
        # exists because a pool of very small pieces against a large budget is
        # thousands of prims, and the scene's ceiling is prim count, not
        # volume. See `mesh_damage.apply_to_stage`'s cell budget for the same
        # trade made one level up.
        for _ in range(int((rules or {}).get("max_per_building", 200))):
            if spent >= budget:
                break
            e = rng.choice(_pick(groups, rng.random()))
            sc = (rng.uniform(0.8, 1.2) * pile_scale if cat == "debris_pile"
                  else rng.uniform(0.7, 1.2))
            ang = rng.uniform(0.0, 2.0 * math.pi)
            if bias is not None and cat == "debris_pile" and rng.random() < 0.6:
                ang = math.atan2(bias[1], bias[0]) + rng.uniform(-0.6, 0.6)
            size_f = _spread_factor(e["vol_m3"] * sc ** 3, v_ref)
            # Mounds slump at the centre; scatter lines the outline.
            x, y = (_gauss(ang, size_f, cat_f) if cat == "debris_pile"
                    else _line(ang, size_f))
            # Charged to the budget BEFORE the veto: the building shed this
            # material either way, the veto only says it may not be authored
            # here. Skipping the charge would make a lot beside an exclusion
            # zone spill everything it has just inside the zone's edge.
            spent += e["vol_m3"] * sc ** 3
            if keep is not None and not keep(x, y):
                continue
            out.append({
                "usd": e["usd"], "x_m": x, "y_m": y,
                "z_m": e["fp"]["base"] + lift,
                "yaw_deg": rng.uniform(0.0, 360.0),
                "roll_deg": 90.0 if e["axis_up"] == "Y" else 0.0,
                "pitch_deg": 0.0, "scale": e["scale"] * sc,
                "category": cat, "axis_up": e["axis_up"],
                # PIECES ARE DROPPED, MOUNDS ARE NOT. A piece is placed by a
                # Gaussian that knows nothing about what is under it — the
                # ground, a mound, another wreck's rubble — so only physics can
                # actually seat it, and `lift` above is the drop height that
                # gives it somewhere to fall from. A mound is authored sitting
                # on the ground already; settling one only makes it jitter.
                #
                # Nothing stays dynamic afterwards: `settle.run(bake_result=
                # True)` disables every body once the pile is at rest, so the
                # debris is immovable in the finished scene either way.
                **({"settle": True} if cat == "debris" else {}),
            })
    return out


def radius_m(placements, centre) -> float:
    """How far the debris reaches from *centre*. 0 when there is none.

    Recorded on the building rather than on the debris because that is where a
    reader of the scene can still find it once the pieces are baked inside an
    archetype and are no longer placements of their own — see
    `targets.survey_from_placements`.
    """
    if not placements:
        return 0.0
    return max(math.hypot(p["x_m"] - centre[0], p["y_m"] - centre[1])
               for p in placements)


# ---------------------------------------------------------------------------
# authoring it — the part that needs a stage
# ---------------------------------------------------------------------------

def _resolver(config):
    """A `SizeResolver` for *config*, memoised on the config object.

    A fresh one per call would defeat `pools`, which caches on the resolver
    identity — so every damaged building would re-measure the whole debris
    library, and a bake asks once per cell.
    """
    got = getattr(_resolver, "_cache", None)
    if got is not None and got[0] is config:
        return got[1]
    import scene_generator as sg
    made = sg._make_resolver(config)
    _resolver._cache = (config, made)
    return made


def shed(stage, config, *, kind, rung, centre, footprint_m, height_m,
         parent_path, rng, resolver=None, bias=None, keep=None,
         scene_scale_factor=1.0):
    """Plan a building's debris and reference it onto *stage*.

    Returns ``{"placements", "paths", "statics", "radius_m"}``. `paths` are
    the pieces, which are DROPPED and need the caller's settle to seat them.
    `statics` are the mounds, authored already at rest: they still have to be
    made COLLIDERS during that settle or the pieces fall straight through them.
    """
    import scene_generator as sg

    dis = sg._stage(config, "disaster") or {}
    if resolver is None:
        resolver = _resolver(config)
    pls = plan(config, resolver, kind=kind, rung=rung, centre=centre,
               footprint_m=footprint_m, height_m=height_m,
               rules=dis.get("debris") or {}, rng=rng, bias=bias, keep=keep)
    if not pls:
        return {"placements": [], "paths": [], "statics": [], "radius_m": 0.0}

    sg.apply_placements(stage, pls, parent_path, scene_scale_factor,
                        resolver=resolver)
    return {"placements": pls,
            "paths": [p["prim_path"] for p in pls
                      if p.get("settle") and p.get("prim_path")],
            "statics": [p["prim_path"] for p in pls
                        if not p.get("settle") and p.get("prim_path")],
            "radius_m": radius_m(pls, centre)}
