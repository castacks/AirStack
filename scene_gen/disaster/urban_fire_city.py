"""urban_fire_city — set a GENERATED city on fire.

WHAT THIS IS, AND WHAT IT IS NOT
---------------------------------
`urban_fire.py` knows how ONE building burns. `urban_fire_spread.py` knows how
a fire gets from one building to the next. Neither of them knows where the
buildings ARE — and every urban-fire launcher so far answered that by rolling
its own packer (`urban_fire_city_launch_script`, `urban_fire_city250_...`),
which is exactly the mistake `generate-urban-city` exists to prevent: two
hand-rolled block packers, both worse than the generator, both hitting bugs
`apply_placements` had already solved.

So this module places NOTHING. It is a pass that runs AFTER
`generate_scene.generate_scene_on_stage`, over the placements it returned —
the same contract `disaster.quake.assemble` / `_mono_pass` keeps for the
earthquake. The city is the generator's; the fire is ours.

WHY EVERY BUILDING TAKES THE `burn_monolith` PATH
--------------------------------------------------
`urban_fire.burn_building` — the gutted interior, the burnt-through floors,
the partial collapse — needs a building assembled from FAÇADE MODULES, because
those recipes work by taking modules away and putting joist stubs and slabs
where they were. The city generator does not build one: `usds.buildings.intact`
is a pool of WHOLE assets, each a single mesh with a handful of `GeomSubset`s
(measured: GreatAmericanCity is 9-15 subsets on one mesh, no elements at all).
There is nothing there to take apart, so the vocabulary is the one
`burn_monolith` has — the asset's own materials sooted in place, plume tongues
rooted on the measured wall, flame out of the burning band, a charred roof
with debris on it, and glass and spalled render on the pavement under it.

That is also why F5 is rare here and should be: a burnt-out RC or steel frame
stands. `urban_fire_spread.level_for_age` only collapses `urm`, and `_btype`
below only calls a building `urm` when it is low enough to plausibly BE
load-bearing masonry over timber floors.

TUNE IT WITH THE DRY RUN, NOT WITH THE SIM
-------------------------------------------
How far the fire gets is set by the layout's GAP DISTRIBUTION, not by the
clock alone — conduction crosses <1.2 m in minutes, radiation 1-13 m in tens
of minutes, and a downwind brand is the only thing that crosses a street. So
the same `elapsed_min` reads as one burning building on one plat and half the
downtown on another. `tools/urban_fire_dryrun.py` solves the whole thing
host-side in a second against the real layout; use it before a launch and
again after any change to block sizing or `packing.building_gap_m`.
"""

import math
import os
import random

# A LEVEL IS A SHARE OF THE CITY, NOT A CLOCK READING.
#
# The first cut of this table set `elapsed_min` per rung, and it was wrong in a
# way that only a re-measure found. MEASURED on `downtown_gac` at 500 m, T+195
# min, one ignition, everything else held: igniting a building with ONE
# neighbour inside radiation reach involved 8 buildings; igniting one with FOUR
# involved 63. An eightfold swing, from which building the fire started in.
#
# The same table then moved under its own feet: an unrelated edit to
# `districts.py` / `city_layout.py` re-rolled the layout, the upwind-quarter
# ignition landed on a different building, and "moderate" went from 21% of the
# city to 6% with no change to the fire code at all.
#
# So the rung states the READ — what share of the plate is alight — and the
# clock is solved for it (`_time_for_fraction`). A layout change then moves the
# time, which is invisible, instead of the scene, which is not.
#
# MEASURED over three layout seeds of `downtown_gac` at 500 m
# (`tools/urban_fire_dryrun.py --wind 20,8`, 97-111 buildings, 15 blocks,
# nearest-neighbour gap median 2.0 m / p90 6.0 m):
#
#   rung       solved T+     involved      a typical ladder
#   light       83- 88 min   10-11%        F1=2  F2=1  F3=7  F4=1
#   moderate   130-149 min   22-23%        F1=3  F2=5  F3=8  F4=8
#   severe     173-237 min   40-40%        F1=5  F2=8  F3=20 F4=11 (F5=3 once)
#
# The clock moves by up to 64 min across those seeds and the READ does not,
# which is the whole point of stating the rung this way round. Moderate is the
# rung where the whole ladder is present at once and most of the city is still
# standing clean — a burnt-out core, a ring of fully-involved façades, an
# outer edge only just alight. That CONTRAST is the read; a milder level that
# only shrinks the damaged count is the same event over fewer buildings.
LADDER = {
    # a working fire: one building well alight and the ones it has reached
    "light":    {"involved_frac": 0.10, "wind": (20.0, 8.0)},
    # a fire district — the whole ladder at once, most of downtown clean
    "moderate": {"involved_frac": 0.22, "wind": (20.0, 8.0)},
    # a conflagration through several blocks
    "severe":   {"involved_frac": 0.40, "wind": (20.0, 8.0)},
}
# Numeric aliases, matching the dataset's level_1/2/3 axis.
LADDER["1"], LADDER["2"], LADDER["3"] = (LADDER["light"], LADDER["moderate"],
                                         LADDER["severe"])

# The solved time is clamped here. Below the floor nothing has spread at all;
# above the ceiling the fire has been burning for half a day, which is not a
# fire any more.
ELAPSED_MIN_RANGE = (30.0, 720.0)

# WIND SPEED SATURATES AT 8 m/s AND ABOVE IT DOES NOTHING.
# `urban_fire_spread._wind_factor` scales its downwind/upwind multiplier by
# `min(1.0, strength / 8.0)`, so 8, 10 and 12 m/s produce a BIT-IDENTICAL
# solve (verified over the ladder above). The ladder therefore states 8 —
# writing 12 would imply a fiercer fire that the model does not deliver.
WIND_SATURATION_MPS = 8.0

# WHERE A CONFLAGRATION STARTS. A fire in an isolated shed stays a fire in an
# isolated shed; the ones that take a block start in the block. So the origin
# is the best-connected building NEAR the upwind quarter-point rather than the
# nearest one to it — measured radiation degree at the quarter-point ranged
# from 1 to 5 over a 90 m search on the same plate, and that alone decided
# whether the scene read as a fire or as a smudge.
IGNITE_SEARCH_FRAC = 0.20       # x region, around the upwind quarter-point

# FLOW IS BUDGETED, BECAUSE THE CITY ALREADY SPENT THE GPU.
#
# MEASURED, 2026-08-29, RTX 5070 Ti (16 GB): the 500 m plate is 68,216 stage
# prims and its geometry + BLASes leave 704 MB free. Carrying the 250 m
# bench's settings across — `density_cell_size_m=0.10`, `max_blocks=32768`,
# nine emitters on each of 21 burning buildings, one of them a 302 m tower —
# asked Flow for a volume it could not have:
#
#   [carb.graphics-vulkan.plugin] Out of GPU memory allocating resource 'flow'
#   [rtx.flow.plugin] Failed to allocate 1x1x1 texture. Allocating 1x1x1
#                     fallback texture to avoid crash.
#
# It does not crash and it does not raise. It renders A CITY WITH NO SMOKE IN
# IT, which is indistinguishable from a fire pass that never ran — the scene
# came up, the banner said 24 buildings involved, and every capture was of an
# untouched downtown.
#
# Three things were wrong and all three are fixed here:
#   * THE CELL. 0.10 m is a single-building bench figure. Voxel count goes as
#     1/cell^3, so 0.40 m is ~64x cheaper over the same volume, and a plume
#     read at street distance does not need centimetre voxels.
#   * THE POOL. `max_blocks` is a ceiling Flow will grow into; on a card this
#     full it has to be small enough that the allocation succeeds.
#   * THE EMITTER COUNT. 21 x 9 = 189 volumetric sources over a 200 m
#     district. The budget below spends them on the WORST buildings and lets
#     the rest carry their fire in geometry — soot, plumes and debris cost
#     nothing per building and read perfectly well at distance.
FLOW_CELL_M = 0.40
FLOW_MAX_BLOCKS = 8192
FLOW_EMITTERS_PER_BUILDING = 4
FLOW_EMITTER_BUDGET = 48        # total, across the whole plate

# Load-bearing masonry with timber floors does not go above five or six
# storeys, and it is the only construction type in the ladder that a fire
# gutted and occasionally dropped. Above this a downtown building is a frame
# — it spalls, it does not fall.
URM_MAX_H_M = 18.0
# Names that are masonry whatever their height.
URM_HINTS = ("brownstone", "rowhouse")


def _bbox_dims(stage, prim, p):
    """(W, D, H) of a placed building in ITS OWN yaw frame.

    The same measurement `quake._mono_dims` makes, and for the same reason:
    the layout places at 0/90/180/270, where the world box is exact once the
    two horizontal extents are swapped for the quarter turns.

    BOTH bbox purposes. A `[default_]`-only cache silently declines to
    measure anything authored under `render`, which is the disagreement that
    had a bake report clean and an audit then find it floating.
    """
    from pxr import Usd, UsdGeom

    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    r = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    if r.IsEmpty():
        return None
    lo, hi = r.GetMin(), r.GetMax()
    sx, sy, sz = hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]
    yaw = float(p.get("yaw_deg", 0.0)) % 180.0
    if 45.0 < yaw < 135.0:
        sx, sy = sy, sx
    return float(sx), float(sy), float(sz)


def _btype(b):
    """`urm` or `rc` for one building record. See `URM_MAX_H_M`."""
    low = str(b.get("usd", "")).lower()
    if any(h in low for h in URM_HINTS):
        return "urm"
    return "urm" if b["H"] < URM_MAX_H_M else "rc"


def buildings(stage, placements, ssf=1.0, category="house", verbose=True):
    """The building records the spread solve needs, in placement order.

    Selected exactly as `quake._mono_pass` selects: `category == "house"` and
    a usd that is NOT a kit archetype. An archetype IS skipped rather than
    burnt as a monolith — its geometry is a bake of already-damaged modules
    and sooting it whole would put an earthquake's rubble under a fire's
    smoke.
    """
    from disaster.quake import style_of

    out, n_arch, n_bad = [], 0, 0
    for i, p in enumerate(placements):
        if p.get("category") != category:
            continue
        if style_of(p.get("usd"))[0]:
            n_arch += 1
            continue
        path = p.get("prim_path")
        prim = stage.GetPrimAtPath(path) if path else None
        if not prim or not prim.IsValid():
            n_bad += 1
            continue
        dims = _bbox_dims(stage, prim, p)
        if not dims:
            n_bad += 1
            continue
        W, D, H = (v / float(ssf) for v in dims)
        b = {"i": len(out), "pi": i, "prim": path, "usd": p["usd"],
             "x": float(p["x_m"]), "y": float(p["y_m"]),
             "yaw": float(p.get("yaw_deg", 0.0)),
             "W": W, "D": D, "H": H,
             "style": os.path.basename(str(p["usd"])).rsplit(".", 1)[0]}
        b["btype"] = _btype(b)
        out.append(b)
    if verbose:
        print("[uf_city] {0} building(s) on the plate ({1} urm, {2} rc); "
              "{3} kit archetype(s) skipped, {4} unmeasurable".format(
                  len(out), sum(1 for b in out if b["btype"] == "urm"),
                  sum(1 for b in out if b["btype"] == "rc"), n_arch, n_bad))
    return out


def ignition_index(bl, wind_dir, region_m, origin=None):
    """Which building started it: the BEST-CONNECTED one near the upwind
    quarter-point.

    UPWIND OF CENTRE BY A QUARTER OF THE PLATE, not on the edge and not in
    the middle. On the edge half the brands blow off the plate and the fire is
    a semicircle cut by the boundary; dead centre it runs out of downwind city
    before it has spread. A quarter of the way in, upwind, is the longest run
    that stays on the plate.

    THEN THE DENSEST CANDIDATE INSIDE THAT SEARCH, not the nearest — see
    `IGNITE_SEARCH_FRAC`. `origin=(x, y)` overrides the target point but not
    the density rule; the nearest building is still the fallback when the
    search finds nothing.
    """
    from disaster import urban_fire_spread as ufs

    if origin is not None:
        ox, oy = float(origin[0]), float(origin[1])
    else:
        ox = -math.cos(wind_dir) * float(region_m) * 0.25
        oy = -math.sin(wind_dir) * float(region_m) * 0.25
    d2 = [(b["x"] - ox) ** 2 + (b["y"] - oy) ** 2 for b in bl]
    r2 = (IGNITE_SEARCH_FRAC * float(region_m)) ** 2
    cand = [k for k in range(len(bl)) if d2[k] <= r2]
    if not cand:
        return min(range(len(bl)), key=lambda k: d2[k])
    deg = {k: sum(1 for j in range(len(bl))
                  if j != k and ufs.gap_m(bl[k], bl[j]) <= ufs.RAD_REACH_M)
           for k in cand}
    return max(cand, key=lambda k: (deg[k], -d2[k]))


def _time_for_fraction(bl, ign, frac, wind_dir, wmps, seed):
    """Minutes since ignition at which `frac` of the plate is alight.

    EXACT, AND ONE EXTRA SOLVE. Ignition times do not depend on the elapsed
    time — `solve` fixes them from the graph — so the count involved at T is
    just how many of them are at or before T. Sort them, take the k-th, and
    add a minute so that building is alight rather than exactly at zero.

    Returns `(minutes, achieved_fraction, reachable)`. `achieved` is below the
    target when the fire's connected component is smaller than the target
    share, which is a property of the LAYOUT, not of the clock — no elapsed
    time reaches a building nothing can carry fire to, and saying so is more
    use than silently returning the ceiling.
    """
    from disaster import urban_fire_spread as ufs

    # `b.get("btype") or _btype(b)` so a caller that built its records some
    # other way — `tools/urban_fire_dryrun.py` does — gets the same
    # classification the build will use rather than the model's `urm` default.
    probe = ufs.solve(bl, ign, 0.0, wind_dir=wind_dir, wind_mps=float(wmps),
                      rng=random.Random(seed + 991),
                      btype_of=lambda b: b.get("btype") or _btype(b))
    ts = sorted(p["t_ignite"] for p in probe if p["t_ignite"] is not None)
    lo, hi = ELAPSED_MIN_RANGE
    if not ts:
        return lo, 0.0, 0
    want = max(1, int(round(float(frac) * len(bl))))
    k = min(want, len(ts))
    mins = max(lo, min(hi, ts[k - 1] / 60.0 + 1.0))
    return mins, k / float(len(bl)), len(ts)


def _soot_roof_plant(stage, placements, b, k, uf):
    """Darken the rooftop plant the LAYOUT already put on this building.

    ADOPTED, NEVER RE-AUTHORED. `detail/gac_props.dress` owns the AC units,
    tanks, masts and stair bulkheads (user, 2026-08-29: "Keep it central with
    the urban layout generator"), and it publishes `roof_plant_of` keyed by
    the exact `(usd, x, y)` the placement carries — identity, not proximity,
    because on a real street the neighbour's tank is nearer than the far side
    of your own roof.

    Leaving them alone is not neutral: they are the pale objects standing on
    the one surface the plume goes straight up over, so on a nadir pass they
    would be the brightest thing on a black roof. That is the same failure as
    the white-roof one, one prim up.
    """
    from detail import gac_props

    try:
        fixed, plant = gac_props.roof_plant_of(placements, b["usd"],
                                               b["x"], b["y"])
    except Exception as exc:
        # NOT SILENT. This is cosmetic, so it must not take the city build
        # down with it — but a swallowed exception here reads exactly like
        # "this building had no roof plant", which is also the common case.
        print("[uf_city] roof plant for {0}: {1}".format(b["style"], exc))
        return 0
    n = 0
    seen = set()
    for path in list(fixed) + list(plant):
        n += uf._darken_asset(stage, path, k, rough_add=0.22, seen=seen)
    return n


def assemble(stage, config, placements, parent="/World/stage/generated",
             level="moderate", elapsed_min=None, wind=None, seed=21,
             origin=None, flow=True, flow_root=None, ssf=1.0,
             max_emitters=FLOW_EMITTERS_PER_BUILDING,
             flow_cell_m=FLOW_CELL_M, flow_max_blocks=FLOW_MAX_BLOCKS,
             flow_budget=FLOW_EMITTER_BUDGET, verbose=True):
    """Run a spreading structure fire over an already-generated city.

    Returns ``{buildings, involved, reachable, tally, records, notes,
    ignition, elapsed_min, wind}``. Authors no physics: like the quake city
    pass, everything here is geometry and materials, so the plate loads in
    seconds.

    `level` names a rung of `LADDER` — a SHARE OF THE CITY, from which the
    elapsed time is solved. Passing `elapsed_min` sets the clock directly
    instead and the share falls out of it, which is the right way round for
    "what does this plate look like an hour in" and the wrong one for "give me
    a moderate fire".
    """
    from disaster import urban_fire as uf
    from disaster import urban_fire_spread as ufs

    rung = LADDER.get(str(level).lower())
    if rung is None:
        raise ValueError("unknown fire level {0!r}; have {1}".format(
            level, ", ".join(sorted(LADDER))))
    wdeg, wmps = rung["wind"] if wind is None else wind
    wind_dir = math.radians(float(wdeg))
    region = max(float(v) for v in
                 ((config.get("layout") or {}).get("region_m") or [500.0]))

    bl = buildings(stage, placements, ssf=ssf, verbose=verbose)
    if not bl:
        print("[uf_city] no buildings to burn")
        return {"buildings": 0, "involved": 0, "reachable": 0, "tally": {},
                "records": [], "notes": [], "ignition": None,
                "elapsed_min": float(elapsed_min or 0.0),
                "wind": (float(wdeg), float(wmps))}

    ign = ignition_index(bl, wind_dir, region, origin)
    # THE CLOCK IS SOLVED FOR THE RUNG, unless the caller names one. See
    # `LADDER`: the same minute count is a different scene on a different
    # layout, so what is held fixed is the share of the city alight.
    if elapsed_min is not None:
        mins, want, reachable = float(elapsed_min), None, None
    else:
        mins, want, reachable = _time_for_fraction(
            bl, ign, rung["involved_frac"], wind_dir, wmps, seed)
        # WARN ONLY ON A REAL CAP. Comparing the achieved FRACTION against the
        # target trips on rounding alone — 40% of 36 buildings is 14, and
        # 14/36 is 38.9% — which cried wolf on a plate where every building
        # was reachable. The cap is a COUNT question: is the connected
        # component smaller than the number of buildings the rung asks for?
        if reachable < int(round(rung["involved_frac"] * len(bl))):
            print("[uf_city] {0} wants {1:.0%} of the plate but only {2} of "
                  "{3} building(s) are reachable from the ignition at all "
                  "({4:.0%}) — the LAYOUT caps this, not the clock".format(
                      level, rung["involved_frac"], reachable, len(bl),
                      reachable / float(len(bl))))
    plan = ufs.solve(bl, ign, mins * 60.0, wind_dir=wind_dir,
                     wind_mps=float(wmps), rng=random.Random(seed + 991),
                     btype_of=lambda b: b["btype"])
    tally = {}
    for p in plan:
        tally[p["level"]] = tally.get(p["level"], 0) + 1
    involved = len(bl) - tally.get("F0", 0)
    if verbose:
        print("[uf_city] ignition: {0} at ({1:.0f}, {2:.0f}), H {3:.0f} m, "
              "{4}; wind {5:.0f} deg @ {6:.0f} m/s; T+{7:.0f} min".format(
                  bl[ign]["style"], bl[ign]["x"], bl[ign]["y"], bl[ign]["H"],
                  bl[ign]["btype"], wdeg, wmps, mins))
        print("[uf_city] {0}: {1} of {2} building(s) involved ({3:.0%})  {4}"
              .format(level, involved, len(bl), involved / float(len(bl)),
                      "  ".join("{0}={1}".format(k, tally[k])
                                for k in ufs.LEVELS if k in tally)))

    if flow and flow_root is None:
        from disaster import fire as fx
        fx.setup_flow_stack(stage, density_cell_size_m=float(flow_cell_m),
                            max_blocks=int(flow_max_blocks),
                            scene_scale_factor=ssf)
        flow_root = fx.FLOW_ROOT
        if verbose:
            print("[uf_city] flow: {0:.2f} m cells, {1} block pool, {2} "
                  "emitter budget over {3} per building — see FLOW_CELL_M for "
                  "why these are not the bench's".format(
                      flow_cell_m, flow_max_blocks, flow_budget, max_emitters))

    from pxr import Sdf, UsdGeom
    scope = parent + "/urban_fire"
    UsdGeom.Scope.Define(stage, Sdf.Path(scope))
    mats = uf.materials(stage, scope)

    # WHICH BUILDINGS GET VOLUMETRIC FIRE. `ACTIVE` says F2/F3 carry flame and
    # F4 only smoulders, so the budget goes to the flaming ones first and,
    # within a rung, to the biggest radiant panel — a 40 m façade venting is
    # what a plume is FOR. Everything else still gets its soot, plumes and
    # debris, which are geometry and cost nothing.
    _rank = {"F3": 0, "F2": 1, "F4": 2, "F5": 3, "F1": 4}
    burning = [(b, p) for b, p in zip(bl, plan) if p["level"] != "F0"]
    emit_ok, spent = set(), 0
    for b, p in sorted(burning,
                       key=lambda q: (_rank.get(q[1]["level"], 9),
                                      -q[0]["H"] * max(q[0]["W"], q[0]["D"]))):
        if not uf.ACTIVE.get(p["level"]) or spent >= flow_budget:
            continue
        emit_ok.add(b["i"])
        spent += max_emitters
    if verbose and flow_root:
        print("[uf_city] flow emitters on {0} of {1} burning building(s); the "
              "rest carry their fire in geometry".format(len(emit_ok),
                                                         len(burning)))

    records, notes = [], []
    n_soot = 0
    for b, p in zip(bl, plan):
        lvl = p["level"]
        if lvl == "F0":
            continue
        tag = "b{0}".format(b["i"])
        bp = "{0}/{1}".format(scope, tag)
        UsdGeom.Scope.Define(stage, Sdf.Path(bp))
        brng = random.Random(seed + 101 * b["i"])
        nrng = _nrng(seed + 101 * b["i"])
        entry = p["entry_side"] or brng.choice(("S", "E", "N", "W"))
        dims = {"W": b["W"], "D": b["D"], "H": b["H"],
                "cx": 0.0, "cy": 0.0, "zmin": 0.0}
        res = uf.burn_monolith(
            stage, bp, b["usd"], b["x"], b["y"], b["yaw"], dims, lvl,
            brng, nrng, mats, tag,
            flow_root=(flow_root if b["i"] in emit_ok else None),
            entry_side=entry,
            origin_frac=p["origin_frac"], holder=b["prim"],
            btype=b["btype"], max_emitters=max_emitters)
        # a burning building's own roof plant is sooted with it
        heavy = {"F1": 0.62, "F2": 0.48, "F3": 0.34, "F4": 0.24,
                 "F5": 0.20}.get(lvl, 0.4)
        n_soot += _soot_roof_plant(stage, placements, b, heavy, uf)
        notes += res.get("notes", [])
        records.append({"i": b["i"], "style": b["style"], "usd": b["usd"],
                        "x": round(b["x"], 1), "y": round(b["y"], 1),
                        "yaw": round(b["yaw"], 1), "W": round(b["W"], 1),
                        "D": round(b["D"], 1), "H": round(b["H"], 1),
                        "btype": b["btype"], "level": lvl,
                        "prim": b["prim"], "entry_side": entry,
                        "origin_frac": round(p["origin_frac"], 2),
                        "how": p["how"],
                        "t_ignite_min": (None if p["t_ignite"] is None
                                         else round(p["t_ignite"] / 60.0, 1)),
                        "authored": len(res.get("authored", []))})
    if verbose:
        print("[uf_city] burnt {0} building(s); {1} rooftop-plant shader(s) "
              "sooted".format(len(records), n_soot))
        for ln in ufs.summarise(bl, plan, mins * 60.0)[:12]:
            print("[uf_city] " + ln)
    return {"buildings": len(bl), "involved": involved, "tally": tally,
            "reachable": reachable,
            "records": records, "notes": notes,
            "ignition": {"i": ign, "style": bl[ign]["style"],
                         "x": bl[ign]["x"], "y": bl[ign]["y"]},
            "elapsed_min": mins, "wind": (float(wdeg), float(wmps))}


def _nrng(seed):
    """`numpy.random.default_rng`, or a stub if numpy is absent.

    Only `fracture` actually consumes `nrng`, and nothing on the monolith
    path fractures — but `burn_monolith` puts it in the ctx, so it has to be
    something. Keeping the import local means `check()` runs on a bare host.
    """
    try:
        import numpy as np
    except ImportError:
        return random.Random(seed)
    return np.random.default_rng(seed)


def check(verbose=True):
    """Host-side: the ladder is well formed and the type rule is sane."""
    bad = []
    for name in ("light", "moderate", "severe", "1", "2", "3"):
        r = LADDER.get(name)
        if not r:
            bad.append("ladder has no {0}".format(name))
            continue
        if not (0.0 < r["involved_frac"] < 1.0):
            bad.append("{0}: involved_frac {1} out of range".format(
                name, r["involved_frac"]))
        if r["wind"][1] > WIND_SATURATION_MPS:
            bad.append("{0}: wind {1} m/s is above the {2} m/s saturation, "
                       "so it is indistinguishable from {2}".format(
                           name, r["wind"][1], WIND_SATURATION_MPS))
    order = [LADDER[k]["involved_frac"]
             for k in ("light", "moderate", "severe")]
    if order != sorted(order) or len(set(order)) != 3:
        bad.append("the ladder is not monotonic: {0}".format(order))
    if ELAPSED_MIN_RANGE[0] >= ELAPSED_MIN_RANGE[1]:
        bad.append("ELAPSED_MIN_RANGE is inverted")
    if _btype({"usd": "x/SM_Building_01.usd", "H": 55.0}) != "rc":
        bad.append("a 55 m building is not rc")
    if _btype({"usd": "x/SM_Building_02.usd", "H": 12.0}) != "urm":
        bad.append("a 12 m building is not urm")
    if _btype({"usd": "aec/brownstone/x.usd", "H": 40.0}) != "urm":
        bad.append("a brownstone is not urm")
    if verbose:
        print("[uf_city] check {0}".format("ok" if not bad else "FAILED"))
        for b in bad:
            print("  " + b)
    return bad


if __name__ == "__main__":
    raise SystemExit(1 if check() else 0)
