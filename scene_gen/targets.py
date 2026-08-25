"""targets — Stage C: put people in the finished scene, and say where they are.

SPEC.md's Stage C is "load the pre-baked scene, place targets (usually human
victims), noting their ground truth locations". This module is that step. It is
deliberately NOT part of Stage B: victims are not baked into the scene USD, so
one baked city can be re-rolled with a different `targets.seed` and searched
again — which is the whole point of having ground truth at all.

    survey(...)  ──▶  sample_targets(...)  ──▶  place(stage, ...)
    what is in            WHO is where           prims, labels, targets.json
    the scene             (pure Python)

`sample_targets` is pure: no USD, no stage, no Isaac. That is what lets
`tests/test_targets.py` assert the distribution on the host in milliseconds.
`place` is the thin half that writes it, and it writes through the SAME
`apply_placements` every other stage uses, so a victim is an ordinary placement
dict with a pose on it.

WHERE PEOPLE ACTUALLY ARE AFTER AN EARTHQUAKE
---------------------------------------------
The cohort mix is not invented. Five sources, and what each one decides:

  * 2023 Türkiye first-week casualty analysis (PMC10267724): of the casualties
    reaching emergency services, 36.8% were rescued FROM UNDER RUBBLE, 40.1%
    were injured WHILE TRYING TO ESCAPE, and a further 19.8% were recovered
    deceased from under rubble. That is the `inside_rubble` / `exit_ring` split.
  * FEMA Hazus Earthquake Model (Technical Manual 6.1) keeps INDOOR and OUTDOOR
    casualty rates as separate quantities driven by time-of-day occupancy. That
    is the `occupancy` knob: at night nearly everyone is indoors, at commute
    time the street is full.
  * USAR structural-collapse doctrine: survivable voids form beside heavy
    structural members, in room corners and stairwells, and survivors
    concentrate AT THE PERIMETER OF THE COLLAPSE ZONE, near exit routes. That is
    why `inside_rubble` samples the footprint edge-biased toward the street
    rather than uniformly (a uniform draw puts most victims in the deep middle
    of the pile, where in practice few are found alive).
  * Post-disaster open-space research (Great East Japan Earthquake; seismic
    resilience reviews): displaced residents converge spontaneously on parks,
    plazas and sports fields, driven by fear of aftershocks, and stay clear of
    facades. That is `open_space`, sampled in clusters with a clearance ring.
  * Aerial SAR person-detection datasets (SARD, C2A) pose their actors
    standing, sitting, walking, lying, and in "exhausted or injured" positions.
    That is the pose set — including `wave_up`, the arm-overhead signal, which
    is the pose an aerial searcher is most likely to catch.

THE COHORTS
-----------
Five, and none of them is earthquake-specific code: each is defined against
something every locale and every disaster has (a damaged building, its street
frontage, a road, open ground, a rubble pile). Earthquake ships non-zero
weights; every other type ships zeros and behaves exactly as it did before.

    inside_rubble   trapped in a collapsed building, in the rim band, sunk in
    in_vehicle      caught in a car, in a seat, visible through the glass
    exit_ring       struck escaping, just outside a damaged facade
    street          caught outdoors, on the road, where the field hit hardest
    open_space      survivors gathered in parks and plazas, in clusters
    rubble_edge     bystanders and rescuers standing at the pile's edge

A cohort that cannot find a site — `inside_rubble` in a scene with no collapsed
buildings — hands its share to the outdoor cohorts rather than special-casing
severity. So a severity sweep moves the MIX without moving the population: the
same N people are somewhere in every scene of the sweep, which is what makes
"how many did the search find" comparable across it.

DETERMINISM
-----------
Its own RNG stream, `targets.seed` if set, else `seed + 20255` — the per-stream
offset convention documented at `scene_generator.py:1395`. Stage C runs after
everything, so it cannot perturb the layout; the separate stream is what lets
the same city be populated N different ways.
"""

from __future__ import annotations

import json
import math
import os
import random

# Cohort names, in the order they are allotted. Order matters only for the
# largest-remainder allotment being reproducible.
COHORTS = ("inside_rubble", "in_vehicle", "exit_ring", "street", "open_space",
           "rubble_edge")

#: Where a cohort's share goes when it cannot be sited — `inside_rubble` in a
#: scene with no collapsed buildings, say. The people existed either way.
_OUTDOOR = ("street", "open_space")

#: Poses drawn per cohort, and how often. Names index `scene_generator._HUMAN_POSES`.
_POSES = {
    "inside_rubble": (("fetal", 0.4), ("crouch", 0.35), ("supine", 0.25)),
    "in_vehicle":    (("seated", 1.0),),
    "exit_ring":     (("prone", 0.4), ("supine", 0.35), ("wave_down", 0.25)),
    "street":        (("prone", 0.4), ("supine", 0.35), ("fetal", 0.25)),
    "open_space":    (("idle", 0.4), ("seated", 0.25), ("kneeling", 0.2),
                      ("wave_up", 0.15)),
    "rubble_edge":   (("idle", 0.6), ("kneeling", 0.4)),
}

#: Poses whose body is horizontal — the placer rolls them flat and lifts them
#: by half a body depth rather than standing them on their feet. `wave_down`
#: belongs here: it is a casualty signalling from the ground, and left off this
#: list it rendered as someone standing in the street with an arm out.
LYING = frozenset(("prone", "supine", "fetal", "wave_down"))

#: How visible each cohort is from the air. `occluded` means an RGB camera
#: cannot see them at all; they are in the ground truth so a run can be scored
#: on what was FINDABLE rather than on what existed.
_VISIBILITY = {
    "inside_rubble": (("occluded", 0.7), ("partial", 0.3)),
    # Glass is not a wall. Someone in a car is behind something from every
    # bearing and hidden by none of them.
    "in_vehicle":    (("partial", 1.0),),
    "exit_ring":     (("partial", 0.5), ("open", 0.5)),
    "street":        (("open", 0.85), ("partial", 0.15)),
    "open_space":    (("open", 1.0),),
    "rubble_edge":   (("open", 1.0),),
}

#: Occupancy multipliers on the cohort weights (Hazus's time-of-day split).
#: Renormalised after, so these are ratios rather than absolute shares.
OCCUPANCY = {
    "night":   {"inside_rubble": 2.0, "in_vehicle": 0.3, "exit_ring": 1.2,
                "street": 0.2, "open_space": 0.8, "rubble_edge": 0.5},
    "day":     {},
    # The rush hour is the one time of day a meaningful share of the population
    # is inside a car rather than a building.
    "commute": {"inside_rubble": 0.7, "in_vehicle": 2.5, "exit_ring": 0.9,
                "street": 1.6, "open_space": 1.1, "rubble_edge": 1.0},
}

#: Fallback when the config carries no `targets` block at all.
DEFAULTS = {
    "enabled": True,
    "count_per_km2": 60.0,
    "count_clamp": [4, 60],
    "seed": None,
    "occupancy": "day",
    "owns_humans": False,
    "cohorts": {},
    "bury_frac": [0.3, 0.8],
    # Metres in from a collapsed footprint's face that a trapped victim may be
    # sampled. Keep it at or under `findability.COVER_M` — see
    # `_s_inside_rubble`.
    "rubble_rim_m": 3.0,
    # Where a seat is, as a fraction of the vehicle's height. Sets the Z of an
    # `in_vehicle` victim directly — they are not settled onto anything, or a
    # downward probe would rest them on the roof of the car they are inside.
    "seat_frac": 0.45,
    "clearance_m": 8.0,
    "cluster_size": [3, 8],
    "cluster_radius_m": 6.0,
    "min_separation_m": 1.5,
    "exit_ring_m": 5.0,
    "street_band_m": 15.0,
    "damaged_from": "soft_storey",
    # Run `findability.check_on_stage` after placing, stamp each victim with
    # its verdict, and print the report. See `place`.
    "validate": True,
    "parent_path": "/World/targets",
    "semantic_class": "person",
}

#: Offset for Stage C's RNG stream. See the DETERMINISM note above.
SEED_OFFSET = 20255

#: How far above the ground `settle_on_surface` starts the downward probe for a
#: victim who is INSIDE a building. High enough to clear the rubble they are
#: lying in, low enough to miss a roof or a reverted fragment hanging in the
#: shell above them — see the comment in `settle_on_surface`.
INDOOR_PROBE_M = 4.0


def settings(config: dict) -> dict:
    """The `targets` block of *config*, over the defaults."""
    out = dict(DEFAULTS)
    out.update((config or {}).get("targets") or {})
    return out


def rng_for(config: dict) -> random.Random:
    """Stage C's RNG. `targets.seed` overrides, so one city can be re-rolled."""
    cfg = settings(config)
    seed = cfg.get("seed")
    if seed is None:
        seed = int((config or {}).get("seed", 0)) + SEED_OFFSET
    return random.Random(int(seed))


# ---------------------------------------------------------------------------
# Survey — what Stage C knows about the scene it is populating
# ---------------------------------------------------------------------------
# Two sources, one shape. Stage C has to work both from a scene generated in
# process (today's launch scripts, where the placement list is right there) and
# from nothing but a baked USD (entrypoint 2), where the only description of
# the scene is the `assetCategory` customData `generate_scene.stamp_asset_
# provenance` already writes on every prim. Everything downstream reads the
# survey, so neither source is privileged.

_BUILDING_CATS = ("building", "house")
#: Every category the disaster stage lays as wreckage. `debris_pile` is the
#: big one — the 10-15 m mounds — and leaving it off this tuple is why the
#: first debris-aware run still put a casualty inside one.
_RUBBLE_CATS = ("debris", "debris_pile", "debris_fragment", "rubble")
#: Things with a cabin someone can be inside. Glass makes them obstructions
#: rather than walls — see `findability.POROUS_CATS`, which agrees.
_VEHICLE_CATS = ("car",)
#: Past this much roll or pitch a vehicle is on its side or its roof, and its
#: seats are not a place to put anybody. `disaster.cars_toppled_fraction`
#: turns some of them over on every seed.
UPRIGHT_DEG = 30.0


def _level_index(level: str, ladder: list) -> int:
    try:
        return ladder.index(str(level))
    except ValueError:
        return 0


def survey_from_placements(placements, layout=None, resolver=None,
                           disaster_type: str = "") -> dict:
    """Survey a scene from the placement list `build_scene` returned."""
    from disaster import levels

    ladder = levels.level_names(disaster_type or "none")
    buildings = []
    for p in placements or []:
        if p.get("category") not in _BUILDING_CATS:
            continue
        w, h, z = _footprint(p, resolver)
        buildings.append({
            "x": float(p.get("x_m", 0.0)), "y": float(p.get("y_m", 0.0)),
            "w": w, "h": h, "z": z,
            "yaw": float(p.get("yaw_deg", 0.0)),
            "level": str(p.get("_damage_level", "pristine")),
            "level_i": _level_index(p.get("_damage_level", "pristine"), ladder),
            # THE LABEL IS NOT THE GEOMETRY — see `_damaged`. An archetype
            # reference is a pre-wrecked model, so it counts here; a live
            # fracture is only known from the stage, and `mark_cut_geometry`
            # fills those in.
            "cut": bool(p.get("_archetype")),
            "prim_path": p.get("prim_path", ""),
        })
    # Debris is surveyed for one reason: to keep people OUT of it. The piles a
    # severe quake lays down are 10-15 m across, and an outdoor victim sampled
    # without regard to them ends up inside one — labelled `open` in the ground
    # truth while being invisible from any angle. Found by looking: the first
    # scene run put a `street` casualty under a pile and the close-up camera
    # came back as a picture of the inside of the rubble.
    debris = []
    for p in placements or []:
        if p.get("category") not in _RUBBLE_CATS:
            continue
        w, h, z = _footprint(p, resolver, "debris")
        debris.append({"x": float(p.get("x_m", 0.0)),
                       "y": float(p.get("y_m", 0.0)),
                       "r": max(w, h) / 2.0, "z": z})
    vehicles = []
    for p in placements or []:
        if p.get("category") not in _VEHICLE_CATS:
            continue
        if (abs(float(p.get("roll_deg", 0.0))) > UPRIGHT_DEG
                or abs(float(p.get("pitch_deg", 0.0))) > UPRIGHT_DEG):
            continue
        w, h, z = _footprint(p, resolver, "car")
        vehicles.append({"x": float(p.get("x_m", 0.0)),
                         "y": float(p.get("y_m", 0.0)),
                         "w": w, "h": h, "z": z,
                         "yaw": float(p.get("yaw_deg", 0.0)),
                         "prim_path": p.get("prim_path", "")})
    lay = layout or {}
    return {
        "region": _wh(lay.get("region")) or _region_of(buildings),
        "debris": debris,
        "vehicles": vehicles,
        "buildings": buildings,
        "roads": [dict(r) if isinstance(r, dict) else _rect(r)
                  for r in (lay.get("road_corridors") or [])],
        "ladder": ladder,
    }


def survey_from_stage(stage, config: dict) -> dict:
    """Survey a scene from a loaded USD alone — entrypoint 2.

    Reads back the `assetCategory` customData every generated prim carries and
    measures each building's world bounds. There is no `_damage_level` on a
    prim, so the level is recovered the way Stage B computed it in the first
    place: the damage field at that position, quantised on the same ladder.
    """
    from pxr import Usd, UsdGeom

    from disaster import levels

    dis = (config or {}).get("disaster") or {}
    dtype = str(dis.get("type", "none"))
    ladder = levels.level_names(dtype)
    region = _wh((config.get("layout") or {}).get("region_m")) or (400.0, 400.0)
    field = field_of(config)
    sev = float(dis.get("severity", 1.0))

    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    buildings, debris, vehicles = [], [], []
    for prim in stage.Traverse():
        cat = prim.GetCustomDataByKey("assetCategory")
        if (cat not in _BUILDING_CATS and cat not in _RUBBLE_CATS
                and cat not in _VEHICLE_CATS):
            continue
        rng_box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        if rng_box.IsEmpty():
            continue
        lo, hi = rng_box.GetMin(), rng_box.GetMax()
        x, y = (lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0
        if cat in _RUBBLE_CATS:
            debris.append({"x": x, "y": y, "z": float(hi[2] - lo[2]),
                           "r": max(float(hi[0] - lo[0]),
                                    float(hi[1] - lo[1])) / 2.0})
            continue
        if cat in _VEHICLE_CATS:
            # World bounds are already rotated, so the box is axis-aligned and
            # a little larger than the car; a seat is well inside either way.
            vehicles.append({"x": x, "y": y, "yaw": 0.0,
                             "w": float(hi[0] - lo[0]),
                             "h": float(hi[1] - lo[1]),
                             "z": float(hi[2] - lo[2]),
                             "prim_path": str(prim.GetPath())})
            continue
        dmg = levels.local_damage(field(x, y), sev)
        src = str(prim.GetCustomDataByKey("sourceAsset") or "")
        buildings.append({
            "cut": bool(prim.GetChild(FRAGMENT_SCOPE).IsValid()
                        or "/archetypes/" in src),
            "x": x, "y": y,
            "w": float(hi[0] - lo[0]), "h": float(hi[1] - lo[1]),
            "z": float(hi[2] - lo[2]),
            # World bounds are already rotated, so the box is axis-aligned.
            "yaw": 0.0,
            "level": levels.level_at(dtype, dmg).name,
            "level_i": _level_index(levels.level_at(dtype, dmg).name, ladder),
            "prim_path": str(prim.GetPath()),
        })
    return {"region": region, "buildings": buildings, "roads": [],
            "debris": debris, "vehicles": vehicles, "ladder": ladder}


#: The scope `mesh_damage` authors a building's fragments under. Its presence
#: is the one honest answer to "was this building actually cut open".
FRAGMENT_SCOPE = "fragments"


def mark_cut_geometry(stage, survey: dict) -> int:
    """Set `cut` on every surveyed building the stage shows really wrecked.

    WHY THE DAMAGE LEVEL IS NOT ENOUGH. `disaster_stage` labels every building
    it *decides* to damage with a `_damage_level`, but `mesh_damage.apply_to_
    stage` only cuts the top `max_buildings` of them — everything past that
    budget keeps the TILT-AND-SINK STAND-IN: the intact model, rotated a few
    degrees and sunk a metre. It still says `partial_collapse` and it is still
    sixty metres tall.

    Siting a trapped victim by the label alone therefore puts people inside
    buildings that are collapsed on paper and intact in the stage — measured on
    `urban_quake_live`, where two of eight `inside_rubble` victims had the
    shell of a standing building 50-60 m over their heads, and worse under
    `SCENE_DAMAGE_BUDGET`, where most marked buildings are stand-ins. Credit to
    coasei-db for tracing the mechanism.

    So this asks the stage instead: a building with a `fragments` scope was
    genuinely cut. Archetype references are already marked by the survey.
    """
    found = 0
    for b in survey.get("buildings") or ():
        if b.get("cut"):
            continue
        path = b.get("prim_path")
        if not path:
            continue
        prim = stage.GetPrimAtPath(path)
        if prim and prim.IsValid() and prim.GetChild(FRAGMENT_SCOPE).IsValid():
            b["cut"] = True
            found += 1
    return found


def field_of(config: dict):
    """The damage field this scene was built with — the disaster's own."""
    from disaster import kinds

    dis = (config or {}).get("disaster") or {}
    region = tuple((config.get("layout") or {}).get("region_m") or (400.0, 400.0))
    fld = kinds.get(config).field(dis, region)
    return lambda x, y: float(fld(x, y))


def _wh(region) -> tuple:
    """(width, height) from either a ``(w, h)`` or a ``(x0, y0, x1, y1)``.

    `city_layout` records the region as BOUNDS while the config records it as
    an extent, and both reach here — a survey that read the bounds as an extent
    would sample targets over a quarter of the city.
    """
    if not region:
        return ()
    r = list(region)
    if len(r) == 4:
        return (abs(float(r[2]) - float(r[0])), abs(float(r[3]) - float(r[1])))
    return (float(r[0]), float(r[1]))


def _rect(r):
    x0, y0, x1, y1 = list(r)[:4]
    return {"x0": x0, "y0": y0, "x1": x1, "y1": y1}


def _region_of(buildings) -> tuple:
    if not buildings:
        return (400.0, 400.0)
    xs = [b["x"] for b in buildings]
    ys = [b["y"] for b in buildings]
    return (max(2.0, (max(xs) - min(xs)) * 1.2),
            max(2.0, (max(ys) - min(ys)) * 1.2))


def _footprint(p, resolver, kind: str = "house") -> tuple:
    """A placement's world extent, ``(w, h, height)``, measured where possible.

    The HEIGHT is what `findability` casts a nadir sightline against: a person
    under a twelve-metre building is not visible from a drone, and a model that
    stopped at the footprint would say they were.
    """
    if resolver is not None:
        try:
            from disaster.disaster_stage import placement_footprint
            fp = placement_footprint(resolver, p, kind)
            return float(fp["sx"]), float(fp["sy"]), float(fp["sz"])
        except Exception:
            pass
    return (12.0, 12.0, 12.0) if kind == "house" else (3.0, 3.0, 2.0)


# ---------------------------------------------------------------------------
# The sampler — pure Python, and the whole distribution model
# ---------------------------------------------------------------------------

def _weights(cfg: dict) -> dict:
    """Cohort weights: config, then the occupancy multipliers, renormalised."""
    raw = {c: float((cfg.get("cohorts") or {}).get(c, 0.0)) for c in COHORTS}
    mult = OCCUPANCY.get(str(cfg.get("occupancy", "day")), {})
    raw = {c: w * float(mult.get(c, 1.0)) for c, w in raw.items()}
    total = sum(raw.values())
    return {c: (w / total if total > 0 else 0.0) for c, w in raw.items()}


def _allot(n: int, weights: dict) -> dict:
    """Largest-remainder allotment of *n* people to cohorts. Deterministic."""
    exact = {c: n * weights.get(c, 0.0) for c in COHORTS}
    out = {c: int(math.floor(v)) for c, v in exact.items()}
    left = n - sum(out.values())
    order = sorted(COHORTS, key=lambda c: (-(exact[c] - out[c]), c))
    for c in order[:left]:
        out[c] += 1
    return out


def target_count(survey: dict, cfg: dict) -> int:
    """How many people are in this scene.

    NOT a function of severity: severity decides where people ended up, not how
    many of them there were. Holding N fixed across a sweep is what makes "how
    many did the search find" mean the same thing at every severity.
    """
    w, h = survey["region"]
    n = float(cfg.get("count_per_km2", 60.0)) * (w * h) / 1e6
    lo, hi = cfg.get("count_clamp", [4, 60])
    return int(max(int(lo), min(int(hi), round(n))))


def _pick(rng, table):
    """Weighted choice over a ``((name, weight), …)`` table."""
    total = sum(w for _, w in table)
    r = rng.random() * total
    for name, w in table:
        r -= w
        if r <= 0:
            return name
    return table[-1][0]


def _damaged(survey: dict, cfg: dict) -> list:
    """Buildings a person could actually be trapped in.

    TWO conditions, and the second is the one that bites. The rung says how bad
    the disaster decided this building's damage was; `cut` says whether any of
    that reached the geometry (`mark_cut_geometry`). A building past the damage
    budget carries the rung and keeps the intact model, and siting a victim in
    one writes a ghost into the ground truth — `occluded`, "trapped in a
    collapsed building", standing in an undamaged tower.

    A scene where nothing was really cut therefore has no trapped victims, and
    the cohort's share goes outdoors through the usual shortfall path. That is
    the right answer: if no building actually collapsed, nobody is under one.
    """
    ladder = survey.get("ladder") or []
    floor = _level_index(cfg.get("damaged_from", "soft_storey"), ladder)
    if floor <= 0:                     # unknown rung: anything past pristine
        floor = 1
    return [b for b in survey["buildings"]
            if b["level_i"] >= floor and b.get("cut")]


def _street_dir(b, survey) -> tuple:
    """Unit vector from a building toward the nearest road, or toward the
    region centre when no road list survived into the survey.

    The exit is on the street side. Sampling the whole ring uniformly puts as
    many victims behind the building as in front of it, and the back of a lot
    is not where people run out of a door.
    """
    best, bd = None, float("inf")
    for r in survey.get("roads") or []:
        cx = (r["x0"] + r["x1"]) / 2.0
        cy = (r["y0"] + r["y1"]) / 2.0
        d = (cx - b["x"]) ** 2 + (cy - b["y"]) ** 2
        if d < bd:
            best, bd = (cx - b["x"], cy - b["y"]), d
    if best is None:
        best = (-b["x"], -b["y"])
    n = math.hypot(*best) or 1.0
    return best[0] / n, best[1] / n


def to_local(b, x, y) -> tuple:
    """(x, y) in the building's own frame, so `w`/`h` mean what they say.

    THE FOOTPRINT IS NOT AXIS-ALIGNED. `w`/`h` come from
    `placement_footprint`, which measures the asset in ITS frame; the placement
    then yaws it. Treating the pair as a world AABB is wrong by the difference
    between the two whenever the footprint is not square, and at yaw 90° it
    swaps them outright — measured on `urban_quake_tiny` seed 3, where a 30x20
    building at yaw 90° let an `exit_ring` victim be sited INSIDE it, opaque
    from all seventeen search sightlines. Every footprint predicate goes
    through here.
    """
    a = math.radians(-float(b.get("yaw") or 0.0))
    dx, dy = x - b["x"], y - b["y"]
    c, sn = math.cos(a), math.sin(a)
    return dx * c - dy * sn, dx * sn + dy * c


def from_local(b, lx, ly) -> tuple:
    """The inverse of `to_local` — a point in the footprint frame, in world."""
    a = math.radians(float(b.get("yaw") or 0.0))
    c, sn = math.cos(a), math.sin(a)
    return b["x"] + lx * c - ly * sn, b["y"] + lx * sn + ly * c


def _inside(b, x, y, pad=0.0) -> bool:
    lx, ly = to_local(b, x, y)
    return abs(lx) <= b["w"] / 2.0 + pad and abs(ly) <= b["h"] / 2.0 + pad


def _inside_any(x, y, buildings, pad=0.0) -> bool:
    return any(_inside(b, x, y, pad) for b in buildings)


def _dist_to_buildings(x, y, buildings) -> float:
    best = float("inf")
    for b in buildings:
        lx, ly = to_local(b, x, y)
        dx = max(abs(lx) - b["w"] / 2.0, 0.0)
        dy = max(abs(ly) - b["h"] / 2.0, 0.0)
        best = min(best, math.hypot(dx, dy))
    return best


def edge_depth_m(b, x, y) -> float:
    """How far inside the footprint of *b* the point (x, y) is, in metres.

    Negative outside. This is the quantity `findability` charges a horizontal
    sightline for: the rubble it has to cross to reach someone in the pile.
    """
    lx, ly = to_local(b, x, y)
    return min(b["w"] / 2.0 - abs(lx), b["h"] / 2.0 - abs(ly))


def _face_toward(b, ux, uy) -> tuple:
    """The footprint face a searcher on the (ux, uy) side meets, in local axes.

    Returns a unit local normal — ``(±1, 0)`` or ``(0, ±1)``.
    """
    a = math.radians(-float(b.get("yaw") or 0.0))
    c, sn = math.cos(a), math.sin(a)
    lx, ly = ux * c - uy * sn, ux * sn + uy * c
    if abs(lx) >= abs(ly):
        return (1.0 if lx >= 0 else -1.0, 0.0)
    return (0.0, 1.0 if ly >= 0 else -1.0)


def _in_debris(x, y, survey) -> bool:
    """Is this spot inside a rubble pile? Anyone but the trapped stays out."""
    for d in survey.get("debris") or ():
        if (x - d["x"]) ** 2 + (y - d["y"]) ** 2 <= d["r"] ** 2:
            return True
    return False


def _in_roads(x, y, roads) -> bool:
    for r in roads:
        if r["x0"] <= x <= r["x1"] and r["y0"] <= y <= r["y1"]:
            return True
    return False


# -- one sampler per cohort. Each returns (x, y, extra) or None. -------------

def _s_inside_rubble(rng, survey, cfg, damaged):
    """Inside a collapsed footprint, in the RIM BAND behind one face.

    Two things want the same answer here and they are worth stating together.

    USAR doctrine says survivors are recovered from the PERIMETER of a collapse
    — beside the perimeter structure, in stairwells, near the exits — not from
    the deep middle of the pile. And `findability` says a sightline can only
    reach so far into rubble before the person at the end of it is buried
    rather than obstructed: `findability.COVER_M`.

    So the depth is sampled DIRECTLY, in metres in from one face, rather than
    as a fraction of the footprint. `rubble_rim_m` is that band, and it is the
    same 3 m the validator charges, which is what makes the cohort findable BY
    CONSTRUCTION instead of by luck. The predecessor's `t**0.35` fraction-based
    draw is what this replaces: on a 30 m footprint it put people 6-10 m deep,
    and `urban_quake_tiny` seed 1 failed the gate at 9.74 m of cover.
    """
    if not damaged:
        return None
    b = rng.choice(damaged)
    hx, hy = b["w"] / 2.0, b["h"] / 2.0
    nx, ny = _face_toward(b, *_street_dir(b, survey))
    rim = max(0.2, min(float(cfg.get("rubble_rim_m", 3.0)), min(hx, hy) * 0.9))
    d = rim * rng.random() ** 0.7          # shallow more often than deep
    lat = rng.uniform(-0.75, 0.75)         # along the face, clear of the corners
    if nx:
        lx, ly = nx * (hx - d), lat * hy
    else:
        lx, ly = lat * hx, ny * (hy - d)
    x, y = from_local(b, lx, ly)
    return x, y, {"building": b["prim_path"], "level": b["level"],
                  "rubble_depth_m": round(edge_depth_m(b, x, y), 3),
                  "bury_frac": rng.uniform(*cfg.get("bury_frac", [0.3, 0.8]))}


def _outside_face(rng, b, survey, out_m, lat=0.5):
    """A point *out_m* clear of one face of *b*, on the street side.

    In the footprint's OWN frame, which the previous version was not: it scaled
    the world street vector by each half-extent independently, so a diagonal
    street direction produced a point still inside the footprint whenever the
    offset was small. `_inside_any` then failed to reject it because it was not
    yaw-aware either, and the two bugs cancelled into a victim sealed inside a
    building — `urban_quake_tiny` seed 3, opaque from all 17 sightlines.
    """
    nx, ny = _face_toward(b, *_street_dir(b, survey))
    hx, hy = b["w"] / 2.0, b["h"] / 2.0
    t = rng.uniform(-lat, lat)
    if nx:
        lx, ly = nx * (hx + out_m), t * hy
    else:
        lx, ly = t * hx, ny * (hy + out_m)
    return from_local(b, lx, ly)


def _s_in_vehicle(rng, survey, cfg, damaged):
    """In a seat of an upright vehicle.

    The spec allows a victim inside a vehicle and calls them findable, and that
    is right for a reason worth writing down: a car is the one enclosure in the
    scene made mostly of GLASS. Someone in a driver's seat is obstructed from
    every bearing and hidden from none, which is exactly the `partial` case the
    search is supposed to be hard at — and unlike a building interior it needs
    no opening to be modelled, because the windows are the opening.

    No `_in_debris` exemption and no damage requirement: an undamaged street
    full of stopped traffic is where these people are.
    """
    vehicles = survey.get("vehicles") or []
    if not vehicles:
        return None
    v = rng.choice(vehicles)
    # A seat, not the centroid: front or back, left or right, in the car's own
    # frame. Two people in the same car would fail `min_separation_m` anyway,
    # so this is about not putting everyone on the gear stick.
    lx = rng.choice((-0.22, 0.18)) * v["w"]
    ly = rng.choice((-0.25, 0.25)) * v["h"]
    x, y = from_local(v, lx, ly)
    return x, y, {"vehicle": v["prim_path"],
                  "seat_z": round(float(cfg.get("seat_frac", 0.45))
                                  * float(v["z"]), 3)}


def _s_exit_ring(rng, survey, cfg, damaged):
    """Just outside a damaged facade, on the street side."""
    if not damaged:
        return None
    b = rng.choice(damaged)
    out = rng.uniform(0.5, float(cfg.get("exit_ring_m", 5.0)))
    x, y = _outside_face(rng, b, survey, out)
    if _inside_any(x, y, survey["buildings"], pad=0.5):
        return None
    return x, y, {"building": b["prim_path"], "level": b["level"]}


def _s_street(rng, survey, cfg, damaged, field=None):
    """On the road, accepted in proportion to how hard the field hit there —
    the rule the old `humans_strewn` pass used, kept because it is right."""
    roads = survey.get("roads") or []
    w, h = survey["region"]
    if roads:
        r = rng.choice(roads)
        x = rng.uniform(r["x0"], r["x1"])
        y = rng.uniform(r["y0"], r["y1"])
    else:
        # No road list (USD-only survey): the street is the band just outside
        # the footprints.
        x = rng.uniform(-w / 2.0, w / 2.0)
        y = rng.uniform(-h / 2.0, h / 2.0)
        d = _dist_to_buildings(x, y, survey["buildings"])
        if _inside_any(x, y, survey["buildings"]) or d > float(
                cfg.get("street_band_m", 15.0)):
            return None
    if field is not None and rng.random() >= field(x, y):
        return None
    return x, y, {}


def _s_open_space(rng, survey, cfg, damaged, field=None):
    """Open ground, clear of every facade — the park people fled to."""
    w, h = survey["region"]
    clear = float(cfg.get("clearance_m", 8.0))
    x = rng.uniform(-w / 2.0, w / 2.0)
    y = rng.uniform(-h / 2.0, h / 2.0)
    if _dist_to_buildings(x, y, survey["buildings"]) < clear:
        return None
    if _in_roads(x, y, survey.get("roads") or []):
        return None
    return x, y, {}


def _s_rubble_edge(rng, survey, cfg, damaged):
    """Standing at the edge of a pile, looking at it."""
    if not damaged:
        return None
    b = rng.choice(damaged)
    x, y = _outside_face(rng, b, survey, rng.uniform(3.0, 8.0))
    if _inside_any(x, y, survey["buildings"], pad=0.5):
        return None
    return x, y, {"building": b["prim_path"]}


def sample_targets(survey: dict, config: dict, rng=None) -> list:
    """WHO is where. Returns a list of victim dicts; no USD involved.

    Each victim: ``{id, cohort, visibility, pose, lying, x, y, yaw_deg,
    damage, …}``. `place` turns that into a prim; the ground-truth file is
    this list verbatim.
    """
    cfg = settings(config)
    if not cfg.get("enabled", True):
        return []
    rng = rng or rng_for(config)
    field = field_of(config)

    n = target_count(survey, cfg)
    weights = _weights(cfg)
    if not any(weights.values()) or n <= 0:
        return []
    want = _allot(n, weights)
    damaged = _damaged(survey, cfg)

    # The street draw is accepted in proportion to the local damage, which at
    # severity 0 is zero everywhere — so an undamaged control scene would
    # reject every street spot and lose that share of its population. Nothing
    # happened there, so nothing gates it.
    hit = field if float((config.get("disaster") or {}).get(
        "severity", 1.0)) > 0.0 else None
    samplers = {
        "inside_rubble": lambda: _s_inside_rubble(rng, survey, cfg, damaged),
        "in_vehicle":    lambda: _s_in_vehicle(rng, survey, cfg, damaged),
        "exit_ring":     lambda: _s_exit_ring(rng, survey, cfg, damaged),
        "street":        lambda: _s_street(rng, survey, cfg, damaged, hit),
        "open_space":    lambda: _s_open_space(rng, survey, cfg, damaged, hit),
        "rubble_edge":   lambda: _s_rubble_edge(rng, survey, cfg, damaged),
    }

    min_sep = float(cfg.get("min_separation_m", 1.5))
    out: list = []

    def _emit(cohort, spot) -> bool:
        x, y, extra = spot
        # Under a pile is where the TRAPPED are, and nobody else — see
        # `_in_debris`. Checked here rather than per sampler so a cohort added
        # later cannot forget it.
        if cohort != "inside_rubble" and _in_debris(x, y, survey):
            return False
        for v in out:
            if math.hypot(v["x"] - x, v["y"] - y) < min_sep:
                return False
        pose = _pick(rng, _POSES[cohort])
        v = {
            "id": len(out),
            "cohort": cohort,
            "visibility": _pick(rng, _VISIBILITY[cohort]),
            "pose": pose,
            "lying": pose in LYING,
            "x": round(float(x), 3),
            "y": round(float(y), 3),
            "yaw_deg": round(rng.uniform(0.0, 360.0), 1),
            "damage": round(field(x, y), 3),
        }
        v.update(extra)
        out.append(v)
        return True

    # `open_space` is drawn as clusters — families and neighbours gather in
    # groups, not as an even sprinkle — so its sampler seeds a centre and the
    # rest of the group lands around it.
    def _fill(cohort, count) -> int:
        placed = 0
        tries = 0
        clustered = cohort == "open_space"
        c_lo, c_hi = cfg.get("cluster_size", [3, 8])
        c_r = float(cfg.get("cluster_radius_m", 6.0))
        while placed < count and tries < count * 60 + 60:
            tries += 1
            spot = samplers[cohort]()
            if spot is None:
                continue
            if not _emit(cohort, spot):
                continue
            placed += 1
            if not clustered:
                continue
            for _ in range(rng.randint(int(c_lo), int(c_hi)) - 1):
                if placed >= count:
                    break
                a = rng.uniform(0.0, math.tau)
                d = c_r * math.sqrt(rng.random())
                mate = (spot[0] + math.cos(a) * d,
                        spot[1] + math.sin(a) * d, dict(spot[2]))
                if _dist_to_buildings(mate[0], mate[1],
                                      survey["buildings"]) < float(
                                          cfg.get("clearance_m", 8.0)):
                    continue
                if _emit(cohort, mate):
                    placed += 1
        return placed

    # First pass in cohort order, then hand whatever could not be sited to the
    # outdoor cohorts. A scene with no collapsed buildings simply has its
    # trapped share standing in the street instead — no severity special case.
    shortfall = 0
    for cohort in COHORTS:
        got = _fill(cohort, want[cohort])
        shortfall += want[cohort] - got
    if shortfall:
        live = [c for c in _OUTDOOR if weights.get(c, 0.0) > 0.0] or list(_OUTDOOR)
        share = _allot(shortfall, {c: 1.0 / len(live) for c in live})
        for cohort in live:
            _fill(cohort, share.get(cohort, 0))

    for i, v in enumerate(out):          # ids are dense after the fallbacks
        v["id"] = i
    return out


def summary(victims: list) -> dict:
    """Counts by cohort and by visibility — what the console line prints."""
    by_c: dict = {}
    by_v: dict = {}
    for v in victims:
        by_c[v["cohort"]] = by_c.get(v["cohort"], 0) + 1
        by_v[v["visibility"]] = by_v.get(v["visibility"], 0) + 1
    return {"total": len(victims), "by_cohort": by_c, "by_visibility": by_v}


# ---------------------------------------------------------------------------
# The USD half — placement, labels, ground truth
# ---------------------------------------------------------------------------

def _human_pool(config: dict):
    """(paths, scale_of, axis_up_of, yaw_offset_of) for the human assets."""
    import scene_generator as sg

    usds = (config or {}).get("usds") or {}
    scale = float(config.get("asset_scale", 1.0))
    root = str(config.get("asset_root", "") or "")
    paths, sc_ov, au_ov, yo_ov, _tags = sg._normalize_usd_list(
        usds.get("humans") or [], scale, root)
    return (paths,
            lambda p: sc_ov.get(p, scale),
            lambda p: au_ov.get(p, "Z"),
            lambda p: yo_ov.get(p, 0.0))


def to_placements(victims: list, config: dict, resolver, rng) -> list:
    """Turn victim dicts into the placement dicts `apply_placements` writes.

    Nothing new is invented here: a victim is an ordinary placement with a
    `pose` on it. Lying poses reuse the roll trick the Stage B casualty pass
    already used (`scene_generator._human_down`) — ±90° about the body's facing
    axis, lifted by half the body depth — and `inside_rubble` victims are then
    sunk by `bury_frac` of their height so the pile closes over them.

    THE Z HERE IS AN ESTIMATE. It is computed from the UNPOSED bbox, because
    that is all a stage-free function can see, and the pose is applied after
    the measurement: a crouch folds the legs and hangs the body in the air, a
    lying pose with an arm thrown out is a different thickness than the
    standing depth. `settle_on_surface` corrects every one of them against the
    real geometry once the prims exist. Measured before it existed: `crouch`
    floating 0.30 m, `wave_down` sunk 0.19 m into the road.
    """
    import scene_generator as sg

    paths, scale_of, axis_of, yaw_of = _human_pool(config)
    if not paths:
        return []
    out = []
    for v in victims:
        usd = rng.choice(paths)
        sc, au = scale_of(usd), axis_of(usd)
        fp = resolver.get(usd, "human", scale=sc, axis_up=au)
        axis_roll = 90.0 if au == "Y" else 0.0
        if v["lying"]:
            z = fp["sy"] / 2.0
            # Which way up. `_human_down` flipped a coin here; the pose names
            # already say which way the body landed, so the sign is read off
            # the pose instead. THE SIGNS ARE FROM THE VIEWPORT, not from the
            # geometry: the first guess had them backwards, and the capture
            # showed `supine` face-down in the dirt and `prone` staring at the
            # sky (targets_showcase_launch_script.py, POSES=all). `fetal` keeps
            # the coin flip — curled on one side or the other is equally true.
            sign = {"prone": 1.0, "supine": -1.0, "wave_down": -1.0}.get(
                v["pose"], rng.choice((-1.0, 1.0)))
            roll = axis_roll + sign * rng.uniform(82.0, 98.0)
        else:
            z = fp["base"]
            roll = axis_roll
        if v["cohort"] == "in_vehicle":
            # Straight to the seat. `settle_on_surface` skips this cohort, so
            # this is the final Z rather than an estimate: a downward probe
            # would find the roof of the car they are sitting in and rest them
            # on top of it.
            z = fp["base"] + float(v.get("seat_z", 0.6))
        if v["cohort"] == "inside_rubble":
            # Sink into the pile by a fraction of the body's VERTICAL extent,
            # which is its height standing and its depth lying — using the
            # standing height for a body already flat on the ground would bury
            # it several times over.
            z -= float(v.get("bury_frac", 0.5)) * (
                fp["sy"] if v["lying"] else fp["sz"])
        p = {
            "usd": usd, "x_m": v["x"], "y_m": v["y"], "z_m": z,
            # Per-asset yaw-offset, exactly as `build_city`'s `add` bakes it
            # in: the art is not authored facing +X.
            "yaw_deg": v["yaw_deg"] + yaw_of(usd),
            "roll_deg": roll, "pitch_deg": 0.0,
            "scale": sc, "category": "victim", "axis_up": au,
            "pose": v["pose"],
        }
        out.append(p)
        v["z"] = round(float(z), 3)
        v["asset"] = usd
    return out


def settle_on_surface(stage, victims: list, placements: list,
                      ssf: float = 1.0) -> int:
    """Rest every placed victim on whatever is actually under it. Returns a count.

    Two things the pure sampler cannot know, resolved here against the built
    stage:

    * **What the surface is.** A victim's Z came out of the sampler as 0, but
      the urban sidewalk is at 0.115 m and a rubble pile is a metre high. The
      PhysX downward raycast (`scene_generator._make_physx_ground_snap`) is the
      same one the generator uses for props; it needs the colliders to be on,
      which is why the launch scripts apply them BEFORE Stage C.
    * **How tall the posed body is.** The placement Z was estimated off the
      unposed bbox; this measures the composed, posed prim.

    `inside_rubble` victims are then sunk `bury_frac` of their own height BELOW
    that surface — the one case where breaking contact is the point.

    Absolute, not incremental: it sets the final Z from the measurement rather
    than nudging the estimate, so running it twice changes nothing.
    """
    from pxr import Gf, Usd, UsdGeom

    import scene_generator as sg

    # TWO PROBES, and the difference between them is a bug that was waiting to
    # happen. `_make_physx_ground_snap(h)` starts its ray at z = h and takes
    # the FIRST thing it hits going down, so the probe height decides what
    # counts as "the surface":
    #
    #   * An OUTDOOR victim has nothing over them by construction (the debris
    #     rejection in `_in_debris` guarantees it), so a high probe is safe and
    #     is what clears a rubble pile they are standing on.
    #   * An `inside_rubble` victim is INSIDE a footprint, and half the rungs
    #     that qualify (`cracked`, `soft_storey`, `partial_collapse`) are still
    #     standing. From 500 m the first hit is the ROOF, and the victim gets
    #     settled onto it — buried, correctly, under a slab twenty metres up.
    #     coasei-db reports a second source of the same shape: a fragment flung
    #     past `max_travel_m` is REVERTED to its authored pose, i.e. back inside
    #     the intact shell, leaving a ghost of the standing building at height
    #     over a collapsed footprint.
    #
    # So the trapped are probed from just above head height, which finds the
    # rubble or the floor they are actually in and cannot see a roof.
    ground_high = sg._make_physx_ground_snap()
    ground_low = sg._make_physx_ground_snap(max_height_m=INDOOR_PROBE_M)
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    scale = float(ssf) or 1.0
    moved = 0
    for v, p in zip(victims, placements):
        if v["cohort"] == "in_vehicle":
            continue                       # already in a seat — see `to_placements`
        prim = stage.GetPrimAtPath(p.get("prim_path", ""))
        if not prim or not prim.IsValid():
            continue
        box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        if box.IsEmpty():
            continue
        lo_m = float(box.GetMin()[2]) / scale
        height_m = (float(box.GetMax()[2]) - float(box.GetMin()[2])) / scale
        probe = ground_low if v["cohort"] == "inside_rubble" else ground_high
        surface = (probe(v["x"], v["y"]) if probe is not None else None)
        surface = 0.0 if surface is None else float(surface) / scale
        target = surface
        if v["cohort"] == "inside_rubble":
            # SINK INTO THE PILE, NOT THROUGH IT. `bury_frac` is a fraction of
            # the BODY, so on a shallow pile it can ask for more depth than the
            # pile has: a crouching victim is 1.7 m tall and a pancaked pile is
            # often under 2 m, so `bury_frac: 0.8` puts them under the terrain,
            # where no sightline reaches them from any bearing and the ground
            # truth claims a person inside the earth. Grade is the floor.
            target = max(surface - float(v.get("bury_frac", 0.5)) * height_m,
                         0.0)
        v["surface_z"] = round(surface, 3)
        shift = target - lo_m
        if abs(shift) < 1e-4:
            continue
        xf = UsdGeom.Xformable(prim)
        ops = [o for o in xf.GetOrderedXformOps()
               if o.GetOpType() == UsdGeom.XformOp.TypeTranslate]
        if not ops:
            continue
        t = ops[0].Get()
        ops[0].Set(Gf.Vec3d(t[0], t[1], t[2] + shift * scale))
        v["z"] = round(float(t[2]) / scale + shift, 3)
        moved += 1
    return moved


def _label(prim, cls: str, cohort: str) -> None:
    """Semantics for Replicator, best effort — as `fire.WildfireDriver._label`."""
    try:
        from isaacsim.core.utils.semantics import add_labels
        add_labels(prim, labels=[cls], instance_name="class")
        add_labels(prim, labels=[cohort], instance_name="victim")
    except Exception:
        try:
            from isaacsim.core.utils.semantics import add_update_semantics
            add_update_semantics(prim, cls)
        except Exception:
            pass


def write_ground_truth(victims: list, config: dict, path: str) -> str:
    """The ground-truth manifest. Returns the path written, or ""."""
    dis = (config or {}).get("disaster") or {}
    cfg = settings(config)
    doc = {
        "config": config.get("_name", ""),
        "locale": config.get("locale", ""),
        "seed": config.get("seed"),
        "target_seed": cfg.get("seed") if cfg.get("seed") is not None
                       else int(config.get("seed", 0)) + SEED_OFFSET,
        "disaster": {"type": dis.get("type"), "severity": dis.get("severity")},
        "occupancy": cfg.get("occupancy"),
        "summary": summary(victims),
        "victims": victims,
    }
    try:
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        with open(path, "w") as fh:
            json.dump(doc, fh, indent=2, sort_keys=False)
        return path
    except OSError as exc:
        print(f"[targets] could not write ground truth: {exc}")
        return ""


def default_gt_path(config: dict) -> str:
    """Beside the baked scene when there is one, else in `_plans/`."""
    here = os.path.dirname(os.path.abspath(__file__))
    try:
        import scene_cache
        cached = scene_cache.SceneCache().get(config)
        if cached:
            return os.path.join(os.path.dirname(cached), "targets.json")
    except Exception:
        pass
    name = str(config.get("_name") or
               (config.get("disaster") or {}).get("type") or "scene")
    return os.path.join(here, "_plans", f"{os.path.splitext(name)[0]}_targets.json")


def _layout_for(config: dict, resolver) -> dict:
    """The road/block plan, re-derived when the caller did not keep one.

    `generate_scene_on_stage` returns the placements but not the layout, and
    the road corridors are what put a street victim ON the street rather than
    in a band guessed from the facades. Re-running the layout stage is pure
    Python and cheap against a warm resolver — `generate_scene.write_run_plan`
    re-runs more than this for the same reason.
    """
    try:
        import generate_scene as gs
        _pl, lay, _c = gs.build_scene(config, resolver, stop_after="layout")
        return {"region": lay.get("region"),
                "road_corridors": lay.get("road_corridors") or []}
    except Exception as exc:                                   # noqa: BLE001
        print(f"[targets] layout unavailable ({type(exc).__name__}: {exc}); "
              f"street targets fall back to the facade band")
        return {"region": (config.get("layout") or {}).get("region_m")}


def place(stage, config: dict, placements=None, layout=None, resolver=None,
          parent_path: str = "", scene_scale_factor: float = 1.0,
          out_path: str = "") -> list:
    """Stage C, step 2. Populate a loaded scene and record where everyone is.

    *placements* / *layout* are the in-process scene description when there is
    one; without them the survey is read back off the stage. *parent_path*
    overrides `targets.parent_path` — a launch script that scaled its stage
    puts them under it, not beside it — and *scene_scale_factor* is the same
    stage-unit conversion `generate_scene_on_stage` takes. The ground truth is
    always in METRES regardless, because that is what a search run is scored
    in. Returns the victim list (also written to *out_path*).
    """
    import scene_generator as sg

    cfg = settings(config)
    if not cfg.get("enabled", True):
        return []
    if resolver is None:
        # Its own resolver, and so its own measurement cache. A caller that
        # already has one should pass it: a human asset measured twice is a
        # second Nucleus round trip.
        resolver = sg._make_resolver(config)

    dtype = str((config.get("disaster") or {}).get("type", "none"))
    if placements is not None:
        sv = survey_from_placements(placements, layout or _layout_for(
            config, resolver), resolver, dtype)
    else:
        sv = survey_from_stage(stage, config)

    # Which buildings the stage actually wrecked, before anyone is put in one.
    mark_cut_geometry(stage, sv)

    rng = rng_for(config)
    victims = sample_targets(sv, config, rng)
    if not victims:
        print("[targets] no targets placed (no cohort weights, or nowhere to "
              "put them)")
        return []

    parent = parent_path or str(cfg.get("parent_path", "/World/targets"))
    vps = to_placements(victims, config, resolver, rng)
    sg.apply_placements(stage, vps, parent_path=parent,
                        scene_scale_factor=scene_scale_factor,
                        resolver=resolver)

    settled = settle_on_surface(stage, victims, vps, scene_scale_factor)

    cls = str(cfg.get("semantic_class", "person"))
    for v, p in zip(victims, vps):
        path = p.get("prim_path")
        if not path:
            continue
        v["prim_path"] = path
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            continue
        # Ground truth on the prim as well as in the file, the way fire stamps
        # its per-emitter truth: selecting a victim in the viewport should
        # answer "who is this and could a camera have seen them".
        prim.SetCustomDataByKey("airstack:victim", {
            "id": int(v["id"]), "cohort": v["cohort"],
            "visibility": v["visibility"], "pose": v["pose"],
            "damage": float(v["damage"]),
        })
        _label(prim, cls, v["cohort"])

    if cfg.get("validate", True):
        import findability

        findability.check_placed(stage, victims, scene_scale_factor,
                                 str(config.get("_name") or ""))

    gt = write_ground_truth(victims, config, out_path or default_gt_path(config))
    s = summary(victims)
    print(f"[targets] {s['total']} target(s) under {parent}: "
          f"{s['by_cohort']} / visibility {s['by_visibility']} "
          f"({settled} settled onto the surface)"
          + (f" -> {gt}" if gt else ""))
    return victims
