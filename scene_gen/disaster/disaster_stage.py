"""
disaster_stage.py — the third stage: what the event does to a finished scene.

Runs after layout and detail, over the placement list, so by the time it is
called every prop already stands where the detail stage decided. That ordering
is the whole point: a locale and a seed fix the scene, and severity only
decides what happens to it.

WHY THIS EXISTS
---------------
`city_detail` places eighteen kinds of street furniture and **none of them
were affected by the disaster** — a downtown could be hit by a tornado with
every bench, bin and sign still standing to attention. The built-in frontage
passes did have topple handling, but the downtown locale switches those off in
favour of `city_detail`, so enabling the detailed generator silently made the
street furniture immune to the event.

WHY A RESPONSE TAXONOMY RATHER THAN MORE FRACTIONS
--------------------------------------------------
The existing config has one knob per prop kind — `trees_toppled_fraction`,
`streetlights_toppled_fraction`, `trash_cans_scatter_m`. Extending that to
eighteen categories means eighteen more knobs per disaster type, six disaster
types, and no way to reason about a new prop except by copying a number off a
similar one.

What actually decides how a prop responds is not what it is called but **how it
is fixed to the ground and how much wind it catches**:

    rooted        bolted to or buried in the ground, low profile. A hydrant, a
                  bollard, a parking meter. Survives almost anything; at high
                  intensity it shears off but does not travel.
    anchored_tall bolted down but tall and slender, so it catches a large
                  moment. Streetlights, signals, sign posts, utility poles.
                  Topples in place — the base stays where it was bolted.
    freestanding  heavy, resting on its own weight. Benches, bike racks, phone
                  booths, bus shelters, dumpsters. Tips at moderate intensity
                  and slides a little.
    loose         light enough for the wind to own. Bins, cones, café sets.
                  Tips early and travels — this is the class that scatters.
    vegetation    trees. Uproot or snap; the trunk stays put.
    vehicle       cars. Flip and, at high intensity, get thrown.
    person        casualties. Prone where they stood.

A category maps to a class, a class maps to (topple, scatter) behaviour scaled
by the local field intensity, and adding a prop means naming its class. Where
the config still carries a per-kind knob (`trees_toppled_fraction` and the five
others that predate this) it wins, so existing presets are unchanged.

DETERMINISM
-----------
Draws come from a stream derived from the seed but distinct from the layout
stream, for the reason documented in `scene_generator.build_city`: a
disaster-conditional draw taken from the layout stream shifts every subsequent
placement, and a severity sweep stops being comparable.
"""

import math
import random

from scene_generator import (_in_exclusion, _normalize_usd_list, _stage,
                             make_damage_field)

# category -> response class. Categories not listed are left alone, which is
# the right default: ground tiles, buildings, debris and trail surfaces are
# either structural or already the disaster's own output.
RESPONSE_CLASS = {
    # rooted — bolted or buried, low profile
    "fire_hydrant": "rooted", "bollard": "rooted", "parking_meter": "rooted",
    "mailbox": "rooted", "bike_lane_delineator": "rooted",
    # anchored_tall — bolted but slender, big wind moment
    "streetlight": "anchored_tall", "traffic_light": "anchored_tall",
    "utility_pole": "anchored_tall", "sign": "anchored_tall",
    # freestanding — heavy, resting on its own weight
    "bench": "freestanding", "bike_rack": "freestanding",
    "phone_booth": "freestanding", "bus_stop": "freestanding",
    "dumpster": "freestanding", "planter": "freestanding",
    "play_structure": "freestanding",
    # loose — the wind owns these
    "trash_can": "loose", "traffic_cone": "loose", "cafe_set": "loose",
    # the rest
    "tree": "vegetation", "street_tree": "vegetation", "plant": "vegetation",
    "car": "vehicle",
    "human": "person",
}

# Per class: (topple fraction, scatter metres) at field intensity 1.0.
# Both are maxima, scaled by the local field the same way every other disaster
# knob is. Calibrated so the ordering matches the classes' descriptions rather
# than to any one disaster.
CLASS_RESPONSE = {
    "rooted":        {"toppled_fraction": 0.10, "scatter_m": 0.0},
    "anchored_tall": {"toppled_fraction": 0.70, "scatter_m": 0.0},
    "freestanding":  {"toppled_fraction": 0.55, "scatter_m": 2.0},
    "loose":         {"toppled_fraction": 0.90, "scatter_m": 10.0},
    "vegetation":    {"toppled_fraction": 0.75, "scatter_m": 0.0},
    "vehicle":       {"toppled_fraction": 0.50, "scatter_m": 0.0},
    "person":        {"toppled_fraction": 0.50, "scatter_m": 0.0},
}

# Per-kind knobs that predate the taxonomy. Where one is present it wins, so
# tuned presets keep the behaviour they were tuned for.
LEGACY_TOPPLE_KEY = {
    "tree": "trees_toppled_fraction",
    "street_tree": "trees_toppled_fraction",
    "streetlight": "streetlights_toppled_fraction",
    "traffic_light": "traffic_lights_toppled_fraction",
    "trash_can": "trash_cans_toppled_fraction",
    "car": "cars_toppled_fraction",
    "human": "humans_prone_fraction",
}
LEGACY_SCATTER_KEY = {"trash_can": "trash_cans_scatter_m"}


def _response(category: str, dis: dict):
    """(toppled_fraction, scatter_m) for *category*, or None if unaffected."""
    cls = RESPONSE_CLASS.get(category)
    if cls is None:
        return None
    base = CLASS_RESPONSE.get(cls, {})
    topple = base.get("toppled_fraction", 0.0)
    scatter = base.get("scatter_m", 0.0)
    lk = LEGACY_TOPPLE_KEY.get(category)
    if lk is not None and lk in dis:
        topple = float(dis[lk])
    sk = LEGACY_SCATTER_KEY.get(category)
    if sk is not None and sk in dis:
        scatter = float(dis[sk])
    return topple, scatter


def apply_to_buildings(config: dict, layout: dict, placements: list,
                       resolver) -> dict:
    """Assign building fate, tilt the ruins, and drop their debris.

    Runs over the finished placement list — **after** `districts` has rezoned
    it. That ordering is the whole reason this lives here rather than inside
    `build_city`: districts drops every intact building and re-packs the block
    from its typology pools, and it treats a ruin as an immovable obstacle. So
    when fate was assigned during packing, districts received a half-ruined
    city, could not lay its terrace and midrise runs around the rubble, and
    demolished buildings it then could not rebuild — at severity 0.6 the
    detailed downtown lost 455 of its 919 buildings and the block count moved
    45 -> 43. Rezoning a pristine city and ruining it afterwards makes that
    class of bug unreachable.

    Mutates house placements in place (usd swap, roll/pitch/z) and appends
    debris. Returns a tally.
    """
    dis = _stage(config, "disaster")
    region = layout.get("region")
    if not dis or not region:
        return {}

    field = make_damage_field(dis.get("field"), tuple(region))
    if field.hi <= 0.0:
        return {}

    damaged_frac = float(dis.get("damaged_fraction", 0.0))
    destroyed_frac = float(dis.get("destroyed_fraction", 0.0))
    if damaged_frac <= 0.0 and destroyed_frac <= 0.0:
        return {}

    rng = random.Random(int(config.get("seed", 0)) + 5501)

    # Pools and per-asset conventions, resolved the same way build_city does.
    usds = config.get("usds") or {}
    bld = usds.get("buildings") or usds.get("houses") or {}
    scale = float(config.get("asset_scale", 1.0))
    root = str(config.get("asset_root", "") or "")

    # `_normalize_usd_list` returns (paths, scale_ovr, axisup_ovr, yaw_ovr,
    # tag_ovr); the per-asset overrides accumulate across every pool so a
    # lookup by path works regardless of which pool the asset came from.
    scale_of: dict = {}
    axis_of: dict = {}

    def _pool(entries):
        paths, sc_ovr, au_ovr, _yaw, _tags = _normalize_usd_list(
            entries or [], scale, root)
        scale_of.update(sc_ovr or {})
        axis_of.update(au_ovr or {})
        return paths

    damaged_l = _pool(bld.get("damaged"))
    destroyed_l = _pool(bld.get("destroyed"))
    destroyed_set = set(destroyed_l)

    debris_cfg = usds.get("debris") or {}
    pile_usds = _pool(debris_cfg.get("piles"))
    piece_usds = _pool(debris_cfg.get("pieces"))

    rules = dis.get("debris") or {}
    piles_per = rules.get("piles_per_building", [2, 4])
    pile_max_off = float(rules.get("pile_max_offset_m", 3.0))
    pieces_per = rules.get("pieces_per_building", [10, 20])
    pieces_scatter = float(rules.get("pieces_scatter_m", 6.0))
    tilt_chance = float(rules.get("tilt_chance", 0.35))
    tilt_deg = rules.get("tilt_deg", [2.0, 6.0])
    sink_m = rules.get("sink_m", [0.4, 1.2])
    lean_piles = rules.get("lean_piles", [2, 3])
    fp_tol = float(dis.get("ruin_footprint_tolerance", 0.25))
    exclusions = config.get("exclusions") or []
    # Mesh damage: deform the building's geometry instead of swapping the
    # asset. See the note at the fate branch for which fate prefers which.
    mesh_cfg = dis.get("mesh_damage") or {}

    def _sc(u):
        return float(scale_of.get(u, scale))

    def _au(u):
        return str(axis_of.get(u, "Z"))

    def _axis_roll(u):
        """+90 deg stands a Y-up-authored asset upright in the Z-up world.
        Mirrors the closure of the same name in `build_city`, which is not
        importable."""
        return 90.0 if _au(u) == "Y" else 0.0

    def _fp(u, cat):
        return resolver.get(u, cat, scale=_sc(u), axis_up=_au(u))

    def _hit(x, y, frac):
        return frac > 0.0 and rng.random() < frac * field(x, y)

    def _hit_count(x, y, rng_pair):
        k = field(x, y)
        if k <= 0.0:
            return 0
        return int(round(rng.randint(int(rng_pair[0]), int(rng_pair[1])) * k))

    def _fits(cand, base_fp):
        c = _fp(cand, "house")
        for a, b in ((c["sx"], base_fp["sx"]), (c["sy"], base_fp["sy"])):
            if b <= 1e-6 or abs(a - b) / b > fp_tol:
                return False
        return True

    tally: dict = {}
    new_placements: list = []

    for p in placements:
        if p.get("category") != "house":
            continue
        cx, cy = p["x_m"], p["y_m"]
        k = field(cx, cy)
        if k <= 0.0:
            continue
        d_here, m_here = destroyed_frac * k, damaged_frac * k
        r_fate = rng.random()
        if r_fate < d_here:
            pools, want_destroyed = (destroyed_l, damaged_l), True
        elif r_fate < d_here + m_here:
            pools, want_destroyed = (damaged_l, destroyed_l), False
        else:
            continue

        base_fp = _fp(p["usd"], "house")
        is_destroyed = False
        tilt_standin = True

        # WHICH MECHANISM, AND WHY THE FATE DECIDES IT.
        #
        # A *destroyed* building prefers an asset swap: a purpose-built ruin —
        # collapsed roof, exposed floors, rubble modelled in — reads far better
        # than any deformation of an intact model, and the destroyed pools are
        # the well-stocked ones. A merely *damaged* building prefers mesh
        # damage: "damaged but still standing" is exactly what the deformation
        # operators produce, and there is little authored art for it (five
        # entries against ten destroyed).
        #
        # Either way the swap is footprint-checked, so the layout never moves —
        # requirement 4d — and mesh damage is the fallback whenever no ruin
        # fits, which is also what happens when a pool is empty.
        prefer_mesh = (mesh_cfg.get("enabled", True)
                       and not want_destroyed
                       and mesh_cfg.get("prefer_for_damaged", True))
        if not prefer_mesh:
            for pool in pools:
                cands = [u for u in pool if _fits(u, base_fp)]
                if cands:
                    pick = rng.choice(cands)
                    p["usd"] = pick
                    p["scale"] = _sc(pick)
                    p["axis_up"] = _au(pick)
                    p["z_m"] = _fp(pick, "house")["base"]
                    is_destroyed = pick in destroyed_set
                    tilt_standin = False
                    break

        axis_roll = 90.0 if p.get("axis_up") == "Y" else 0.0
        if tilt_standin:
            # Nothing in either ruin pool fits this footprint. Mark it for
            # mesh damage, which runs once the prims are on the stage
            # (mesh_damage.apply_to_stage) and actually deforms the building.
            # The tilt-and-sink below is the fallback's fallback — it is what
            # the generator has always done, and on its own it reads as the
            # building being drunk rather than having failed.
            p["_mesh_damage"] = float(k)
            p["roll_deg"] = axis_roll + rng.uniform(-6.0, 6.0)
            p["pitch_deg"] = rng.uniform(-6.0, 6.0)
            p["z_m"] -= rng.uniform(0.1, 0.4)
        tally["damaged" if not is_destroyed else "destroyed"] = \
            tally.get("damaged" if not is_destroyed else "destroyed", 0) + 1

        if not is_destroyed:
            continue

        # A leaning ruin piles rubble against the side now overhanging.
        lean_dir = None
        if _hit(cx, cy, tilt_chance):
            t_roll = rng.choice([-1.0, 1.0]) * rng.uniform(
                float(tilt_deg[0]), float(tilt_deg[1]))
            t_pitch = rng.choice([-1.0, 1.0]) * rng.uniform(
                float(tilt_deg[0]), float(tilt_deg[1]))
            p["roll_deg"] = float(p.get("roll_deg", 0.0)) + t_roll
            p["pitch_deg"] = float(p.get("pitch_deg", 0.0)) + t_pitch
            p["z_m"] -= rng.uniform(float(sink_m[0]), float(sink_m[1]))
            lx_ = math.sin(math.radians(t_pitch))
            ly_ = -math.sin(math.radians(t_roll))
            yr_ = math.radians(float(p.get("yaw_deg", 0.0)))
            wx_ = lx_ * math.cos(yr_) - ly_ * math.sin(yr_)
            wy_ = lx_ * math.sin(yr_) + ly_ * math.cos(yr_)
            n_ = math.hypot(wx_, wy_)
            if n_ > 1e-9:
                lean_dir = (wx_ / n_, wy_ / n_)

        # Footprint half-extents, recorded by build_city when it packed the
        # slot — the ruin standing on it may be a different asset, but the
        # rubble belongs to the footprint, not to the model.
        bw, bh = p.get("_footprint_m") or (base_fp["sx"], base_fp["sy"])
        hw_, hh_ = bw / 2.0, bh / 2.0

        def _edge_r(ca, sa):
            return min(hw_ / max(abs(ca), 1e-6), hh_ / max(abs(sa), 1e-6))

        def _ring_pos(lo, hi, ang=None):
            if ang is None:
                ang = rng.uniform(0.0, 2.0 * math.pi)
            ca, sa = math.cos(ang), math.sin(ang)
            r_ = _edge_r(ca, sa) + rng.uniform(lo, hi)
            return cx + ca * r_, cy + sa * r_

        def _emit(u, x, y, z, cat, sc, settle=False):
            if exclusions and _in_exclusion(x, y, exclusions):
                return
            q = {"usd": u, "x_m": x, "y_m": y, "z_m": z,
                 "yaw_deg": rng.uniform(0.0, 360.0), "roll_deg": _axis_roll(u),
                 "pitch_deg": 0.0, "scale": sc, "category": cat,
                 "axis_up": _au(u)}
            if settle:
                q["settle"] = True
            new_placements.append(q)
            tally[cat] = tally.get(cat, 0) + 1

        if pile_usds:
            for _ in range(_hit_count(cx, cy, piles_per)):
                du = rng.choice(pile_usds)
                x_, y_ = _ring_pos(-1.0, pile_max_off)
                _emit(du, x_, y_, _fp(du, "debris_pile")["base"] + 0.02,
                      "debris_pile", _sc(du) * rng.uniform(0.8, 1.2))
            if lean_dir is not None:
                base_ang = math.atan2(lean_dir[1], lean_dir[0])
                for _ in range(rng.randint(int(lean_piles[0]),
                                           int(lean_piles[1]))):
                    du = rng.choice(pile_usds)
                    x_, y_ = _ring_pos(-1.5, 1.0,
                                       ang=base_ang + rng.uniform(-0.5, 0.5))
                    _emit(du, x_, y_, _fp(du, "debris_pile")["base"] + 0.02,
                          "debris_pile", _sc(du) * rng.uniform(0.8, 1.2))

        if piece_usds:
            for _ in range(_hit_count(cx, cy, pieces_per)):
                du = rng.choice(piece_usds)
                x_, y_ = _ring_pos(0.3, pieces_scatter)
                _emit(du, x_, y_, _fp(du, "debris")["base"] + 0.4,
                      "debris", _sc(du) * rng.uniform(0.7, 1.2), settle=True)

    placements.extend(new_placements)
    if tally:
        print("[disaster] buildings  "
              + "  ".join(f"{k}={v}" for k, v in sorted(tally.items())))
    return tally


def apply(config: dict, layout: dict, placements: list,
          only_categories=None) -> dict:
    """Apply disaster effects to *placements*, in place. Returns a tally.

    *only_categories* restricts the pass to a set of categories — used to run
    it over the `city_detail` props alone, since `build_city` already handles
    the ones it places itself. Once the damage logic is lifted out of
    `build_city` this argument goes away and the pass owns all of them.
    """
    dis = _stage(config, "disaster")
    if not dis:
        return {}

    region = layout.get("region")
    if not region:
        return {}
    x0, y0, x1, y1 = region
    field = make_damage_field(dis.get("field"), (x0, y0, x1, y1))
    if field.hi <= 0.0:
        return {}                      # `none` — nothing to do

    rng = random.Random(int(config.get("seed", 0)) + 90210)
    tally: dict = {}

    for p in placements:
        cat = p.get("category")
        if only_categories is not None and cat not in only_categories:
            continue
        resp = _response(cat, dis)
        if resp is None:
            continue
        topple_frac, scatter_m = resp
        k = field(p["x_m"], p["y_m"])
        if k <= 0.0:
            continue

        if topple_frac > 0.0 and rng.random() < topple_frac * k:
            # Knocked flat: ~90 degrees about the base, random heading. The
            # anchor stays where it was — a felled lamp post lies from its own
            # base — so this does not move the prop.
            p["roll_deg"] = float(p.get("roll_deg", 0.0)) + \
                rng.choice([-1.0, 1.0]) * rng.uniform(78.0, 102.0)
            p["pitch_deg"] = float(p.get("pitch_deg", 0.0)) + \
                rng.uniform(-8.0, 8.0)
            p["settle"] = True          # let PhysX find the resting pose
            tally[cat] = tally.get(cat, 0) + 1

            # Only what the wind owns actually travels.
            if scatter_m > 0.0:
                ang = rng.uniform(0.0, 2.0 * math.pi)
                d = rng.uniform(0.0, scatter_m * k)
                p["x_m"] += d * math.cos(ang)
                p["y_m"] += d * math.sin(ang)

    if tally:
        n = sum(tally.values())
        top = sorted(tally.items(), key=lambda kv: -kv[1])[:6]
        print(f"[disaster] {n} props hit  "
              + "  ".join(f"{c}={v}" for c, v in top))
    return tally
