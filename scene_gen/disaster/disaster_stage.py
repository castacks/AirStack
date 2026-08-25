"""
disaster_stage.py — the third stage: what the event does to a finished scene.

Runs after layout and detail, over the placement list, so by the time it is
called every prop already stands where the detail stage decided. That ordering
is the whole point: a locale and a seed fix the scene, and severity only
decides what happens to it.

WHY THIS EXISTS
---------------
`city_detail` places eighteen kinds of street furniture and **none of them
were affected by the disaster** — an urban scene could be hit by a tornado with
every bench, bin and sign still standing to attention. The built-in frontage
passes did have topple handling, but the urban locale switches those off in
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
import os
import random

from scene_generator import (_in_exclusion, _normalize_usd_list,
                             placement_footprint, _stage)

from disaster import kinds, levels

#: Loaded archetype manifests, by disaster. Reading one is a JSON parse, but
#: `apply_to_buildings` is called per scene and a preset sweep calls it dozens
#: of times; caching also means the "no library" message prints once.
_ARCH_CACHE: dict = {}


def _archetype_url(path: str) -> str:
    """An absolute archetype path as the generator's asset resolver wants it.

    `airstack://` when the library is inside the repo (portable across
    checkouts, which the tests' path-anchoring depends on), the raw absolute
    path when it is not — a library baked to a scratch directory or mounted
    from elsewhere is still perfectly loadable, just not relocatable.
    """
    repo = os.path.dirname(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    p = os.path.abspath(path)
    if p.startswith(repo + os.sep):
        return "airstack://" + os.path.relpath(p, repo).replace(os.sep, "/")
    return p


def _archetypes_wanted(dis: dict) -> bool:
    """Whether Stage A's library may be used at all.

    Off means every damaged building is wrecked LIVE, by `mesh_damage`, at
    scene time. That is the slow path by design — it is also the only one that
    shows the current damage code, since an archetype is whatever the library
    was baked from. Turn it off to look at a pipeline change without a re-bake:

        disaster: {archetypes: false}    # in a preset's `overrides:`
        SCENE_ARCHETYPES=0               # same thing, without editing a config

    The env var wins, so a baked preset can be switched to live for one run.
    """
    env = os.environ.get("SCENE_ARCHETYPES")
    if env is not None and env.strip() != "":
        want = env.strip().lower() not in ("0", "false", "no", "off")
    else:
        want = bool((dis or {}).get("archetypes", True))
    if not want:
        print("[disaster_stage] archetype library disabled — damaging every "
              "building live (mesh_damage)")
    return want


def _archetypes(disaster_type: str):
    """Stage A's library for *disaster_type*, or None if it was never baked.

    RETURNING None IS A SUPPORTED STATE, not an error. A fresh checkout has no
    library — it is gitignored and takes an Isaac Sim session to produce — and
    a scene must still build without one, falling back to the authored-ruin
    swap and live mesh damage exactly as it did before Stage A existed. The
    difference a baked library makes is that EVERY building can be damaged
    rather than the 50-80 `mesh_damage.fracture.max_buildings` can afford.
    """
    key = str(disaster_type or "none").lower()
    if key in _ARCH_CACHE:
        return _ARCH_CACHE[key]

    from archetypes import library as _lib

    scene_gen = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    path = os.path.join(_lib.disaster_dir(scene_gen, key), _lib.MANIFEST_NAME)
    doc = _lib.read_manifest(path)
    got = _lib.Library(path, doc) if doc.get("archetypes") else None
    if got is not None:
        print(f"[disaster_stage] archetype library: {len(got)} baked for "
              f"'{key}' ({len(got.types())} types)")
    elif key != "none":
        print(f"[disaster_stage] no archetype library for '{key}' — falling "
              f"back to live damage. Bake one with "
              f"ISAAC_SIM_SCRIPT_NAME=bake_archetypes_launch_script.py")
    _ARCH_CACHE[key] = got
    return got

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
    detailed urban lost 455 of its 919 buildings and the block count moved
    45 -> 43. Rezoning a pristine city and ruining it afterwards makes that
    class of bug unreachable.

    Mutates house placements in place (usd swap, roll/pitch/z) and appends
    debris around **every** building the disaster touched, scaled by how ruined
    it ended up — see the `debris_scale` note at the emission block for why
    that used to be only the asset-swapped ruins. Returns a tally.
    """
    dis = _stage(config, "disaster")
    region = layout.get("region")
    if not dis or not region:
        return {}

    # THE DISASTER'S OWN FIELD. `Disaster.field` still honours the compiled
    # `field.kind`, so this is the same shape it always was — but "where did
    # this event hit" is now asked of the event rather than assembled from its
    # config at three separate call sites.
    field = kinds.get(dis.get("type")).field(dis, tuple(region))
    if field.hi <= 0.0:
        return {}

    damaged_frac = float(dis.get("damaged_fraction", 0.0))
    destroyed_frac = float(dis.get("destroyed_fraction", 0.0))
    if damaged_frac <= 0.0 and destroyed_frac <= 0.0:
        return {}

    rng = random.Random(int(config.get("seed", 0)) + 5501)

    # HOW HARD a building is hit, as opposed to WHETHER it is hit. The field is
    # a spatial shape; severity is the magnitude; `levels.local_damage` is the
    # one place their product is written down. See `disaster/levels.py` for why
    # severity must not be folded into the field instead.
    #
    # Hand-written low-level configs may not carry a severity, and 1.0 is the
    # right default for them — it is what they already got.
    sev = float(dis.get("severity", 1.0))
    dtype = str(dis.get("type", "none"))

    # STAGE A's LIBRARY, if it has been baked. Preferred over everything below:
    # an archetype is THIS building at THIS level, already fractured and
    # settled, so referencing one costs nothing at scene time and every
    # building can have it. The authored-ruin swap and live mesh damage remain
    # as fallbacks for an unbaked library — see `_archetypes`.
    arch = _archetypes(dtype) if _archetypes_wanted(dis) else None
    arch_ladder = levels.level_names(dtype)

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
    # How much rubble a building that is DAMAGED but still standing sheds,
    # relative to one that was destroyed. See the emission block below for why
    # this is not zero — which is what it effectively was.
    damaged_debris = float(rules.get("damaged_debris_scale", 0.45))
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
        """Footprint of an asset from one of THIS pass's pools.

        `_sc`/`_au` only know the pools `_pool` normalised — damaged, destroyed,
        piles, pieces. Anything already on the plan goes through
        `placement_footprint` instead, which reads the placement.
        """
        return resolver.get(u, cat, scale=_sc(u), axis_up=_au(u))

    def _hit(x, y, frac):
        return frac > 0.0 and rng.random() < frac * field(x, y)

    def _hit_count(x, y, rng_pair, scale=1.0):
        k = field(x, y) * scale
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
    dropped: dict = {}
    new_placements: list = []
    region = layout.get("region")
    rx0, ry0, rx1, ry1 = ([float(v) for v in region] if region
                          else (0.0, 0.0, 0.0, 0.0))

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

        base_fp = placement_footprint(resolver, p, "house")
        is_destroyed = False
        tilt_standin = True

        # Local field x how bad the event was, and the rung it lands on.
        # Computed here rather than in the fallback branch because the
        # archetype swap needs it too, and the two must never disagree about
        # which level a building is at.
        p["_mesh_damage"] = levels.local_damage(k, sev)
        level = levels.level_at(dtype, p["_mesh_damage"]).name
        p["_damage_level"] = level

        # --- Stage B, step 5: reference the pre-baked archetype -------------
        if arch is not None and level != "pristine":
            rec = arch.resolve(p.get("usd", ""), level, arch_ladder)
            if rec:
                p["usd"] = _archetype_url(arch.usd_path(rec))
                # The archetype was exported with its world transform baked and
                # re-centred to the origin, so it needs no scale, no axis
                # correction and no base offset — it IS metres, Z-up, sitting
                # on the ground. Carrying the source asset's scale here is how
                # a 0.01-scaled Nucleus building would come back 100x too small.
                p["scale"] = 1.0
                p["axis_up"] = "Z"
                p["z_m"] = 0.0
                # NOT INSTANCED YET, and that is a known gap against SPEC's
                # "repeated archetypes are instanced so identical references
                # share geometry."
                #
                # `apply_placements` deliberately does not instance (see its
                # docstring, and 8187043e): `scene_prep.add_colliders` walks
                # `GetChildren()` to apply `UsdPhysics.CollisionAPI`, and an
                # instanceable prim has no traversable children — so an
                # instanced building gets NO COLLIDER and the drone flies
                # through it. Marking these instanceable here would trade a
                # memory win for a silently broken sim.
                #
                # The unblock is to author colliders INSIDE the archetype at
                # bake time: once the collider ships as part of the referenced
                # asset, instancing is free and correct. That belongs in
                # `archetypes/bake.py`'s export, not here.
                p["_archetype"] = True
                p.pop("_mesh_damage", None)
                is_destroyed = level == arch_ladder[-1]
                tally["archetype"] = tally.get("archetype", 0) + 1
                tilt_standin = False

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
        if tilt_standin and not prefer_mesh:
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
                    # An authored ruin already looks ruined; deforming it again
                    # is the "ruin's ruin" that Stage A's plan excludes too.
                    p.pop("_mesh_damage", None)
                    break

        axis_roll = 90.0 if p.get("axis_up") == "Y" else 0.0
        if tilt_standin:
            # Nothing in either ruin pool fits this footprint. Mark it for
            # mesh damage, which runs once the prims are on the stage
            # (mesh_damage.apply_to_stage) and actually deforms the building.
            # The tilt-and-sink is the fallback's fallback — it is what the
            # generator has always done, and on its own it reads as the
            # building being drunk rather than having failed.
            #
            # DRAWN HERE, APPLIED AFTER THE LOOP, and only to the buildings
            # the mesh-damage budget will not reach. A building that IS going
            # to be cut must keep its true pose: pitched 6 degrees about its
            # corner and sunk, an 80 m tower had one end authored metres
            # underground, and every fragment cut there "fell through the
            # ground" before the settle began. Drawing unconditionally keeps
            # the RNG stream where it was.
            p["_standin"] = (axis_roll + rng.uniform(-6.0, 6.0),
                             rng.uniform(-6.0, 6.0),
                             rng.uniform(0.1, 0.4))
        tally["damaged" if not is_destroyed else "destroyed"] = \
            tally.get("damaged" if not is_destroyed else "destroyed", 0) + 1

        # HOW MUCH RUBBLE THIS BUILDING SHED.
        #
        # Debris used to be emitted only under `if is_destroyed`, and
        # `is_destroyed` is true only when a purpose-built ruin asset was
        # swapped in. So every other outcome — a building the mesh-damage pass
        # pancaked and shattered, a damaged-pool swap, a destroyed building
        # whose footprint no ruin fitted — got a wrecked structure standing on
        # a spotlessly clean lot. That is the single most obviously wrong thing
        # in an aerial view of a damaged block, and it is also what the search
        # algorithms fly over: debris is occupancy, so its absence changed what
        # the scene meant, not just how it looked.
        #
        # Now every damaged building drops rubble, in proportion to how ruined
        # it is. `_hit_count` already scales by the local field, so this is the
        # fate multiplier on top of it: a collapsed ruin sheds all of it, one
        # that is merely cracked sheds `damaged_debris_scale` of it, and the
        # spread shrinks with it too — a standing building drops its rubble at
        # its own facade, it does not throw it across the street.
        # `want_destroyed` rather than `is_destroyed`: a building whose fate was
        # destruction but whose footprint no ruin asset fitted was mesh-damaged
        # instead. It is still a destroyed building and still sheds a destroyed
        # building's rubble — `is_destroyed` only records which MECHANISM ran.
        debris_scale = 1.0 if (is_destroyed or want_destroyed) else damaged_debris
        if debris_scale <= 0.0:
            continue
        reach = 0.35 + 0.65 * debris_scale

        # A leaning ruin piles rubble against the side now overhanging.
        lean_dir = None
        if is_destroyed and _hit(cx, cy, tilt_chance):
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
            # Debris is scene content, so it lives in the scene. `_ring_pos`
            # walks outward from the building's own footprint and knows nothing
            # about the region, and `apply_ground_planes` lays exactly one
            # asphalt mesh spanning `layout["region"]` — so a piece past the
            # edge is not "on the outskirts", it is hanging over nothing.
            # `apply_path_scour._point` has always sampled inside the region;
            # this is the same bound on the other emitter.
            if region and not (rx0 <= x <= rx1 and ry0 <= y <= ry1):
                dropped[cat] = dropped.get(cat, 0) + 1
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
            for _ in range(_hit_count(cx, cy, piles_per, debris_scale)):
                du = rng.choice(pile_usds)
                x_, y_ = _ring_pos(-1.0, pile_max_off * reach)
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
            for _ in range(_hit_count(cx, cy, pieces_per, debris_scale)):
                du = rng.choice(piece_usds)
                x_, y_ = _ring_pos(0.3, pieces_scatter * reach)
                _emit(du, x_, y_, _fp(du, "debris")["base"] + 0.4,
                      "debris", _sc(du) * rng.uniform(0.7, 1.2), settle=True)

    _apply_standins(placements, dis, resolver)
    placements.extend(new_placements)
    if tally:
        print("[disaster] buildings  "
              + "  ".join(f"{k}={v}" for k, v in sorted(tally.items())))
    if dropped:
        # Loud on purpose: a handful is the rim of the city behaving correctly,
        # but a large number means something upstream is throwing debris where
        # no building is — which is how the 100x footprint bug looked.
        print("[disaster] outside the region, dropped  "
              + "  ".join(f"{k}={v}" for k, v in sorted(dropped.items())))
    return tally


def _apply_standins(placements: list, dis: dict, resolver=None) -> None:
    """Tilt-and-sink the marked buildings live mesh damage will not cut.

    Ranked exactly as `mesh_damage.apply_to_stage` spends its budget — worst
    hit first, placement order breaking ties — so the two never disagree
    about which buildings get the stand-in and which get the cut.

    THE SINK COVERS THE LIFTED CORNER. `apply_placements` rotates about the
    footprint's base centre, so a 6 degree pitch on an 80 m building raises
    one end 4 m into the air — a floating building, which is worse than an
    intact one. The sink is therefore the drawn amount PLUS whatever the
    tilt lifts the highest base corner by, so every corner ends at or below
    grade: a leaning ruin has settled into the ground, not levitated off it.
    """
    from disaster import mesh_damage

    marked = [(i, p) for i, p in enumerate(placements) if "_standin" in p]
    if not marked:
        return
    budget = mesh_damage.damage_budget(dis)[0] if (
        (dis.get("mesh_damage") or {}).get("enabled", True)) else 0
    rank = sorted((ip for ip in marked if ip[1].get("_mesh_damage")),
                  key=lambda ip: (-float(ip[1]["_mesh_damage"]), ip[0]))
    cut = {i for i, _ in rank[:budget]}
    for i, p in marked:
        roll, pitch, sink = p.pop("_standin")
        if i in cut:
            continue
        axis_roll = 90.0 if p.get("axis_up") == "Y" else 0.0
        lift = 0.0
        if resolver is not None:
            try:
                fp = placement_footprint(resolver, p, "house")
                lift = (0.5 * float(fp.get("sx", 0.0))
                        * abs(math.sin(math.radians(pitch)))
                        + 0.5 * float(fp.get("sy", 0.0))
                        * abs(math.sin(math.radians(roll - axis_roll))))
            except Exception:                                 # noqa: BLE001
                lift = 0.0
        p["roll_deg"] = roll
        p["pitch_deg"] = pitch
        p["z_m"] -= sink + lift


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
    field = kinds.get(dis.get("type")).field(dis, (x0, y0, x1, y1))
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


def apply_path_scour(config: dict, layout: dict, placements: list,
                     resolver) -> dict:
    """Strew debris along the damage corridor itself. Returns a tally.

    Building debris hangs off each ruin, so it inherits the *layout*: it
    clusters on lots and stops at the property line. A tornado track does not.
    It lays a continuous band of dirt, splintered wood and scraped ground
    straight across lawns, streets and open fields alike, and that unbroken
    line is the single most recognisable thing about the aftermath from the
    air. This fills it in, sampling the whole region and keeping points in
    proportion to the local damage field — so the band appears wherever the
    disaster is intense, tapers at its edges, and is absent entirely for a
    `field` that never gets there.

    WHY IT LIVES HERE AND NOT IN `build_city`
    -----------------------------------------
    It used to run inside `build_city`, i.e. in the LAYOUT stage, and
    `districts` then re-packs every block treating debris as an immovable
    obstacle. So how much scour a severity produced decided how many buildings
    the city ended up with — 919 at severity 0 against 869 at 0.6 on the same
    seed, houses moved all over the plan. Scour is disaster output; running it
    in the stage that runs last is what makes it unable to perturb anything.

    Draws come from the disaster stream, distinct from the layout stream, for
    the reason in this module's header.
    """
    dis = _stage(config, "disaster")
    region = layout.get("region")
    if not dis or not region:
        return {}

    rules = dis.get("debris") or {}
    path_pieces = float(rules.get("path_pieces_per_100m2", 0.0))
    path_piles = float(rules.get("path_piles_per_100m2", 0.0))
    if path_pieces <= 0.0 and path_piles <= 0.0:
        return {}

    field = kinds.get(dis.get("type")).field(dis, tuple(region))
    if field.hi <= 0.0:
        return {}

    usds = config.get("usds") or {}
    scale = float(config.get("asset_scale", 1.0))
    root = str(config.get("asset_root", "") or "")
    scale_of: dict = {}
    axis_of: dict = {}

    def _pool(entries):
        paths, sc_ovr, au_ovr, _yaw, _tags = _normalize_usd_list(
            entries or [], scale, root)
        scale_of.update(sc_ovr or {})
        axis_of.update(au_ovr or {})
        return paths

    debris_cfg = usds.get("debris") or {}
    piece_usds = _pool(debris_cfg.get("pieces"))
    pile_usds = _pool(debris_cfg.get("piles"))
    if not piece_usds and not pile_usds:
        return {}

    def _sc(u):
        return float(scale_of.get(u, scale))

    def _au(u):
        return str(axis_of.get(u, "Z"))

    def _axis_roll(u):
        return 90.0 if _au(u) == "Y" else 0.0

    x0, y0, x1, y1 = (float(v) for v in region)
    rng = random.Random(int(config.get("seed", 0)) + 7717)
    path_min = float(rules.get("path_min_intensity", 0.25))
    path_shape = float(rules.get("path_density_shape", 1.6))
    exclusions = config.get("exclusions") or []

    # Densities are per 100 m2 of *affected ground*, not of the region, so the
    # same number means the same thing whether the field is a narrow tornado
    # corridor or a hurricane's region-wide blanket. Integrating the field
    # gives the effective affected area.
    n_ = 32
    mean_k = sum(field(x0 + (x1 - x0) * (i + 0.5) / n_,
                       y0 + (y1 - y0) * (j + 0.5) / n_)
                 for i in range(n_) for j in range(n_)) / (n_ * n_)
    area_100 = (x1 - x0) * (y1 - y0) / 100.0 * mean_k

    # Density gradient, peaked on the track centerline / epicenter and tapering
    # toward the edge — see `DamageField.density` for why this has to be
    # separate from the intensity itself, which stays a flat plateau
    # (correctly) for building and topple placement.
    #
    # OFF THE SAME FIELD OBJECT. The two used to be built from the config
    # independently and each dispatched on `kind` in its own way, so the
    # corridor debris lay on could drift from the corridor the damage was in.
    density = field.scour_density(shape=path_shape)

    # Buildings as they FINALLY stand — this runs after districts has rezoned
    # and after fate has been assigned, so it is the real footprint list, not
    # the one build_city happened to have mid-pass.
    house_rects = []
    for p in placements:
        if p.get("category") != "house":
            continue
        fp = p.get("_footprint_m")
        if not fp:
            f = placement_footprint(resolver, p, "house")
            fp = (f["sx"], f["sy"])
        house_rects.append((p["x_m"], p["y_m"], fp[0] / 2.0, fp[1] / 2.0))

    def _point():
        """A point on the track, or None.

        The field gates the extent (same threshold the rest of the disaster
        uses); acceptance is then weighted by the peaked density gradient
        rather than the flat field value, so the strip reads as densest along
        its centerline.
        """
        for _ in range(12):
            x, y = rng.uniform(x0, x1), rng.uniform(y0, y1)
            if field(x, y) < path_min:
                continue
            if rng.random() > density(x, y):
                continue
            # Keep the ground clear inside buildings that are still standing;
            # everything else — road, sidewalk, lawn, field — is fair game,
            # because the real thing covers all of it.
            if any(abs(x - hx) <= hw and abs(y - hy) <= hh
                   for hx, hy, hw, hh in house_rects):
                continue
            if exclusions and _in_exclusion(x, y, exclusions):
                continue
            return x, y
        return None

    tally: dict = {}
    new: list = []

    def _emit(pool, cat, n, lift, scale_lo, scale_hi, settle):
        for _ in range(int(round(n))):
            pt = _point()
            if pt is None:
                continue
            u = rng.choice(pool)
            fp = resolver.get(u, cat, scale=_sc(u), axis_up=_au(u))
            q = {"usd": u, "x_m": pt[0], "y_m": pt[1], "z_m": fp["base"] + lift,
                 "yaw_deg": rng.uniform(0.0, 360.0), "roll_deg": _axis_roll(u),
                 "pitch_deg": 0.0, "category": cat, "axis_up": _au(u),
                 "scale": _sc(u) * rng.uniform(scale_lo, scale_hi)}
            if settle:
                q["settle"] = True
            new.append(q)
            tally[cat] = tally.get(cat, 0) + 1

    if piece_usds and path_pieces > 0.0:
        _emit(piece_usds, "debris", path_pieces * area_100, 0.4, 0.7, 1.2, True)
    if pile_usds and path_piles > 0.0:
        _emit(pile_usds, "debris_pile", path_piles * area_100, 0.02, 0.8, 1.3,
              False)

    placements.extend(new)
    if tally:
        print("[disaster] path scour  "
              + "  ".join(f"{k}={v}" for k, v in sorted(tally.items())))
    return tally
