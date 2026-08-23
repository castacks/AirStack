#!/usr/bin/env python
"""
250 x 250 m of suburb, burnt from one corner. The minimum viable disaster scene.

    ISAAC_SIM_SCRIPT_NAME=suburb_mini_wildfire_launch_script.py \\
    airstack up isaac-sim

WHY 250 AND NOT 100
-------------------
100 x 100 m produces ZERO houses. `suburb_net` lays a street hierarchy —
collector, looping local street, cul-de-sac — and a single lot is 21-30 m wide
before carriageway and verge, so below about 250 m the graph yields one
unbuildable block. Measured: 100/150/200 m give 0 placements, 250 m gives ~630
placements across 2 blocks, which is roughly a dozen houses. That is still one
camera frame and about 3% of the full plat's area.

THE GRADIENT IS THE WHOLE POINT
-------------------------------
Damage is not scattered. `disaster.fire` already solves WHEN the elliptical
front reaches any point, so every building's state is a function of how long
ago it burned — `damage.level_for_age` is that mapping, and it drives the
structural level, the finish (fresh char vs cooled ash), the scorch coverage
AND the fire state together. One number, four consequences, so the scene reads
as a single event rather than four unrelated effects.

Running the front from the -X -Y corner along the diagonal gives:

    near the corner   burnt out and cold, ash, residual smoke only
    mid-field         partial collapse, fresh char, still flaming
    far corner        scorched but standing, small fires, roof intact

NO TREES. Vegetation is its own damage vocabulary and is not built; leaving it
green would fight everything else in frame. Roads, houses, fences, lot props.
"""

import math
import os
import random
import sys
import time

import carb
from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={
    "headless": False,
    # THE GROUND OVERLAY NEEDS FRACTIONAL CUTOUT OPACITY, AND IT HAS TO BE A
    # COMMAND-LINE FLAG: == disaster.ground.KIT_ARGS (not importable yet —
    # sys.path is set below). Setting it with carb.settings after startup is
    # never copied onto the USD render property and the overlay vanishes.
    "extra_args": ["--/rtx/raytracing/fractionalCutoutOpacity=true",
                   "--/rtx/pathtracing/fractionalCutoutOpacity=true"],
})

from isaacsim.core.utils.extensions import enable_extension

enable_extension("omni.flowusd")
enable_extension("omni.kit.window.script_editor")

import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade

from omni.isaac.core.world import World
from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

_ISAAC_SIM_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
_SCENE_GEN_DIR = os.path.normpath(
    os.path.join(_ISAAC_SIM_DIR, "..", "..", "scene_gen"))
sys.path.insert(0, os.path.join(_ISAAC_SIM_DIR, "utils"))
sys.path.insert(0, _SCENE_GEN_DIR)

from scene_prep import (scale_stage_prim, add_sky, get_stage_meters_per_unit)
from scene_generator import resolve_sky
import suburb_scene as ss
from suburb_scene import generate_suburb_on_stage
from compile_disaster import load_scene_config
from disaster import damage, fire, fracture, ground, settle
from disaster import vegetation as veg

ENV_URL = SIMULATION_ENVIRONMENTS["Default Environment"]
PARENT = "/World/stage/generated"
SCENE_CONFIG = os.environ.get("SCENE_CONFIG", "suburb_mini_wildfire")
SEED = int(os.environ.get("MINI_SEED", "11"))
# A CEILING, NOT A TARGET. `settle.run` steps in chunks and stops as soon as
# the busiest body moves less than a millimetre, so a pile that settles in 200
# steps costs 200 — this number only decides how long it is willing to wait.
#
# 300 -> 900 -> 2400, and the last raise was not a guess: at 900 the run
# reported "74 body(s) STILL MOVING at bake time", which is the solver saying
# the ceiling stopped it rather than rest did. Baking writes each body's
# CURRENT transform, so those 74 were frozen mid-fall — the floating house
# parts. Everything settles slower as the count climbs and this block now
# carries ~4,800 bodies against the few hundred 300 was set for.
SETTLE_STEPS = int(os.environ.get("SETTLE_STEPS", "2400"))
KEEP_PHYSICS = os.environ.get("KEEP_PHYSICS", "0") not in ("0", "", "false")
# How far through the burn the scene is. The front's own duration, so the far
# corner has only just been reached while the origin has long gone cold.
ELAPSED_S = float(os.environ.get("MINI_ELAPSED", "0")) or None
# The burn scar on the ground (`disaster.ground`). GROUND_* env knobs tune it,
# the same ones the bench uses; MINI_GROUND=0 leaves the grass plain.
GROUND_ON = os.environ.get("MINI_GROUND", "1") not in ("0", "false", "no")
# BAKE: after the full build, export each damaged HOUSE and TREE to its own
# self-contained USD under this dir (+ a manifest). Reload references them
# instead of regenerating / fracturing / settling — the per-object proof of
# the "bake damage, reference it" scaling path. Measures the reload speedup.
BAKE_DIR = os.environ.get("MINI_BAKE_DIR", "")

# level -> (walls to break, chance each is partial, fracture seeds per module)
# level -> (walls to break, chance each is partial, fracture seeds,
#           break-height range as a fraction of wall height)
#
# TWO AXES, NOT ONE. Severity has to raise how MANY walls come down AND how
# much of EACH one does. With a flat break height a `rubble` house lost the
# same proportion of every wall as a lightly damaged one, so the worst houses
# differed only in how many walls were touched — the silhouettes stayed alike.
#
# The break range therefore drops with severity: a light hit takes the top
# quarter of a wall and leaves a tall stub, a burnt-out shell breaks near the
# footing and leaves almost nothing standing. Wall counts run past what a
# cottage actually has (5), which simply means "all of them".
# The trailing count is FLOOR modules to break. The floor plate here is timber,
# not a concrete slab, so it burns through with everything else — leaving it
# whole under a collapsed house put an intact deck under the wreckage. It
# fractures like the rest and takes the same 45% consumption, so a burnt-out
# shell ends up with gaps in its floor rather than a clean stage.
BREAK_PLAN = {
    "pristine":         (0, 0.00, 0,  (0.00, 0.00), 0),
    "scorched":         (0, 0.00, 0,  (0.00, 0.00), 0),
    "roof_collapsed":   (1, 0.90, 9,  (0.58, 0.80), 0),
    "partial_collapse": (3, 0.75, 11, (0.38, 0.62), 1),
    "burned_out":       (5, 0.55, 12, (0.20, 0.44), 2),
    "rubble":           (8, 0.30, 14, (0.06, 0.26), 4),
}

_ENV_CLUTTER = {"GroundPlane", "Environment"}


def _remove_env_clutter(stage):
    n = 0
    for root_path in ("/", "/World", "/World/stage"):
        root = (stage.GetPseudoRoot() if root_path == "/"
                else stage.GetPrimAtPath(root_path))
        if not root or not root.IsValid():
            continue
        for child in root.GetChildren():
            if child.GetName() not in _ENV_CLUTTER or not child.IsActive():
                continue
            if child.SetActive(False):
                n += 1
    return n


def wait_for_stage(stage, timeout_s=10.0):
    for _ in range(int(timeout_s / 0.1)):
        omni.kit.app.get_app().update()
        wp = stage.GetPrimAtPath("/World")
        if wp.IsValid() and [c for c in wp.GetChildren()
                             if c.GetName() != "PhysicsScene"]:
            return True
        time.sleep(0.1)
    return False


def cluster_houses(placements, radius_m=13.0):
    """Group house modules into buildings by proximity.

    `generate_suburb_on_stage` returns one flat list, so the modules of every
    house are mixed together. They are tightly grouped in space though — a
    cottage is about 10 m across and lots are 20 m apart — so a greedy pass at
    13 m separates them reliably without needing the generator to tell us.
    """
    house = [p for p in placements
             if str(p.get("category", "")).startswith("house_")]
    clusters = []
    for p in sorted(house, key=lambda q: (q["x_m"], q["y_m"])):
        for c in clusters:
            if math.hypot(p["x_m"] - c["cx"], p["y_m"] - c["cy"]) <= radius_m:
                c["items"].append(p)
                n = len(c["items"])
                c["cx"] += (p["x_m"] - c["cx"]) / n
                c["cy"] += (p["y_m"] - c["cy"]) / n
                break
        else:
            clusters.append({"cx": p["x_m"], "cy": p["y_m"], "items": [p]})
    return [c for c in clusters if len(c["items"]) >= 3]


def consume_prim(stage, placement):
    """Deactivate a placement, unless it is something that does not burn.

    A SINGLE CHOKE POINT for every "this was consumed by the fire" removal.
    The upstream filters already exclude pools, but a guard that lives next to
    the filter is a guard that gets forgotten when a new pass is added — and
    losing a pool to a stray match is both wrong and hard to spot, because the
    symptom is a hole in the scene rather than an error.

    Returns True if it actually deactivated anything.
    """
    if damage.is_incombustible(placement.get("category")):
        return False
    path = placement.get("prim_path")
    prim = stage.GetPrimAtPath(path) if path else None
    if not prim or not prim.IsValid() or not prim.IsActive():
        return False
    return bool(prim.SetActive(False))


def cascade_supports(items, broken, radius_m=4.0, z_eps=0.5):
    """Add everything that stood ON something already broken.

    NOTHING HOLDS ITSELF UP. Choosing modules to destroy at random ignores the
    fact that a building is a stack: knock out a ground-floor wall and the wall
    above it, the floor it carried and the roof over that have nothing left to
    stand on. Picking without this produced exactly that — an intact upper wall
    hanging in the air over a destroyed lower one, and second-storey floor
    plates floating with no walls beneath them.

    So the pick is closed under support: repeatedly add any module sitting
    ABOVE a broken one and within `radius_m` horizontally, until nothing new
    is added. Roofs come along for free, because a roof is always above
    something.

    `broken` is a set of `id(placement)`; returns it extended.
    """
    by_id = {id(q): q for q in items}
    changed = True
    while changed:
        changed = False
        low = [by_id[i] for i in broken if i in by_id]
        for q in items:
            if id(q) in broken:
                continue
            zq = float(q.get("z_m", 0.0))
            for b in low:
                if float(b.get("z_m", 0.0)) >= zq - z_eps:
                    continue                      # not below it
                if math.hypot(q["x_m"] - b["x_m"],
                              q["y_m"] - b["y_m"]) <= radius_m:
                    broken.add(id(q))
                    changed = True
                    break
    return broken


def main():
    timeline = omni.timeline.get_timeline_interface()
    timeline.stop()

    pg = PegasusInterface()
    pg._world = World(**pg._world_settings)
    pg.load_environment(ENV_URL)

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("Stage failed to load")
    wait_for_stage(stage)
    _remove_env_clutter(stage)

    _t_build0 = time.time()
    config = load_scene_config(SCENE_CONFIG)
    _, ssf = get_stage_meters_per_unit(stage)
    binfo = {}        # region, pool holes, z ladder — for the ground overlay
    placements = generate_suburb_on_stage(
        stage, config, parent_path=PARENT, scene_scale_factor=ssf,
        info_out=binfo)
    for _ in range(10):
        omni.kit.app.get_app().update()
    add_sky(stage, resolve_sky(config))

    # placeholder replaced below
    fcfg = dict(fire.DEFAULTS)
    fcfg.update((config.get("disaster") or {}).get("fire") or {})
    ox, oy = fcfg["origin_m"]
    th = math.radians(float(fcfg["heading_deg"]))
    cos_t, sin_t = math.cos(th), math.sin(th)
    head, flank, back = (float(fcfg["head_mps"]), float(fcfg["flank_mps"]),
                         float(fcfg["back_mps"]))
    def arrival(x, y):
        dx, dy = x - ox, y - oy
        u = dx * cos_t + dy * sin_t
        v = -dx * sin_t + dy * cos_t
        return fire._ignition_time(u, v, head, flank, back)

    def age(x, y):
        t = arrival(x, y)
        return -1.0 if not math.isfinite(t) else elapsed - t

    def coverage_at(x, y):
        """Scorch coverage from burn age: everything the front reached is
        marked, and the longer ago it passed the heavier the deposit."""
        d = age(x, y)
        if d < 0.0:
            return 0.0
        return min(1.0, 0.45 + 0.55 * min(1.0, d / max(1e-6, elapsed)))

    clusters = cluster_houses(placements)

    # ELAPSED AND THE PHASE LENGTHS ARE DERIVED, NOT GUESSED. What matters is
    # that the buildings SPAN the damage range, and that depends on where the
    # generator happened to put them relative to the ignition point. Fixing
    # either by hand puts the whole block in one bucket — 300 s made all 13
    # `rubble`, the config's own 149 s left 7 of 13 pristine.
    #
    # So: run the clock to the moment the LAST building is reached, and scale
    # the phases to the spread of arrival times. The far corner is then always
    # just alight and the origin always long cold, whatever the layout.
    arrivals = [arrival(c["cx"], c["cy"]) for c in clusters]
    finite = [t for t in arrivals if math.isfinite(t)]
    elapsed = (ELAPSED_S or (max(finite) if finite else 1.0))
    span = max(1.0, elapsed - (min(finite) if finite else 0.0))
    phases = dict(ignition_s=0.03 * span, flame_s=0.35 * span,
                  smoulder_s=0.25 * span, ash_after_s=0.45 * span)
    print("[mini] {0} placements, {1} buildings, front crosses in {2:.0f}s "
          "(arrival spread {3:.0f}s)".format(
              len(placements), len(clusters), elapsed, span))

    fracture.ensure_deps()
    mats = damage.char_materials(stage, PARENT)
    tally = {}
    loose_all, static_extra, per_building = [], [], []

    for bi, c in enumerate(clusters):
        d = age(c["cx"], c["cy"])
        level, finish, fstate = damage.level_for_age(d, **phases)
        tally[level] = tally.get(level, 0) + 1
        n_walls, partial_p, seeds, cut_range, n_floors = BREAK_PLAN[level]
        rng = random.Random(SEED + 97 * bi)
        import numpy as np
        nrng = np.random.default_rng(SEED + 97 * bi)

        frags = []
        if seeds:
            roofs = [q for q in c["items"]
                     if damage._sub_of(q.get("category")) in ("roof", "bay_roof")]
            walls = [q for q in c["items"]
                     if damage._sub_of(q.get("category")) == "wall"]
            floors = [q for q in c["items"]
                      if damage._sub_of(q.get("category")) == "floor"]
            # LOWER WALLS FIRST. A fire burns from the ground up, and picking
            # uniformly left upper storeys destroyed over intact lower ones.
            walls_sorted = sorted(walls, key=lambda q: float(q.get("z_m", 0.0)))
            take = min(n_walls, len(walls_sorted))
            pick = walls_sorted[:take] if take else []
            if take and len(walls_sorted) > take:
                # a little randomness so every house does not fail identically
                pool = walls_sorted[:min(len(walls_sorted), take + 3)]
                pick = rng.sample(pool, take)

            # Floors break whole rather than partially: a deck has no "upper
            # part" to come down, it burns through and drops. UPPER floors go
            # first and always — an unsupported second storey is the most
            # obviously wrong thing in the scene.
            floors_sorted = sorted(floors, key=lambda q: -float(q.get("z_m", 0.0)))
            pick_f = floors_sorted[:min(n_floors, len(floors_sorted))]

            # NO UNTOUCHED WALL ON A WRECKED HOUSE. Above `roof_collapsed`,
            # every remaining wall gets at least a light break — the top
            # course chipped off — because a heavily damaged building with one
            # pristine wall standing in it reads as a bug, not as damage.
            light = []
            if level in ("partial_collapse", "burned_out", "rubble"):
                light = [q for q in walls if q not in pick]

            broken_ids = cascade_supports(
                c["items"], {id(q) for q in pick + pick_f + roofs + light})
            light_ids = {id(q) for q in light}
            targets = [q for q in c["items"] if id(q) in broken_ids]
            for q in targets:
                path = q.get("prim_path")
                if not path:
                    continue
                if damage.is_incombustible(q.get("category")):
                    continue      # a pool surround does not break up
                out = "{0}/brk_{1}_{2}".format(PARENT, bi,
                                               path.rsplit("/", 1)[-1])
                src_tex = damage.bound_texture(stage, path)
                is_wall = damage._sub_of(q.get("category")) == "wall"
                # A "light" wall keeps almost all of itself: the break line
                # sits high, so it loses a course off the top and stays
                # standing and recognisable.
                cut = ((0.74, 0.93) if id(q) in light_ids else cut_range)
                if is_wall and (id(q) in light_ids
                                or (q in pick and rng.random() < partial_p)):
                    st, lo = fracture.fracture_partial(
                        stage, path, out, n_pieces=seeds, rng=nrng,
                        cut_frac=rng.uniform(*cut), mode="char")
                    # THE STUB IS DYNAMIC TOO. Treating it as static geometry
                    # pinned it exactly where the cut left it — so a stub
                    # whose floor had been fractured away, or whose lower wall
                    # was destroyed, simply hung there. Once a module has been
                    # broken it is no longer attached to anything and has to
                    # obey gravity like every other piece.
                    frags.extend(st)
                    frags.extend(lo)
                    if src_tex:
                        heavy = damage.scorched_material(
                            stage, PARENT, None, len(damage.SOOT_LEVELS) - 1,
                            texture=src_tex, triplanar=True)
                        for pth in st + lo:
                            pr = stage.GetPrimAtPath(pth)
                            if pr and pr.IsValid():
                                UsdShade.MaterialBindingAPI(pr).Bind(heavy)
                else:
                    hi_floor = (damage._sub_of(q.get("category")) == "floor"
                                and float(q.get("z_m", 0.0)) > 1.5)
                    made = fracture.fracture_prim(
                        stage, path, out,
                        n_pieces=seeds + (3 if hi_floor else 0), rng=nrng,
                        mode="char", rough=0.045, verbose=False,
                        # MORE OF THE HOUSE IS GONE. Timber does not merely
                        # break in a fire, it burns away, and 30% left piles
                        # holding too much of what the building was made of.
                        # The skew stays LARGE-biased: a big surviving panel
                        # reads as a wall that fell over, and taking those out
                        # is what makes the remainder read as debris.
                        consume=0.55,
                        # ALMOST STRICTLY THE LARGEST. 1.25 kept letting
                        # mid-size panels into the draw and leaving big ones
                        # standing; 1.02 means the pieces that go are the
                        # pieces that read as "a wall that fell over".
                        consume_pool=1.02)
                    frags.extend(made)
                    # THE SAME TREATMENT AS A PARTIAL BREAK. Generic char maps
                    # made every fragment look like it came from the same
                    # anonymous burnt object; carrying the module's own
                    # texture at maximum scorch keeps a brick house's rubble
                    # visibly brick.
                    # A MIX, not one look. Every fragment carrying its own
                    # wall texture makes a pile read as a wall that fell over;
                    # every fragment carrying a char map makes it read as
                    # generic burnt matter. Real wreckage is both — pieces
                    # that still show what they were, next to pieces burnt
                    # past recognition. Roughly half and half.
                    heavy = (damage.scorched_material(
                        stage, PARENT, None, len(damage.SOOT_LEVELS) - 1,
                        texture=src_tex, triplanar=True) if src_tex else None)
                    for pth in made:
                        pr = stage.GetPrimAtPath(pth)
                        if not pr or not pr.IsValid():
                            continue
                        # 0.25, NOT 0.5. The mix was half the fragments
                        # keeping their own wall texture at maximum scorch and
                        # half taking the burn maps. That reads as too much
                        # surviving cladding in a pile that burned: three
                        # quarters now take the char/ash maps, and the quarter
                        # that keeps its own texture is what stops the rubble
                        # going anonymous.
                        if heavy is not None and rng.random() < 0.25:
                            UsdShade.MaterialBindingAPI(pr).Bind(heavy)
                        else:
                            UsdShade.MaterialBindingAPI(pr).Bind(
                                damage._pick(rng, finish or "char", mats))
        loose_all.extend(frags)
        per_building.append((bi, c, level, finish, fstate, frags))

    # FENCES BURN LIKE FUSES. A timber fence is a continuous line of dry fuel
    # touching the ground, so in a wildfire it is one of the first things to
    # go and one of the clearest signals from the air: runs simply vanish
    # where the fire passed, and survive as scorched stubs at the edge of it.
    #
    # Graded by burn age, and CAPPED: there are ~200 fence modules in this
    # block and fracturing them all would cost more fragments than the houses
    # do, for something read at a glance.
    fences = [q for q in placements
              if str(q.get("category", "")) == "fence" and q.get("prim_path")]
    frng = random.Random(SEED + 5)
    import numpy as np
    fnrng = np.random.default_rng(SEED + 5)
    n_gone = n_frac = 0
    # 26 -> 140. The cap is there because fracturing a whole fence line costs
    # more fragments than the houses do, and 26 was set when the block carried
    # ~180 fence modules. It now carries ~330, so the transition zone was
    # showing one part-broken fence in a dozen and every other run was binary:
    # untouched, or gone. The budget is the thing deciding that, not the fire.
    budget = 140
    for q in fences:
        d = age(q["x_m"], q["y_m"])
        if d <= 0.0:
            continue                       # front never reached it
        frac_of_burn = d / max(1e-6, span)
        prim = stage.GetPrimAtPath(q["prim_path"])
        if not prim or not prim.IsValid():
            continue
        # BANDS WIDENED. At 0.28-0.62 with a 45% roll this produced ZERO
        # part-broken fences on two consecutive runs: almost everything the
        # front reached sat above 0.62, so the transition zone caught nothing
        # and fences were binary — gone, or merely scorched.
        # WIDENED AGAIN. At >0.80 only the deepest fifth of the burn lost its
        # fences outright, so most of what the front crossed stood there
        # scorched and whole — a timber fence is a continuous line of dry fuel
        # touching the ground and is one of the FIRST things a wildfire takes.
        if frac_of_burn > 0.55 and frng.random() < 0.90:
            # Deep in the burn: consumed. Deactivating is both right and far
            # cheaper than fracturing a run into gravel.
            if consume_prim(stage, q):
                n_gone += 1
        elif frac_of_burn > 0.04 and budget > 0 and frng.random() < 0.85:
            src_tex = damage.bound_texture(stage, q["prim_path"])
            out = "{0}/fence_brk_{1}".format(
                PARENT, q["prim_path"].rsplit("/", 1)[-1])
            st, lo = fracture.fracture_partial(
                stage, q["prim_path"], out, n_pieces=5, rng=fnrng,
                cut_frac=frng.uniform(0.25, 0.55), mode="char", rough=0.05)
            if lo:
                loose_all.extend(st)      # stubs fall too — see above
                loose_all.extend(lo)
                if src_tex:
                    heavy = damage.scorched_material(
                        stage, PARENT, None, len(damage.SOOT_LEVELS) - 1,
                        texture=src_tex,
                        triplanar=True)
                    for pth in st + lo:
                        pr = stage.GetPrimAtPath(pth)
                        if pr and pr.IsValid():
                            UsdShade.MaterialBindingAPI(pr).Bind(heavy)
                budget -= 1
                n_frac += 1
    print("[mini] fences: {0} of {1} consumed, {2} part-broken".format(
        n_gone, len(fences), n_frac))

    # PROPS THAT BURN. Timber and plastic street furniture is fuel and goes
    # the same way the fences do; hydrants, poles and signs are metal and
    # survive, so they are left alone rather than sooted into invisibility.
    BURNABLE = ("bench", "chair", "picnic_table", "table", "trash_can",
                "play_structure", "planter", "bin", "cafe_set", "swing",
                "seesaw", "bike_rack", "mailbox", "bus_stop", "car")
    props = [q for q in placements
             if q.get("prim_path")
             and any(k in str(q.get("category", "")) for k in BURNABLE)
             and not damage.is_incombustible(q.get("category"))]
    prng = random.Random(SEED + 9)
    pnrng = np.random.default_rng(SEED + 9)
    n_pgone = n_pfrac = 0
    pbudget = 18
    for q in props:
        d = age(q["x_m"], q["y_m"])
        if d <= 0.0:
            continue
        f = d / max(1e-6, span)
        prim = stage.GetPrimAtPath(q["prim_path"])
        if not prim or not prim.IsValid():
            continue
        if f > 0.70 and prng.random() < 0.65:
            if consume_prim(stage, q):
                n_pgone += 1
        elif f > 0.20 and pbudget > 0 and prng.random() < 0.55:
            src_tex = damage.bound_texture(stage, q["prim_path"])
            out = "{0}/prop_brk_{1}".format(
                PARENT, q["prim_path"].rsplit("/", 1)[-1])
            made = fracture.fracture_prim(
                stage, q["prim_path"], out, n_pieces=6, rng=pnrng,
                mode="uniform", rough=0.05, verbose=False)
            if made:
                loose_all.extend(made)
                if src_tex:
                    heavy = damage.scorched_material(
                        stage, PARENT, None, len(damage.SOOT_LEVELS) - 1,
                        texture=src_tex,
                        triplanar=True)
                    for pth in made:
                        pr = stage.GetPrimAtPath(pth)
                        if pr and pr.IsValid():
                            UsdShade.MaterialBindingAPI(pr).Bind(heavy)
                pbudget -= 1
                n_pfrac += 1
    print("[mini] props: {0} of {1} consumed, {2} broken".format(
        n_pgone, len(props), n_pfrac))

    # VEGETATION. The block's trees burn off the SAME burn-age field the
    # buildings do — `veg.level_for_age` mirrors `damage.level_for_age`
    # deliberately — so a street of burnt-out houses stands in burnt trees and
    # the unburnt margin keeps its green ones. Scattering tree damage
    # independently is what would make the block read as two unrelated events.
    #
    # `stand_outcome` then rolls a minority of dead trees down to fallen or
    # stump. It is deliberately a minority: behind a real front a burnt stand
    # is a field of STANDING black poles — they come down over months to years
    # — and a block full of downed trunks reads as a tornado, not a fire.
    trees = [q for q in placements
             if str(q.get("category", "")).endswith("tree")
             and q.get("prim_path")]
    trng = random.Random(SEED + 13)
    veg_tally = {}
    n_veg_loose = 0
    for q in trees:
        d = age(q["x_m"], q["y_m"])
        level, _fstate = veg.level_for_age(d)
        if level != "pristine":
            level = veg.stand_outcome(level, trng)
        veg_tally[level] = veg_tally.get(level, 0) + 1
        if level == "pristine":
            continue
        res = veg.burn_tree(
            stage, q["prim_path"], level, PARENT,
            PARENT + "/tree_debris", trng,
            # HALF the bench's debris, up 5x from a tenth. The per-piece cost
            # is small — `wood_debris` cuts its stock ONCE per tree and every
            # piece after that comes off a small mesh — so this multiplies the
            # count without multiplying the slicing. What it does multiply is
            # RIGID BODIES: ~3,000 tree pieces becomes ~15,000, and every one
            # of them is a collider to cook and a body for the solver to carry
            # to rest. That, not the geometry, is what makes the build long.
            debris_scale=0.50, verbose=False)
        loose_all.extend(res["loose"])
        static_extra.extend(res["statics"])
        n_veg_loose += len(res["loose"])
        q["_veg_level"] = level
        q["_veg_made"] = (res["statics"] + res["loose"]
                          + list((res.get("info") or {}).get("made") or []))
    print("[mini] trees: {0} total -> {1}".format(
        len(trees), ", ".join("{0}={1}".format(k, v)
                              for k, v in sorted(veg_tally.items()))))
    print("[mini] trees: {0} loose piece(s)".format(n_veg_loose))

    # DOES ANYTHING STILL REACH THE GROUND? "The stumps seem to be missing so
    # they look like they're floating" is a claim that can be measured rather
    # than guessed at: for every burnt tree, find the lowest point of whatever
    # geometry it still owns — its own prims plus anything the passes wrote
    # for it — and compare against where it was planted.
    bc_v = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    gaps = []
    for q in trees:
        if q.get("_veg_level") in (None, "pristine"):
            continue
        lows = []
        for path in [q["prim_path"]] + list(q.get("_veg_made") or []):
            pr = stage.GetPrimAtPath(path)
            if not pr or not pr.IsValid() or not pr.IsActive():
                continue
            r = bc_v.ComputeWorldBound(pr).ComputeAlignedRange()
            if not r.IsEmpty():
                lows.append(float(r.GetMin()[2]))
        if lows:
            gaps.append(min(lows) - float(q.get("z_m", 0.0)))
    if gaps:
        gaps.sort()
        floating = sum(1 for g in gaps if g > 0.30)
        print("[mini] trees: base gap min {0:+.2f} m, median {1:+.2f} m, "
              "max {2:+.2f} m; {3} of {4} sit >0.30 m off the ground".format(
                  gaps[0], gaps[len(gaps) // 2], gaps[-1], floating,
                  len(gaps)))

    print("[mini] damage mix: " + ", ".join(
        "{0}={1}".format(k, v) for k, v in sorted(tally.items())))

    # Fragments of fully-broken modules take the burn maps; partial-wall pieces
    # already wear their own wall texture, heavily scorched.
    r = random.Random(SEED)
    for bi, c, level, finish, fstate, frags in per_building:
        for path in frags:
            prim = stage.GetPrimAtPath(path)
            if not prim or not prim.IsValid():
                continue
            if UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]:
                continue
            UsdShade.MaterialBindingAPI(prim).Bind(
                damage._pick(r, finish or "char", mats))

    # THE GROUND: a translucent burnt overlay following the fire's own field.
    #
    # `disaster.ground` — the fourth of the approaches catalogued in the
    # `build-wildfire-scenes` skill, the one that "drew nothing" until the
    # fractional-cutout flag (see SimulationApp above) and the single-tile
    # projection were understood, tuned on `burn_ground_preview_launch_script`
    # and brought here unchanged. Its own `feathered_coverage` is used rather
    # than `coverage_at` above: that one steps to 0.45 at the arrival line,
    # which is right for bucketing a house's scorch and wrong for a surface,
    # where it ends the scar on a conic with a hard cut against the grass.
    # The overlay sits between the grass and the asphalt on the same z ladder
    # `apply_ground` used, so roads, drives and walks stay unburnt over it —
    # which is correct — and pool holes are skipped, because you cannot
    # scorch water.
    n_ground = 0
    if GROUND_ON:
        region = tuple(binfo.get("region") or
                       (-0.5 * float(config["layout"]["region_m"][0]),
                        -0.5 * float(config["layout"]["region_m"][1]),
                        0.5 * float(config["layout"]["region_m"][0]),
                        0.5 * float(config["layout"]["region_m"][1])))
        zs = float(binfo.get("z_scale") or ss.ground_z_scale(config, region))
        burn_z = (ss._Z_GRASS + 0.5 * (ss._Z_ASPHALT - ss._Z_GRASS)) * zs
        knobs = ground.knobs_from_env(max(region[2] - region[0],
                                          region[3] - region[1]))
        print("[mini] ground: feather {0:.0f} m, fingers +-{1:.0f} m, islands "
              "{2:.0%}, overlay z {3:.4f} m".format(
                  knobs["edge_m"], knobs["finger_m"], knobs["islands"],
                  burn_z))
        ground_cov = ground.feathered_coverage(
            arrival, elapsed, (ox, oy), region,
            np.random.default_rng(SEED + 23),
            edge_m=knobs["edge_m"], finger_m=knobs["finger_m"],
            islands=knobs["islands"])
        made = ground.build_overlay(
            stage, ground_cov, region, ssf, burn_z, material_parent=PARENT,
            cell_m=knobs["cell_m"], bands=knobs["bands"],
            tile_m=knobs["tile_m"], op_range=knobs["op_range"],
            skip=ground.skip_rects(binfo.get("pool_rects") or (), pad=0.0))
        n_ground = len(made)
        print("[mini] ground overlay: {0} band(s)".format(n_ground))

    # EVERYTHING the front reached is scorched, not just the houses — fences,
    # bins, cars. A burnt street with clean props reads as a film set.
    # TREES ARE EXCLUDED: `disaster.vegetation` has already given every one of
    # them the treatment its own burn level calls for — browned foliage, a
    # charred bole, or no crown at all — and a generic soot wash composited on
    # top would overwrite that with the same map the bins get.
    n_soot = damage.soot_materials(
        stage, [q for q in placements
                if not str(q.get("category", "")).endswith("tree")],
        PARENT, random.Random(SEED), coverage_at=coverage_at)
    print("[mini] {0} subsets scorched".format(n_soot))

    for _ in range(15):
        omni.kit.app.get_app().update()

    # UPPER-STOREY SURVIVORS ARE DYNAMIC. A module that was never broken can
    # still be left unsupported — the floor beneath it may have been fractured
    # and 45% of that consumed, or the wall under it destroyed. Anything above
    # ground level in a building the fire reached is therefore simulated, so
    # it drops if nothing is holding it. Ground-level survivors stay static:
    # they rest on the terrain and would only add solver cost.
    # EVERYTHING IN A DAMAGED BUILDING IS SIMULATED EXCEPT AN INTACT WALL.
    #
    # The z > 0.3 rule was a guess at which survivors could be left
    # unsupported, and it was wrong in both directions: ground-level floors
    # sitting over consumed fragments stayed pinned, while poking anything
    # else showed it had no physics at all. A standing wall is the only thing
    # that genuinely still carries its own load — a floor, a roof, a door, a
    # canopy are all held up by something that may well have burned away.
    # `scorched` counts as INTACT. The front marked its outside and nothing
    # more — no module was broken, so nothing in it is unsupported and
    # simulating it could only make a sound house fall down.
    INTACT_LEVELS = ("pristine", "scorched")
    damaged = {id(q) for bi, c, lvl, fin, fs, fr in per_building
               if lvl not in INTACT_LEVELS for q in c["items"]}
    dynamic_survivors, standing = [], []
    for q in placements:
        path = q.get("prim_path")
        if not path or not stage.GetPrimAtPath(path).IsActive():
            continue
        intact_wall = damage._sub_of(q.get("category")) == "wall"
        if (id(q) in damaged and not intact_wall
                and not damage.is_incombustible(q.get("category"))):
            dynamic_survivors.append(path)
        else:
            standing.append(path)
    print("[mini] {0} unsupported survivors simulated".format(
        len(dynamic_survivors)))

    settle.run(stage, loose_all + dynamic_survivors, standing + static_extra,
               steps=SETTLE_STEPS, kick=0.15, rng=random.Random(SEED),
               bake_result=not KEEP_PHYSICS)

    fire.setup_flow_stack(stage, density_cell_size_m=0.12, max_blocks=16384,
                          scene_scale_factor=ssf)
    # AFTER SETTLING, so the plume sits on what is actually left. A burnt-out
    # shell is a low pile; taking its height from the authored roof left the
    # smoke floating several metres above the debris.
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    n_em = 0
    for bi, c, level, finish, fstate, frags in per_building:
        if not fstate:
            continue
        lo_z, hi_z = None, None
        for q in list(c["items"]) + [{"prim_path": f} for f in frags]:
            path = q.get("prim_path")
            prim = stage.GetPrimAtPath(path) if path else None
            if not prim or not prim.IsValid() or not prim.IsActive():
                continue
            r = bc.ComputeWorldBound(prim).ComputeAlignedRange()
            if r.IsEmpty():
                continue
            lo_z = r.GetMin()[2] if lo_z is None else min(lo_z, r.GetMin()[2])
            hi_z = r.GetMax()[2] if hi_z is None else max(hi_z, r.GetMax()[2])
        n_em += fire.add_structure_fire(
            stage, c["items"], fstate, fire.FLOW_ROOT, "b{0}".format(bi),
            random.Random(SEED + 300 + bi), max_emitters=3,
            base_z=(lo_z if lo_z is not None else 0.0),
            top_z=(hi_z if hi_z is not None else None),
            # A FULLY COLLAPSED HOUSE IS THE SMOKIEST THING IN THE SCENE. It
            # is a deep bed of smouldering timber with nothing left standing —
            # visually almost pure smoke — whereas a partly standing shell
            # still has structure doing most of the reading. Only `rubble`
            # gets the boost; the others were already right.
            strength=(1.9 if level == "rubble" else 1.0))
    print("[mini] {0} structure emitters".format(n_em))

    print("\n" + "=" * 72)
    print("MINI WILDFIRE — 250 x 250 m, front from ({0:.0f}, {1:.0f}) "
          "toward {2:.0f} deg".format(ox, oy, fcfg["heading_deg"]))
    print("  buildings: " + ", ".join(
        "{0}={1}".format(k, v) for k, v in sorted(tally.items())))
    print("  ground:    {0}".format(
        "{0} overlay band(s)".format(n_ground) if n_ground
        else "plain grass (MINI_GROUND=0)" if not GROUND_ON
        else "overlay drew nothing"))
    print("=" * 72 + "\n")

    # RE-ASSERT FRACTIONAL CUTOUT OPACITY AFTER THE STAGE IS BUILT.
    #
    # The flag is passed on the kit command line (SimulationApp extra_args)
    # and maps onto a render-settings USD PROPERTY at startup. This launcher
    # then brings up a Pegasus ENVIRONMENT stage, and loading a stage that
    # authors render settings resets that property to its default (OFF) —
    # so the ground overlay, whose only visibility is fractional cutout,
    # renders fully transparent even though the command-line flag was right.
    # (The bench keeps it because its stage is an empty new_stage() with no
    # authored render settings.) Setting the carb value again here, after the
    # final stage exists, pushes it back onto the live render property; a few
    # updates let it take before the loop.
    for _key in ("/rtx/raytracing/fractionalCutoutOpacity",
                 "/rtx/pathtracing/fractionalCutoutOpacity"):
        carb.settings.get_settings().set_bool(_key, True)
    for _ in range(5):
        omni.kit.app.get_app().update()

    _t_build = time.time() - _t_build0
    print("[bake] full build (generate + damage + fracture + settle + scorch "
          "+ ground + fire) took {0:.1f} s".format(_t_build))

    # BAKE: one self-contained USD per damaged house and per damaged tree.
    # `disaster.bake.export_object` flattens the stage once, then for each
    # object copies its scattered prims (surviving modules + `brk_*`
    # fragments, or the tree subtree + its `tree_debris` pieces) AND the
    # SHARED materials they bind (SootLooks/BurnLooks), remapping the bindings
    # so each file stands alone. World transforms ride along, so a reference
    # lands the object back where it was. A manifest records the files; the
    # reload references them with no generate / fracture / settle.
    if BAKE_DIR:
        # WHY NOT stage.Flatten(): raw USD Flatten() unpacks every crate value
        # in the composed stage, and chokes (`Usd_CrateFile::_UnpackValue`) on
        # the custom types the Flow fire / some Kit-schema referenced assets
        # carry — types Kit's own renderer and exporter handle but core USD
        # does not. So flatten with KIT's exporter first (the same
        # `export_as_stage_async` example_one_px4 uses), then SLICE per object
        # out of that normalised file with `Sdf.CopySpec`, which reads only the
        # house/tree/material specs and never touches the Flow prims.
        import asyncio
        from disaster import bake
        try:
            os.makedirs(BAKE_DIR, exist_ok=True)
            for _ in range(5):
                omni.kit.app.get_app().update()
            _t0 = time.time()
            _tmp = os.path.join(BAKE_DIR, "_flat_tmp.usd")
            _ok, _err = asyncio.get_event_loop().run_until_complete(
                omni.usd.get_context().export_as_stage_async(_tmp))
            if not _ok:
                raise RuntimeError("export_as_stage failed: {0}".format(_err))
            # Open the Kit-flattened file; the by-value exporter reads
            # geometry off THIS stage (raw Sdf.CopySpec / Flatten choke on the
            # kit modules' unreadable `assetInfo` crate field — see bake.py).
            src = Usd.Stage.Open(_tmp)
            records, miss_total = [], 0
            for bi, c, level, finish, fstate, frags in per_building:
                if level == "pristine":
                    continue
                paths = [it.get("prim_path") for it in c["items"]
                         if it.get("prim_path")] + list(frags)
                outp = os.path.join(BAKE_DIR, "house_{0:02d}.usd".format(bi))
                if bake.export_object(src, None, paths, outp):
                    m, ok, ms = bake.validate(outp)
                    miss_total += ms
                    records.append(dict(kind="house", id=bi, level=level,
                                        usd=os.path.abspath(outp), meshes=m,
                                        bound_missing=ms))
            for ti, q in enumerate(trees):
                if q.get("_veg_level") in (None, "pristine"):
                    continue
                paths = [q["prim_path"]] + list(q.get("_veg_made") or [])
                outp = os.path.join(BAKE_DIR, "tree_{0:03d}.usd".format(ti))
                if bake.export_object(src, None, paths, outp):
                    m, ok, ms = bake.validate(outp)
                    miss_total += ms
                    records.append(dict(kind="tree", id=ti,
                                        level=q.get("_veg_level"),
                                        usd=os.path.abspath(outp), meshes=m,
                                        bound_missing=ms))
            bake.write_manifest(os.path.join(BAKE_DIR, "manifest.json"), records)
            try:
                os.remove(_tmp)
            except OSError:
                pass
            _dt = time.time() - _t0
            nh = sum(1 for r in records if r["kind"] == "house")
            nt = sum(1 for r in records if r["kind"] == "tree")
            sz = sum(os.path.getsize(r["usd"]) for r in records
                     if os.path.exists(r["usd"])) / 1e6
            print("[bake] {0} house + {1} tree object(s) -> {2} "
                  "({3:.0f} MB total) in {4:.1f} s; {5} mesh material(s) "
                  "unresolved".format(nh, nt, BAKE_DIR, sz, _dt, miss_total))
            print("[bake] RELOAD WITH: MINI_RELOAD_DIR={0} "
                  "ISAAC_SIM_SCRIPT_NAME=suburb_reload_launch_script.py".format(
                      os.path.abspath(BAKE_DIR)))
        except Exception as _exc:
            # A bake failure must NOT crash the build — keep the app up so the
            # 25-min scene is not lost and can be inspected / retried.
            import traceback
            print("[bake] FAILED (scene kept live): {0}".format(_exc))
            traceback.print_exc()

    timeline.play()
    app = omni.kit.app.get_app()
    while simulation_app.is_running():
        app.update()
    timeline.stop()
    simulation_app.close()


if __name__ == "__main__":
    main()
