#!/usr/bin/env python
"""
Bake one USD per house TYPE and tree TYPE for a TORNADO — the archetype library
`suburb_tornado_launch_script.py` assembles the track from.

    ARCH_DIR=/isaac-sim/AirStack/scene_gen/assets/archetypes_tornado \
    ISAAC_SIM_SCRIPT_NAME=bake_tornado_archetypes_launch_script.py \
    airstack up isaac-sim

The wind counterpart of `bake_archetypes_launch_script.py`, and structurally
the same script: build every (house style x damage level) and (tree species x
wind level) ONCE on a spread-out grid, break each with the SAME code the live
scene would use, settle the whole grid in one pass, and export each object
re-centred to the origin via `disaster.bake`. What differs is every table and
three of the four fracture arguments — see `disaster/wind_flow.py`.

WHY THIS IS A SEPARATE DIRECTORY FROM THE FIRE BAKE
---------------------------------------------------
`ARCH_DIR` must NOT be the wildfire library. Both write
`house_<style>_<level>.usd`, and while the level names differ today
(`roof_collapsed` and `partial_collapse` are in BOTH ladders), a shared
directory means a tornado `roof_collapsed` silently overwrites the burnt one
and the wildfire plat comes up with unburnt wreckage in it. Default is
`assets/archetypes_tornado`.

THE THROW DIRECTION IS LOCAL +X, ALWAYS
---------------------------------------
`settle` runs with a `bias` along +X, so every archetype's debris leans and
travels the same way in its own frame. The assembly then chooses the yaw:

  * a house that is still recognisable as a house (`roof_stripped` up to
    `partial_collapse`) is yawed to the STREET, because a house facing the
    wrong way is the most obvious defect a suburb can have, and its debris
    field is small enough that its direction is not the read;
  * a house that is a pile (`leveled`, `swept`) is yawed to the TRACK,
    because nobody can tell which way a pile was facing and the direction its
    material went is the whole point.

Same for trees: a fallen archetype is baked lying toward +X and yawed into the
track at assembly. That is legal — the skill's "do not roll or pitch a baked
archetype" warning is about X and Y, and a yaw about Z carries the debris bed
round with the object exactly as intended.

Env knobs:

    ARCH_DIR    output directory (default `scene_gen/assets/archetypes_tornado`)
    ARCH_SEED   rng seed (default 7)
    ARCH_KINDS  which halves to rebuild — `house,tree` (default), `tree`,
                `house`. The manifest is MERGED on a restricted run.
    THROW_MPS   settle bias magnitude, m/s (default 9.0). This is the ONE knob
                that decides whether wreckage leans downwind or piles in place.

Settle knobs — every one of these exists because a bake shipped debris frozen
in mid-air and nothing in the run said so:

    SETTLE_STEPS      throw-phase target (default 1800)
    SETTLE_MAX_STEPS  hard cap the throw phase may run to when it has not
                      come to rest (default 6000). `steps` is a target now,
                      not a ceiling — see `settle.run(converge=True)`
    SETTLE_QUIET_STEPS  the settling-out phase after the throw, damped, no
                      new impulse (default 900). This is what lands the
                      pieces that are still in the air when the throw budget
                      would have expired
    SETTLE_DECOMP_M   cook loose pieces this size and up (bbox diagonal, m)
                      as convex DECOMPOSITIONS instead of hulls (default 0 =
                      off). The hull of an L-shaped wall section encloses its
                      own concavity and the next piece rests on that
                      invisible volume — try 2.5 if big parts still float,
                      and expect the settle to take minutes rather than one
                      minute
    BAKE_STRICT       1 = refuse to export at all if the settle finished with
                      bodies still moving or below grade (default 0: export,
                      but say so in a banner nobody can miss)
    ARCH_AUDIT        1 = re-open every exported archetype and measure what
                      fraction of its meshes have nothing under them / sit
                      below grade (default 1; this is the number the floating
                      debris complaint is about)
"""

import math
import os
import random
import sys
import time

import carb
from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": False})

from isaacsim.core.utils.extensions import enable_extension

enable_extension("omni.kit.window.script_editor")

import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdShade

_ISAAC_SIM_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
_SCENE_GEN_DIR = os.path.normpath(
    os.path.join(_ISAAC_SIM_DIR, "..", "..", "scene_gen"))
sys.path.insert(0, os.path.join(_ISAAC_SIM_DIR, "utils"))
sys.path.insert(0, _SCENE_GEN_DIR)

import numpy as np                                              # noqa: E402
import scene_generator as sg                                   # noqa: E402
from scene_prep import get_stage_meters_per_unit               # noqa: E402
from detail import modular_house as mh                         # noqa: E402
from disaster import bake, fracture, planks, settle, wind_flow  # noqa: E402
from disaster import tornado as tn                             # noqa: E402
from disaster import vegetation as veg                         # noqa: E402

PARENT = "/World/stage/generated"


def _env(name, default):
    """`os.environ.get` with EMPTY TREATED AS ABSENT, which it is not.

    THE COMPOSE FILE EXPORTS THESE EMPTY. `docker exec isaac-sim env` on this
    container shows `ARCH_SEED=`, `ARCH_DIR=`, `SETTLE_STEPS=`, `ARCH_STYLES=`
    and friends — declared, unset, and therefore present as `""`. So
    `os.environ.get(name, default)` returns the empty string and the default
    is never reached, and each of these then fails in its own way:

      * `int("")` / `float("")` raise `ValueError: invalid literal for int()
        with base 10: ''` fourteen seconds into a launch, which reads like a
        code bug and is an env one;
      * `ARCH_DIR` is far worse because it does NOT raise — `OUT_DIR` becomes
        `""`, `os.makedirs("", exist_ok=True)` is a no-op, and the whole
        library is written into the container's CWD with a clean banner.

    Every knob in this file goes through here.
    """
    v = os.environ.get(name)
    return default if v is None or not v.strip() else v.strip()


SEED = int(_env("ARCH_SEED", "7"))
OUT_DIR = _env(
    "ARCH_DIR", os.path.join(_SCENE_GEN_DIR, "assets", "archetypes_tornado"))
# 50 rather than the fire bake's 40. Nothing here is consumed, so a wrecked
# house keeps more material than a burnt one does, and the settle THROWS it —
# at 9 m/s a fragment can carry the better part of ten metres before it beds
# in. A piece that lands inside the neighbouring cell is not an export bug
# (the exporter is given explicit prim paths) but it does settle against the
# wrong geometry, which is a physically wrong resting pose.
GRID = 50.0
KINDS = set(k.strip().lower()
            for k in _env("ARCH_KINDS", "house,tree").split(",")
            if k.strip())
THROW_MPS = float(_env("THROW_MPS", "9.0"))
# THE SETTLE IS THE FLOATING-DEBRIS FIX, and these are its knobs. Measured
# 2026-08-27 on the previous library: 1200 of 1200 steps consumed with bodies
# still moving, 11-29% of the meshes in every levelled house with nothing
# under them within 0.10 m, and 21-42% of them BELOW the ground — material
# the solver pushed through a four-vertex ground quad at ~20 m/s.
# MEASURED ON THE 2026-08-27 18:06 HOUSE BAKE, which is what the "floating
# house debris" report is looking at:
#
#     [settle] 6784 rigid, 722 static, baked 6784
#     [settle]   248.4s solving (GPU)
#     [settle]   1200 of 1200 steps used; 16 body(s) STILL MOVING at bake time
#                                                <-- RAISE THE STEP BUDGET
#
# The budget was consumed whole, and a body still moving at bake time is a
# body FROZEN WHERE IT WAS — mid-flight, if that is where it happened to be.
# 1200 was never a convergence result; it was the ceiling, reported as one.
# 1800 is the new target and the throw phase may run to 6000 for as long as
# the moving count is still falling (`converge=True` gives up early and loudly
# when it is not, because past that point more steps are not the fix).
#
# THE QUIET PHASE IS THE HALF THAT ACTUALLY LANDS THINGS and it was the
# smallest number here. At 9 m/s into 0.16 damping a fragment stays airborne
# for a long time; 360 damped steps is not enough to bring down what the throw
# left in the air. 900.
#
# Cost, at the measured 0.207 s/step for 6,784 bodies: ~6 min if the throw
# runs its target and the quiet phase runs in full, ~25 min in the worst case
# where it goes all the way to 6000. That is a once-per-library price and the
# early exit means the cheap cells pay none of it.
SETTLE_STEPS = int(_env("SETTLE_STEPS", "1800"))
SETTLE_MAX_STEPS = int(_env("SETTLE_MAX_STEPS", "6000"))
SETTLE_QUIET_STEPS = int(_env("SETTLE_QUIET_STEPS", "900"))
SETTLE_DECOMP_M = float(_env("SETTLE_DECOMP_M", "0"))


def _flag(name, default="0"):
    return _env(name, default) not in ("", "0", "false",
                                                         "False")


BAKE_STRICT = _flag("BAKE_STRICT")
# WATCH IT SETTLE INSTEAD OF BAKING IT. `KEEP_PHYSICS=1` leaves every rigid
# body, collider and physics scene in place, writes NOTHING to disk, and
# leaves the timeline PAUSED on the settled pile — so the run can be watched
# with the play button instead of inspected after the fact. Stop then play
# replays the whole settle from the authored start.
#
# It also skips the export, which is the point: an export is what freezes the
# pose and strips the physics, and it would overwrite a library that is
# already good.
KEEP_PHYSICS = _flag("KEEP_PHYSICS")
# EVERY LOG A RIGID BODY. `wood_debris` normally SEATS anything shorter than
# 0.8 m — laid flat by arithmetic, never simulated — which is most of the
# debris under a tree. `SIM_ALL_DEBRIS=1` sets that threshold to 0 so every
# piece is thrown, falls and comes to rest under the solver.
SIM_ALL_DEBRIS = _flag("SIM_ALL_DEBRIS")
# Touch this file to make a KEEP_PHYSICS run bake what is on screen. Outside
# the repo by default so an approval gesture never lands in git.
BAKE_TRIGGER = _env("BAKE_TRIGGER", "/isaac-sim/.bake_now")
ARCH_AUDIT = _flag("ARCH_AUDIT", "1")

HOUSE_LEVELS = tn.HOUSE_LEVELS            # pristine .. swept
# `pristine` trees are referenced from the green species USD by the assembly
# and never baked — same as the wildfire path.
TREE_LEVELS = ("limbed", "leaning", "fallen", "snapped")

TREE_SPECIES = {
    "Black_Oak": "airstack://scene_gen/assets/aec/tower/Assets/Vegetation/Black_Oak/Black_Oak.usd",
    "Shumard_Oak": "airstack://scene_gen/assets/aec/tower/Assets/Vegetation/Shumard_Oak/Shumard_Oak.usd",
    "Douglas_Fir": "airstack://scene_gen/assets/aec/brownstone/Assets/Vegetation/Trees/Douglas_Fir.usd",
    "Largetooth_Aspen": "airstack://scene_gen/assets/aec/brownstone/Assets/Vegetation/Trees/Largetooth_Aspen.usd",
    "Common_Apple": "airstack://scene_gen/assets/aec/tower/Assets/Vegetation/Common_Apple/Common_Apple.usd",
    "American_Beech": "airstack://scene_gen/assets/aec/brownstone/Assets/Vegetation/Trees/American_Beech.usd",
}

# LEVELS WHOSE BASE IS DELIBERATELY OFF THE GROUND. `bake.export_object`'s
# `drop_to_ground` seats an object by its FIRST root's world min-z, and for a
# tipped tree that minimum is a crown branch below grade — so dropping it
# would lift the whole tree until the lowest twig touched z=0 and leave the
# trunk floating several metres up. `tip_tree` already seats the base by
# construction (it pivots about the local origin) and lifts it by the root
# plate's own radius on purpose.
_NO_DROP = ("leaning", "fallen")


def build_ground_and_light(stage):
    plane = UsdGeom.Mesh.Define(stage, Sdf.Path("/World/arch_ground"))
    e = 900.0
    plane.CreatePointsAttr([Gf.Vec3f(-e, -e, 0), Gf.Vec3f(e, -e, 0),
                            Gf.Vec3f(e, e, 0), Gf.Vec3f(-e, e, 0)])
    plane.CreateFaceVertexCountsAttr([4])
    plane.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    plane.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
    plane.CreateDisplayColorAttr([Gf.Vec3f(0.2, 0.25, 0.15)])
    dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/dome"))
    dome.CreateIntensityAttr(900.0)
    key = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/key"))
    key.CreateIntensityAttr(2200.0)
    key.AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 0.0, 30.0))


# The audit and the pile-level list now live in `disaster.bake`, shared with
# the wildfire and earthquake bakes so the three cannot drift apart.
_PILE_LEVELS = bake.PILE_LEVELS


def merge_manifest(path, records, kinds):
    """The new records, plus whatever the manifest had for OTHER kinds.

    Same bookkeeping the fire bake documents: a restricted run
    (`ARCH_KINDS=tree`) must not delete the manifest entries for the half it
    did not build, because those USDs are still on disk and the assembly
    drives entirely off the manifest.
    """
    old = []
    if os.path.exists(path):
        try:
            old = bake.read_manifest(path)
        except Exception as exc:
            print("[tarch] existing manifest unreadable, replacing it: "
                  "{0}".format(exc))
            old = []

    def key(r):
        return (r.get("kind"), r.get("style") or r.get("species"),
                r.get("level"))

    fresh = set(key(r) for r in records)
    kept = [r for r in old
            if r.get("kind") not in kinds and key(r) not in fresh]
    if kept:
        print("[tarch] manifest: keeping {0} record(s) for kind(s) not built "
              "this run".format(len(kept)))
    return kept + records


def main():
    omni.timeline.get_timeline_interface().stop()
    ctx = omni.usd.get_context()
    ctx.new_stage()
    stage = ctx.get_stage()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    world = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    stage.SetDefaultPrim(world.GetPrim())
    UsdGeom.Xform.Define(stage, Sdf.Path("/World/stage"))
    _, ssf = get_stage_meters_per_unit(stage)
    build_ground_and_light(stage)
    # A FRESH CONTAINER PAYS A RUINED RUN, NOT A PIP INSTALL. trimesh caches
    # which engines are available at import time, so a package installed after
    # `import trimesh` is invisible for the life of the process: every capped
    # slice silently returns empty and every archetype bakes with no debris,
    # with a complete-looking banner. See the wildfire skill's Environment
    # section.
    fracture.ensure_deps()
    os.makedirs(OUT_DIR, exist_ok=True)
    t_build0 = time.time()

    # One shared plank/lumber material set for the whole grid — `wreck_building`
    # binds bare-timber fragments out of it, so a broken stud in an archetype
    # and a loose board the assembly scatters beside it agree.
    pmats = planks.materials(stage, PARENT)
    mat_cache = {}

    # A NOTE ON WHY THERE IS NO PER-STYLE PLANK SET HERE, because the obvious
    # fix is the wrong one and it was written and taken out again.
    #
    # `wind_flow._debris_material` sends a share of every module's fragments
    # (`_BARE`) to bare sawn timber and the rest to the module's OWN cladding
    # texture. Until 2026-08-27 the bare draw ran over the whole
    # `planks.STOCK` cutting list, which by then included `siding` and `deck` —
    # the two classes that exist to carry a HOUSE'S colour — and on this
    # shared `pmats` those two get the `_TINT` fallback, i.e. the same
    # Ash_Planks map as a stud. Measured on the archetypes baked 2026-08-24
    # (pxr probe, share of bound targets wearing Ash_Planks_BaseColor.png):
    #
    #     ranch (brick_red)   leveled  63.2%   swept 73.0%
    #     villa (stucco)      leveled  70.1%   swept 70.4%
    #     terrace (brick_red) leveled  69.1%   swept 69.4%
    #     wide_house (cream)  leveled  67.8%   swept 72.5%
    #     cottage (wood_white) leveled 74.6%   swept 72.4%
    #
    # — identical whatever the palette, which is the "every wrecked house turns
    # to wood" report. Pristine is 0%, so `mh.apply_palette` is fine and
    # `fracture` (which binds nothing at all) is fine; the loss was entirely in
    # that draw.
    #
    # Skinning `siding` and `deck` per style here WOULD have worked, and it is
    # the wrong place: a fragment of a wrecked house is not a loose board, it
    # already knows what it was clad in, and `_debris_material`'s other branch
    # binds exactly that. The fix is therefore in `wind_flow` — `_BARE_STOCK`
    # restricts the bare draw to the four classes that really are bare timber,
    # and `_BARE` itself was halved because two thirds of a levelled house
    # being framing is true of its VOLUME and not of the AREA a camera sees.
    # Nothing needs to be passed from here.

    # SOIL FOR THE ROOT PLATES. A windthrown tree levers a wheel of earth out
    # of the ground and that plate is the feature that tells a fallen tree
    # from a felled one at any distance — so it has to look like earth rather
    # than like whatever `displayColor` the renderer falls back to. Referenced
    # once here and handed to every `wind_tree` call; `bake.export_object`
    # rebuilds it by value into each archetype that uses it.
    SOIL_PATH = PARENT + "/WindLooks/soil"
    try:
        _sp = stage.DefinePrim(Sdf.Path(SOIL_PATH))
        _sp.GetReferences().AddReference(sg._join_asset_root(
            "airstack://scene_gen/assets/materials/Dirt.usda", ""))
        _sp.Load()
        if not UsdShade.Material.Get(stage, SOIL_PATH):
            SOIL_PATH = ""
    except Exception as _exc:
        print("[tarch] soil material unavailable ({0}); root plates will "
              "render untextured".format(_exc))
        SOIL_PATH = ""

    # THE LOG SURFACES, SHARED THE SAME WAY THE SOIL IS. Two of them, because
    # `wood_debris` cuts riven columns out of the bole rather than branches off
    # the outside and most pieces therefore have no bark on them at all — see
    # `tornado.log_materials` for the measured value table this replaces and
    # `vegetation.split_wood_material` for the share.
    #
    # PASSING THESE IS WHAT KEEPS `wind_tree` OFF ITS FALLBACK, and the
    # fallback is the bug: with no material path it reads each tree's own bark
    # diffuse through `damage._pbr`, which carries no normal map, at one tile
    # per 3.33 m — a soft mottled patch whose value is whatever species the
    # tree happened to be, reported as "the logs ... look like the burnt
    # forest floor one".
    try:
        LOG_BARK, LOG_SPLIT = tn.log_materials(stage, PARENT)
    except Exception as _exc:
        print("[tarch] log materials unavailable ({0}); debris falls back to "
              "the per-species bark diffuse".format(_exc))
        LOG_BARK = LOG_SPLIT = ""

    # ---- HOUSES ----------------------------------------------------------
    # The grid geometry is computed whether or not houses are built this run,
    # so the TREE rows do not move between a full bake and `ARCH_KINDS=tree`.
    # ARCH_STYLES — bake a SUBSET, so the library can be built in batches.
    #
    # Added 2026-08-31 after this bake OOM-killed an OSMO pod and took every
    # container on it down with it. The chain: CUDA is in a bad state on that
    # worker, so PhysX silently falls back to CPU; a CPU settle holds all
    # 6,845 rigid bodies and their cooked collision meshes in HOST RAM rather
    # than on the card; and that exceeds the pod's 80Gi memory cgroup. The
    # kernel log is unambiguous -- "Memory cgroup out of memory: Killed
    # process (python.sh)".
    #
    # Peak memory scales with the bodies alive at once, i.e. styles x levels.
    # Three styles at a time cuts it roughly threefold for the same total
    # output, since each batch exports and exits before the next starts. Same
    # knob name and semantics as bake_quake_archetypes_launch_script.py.
    styles = list(mh.STYLES.keys())
    _want = _env("ARCH_STYLES", "")
    if _want:
        _sel = [q.strip() for q in _want.split(",") if q.strip()]
        _bad = [q for q in _sel if q not in styles]
        if _bad:
            print("[tarch] unknown ARCH_STYLES: {0}; known: {1}"
                  .format(", ".join(_bad), ", ".join(styles)))
        styles = [q for q in _sel if q in styles] or styles
        print("[tarch] ARCH_STYLES -> {0} style(s): {1}"
              .format(len(styles), ", ".join(styles)))
    hcombos = [(st, lv) for st in styles for lv in HOUSE_LEVELS]
    ncol = max(1, int(math.ceil(math.sqrt(len(hcombos)))))
    house_specs = []          # [style, level, X, Y, parent, placements, frags]
    for idx, (st, lv) in enumerate(hcombos if "house" in KINDS else []):
        X, Y = (idx % ncol) * GRID, (idx // ncol) * GRID
        parent = "{0}/h_{1}_{2}".format(PARENT, st, lv)
        UsdGeom.Scope.Define(stage, Sdf.Path(parent))
        pls = mh.build_building(st, X, Y, 0.0, random.Random(SEED),
                                category="house")
        pal = mh.STYLES[st].get("palette")
        if pal:
            for q in pls:
                q["palette"] = pal
        sg.apply_placements(stage, pls, parent, ssf)
        mh.apply_palette(stage, pls, parent)
        house_specs.append([st, lv, X, Y, parent, pls, []])
    print("[tarch] built {0} house grid cells".format(len(house_specs)))

    all_loose, all_static = [], []
    for spec in house_specs:
        st, lv, X, Y, parent, pls, _f = spec
        if lv == "pristine":
            continue
        seed = SEED + (abs(hash((st, lv))) % 100000)
        frags = wind_flow.wreck_building(
            stage, parent, pls, "{0}_{1}".format(st, lv), lv,
            random.Random(seed), np.random.default_rng(seed), pmats,
            mat_cache=mat_cache)
        spec[6] = frags
        all_loose.extend(frags)
        # NO `soot_materials` CALL, and its absence is the point. Every
        # material pass in `disaster.damage` composites a soot wash; there is
        # no parameterisation that turns that off, and its lowest coverage
        # bucket is 0.52. A tornado scene calls none of them.
    print("[tarch] houses wrecked: {0} loose fragment(s)".format(
        len(all_loose)))

    # ---- TREES -----------------------------------------------------------
    def n_limbs_for(level):
        try:
            return veg.wind_plan(level)[2]
        except Exception:
            return 0

    tree_specs = []   # [species, level, X, Y, tree_path, extra, anchored]
    tree_loose = []           # -> convexDecomposition (a limb is not convex)
    # SKIP WHAT THE ASSEMBLY WILL NEVER ASK FOR. `tornado.NO_UPROOT` promotes
    # `fallen` to `snapped` for the wide-crowned species, so baking their
    # fallen archetype is pure cost — and a stale one on disk is worse than
    # absent, because it would be referenced by any future caller that forgot
    # the promotion.
    tcombos = [(sp, lv) for sp in TREE_SPECIES for lv in TREE_LEVELS
               if not (lv == "fallen" and sp in tn.NO_UPROOT)]
    tcol = max(1, int(math.ceil(math.sqrt(len(tcombos)))))
    y0 = (len(hcombos) // ncol + 2) * GRID    # trees below the house grid
    for idx, (sp, lv) in enumerate(tcombos if "tree" in KINDS else []):
        X = (idx % tcol) * GRID
        Y = y0 + (idx // tcol) * GRID
        tp = "{0}/t_{1}_{2}".format(PARENT, sp, lv)
        prim = stage.DefinePrim(Sdf.Path(tp))
        prim.GetReferences().AddReference(
            sg._join_asset_root(TREE_SPECIES[sp], ""))
        xf = UsdGeom.Xformable(prim)
        xf.AddTranslateOp().Set(Gf.Vec3d(X, Y, 0.0))
        xf.AddScaleOp().Set(Gf.Vec3f(0.01, 0.01, 0.01))
        for _ in range(2):
            omni.kit.app.get_app().update()
        rng = random.Random(SEED + (abs(hash((sp, lv))) % 100000))
        # `azimuth_deg=0` — everything falls toward +X in the archetype's own
        # frame, and the assembly's yaw aims it. See the header.
        res = veg.wind_tree(stage, tp, lv, PARENT,
                            PARENT + "/tdeb_%s_%s" % (sp, lv), rng,
                            azimuth_deg=0.0, soil_material_path=SOIL_PATH,
                            wood_material_path=LOG_BARK,
                            split_material_path=LOG_SPLIT,
                            debris_scale=0.5, ground_z=0.0,
                            # SIMULATE EVERYTHING when asked: 0.0 means no
                            # piece is small enough to be seated by
                            # arithmetic, so every log is a rigid body.
                            simulate_above_m=(0.0 if SIM_ALL_DEBRIS else None),
                            seated_collide=False, verbose=False)
        extra = list(res.get("statics", [])) + list(res.get("loose", [])) \
            + list(res.get("seated", [])) \
            + list((res.get("info") or {}).get("made", []))
        all_loose.extend(res.get("loose", []))
        all_static.extend(res.get("statics", []))
        tree_loose.extend(res.get("loose", []))
        tree_specs.append([sp, lv, X, Y, tp, extra,
                           list(res.get("anchored") or [tp])])
        # WHAT ACTUALLY GOT PHYSICS, per cell, because "the broken branches
        # didn't settle" is invisible in every other number this script
        # prints. Three populations and only ONE of them is simulated:
        #
        #   loose   -> rigid bodies in the settle. `fell_branches` limbs and
        #              any `wood_debris` piece longer than `simulate_above_m`.
        #   seated  -> `wood_debris` pieces UNDER that threshold, laid flat
        #              analytically and never handed to the solver. They look
        #              settled because arithmetic put them flat, not because
        #              anything simulated them.
        #   statics -> colliders only (the snapped spar, the root ball).
        #
        # `fell_branches` needs a WOODY PointInstancer to bake limbs out of,
        # and half this library ships its crown as one plain mesh — those
        # species get `limbs 0` here and their only debris is `wood_debris`.
        # That is the line to read when a species' branches look untouched.
        _nl, _ns = len(res.get("loose") or []), len(res.get("seated") or [])
        print("[tarch]   {0:<18} {1:<8} bodies {2:>3} loose (limbs {3}) / "
              "{4:>3} seated (no physics) / {5} static".format(
                  sp, lv, _nl, int(n_limbs_for(lv)), _ns,
                  len(res.get("statics") or [])))
        # THE SEATED LEAN IS THE DIAGNOSTIC for the whole windthrow path, and
        # it is not recoverable after the export. `tip_tree` bisects it down
        # per species until the crown is just into the turf, so a value stuck
        # at the 46 deg floor means that species' crown is too wide to lie
        # down and it should be in `tornado.NO_UPROOT` — which is exactly how
        # Black_Oak was found. Printed here rather than inferred later.
        if res.get("lean_deg"):
            print("[tarch]   {0:<18} {1:<8} seated at {2:.0f} deg".format(
                sp, lv, res["lean_deg"]))
    print("[tarch] built {0} tree grid cells".format(len(tree_specs)))

    # ---- settle the whole grid once, WITH A WIND ---------------------------
    for spec in house_specs:
        for q in spec[5]:
            pr = stage.GetPrimAtPath(q["prim_path"])
            if pr and pr.IsValid() and pr.IsActive():
                all_static.append(q["prim_path"])
    all_static.append("/World/arch_ground")
    for _ in range(20):
        omni.kit.app.get_app().update()
    if all_loose:
        # HULLS, NOT DECOMPOSITION, AND THAT IS A REAL DIFFERENCE FROM THE
        # FIRE BAKE — which passes `convexDecomposition` for every tree body.
        #
        # It needs it: the burnt path TOPPLES a bole, and a whole branching
        # trunk hulls to a 20 m blob so it comes to rest balanced on its own
        # limb tips, floating. The wind path never topples anything. A
        # windthrown tree is `tip_tree` — a transform on the intact prim, not
        # a simulated body — so the only loose tree geometry here is debris
        # sticks and `fell_branches` limbs, and a stick is convex to within
        # its own bark.
        #
        # Copying the fire bake's map cost 20+ MINUTES on a settle that takes
        # 61 SECONDS with hulls. The limbs are branchy enough to hit
        # `ConvexDecompositionTask: polygon limit reached`, and the hull sets
        # that come out of that make every solver step crawl — the process
        # sits at 30% of one core, logging nothing, looking hung. That log
        # line is the entire diagnosis; there is no error.
        approx_map = {}
        bias = wind_flow.throw_bias(0.0, 0.0, THROW_MPS)
        print("[tarch] settling {0} bodies with a {1:.1f} m/s bias toward +X"
              .format(len(all_loose), THROW_MPS))
        sinfo = settle.run(stage, all_loose, all_static,
                   # FAR MORE STEPS THAN THE FIRE BAKE'S 420, and 700 was
                   # still not enough: measured on the first full run, all
                   # 700 were consumed and 182 of 7,219 bodies were STILL
                   # MOVING when it baked — frozen mid-flight, which is the
                   # one failure `bake_result` can produce and the wildfire
                   # skill warns about. A thrown fragment has a trajectory as
                   # well as a fall, and at 9 m/s into 0.16 damping it is
                   # airborne for a long time. `run` treats `steps` as a
                   # CEILING with an early exit once nothing is moving, so a
                   # generous budget costs nothing on the cells that settle
                   # quickly. Watch the "STILL MOVING" line: if it is not
                   # zero, this number is too low.
                   steps=SETTLE_STEPS, kick=0.15,
                   rng=random.Random(SEED), bake_result=not KEEP_PHYSICS,
                   approx_map=approx_map,
                   bias=bias,
                   # `steps` IS NOW A TARGET, NOT A CEILING. `converge=True`
                   # lets the throw phase run past it, to SETTLE_MAX_STEPS,
                   # for as long as the count of moving bodies is still
                   # falling — and gives up early, loudly, when it is not,
                   # because at that point more steps are not the fix.
                   converge=True, max_steps=SETTLE_MAX_STEPS,
                   # THE QUIET PHASE is what actually lands the stragglers.
                   # The throw is ballistic: at 9 m/s with 0.16 damping a
                   # fragment is airborne for a long time, and whatever is
                   # still in the air when the budget expires gets BAKED
                   # there. These steps run with the damping raised to 1.6
                   # and the cap dropped to 6 m/s — no new impulse, so the
                   # downwind lean the bias bought is untouched; pieces
                   # simply come down and stop.
                   quiet_steps=SETTLE_QUIET_STEPS,
                   # A REAL PHYSX HALF-SPACE AT z=0, plus CCD on every body.
                   # `/World/arch_ground` is a four-vertex quad cooked as a
                   # triangle mesh — infinitely thin, and a fragment at
                   # ~20 m/s covers 0.33 m per step, more than its own
                   # thickness, so it can be wholly through the mesh by the
                   # end of a step and never generate a contact. That is the
                   # 21-42% of every wrecked archetype that finished BELOW
                   # the world, to -2.9 m. A half-space cannot be passed
                   # through: being under it is penetration, not escape.
                   ground_plane_z=0.0, ccd=True,
                   # And the belt to that braces: anything that still ends up
                   # under grade is lifted back onto it — before the quiet
                   # phase (so it re-settles onto the pile) and again after
                   # the bake (so nothing under the lawn can ship).
                   floor_z=0.0,
                   # OFF BY DEFAULT: hulls for everything. Set SETTLE_DECOMP_M
                   # if big pieces still float — see the header.
                   decompose_larger_than=(SETTLE_DECOMP_M or None),
                   # BOTH CEILINGS MUST MOVE WITH THE BIAS or it is silently
                   # clamped: `maxLinearVelocity` defaults to 4 m/s and
                   # `linearDamping` to 0.55, which between them turn a 9 m/s
                   # throw into a 4 m/s one that has lost half its speed
                   # within a second. See `settle.prepare`.
                   max_speed=max(6.0, THROW_MPS * 2.2), damping=0.16)
        # THE SETTLE'S VERDICT IS THE BAKE'S VERDICT. `settle.run` has
        # already printed a banner if anything is wrong; this decides what
        # to do about it. Default is to export anyway (a flawed library
        # beats no library, and `bake._reseat_roots` still gets its go at
        # the geometry), with BAKE_STRICT=1 for the run you would rather
        # lose than ship.
        if sinfo.get("faults"):
            print("[tarch] " + "!" * 68)
            print("[tarch] !! THE SETTLE DID NOT FINISH CLEANLY — these "
                  "archetypes will contain")
            print("[tarch] !! debris frozen in mid-air and/or below grade:")
            for f in sinfo["faults"]:
                print("[tarch] !!   - " + f)
            print("[tarch] !! Knobs: SETTLE_STEPS / SETTLE_MAX_STEPS / "
                  "SETTLE_QUIET_STEPS / SETTLE_DECOMP_M")
            print("[tarch] " + "!" * 68)
            if BAKE_STRICT:
                raise SystemExit(
                    "[tarch] BAKE_STRICT=1 and the settle did not converge; "
                    "refusing to export. Re-run with a bigger "
                    "SETTLE_MAX_STEPS / SETTLE_QUIET_STEPS, or with "
                    "BAKE_STRICT=0 to export it anyway.")
        else:
            print("[tarch] settle AT REST: nothing moving, nothing below "
                  "grade at bake time")
    for _ in range(10):
        omni.kit.app.get_app().update()

    # ---- KEEP_PHYSICS: pause here, then bake IN PLACE when told to --------
    #
    # THE WHOLE POINT IS THAT THE APPROVED POSE IS THE ONE THAT SHIPS. Running
    # the settle again to export it — even with the same seed and the same
    # knobs — is a DIFFERENT PILE: measured on two back-to-back runs of this
    # script with identical arguments, one finished with 0 bodies still moving
    # and 0 clamped, the next with 1 still moving and 595 clamped. A solver
    # stepped by a live app does not reproduce; whatever was on screen when
    # somebody said "that looks right" is the only copy of it there is.
    #
    # So this mode does not export on a timer and does not exit. It PAUSES,
    # holds the stage, and waits for a trigger file to appear. When it does,
    # `settle.bake` freezes every body exactly where it is standing, physics
    # comes off, and the ordinary export/audit/manifest path runs on THAT
    # stage. Press play in the meantime and it settles further; the bake takes
    # wherever it has got to.
    if KEEP_PHYSICS:
        # PAUSE, NOT STOP. `timeline.stop()` rewinds every rigid body to its
        # AUTHORED transform, which for this grid is the pre-settle pose —
        # the logs back up in the air where `wood_debris` threw them. Pause
        # leaves the pile exactly where the solver put it.
        tl = omni.timeline.get_timeline_interface()
        try:
            tl.pause()
        except Exception as exc:
            print("[tarch] could not pause the timeline: {0}".format(exc))
        for _ in range(10):
            omni.kit.app.get_app().update()
        n_static_bodies = len(all_static)
        print("\n" + "=" * 72)
        print("TORNADO TREE GRID — PHYSICS LEFT ON, NOTHING WRITTEN")
        print("  {0} rigid bod(ies) still simulated, {1} static collider(s)"
              .format(len(all_loose), n_static_bodies))
        print("  every debris piece is a rigid body"
              if SIM_ALL_DEBRIS else
              "  only pieces over 0.8 m are rigid bodies "
              "(set SIM_ALL_DEBRIS=1 for all of them)")
        print("  the timeline is PAUSED on the settled pile.")
        print("    PLAY   continues from here — a pile at rest should barely "
              "move, and anything that")
        print("           drops when you press it was still in the air when "
              "the budget ran out.")
        print("    STOP then PLAY replays the whole settle from the start.")
        print("  the trees are on a {0:.0f} m grid starting at y={1:.0f}; "
              "nothing has been exported, so".format(GRID, y0))
        print("  the archetype library on disk is untouched.")
        print("  BAKE IN PLACE when it looks right:")
        print("    touch {0}".format(BAKE_TRIGGER))
        print("  ...and this run will freeze the pile exactly as it stands "
              "and write the library.")
        print("=" * 72 + "\n")
        app = omni.kit.app.get_app()
        while simulation_app.is_running():
            app.update()
            if not os.path.exists(BAKE_TRIGGER):
                continue
            try:
                os.remove(BAKE_TRIGGER)
            except OSError:
                pass
            print("[tarch] BAKE IN PLACE requested — freezing {0} body(s) "
                  "where they stand".format(len(all_loose)))
            tl.pause()
            for _ in range(5):
                app.update()
            # FREEZE, DO NOT RE-SIMULATE. `settle.bake` reads each body's
            # current local-to-world and re-authors it as translate/orient/
            # scale, then switches the physics off. Nothing steps in between.
            try:
                n_frozen = settle.bake(stage, [
                    stage.GetPrimAtPath(q) for q in all_loose
                    if stage.GetPrimAtPath(q)
                    and stage.GetPrimAtPath(q).IsValid()])
                print("[tarch] froze {0} body(s)".format(n_frozen))
            except Exception as exc:
                print("[tarch] freeze FAILED: {0}".format(exc))
                continue
            for _ in range(10):
                app.update()
            break
        else:
            simulation_app.close()
            return
        if not simulation_app.is_running():
            simulation_app.close()
            return

    # ---- export every archetype, re-centred to the origin ------------------
    records, miss, audits = [], 0, []

    def _safe_export(paths, out, rec, meta, drop=False, reseat=False,
                     reseat_first=True, reseat_freeze=None):
        est = {}
        try:
            if bake.export_object(stage, None, paths, out, recenter=rec,
                                  drop_to_ground=drop, reseat=reseat,
                                  reseat_first=reseat_first,
                                  reseat_freeze=reseat_freeze,
                                  stats_out=est):
                m, ok, ms = bake.validate(out)
                records.append(dict(usd=os.path.abspath(out), meshes=m,
                                    bound_missing=ms, **meta))
                # THE EXPORT'S Z DECISIONS, WHICH ARE OTHERWISE INVISIBLE.
                # `seat_rz` is the drop datum — the first root's own world
                # min-z. A LARGE NEGATIVE VALUE IS THE FLOATING-DEBRIS BUG:
                # it used to be subtracted from every mesh in the file and
                # therefore lifted correctly-seated debris by |rz| (measured
                # 0.53 m on the Shumard oaks). It is now compensated back out
                # for everything but the object, so a big number here is
                # information rather than a fault — but it is the number to
                # look at first when debris sits off the ground.
                if abs(est.get("seat_rz", 0.0)) > 0.05 \
                        or est.get("seat_lifted"):
                    print("[tarch]   seat  {0:<34} datum {1:+.3f} m, "
                          "{2} root(s) moved, worst {3:.3f} m".format(
                              os.path.basename(out)[:34],
                              est.get("seat_rz", 0.0),
                              est.get("seat_lifted", 0),
                              est.get("seat_lift_max", 0.0)))
                if ARCH_AUDIT:
                    try:
                        au = bake.audit_archetype(out)
                    except Exception as exc:
                        au = None
                        print("[tarch] audit failed for {0}: {1}".format(
                            os.path.basename(out), exc))
                    if au:
                        au["name"] = os.path.splitext(os.path.basename(out))[0]
                        au["level"] = meta.get("level")
                        audits.append(au)
                return ms
        except Exception as exc:
            print("[tarch] export FAILED for {0}: {1}".format(
                os.path.basename(out), exc))
        return 0

    for st, lv, X, Y, parent, pls, frags in house_specs:
        # RESEAT, NOT DROP. A wrecked house was authored on the harness ground
        # at z=0 and that IS where it belongs, so the datum must not move —
        # `drop_to_ground` would redefine it from the first root, which for a
        # house is one module of many. What the house needs is the other half
        # of that pass: put its individual FRAGMENTS back on the pile. Two
        # populations, both measured on the previous bake:
        #
        #   * 21-42% of the meshes in every wrecked house sat BELOW -0.05 m,
        #     whole-object `z_min` down to -2.9 m — the solver tunnelling them
        #     through a four-vertex ground quad at ~20 m/s;
        #   * 11-29% of the meshes in a `leveled` or `partial_collapse` house
        #     had NOTHING under them within 0.10 m, median gap 0.19 m and a
        #     tail to 2.6 m — `settle.run`'s step budget running out with
        #     bodies still in flight.
        #
        # Both are pure geometry to correct and neither needs re-simulating.
        # See `bake._reseat_roots`.
        miss += _safe_export(
            [q["prim_path"] for q in pls] + frags,
            os.path.join(OUT_DIR, "house_{0}_{1}.usd".format(st, lv)),
            (X, Y, 0.0), dict(kind="house", style=st, level=lv), reseat=True)
    for sp, lv, X, Y, tp, extra, anchored in tree_specs:
        # SEAT THE DEBRIS, FREEZE THE TREE. Measured 2026-08-27 with a
        # bare-`pxr` probe over the shipped library: `tree_Shumard_Oak_snapped`
        # 39.5% of meshes with nothing under them, `_limbed` 35.3%, and three
        # independent seated sticks in each sitting at EXACTLY 0.53 m. A solver
        # does not leave unrelated bodies at the same height to the centimetre;
        # a single uniform offset does, and `drop_to_ground` was it — the
        # Shumard's own base dips 0.53 m below grade, `rz` went negative, and
        # subtracting it lifted every stick `wood_debris` had already placed
        # analytically at `ground_z - 0.001`. Every species whose base is at
        # its origin was clean, and BOTH `_NO_DROP` levels were clean at every
        # species: the float appeared exactly where the drop was applied.
        #
        # `export_object` now compensates that datum shift back out for
        # everything except the object itself, and `reseat` runs the same pure-
        # geometry pass the houses get. `reseat_first=False` keeps it off the
        # TREE: `tip_tree` presses the crown into the turf on purpose
        # (`seat_band = (-1.1, -0.15)`) and `wind_tree` lifts a fallen base by
        # the root plate's radius, both of which read as "sank through the
        # floor" and would stand every windthrown trunk back up on the lawn.
        miss += _safe_export(
            [tp] + extra,
            os.path.join(OUT_DIR, "tree_{0}_{1}.usd".format(sp, lv)),
            (X, Y, 0.0), dict(kind="tree", species=sp, level=lv),
            drop=(lv not in _NO_DROP), reseat=True, reseat_first=False,
            reseat_freeze=anchored)
    man_path = os.path.join(OUT_DIR, "archetypes.json")
    merged = merge_manifest(man_path, records, KINDS)
    bake.write_manifest(man_path, merged)
    dt = time.time() - t_build0
    nh = sum(1 for r in merged if r["kind"] == "house")
    nt = sum(1 for r in merged if r["kind"] == "tree")
    sz = sum(os.path.getsize(r["usd"]) for r in records
             if os.path.exists(r["usd"])) / 1e6
    nm = sum(r["meshes"] for r in records)

    if audits:
        print("\n[tarch] FLOATING-DEBRIS AUDIT of the files just written "
              "(lower bounds — see audit_archetype)")
        print("[tarch]   {0:<34}{1:>7}{2:>16}{3:>15}{4:>8}".format(
            "archetype", "meshes", "airborne", "sunk", "z_min"))
        for au in sorted(audits, key=lambda q: -q["airborne"] / max(
                1, q["meshes"])):
            m = max(1, au["meshes"])
            print("[tarch]   {0:<34}{1:>7}{2:>9} {3:5.1f}%{4:>8} {5:5.1f}%"
                  "{6:>8.2f}{7}".format(
                      au["name"][:34], au["meshes"], au["airborne"],
                      100.0 * au["airborne"] / m, au["sunk"],
                      100.0 * au["sunk"] / m, au["z_min"],
                      "" if au.get("level") in _PILE_LEVELS
                      else "   (has legitimate air: not a pile)"))
        pile = [au for au in audits if au.get("level") in _PILE_LEVELS]
        pm = sum(au["meshes"] for au in pile)
        pa = sum(au["airborne"] for au in pile)
        ps = sum(au["sunk"] for au in pile)
        if pm:
            print("[tarch]   PILE LEVELS ONLY ({0}): {1} of {2} meshes with "
                  "nothing under them ({3:.1f}%), {4} below grade ({5:.1f}%)"
                  .format("/".join(_PILE_LEVELS), pa, pm,
                          100.0 * pa / pm, ps, 100.0 * ps / pm))
            # THE NUMBERS THIS RUN IS TRYING TO BEAT, kept here so the next
            # person does not have to find the skill to know whether the bake
            # improved anything: 11-29% airborne and 21-42% below grade,
            # measured on the library baked before the settle was fixed.
            if 100.0 * pa / pm > 8.0 or ps:
                print("[tarch]   " + "!" * 64)
                print("[tarch]   !! STILL FLOATING. Before the settle fix the "
                      "pile levels ran 11-29%")
                print("[tarch]   !! airborne and 21-42% below grade. If this "
                      "run is no better, the")
                print("[tarch]   !! next lever is SETTLE_DECOMP_M=2.5 (hull "
                      "of an L-shaped piece")
                print("[tarch]   !! encloses its own concavity) and then "
                      "SETTLE_QUIET_STEPS.")
                print("[tarch]   " + "!" * 64)
            else:
                print("[tarch]   pile levels are clean by this test "
                      "(<8% airborne, nothing below grade)")

    print("\n" + "=" * 72)
    print("TORNADO ARCHETYPE BAKE  (kinds: {0})".format(",".join(sorted(KINDS))))
    print("  manifest: {0} house + {1} tree archetypes -> {2}"
          .format(nh, nt, OUT_DIR))
    print("  this run: {0} archetype(s), {1} mesh(es), {2:.0f} MB, "
          "{3:.0f} s, {4} unresolved material(s)"
          .format(len(records), nm, sz, dt, miss))
    print("  throw     {0:.1f} m/s toward +X (assembly yaws each into the "
          "track)".format(THROW_MPS))
    print("=" * 72 + "\n")

    app = omni.kit.app.get_app()
    while simulation_app.is_running():
        app.update()
    simulation_app.close()


if __name__ == "__main__":
    main()
