#!/usr/bin/env python
"""
Assemble a 500 x 500 m suburb with a TORNADO TRACK cut across it — by
reference, with no live fracture and no settle.

    ARCH_DIR=/isaac-sim/AirStack/scene_gen/assets/archetypes_tornado \
    SCENE_CONFIG=suburb_tornado \
    ISAAC_SIM_SCRIPT_NAME=suburb_tornado_launch_script.py \
    airstack up isaac-sim

Bake the archetype library FIRST with `bake_tornado_archetypes_launch_script.py`
into the same `ARCH_DIR`. Nothing here is fractured or settled: the expensive
work is done once per (style, level) and this script references it.

WHAT THIS SCENE IS
------------------
The wind counterpart of `suburb_assemble_launch_script.py`, against the same
generator and the same assembly pattern. `tornado.jpeg` in the repo root is
the reference photograph, and everything below is one of the features in it:

  1. A CORRIDOR of structural damage, graded from roofs-stripped at the edge
     to swept slabs on the centreline, with intact green suburb either side.
  2. Everything BLOWN ONE WAY. Wrecked houses trail plank debris downtrack;
     fallen trees point the same way; the corridor between the lots is
     covered in boards that came from somewhere else.
  3. BROWN SCOURED GROUND, darkest on the centreline and fading out to the
     path edge, where the turf was peeled off — and STANDING ON IT, the earth
     that came out: the spoil of the grooves the suction vortices cut, low
     windrows, heaps of subsoil, mats of turf rolled up like carpet at the
     edge of the scour, and thrown clods. The mud overlay is the colour and
     `disaster.scour_relief` is the SHADOW; without the second the track
     photographs as a stain on a billiard table.
  4. PALE debris, not dark. A tornado exposes the inside of buildings —
     framing, sheathing, the unpainted back of everything — and that
     photographs light against grass. A burn scar photographs dark. Getting
     that backwards is the fastest way to build the wrong disaster.
  5. Roads, driveways, pools and slabs COMING THROUGH INTACT. They are in the
     ground or full of water; the overlay sits under the asphalt on the z
     ladder so the streets stay legible straight through the track, which is
     exactly what the aerial shows.

NO PEOPLE, AND NO FIRE. Survivors are a later pass — nothing here calls
`disaster.people`. Nothing here calls `disaster.fire`, `damage.soot_materials`,
`damage.char_materials` or `damage.scorched_material` either; see
`disaster/wind_flow.py` for why every one of those is a fire pass with no
no-fire mode.

THE KIT FLAG IS LOAD-BEARING, TWICE
-----------------------------------
The scour overlay is a translucent OmniPBR whose opacity is a FRACTIONAL
CUTOUT, and RTX Real-Time discards fractional cutout unless
`/rtx/raytracing/fractionalCutoutOpacity` is on. It has to be a command-line
flag (below) AND re-asserted after the Pegasus environment stage is loaded,
because loading a stage with authored render settings resets the property to
its default. The symptom of missing either is "I don't see the ground at all".
Full account in the `build-wildfire-scenes` skill.

Env knobs:

    ARCH_DIR      archetype library (default `scene_gen/assets/archetypes_tornado`)
    SCENE_CONFIG  preset (default `suburb_tornado`)
    TOR_SEED      layout seed override for the damage draws (default 11)
    TOR_PLANKS    boards per wrecked house (default 140); 0 disables the field
    TOR_TRACK_PER100  boards per 100 m2 of corridor (default 4.5)
    TOR_GROUND    0 disables the mud overlay AND the 3D relief that rides it
    MUD_*         overlay tuning — see `disaster.tornado.knobs_from_env`
    SCOUR_RELIEF  0 disables the 3D relief alone, leaving the flat overlay
    SCOUR_*       relief tuning — see `disaster.scour_relief.knobs_from_env`;
                  SCOUR_HEIGHT is the one to reach for first
    SNAP_DIR      write viewport PNGs here (must be under the mounted log dir)
    TOR_MIN_TIPPED  force at least this many cars onto a side/roof/nose. 0 is
                  off and is the only honest setting for a measured scene.
    PEOPLE_SNAPS  photograph this many casualties close in (0 = off). Named
                  after the ground-truth record, so the frame and the label
                  can be read against each other.
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
    "extra_args": ["--/rtx/raytracing/fractionalCutoutOpacity=true",
                   "--/rtx/pathtracing/fractionalCutoutOpacity=true"],
})

from isaacsim.core.utils.extensions import enable_extension

enable_extension("omni.kit.window.script_editor")

import omni.kit.app
import omni.timeline
import omni.usd
from omni.isaac.core.world import World
from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade

_ISAAC_SIM_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
_SCENE_GEN_DIR = os.path.normpath(
    os.path.join(_ISAAC_SIM_DIR, "..", "..", "scene_gen"))
sys.path.insert(0, os.path.join(_ISAAC_SIM_DIR, "utils"))
sys.path.insert(0, _SCENE_GEN_DIR)

import numpy as np                                             # noqa: E402
import scene_generator as sg                                   # noqa: E402
from scene_prep import add_sky, get_stage_meters_per_unit      # noqa: E402
from scene_generator import resolve_sky                        # noqa: E402
import suburb_scene as ss                                      # noqa: E402
from suburb_scene import generate_suburb_on_stage              # noqa: E402
from compile_disaster import load_scene_config                 # noqa: E402
from disaster import ground, planks, wind_flow                 # noqa: E402
from disaster import scour_relief as srl                       # noqa: E402
from disaster import tornado_people as tpp                     # noqa: E402
from disaster import tornado as tn                             # noqa: E402
from detail import modular_house as mh                         # noqa: E402

PARENT = "/World/stage/generated"
SCENE_CONFIG = os.environ.get("SCENE_CONFIG", "suburb_tornado")
SEED = int(os.environ.get("TOR_SEED", "11"))
ARCH_DIR = os.environ.get(
    "ARCH_DIR", os.path.join(_SCENE_GEN_DIR, "assets", "archetypes_tornado"))
ENV_URL = SIMULATION_ENVIRONMENTS["Default Environment"]
N_PLANKS = int(os.environ.get("TOR_PLANKS", "140"))
# 4.5 BOARDS PER 100 m2 OF CORRIDOR, and the number is lower than it sounds
# because the density is scaled by `intensity ** 1.4`: at the path edge it is
# almost nothing and only the core gets the full rate. Measured on this preset,
# 1.6 put one board every ~13 m across the track, which reads as litter rather
# than as a debris field; 4.5 puts one every ~5 m in the core and still leaves
# the shoulders sparse. Cheap either way — the whole field is five meshes.
TRACK_PER100 = float(os.environ.get("TOR_TRACK_PER100", "4.5"))
DO_GROUND = os.environ.get("TOR_GROUND", "1").strip().lower() not in (
    "0", "false", "no")
DO_PEOPLE = os.environ.get("TOR_PEOPLE", "1").strip().lower() not in (
    "0", "false", "no")
PEOPLE_JSON = os.environ.get(
    "PEOPLE_JSON", os.path.join(ARCH_DIR, "humans_{0}.json".format(SEED)))
SNAP_DIR = os.environ.get("SNAP_DIR", "")
# HOW MANY CASUALTIES TO PHOTOGRAPH INDIVIDUALLY, close in. 0 is off; the
# frames are cheap (two viewport captures each) and they are the only way to
# review the thing this pass is actually judged on.
PEOPLE_SNAPS = int(os.environ.get("PEOPLE_SNAPS", "0"))
# A FLOOR UNDER THE TIPPED-CAR COUNT, FOR REVIEW SCENES. 0 leaves the
# displacement ladder exactly as `tornado.car_pose` draws it; any higher value
# forces the highest-intensity in-track cars over until the count is met, and
# the banner says how many were forced. See the block after the car loop.
MIN_TIPPED = int(os.environ.get("TOR_MIN_TIPPED", "0"))
if SNAP_DIR:
    os.makedirs(SNAP_DIR, exist_ok=True)


def _rigged_humans(config, rp):
    """The RIGGED RenderPeople, which are the only ones a pose can bind to.

    `posed_standing` assets are static meshes frozen in one attitude — binding
    a `walk` or `crouch` pose to one does nothing at all and the figure ships
    in whatever attitude it was authored in. Same selection
    `disaster.people.build_ctx` makes, with the same fallback.
    """
    from suburb_scene import _raw_pool

    pools = rp[1]
    raw = _raw_pool(config, "humans")
    rigged = pools.load_tagged(raw, "rigged")
    if not rigged:
        posed = pools.load_tagged(raw, "posed_standing")
        rigged = [u for u in pools.load(raw) if u not in posed]
    return rigged

TREE_SPECIES = {
    "Black_Oak": "airstack://scene_gen/assets/aec/tower/Assets/Vegetation/Black_Oak/Black_Oak.usd",
    "Shumard_Oak": "airstack://scene_gen/assets/aec/tower/Assets/Vegetation/Shumard_Oak/Shumard_Oak.usd",
    "Douglas_Fir": "airstack://scene_gen/assets/aec/brownstone/Assets/Vegetation/Trees/Douglas_Fir.usd",
    "Largetooth_Aspen": "airstack://scene_gen/assets/aec/brownstone/Assets/Vegetation/Trees/Largetooth_Aspen.usd",
    "Common_Apple": "airstack://scene_gen/assets/aec/tower/Assets/Vegetation/Common_Apple/Common_Apple.usd",
    "American_Beech": "airstack://scene_gen/assets/aec/brownstone/Assets/Vegetation/Trees/American_Beech.usd",
}

# WHICH LEVELS ARE YAWED TO THE TRACK RATHER THAN TO THE STREET.
#
# Every archetype is baked with its debris thrown toward local +X, so the yaw
# a reference is given decides which way its wreckage points. For a house that
# still reads as a house, the street wins — a building facing the wrong way is
# the most obvious defect a suburb can have, and its debris field is small
# enough that its direction is not what you are looking at. For a pile, the
# track wins, because nobody can tell which way a pile was facing and where
# its material went is the entire read.
_TRACK_YAWED = ("leveled", "swept")
# `snapped` JOINED THIS LIST once the archetype reach was measured. A snapped
# tree still has a standing stub, so the first instinct is to treat it like a
# house that still reads as a house and leave it on its own bearing — but the
# archetype throws THIRTY-NINE logs 20-25 m toward local +X, and yawing it to
# the tree's arbitrary layout bearing points each tree's debris a different
# way. A stand whose fallen material has no common direction is the one thing
# this scene cannot afford: the corridor's whole read is that everything went
# ONE way. Same rule the houses already follow — a pile is yawed to the track,
# because nobody can tell which way a pile was facing and where its material
# went is the entire point. `limbed` reaches as far (up to 26.7 m) and is
# deliberately NOT here: it is a standing tree with its crown on, its canopy
# shape is what you see, and its dropped limbs are a minority of the picture.
_TREE_TRACK_YAWED = ("leaning", "fallen", "snapped")


def _ref(stage, dst, usd, x, y, yaw, ssf, scale=1.0, instance=True):
    """Reference `usd` at a pose. Same helper as the wildfire assembly."""
    prim = stage.DefinePrim(Sdf.Path(dst), "Xform")
    if not prim.GetReferences().AddReference(usd):
        return False
    xf = UsdGeom.Xformable(prim)
    xf.AddTranslateOp().Set(Gf.Vec3d(x * ssf, y * ssf, 0.0))
    xf.AddRotateZOp().Set(float(yaw))
    if scale != 1.0:
        xf.AddScaleOp().Set(Gf.Vec3f(scale, scale, scale))
    # INSTANCE IT. The same archetype is referenced many times across the
    # plate, and un-instanced trees were the 186M-point wall that OOM'd Isaac
    # on the wildfire plat. Transform ops on the instance ROOT are still
    # legal; only authoring INSIDE the referenced content is forbidden, which
    # is why a recoloured row home has to opt out.
    if instance:
        prim.SetInstanceable(True)
    return True


# `_toss` MOVED TO `disaster.tornado.toss_prim`. It was here first, then the
# people preview needed exactly the same seating maths to stand a toppled car
# up correctly — and a second copy of a routine whose whole point is that it
# MEASURES rather than guesses is the fastest way to end up with two different
# guesses again. Aliased rather than renamed at the call sites so the diff
# stays about the move.
_toss = tn.toss_prim


def wait_for_stage(stage, timeout_s=20.0):
    app = omni.kit.app.get_app()
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        if stage and stage.GetPrimAtPath("/World").IsValid():
            return True
        app.update()
    return False


def main():
    omni.timeline.get_timeline_interface().stop()
    pg = PegasusInterface()
    pg._world = World(**pg._world_settings)
    pg.load_environment(ENV_URL)
    stage = omni.usd.get_context().get_stage()
    wait_for_stage(stage)
    for name in ("GroundPlane", "Environment"):
        p = stage.GetPrimAtPath("/World/" + name)
        if p and p.IsValid():
            p.SetActive(False)

    config = load_scene_config(SCENE_CONFIG)
    _, ssf = get_stage_meters_per_unit(stage)

    # 1) LAYOUT + CHEAP DETAIL, houses/trees returned as instances -----------
    t0 = time.time()
    binfo = {}
    placements = generate_suburb_on_stage(stage, config, parent_path=PARENT,
                                          scene_scale_factor=ssf,
                                          info_out=binfo, assembly=True)
    add_sky(stage, resolve_sky(config))
    houses = binfo.get("house_instances", [])
    trees = binfo.get("tree_instances", [])
    print("[tornado] layout in {0:.0f}s: {1} house + {2} tree instance(s)"
          .format(time.time() - t0, len(houses), len(trees)))

    if not os.path.isdir(ARCH_DIR):
        print("[tornado] ARCH_DIR {0} does not exist — run "
              "bake_tornado_archetypes_launch_script.py first".format(ARCH_DIR))
    arch = {os.path.splitext(f)[0]: os.path.join(ARCH_DIR, f)
            for f in (os.listdir(ARCH_DIR) if os.path.isdir(ARCH_DIR) else [])
            if f.endswith(".usd")}

    # 2) THE TRACK -----------------------------------------------------------
    tcfg = tn.resolve_cfg(config)
    region = tuple(binfo.get("region") or (-250, -250, 250, 250))
    span = max(region[2] - region[0], region[3] - region[1])
    # ONE rng for the field, a DIFFERENT one for the per-object draws. The
    # field's noise is numpy and the ladders are stdlib; sharing a generator
    # between them would make a re-tune of one silently re-roll the other.
    inten = tn.intensity_field(tcfg, region, np.random.default_rng(SEED + 23))
    throw_deg = float(tcfg["heading_deg"]) + float(tcfg["curl_deg"])
    drng = random.Random(SEED + 5)
    summ = tn.summarise(tcfg, region, np.random.default_rng(SEED + 23))
    print("[tornado] track {0:.0f} m wide toward {1:.0f} deg through "
          "({2:.0f}, {3:.0f}); {4:.1%} of the plate in the path, debris "
          "toward {5:.0f} deg".format(
              tcfg["width_m"], tcfg["heading_deg"], tcfg["origin_m"][0],
              tcfg["origin_m"][1], summ["in_path_frac"], throw_deg))

    # 3) REFERENCE A HOUSE ARCHETYPE PER INSTANCE ----------------------------
    fp_by_style = {e["style"]: max(e["w"], e["d"])
                   for e in ss.modular_catalogue(config)}
    UsdGeom.Scope.Define(stage, Sdf.Path(PARENT + "/inst"))
    n_h = miss_h = 0
    htally = {}
    pal_jobs = []
    # (x, y, footprint, intensity, level, PALETTE). The palette is what makes
    # the debris off this house look like THIS HOUSE: `planks` stamps its
    # `siding` and `deck` pieces with the wall and roof material names and
    # binds the same material prims the standing houses wear.
    wrecks = []
    intact = []          # standing houses — the neighbour-dig prior
    # ...and every house that still has a BUILDING on its lot, wrecked or not.
    # Not the same list as `intact`: a house with its roof off is damaged but
    # it still has a floor, and a 0.4 m heap of subsoil authored in its living
    # room comes through it. Only `leveled` and `swept` leave ground.
    standing = []        # (x, y, footprint)
    for i, h in enumerate(houses):
        it = float(inten(h["x"], h["y"]))
        level = tn.house_level_for_intensity(it, drng)
        htally[level] = htally.get(level, 0) + 1
        if level == "pristine":
            intact.append((h["x"], h["y"]))
        if level not in ("leveled", "swept"):
            standing.append((h["x"], h["y"],
                             fp_by_style.get(h["style"], 12.0)))
        key = "house_{0}_{1}".format(h["style"], level)
        usd = arch.get(key) or arch.get("house_{0}_pristine".format(h["style"]))
        yaw = (throw_deg + drng.uniform(-14.0, 14.0)
               if level in _TRACK_YAWED else h["yaw"])
        if not usd:
            # BUILD IT LIVE INSTEAD OF LEAVING A HOLE. A missing archetype
            # used to `continue`, which is the worst possible failure mode for
            # a scene you are about to look at: the plat comes up with the
            # streets, the debris and the scour all present and the BUILDINGS
            # simply absent, and nothing on screen says why. An empty lot also
            # reads as a deliberate feature — this is a disaster scene — so it
            # is not even obviously wrong.
            #
            # The kit can always build the house; what it cannot do cheaply is
            # WRECK it, which is the whole reason the archetypes exist. So the
            # fallback is an intact house at the right pose, counted and
            # reported separately, and its damage is carried entirely by the
            # plank field. Good enough to fly, obvious in the banner, and it
            # keeps a half-baked library usable.
            miss_h += 1
            try:
                _fp = "{0}/inst/h_{1}".format(PARENT, i)
                UsdGeom.Scope.Define(stage, Sdf.Path(_fp))
                _pls = mh.build_building(h["style"], h["x"], h["y"], yaw,
                                         random.Random(SEED + i),
                                         category="house")
                _hpal = h.get("palette") or mh.STYLES[h["style"]].get("palette")
                if _hpal:
                    for _q in _pls:
                        _q["palette"] = _hpal
                sg.apply_placements(stage, _pls, _fp, ssf)
                mh.apply_palette(stage, _pls, _fp)
                n_h += 1
                if level != "pristine":
                    wrecks.append((h["x"], h["y"],
                                   fp_by_style.get(h["style"], 12.0), it,
                                   level, _hpal))
            except Exception as _exc:
                if miss_h == 1:
                    print("[tornado] live house fallback FAILED: {0}"
                          .format(_exc))
            continue
        # Row homes are recoloured per unit, and USD forbids authoring inside
        # an instance — so a terrace opts out of instancing. A few dozen
        # prims against a plate's worth; the trees are what the budget is for.
        _pal = h.get("palette")
        _recolour = bool(h.get("row")) and bool(_pal)
        _hp = "{0}/inst/h_{1}".format(PARENT, i)
        if _ref(stage, _hp, usd, h["x"], h["y"], yaw, ssf,
                instance=not _recolour):
            n_h += 1
            if _recolour:
                pal_jobs.append({"prim_path": _hp, "palette": _pal,
                                 "category": "house"})
            if level != "pristine":
                # THE PALETTE THE ARCHETYPE WAS BAKED WITH. The bake
                # (`bake_tornado_archetypes_launch_script`, which walks
                # `mh.STYLES`) dresses each style with its OWN default palette
                # before it fractures it, so a `cottage` archetype is wearing
                # `wood_white` whatever this plat wrote on the placement. Read
                # the style default first and only fall back to the placement's
                # own palette — which is the right answer for the live-built
                # fallback above and the wrong one here.
                wrecks.append((h["x"], h["y"],
                               fp_by_style.get(h["style"], 12.0), it, level,
                               mh.STYLES.get(h["style"], {}).get("palette")
                               or h.get("palette")))
    if pal_jobs:
        try:
            n_pal = mh.apply_palette(stage, pal_jobs, PARENT)
            print("[tornado] row homes: {0} subset(s) recoloured across "
                  "{1} unit(s)".format(n_pal, len(pal_jobs)))
        except Exception as _exc:
            print("[tornado] row-home palette FAILED: {0}".format(_exc))

    # 4) REFERENCE A TREE ARCHETYPE PER INSTANCE -----------------------------
    n_t = miss_t = 0
    t_turned = t_dropped = 0
    ttally = {}
    canopies = []        # (x, y, r) for trees the people pass must stay under

    trng = random.Random(SEED + 71)
    for i, t in enumerate(trees):
        it = float(inten(t["x"], t["y"]))
        sp = t["species"]
        # THE ARCHETYPE HAS TO LAND ON THE PLATE, and until this call it did
        # not. Every tree archetype is baked with its debris thrown toward
        # local +X and reaches up to 26.7 m that way, while `apply_ground`
        # lays its sheet over exactly `region` and nothing beyond it — so any
        # tree within a reach of the boundary throws its logs out over the
        # void, which is what "the logs that are thrown around look like
        # they're floating" was. It cannot be trimmed per placement: the
        # reference below is an INSTANCE and USD forbids authoring inside one.
        # So `tree_level_and_yaw` turns the tree INWARD instead, and only
        # walks the damage ladder down when no yaw on the circle fits — a
        # level drop changes what the scene SAYS about the wind at that point
        # and a fall bearing does not.
        #
        # It also owns the fall-bearing draw that used to live here. A fallen
        # trunk points somewhere and that is the cheapest directional cue in
        # the scene; the jitter is wide because trees go over in the direction
        # their own rooting and neighbours allow as much as in the direction
        # the wind was going, and a stand all pointing within five degrees of
        # each other reads as a logging operation rather than as windthrow.
        level, yaw, tinfo = tn.tree_level_and_yaw(
            it, trng, sp, t["x"], t["y"], region,
            track_yaw_deg=throw_deg, base_yaw_deg=t["yaw"],
            track_yawed=_TREE_TRACK_YAWED)
        ttally[level] = ttally.get(level, 0) + 1
        t_turned += 1 if tinfo["turned_deg"] else 0
        t_dropped += 1 if tinfo["downgraded"] else 0
        # WHAT STILL HAS A CROWN, for the people pass. A casualty under an
        # intact canopy is photographed as leaves — the top-down frame of the
        # 100 m scene's `yard` figure was a tree and nothing else — and a
        # target a camera cannot see is worse than no target. `snapped`,
        # `fallen` and `limbed` trees hide nothing from above, so only these
        # three levels are blockers. The radius is nominal: these are 6-12 m
        # species and a `leaning` one has swung its crown off its own trunk,
        # which is why it gets less rather than the same.
        _cr = {"pristine": 4.2, "limbed": 2.6, "leaning": 3.0}.get(level)
        if _cr:
            canopies.append((t["x"], t["y"], _cr))
        if level == "pristine":
            usd = sg._join_asset_root(TREE_SPECIES.get(sp, ""), "")
            scale = 0.01
        else:
            usd = arch.get("tree_{0}_{1}".format(sp, level))
            scale = 1.0
        if not usd:
            # SAME ARGUMENT AS THE HOUSES: a green tree at the right place is
            # a far better failure than no tree at all, and for vegetation the
            # fallback is free — the species USD is what a `pristine` tree
            # references anyway.
            miss_t += 1
            usd = sg._join_asset_root(TREE_SPECIES.get(sp, ""), "")
            scale, yaw = 0.01, t["yaw"]
            if not usd:
                continue
        if _ref(stage, "{0}/inst/t_{1}".format(PARENT, i), usd,
                t["x"], t["y"], yaw, ssf, scale=scale):
            n_t += 1

    # The resolver measures every asset's real footprint and the pools resolve
    # tag queries; both are needed by the people pass and neither is cheap, so
    # they are built once here rather than per scenario.
    resolver_pools = (sg._make_resolver(config), ss.AssetPools(config))
    drng = random.Random(SEED + 404)

    # 5) STREET FURNITURE, FENCES AND CARS IN THE CORRIDOR -------------------
    #
    # `damage.INCOMBUSTIBLE` IS THE WRONG LIST HERE and it is worth saying so
    # explicitly: it exists to say what does not BURN, and it contains
    # `streetlight` and `sign` because those come through a wildfire visibly
    # untouched. A tornado shears them off. Reusing the fire list would leave
    # a corridor of levelled houses with every street sign standing perfectly
    # upright in it — see `wind_flow.IMMOVABLE` for the list that is right.
    # WHAT THE WIND DOES DEPENDS ON WHAT THE THING IS, and splitting the one
    # list in two is the difference between a corridor that reads as struck and
    # one with a rank of perfectly plumb lamp standards down the middle of it.
    #
    #   CARRIED   light, loose, or made of boards. It goes AWAY: a timber
    #             fence leaves a line of post stubs and its boards somewhere
    #             else, and the boards are already being authored by the plank
    #             field. Deleting is the right model.
    #   FELLED    bolted or founded, and tall. A 5 m steel lamp standard on a
    #             cast base is not carried anywhere by an EF3 — it BENDS OVER,
    #             and a row of them lying downtrack is one of the most
    #             recognisable things in a damage photograph. Deleting one is
    #             the wrong model and it is what the review saw: "why don't we
    #             get street light to fall over".
    #
    # A FELLED item can still be carried at high intensity (the shares below
    # invert), and a CARRIED one can still be left lying — neither list is
    # absolute, they set which outcome is the DEFAULT for that kind of thing.
    CARRIED = ("fence", "trash_can", "bin", "mailbox", "chair",
               "picnic_table", "table", "planter", "cafe_set", "patio",
               "bench")
    FELLED = ("streetlight", "sign", "bus_stop", "play_structure", "swing",
              "seesaw", "bike_rack", "goal", "basket", "hoop")
    BLOWN = CARRIED + FELLED
    # ...AND `prop_kind`, BECAUSE `category` LIES ABOUT YARD PROPS.
    # `detail.suburb_yardplan` charges everything it places to the `plant`
    # budget — that is what its purse is denominated in — so a mailbox, a
    # wheelie bin and a patio table all ship with `category: "plant"` and none
    # of the names above matches any of them. The whole yard-furniture
    # population therefore came through the corridor untouched: every mailbox
    # on the plate standing perfectly upright in a levelled block, which is
    # what the review saw. `prop_kind` is the truthful name and is tested
    # alongside the category.
    frng = random.Random(SEED + 13)
    n_gone = n_down = n_car = n_car_in = n_car_tip = n_car_jam = 0
    car_recs = []        # {x, y, toppled}, the published record
    car_cands = []       # (intensity, placement, prim path, length) in-track
    car_done = set()     # prim paths already moved by the draw
    n_car_forced = 0     # ...and how many `TOR_MIN_TIPPED` put over by hand
    # WHAT A DISPLACED CAR CAN JAM AGAINST, built once outside the loop.
    # `standing` is every house that still has a building on its lot — a
    # WRECKED house is still a heap of material that stops a car, so this is
    # deliberately not `intact` — and `trees` is every tree on the plate,
    # upright or windthrown, because a fallen bole stops a car at least as
    # well as a standing one. Without it a thrown car lands wherever its
    # vector put it, which in an open plat is most often the middle of a lawn:
    # the one vehicle pose that reads as "somebody moved this asset" rather
    # than as wind. `tornado.car_blockers` says why a car does not block
    # itself and why the radii are nominal.
    _pools = resolver_pools[1]
    car_block = tn.car_blockers(
        standing=standing,
        trees=[(t["x"], t["y"]) for t in trees],
        cars=[(float(q.get("x_m", 0.0)), float(q.get("y_m", 0.0)))
              for q in placements if "car" in str(q.get("category", ""))])
    for q in placements:
        path = q.get("prim_path")
        cat = str(q.get("category", ""))
        if not path:
            continue
        it = float(inten(float(q.get("x_m", 0.0)), float(q.get("y_m", 0.0))))
        # THE FRINGE COUNTS FOR STREET FURNITURE, and 0.10 was too high.
        #
        # 0.10 is the right floor for a HOUSE — a dwelling at the very edge of
        # the track keeps its roof — and it is the wrong one for a 5 m lamp
        # standard, which is a lever with a sail on the end and goes over in
        # winds that leave a roof alone. On a 46 m track with `core_frac 0.22`
        # most of the corridor sits at 0.2-0.3, and the handful of lamps a
        # 100 m plate carries mostly sat just OUTSIDE the 0.10 line: the
        # review saw them standing perfectly plumb next to a levelled house
        # ("the street lights still haven't really bent"). Cars keep the old
        # floor, because a car at the fringe genuinely is untouched.
        if it <= (0.04 if "car" not in cat else 0.10):
            continue
        if wind_flow.is_immovable(cat):
            continue
        kind = str(q.get("prop_kind") or "")
        if any(k in cat for k in BLOWN) or (kind and kind in BLOWN):
            # TWO OUTCOMES, NOT ONE, and the second is why a corridor full of
            # upright lamp standards was the review's other complaint.
            #
            # GONE is right for the light stuff and for the core: what a track
            # leaves of a timber fence is a line of post stubs and a lot of
            # boards somewhere else, and the boards are already being authored
            # by the plank field. But deleting is ALL this pass could do, so
            # anything that failed the roll stood there perfectly plumb — and a
            # streetlight is a 5 m steel pole that a tornado bends over rather
            # than removes. A prop that is still standing has to be visibly
            # DOWN.
            #
            # So: one draw against nested thresholds, the same structure
            # `tornado.car_pose` uses for the Paulikas rates and for the same
            # reason — testing a lean only on props that had already survived a
            # removal roll multiplies the two probabilities and leaves the
            # tail standing anyway.
            #
            #   r < p_gone           carried away
            #   r < p_gone + p_down  still here, and over
            #   otherwise            untouched — the corridor edge, where
            #                        partial runs of fence are what makes the
            #                        edge read
            r = frng.random()
            felled = any(k in cat for k in FELLED) or (kind and kind in FELLED)
            if felled:
                # A POLE GOES OVER FIRST AND AWAY SECOND. Even on the
                # centreline most of a lamp standard is still on its base,
                # bent; it takes the very core to shear one off and carry it.
                p_down = min(0.92, 0.55 + 0.40 * it)
                p_gone = 0.55 * it * (1.0 - p_down)
            else:
                p_gone = min(0.90, 0.18 + 0.86 * it)
                p_down = 0.85 * (1.0 - p_gone)
            if r < p_gone:
                pr = stage.GetPrimAtPath(path)
                if pr and pr.IsValid() and pr.SetActive(False):
                    n_gone += 1
            elif r < p_gone + p_down:
                # (the two bands are disjoint and ordered gone-then-down for
                #  both kinds; only the SHARES differ, so a pole is over far
                #  more often than it is missing and a fence panel the reverse)
                # OVER, AND SEATED. `toss_prim` measures the prop's own bbox
                # and lifts by whatever the rotation puts under its old ground
                # contact, so a 5 m pole laid at 74 degrees is not half buried
                # and not hovering. It works on an INSTANCED prim because the
                # ops live on the instance root, which is still editable —
                # the same reason `apply_placements` sets `instanceable` after
                # authoring them.
                #
                # Downtrack, because a pole that has been pushed over lies the
                # way the wind went, and the small displacement is the base
                # dragging as it goes.
                # A POLE GOES FLATTER THAN A FENCE PANEL: it hinges at the
                # base and lies down, so 62-100 degrees rather than a lean.
                _rl = (frng.uniform(62.0, 100.0) if felled
                       else frng.uniform(38.0, 96.0)) * (
                    1.0 if frng.random() < 0.5 else -1.0)
                _d = frng.uniform(0.2, 1.4) * (0.4 + 0.6 * it)
                if tn.toss_prim(stage, path,
                                math.cos(math.radians(throw_deg)) * _d * ssf,
                                math.sin(math.radians(throw_deg)) * _d * ssf,
                                _rl, frng.uniform(-40.0, 40.0),
                                pitch_deg=frng.uniform(-18.0, 18.0)):
                    n_down += 1
        elif "car" in cat:
            n_car_in += 1
            # THE WHOLE POSE LADDER LIVES IN `tornado.car_pose` — the Paulikas
            # displacement rates, the one-draw structure that keeps the tip
            # share unconditional, the four resting poses, the two distance
            # regimes, the resting heading and the jam. It is there rather
            # than here for the reason every other model in this pipeline left
            # the launcher: it is pure geometry, so it can be pinned offline,
            # and `scene_gen/tests/test_car_toss.py` pins nine claims about it
            # in 0.2 s. Nothing about what a track does to a car needs a stage.
            #
            # THE TWO ARGUMENTS THAT ARE NEW HERE. `length_m`, because mass
            # matters and the same wind that rolls a saloon does not roll the
            # pool's 9.5 m bus; and `long_axis_deg`, because a rolled car comes
            # to rest with its long axis ACROSS the way it travelled and that
            # is only computable against the heading it was parked at. Both
            # derivations live in `suburb_scene` beside the pass that parked
            # the car, so a car boxed there and a car thrown here cannot
            # disagree about its own geometry.
            _ln, _ = ss.car_dims(resolver_pools[0], _pools, q["usd"])
            # EVERY IN-TRACK CAR, KEPT, so `TOR_MIN_TIPPED` can come back to
            # the ones the draw left alone. See the block after this loop.
            car_cands.append((it, q, path, _ln))
            pose = tn.car_pose(it, frng, throw_deg, float(tcfg["throw_m"]),
                               x=float(q.get("x_m", 0.0)),
                               y=float(q.get("y_m", 0.0)),
                               long_axis_deg=ss.car_heading(_pools, q),
                               length_m=_ln, blocked=car_block)
            if not pose["moved"]:
                continue
            if pose["toppled"]:
                n_car_tip += 1
            car_done.add(path)
            if _toss(stage, path, pose["dx"] * ssf, pose["dy"] * ssf,
                     pose["roll_deg"], pose["yaw_delta_deg"],
                     pitch_deg=pose["pitch_deg"]):
                n_car += 1
                if pose["arrested_by"]:
                    n_car_jam += 1
                # THE `toppled` FLAG, KEPT. `tornado_people` no longer places
                # anybody beside a car — every figure it authors is a casualty
                # in the debris, and the standing occupants went with the rest
                # of the uninvolved on 2026-08-27 — but the record is still the
                # contract `test_car_toss` asserts on and the flag is still
                # drawn at the same 30 degree line `car_pose` uses.
                car_recs.append({
                    "x": float(q.get("x_m", 0.0)) + pose["dx"],
                    "y": float(q.get("y_m", 0.0)) + pose["dy"],
                    "toppled": pose["toppled"], "pose": pose["pose"],
                    "prim_path": path})
    # A FLOOR UNDER THE TIPPED COUNT, FOR REVIEW SCENES ONLY.
    #
    # `TOR_MIN_TIPPED` defaults to 0 and the model is untouched at that value.
    # It exists because the ladder is a DRAW and the scene is DETERMINISTIC:
    # `car_pose` puts p_move at 0.10 + 0.62*i and p_tip at 0.05 + 0.30*i^1.5,
    # and a 100 m plate holds about eight in-track cars sitting mostly on the
    # track SHOULDER where i is 0.2-0.3 — so p_tip is under a tenth apiece and
    # "no toppled car" is the ORDINARY outcome, not a fault. Measured on
    # TOR_SEED=5: 8 cars in the path, 0 moved, three builds running. Re-rolling
    # the seed to find one is a poor way to answer "show me what a rolled car
    # looks like", so this forces the highest-intensity cars over instead and
    # SAYS SO in the banner. Never set it on a scene that is being measured.
    if MIN_TIPPED > 0 and n_car_tip < MIN_TIPPED and car_cands:
        car_cands.sort(key=lambda c: -c[0])
        for (it, q, path, _ln) in car_cands:
            if n_car_tip >= MIN_TIPPED:
                break
            if path in car_done:
                continue
            # The same three tipped attitudes `car_pose` draws, in the same
            # proportions, and the same downtrack march — this forces the
            # OUTCOME, not a different model.
            pose = tn.car_pose(max(it, 0.75), frng, throw_deg,
                               float(tcfg["throw_m"]),
                               x=float(q.get("x_m", 0.0)),
                               y=float(q.get("y_m", 0.0)),
                               long_axis_deg=ss.car_heading(_pools, q),
                               length_m=_ln, blocked=car_block,
                               force="tip")
            if not pose["toppled"]:
                continue
            if _toss(stage, path, pose["dx"] * ssf, pose["dy"] * ssf,
                     pose["roll_deg"], pose["yaw_delta_deg"],
                     pitch_deg=pose["pitch_deg"]):
                car_done.add(path)
                n_car += 1
                n_car_tip += 1
                n_car_forced += 1
                if pose["arrested_by"]:
                    n_car_jam += 1
                car_recs.append({
                    "x": float(q.get("x_m", 0.0)) + pose["dx"],
                    "y": float(q.get("y_m", 0.0)) + pose["dy"],
                    "toppled": True, "pose": pose["pose"],
                    "prim_path": path})

    print("[tornado] corridor: {0} fence/furniture prim(s) blown away and {1} "
          "left standing but over; {2} of {3} car(s) in the path moved, {4} of "
          "those tipped onto a side, roof or nose{6}, {5} came to rest jammed "
          "against a wall, a tree or another car".format(
              n_gone, n_down, n_car, n_car_in, n_car_tip, n_car_jam,
              "" if not n_car_forced
              else " (%d FORCED by TOR_MIN_TIPPED — this scene is a review "
                   "scene, not a measurement)" % n_car_forced))

    # 6) THE PLANK FIELD ------------------------------------------------------
    #
    # AUTHORED, NOT SIMULATED, and it is what carries the whole directional
    # read. See `disaster/planks.py` for why boards are boxes rather than
    # fracture output, and why they are merged into one mesh per stock class.
    n_boards = 0
    # THE BOARD FIELD, KEPT. `tornado_people` measures its own debris surface
    # off these specs (`_Deck`) instead of guessing it from the damage level,
    # which is what makes a casualty lie ON the debris rather than through it —
    # so the list has to outlive this block.
    plank_specs = []
    if N_PLANKS > 0:
        prng = random.Random(SEED + 77)
        specs = []
        # WHAT EACH HOUSE WAS WEARING, so its debris wears it too. Before this
        # every board in the field took one sawn-timber map and a levelled
        # block came out as a lumber yard: no cladding colour anywhere, no
        # grey roof slab, nothing to say the pile used to be a house. The
        # `siding` and `deck` classes now carry the wrecked house's own wall
        # and roof material NAMES; `skin_mats` below resolves each name once
        # through `modular_house.palette_material`, which is the same call
        # `apply_palette` makes for the standing houses — so the cladding in
        # the road and the cladding still on the neighbour are the same prim.
        # The per-class tints `planks` would have used, so a skin that finds
        # no map of its own still lands in the right value range.
        _TINT_FALLBACK = planks._TINT
        _skin_names = set()
        for (hx, hy, fp, it, _lv, _pal) in wrecks:
            _sk = mh.palette_skins(_pal)
            specs += planks.scatter_from_wreck(
                hx, hy, fp, it, throw_deg, float(tcfg["throw_m"]), prng,
                n_pieces=N_PLANKS, skins=_sk)
            _skin_names.update(v for v in _sk.values() if v)
        # ON THE PLATE, and clipped in TWO GROUPS so the banner's arithmetic
        # stays honest. `scatter_from_wreck` throws its comet tail `throw_m`
        # downtrack and knows nothing about the region, which is right for the
        # model and wrong for the picture: the ground sheet ends at `region`
        # and a board past it hangs over the void (see `planks.clip_to_region`).
        # Clipping the combined list after `n_house_boards` was taken would
        # subtract the dropped boards from the CORRIDOR count, which is the one
        # group that was already inside the plate.
        specs, _n_off = planks.clip_to_region(specs, region)
        n_house_boards = len(specs)
        if TRACK_PER100 > 0.0:
            _track = planks.scatter_over_region(
                region, inten, throw_deg, prng, per_100m2=TRACK_PER100,
                cell_m=10.0)
            # Region-sampled, so only the half-board that overhangs the very
            # edge can escape — a handful, and the same argument applies.
            _track, _n2 = planks.clip_to_region(_track, region)
            specs += _track
            _n_off += _n2
        pmats = planks.materials(stage, PARENT)
        # THE TEXTURE, NOT THE MATERIAL. Binding the kit's own MDL to a plank
        # mesh renders it BLACK — those are UV-space materials and a board is
        # an authored box with no `st` at all, which is the whole reason
        # `planks.wood_material` is triplanar. The first run that skinned the
        # debris put a field of black roof slab in the corridor. So the map is
        # pulled off the palette material and re-projected from world
        # coordinates, exactly like the timber.
        skin_mats = {}
        for _n in sorted(_skin_names):
            try:
                _tex, _tint = mh.palette_texture(stage, PARENT, _n)
                if not _tex:
                    # STILL SKIN IT, WITH THE TINT ALONE. Falling through to
                    # the class default put pale sawn timber where a grey roof
                    # slab belongs, which is the fault this whole pass exists
                    # to fix — and the palette's albedo multiplier is enough on
                    # its own to make a shingle slab read as one at the range
                    # this scene is flown at. Say so, because a textureless
                    # skin is worth chasing later.
                    print("[tornado] debris skin {0}: no base-colour map "
                          "found; using the timber map with the palette tint"
                          .format(_n))
                    _base = _TINT_FALLBACK.get(
                        "deck" if _n.startswith("shingle") else "siding",
                        (1.0, 1.0, 1.0))
                    skin_mats[_n] = planks.wood_material(
                        stage, "{0}/PlankLooks/skin_{1}".format(PARENT, _n),
                        tile_m=1.2,
                        tint=tuple(min(1.0, a * b)
                                   for a, b in zip(_base, _tint)))
                    continue
                skin_mats[_n] = planks.skin_material(
                    stage, "{0}/PlankLooks/skin_{1}".format(PARENT, _n),
                    _tex, tint=_tint,
                    tile_m=1.35 if _n.startswith("shingle") else 1.05)
            except Exception as _exc:
                print("[tornado] debris skin {0} unavailable: {1}"
                      .format(_n, _exc))
        made = planks.build(stage, PARENT + "/debris", specs, pmats, ssf,
                            skin_mats=skin_mats)
        n_boards = len(specs)
        plank_specs = specs
        print("[tornado] plank field: {0} board(s) ({1} off {2} wreck(s), "
              "{3} across the corridor) in {4} mesh(es)".format(
                  n_boards, n_house_boards, len(wrecks),
                  n_boards - n_house_boards, len(made)))

    # 7) THE GROUND SCOUR -----------------------------------------------------
    made_g = []
    if DO_GROUND:
        zs = float(binfo.get("z_scale") or ss.ground_z_scale(config, region))
        # BETWEEN GRASS AND ASPHALT on the z ladder, exactly where the burn
        # scar sits and for the same reason: roads, drives and walks then come
        # through the track UNSCOURED, which is correct (they are pavement)
        # and is what makes the streets legible straight across the corridor
        # in the reference photograph.
        mud_z = (ss._Z_GRASS + 0.5 * (ss._Z_ASPHALT - ss._Z_GRASS)) * zs
        kn = tn.knobs_from_env(span)
        cov = tn.scour_coverage(tcfg, region,
                                np.random.default_rng(SEED + 31),
                                intensity=inten, gamma=kn["gamma"],
                                islands=kn["islands"])
        made_g = ground.build_overlay(
            stage, cov, region, ssf, mud_z, material_parent=PARENT,
            root="/World/scourGround", cell_m=kn["cell_m"], bands=kn["bands"],
            tile_m=kn["tile_m"], op_range=kn["op_range"],
            texture=tn.MUD_TEXTURE,
            # You cannot scour water. A pool is a hole in the lawn with water
            # below grade and an overlay cell drawn over it is a film floating
            # on the surface — the same exclusion the burn scar makes.
            skip=ground.skip_rects(binfo.get("pool_rects") or (), pad=0.0))

    # 7a) THE 3D SCOUR RELIEF -------------------------------------------------
    #
    # THE OVERLAY IS THE COLOUR, THIS IS THE SHADOW. `build_overlay` is a flat
    # translucent film on a flat lawn: from 40 m up the corridor is the right
    # brown and the ground is still a billiard table. `disaster.scour_relief`
    # authors what came OUT of the ground — see that module's docstring for the
    # trochoid the cycloidal marks are traced from, for why the sod rolls sit
    # at the EDGE of the band rather than in the core, and for why the
    # earthquake pipeline's soil passes could not simply be called.
    #
    # It rides on top of the overlay (`ground_z = mud_z`), so the overlay's own
    # islands and its `skip` list have to be honoured here too or a heap of
    # subsoil ends up standing on the green patch the vortex missed, or in a
    # swimming pool. Same `coverage_at`, same rectangles, one more skip: a
    # house that still has a floor.
    made_r = []
    _sm = None
    # GATED ON THE OVERLAY, not only on its own switch. `cov`, `mud_z` and
    # `zs` are computed in the block above, and the relief has no meaning
    # without the film it stands on: bare heaps of earth on an unstained lawn
    # read as molehills.
    if DO_GROUND and srl.enabled():
        _pool_skip = ground.skip_rects(binfo.get("pool_rects") or (), pad=0.0)
        _keep = [(hx, hy, 0.55 * hfp + 0.8) for (hx, hy, hfp) in standing]

        def _relief_skip(px, py):
            # `skip_rects` tests a CELL's centre, so a degenerate cell is a
            # point test — the same rings the overlay skipped, read the same
            # way, rather than a second copy of the pool geometry.
            if _pool_skip(px, py, px, py):
                return True
            for (hx, hy, hr) in _keep:
                if abs(px - hx) <= hr and abs(py - hy) <= hr:
                    return True
            return False

        # THE ROADS. The overlay passes UNDER the asphalt on purpose so the
        # streets stay legible straight through the track — which also leaves
        # them spotless, and no photograph of a track has clean roads in it.
        # The relief is the only pass that can put mud back on them, as low
        # fans at PAVEMENT grade; `pavement_mask` is what tells it where the
        # carriageway is, and it also keeps the heaps and windrows off it.
        _net = binfo.get("net")
        _corr = [(list(getattr(e, "pts", ()) or ()),
                  float(getattr(e, "half_w", 0.0)))
                 for e in (getattr(_net, "edges", {}) or {}).values()]
        _pave_at = srl.pavement_mask(_corr, region) if _corr else None
        _kn_r = srl.knobs_from_env()
        _t_r = time.time()
        _specs = srl.scatter(
            tcfg, region, cov, random.Random(SEED + 61),
            flow_deg=throw_deg,
            # ON the overlay, not under it: a couple of millimetres clear so
            # the rim does not co-planar-fight the band it stands on.
            ground_z=mud_z + 0.002 * zs,
            pave_z=ss._Z_ASPHALT * zs + 0.002 * zs,
            pavement_at=_pave_at, skip=_relief_skip, knobs=_kn_r)
        _rmats = srl.materials(stage, PARENT)
        made_r = srl.build(stage, PARENT + "/scourRelief", _specs, _rmats, ssf)
        _sm = srl.summarise(_specs)
        print("[tornado] scour relief in {0:.1f}s: {1} feature(s) {2} in "
              "{3} mesh(es), {4} point(s), tallest {5:.2f} m".format(
                  time.time() - _t_r, _sm["features"], _sm["by_kind"],
                  len(made_r), _sm["points"], _sm["max_height_m"]))

    # 7b) THE PEOPLE ---------------------------------------------------------
    #
    # WHERE THE DEBRIS SURFACE ACTUALLY IS, sampled off the archetypes.
    #
    # `tornado_people._Deck` measures the board field, and the board field is
    # the THIN part of the debris on a levelled lot — the deep part is the
    # baked wreck USD, which the planner cannot see because it is a referenced
    # INSTANCE and that module never touches a stage. It did not matter while
    # `planks._lay` was levering boards up onto their corners, because the
    # accidental lumpiness cleared `_DECK_BAND["pile"]`'s floor. The moment the
    # boards were laid flat it mattered a great deal: the measured deck fell to
    # a couple of centimetres everywhere, `pile` stopped qualifying, and the
    # casualties were pushed out of the wreckage onto the skirt and the street
    # — pile=8 skirt=5 in one build, pile=3 skirt=7 in the next.
    #
    # `Usd.TraverseInstanceProxies` is what makes this cheap and possible: it
    # walks INTO an instance without authoring anything there, so each wreck's
    # own meshes can be measured in world space and their tops stamped into
    # the same 0.8 m grid the boards use. ~250 meshes a wreck against a
    # handful of wrecks a plate.
    deck_points = []
    if DO_PEOPLE and wrecks:
        try:
            _bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                                    [UsdGeom.Tokens.default_,
                                     UsdGeom.Tokens.render],
                                    useExtentsHint=True)
            for _i in range(len(houses)):
                _hp = "{0}/inst/h_{1}".format(PARENT, _i)
                _pr = stage.GetPrimAtPath(_hp)
                if not _pr or not _pr.IsValid():
                    continue
                for _q in Usd.PrimRange(_pr, Usd.TraverseInstanceProxies()):
                    if _q.GetTypeName() != "Mesh":
                        continue
                    _r = _bc.ComputeWorldBound(_q).ComputeAlignedRange()
                    if _r.IsEmpty():
                        continue
                    _mn, _mx = _r.GetMin(), _r.GetMax()
                    # The mesh's own top, at its plan centre. A fragment is
                    # small enough that one sample is the whole story; a wall
                    # is not, but a wall is not something a body lies on.
                    deck_points.append((
                        0.5 * (_mn[0] + _mx[0]) / ssf,
                        0.5 * (_mn[1] + _mx[1]) / ssf,
                        float(_mx[2]) / ssf))
            print("[tornado] debris surface: {0} sample(s) off the wreck "
                  "archetypes".format(len(deck_points)))
        except Exception as _exc:
            print("[tornado] archetype deck sampling FAILED ({0}); the people "
                  "pass will see the board field only".format(_exc))

    #
    # LAST, DELIBERATELY. Every pass above moves, deletes or re-materialises
    # something, and a survivor is not debris — `disaster.people` holds people
    # back past the wildfire scorch pass for the same reason. Nothing here
    # touches a figure once it is placed.
    n_people = n_trap = 0
    p_recs = []
    if DO_PEOPLE:
        try:
            pcfg = tpp.resolve_cfg(config)
            # ROAD POINTS: the carriageway is the only navigable ground in a
            # levelled subdivision, and it is where `street` and `assisted`
            # put people. Sampled off the street graph and kept only where the
            # track actually reached — a walker on an untouched street at the
            # far corner is a bystander, and this scene is about the people
            # the tornado hit.
            road_pts = []
            _net = binfo.get("net")
            for _e in (getattr(_net, "edges", {}) or {}).values():
                _pts = list(getattr(_e, "pts", ()) or ())
                for _i in range(0, max(0, len(_pts) - 1), 2):
                    _ax, _ay = float(_pts[_i][0]), float(_pts[_i][1])
                    if inten(_ax, _ay) < 0.15:
                        continue
                    _bx, _by = float(_pts[_i + 1][0]), float(_pts[_i + 1][1])
                    road_pts.append((_ax, _ay, math.degrees(
                        math.atan2(_by - _ay, _bx - _ax))))
            pctx = {
                "wrecks": [{"x": w[0], "y": w[1], "fp": w[2],
                            "intensity": w[3], "level": w[4]} for w in wrecks],
                "intact": intact,
                "road_pts": road_pts,
                "cars": car_recs,
                "throw_deg": throw_deg,
                # THE PLATE. Without it the planner will stand somebody off
                # the edge of the world: the boundary ring road runs ALONG
                # `region`, `_street` scatters 2.6 m off a road point and
                # `_thrown` throws people 10-40 m off a wreck. The 100 m
                # scene's first render had two walkers at (-51.6, -51.2) and
                # (-48.8, -52.3) with no ground under them.
                "region": region,
                # THE DEBRIS SURFACE, MEASURED. Every figure in the first 100 m
                # render was seated on one constant per damage level in a field
                # of 755 boards whose real top swings from 0 to ~1.4 m over a
                # couple of metres, so some floated, some were buried to the
                # shins and several stood on the tilted face of a half sheet of
                # plywood. `tornado_people._Deck` stamps the top of every board
                # into a 0.8 m grid off exactly these specs — the ones
                # `planks.build` just authored, not a model of them — and
                # refuses any body whose three stations disagree by more than
                # `max_deck_tilt_m`.
                "plank_specs": plank_specs,
                "deck_points": deck_points,
                # ONLY WHERE THE TORNADO WENT, AND NOT UNDER A TREE. The 100 m
                # render laid a casualty on an untouched green lawn under an
                # untouched canopy, two lots outside the corridor — a person
                # the tornado did not hit, photographed as leaves. `inten` is
                # the same field the damage ladder, the scour and the plank
                # scatter are drawn from, so the planner refuses against the
                # scene's own definition of "in the path" rather than a second
                # one; `canopies` is the trees that still have a crown, from
                # the damage draw above.
                "intensity_at": inten,
                "canopies": canopies,
                "humans": _rigged_humans(config, resolver_pools),
                "resolver": resolver_pools[0],
                "asset_pools": resolver_pools[1],
            }
            p_humans, p_debris, p_recs = tpp.plan_people(
                pcfg, pctx, random.Random(SEED + 91))
            if p_humans:
                sg.apply_placements(stage, p_humans, PARENT + "/people", ssf,
                                    resolver=resolver_pools[0],
                                    instance_categories=set())
                n_people = len(p_humans)
            if p_debris:
                # The boards that make `trapped_partial` partial. Same
                # `planks` path as the main debris field, so a board over a
                # casualty and a board in the field are the same material.
                #
                # THE POSE, THE THICKNESS AND THE TILT COME FROM THE PLANNER
                # NOW. `_trapped_partial` solves each board against the body
                # it is lying on — a propped one runs from the debris surface
                # up over the casualty and its pitch is `-atan(rise / run)` —
                # so randomising the tilt here is the difference between a
                # board resting on somebody and a board that has fallen off
                # them. `.get` with the old random default keeps any other
                # caller of this block working unchanged.
                _spec = [{"x": d["x"], "y": d["y"], "z": d["z"],
                          "l": d["len"], "w": d["wide"], "yaw": d["yaw"],
                          "t": d.get("t", drng.uniform(0.02, 0.05)),
                          "pitch": d.get("pitch", drng.uniform(-8.0, 8.0)),
                          "roll": d.get("roll", drng.uniform(-10.0, 10.0)),
                          "class": d.get("class",
                                         "sheathing" if d["wide"] > 0.4
                                         else "board")}
                         for d in p_debris]
                _spec, _n_trap_off = planks.clip_to_region(_spec, region)
                planks.build(stage, PARENT + "/trap_debris", _spec,
                             planks.materials(stage, PARENT), ssf,
                             verbose=False)
                n_trap = len(_spec)
            tpp.write_records(PEOPLE_JSON, p_recs, meta={
                "seed": SEED, "scene_config": SCENE_CONFIG,
                "epoch_min": pcfg.get("epoch_min"),
                "track_deg": float(tcfg["heading_deg"]),
                "throw_deg": throw_deg,
                "in_path_frac": summ["in_path_frac"]})
            print("[tornado] people: {0} authored, {1} board(s) over trapped "
                  "figures, ground truth -> {2}".format(
                      n_people, n_trap, PEOPLE_JSON))
        except Exception as _exc:
            import traceback
            print("[tornado] PEOPLE PASS FAILED: {0}".format(_exc))
            traceback.print_exc()

    # 8) RE-ASSERT THE CUTOUT FLAG, AFTER THE STAGE LOAD ---------------------
    #
    # THE COMMAND-LINE FLAG IS NOT ENOUGH IN A SCENE THAT LOADS A STAGE. Kit
    # maps the flag onto a render-settings USD property at ~12 s; the Pegasus
    # environment stage is then loaded, and loading a stage with authored
    # render settings RESETS that property to its default (OFF) — so the
    # overlay comes out fully transparent with the flag plainly correct on the
    # command line. Set THIS late it does push onto the live property.
    for _key in ("/rtx/raytracing/fractionalCutoutOpacity",
                 "/rtx/pathtracing/fractionalCutoutOpacity"):
        carb.settings.get_settings().set_bool(_key, True)
    for _ in range(30):
        omni.kit.app.get_app().update()

    rw, rh = region[2] - region[0], region[3] - region[1]
    print("\n" + "=" * 72)
    print("TORNADO TRACK ASSEMBLED  {0:.0f} x {1:.0f} m  (by reference)"
          .format(rw, rh))
    print("  total       {0:.0f} s".format(time.time() - t0))
    print("  track       {0:.0f} m wide toward {1:.0f} deg, {2:.1%} of the "
          "plate".format(tcfg["width_m"], tcfg["heading_deg"],
                         summ["in_path_frac"]))
    print("  houses      {0} placed ({1} built LIVE and INTACT — archetype "
          "missing); {2}".format(
              n_h, miss_h,
              ", ".join("%s=%d" % kv for kv in sorted(htally.items()))))
    print("  trees       {0} placed ({1} fell back to the green species USD); "
          "{2}".format(
              n_t, miss_t,
              ", ".join("%s=%d" % kv for kv in sorted(ttally.items()))))
    # THE EDGE BUDGET. `t_turned` climbing toward the tree count means the
    # archetypes out-reach the plate and the stand is losing its fall grain to
    # the boundary rather than to the wind; `t_dropped` climbing means the
    # damage ladder is being overruled near the edges, which prints as a ring
    # of healthier vegetation round the scene. Both should stay a minority on
    # a 100 m plate; the fix for either is a shorter `_WIND_DEBRIS` scatter
    # radius in the bake, not a wider yaw sweep here.
    print("  tree edge   {0} turned inward, {1} level-dropped to keep their "
          "debris on the {2:.0f} x {3:.0f} m plate".format(
              t_turned, t_dropped, region[2] - region[0],
              region[3] - region[1]))
    print("  debris      {0} board(s); {1} prop(s) blown away, {2} left "
          "standing but over".format(n_boards, n_gone, n_down))
    print("  vehicles    {0} of {1} in the path moved, {2} tipped, {3} "
          "jammed".format(n_car, n_car_in, n_car_tip, n_car_jam))
    print("  scour       {0} band(s) of Soil_Mud".format(len(made_g)))
    if _sm:
        print("  relief      {0} feature(s) in {1} mesh(es) — {2}".format(
            _sm["features"], len(made_r),
            ", ".join("%s=%d" % kv for kv in sorted(_sm["by_kind"].items()))))
    elif DO_GROUND:
        print("  relief      OFF (SCOUR_RELIEF=0) — the track is a flat stain")
    if DO_PEOPLE:
        _ps = tpp.summarise(p_recs)
        # THE TWO AXES FIRST, because they ARE the benchmark and because the
        # counts that were printed before this — scenario totals — looked
        # entirely reasonable in the run that was rejected on sight. A scene
        # whose casualties are all face-up, or none of which is covered below
        # the waist, has to be visible from the banner.
        print("  casualties  {0}; attitude {1}".format(
            _ps["total"], ", ".join("%s=%d" % kv for kv in
                                    sorted(_ps["by_attitude"].items()))))
        print("              visibility {0} (max covered {1:.0%}); "
              "ground truth -> {2}".format(
                  ", ".join("%s=%d" % kv for kv in
                            sorted(_ps["by_visibility"].items())),
                  _ps["max_covered_frac"], PEOPLE_JSON))
        print("              occlusion {0}".format(", ".join(
            "%s=%d" % kv for kv in sorted(_ps["by_occlusion"].items()))))
        print("              where {0}; {1} board(s) laid on bodies".format(
            ", ".join("%s=%d" % kv for kv in sorted(_ps["by_where"].items())),
            _ps["boards"]))
    if miss_h or miss_t:
        print("  !! {0} house(s) and {1} tree(s) are UNDAMAGED because their "
              "archetype is missing.".format(miss_h, miss_t))
        print("     The track, the debris and the scour are all real; the "
              "STRUCTURES are not.")
        print("     Bake them with bake_tornado_archetypes_launch_script.py "
              "into {0}".format(ARCH_DIR))
    if DO_GROUND and not made_g:
        print("  !! NO SCOUR BANDS. Either the track misses the plate or the "
              "coverage is zero everywhere — check tools/tornado_png.py")
    print("=" * 72 + "\n")

    # SNAPSHOTS. Nobody is at the GUI on a scripted run and a scene this size
    # can only be judged by looking at it, so the launcher photographs its own
    # result. `SNAP_DIR=` turns it on and MUST sit under the mounted log
    # directory or the PNGs are written somewhere the host cannot read.
    if SNAP_DIR:
        try:
            import importlib.util as _ilu
            _sp = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               "..", "utils", "snapshots.py")
            _spec = _ilu.spec_from_file_location("snapshots",
                                                 os.path.normpath(_sp))
            _snaps = _ilu.module_from_spec(_spec)
            _spec.loader.exec_module(_snaps)
            _snaps.overview(stage, (0.5 * (region[0] + region[2]),
                                    0.5 * (region[1] + region[3])),
                            max(rw, rh),
                            os.path.join(SNAP_DIR, "overview.png"), ssf)
            # ALONG THE TRACK, NOT AT A LIST OF LANDMARKS. What has to be
            # judged here is a gradient across a corridor, so the subjects are
            # points sampled along the centreline plus one on each flank — a
            # top-down and an oblique of each shows the ladder in order.
            _tt, (ux, uy), (vx, vy) = tn.frame(tcfg)
            ox, oy = tcfg["origin_m"]
            _pts = {}
            for k, s in enumerate((-0.28, 0.0, 0.28)):
                a = s * max(rw, rh)
                _pts["core_%d" % k] = (ox + ux * a, oy + uy * a)
            half = 0.5 * float(tcfg["width_m"])
            _pts["edge_left"] = (ox + vx * half * 0.95, oy + vy * half * 0.95)
            _pts["edge_right"] = (ox - vx * half * 0.95, oy - vy * half * 0.95)
            if wrecks:
                w0 = max(wrecks, key=lambda w: w[3])
                _pts["worst_wreck"] = (w0[0], w0[1])
            _snaps.views_around(stage, _pts, SNAP_DIR, ssf)
            # ...AND ONE PHOTOGRAPH PER CASUALTY, at a range where a body is a
            # body. `PEOPLE_SNAPS=n` turns it on.
            #
            # A 100 m plate at `views_around`'s 60 m top-down puts a 1.8 m
            # figure lying flat at about twenty pixels, which is precisely the
            # range at which the FIRST version of this pass shipped a scene
            # full of standing commuters and nobody could tell. The whole
            # point of the module is now HOW MUCH OF EACH BODY IS VISIBLE, and
            # that is a claim you cannot check from altitude. Named after the
            # record — `p03_side_legs_hips` — so the frame and the ground
            # truth can be read against each other without counting.
            if PEOPLE_SNAPS > 0 and p_recs:
                _sel = p_recs[:PEOPLE_SNAPS]
                _pp = {}
                for _i, _r in enumerate(_sel):
                    _a = math.radians(float(_r.get("body_axis_deg", 0.0)))
                    _rm = float(_r.get("reach_m", 1.8)) * 0.5
                    _pp["p%02d_%s_%s" % (_i, _r.get("attitude"),
                                         _r.get("occlusion"))] = (
                        _r["x"] + math.cos(_a) * _rm,
                        _r["y"] + math.sin(_a) * _rm)
                _snaps.views_around(stage, _pp, SNAP_DIR, ssf,
                                    top_h=11.0, obl_dist=9.0, obl_h=6.0)
                _pts.update(_pp)
            # ...AND EVERY DISPLACED VEHICLE, for the same reason. A rolled
            # car is the most arresting object in a damage photograph and the
            # one whose SEATING is hardest to judge from altitude — "the car
            # seems to have been turned 90 into the ground" is not a question
            # a 60 m top-down can answer. Named after the pose `car_pose` drew.
            if car_recs:
                # WHERE THE PRIM ACTUALLY IS, not where the record says. A
                # `car01_side_TIPPED` frame came back as empty carriageway,
                # which means the two had parted company — and a review camera
                # aimed at a record rather than at geometry cannot tell you
                # that. Read the authored translate back and aim at THAT,
                # reporting any disagreement.
                for _r in car_recs:
                    _p = _r.get("prim_path")
                    _pr = stage.GetPrimAtPath(_p) if _p else None
                    if not (_pr and _pr.IsValid()):
                        continue
                    for _op in UsdGeom.Xformable(_pr).GetOrderedXformOps():
                        if _op.GetOpName().split(":")[-1] != "translate":
                            continue
                        _t = _op.Get()
                        _ax, _ay = float(_t[0]) / ssf, float(_t[1]) / ssf
                        if (abs(_ax - _r["x"]) > 0.5
                                or abs(_ay - _r["y"]) > 0.5):
                            print("[tornado] car record/prim MISMATCH {0}: "
                                  "record ({1:.1f}, {2:.1f}) prim "
                                  "({3:.1f}, {4:.1f})".format(
                                      _p, _r["x"], _r["y"], _ax, _ay))
                        _r["x"], _r["y"] = _ax, _ay
                    # ...AND IS IT ON THE ROAD OR UNDER IT? A `side_TIPPED`
                    # frame that shows empty carriageway with the record and
                    # the prim in agreement means the car is THERE and BURIED
                    # — which is the other reading of "turned 90 into the
                    # ground". `toss_prim` seats by measuring, so a buried car
                    # is a measurement that came back wrong, and the only way
                    # to tell is to read the world bound back.
                    _wb = UsdGeom.BBoxCache(
                        Usd.TimeCode.Default(),
                        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
                        useExtentsHint=True).ComputeWorldBound(
                            _pr).ComputeAlignedRange()
                    if not _wb.IsEmpty():
                        _z0 = float(_wb.GetMin()[2]) / ssf
                        _z1 = float(_wb.GetMax()[2]) / ssf
                        print("[tornado] car {0} {1}: z {2:+.2f} .. {3:+.2f} m"
                              "{4}".format(_r.get("pose"),
                                           "TIPPED" if _r.get("toppled")
                                           else "moved", _z0, _z1,
                                           "   <-- BURIED" if _z0 < -0.25
                                           else ("   <-- FLOATING"
                                                 if _z0 > 0.25 else "")))
                _cc = {}
                for _i, _r in enumerate(car_recs):
                    _cc["car%02d_%s%s" % (_i, _r.get("pose", "moved"),
                                          "_TIPPED" if _r.get("toppled")
                                          else "")] = (_r["x"], _r["y"])
                _snaps.views_around(stage, _cc, SNAP_DIR, ssf,
                                    top_h=14.0, obl_dist=12.0, obl_h=5.0)
                _pts.update(_cc)
            print("[tornado] snapshots -> {0} ({1} subject(s))"
                  .format(SNAP_DIR, len(_pts)))
        except Exception as _exc:
            print("[tornado] snapshots FAILED: {0}".format(_exc))

    app = omni.kit.app.get_app()
    omni.timeline.get_timeline_interface().play()
    while simulation_app.is_running():
        app.update()
    simulation_app.close()


if __name__ == "__main__":
    main()
