#!/usr/bin/env python
"""
Assemble a 500 x 500 m suburb AFTER A HURRICANE — by reference, with no live
fracture, no settle and no fluid simulation.

    ARCH_DIR=/isaac-sim/AirStack/scene_gen/assets/archetypes_hurricane \
    SCENE_CONFIG=suburb_hurricane_500_l2 \
    ISAAC_SIM_SCRIPT_NAME=suburb_hurricane_launch_script.py \
    airstack up isaac-sim

Bake the archetype library FIRST with `bake_hurricane_archetypes_launch_script.py`
into the same `ARCH_DIR`.

WHAT THIS SCENE IS, AND HOW IT DIFFERS FROM THE TORNADO
-------------------------------------------------------
`suburb_tornado_launch_script.py` is the nearest relative and most of the
assembly pattern is lifted from it. Everything below is a place where the
hurricane is NOT a wide tornado, and reaching for the tornado code there is
the mistake this file exists to avoid:

  1. THERE IS NO TRACK. `disaster.hurricane.intensity_field` is not a function
     of distance from a line. It is near-uniform over a 500 m plate — a
     coastal roughness gradient worth a few percent, optional roll-vortex
     banding, and nothing else. Every house is in the storm.

  2. SO THE VARIANCE COMES FROM THE BUILDINGS. `draw_vulnerability` gives each
     house a construction-era draw, and THAT is what decides whether it loses
     shingles or loses its roof. Two identical houses side by side land on
     different rungs. On a tornado plate that would be a bug; here it is the
     entire spatial model.

  3. THE PROGRESSION LANDS MOSTLY IN THE LIGHT RUNGS AND STOPS SHORT OF A
     TORNADO'S. As of STREAM S (2026-08-31, "adjust the pattern of house
     damage") the houses reference the TORNADO's own archetype library and
     six-level ladder (`hurricane.house_level_for_intensity` maps this
     file's near-uniform field onto it) — most of the plate lands in
     `pristine`/`roof_stripped`, structural levels stay a minority even at
     L3, and `leveled` is rare. A hurricane scene where a large share of the
     houses are `leveled` is a tornado scene with the track taken out; see
     that function's own comment for the fitted target bands.

  4. THE HOUSE DEBRIS IS THE TORNADO'S OWN MACHINERY (`disaster.washaway.
     land_debris_specs` over `disaster.planks.scatter_from_wreck`), FLOOD-
     AWARE: the per-house budget is keyed to level rather than to a flat
     count, a region-wide litter field only ever lands on DRY ground (a
     piece the mask would drop into real water is the raft field's job),
     and every piece's bearing comes from `hurricane.wind_bearing_at`
     rather than one fixed track heading. See `# LAND DEBRIS`. The TREE
     debris is still this file's own model — mostly vegetation (leaves,
     twigs, fronds, limbs; see `disaster.hurricane.debris_mix`), which has
     no tornado equivalent to borrow.

  5. THERE IS WATER, AND IT IS THE OTHER HALF OF THE SCENE. `disaster.surge`
     imposes a synthetic ground (the suburb plate is dead flat — see that
     module) and cuts a static water surface against it. No fluid sim: an
     opaque muddy OmniPBR_ClearCoat on flat geometry, a shoreline that comes
     from an alpha map, ponding in the low spots, and mud/seed lines as proud
     ribbons rather than as per-building texture rebinds.

  6. `swept` IS NOT A WIND LEVEL. A slab swept clean is a SURGE signature and
     `hurricane.house_level_for_intensity` structurally cannot return one —
     it has no depth/surge input at all. Where this scene shows bare slabs,
     `surge.house_water_state(...)["swept"]` (or `washaway.house_surge_
     state`) put them there.

NO PEOPLE. Survivors are a later pass — nothing here calls `disaster.people`
or `disaster.tornado_people`. NO FIRE.

THE KIT FLAG IS LOAD-BEARING, TWICE, exactly as in the tornado and wildfire
launchers: the ground overlay and the water alpha are translucent OmniPBRs
whose opacity is a FRACTIONAL CUTOUT, and RTX Real-Time discards fractional
cutout unless `/rtx/raytracing/fractionalCutoutOpacity` is on. It must be a
command-line flag (below) AND re-asserted after the Pegasus environment stage
is loaded, because loading a stage with authored render settings resets the
property. The symptom of missing either is "I cannot see the ground/water at
all". Full account in the `build-wildfire-scenes` skill.

Env knobs:

    ARCH_DIR      TREE archetype library (default `scene_gen/assets/archetypes_hurricane`)
    HOUSE_ARCH_DIR  HOUSE archetype library — the TORNADO's six-level library
                  (default `scene_gen/assets/archetypes_tornado`; bake it
                  with `bake_tornado_archetypes_launch_script.py`, not the
                  hurricane baker). See "HOUSES = TORNADO, VERBATIM" above.
    SCENE_CONFIG  preset (default `suburb_hurricane_500_l2`)
    HUR_SEED      seed for the damage draws (default 11)
    HUR_WATER     0 disables every water layer (wind-only scene, for A/B)
    HUR_GROUND    0 disables the wet/mud ground overlay
    HUR_DEBRIS    structural-level land-debris ceiling, matched to the
                  tornado's own `TOR_PLANKS` (default 140); `roof_stripped`
                  houses are capped well under it — see `# LAND DEBRIS`
    HUR_DEBRIS_REGION_PER100  ambient litter density across the DRY plate,
                  boards per 100 m2 (default 1.0; 0 disables it) — the
                  tornado's `scatter_over_region` idea, masked to dry ground
                  only (a piece the mask would drop into real water is the
                  raft field's job, never double-placed here)
    SURGE_*       water tuning — see `disaster.surge.knobs_from_env`
    SNAP_DIR      write viewport PNGs here (MUST sit under the mounted log
                  dir). Defaults to `$FREEZE_OUT/snaps` when freezing.

  THE DATASET FREEZE (`final_disaster_dataset/<Disaster>/<Locale>/level_<n>/
  <k>/`). Every name here is `freeze_dataset_launch_script.py`'s verbatim, so
  the dataset's own tooling drives a hurricane cell with the same lines it
  drives a wildfire one:

    FREEZE_OUT    the cell directory. Setting it moves `GT_people.json`,
                  `GT_hurricane.json`, `GT_hints.json`, `build_stats.json`
                  and `snaps/` under it. Unset = the old behaviour exactly.
    FREEZE_NAME   basename for the exported `.usd`. Default is derived from
                  FREEZE_OUT by the dataset contract
                  (`hurricane_suburban_lvl2_3`), so it normally needs no value.
    FREEZE_EXPORT 1 runs `disaster.freeze.export_scene` after the captures.
    FREEZE_COLLECT 1 localises dependencies into `Materials/` (default 0,
                  matching every cell shipped so far).
    FREEZE_SNAPS  0 skips the review captures (default 1).
    FREEZE_EXIT   1 closes the app after the export — WINS over KEEP_OPEN,
                  because a loop over cells must not stall on a held stage.
    FREEZE_WAIVE_VEGETATION / FREEZE_WAIVE_ABOVE_INSTANCES /
    FREEZE_WAIVE_MIRRORED   the two portability safety valves; see the
                  comment on their definitions and `freeze.make_portable`.
    PEOPLE_VARIANT  integer k, the dataset's fifth axis. Offsets ONLY the
                  people RNG (`SEED + 191 + 1000 * k`), leaving the wind
                  field, houses, water, trees and cars bit-identical.
"""

import math
import os
import random
import sys
import time

import carb
from isaacsim import SimulationApp

# HEADLESS IS NOT A PREFERENCE HERE, IT IS A PROPERTY OF THE CONTAINER. The
# `isaac-sim-livestream` variant this scene runs under on OSMO is, in its own
# compose comment, "Headless: no X server, no display, no GUI window" — and
# asking for `headless: False` there produced a viewport with no window
# behind it, which rendered fine and then took SIGSEGV the moment anything
# tried to read an AOV out of it. Default to headless and let a desktop
# container opt back in.
_HEADLESS = os.environ.get("HUR_HEADLESS", "1").strip() not in ("0", "false",
                                                               "False")
simulation_app = SimulationApp(launch_config={
    "headless": _HEADLESS,
    "extra_args": ["--/rtx/raytracing/fractionalCutoutOpacity=true",
                   "--/rtx/pathtracing/fractionalCutoutOpacity=true",
                   # REFLECTIONS, AND THE ROUGHNESS CLAMP THAT WAS EATING THEM.
                   #
                   # The floodwater returned NO sky reflection at all — the one
                   # cue that makes a viewer read "water" rather than "paint" —
                   # despite `enable_clearcoat`, `clearcoat_ior 1.333` and a
                   # coat normal map all being set, and despite the dome
                   # verifiably carrying the storm HDRI ("1 re-textured ->
                   # approaching_storm_4k.hdr" in the run log).
                   #
                   # RTX Real-Time 2.0 defaults `/rtx/rtpt/maxRoughness` to
                   # 0.3 and skips reflections on any material rougher than
                   # that. The water body's `reflection_roughness_constant` is
                   # 0.90 — correct for a sediment body, and three times the
                   # threshold, so every reflection on it was being discarded
                   # before the material ever had a say.
                   #
                   # Same class as `fractionalCutoutOpacity` above and as
                   # `enable_opacity` in `surge._make`: a feature that is
                   # configured, plausible, and silently inert. Forced here
                   # rather than trusted, for the same reason those two are.
                   "--/rtx/reflections/enabled=true",
                   "--/rtx/rtpt/maxRoughness=1.0"],
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
from disaster import ground, planks                            # noqa: E402
from disaster import hurricane as hu                           # noqa: E402
from disaster import tornado as tn                            # noqa: E402
from disaster import surge as sgw                              # noqa: E402
from disaster import washaway as wash                        # noqa: E402
from disaster import hurricane_people as hpp                   # noqa: E402
from disaster import street_furniture as sfu                   # noqa: E402
from detail import modular_house as mh                         # noqa: E402


def _env(name, default):
    """`os.environ.get` with EMPTY TREATED AS ABSENT.

    The compose file exports these declared-but-unset, so they arrive as `""`
    and `os.environ.get(name, default)` never reaches the default. Same trap
    the archetype baker documents at length; every knob here goes through it.
    """
    v = os.environ.get(name)
    return default if v is None or not v.strip() else v.strip()


def _flag(name, default="1"):
    return _env(name, default) not in ("", "0", "false", "False")


def _default_freeze_name(out_dir):
    """`<disaster>_<locale>_lvl<n>_<k>` from the dataset path.

    The contract is `.../<Disaster>/<Locale>/level_<n>/<k>/`, capitalised in
    the PATH and lowercase in the FILENAME, so the basename is derivable and
    does not need a second env var that can disagree with the directory it
    lands in. Byte-for-byte `freeze_dataset_launch_script._default_name` —
    pure path string logic, no `scene_api` dependency — so a hurricane cell
    and a wildfire cell are named by the same rule.
    """
    parts = [q for q in os.path.abspath(out_dir).split(os.sep) if q]
    if len(parts) >= 4 and parts[-2].startswith("level_"):
        dis, loc = parts[-4].lower(), parts[-3].lower()
        return "{0}_{1}_lvl{2}_{3}".format(
            dis, loc, parts[-2].split("_", 1)[1], parts[-1])
    return "scene"


PARENT = "/World/stage/generated"
SCENE_CONFIG = _env("SCENE_CONFIG", "suburb_hurricane_500_l2")
SEED = int(_env("HUR_SEED", "11"))
ARCH_DIR = _env("ARCH_DIR",
                os.path.join(_SCENE_GEN_DIR, "assets", "archetypes_hurricane"))
# THE HOUSE LIBRARY IS THIS FILE'S OWN, NOT THE TORNADO'S — changed
# 2026-09-02, and the old default was a silent-failure trap.
#
# This file calls `hu.house_level_for_intensity`, the hurricane's EIGHT-level
# ladder, whose bottom three rungs (`shingles_lost`, `cover_lost`,
# `deck_panels_lost`) have no tornado counterpart: `archetypes_tornado` is a
# six-level library and always was. The house loop below resolves a missing
# key with `harch.get(key) or harch.get("house_<style>_pristine")`, so
# pointing at the tornado library does not raise, does not warn and does not
# leave a hole — it silently substitutes an UNDAMAGED house for every
# cladding rung. Level 1 is 20% cladding rungs and level 2 is 85%, so the
# whole of those two cells would render as a pristine suburb with some water
# in it, and the only symptom is that the scene looks wrong.
#
# `bake_hurricane_archetypes_launch_script.py` builds all seven non-pristine
# rungs into `archetypes_hurricane` and copies `pristine`/`swept` across from
# the tornado library (`_link_shared`), so ONE directory carries the complete
# eight-rung ladder — 8 styles x 8 rungs = 64 house files, plus the 34 tree
# archetypes `ARCH_DIR` reads out of the same place. Set `HOUSE_ARCH_DIR`
# back to `archetypes_tornado` only if you have deliberately reverted the
# launcher to `tornado_level_for_intensity`.
HOUSE_ARCH_DIR = _env(
    "HOUSE_ARCH_DIR",
    os.path.join(_SCENE_GEN_DIR, "assets", "archetypes_hurricane"))
ENV_URL = SIMULATION_ENVIRONMENTS["Default Environment"]
DO_WATER = _flag("HUR_WATER")
DO_GROUND = _flag("HUR_GROUND")
# STRUCTURAL-LEVEL LAND-DEBRIS CEILING, matched to the tornado's own
# `TOR_PLANKS` default (140) — see "# LAND DEBRIS" in the house/water pass:
# `roof_collapsed`/`partial_collapse`/`leveled` houses approach this budget,
# `roof_stripped` houses are capped well under it.
N_DEBRIS = int(_env("HUR_DEBRIS", "140"))
# THE WASH-AWAY PASS HAS ITS OWN SWITCH, separate from `HUR_WATER`. It moves
# and re-seats real prims where the rest of the water half only adds surfaces,
# so it is the part most likely to take the process down with it — and when a
# 4-minute scene build dies there, the cheapest next question is always "is it
# this pass?". `HUR_WASHAWAY=0` answers it in one run.
DO_WASHAWAY = _flag("HUR_WASHAWAY")

# ---------------------------------------------------------------------------
# DATASET FREEZE
# ---------------------------------------------------------------------------
#
# `final_disaster_dataset/<Disaster>/<Locale>/level_<n>/<k>/` — the contract
# `freeze-disaster-dataset` owns, capitalised in the PATH and lowercase in the
# FILENAME. Knob names are `freeze_dataset_launch_script.py`'s VERBATIM
# (`FREEZE_OUT`/`FREEZE_NAME`/`FREEZE_EXPORT`/`FREEZE_COLLECT`/`FREEZE_EXIT`/
# `PEOPLE_VARIANT`) so the dataset's own tooling — `dataset_upload.py`, the
# `FROZEN_SCENE` launcher path, the per-cell shell loops — drives a hurricane
# cell with the same lines it drives a wildfire one.
#
# WHY THIS LIVES HERE AND NOT IN `freeze_dataset_launch_script.py`: that
# launcher's only build path is `scene_api.build_scene()`, which is a wildfire
# monolith — every damage decision keys off a fire arrival-time field and
# `has_disaster` is gated on `config["disaster"]["fire"]`. Pointing it at a
# hurricane preset builds an undamaged suburb. The hurricane pipeline is this
# file's own `main()`; the export belongs where the scene is.
FREEZE_OUT = _env("FREEZE_OUT", "")
FREEZE_NAME = _env("FREEZE_NAME", "")
FREEZE_EXPORT = _flag("FREEZE_EXPORT", "0")
FREEZE_COLLECT = _flag("FREEZE_COLLECT", "0")
FREEZE_SNAPS = _flag("FREEZE_SNAPS", "1")
FREEZE_EXIT = _flag("FREEZE_EXIT", "0")
# THE TWO PORTABILITY SAFETY VALVES, carried over from
# `freeze_urban_fire_city_launch_script.py` unchanged in name, default and
# meaning. Both OFF: the primary path de-instances every offending prototype
# unconditionally, and this plate has thousands of AEC tree instances that
# make that expensive. Turn `FREEZE_WAIVE_VEGETATION=1` on if de-instancing
# proves too heavy, and `FREEZE_WAIVE_MIRRORED=1` only as the last-resort
# ship path when the de-instancing fixpoint alone cannot clear the gate — it
# waives ONLY paths with a stat-verified Nucleus twin, never a twin-less one.
FREEZE_WAIVE_VEGETATION = _flag("FREEZE_WAIVE_VEGETATION", "0")
FREEZE_WAIVE_ABOVE_INSTANCES = int(_env("FREEZE_WAIVE_ABOVE_INSTANCES", "500"))
FREEZE_WAIVE_MIRRORED = _flag("FREEZE_WAIVE_MIRRORED", "0")
# PEOPLE VARIANT — the dataset's fifth axis, and the ONLY seed it is allowed
# to touch. The people draw is `SEED + 191`; the wind field is `SEED + 23`,
# house/water `SEED + 5`, trees `SEED + 9`, cars `SEED + 77`. Offsetting the
# people RNG by `1000 * VARIANT` leaves all five of those bit-identical, which
# is what makes the k cells of a level five casts over ONE geometry rather
# than five different scenes. Same arithmetic `freeze_dataset_launch_script`
# uses for the wildfire cells.
PEOPLE_VARIANT = int(_env("PEOPLE_VARIANT", "0"))
if FREEZE_OUT:
    os.makedirs(FREEZE_OUT, exist_ok=True)
SNAP_DIR = _env("SNAP_DIR",
                os.path.join(FREEZE_OUT, "snaps")
                if (FREEZE_OUT and FREEZE_SNAPS) else "")
if SNAP_DIR:
    os.makedirs(SNAP_DIR, exist_ok=True)
# THE PEOPLE PASS — see "7c) THE PEOPLE" below and `disaster/hurricane_
# people.py`. ON by default, same convention as every other `DO_*` flag on
# this file.
DO_PEOPLE = _flag("HUR_PEOPLE")
# `FREEZE_OUT` WINS OVER `SNAP_DIR` for the ground truth. The captures are a
# review artefact and may be turned off or redirected; `GT_people.json` is
# part of the cell and has to land in the cell directory whatever the
# snapshots do.
PEOPLE_JSON = _env("HUR_PEOPLE_JSON",
                   os.path.join(FREEZE_OUT or SNAP_DIR or HOUSE_ARCH_DIR,
                                "GT_people.json"))

# EVERY HOUSE KEEPS ITS STREET YAW, always — unlike the tornado's own
# `_TRACK_YAWED` trick, which turns `leveled`/`swept` piles to face the
# track because a corridor's whole read is "everything went one way."
# A hurricane has no such single corridor bearing to turn a pile toward:
# `hurricane.wind_bearing_at` varies continuously across the plate (it is a
# spatial stand-in for time-of-arrival, not a track), so re-yawing a pile to
# it would rotate each wrecked house to a DIFFERENT bearing depending on
# where it sits — the opposite of "everything went one way" and not a
# reading anyone could make sense of from the air. Wind bearing is used for
# exactly one thing on this plate: the DEBRIS a house sheds (`# LAND
# DEBRIS`, below), never the house's own placement yaw.

# THE GREEN SPECIES USD PER SPECIES. Copied from the tornado assembly, and it
# is not optional: under `assembly=True` the layout STRIPS every tree out of
# the placement list into `info_out["tree_instances"]` and expects the
# launcher to reference each one itself. The first hurricane render skipped
# `pristine` on the theory that the layout had already drawn it, and skipped
# any damaged level whose archetype was missing — so with no tree bake yet,
# ALL 1,454 trees were skipped and the suburb came up completely bare. A
# treeless suburb is a much bigger lie than a green one.
TREE_SPECIES = {
    "Black_Oak": "airstack://scene_gen/assets/aec/tower/Assets/Vegetation/Black_Oak/Black_Oak.usd",
    "Shumard_Oak": "airstack://scene_gen/assets/aec/tower/Assets/Vegetation/Shumard_Oak/Shumard_Oak.usd",
    "Douglas_Fir": "airstack://scene_gen/assets/aec/brownstone/Assets/Vegetation/Trees/Douglas_Fir.usd",
    "Largetooth_Aspen": "airstack://scene_gen/assets/aec/brownstone/Assets/Vegetation/Trees/Largetooth_Aspen.usd",
    "Common_Apple": "airstack://scene_gen/assets/aec/tower/Assets/Vegetation/Common_Apple/Common_Apple.usd",
    "American_Beech": "airstack://scene_gen/assets/aec/brownstone/Assets/Vegetation/Trees/American_Beech.usd",
}



class _SurgeDisabled(Exception):
    """`SURGE=0`: not an error, and not caught by the water block's own
    `except Exception` as a failure -- see where it is raised."""


def _ref(stage, dst, usd, x, y, yaw, ssf, scale=1.0, instance=True):
    """Reference `usd` at a pose. Same helper as the tornado assembly."""
    prim = stage.DefinePrim(Sdf.Path(dst), "Xform")
    if not prim.GetReferences().AddReference(usd):
        return False
    xf = UsdGeom.Xformable(prim)
    xf.AddTranslateOp().Set(Gf.Vec3d(x * ssf, y * ssf, 0.0))
    xf.AddRotateZOp().Set(float(yaw))
    if scale != 1.0:
        xf.AddScaleOp().Set(Gf.Vec3f(scale, scale, scale))
    # INSTANCE IT — the same archetype lands many times on a plate, and
    # un-instanced trees were the 186M-point wall that OOM'd Isaac on the
    # wildfire plat. Only authoring INSIDE referenced content is forbidden.
    if instance:
        prim.SetInstanceable(True)
    return True


def wait_for_stage(stage, timeout_s=20.0):
    app = omni.kit.app.get_app()
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        if stage and stage.GetPrimAtPath("/World").IsValid():
            return True
        app.update()
    return False


def _reassert_cutout():
    """Re-assert fractional cutout AFTER the environment stage is loaded.

    Loading a stage with authored render settings resets this to its default,
    which silently drops every translucent overlay in the scene — the ground
    wetness and the water surface both. The command-line flag alone is not
    enough and this has been rediscovered three times.
    """
    try:
        st = carb.settings.get_settings()
        for k in ("/rtx/raytracing/fractionalCutoutOpacity",
                  "/rtx/pathtracing/fractionalCutoutOpacity"):
            st.set_bool(k, True)
    except Exception as exc:
        print("[hurricane] could not re-assert fractionalCutoutOpacity: {0}"
              .format(exc))


def _make_overcast(stage):
    """Put the scene under a STORM SKY instead of a clear noon one.

    This is not decoration. A hurricane scene is photographed hours after
    landfall under the trailing edge of the storm, and the first render came
    out under a hard sun with crisp black shadows — which reads as a sunny
    afternoon in a flooded suburb and undoes most of what the damage passes
    are trying to say. It also actively hurts the water: a specular sun spot
    is the one lighting cue that makes a flat sheet read as a mirror rather
    than as sand.

    OVERCAST IS NOT DARK, and the first attempt at this got that backwards.
    Sun cut to 6% and the dome to 900 produced a frame that was very nearly
    black — because an overcast sky is still 10,000-20,000 lux, and the scene
    under it is BRIGHT. What changes is the CONTRAST: the shadows go soft and
    shallow and the whole frame flattens. So the dome barely moves from its
    daylight value, and almost all of the work is done by widening the sun's
    angular diameter rather than by turning anything down.

    Three changes, in order of how much each one does:

    1. **Kill the hard sun.** Every `DistantLight` on the stage — the Pegasus
       environment ships one and `add_sky` cannot see it — is dropped to a
       fraction of its intensity and given a very wide angular diameter. A
       wide `angle` is what makes a shadow soft: the sun becomes a bright
       patch of cloud rather than a point. This is the change that matters.
    2. **Dim and cool the dome.** Overcast daylight is both dimmer and bluer
       than direct sun.
    3. **A little RTX fog**, which is the `build-hurricane-scenes` skill's
       standing recommendation in place of authoring rain: rain is motion and
       a static scene full of streaks is a still frame of a video effect that
       a detector would learn as scene identity. `fogZup` matters — these
       scenes are Z-up and the default assumes Y-up.

    Every number is overridable so a brighter frame can be taken for review
    without editing this file.
    """
    from pxr import UsdLux
    n_sun = 0
    keep = float(_env("HUR_SUN_KEEP", "0.30"))
    ang = float(_env("HUR_SUN_ANGLE", "25.0"))
    for prim in stage.Traverse():
        light = UsdLux.DistantLight(prim)
        if not light:
            continue
        attr = light.GetIntensityAttr()
        cur = attr.Get() if attr and attr.Get() is not None else 1000.0
        light.CreateIntensityAttr(float(cur) * keep)
        # 18 degrees is ~35x the sun's real 0.53 deg disc. Physically that is
        # not a sun at all, which is the point: it is the bright part of an
        # overcast sky, and it throws the diffuse edge-less shadow that an
        # overcast day actually has.
        light.CreateAngleAttr(ang)
        n_sun += 1
    # EVERY DOME ON THE STAGE, not just the one `add_sky` made.
    #
    # This is the fix for a full round of "the lighting change did nothing".
    # Two consecutive renders with very different sun and dome settings came
    # back BYTE-IDENTICAL, because the light that actually matters is not
    # `/World/DomeLight` at all: the Pegasus "Default Environment" brings its
    # own dome, and the log names its material —
    #     Environments/2023_1/DomeLights/Sky_Elements/materials/cloudy/SunsetSkyMat.mdl
    # — a SUNSET sky, which is where the warm hard light was coming from. It
    # is composed in from the environment stage, so it is at an environment
    # path this launcher never guessed. Traverse for the type instead of
    # reaching for a path.
    # SCALE THE DOME, DO NOT SET IT. Setting every dome to one absolute
    # number blew the plate out to near-white: these domes do not share a
    # unit. `add_sky`'s plain dome runs at 3500 with exposure -3, while a
    # TEXTURED environment dome is typically authored around 1.0 because its
    # HDRI carries the actual radiance. Forcing the second one to 3500 is
    # three thousand times too much light, and the frame came back white.
    n_dome = 0
    n_tex = 0
    # 2.4, NOT 0.55. That 0.55 was tuned against the environment's ORIGINAL
    # dome — a bright SunsetSky — where the job was to take the scene down.
    # Once `sky_hdri` swaps in `approaching_storm_4k`, the job reverses: a
    # storm HDRI is intrinsically far dimmer, so scaling it DOWN as well
    # double-dims and the plate comes back nearly black with the floodwater
    # reading as a black hole (it is a mirror with nothing bright to mirror).
    #
    # A scale that made sense for one sky is meaningless for another. This is
    # paired with the HDRI above and the two must be retuned together.
    dome_k = float(_env("HUR_DOME_SCALE", "2.4"))

    # SWAP THE SKY IMAGE, NOT JUST ITS BRIGHTNESS. This is the fix for "it
    # still looks bright and sunny" after two rounds of dimming lights.
    #
    # Scaling a dome's intensity changes how much light it CASTS. It does
    # nothing to what a camera sees when it looks AT the sky, because that
    # gradient is painted by the MDL bound to the dome — here `SunsetSkyMat`,
    # named in this scene's own Kit log — and a bound material's procedural
    # output ignores the light prim's `texture:file` entirely. So the scene
    # got darker and the sky stayed a sunset. Two separate things, and only
    # one of them was being touched.
    #
    # NVIDIA's content pack ships `NVIDIA/Assets/Skies/{Clear,Cloudy,Evening,
    # Indoor,Night,Storm}/*.hdr` — there is no "Overcast" category; `Storm`
    # and `Cloudy` are the two real candidates. There is also no cloud-cover
    # PARAMETER to drive: NVIDIA staff have stated on the developer forum that
    # Isaac Sim has no native weather simulation, and the practitioner
    # technique is swapping the HDRI. Full survey, the mirror-path fallbacks
    # and the trap list: `scene_gen/_plans/isaac_overcast_sky.md`.
    #
    # THE URL IS VERIFIED, not guessed. Probed with a ranged GET from the dev
    # host on 2026-08-30: all of {4.2, 4.5, 5.1} x {Storm/approaching_storm_4k,
    # Cloudy/abandoned_parking_4k, Cloudy/kloofendal_48d_partly_cloudy_4k}
    # return HTTP 206. `Storm` is the pick because this scene is the trailing
    # edge of a hurricane, not a fine day with cloud; `Cloudy/abandoned_parking`
    # is the flatter, less dramatic alternative if the storm sky reads too dark.
    #
    # Worth doing that probe before any render that depends on an asset path:
    # it cost one curl and removed the option-1 risk the survey flagged.
    #
    # `HUR_SKY_HDRI=""` restores the old behaviour exactly, for an A/B.
    sky_hdri = _env("HUR_SKY_HDRI",
                    "https://omniverse-content-production.s3-us-west-2"
                    ".amazonaws.com/Assets/Isaac/4.5/NVIDIA/Assets/Skies/"
                    "Storm/approaching_storm_4k.hdr")
    for prim in stage.Traverse():
        dome = UsdLux.DomeLight(prim)
        if not dome:
            continue
        _a = dome.GetIntensityAttr()
        _cur = _a.Get() if _a and _a.Get() is not None else 1000.0
        if sky_hdri:
            # ABSOLUTE, NOT SCALED, once we swap the texture.
            #
            # Scaling the existing intensity assumes that number was lighting
            # the scene. It was not: the environment's dome was lit by its
            # BOUND MDL (`SunsetSkyMat`), and `UnbindAllBindings()` below
            # removes exactly that. So after the swap the dome's own intensity
            # is all there is — and whatever it happened to hold before is
            # meaningless. Scaling it by 2.4 changed a number nothing read,
            # which is why a 4.4x "brightness increase" produced a
            # near-identical black frame.
            #
            # 3000 is a daylight-overcast dome in Kit's units, in the same
            # range `add_sky` uses for its own plain dome (3500 at exposure
            # -3). Tune with HUR_DOME_INT, not HUR_DOME_SCALE.
            dome.CreateIntensityAttr(float(_env("HUR_DOME_INT", "3000")))
        else:
            dome.CreateIntensityAttr(float(_cur) * dome_k)
        if sky_hdri:
            # UNBIND FIRST. With a material still bound the texture set below
            # is silently ignored and the render comes back looking completely
            # unchanged — which is the same class of failure as the byte-
            # identical renders that cost a round earlier in this build.
            try:
                UsdShade.MaterialBindingAPI(prim).UnbindAllBindings()
                dome.CreateTextureFileAttr(Sdf.AssetPath(sky_hdri))
                dome.CreateTextureFormatAttr(UsdLux.Tokens.latlong)
                n_tex += 1
            except Exception as exc:
                print("[hurricane] sky HDRI not applied to {0}: {1}"
                      .format(prim.GetPath(), exc))
        # Overcast light is DIMMER IN THE WARM CHANNELS and relatively bluer:
        # the direct solar beam is what carries the red, and cloud removes it.
        dome.CreateColorAttr(Gf.Vec3f(0.80, 0.84, 0.92))
        n_dome += 1
    print("[hurricane] overcast: {0} dome light(s) scaled to {1:.0%}, {2} "
          "re-textured{3}".format(n_dome, dome_k, n_tex,
                                  " -> " + sky_hdri.rsplit("/", 1)[-1]
                                  if sky_hdri else " (HDRI swap off)"))
    # FOG DEFAULTS OFF, and that is a change from the first attempt. The
    # skill recommends RTX fog in place of authored rain, and for an oblique
    # or ground-level frame it is right — but the review overview looks
    # straight DOWN from 475 m, so the whole column between camera and ground
    # sits inside the fog and the plate comes back veiled white. Fog is a
    # ground-level atmosphere effect; turn it on for obliques
    # (`HUR_FOG=1`) and leave the nadir plan clean.
    if _flag("HUR_FOG", "0"):
        try:
            st = carb.settings.get_settings()
            st.set_bool("/rtx/fog/enabled", True)
            st.set_bool("/rtx/fog/fogZup/enabled", True)
            st.set_float("/rtx/fog/fogStartDist", 180.0)
            st.set_float("/rtx/fog/fogEndDist",
                         float(_env("HUR_FOG_END", "1400.0")))
            st.set_float("/rtx/fog/fogColorIntensity", 1.0)
        except Exception as exc:
            print("[hurricane] RTX fog unavailable: {0}".format(exc))
    print("[hurricane] overcast: {0} distant light(s) softened to {1:.0%} at "
          "{2:.0f} deg".format(n_sun, keep, ang))

def main():
    omni.timeline.get_timeline_interface().stop()
    pg = PegasusInterface()
    pg._world = World(**pg._world_settings)
    pg.load_environment(ENV_URL)
    stage = omni.usd.get_context().get_stage()
    wait_for_stage(stage)
    _reassert_cutout()
    for name in ("GroundPlane", "Environment"):
        p = stage.GetPrimAtPath("/World/" + name)
        if p and p.IsValid():
            p.SetActive(False)

    config = load_scene_config(SCENE_CONFIG)
    _, ssf = get_stage_meters_per_unit(stage)

    # 1) LAYOUT ------------------------------------------------------------
    t0 = time.time()
    binfo = {}
    placements = generate_suburb_on_stage(stage, config, parent_path=PARENT,
                                          scene_scale_factor=ssf,
                                          info_out=binfo, assembly=True)
    _sky = resolve_sky(config)
    _si = config.get("sky_intensity")
    _se = config.get("sky_exposure")
    add_sky(stage, _sky,
            **({"intensity": float(_si)} if _si is not None else {}),
            **({"exposure": float(_se)} if _se is not None else {}))
    _make_overcast(stage)
    houses = binfo.get("house_instances", [])
    trees = binfo.get("tree_instances", [])
    print("[hurricane] layout in {0:.0f}s: {1} house + {2} tree instance(s)"
          .format(time.time() - t0, len(houses), len(trees)))

    if not os.path.isdir(ARCH_DIR):
        print("[hurricane] ARCH_DIR {0} does not exist — run "
              "bake_hurricane_archetypes_launch_script.py first"
              .format(ARCH_DIR))
    arch = {os.path.splitext(f)[0]: os.path.join(ARCH_DIR, f)
            for f in (os.listdir(ARCH_DIR) if os.path.isdir(ARCH_DIR) else [])
            if f.endswith(".usd")}
    # THE HOUSE LIBRARY, separately -- see `HOUSE_ARCH_DIR`'s own comment.
    # `arch` above stays the TREE dict; `harch` is the EIGHT-rung house
    # library (see HOUSE_ARCH_DIR's own comment) and is what the house loop
    # below reads. Both default to `archetypes_hurricane`, which carries
    # both kinds.
    if not os.path.isdir(HOUSE_ARCH_DIR):
        print("[hurricane] HOUSE_ARCH_DIR {0} does not exist — run "
              "bake_hurricane_archetypes_launch_script.py first"
              .format(HOUSE_ARCH_DIR))
    harch = {os.path.splitext(f)[0]: os.path.join(HOUSE_ARCH_DIR, f)
            for f in (os.listdir(HOUSE_ARCH_DIR)
                     if os.path.isdir(HOUSE_ARCH_DIR) else [])
            if f.endswith(".usd")}

    region = tuple(binfo.get("region") or (-250, -250, 250, 250))
    rw, rh = region[2] - region[0], region[3] - region[1]
    span = max(rw, rh)

    # 2) THE STORM ----------------------------------------------------------
    #
    # ONE rng for the field, a DIFFERENT one for the per-object draws — the
    # field's noise is numpy and the ladders are stdlib, and sharing a
    # generator between them would make a re-tune of one silently re-roll the
    # other. Same split the tornado launcher makes.
    hcfg = hu.resolve_cfg(config)
    inten = hu.intensity_field(hcfg, region, np.random.default_rng(SEED + 23))
    gust = hu.gust_field(hcfg, region, np.random.default_rng(SEED + 23))
    drng = random.Random(SEED + 5)
    hsumm = hu.summarise(hcfg, region, np.random.default_rng(SEED + 23))
    print("[hurricane] site gust {0:.0f} m/s ({1:.0f} mph), exposure {2}, "
          "wind toward {3:.0f} deg veering {4:.0f} deg across the plate"
          .format(float(hcfg["site_gust_mps"]),
                  float(hcfg["site_gust_mps"]) * 2.23694,
                  hcfg.get("exposure", "C"), float(hcfg["heading_deg"]),
                  float(hcfg.get("rotation_deg", 0.0))))
    print("[hurricane] field: {0}".format(hsumm))

    # 3) THE WATER ----------------------------------------------------------
    #
    # Resolved BEFORE the houses are placed, because `house_water_state`
    # decides which lots are swept — and a swept lot takes a different
    # archetype, not a different pass over the same one.
    # `surge.resolve_cfg` layers a FLAT dict over its own DEFAULTS — unlike
    # `hurricane.resolve_cfg`, it does not dig the sub-block out of a scene
    # config itself. Hand it the sub-block. Getting this wrong is silent: the
    # module returns its defaults and the preset's `surge_m` never lands, so
    # levels 2 and 3 flood identically.
    _hsub = ((config.get("disaster") or {}).get("hurricane") or {})
    scfg = sgw.resolve_cfg({k: v for k, v in _hsub.items()
                            if k in sgw.DEFAULTS})
    scfg.update(sgw.knobs_from_env(span))
    for k, v in _hsub.items():          # preset wins over the env defaults
        if k in sgw.DEFAULTS:
            scfg[k] = v
    depth = sgw.depth_at(scfg, region, np.random.default_rng(SEED + 41))
    ssumm = sgw.summarise(scfg, region, np.random.default_rng(SEED + 41))
    print("[hurricane] surge {0:.1f} m -> {1}".format(
        float(scfg["surge_m"]), ssumm))

    # 4) HOUSES -- THE TORNADO'S OWN LOOP, VERBATIM, over the HURRICANE
    #    PATTERN. Reference `house_<style>_<level>.usd` from `HOUSE_ARCH_DIR`
    #    (the tornado's six-level library), keep every house's STREET yaw
    #    (see the comment above the removed `_WIND_YAWED`), recolour row
    #    homes only (`bool(h.get("row")) and bool(_pal)`, batched through
    #    `pal_jobs` exactly like `suburb_tornado_launch_script`), and fall
    #    back to a live-built house on a missing archetype -- identical to
    #    that script's own fallback, not a hurricane-specific rewrite of it.
    #    THE ONLY HURRICANE-SPECIFIC PIECE IN THIS LOOP is which LEVEL a
    #    house lands on (`hu.house_level_for_intensity`, the hurricane
    #    field mapped onto the tornado's six-level vocabulary) and that
    #    `swept` can ONLY come from the surge, never from wind -- everything
    #    else below is the tornado's own mechanism.
    fp_by_style = {e["style"]: max(e["w"], e["d"])
                   for e in ss.modular_catalogue(config)}
    UsdGeom.Scope.Define(stage, Sdf.Path(PARENT + "/inst"))
    n_h = miss_h = n_swept = 0
    htally = {}
    era_tally = {}
    pal_jobs = []
    # (x, y, footprint, intensity, level, palette) -- same schema and same
    # "archetype's own baked palette wins" rule the tornado's `wrecks` list
    # uses, so `land_debris_specs` (job 4, below) can be handed this list
    # exactly as `planks.scatter_from_wreck` already expects it.
    wrecks = []
    wtally = {}          # what the WATER did, counted separately from the wind
    _h_recs = []
    for i, h in enumerate(houses):
        it = float(inten(h["x"], h["y"]))
        # THE PER-BUILDING DRAW IS THE SPATIAL MODEL. On a 500 m plate the
        # field varies by a few percent end to end; this is what makes two
        # neighbours land on different rungs, and without it the whole plate
        # is one damage state.
        era, vuln = hu.draw_vulnerability(drng)
        era_tally[era] = era_tally.get(era, 0) + 1
        wst = sgw.house_water_state(scfg, h["x"], h["y"], drng)
        # WHAT THE WATER DID TO THIS HOUSE, decided here and applied after the
        # archetype is referenced (the prim has to exist first). `swept` is
        # the only state that changes which ARCHETYPE is chosen; the rest are
        # transforms and added geometry on whatever the wind ladder picked.
        _sl = None
        if DO_WASHAWAY and float(wst.get("depth", 0.0)) > 0.0:
            try:
                _sl = wash.house_surge_state(float(wst["depth"]), vuln, drng)
            except Exception as _sexc:
                if not wtally.get("_serr"):
                    print("[hurricane] house_surge_state FAILED: {0}"
                          .format(_sexc))
                wtally["_serr"] = wtally.get("_serr", 0) + 1
        if wst.get("swept") or _sl == "swept":
            # SURGE, NOT WIND -- "slab swept clean is surge, never wind."
            # Neither wind ladder returns `swept`; the water is the only
            # thing on this plate allowed to clear a slab.
            level = "swept"
            _sl = "swept"
            n_swept += 1
        else:
            # THE HURRICANE'S OWN EIGHT-LEVEL LADDER, restored 2026-09-01 on
            # the user's instruction ("make this ladder for hurricane too").
            #
            # This file shipped `tornado_level_for_intensity` from the
            # 2026-08-31 STREAM S parity decision ("tornado and hurricane are
            # both largely wind damage"), which is defensible at the top of
            # the range and indefensible at the bottom. MEASURED, replaying
            # each dataset level's own intensity field:
            #
            #   level 1 (38 m/s, i 0.30-0.45)
            #       tornado ladder : 86.8% pristine, 13.2% roof_stripped
            #       eight-level    : 79.2% pristine, 20.8% shingles_lost
            #
            # The dataset ladder specifies level 1 as "cladding only; green
            # scene with litter" — and the six-level vocabulary has NO
            # cladding rung, so its only way to show damage at all is to
            # jump a house straight to `roof_stripped`, the entire covering
            # gone. Level 1 was therefore either untouched or over-damaged,
            # with nothing in between. `hurricane.house_level_for_intensity`
            # exists precisely for this and its own docstring is calibrated
            # against these three gusts.
            #
            # REQUIRES the three low rungs to be baked —
            # `house_<style>_{shingles_lost,cover_lost,deck_panels_lost}.usd`
            # — or the `harch.get(key) or ...pristine` fallback below turns
            # every cladding-damaged house back into an undamaged one, in
            # silence. See HURRICANE_RUNBOOK.md step 1.
            level = hu.house_level_for_intensity(it, drng, vuln=vuln)
        htally[level] = htally.get(level, 0) + 1
        # STREET YAW, ALWAYS -- see the comment where `_WIND_YAWED` used to
        # live. `hurricane.wind_bearing_at` is used ONLY for debris (job 4,
        # `# LAND DEBRIS` below), never for a house's own placement yaw.
        yaw = h["yaw"]
        key = "house_{0}_{1}".format(h["style"], level)
        usd = harch.get(key) or harch.get("house_{0}_pristine".format(h["style"]))
        if not usd:
            # BUILD IT LIVE INSTEAD OF LEAVING A HOLE -- the tornado
            # launcher's fallback, unchanged: an absent building in a
            # disaster scene reads as a deliberate empty lot, so a missing
            # archetype must not `continue` before at least an intact house
            # stands at the right pose.
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
                    print("[hurricane] live house fallback FAILED: {0}"
                          .format(_exc))
            continue
        # Row homes are recoloured per unit; the same USD forbids authoring
        # inside an instance that makes the tornado terrace opt out of
        # instancing -- EXACTLY that condition, no "differs from the
        # archetype's own default" extension.
        _pal = h.get("palette")
        _recolour = bool(h.get("row")) and bool(_pal)
        # A WRAPPER XFORM PER HOUSE, and the reference one level down --
        # KEPT from the pre-rewrite file, not part of the tornado's own
        # pattern. `_ref` authors `xformOp:translate`/`rotateZ` on whatever
        # prim it is given, and `washaway.apply_washaway` (below) wants to
        # author its OWN translate/rotate to push a flooded house off its
        # slab -- USD refuses to add an xform op that already exists, with
        # an exception whose `str()` is EMPTY, which is what silently ate
        # every one of 43 flooded houses' displacement before this split
        # existed. So `h_{i}` is a bare Xform nothing else touches and the
        # surge is free to move, and `h_{i}/model` carries the reference and
        # the placement ops; instancing still applies to the child, which is
        # where the geometry actually is.
        _hp = "{0}/inst/h_{1}".format(PARENT, i)
        UsdGeom.Xform.Define(stage, Sdf.Path(_hp))
        if _ref(stage, _hp + "/model", usd, h["x"], h["y"], yaw, ssf,
                instance=not _recolour):
            n_h += 1
            # FOR THE GROUND TRUTH. The level lives in the ARCHETYPE'S
            # FILENAME and a USD reference does not publish it, so a stage
            # walk can never recover it -- the same reason `scene_api.
            # build_scene` keeps `house_objects`. Collected here because
            # this is the only place that knows the prim path, the style,
            # the level, the era and the water depth together.
            _h_recs.append({"prim_path": _hp, "style": h["style"],
                            "level": level, "x": float(h["x"]),
                            "y": float(h["y"]), "yaw_deg": float(yaw),
                            "row": bool(h.get("row")),
                            "intensity": float(it),
                            "gust_mps": float(gust(h["x"], h["y"])),
                            "code_era": era, "vulnerability": float(vuln),
                            "water_depth_m": float(wst.get("depth", 0.0)),
                            "swept_by_surge": bool(wst.get("swept"))})
            if _recolour:
                pal_jobs.append({"prim_path": _hp + "/model",
                                 "palette": _pal, "category": "house"})
            if level != "pristine":
                # THE PALETTE THE ARCHETYPE WAS BAKED WITH -- same rule the
                # tornado's own `wrecks` entry uses: `bake_tornado_
                # archetypes_launch_script` dresses each style with `mh.
                # STYLES[style]["palette"]` before it damages it, so read
                # the style default first and only fall back to this
                # placement's own palette.
                wrecks.append((h["x"], h["y"],
                               fp_by_style.get(h["style"], 12.0), it, level,
                               mh.STYLES.get(h["style"], {}).get("palette")
                               or h.get("palette")))
            # THE WATER'S OWN DAMAGE, on top of the wind's. This is what makes
            # the flooded half read as flooded rather than as a dry
            # neighbourhood with a blue sheet over it: houses off their slabs
            # and turned down-flow, scoured yards, mud rings. See
            # `disaster/washaway.py` — and note the flow bearing is taken from
            # the SURGE config, so every displaced house on the plate agrees
            # with its neighbours. A street where each house moved a different
            # way reads as noise, not as a current.
            if DO_WASHAWAY and _sl and _sl != "wet":
                try:
                    # NEIGHBOUR-AWARE DRIFT (D6, 2026-09-01): a floated house
                    # strands AGAINST an obstacle, it never overlaps one -- the
                    # "house flying onto another house" review note. `blocked`
                    # carries every OTHER house footprint; the spec marches the
                    # drift and stops 0.5-1 m clear (same idiom as the car
                    # blockers). Pitch/roll capped in `collapse_spec` itself.
                    # A CALLABLE, NOT A LIST. `shift_spec`/`collapse_spec`
                    # march the drift against `blocked(x, y) -> tag`; handing
                    # them a list of `(x, y, fp)` tuples raises "'list' object
                    # is not callable" INSIDE the march, which this block's
                    # own `except` then swallows into a one-line warning —
                    # so every shifted/collapsed house silently kept its
                    # original pose and the D6 "house flying onto another
                    # house" fix did nothing at all.
                    #
                    # THIS IS THE SECOND TIME THIS EXACT BUG HAS SHIPPED IN
                    # THIS FILE. The car pass below carries a comment
                    # describing it happening there first ("every car stayed
                    # put: cars in water: piled 1 (0 moved)"), and the fix
                    # was the same one: build `tornado.car_blockers`, which
                    # RETURNS the predicate. The house path was written later
                    # and reintroduced it. `scene_gen/tests/
                    # test_hurricane_washaway_blockers.py` now fails if
                    # either call site is handed a non-callable.
                    #
                    # Per house, excluding itself: `car_blockers` documents a
                    # thin self-blocking margin that holds for a car probing
                    # ahead of its own nose, and a house is far larger than a
                    # car — so rather than lean on that margin, the house's
                    # own disc is simply left out. The grid is 94 entries and
                    # rebuilding it per house is free.
                    _blocked = tn.car_blockers(
                        standing=[(o["x"], o["y"],
                                   fp_by_style.get(o["style"], 12.0))
                                  for o in houses if o is not h])
                    _spec = (wash.shift_spec(float(wst["depth"]),
                                             float(scfg["shore_bearing_deg"]),
                                             drng,
                                             x=float(h["x"]), y=float(h["y"]),
                                             own_fp_m=fp_by_style.get(
                                                 h["style"], 12.0),
                                             blocked=_blocked)
                             if _sl == "shifted" else
                             wash.collapse_spec(float(wst["depth"]), drng,
                                                x=float(h["x"]), y=float(h["y"]),
                                                own_fp_m=fp_by_style.get(
                                                    h["style"], 12.0),
                                                blocked=_blocked)
                             if _sl == "collapsed" else {})
                    _spec.setdefault("mudline_z", wst.get("mudline_z"))
                    wash.apply_washaway(stage, _hp, _sl, _spec, ssf=ssf)
                    wtally[_sl] = wtally.get(_sl, 0) + 1
                except Exception as _wexc:
                    if not wtally.get("_err"):
                        print("[hurricane] washaway FAILED on {0}: {1}"
                              .format(_hp, _wexc))
                    wtally["_err"] = wtally.get("_err", 0) + 1
    if pal_jobs:
        try:
            n_pal = mh.apply_palette(stage, pal_jobs, PARENT)
            print("[hurricane] row homes: {0} subset(s) recoloured across "
                  "{1} unit(s)".format(n_pal, len(pal_jobs)))
        except Exception as _exc:
            print("[hurricane] row-home palette FAILED: {0}".format(_exc))
    print("[hurricane] houses {0} referenced, {1} live fallback(s); "
          "{2} swept by surge".format(n_h, miss_h, n_swept))
    if wtally:
        print("[hurricane] surge damage: {0}".format(
            ", ".join("{0} {1}".format(k, wtally[k])
                      for k in sorted(wtally) if not k.startswith("_"))))
    print("[hurricane] level tally: {0}".format(
        ", ".join("{0} {1}".format(k, htally[k])
                  for k in sorted(htally, key=lambda z: -htally[z]))))
    print("[hurricane] construction era: {0}".format(
        ", ".join("{0} {1}".format(k, era_tally[k]) for k in sorted(era_tally))))

    # 5) TREES --------------------------------------------------------------
    #
    # THE STRONGEST CLASS SEPARATOR IN THE DATASET, and the one place the
    # hurricane's tree story is genuinely its own rather than the tornado's
    # with different numbers. `defoliated` — standing, structurally whole,
    # canopy stripped to bare sticks — is the PLURALITY outcome and has no
    # tornado equivalent at all: a tornado breaks wood over a corridor, a
    # hurricane strips leaves over a county. It is also what dates the scene,
    # since NDVI recovers in about six weeks.
    trng = random.Random(SEED + 9)
    ttally = {}
    n_t = miss_t = 0
    _t_recs = []
    for j, t in enumerate(trees):
        it = float(inten(t["x"], t["y"]))
        sp = t.get("species") or ""
        # SATURATED-SOIL WINDTHROW BOOST (see `hurricane.windthrow_depth_
        # boost`'s docstring). `depth` is the SAME callable §3 built for the
        # houses (`sgw.depth_at`) — reused here rather than re-resolved, so
        # the tree ladder and the house/water passes always agree about
        # where the water is. Without this, L2's field tops out at 0.598
        # and even the full jitter cannot reach `leaning`/`fallen`/
        # `snapped`, so a flooded plate could never show a single fallen
        # tree — precisely backwards, since standing water is where root
        # anchorage fails first.
        depth_m = float(depth(t["x"], t["y"]))
        level = hu.tree_level_for_intensity(it, trng, species=sp or None,
                                             depth_m=depth_m)
        ttally[level] = ttally.get(level, 0) + 1
        # THE TWO SOURCES ARE AUTHORED AT DIFFERENT SCALES AND MIXING THEM UP
        # DESTROYS THE SCENE. A baked archetype is metres (`scale = 1.0`); the
        # green species USD is CENTIMETRES and must be referenced at 0.01,
        # exactly as `suburb_tornado_launch_script.py:474` does.
        #
        # Referencing the species USD at 1.0 makes every tree a HUNDRED TIMES
        # too big, and with 1,684 of them the overview camera at 475 m ends up
        # inside the bark of one of them. The frame that produced was a nearly
        # black image of a few enormous flat planes — which reads as "the
        # scene is too dark" or "the water is occluding everything", and cost
        # a full round of chasing the lighting and then the water before the
        # give-away turned up: two renders with completely different lighting
        # came back BYTE-IDENTICAL, because the camera could not see the
        # lights at all.
        # `pristine` IS NOW BAKED TOO (`bake_hurricane_trees.py`'s cotton-
        # ball section) and is tried here like every other level — this
        # used to skip the archetype dict outright for `pristine` and go
        # straight to the raw, undamaged, CENTIMETRE-scale species USD,
        # which is the file that put a full, undamaged, unverified-material
        # canopy on the hot path for every tree the intensity ladder itself
        # called undamaged. That raw USD is now only a last-resort fallback
        # for a species missing from `TREE_SPECIES`/`arch` entirely.
        key = "tree_{0}_{1}".format(sp, level)
        usd = arch.get(key)
        scale = 1.0
        if not usd:
            # FALL BACK TO THE GREEN SPECIES USD, never to nothing. A missing
            # archetype must degrade to "this tree is undamaged", which is
            # wrong about ONE tree, not to "there is no tree here", which is
            # wrong about the whole neighbourhood. Counted separately so the
            # banner says how much of the canopy is honest.
            usd = sg._join_asset_root(TREE_SPECIES.get(sp, ""), "")
            scale = 0.01                      # centimetre-authored source
            if level != "pristine":
                miss_t += 1
                level = "pristine"
            if not usd:
                continue
        # A FALLEN TRUNK POINTS SOMEWHERE and that is the cheapest directional
        # cue in the scene. The jitter is wide on purpose: trees go over in
        # the direction their own rooting and their neighbours allow as much
        # as in the direction the wind was going, and a stand all pointing
        # within five degrees reads as a logging operation rather than as
        # windthrow.
        if level in ("leaning", "fallen", "snapped"):
            # +-26, NARROWED from +-38 (2026-09-01, "bent trees in the
            # direction of wind"): a 76 deg spread is wide enough that a
            # stand reads as randomly oriented rather than as one storm's
            # doing. 52 deg still avoids the "logging operation" look the
            # original comment is guarding against — trees do go over where
            # their own rooting and their neighbours allow — while leaving
            # the downwind bearing legible from the air, which is the whole
            # point of the cue.
            yaw = (float(hu.wind_bearing_at(hcfg, t["x"], t["y"]))
                   + trng.uniform(-26.0, 26.0))
        else:
            yaw = t.get("yaw", 0.0)
        _tp = "{0}/inst/t_{1}".format(PARENT, j)
        if _ref(stage, _tp, usd, t["x"], t["y"], yaw, ssf, scale=scale):
            n_t += 1
            _t_recs.append({"prim_path": _tp, "species": sp, "level": level,
                            "x": float(t["x"]), "y": float(t["y"]),
                            "yaw_deg": float(yaw), "intensity": float(it)})
    print("[hurricane] trees {0} placed, {1} fell back to the green species "
          "USD (no archetype); tally {2}"
          .format(n_t, miss_t,
                  ", ".join("{0} {1}".format(k, ttally[k])
                            for k in sorted(ttally, key=lambda z: -ttally[z]))))

    # 5b) THE CARS, WHICH THE WATER MOVES AND THE WIND DOES NOT --------------
    #
    # `hurricane.py` says twice that vehicles are moved by WATER rather than
    # wind, and a coverage audit against the FEMA/StEER record found that
    # nothing in the pipeline was doing it: the suburb's cars sat through a
    # Category-3 surge exactly as they would on a clear day. This is that pass.
    #
    # A car floats in FAR less water than a house does — NWS's "Turn Around
    # Don't Drown" figures are 6 inches to lose control, 12 to float most
    # passenger cars, 24 to carry nearly anything — so `washaway`'s car ladder
    # sits an order of magnitude below the house one (0.15 / 0.30 m against
    # 0.75 / 2.0 m) and is keyed to vehicle LENGTH rather than to construction
    # era, because a car does not have one.
    #
    # THE CARS ARE FOUND BY WALKING THE STAGE, not from `binfo`. `info_out`
    # publishes only cars/clusters/house_instances/park/tree_instances, and
    # `cars` among them is a comment rather than an assignment — the suburb
    # generator never actually fills it. A placement's leaf prim name IS
    # `f"{category}_{group}_{i}"` (`scene_generator.apply_placements`), which
    # is the same route `hurricane_flow._category_of` takes to recover a
    # category from a live prim.
    n_car = 0
    ctally = {}
    if DO_WASHAWAY:
        crng = random.Random(SEED + 77)
        _flow = float(scfg["shore_bearing_deg"])
        # `car_shift_spec(blocked=...)` wants the CALLABLE `tornado.car_blockers`
        # returns — `blocked(x, y) -> tag` — not a list of positions. Passing a
        # list raised "'list' object is not callable" inside the drift march,
        # the whole car pass was caught by its own except, and every car stayed
        # put: "cars in water: piled 1 (0 moved)".
        try:
            _blockers = tn.car_blockers(
                standing=[(r["x"], r["y"], fp_by_style.get(r["style"], 12.0))
                          for r in _h_recs],
                trees=[(t["x"], t["y"]) for t in _t_recs],
                cars=[])
        except Exception as _bexc:
            print("[hurricane] car blockers unavailable ({0}); floated cars "
                  "will drift without arresting".format(_bexc))
            _blockers = None
        try:
            _root = stage.GetPrimAtPath(Sdf.Path(PARENT))
            for prim in (Usd.PrimRange(_root) if _root and _root.IsValid()
                         else ()):
                if not prim.IsA(UsdGeom.Xformable):
                    continue
                if not prim.GetName().lower().startswith("car_"):
                    continue
                try:
                    _cw = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
                        Usd.TimeCode.Default())
                    _cx, _cy = float(_cw[3][0]) / ssf, float(_cw[3][1]) / ssf
                except Exception:
                    continue
                _cd = float(depth(_cx, _cy))
                if _cd <= 0.0:
                    continue
                _lv = wash.car_surge_state(_cd, crng)
                ctally[_lv] = ctally.get(_lv, 0) + 1
                if _lv in ("dry", "swamped"):
                    continue
                _cs = wash.car_shift_spec(_cd, _flow, crng, x=_cx, y=_cy,
                                          blocked=_blockers)
                try:
                    wash.apply_car_washaway(stage, prim, _lv, _cs, ssf=ssf)
                    n_car += 1
                except Exception as _cexc:
                    if not ctally.get("_err"):
                        print("[hurricane] car washaway FAILED on {0}: {1}"
                              .format(prim.GetPath(), _cexc))
                    ctally["_err"] = ctally.get("_err", 0) + 1
        except Exception as _cexc:
            print("[hurricane] car pass FAILED: {0}".format(_cexc))
    if ctally:
        print("[hurricane] cars in water: {0} ({1} moved)".format(
            ", ".join("{0} {1}".format(k, ctally[k])
                      for k in sorted(ctally) if not k.startswith("_")),
            n_car))

    # 6) THE WATER ----------------------------------------------------------
    #
    # STATIC GEOMETRY. No fluid solver, no particles, zero physics bodies —
    # see `disaster/surge.py` for the whole argument, and for the synthetic
    # ground it has to impose because this plate is dead flat and a flat water
    # plane over a flat plate floods everything or nothing.
    made_w = {}
    if DO_WATER:
        tw = time.time()
        wrng = random.Random(SEED + 61)
        try:
            # `SURGE=0` IS DOCUMENTED AT THE TOP OF THIS FILE AND WAS NEVER
            # HONOURED HERE. `surge.enabled()` exists for exactly this and no
            # caller in this launcher consulted it, so setting SURGE=0 built
            # the whole flood anyway and reported success.
            #
            # That is not a cosmetic gap: an off-switch that silently does
            # nothing invalidates any experiment run through it. A control
            # render taken with SURGE=0 came back all but identical to the
            # water-on plate (`deep_water_top` differed by 0.12/255, with no
            # pixel off by more than 8), which reads as overwhelming evidence
            # that the water is invisible -- and is instead just two renders
            # of the same scene. Being able to turn the water OFF is what
            # makes "is the water rendering at all?" answerable.
            if not sgw.enabled():
                print("[hurricane] SURGE=0 -- water pass skipped "
                      "(no inundation, ponding, deposits or rafts)")
                raise _SurgeDisabled()
            # `water_materials` is called by the builders themselves; it is
            # invoked here only so a material failure is reported against the
            # material rather than surfacing later as an untextured plane.
            # ONE material set, built once and handed to all three builders.
            # Letting each build its own would give the ponding a different
            # water than the inundation it feeds into, which reads as two
            # unrelated liquids sitting next to each other.
            wmats = sgw.water_materials(stage, PARENT)
            made_w["inundation"] = sgw.build_inundation(
                stage, PARENT + "/flood", scfg, region, wrng, ssf=ssf,
                materials=wmats)
            made_w["ponding"] = sgw.build_ponding(
                stage, PARENT + "/ponding", scfg, region, wrng, ssf=ssf,
                materials=wmats)
            made_w["deposits"] = sgw.build_deposits(
                stage, PARENT + "/deposits", scfg, region, wrng, ssf=ssf,
                materials=wmats)
            # DEBRIS RAFTS — the mats of timber and sheet goods that collect
            # against whatever is still standing. Without them the flood is an
            # empty sheet, which is the other half of "it doesn't look like
            # anything happened here".
            #
            # DEBRIS STREAM REVIEW, 2026-08-31: rafts lay perfectly flat on
            # TOP of the opaque quad (paper on a table — see `BASE_L3/
            # deepest_flooded_house_obl.png`), were one uniform pale colour,
            # confetti'd evenly over open water with nothing at the waterline
            # and nothing against a house. `washaway._one_raft`/`raft_specs`
            # now give every piece a kind-dependent draft (it actually
            # crosses the water surface instead of resting on it), a real
            # tilt, a wet-darkened tint, a denser strand line at the
            # waterline, coarse hurricane stock (panels/fence/siding
            # strips/two-tone roof sections), and — new here — a real
            # per-style footprint (`fp_by_style`, not the old bare `(x, y)`
            # that fell back to a generic 10 m box for every house) plus
            # `obstacles=` so trees (and, best-effort, cars found by a stage
            # walk) collect their own up-flow clusters too.
            try:
                if not DO_WASHAWAY:
                    raise RuntimeError("washaway disabled (HUR_WASHAWAY=0)")
                _wcfg = wash.resolve_cfg(config)
                _wcfg["water_level_m"] = sgw.water_level(scfg)
                # THE RAFTS ARE MOSTLY VEGETATION, and the share comes from
                # `hurricane.debris_mix` rather than a second hardcoded copy
                # of it. The audit's finding: the raft field was 100% building
                # material against this codebase's own citation (Lee County
                # after Ian, 734,136 yd3 vegetative against 285,282 yd3 C&D —
                # vegetation the majority by more than 2:1).
                try:
                    _dm = hu.debris_mix(0.70, wrng)
                    _veg = sum(v for k, v in _dm.items()
                               if any(w in k for w in
                                      ("leaf", "limb", "frond", "veg")))
                    _kw = wash.raft_kind_weights(_veg)
                except Exception:
                    _kw = None
                _depth_fn = sgw.depth_at(scfg, region,
                                         np.random.default_rng(SEED + 41))
                # A canopy/parked-car footprint is not recorded per instance
                # anywhere upstream, so a flat stand-in is used — best-effort,
                # not a claim of measured size. Cars are walked off the stage
                # the same way the surge car pass above already does; failing
                # that walk only loses the car clusters, never the rafts.
                _TREE_OBSTACLE_FP_M = 3.0
                _CAR_OBSTACLE_FP_M = 4.6
                _obstacles = [(t["x"], t["y"], _TREE_OBSTACLE_FP_M)
                             for t in _t_recs]
                try:
                    _oroot = stage.GetPrimAtPath(Sdf.Path(PARENT))
                    for _cp in (Usd.PrimRange(_oroot)
                               if _oroot and _oroot.IsValid() else ()):
                        if not (_cp.IsA(UsdGeom.Xformable)
                               and _cp.GetName().lower().startswith("car_")):
                            continue
                        _cw = UsdGeom.Xformable(_cp).ComputeLocalToWorldTransform(
                            Usd.TimeCode.Default())
                        _obstacles.append((float(_cw[3][0]) / ssf,
                                          float(_cw[3][1]) / ssf,
                                          _CAR_OBSTACLE_FP_M))
                except Exception:
                    pass   # obstacle clustering is a bonus, never a gate
                # `wind_bearing_fn`, ADDED for the DENSITY pass (2026-08-31,
                # user: "the debris in the flooded area needs to increase a
                # lot in number"): the same `hu.wind_bearing_at(hcfg, x, y)`
                # lambda the "# LAND DEBRIS" block below already builds,
                # threaded here too so `raft_specs`'s new mid-water
                # drift-line pass (`washaway._drift_line_specs`) has a real
                # wind direction to align to instead of authoring nothing.
                #
                # HOUSE-MATCHED SKIN, ADDED (DEBRIS D5 review: "the debris
                # placed in the water all look like they have the same
                # texture. match them to the houses they are near"). `wrecks`
                # (built during the house loop, §3 above) already carries
                # each non-pristine house's own baked-style PALETTE, exactly
                # the same list `land_debris_specs`'s `skins_fn` reads below
                # — `wash.raft_specs`'s new optional 4th `houses` tuple field
                # wants a `mh.palette_skins(...)`-shaped dict, not a raw
                # palette name, so it is resolved here once per house rather
                # than per raft.
                # WIDENED, DEBRIS D6 review (2026-09-01): "match them to the
                # nearest house's colors" — the ORIGINAL loop below only
                # covered `wrecks` (non-pristine houses), so a raft's
                # nearest neighbour being a PRISTINE house (the common case
                # on a lightly-damaged plate) fell straight through to
                # `_RAFT_TINT`'s generic tint no matter how close it stood.
                # `washaway._apply_nearest_house_skin` (inside `raft_specs`
                # now) does the actual nearest-neighbour search over the
                # WHOLE `houses` list it is handed, so all this loop has to
                # do is make sure every house — not just the wrecked ones —
                # has an entry here: every `_h_recs` record's own style has a
                # baked default palette (`mh.STYLES[style]["palette"]`), the
                # same fallback `wrecks`' own entries already use.
                _house_skins_by_xy = {}
                for _rec in _h_recs:
                    _pal = mh.STYLES.get(_rec["style"], {}).get("palette")
                    if _pal:
                        try:
                            _house_skins_by_xy[(float(_rec["x"]), float(_rec["y"]))] \
                                = mh.palette_skins(_pal)
                        except Exception:
                            pass
                # THE PLATE'S OWN NEIGHBOURHOOD PALETTE, for pieces beyond
                # `washaway._HOUSE_SKIN_MATCH_R_M` of every house — one skin
                # dict per distinct STYLE actually built here (not per
                # house), so a piece far from any specific house still draws
                # a colour a real house on THIS plate wears rather than a
                # single hardcoded generic tint. Never falls back to
                # anything green: every name comes from `mh.PALETTES`.
                _skin_pool_by_style = {}
                for _rec in _h_recs:
                    _st = _rec["style"]
                    if _st in _skin_pool_by_style:
                        continue
                    _pal = mh.STYLES.get(_st, {}).get("palette")
                    if _pal:
                        try:
                            _skin_pool_by_style[_st] = mh.palette_skins(_pal)
                        except Exception:
                            pass
                _rs = wash.raft_specs(
                    _wcfg, region, wrng,
                    [(r["x"], r["y"], fp_by_style.get(r["style"], 12.0),
                     _house_skins_by_xy.get((float(r["x"]), float(r["y"]))))
                    for r in _h_recs],
                    _depth_fn, kind_weights=_kw, obstacles=_obstacles,
                    wind_bearing_fn=lambda x, y: hu.wind_bearing_at(hcfg, x, y),
                    skin_pool=list(_skin_pool_by_style.values()))
                # MATERIALS FOR WHATEVER SKINS THE FIELD ACTUALLY DREW — one
                # `planks.skin_material` per DISTINCT name found on `_rs`,
                # the same `mh.palette_texture` + `planks.skin_material` pair
                # the "# LAND DEBRIS" block below already uses for its own
                # skinned pieces (and the SAME material-name vocabulary, so
                # a plate that uses both never builds the same look twice
                # under two different paths).
                _raft_skin_names = sorted({s["skin"] for s in _rs
                                          if s.get("skin")})
                _raft_skin_mats = {}
                for _n in _raft_skin_names:
                    try:
                        _tex, _tint = mh.palette_texture(stage, PARENT, _n)
                        if _tex:
                            _raft_skin_mats[_n] = planks.skin_material(
                                stage, "{0}/RaftLooks/skin_{1}"
                                      .format(PARENT, _n),
                                _tex, tint=_tint,
                                tile_m=1.35 if _n.startswith("shingle")
                                else 1.05)
                    except Exception as _skexc:
                        print("[hurricane] raft skin {0} unavailable: {1}"
                             .format(_n, _skexc))
                made_w["rafts"] = wash.build_rafts(
                    stage, PARENT + "/rafts", _rs, ssf=ssf,
                    skin_mats=_raft_skin_mats)
                print("[hurricane] debris rafts: {0} piece(s), {1} "
                     "house-matched skin group(s) ({2})".format(
                         len(_rs), len(_raft_skin_mats),
                         ", ".join(sorted(_raft_skin_mats)) or "none"))

                # FLOATING FALLEN TREES — ADDED (DEBRIS D5 review, second
                # half of "I don't really see floating tree trunks/fallen
                # down trees in the flood. Add that."): whole
                # `tree_<species>_fallen.usd` archetypes, the SAME baked
                # library `arch` (§5 TREES, above) already references for
                # windthrow on DRY ground, referenced a second time in OPEN
                # water with a Z translate that seats the TRUNK at the
                # waterline instead of on grade. `wash.floating_tree_specs`
                # is pure Python (no `pxr`); this loop is the "small
                # reference loop" its own docstring points to, following the
                # tree pass's own `_ref` idiom (`_ref` itself always writes
                # Z=0, so a full 3-component translate needs its own copy of
                # the pattern rather than a call to that shared helper).
                try:
                    _float_tree_names = [n for n in wash._FLOAT_TREE_TRUNK_M
                                         if n in arch]
                    if not _float_tree_names:
                        raise RuntimeError(
                            "no tree_*_fallen archetype in ARCH_DIR ({0})"
                            .format(ARCH_DIR))
                    # HOUSES ONLY -- NOT `_obstacles` (trees, cars).
                    # `floating_tree_specs`'s own "HOUSES ONLY, DELIBERATELY"
                    # docstring section records why: this plate's ~1,684
                    # trees, fed in as a second exclusion population the way
                    # `raft_specs`'s `obstacles=` works, were MEASURED to
                    # leave almost no point on the whole plate more than
                    # 12 m from its nearest tree, and cut the placed count
                    # from 27 to 2 on the real L3 layout.
                    _float_specs = wash.floating_tree_specs(
                        _wcfg, region, wrng, _depth_fn,
                        houses=[(r["x"], r["y"],
                                fp_by_style.get(r["style"], 12.0))
                               for r in _h_recs],
                        archetypes=_float_tree_names)
                    _ft_scope = PARENT + "/rafts/float_trees"
                    UsdGeom.Scope.Define(stage, Sdf.Path(_ft_scope))
                    _n_float = 0
                    for _fi, _fs in enumerate(_float_specs):
                        _fusd = arch.get(_fs["archetype"])
                        if not _fusd:
                            continue
                        _fp_path = "{0}/t_{1}".format(_ft_scope, _fi)
                        _fprim = stage.DefinePrim(Sdf.Path(_fp_path), "Xform")
                        if not _fprim.GetReferences().AddReference(_fusd):
                            continue
                        _fxf = UsdGeom.Xformable(_fprim)
                        _fxf.AddTranslateOp().Set(Gf.Vec3d(
                            _fs["x"] * ssf, _fs["y"] * ssf, _fs["z"] * ssf))
                        _fxf.AddRotateZOp().Set(float(_fs["yaw"]))
                        _fprim.SetInstanceable(True)
                        _n_float += 1
                    made_w["float_trees"] = _n_float
                    print("[hurricane] floating fallen trees: {0} placed "
                         "(species: {1})".format(
                             _n_float, ", ".join(sorted(
                                 set(s["archetype"]
                                    for s in _float_specs)))))
                except Exception as _ftexc:
                    print("[hurricane] floating fallen trees FAILED: {0}"
                         .format(_ftexc))
            except Exception as _rexc:
                print("[hurricane] debris rafts FAILED: {0}".format(_rexc))

            # LAND DEBRIS — wind-torn sheathing, shingle sheets, siding and
            # framing littering the yard downwind of a damaged house. THE
            # TORNADO'S OWN MACHINERY (`washaway.land_debris_specs` wraps
            # `planks.scatter_from_wreck` exactly as the tornado's plank
            # field does, same comet shape, same "the debris wears what the
            # house wore" skinning), FLOOD-AWARE: budget keyed to LEVEL
            # rather than to `HUR_DEBRIS` alone, and a `# REGION FIELD`
            # addition (below) that only ever lands on dry ground.
            #
            # THE BUDGET IS SPLIT BY LEVEL, not read off intensity alone.
            # `wrecks[i][3]` is the RAW field intensity, and this file's
            # storm is near-uniform (`hurricane.intensity_field`'s own
            # docstring: single digits of percent spread across a plate) —
            # measured on the real GT populations, `it` sits in a ~0.10-wide
            # band regardless of level (L2 0.50-0.60, L3 0.65-0.77), so an
            # intensity-only budget (the tornado's own approach, where
            # level and intensity move together along one corridor gradient)
            # would hand a `roof_stripped` house nearly the SAME piece count
            # as a `leveled` one on this plate. Splitting the budget by level
            # is what "roof_stripped gets fewer" actually requires here.
            # `HUR_DEBRIS` sets the STRUCTURAL ceiling (`TOR_PLANKS`'s own
            # 140 default), matching the tornado's own per-wreck budget at
            # its highest levels; the light tier is capped well under it.
            try:
                if not DO_WASHAWAY:
                    raise RuntimeError("washaway disabled (HUR_WASHAWAY=0)")
                _lrng = random.Random(SEED + 97)
                _n_hi_heavy = float(N_DEBRIS)          # TOR_PLANKS's own 140
                _n_lo_heavy = 0.5 * _n_hi_heavy
                _n_hi_light = min(60.0, 0.43 * _n_hi_heavy)
                _n_lo_light = 0.42 * _n_hi_light
                _wrecks_light = [w for w in wrecks if w[4] == "roof_stripped"]
                _wrecks_heavy = [w for w in wrecks
                                 if w[4] in ("roof_collapsed",
                                            "partial_collapse", "leveled")]
                # THE DEBRIS WEARS WHAT THE HOUSE WORE — the same
                # `mh.palette_skins`/`mh.palette_texture` re-projection the
                # tornado plank field already uses, so a `siding`/`deck`
                # piece off a coastal-blue cottage is that cottage's own
                # wall/roof material, not a generic fallback tint.
                # LAND DEBRIS ON SUBMERGED GROUND (D2 review): a house still
                # standing IN the flood sheds wind debris same as a dry one,
                # but a comet that lands on ground actually underwater
                # should float, not sit invisibly on the sea floor under an
                # opaque water body. `_depth_fn`/`_wcfg` are the SAME ones
                # the "# DEBRIS RAFTS" block above already built for
                # `raft_specs` — reused, not recomputed — so land debris and
                # rafts agree on exactly where the water is.
                _skins_fn = lambda w: mh.palette_skins(w[5]) if w[5] else None
                _bearing_fn = lambda x, y: hu.wind_bearing_at(hcfg, x, y)
                _ld, _ld_rafts = [], []
                if _wrecks_light:
                    _l, _r = wash.land_debris_specs(
                        _wrecks_light, _bearing_fn, _lrng,
                        min_level="roof_stripped",
                        n_lo=_n_lo_light, n_hi=_n_hi_light,
                        skins_fn=_skins_fn, depth_fn=_depth_fn,
                        water_level_m=_wcfg["water_level_m"])
                    _ld += _l
                    _ld_rafts += _r
                if _wrecks_heavy:
                    _l, _r = wash.land_debris_specs(
                        _wrecks_heavy, _bearing_fn, _lrng,
                        min_level="roof_collapsed",
                        n_lo=_n_lo_heavy, n_hi=_n_hi_heavy,
                        skins_fn=_skins_fn, depth_fn=_depth_fn,
                        water_level_m=_wcfg["water_level_m"])
                    _ld += _l
                    _ld_rafts += _r
                # NEAR-FIELD APRON — ISOTROPIC, ADDED 2026-09-01.
                #
                # WHY THE COMETS ARE NOT ENOUGH. `land_debris_specs` narrows
                # `scatter_from_wreck`'s cone deliberately (`tail_pow` down,
                # `tail_lateral_base` 0.30 -> 0.08) because the DEBRIS review
                # found a symmetric halo reading as wrong against a storm
                # with a wind direction: what a hurricane throws, it throws
                # DOWNWIND. That is still right for the far field and is not
                # touched here.
                #
                # It is wrong for the FIRST FOOTPRINT. Measured on this
                # plate's own people pass: `tornado_people._candidate` draws
                # a casualty at a UNIFORM RANDOM ANGLE off the wreck, so a
                # body drawn crosswind or upwind of a house landed on clean
                # lawn — 42% of dry casualties sat on a deck under 5 cm, i.e.
                # on bare grass next to a wrecked house, which is the "I see
                # dry casualties only in the open" complaint's second half
                # (the first half, the radius, is `hurricane_people`'s
                # `where_bands`). No radius change can fix it: the debris
                # simply is not there to lie on at most bearings.
                #
                # AND THE PHYSICS SAYS IT SHOULD BE. The far-field trail is
                # material the WIND carried, and it is properly directional.
                # The near field is different in kind: it is the structure
                # that fell down, and a collapsing house sheds its own walls,
                # sill plate, roof deck and contents ISOTROPICALLY onto its
                # own lot before the wind has any say. Two mechanisms, two
                # shapes — a directional tail over an omnidirectional apron —
                # which is also what the Andrew/Ian case material describes
                # (victims "found in destroyed mobile home", "collapsing
                # roof": the material is on the lot, all round).
                #
                # SIX SHORT COMETS AT 60 deg SPACING rather than new geometry
                # code: `reach` is 1.05 footprints, which is exactly where
                # `hurricane_people`'s retuned `yard` band now stops, so the
                # apron covers every radius a casualty can be drawn at.
                # `base_frac=0.0` keeps all of it OUT of the slab cloud the
                # comets already lay; `tail_pow=1.0` spreads it evenly along
                # that short reach instead of piling it on the wall line; the
                # wide lateral coefficients are `scatter_from_wreck`'s own
                # defaults, so the six cones overlap into a ring rather than
                # reading as six spokes.
                #
                # 300 IS A MEASURED NUMBER, NOT A GUESS, and the first
                # attempt at 48 was worth almost nothing. Pooled over 5 seeds
                # on a 16-wreck archetype fixture, sharing the deck with the
                # real directional comet, share of dry casualties left lying
                # on BARE GROUND (measured deck under 5 cm):
                #
                #     comet only ................ 45%
                #     + apron  48/wreck ......... 42%   (+576 planks)
                #     + apron 300/wreck ......... 13%   (+3744 planks)
                #
                # The reason 48 fails is areal coverage, and it is simple
                # arithmetic: 48 pieces of ~0.2 m2 spread over the ~680 m2
                # annulus inside 1.05 footprints is 1.4% of the ground, so a
                # body's three deck stations almost never land on one. 300
                # takes it to roughly a tenth of the ground, which is where
                # the three-station test starts passing.
                #
                # REWEIGHTING IS NOT AN ALTERNATIVE, also measured: pushing
                # `where` to Andrew's raw 0.67/0.17/0.16 with no apron only
                # reaches 38%, and adding it ON TOP of the 300 apron gives
                # 12% against 13% — inside the noise. So the weights stay
                # where the epidemiology put them and the debris field is
                # what changes, which is the right place for it: this was
                # never a question about where casualties are, it was a
                # question about what is lying on the ground when they get
                # there.
                #
                # THE COST IS GEOMETRY-CHEAP. `planks.build` merges to ONE
                # MESH PER (class, skin), so ~6.8k extra boxes on this
                # plate's 29 wrecks is ~54k extra points across a handful of
                # meshes, not 6.8k prims. `HUR_DEBRIS_APRON=0` disables it.
                _apron_n = int(_env("HUR_DEBRIS_APRON", "300"))
                if _apron_n > 0:
                    _apron = []
                    _apron_wet = 0
                    for _w in (_wrecks_light + _wrecks_heavy):
                        _wfp = float(_w[2])
                        _wskins = mh.palette_skins(_w[5]) if _w[5] else None
                        for _k in range(6):
                            _apron += planks.scatter_from_wreck(
                                float(_w[0]), float(_w[1]), _wfp,
                                float(_w[3]), 60.0 * _k + _lrng.uniform(-12.0, 12.0),
                                1.05 * _wfp, _lrng,
                                n_pieces=max(1, _apron_n // 6),
                                ground_z=wash.LAND_DEBRIS_GROUND_Z_M,
                                skins=_wskins, base_frac=0.0, tail_pow=1.0)
                    # SAME SUBMERGED MASK the region field uses below: a
                    # piece that lands in real water is the raft field's job.
                    for _sp in _apron:
                        if _depth_fn(_sp["x"], _sp["y"]) > \
                                wash.LAND_DEBRIS_SUBMERGED_DEPTH_M:
                            _apron_wet += 1
                            continue
                        _ld.append(_sp)
                    print("[hurricane] land debris near-field apron: {0} "
                         "piece(s) round {1} wreck(s), {2} dropped (submerged)"
                         .format(len(_apron) - _apron_wet,
                                 len(_wrecks_light) + len(_wrecks_heavy),
                                 _apron_wet))

                # # REGION FIELD — loose litter across the plate at large,
                # not just on a damaged lot, the same argument the tornado's
                # `scatter_over_region` makes for its corridor ("what is
                # lying there came from somewhere else"). MASKED TO DRY
                # GROUND ONLY (`depth <= 0.10 m`, the same
                # `LAND_DEBRIS_SUBMERGED_DEPTH_M` floor `land_debris_specs`
                # itself uses to decide "is this piece actually underwater"):
                # a piece the mask would place in real water is the RAFT
                # field's job (`raft_specs`, above) and is dropped here
                # rather than double-placed or converted, exactly as this
                # stream's own brief asks. `hcfg["heading_deg"]` (the storm's
                # dominant heading) stands in for `heading_deg` because
                # `planks.scatter_over_region` takes one fixed bearing, not a
                # per-point callable — the per-house comets above are what
                # carry `wind_bearing_at`'s spatial variation; this field is
                # ambient background litter, not a directional signature.
                _region_per100 = float(_env("HUR_DEBRIS_REGION_PER100", "1.0"))
                if _region_per100 > 0.0:
                    _region_specs = planks.scatter_over_region(
                        region, inten, float(hcfg["heading_deg"]), _lrng,
                        per_100m2=_region_per100, cell_m=14.0,
                        ground_z=wash.LAND_DEBRIS_GROUND_Z_M,
                        min_intensity=0.12)
                    _n_region_wet = 0
                    for _sp in _region_specs:
                        if _depth_fn(_sp["x"], _sp["y"]) > \
                                wash.LAND_DEBRIS_SUBMERGED_DEPTH_M:
                            _n_region_wet += 1
                            continue
                        _ld.append(_sp)
                    print("[hurricane] land debris region field: {0} piece(s) "
                         "on dry ground, {1} dropped (submerged, the rafts' "
                         "job)".format(len(_region_specs) - _n_region_wet,
                                      _n_region_wet))
                # SKIN MATERIALS BUILT ONCE, BEFORE either consumer — moved
                # ahead of the `build_rafts` call below (DEBRIS D5 review:
                # "match them to the houses they are near"). `_ld_rafts`
                # (land debris that turned out to land on submerged ground,
                # `land_debris_specs`'s own `depth_fn` branch) now carries a
                # `"skin"` field too, preserved from the land piece it
                # converted from — a house-matched skin should not vanish
                # just because the piece floats instead of lying on grass —
                # so the skin-name set has to union BOTH lists before either
                # is built, not just `_ld`'s.
                _ld_skins = sorted({s["skin"] for s in _ld if s.get("skin")}
                                   | {s["skin"] for s in _ld_rafts
                                      if s.get("skin")})
                _ld_skin_mats = {}
                for _n in _ld_skins:
                    try:
                        _tex, _tint = mh.palette_texture(stage, PARENT, _n)
                        if _tex:
                            _ld_skin_mats[_n] = planks.skin_material(
                                stage, "{0}/LandDebrisLooks/skin_{1}"
                                      .format(PARENT, _n),
                                _tex, tint=_tint,
                                tile_m=1.35 if _n.startswith("shingle")
                                else 1.05)
                    except Exception as _skexc:
                        print("[hurricane] land debris skin {0} "
                             "unavailable: {1}".format(_n, _skexc))
                if _ld_rafts:
                    made_w["land_debris_rafts"] = wash.build_rafts(
                        stage, PARENT + "/rafts_from_land", _ld_rafts,
                        ssf=ssf, skin_mats=_ld_skin_mats)
                made_w["land_debris"] = planks.build(
                    stage, PARENT + "/land_debris", _ld,
                    planks.materials(stage, PARENT + "/land_debris"), ssf,
                    skin_mats=_ld_skin_mats)
            except Exception as _lexc:
                print("[hurricane] land debris FAILED: {0}".format(_lexc))

            # FENCES — "fences stand intact in 2 m of surge" (DEBRIS D3
            # review). `suburb_scene`/`scene_generator.apply_placements`
            # lands ~600 picket/rail panels under
            # `PARENT + "/fence_<group>_<i>"`, one placement per panel, and
            # nothing above this point (the wind pass, `apply_washaway`/
            # `apply_car_washaway`, the raft/land-debris blocks) ever reads
            # one. `wash.fence_specs` is the pure decision (measured
            # geometry + depth/wind/house callables -> gone/flat/stands);
            # `wash.apply_fence_pose` is the stage edit. This block is
            # ONLY the walk between them: find every `fence_*` prim under
            # `PARENT`, measure it (`wash.measure_fence`, points-based —
            # see that function's own docstring), decide, apply, and merge
            # whatever floated loose into the SAME per-kind raft mesh the
            # rest of the field uses.
            try:
                if not DO_WASHAWAY:
                    raise RuntimeError("washaway disabled (HUR_WASHAWAY=0)")
                _frng = random.Random(SEED + 131)
                _froot = stage.GetPrimAtPath(Sdf.Path(PARENT))
                _fence_paths = [
                    str(_cp.GetPath())
                    for _cp in (Usd.PrimRange(_froot)
                               if _froot and _froot.IsValid() else ())
                    if _cp.GetName().startswith("fence_")]
                _fence_geo = [
                    wash.measure_fence(stage, stage.GetPrimAtPath(_pth),
                                       ssf=ssf)
                    for _pth in _fence_paths]
                # `inten` -- NOT a freshly-built `hu.intensity_field(...)`.
                # That closure was already made once, at the top of this
                # script (`inten = hu.intensity_field(hcfg, region, np.
                # random.default_rng(SEED + 23))`), and is what EVERY
                # house's own wind-damage LEVEL was read from
                # (`house_level_for_intensity(inten(h["x"], h["y"]), ...)`
                # in the wind pass above). Building a second one here from
                # `wrng` -- a plain `random.Random` already advanced by
                # however many draws every earlier block in this function
                # happened to make -- would give the fence wind-percentile
                # check a DIFFERENT small-noise realisation of the same
                # field than the one that actually damaged the houses, and
                # a non-reproducible one (its exact draw depends on call
                # order elsewhere in this function). Reusing `inten`
                # verbatim is both more correct and simpler.
                _fdecisions = wash.fence_specs(
                    _fence_geo, _depth_fn,
                    lambda x, y: hu.wind_bearing_at(hcfg, x, y), inten,
                    wrecks, _wcfg["water_level_m"], _frng)
                _n_fgone = _n_fflat = _n_fstand = 0
                _fence_rafts = []
                for _pth, _fdec in zip(_fence_paths, _fdecisions):
                    wash.apply_fence_pose(stage, _pth, _fdec, ssf=ssf)
                    _n_fgone += int(_fdec["action"] == "gone")
                    _n_fflat += int(_fdec["action"] == "flat")
                    _n_fstand += int(_fdec["action"] == "stands")
                    _fence_rafts.extend(_fdec.get("rafts") or ())
                if _fence_rafts:
                    made_w["fence_rafts"] = wash.build_rafts(
                        stage, PARENT + "/rafts_from_fences", _fence_rafts,
                        ssf=ssf)
                made_w["fences"] = "{0} ({1} gone, {2} flat, {3} standing)" \
                    .format(len(_fence_paths), _n_fgone, _n_fflat, _n_fstand)
            except Exception as _fexc:
                print("[hurricane] fences FAILED: {0}".format(_fexc))

            # STREET FURNITURE — signs, streetlights, bins, hydrants.
            # ADDED 2026-09-01, user: "I don't see fallen over [signs], stop
            # signs, etc. Those should also be thrown away/uprooted."
            #
            # Nothing in this repo read those prims before: 45 signs, 37
            # streetlights, 23 bins and 19 hydrants stood pristine through a
            # Cat-3 wind field. `disaster.street_furniture` is the same
            # pure-decision / stage-apply split `fence_specs` +
            # `apply_fence_pose` use, and reuses the SAME `_depth_fn` and
            # `inten` closures the fence and debris passes already read, so
            # every pass agrees about where the water and the wind are
            # rather than each realising its own field.
            #
            # DISCRIMINATING, NOT FLATTENING, which is the whole point of
            # modelling it per class: a bin is the most wind-vulnerable
            # object on the plate and essentially always leaves, a sign
            # shears or uproots, a steel streetlight column mostly SURVIVES,
            # and a cast-iron hydrant bolted to a buried main NEVER moves —
            # measured 0 of 8000 at maximum intensity, and it is the
            # deliberate control that proves the pass is choosing rather
            # than knocking everything down.
            try:
                _sf_cats = ("sign", "streetlight", "trash_can",
                            "fire_hydrant")
                _sf_paths, _sf_items = [], []
                for _cp in Usd.PrimRange(stage.GetPrimAtPath(Sdf.Path(PARENT))):
                    _cat = None
                    for _c in _sf_cats:      # prefix match, NOT split("_"):
                        if _cp.GetName().startswith(_c + "_"):
                            _cat = _c        # `trash_can` contains the
                            break            # delimiter itself
                    if _cat is None:
                        continue
                    _sf_paths.append(str(_cp.GetPath()))
                    _sf_items.append(sfu.measure_street_item(
                        stage, _cp, _cat, ssf=ssf))
                if _sf_items:
                    _sf_dec = sfu.street_furniture_specs(
                        _sf_items, _depth_fn,
                        lambda x, y: hu.wind_bearing_at(hcfg, x, y), inten,
                        random.Random(SEED + 151))
                    _sf_tally = {}
                    for _pth, _dec in zip(_sf_paths, _sf_dec):
                        sfu.apply_street_furniture_pose(stage, _pth, _dec,
                                                        ssf=ssf)
                        _k = "{0}:{1}".format(_dec["category"],
                                              _dec["action"])
                        _sf_tally[_k] = _sf_tally.get(_k, 0) + 1
                    made_w["street_furniture"] = "{0} item(s) {1}".format(
                        len(_sf_items), dict(sorted(_sf_tally.items())))
            except Exception as _sfexc:
                import traceback
                print("[hurricane] street furniture FAILED: {0}"
                      .format(_sfexc))
                traceback.print_exc()

            print("[hurricane] water in {0:.0f}s: {1}".format(
                time.time() - tw,
                ", ".join("{0} {1}".format(k, v) for k, v in made_w.items())))
        except _SurgeDisabled:
            # SURGE=0, deliberate -- already announced at the raise site.
            # Caught BEFORE the generic handler so an intentional skip is
            # never reported as "WATER PASS FAILED".
            pass
        except Exception as exc:
            import traceback
            print("[hurricane] WATER PASS FAILED: {0}".format(exc))
            traceback.print_exc()

    # 7) THE WET GROUND -----------------------------------------------------
    #
    # The colour under the water and around it. Same z-ladder slot the burn
    # scar and the tornado's scour take — BETWEEN GRASS AND ASPHALT — so
    # roads, drives and walks come through unmuddied, which is correct (they
    # are pavement) and keeps the street grid legible, exactly what the
    # reference aerials show through a flooded neighbourhood.
    if DO_GROUND:
        try:
            zs = float(binfo.get("z_scale") or ss.ground_z_scale(config, region))
            mud_z = (ss._Z_GRASS + 0.5 * (ss._Z_ASPHALT - ss._Z_GRASS)) * zs
            # `ground.knobs_from_env`, NOT `hurricane.knobs_from_env`. The
            # first render died here with `KeyError: 'bands'`: the two modules
            # both publish a `knobs_from_env`, but only `ground`'s returns the
            # overlay's own vocabulary (`cell_m`, `bands`, `tile_m`,
            # `op_range`) — `hurricane`'s returns FIELD knobs. Same name,
            # different contract, and the overlay is the caller here.
            kn = ground.knobs_from_env(span)
            # `surge.silt_coverage`, not a field derived here from
            # `depth_at`. They are not equivalent: `depth_at`'s public
            # contract is CLAMPED at zero, so through it a point one metre
            # past the shoreline and a point a kilometre inland are both
            # exactly 0.0 and indistinguishable. `silt_coverage` is built on
            # the module's unclamped signed depth and can therefore put a
            # soft edge in the right place instead of a step.
            cov = sgw.silt_coverage(scfg, region,
                                    np.random.default_rng(SEED + 31))

            # WET vs DRY SILT, split by distance from the CURRENT waterline
            # (`sgw.signed_depth_at`, unclamped — never `sgw.depth_at`'s
            # clamped output, same reason every hump inside `surge.py` uses
            # the signed form). Reviewed: "flat brown polygons... wearing a
            # photograph of DRY cracked earth", including right at the
            # water's edge. `SILT_TEXTURE` genuinely IS that dry-earth
            # photograph — correct for silt that has had days to dry out
            # further inland, wrong for silt the surge only just uncovered.
            # `sgw.WET_SILT_TEXTURE` (derived: darker, more saturated, a
            # flattened normal, lower roughness — see `disaster.surge`'s WET
            # SILT comment block) takes over inside `sgw._DEPOSIT_WET_BAND_M`
            # of the waterline; `SILT_TEXTURE` keeps the drier, outer extent
            # of the same coverage field unchanged.
            sd = sgw.signed_depth_at(scfg, region, None)
            wet_band_m = float(os.environ.get(
                "SILT_WET_BAND_M", sgw._DEPOSIT_WET_BAND_M))

            def _wet_cov(x, y):
                return cov(x, y) if abs(sd(x, y)) <= wet_band_m else 0.0

            def _dry_cov(x, y):
                return cov(x, y) if abs(sd(x, y)) > wet_band_m else 0.0

            # SOFTER OPACITY FLOOR + MORE BANDS than `ground.knobs_from_env`'s
            # shared default (0.30-0.97 over 12 bands, tuned for the BURN
            # SCAR, which this call also drives via `kn`). At `op_lo=0.30`,
            # the `coverage_at <= 0.06` skip inside `ground.build_overlay`
            # jumps straight from "no overlay at all" to a ~33% opacity band
            # the instant a cell crosses that threshold — the "hard darker
            # outline" the review named around every silt patch. Kept
            # `SILT_`-prefixed (not `GROUND_*`) so retuning it does not also
            # retune the fire scar, which reads the same `GROUND_*` knobs.
            silt_bands = int(os.environ.get("SILT_BANDS", "0") or 0) or max(
                24, int(kn["bands"]))
            op_lo = float(os.environ.get("SILT_OPACITY_MIN", "0.04"))
            op_hi = float(os.environ.get("SILT_OPACITY_MAX", "")
                         or kn["op_range"][1])
            # You cannot silt water. A pool is a hole in the lawn with water
            # below grade; an overlay cell drawn over it is a film floating
            # on the surface. Same exclusion the burn scar makes.
            pool_skip = ground.skip_rects(binfo.get("pool_rects") or (),
                                          pad=0.0)

            made_wet = ground.build_overlay(
                stage, _wet_cov, region, ssf, mud_z, material_parent=PARENT,
                root="/World/siltGroundWet", cell_m=kn["cell_m"],
                bands=silt_bands, tile_m=kn["tile_m"], op_range=(op_lo, op_hi),
                texture=sgw.WET_SILT_TEXTURE,
                normal_tex=sgw.WET_SILT_NORMAL_TEXTURE,
                orm_tex=sgw.WET_SILT_ORM_TEXTURE, roughness=0.18,
                skip=pool_skip)
            made_dry = ground.build_overlay(
                stage, _dry_cov, region, ssf, mud_z, material_parent=PARENT,
                root="/World/siltGroundDry", cell_m=kn["cell_m"],
                bands=silt_bands, tile_m=kn["tile_m"], op_range=(op_lo, op_hi),
                texture=sgw.SILT_TEXTURE, normal_tex=sgw.SILT_NORMAL_TEXTURE,
                orm_tex=sgw.SILT_ORM_TEXTURE, roughness=0.85, skip=pool_skip)
            made_g = made_wet + made_dry
            print("[hurricane] silt overlay: {0} cell(s) ({1} wet, {2} dry, "
                 "wet band {3:.1f} m)".format(
                     len(made_g), len(made_wet), len(made_dry), wet_band_m))
        except Exception as exc:
            print("[hurricane] silt overlay FAILED: {0}".format(exc))

    # 7c) THE PEOPLE ----------------------------------------------------------
    #
    # See `disaster/hurricane_people.py`'s own module docstring for the full
    # design. In one line: DRY LAND reuses `tornado_people`'s wrecked-house
    # casualty model VERBATIM, restricted to the houses this scene's own
    # surge field calls dry; IN THE WATER stands a figure CHEST-DEEP
    # (`people.CHEST_FRAC`) near a wrecked house's own debris or the up-flow
    # face of a standing one; ON ROOFS — REWRITTEN 2026-09-01, user on the
    # live scene ("sitting on the sides... too close to the edge... intact
    # houses's roofs only") — seats 1-3 `sit_slump` figures per roof (PPL3: sit_ground rolled to full pitch read as a torso emerging from the roof) on the
    # measured PITCHED SLOPE (never the ridge apex, never the eave), tilted
    # to the slope's own measured pitch, on `pristine` houses ONLY, preferring
    # the flooded band.
    #
    # `depth`/`scfg`/`inten` are step 3's ("THE WATER") and step 2's ("THE
    # STORM") own objects, REUSED here rather than re-derived — same rule
    # `disaster.hurricane_people`'s own docstring states for `depth_at`.
    # `wrecks`/`_h_recs`/`fp_by_style` are step 4's ("HOUSES") own lists.
    #
    # LAST, DELIBERATELY, same reasoning `tornado_people`'s own launcher
    # gives: every pass above moves, deletes or re-materialises something,
    # and a survivor is not debris.
    n_people = n_water = n_roof = n_trap = 0
    p_recs = []
    # THE WRECK'S OWN DEBRIS SURFACE, SAMPLED OFF THE ARCHETYPE. Ported from
    # `suburb_tornado_launch_script.py` verbatim, 2026-09-01, on the user's
    # own report against the live 500 m plate: "I see dry casualties only in
    # the open."
    #
    # This is the SAME defect the tornado already hit and already fixed, and
    # `tornado_people._Deck`'s own docstring records it in the reviewer's
    # words: "I no longer see people inside the house debris, they seem to
    # only be surrounding it". The cause is that `_Deck` measures the PLANK
    # FIELD, and on a wrecked lot the boards are the THIN part of the debris
    # — the deep part is the baked wreck USD, which the planner cannot see
    # because it is a referenced INSTANCE and that module never touches a
    # stage. With no archetype samples the measured deck is ~0 over the whole
    # lot, `_DECK_BAND["pile"]`'s 0.03 m floor stops being cleared, and every
    # casualty is pushed off the wreckage onto the skirt and the yard.
    #
    # Measured on this plate BEFORE the fix: 0 of 30 dry casualties within a
    # footprint of any house, `pile` sitting at a median of 14.0 m from the
    # nearest house centre, 24 of 30 more than 12 m out. `_Deck` takes these
    # samples through `ctx["deck_points"]` and has since it was written; this
    # launcher was passing `[]`.
    #
    # `Usd.TraverseInstanceProxies` is what makes it possible: it walks INTO
    # an instance without authoring anything there, so each wreck's own
    # meshes are measured in world space and their tops stamped into the same
    # 0.8 m grid the boards use.
    # `tornado_people._Deck`'s own default cell. Sampling on the SAME grid the
    # deck is built on is what makes a stamped cell mean exactly one cell.
    _DECK_CELL_M = 0.8
    deck_points = []
    if DO_PEOPLE and _h_recs:
        try:
            _bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                                    [UsdGeom.Tokens.default_,
                                     UsdGeom.Tokens.render],
                                    useExtentsHint=True)
            for _hr in _h_recs:
                _pr = stage.GetPrimAtPath(Sdf.Path(_hr["prim_path"]))
                if not _pr or not _pr.IsValid():
                    continue
                for _q in Usd.PrimRange(_pr, Usd.TraverseInstanceProxies()):
                    if _q.GetTypeName() != "Mesh":
                        continue
                    _r = _bc.ComputeWorldBound(_q).ComputeAlignedRange()
                    if _r.IsEmpty():
                        continue
                    _mn, _mx = _r.GetMin(), _r.GetMax()
                    # DENSE, NOT ONE SAMPLE PER MESH. The tornado stamps each
                    # mesh's top at its PLAN CENTRE only, on the argument
                    # that "a wall is not something a body lies on" — true
                    # there, because `wreck_clear_m` keeps every body outside
                    # the wall line anyway. This file drops that keepout (see
                    # `hurricane_people.resolve_cfg`) so that casualties can
                    # lie INSIDE an open wrecked footprint, and the moment a
                    # body may be inside, the deck has to know where the
                    # walls are along their whole LENGTH: a centre-only
                    # sample leaves a 6 m wall reading as bare ground over
                    # every cell but one, and `_DECK_BAND` would happily
                    # accept a body lying through it.
                    #
                    # So stamp the mesh's top across its whole bbox plan
                    # extent, on the same 0.8 m grid `_Deck` uses. The
                    # 400-cell cap is `_Deck`'s own guard against a single
                    # house-sized bbox costing thousands of cells; past it,
                    # fall back to the centre sample.
                    _z = float(_mx[2]) / ssf
                    _x0, _x1 = _mn[0] / ssf, _mx[0] / ssf
                    _y0, _y1 = _mn[1] / ssf, _mx[1] / ssf
                    _i0, _i1 = int(_x0 // _DECK_CELL_M), int(_x1 // _DECK_CELL_M)
                    _j0, _j1 = int(_y0 // _DECK_CELL_M), int(_y1 // _DECK_CELL_M)
                    if (_i1 - _i0 + 1) * (_j1 - _j0 + 1) > 400:
                        deck_points.append((0.5 * (_x0 + _x1),
                                            0.5 * (_y0 + _y1), _z))
                        continue
                    for _ii in range(_i0, _i1 + 1):
                        for _jj in range(_j0, _j1 + 1):
                            deck_points.append((
                                (_ii + 0.5) * _DECK_CELL_M,
                                (_jj + 0.5) * _DECK_CELL_M, _z))
            print("[hurricane] debris surface: {0} sample(s) off {1} house "
                  "archetype(s)".format(len(deck_points), len(_h_recs)))
        except Exception as _exc:
            print("[hurricane] archetype deck sampling FAILED ({0}); the "
                  "people pass will see the board field only".format(_exc))
    if DO_PEOPLE:
        try:
            _hpp_cfg = hpp.resolve_cfg(config)
            _hpp_resolver = sg._make_resolver(config)
            _hpp_pools = ss.AssetPools(config)
            # THE RIGGED RENDERPEOPLE ONLY — a `posed_standing` static has no
            # skeleton and cannot take a pose. Same selection
            # `suburb_tornado_launch_script._rigged_humans` makes.
            from suburb_scene import _raw_pool as _hpp_raw_pool
            _hpp_raw = _hpp_raw_pool(config, "humans")
            _hpp_rigged = _hpp_pools.load_tagged(_hpp_raw, "rigged")
            if not _hpp_rigged:
                _hpp_posed = _hpp_pools.load_tagged(_hpp_raw, "posed_standing")
                _hpp_rigged = [u for u in _hpp_pools.load(_hpp_raw)
                              if u not in _hpp_posed]
            _hpp_ctx = {
                "region": region, "wrecks": wrecks, "houses": _h_recs,
                "fp_by_style": fp_by_style, "depth_at": depth,
                "water_level": sgw.water_level(scfg),
                "shore_bearing_deg": float(scfg["shore_bearing_deg"]),
                "intensity_at": inten, "humans": _hpp_rigged,
                "resolver": _hpp_resolver, "asset_pools": _hpp_pools,
                # THE LAND-DEBRIS FIELD, IF THE WATER PASS BUILT ONE — `_ld`
                # is a local of the "# LAND DEBRIS" block inside step 6) THE
                # WATER, assigned only when `DO_WATER`/`DO_WASHAWAY` both ran
                # clean. `locals().get` rather than a bare name so a skipped
                # or failed water pass degrades to `tornado_people`'s own
                # `_FlatDeck` fallback (`DEBRIS_Z_M`, flat) instead of an
                # `UnboundLocalError` here.
                "plank_specs": locals().get("_ld") or [],
                # SAMPLED ABOVE, not empty any more — see the block before
                # `if DO_PEOPLE`. This is what puts a casualty ON the wreck
                # instead of in the lawn beside it.
                "deck_points": deck_points,
            }
            # `SEED + 191` is the people draw's own offset; `1000 * VARIANT`
            # is the dataset's fifth axis (see `PEOPLE_VARIANT` at the top of
            # this file). NEVER fold the variant into `SEED` itself — that
            # would move the wind field, the houses, the trees and the cars
            # too, and the five k cells of a level would stop being one
            # geometry.
            p_humans, p_debris, p_recs = hpp.plan_people(
                _hpp_cfg, _hpp_ctx,
                random.Random(SEED + 191 + 1000 * PEOPLE_VARIANT))
            if PEOPLE_VARIANT:
                print("[hurricane] people variant {0}: rng seed {1} -> {2}"
                      .format(PEOPLE_VARIANT, SEED + 191,
                              SEED + 191 + 1000 * PEOPLE_VARIANT))
            if p_humans:
                sg.apply_placements(stage, p_humans, PARENT + "/people", ssf,
                                    resolver=_hpp_resolver,
                                    instance_categories=set())
            if p_debris:
                # THE BOARDS THAT MAKE A DRY-LAND CASUALTY *PARTIAL*, ADDED
                # 2026-09-01. `plan_people`'s second return value was being
                # bound and then never authored, so every dry figure this
                # scene ever shipped lay FULLY EXPOSED on the debris mat — the
                # `occlusion`/`covered_frac`/`visible_parts` written into
                # `PEOPLE_JSON` described a covering that did not exist on the
                # stage, and the ground truth was wrong for 24 of 28 dry
                # casualties on the reference plate. `_plan_dry` already runs
                # `tornado_people` verbatim and already solves each covering
                # piece against the body under it (`tornado_people._cover`);
                # the only thing missing here was this build, which is the
                # tornado launcher's own `trap_debris` block unchanged.
                #
                # Note the hurricane's own `occlusion` reweighting in
                # `hurricane_people.resolve_cfg` ("more under debris, fewer
                # lying in the open") has been feeding this list all along —
                # it only ever showed up in the JSON.
                #
                # THE POSE, THE THICKNESS AND THE TILT COME FROM THE PLANNER.
                # A propped piece runs from the debris surface up over the
                # casualty and carries a SOLVED pitch; randomising it here is
                # the difference between a board resting on somebody and a
                # board that has fallen off them. The `.get` defaults exist
                # only for a spec that predates that solve.
                _trng = random.Random(SEED + 193)
                _tspec = [{"x": d["x"], "y": d["y"], "z": d["z"],
                           "l": d["len"], "w": d["wide"], "yaw": d["yaw"],
                           "t": d.get("t", _trng.uniform(0.02, 0.05)),
                           "pitch": d.get("pitch", _trng.uniform(-8.0, 8.0)),
                           "roll": d.get("roll", _trng.uniform(-10.0, 10.0)),
                           "class": d.get("class",
                                          "sheathing" if d["wide"] > 0.4
                                          else "board")}
                          for d in p_debris]
                _tspec, _n_trap_off = planks.clip_to_region(_tspec, region)
                try:
                    planks.build(stage, PARENT + "/trap_debris", _tspec,
                                 planks.materials(stage,
                                                  PARENT + "/trap_debris"),
                                 ssf, verbose=False)
                    n_trap = len(_tspec)
                    print("[hurricane] people: {0} board(s) over trapped "
                          "figures ({1} clipped off-plate)".format(
                              n_trap, _n_trap_off))
                except Exception as _texc:
                    print("[hurricane] trap debris build FAILED: {0}"
                          .format(_texc))
            # `prim_path` LANDS ON THE PLACEMENT DICT, not the record —
            # `apply_placements` mutates each `p_humans[i]` in place (see
            # that function's own comment). `hurricane_people`'s own `_Field`-
            # style helpers append to `humans` and `records` TOGETHER, every
            # time, so the two lists are always the same length and the same
            # order — zipping them is exact, not a nearest-match guess.
            # `house_prim_path` (water/roof records only) IS a nearest-match
            # join, the same pattern `disaster.people.house_table` already
            # uses to attach a fire level to a plat house by position.
            for _prec, _pph in zip(p_recs, p_humans):
                _prec["prim_path"] = _pph.get("prim_path")
                if _prec.get("house_style") is not None:
                    _cands = [_hh for _hh in _h_recs
                             if _hh["style"] == _prec["house_style"]
                             and str(_hh["level"]) == str(
                                 _prec.get("house_level"))]
                    if _cands:
                        _best = min(_cands, key=lambda _hh: (
                            (_hh["x"] - _prec["x"]) ** 2
                            + (_hh["y"] - _prec["y"]) ** 2))
                        _prec["house_prim_path"] = _best["prim_path"]
            hpp.write_records(PEOPLE_JSON, p_recs, meta={
                "seed": SEED, "scene_config": SCENE_CONFIG,
                "water_level_m": sgw.water_level(scfg),
                "shore_bearing_deg": float(scfg["shore_bearing_deg"])})
            _ps = hpp.summarise(p_recs)
            n_people = _ps["by_domain"].get("dry_wreck", 0)
            n_water = _ps["by_domain"].get("water", 0)
            n_roof = _ps["by_domain"].get("roof", 0)
            print("[hurricane] people: {0} dry-land casualt(ies) under {4} "
                 "covering board(s), {1} in the water, {2} on a roof; "
                 "ground truth -> {3}".format(
                     n_people, n_water, n_roof, PEOPLE_JSON, n_trap))
            print("[hurricane] people water depths: {0}".format(
                _ps["water_depths"]))
        except Exception as exc:
            import traceback
            print("[hurricane] PEOPLE PASS FAILED: {0}".format(exc))
            traceback.print_exc()

    # 8) GROUND TRUTH -------------------------------------------------------
    gt_path = os.path.join(FREEZE_OUT or SNAP_DIR or ARCH_DIR,
                           "GT_hurricane.json")
    try:
        import json as _json
        with open(gt_path, "w") as fh:
            _json.dump({"scene": SCENE_CONFIG, "seed": SEED,
                        "region": list(region),
                        "hurricane": {k: (list(v) if isinstance(v, tuple) else v)
                                      for k, v in hcfg.items()},
                        "surge": {k: (list(v) if isinstance(v, tuple) else v)
                                  for k, v in scfg.items()},
                        "houses": _h_recs, "trees": _t_recs}, fh, indent=1)
        print("[hurricane] ground truth -> {0} ({1} house, {2} tree)"
              .format(gt_path, len(_h_recs), len(_t_recs)))
    except Exception as exc:
        print("[hurricane] GT write FAILED: {0}".format(exc))

    # 8b) THE DATASET GROUND TRUTH ------------------------------------------
    #
    # `GT_hurricane.json` above is this launcher's own record and stays as it
    # is. `GT_hints.json` is the DATASET's class vocabulary — the same file
    # every Fire and Tornado cell already carries — and `disaster/gt_hints.py`
    # is already hurricane-aware (`EXTRA_CLASSES["hurricane"]`,
    # `_HOUSE_DESTROYED["hurricane"]`). It wants `scene_api.build_scene`'s
    # `info_out` shape; this launcher's own locals are already most of the way
    # there and the gap is assembled here.
    if FREEZE_OUT:
        try:
            import json as _json
            from disaster import gt_hints as _gth

            # CARS: WALKED OFF THE STAGE, UNCONDITIONALLY. `binfo["cars"]` is
            # never filled by the suburb generator (`scene_api`'s own comment
            # says so), and this file's existing car walk lives inside
            # `if DO_WASHAWAY`, so a cell built with the wash-away pass off
            # would ship a hint file with no vehicles in it at all. Same
            # leaf-name route the wash-away pass takes — a placement's leaf
            # prim name IS `f"{category}_{group}_{i}"`.
            #
            # A FRESH XformCache. The wash-away, surge and settle passes all
            # re-author xformOps after any cache built earlier in `main()`,
            # and `UsdGeom.XformCache` memoises without invalidating — a
            # stale cache here would report every floated car at its
            # pre-drift pose. See the fence/pose bug in the skill's catalogue.
            _car_recs = []
            try:
                _xc = UsdGeom.XformCache(Usd.TimeCode.Default())
                _croot = stage.GetPrimAtPath(Sdf.Path(PARENT))
                for _cp in (Usd.PrimRange(_croot)
                            if _croot and _croot.IsValid() else ()):
                    if not _cp.IsA(UsdGeom.Xformable):
                        continue
                    if not _cp.GetName().lower().startswith("car_"):
                        continue
                    _m = _xc.GetLocalToWorldTransform(_cp)
                    _rot = _m.ExtractRotation()
                    _rpy = _rot.Decompose(Gf.Vec3d(1, 0, 0), Gf.Vec3d(0, 1, 0),
                                          Gf.Vec3d(0, 0, 1))
                    # THE REFERENCED ASSET, NOT THE PRIM NAME.
                    # `gt_hints.vehicle_class` decides Car/Van/Truck from the
                    # usd stem and nothing else, so a record with no `usd`
                    # falls back to the base class for every vehicle.
                    #
                    # READ OFF THE PRIM STACK, NOT `GetMetadata("references")`
                    # — that returns an `Sdf.ReferenceListOp`, which is not
                    # iterable, so the obvious one-liner raises `TypeError`
                    # on the FIRST car and takes the whole walk down with it
                    # (the handler below turns that into an empty vehicle
                    # class in the hint file, which is a quiet way to lose
                    # every car). A reference can also arrive on any of four
                    # list-op fields depending on how it was authored, so
                    # check all of them rather than assuming `prepended`.
                    _usd = ""
                    for _spec in _cp.GetPrimStack():
                        _rl = getattr(_spec, "referenceList", None)
                        if _rl is None:
                            continue
                        for _fld in ("prependedItems", "explicitItems",
                                     "addedItems", "appendedItems"):
                            for _it in (getattr(_rl, _fld, None) or []):
                                if getattr(_it, "assetPath", ""):
                                    _usd = str(_it.assetPath)
                                    break
                            if _usd:
                                break
                        if _usd:
                            break
                    _car_recs.append({
                        "prim_path": str(_cp.GetPath()), "usd": _usd,
                        "roll_deg": round(float(_rpy[0]), 2),
                        "pitch_deg": round(float(_rpy[1]), 2),
                        "yaw_deg": round(float(_rpy[2]), 2),
                        "axis_up": "Z", "heading_deg": None,
                        # NO CAR OCCUPANTS IN THIS SCENE. `hurricane_people`
                        # places dry-wreck, water and roof figures only —
                        # there is no car-cabin scenario here, so claiming
                        # `occupied` would be a label with nothing behind it.
                        "occupied": False,
                    })
            except Exception as _cexc:                           # noqa: BLE001
                print("[hurricane] car walk for GT_hints FAILED: {0}"
                      .format(_cexc))

            _info = {"parent": PARENT, "binfo": binfo,
                     "house_objects": _h_recs, "tree_objects": _t_recs,
                     "cars": _car_recs,
                     # NO ROAD-BLOCKAGE MODEL IN THIS PIPELINE. The wildfire/
                     # tornado "fallen tree across the carriageway" pass has
                     # no hurricane equivalent — this scene's land debris is a
                     # different, larger mechanism (`washaway.land_debris_
                     # specs` / `planks.scatter_from_wreck`). Empty is the
                     # honest value; it thins the Debris/Fallen Tree classes
                     # and breaks nothing.
                     "blockers": []}
            _recs = _gth.build(stage, _info, ssf, disaster="hurricane")
            _gth.write(os.path.join(FREEZE_OUT, "GT_hints.json"), _recs, meta={
                "scene_config": SCENE_CONFIG, "seed": SEED,
                "people_variant": PEOPLE_VARIANT, "disaster": "hurricane",
                "region_m": [float(v) for v in region],
                "site_gust_mps": hcfg.get("site_gust_mps"),
                "surge_m": scfg.get("surge_m"),
                "arch_dir": ARCH_DIR, "house_arch_dir": HOUSE_ARCH_DIR,
                "units": "metres, world frame, plate centred on the origin",
            })

            _stats = {
                "scene_config": SCENE_CONFIG, "seed": SEED,
                "people_variant": PEOPLE_VARIANT, "disaster": "hurricane",
                "region": [float(v) for v in region],
                "houses": len(_h_recs), "house_tally": htally,
                "era_tally": era_tally, "water_tally": wtally,
                "trees": len(_t_recs), "tree_tally": ttally,
                "cars": len(_car_recs), "car_tally": ctally,
                "people": len(p_recs or ()),
                "hurricane": {k: (list(v) if isinstance(v, tuple) else v)
                              for k, v in hcfg.items()},
                "surge": {k: (list(v) if isinstance(v, tuple) else v)
                          for k, v in scfg.items()},
                "arch_dir": ARCH_DIR, "house_arch_dir": HOUSE_ARCH_DIR,
                "hint_counts": _gth.summarise(_recs),
            }
            with open(os.path.join(FREEZE_OUT, "build_stats.json"), "w") as fh:
                _json.dump(_stats, fh, indent=1)
            print("[hurricane] build_stats -> {0}".format(
                os.path.join(FREEZE_OUT, "build_stats.json")))
        except Exception as exc:
            import traceback
            print("[hurricane] GT_hints/build_stats FAILED: {0}".format(exc))
            traceback.print_exc()

    # 9) LOOK AT IT ---------------------------------------------------------
    #
    # Nobody is at the GUI on a scripted run and a scene this size can only be
    # judged by looking at it, so the launcher photographs its own result.
    # `SNAP_DIR=` MUST sit under the mounted log directory or the PNGs land
    # somewhere the host cannot read them.
    _reassert_cutout()
    for _ in range(60):
        omni.kit.app.get_app().update()
    if SNAP_DIR:
        try:
            # RENDER PRODUCT FIRST, VIEWPORT SECOND. `snapshots.py` captures
            # the active viewport, which is the right tool on a desktop
            # container and segfaults on the headless livestream one — see
            # `snapshots_rp.py`'s header for the measured stack. The two
            # modules share an API, so this picks whichever one can actually
            # run here and the rest of this block does not care which.
            import importlib.util as _ilu
            _ud = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               "..", "utils")

            def _load(_name):
                _spec = _ilu.spec_from_file_location(
                    _name, os.path.normpath(os.path.join(_ud, _name + ".py")))
                _m = _ilu.module_from_spec(_spec)
                _spec.loader.exec_module(_m)
                return _m

            try:
                _snaps = _load("snapshots_rp")
                print("[hurricane] capture: replicator render product")
            except Exception as _e:
                print("[hurricane] render-product capture unavailable ({0}); "
                      "falling back to the viewport path".format(_e))
                _snaps = _load("snapshots")
                _snaps.hide_decorations()
            cx, cy = 0.5 * (region[0] + region[2]), 0.5 * (region[1] + region[3])
            import inspect
            # Only the render-product module takes `avoid`/`region`; the
            # viewport FALLBACK above is the shared `snapshots` module and
            # has neither. Passing either blind would TypeError inside the
            # outer handler and cost every snapshot on exactly the path
            # taken when the preferred one already failed -- so ask first.
            _ov_kw = {}
            if "region" in inspect.signature(_snaps.overview).parameters:
                _ov_kw["region"] = region
            _snaps.overview(stage, (cx, cy), max(rw, rh),
                            os.path.join(SNAP_DIR, "overview.png"), ssf,
                            **_ov_kw)
            # THE SUBJECTS ARE CHOSEN ALONG THE WATER GRADIENT, not along a
            # track — there is no track. What has to be judged here is (a) the
            # shoreline, (b) a flooded street, (c) dry damaged houses well
            # inland, so the ladder and the water can each be seen doing their
            # own job rather than being confused for one another.
            #
            # STREAM C, 2026-08-31: `hurricane_cameras_png.select_review_
            # subjects` is now the single source of truth for the FULL
            # 10-subject list -- the 3 water-gradient ones (`review_points`,
            # margin-fixed so none of them land on the plate's cut edge any
            # more), `worst_house`/`deepest_flooded_house`, and 5 NEW close
            # subjects (`stripped_roof_house`/`collapsed_house`/`raft_field`/
            # `fallen_tree`/`flooded_street`) so debris, house damage and
            # trees have something closer than a 45 m establishing shot to
            # be judged from. Offline-verified against this exact function in
            # `tests/test_review_points_cameras.py` and plotted (no render)
            # by `tools/hurricane_cameras_png.py`.
            _pts, _overrides = {}, {}
            try:
                _tools_dir = os.path.join(_SCENE_GEN_DIR, "tools")
                if _tools_dir not in sys.path:
                    sys.path.insert(0, _tools_dir)
                import hurricane_cameras_png as _hcp
                _gt_like = {"region": region, "surge": scfg,
                           "houses": _h_recs, "trees": _t_recs}
                for _name, _s in _hcp.select_review_subjects(_gt_like, depth_fn=locals().get("depth")).items():
                    _pts[_name] = (_s["x"], _s["y"])
                    if "obl_dist" in _s:
                        _overrides[_name] = {k: _s[k] for k in (
                            "obl_dist", "obl_h", "aim_h", "azimuth_deg")
                            if k in _s}
                    if _s.get("note"):
                        print("[hurricane] {0}: {1}".format(_name, _s["note"]))
                # `raft_field` above always used the documented FALLBACK (the
                # deepest flooded house's up-flow side) -- `GT_hurricane.json`
                # never carries raft specs, but THIS launcher just built the
                # real ones (`_rs`, step 6) and can do better when they made
                # it that far (`DO_WASHAWAY`/a raft-build failure both leave
                # `_rs` unbound -- `locals().get` rather than a bare name so
                # neither case is an exception here).
                _rs_live = locals().get("_rs")
                if _rs_live and "raft_field" in _pts:
                    _dc = _hcp.densest_cluster(_rs_live)
                    if _dc is not None:
                        _rx, _ry, _rn = _dc
                        _pts["raft_field"] = (_rx, _ry)
                        _overrides.setdefault("raft_field", {})[
                            "azimuth_deg"] = sgw._grad_deg(depth, _rx, _ry) % 360.0
                        print("[hurricane] raft_field: real cluster of {0} "
                              "piece(s) at ({1:.1f}, {2:.1f}) -- overriding "
                              "the up-flow-side fallback"
                              .format(_rn, _rx, _ry))
            except Exception as _e:
                print("[hurricane] select_review_subjects unavailable ({0}); "
                      "falling back to the water-gradient subjects only"
                      .format(_e))
            if not _pts:
                # TOTAL fallback (helper import/selection itself failed) --
                # the pre-STREAM-C behaviour, so a broken import never costs
                # every subject.
                try:
                    _pts.update(sgw.review_points(scfg, region))
                except Exception as _e:
                    print("[hurricane] review points unavailable: {0}"
                          .format(_e))
                if _h_recs:
                    _worst = max(_h_recs, key=lambda r: hu.HOUSE_LEVELS.index(
                        r["level"]) if r["level"] in hu.HOUSE_LEVELS else 99)
                    _pts["worst_house"] = (_worst["x"], _worst["y"])
                if not _pts:
                    _pts["centre"] = (cx, cy)
            # Trees are the only thing on this plate tall and dense enough to
            # stand between the oblique camera and its subject; hand over the
            # STANDING ones (a felled trunk lying flat is not a sightline
            # obstruction the way a canopy is) so the bearing can step off
            # one rather than shooting the flood through a canopy
            # (`snapshots_rp._clear_azimuth`). `region` lets that same
            # function ALSO require the bearing to look toward the plate's
            # interior, and lets `_cap_oblique_range` shrink a close
            # subject's range if its frame's top edge would otherwise
            # ray-trace past the plate — see `views_around`'s docstring.
            _kw = {}
            _params = inspect.signature(_snaps.views_around).parameters
            if "avoid" in _params:
                _kw["avoid"] = [(t["x"], t["y"]) for t in _t_recs
                               if t.get("level") not in ("fallen", "snapped")]
            if "region" in _params:
                _kw["region"] = region
            _default_pts = {k: v for k, v in _pts.items()
                           if k not in _overrides}
            if _default_pts:
                _snaps.views_around(stage, _default_pts, SNAP_DIR, ssf, **_kw)
            for _name, _xy in _pts.items():
                if _name not in _overrides:
                    continue
                _ckw = dict(_kw)
                _ckw.update(_overrides[_name])
                try:
                    _snaps.views_around(stage, {_name: _xy}, SNAP_DIR, ssf,
                                        **_ckw)
                except Exception as _e:
                    print("[hurricane] close subject {0} snapshot FAILED: {1}"
                          .format(_name, _e))
            print("[hurricane] snapshots -> {0} ({1} subject(s): {2})"
                  .format(SNAP_DIR, len(_pts), ", ".join(sorted(_pts))))

            # ONE CLOSE FRAME PER DRY CASUALTY (2026-09-01, user: "can you
            # show me photos of the people in house debris? I can't see
            # them"). The ten review subjects above are chosen for the
            # WATER and the HOUSES — none of them is aimed at a body, and a
            # casualty is a couple of metres of a 500 m plate, so at review
            # framing it is a few pixels. This shoots the casualties the
            # people pass actually placed, using their own recorded
            # positions, so what gets photographed is guaranteed to be a
            # figure rather than a hopeful bearing.
            #
            # `overview` (top-down), not `views_around` (oblique): the
            # people bench measured the oblique path returning a uniform
            # frame for close subjects at every distance tried while
            # `overview` rendered on every run, and a top-down is the right
            # view for an aerial dataset anyway. 9 m span puts the camera at
            # ~8.6 m (`overview` uses 0.95 * span), clear of this kit's
            # 5.6 m ridge, with a 1.8 m body across a fifth of the frame —
            # the bench proved a 5.5 m span sits BELOW the roofline and
            # renders the inside of a house.
            #
            # Named for the occlusion pattern each one demonstrates, so a
            # frame can be checked against the label this scene wrote into
            # `PEOPLE_JSON` for that same figure. Capped by
            # HUR_PEOPLE_SHOTS (0 disables) because each costs a render.
            _n_shots = int(_env("HUR_PEOPLE_SHOTS", "8"))
            _shot = [r for r in (p_recs or ())
                     if r.get("domain") == "dry_wreck"]
            if _n_shots > 0 and _shot:
                # Most-covered first: those are the ones worth judging, and
                # a fully exposed body is already visible in the wide views.
                _shot.sort(key=lambda r: -float(r.get("covered_frac") or 0.0))
                _seen, _made = set(), 0
                for _r in _shot:
                    if _made >= _n_shots:
                        break
                    _k = _r.get("occlusion") or "none"
                    if _k in _seen:          # one per pattern, for variety
                        continue
                    _seen.add(_k)
                    _nm = "casualty_{0}".format(_k)
                    try:
                        _snaps.overview(stage, (_r["x"], _r["y"]), 9.0,
                                        os.path.join(SNAP_DIR, _nm + ".png"),
                                        ssf=ssf)
                        _made += 1
                        print("[hurricane] {0}.png -> covered {1:.2f}, "
                              "visible {2}".format(
                                  _nm, float(_r.get("covered_frac") or 0.0),
                                  ",".join(_r.get("visible_parts") or ["-"])))
                    except Exception as _e:
                        print("[hurricane] casualty shot {0} FAILED: {1}"
                              .format(_nm, _e))
                print("[hurricane] casualty close-ups: {0} frame(s)"
                      .format(_made))
        except Exception as _exc:
            import traceback
            print("[hurricane] snapshots FAILED: {0}".format(_exc))
            traceback.print_exc()

    # HUR_EXPORT_STAGE=1: dump the ROOT LAYER (references intact, cheap --
    # not a flatten) next to the snapshots so the composed scene can be
    # audited offline with bare pxr. Added 2026-09-01 while chasing trees
    # that tally as placed but do not render.
    if os.environ.get("HUR_EXPORT_STAGE", "").strip() == "1" and SNAP_DIR:
        try:
            _sp = os.path.join(SNAP_DIR, "stage_root.usda")
            stage.GetRootLayer().Export(_sp)
            print("[hurricane] stage root layer -> {0}".format(_sp))
        except Exception as _exc:
            print("[hurricane] stage export FAILED: {0}".format(_exc))
    # 10) THE EXPORT --------------------------------------------------------
    #
    # RUNS LAST, after the ground truth and the review captures, for the same
    # reason `freeze_dataset_launch_script.py` orders it that way: those are
    # cheap and this is not, so an export that fails must not cost the labels,
    # and a cell whose GT is already on disk can be re-frozen from the same
    # seeds without rebuilding anything.
    #
    # `disaster.freeze.export_scene` reads the live stage out of
    # `omni.usd.get_context()` and has no dependency on `scene_api`'s `info`
    # shape, so it needs no hurricane-specific change — but this IS the first
    # non-wildfire stage it has ever run on, and it has no offline test
    # coverage at all. Read `freeze_report.json`'s `portable_ok` before
    # trusting a cell.
    if FREEZE_EXPORT and FREEZE_OUT:
        import json as _json
        from disaster import freeze as _freeze
        _name = FREEZE_NAME or _default_freeze_name(FREEZE_OUT)
        try:
            _finfo = _freeze.export_scene(
                FREEZE_OUT, _name, collect=FREEZE_COLLECT,
                waive_above_instances=(FREEZE_WAIVE_ABOVE_INSTANCES
                                       if FREEZE_WAIVE_VEGETATION else None),
                waive_mirrored=FREEZE_WAIVE_MIRRORED)
            _freeze.report(_finfo)
            with open(os.path.join(FREEZE_OUT, "freeze_report.json"), "w") as fh:
                _json.dump(_finfo, fh, indent=1)
        except _freeze.PortabilityError as _exc:
            # The gate fired — nothing ships, but `_exc.info` is the REAL,
            # complete `verify()` result `_enforce_portable` already computed.
            # Write it so the report says WHY instead of landing empty.
            print("[hurricane] EXPORT FAILED (portability gate): {0}"
                  .format(_exc))
            try:
                with open(os.path.join(FREEZE_OUT,
                                       "freeze_report.json"), "w") as fh:
                    _json.dump(_exc.info, fh, indent=1)
            except Exception as _exc2:                           # noqa: BLE001
                print("[hurricane] *** could not even write the failure "
                      "report: {0}".format(_exc2))
        except Exception as _exc:
            import traceback
            print("[hurricane] EXPORT FAILED: {0}".format(_exc))
            traceback.print_exc()

    print("[hurricane] SCENE_DONE")
    # EXIT ONCE THE CAPTURES ARE ON DISK. Spinning in `app.update()` after
    # SCENE_DONE kept every finished render resident on the card (11.6 GB at
    # L2, measured 2026-08-31) until someone killed it by hand, and the next
    # launch then shared the GPU with a zombie -- the two-Isaac black-frame
    # trap. Same contract as `downtown_quake_launch_script`: KEEP_OPEN=1 (or
    # HUR_KEEP_OPEN=1) holds the stage open for inspection; the default is
    # to fall through to `simulation_app.close()` in `__main__`.
    # `FREEZE_EXIT` WINS OVER `KEEP_OPEN`. A loop over cells is exactly the
    # case where a held-open app means the shell's `for` never reaches the
    # second iteration, and the two knobs are set by different people —
    # `KEEP_OPEN` by whoever wants to look at a scene, `FREEZE_EXIT` by the
    # batch that must not stall.
    if not FREEZE_EXIT and (
            os.environ.get("HUR_KEEP_OPEN", "").strip() == "1"
            or os.environ.get("KEEP_OPEN", "").strip() == "1"):
        while simulation_app.is_running():
            omni.kit.app.get_app().update()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
