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

  3. THE PROGRESSION IS ROOF-DOWN AND IT STOPS THERE. Most of this plate is
     in the four roof states — `shingles_lost` through `roof_stripped` — and
     structural collapse is a minority. A hurricane scene where half the
     houses are levelled is a tornado scene with the track taken out.

  4. THE DEBRIS IS VEGETATION, NOT PLANKS. ~70% leaves, twigs, fronds and
     limbs. `disaster.planks`' sawn-timber field is the tornado's signature
     and is deliberately not called at low levels.

  5. THERE IS WATER, AND IT IS THE OTHER HALF OF THE SCENE. `disaster.surge`
     imposes a synthetic ground (the suburb plate is dead flat — see that
     module) and cuts a static water surface against it. No fluid sim: an
     opaque muddy OmniPBR_ClearCoat on flat geometry, a shoreline that comes
     from an alpha map, ponding in the low spots, and mud/seed lines as proud
     ribbons rather than as per-building texture rebinds.

  6. `swept` IS NOT A WIND LEVEL. A slab swept clean is a SURGE signature and
     `disaster.hurricane_flow` refuses to produce one. Where this scene shows
     bare slabs, `surge.house_water_state(...)["swept"]` put them there.

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

    ARCH_DIR      archetype library (default `scene_gen/assets/archetypes_hurricane`)
    SCENE_CONFIG  preset (default `suburb_hurricane_500_l2`)
    HUR_SEED      seed for the damage draws (default 11)
    HUR_WATER     0 disables every water layer (wind-only scene, for A/B)
    HUR_GROUND    0 disables the wet/mud ground overlay
    HUR_DEBRIS    vegetation debris pieces per damaged house (default 60)
    SURGE_*       water tuning — see `disaster.surge.knobs_from_env`
    SNAP_DIR      write viewport PNGs here (MUST sit under the mounted log dir)
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


PARENT = "/World/stage/generated"
SCENE_CONFIG = _env("SCENE_CONFIG", "suburb_hurricane_500_l2")
SEED = int(_env("HUR_SEED", "11"))
ARCH_DIR = _env("ARCH_DIR",
                os.path.join(_SCENE_GEN_DIR, "assets", "archetypes_hurricane"))
ENV_URL = SIMULATION_ENVIRONMENTS["Default Environment"]
DO_WATER = _flag("HUR_WATER")
DO_GROUND = _flag("HUR_GROUND")
N_DEBRIS = int(_env("HUR_DEBRIS", "60"))
# THE WASH-AWAY PASS HAS ITS OWN SWITCH, separate from `HUR_WATER`. It moves
# and re-seats real prims where the rest of the water half only adds surfaces,
# so it is the part most likely to take the process down with it — and when a
# 4-minute scene build dies there, the cheapest next question is always "is it
# this pass?". `HUR_WASHAWAY=0` answers it in one run.
DO_WASHAWAY = _flag("HUR_WASHAWAY")
SNAP_DIR = _env("SNAP_DIR", "")
if SNAP_DIR:
    os.makedirs(SNAP_DIR, exist_ok=True)

# THE FOUR ROOF STATES KEEP THEIR STREET YAW. A house that has lost its
# covering is still a house facing the street, and yawing it to the wind — the
# tornado's `_TRACK_YAWED` trick — would make a neighbourhood of intact
# footprints all point the same way, which is the one thing a hurricane does
# NOT do to a street grid. Only the two collapse levels, whose footprint is a
# heap rather than a building, take the wind bearing.
_WIND_YAWED = ("partial_collapse", "leveled")

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

    # 4) HOUSES -------------------------------------------------------------
    fp_by_style = {e["style"]: max(e["w"], e["d"])
                   for e in ss.modular_catalogue(config)}
    UsdGeom.Scope.Define(stage, Sdf.Path(PARENT + "/inst"))
    n_h = miss_h = n_swept = 0
    htally = {}
    era_tally = {}
    wrecks = []          # (x, y, footprint, intensity, level, palette)
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
            # SURGE, NOT WIND. `hurricane_flow` refuses to emit `swept`; the
            # water is the only thing on this plate allowed to clear a slab.
            level = "swept"
            _sl = "swept"
            n_swept += 1
        else:
            level = hu.house_level_for_intensity(it, drng, vuln=vuln)
        htally[level] = htally.get(level, 0) + 1
        yaw = (float(hu.wind_bearing_at(hcfg, h["x"], h["y"]))
               + drng.uniform(-14.0, 14.0)
               if level in _WIND_YAWED else h["yaw"])
        key = "house_{0}_{1}".format(h["style"], level)
        usd = arch.get(key) or arch.get("house_{0}_pristine".format(h["style"]))
        if not usd:
            # BUILD IT LIVE INSTEAD OF LEAVING A HOLE — the tornado launcher's
            # argument holds unchanged: an absent building in a disaster scene
            # reads as a deliberate empty lot, so a missing archetype must not
            # `continue`.
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
            except Exception as _exc:
                if miss_h == 1:
                    print("[hurricane] live house fallback FAILED: {0}"
                          .format(_exc))
            continue
        _pal = h.get("palette")
        _recolour = bool(h.get("row")) and bool(_pal)
        # A WRAPPER XFORM PER HOUSE, and the reference one level down.
        #
        # `_ref` authors `xformOp:translate` and `xformOp:rotateZ` on whatever
        # prim it is given. `washaway.apply_washaway` then wants to author its
        # OWN translate/rotate to push the house off its slab — and USD
        # refuses to add an xform op that already exists, with an exception
        # whose `str()` is EMPTY. That produced `washaway FAILED on
        # .../h_2: ` with nothing after the colon, and every one of the 43
        # flooded houses silently kept its original pose.
        #
        # So: `h_{i}` is a bare Xform that nothing else touches and the surge
        # is free to move, and `h_{i}/model` carries the reference and the
        # placement ops. Instancing still applies to the child, which is where
        # the geometry actually is.
        _hp = "{0}/inst/h_{1}".format(PARENT, i)
        UsdGeom.Xform.Define(stage, Sdf.Path(_hp))
        if _ref(stage, _hp + "/model", usd, h["x"], h["y"], yaw, ssf,
                instance=not _recolour):
            n_h += 1
            if level != "pristine":
                wrecks.append((h["x"], h["y"],
                               fp_by_style.get(h["style"], 12.0), it, level,
                               _pal))
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
                    _spec = (wash.shift_spec(float(wst["depth"]),
                                             float(scfg["shore_bearing_deg"]),
                                             drng)
                             if _sl == "shifted" else
                             wash.collapse_spec(float(wst["depth"]), drng)
                             if _sl == "collapsed" else {})
                    _spec.setdefault("mudline_z", wst.get("mudline_z"))
                    wash.apply_washaway(stage, _hp, _sl, _spec, ssf=ssf)
                    wtally[_sl] = wtally.get(_sl, 0) + 1
                except Exception as _wexc:
                    if not wtally.get("_err"):
                        print("[hurricane] washaway FAILED on {0}: {1}"
                              .format(_hp, _wexc))
                    wtally["_err"] = wtally.get("_err", 0) + 1
            # FOR THE GROUND TRUTH. The level lives in the ARCHETYPE FILENAME
            # and a USD reference does not publish it, so a stage walk can
            # never recover it. This is the only place that knows the prim
            # path, the style, the level, the era and the water depth together.
            _h_recs.append({"prim_path": _hp, "style": h["style"],
                            "level": level, "x": float(h["x"]),
                            "y": float(h["y"]), "yaw_deg": float(yaw),
                            "row": bool(h.get("row")),
                            "intensity": float(it),
                            "gust_mps": float(gust(h["x"], h["y"])),
                            "code_era": era, "vulnerability": float(vuln),
                            "water_depth_m": float(wst.get("depth", 0.0)),
                            "swept_by_surge": bool(wst.get("swept"))})
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
        level = hu.tree_level_for_intensity(it, trng, species=sp or None)
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
        key = "tree_{0}_{1}".format(sp, level)
        usd = arch.get(key) if level != "pristine" else None
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
            yaw = (float(hu.wind_bearing_at(hcfg, t["x"], t["y"]))
                   + trng.uniform(-38.0, 38.0))
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
                _rs = wash.raft_specs(_wcfg, region, wrng,
                                      [(r["x"], r["y"]) for r in _h_recs],
                                      sgw.depth_at(scfg, region,
                                                   np.random.default_rng(SEED + 41)),
                                      kind_weights=_kw)
                made_w["rafts"] = wash.build_rafts(
                    stage, PARENT + "/rafts", _rs, ssf=ssf)
            except Exception as _rexc:
                print("[hurricane] debris rafts FAILED: {0}".format(_rexc))
            print("[hurricane] water in {0:.0f}s: {1}".format(
                time.time() - tw,
                ", ".join("{0} {1}".format(k, v) for k, v in made_w.items())))
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
            made_g = ground.build_overlay(
                stage, cov, region, ssf, mud_z, material_parent=PARENT,
                root="/World/siltGround", cell_m=kn["cell_m"],
                bands=kn["bands"], tile_m=kn["tile_m"],
                op_range=kn["op_range"], texture=sgw.SILT_TEXTURE,
                # You cannot silt water. A pool is a hole in the lawn with
                # water below grade; an overlay cell drawn over it is a film
                # floating on the surface. Same exclusion the burn scar makes.
                skip=ground.skip_rects(binfo.get("pool_rects") or (), pad=0.0))
            print("[hurricane] silt overlay: {0} cell(s)".format(len(made_g)))
        except Exception as exc:
            print("[hurricane] silt overlay FAILED: {0}".format(exc))

    # 8) GROUND TRUTH -------------------------------------------------------
    gt_path = os.path.join(SNAP_DIR or ARCH_DIR, "GT_hurricane.json")
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
            _snaps.overview(stage, (cx, cy), max(rw, rh),
                            os.path.join(SNAP_DIR, "overview.png"), ssf)
            # THE SUBJECTS ARE CHOSEN ALONG THE WATER GRADIENT, not along a
            # track — there is no track. What has to be judged here is (a) the
            # shoreline, (b) a flooded street, (c) dry damaged houses well
            # inland, so the ladder and the water can each be seen doing their
            # own job rather than being confused for one another.
            _pts = {}
            # Subjects along the WATER gradient — there is no track to
            # walk. `surge.review_points` picks the shoreline, the deepest
            # standing water and a dry inland spot, so the damage ladder and
            # the flood can each be seen doing their own job rather than
            # being confused for one another.
            try:
                _pts.update(sgw.review_points(scfg, region))
            except Exception as _e:
                print("[hurricane] review points unavailable: {0}".format(_e))
            if _h_recs:
                _worst = max(_h_recs, key=lambda r: hu.HOUSE_LEVELS.index(
                    r["level"]) if r["level"] in hu.HOUSE_LEVELS else 99)
                _pts["worst_house"] = (_worst["x"], _worst["y"])
                _wet = [r for r in _h_recs if r["water_depth_m"] > 0.15]
                if _wet:
                    _w0 = max(_wet, key=lambda r: r["water_depth_m"])
                    _pts["deepest_flooded_house"] = (_w0["x"], _w0["y"])
            if not _pts:
                _pts["centre"] = (cx, cy)
            _snaps.views_around(stage, _pts, SNAP_DIR, ssf)
            print("[hurricane] snapshots -> {0} ({1} subject(s))"
                  .format(SNAP_DIR, len(_pts)))
        except Exception as _exc:
            import traceback
            print("[hurricane] snapshots FAILED: {0}".format(_exc))
            traceback.print_exc()

    print("[hurricane] SCENE_DONE")
    while simulation_app.is_running():
        omni.kit.app.get_app().update()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
