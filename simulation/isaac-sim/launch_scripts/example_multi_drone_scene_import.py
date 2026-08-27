#!/usr/bin/env python3
"""
Spawn PX4 drones into a scene. The scene comes from ONE of two sources.

  ENV_URL (the default)  a finished USD pulled off Nucleus by
                         `pg.load_environment`. This is the historical path and
                         nothing about it changes.
  SCENE_CONFIG=<preset>  a GENERATED scene: `scene_gen/scene_api.build_scene`
                         authors the whole plat into the stage in-process —
                         layout, fire field, damage archetypes, ground scar,
                         survivors. Nothing is loaded and NOTHING IS RESCALED.

Setting SCENE_CONFIG is what selects the second path; every `_GENERATED` guard
below is that switch. See
`.agents/skills/launch-generated-scene-with-drones/SKILL.md` for the env knobs,
the traps, and why the generated path skips STAGE_SCALE.
"""

import os

import carb
from isaacsim import SimulationApp

SCENE_CONFIG = os.environ.get("SCENE_CONFIG", "").strip()
_GENERATED = bool(SCENE_CONFIG)

# fractionalCutoutOpacity: this renderer forces cutout opacity to 1.0 unless
# asked otherwise, which makes car glass opaque and hides every occupant the
# people pass put inside a vehicle. Both raster and path-traced paths need it,
# and the flag has to be BOTH a startup arg and a carb setting re-asserted
# after the stage is composed (see PegasusApp._build_generated_scene).
_CUTOUT_ARGS = ["--/rtx/raytracing/fractionalCutoutOpacity=true",
                "--/rtx/pathtracing/fractionalCutoutOpacity=true"]
_LIVESTREAM = _GENERATED and os.environ.get(
    "ISAAC_SIM_LIVESTREAM", "").lower() == "true"

# Start Isaac Sim's simulation environment (Must start this before importing omni modules)
if not _GENERATED:
    simulation_app = SimulationApp({"headless": False})
elif _LIVESTREAM:
    # Mirrors the NVIDIA reference config — headless with the UI kept VISIBLE,
    # so the Kit GUI is what renders into the WebRTC stream. See
    # example_one_px4_pegasus_launch_script.py for the full rationale.
    simulation_app = SimulationApp(launch_config={
        "width": 1280,
        "height": 720,
        "window_width": 1920,
        "window_height": 1080,
        "headless": True,
        "hide_ui": False,
        "renderer": "RaytracedLighting",
        "display_options": 3286,
        "extra_args": _CUTOUT_ARGS,
    })
else:
    simulation_app = SimulationApp(launch_config={
        "headless": os.getenv("ISAAC_SIM_HEADLESS", "false").lower() == "true",
        "extra_args": _CUTOUT_ARGS,
    })

if _GENERATED:
    from isaacsim.core.utils.extensions import enable_extension

    if _LIVESTREAM:
        # Pin the UDP media port to the single port the isaac-sim-livestream
        # profile publishes; Kit 107 otherwise picks a dynamic one outside the
        # forwarded set and the viewport stays black.
        simulation_app.set_setting("/app/window/drawMouse", True)
        simulation_app.set_setting("/app/livestream/enabled", True)
        _LS_PORT = int(os.environ.get("ISAAC_SIM_LIVESTREAM_UDP_PORT", "49099"))
        simulation_app.set_setting("/app/livestream/fixedHostPort", _LS_PORT)
        simulation_app.set_setting("/app/livestream/minHostPort", _LS_PORT)
        simulation_app.set_setting("/app/livestream/maxHostPort", _LS_PORT)
        enable_extension("omni.kit.livestream.webrtc")

    # Flow supplies the plumes over the road blockages. Enable before the scene
    # modules import, since `disaster.fire` touches Flow prim types.
    enable_extension("omni.flowusd")

# Set local Nucleus as asset root before importing Pegasus (which resolves it at import time)
# Unrelated to AIRSTACK_ASSET_ROOT: this one is NVIDIA's Isaac asset tree,
# which is where Pegasus' default environment comes from.
carb.settings.get_settings().set(
    "/persistent/isaac/asset_root/default",
    "omniverse://airlab-nucleus.andrew.cmu.edu/NVIDIA/Assets/Isaac/5.1"
)

if _GENERATED:
    # pymavlink waits on its socket with select.select(), which cannot handle
    # file descriptors >= 1024. DDS-over-TCP (fastdds LARGE_DATA) opens a TCP
    # connection per remote participant, pushing this process's fd numbers past
    # that, and every MAVLink read then throws "filedescriptor out of range in
    # select()". poll() has no fd-number limit. Must run before Pegasus creates
    # its MAVLink connections. Scoped to this path so the ENV_URL path stays
    # exactly what it was.
    import select as _select
    import time as _time
    from pymavlink import mavutil as _mavutil

    def _mavfile_select_poll(self, timeout):
        if self.fd is None:
            _time.sleep(min(timeout, 0.5))
            return True
        poller = _select.poll()
        poller.register(self.fd, _select.POLLIN)
        try:
            return len(poller.poll(timeout * 1000)) > 0
        except OSError:
            return False

    _mavutil.mavfile.select = _mavfile_select_poll

import json
import sys
import time

import omni.kit.app
import omni.timeline
import omni.usd
import omni.client

from omni.isaac.core.world import World

# Pegasus imports
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.ogn.api.spawn_multirotor import spawn_px4_multirotor_node
from pegasus.simulator.ogn.api.spawn_zed_camera import add_zed_stereo_camera_subgraph
from pegasus.simulator.ogn.api.spawn_rtx_lidar import add_rtx_lidar_subgraph

# gps_utils lives in the same directory as this script
_LAUNCH_SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
if _LAUNCH_SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _LAUNCH_SCRIPTS_DIR)
from gps_utils import set_gps_origins, DEFAULT_WORLD_ORIGIN

sys.path.insert(0, os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "utils")))
from scene_prep import (
    scale_stage_prim, add_colliders, add_dome_light, get_stage_meters_per_unit,
    reference_root_prims_under_world, dedupe_physics_scenes,
    add_orthographic_camera, add_overhead_camera_publisher,
)

if _GENERATED:
    from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
    from spawn_utils import generate_spawn_configs

    # scene_gen goes on sys.path only for the generated path, and only after
    # everything alongside this script has been imported: it drops a dozen bare
    # module names (`layout`, `detail`, `disaster`, `scene_generator`) at the
    # front of the search order, and there is no reason to hand the ENV_URL
    # path that exposure.
    sys.path.insert(0, os.path.normpath(os.path.join(
        _LAUNCH_SCRIPTS_DIR, "..", "..", "..", "scene_gen")))
    from scene_api import build_scene, LOCAL_ARCH_DIR
    from disaster import people as ppl


# --------------------- CONFIGURATION ---------------------
NUCLEUS_SERVER = "airlab-nucleus.andrew.cmu.edu"

#env/stage path and scale
_LOCAL_SCENES_DIR = os.path.normpath(os.path.join(_LAUNCH_SCRIPTS_DIR, "..", "assets", "scenes"))
#ENV_URL = f"file://{_LOCAL_SCENES_DIR}/ModernCityDowntown.usd"
#ENV_URL = f"file://{_LOCAL_SCENES_DIR}/Shipyard.usd"
#ENV_URL = f"file://{_LOCAL_SCENES_DIR}/ModernCityDowntown.usd"
#ENV_URL = f"omniverse://{NUCLEUS_SERVER}/Projects/AirStack/scenes/urban/allegheny_county_fire_academy/fire_academy.scene.usd"
#ENV_URL = f"omniverse://{NUCLEUS_SERVER}/Library/Stages/RetroNeighborhood/RetroNeighborhood.stage.usd" # scale = 0.01
#ENV_URL = f"omniverse://{NUCLEUS_SERVER}/Library/Stages/AbandonedFactory/AbandonedFactory.stage.usd" # scale=1.0
# ENV_URL = f"omniverse://{NUCLEUS_SERVER}/Library/Stages/ConstructionSite/ConstructionSite.stage.usd" # scale = 0.01
#ENV_URL = f"omniverse://{NUCLEUS_SERVER}/Library/Stages/Dmytro/MilitaryBase_t_x1100_y200_z0_o_x0_y0_z90.scene.usd"
#ENV_URL = f"omniverse://{NUCLEUS_SERVER}/Library/Stages/Dmytro/copy-rayfronts-planner/AbandonedCity.scene.usd"
#ENV_URL = f"omniverse://{NUCLEUS_SERVER}/Library/Stages/Dmytro/downtown_edited_v3_818.usd"
#ENV_URL = f"omniverse://{NUCLEUS_SERVER}/Library/Stages/Dmytro/copy-rayfronts-planner/environments_start_pos/SnowyVillage_t_x-152_y-80_z-2_o_x0_y0_z_90.scene.usd"
#ENV_URL = f"omniverse://{NUCLEUS_SERVER}/Library/Stages/Dmytro/edit_v1_shipyard.usd"
#ENV_URL = f"omniverse://{NUCLEUS_SERVER}/Library/Stages/Dmytro/ModernCityDowntown.stage.usd"

ENV_URL = f"omniverse://{NUCLEUS_SERVER}/Library/Stages/Muyang/LevelTest2.usd" # scale = 0.01

STAGE_SCALE = 0.01

# --------------------- GENERATED SCENE (SCENE_CONFIG) ---------------------
# Only read when SCENE_CONFIG is set. `AIRSTACK_ASSET_ROOT` repoints
# `airstack://` onto Nucleus for pods whose clone carries no local AEC tree;
# `scene_generator` reads it at IMPORT time, so it must already be in the
# environment when this process starts — nothing here writes it.
if _GENERATED:
    # Pegasus' flat default env, NOT a finished scene: it exists only to give
    # PegasusInterface a World with a PhysicsScene to hang the plat off.
    BASE_ENV_URL = SIMULATION_ENVIRONMENTS["Default Environment"]
    SCENE_PARENT = "/World/stage/generated"
    SEED = int(os.environ.get("MINI_SEED", "11"))
    # None -> scene_api.default_arch_dir(): the local bake if it exists, else
    # the copy under AIRSTACK_ASSET_ROOT on Nucleus. A URL is fine — the bake
    # directory is untracked, so a pod's fresh clone has none, and
    # `os.listdir` cannot enumerate `omniverse://`.
    ARCH_DIR = os.environ.get("ARCH_DIR") or None
    # Survivor ground truth is written with plain `open()`, so it lands on the
    # FILESYSTEM even when the bake it is named after lives on Nucleus.
    PEOPLE_JSON = os.environ.get("PEOPLE_JSON") or os.path.join(
        LOCAL_ARCH_DIR, "humans_{0}.json".format(SEED))
    POLES = os.environ.get("PEOPLE_POLES", "").strip().lower() in (
        "1", "true", "yes")
    BURN_FRAC = float(os.environ.get("MINI_BURN_FRAC", "0.45"))
    ELAPSED = float(os.environ.get("MINI_ELAPSED", "0"))

    # SPEC OVERRIDES. A preset names a KIND of scene (suburb, downtown); these
    # pick the instance of it, so "a 250 m undamaged suburb" needs no preset
    # file of its own:
    #
    #   SCENE_CONFIG=suburb REGION_M=250x250 DISASTER_TYPE=none
    #
    # Merged into the high-level spec BEFORE it compiles, so DISASTER_TYPE=none
    # compiles to a config with no disaster rather than one the launcher has to
    # remember to skip. Unset leaves the preset's own value alone.
    def _region_m(raw):
        """REGION_M as "250", "250x250" or "250,250" -> [w, h] metres.

        A bare number is square: "250" and "250x250" mean the same thing to
        anyone asking for a 250 m block.
        """
        if not raw:
            return None
        parts = [p for p in raw.replace("x", ",").replace("X", ",").split(",")
                 if p.strip()]
        try:
            vals = [float(p) for p in parts]
        except ValueError:
            raise SystemExit("REGION_M={0!r}: expected N, NxN or N,N".format(raw))
        # Delimiter-only input ("x", ",", ",,") filters down to NOTHING. Without
        # this it returned [], which is not None, so it was applied as an
        # override and wrote an EMPTY region — compiling fine for
        # `disaster-type: none` and only exploding much later inside the
        # generator, or as an unpack error out of compile_disaster.
        if not vals:
            raise SystemExit("REGION_M={0!r}: no number in it; expected N, NxN "
                             "or N,N".format(raw))
        vals = [vals[0], vals[0]] if len(vals) == 1 else vals[:2]
        if any(v <= 0.0 for v in vals):
            raise SystemExit("REGION_M={0!r}: sides must be positive, got "
                             "{1}".format(raw, vals))
        return vals

    _dtype = os.environ.get("DISASTER_TYPE", "").strip() or None
    _sev = os.environ.get("SEVERITY", "").strip()
    _severity = float(_sev) if _sev else None

    # SEVERITY GATES THE DISASTER, so asking for one without it is a silent
    # no-op. `compile_spec` picks the compiler with
    #     fn = DISASTERS[dtype] if severity > 0.0 else compile_none
    # and the generic presets (`suburb`, `downtown`, `suburb_net`) ship
    # `severity: 0.0`. So `DISASTER_TYPE=wildfire` alone compiles through
    # compile_none and yields a PRISTINE plat that looks like the request
    # worked. Give an asked-for disaster a real default instead; 0.6 is what
    # suburb_wildfire.yaml uses.
    if _dtype and _dtype.lower() != "none" and _severity is None:
        _severity = 0.6
        print("[spawn] DISASTER_TYPE={0} with no SEVERITY -> defaulting to 0.6 "
              "(severity 0 compiles to no disaster at all)".format(_dtype),
              flush=True)

    SPEC_OVERRIDES = {
        "region_m": _region_m(os.environ.get("REGION_M", "").strip()),
        "disaster-type": _dtype,
        "severity": _severity,
    }

    # Which way a house faces. `h["yaw_deg"]` is the frontage TANGENT (along
    # the street) and the house must face ACROSS it at the kerb, so the plat
    # adds this offset. -90 is the code default and what every purpose-built
    # suburb preset sets; expose it so a scene whose houses look turned can be
    # corrected in one restart instead of a source edit.
    _hyaw = os.environ.get("HOUSE_YAW_OFFSET_DEG", "").strip()
    if _hyaw:
        SPEC_OVERRIDES["overrides"] = {
            "suburb_parcel": {"house_yaw_offset_deg": float(_hyaw)}}
        print("[spawn] HOUSE_YAW_OFFSET_DEG={0} (default -90)".format(_hyaw),
              flush=True)
    # MINI_SEED already drives layout/damage/vegetation through build_scene's
    # `seed`; carry it into the spec too so the two cannot disagree.
    if os.environ.get("MINI_SEED", "").strip():
        SPEC_OVERRIDES["seed"] = SEED
    if any(v is not None for v in SPEC_OVERRIDES.values()):
        print("[spawn] spec overrides: {0}".format(
            {k: v for k, v in SPEC_OVERRIDES.items() if v is not None}),
            flush=True)
    # off | ground | all. The plat is ~10^5 prims and `add_colliders` is a
    # Python recursion that prints per mesh, so a full pass is minutes of log
    # spam — and mostly futile: houses and trees are referenced INSTANCEABLE
    # (what keeps 9k trees inside VRAM) and `GetChildren()` does not descend
    # into an instance, so they get no collider either way. `ground` gives the
    # drone the one surface it needs to land on and take off from.
    COLLIDERS = os.environ.get("SUBURB_COLLIDERS", "ground").strip().lower()

DRONE_USD = "~/.local/share/ov/data/documents/Kit/shared/exts/pegasus.simulator/pegasus/simulator/assets/Robots/Iris/iris.usd"

# DOWNWARD tilt of every drone's ZED, in degrees about the body Y axis —
# positive is down. Default 0 is the LEVEL mount every method in this stack
# assumes today; a mission that wants a tilt sets ZED_PITCH_DEG in its `env:`.
#
# Set for CoNavGPT runs. CoNavGPT builds its occupancy map by unprojecting FPV
# depth and rasterising it top-down, so a level camera at 15-40 m AGL spends the
# top half of every frame on sky and never sees the ground under the drone,
# leaving a permanent hole in the map around the robot.
#
# THE PLANNER MUST BE TOLD, because TF walks the URDF and the URDF models NO
# mount pitch: a camera tilted here is a camera TF cannot see, and every point
# would unproject to the wrong bearing without a word of warning.
# search_planner reads this SAME variable as the default for its
# camera_pitch_rad so the two
# cannot drift apart; any other consumer of this tilt must do likewise.
ZED_PITCH_DEG = float(os.environ.get("ZED_PITCH_DEG", "").strip() or 0.0)

# ZED RESOLUTION. Pegasus defaults to 480x300, and TARGET PIXEL HEIGHT is what
# gates detection: measured on a wildfire frame, YOLO needs a person ~20 px
# tall to clear a 0.5 threshold and is blind by ~9 px. At 12 m cruise with the
# camera pitched down, 480x300 puts a person under that floor, so the drone
# flies over people and detects nothing. Raising this is the one fix that does
# NOT cost flight time and does not depend on descending first.
# 720x450 is 1.5x linear, i.e. 1.5x the pixels on target at every altitude.
ZED_WIDTH = int(os.environ.get("ZED_WIDTH", "").strip() or 720)
ZED_HEIGHT = int(os.environ.get("ZED_HEIGHT", "").strip() or 450)

# RTX lidar per drone. Defaults to ON, which is what this launcher has always
# done — an UNSET value means "unchanged", not "off", because the compose file
# forwards ENABLE_LIDAR empty rather than false (each launcher owns its own
# default; example_multi_px4_pegasus_launch_script.py's is off).
#
# Worth turning OFF for a CoNavGPT mission. That method consumes no lidar and no
# point clouds — it builds its occupancy map purely from RGBD depth — and the
# lidar is expensive: measured 2026-08-25 on the house bench, cameras fall from
# ~53 Hz to ~17 Hz and /clock from ~53 to ~15 Hz with it attached. Since an OSMO
# run is bounded in SIM seconds, that is a ~3x multiplier on the WALL CLOCK, and
# therefore on the GPU hours, for identical coverage.
#
# It is NOT off because of the fault §4.1 once blamed on it. That was retested
# with the lidar attached and the scene produced no cudaErrorIllegalAddress, no
# render-graph failure and no dropped frames; the real cause was a stale DDS
# profile in the isaac-sim image. Anything that needs vdb_mapping needs this on.
ENABLE_LIDAR = (os.environ.get("ENABLE_LIDAR", "").strip().lower()
                or "true") in ("1", "true", "yes", "on")

# Lighting
ADD_DOME_LIGHT = False
DOME_LIGHT_PATH = "/World/DomeLight"
DOME_LIGHT_INTENSITY = 3500.0
DOME_LIGHT_EXPOSURE = -5.0

# GPS world anchor: what world (0, 0, 0) maps to in real GPS coordinates.
# Matches the Lisbon default in px4_config.yaml — change here to relocate the sim world.
WORLD_GPS_ORIGIN = DEFAULT_WORLD_ORIGIN

# Drone spawn configs.
# x_m = East offset from world origin (meters)
# y_m = North offset from world origin (meters)
# z_m = Up offset / height above floor (meters)
# orient = initial quaternion [x, y, z, w]
# spawn location for /Assets/Fire_Academy_Digital_Twin/fire_academy.usd:
# {"domain_id": 1, "x_m": 20.0, "y_m": -7.0, ...}
# {"domain_id": 2, "x_m": 17.0, "y_m":  1.5, ...}

SPAWN_HEIGHT_ABOVE_FLOOR_M = 0.3#0.03
# DRONE_CONFIGS = [
#     {"domain_id": 1, "x_m": 32.0, "y_m": 12.6, "z_m": SPAWN_HEIGHT_ABOVE_FLOOR_M, "orient": [0.0, 0.0, -0.937, 0.35], "lidar_min_range": 4.0},
#     {"domain_id": 2, "x_m": 28.0, "y_m": 14.8, "z_m": SPAWN_HEIGHT_ABOVE_FLOOR_M, "orient": [0.0, 0.0, -0.937, 0.35], "lidar_min_range": 4.0},
#     {"domain_id": 3, "x_m": 32.0, "y_m": 19.8, "z_m": SPAWN_HEIGHT_ABOVE_FLOOR_M, "orient": [0.0, 0.0, -0.937, 0.35], "lidar_min_range": 4.0}
#     ]


DRONE_CONFIGS = [
    {"domain_id": 1, "x_m": 3.0, "y_m": 3.0, "z_m": SPAWN_HEIGHT_ABOVE_FLOOR_M, "orient": [0.0, 0.0, 0.0, 1.0], "lidar_min_range": 0.75},
    {"domain_id": 2, "x_m": 0.0, "y_m": 0.0, "z_m": SPAWN_HEIGHT_ABOVE_FLOOR_M, "orient": [0.0, 0.0, 0.0, 1.0], "lidar_min_range": 0.75},
    {"domain_id": 3, "x_m": -3.0, "y_m": -3.0, "z_m": SPAWN_HEIGHT_ABOVE_FLOOR_M, "orient": [0.0, 0.0, 0.0, 1.0], "lidar_min_range": 0.75},
    ]

# SPAWN_CONFIGS / SPAWN_POLY, GENERATED PATH ONLY. The three spots above are
# metres from the origin and mean nothing on a 1600 x 1200 m plat, so the
# generated path takes its fleet from the mission instead: SPAWN_CONFIGS is a
# JSON list of dicts needing only x_m/y_m, SPAWN_POLY randomizes NUM_ROBOTS
# spawns inside a rect or convex polygon. Deliberately NOT applied to the
# ENV_URL path, whose fleet is the list above and stays that way.
if _GENERATED:
    NUM_ROBOTS = int(os.environ.get("NUM_ROBOTS") or 1)
    SPAWN_HEIGHT_ABOVE_FLOOR_M = float(os.environ.get("SPAWN_HEIGHT_M") or 0.5)
    LIDAR_MIN_RANGE_M = 0.75

    # DEFAULT SPAWN, DERIVED — not eyeballed. One drone on a 10.7 m local road
    # at the SE flank of the suburb_wildfire plat:
    #   * 166 s of arrival-time margin outside the severity-0.6 fire front at
    #     MINI_BURN_FRAC=0.45, so it starts in clear air rather than smoke;
    #   * 381 m from the burn centre at (19, 110) — close enough that the scar
    #     is the first thing in frame, far enough not to spawn inside it;
    #   * yaw 138.1 deg = atan2(110 - -144.3, 19 - 302.4), i.e. facing the burn
    #     centre, which as a quaternion [x, y, z, w] about +Z is
    #     [0, 0, sin(69.05 deg), cos(69.05 deg)] = [0, 0, 0.9342, 0.3568].
    # Changing MINI_SEED / MINI_BURN_FRAC / the preset's epicenter invalidates
    # the margin figure; the point stays on road either way.
    _DEFAULT_DRONE_CONFIGS = [
        {"domain_id": 1, "x_m": 302.4, "y_m": -144.3,
         "z_m": SPAWN_HEIGHT_ABOVE_FLOOR_M,
         "orient": [0.0, 0.0, 0.9342, 0.3568],
         "lidar_min_range": LIDAR_MIN_RANGE_M},
    ]

    _SPAWN_CONFIGS = os.environ.get("SPAWN_CONFIGS")
    _SPAWN_POLY = os.environ.get("SPAWN_POLY")
    if _SPAWN_CONFIGS:
        DRONE_CONFIGS = json.loads(_SPAWN_CONFIGS)
        for _i, _c in enumerate(DRONE_CONFIGS, start=1):
            _c.setdefault("domain_id", _i)
            _c.setdefault("z_m", SPAWN_HEIGHT_ABOVE_FLOOR_M)
            _c.setdefault("orient", [0.0, 0.0, 0.0, 1.0])
            _c.setdefault("lidar_min_range", LIDAR_MIN_RANGE_M)
        _spawn_seed = "hardcoded"
    elif _SPAWN_POLY:
        _spawn_seed = os.environ.get("SPAWN_SEED") or "{0}-{1}".format(
            time.time_ns(), os.getpid())
        DRONE_CONFIGS = generate_spawn_configs(
            json.loads(_SPAWN_POLY),
            n=NUM_ROBOTS,
            z_m=SPAWN_HEIGHT_ABOVE_FLOOR_M,
            lidar_min_range=LIDAR_MIN_RANGE_M,
            min_dist=float(os.environ.get("SPAWN_MIN_DIST_M") or 3.0),
            margin=float(os.environ.get("SPAWN_MARGIN_M") or 0.0),
            seed=_spawn_seed,
        )
    else:
        DRONE_CONFIGS = [dict(c) for c in _DEFAULT_DRONE_CONFIGS[:NUM_ROBOTS]]
        _spawn_seed = None

    if len(DRONE_CONFIGS) != NUM_ROBOTS:
        # Not fatal — the robot containers are what NUM_ROBOTS actually sizes,
        # and a mismatch shows up as a robot with no airframe rather than a
        # crash. Say so loudly, because that failure otherwise reads as a PX4
        # timeout.
        print("[spawn] WARNING: NUM_ROBOTS={0} but {1} spawn config(s) — robot "
              "containers without an airframe will never report ready"
              .format(NUM_ROBOTS, len(DRONE_CONFIGS)), flush=True)

    print("[spawn] SCENE_CONFIG={0} (BUILT in-process, no ENV_URL)"
          .format(SCENE_CONFIG), flush=True)
    print("[spawn] AIRSTACK_ASSET_ROOT={0}"
          .format(os.environ.get("AIRSTACK_ASSET_ROOT") or "<repo>"), flush=True)
    print("[spawn] SPAWN_SEED={0}".format(_spawn_seed), flush=True)
    print("[spawn] DRONE_CONFIGS={0}".format(json.dumps(DRONE_CONFIGS)),
          flush=True)

# Top-down "map" camera. Captures one aerial of the static scene that the
# GCS visualizer turns into a textured ground in Foxglove's 3D panel. The
# camera centers on (OVERHEAD_CENTER_X_M, OVERHEAD_CENTER_Y_M) in world
# meters — leave both 0.0 for the legacy origin-centered behavior.
OVERHEAD_ALTITUDE_M    = 165.0
OVERHEAD_COVERAGE_M    = 225  # per-map knob: world meters per side.
OVERHEAD_CENTER_X_M    = 0.0 #-152     # world-X of camera center / texture center.
OVERHEAD_CENTER_Y_M    = 0.0 #-80     # world-Y of camera center / texture center.
OVERHEAD_PX_PER_METER  = 10.0     # Source-image density. Bump for sharper texture.
OVERHEAD_TOPIC         = "/sim/overhead/image"
OVERHEAD_SPEC_TOPIC    = "/sim/overhead/spec"
OVERHEAD_CENTER_X_TOPIC = "/sim/overhead/center_x"
OVERHEAD_CENTER_Y_TOPIC = "/sim/overhead/center_y"
OVERHEAD_FRAME_ID      = "map"
OVERHEAD_DOMAIN_ID     = 0

# THE PLAT, NOT THE SPAWN CROP. A generated region is 1600 x 1200 m centred on
# the origin and the 225 m default above is a crop of it that is useless as a
# mission map. 1600 m x 1.28 px/m = 2048 px, which is the helper's
# max_resolution — asking for more is silently clamped.
if _GENERATED:
    OVERHEAD_ALTITUDE_M   = float(os.environ.get("OVERHEAD_ALTITUDE_M") or 400.0)
    OVERHEAD_COVERAGE_M   = float(os.environ.get("OVERHEAD_COVERAGE_M") or 1600.0)
    OVERHEAD_CENTER_X_M   = float(os.environ.get("OVERHEAD_CENTER_X_M") or 0.0)
    OVERHEAD_CENTER_Y_M   = float(os.environ.get("OVERHEAD_CENTER_Y_M") or 0.0)
    OVERHEAD_PX_PER_METER = float(os.environ.get("OVERHEAD_PX_PER_METER") or 1.28)
# ---------------------------------------------------------


ext_manager = omni.kit.app.get_app().get_extension_manager()
for ext in [
    "omni.graph.core",                  # Core runtime for OmniGraph engine
    "omni.graph.action",                # Action Graph framework
    "omni.graph.action_nodes",          # Built-in Action Graph node library
    "omni.graph.ui",                    # UI scaffolding for graph tools
    "omni.graph.visualization.nodes",   # Visualization helper nodes
    "omni.graph.scriptnode",            # Python script node support
    "omni.graph.window.action",         # Action Graph editor window
    "omni.graph.window.generic",        # Generic graph UI tools
    "omni.graph.ui_nodes",              # UI node building helpers
    "pegasus.simulator",
]:
    if not ext_manager.is_extension_enabled(ext):
        # Try immediate enable if available (more robust across Kit versions), fall back otherwise
        try:
            ext_manager.set_extension_enabled_immediate(ext, True)
        except Exception:
            ext_manager.set_extension_enabled(ext, True)


def nucleus_stat(url: str) -> bool:
    result, info = omni.client.stat(url)
    return result == omni.client.Result.OK


def wait_for_stage(stage, timeout_s: float = 10.0):
    """Pump the Kit app loop until /World has content (scene fully loaded)."""
    for _ in range(int(timeout_s / 0.1)):
        omni.kit.app.get_app().update()
        world_prim = stage.GetPrimAtPath("/World")
        if world_prim.IsValid():
            non_physics = [c for c in world_prim.GetChildren() if c.GetName() != "PhysicsScene"]
            if non_physics:
                return True
        time.sleep(0.1)
    return False

class PegasusApp:

    def __init__(self):
        # Write GPS origins immediately so robot containers can read them
        # before this container finishes its heavy USD loading.
        set_gps_origins(DRONE_CONFIGS, world_origin=WORLD_GPS_ORIGIN)

        omni.client.initialize()
        nucleus_stat(f"omniverse://{NUCLEUS_SERVER}")
        if not _GENERATED:
            nucleus_stat(ENV_URL)

        # Timeline for controlling play/stop
        self.timeline = omni.timeline.get_timeline_interface()

        # Start Pegasus interface + world
        self.pg = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        self.timeline.stop()

        self.pg.load_environment(BASE_ENV_URL if _GENERATED else ENV_URL)

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("Stage failed to load")

        if not wait_for_stage(stage):
            carb.log_warn("Stage load timed out — continuing anyway.")

        if _GENERATED:
            # The Pegasus default env was loaded only to give the World a valid
            # base; its ground plane and lighting go straight back off, because
            # the generated plat brings its own ground and the config's sky.
            for _name in ("GroundPlane", "Environment"):
                _p = stage.GetPrimAtPath("/World/" + _name)
                if _p and _p.IsValid():
                    _p.SetActive(False)

        dedupe_physics_scenes(stage)

        if _GENERATED:
            # Units FIRST: the plat is authored in metres x ssf, and that is
            # also why nothing rescales it afterwards.
            mpu, s = get_stage_meters_per_unit(stage)
            self._build_generated_scene(stage, s)
        else:
            self._prepare_env_url_scene(stage)
            # Units
            mpu, s = get_stage_meters_per_unit(stage)

        # Top-down orthographic camera over (0, 0). Publishes one JPEG aerial
        # of the static scene at low rate; the GCS visualizer republishes it
        # as a textured ground for Foxglove's 3D panel.
        cam_path = add_orthographic_camera(
            stage,
            prim_path="/World/MapCamera",
            altitude_m=OVERHEAD_ALTITUDE_M,
            coverage_m=OVERHEAD_COVERAGE_M,
            scene_scale_factor=s,
            center_x_m=OVERHEAD_CENTER_X_M,
            center_y_m=OVERHEAD_CENTER_Y_M,
        )
        add_overhead_camera_publisher(
            parent_graph_path="/World/MapCameraGraph",
            camera_prim_path=cam_path,
            topic=OVERHEAD_TOPIC,
            spec_topic=OVERHEAD_SPEC_TOPIC,
            center_x_topic=OVERHEAD_CENTER_X_TOPIC,
            center_y_topic=OVERHEAD_CENTER_Y_TOPIC,
            frame_id=OVERHEAD_FRAME_ID,
            coverage_m=OVERHEAD_COVERAGE_M,
            center_x_m=OVERHEAD_CENTER_X_M,
            center_y_m=OVERHEAD_CENTER_Y_M,
            pixels_per_meter=OVERHEAD_PX_PER_METER,
            domain_id=OVERHEAD_DOMAIN_ID,
        )

        # Spawn all drones
        for cfg in DRONE_CONFIGS:
            i = cfg["domain_id"]
            pos = [cfg["x_m"] * s, cfg["y_m"] * s, cfg["z_m"] * s]

            graph_handle = spawn_px4_multirotor_node(
                pegasus_node_name=f"PX4Multirotor_{i}",
                drone_prim=f"/World/drone{i}/base_link",
                robot_name=f"robot_{i}",
                vehicle_id=i,
                domain_id=i,
                usd_file=DRONE_USD,
                init_pos=pos,
                init_orient=cfg["orient"],
            )

            add_zed_stereo_camera_subgraph(
                parent_graph_handle=graph_handle,
                drone_prim=f"/World/drone{i}/base_link",
                robot_name=f"robot_{i}",
                camera_name="ZEDCamera",
                camera_offset=[0.21, 0.0, 0.05],
                camera_rotation_offset=[0.0, ZED_PITCH_DEG, 0.0],
                frame_width=ZED_WIDTH,
                frame_height=ZED_HEIGHT,
            )

            if ENABLE_LIDAR:
                add_rtx_lidar_subgraph(
                    parent_graph_handle=graph_handle,
                    drone_prim=f"/World/drone{i}/base_link",
                    robot_name=f"robot_{i}",
                    lidar_config="ouster_os1",
                    lidar_topic_name="point_cloud_raw",
                    lidar_offset=[0.0, 0.0, 0.025],
                    lidar_rotation_offset=[0.0, 0.0, 0.0],
                    min_range=cfg["lidar_min_range"],
                )

        # POST-SCALE ONLY, so the generated path never does it: there is no
        # scale_stage_prim on a generated plat, and toggling ~10^5 prims off
        # and on for 8 s buys nothing.
        if not _GENERATED:
            self._clear_post_scale_artifact(stage)

        self.play_on_start = os.environ.get("PLAY_SIM_ON_START", "true").lower() == "true"
        self.stop_sim = False
        if _GENERATED:
            self._print_drone_banner()

    def _prepare_env_url_scene(self, stage):
        # ----- Scene preparation -----
        # Bring in sky/sun/environment prims that sit at root level in the
        # source USD next to the defaultPrim that pg.load_environment already
        # loaded into /World/stage. reference_root_prims_under_world skips
        # the defaultPrim, so this can't duplicate geometry.
        reference_root_prims_under_world(stage, ENV_URL)

        stage_prim = stage.GetPrimAtPath("/World/stage")
        if stage_prim.IsValid():
            scale_stage_prim(stage, "/World/stage", STAGE_SCALE)
            add_colliders(stage_prim)
            for _ in range(10):
                omni.kit.app.get_app().update()
        else:
            carb.log_warn("/World/stage not found — skipping scale and collision.")

        if ADD_DOME_LIGHT:
            add_dome_light(stage, DOME_LIGHT_PATH, DOME_LIGHT_INTENSITY, DOME_LIGHT_EXPOSURE)

    def _clear_post_scale_artifact(self, stage):
        # Toggle /World/stage off and on to clear a post-scale visual artifact
        # that lingers until the prim is deactivated/reactivated.
        stage_prim = stage.GetPrimAtPath("/World/stage")
        if stage_prim.IsValid():
            stage_prim.SetActive(False)
            app = omni.kit.app.get_app()
            wait_end = time.time() + 8.0
            while time.time() < wait_end:
                app.update()
            stage_prim.SetActive(True)

    # ----------------------- GENERATED SCENE -----------------------

    def _build_generated_scene(self, stage, s):
        """Author the plat, then everything that needs the geometry to exist."""
        info = {}
        st = build_scene(stage, SCENE_CONFIG, s, arch_dir=ARCH_DIR, seed=SEED,
                         burn_frac=BURN_FRAC, elapsed=ELAPSED, poles=POLES,
                         parent_path=SCENE_PARENT, people_json=PEOPLE_JSON,
                         info_out=info, spec_overrides=SPEC_OVERRIDES)
        # The extra_args form only covers startup; re-assert once the stage is
        # composed so car glass stays see-through for the whole run.
        for _key in ("/rtx/raytracing/fractionalCutoutOpacity",
                     "/rtx/pathtracing/fractionalCutoutOpacity"):
            carb.settings.get_settings().set_bool(_key, True)
        app = omni.kit.app.get_app()
        for _ in range(30):
            app.update()
        self._print_scene_banner(st)
        self._add_scene_colliders(stage)
        self._write_scene_annotations(stage, st)

    def _add_scene_colliders(self, stage):
        if COLLIDERS == "off":
            print("[scene] colliders: OFF (SUBURB_COLLIDERS=off)", flush=True)
            return
        target = (SCENE_PARENT + "/ground") if COLLIDERS == "ground" else "/World/stage"
        prim = stage.GetPrimAtPath(target)
        if not (prim and prim.IsValid()):
            carb.log_warn("{0} not found - no colliders added.".format(target))
            return
        t0 = time.time()
        add_colliders(prim)
        app = omni.kit.app.get_app()
        for _ in range(10):
            app.update()
        print("[scene] colliders on {0} in {1:.0f}s"
              .format(target, time.time() - t0), flush=True)

    def _write_scene_annotations(self, stage, st):
        """Ground truth for a GENERATED scene, from the generator's own record.

        The survivors are the point of these scenes and they are the one class
        nothing else can supply: their positions exist only in the people pass's
        `humans.json`, which is written from the same records the scenario was
        authored from. Reading GT from there means the boxes and the scenario
        cannot disagree.

        Off unless GT_ANNOTATIONS is set — it overwrites the annotation file for
        RESULTS_SCENE, and a hand-authored file for a Nucleus stage of the same
        name is not something to clobber by default.
        """
        if os.environ.get("GT_ANNOTATIONS", "off").strip().lower() not in (
                "1", "true", "yes", "on"):
            return
        scene = os.environ.get("RESULTS_SCENE", "").strip()
        if not scene:
            print("[annotations] GT_ANNOTATIONS is on but RESULTS_SCENE is "
                  "unset — nothing to name the file after, skipping")
            return
        try:
            import scene_annotations as sa
        except ImportError as exc:
            print(f"[annotations] unavailable: {exc}")
            return
        # launch_scripts/ -> isaac-sim/ -> simulation/ -> repo root
        repo = os.path.normpath(
            os.path.join(_LAUNCH_SCRIPTS_DIR, "..", "..", ".."))
        boxes = []
        pj = (st or {}).get("people_json") if isinstance(st, dict) else None
        if pj:
            boxes += sa.boxes_from_people(sa.people_records(pj))
        # Everything else the generator placed that maps to a class, measured
        # off the composed stage. `placements` is only present when the API
        # hands it back; people alone are still worth writing without it.
        placements = (st or {}).get("placements") if isinstance(st, dict) else None
        if placements:
            boxes += sa.boxes_from_placements(stage, placements)
        # THE OBSTACLES go to a SEPARATE file. The GT the GCS draws and the
        # scorer reads is PEOPLE ONLY — that is what the benchmark searches
        # for, and a box per house and tree is noise there. But a by-reference
        # scene has no placements for its houses and trees (each is one
        # referenced prim under <parent>/inst), and the 3D lawnmower needs
        # exactly those to fly over (search_baselines/clearance.py). So they
        # are measured off the stage the same way and written as
        # `<scene>_obstacles.json` beside the GT, where only the planner's
        # known-obstacle loader looks. Cars come from the survivor plan's own
        # scope; the kerb/driveway fleet is inside the referenced plat.
        parent = ((st or {}).get("parent") if isinstance(st, dict) else None) \
            or "/World/stage/generated"
        obstacles = sa.boxes_from_scopes(stage, [
            (parent + "/inst", {"h_": "house", "t_": "tree", "blocker_": "tree"}),
            (parent + "/people_cars", {"": "car"}),
        ])
        if not boxes and not obstacles:
            print("[annotations] nothing to write (no people_json, no placements)")
            return
        if boxes:
            sa.write_annotations(scene, boxes, sa.annotation_dirs(repo))
        if obstacles:
            sa.write_annotations(scene + "_obstacles", obstacles,
                                 sa.annotation_dirs(repo))

    def _print_scene_banner(self, st):
        r = st["region"]
        print("\n" + "=" * 72, flush=True)
        print("SCENE BUILT - {0} ({1:.0f} x {2:.0f} m, by reference)".format(
            st["scene_config"], r[2] - r[0], r[3] - r[1]))
        print("  assembly    {0:.0f} s".format(st["seconds"]))
        print("  archetypes  {0}".format(st["arch_dir"]))
        print("  fire        t+{0:.0f} s elapsed over a {1:.0f} s span "
              "(burn_frac {2:.2f}, seed {3})".format(
                  st["elapsed_s"], st["span_s"], st["burn_frac"], st["seed"]))
        print("  houses      {0} referenced ({1} missing); {2}".format(
            st["houses"], st["houses_missing"],
            ", ".join("%s=%d" % kv for kv in sorted(st["house_tally"].items()))))
        print("  trees       {0} referenced ({1} missing); {2}".format(
            st["trees"], st["trees_missing"],
            ", ".join("%s=%d" % kv for kv in sorted(st["tree_tally"].items()))))
        print("  ground scar {0} band(s); {1} Flow emitter(s)".format(
            st["bands"], st["flow"]))
        print("  people      {0} authored ({1} alive), {2} car(s), "
              "{3} blocker(s) -> {4}".format(
                  st["people"], st["people_alive"], st["cars"], st["blockers"],
                  st["people_json"]))
        for _name in ppl.SCENARIOS:
            print("    {0:<18} {1:>3}".format(
                _name, st["people_tally"].get(_name, 0)))
        if st["poles"]:
            print("  poles       {0} locator marker(s) (PEOPLE_POLES)".format(
                st["poles"]))
        print("  NEXT        colliders, map camera, then {0} PX4 drone(s)"
              .format(len(DRONE_CONFIGS)))
        print("=" * 72 + "\n", flush=True)

    def _print_drone_banner(self):
        print("\n" + "=" * 72, flush=True)
        print("DRONES UP - {0} PX4 multirotor(s) on {1}".format(
            len(DRONE_CONFIGS), SCENE_CONFIG))
        for cfg in DRONE_CONFIGS:
            print("  robot_{0}  domain {0}  ({1:.1f}, {2:.1f}, {3:.1f}) m  "
                  "orient {4}".format(cfg["domain_id"], cfg["x_m"], cfg["y_m"],
                                      cfg["z_m"], cfg["orient"]))
        print("  overhead    {0} @ {1:.0f} m coverage, centred "
              "({2:.0f}, {3:.0f})".format(OVERHEAD_TOPIC, OVERHEAD_COVERAGE_M,
                                          OVERHEAD_CENTER_X_M,
                                          OVERHEAD_CENTER_Y_M))
        print("  timeline    {0}".format(
            "PLAY" if self.play_on_start else "STOPPED (PLAY_SIM_ON_START)"))
        print("  PX4 is now booting; mission readiness gates on that, not on "
              "this banner.")
        print("=" * 72 + "\n", flush=True)

    def run(self):
        if self.play_on_start:
            self.timeline.play()
        else:
            self.timeline.stop()

        app = omni.kit.app.get_app()
        while simulation_app.is_running() and not self.stop_sim:
            world = World.instance()
            if world is not None and hasattr(world, '_scene'):
                world.step(render=True)
                if world is not self.world:
                    self.world = world
                    self.pg._world = world
            else:
                app.update()

        carb.log_warn("PegasusApp Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()


def main():
    pg_app = PegasusApp()
    pg_app.run()


if __name__ == "__main__":
    main()