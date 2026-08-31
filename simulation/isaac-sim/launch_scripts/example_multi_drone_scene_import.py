#!/usr/bin/env python3

import os

import carb
from isaacsim import SimulationApp

_LIVESTREAM = os.environ.get("ISAAC_SIM_LIVESTREAM", "").lower() == "true"

# Start Isaac Sim's simulation environment (Must start this before importing
# omni modules). When livestreaming, mirror the NVIDIA reference config —
# headless with the UI kept visible so the Kit GUI is rendered into the
# WebRTC stream (see example_one_px4_pegasus_launch_script.py for the full
# rationale; this script previously lacked the livestream branch entirely, so
# the isaac-sim-livestream profile ran it with nothing listening on the
# WebRTC ports).
if _LIVESTREAM:
    _SIM_APP_CONFIG = {
        "width": 1280,
        "height": 720,
        "window_width": 1920,
        "window_height": 1080,
        "headless": True,
        "hide_ui": False,
        "renderer": "RaytracedLighting",
        "display_options": 3286,
    }
else:
    _SIM_APP_CONFIG = {
        "headless": os.getenv("ISAAC_SIM_HEADLESS", "false").lower() == "true"}

simulation_app = SimulationApp(launch_config=_SIM_APP_CONFIG)

# Single-GPU rendering, stated explicitly rather than left to the default.
# NOTE: this is *not* what fixes the "no GPU / software fallback" failure —
# that was CUDA_VISIBLE_DEVICES being pinned in the Compose env (see the
# comment on it in simulation/isaac-sim/docker/docker-compose.yaml). Setting
# this here cannot fix it: the device skip happens inside
# gpu.foundation.plugin, before any renderer setting is applied. Kept only
# because being explicit about the intended single-GPU layout is correct.
simulation_app.set_setting("/renderer/multiGpu/enabled", False)

if _LIVESTREAM:
    # Enable WebRTC livestream and pin the UDP media port to the single port
    # published by the isaac-sim-livestream Compose profile and forwarded by
    # `airstack osmo:webrtc`. Kit 107 otherwise picks a dynamic media port
    # outside the forwarded set — signaling (TCP 49100) connects and the
    # client opens, but every media packet drops and the viewport stays
    # black. Full history in example_one_px4_pegasus_launch_script.py.
    from isaacsim.core.utils.extensions import enable_extension
    simulation_app.set_setting("/app/window/drawMouse", True)
    simulation_app.set_setting("/app/livestream/enabled", True)
    LIVESTREAM_UDP_PORT = int(
        os.environ.get("ISAAC_SIM_LIVESTREAM_UDP_PORT", "49099"))
    simulation_app.set_setting("/app/livestream/fixedHostPort", LIVESTREAM_UDP_PORT)
    simulation_app.set_setting("/app/livestream/minHostPort", LIVESTREAM_UDP_PORT)
    simulation_app.set_setting("/app/livestream/maxHostPort", LIVESTREAM_UDP_PORT)
    enable_extension("omni.kit.livestream.webrtc")

# Set local Nucleus as asset root before importing Pegasus (which resolves it at import time)
carb.settings.get_settings().set(
    "/persistent/isaac/asset_root/default",
    "omniverse://airlab-nucleus.andrew.cmu.edu/NVIDIA/Assets/Isaac/5.1"
)

import json
import os
import sys
import time

import omni.kit.app
import omni.timeline
import omni.usd
import omni.client

# pymavlink waits on its socket with select.select(), which cannot handle
# file descriptors >= 1024. DDS-over-TCP (fastdds LARGE_DATA) opens a TCP
# connection per remote participant, pushing this process's fd numbers past
# that, and every MAVLink read then throws "filedescriptor out of range in
# select()". Replace the wait with poll(), which has no fd-number limit.
# Must run before Pegasus creates its MAVLink connections.
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
from spawn_utils import generate_spawn_configs, polygon_centroid

sys.path.insert(0, os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "utils")))
from scene_prep import (
    scale_stage_prim, add_colliders, add_dome_light, get_stage_meters_per_unit,
    reference_root_prims_under_world, dedupe_physics_scenes,
    add_orthographic_camera, add_overhead_camera_publisher,
)
from scene_props import (
    add_vehicles, snap_prims_to_ground, yaw_deg_from_quat_xyzw,
    DEFAULT_VEHICLE_LIBRARY,
)
from gt_annotations import GTAnnotationRecorder, add_bbox_ros_publisher


# --------------------- CONFIGURATION ---------------------
NUCLEUS_SERVER = "airlab-nucleus.andrew.cmu.edu"

#env/stage path and scale
_LOCAL_SCENES_DIR = os.path.normpath(os.path.join(_LAUNCH_SCRIPTS_DIR, "..", "assets", "scenes"))
#ENV_URL = f"file://{_LOCAL_SCENES_DIR}/ModernCityDowntown.usd"
#ENV_URL = f"file://{_LOCAL_SCENES_DIR}/Shipyard.usd"
#ENV_URL = f"file://{_LOCAL_SCENES_DIR}/ModernCityDowntown.usd"
#ENV_URL = f"omniverse://{NUCLEUS_SERVER}/Projects/AirStack/scenes/urban/allegheny_county_fire_academy/fire_academy.scene.usd"
#ENV_URL = f"omniverse://{NUCLEUS_SERVER}/Library/Stages/RetroNeighborhood/RetroNeighborhood.stage.usd"
#ENV_URL = f"omniverse://{NUCLEUS_SERVER}/Library/Stages/AbandonedFactory/AbandonedFactory.stage.usd"
ENV_URL = f"omniverse://{NUCLEUS_SERVER}/Library/Stages/ConstructionSite/ConstructionSite.stage.usd"
#ENV_URL = f"omniverse://{NUCLEUS_SERVER}/Library/Stages/Dmytro/MilitaryBase_t_x1100_y200_z0_o_x0_y0_z90.scene.usd"
#ENV_URL = f"omniverse://{NUCLEUS_SERVER}/Library/Stages/Dmytro/copy-rayfronts-planner/AbandonedCity.scene.usd"
#ENV_URL = f"omniverse://{NUCLEUS_SERVER}/Library/Stages/Dmytro/downtown_edited_v3_818.usd"
#ENV_URL = f"omniverse://{NUCLEUS_SERVER}/Library/Stages/Dmytro/copy-rayfronts-planner/environments_start_pos/SnowyVillage_t_x-152_y-80_z-2_o_x0_y0_z_90.scene.usd"
#ENV_URL = f"omniverse://{NUCLEUS_SERVER}/Library/Stages/Dmytro/edit_v1_shipyard.usd"
#ENV_URL = f"omniverse://{NUCLEUS_SERVER}/Library/Stages/Dmytro/ModernCityDowntown.stage.usd"

_env_url_override = os.environ.get("ENV_URL")
if _env_url_override:
    ENV_URL = (_env_url_override if "://" in _env_url_override
               else f"omniverse://{NUCLEUS_SERVER}/{_env_url_override.lstrip('/')}")
# Per-environment override (mission runner sets STAGE_SCALE per iteration);
# falls back to 0.01 for standalone runs.
STAGE_SCALE = float(os.environ.get("STAGE_SCALE") or 0.01)

DRONE_USD = "~/.local/share/ov/data/documents/Kit/shared/exts/pegasus.simulator/pegasus/simulator/assets/Robots/Iris/iris.usd"

# Lighting
ADD_DOME_LIGHT = False
DOME_LIGHT_PATH = "/World/DomeLight"
DOME_LIGHT_INTENSITY = 3500.0
DOME_LIGHT_EXPOSURE = -5.0
DOME_LIGHT_TEXTURE = None   # optional HDRI sky URL; None → plain uniform dome

# DowntownWest / ModernCityDowntown ship a baked SkySphere that renders black
# headless; deactivate it (below, after the stage loads) and replace it with a
# dome light textured with an HDRI sky, which gives both a visible sky backdrop
# and its baked sun. Other scenes just get the dome-light sky.
_IS_DOWNTOWNWEST = "downtown_edited" in ENV_URL
_IS_MODERNCITY = "ModernCity" in ENV_URL
_IS_CONSTRUCTIONSITE = "ConstructionSite" in ENV_URL
_IS_SHIPYARD = "shipyard" in ENV_URL

_SKIES = f"omniverse://{NUCLEUS_SERVER}/NVIDIA/Assets/Skies"
# Per-scene HDRI sky (all under Skies/Clear/).
if _IS_DOWNTOWNWEST:
    DOME_LIGHT_TEXTURE = f"{_SKIES}/Clear/noon_grass_4k.hdr"
elif _IS_MODERNCITY or _IS_CONSTRUCTIONSITE:
    DOME_LIGHT_TEXTURE = f"{_SKIES}/Clear/evening_road_01_4k.hdr"
elif _IS_SHIPYARD:
    DOME_LIGHT_TEXTURE = f"{_SKIES}/Clear/syferfontein_18d_clear_4k.hdr"

if DOME_LIGHT_TEXTURE is not None:
    ADD_DOME_LIGHT = True
    # With a texture, intensity multiplies the HDRI's own radiance, so these maps
    # need far less than a plain white dome. Tune per scene if over-/under-exposed.
    DOME_LIGHT_INTENSITY = 1000.0
    DOME_LIGHT_EXPOSURE = 0.0


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

# Per-environment override (mission runner sets SPAWN_HEIGHT_M per iteration);
# falls back to 0.3 for standalone runs.
SPAWN_HEIGHT_ABOVE_FLOOR_M = float(os.environ.get("SPAWN_HEIGHT_M") or 0.5)
LIDAR_MIN_RANGE_M = 0.75

# _DEFAULT_DRONE_CONFIGS = [
#     {"domain_id": 1, "x_m": 32.0, "y_m": 12.6, "z_m": SPAWN_HEIGHT_ABOVE_FLOOR_M, "orient": [0.0, 0.0, -0.937, 0.35], "lidar_min_range": LIDAR_MIN_RANGE_M},
#     {"domain_id": 2, "x_m": 28.0, "y_m": 14.8, "z_m": SPAWN_HEIGHT_ABOVE_FLOOR_M, "orient": [0.0, 0.0, -0.937, 0.35], "lidar_min_range": LIDAR_MIN_RANGE_M},
#     {"domain_id": 3, "x_m": 32.0, "y_m": 19.8, "z_m": SPAWN_HEIGHT_ABOVE_FLOOR_M, "orient": [0.0, 0.0, -0.937, 0.35], "lidar_min_range": LIDAR_MIN_RANGE_M}
#     ]

_DEFAULT_DRONE_CONFIGS = [
    {"domain_id": 1, "x_m": 3.0, "y_m": 3.0, "z_m": SPAWN_HEIGHT_ABOVE_FLOOR_M, "orient": [0.0, 0.0, 0.0, 1.0], "lidar_min_range": LIDAR_MIN_RANGE_M},
    {"domain_id": 2, "x_m": 0.0, "y_m": 0.0, "z_m": SPAWN_HEIGHT_ABOVE_FLOOR_M, "orient": [0.0, 0.0, 0.0, 1.0], "lidar_min_range": LIDAR_MIN_RANGE_M},
    {"domain_id": 3, "x_m": -3.0, "y_m": -3.0, "z_m": SPAWN_HEIGHT_ABOVE_FLOOR_M, "orient": [0.0, 0.0, 0.0, 1.0], "lidar_min_range": LIDAR_MIN_RANGE_M},
    ]

# _DEFAULT_DRONE_CONFIGS = [
#     {"domain_id": 1, "x_m": -1.0, "y_m": -41.0, "z_m": SPAWN_HEIGHT_ABOVE_FLOOR_M, "orient": [0,0,0.698,0.716], "lidar_min_range": LIDAR_MIN_RANGE_M},
#     {"domain_id": 2, "x_m": 3.0, "y_m": -35.0, "z_m": SPAWN_HEIGHT_ABOVE_FLOOR_M, "orient": [0,0,0.736,0.677], "lidar_min_range": LIDAR_MIN_RANGE_M},
#     {"domain_id": 3, "x_m": 7.0, "y_m": -30.0, "z_m": SPAWN_HEIGHT_ABOVE_FLOOR_M, "orient": [0,0,0.784,0.621], "lidar_min_range": LIDAR_MIN_RANGE_M},
#     ]


# Spawn-area override: when SPAWN_POLY is set (the mission runner sets it per
# iteration), generate a randomized, collision-free layout inside it instead of
# using the fallback above. SPAWN_POLY is JSON — 2 opposite corners for an
# axis-aligned rectangle ("[[11,6.5],[-13,-6.5]]") or >=3 corners for a convex
# polygon. NUM_ROBOTS sets the count; SPAWN_SEED (optional) makes it
# reproducible; SPAWN_MIN_DIST_M / SPAWN_MARGIN_M tune spacing and edge inset.
_SPAWN_CONFIGS = os.environ.get("SPAWN_CONFIGS")
_SPAWN_POLY = os.environ.get("SPAWN_POLY")
if _SPAWN_CONFIGS:
    # Explicit hardcoded layout (mission pins fixed spawn positions + orient).
    # JSON list of dicts; only x_m, y_m (and optional orient [x,y,z,w]) required —
    # z_m / orient / lidar_min_range default. Takes priority over SPAWN_POLY.
    DRONE_CONFIGS = json.loads(_SPAWN_CONFIGS)
    for _i, _c in enumerate(DRONE_CONFIGS, start=1):
        _c.setdefault("domain_id", _i)
        _c.setdefault("z_m", SPAWN_HEIGHT_ABOVE_FLOOR_M)
        _c.setdefault("orient", [0.0, 0.0, 0.0, 1.0])
        _c.setdefault("lidar_min_range", LIDAR_MIN_RANGE_M)
    _xs = [_c["x_m"] for _c in DRONE_CONFIGS]
    _ys = [_c["y_m"] for _c in DRONE_CONFIGS]
    _spawn_center = ((sum(_xs) / len(_xs), sum(_ys) / len(_ys))
                     if DRONE_CONFIGS else (0.0, 0.0))
    _spawn_seed = "hardcoded"
elif _SPAWN_POLY:
    _poly = json.loads(_SPAWN_POLY)
    # Seed: honour SPAWN_SEED when set (reproducible layout), otherwise derive a
    # fresh seed from the wall clock + PID so every container bring-up gets a
    # different spawn layout. The effective seed is logged below so any run can
    # still be reproduced by setting SPAWN_SEED to it.
    _spawn_seed = os.environ.get("SPAWN_SEED") or f"{time.time_ns()}-{os.getpid()}"
    DRONE_CONFIGS = generate_spawn_configs(
        _poly,
        n=int(os.environ.get("NUM_ROBOTS", "3")),
        z_m=SPAWN_HEIGHT_ABOVE_FLOOR_M,
        lidar_min_range=LIDAR_MIN_RANGE_M,
        min_dist=float(os.environ.get("SPAWN_MIN_DIST_M") or 3.0),
        margin=float(os.environ.get("SPAWN_MARGIN_M") or 0.0),
        seed=_spawn_seed,
    )
    _spawn_center = polygon_centroid(_poly)
else:
    DRONE_CONFIGS = _DEFAULT_DRONE_CONFIGS
    _spawn_center = (0.0, 0.0)
    _spawn_seed = None

# Logged so each iteration's chosen environment + layout (+ the seed that
# produced it) is captured in the isaac-sim container logs, which mission_runner
# snapshots per iteration.
print(f"[spawn] ENV_URL={ENV_URL}", flush=True)
print(f"[spawn] SPAWN_SEED={_spawn_seed}", flush=True)
print(f"[spawn] DRONE_CONFIGS={json.dumps(DRONE_CONFIGS)}", flush=True)

# Top-down "map" camera. Captures one aerial of the static scene that the
# GCS visualizer turns into a textured ground in Foxglove's 3D panel. The
# camera centers on (OVERHEAD_CENTER_X_M, OVERHEAD_CENTER_Y_M) in world
# meters — leave both 0.0 for the legacy origin-centered behavior.
OVERHEAD_ALTITUDE_M    = 165.0
OVERHEAD_COVERAGE_M    = 300  # per-map knob: world meters per side.
# Default the overhead camera to the centre of the spawn area so the aerial
# texture frames the drones in whichever environment is active (the three test
# scenes sit in completely different coordinate regions). Override explicitly
# with OVERHEAD_CENTER_X_M / OVERHEAD_CENTER_Y_M.
OVERHEAD_CENTER_X_M    = float(os.environ.get("OVERHEAD_CENTER_X_M") or _spawn_center[0])
OVERHEAD_CENTER_Y_M    = float(os.environ.get("OVERHEAD_CENTER_Y_M") or _spawn_center[1])
OVERHEAD_PX_PER_METER  = 10.0     # Source-image density. Bump for sharper texture.
OVERHEAD_TOPIC         = "/sim/overhead/image"
OVERHEAD_SPEC_TOPIC    = "/sim/overhead/spec"
OVERHEAD_CENTER_X_TOPIC = "/sim/overhead/center_x"
OVERHEAD_CENTER_Y_TOPIC = "/sim/overhead/center_y"
OVERHEAD_FRAME_ID      = "map"
OVERHEAD_DOMAIN_ID     = 0

# ---------------- Static traffic props (cars / trucks) ----------------
# downtown_edited_v3_818.usd ships no vehicles (its `prop_car_pillar*` prims
# are parking-garage pillars), so cars and trucks are borrowed by reference
# from another Nucleus stage — ModernCityDowntown by default.
#
#   VEHICLE_PLACEMENTS  JSON list of exact placements, world metres:
#                       [{"kind":"car","x_m":12,"y_m":-4,"yaw_deg":90}, ...]
#   NUM_VEHICLES        count to scatter randomly (needs VEHICLE_POLY);
#                       NUM_VEHICLES=0 turns traffic off entirely
#   VEHICLE_POLY        JSON area to scatter in, same format as SPAWN_POLY
#   VEHICLE_KINDS       comma-separated keys from scene_props.VEHICLE_CATALOG,
#                       cycled over the scattered placements
#   VEHICLE_SEED        reproducible scatter
#   VEHICLE_LIBRARY_URL donor stage to reference vehicles out of
#   VEHICLE_GROUND_Z_M  fallback ground height when the raycast finds nothing
#   VEHICLE_SNAP_GROUND drop each vehicle onto the surface below it (default on)

# Hand-surveyed layout for downtown_edited_v3_818: 4 trucks + 7 cars picked off
# the roads in the Isaac Sim viewport. x_m/y_m/z_m are world metres (z_m is
# where the vehicle's *underside* rests, which for these assets is the origin
# the viewport reports); yaw_deg is the heading about +Z — the third component
# of the roll/pitch/yaw the viewport shows, since all of these are level.
#
# Ground snapping stays OFF for this layout (see VEHICLE_SNAP_GROUND below):
# these z values are already surveyed, and a raycast would only move them.
# The stage this layout was surveyed against; it is applied to no other.
_DEFAULT_VEHICLE_STAGE = "downtown_edited_v3_818"

_DEFAULT_VEHICLE_PLACEMENTS = [
    {"name": "truck_1", "kind": "truck", "x_m":    7.8,  "y_m": -89.54, "z_m": 0.122, "yaw_deg":  90.0},
    {"name": "truck_2", "kind": "truck", "x_m":  115.3,  "y_m":  56.9,  "z_m": 0.122, "yaw_deg": -90.0},
    {"name": "truck_3", "kind": "truck", "x_m": -133.4,  "y_m":  57.6,  "z_m": 0.122, "yaw_deg":   0.0},
    {"name": "truck_4", "kind": "truck", "x_m": -245.0,  "y_m":  39.0,  "z_m": 0.122, "yaw_deg": -90.0},
    {"name": "car_1",   "kind": "car",   "x_m":    7.0,  "y_m": -26.4,  "z_m": 0.001, "yaw_deg":  90.0},
    {"name": "car_2",   "kind": "car",   "x_m":    7.0,  "y_m":  47.7,  "z_m": 0.001, "yaw_deg":  90.0},
    {"name": "car_3",   "kind": "car",   "x_m":  114.25, "y_m":   5.8,  "z_m": 0.001, "yaw_deg": -90.0},
    {"name": "car_4",   "kind": "car",   "x_m": -124.8,  "y_m":  51.6,  "z_m": 0.001, "yaw_deg": -90.0},
    {"name": "car_5",   "kind": "car",   "x_m": -108.0,  "y_m":  -3.7,  "z_m": 0.001, "yaw_deg":  90.0},
    {"name": "car_6",   "kind": "car",   "x_m": -136.1,  "y_m": -72.6,  "z_m": 0.001, "yaw_deg":   0.0},
    {"name": "car_7",   "kind": "car",   "x_m": -136.0,  "y_m":  65.0,  "z_m": 0.001, "yaw_deg": 180.0},
]

_VEHICLE_PLACEMENTS = os.environ.get("VEHICLE_PLACEMENTS")
NUM_VEHICLES        = int(os.environ.get("NUM_VEHICLES") or 0)
_VEHICLE_POLY       = os.environ.get("VEHICLE_POLY")
VEHICLE_KINDS       = [k.strip() for k in
                       os.environ.get("VEHICLE_KINDS", "car,truck").split(",") if k.strip()]
VEHICLE_LIBRARY_URL = os.environ.get("VEHICLE_LIBRARY_URL") or DEFAULT_VEHICLE_LIBRARY
VEHICLE_GROUND_Z_M  = float(os.environ.get("VEHICLE_GROUND_Z_M") or 0.0)
VEHICLE_MIN_DIST_M  = float(os.environ.get("VEHICLE_MIN_DIST_M") or 8.0)

# Surveyed layouts carry their own z; only randomly scattered ones need the
# ground raycast to find the road surface for them.
_vehicle_layout_surveyed = True

if _VEHICLE_PLACEMENTS:
    VEHICLE_PLACEMENTS = json.loads(_VEHICLE_PLACEMENTS)
    for _i, _v in enumerate(VEHICLE_PLACEMENTS):
        _v.setdefault("kind", VEHICLE_KINDS[_i % len(VEHICLE_KINDS)] if VEHICLE_KINDS else "car")
        _v.setdefault("z_m", VEHICLE_GROUND_Z_M)
elif NUM_VEHICLES > 0:
    # Reuse the drone spawn sampler: same rejection sampling, wider spacing so
    # cars don't intersect each other.
    _poly = json.loads(_VEHICLE_POLY) if _VEHICLE_POLY else (
        json.loads(_SPAWN_POLY) if _SPAWN_POLY else [[30.0, 30.0], [-30.0, -30.0]])
    _v_cfgs = generate_spawn_configs(
        _poly, n=NUM_VEHICLES, z_m=VEHICLE_GROUND_Z_M, lidar_min_range=0.0,
        min_dist=VEHICLE_MIN_DIST_M, margin=float(os.environ.get("VEHICLE_MARGIN_M") or 0.0),
        seed=os.environ.get("VEHICLE_SEED") or f"veh-{time.time_ns()}-{os.getpid()}",
    )
    VEHICLE_PLACEMENTS = [
        {"kind": VEHICLE_KINDS[i % len(VEHICLE_KINDS)] if VEHICLE_KINDS else "car",
         "x_m": c["x_m"], "y_m": c["y_m"], "z_m": VEHICLE_GROUND_Z_M,
         "yaw_deg": round(yaw_deg_from_quat_xyzw(c["orient"]), 1)}
        for i, c in enumerate(_v_cfgs)
    ]
    _vehicle_layout_surveyed = False
elif "NUM_VEHICLES" in os.environ:
    VEHICLE_PLACEMENTS = []          # explicit NUM_VEHICLES=0 → no traffic
elif _DEFAULT_VEHICLE_STAGE in ENV_URL:
    VEHICLE_PLACEMENTS = [dict(v) for v in _DEFAULT_VEHICLE_PLACEMENTS]
else:
    # Every other scene gets no traffic. The surveyed layout is a hand-placed
    # fit to downtown_edited_v3_818's roads; applying it elsewhere drops 11
    # vehicles at DowntownWest coordinates into an unrelated stage, where they
    # land on buildings, in water, or float — and they are query targets, so
    # they corrupt that scene's ground truth too.
    VEHICLE_PLACEMENTS = []

VEHICLE_SNAP_GROUND = (os.environ["VEHICLE_SNAP_GROUND"].lower() == "true"
                       if "VEHICLE_SNAP_GROUND" in os.environ
                       else not _vehicle_layout_surveyed)

if VEHICLE_PLACEMENTS:
    print(f"[spawn] VEHICLE_LIBRARY_URL={VEHICLE_LIBRARY_URL}", flush=True)
    print(f"[spawn] VEHICLE_PLACEMENTS={json.dumps(VEHICLE_PLACEMENTS)}", flush=True)

# ---------------- Ground-truth annotations (bounding boxes) ----------------
# GT_ANNOTATIONS: off | files | ros | both
#   files — Replicator BasicWriter dataset (rgb + bbox_2d_tight + bbox_3d +
#           camera_params) under GT_OUTPUT_DIR, captured at GT_CAPTURE_HZ
#   ros   — live vision_msgs/Detection{2D,3D}Array on each robot's DDS domain
# Boxes only cover prims carrying semantic labels; add_vehicles labels every
# vehicle it spawns, so with GT_ANNOTATIONS=files and NUM_VEHICLES>0 you get a
# labelled car/truck dataset out of the box.
GT_ANNOTATIONS  = os.environ.get("GT_ANNOTATIONS", "off").lower()
GT_OUTPUT_DIR   = os.environ.get("GT_OUTPUT_DIR") or "/isaac-sim/AirStack/analysis_runs/gt_annotations"
GT_CAPTURE_HZ   = float(os.environ.get("GT_CAPTURE_HZ") or 2.0)
GT_WIDTH        = int(os.environ.get("GT_WIDTH") or 1280)
GT_HEIGHT       = int(os.environ.get("GT_HEIGHT") or 720)
GT_SEMANTIC_SEG = os.environ.get("GT_SEMANTIC_SEG", "false").lower() == "true"
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
        nucleus_stat(ENV_URL)

        # Timeline for controlling play/stop
        self.timeline = omni.timeline.get_timeline_interface()

        # Start Pegasus interface + world
        self.pg = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        self.timeline.stop()

        self.pg.load_environment(ENV_URL)

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("Stage failed to load")

        if not wait_for_stage(stage):
            carb.log_warn("Stage load timed out — continuing anyway.")

        dedupe_physics_scenes(stage)

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
            add_dome_light(stage, DOME_LIGHT_PATH, DOME_LIGHT_INTENSITY, DOME_LIGHT_EXPOSURE,
                           texture_file=DOME_LIGHT_TEXTURE)

        if _IS_DOWNTOWNWEST or _IS_MODERNCITY:
            sky = stage.GetPrimAtPath("/World/stage/SM_SkySphere")
            if sky.IsValid():
                sky.SetActive(False)
            else:
                carb.log_warn("/World/stage/SM_SkySphere not found — not deactivated.")

        # Units
        mpu, s = get_stage_meters_per_unit(stage)
        self.scene_scale_factor = s

        # Traffic props. Parented at /World/vehicles — a sibling of
        # /World/stage, so they don't inherit STAGE_SCALE and their placements
        # stay plain world metres.
        self.vehicle_paths = []
        if VEHICLE_PLACEMENTS:
            self.vehicle_paths = add_vehicles(
                stage, VEHICLE_PLACEMENTS,
                source_url=VEHICLE_LIBRARY_URL,
                scene_scale_factor=s,
            )

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
        gt_cameras = []
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
                camera_rotation_offset=[0.0, 0.0, 0.0],
            )

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

            # Left eye of the ZED — the view ground truth is generated for.
            # Path matches what add_zed_stereo_camera_subgraph builds:
            # <drone_prim>/<camera_name>/base_link/ZED_X/<left_frame_id>.
            gt_cameras.append(
                (f"robot_{i}",
                 f"/World/drone{i}/base_link/ZEDCamera/base_link/ZED_X/camera_left")
            )

            if GT_ANNOTATIONS in ("ros", "both"):
                add_bbox_ros_publisher(
                    graph_path=f"/World/GTBBoxGraph_{i}",
                    camera_prim_path=gt_cameras[-1][1],
                    robot_name=f"robot_{i}",
                    domain_id=i,
                    width=GT_WIDTH,
                    height=GT_HEIGHT,
                )

        # Annotation dataset writer. Created after the cameras exist; capture is
        # driven from run() so it can't outpace the sim loop.
        self.gt_recorder = None
        if GT_ANNOTATIONS in ("files", "both"):
            self.gt_recorder = GTAnnotationRecorder(
                gt_cameras,
                output_dir=GT_OUTPUT_DIR,
                resolution=(GT_WIDTH, GT_HEIGHT),
                capture_hz=GT_CAPTURE_HZ,
                semantic_segmentation=GT_SEMANTIC_SEG,
            )

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

        self.play_on_start = os.environ.get("PLAY_SIM_ON_START", "true").lower() == "true"
        self.stop_sim = False

    def run(self):
        if self.play_on_start:
            self.timeline.play()
        else:
            self.timeline.stop()

        app = omni.kit.app.get_app()
        steps = 0
        # PhysX only knows about the scene's colliders once the timeline has
        # been running, so the ground raycast has to wait a few frames.
        snap_at = 60 if (VEHICLE_SNAP_GROUND and self.vehicle_paths) else -1

        while simulation_app.is_running() and not self.stop_sim:
            world = World.instance()
            if world is not None and hasattr(world, '_scene'):
                world.step(render=True)
                if world is not self.world:
                    self.world = world
                    self.pg._world = world
            else:
                app.update()

            steps += 1
            if steps == snap_at:
                snap_prims_to_ground(
                    omni.usd.get_context().get_stage(),
                    self.vehicle_paths,
                    scene_scale_factor=self.scene_scale_factor,
                )

            if self.gt_recorder is not None:
                self.gt_recorder.maybe_capture()

        carb.log_warn("PegasusApp Simulation App is closing.")
        if self.gt_recorder is not None:
            self.gt_recorder.close()
        self.timeline.stop()
        simulation_app.close()


def main():
    pg_app = PegasusApp()
    pg_app.run()


if __name__ == "__main__":
    main()