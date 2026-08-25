#!/usr/bin/env python3
"""
BUILD the procedural wildfire suburb, then spawn PX4 drones into it.

    SCENE_CONFIG=suburb_wildfire NUM_ROBOTS=1 \
    ISAAC_SIM_SCRIPT_NAME=suburb_wildfire_drones_launch_script.py \
    airstack up isaac-sim

`suburb_assemble_launch_script.py` (the scene) welded to
`example_multi_drone_scene_import.py` (the drones). The one structural
difference from the latter: there is NO `pg.load_environment(<Nucleus stage>)`.
The plat does not exist anywhere to be loaded — it is authored into the stage
here, by reference to pre-baked damage archetypes, and the drones go in on top
of the result. This is what lets an OSMO mission fly a GENERATED scene
(`osmo/missions/conavgpt_wildfire_1robot.yaml`).

ORDERING IS LOAD-BEARING, in two independent ways.

  ACROSS the two halves: colliders, the overhead camera and the drone spawn all
  need the geometry to exist, so the whole assembly runs first and everything
  from the drone script runs after it. `set_gps_origins` is the exception and
  goes FIRST — robot containers block on those files, and making them wait out
  a ~450 s scene build for a number that is already known is pure dead time.

  WITHIN the assembly: the evacuation queue's CARS go in before the burnable /
  scorch pass so they char like every other car on the plat, and the PEOPLE go
  in after it, because a survivor is not scorched. See `_build_scene`.

`AIRSTACK_ASSET_ROOT` repoints `airstack://` onto Nucleus for pods whose clone
carries no local AEC tree. `scene_generator` reads it at IMPORT time, so it
must be set in the environment before this process starts — nothing here
writes it, and the Isaac `asset_root/default` carb setting below is a
different, unrelated root (Pegasus' NVIDIA asset tree). `ARCH_DIR` is the
same story for the damage bake and takes a URL too (see `_load_archetypes`).

`objaverse://` has NO Nucleus copy and `scene_gen/assets/objaverse/` is
untracked, so ~41 small props render as placeholder prisms on a pod.
`scene_generator` already reports that and carries on; it is not fatal and is
not fixed here.
"""

import os

import carb
from isaacsim import SimulationApp

_LIVESTREAM = os.environ.get("ISAAC_SIM_LIVESTREAM", "").lower() == "true"

# fractionalCutoutOpacity: this renderer forces cutout opacity to 1.0 unless
# asked otherwise, which makes car glass opaque and hides every occupant the
# people pass put inside a vehicle. Both raster and path-traced paths need it.
_CUTOUT_ARGS = ["--/rtx/raytracing/fractionalCutoutOpacity=true",
                "--/rtx/pathtracing/fractionalCutoutOpacity=true"]

# Livestream mirrors the NVIDIA reference config — headless with the UI kept
# visible so the Kit GUI renders into the WebRTC stream. See
# example_one_px4_pegasus_launch_script.py for the full rationale.
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
        "extra_args": _CUTOUT_ARGS,
    }
else:
    _SIM_APP_CONFIG = {
        "headless": os.getenv("ISAAC_SIM_HEADLESS", "false").lower() == "true",
        "extra_args": _CUTOUT_ARGS,
    }

simulation_app = SimulationApp(launch_config=_SIM_APP_CONFIG)

from isaacsim.core.utils.extensions import enable_extension  # noqa: E402

if _LIVESTREAM:
    # Pin the UDP media port to the single port the isaac-sim-livestream
    # profile publishes; Kit 107 otherwise picks a dynamic one outside the
    # forwarded set and the viewport stays black.
    simulation_app.set_setting("/app/window/drawMouse", True)
    simulation_app.set_setting("/app/livestream/enabled", True)
    LIVESTREAM_UDP_PORT = int(
        os.environ.get("ISAAC_SIM_LIVESTREAM_UDP_PORT", "49099"))
    simulation_app.set_setting("/app/livestream/fixedHostPort", LIVESTREAM_UDP_PORT)
    simulation_app.set_setting("/app/livestream/minHostPort", LIVESTREAM_UDP_PORT)
    simulation_app.set_setting("/app/livestream/maxHostPort", LIVESTREAM_UDP_PORT)
    enable_extension("omni.kit.livestream.webrtc")

# Flow supplies the plumes over the road blockages. Enable before the scene
# modules import, since `disaster.fire` touches Flow prim types.
enable_extension("omni.flowusd")

# Local Nucleus as Isaac's asset root, before importing Pegasus (which resolves
# it at import time). Unrelated to AIRSTACK_ASSET_ROOT — this one is NVIDIA's
# Isaac asset tree, which is where Pegasus' default environment comes from.
carb.settings.get_settings().set(
    "/persistent/isaac/asset_root/default",
    "omniverse://airlab-nucleus.andrew.cmu.edu/NVIDIA/Assets/Isaac/5.1"
)

import json      # noqa: E402
import math      # noqa: E402
import random    # noqa: E402
import sys       # noqa: E402
import time      # noqa: E402

import omni.client      # noqa: E402
import omni.kit.app     # noqa: E402
import omni.timeline    # noqa: E402
import omni.usd         # noqa: E402

# pymavlink waits on its socket with select.select(), which cannot handle file
# descriptors >= 1024. DDS-over-TCP (fastdds LARGE_DATA) opens a TCP connection
# per remote participant, pushing this process's fd numbers past that, and
# every MAVLink read then throws "filedescriptor out of range in select()".
# Replace the wait with poll(), which has no fd-number limit. Must run before
# Pegasus creates its MAVLink connections.
import select as _select                       # noqa: E402
import time as _time                           # noqa: E402
from pymavlink import mavutil as _mavutil      # noqa: E402


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

from omni.isaac.core.world import World                                       # noqa: E402
from pegasus.simulator.params import SIMULATION_ENVIRONMENTS                  # noqa: E402
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface  # noqa: E402
from pegasus.simulator.ogn.api.spawn_multirotor import spawn_px4_multirotor_node  # noqa: E402
from pegasus.simulator.ogn.api.spawn_zed_camera import add_zed_stereo_camera_subgraph  # noqa: E402
from pegasus.simulator.ogn.api.spawn_rtx_lidar import add_rtx_lidar_subgraph  # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade                               # noqa: E402

_LAUNCH_SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
_ISAAC_SIM_DIR = os.path.normpath(os.path.join(_LAUNCH_SCRIPTS_DIR, ".."))
_SCENE_GEN_DIR = os.path.normpath(
    os.path.join(_ISAAC_SIM_DIR, "..", "..", "scene_gen"))
for _p in (_LAUNCH_SCRIPTS_DIR, os.path.join(_ISAAC_SIM_DIR, "utils"),
           _SCENE_GEN_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np                                                    # noqa: E402

from gps_utils import set_gps_origins, DEFAULT_WORLD_ORIGIN           # noqa: E402
from spawn_utils import generate_spawn_configs                       # noqa: E402
from scene_prep import (                                              # noqa: E402
    add_colliders, add_sky, dedupe_physics_scenes, get_stage_meters_per_unit,
    add_orthographic_camera, add_overhead_camera_publisher,
)
from gt_annotations import GTAnnotationRecorder, add_bbox_ros_publisher  # noqa: E402

import scene_generator as sg                          # noqa: E402
from scene_generator import resolve_sky               # noqa: E402
import suburb_scene as ss                             # noqa: E402
from suburb_scene import generate_suburb_on_stage     # noqa: E402
from compile_disaster import load_scene_config        # noqa: E402
from disaster import damage, fire, ground             # noqa: E402
from disaster import people as ppl                    # noqa: E402
from disaster import vegetation as veg                # noqa: E402
from detail import modular_house as mh                # noqa: E402


# --------------------- SCENE CONFIGURATION ---------------------
PARENT = "/World/stage/generated"
SCENE_CONFIG = os.environ.get("SCENE_CONFIG", "suburb_wildfire")
SEED = int(os.environ.get("MINI_SEED", "11"))
# THE BAKE, LOCAL OR ON NUCLEUS. `scene_gen/assets/archetypes/` is untracked,
# so a pod's fresh clone has no such directory and the default below does not
# exist there. A URL is accepted and enumerated over omni.client instead — see
# `_load_archetypes`. The mission sets:
#   ARCH_DIR=omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/scene_gen/assets/archetypes
# and if it does not, AIRSTACK_ASSET_ROOT already names that same root.
_LOCAL_ARCH_DIR = os.path.join(_SCENE_GEN_DIR, "assets", "archetypes")
# The bake sits under AIRSTACK_ASSET_ROOT on Nucleus at the same relative path
# it has in the repo, so a pod that already points `airstack://` at Nucleus
# needs no second variable: fall back there, and only when the local bake is
# genuinely absent, so a workstation run is unchanged. Explicit ARCH_DIR wins.
_NUCLEUS_ROOT = os.environ.get("AIRSTACK_ASSET_ROOT", "").strip().rstrip("/")
ARCH_DIR = os.environ.get("ARCH_DIR") or (
    _LOCAL_ARCH_DIR if os.path.isdir(_LOCAL_ARCH_DIR)
    else (_NUCLEUS_ROOT + "/scene_gen/assets/archetypes"
          if "://" in _NUCLEUS_ROOT else _LOCAL_ARCH_DIR))
# Survivor ground truth is written with plain `open()`, so it must land on the
# FILESYSTEM even when the bake it is named after lives on Nucleus.
PEOPLE_JSON = os.environ.get("PEOPLE_JSON") or os.path.join(
    _LOCAL_ARCH_DIR, "humans_{0}.json".format(SEED))
# PEOPLE_POLES=1 authors the magenta survivor markers + cyan row-home markers.
POLES = os.environ.get("PEOPLE_POLES", "").strip().lower() in ("1", "true", "yes")
BURN_FRAC = float(os.environ.get("MINI_BURN_FRAC", "0.45"))
ELAPSED_OVERRIDE = float(os.environ.get("MINI_ELAPSED", "0"))

# Pegasus' flat default env, NOT a finished scene: it is what gives
# PegasusInterface a World with a PhysicsScene to hang the plat off. Its ground
# plane and lighting are deactivated immediately after — the generated ground
# and the config's own sky replace them.
BASE_ENV_URL = SIMULATION_ENVIRONMENTS["Default Environment"]

# off | ground | all. The plat is ~10^5 prims and `add_colliders` is a Python
# recursion that prints per mesh, so a full pass is minutes of log spam. It is
# also mostly futile: houses and trees are referenced INSTANCEABLE (that is
# what keeps 9k trees inside VRAM) and `GetChildren()` does not descend into an
# instance, so they get no collider either way. `ground` gives the drone the
# one surface it actually needs to land on and take off from.
COLLIDERS = os.environ.get("SUBURB_COLLIDERS", "ground").strip().lower()

TREE_SPECIES = {
    "Black_Oak": "airstack://scene_gen/assets/aec/tower/Assets/Vegetation/Black_Oak/Black_Oak.usd",
    "Shumard_Oak": "airstack://scene_gen/assets/aec/tower/Assets/Vegetation/Shumard_Oak/Shumard_Oak.usd",
    "Douglas_Fir": "airstack://scene_gen/assets/aec/brownstone/Assets/Vegetation/Trees/Douglas_Fir.usd",
    "Largetooth_Aspen": "airstack://scene_gen/assets/aec/brownstone/Assets/Vegetation/Trees/Largetooth_Aspen.usd",
    "Common_Apple": "airstack://scene_gen/assets/aec/tower/Assets/Vegetation/Common_Apple/Common_Apple.usd",
    "American_Beech": "airstack://scene_gen/assets/aec/brownstone/Assets/Vegetation/Trees/American_Beech.usd",
}

POLE_H_M = 25.0
POLE_R_M = 0.35
POLE_SCOPE = "/_people_poles"
ROW_POLE_SCOPE = "/_rowhome_poles"

# `Burnt_Forest_Floor` IS DELIBERATELY NOT USED ON TREES OR LOGS any more.
# It is a photographed GROUND surface; wrapped round a trunk or a log it
# reads as ground standing up. `veg.char_bole` records the same finding on
# a standing bole and rejected it there too. Generated log debris takes
# `veg.bark_material` (real bark, dark-tinted for a burnt scene); the ground
# scar still uses the surface, which is what it is for.


# --------------------- DRONE CONFIGURATION ---------------------
NUCLEUS_SERVER = "airlab-nucleus.andrew.cmu.edu"
DRONE_USD = ("~/.local/share/ov/data/documents/Kit/shared/exts/"
             "pegasus.simulator/pegasus/simulator/assets/Robots/Iris/iris.usd")

# GPS world anchor: what world (0, 0, 0) maps to in real GPS coordinates.
WORLD_GPS_ORIGIN = DEFAULT_WORLD_ORIGIN

NUM_ROBOTS = int(os.environ.get("NUM_ROBOTS") or 1)
SPAWN_HEIGHT_ABOVE_FLOOR_M = float(os.environ.get("SPAWN_HEIGHT_M") or 0.5)
LIDAR_MIN_RANGE_M = 0.75

# DEFAULT SPAWN, DERIVED — not eyeballed. One drone on a 10.7 m local road at
# the SE flank of the plat:
#   * 166 s of arrival-time margin outside the severity-0.6 fire front at the
#     default MINI_BURN_FRAC=0.45, so it starts in clear air rather than smoke;
#   * 381 m from the burn centre at (19, 110) — close enough that the scar is
#     the first thing in frame, far enough not to spawn inside it;
#   * yaw 138.1 deg = atan2(110 - -144.3, 19 - 302.4), i.e. facing the burn
#     centre, which as a quaternion [x, y, z, w] about +Z is
#     [0, 0, sin(69.05 deg), cos(69.05 deg)] = [0, 0, 0.9342, 0.3568].
# Changing MINI_SEED / MINI_BURN_FRAC / the preset's epicenter invalidates the
# margin figure; the point stays on road either way.
_DEFAULT_DRONE_CONFIGS = [
    {"domain_id": 1, "x_m": 302.4, "y_m": -144.3,
     "z_m": SPAWN_HEIGHT_ABOVE_FLOOR_M, "orient": [0.0, 0.0, 0.9342, 0.3568],
     "lidar_min_range": LIDAR_MIN_RANGE_M},
]

# Same env contract as example_multi_drone_scene_import.py. SPAWN_CONFIGS is a
# JSON list of dicts needing only x_m/y_m; SPAWN_POLY randomizes NUM_ROBOTS
# spawns inside a rect or convex polygon.
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
    _poly = json.loads(_SPAWN_POLY)
    _spawn_seed = os.environ.get("SPAWN_SEED") or "{0}-{1}".format(
        time.time_ns(), os.getpid())
    DRONE_CONFIGS = generate_spawn_configs(
        _poly,
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
    # Not fatal — the robot containers are what NUM_ROBOTS actually sizes, and
    # a mismatch shows up as a robot with no airframe rather than a crash. Say
    # so loudly, because that failure otherwise reads as a PX4 timeout.
    print("[spawn] WARNING: NUM_ROBOTS={0} but {1} spawn config(s) — robot "
          "containers without an airframe will never report ready"
          .format(NUM_ROBOTS, len(DRONE_CONFIGS)), flush=True)

print("[spawn] SCENE_CONFIG={0} (BUILT in-process, no ENV_URL)"
      .format(SCENE_CONFIG), flush=True)
print("[spawn] AIRSTACK_ASSET_ROOT={0}"
      .format(os.environ.get("AIRSTACK_ASSET_ROOT") or "<repo>"), flush=True)
print("[spawn] SPAWN_SEED={0}".format(_spawn_seed), flush=True)
print("[spawn] DRONE_CONFIGS={0}".format(json.dumps(DRONE_CONFIGS)), flush=True)

# Top-down "map" camera, republished by the GCS visualizer as a textured ground
# in Foxglove's 3D panel. Framed on the PLAT, not on the spawn area: the region
# is 1600 x 1200 m centred on the origin and a 300 m crop of it is useless as a
# mission map. 1600 m x 1.28 px/m = 2048 px, which is the helper's
# max_resolution — asking for more is silently clamped.
OVERHEAD_ALTITUDE_M     = float(os.environ.get("OVERHEAD_ALTITUDE_M") or 400.0)
OVERHEAD_COVERAGE_M     = float(os.environ.get("OVERHEAD_COVERAGE_M") or 1600.0)
OVERHEAD_CENTER_X_M     = float(os.environ.get("OVERHEAD_CENTER_X_M") or 0.0)
OVERHEAD_CENTER_Y_M     = float(os.environ.get("OVERHEAD_CENTER_Y_M") or 0.0)
OVERHEAD_PX_PER_METER   = float(os.environ.get("OVERHEAD_PX_PER_METER") or 1.28)
OVERHEAD_TOPIC          = "/sim/overhead/image"
OVERHEAD_SPEC_TOPIC     = "/sim/overhead/spec"
OVERHEAD_CENTER_X_TOPIC = "/sim/overhead/center_x"
OVERHEAD_CENTER_Y_TOPIC = "/sim/overhead/center_y"
OVERHEAD_FRAME_ID       = "map"
OVERHEAD_DOMAIN_ID      = 0

# ---------------- Ground-truth annotations (bounding boxes) ----------------
# GT_ANNOTATIONS: off | files | ros | both — wired exactly as the multi-drone
# script wires it. NOTE: boxes only cover prims carrying SEMANTIC LABELS, and
# nothing in the procedural plat authors one except the Flow fire emitters
# (`disaster.fire`, semantic_class "fire"). Houses, trees, cars and survivors
# are unlabelled, so a run with this on produces a near-empty dataset. The
# scene's real ground truth is PEOPLE_JSON, written by the people pass.
GT_ANNOTATIONS  = os.environ.get("GT_ANNOTATIONS", "off").lower()
GT_OUTPUT_DIR   = (os.environ.get("GT_OUTPUT_DIR")
                   or "/isaac-sim/AirStack/analysis_runs/gt_annotations")
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
        try:
            ext_manager.set_extension_enabled_immediate(ext, True)
        except Exception:
            ext_manager.set_extension_enabled(ext, True)


# =====================================================================
# SCENE — verbatim from suburb_assemble_launch_script.py
# =====================================================================

def _sanitize(name):
    return "".join(c if c.isalnum() else "_" for c in str(name))


def _ref(stage, dst, usd, x, y, yaw, ssf, scale=1.0, instance=True):
    prim = stage.DefinePrim(Sdf.Path(dst), "Xform")
    if not prim.GetReferences().AddReference(usd):
        return False
    xf = UsdGeom.Xformable(prim)
    xf.AddTranslateOp().Set(Gf.Vec3d(x * ssf, y * ssf, 0.0))
    xf.AddRotateZOp().Set(float(yaw))
    if scale != 1.0:
        xf.AddScaleOp().Set(Gf.Vec3f(scale, scale, scale))
    # INSTANCE IT. The same archetype is referenced many times across the plat;
    # without instancing, N copies cost N x the geometry and the trees alone
    # OOM'd Isaac at ~186M points. The transform ops sit on the instance ROOT,
    # which instancing still allows — only editing INSIDE the referenced
    # content is forbidden, which is exactly what a palette rebind does, so a
    # caller that means to recolour a building has to opt out.
    if instance:
        prim.SetInstanceable(True)
    return True


def build_row_poles(stage, clusters, ssf):
    """One CYAN pole at each row-home court. Returns the count.

    62 row units on a 1600 x 1200 m plat are perfectly present and completely
    unfindable, and "I don't see any row homes" is what that looks like from
    the cockpit. Its own scope, so it switches off independently of the
    survivor markers.
    """
    root = PARENT + ROW_POLE_SCOPE
    UsdGeom.Scope.Define(stage, Sdf.Path(root))
    n = 0
    for i, c in enumerate(clusters or ()):
        pk = (c or {}).get("parking") or {}
        ctr = pk.get("centre") or c.get("centre")
        if not ctr:
            continue
        cyl = UsdGeom.Cylinder.Define(
            stage, Sdf.Path("{0}/row_{1}".format(root, i)))
        cyl.CreateAxisAttr("Z")
        cyl.CreateHeightAttr(POLE_H_M * ssf)
        cyl.CreateRadiusAttr(POLE_R_M * ssf)
        cyl.AddTranslateOp().Set(Gf.Vec3d(float(ctr[0]) * ssf,
                                          float(ctr[1]) * ssf,
                                          POLE_H_M * 0.5 * ssf))
        cyl.CreateDisplayColorAttr([Gf.Vec3f(0.0, 0.95, 1.0)])
        n += 1
    return n


def build_people_poles(stage, recs, ssf):
    """One magenta pole at each survivor group's centroid. Returns the count.

    Sixty people over 1600 x 1200 m are individually invisible until you are
    almost on top of them. One pole per GROUP (a group is the thing you fly
    to), 25 m so it clears the houses and every tree, magenta because nothing
    in a burn scar is. All under one scope so the set switches off with a
    single prim, and carrying NO semantic label, so a run that forgets to
    disable it still produces no annotation for it.
    """
    groups = {}
    for r in recs:
        groups.setdefault((r["scenario"], int(r.get("group", 0))), []).append(r)
    root = PARENT + POLE_SCOPE
    UsdGeom.Scope.Define(stage, Sdf.Path(root))
    n = 0
    for (scenario, gi), members in sorted(groups.items()):
        cx = sum(float(m["x"]) for m in members) / len(members)
        cy = sum(float(m["y"]) for m in members) / len(members)
        path = "{0}/pole_{1}_{2}".format(root, _sanitize(scenario), gi)
        cyl = UsdGeom.Cylinder.Define(stage, Sdf.Path(path))
        cyl.CreateAxisAttr("Z")
        cyl.CreateHeightAttr(POLE_H_M * ssf)
        cyl.CreateRadiusAttr(POLE_R_M * ssf)
        # Cylinders are authored centred on their origin.
        cyl.AddTranslateOp().Set(
            Gf.Vec3d(cx * ssf, cy * ssf, POLE_H_M * 0.5 * ssf))
        cyl.CreateDisplayColorAttr([Gf.Vec3f(1.0, 0.0, 0.85)])
        # An unlit-looking marker reads at any time of day and in smoke.
        UsdGeom.Gprim(cyl).CreateDisplayOpacityAttr([1.0])
        n += 1
    return n


def _load_burnt_wood(stage):
    """The material for generated log debris: BARK, not burnt ground.

    Was a reference to `Burnt_Forest_Floor`, chosen because a flat
    colour "has no normal or ORM map, so a cylinder lit by one sun
    reads as painted pipe". The reasoning was right and the surface was
    wrong: it is a photographed GROUND texture, and `veg.char_bole`
    records the same experiment on a standing trunk and rejected it —
    ground wrapped round a log reads as ground. `veg.bark_material` is
    real oak bark at 4K WITH a normal map, tinted dark here because
    this is a burnt scene; a charred log is charred BARK.
    """
    return veg.bark_material(stage, PARENT + "/BurnLooks/log_bark",
                             tile_m=1.7, tint=(0.30, 0.26, 0.23))


def _tube(stage, path, p0, p1, r0, r1, ssf, sides=8):
    """One log or limb of the blockage — see `veg.log_mesh`.

    The private implementation this replaces authored the barrel quads
    and NOTHING ELSE, so every piece was an open tube you could see
    straight down the inside of, and it was a mathematically exact
    cylinder — the one shape nothing in a forest has. Together those
    are why a fallen log read as a hollow pipe rather than as a trunk
    that broke. `log_mesh` caps both ends and jitters the girth.
    """
    return veg.log_mesh(stage, path, p0, p1, r0, r1, ssf,
                        sides=max(7, int(sides) + 1),
                        rng=random.Random(abs(hash(path)) % 99991))


def _place_debris(stage, spec, ssf, i, mat):
    """The strewn field that makes a blockage impassable.

    `people._blocker_debris` decides WHERE (it is a planner and touches no
    stage); this authors it. Bound to the photographed charred surface rather
    than a flat dark PBR — a plain colour has no normal or ORM map, so a
    cylinder lit by one sun reads as painted pipe.
    """
    n = 0
    for j, d in enumerate(spec.get("debris") or ()):
        prim = _tube(stage, "{0}/inst/blockdeb_{1}_{2}".format(PARENT, i, j),
                     d["p0"], d["p1"], d["r0"], d["r1"], ssf,
                     sides=8 if d["kind"] == "log" else 6)
        if mat is not None:
            UsdShade.MaterialBindingAPI(prim).Bind(mat)
        n += 1
    return n


def _place_blocker(stage, spec, ssf, i):
    """Put a blockage across the carriageway. See `people._add_blocker`.

    TWO KINDS AND TWO MECHANISMS. A fallen tree is a baked archetype and lands
    the same way every other tree does — one reference, instanceable. A
    toppled streetlight is a prim that is ALREADY ON THE STAGE, so it is
    re-authored rather than added: `ClearXformOpOrder` + fresh ops on the
    instance ROOT, which USD allows.
    """
    if spec["kind"] == "fallen_tree":
        return _ref(stage, "{0}/inst/blocker_{1}".format(PARENT, i),
                    spec["usd"], spec["x"], spec["y"], spec["yaw_deg"], ssf)
    if spec["kind"] != "streetlight" or not spec.get("prim_path"):
        return False
    prim = stage.GetPrimAtPath(spec["prim_path"])
    if not (prim and prim.IsValid()):
        return False
    xf = UsdGeom.Xformable(prim)
    if not xf:
        return False
    sc = float(spec.get("scale", 1.0))
    xf.ClearXformOpOrder()
    sg._set_xform_ops(xf, prim,
                      translate=(spec["x"] * ssf, spec["y"] * ssf,
                                 spec["z"] * ssf),
                      rotate=(spec["roll_deg"], 0.0, spec["yaw_deg"]),
                      scale=(sc, sc, sc))
    return True


def _arch_from_nucleus(base):
    """Enumerate the archetype bake at a URL. Returns {name: url}.

    scene_gen/assets/archetypes/ is UNTRACKED — a fresh clone on a pod has no
    such directory, and `os.listdir` on it raises before a single prim is
    authored. The bake lives on Nucleus instead, and only DISCOVERY is
    filesystem-bound: `_ref` hands the value straight to
    `GetReferences().AddReference`, which takes an `omniverse://` URL happily.

    `omni.client.list` is the direct analogue of `os.listdir`. `archetypes.json`
    (the bake manifest) is the fallback for a server that refuses a listing; its
    `usd` fields carry the BAKE MACHINE's absolute paths, so only the basename
    is usable and it is re-joined onto *base*.
    """
    base = base.rstrip("/")
    res, entries = omni.client.list(base + "/")
    if res == omni.client.Result.OK:
        names = [e.relative_path for e in entries]
        if any(n.endswith(".usd") for n in names):
            print("[assemble] archetypes: omni.client.list on {0}".format(base))
            return {os.path.splitext(n)[0]: base + "/" + n
                    for n in names if n.endswith(".usd")}
    res, _ver, content = omni.client.read_file(base + "/archetypes.json")
    if res != omni.client.Result.OK:
        raise RuntimeError(
            "ARCH_DIR is a URL but neither listable nor carrying "
            "archetypes.json: {0} ({1})".format(base, res))
    print("[assemble] archetypes: archetypes.json manifest on {0}".format(base))
    raw = (content if isinstance(content, (bytes, bytearray))
           else bytes(memoryview(content)))
    manifest = json.loads(raw.decode("utf-8"))
    out = {}
    for rec in manifest:
        f = os.path.basename(str(rec.get("usd", "")))
        if f.endswith(".usd"):
            out[os.path.splitext(f)[0]] = base + "/" + f
    return out


def _load_archetypes():
    """The bake, from wherever ARCH_DIR points. Local path or URL, same dict."""
    if "://" in ARCH_DIR:
        arch = _arch_from_nucleus(ARCH_DIR)
    elif os.path.isdir(ARCH_DIR):
        print("[assemble] archetypes: os.listdir on {0}".format(ARCH_DIR))
        arch = {os.path.splitext(f)[0]: os.path.join(ARCH_DIR, f)
                for f in os.listdir(ARCH_DIR) if f.endswith(".usd")}
    else:
        raise RuntimeError(
            "ARCH_DIR does not exist: {0}\nThe plat is assembled BY REFERENCE "
            "to pre-baked damage archetypes; without them every house and tree "
            "is missing, and the bake is untracked so a fresh clone has none. "
            "Point ARCH_DIR at the Nucleus copy "
            "(omniverse://<host>:443/Projects/SEI-COA/scene_gen/assets/"
            "archetypes) or bake locally with "
            "bake_archetypes_launch_script.py.".format(ARCH_DIR))
    # LOUD, because zero archetypes is not an error anywhere downstream — it
    # builds a plat with roads and no houses, which reads as a bad scene rather
    # than as a broken path.
    if not arch:
        raise RuntimeError("ARCH_DIR holds no .usd archetypes: " + ARCH_DIR)
    print("[assemble] archetypes: {0} found ({1} house, {2} tree)".format(
        len(arch),
        sum(1 for k in arch if k.startswith("house_")),
        sum(1 for k in arch if k.startswith("tree_"))))
    return arch


def build_scene(stage, ssf):
    """Author the whole plat into *stage*. Returns a stats dict for the banner.

    The step numbering matches suburb_assemble_launch_script.py so the two stay
    diffable. THE ORDER IS LOAD-BEARING at 4a/4b/4c: the evacuation queue's
    cars go in BEFORE the burnable/scorch pass so they char like every other
    car on the plat, and the PEOPLE go in AFTER it, because a survivor is not
    scorched.
    """
    t0 = time.time()
    config = load_scene_config(SCENE_CONFIG)

    # 1) LAYOUT + CHEAP DETAIL, houses/trees returned as instances
    binfo = {}
    placements = generate_suburb_on_stage(stage, config, parent_path=PARENT,
                                          scene_scale_factor=ssf,
                                          info_out=binfo, assembly=True)
    add_sky(stage, resolve_sky(config))
    houses = binfo.get("house_instances", [])
    trees = binfo.get("tree_instances", [])
    print("[assemble] layout in {0:.0f}s: {1} house + {2} tree instance(s)"
          .format(time.time() - t0, len(houses), len(trees)))

    arch = _load_archetypes()

    # 2) FIRE FIELD
    fcfg = dict(fire.DEFAULTS)
    fcfg.update((config.get("disaster") or {}).get("fire") or {})
    ox, oy = fcfg["origin_m"]
    th = math.radians(float(fcfg["heading_deg"]))
    ct, stt = math.cos(th), math.sin(th)
    head, flank, back = (float(fcfg["head_mps"]), float(fcfg["flank_mps"]),
                         float(fcfg["back_mps"]))

    def arrival(x, y):
        dx, dy = x - ox, y - oy
        return fire._ignition_time(dx * ct + dy * stt, -dx * stt + dy * ct,
                                   head, flank, back)

    arr = [arrival(h["x"], h["y"]) for h in houses]
    fin = sorted(t for t in arr if math.isfinite(t))
    # SMALLER BURN by default. `max(fin)` runs the front until the LAST house
    # is reached, burning almost the whole plat; a quantile stops it earlier so
    # a clear unburnt fraction remains, which is what the drone flies toward.
    elapsed = ELAPSED_OVERRIDE or (
        fin[int(min(0.999, max(0.0, BURN_FRAC)) * (len(fin) - 1))]
        if fin else 300.0)
    span = max(1.0, elapsed - (min(fin) if fin else 0.0))
    phases = dict(ignition_s=0.03 * span, flame_s=0.35 * span,
                  smoulder_s=0.25 * span, ash_after_s=0.45 * span)

    def age(x, y):
        t = arrival(x, y)
        return -1.0 if not math.isfinite(t) else elapsed - t

    # 3) REFERENCE A HOUSE ARCHETYPE PER INSTANCE
    UsdGeom.Scope.Define(stage, Sdf.Path(PARENT + "/inst"))
    n_h = miss_h = 0
    htally = {}
    hlevels = []
    pal_jobs = []
    for i, h in enumerate(houses):
        d = age(h["x"], h["y"])
        level = "pristine" if d < 0 else damage.level_for_age(d, **phases)[0]
        # CAPTURED, because it is computed here and nowhere else. The people
        # pass needs it to tell a house that sheltered nobody from one that
        # came through — that is how the pool refuges and the exposed-interior
        # figures are chosen.
        hlevels.append(level)
        htally[level] = htally.get(level, 0) + 1
        key = "house_{0}_{1}".format(h["style"], level)
        usd = arch.get(key) or arch.get("house_{0}_pristine".format(h["style"]))
        if not usd:
            miss_h += 1
            continue
        # ROW HOMES ARE RECOLOURED, so they cannot be instanced. A terrace
        # whose units are all the same colour reads as one long building rather
        # than as eight houses, and the whole point of the morphology is that
        # they are eight houses.
        _pal = h.get("palette")
        _recolour = bool(h.get("row")) and bool(_pal)
        _hp = "{0}/inst/h_{1}".format(PARENT, i)
        if _ref(stage, _hp, usd, h["x"], h["y"], h["yaw"], ssf,
                instance=not _recolour):
            n_h += 1
            if _recolour:
                pal_jobs.append({"prim_path": _hp, "palette": _pal,
                                 "category": "house"})

    if pal_jobs:
        try:
            n_pal = mh.apply_palette(stage, pal_jobs, PARENT)
            print("[assemble] row homes: {0} subset(s) recoloured across "
                  "{1} unit(s)".format(n_pal, len(pal_jobs)))
        except Exception as _exc:
            print("[assemble] row-home palette FAILED: {0}".format(_exc))

    # 4) REFERENCE A TREE ARCHETYPE PER INSTANCE (green species usd if pristine)
    n_t = miss_t = 0
    ttally = {}
    tree_prims = []
    trng = random.Random(SEED + 71)
    for i, t in enumerate(trees):
        d = age(t["x"], t["y"])
        sp = t["species"]
        if d < 0:
            level = "pristine"
            usd = sg._join_asset_root(TREE_SPECIES.get(sp, ""), "")
            scale = 0.01
        else:
            # SAME SCALED PHASES AS THE HOUSES. With the default (fixed) phases
            # every tree in a 1600 m burn is past `ash_after` and comes out
            # `snag` — one look for the whole plat. The scaled phases spread the
            # ladder across the real arrival range.
            level = veg.level_for_age(d, **phases)[0]
            if level != "pristine":
                level = veg.stand_outcome(level, trng)
            if level == "pristine":
                usd = sg._join_asset_root(TREE_SPECIES.get(sp, ""), "")
                scale = 0.01
            else:
                usd = arch.get("tree_{0}_{1}".format(sp, level))
                scale = 1.0
        ttally[level] = ttally.get(level, 0) + 1
        if not usd:
            miss_t += 1
            continue
        _tpath = "{0}/inst/t_{1}".format(PARENT, i)
        if _ref(stage, _tpath, usd, t["x"], t["y"], t["yaw"], ssf, scale=scale):
            n_t += 1
            tree_prims.append((_tpath, float(t["x"]), float(t["y"])))

    # 4a) SURVIVORS, PLANNED. Nothing is authored here except the EVACUATION
    # QUEUE and its blockage: those are cars and they belong to the fire, so
    # they have to exist before 4b scorches everything the front reached. The
    # people themselves are held back until 4c.
    resolver = sg._make_resolver(config)
    apools = ss.AssetPools(config)
    pcfg = ppl.resolve_cfg(config)
    pctx = ppl.build_ctx(config, binfo, placements, resolver, apools,
                         age, elapsed, span, arch=arch, levels=hlevels)
    p_cars, p_blockers, p_humans, p_recs = ppl.plan_people(
        pcfg, pctx, random.Random(int(pcfg.get("seed", 91)) + SEED))
    n_blocked = n_deb = 0
    _deb_mat = None
    try:
        _deb_mat = _load_burnt_wood(stage)
    except Exception as _exc:
        print("[assemble] blockage material unavailable: {0}".format(_exc))
    for _i, _spec in enumerate(p_blockers):
        n_blocked += 1 if _place_blocker(stage, _spec, ssf, _i) else 0
        n_deb += _place_debris(stage, _spec, ssf, _i, _deb_mat)
    if n_deb:
        print("[assemble] blockage: {0} strewn piece(s) on the carriageway"
              .format(n_deb))

    # FIRE ON THE BLOCKAGE. A pile of charred timber across a road is ambiguous
    # from the air — it could be a woodpile, it could be a shadow. A plume over
    # it is not, and it is the one cue that says the road is closed NOW rather
    # than at some point in the past.
    n_flow = 0
    if p_blockers:
        try:
            fire.setup_flow_stack(stage, density_cell_size_m=0.16,
                                  max_blocks=16384, scene_scale_factor=ssf,
                                  root="/World/flow_blockage")
            for _i, _spec in enumerate(p_blockers):
                # NOT EVERY BLOCKAGE BURNS. A tree across a road is a blockage
                # whether or not it is alight, and a plat where every one of
                # them is on fire reads as staged.
                if not _spec.get("fire", True):
                    continue
                _bx = float(_spec.get("road_x", _spec.get("x", 0.0)))
                _by = float(_spec.get("road_y", _spec.get("y", 0.0)))
                # AHEAD OF THE BLOCKAGE, NOT ON THE QUEUE. The cars back up
                # BEHIND the blockage, so an emitter centred on it puts flame
                # among them. `out_bear` is the direction the queue is trying to
                # travel, so +bearing is past the blockage and away from the
                # cars — which is also where the timber fell from.
                _ob = math.radians(float(_spec.get("out_bear_deg", 0.0)))
                _fx, _fy = math.cos(_ob), math.sin(_ob)
                for _k, (_st, _along, _hz) in enumerate(
                        (("flame", 7.0, 1.1),
                         ("smoke", 15.0, 0.9))):
                    _dx, _dy = _fx * _along, _fy * _along
                    _pr = fire._flow_create(
                        stage, "/World/flow_blockage/em_{0}_{1}".format(_i, _k),
                        "FlowEmitterBox")
                    if not _pr or not _pr.IsValid():
                        continue
                    fire._set(_pr, "layer", Sdf.ValueTypeNames.Int,
                              fire.FLOW_LAYER)
                    fire._set(_pr, "position", Sdf.ValueTypeNames.Float3,
                              Gf.Vec3f((_bx + _dx) * ssf, (_by + _dy) * ssf,
                                       _hz * ssf))
                    fire._set(_pr, "halfSize", Sdf.ValueTypeNames.Float3,
                              Gf.Vec3f(3.2 * ssf, 2.6 * ssf, 1.3 * ssf))
                    fire.set_emission(_pr, _st, scale=1.0)
                    n_flow += 1
            print("[assemble] blockage: {0} Flow emitter(s) burning"
                  .format(n_flow))
        except Exception as _exc:
            print("[assemble] blockage fire FAILED: {0}".format(_exc))

    if p_cars:
        # UN-INSTANCED, like every other car on the plat: a car has to be
        # authorable to be scorched, and to have its GLASS removed — which is
        # the only way the camera ever sees the people sitting in one.
        sg.apply_placements(stage, p_cars, PARENT + "/people_cars", ssf,
                            resolver=resolver, instance_categories=set())
        from detail import vehicles as _veh
        _n_mesh = sum(_veh.open_cabin(stage, q["prim_path"], q.get("usd", ""))
                      for q in p_cars if q.get("prim_path"))
        print("[assemble] people: {0} car(s) placed ({1} glass mesh(es) "
              "stripped), {2} blocker(s)".format(len(p_cars), _n_mesh,
                                                 n_blocked))

    # 4b) FENCES + BURNABLE FURNITURE. A timber fence is a line of dry fuel on
    # the ground and one of the first things a wildfire takes: runs VANISH deep
    # in the burn and stand SCORCHED at its edge. No fracture — there is no
    # settle in the assembly, so consumed-or-scorched is the whole vocabulary.
    def coverage_at(x, y):
        d = age(x, y)
        if d < 0.0:
            return 0.0
        return min(1.0, 0.45 + 0.55 * min(1.0, d / max(1e-6, elapsed)))

    # FENCES take one shared dark OmniPBR when they survive, NOT per-subset
    # soot: there are ~7k of them and compositing a texture each blew VRAM to
    # 17 GB. EVERYTHING ELSE the fire reached is SCORCHED IN PLACE with
    # `soot_materials` and never consumed, so nothing "disappears".
    char = damage._pbr(stage, PARENT + "/BurnLooks/fence_char",
                       (0.05, 0.045, 0.04), 0.9)
    # `car` IS DELIBERATELY NOT IN THIS LIST. Car materials are glossy, tightly
    # mapped paint over a small UV layout, and a soot wash built for a brick
    # wall comes out as grey blotches. A car in a burn scar is also not usually
    # charred — the ones that burn are consumed outright and the rest are
    # ordinary cars standing in a black landscape, which is exactly what an
    # abandoned evacuation queue looks like.
    BURNABLE = ("fence", "bench", "chair", "picnic_table", "table",
                "trash_can", "play_structure", "planter", "bin", "cafe_set",
                "swing", "seesaw", "bike_rack", "mailbox", "bus_stop",
                "park_feature", "goal", "basket", "hoop")
    brng = random.Random(SEED + 5)
    n_gone = n_char = 0
    scorch_props = []
    for q in placements:
        path = q.get("prim_path")
        cat = str(q.get("category", ""))
        if not path or not any(k in cat for k in BURNABLE):
            continue                                   # only combustibles
        if damage.is_incombustible(cat):
            continue
        d = age(float(q.get("x_m", 0.0)), float(q.get("y_m", 0.0)))
        if d <= 0.0:
            continue                                   # front never reached it
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid():
            continue
        if "fence" in cat:                             # line fuel
            if d / max(1e-6, span) > 0.55 and brng.random() < 0.90:
                if prim.SetActive(False):
                    n_gone += 1
            else:
                for m in Usd.PrimRange(prim):
                    if m.IsA(UsdGeom.Mesh):
                        UsdShade.MaterialBindingAPI(m).Bind(char)
                n_char += 1
        else:                                          # cars, park props, ...
            scorch_props.append(q)
    n_soot = damage.soot_materials(stage, scorch_props, PARENT,
                                   random.Random(SEED), coverage_at=coverage_at)
    print("[assemble] burnable: {0} fence(s) consumed, {1} fence(s) charred, "
          "{2} prop subset(s) scorched (cars/park/furniture)"
          .format(n_gone, n_char, n_soot))

    # PARK GROUND SURFACES scorched. The court slabs, pitch, line markings and
    # paths are drawn geometry (not placements), so they miss the soot pass
    # above; re-bind the ones the front reached to the dark char material.
    gnd_prim = stage.GetPrimAtPath(PARENT + "/ground")
    _bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    n_park = 0
    if gnd_prim and gnd_prim.IsValid():
        for prim in gnd_prim.GetChildren():
            _nm = prim.GetName()
            if not _nm.startswith("park_"):
                continue
            # ASPHALT DOES NOT CHAR, and charring the refuge lot black would
            # delete the feature the survivors are standing on: a parking lot
            # reads as a refuge BECAUSE it is bare pavement.
            if _nm.startswith("park_parking"):
                continue
            r = _bc.ComputeWorldBound(prim).ComputeAlignedRange()
            if r.IsEmpty():
                continue
            c = r.GetMidpoint()
            if age(c[0] / ssf, c[1] / ssf) < 0.0:
                continue
            for m in Usd.PrimRange(prim):
                if m.IsA(UsdGeom.Mesh):
                    UsdShade.MaterialBindingAPI(m).Bind(char)
            n_park += 1
    print("[assemble] park ground: {0} surface(s) scorched".format(n_park))

    # 5) GROUND SCAR (built on the fly, cheap)
    region = tuple(binfo.get("region") or (-800, -600, 800, 600))
    zs = float(binfo.get("z_scale") or ss.ground_z_scale(config, region))
    burn_z = (ss._Z_GRASS + 0.5 * (ss._Z_ASPHALT - ss._Z_GRASS)) * zs
    kn = ground.knobs_from_env(max(region[2] - region[0], region[3] - region[1]))
    cov = ground.feathered_coverage(
        arrival, elapsed, (ox, oy), region, np.random.default_rng(SEED + 23),
        edge_m=kn["edge_m"], finger_m=kn["finger_m"], islands=kn["islands"])
    made = ground.build_overlay(
        stage, cov, region, ssf, burn_z, material_parent=PARENT,
        cell_m=kn["cell_m"], bands=kn["bands"], tile_m=kn["tile_m"],
        op_range=kn["op_range"],
        skip=ground.skip_rects(binfo.get("pool_rects") or (), pad=0.0))

    # FIRE-DAMAGED PAVING -> Damaged_Asphalt. `apply_ground` builds the road
    # and drive ribbons before any fire field exists, so re-bind here. Brick
    # drives are left alone — worn brick already reads as damaged.
    dmg_url = sg._join_asset_root(
        "airstack://scene_gen/assets/materials/megascans/Damaged_Asphalt.usda", "")
    dmg_path = PARENT + "/ground/materials/damaged_asphalt"
    dprim = stage.DefinePrim(Sdf.Path(dmg_path))
    dprim.GetReferences().AddReference(dmg_url)
    dprim.Load()
    dmat = UsdShade.Material(stage.GetPrimAtPath(dmg_path))
    gnd = stage.GetPrimAtPath(PARENT + "/ground")
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    n_dmg = 0
    if gnd and gnd.IsValid() and dmat:
        for prim in gnd.GetChildren():
            nm = prim.GetName()
            if not nm.startswith(("road_", "bulb_", "drive_")):
                continue
            bound = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
            if bound and "Brick" in bound.GetPrim().GetPath().pathString:
                continue                       # worn-brick drive stays brick
            r = bc.ComputeWorldBound(prim).ComputeAlignedRange()
            if r.IsEmpty():
                continue
            c = r.GetMidpoint()
            if age(c[0] / ssf, c[1] / ssf) >= 0.0:   # front reached it
                UsdShade.MaterialBindingAPI(prim).Bind(dmat)
                n_dmg += 1
    print("[assemble] {0} road/drive ribbon(s) re-bound to Damaged_Asphalt "
          "in the burn".format(n_dmg))

    # 4b-2) CLEAR A GLADE ROUND EACH OPEN-GROUND GROUP. `_open_ground` already
    # requires 15 m from any structure or tree TRUNK, but a trunk keep-out is
    # not a clearing: these crowns are 10-25 m across, so a group can satisfy
    # the rule and still sit under a closed canopy where a drone sees leaves.
    # The scenario IS "people on open ground", so the ground is opened.
    n_glade = 0
    _glade_r = float((pcfg.get("scenarios", {}).get("open_ground") or {})
                     .get("glade_r_m", 16.0))
    if _glade_r > 0.0 and tree_prims:
        _seeds = [(r["x"], r["y"]) for r in p_recs
                  if r.get("scenario") == "open_ground"]
        if _seeds:
            _r2 = _glade_r * _glade_r
            for _tp, _tx, _ty in tree_prims:
                for (_sx, _sy) in _seeds:
                    if (_tx - _sx) ** 2 + (_ty - _sy) ** 2 <= _r2:
                        _pr = stage.GetPrimAtPath(_tp)
                        if _pr and _pr.IsValid() and _pr.SetActive(False):
                            n_glade += 1
                        break
            print("[assemble] clearings: {0} tree(s) removed within {1:.0f} m "
                  "of {2} open-ground group(s)".format(
                      n_glade, _glade_r, len(_seeds)))

    # 4c) THE PEOPLE, AFTER THE SCORCH PASS. `"human"` deliberately matches
    # nothing in BURNABLE above, and holding them back until here is the
    # belt-and-braces version of that: a survivor is not scorched. Authored
    # through `apply_placements` because that is what binds each `pose` onto
    # the character's own UsdSkel rig, and NOT instanced, because the pose
    # animation is authored inside each prim.
    n_people = n_poles = n_rowp = 0
    if p_humans:
        sg.apply_placements(stage, p_humans, PARENT + "/people", ssf,
                            resolver=resolver, instance_categories=set())
        n_people = len(p_humans)
        ppl.write_records(PEOPLE_JSON, p_recs, meta={
            "seed": SEED, "scene_config": SCENE_CONFIG,
            "elapsed_s": round(elapsed, 1), "span_s": round(span, 1),
            "burn_frac": BURN_FRAC,
            "fire_origin_m": [ox, oy],
            "fire_heading_deg": float(fcfg["heading_deg"]),
            "blockers": p_blockers,
        })
        print("[assemble] people: {0} authored, ground truth -> {1}"
              .format(n_people, PEOPLE_JSON))
        if POLES:
            n_poles = build_people_poles(stage, p_recs, ssf)
            n_rowp = build_row_poles(stage, binfo.get("clusters"), ssf)
            print("[assemble] poles: {0} survivor (magenta, {1}{2}) + {3} "
                  "row-home (cyan, {1}{4}) — deactivate the scope to hide"
                  .format(n_poles, PARENT, POLE_SCOPE, n_rowp, ROW_POLE_SCOPE))

    # The extra_args form only covers startup; re-assert once the stage is
    # composed so car glass stays see-through for the whole run.
    for _key in ("/rtx/raytracing/fractionalCutoutOpacity",
                 "/rtx/pathtracing/fractionalCutoutOpacity"):
        carb.settings.get_settings().set_bool(_key, True)
    for _ in range(30):
        omni.kit.app.get_app().update()

    _ptally = {}
    for _r in p_recs:
        _ptally[_r["scenario"]] = _ptally.get(_r["scenario"], 0) + 1
    return {
        "seconds": time.time() - t0,
        "region": region, "elapsed_s": elapsed, "span_s": span,
        "houses": n_h, "houses_missing": miss_h, "house_tally": htally,
        "trees": n_t, "trees_missing": miss_t, "tree_tally": ttally,
        "bands": len(made), "flow": n_flow,
        "people": n_people, "people_alive": sum(1 for _r in p_recs
                                                if _r.get("alive")),
        "people_tally": _ptally,
        "cars": len(p_cars), "blockers": len(p_blockers),
        "poles": n_poles + n_rowp,
    }


# =====================================================================
# DRONES — from example_multi_drone_scene_import.py
# =====================================================================

def nucleus_stat(url: str) -> bool:
    result, info = omni.client.stat(url)
    return result == omni.client.Result.OK


def wait_for_stage(stage, timeout_s: float = 20.0):
    """Pump the Kit app loop until /World has content."""
    for _ in range(int(timeout_s / 0.1)):
        omni.kit.app.get_app().update()
        world_prim = stage.GetPrimAtPath("/World")
        if world_prim.IsValid():
            non_physics = [c for c in world_prim.GetChildren()
                           if c.GetName() != "PhysicsScene"]
            if non_physics:
                return True
    return False


class PegasusApp:

    def __init__(self):
        # FIRST, before ~450 s of scene build. Robot containers block on these
        # files, and the origins are known from DRONE_CONFIGS alone — making
        # them wait out the assembly is pure dead time on the readiness gate.
        set_gps_origins(DRONE_CONFIGS, world_origin=WORLD_GPS_ORIGIN)

        omni.client.initialize()
        nucleus_stat("omniverse://{0}".format(NUCLEUS_SERVER))

        self.timeline = omni.timeline.get_timeline_interface()
        self.timeline.stop()

        self.pg = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        # Pegasus' flat default env — NOT the finished Nucleus stage the
        # multi-drone script loads. It exists only to give the World a valid
        # base; its ground plane and lighting are switched off immediately
        # because the generated plat brings its own.
        self.pg.load_environment(BASE_ENV_URL)

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("Stage failed to load")
        if not wait_for_stage(stage):
            carb.log_warn("Base stage load timed out - continuing anyway.")
        for name in ("GroundPlane", "Environment"):
            p = stage.GetPrimAtPath("/World/" + name)
            if p and p.IsValid():
                p.SetActive(False)

        dedupe_physics_scenes(stage)

        # Units. The generated plat is authored in metres * ssf, so NOTHING is
        # rescaled afterwards — `scale_stage_prim` deliberately does not appear
        # in this script.
        mpu, s = get_stage_meters_per_unit(stage)
        self.scene_scale_factor = s

        # ---------- THE SCENE ----------
        stats = build_scene(stage, s)
        self._print_scene_banner(stats)

        # ---------- EVERYTHING BELOW NEEDS THE GEOMETRY ----------
        self._add_colliders(stage)

        # Top-down orthographic map camera over the plat. One JPEG-rate aerial
        # of the static scene that the GCS visualizer republishes as a textured
        # ground for Foxglove's 3D panel.
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

        # ---------- THE DRONES ----------
        gt_cameras = []
        for cfg in DRONE_CONFIGS:
            i = cfg["domain_id"]
            pos = [cfg["x_m"] * s, cfg["y_m"] * s, cfg["z_m"] * s]

            graph_handle = spawn_px4_multirotor_node(
                pegasus_node_name="PX4Multirotor_{0}".format(i),
                drone_prim="/World/drone{0}/base_link".format(i),
                robot_name="robot_{0}".format(i),
                vehicle_id=i,
                domain_id=i,
                usd_file=DRONE_USD,
                init_pos=pos,
                init_orient=cfg["orient"],
            )

            add_zed_stereo_camera_subgraph(
                parent_graph_handle=graph_handle,
                drone_prim="/World/drone{0}/base_link".format(i),
                robot_name="robot_{0}".format(i),
                camera_name="ZEDCamera",
                camera_offset=[0.21, 0.0, 0.05],
                camera_rotation_offset=[0.0, 0.0, 0.0],
            )

            add_rtx_lidar_subgraph(
                parent_graph_handle=graph_handle,
                drone_prim="/World/drone{0}/base_link".format(i),
                robot_name="robot_{0}".format(i),
                lidar_config="ouster_os1",
                lidar_topic_name="point_cloud_raw",
                lidar_offset=[0.0, 0.0, 0.025],
                lidar_rotation_offset=[0.0, 0.0, 0.0],
                min_range=cfg["lidar_min_range"],
            )

            # Left eye of the ZED — the view ground truth is generated for.
            # Path matches what add_zed_stereo_camera_subgraph builds.
            gt_cameras.append(
                ("robot_{0}".format(i),
                 "/World/drone{0}/base_link/ZEDCamera/base_link/ZED_X/"
                 "camera_left".format(i)))

            if GT_ANNOTATIONS in ("ros", "both"):
                add_bbox_ros_publisher(
                    graph_path="/World/GTBBoxGraph_{0}".format(i),
                    camera_prim_path=gt_cameras[-1][1],
                    robot_name="robot_{0}".format(i),
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

        self.play_on_start = (
            os.environ.get("PLAY_SIM_ON_START", "true").lower() == "true")
        self.stop_sim = False
        self._print_drone_banner()

    def _add_colliders(self, stage):
        if COLLIDERS == "off":
            print("[suburb+drones] colliders: OFF (SUBURB_COLLIDERS=off)",
                  flush=True)
            return
        target = (PARENT + "/ground") if COLLIDERS == "ground" else "/World/stage"
        prim = stage.GetPrimAtPath(target)
        if not (prim and prim.IsValid()):
            carb.log_warn("{0} not found - no colliders added.".format(target))
            return
        t0 = time.time()
        add_colliders(prim)
        for _ in range(10):
            omni.kit.app.get_app().update()
        print("[suburb+drones] colliders on {0} in {1:.0f}s"
              .format(target, time.time() - t0), flush=True)

    def _print_scene_banner(self, st):
        r = st["region"]
        print("\n" + "=" * 72, flush=True)
        print("SCENE BUILT - {0} ({1:.0f} x {2:.0f} m, by reference)".format(
            SCENE_CONFIG, r[2] - r[0], r[3] - r[1]))
        print("  assembly    {0:.0f} s".format(st["seconds"]))
        print("  fire        t+{0:.0f} s elapsed over a {1:.0f} s span "
              "(burn_frac {2:.2f}, seed {3})".format(
                  st["elapsed_s"], st["span_s"], BURN_FRAC, SEED))
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
                  PEOPLE_JSON))
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
        print("  gt          GT_ANNOTATIONS={0}".format(GT_ANNOTATIONS))
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
