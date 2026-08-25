#!/usr/bin/env python
"""
Preview launcher for the ModularNeighborhood house kit
(scene_gen/detail/modular_house.py).

Spawns every style in the catalogue — 6 residential, 8 suburban commercial —
assembled piece by piece from
`/Library/Stages/Muyang/ModularNeighborhood/Assets/` rather than placed as
whole-house models. Nothing else: no drone, no sensors, no ROS 2 bridge, so it
boots fast and is only about whether the kit assembles correctly.

    ISAAC_SIM_SCRIPT_NAME=modular_house_preview_launch_script.py \
    airstack up isaac-sim

This is a CATALOGUE, not a neighbourhood: every style on a grid plus a strip of
the pack's loose props, so the range of shapes, materials and roof forms can be
read off in one view. Making it look like a coherent place is the suburb
generator's job.

Environment:
    HOUSE_SEED      RNG seed for the fenestration (default 7)
    HOUSE_DRESS     0 to skip driveways/fences/pools and see the shells alone

RELOAD WITHOUT RESTARTING
-------------------------
Open the Kit Script Editor (Window -> Script Editor) and run:

    import os, sys, importlib, random
    SCENE_GEN = "/isaac-sim/AirStack/scene_gen"
    ISAAC_DIR = "/isaac-sim/AirStack/simulation/isaac-sim"
    sys.path[:0] = [SCENE_GEN, os.path.join(ISAAC_DIR, "utils")]
    import scene_generator, scene_prep
    from detail import modular_house
    for m in (scene_generator, modular_house):
        importlib.reload(m)
    import omni.usd, omni.timeline
    from pxr import Sdf
    stage = omni.usd.get_context().get_stage()
    omni.timeline.get_timeline_interface().stop()
    stage.RemovePrim(Sdf.Path("/World/stage/generated"))    # clear the old build
    _, ssf = scene_prep.get_stage_meters_per_unit(stage)
    pl = modular_house.build_catalogue(random.Random(7))
    scene_generator.apply_placements(stage, pl, "/World/stage/generated", ssf)
    modular_house.apply_palette(stage, pl, "/World/stage/generated")
    omni.timeline.get_timeline_interface().play()
"""

import os
import random
import sys
import time


def _env(name, default):
    """Compose forwards an unset mission var as the EMPTY STRING, not as absent,
    so `os.environ.get(name, default)` returns "" and int()/float() raise."""
    v = os.environ.get(name)
    return default if v is None or v.strip() == "" else v.strip()


def _flag(name, default=False):
    v = _env(name, "1" if default else "0").lower()
    return v in ("1", "true", "yes", "on")

import carb
from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": _flag("ISAAC_SIM_HEADLESS")})

from isaacsim.core.utils.extensions import enable_extension

# The reload loop above depends on the Script Editor every run.
enable_extension("omni.kit.window.script_editor")

import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics

from omni.isaac.core.world import World
from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

_ISAAC_SIM_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
_SCENE_GEN_DIR = os.path.normpath(
    os.path.join(_ISAAC_SIM_DIR, "..", "..", "scene_gen"))
sys.path.insert(0, os.path.join(_ISAAC_SIM_DIR, "utils"))
sys.path.insert(0, _SCENE_GEN_DIR)

sys.path.insert(0, os.path.join(_ISAAC_SIM_DIR, "launch_scripts"))

from scene_prep import (scale_stage_prim, add_sky, get_stage_meters_per_unit,
                        add_colliders, add_orthographic_camera,
                        add_overhead_camera_publisher)
import scene_annotations as sa
import scene_generator as sg
from detail import modular_house

# ----- CONFIGURATION -----
ENV_URL = SIMULATION_ENVIRONMENTS["Default Environment"]
STAGE_SCALE = 1.00
SEED = int(_env("HOUSE_SEED", "7"))
SPACING_M = float(_env("HOUSE_SPACING", "30"))
DRESS = _flag("HOUSE_DRESS", True)
PARENT = "/World/stage/generated"

# SPAWN_DRONE=1 turns this catalogue into a MAPPING TEST BED: the same
# undamaged shells, plus one PX4 multirotor with a ZED stereo camera, so an
# occupancy-mapping planner has real geometry to see. Off by default — the
# catalogue's own job is "does the kit assemble", and a drone costs it a PX4
# boot and a camera render it does not need.
#
# Buildings on flat ground are deliberately the WHOLE scene. A fully empty map
# exercises nothing: an occupancy grid built over featureless ground is
# indistinguishable from one that is broken.
SPAWN_DRONE = _flag("SPAWN_DRONE")
# HOUSE_LIMIT trims the catalogue to the first N styles. For a mapping test a
# handful of shells is the whole point — enough geometry that an occupancy grid
# has something to be right or wrong about, few enough that Isaac leaves room
# on the card for a VLM beside it. 0 = the full catalogue.
HOUSE_LIMIT = int(_env("HOUSE_LIMIT", "0"))

if SPAWN_DRONE:
    # `spawn_px4_multirotor_node` builds an OmniGraph node whose TYPE is
    # registered by the pegasus extension. Importing PegasusInterface is not
    # enough — without these enabled the spawn dies with
    # "unrecognized type 'pegasus.simulator.PegasusMultirotorPX4Node'".
    # Enabled only in drone mode so the catalogue keeps its fast boot.
    _ext = omni.kit.app.get_app().get_extension_manager()
    for _e in ("omni.graph.core", "omni.graph.action", "omni.graph.action_nodes",
               "omni.graph.ui", "omni.graph.visualization.nodes",
               "omni.graph.scriptnode", "omni.graph.window.action",
               "omni.graph.window.generic", "omni.graph.ui_nodes",
               "pegasus.simulator"):
        if not _ext.is_extension_enabled(_e):
            try:
                _ext.set_extension_enabled_immediate(_e, True)
            except Exception:
                _ext.set_extension_enabled(_e, True)
# Clear of the catalogue grid, looking back along +Y at the row of shells.
DRONE_XY = [float(v) for v in _env("DRONE_XY", "0,-45").split(",")[:2]]
DRONE_Z_M = float(_env("DRONE_Z_M", "1.0"))
# The RTX lidar is what `example_multi_drone_scene_import.py` adds and this
# script omitted. Same var, same default as that launcher, so the two spawn
# paths are comparable and "cameras go dark with the lidar attached" is one
# flag apart from "they do not".
ENABLE_LIDAR = _flag("ENABLE_LIDAR")

# Emit ground-truth boxes for what the layout generator placed — houses, cars,
# trees, pool, props — measured off the composed stage and written to the same
# annotation files the GCS already draws (`/gcs/annotations/bboxes`) and raven's
# scorer already reads. The generator knows exactly what it placed, so its GT
# should come from the generator rather than be hand-authored or measured back
# off a render.
#
# RESULTS_SCENE names the file, because that is the variable the GCS's
# annotation_viz_node already uses to choose one.
GT_ANNOTATIONS = _flag("GT_ANNOTATIONS")
RESULTS_SCENE = _env("RESULTS_SCENE", "ModularHousePreview")

# DOWNWARD tilt of the ZED, in degrees, about the drone's body Y — positive is
# down. Default 0 keeps the LEVEL mount every other method in this stack assumes;
# only a CoNavGPT run should set it.
#
# Why it helps CoNavGPT and not much else: CoNavGPT builds its occupancy map by
# unprojecting FPV depth and rasterising it top-down. A level camera at 12-20 m
# AGL spends the top half of every frame on sky and never observes the ground
# under the drone, so the map keeps a permanent hole around the robot. At 30 deg
# the top ray is still 9 deg above the horizon — horizon context is kept — while
# the bottom ray meets the ground a few metres ahead instead of ~15.
#
# THE NODE MUST BE TOLD. TF walks the URDF, which models no pitch, so a tilted
# camera is one TF cannot see. search_planner reads this SAME variable as the
# default for camera_pitch_rad precisely so the two cannot disagree.
ZED_PITCH_DEG = float(_env("ZED_PITCH_DEG", "0"))

# Top-down orthographic camera over the catalogue. The GCS visualizer
# republishes this one JPEG as the textured ground under Foxglove's 3D panel
# (/gcs/sim_ground), so WITHOUT it the 3D panel has no scene under the drone —
# which reads as "the GCS is broken" rather than "nothing publishes the ground".
# On domain 0, the GCS's, because that is who consumes it.
#
# The catalogue spans roughly x[-105, 105] and y[-130, 30], so 320 m of coverage
# centred on (0, -50) holds it with margin. Only spawned in drone mode: the bare
# catalogue has no GCS to feed.
OVERHEAD_COVERAGE_M = float(_env("OVERHEAD_COVERAGE_M", "320"))
OVERHEAD_CENTER_X_M = float(_env("OVERHEAD_CENTER_X_M", "0"))
OVERHEAD_CENTER_Y_M = float(_env("OVERHEAD_CENTER_Y_M", "-50"))
OVERHEAD_ALTITUDE_M = float(_env("OVERHEAD_ALTITUDE_M", "400"))
# 320 m x 6.4 px/m = 2048 px, exactly the helper's cap.
OVERHEAD_PX_PER_METER = float(_env("OVERHEAD_PX_PER_METER", "6.4"))
# -------------------------

_ENV_CLUTTER = {"GroundPlane", "Environment"}


def _remove_env_clutter(stage):
    """Deactivate the base environment's GroundPlane and Environment xforms.

    Same reasoning as `scene_launch_script`: `RemovePrim` cannot delete across a
    reference — it returns False rather than raising — so these are deactivated,
    which does compose and is reversible in the viewport.
    """
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
            else:
                UsdGeom.Imageable(child).MakeInvisible()
    print(f"[modular_house] env clutter: {n} prim(s) deactivated")


def add_colliders_skip_empty(prim):
    """`scene_prep.add_colliders`, skipping point-less stub meshes.

    The kit carries material-only USDs whose meshes have no points; PhysX logs
    one error per empty mesh per instance, which buries real problems.
    """
    applied = skipped = 0
    for p in Usd.PrimRange(prim):
        if not p.IsA(UsdGeom.Gprim):
            continue
        if p.IsA(UsdGeom.Mesh):
            pts = p.GetAttribute("points")
            if not pts or not pts.HasAuthoredValue() or not (pts.Get() or []):
                skipped += 1
                continue
        if not p.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI.Apply(p)
            applied += 1
    print(f"[modular_house] colliders: {applied} applied, {skipped} empty skipped")
    return applied, skipped


def report_missing(stage, placements):
    """Name any placement that resolved to nothing.

    The kit has 293 USDs of which a good number are MATERIALS, not geometry —
    `Fence_01.usd` is a material and `Fence_01_0.usd` is the fence. Referencing
    the wrong one places a prim that loads clean and draws nothing, so it is
    worth saying out loud rather than wondering where the fences went.
    """
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    empty = {}
    for p in placements:
        path = p.get("prim_path")
        if not path:
            continue
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid():
            continue
        if bc.ComputeWorldBound(prim).ComputeAlignedRange().IsEmpty():
            empty[p["usd"]] = empty.get(p["usd"], 0) + 1
    if empty:
        print(f"[modular_house] WARN {sum(empty.values())} placements drew nothing:")
        for usd, n in sorted(empty.items(), key=lambda kv: -kv[1]):
            print(f"[modular_house]   {n:4d} x {usd}")
    else:
        print("[modular_house] every placement drew geometry")
    return empty


def wait_for_stage(stage, timeout_s: float = 10.0):
    for _ in range(int(timeout_s / 0.1)):
        omni.kit.app.get_app().update()
        world_prim = stage.GetPrimAtPath("/World")
        if world_prim.IsValid():
            non_physics = [c for c in world_prim.GetChildren()
                           if c.GetName() != "PhysicsScene"]
            if non_physics:
                return True
        time.sleep(0.1)
    return False


class ModularHousePreviewApp:
    def __init__(self):
        self.timeline = omni.timeline.get_timeline_interface()
        self.timeline.stop()

        pg = PegasusInterface()
        pg._world = World(**pg._world_settings)
        pg.load_environment(ENV_URL)

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("Stage failed to load")
        if not wait_for_stage(stage):
            carb.log_warn("Stage load timed out — continuing anyway.")

        # The GroundPlane is clutter for a catalogue and load-bearing for a
        # drone — without it there is nothing to take off from or land on.
        if not SPAWN_DRONE:
            _remove_env_clutter(stage)
        else:
            p = stage.GetPrimAtPath("/World/Environment")
            if p and p.IsValid():
                p.SetActive(False)
        stage_prim = stage.GetPrimAtPath("/World/stage")
        if stage_prim.IsValid():
            scale_stage_prim(stage, "/World/stage", STAGE_SCALE)
            for _ in range(10):
                omni.kit.app.get_app().update()

        rng = random.Random(SEED)
        # Fails with a sentence if a style's roof does not cover its
        # footprint or a style has no front door — cheaper than
        # finding either in a render.
        modular_house.check()
        if HOUSE_LIMIT > 0:
            # build_catalogue walks the module-level ORDER, so trimming it here
            # is the whole knob — no second layout path to keep in step.
            modular_house.ORDER = modular_house.ORDER[:HOUSE_LIMIT]
            print(f"[modular_house] HOUSE_LIMIT={HOUSE_LIMIT} -> styles "
                  f"{list(modular_house.ORDER)}")
        placements = modular_house.build_catalogue(rng, dress=DRESS)
        print("[modular_house] " +
              modular_house.summarise(placements).replace("\n", "\n[modular_house] "))

        _, ssf = get_stage_meters_per_unit(stage)
        sg.apply_placements(stage, placements, PARENT, ssf)
        # After placement, because it needs each prim_path. The kit ships ONE
        # wall texture, so without this every building is the same cream.
        n_mat = modular_house.apply_palette(stage, placements, PARENT)
        print(f'[modular_house] palette: {n_mat} subsets rebound')

        # PHYSICS COLLIDERS ARE OFF, deliberately.
        #
        # `add_colliders_skip_empty` applies UsdPhysics.CollisionAPI to EVERY
        # gprim under the generated root — 76,668 of them on a full suburb —
        # and PhysX then cooks collision for all of them inside the FIRST
        # `app.update()`. That one frame ran long enough that the window stopped
        # responding, X invalidated it (`BadWindow`), and Kit tore down a render
        # thread through pybind11 without the GIL, which aborts the process.
        #
        # Nothing here needs physics yet: this is a capture scene, and neither
        # rendering nor LiDAR reads CollisionAPI. Re-enable when something has
        # to fly into the geometry — and filter by category first, because a
        # drone needs to collide with building shells, trees and poles, not with
        # hedges, bins, chairs, paving slabs or window frames.
        #
        # gen = stage.GetPrimAtPath(PARENT); if gen.IsValid():
        #     add_colliders_skip_empty(gen)
        for _ in range(10):
            omni.kit.app.get_app().update()

        report_missing(stage, placements)
        add_sky(stage, "")

        if GT_ANNOTATIONS:
            # AFTER apply_placements, because every box is read off the composed
            # stage — a placement has no world bound until it is referenced in.
            repo = os.path.normpath(os.path.join(_ISAAC_SIM_DIR, "..", ".."))
            boxes = sa.boxes_from_placements(stage, placements)
            sa.write_annotations(RESULTS_SCENE, boxes, sa.annotation_dirs(repo))
            print("[modular_house] set RESULTS_SCENE={0} on the GCS to draw "
                  "these".format(RESULTS_SCENE))

        if SPAWN_DRONE:
            self._spawn_drone(stage, ssf)

        print("\n" + "=" * 70)
        print("MODULAR HOUSE PREVIEW READY")
        print(f"  seed {SEED}   spacing {SPACING_M} m   dressing {'on' if DRESS else 'off'}")
        for s in modular_house.ORDER:
            print(f"    {s:<12} {modular_house.STYLES[s]['note']}")
        print("  reload: see the snippet in this script's docstring")
        print("=" * 70 + "\n")
        self.timeline.play()

    def _spawn_drone(self, stage, ssf):
        """One PX4 multirotor with a ZED stereo camera, for mapping tests.

        Colliders go on the GROUND ONLY. The comment above explains why the
        blanket pass is off (76k gprims cooked in one frame took the window
        down); a mapping test needs somewhere to stand, and the depth image
        that feeds the occupancy grid comes from the RENDERER, which does not
        read CollisionAPI — so the shells need no collider to be mapped.
        """
        from gps_utils import set_gps_origins, DEFAULT_WORLD_ORIGIN
        from pegasus.simulator.ogn.api.spawn_multirotor import spawn_px4_multirotor_node
        from pegasus.simulator.ogn.api.spawn_zed_camera import add_zed_stereo_camera_subgraph
        from pegasus.simulator.ogn.api.spawn_rtx_lidar import add_rtx_lidar_subgraph

        x, y = DRONE_XY
        cfg = [{"domain_id": 1, "x_m": x, "y_m": y, "z_m": DRONE_Z_M,
                "orient": [0.0, 0.0, 0.0, 1.0], "lidar_min_range": 0.75}]
        # Written before the spawn: robot containers block on these files, and
        # making them wait out a PX4 boot for a number already known is dead time.
        set_gps_origins(cfg, world_origin=DEFAULT_WORLD_ORIGIN)

        gp = stage.GetPrimAtPath("/World/GroundPlane")
        if gp and gp.IsValid():
            add_colliders(gp)

        cam_path = add_orthographic_camera(
            stage, prim_path="/World/MapCamera",
            altitude_m=OVERHEAD_ALTITUDE_M, coverage_m=OVERHEAD_COVERAGE_M,
            scene_scale_factor=ssf,
            center_x_m=OVERHEAD_CENTER_X_M, center_y_m=OVERHEAD_CENTER_Y_M)
        add_overhead_camera_publisher(
            parent_graph_path="/World/MapCameraGraph",
            camera_prim_path=cam_path, frame_id="map",
            coverage_m=OVERHEAD_COVERAGE_M,
            center_x_m=OVERHEAD_CENTER_X_M, center_y_m=OVERHEAD_CENTER_Y_M,
            pixels_per_meter=OVERHEAD_PX_PER_METER, domain_id=0)
        print("[modular_house] overhead map camera: {0:.0f} m over "
              "({1:.0f}, {2:.0f}) -> /sim/overhead/image on domain 0"
              .format(OVERHEAD_COVERAGE_M, OVERHEAD_CENTER_X_M,
                      OVERHEAD_CENTER_Y_M))

        graph = spawn_px4_multirotor_node(
            pegasus_node_name="PX4Multirotor_1",
            drone_prim="/World/drone1/base_link",
            robot_name="robot_1", vehicle_id=1, domain_id=1,
            usd_file="~/.local/share/ov/data/documents/Kit/shared/exts/"
                     "pegasus.simulator/pegasus/simulator/assets/Robots/Iris/iris.usd",
            init_pos=[x * ssf, y * ssf, DRONE_Z_M * ssf],
            init_orient=cfg[0]["orient"])
        add_zed_stereo_camera_subgraph(
            parent_graph_handle=graph, drone_prim="/World/drone1/base_link",
            robot_name="robot_1", camera_name="ZEDCamera",
            camera_offset=[0.21, 0.0, 0.05],
            camera_rotation_offset=[0.0, ZED_PITCH_DEG, 0.0])
        if ENABLE_LIDAR:
            # Argument-for-argument what example_multi_drone_scene_import.py
            # spawns, so the only difference between the two paths is the scene.
            add_rtx_lidar_subgraph(
                parent_graph_handle=graph, drone_prim="/World/drone1/base_link",
                robot_name="robot_1", lidar_config="ouster_os1",
                lidar_topic_name="point_cloud_raw",
                lidar_offset=[0.0, 0.0, 0.025],
                lidar_rotation_offset=[0.0, 0.0, 0.0],
                min_range=cfg[0]["lidar_min_range"])
        print("=" * 70)
        print("DRONE UP - robot_1 at ({0:.1f}, {1:.1f}, {2:.1f}) with a ZED stereo "
              "camera pitched {3:.0f} deg down, lidar {4}"
              .format(x, y, DRONE_Z_M, ZED_PITCH_DEG,
                      "on" if ENABLE_LIDAR else "off"))
        print("  PX4 is booting; readiness gates on that, not on this banner.")
        print("=" * 70)

    def run(self):
        app = omni.kit.app.get_app()
        n = 0
        while simulation_app.is_running():
            # A catalogue preview only needs frames, so app.update() is enough.
            # A drone needs PHYSICS: without world.step() nothing advances sim
            # time and the ROS graphs never tick. Their nodes hang off
            # OnPlaybackTick, which fires ONLY while the timeline is playing --
            # a stopped timeline leaves every sensor topic ADVERTISED but
            # silent, which reads like a DDS problem and is not one.
            world = World.instance() if SPAWN_DRONE else None
            if world is not None and hasattr(world, "_scene"):
                world.step(render=True)
            else:
                app.update()
            if SPAWN_DRONE:
                n += 1
                # Pegasus can swap the World out from under us while the
                # vehicle initialises, which stops the timeline. Re-assert it.
                if n % 120 == 0 and not self.timeline.is_playing():
                    print("[preview] timeline stopped - replaying")
                    self.timeline.play()
                if n % 600 == 0:
                    print("[preview] tick {0}  playing={1}  world={2}"
                          .format(n, self.timeline.is_playing(), world is not None))
        self.timeline.stop()
        simulation_app.close()


def main():
    ModularHousePreviewApp().run()


if __name__ == "__main__":
    main()
