#!/usr/bin/env python
"""
Keyboard-driven quadruped in a procedurally generated scene.

Builds the same city as scene_preview_launch_script.py, then spawns one of Isaac
Sim's policy quadrupeds (Spot or ANYmal C) and drives it manually. No drone, no
PX4, no ROS 2 bridge: the pretrained locomotion policy runs in-process in the
physics callback and consumes a body-frame velocity command (v_x, v_y, w_z).

    QUADRUPED=spot SCENE_CONFIG=urban_earthquake airstack up isaac-sim

with ISAAC_SIM_SCRIPT_NAME=quadruped_scene_launch_script.py in .env.

Controls — click the viewport first so Kit has keyboard focus:

    I   forward        J   yaw left        U   all stop
    K   back           L   yaw right

Two things about the generated scene need fixing up before a legged robot can
walk in it; both are handled below and neither affects the drone scripts:

  * Mesh colliders are approximated, not exact, so a streetlight collides as
    the convex hull of its pole *and* overhanging arm — a solid invisible wedge
    you cannot walk under — and each stretched, deliberately overlapping
    sidewalk tile becomes its own lumpy hull. See _use_exact_mesh_collision.
  * The spawn has to land on open ground rather than on that tiling. See
    _find_open_spawn.
"""

import math
import os
import sys
import time

import carb
import numpy as np
from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": False})

from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.robot.policy.examples")

import omni.appwindow
import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics
from omni.isaac.core.world import World
from isaacsim.core.utils.viewports import set_active_viewport_camera
from isaacsim.robot.policy.examples.robots import (AnymalFlatTerrainPolicy,
                                                   SpotFlatTerrainPolicy)
from isaacsim.storage.native import get_assets_root_path
from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

_ISAAC_SIM_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
_SCENE_GEN_DIR = os.path.normpath(
    os.path.join(_ISAAC_SIM_DIR, "..", "..", "scene_gen"))
sys.path.insert(0, os.path.join(_ISAAC_SIM_DIR, "utils"))
sys.path.insert(0, _SCENE_GEN_DIR)
from scene_prep import (scale_stage_prim, add_colliders, add_dome_light,
                        get_stage_meters_per_unit, settle_rigid_props)
from scene_generator import generate_scene_on_stage, _make_physx_ground_snap
from compile_disaster import load_scene_config

# ----- CONFIGURATION -----
ENV_URL     = SIMULATION_ENVIRONMENTS["Default Environment"]
STAGE_SCALE = 1.00
SCENE_CONFIG = os.environ.get("SCENE_CONFIG") or os.path.join(
    _SCENE_GEN_DIR, "config", "presets", "urban_earthquake.yaml")

# body_link is the rigid body the chase camera rides. The prim the robot is
# referenced at is only the articulation root Xform and never moves — the links
# under it are what physics drives, which is why a camera parented to the root
# sits at the spawn pose forever. Verified against the 5.1 assets: spot.usd has
# /spot/body, anymal_c.usd has /anymal/base.
# physics_dt / rendering_dt here are only a bootstrap; the real values come from
# the policy's own env yaml once it loads (see the set_simulation_dt call).
ROBOTS = {
    "spot":   {"cls": SpotFlatTerrainPolicy,   "physics_dt": 1 / 500,
               "rendering_dt": 1 / 50, "stand_h": 0.80, "body_link": "body"},
    "anymal": {"cls": AnymalFlatTerrainPolicy, "physics_dt": 1 / 200,
               "rendering_dt": 1 / 50, "stand_h": 0.70, "body_link": "base"},
}
QUADRUPED = os.environ.get("QUADRUPED", "spot").strip().lower()

SPAWN_XY = [float(v) for v in
            os.environ.get("QUADRUPED_SPAWN", "0,0").split(",")]
# Metres of open ground the spawn search wants around the robot. Spot is a bit
# over a metre long, so this leaves room to stand and start walking.
SPAWN_CLEARANCE = float(os.environ.get("SPAWN_CLEARANCE", "3.0"))

# Physics steps to hold a zero command after reset so the legs settle before
# the keyboard is live.
WARMUP_STEPS = int(os.environ.get("QUADRUPED_WARMUP", "250"))

# Chase camera: fixed local offset on the body link, in metres, plus a
# rotateXYZ. USD cameras look down -Z with +Y up, so (72, 0, -90) points it
# along the body's +X with +Z up and 18 degrees of downward tilt.
CHASE_OFFSET = [float(v) for v in
                os.environ.get("CHASE_OFFSET", "-4.8,0,2.5").split(",")]
CHASE_ROT = [float(v) for v in
             os.environ.get("CHASE_ROT", "72,0,-90").split(",")]
# Horizontal field of view in degrees; USD cameras carry an aperture and focal
# length rather than an FOV, so this is converted below.
CHASE_FOV = float(os.environ.get("CHASE_FOV", "90"))
CHASE_APERTURE = 20.955        # mm, the USD/Kit default horizontal aperture

# get_assets_root_path() reads this carb setting and raises if unset. runapp.sh
# passes it on the command line, but the standalone launch path does not, so the
# robot USD and the policy .pt would both fail to resolve without this.
ASSET_ROOT = os.environ.get("OMNI_SERVER", "").strip().strip('"').strip("'")

# Lighting. The asset sets configure `sky:` as a stage USD (RetroNeighborhood),
# which add_sky composes in as a baked sky sphere plus its own lights. A dome
# light textured with an HDRI is used unconditionally here instead.
DOME_LIGHT_PATH = "/World/DomeLight"
DOME_LIGHT_INTENSITY = float(os.environ.get("DOME_LIGHT_INTENSITY", "1000.0"))
DOME_LIGHT_EXPOSURE = float(os.environ.get("DOME_LIGHT_EXPOSURE", "0.0"))
DOME_LIGHT_TEXTURE = (os.environ.get("DOME_LIGHT_TEXTURE", "").strip()
                      or "{root}/NVIDIA/Assets/Skies/Clear/noon_grass_4k.hdr")
# -------------------------


_ENV_CLUTTER = {"GroundPlane", "Environment"}
# USD's physics:approximation value meaning "collide against the actual
# triangles". Valid for static colliders, which is all of the generated scene.
_EXACT_MESH = getattr(UsdPhysics.Tokens, "none", "none")


def _remove_env_clutter(stage):
    """Deactivate the GroundPlane and Environment xforms the base environment
    brings in. The scene generator lays its own ground, so these cause
    z-fighting and an unwanted visual backdrop — `Environment/Geometry` is the
    grid mesh that reads as a blue square under the spawn point.

    `default_environment.usd` has defaultPrim `/World`, which Pegasus references
    at `/World/stage`, so everything here composes from a referenced layer.
    `RemovePrim` cannot delete across a reference: it returns False rather than
    raising, so a try/except around it never fires and the prim stays visible.
    Deactivation does compose, and is reversible in the viewport.

    `SphereLight` is a sibling of these, not a child, so lighting survives.
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
                carb.log_info(f"[quadruped] deactivated {child.GetPath()}")
            else:
                UsdGeom.Imageable(child).MakeInvisible()
                carb.log_info(f"[quadruped] hid {child.GetPath()}")
    print(f"[quadruped] env clutter: {n} prim(s) deactivated")


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


def _use_exact_mesh_collision(stage, root_path: str) -> int:
    """Force exact triangle-mesh collision on every Mesh under *root_path*.

    A convex-hull collider is the wrong shape for almost everything in this
    scene. A streetlight hulls into a solid wedge spanning pole, arm and lamp
    head, so the robot walks into a barrier metres wide with nothing visible
    there. Sidewalk tiles are worse: _tile_rect stretches each tile per-axis to
    cover its step and inflates it by `tile_overlap` (1.02) so neighbours
    overlap rather than crack, which is fine to look at and terrible to stand
    on — the feet land on a lattice of interpenetrating lumpy hulls and the
    contact solver kicks the robot over. The road does not have this problem
    because apply_ground_planes writes it as one flat procedural Mesh.

    Exact meshes are static-only in PhysX, which is what everything here is:
    call this *after* settle_rigid_props, whose dynamic props legitimately need
    convex hulls while they are falling and are frozen static again afterwards.
    """
    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        return 0

    def still_dynamic(prim):
        p = prim
        while p.IsValid() and p.GetPath() != Sdf.Path.absoluteRootPath:
            if p.HasAPI(UsdPhysics.RigidBodyAPI):
                return True
            p = p.GetParent()
        return False

    n = skipped = 0
    for prim in Usd.PrimRange(root):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        # PhysX rejects triangle-mesh collision on a dynamic body. settle_rigid_props
        # strips RigidBodyAPI from every prop it froze, so nothing here should
        # still be dynamic — but leave anything that is on its convex hull
        # rather than authoring a collider PhysX will refuse.
        if still_dynamic(prim):
            skipped += 1
            continue
        mc = UsdPhysics.MeshCollisionAPI.Apply(prim)
        mc.CreateApproximationAttr().Set(_EXACT_MESH)
        n += 1
    print(f"[quadruped] exact mesh collision on {n} meshes under {root_path}"
          + (f" ({skipped} left convex — still dynamic)" if skipped else ""))
    return n


def _find_open_spawn(placements, want_xy, clearance_m,
                     search_m=60.0, step_m=1.5):
    """Nearest point to *want_xy* with at least *clearance_m* of open ground.

    Roads and lawns are procedural ground meshes written by apply_ground_planes,
    not placements — so "no placement nearby" means open asphalt or grass, the
    continuous flat surface the flat-terrain policy is happy on. Everything
    that is a placement (sidewalk and concrete tiles, buildings, debris, poles,
    cars) is something to stand clear of.
    """
    pts = np.array([[float(p["x_m"]), float(p["y_m"])] for p in placements
                    if p.get("x_m") is not None and p.get("y_m") is not None],
                   dtype=float)
    want = np.asarray(want_xy, dtype=float)
    if pts.size == 0:
        return want, float("inf")

    # Only placements that could possibly matter within the search window.
    near = pts[np.abs(pts - want).max(axis=1) <= search_m + clearance_m + 5.0]
    if near.size == 0:
        return want, float("inf")

    axis = np.arange(-search_m, search_m + step_m, step_m)
    gx, gy = np.meshgrid(want[0] + axis, want[1] + axis)
    cand = np.stack([gx.ravel(), gy.ravel()], axis=1)
    cand = cand[np.argsort(np.linalg.norm(cand - want, axis=1))]

    best_xy, best_clear = want, -1.0
    for i in range(0, len(cand), 128):
        chunk = cand[i:i + 128]
        d = np.sqrt(((chunk[:, None, :] - near[None, :, :]) ** 2)
                    .sum(-1)).min(axis=1)
        ok = np.nonzero(d >= clearance_m)[0]
        if ok.size:                      # candidates are distance-sorted
            return chunk[ok[0]], float(d[ok[0]])
        j = int(np.argmax(d))
        if d[j] > best_clear:
            best_xy, best_clear = chunk[j], float(d[j])
    return best_xy, best_clear


def _define_chase_camera(stage, prim_path, offset, rot_xyz, fov_deg):
    cam = UsdGeom.Camera.Define(stage, Sdf.Path(prim_path))
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 5000.0))
    cam.CreateHorizontalApertureAttr(CHASE_APERTURE)
    focal = CHASE_APERTURE / (2.0 * math.tan(math.radians(fov_deg) / 2.0))
    cam.CreateFocalLengthAttr(focal)
    xform = UsdGeom.Xformable(cam.GetPrim())
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in offset]))
    xform.AddRotateXYZOp().Set(Gf.Vec3f(*[float(v) for v in rot_xyz]))
    print(f"[quadruped] chase cam at {prim_path} offset={offset} "
          f"rot={rot_xyz} fov={fov_deg}deg (focal {focal:.2f} mm)")
    return cam


class QuadrupedSceneApp:

    # SPACE is Kit's play/pause and P is taken too, so U is the stop key.
    KEYMAP = {
        "I": [1.0, 0.0, 0.0],     # forward
        "K": [-1.0, 0.0, 0.0],    # back
        "J": [0.0, 0.0, 1.0],     # yaw left
        "L": [0.0, 0.0, -1.0],    # yaw right
    }
    STOP_KEY = "U"

    def __init__(self):
        if QUADRUPED not in ROBOTS:
            raise ValueError(f"unknown QUADRUPED {QUADRUPED!r}; "
                             f"expected one of {', '.join(sorted(ROBOTS))}")
        spec = ROBOTS[QUADRUPED]

        self.timeline = omni.timeline.get_timeline_interface()
        self.timeline.stop()

        self._world = World(stage_units_in_meters=1.0,
                            physics_dt=spec["physics_dt"],
                            rendering_dt=spec["rendering_dt"],
                            physics_prim_path="/World/PhysicsScene")
        pg = PegasusInterface()
        pg._world = self._world
        pg.load_environment(ENV_URL)

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("Stage failed to load")
        if not wait_for_stage(stage):
            carb.log_warn("Stage load timed out — continuing anyway.")

        _remove_env_clutter(stage)

        stage_prim = stage.GetPrimAtPath("/World/stage")
        if stage_prim.IsValid():
            scale_stage_prim(stage, "/World/stage", STAGE_SCALE)
            add_colliders(stage_prim)
            for _ in range(10):
                omni.kit.app.get_app().update()
        else:
            carb.log_warn("/World/stage not found — skipping scale and collision.")

        config = load_scene_config(SCENE_CONFIG)

        _, ssf = get_stage_meters_per_unit(stage)
        placements = generate_scene_on_stage(stage, config,
                                             parent_path="/World/stage/generated",
                                             scene_scale_factor=ssf)

        generated_prim = stage.GetPrimAtPath("/World/stage/generated")
        if generated_prim.IsValid():
            add_colliders(generated_prim)
        for _ in range(10):
            omni.kit.app.get_app().update()

        # Runs the timeline briefly, so it has to happen before the robot
        # exists — and before the collision fix-up, since the props need convex
        # hulls while they are dynamic.
        settle_rigid_props(
            stage,
            [p["prim_path"] for p in placements
             if p.get("settle") and p.get("prim_path")],
            ground_path="/World/stage/generated/ground",
        )
        _use_exact_mesh_collision(stage, "/World/stage/generated")
        for _ in range(5):
            omni.kit.app.get_app().update()

        if ASSET_ROOT:
            carb.settings.get_settings().set(
                "/persistent/isaac/asset_root/default", ASSET_ROOT)

        sky_root = (ASSET_ROOT or get_assets_root_path() or "").rstrip("/")
        add_dome_light(stage, DOME_LIGHT_PATH, DOME_LIGHT_INTENSITY,
                       DOME_LIGHT_EXPOSURE,
                       texture_file=DOME_LIGHT_TEXTURE.format(root=sky_root))

        spawn_xy, clear_m = _find_open_spawn(placements, SPAWN_XY,
                                             SPAWN_CLEARANCE)
        ground_z = self._ground_z(spawn_xy[0], spawn_xy[1])
        if ground_z is None:
            ground_z = 0.0
            carb.log_warn(f"[quadruped] no ground hit at {spawn_xy} — "
                          f"assuming z=0")
        spawn = np.array([spawn_xy[0], spawn_xy[1], ground_z + spec["stand_h"]])

        self._robot = spec["cls"](
            prim_path="/World/Quadruped",
            name=QUADRUPED,
            position=spawn,
        )

        # The physics rate the policy expects lives in its own env yaml, which
        # PolicyController already parsed into _dt / render_interval. Measured
        # for these two: Spot dt=0.002 (500 Hz, decimation 10), ANYmal dt=0.005
        # (200 Hz, decimation 4) — both 50 Hz control.
        pol_dt = float(self._robot._dt)
        render_interval = int(getattr(self._robot, "render_interval", 1) or 1)
        self._world.set_simulation_dt(physics_dt=pol_dt,
                                      rendering_dt=pol_dt * render_interval)

        self._cam_path = f"/World/Quadruped/{spec['body_link']}/ChaseCam"
        _define_chase_camera(stage, self._cam_path, CHASE_OFFSET, CHASE_ROT,
                             CHASE_FOV)

        self._base_command = np.zeros(3)
        self._held = set()
        self._steps = 0
        self.first_step = True
        self.needs_reset = False

        print("\n" + "=" * 70)
        print(f"QUADRUPED: {QUADRUPED}   spawn={spawn.round(2).tolist()}")
        print(f"  requested xy {SPAWN_XY} -> open ground at "
              f"{np.round(spawn_xy, 2).tolist()} ({clear_m:.1f} m clear)")
        print(f"  physics {1.0 / self._world.get_physics_dt():.0f} Hz  "
              f"control {1.0 / (pol_dt * self._robot._decimation):.0f} Hz")
        print("Click the viewport, then drive:")
        print("  I forward   K back   J yaw left   L yaw right   U stop")
        print("=" * 70 + "\n")

    def _ground_z(self, x_m, y_m):
        snap = _make_physx_ground_snap()
        return snap(x_m, y_m) if snap else None

    def setup(self):
        self._appwindow = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._keyboard = self._appwindow.get_keyboard()
        self._sub_keyboard = self._input.subscribe_to_keyboard_events(
            self._keyboard, self._on_keyboard_event)
        self._world.add_physics_callback("quadruped_forward",
                                         callback_fn=self.on_physics_step)
        set_active_viewport_camera(self._cam_path)

    def on_physics_step(self, step_size):
        if self.first_step:
            self._robot.initialize()
            self.first_step = False
            self._steps = 0
            return
        if self.needs_reset:
            self._world.reset(True)
            self.needs_reset = False
            self.first_step = True
            self._held.clear()
            self._base_command = np.zeros(3)
            return
        self._steps += 1
        cmd = (np.zeros(3) if self._steps < WARMUP_STEPS
               else self._base_command)
        self._robot.forward(step_size, cmd)

    def _on_keyboard_event(self, event, *args, **kwargs):
        # carb.input.KeyboardEvent.input is typed `object` and arrives as a
        # plain str in this Kit build, not the KeyboardInput enum the stock
        # examples assume — event.input.name raises on every single event.
        raw = event.input
        name = getattr(raw, "name", None) or str(raw).rsplit(".", 1)[-1]
        name = name.upper()
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            if name == self.STOP_KEY:
                self._held.clear()
            elif name in self.KEYMAP:
                self._held.add(name)
        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            self._held.discard(name)
        else:
            return True   # KEY_REPEAT and friends change nothing
        # Recomputed from what is held, so a dropped release cannot accumulate.
        cmd = np.zeros(3)
        for key in self._held:
            cmd += np.array(self.KEYMAP[key])
        self._base_command = cmd
        return True

    def run(self):
        while simulation_app.is_running():
            self._world.step(render=True)
            if self._world.is_stopped():
                self.needs_reset = True
        self.timeline.stop()
        simulation_app.close()


def main():
    app = QuadrupedSceneApp()
    simulation_app.update()
    app._world.reset()
    simulation_app.update()
    app.setup()
    simulation_app.update()
    app.run()


if __name__ == "__main__":
    main()
