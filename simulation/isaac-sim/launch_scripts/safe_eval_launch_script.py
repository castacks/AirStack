#!/usr/bin/env python
"""
S.A.F.E. evaluation launcher: AirStack's standard single-drone PX4 bring-up
plus an in-process ScenarioManager that lets an external harness drive
benchmark episodes over ROS 2 — no changes to the autonomy stack.

This is example_one_px4_pegasus_launch_script.py (same drone, same ZED rig,
same PX4 SITL wiring, so the full AirStack perception→planning→control chain
runs unmodified against it) with three additions, all inside this process:

 1. ScenarioManager — a JSON-lines TCP server (port 8899 on the docker
    network; rclpy cannot host a node inside the Kit interpreter, so this
    speaks plain TCP the same way PX4 SITL's MAVLink link does) with
    commands to spawn/teleport obstacles, reset the vehicle, and query info.
 2. Ground-truth state stream — pushed to every connected client as JSON
    lines: sim time, drone world pose/velocity, all obstacle positions, and
    a latched analytic collision flag. The eval harness paces episodes and
    scores safety metrics from this stream.
 3. Obstacle field — kinematic cylinder prims (static pillars + waypoint-
    walking "pedestrians") teleported each sim step. They are plain render
    geometry, so the drone's ZED / lidar genuinely sees them.

Config arrives as JSON in SAFE_EVAL_CONFIG (set by eval/stack/compose.py):
    env_url            Nucleus-relative USD path, SIMULATION_ENVIRONMENTS key,
                       full omniverse:// URL, or null for the default grid.
    stage_scale        scale for /World/stage (Nucleus cm assets → 0.01)
    spawn              [x, y] drone spawn (z fixed at 0.07)
    max_dynamic        obstacle prim pool size (actives set per scenario)
    max_static         static cylinder prim pool size
    obstacle_radius    cylinder radius (m)
    drone_radius       collision radius of the drone body (m)
    arena              [x_min, y_min, x_max, y_max] wall bounds for collision

Selected via ISAAC_SIM_SCRIPT_NAME=safe_eval_launch_script.py at airstack up.
"""

import carb
from isaacsim import SimulationApp

import json
import os

_headless = os.environ.get("ISAAC_SIM_HEADLESS", "false").lower() == "true"
simulation_app = SimulationApp({"headless": _headless})

import queue
import math
import socketserver
import sys
import threading

# Docker mounts the SAFE checkout read-only. World and human modules are imported
# from that single source, never vendored into AirStack.
_safe_source = "/isaac-sim/SAFE_BENCHMARK"
if os.path.isfile(os.path.join(_safe_source, "safe_isaac", "world_builder.py")):
    sys.path.insert(0, _safe_source)
sys.path.insert(0, "/isaac-sim/AirStack")
import time

import numpy as np

import omni.graph.core as og
import omni.kit.app
import omni.timeline
import omni.usd

from pxr import Gf, Sdf, UsdGeom, UsdShade

from omni.isaac.core.world import World

from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.ogn.api.spawn_multirotor import spawn_px4_multirotor_node
from pegasus.simulator.ogn.api.spawn_zed_camera import add_zed_stereo_camera_subgraph

sys.path.insert(0, os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "utils")))
from scene_prep import scale_stage_prim, add_colliders, add_dome_light

DRONE_USD = "~/.local/share/ov/data/documents/Kit/shared/exts/pegasus.simulator/pegasus/simulator/assets/Robots/Iris/iris.usd"
ROBOT_NAME = "robot_1"
DRONE_PRIM = "/World/base_link"
ROS_DOMAIN = 1  # matches robot_1's resolved ROS_DOMAIN_ID in the robot container

SERVER_PORT = 8899
STATE_HZ = float(os.environ.get("SAFE_EVAL_STATE_HZ") or "10.0")
RENDER_DT = 1.0 / 60.0
PHYSICS_DT = RENDER_DT / 4
# Sim times closer than this are the same tick; accumulated float sums differ in the last bits.
TICK_TOLERANCE_S = PHYSICS_DT / 2

_DEFAULT_CONFIG = {
    "env_url": None,
    "stage_scale": 1.0,
    "spawn": [0.0, 0.0],
    "max_dynamic": 30,
    "max_static": 12,
    "obstacle_radius": 0.3,
    "drone_radius": 0.31,
    "arena": [-10.0, -10.0, 10.0, 10.0],
    "safe_domain": None,
    "humans": False,
    "static_only": False,
    "goal_distance": None,
    "spawn_z": 0.07,
}


def load_config() -> dict:
    cfg = dict(_DEFAULT_CONFIG)
    raw = os.environ.get("SAFE_EVAL_CONFIG", "")
    if raw:
        try:
            cfg.update(json.loads(raw))
        except (ValueError, TypeError) as e:
            carb.log_error(f"[safe_eval] bad SAFE_EVAL_CONFIG ({e}) — using defaults")
    return cfg


CONFIG = load_config()

ext_manager = omni.kit.app.get_app().get_extension_manager()
for ext in [
    "omni.graph.core",
    "omni.graph.action",
    "omni.graph.action_nodes",
    "isaacsim.core.nodes",
    "omni.graph.ui",
    "omni.graph.visualization.nodes",
    "omni.graph.scriptnode",
    "omni.graph.window.action",
    "omni.graph.window.generic",
    "omni.graph.ui_nodes",
    "pegasus.simulator",
]:
    if not ext_manager.is_extension_enabled(ext):
        ext_manager.set_extension_enabled_immediate(ext, True)


def resolve_env_url(env_url):
    if not env_url:
        return SIMULATION_ENVIRONMENTS["Default Environment"]
    if env_url in SIMULATION_ENVIRONMENTS:
        return SIMULATION_ENVIRONMENTS[env_url]
    if env_url.startswith("omniverse://") or env_url.startswith("http"):
        return env_url
    # Nucleus-relative path like /Isaac/Environments/Simple_Warehouse/full_warehouse.usd
    from isaacsim.storage.native import nucleus
    root = nucleus.get_assets_root_path()
    if root is None:
        carb.log_error("[safe_eval] no Nucleus assets root — falling back to default grid")
        return SIMULATION_ENVIRONMENTS["Default Environment"]
    return root + env_url


def wait_for_stage(stage, timeout_s: float = 10.0):
    for _ in range(int(timeout_s / 0.1)):
        omni.kit.app.get_app().update()
        world_prim = stage.GetPrimAtPath("/World")
        if world_prim.IsValid():
            non_physics = [c for c in world_prim.GetChildren() if c.GetName() != "PhysicsScene"]
            if non_physics:
                return True
        time.sleep(0.1)
    return False


def _write_noise_texture(path: str, size: int = 512, seed: int = 7) -> None:
    """Multi-scale grey noise PNG: dense stereo texture at every viewing distance."""
    import zlib
    import struct
    rng = np.random.RandomState(seed)
    img = np.zeros((size, size), dtype=np.float64)
    for cells in (8, 32, 128, 512):
        coarse = rng.uniform(0.0, 1.0, (cells, cells))
        img += np.kron(coarse, np.ones((size // cells, size // cells)))
    img = (img - img.min()) / (img.max() - img.min())
    px = (40 + 175 * img).astype(np.uint8)
    raw = b"".join(b"\x00" + px[y].tobytes() for y in range(size))

    def chunk(tag, data):
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xffffffff)
    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n"
                + chunk(b"IHDR", struct.pack(">IIBBBBB", size, size, 8, 0, 0, 0, 0))
                + chunk(b"IDAT", zlib.compress(raw))
                + chunk(b"IEND", b""))


def _textured_quad(stage, path: str, corners, material, repeat_u: float, repeat_v: float) -> None:
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr([Gf.Vec3f(*c) for c in corners])
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateDoubleSidedAttr(True)
    st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying)
    st.Set([Gf.Vec2f(0, 0), Gf.Vec2f(repeat_u, 0), Gf.Vec2f(repeat_u, repeat_v), Gf.Vec2f(0, repeat_v)])
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)


def build_textured_arena(stage, arena, height: float) -> None:
    """Floor and four walls at the arena boundary with a noise texture, for scenes
    with no USD environment. The default grid is textureless, so stereo planners
    see nothing and never certify a free trajectory."""
    tex = "/tmp/safe_eval_noise.png"
    _write_noise_texture(tex)
    x0, y0, x1, y1 = [float(v) for v in arena]
    root = "/World/safe_arena"
    UsdGeom.Xform.Define(stage, root)
    material = UsdShade.Material.Define(stage, root + "/noise_mat")
    shader = UsdShade.Shader.Define(stage, root + "/noise_mat/surface")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.9)
    reader = UsdShade.Shader.Define(stage, root + "/noise_mat/st_reader")
    reader.CreateIdAttr("UsdPrimvarReader_float2")
    reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    texture = UsdShade.Shader.Define(stage, root + "/noise_mat/texture")
    texture.CreateIdAttr("UsdUVTexture")
    texture.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(tex)
    texture.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
    texture.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
    texture.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
        reader.CreateOutput("result", Sdf.ValueTypeNames.Float2))
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
        texture.CreateOutput("rgb", Sdf.ValueTypeNames.Float3))
    material.CreateSurfaceOutput().ConnectToSource(shader.CreateOutput("surface", Sdf.ValueTypeNames.Token))

    # 1 m per texture tile: features from a few cm to a metre.
    z = 0.01
    _textured_quad(stage, root + "/floor",
                   [(x0, y0, z), (x1, y0, z), (x1, y1, z), (x0, y1, z)],
                   material, x1 - x0, y1 - y0)
    walls = {
        "wall_south": ([(x0, y0, 0), (x1, y0, 0), (x1, y0, height), (x0, y0, height)], x1 - x0),
        "wall_north": ([(x1, y1, 0), (x0, y1, 0), (x0, y1, height), (x1, y1, height)], x1 - x0),
        "wall_west":  ([(x0, y1, 0), (x0, y0, 0), (x0, y0, height), (x0, y1, height)], y1 - y0),
        "wall_east":  ([(x1, y0, 0), (x1, y1, 0), (x1, y1, height), (x1, y0, height)], y1 - y0),
    }
    for name, (corners, length) in walls.items():
        _textured_quad(stage, f"{root}/{name}", corners, material, length, height)


class ObstaclePool:
    """Pre-spawned kinematic cylinder prims, teleported per step.

    Two pools: 'static' pillars (full flight-height cylinders) and 'dynamic'
    pedestrian stand-ins. Inactive prims park underground at z=-100 so a
    scenario can use any count up to the pool size without stage edits.
    """

    PARKED_Z = -100.0

    def __init__(self, stage, radius: float):
        self.stage = stage
        self.radius = radius
        self.static_ops = []
        self.dynamic_ops = []

    def _spawn_cylinder(self, path: str, height: float, color) -> "UsdGeom.XformOp":
        xform_prim = self.stage.DefinePrim(path, "Xform")
        xform = UsdGeom.Xformable(xform_prim)
        xform.ClearXformOpOrder()
        translate_op = xform.AddTranslateOp()
        translate_op.Set(Gf.Vec3d(0.0, 0.0, self.PARKED_Z))
        xform.SetXformOpOrder([translate_op])

        cyl = UsdGeom.Cylinder.Define(self.stage, path + "/geom")
        cyl.GetRadiusAttr().Set(self.radius)
        cyl.GetHeightAttr().Set(height)
        cyl.GetAxisAttr().Set("Z")
        # Cylinder prim is centered on its origin — lift geometry so the base
        # sits on the floor when the Xform z is 0.
        UsdGeom.Xformable(cyl.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, height / 2.0))
        cyl.CreateDisplayColorAttr([color])
        return translate_op

    def build(self, max_static: int, max_dynamic: int):
        for i in range(max_static):
            op = self._spawn_cylinder(f"/World/safe_eval/static_{i}", height=3.0,
                                      color=Gf.Vec3f(0.35, 0.35, 0.4))
            self.static_ops.append(op)
        for i in range(max_dynamic):
            op = self._spawn_cylinder(f"/World/safe_eval/dynamic_{i}", height=1.8,
                                      color=Gf.Vec3f(0.85, 0.35, 0.1))
            self.dynamic_ops.append(op)

    @staticmethod
    def _place(ops, positions):
        for i, op in enumerate(ops):
            if i < len(positions):
                x, y = float(positions[i][0]), float(positions[i][1])
                op.Set(Gf.Vec3d(x, y, 0.0))
            else:
                op.Set(Gf.Vec3d(0.0, 0.0, ObstaclePool.PARKED_Z))



class ScenarioManager:
    """Episode control + ground-truth state stream over a JSON-lines TCP
    server on the docker network.

    Reader threads enqueue client commands; update() (called once per
    world.step, on the main thread) executes them against the stage, sends
    responses, steps the obstacle field, and broadcasts the state line to
    every connected client.
    """

    def __init__(self, stage, pool: ObstaclePool, cfg: dict, pegasus: PegasusInterface,
                 humans=None, safe_cfg=None, safe_static_prims=None):
        self._clients: list = []
        self._clients_lock = threading.Lock()
        self._inbox: "queue.Queue" = queue.Queue()
        self._server = socketserver.ThreadingTCPServer(
            ("0.0.0.0", SERVER_PORT), self._make_handler())
        self._server.daemon_threads = True
        threading.Thread(target=self._server.serve_forever, daemon=True).start()

        self.stage = stage
        self.pool = pool
        self.humans = humans
        self.safe_cfg = safe_cfg
        self.safe_static_prims = safe_static_prims or []
        self.cfg = cfg
        # Pegasus interface: its vehicle_manager holds the PX4 backend restarted after a teleport.
        self.pegasus = pegasus
        self.arena = [float(v) for v in cfg["arena"]]
        self.obstacle_radius = float(cfg["obstacle_radius"])
        self.drone_radius = float(cfg["drone_radius"])
        # The drone's collider box, measured from the stage on first use, and its latest clearance.
        self._box = None
        self.body_clearance = float("inf")
        self._measured = None

        self._drone_xform = None  # created lazily once the drone prim exists
        self._last_drone_pos = None
        self._last_sim_time = None
        self._drone_vel = np.zeros(3)

        # Scenario state (set by set_scenario)
        self.static_positions = np.zeros((0, 3), dtype=np.float32)
        self.static_radii = np.zeros(0, dtype=np.float32)
        self.dyn_positions = np.zeros((0, 3), dtype=np.float32)
        self.dyn_velocities = np.zeros((0, 3), dtype=np.float32)
        self.dyn_waypoints = np.zeros((0, 3), dtype=np.float32)
        self.n_active_dyn = 0
        self.target = None
        self._rng = np.random.RandomState(0)

        self.collision = False
        self.collision_with = ""
        self._last_state_pub = -1e9
        self._pending_vehicle_reset = None
        # Ticks the host has released while it steps the sim; None while the sim free-runs.
        self._ticks_left: int | None = None
        self._last_scenario_args: dict | None = None

    def holding(self) -> bool:
        """Whether the host is stepping the sim and has no tick released."""
        return self._ticks_left == 0

    def tick_done(self) -> None:
        if self._ticks_left:
            self._ticks_left -= 1

    # ── TCP plumbing ─────────────────────────────────────────────────────

    def _make_handler(self):
        manager = self

        class Handler(socketserver.StreamRequestHandler):
            def handle(self):
                with manager._clients_lock:
                    manager._clients.append(self.wfile)
                try:
                    for line in self.rfile:
                        line = line.strip()
                        if not line:
                            continue
                        try:
                            manager._inbox.put((self.wfile, json.loads(line)))
                        except (ValueError, TypeError):
                            pass
                finally:
                    with manager._clients_lock:
                        if self.wfile in manager._clients:
                            manager._clients.remove(self.wfile)

        return Handler

    @staticmethod
    def _send_line(wfile, obj: dict) -> bool:
        try:
            wfile.write((json.dumps(obj) + "\n").encode())
            wfile.flush()
            return True
        except (OSError, ValueError):
            return False

    def _broadcast(self, obj: dict) -> None:
        with self._clients_lock:
            clients = list(self._clients)
        dead = [w for w in clients if not self._send_line(w, obj)]
        if dead:
            with self._clients_lock:
                for w in dead:
                    if w in self._clients:
                        self._clients.remove(w)

    def _process_commands(self) -> None:
        while True:
            try:
                wfile, req = self._inbox.get_nowait()
            except queue.Empty:
                return
            cmd = req.get("cmd", "")
            cmd_id = req.get("id", "")
            try:
                data = self._dispatch(cmd, req.get("args") or {})
                resp = {"kind": "response", "id": cmd_id, "cmd": cmd, "ok": True,
                        "data": data}
            except Exception as e:  # report failure to harness, keep sim alive
                import traceback
                resp = {"kind": "response", "id": cmd_id, "cmd": cmd, "ok": False,
                        "error": f"{e}\n{traceback.format_exc(limit=-4)}"}
            self._send_line(wfile, resp)

    def _dispatch(self, cmd, args):
        if cmd == "ping":
            return {"sim_time": self._sim_time()}
        if cmd == "get_info":
            return {
                "arena": self.arena,
                "obstacle_radius": self.obstacle_radius,
                "drone_radius": self.drone_radius,
                "max_static": (len(self.safe_static_prims) if self.safe_cfg is not None
                               else len(self.pool.static_ops)),
                "max_dynamic": self.humans.n if self.humans is not None else len(self.pool.dynamic_ops),
                "safe_domain": self.cfg.get("safe_domain"),
                "domain_info": self.cfg.get("domain_info"),
            }
        if cmd == "set_scenario":
            return self._set_scenario(args)
        if cmd == "restart_movers":
            # Episodes start with the movers at the scenario's sampled poses, not wherever
            # they walked during takeoff and the run-up.
            if self._last_scenario_args is None:
                raise ValueError("restart_movers needs a scenario set first")
            return self._set_scenario(self._last_scenario_args)
        if cmd == "generate_safe_scenario":
            return self._generate_safe_scenario(args)
        if cmd == "reset_vehicle":
            # Defer to the main loop so the teleport happens between physics
            # steps, not inside a ROS callback.
            self._pending_vehicle_reset = {
                "pos": [float(v) for v in args.get("pos", [0.0, 0.0, 0.07])],
                "yaw": float(args.get("yaw", 0.0)),
            }
            return {}
        if cmd == "clear_collision":
            self.collision = False
            self.collision_with = ""
            return {}
        if cmd == "set_stepping":
            # On, the sim holds after this tick until the host advances it; off, it free-runs.
            self._ticks_left = 0 if args.get("on") else None
            return {"t": self._sim_time()}
        if cmd == "advance":
            if self._ticks_left is None:
                raise ValueError("advance needs set_stepping on")
            self._ticks_left += int(args.get("ticks", 1))
            return {}
        raise ValueError(f"unknown cmd '{cmd}'")

    def _generate_safe_scenario(self, args):
        if self.safe_cfg is None:
            raise RuntimeError('generate_safe_scenario requires a SAFE source domain')
        from eval.sim.safe_source_scene import sample_scenario
        layout, behavior, difficulty = sample_scenario(
            self.safe_cfg,
            seed=int(args['seed']),
            goal_distance=args.get('goal_distance'),
            perturbation_seed=args.get('perturbation_seed'),
            humans=self.humans is not None,
            agent_radius=float(self.cfg['drone_radius']),
            obstacle_radius=float(self.cfg['obstacle_radius']),
            scenario_index=args.get('scenario_index'),
        )
        if self.humans is not None:
            self.humans.set_behavior(behavior)
        from safe_isaac.world_builder import (
            rebuild_static_obstacle_shapes, spawn_tree_obstacles,
            rebuild_blocker_wall_gap)
        static_radii = None
        if layout.get('static_obstacle_shapes') is not None:
            rebuild_static_obstacle_shapes(
                layout['static_obstacle_shapes'], env_prefix='safe_eval')
            static_radii = layout['static_obstacle_shapes']['radii']
        if layout.get('tree_variants') is not None:
            static_radii, _ = spawn_tree_obstacles(
                layout['tree_variants'], self.safe_cfg, env_prefix='safe_eval')
        if layout.get('wall_gap') is not None:
            rebuild_blocker_wall_gap(
                layout['wall_gap'], self.safe_cfg.size_y, env_prefix='safe_eval')

        def world_positions(value, *, ground=False):
            arr = np.asarray(value, dtype=np.float32).copy().reshape(-1, 3)
            if arr.size:
                arr[:, [0, 1]] = arr[:, [1, 0]]
                if not ground:
                    arr[:, 2] += float(self.safe_cfg.origin_z)
            return arr.tolist()

        start = world_positions([layout['agent_location']])[0]
        target = world_positions([layout['target_location']])[0]
        static = world_positions(layout['static_cylinder_locations'], ground=True)
        dynamic = world_positions(layout['dynamic_obstacle_locations'])
        velocities = world_positions(layout['dynamic_obstacle_velocities'], ground=True)
        waypoints = world_positions(layout['dynamic_obstacle_waypoints'], ground=True)
        placed = self._set_scenario({
            'static': static, 'dynamic': dynamic,
            'dynamic_vel': velocities, 'waypoints': waypoints,
            'target': target, 'seed': int(args['seed']),
            'start': start,
            'static_radii': (np.asarray(static_radii).tolist()
                             if static_radii is not None else None),
        })
        return {
            'start': start, 'target': target,
            'static_cylinder_locations': static,
            'static_radii': placed['static_radii'],
            'dynamic_radius': placed['dynamic_radius'],
            'static_obstacle_locations': world_positions(layout['static_obstacle_locations']),
            'dynamic_obstacle_locations': dynamic,
            'dynamic_obstacle_velocities': velocities,
            'dynamic_obstacle_waypoints': waypoints,
            # The sampler moves origin_z to the floor of the window it drew on a photoreal
            # stage, so the host teleports the drone onto this scenario's own floor.
            'ground_z': float(self.safe_cfg.origin_z),
            'scenario_difficulty': difficulty,
        }

    def _set_scenario(self, args):
        self._last_scenario_args = dict(args)
        static =np.asarray(args.get("static", []), dtype=np.float32).reshape(-1, 3) \
            if args.get("static") else np.zeros((0, 3), dtype=np.float32)
        dyn = np.asarray(args.get("dynamic", []), dtype=np.float32).reshape(-1, 3) \
            if args.get("dynamic") else np.zeros((0, 3), dtype=np.float32)
        vel = np.asarray(args.get("dynamic_vel", []), dtype=np.float32).reshape(-1, 3) \
            if args.get("dynamic_vel") else np.zeros_like(dyn)
        wpts = np.asarray(args.get("waypoints", []), dtype=np.float32).reshape(-1, 3) \
            if args.get("waypoints") else dyn.copy()

        max_static = (len(self.safe_static_prims) if self.safe_cfg is not None
                      else len(self.pool.static_ops))
        if len(static) > max_static:
            raise ValueError(f"{len(static)} static > pool {max_static}")
        max_dynamic = (self.humans.n if self.humans is not None
                       else len(self.pool.dynamic_ops))
        if len(dyn) > max_dynamic:
            raise ValueError(f"{len(dyn)} dynamic > pool {max_dynamic}")

        self.static_positions = static
        radii = args.get('static_radii')
        self.static_radii = (np.asarray(radii, dtype=np.float32)
                             if radii is not None else
                             np.full(len(static), self.obstacle_radius, dtype=np.float32))
        self.dyn_positions = dyn
        self.dyn_velocities = vel
        self.dyn_waypoints = wpts
        self.n_active_dyn = len(dyn)
        self.target = [float(v) for v in args["target"]] if args.get("target") else None
        self._rng = np.random.RandomState(int(args.get("seed", 0)))

        if self.safe_cfg is not None:
            gm = getattr(self.safe_cfg, '_ground_map', None)
            for i, prim in enumerate(self.safe_static_prims):
                if i < len(static):
                    pos = static[i].astype(np.float32).copy()
                    # The layout's z is the base height above the ground, which lifts
                    # floating obstacles off it, as SAFE's _to_isaac_on_ground keeps it.
                    ground = gm.ground_at(float(pos[0]), float(pos[1])) if gm else None
                    pos[2] += (float(ground) if ground is not None
                               else float(self.safe_cfg.origin_z))
                else:
                    pos = np.array([0.0, 0.0, -100.0], dtype=np.float32)
                prim.set_world_poses(positions=pos.reshape(1, 3))
        else:
            self.pool._place(self.pool.static_ops, static)
        if self.humans is not None:
            # SAFE's character controller accepts benchmark XY (Isaac Y/X).
            bench_pos, bench_vel = dyn.copy(), vel.copy()
            bench_pos[:, [0, 1]] = bench_pos[:, [1, 0]]
            bench_vel[:, [0, 1]] = bench_vel[:, [1, 0]]
            corridor = None
            if args.get('start') and args.get('target'):
                start_b = np.asarray(args['start'], dtype=np.float32).copy()
                target_b = np.asarray(args['target'], dtype=np.float32).copy()
                start_b[[0, 1]] = start_b[[1, 0]]
                target_b[[0, 1]] = target_b[[1, 0]]
                corridor = (start_b, target_b)
            self.humans.reset_episode(
                bench_pos, bench_vel,
                bounds=(self.cfg['domain_info']['origin_x'],
                        self.cfg['domain_info']['origin_y'],
                        self.cfg['domain_info']['size_x'],
                        self.cfg['domain_info']['size_y']),
                corridor=corridor)
            static_bench = static.copy()
            static_bench[:, [0, 1]] = static_bench[:, [1, 0]]
            self.humans.set_static_obstacles(static_bench)
        else:
            self.pool._place(self.pool.dynamic_ops, dyn)
        self.collision = False
        self.collision_with = ""
        return {"n_static": len(static), "n_dynamic": len(dyn),
                "static_radii": self.static_radii.tolist(),
                "dynamic_radius": self._dynamic_radius()}

    # ── per-step update ──────────────────────────────────────────────────

    def _sim_time(self):
        world = World.instance()
        return float(world.current_time) if world is not None else 0.0

    def _vehicle(self):
        try:
            return next(iter(self.pegasus.vehicle_manager.vehicles.values()))
        except (AttributeError, StopIteration):
            return None

    def _drone_world_pose(self):
        """Ground truth from the Pegasus vehicle state, which reads the PhysX body.
        The USD xform is only a fallback before the vehicle exists: physics does
        not write back to it, so it freezes at the last teleport."""
        vehicle = self._vehicle()
        if vehicle is not None:
            s = vehicle.state
            q = s.attitude  # [qx, qy, qz, qw]
            return (np.asarray(s.position, dtype=np.float64),
                    [float(q[3]), float(q[0]), float(q[1]), float(q[2])],
                    np.asarray(s.linear_velocity, dtype=np.float64))
        if self._drone_xform is None:
            prim = self.stage.GetPrimAtPath(DRONE_PRIM)
            if not prim.IsValid():
                return None, None, None
            self._drone_xform = UsdGeom.Xformable(prim)
        mat = self._drone_xform.ComputeLocalToWorldTransform(0)
        pos = mat.ExtractTranslation()
        quat = mat.ExtractRotationQuat()
        im = quat.GetImaginary()
        return (np.array([pos[0], pos[1], pos[2]], dtype=np.float64),
                [float(quat.GetReal()), float(im[0]), float(im[1]), float(im[2])],
                None)

    def _apply_pending_vehicle_reset(self):
        if self._pending_vehicle_reset is None:
            return
        req = self._pending_vehicle_reset
        self._pending_vehicle_reset = None
        vehicle = self._vehicle()
        if vehicle is None:
            carb.log_warn("[safe_eval] reset_vehicle: no Pegasus vehicle yet")
            return
        try:
            from omni.isaac.dynamic_control import _dynamic_control
            dc = vehicle.get_dc_interface()
            body = dc.get_rigid_body(vehicle._stage_prefix + "/body")
            yaw = req["yaw"]
            pose = _dynamic_control.Transform(
                [float(v) for v in req["pos"]],
                [0.0, 0.0, float(np.sin(yaw / 2.0)), float(np.cos(yaw / 2.0))])
            dc.set_rigid_body_pose(body, pose)
            dc.set_rigid_body_linear_velocity(body, [0.0, 0.0, 0.0])
            dc.set_rigid_body_angular_velocity(body, [0.0, 0.0, 0.0])
        except Exception as e:
            carb.log_error(f"[safe_eval] reset_vehicle failed: {e}")
            return
        self._restart_px4_for_fresh_ekf()

    def _restart_px4_for_fresh_ekf(self):
        """Relaunch PX4 after the teleport so the EKF re-inits at the new position.
        A position teleport poisons the estimator and PX4 then denies the arm."""
        backend = None
        try:
            for v in self.pegasus.vehicle_manager.vehicles.values():
                for b in getattr(v, "_backends", []):
                    if type(b).__name__ == "PX4MavlinkBackend":
                        backend = b
                        break
        except Exception as e:
            carb.log_warn(f"[safe_eval] could not reach PX4 backend: {e}")
        if backend is None:
            carb.log_warn("[safe_eval] no PX4MavlinkBackend to restart — arm may fail")
            return
        # Boot checked that Pegasus autolaunches PX4, so this backend owns the PX4 process.
        try:
            backend.stop()
        except Exception:
            pass
        # Pegasus stop() kills only the top px4; sweep the rcS/param children still
        # holding the HIL sockets, or the relaunch blocks.
        import subprocess
        subprocess.run(["pkill", "-9", "-f", "PX4-Autopilot"], capture_output=True)
        time.sleep(0.5)
        try:
            backend._received_first_hearbeat = False
            backend.start()
        except Exception as e:
            carb.log_error(f"[safe_eval] PX4 relaunch failed: {e}")
            return
        app = omni.kit.app.get_app()
        for _ in range(8000):
            app.update()
            if getattr(backend, "_received_first_hearbeat", False):
                carb.log_info("[safe_eval] PX4 relaunched — fresh EKF at new position")
                return
        carb.log_warn("[safe_eval] PX4 HIL did not reconnect after relaunch")

    def _step_dynamic_obstacles(self, dt: float):
        if self.humans is not None and self.n_active_dyn and dt > 0.0:
            self.humans.step(dt)
            pos, vel = self.humans.get_positions_velocities_isaac(dt)
            self.dyn_positions = pos[:self.n_active_dyn]
            self.dyn_velocities = vel[:self.n_active_dyn]
            return
        if self.n_active_dyn == 0 or dt <= 0.0:
            return
        x_min, y_min, x_max, y_max = self.arena
        # waypoint walk: mirrors eval/sim/scenario_config.update_dynamic_obstacles
        r = self.obstacle_radius
        arrival = max(r * 2.0, 1.0)
        for i in range(self.n_active_dyn):
            pos = self.dyn_positions[i]
            diff = self.dyn_waypoints[i, :2] - pos[:2]
            dist = float(np.linalg.norm(diff))
            if dist < arrival:
                self.dyn_waypoints[i, 0] = self._rng.uniform(x_min + r, x_max - r)
                self.dyn_waypoints[i, 1] = self._rng.uniform(y_min + r, y_max - r)
                diff = self.dyn_waypoints[i, :2] - pos[:2]
                dist = float(np.linalg.norm(diff))
            speed = float(np.linalg.norm(self.dyn_velocities[i, :2]))
            if speed < 1e-8:
                speed = 0.05
            if dist > 1e-6:
                self.dyn_velocities[i, 0] = diff[0] / dist * speed
                self.dyn_velocities[i, 1] = diff[1] / dist * speed
            self.dyn_positions[i, :2] = np.clip(
                pos[:2] + self.dyn_velocities[i, :2] * dt,
                [x_min + r, y_min + r], [x_max - r, y_max - r],
            )
        self.pool._place(self.pool.dynamic_ops, self.dyn_positions[: self.n_active_dyn])

    def _dynamic_radius(self) -> float:
        """Collision radius of every moving obstacle: the walker capsule with people, else the
        pool cylinder."""
        if self.humans is not None:
            return float(self.humans.CAPSULE_RADIUS)
        return float(self.obstacle_radius)

    def _dynamic_targets(self) -> np.ndarray:
        """The waypoint each active mover is walking toward, in Isaac world XY at the mover's height."""
        targets = np.asarray(self.dyn_positions[: self.n_active_dyn], dtype=np.float32).copy()
        if self.humans is not None:
            bench = self.humans.get_waypoints_benchmark()[: len(targets)]
            targets[:, 0], targets[:, 1] = bench[:, 1], bench[:, 0]
        else:
            targets[:, :2] = self.dyn_waypoints[: len(targets), :2]
        return targets

    # Hits that are never obstacles, as SAFE's PhysX probe skips them: ground and terrain, and
    # synthetic boundary leaves. People's capsules are kept, since the probe is the drone's real box.
    _BOUNDARY = ("floor", "ground", "plane", "terrain", "road", "pavement", "room_bound")
    _BOUNDARY_LEAF_NAMES = ("wall_n", "wall_s", "wall_e", "wall_w", "ceiling")
    # How far beyond the drone's surface the clearance search looks, and its resolution.
    CLEARANCE_TOLERANCE_M = 0.02

    def _drone_box(self):
        """(centre, half extents) of the box around the drone's collision shapes in its body frame,
        read from the stage once. A rotor's shapes count as the disk they sweep about its z axis."""
        if self._box is None:
            from pxr import Usd, UsdPhysics
            body = self.stage.GetPrimAtPath(f"{DRONE_PRIM}/body")
            if not body.IsValid():
                raise RuntimeError(f"no drone body prim at {DRONE_PRIM}/body to measure the collider box from")
            purposes = [UsdGeom.Tokens.default_, UsdGeom.Tokens.proxy, UsdGeom.Tokens.guide,
                        UsdGeom.Tokens.render]
            time = Usd.TimeCode.Default()
            cache = UsdGeom.BBoxCache(time, purposes)
            to_body = UsdGeom.Xformable(body).ComputeLocalToWorldTransform(time).GetInverse()
            signs = [(sx, sy, sz) for sx in (0, 1) for sy in (0, 1) for sz in (0, 1)]
            points = []
            for prim in Usd.PrimRange(self.stage.GetPrimAtPath(DRONE_PRIM)):
                if not (prim.HasAPI(UsdPhysics.CollisionAPI) and prim.IsA(UsdGeom.Gprim)):
                    continue
                owner = prim
                while owner.IsValid() and not owner.HasAPI(UsdPhysics.RigidBodyAPI):
                    owner = owner.GetParent()
                if not owner.IsValid():
                    continue
                r = cache.ComputeRelativeBound(prim, owner).ComputeAlignedRange()
                lo, hi = np.asarray(r.GetMin(), dtype=np.float64), np.asarray(r.GetMax(), dtype=np.float64)
                if owner == body:
                    to_frame = Gf.Matrix4d(1.0)
                else:
                    reach = max(np.hypot(x, y) for x in (lo[0], hi[0]) for y in (lo[1], hi[1]))
                    lo[:2], hi[:2] = -reach, reach
                    to_frame = UsdGeom.Xformable(owner).ComputeLocalToWorldTransform(time) * to_body
                for s in signs:
                    c = np.where(s, hi, lo)
                    points.append(to_frame.Transform(Gf.Vec3d(*c.tolist())))
            if not points:
                raise RuntimeError(f"no collision shape under {DRONE_PRIM} to measure the collider box from")
            points = np.asarray(points, dtype=np.float64)
            lo, hi = points.min(axis=0), points.max(axis=0)
            self._box = ((lo + hi) / 2.0, (hi - lo) / 2.0)
            print(f"[SafeEval] drone collider box: centre {self._box[0].round(3).tolist()}, "
                  f"half extents {self._box[1].round(3).tolist()} m", flush=True)
        return self._box

    @staticmethod
    def _rotation(quat_wxyz):
        w, x, y, z = (float(v) for v in quat_wxyz)
        return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                         [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                         [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])

    def _box_hit(self, margin, pos, quat_wxyz):
        """The first obstacle prim the drone's collider box, grown by `margin` on every side, overlaps."""
        from omni.physx import get_physx_scene_query_interface
        centre, half = self._drone_box()
        world = np.asarray(pos, dtype=np.float64) + self._rotation(quat_wxyz) @ centre
        w, x, y, z = (float(v) for v in quat_wxyz)
        found = [None]
        drone = DRONE_PRIM.lower()

        def report(hit):
            path = str(hit.rigid_body)
            low = path.lower()
            if low.startswith(drone) or low.rsplit("/", 1)[-1] in self._BOUNDARY_LEAF_NAMES:
                return True
            if any(b in low for b in self._BOUNDARY):
                return True
            found[0] = path
            return False

        get_physx_scene_query_interface().overlap_box(
            carb.Float3(*(half + margin).tolist()), carb.Float3(*world.tolist()),
            carb.Float4(x, y, z, w), report, False)
        return found[0]

    def _box_corners(self, pos, quat_wxyz):
        centre, half = self._drone_box()
        signs = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
        return np.asarray(pos, dtype=np.float64) + (self._rotation(quat_wxyz) @ (centre + signs * half).T).T

    @staticmethod
    def _footprint_distance(points, corners):
        """Horizontal distance from each point to the convex footprint of `corners`, 0 inside it."""
        from scipy.spatial import ConvexHull
        a = corners[ConvexHull(corners).vertices]
        e = np.roll(a, -1, axis=0) - a
        q = np.asarray(points, dtype=np.float64).reshape(-1, 1, 2) - a
        # ConvexHull lists 2-D vertices counter-clockwise, so inside is left of every edge.
        inside = (e[:, 0] * q[..., 1] - e[:, 1] * q[..., 0] >= 0.0).all(axis=1)
        t = np.clip((q * e).sum(axis=2) / np.maximum((e * e).sum(axis=1), 1e-12), 0.0, 1.0)
        d = np.linalg.norm(q - t[..., None] * e, axis=2).min(axis=1)
        return np.where(inside, 0.0, d)

    def _body_clearance(self, pos, quat_wxyz):
        """(clearance, what) from the drone's collider box to the nearest surface: every PhysX
        collider by growing the box until it touches, then the arena walls and colliderless
        pool columns exactly."""
        from safe_navigation.common.config.run_settings import CLEARANCE_SEARCH_REACH_M
        reach = float(CLEARANCE_SEARCH_REACH_M)
        hit = self._box_hit(0.0, pos, quat_wxyz)
        if hit is not None:
            best = (0.0, hit)
        else:
            what = self._box_hit(reach, pos, quat_wxyz)
            if what is None:
                best = (reach, "")
            else:
                lo, hi = 0.0, reach
                while hi - lo > self.CLEARANCE_TOLERANCE_M:
                    mid = (lo + hi) / 2.0
                    h = self._box_hit(mid, pos, quat_wxyz)
                    if h is None:
                        lo = mid
                    else:
                        hi, what = mid, h
                best = ((lo + hi) / 2.0, what)
        if best[1] and self.humans is not None:
            i = self.humans.human_index(best[1])
            if i is not None:
                best = (best[0], f"dynamic_obstacle_{i}")
        corners = self._box_corners(pos, quat_wxyz)
        x_min, y_min, x_max, y_max = self.arena
        wall = float(min((corners[:, 0] - x_min).min(), (x_max - corners[:, 0]).min(),
                         (corners[:, 1] - y_min).min(), (y_max - corners[:, 1]).min()))
        if wall < best[0]:
            best = (max(wall, 0.0), "wall")
        # AirStack's own pool columns are visual only: moving ones when there are no people, and
        # static ones on its legacy domains, where SAFE's collider prims are not built.
        if self.humans is None and self.n_active_dyn > 0:
            d = self._footprint_distance(self.dyn_positions[: self.n_active_dyn, :2],
                                         corners[:, :2]) - self.obstacle_radius
            i = int(np.argmin(d))
            if d[i] < best[0]:
                best = (max(float(d[i]), 0.0), f"dynamic_obstacle_{i}")
        if self.safe_cfg is None and len(self.static_positions):
            d = self._footprint_distance(self.static_positions[:, :2], corners[:, :2]) - self.static_radii
            i = int(np.argmin(d))
            if d[i] < best[0]:
                best = (max(float(d[i]), 0.0), f"static_obstacle_{i}")
        return best

    def _check_collision(self, drone_pos, quat_wxyz):
        """Measure the drone's clearance every tick and latch the first contact, which is the
        collider box touching anything. A wall touch counts only once airborne, off the pad."""
        self.body_clearance, what = self._body_clearance(drone_pos, quat_wxyz)
        if self.collision or self.body_clearance > 0.0:
            return
        if what == "wall" and drone_pos[2] <= 0.3:
            return
        self.collision, self.collision_with = True, what

    def update(self):
        self._process_commands()
        self._apply_pending_vehicle_reset()

        t = self._sim_time()
        dt = 0.0 if self._last_sim_time is None else max(0.0, t - self._last_sim_time)
        self._step_dynamic_obstacles(dt)

        pos, quat, vel = self._drone_world_pose()
        if pos is not None:
            if vel is not None:
                self._drone_vel = vel
            elif self._last_drone_pos is not None and dt > 1e-6:
                self._drone_vel = (pos - self._last_drone_pos) / dt
            self._last_drone_pos = pos
            # A held sim calls this between ticks at a fixed pose, and that pose is measured once.
            measured = (t, tuple(pos.tolist()), tuple(quat))
            if measured != self._measured:
                self._measured = measured
                self._check_collision(pos, quat)
        self._last_sim_time = t

        if t - self._last_state_pub >= 1.0 / STATE_HZ - TICK_TOLERANCE_S and pos is not None:
            self._last_state_pub = t
            self._broadcast({
                "kind": "state",
                "t": t,
                "drone_pos": [round(float(v), 4) for v in pos],
                "drone_quat": [round(float(v), 5) for v in quat],
                "drone_vel": [round(float(v), 4) for v in self._drone_vel],
                "static": self.static_positions.round(3).tolist(),
                "dynamic": self.dyn_positions[: self.n_active_dyn].round(3).tolist(),
                "dynamic_vel": self.dyn_velocities[: self.n_active_dyn].round(3).tolist(),
                "dynamic_targets": self._dynamic_targets().round(3).tolist(),
                "collision": self.collision,
                "collision_with": self.collision_with,
                "body_clearance": round(float(self.body_clearance), 4),
            })

    def shutdown(self):
        try:
            self._server.shutdown()
            self._server.server_close()
        except Exception:
            pass


class SafeEvalApp:

    def __init__(self):
        self.timeline = omni.timeline.get_timeline_interface()
        self.pg = PegasusInterface()
        # One loop step is exactly SAFE's 1/60 s control tick: four 1/240 s physics substeps
        # per render, where Pegasus's 1/250 s default would make each step 0.016 s.
        self.pg.set_world_settings(physics_dt=PHYSICS_DT, rendering_dt=RENDER_DT)
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world
        self.timeline.stop()

        source_domain = CONFIG.get("safe_domain")
        self.pg.load_environment(resolve_env_url(None if source_domain else CONFIG["env_url"]))

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("Stage failed to load")
        if not wait_for_stage(stage):
            carb.log_warn("Stage load timed out — continuing anyway.")

        safe_cfg = None
        humans = None
        if source_domain:
            if _safe_source not in sys.path:
                raise RuntimeError('SAFE source checkout is not mounted in Isaac')
            # Pegasus owns the vehicle; SAFE owns the scene. Remove Pegasus's
            # default grid before adding SAFE's authored stage/ground.
            if stage.GetPrimAtPath('/World/stage').IsValid():
                stage.RemovePrim(Sdf.Path('/World/stage'))
            if CONFIG.get('static_only'):
                os.environ['SAFE_NO_DYNAMIC'] = '1'
                os.environ['SAFE_NO_DYNAMIC_FROM_YAML'] = '1'
            from eval.sim.safe_source_scene import load_domain, create_humans
            safe_cfg = load_domain(source_domain, CONFIG.get('goal_distance'))
            CONFIG['domain_info'] = {
                'size_x': float(safe_cfg.size_x),
                'size_y': float(safe_cfg.size_y),
                'size_z': float(safe_cfg.size_z),
                'origin_x': float(safe_cfg.origin_x),
                'origin_y': float(safe_cfg.origin_y),
                'origin_z': float(safe_cfg.origin_z),
            }
            paths = safe_cfg.static_obstacle_prim_paths or []
            if paths:
                from safe_navigation.common.config.scenarios import (
                    _read_obstacle_positions_from_stage,
                    _read_obstacle_aabbs_from_stage,
                )
                mesh_pos = _read_obstacle_positions_from_stage(
                    paths, 3, origin_z=safe_cfg.origin_z)
                mesh_aabb = _read_obstacle_aabbs_from_stage(paths)
                mesh_pos[:, [0, 1]] = mesh_pos[:, [1, 0]]
                mesh_pos[:, 2] += float(safe_cfg.origin_z)
                mesh_aabb[:, [0, 1, 2, 3]] = mesh_aabb[:, [1, 0, 3, 2]]
                CONFIG['domain_info']['static_mesh_positions'] = mesh_pos.tolist()
                CONFIG['domain_info']['static_mesh_aabbs'] = mesh_aabb.tolist()
            else:
                CONFIG['domain_info']['static_mesh_positions'] = []
                CONFIG['domain_info']['static_mesh_aabbs'] = []
            CONFIG['arena'] = [float(safe_cfg.origin_y), float(safe_cfg.origin_x),
                               float(safe_cfg.origin_y + safe_cfg.size_y),
                               float(safe_cfg.origin_x + safe_cfg.size_x)]
            CONFIG['spawn'] = [(CONFIG['arena'][0] + CONFIG['arena'][2]) / 2,
                               (CONFIG['arena'][1] + CONFIG['arena'][3]) / 2]
            CONFIG['spawn_z'] = float(safe_cfg.origin_z) + 0.07
            if safe_cfg.usd_path:
                # A stage's box centre can sit inside or under its terrain, where the vehicle
                # never settles and PX4's EKF never converges.
                from eval.sim.safe_source_scene import stage_spawn
                sx, sy, gz = stage_spawn(safe_cfg, CONFIG['spawn'])
                CONFIG['spawn'], CONFIG['spawn_z'] = [sx, sy], gz + 0.07
            CONFIG['max_dynamic'] = int(safe_cfg.num_dynamic_obstacles)
            CONFIG['max_static'] = int(safe_cfg.num_static_cylinders)
            if CONFIG.get('humans') and safe_cfg.num_dynamic_obstacles:
                humans = create_humans(safe_cfg)

        stage_prim = stage.GetPrimAtPath("/World/stage")
        if stage_prim.IsValid():
            scale_stage_prim(stage, "/World/stage", float(CONFIG["stage_scale"]))
            add_colliders(stage_prim)
            for _ in range(10):
                omni.kit.app.get_app().update()
        add_dome_light(stage)
        if not source_domain and not CONFIG["env_url"]:
            build_textured_arena(stage, CONFIG["arena"], float(CONFIG.get("size_z", 5.0)))

        spawn = CONFIG["spawn"]
        graph_handle = spawn_px4_multirotor_node(
            pegasus_node_name="PX4Multirotor",
            drone_prim=DRONE_PRIM,
            robot_name=ROBOT_NAME,
            vehicle_id=1,
            domain_id=ROS_DOMAIN,
            usd_file=DRONE_USD,
            init_pos=[float(spawn[0]), float(spawn[1]), float(CONFIG["spawn_z"])],
            init_orient=[0.0, 0.0, 0.0, 1.0],
        )
        # Each teleport relaunches PX4 for a fresh EKF, which only a PX4 Pegasus launched allows.
        px4_node = f"{graph_handle.get_path_to_graph()}/PX4Multirotor"
        if not og.Controller.get(og.Controller.attribute("inputs:px4Autolaunch", px4_node)):
            raise RuntimeError(
                f"{px4_node} has inputs:px4Autolaunch false, so PX4 cannot be relaunched after "
                "a teleport and the next arm fails; spawn it with "
                "spawn_px4_multirotor_node(px4_autolaunch=True)")
        pov_res = int(os.environ.get("SAFE_POV_RES") or "0")
        if pov_res and not 32 <= pov_res <= 1024:
            raise ValueError(f"SAFE_POV_RES must be in [32, 1024], got {pov_res}")
        camera_kwargs = {"frame_height": pov_res, "frame_width": pov_res} if pov_res else {}
        add_zed_stereo_camera_subgraph(
            parent_graph_handle=graph_handle,
            drone_prim=DRONE_PRIM,
            robot_name=ROBOT_NAME,
            camera_name="ZEDCamera",
            camera_offset=[0.2, 0.0, -0.05],
            camera_rotation_offset=[0.0, 0.0, 0.0],
            **camera_kwargs,
        )
        # Policy camera intrinsics are a real part of the observation contract.
        # Configure the attached ZED optics rather than resizing a wrong-FOV image.
        hfov = os.environ.get("SAFE_POV_HFOV_DEG")
        vfov = os.environ.get("SAFE_POV_VFOV_DEG")
        if hfov:
            for side in ("camera_left", "camera_right"):
                prim = stage.GetPrimAtPath(f"{DRONE_PRIM}/ZEDCamera/base_link/ZED_X/{side}")
                if not prim.IsValid():
                    raise RuntimeError(f"attached ZED camera {side} is missing")
                camera = UsdGeom.Camera(prim)
                aperture = camera.GetHorizontalApertureAttr().Get()
                focal = 0.5 * float(aperture) / math.tan(math.radians(float(hfov)) / 2.0)
                camera.GetFocalLengthAttr().Set(focal)
                if vfov:
                    camera.GetVerticalApertureAttr().Set(
                        2.0 * focal * math.tan(math.radians(float(vfov)) / 2.0))
                elif pov_res:
                    camera.GetVerticalApertureAttr().Set(float(aperture))
        if os.environ.get("ENABLE_LIDAR", "false").lower() == "true":
            try:
                from pegasus.simulator.ogn.api.spawn_ouster_lidar import add_ouster_lidar_subgraph
                add_ouster_lidar_subgraph(
                    parent_graph_handle=graph_handle,
                    drone_prim=DRONE_PRIM,
                    robot_name=ROBOT_NAME,
                    lidar_name="OS1_REV6_128_10hz___512_resolution",
                    lidar_topic_name="point_cloud_raw",
                    lidar_offset=[0.0, 0.0, 0.025],
                    lidar_rotation_offset=[0.0, 0.0, 0.0],
                    lidar_min_range=0.75,
                )
            except ImportError as e:
                raise RuntimeError(f"[safe_eval] requested lidar unavailable: {e}") from e

        safe_static_prims = []
        if safe_cfg is not None:
            from safe_isaac.world_builder import spawn_static_cylinder_obstacles
            safe_static_prims = spawn_static_cylinder_obstacles(
                int(safe_cfg.num_static_cylinders), env_prefix='safe_eval',
                trees=bool(safe_cfg.tree_usd))
        pool = ObstaclePool(stage, radius=float(CONFIG["obstacle_radius"]))
        pool.build(0 if safe_cfg is not None else int(CONFIG["max_static"]),
                   0 if humans is not None else int(CONFIG["max_dynamic"]))
        self.manager = ScenarioManager(stage, pool, CONFIG, self.pg,
                                       humans=humans, safe_cfg=safe_cfg,
                                       safe_static_prims=safe_static_prims)

        self.play_on_start = os.environ.get("PLAY_SIM_ON_START", "true").lower() == "true"

    def run(self):
        if self.play_on_start:
            self.timeline.play()
        else:
            self.timeline.stop()

        app = omni.kit.app.get_app()
        print(f"[safe_eval] running — ScenarioManager TCP on :{SERVER_PORT}, "
              f"drone topics on ROS domain {ROS_DOMAIN}", flush=True)
        while simulation_app.is_running():
            if self.manager.holding():
                # Neither physics nor rendering advances; host commands are still served.
                self.manager.update()
                time.sleep(0.0005)
                continue
            world = World.instance()
            if world is not None and hasattr(world, "_scene"):
                world.step(render=True)
                if world is not self.world:
                    self.world = world
                    self.pg._world = world
            else:
                app.update()
            self.manager.update()
            self.manager.tick_done()

        carb.log_warn("Closing simulation.")
        self.manager.shutdown()
        self.timeline.stop()
        simulation_app.close()


def main():
    SafeEvalApp().run()


if __name__ == "__main__":
    main()
