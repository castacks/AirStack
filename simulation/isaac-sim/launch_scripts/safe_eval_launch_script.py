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
import socketserver
import sys
import threading
import time

import numpy as np

import omni.kit.app
import omni.timeline
import omni.usd

from pxr import Gf, UsdGeom

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
STATE_HZ = 10.0

_DEFAULT_CONFIG = {
    "env_url": None,
    "stage_scale": 1.0,
    "spawn": [0.0, 0.0],
    "max_dynamic": 30,
    "max_static": 12,
    "obstacle_radius": 0.3,
    "drone_radius": 0.31,
    "arena": [-10.0, -10.0, 10.0, 10.0],
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

    def __init__(self, stage, pool: ObstaclePool, cfg: dict):
        self._clients: list = []
        self._clients_lock = threading.Lock()
        self._inbox: "queue.Queue" = queue.Queue()
        self._server = socketserver.ThreadingTCPServer(
            ("0.0.0.0", SERVER_PORT), self._make_handler())
        self._server.daemon_threads = True
        threading.Thread(target=self._server.serve_forever, daemon=True).start()

        self.stage = stage
        self.pool = pool
        self.cfg = cfg
        self.arena = [float(v) for v in cfg["arena"]]
        self.obstacle_radius = float(cfg["obstacle_radius"])
        self.drone_radius = float(cfg["drone_radius"])

        self._drone_xform = None  # created lazily once the drone prim exists
        self._last_drone_pos = None
        self._last_sim_time = None
        self._drone_vel = np.zeros(3)

        # Scenario state (set by set_scenario)
        self.static_positions = np.zeros((0, 3), dtype=np.float32)
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
                resp = {"kind": "response", "id": cmd_id, "cmd": cmd, "ok": False,
                        "error": str(e)}
            self._send_line(wfile, resp)

    def _dispatch(self, cmd, args):
        if cmd == "ping":
            return {"sim_time": self._sim_time()}
        if cmd == "get_info":
            return {
                "arena": self.arena,
                "obstacle_radius": self.obstacle_radius,
                "drone_radius": self.drone_radius,
                "max_static": len(self.pool.static_ops),
                "max_dynamic": len(self.pool.dynamic_ops),
            }
        if cmd == "set_scenario":
            return self._set_scenario(args)
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
        raise ValueError(f"unknown cmd '{cmd}'")

    def _set_scenario(self, args):
        static = np.asarray(args.get("static", []), dtype=np.float32).reshape(-1, 3) \
            if args.get("static") else np.zeros((0, 3), dtype=np.float32)
        dyn = np.asarray(args.get("dynamic", []), dtype=np.float32).reshape(-1, 3) \
            if args.get("dynamic") else np.zeros((0, 3), dtype=np.float32)
        vel = np.asarray(args.get("dynamic_vel", []), dtype=np.float32).reshape(-1, 3) \
            if args.get("dynamic_vel") else np.zeros_like(dyn)
        wpts = np.asarray(args.get("waypoints", []), dtype=np.float32).reshape(-1, 3) \
            if args.get("waypoints") else dyn.copy()

        if len(static) > len(self.pool.static_ops):
            raise ValueError(f"{len(static)} static > pool {len(self.pool.static_ops)}")
        if len(dyn) > len(self.pool.dynamic_ops):
            raise ValueError(f"{len(dyn)} dynamic > pool {len(self.pool.dynamic_ops)}")

        self.static_positions = static
        self.dyn_positions = dyn
        self.dyn_velocities = vel
        self.dyn_waypoints = wpts
        self.n_active_dyn = len(dyn)
        self.target = [float(v) for v in args["target"]] if args.get("target") else None
        self._rng = np.random.RandomState(int(args.get("seed", 0)))

        self.pool._place(self.pool.static_ops, static)
        self.pool._place(self.pool.dynamic_ops, dyn)
        self.collision = False
        self.collision_with = ""
        return {"n_static": len(static), "n_dynamic": len(dyn)}

    # ── per-step update ──────────────────────────────────────────────────

    def _sim_time(self):
        world = World.instance()
        return float(world.current_time) if world is not None else 0.0

    def _drone_world_pose(self):
        if self._drone_xform is None:
            prim = self.stage.GetPrimAtPath(DRONE_PRIM)
            if not prim.IsValid():
                return None, None
            self._drone_xform = UsdGeom.Xformable(prim)
        mat = self._drone_xform.ComputeLocalToWorldTransform(0)
        pos = mat.ExtractTranslation()
        quat = mat.ExtractRotationQuat()
        im = quat.GetImaginary()
        return (np.array([pos[0], pos[1], pos[2]], dtype=np.float64),
                [float(quat.GetReal()), float(im[0]), float(im[1]), float(im[2])])

    def _apply_pending_vehicle_reset(self):
        if self._pending_vehicle_reset is None:
            return
        req = self._pending_vehicle_reset
        self._pending_vehicle_reset = None
        prim = self.stage.GetPrimAtPath(DRONE_PRIM)
        if not prim.IsValid():
            carb.log_warn("[safe_eval] reset_vehicle: drone prim not found")
            return
        try:
            from isaacsim.core.prims import XFormPrim
            xf = XFormPrim(DRONE_PRIM)
            yaw = req["yaw"]
            quat = np.array([[np.cos(yaw / 2.0), 0.0, 0.0, np.sin(yaw / 2.0)]])
            xf.set_world_poses(positions=np.array([req["pos"]]), orientations=quat)
        except Exception as e:
            carb.log_error(f"[safe_eval] reset_vehicle failed: {e}")
            return
        self._restart_px4_for_fresh_ekf()

    def _restart_px4_for_fresh_ekf(self):
        """Relaunch PX4 after the teleport so the EKF re-inits at the new position.
        A position teleport poisons the estimator and PX4 then denies the arm."""
        backend = None
        try:
            for v in self.pg.vehicle_manager.vehicles.values():
                for b in getattr(v, "_backends", []):
                    if type(b).__name__ == "PX4MavlinkBackend":
                        backend = b
                        break
        except Exception as e:
            carb.log_warn(f"[safe_eval] could not reach PX4 backend: {e}")
        if backend is None:
            carb.log_warn("[safe_eval] no PX4MavlinkBackend to restart — arm may fail")
            return
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

    def _check_collision(self, drone_pos):
        if self.collision:
            return
        p2d = drone_pos[:2]
        contact = self.drone_radius + self.obstacle_radius
        if self.n_active_dyn > 0:
            d = np.linalg.norm(self.dyn_positions[: self.n_active_dyn, :2] - p2d, axis=1)
            i = int(np.argmin(d))
            if d[i] < contact:
                self.collision, self.collision_with = True, f"dynamic_obstacle_{i}"
                return
        if len(self.static_positions) > 0:
            d = np.linalg.norm(self.static_positions[:, :2] - p2d, axis=1)
            i = int(np.argmin(d))
            if d[i] < contact:
                self.collision, self.collision_with = True, f"static_obstacle_{i}"
                return
        x_min, y_min, x_max, y_max = self.arena
        wall = min(p2d[0] - x_min, x_max - p2d[0], p2d[1] - y_min, y_max - p2d[1])
        # Only flag wall hits while airborne — sitting on the pad near a wall
        # before takeoff is not a collision.
        if wall < self.drone_radius / 2.0 and drone_pos[2] > 0.3:
            self.collision, self.collision_with = True, "wall"

    def update(self):
        self._process_commands()
        self._apply_pending_vehicle_reset()

        t = self._sim_time()
        dt = 0.0 if self._last_sim_time is None else max(0.0, t - self._last_sim_time)
        self._step_dynamic_obstacles(dt)

        pos, quat = self._drone_world_pose()
        if pos is not None:
            if self._last_drone_pos is not None and dt > 1e-6:
                self._drone_vel = (pos - self._last_drone_pos) / dt
            self._last_drone_pos = pos
            self._check_collision(pos)
        self._last_sim_time = t

        if t - self._last_state_pub >= 1.0 / STATE_HZ and pos is not None:
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
                "collision": self.collision,
                "collision_with": self.collision_with,
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
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world
        self.timeline.stop()

        self.pg.load_environment(resolve_env_url(CONFIG["env_url"]))

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("Stage failed to load")
        if not wait_for_stage(stage):
            carb.log_warn("Stage load timed out — continuing anyway.")

        stage_prim = stage.GetPrimAtPath("/World/stage")
        if stage_prim.IsValid():
            scale_stage_prim(stage, "/World/stage", float(CONFIG["stage_scale"]))
            add_colliders(stage_prim)
            for _ in range(10):
                omni.kit.app.get_app().update()
        add_dome_light(stage)

        spawn = CONFIG["spawn"]
        graph_handle = spawn_px4_multirotor_node(
            pegasus_node_name="PX4Multirotor",
            drone_prim=DRONE_PRIM,
            robot_name=ROBOT_NAME,
            vehicle_id=1,
            domain_id=ROS_DOMAIN,
            usd_file=DRONE_USD,
            init_pos=[float(spawn[0]), float(spawn[1]), 0.07],
            init_orient=[0.0, 0.0, 0.0, 1.0],
        )
        add_zed_stereo_camera_subgraph(
            parent_graph_handle=graph_handle,
            drone_prim=DRONE_PRIM,
            robot_name=ROBOT_NAME,
            camera_name="ZEDCamera",
            camera_offset=[0.2, 0.0, -0.05],
            camera_rotation_offset=[0.0, 0.0, 0.0],
        )
        if os.environ.get("ENABLE_LIDAR", "false").lower() == "true":
            try:
                from pegasus.simulator.ogn.api.spawn_rtx_lidar import add_rtx_lidar_subgraph
                add_rtx_lidar_subgraph(
                    parent_graph_handle=graph_handle,
                    drone_prim=DRONE_PRIM,
                    robot_name=ROBOT_NAME,
                    lidar_config="ouster_os1",
                    lidar_topic_name="point_cloud_raw",
                    lidar_offset=[0.0, 0.0, 0.025],
                    lidar_rotation_offset=[0.0, 0.0, 0.0],
                    min_range=0.75,
                )
            except ImportError as e:
                carb.log_warn(f"[safe_eval] lidar unavailable ({e}) — continuing without it")

        pool = ObstaclePool(stage, radius=float(CONFIG["obstacle_radius"]))
        pool.build(int(CONFIG["max_static"]), int(CONFIG["max_dynamic"]))
        self.manager = ScenarioManager(stage, pool, CONFIG)

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
            world = World.instance()
            if world is not None and hasattr(world, "_scene"):
                world.step(render=True)
                if world is not self.world:
                    self.world = world
                    self.pg._world = world
            else:
                app.update()
            self.manager.update()

        carb.log_warn("Closing simulation.")
        self.manager.shutdown()
        self.timeline.stop()
        simulation_app.close()


def main():
    SafeEvalApp().run()


if __name__ == "__main__":
    main()
