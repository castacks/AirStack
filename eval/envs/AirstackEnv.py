from __future__ import annotations

import time
import numpy as np
import omni.replicator.core as rep

from safe_core.core.env_backend import BaseEnvBackend

from ..sim.scenario_config import configure_scenario, update_dynamic_obstacles


class AirstackEnv(BaseEnvBackend):

    def __init__(
        self,
        sim,
        adapter=None,
        env_cfg=None,
        sensors=None,
        size_x: float = 20.0,
        size_y: float = 20.0,
        dimension: int = 3,
        max_speed: float = 3.0,
        dynamic_obstacle_speed: float = 0.9,
        dynamic_motion_behavior: str = "waypoint",
        max_episode_time: float = 60.0,
        dt: float = 0.1,
        max_steps: int = 600,
        epsilon_unsafe: float = 0.3,
    ):
        self.sim = sim
        self.adapter = adapter
        self.env_cfg = env_cfg
        self.sensors = sensors or ["full_state"]

        self.size_x = size_x
        self.size_y = size_y
        self.size_z = float(getattr(env_cfg, "size_z", max(size_x, size_y))) if env_cfg else max(size_x, size_y)
        self.origin_x = float(getattr(env_cfg, "origin_x", 0.0)) if env_cfg else 0.0
        self.origin_y = float(getattr(env_cfg, "origin_y", 0.0)) if env_cfg else 0.0
        self.dimension = dimension
        self.max_speed = max_speed

        self.goal_threshold = 1.0
        self.max_episode_time = max_episode_time
        self.dt = dt
        self.max_steps = max_steps

        self.v_threshold = 0.01
        self.epsilon_unsafe = epsilon_unsafe

        self.agent_radius = 0.15
        self.obstacle_radius = 0.3
        self.r_failure = self.obstacle_radius + 0.01
        self.r_unsafe = self.r_failure + epsilon_unsafe

        self.num_obstacles = len(getattr(env_cfg, "static_obstacle_prim_paths", None) or []) if env_cfg else 0
        self.num_dynamic_obstacles = getattr(env_cfg, "num_dynamic_obstacles", 0) if env_cfg else 0
        self.num_static_cylinders = int(getattr(env_cfg, "num_static_cylinders", 0)) if env_cfg else 0
        self.dynamic_obstacle_speed = dynamic_obstacle_speed
        self.dynamic_motion_behavior = dynamic_motion_behavior

        self.obstacle_values = ["high"] * self.num_dynamic_obstacles + ["low"] * self.num_obstacles

        self._step_count = 0
        self._episode_start_time = 0.0
        self._current_scenario_seed = object()
        self._camera_ready = False
        self._capture_camera_initialized = False
        self._rgb_annotator = None
        self._track_annotator = None

        self.np_random = np.random.RandomState()
        self.static_obstacle_aabbs = np.zeros((0, 4), dtype=np.float32)

        self._agent_location = np.zeros(dimension, dtype=np.float32)
        self._target_location = np.zeros(dimension, dtype=np.float32)
        self._static_obstacle_locations = np.zeros((self.num_obstacles, dimension), dtype=np.float32)
        self._static_cylinder_locations = np.zeros((self.num_static_cylinders, dimension), dtype=np.float32)
        self._dynamic_obstacle_locations = np.zeros((self.num_dynamic_obstacles, dimension), dtype=np.float32)
        self._dynamic_obstacle_velocities = np.zeros((self.num_dynamic_obstacles, dimension), dtype=np.float32)
        self._dynamic_obstacle_waypoints = np.zeros((self.num_dynamic_obstacles, dimension), dtype=np.float32)

    def _get_obs(self):
        dyn_combined = np.concatenate(
            [self._dynamic_obstacle_locations, self._dynamic_obstacle_velocities],
            axis=1,
        ) if self.num_dynamic_obstacles > 0 else np.zeros((0, self.dimension * 2), dtype=np.float32)

        return {
            "agent":             self._agent_location.copy(),
            "target":            self._target_location.copy(),
            "dynamic_obstacles": dyn_combined.astype(np.float32),
            "static_obstacles":  self._static_obstacle_locations.copy(),
            "static_cylinders":  self._static_cylinder_locations.copy(),
            "env_bounds":        np.array([
                self.origin_x,
                self.origin_y,
                0.0,
                self.origin_x + self.size_x,
                self.origin_y + self.size_y,
                self.size_z,
            ], dtype=np.float32),
        }

    def get_evaluator_state(self):
        """Ground-truth state for S.A.F.E. metrics (independent of policy sensor subset)."""
        state = {
            "agent":             self._agent_location.copy(),
            "target":            self._target_location.copy(),
            "static_obstacles":  self._static_obstacle_locations.copy(),
            "static_cylinders":  self._static_cylinder_locations.copy(),
        }
        if self.num_dynamic_obstacles > 0:
            state["dynamic_obstacles"] = np.concatenate(
                [self._dynamic_obstacle_locations.copy(),
                 self._dynamic_obstacle_velocities.copy()],
                axis=1,
            )
        return state

    def _get_info(self):
        return {"distance": float(np.linalg.norm(self._agent_location - self._target_location))}

    # ------------------------------------------------------------------
    # Reset
    # ------------------------------------------------------------------

    def reset(self, seed=None, options=None):
        self._step_count = 0

        is_new_scenario = (seed != self._current_scenario_seed)
        if is_new_scenario:
            self._current_scenario_seed = seed
            if seed is not None:
                self.np_random = np.random.RandomState(seed)

            scenario = configure_scenario(self)
            self._agent_location              = scenario["agent_location"]
            self._target_location             = scenario["target_location"]
            self._static_obstacle_locations   = scenario["static_obstacle_locations"]
            self._dynamic_obstacle_locations  = scenario["dynamic_obstacle_locations"]
            self._dynamic_obstacle_velocities = scenario["dynamic_obstacle_velocities"]
            self._dynamic_obstacle_waypoints  = scenario["dynamic_obstacle_waypoints"]
            self._static_cylinder_locations   = scenario["static_cylinder_locations"]

            self._initial_agent_location              = self._agent_location.copy()
            self._initial_target_location             = self._target_location.copy()
            self._initial_static_obstacle_locations   = self._static_obstacle_locations.copy()
            self._initial_dynamic_obstacle_locations  = self._dynamic_obstacle_locations.copy()
            self._initial_dynamic_obstacle_velocities = self._dynamic_obstacle_velocities.copy()
            self._initial_dynamic_obstacle_waypoints  = self._dynamic_obstacle_waypoints.copy()
            self._initial_static_cylinder_locations   = self._static_cylinder_locations.copy()
        else:
            self._agent_location              = self._initial_agent_location.copy()
            self._target_location             = self._initial_target_location.copy()
            self._static_obstacle_locations   = self._initial_static_obstacle_locations.copy()
            self._dynamic_obstacle_locations  = self._initial_dynamic_obstacle_locations.copy()
            self._dynamic_obstacle_velocities = self._initial_dynamic_obstacle_velocities.copy()
            self._dynamic_obstacle_waypoints  = self._initial_dynamic_obstacle_waypoints.copy()
            self._static_cylinder_locations   = self._initial_static_cylinder_locations.copy()

        if self.dimension > 2:
            flight_z = float(self.env_cfg.flight_z) if self.env_cfg else 1.5
            self._target_location = self._target_location.copy()
            self._target_location[2] = flight_z

        # Delegate vehicle reset to AirStack's SimulatorManager
        self.sim.set_obstacles(self._static_obstacle_locations, self._dynamic_obstacle_locations)
        if self.num_static_cylinders > 0 and hasattr(self.sim, "set_static_cylinders"):
            self.sim.set_static_cylinders(self._static_cylinder_locations)
        self.sim.set_goal_marker(self._target_location)
        self.sim.reset_vehicle(self._agent_location)

        if not self._capture_camera_initialized:
            self._setup_capture_camera()
            self._capture_camera_initialized = True

        self._episode_start_time = time.time()

        print(
            f"[AirstackEnv.reset] agent={self._agent_location} goal={self._target_location}",
            flush=True,
        )
        return self._get_obs(), self._get_info()

    # ------------------------------------------------------------------
    # Step
    # ------------------------------------------------------------------

    def step(self, action):
        self._step_count += 1

        # Advance dynamic obstacles in benchmark frame (teleport mode)
        if self.num_dynamic_obstacles > 0:
            n_dyn = getattr(self, "_active_dynamic_obstacles", self.num_dynamic_obstacles)
            ox = float(getattr(self.env_cfg, "origin_x", 0.0)) if self.env_cfg else 0.0
            oy = float(getattr(self.env_cfg, "origin_y", 0.0)) if self.env_cfg else 0.0
            self._dynamic_obstacle_locations, self._dynamic_obstacle_velocities = update_dynamic_obstacles(
                self._dynamic_obstacle_locations,
                self._dynamic_obstacle_velocities,
                self.size_x,
                self.size_y,
                self.obstacle_radius,
                origin_x=ox,
                origin_y=oy,
                n_active=n_dyn,
                waypoints=self._dynamic_obstacle_waypoints,
                rng=self.np_random,
                dt=self.dt,
            )
            self.sim.set_obstacles(self._static_obstacle_locations, self._dynamic_obstacle_locations)

        # Send normalised velocity to AirStack's trajectory controller
        vel = np.asarray(action, dtype=np.float32) * self.max_speed
        self.sim.send_velocity(vel)
        self.sim.step()

        # Read drone position back from AirStack
        pos = self.sim.get_position()
        self._agent_location = pos[:self.dimension]

        collision = self._check_collision()

        dist = float(np.linalg.norm(self._agent_location[:2] - self._target_location[:2]))
        terminated = dist < self.goal_threshold or collision
        truncated = self._step_count >= self.max_steps

        info = self._get_info()
        info["collision"] = collision
        info["elapsed_time"] = time.time() - self._episode_start_time

        return self._get_obs(), terminated, truncated, info

    def _check_collision(self) -> bool:
        """Check collision via PhysX overlap sphere if available, otherwise SDF fallback."""
        try:
            from omni.physx import get_physx_scene_query_interface
            import carb

            p = self._to_isaac_pos(self._agent_location)
            origin = carb.Float3(float(p[0]), float(p[1]), float(p[2]))

            _BOUNDARY = ("floor", "wall", "ceiling", "ground", "plane", "terrain")
            drone_token = "/agent"
            goal_token  = "/goal_marker"

            hit_found = [False]

            def _report(hit):
                path = str(hit.rigid_body).lower()
                if any(b in path for b in _BOUNDARY):
                    return True
                if drone_token in path:
                    return True
                if goal_token in path:
                    return True
                hit_found[0] = True
                return False

            get_physx_scene_query_interface().overlap_sphere(self.r_failure, origin, _report, False)
            return hit_found[0]
        except Exception:
            pass

        # SDF fallback
        all_obs = self._all_obstacle_positions_2d()
        if len(all_obs) > 0:
            dists = np.linalg.norm(np.asarray(all_obs)[:, :2] - self._agent_location[:2], axis=1)
            if dists.min() <= self.r_failure:
                return True
        return False

    def _all_obstacle_positions_2d(self):
        parts = []
        n_dyn = getattr(self, "_active_dynamic_obstacles", self.num_dynamic_obstacles)
        if n_dyn > 0:
            parts.append(self._dynamic_obstacle_locations[:n_dyn])
        if self.num_obstacles > 0:
            parts.append(self._static_obstacle_locations)
        if self.num_static_cylinders > 0:
            parts.append(self._static_cylinder_locations)
        return np.concatenate(parts, axis=0) if parts else np.zeros((0, self.dimension), dtype=np.float32)

    @property
    def action_space(self):
        class _Box:
            def __init__(self, dim):
                self._dim = dim
            def sample(self):
                return np.random.uniform(-1.0, 1.0, size=self._dim).astype(np.float32)
        return _Box(self.dimension)

    def get_env_params(self) -> dict:
        return {
            "RE_max":          self.r_unsafe - self.r_failure,
            "r_unsafe":        self.r_unsafe,
            "r_failure":       self.r_failure,
            "obstacle_radius": self.obstacle_radius,
            "max_speed":       self.max_speed,
            "v_threshold":     self.v_threshold,
            "epsilon_unsafe":  self.epsilon_unsafe,
            "goal_threshold":  self.goal_threshold,
            "max_episode_time": self.max_episode_time,
            "obstacle_values": self.obstacle_values,
            "max_steps":       self.max_steps,
            "dt":              self.dt,
            "dim":             self.dimension,
            "x_min":           self.origin_x,
            "x_max":           self.origin_x + self.size_x,
            "y_min":           self.origin_y,
            "y_max":           self.origin_y + self.size_y,
        }

    # ------------------------------------------------------------------
    # Coordinate helpers
    # ------------------------------------------------------------------

    def _to_isaac_pos(self, pos):
        """Convert benchmark XYZ (X-forward) to Isaac XYZ (Y-forward)."""
        p = np.asarray(pos, dtype=float)
        oz = float(getattr(self.env_cfg, "origin_z", 0.0)) if self.env_cfg else 0.0
        ox, oy = getattr(self.sim, "_world_offset_isaac", (0.0, 0.0))
        if p.size == 2:
            return np.array([p[1] + ox, p[0] + oy, 1.5 + oz])
        if p.size >= 3:
            return np.array([p[1] + ox, p[0] + oy, p[2] + oz])
        return np.array([p[0] + ox, 0.0, 1.5 + oz])

    def _setup_capture_camera(self):
        """Add a top-down overview camera and a drone-tracking camera."""
        try:
            origin_x = getattr(self.env_cfg, 'origin_x', 0.0) if self.env_cfg else 0.0
            origin_y = getattr(self.env_cfg, 'origin_y', 0.0) if self.env_cfg else 0.0
            # Use env_cfg size — it's updated from the real AABB after load_environment_usd,
            # whereas self.size_x/y is frozen at construction time (pre-AABB).
            size_x = float(self.env_cfg.size_x) if self.env_cfg else self.size_x
            size_y = float(self.env_cfg.size_y) if self.env_cfg else self.size_y
            mid_x = origin_x + size_x / 2.0
            mid_y = origin_y + size_y / 2.0
            # Benchmark-frame center; _to_isaac_pos applies axis swap + parallel offset.
            look_bench = np.array([mid_x, mid_y, 0.0], dtype=float)
            pos_bench = np.array(
                [mid_x, mid_y, float(max(self.size_x, self.size_y))], dtype=float
            )
            look_isaac = self._to_isaac_pos(look_bench)
            pos_isaac = self._to_isaac_pos(pos_bench)
            self._capture_camera = rep.create.camera(
                position=tuple(float(x) for x in pos_isaac),
                look_at=tuple(float(x) for x in look_isaac),
                focal_length=18.0,
            )
            self._render_product = rep.create.render_product(self._capture_camera, (640, 640))
            self._rgb_annotator = rep.AnnotatorRegistry.get_annotator("rgb")
            self._rgb_annotator.attach([self._render_product])
        except Exception as e:
            print(f"[IsaacSimEnv] Overview camera setup failed: {e}")
            self._rgb_annotator = None

        # Tracking camera: USD prim at a per-env path so parallel envs don't collide.
        # xformOp:translate is added only if not already present (safe on re-setup).
        try:
            import omni.usd
            from pxr import UsdGeom, Sdf, Gf

            stage      = omni.usd.get_context().get_stage()
            env_prefix = getattr(self.sim, "_env_prefix", "env_0")
            track_path = f"/World/{env_prefix}/BenchmarkTrackingCamera"
            cam_geom   = UsdGeom.Camera.Define(stage, Sdf.Path(track_path))
            cam_geom.GetFocalLengthAttr().Set(18.0)
            xformable  = UsdGeom.Xformable(cam_geom)
            existing   = [op.GetOpName() for op in xformable.GetOrderedXformOps()]
            if "xformOp:translate" not in existing:
                xformable.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 8.0))
            self._track_camera_prim_path = track_path
            self._track_product   = rep.create.render_product(track_path, (640, 640))
            self._track_annotator = rep.AnnotatorRegistry.get_annotator("rgb")
            self._track_annotator.attach([self._track_product])
        except Exception as e:
            print(f"[IsaacSimEnv] Tracking camera setup failed: {e}")
            self._track_annotator = None

    # ------------------------------------------------------------------
    # Real-time debug overlay  (omni.isaac.debug_drawing)
    # ------------------------------------------------------------------

    _FLOOR_Z_THRESHOLD = 0.25

    def _ring_benchmark_pos(self, obs_pos):
        """Lift floor-placed shelf obstacles so rings sit at flight height."""
        flight_z = float(getattr(self.env_cfg, "flight_z", 0.5) if self.env_cfg else 0.5)
        p = np.asarray(obs_pos, dtype=float).copy()
        if p.size >= 3 and abs(p[2]) < self._FLOOR_Z_THRESHOLD:
            p[2] = flight_z
        return p

    @staticmethod
    def _draw_circle(draw_iface, center, radius, color, n=32):
        """Draw a horizontal circle in the Isaac XY plane using line segments."""
        starts, ends = [], []
        for i in range(n):
            a1 = 2 * math.pi * i / n
            a2 = 2 * math.pi * (i + 1) / n
            starts.append((center[0] + radius * math.cos(a1),
                           center[1] + radius * math.sin(a1),
                           center[2]))
            ends.append((  center[0] + radius * math.cos(a2),
                           center[1] + radius * math.sin(a2),
                           center[2]))
        draw_iface.draw_lines(starts, ends, [color] * n, [1.5] * n)

    def _draw_wireframe_cylinder(self, draw_iface, obs_pos, radius, color, n=16):
        """Vertical safe-set cage: struts plus top/bottom rings (floor-centered 1.8 m cylinder)."""
        p = np.asarray(obs_pos, dtype=float)
        obs_z = float(p[2]) if p.size >= 3 else 0.0
        z_lo = obs_z
        z_hi = obs_z + sim_config.CYLINDER_HEIGHT

        bot_bench = np.array([p[0], p[1], z_lo], dtype=float)
        top_bench = np.array([p[0], p[1], z_hi], dtype=float)

        bot = self._to_isaac_pos(bot_bench)
        top = self._to_isaac_pos(top_bench)
        bot_center = (float(bot[0]), float(bot[1]), float(bot[2]))
        top_center = (float(top[0]), float(top[1]), float(top[2]))

        starts, ends = [], []
        for i in range(n):
            theta = 2 * math.pi * i / n
            dx = radius * math.cos(theta)
            dy = radius * math.sin(theta)
            starts.append((bot_center[0] + dx, bot_center[1] + dy, bot_center[2]))
            ends.append((top_center[0] + dx, top_center[1] + dy, top_center[2]))
        draw_iface.draw_lines(starts, ends, [color] * n, [1.5] * n)

        self._draw_circle(draw_iface, bot_center, radius, color)
        self._draw_circle(draw_iface, top_center, radius, color)

    def _draw_wall_faces(self, draw_iface):
        """Draw each arena wall face with its failure (red) and unsafe (orange) insets."""
        if not self.env_cfg:
            return
        ox = float(getattr(self.env_cfg, 'origin_x', 0.0))
        oy = float(getattr(self.env_cfg, 'origin_y', 0.0))
        sx = float(self.size_x)
        sy = float(self.size_y)
        fz = float(getattr(self.env_cfg, 'flight_z', 0.5))

        x_min, x_max = ox, ox + sx
        y_min, y_max = oy, oy + sy

        wall_color   = (0.8, 0.8, 0.8, 1.0)
        fail_color   = (1.0, 0.15, 0.15, 1.0)
        unsafe_color = (1.0, 0.60, 0.00, 1.0)

        # Failure inset = agent_radius (matches the 0.15 buffer in adapter l_failure wall term)
        # Unsafe inset  = (r_unsafe - obstacle_radius) + agent_radius  (matches l_unsafe wall term)
        fail_buf   = self.agent_radius
        unsafe_buf = (self.r_unsafe - self.obstacle_radius) + self.agent_radius

        def seg(bench_start, bench_end):
            p0 = self._to_isaac_pos(np.array(bench_start, dtype=float))
            p1 = self._to_isaac_pos(np.array(bench_end,   dtype=float))
            return tuple(float(v) for v in p0), tuple(float(v) for v in p1)

        # Each tuple: (wall_start, wall_end, fail_start, fail_end, unsafe_start, unsafe_end)
        # Wall faces in benchmark frame (x_fixed walls span Y; y_fixed walls span X):
        #   x_min wall: inset goes +x (inward)
        #   x_max wall: inset goes -x (inward)
        #   y_min wall: inset goes +y (inward)
        #   y_max wall: inset goes -y (inward)
        face_specs = [
            # x_min wall
            (seg([x_min, y_min, fz], [x_min, y_max, fz]),
             seg([x_min + fail_buf,   y_min, fz], [x_min + fail_buf,   y_max, fz]),
             seg([x_min + unsafe_buf, y_min, fz], [x_min + unsafe_buf, y_max, fz])),
            # x_max wall
            (seg([x_max, y_min, fz], [x_max, y_max, fz]),
             seg([x_max - fail_buf,   y_min, fz], [x_max - fail_buf,   y_max, fz]),
             seg([x_max - unsafe_buf, y_min, fz], [x_max - unsafe_buf, y_max, fz])),
            # y_min wall
            (seg([x_min, y_min, fz], [x_max, y_min, fz]),
             seg([x_min, y_min + fail_buf,   fz], [x_max, y_min + fail_buf,   fz]),
             seg([x_min, y_min + unsafe_buf, fz], [x_max, y_min + unsafe_buf, fz])),
            # y_max wall
            (seg([x_min, y_max, fz], [x_max, y_max, fz]),
             seg([x_min, y_max - fail_buf,   fz], [x_max, y_max - fail_buf,   fz]),
             seg([x_min, y_max - unsafe_buf, fz], [x_max, y_max - unsafe_buf, fz])),
        ]

        s_wall,   e_wall   = [], []
        s_fail,   e_fail   = [], []
        s_unsafe, e_unsafe = [], []
        for (ws, we), (fs, fe), (us, ue) in face_specs:
            s_wall.append(ws);   e_wall.append(we)
            s_fail.append(fs);   e_fail.append(fe)
            s_unsafe.append(us); e_unsafe.append(ue)

        draw_iface.draw_lines(s_wall, e_wall, [wall_color] * 4, [2.0] * 4)
        if not _cfg.SIMPLE:
            draw_iface.draw_lines(s_fail,   e_fail,   [fail_color]   * 4, [1.5] * 4)
            draw_iface.draw_lines(s_unsafe, e_unsafe, [unsafe_color] * 4, [1.5] * 4)

    def _draw_debug_overlay(self):
        """Update trajectory trail and safe-set rings in the viewport each step."""
        try:
            if not _cfg.DEBUG:
                return
        except ImportError:
            return

        try:
            if self._draw is None:
                try:
                    from isaacsim.util.debug_draw import _debug_draw   # Isaac Sim 4.5+
                except ModuleNotFoundError:
                    from omni.isaac.debug_draw import _debug_draw       # Isaac Sim < 4.5
                self._draw = _debug_draw.acquire_debug_draw_interface()

            self._draw.clear_lines()

            # --- Trajectory trail (agent) ---
            ip = self._to_isaac_pos(self._agent_location)
            self._traj_points.append((float(ip[0]), float(ip[1]), float(ip[2])))
            pts = self._traj_points
            if len(pts) >= 2:
                cyan = (0.2, 0.9, 1.0, 1.0)
                self._draw.draw_lines(
                    pts[:-1], pts[1:],
                    [cyan] * (len(pts) - 1),
                    [2.0]  * (len(pts) - 1),
                )

            # --- Agent radius ring (white) ---
            agent_center = (float(ip[0]), float(ip[1]), float(ip[2]))
            self._draw_circle(self._draw, agent_center, self.agent_radius,
                              (1.0, 1.0, 1.0, 1.0))

            # --- Trajectory trails (dynamic obstacles) ---
            if len(self._dyn_traj_points) != self.num_dynamic_obstacles:
                self._dyn_traj_points = [[] for _ in range(self.num_dynamic_obstacles)]
            for i, obs_pos in enumerate(self._dynamic_obstacle_locations):
                dip = self._to_isaac_pos(obs_pos)
                self._dyn_traj_points[i].append((float(dip[0]), float(dip[1]), float(dip[2])))
                dpts = self._dyn_traj_points[i]
                if len(dpts) >= 2:
                    self._draw.draw_lines(
                        dpts[:-1], dpts[1:],
                        [self._dyn_trail_color] * (len(dpts) - 1),
                        [1.5] * (len(dpts) - 1),
                    )

            # --- Safe-set wireframes for cylinders (static + dynamic) ---
            if not _cfg.SIMPLE:
                for locations in (
                    self._static_cylinder_locations,
                    self._dynamic_obstacle_locations,
                ):
                    for obs_pos in locations:
                        self._draw_wireframe_cylinder(
                            self._draw, obs_pos, self.r_failure, (1.0, 0.15, 0.15, 1.0))
                        self._draw_wireframe_cylinder(
                            self._draw, obs_pos, self.r_unsafe, (1.0, 0.6, 0.0, 1.0))
                        self._draw_wireframe_cylinder(
                            self._draw, obs_pos, self.r_unsafe + self.epsilon_unsafe,
                            (1.0, 1.0, 0.0, 1.0))

            # --- Wall faces with safe-set insets ---
            self._draw_wall_faces(self._draw)

            # --- Goal threshold ring (green) ---
            gt = self._to_isaac_pos(self._target_location)
            goal_center = (float(gt[0]), float(gt[1]), float(gt[2]))
            self._draw_circle(self._draw, goal_center, self.goal_threshold, (0.1, 1.0, 0.1, 1.0))

            # --- Safe-set rings for mesh static shelves ---
            if not _cfg.SIMPLE:
                for obs_pos in self._static_obstacle_locations:
                    p = self._ring_benchmark_pos(obs_pos)
                    c = self._to_isaac_pos(p)
                    center = (float(c[0]), float(c[1]), float(c[2]))
                    self._draw_circle(self._draw, center, self.r_failure,
                                      (1.0, 0.15, 0.15, 1.0))
                    self._draw_circle(self._draw, center, self.r_unsafe,
                                      (1.0, 0.6, 0.0, 1.0))
                    self._draw_circle(self._draw, center, self.r_unsafe + self.epsilon_unsafe,
                                      (1.0, 1.0, 0.0, 1.0))

            # --- Collision point (red dot, persists until episode reset) ---
            if self._collision_pos_isaac is not None:
                self._draw.draw_points(
                    [self._collision_pos_isaac],
                    [(1.0, 0.1, 0.1, 1.0)],
                    [12.0],
                )

        except Exception as e:
            print(f"[IsaacSimEnv] Debug overlay failed: {e}")

    def _refresh_capture_render(self):
        """One Replicator render tick for overview/tracking cameras (no extra physics step)."""
        try:
            import omni.replicator.core as rep
            rep.orchestrator.step(rt_subframes=1)
        except Exception as e:
            print(f"[IsaacSimEnv] _refresh_capture_render failed: {e}", flush=True)

    def _read_overview_frame(self):
        """Read overview RGB annotator only; call _refresh_capture_render() first."""
        if self._rgb_annotator is None:
            return None
        try:
            return self._decode_frame(self._rgb_annotator.get_data())
        except Exception:
            return None

    def _update_tracking_camera_pose(self):
        """Move the tracking camera above the current agent position."""
        if self._track_camera_prim_path is None:
            return
        import omni.usd
        from pxr import UsdGeom, Gf

        ip = self._to_isaac_pos(self._agent_location)
        track_offset = max(8.0, self.size_z * 0.6)
        track_z = float(ip[2]) + track_offset
        stage = omni.usd.get_context().get_stage()
        cam_prim = stage.GetPrimAtPath(self._track_camera_prim_path)
        if not cam_prim or not cam_prim.IsValid():
            return
        for op in UsdGeom.Xformable(cam_prim).GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                op.Set(Gf.Vec3d(float(ip[0]), float(ip[1]), track_z))
                break

    def _read_tracking_frame_buffer(self):
        """Read tracking RGB annotator only; update pose before refresh, not before read."""
        if self._track_annotator is None:
            return None
        try:
            return self._decode_frame(self._track_annotator.get_data())
        except Exception:
            return None

    def get_frame(self, advance_render=True):
        """Return the current rendered frame as an HxWx3 uint8 array, or None on failure."""
        if self._rgb_annotator is None:
            print("[IsaacSimEnv] get_frame() called but annotator not set up — GIF will be empty")
            return None
        try:
            if advance_render:
                if not self.sim.render and getattr(self, '_lockstep_render_only', False):
                    # Lockstep: caller must _refresh_capture_render() once per timestep.
                    pass
                elif not self.sim.render:
                    self.sim.world.step(render=True)
                else:
                    import omni.replicator.core as rep
                    rep.orchestrator.step(rt_subframes=1)
            return self._read_overview_frame()
        except Exception:
            pass
        return None

    def _decode_frame(self, data) -> "np.ndarray | None":
        """Convert raw annotator RGBA output to HxWx3 uint8."""
        if data is None or data.size == 0:
            return None
        rgb = np.asarray(data[:, :, :3])
        if np.issubdtype(rgb.dtype, np.floating):
            mx = float(np.nanmax(rgb)) if rgb.size else 0.0
            rgb = (np.clip(rgb, 0.0, 1.0) * 255.0).round() if mx <= 1.0 + 1e-3 else np.clip(rgb, 0.0, 255.0)
        return rgb.astype(np.uint8)

    def get_tracking_frame(self, advance_render=True):
        """Return a drone-tracking overhead frame, or None on failure."""
        if self._track_annotator is None:
            return None
        try:
            self._update_tracking_camera_pose()
            if advance_render:
                if not self.sim.render and getattr(self, '_lockstep_render_only', False):
                    pass
                elif not self.sim.render:
                    self.sim.world.step(render=True)
                else:
                    import omni.replicator.core as rep
                    rep.orchestrator.step(rt_subframes=1)
            return self._read_tracking_frame_buffer()
        except Exception as e:
            print(f"[IsaacSimEnv] get_tracking_frame failed: {e}")
            return None


    def close(self):
        pass
