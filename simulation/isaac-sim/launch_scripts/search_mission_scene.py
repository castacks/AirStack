#!/usr/bin/env python
"""MTL search mission scene: belief-textured search area, ground-truth targets,
and a NATIVE camera gimbal on every drone.

    ISAAC_SIM_SCRIPT_NAME=search_mission_scene.py \\
      airstack up --sim isaac --fleet mtl_search_fleet --stack mtl_search --play --wait

What it adds on top of ``PegasusApp`` (the stock scene stays untouched):

* ``post_scene_prep``: a collider ground slab, the search plane draped with the
  prior (``belief.png``), boundary bars, and one beacon per ground-truth target
  (``ground_truth.json``) — all from ``stacks/mtl_search/config`` (override with
  ``MTL_SCENARIO_DIR``), the same files the robots' planner and scorer read.
* ``spawn_drone``: after the Pegasus PX4 multirotor (and, only if the vehicle
  manifest lists them, the ZED / lidar subgraphs) it authors
  ``/World/drone{i}/base_link/camera_gimbal`` (Xform) with a ``camera`` prim
  whose field of view is the mission's ``sensor.fov_deg``.
* ``post_spawn``: one action graph per drone at ``/World/MTL/Gimbal_<robot>``
  with its own ``ROS2Context(domain_id = i)``:
    - ``ROS2Subscriber``  geometry_msgs/Vector3  /<robot>/gimbal/cmd_pitch_yaw
    - ``ROS2Publisher``   geometry_msgs/Vector3  /<robot>/gimbal/state   (measured)
    - render product -> ``ROS2CameraHelper`` /<robot>/gimbal/rgb and
      ``ROS2CameraInfoHelper`` /<robot>/gimbal/camera_info (frame camera_optical_frame)
  and an app-update callback that reads the command, runs the actuator model
  (travel limits + slew rate, sim time), and writes the gimbal prim's local
  transform from the Pegasus vehicle state (the rigid body's pose lives in
  fabric, not USD, so the vehicle state is the authoritative source — the same
  reason the follow-cam uses it).

Gimbal convention (command and state): x = roll, y = pitch, z = yaw [rad],
Z-Y-X Euler of the camera frame (x = boresight) in the EARTH frame (ENU); pitch
> 0 looks down (nadir = +90 deg). The mount is earth-stabilised; its position
rides the airframe (``sim_gimbal.mount_offset_m`` in mission.yaml).

Spawns: ``FLEET_CONFIG_FILE`` (``--fleet``) wins; otherwise one drone per
scenario team agent at its home. GPS homes are always written
(``world_gps_origin``) so PX4 and the GCS agree with the scenario's world frame.

Env: ISAAC_SIM_HEADLESS / ISAAC_SIM_LIVESTREAM / PLAY_SIM_ON_START (base class),
MTL_SCENARIO_DIR, MTL_GIMBAL_RENDER_HZ (assumed render rate for frame skipping,
default 60).
"""

import os
import sys
from pathlib import Path

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
from pegasus_app import create_simulation_app  # noqa: E402

simulation_app = create_simulation_app()  # FIRST — before any omni/pegasus import

import carb  # noqa: E402
import omni.graph.core as og  # noqa: E402
import omni.kit.app  # noqa: E402
from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade  # noqa: E402

from pegasus.simulator.params import SIMULATION_ENVIRONMENTS  # noqa: E402
from pegasus_app import PegasusApp  # noqa: E402
from gps_utils import DEFAULT_WORLD_ORIGIN  # noqa: E402

sys.path.insert(0, os.path.normpath(os.path.join(_HERE, "..", "utils")))
from mtl_scene import config as mcfg  # noqa: E402
from mtl_scene import gimbal as mg  # noqa: E402

MTL_ROOT = "/World/MTL"
GIMBAL_NAME = "camera_gimbal"
CAMERA_NAME = "camera"
OPTICAL_FRAME = "camera_optical_frame"


def _log(msg):
    print(f"[search_mission_scene] {msg}", flush=True)


# ----------------------------------------------------------------------------
# USD helpers
# ----------------------------------------------------------------------------
def _preview_material(stage, path, rgb, *, emissive=None, texture=None, roughness=0.9):
    """UsdPreviewSurface material (optionally textured) — renders in RTX."""
    mat = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path + "/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(roughness))
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    if texture:
        st = UsdShade.Shader.Define(stage, path + "/st")
        st.CreateIdAttr("UsdPrimvarReader_float2")
        st.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
        tex = UsdShade.Shader.Define(stage, path + "/Texture")
        tex.CreateIdAttr("UsdUVTexture")
        tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(texture)
        tex.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("clamp")
        tex.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("clamp")
        tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(st.ConnectableAPI(), "result")
        tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
            tex.ConnectableAPI(), "rgb")
    else:
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb[:3]))
    if emissive is not None:
        shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*emissive))
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return mat


def _bind(prim, mat):
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat)


def _box(stage, path, center, size, rgb, mat=None, collider=False):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    x = UsdGeom.Xformable(cube.GetPrim())
    x.ClearXformOpOrder()
    x.AddTranslateOp().Set(Gf.Vec3d(*center))
    x.AddScaleOp().Set(Gf.Vec3f(*size))
    cube.CreateDisplayColorAttr([Gf.Vec3f(*rgb[:3])])
    if mat is not None:
        _bind(cube.GetPrim(), mat)
    if collider:
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    return cube


def _cylinder(stage, path, center, radius, height, rgb, mat=None):
    cyl = UsdGeom.Cylinder.Define(stage, path)
    cyl.CreateRadiusAttr(float(radius))
    cyl.CreateHeightAttr(float(height))
    cyl.CreateAxisAttr("Z")
    x = UsdGeom.Xformable(cyl.GetPrim())
    x.ClearXformOpOrder()
    x.AddTranslateOp().Set(Gf.Vec3d(*center))
    cyl.CreateDisplayColorAttr([Gf.Vec3f(*rgb[:3])])
    if mat is not None:
        _bind(cyl.GetPrim(), mat)
    return cyl


def _set_pose(prim, pos, quat_xyzw):
    """Author translate + orient on a prim, reusing the ops after the first call."""
    x = UsdGeom.Xformable(prim)
    ops = x.GetOrderedXformOps()
    if len(ops) != 2:
        x.ClearXformOpOrder()
        t_op = x.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
        o_op = x.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)
    else:
        t_op, o_op = ops
    t_op.Set(Gf.Vec3d(*pos))
    o_op.Set(Gf.Quatd(quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]))


# ----------------------------------------------------------------------------
class SearchMissionApp(PegasusApp):
    """PegasusApp + MTL search area + a native, ROS 2-driven gimbal per drone."""

    def __init__(self, *, repo, scenario, ground_truth, belief_png, drone_configs, **kwargs):
        self.repo = repo
        self.scenario = scenario
        self.ground_truth = ground_truth
        self.belief_png = belief_png
        self.gp = mcfg.gimbal_params(scenario)
        self.gimbals = []   # filled by spawn_drone, wired by post_spawn
        self._update_sub = None
        self._last_sim_t = None
        super().__init__(drone_configs=drone_configs, **kwargs)

    # ---------------------------------------------------------------- scene
    def post_scene_prep(self, stage):
        area = self.scenario["mission"]["area"]
        size = float(area["size_m"])
        cn, ce = (float(v) for v in area.get("center_ned", (0.0, 0.0)))
        cx, cy = ce, cn  # ENU centre
        render = self.scenario.get("airstack", {}).get("render", {})
        surround = float(render.get("surround_m", 150.0))
        ground_rgb = render.get("ground_rgba", [0.30, 0.38, 0.23, 1.0])
        UsdGeom.Xform.Define(stage, MTL_ROOT)

        # Collider slab under everything (the launch field sits outside the
        # Default Environment's small plane).
        span = size + 2.0 * surround
        ground_mat = _preview_material(stage, MTL_ROOT + "/Looks/Ground", ground_rgb)
        _box(stage, MTL_ROOT + "/Ground", (cx, cy, -0.105), (span, span, 0.2), ground_rgb,
             mat=ground_mat, collider=True)

        # Search plane draped with the prior. Row 0 of belief.png is the NORTH
        # edge, column 0 the WEST edge; UsdUVTexture st (0,0) = image bottom-left.
        half = size / 2.0
        plane = UsdGeom.Mesh.Define(stage, MTL_ROOT + "/SearchArea")
        z = 0.02
        plane.CreatePointsAttr([Gf.Vec3f(cx - half, cy - half, z), Gf.Vec3f(cx + half, cy - half, z),
                                Gf.Vec3f(cx + half, cy + half, z), Gf.Vec3f(cx - half, cy + half, z)])
        plane.CreateFaceVertexCountsAttr([4])
        plane.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
        plane.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
        plane.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
        st = UsdGeom.PrimvarsAPI(plane.GetPrim()).CreatePrimvar(
            "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
        st.Set([Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)])
        if self.belief_png is not None:
            _bind(plane.GetPrim(), _preview_material(stage, MTL_ROOT + "/Looks/Belief", (1, 1, 1),
                                                     texture=str(self.belief_png)))
        else:
            carb.log_warn("belief.png missing - search area drawn untextured")

        # Boundary bars
        bar_rgb = render.get("boundary_rgba", [1.0, 0.85, 0.1, 1.0])
        bar_mat = _preview_material(stage, MTL_ROOT + "/Looks/Boundary", bar_rgb, emissive=bar_rgb[:3])
        w, h = 0.6, 0.15
        for name, c, s in (("S", (cx, cy - half, h / 2), (size + w, w, h)),
                           ("N", (cx, cy + half, h / 2), (size + w, w, h)),
                           ("W", (cx - half, cy, h / 2), (w, size + w, h)),
                           ("E", (cx + half, cy, h / 2), (w, size + w, h))):
            _box(stage, f"{MTL_ROOT}/Boundary/{name}", c, s, bar_rgb, mat=bar_mat)

        # Ground-truth targets: pad + post + emissive top (visible from 30 m)
        tr = self.ground_truth.get("render", {})
        tsize = float(tr.get("size_m", 1.6))
        trgb = tr.get("rgba", [0.95, 0.12, 0.10, 1.0])
        tmat = _preview_material(stage, MTL_ROOT + "/Looks/Target", trgb, emissive=[0.6 * c for c in trgb[:3]])
        pad_mat = _preview_material(stage, MTL_ROOT + "/Looks/TargetPad", (0.95, 0.95, 0.95))
        for t in self.ground_truth.get("targets", []):
            x, y = float(t["e"]), float(t["n"])
            base = f"{MTL_ROOT}/Targets/target_{int(t['index']):02d}"
            UsdGeom.Xform.Define(stage, base)
            _cylinder(stage, base + "/pad", (x, y, 0.05), tsize * 0.75, 0.06, (0.95, 0.95, 0.95), pad_mat)
            _cylinder(stage, base + "/post", (x, y, 0.08 + tsize * 0.3), tsize * 0.5, tsize * 0.6, trgb, tmat)
        _log(f"search area {size:g} m at ENU ({cx:g}, {cy:g}), {len(self.ground_truth.get('targets', []))} "
             f"targets, belief texture {'on' if self.belief_png else 'OFF'}")

    # ---------------------------------------------------------------- drones
    def spawn_drone(self, cfg):
        handle = super().spawn_drone(cfg)
        if cfg.get("gimbal", True):
            self._author_gimbal(cfg)
        return handle

    def _author_gimbal(self, cfg):
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        i = cfg["domain_id"]
        robot = cfg.get("robot_name", f"robot_{i}")
        drone_prim = cfg.get("prim", f"/World/drone{i}/base_link")
        gimbal_path = f"{drone_prim}/{GIMBAL_NAME}"
        cam_path = f"{gimbal_path}/{CAMERA_NAME}"
        gx = UsdGeom.Xform.Define(stage, gimbal_path)
        cam = UsdGeom.Camera.Define(stage, cam_path)
        cam.CreateHorizontalApertureAttr(20.955)
        cam.CreateVerticalApertureAttr(20.955 * self.gp["height"] / self.gp["width"])
        cam.CreateFocalLengthAttr(mg.focal_length_mm(self.gp["fov_deg"], 20.955))
        cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, float(self.gp["max_range_m"])))
        _set_pose(cam.GetPrim(), (0.0, 0.0, 0.0), mg.USD_CAMERA_IN_GIMBAL_QUAT)

        s = self._position_scale
        root_pos = (cfg.get("x_m", 0.0) * s, cfg.get("y_m", 0.0) * s, cfg.get("z_m", 0.07) * s)
        ro = cfg.get("orient", [0.0, 0.0, 0.0, 1.0])
        root_quat = (float(ro[0]), float(ro[1]), float(ro[2]), float(ro[3]))
        lim = mg.GimbalLimits(roll_min=self.gp["roll_limit_rad"][0], roll_max=self.gp["roll_limit_rad"][1],
                              pitch_min=self.gp["pitch_limit_rad"][0], pitch_max=self.gp["pitch_limit_rad"][1],
                              slew_rate=self.gp["slew_rate_rad_s"])
        init_yaw = mg.euler_zyx_from_quat(root_quat)[2]
        g = {
            "robot": robot, "domain_id": i, "drone_prim": drone_prim, "gimbal_prim": gx.GetPrim(),
            "camera_path": cam_path, "root_pos": root_pos, "root_quat": root_quat,
            "axis": mg.GimbalAxis(lim, (0.0, self.gp["initial_pitch_rad"], init_yaw)),
            "graph": f"{MTL_ROOT}/Gimbal_{robot}", "vehicle": None, "have_cmd": False, "last_cmd": None,
        }
        # rest pose before Play: body frame = spawn root
        self._apply_gimbal_pose(g, root_pos, root_quat)
        self.gimbals.append(g)
        _log(f"{robot}: native gimbal at {gimbal_path} (FOV {self.gp['fov_deg']:g} deg, "
             f"{self.gp['width']}x{self.gp['height']})")

    # ---------------------------------------------------------------- ROS I/O
    def post_spawn(self, stage):
        render_hz = float(os.environ.get("MTL_GIMBAL_RENDER_HZ", "60") or 60.0)
        skip = max(0, int(round(render_hz / max(self.gp["publish_hz"], 1e-3))) - 1)
        for g in self.gimbals:
            self._build_gimbal_graph(g, skip)
        self._update_sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
            self._on_update, name="mtl_gimbal_update")
        _log(f"{len(self.gimbals)} gimbal graph(s) wired (camera frame skip {skip} at ~{render_hz:g} Hz render)")

    def _build_gimbal_graph(self, g, frame_skip):
        robot, gp = g["robot"], g["graph"]
        n = {
            "tick": f"{gp}/OnPlaybackTick", "ctx": f"{gp}/ROS2Context",
            "sub": f"{gp}/CmdSubscriber", "pub": f"{gp}/StatePublisher",
            "rp": f"{gp}/RenderProduct", "rgb": f"{gp}/RGB", "info": f"{gp}/CameraInfo",
        }
        keys = og.Controller.Keys
        og.Controller.edit(
            {"graph_path": gp, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
                    ("CmdSubscriber", "isaacsim.ros2.bridge.ROS2Subscriber"),
                    ("StatePublisher", "isaacsim.ros2.bridge.ROS2Publisher"),
                    ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                    ("RGB", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                    ("CameraInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                ],
                keys.SET_VALUES: [
                    ("ROS2Context.inputs:domain_id", int(g["domain_id"])),
                    ("CmdSubscriber.inputs:messagePackage", "geometry_msgs"),
                    ("CmdSubscriber.inputs:messageSubfolder", "msg"),
                    ("CmdSubscriber.inputs:messageName", "Vector3"),
                    ("CmdSubscriber.inputs:topicName", f"/{robot}/gimbal/cmd_pitch_yaw"),
                    ("StatePublisher.inputs:messagePackage", "geometry_msgs"),
                    ("StatePublisher.inputs:messageSubfolder", "msg"),
                    ("StatePublisher.inputs:messageName", "Vector3"),
                    ("StatePublisher.inputs:topicName", f"/{robot}/gimbal/state"),
                    ("RenderProduct.inputs:cameraPrim", g["camera_path"]),
                    ("RenderProduct.inputs:width", int(self.gp["width"])),
                    ("RenderProduct.inputs:height", int(self.gp["height"])),
                    ("RGB.inputs:type", "rgb"),
                    ("RGB.inputs:topicName", "gimbal/rgb"),
                    ("RGB.inputs:nodeNamespace", robot),
                    ("RGB.inputs:frameId", OPTICAL_FRAME),
                    ("RGB.inputs:frameSkipCount", int(frame_skip)),
                    ("CameraInfo.inputs:topicName", "gimbal/camera_info"),
                    ("CameraInfo.inputs:nodeNamespace", robot),
                    ("CameraInfo.inputs:frameId", OPTICAL_FRAME),
                    ("CameraInfo.inputs:frameSkipCount", int(frame_skip)),
                ],
                keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "CmdSubscriber.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "StatePublisher.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "RenderProduct.inputs:execIn"),
                    ("ROS2Context.outputs:context", "CmdSubscriber.inputs:context"),
                    ("ROS2Context.outputs:context", "StatePublisher.inputs:context"),
                    ("ROS2Context.outputs:context", "RGB.inputs:context"),
                    ("ROS2Context.outputs:context", "CameraInfo.inputs:context"),
                    ("RenderProduct.outputs:execOut", "RGB.inputs:execIn"),
                    ("RenderProduct.outputs:execOut", "CameraInfo.inputs:execIn"),
                    ("RenderProduct.outputs:renderProductPath", "RGB.inputs:renderProductPath"),
                    ("RenderProduct.outputs:renderProductPath", "CameraInfo.inputs:renderProductPath"),
                ],
            },
        )
        # Message-count latch: tells a real (0, 0, 0) command from "nothing received yet".
        g["counter"] = None
        try:
            og.Controller.edit(gp, {
                keys.CREATE_NODES: [("CmdCounter", "omni.graph.action.Counter")],
                keys.CONNECT: [("CmdSubscriber.outputs:execOut", "CmdCounter.inputs:execIn")],
            })
            g["counter"] = og.Controller.attribute(f"{gp}/CmdCounter.outputs:count")
        except Exception as exc:  # node library differences across Kit versions
            carb.log_warn(f"[{robot}] gimbal command counter unavailable ({exc}); "
                          "falling back to change detection")
        # Pin the context to this robot's domain even if ROS_DOMAIN_ID is set in
        # the Isaac container (the input only exists on newer bridges).
        try:
            og.Controller.set(og.Controller.attribute(f"{n['ctx']}.inputs:useDomainIDEnvVar"), False)
        except Exception:
            pass
        # ROS2Subscriber/Publisher fields are DYNAMIC attributes created once the
        # message type is known; resolve them lazily in the update loop.
        g["cmd_paths"] = [f"{n['sub']}.outputs:{c}" for c in ("x", "y", "z")]
        g["state_paths"] = [f"{n['pub']}.inputs:{c}" for c in ("x", "y", "z")]
        g["cmd_attrs"] = None
        g["state_attrs"] = None

    @staticmethod
    def _resolve(paths):
        try:
            attrs = [og.Controller.attribute(p) for p in paths]
            return attrs if all(a is not None and a.is_valid() for a in attrs) else None
        except Exception:
            return None

    # ---------------------------------------------------------------- per frame
    def _vehicle_for(self, g):
        if g["vehicle"] is not None:
            return g["vehicle"]
        try:
            from pegasus.simulator.logic.vehicle_manager import VehicleManager
            # OGN-spawned vehicles register under the full drone prim path; UI
            # spawns may use the parent (same rule as the follow-cam).
            target = g["drone_prim"].rstrip("/")
            parent = target.rsplit("/", 1)[0]
            for prefix, vehicle in VehicleManager.get_vehicle_manager().vehicles.items():
                sp = prefix.rstrip("/")
                if sp in (target, parent) or sp.startswith(parent + "/"):
                    g["vehicle"] = vehicle
                    _log(f"{g['robot']}: gimbal tracking Pegasus vehicle '{prefix}'")
                    return vehicle
        except Exception:
            pass
        return None

    def _read_command(self, g):
        if g["cmd_attrs"] is None:
            g["cmd_attrs"] = self._resolve(g["cmd_paths"])
            if g["cmd_attrs"] is None:
                return None
        try:
            vals = tuple(float(og.Controller.get(a)) for a in g["cmd_attrs"])
        except Exception:
            return None
        received = False
        if g["counter"] is not None:
            try:
                received = int(og.Controller.get(g["counter"])) > 0
            except Exception:
                received = False
        if not received and vals != (0.0, 0.0, 0.0):
            received = True  # change detection fallback
        if received:
            g["have_cmd"] = True
        return vals if g["have_cmd"] else None

    def _apply_gimbal_pose(self, g, body_pos, body_quat):
        pos, quat = mg.gimbal_world_pose(body_pos, body_quat, self.gp["mount_offset_m"], g["axis"].state)
        lp, lq = mg.relative_pose(g["root_pos"], g["root_quat"], pos, quat)
        _set_pose(g["gimbal_prim"], lp, lq)

    def _on_update(self, _event):
        try:
            sim_t = float(self.timeline.get_current_time())
        except Exception:
            return
        dt = 0.0 if self._last_sim_t is None else max(0.0, sim_t - self._last_sim_t)
        if self._last_sim_t is not None and sim_t < self._last_sim_t:
            dt = 0.0  # timeline stopped / rewound
        self._last_sim_t = sim_t
        for g in self.gimbals:
            try:
                vehicle = self._vehicle_for(g)
                if vehicle is not None:
                    st = vehicle.state
                    body_pos = (float(st.position[0]), float(st.position[1]), float(st.position[2]))
                    q = st.attitude
                    body_quat = (float(q[0]), float(q[1]), float(q[2]), float(q[3]))
                else:
                    body_pos, body_quat = g["root_pos"], g["root_quat"]
                cmd = self._read_command(g)
                if cmd is None:  # park: initial pitch, along the airframe heading
                    cmd = (0.0, self.gp["initial_pitch_rad"], mg.euler_zyx_from_quat(body_quat)[2])
                g["axis"].step(cmd, dt)
                self._apply_gimbal_pose(g, body_pos, body_quat)
                if g["state_attrs"] is None:
                    g["state_attrs"] = self._resolve(g["state_paths"])
                for attr, v in zip(g["state_attrs"] or (), g["axis"].state):
                    og.Controller.set(attr, float(v))
            except Exception as exc:
                if not g.get("warned"):
                    g["warned"] = True
                    carb.log_warn(f"[{g['robot']}] gimbal update failed: {exc}")


def main():
    repo = mcfg.repo_root(_HERE)
    bundle_dir = mcfg.scenario_dir(repo)
    scenario, ground_truth, belief_png = mcfg.load_bundle(bundle_dir)

    fleet_env = os.environ.get("FLEET_CONFIG_FILE", "").strip()
    if fleet_env:
        fleet_path = mcfg.remap_path(fleet_env, repo)
        drone_configs = mcfg.drone_configs_from_fleet(Path(fleet_path), repo)
        source = f"fleet {os.path.basename(fleet_path)}"
    else:
        drone_configs = mcfg.drone_configs_from_scenario(scenario)
        source = "scenario team homes"
        n_env = int(os.environ.get("NUM_ROBOTS", "0") or 0)
        if n_env and n_env != len(drone_configs):
            carb.log_warn(f"NUM_ROBOTS={n_env} but the scenario team has {len(drone_configs)} agents - "
                          "spawning the scenario team")
    for problem in mcfg.spawn_mismatches(drone_configs, scenario):
        carb.log_warn(f"[search_mission_scene] {problem}")
    _log(f"{bundle_dir}: {scenario['mission']['name']}; spawning {len(drone_configs)} drone(s) from {source}: "
         + ", ".join(f"{c['robot_name']}@({c['x_m']:g},{c['y_m']:g})"
                     f"{' +gimbal' if c.get('gimbal') else ''}{' +zed' if c.get('camera') else ''}"
                     f"{' +lidar' if c.get('lidar') else ''}" for c in drone_configs))

    # A wide, high chase view suits a 400 m search area (overridable).
    os.environ.setdefault("ISAAC_SIM_FOLLOW_CAM_OFFSET", "-14,-14,9")

    SearchMissionApp(
        repo=repo, scenario=scenario, ground_truth=ground_truth, belief_png=belief_png,
        env_url=SIMULATION_ENVIRONMENTS["Default Environment"],
        drone_configs=drone_configs,
        enable_camera=False,   # per-drone "camera" (ZED) comes from the vehicle manifest
        enable_lidar=False,    # per-drone "lidar" likewise
        # Always write PX4 GPS homes (set_gps_origins), also for a single drone,
        # so the GCS map and PX4 share the scenario's world frame.
        world_gps_origin=DEFAULT_WORLD_ORIGIN,
    ).run()


if __name__ == "__main__":
    main()
