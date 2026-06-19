#!/usr/bin/env python
"""
Config-driven person-tracking scene loader.

Loads an arbitrary USD scene, spawns a Pegasus PX4 Iris drone, adds a textured
USD patch on a target cube, and spawns a walking person -- but every
scene-specific value (scene USD, person position/target, patch position ranges,
patch size ranges, RNG seeds, drone settings, camera) is read from a YAML config
file instead of being hardcoded. Copy the config to describe a new scene with a
different USD file.

Config file:
    Path is taken from SCENE_CONFIG_PATH (default:
    .../launch_scripts/configs/office_one_drone_one_person.yaml). Because the repo is
    mounted into the Isaac Sim container at /isaac-sim/AirStack, use the
    in-container path when setting SCENE_CONFIG_PATH.
    See configs/office_one_drone_one_person.yaml for the schema.

Scene units:
    If the scene USD is authored in different units than the meter-scale Pegasus
    drone/person (e.g. a centimeter scene), set scene.scale (e.g. 0.01) to rescale
    the loaded scene. Works in both load modes (in sublayer mode the scale is
    authored as an override on the scene's root prim). All positions in the
    config must then be given in the scaled (post-scale) frame.

Env overrides (optional, win over the config file when set):
    SCENE_USD_PATH / ISAAC_SIM_GUI   fallback scene USD when scene.usd_path is empty
    MESH_PLANE_SIZE_SEED             overrides patch.size_seed
    MESH_PLANE_POSITION_SEED         overrides patch.position_seed
    PLAY_SIM_ON_START                overrides scene.play_sim_on_start

Streaming:
    Set ISAAC_SIM_HEADLESS=true to run headless with WebRTC streaming on port 49100.
"""

import os
import time

import carb

from isaacsim import SimulationApp

_HEADLESS = os.environ.get("ISAAC_SIM_HEADLESS", "false").lower() == "true"

simulation_app = SimulationApp({"headless": _HEADLESS, "hide_ui": False})

import omni.kit.app
import omni.timeline
import omni.usd
import numpy as np
import yaml
from isaacsim.core.api.materials.omni_pbr import OmniPBR
from isaacsim.core.utils.semantics import add_update_semantics
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.viewports import set_camera_view
from omni.isaac.core.world import World
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

if _HEADLESS:
    simulation_app.set_setting("/app/window/drawMouse", True)
    enable_extension("omni.kit.livestream.webrtc")


# ---------------------------------------------------------------------------
# Config loading
# ---------------------------------------------------------------------------
_DEFAULT_CONFIG_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "configs",
    "office_one_drone_one_person.yaml",
)


def _get(mapping, key, default=None):
    """Return mapping[key], treating a present-but-null value as the default."""
    if not isinstance(mapping, dict):
        return default
    value = mapping.get(key, default)
    return default if value is None else value


def load_config():
    config_path = os.path.expanduser(
        os.environ.get("SCENE_CONFIG_PATH", _DEFAULT_CONFIG_PATH)
    )
    if not os.path.isfile(config_path):
        raise RuntimeError(
            f"Scene config not found: {config_path}. Set SCENE_CONFIG_PATH to a "
            "YAML file (see launch_scripts/configs/office_one_drone_one_person.yaml)."
        )
    with open(config_path, "r") as handle:
        config = yaml.safe_load(handle) or {}
    carb.log_warn(f"Loaded scene config: {config_path}")
    return config


PEOPLE_EXTENSIONS = [
    "omni.graph.core",
    "omni.graph.action",
    "omni.graph.action_nodes",
    "isaacsim.core.nodes",
    "isaacsim.ros2.bridge",
    "omni.graph.ui",
    "omni.graph.visualization.nodes",
    "omni.graph.scriptnode",
    "omni.graph.window.action",
    "omni.graph.window.generic",
    "omni.graph.ui_nodes",
    "omni.isaac.core",
    "omni.anim.people",
    "omni.anim.navigation.bundle",
    "omni.anim.timeline",
    "omni.anim.graph.bundle",
    "omni.anim.graph.core",
    "omni.anim.graph.ui",
    "omni.anim.retarget.bundle",
    "omni.anim.retarget.core",
    "omni.anim.retarget.ui",
    "omni.kit.scripting",
    "pegasus.simulator",
]


class StraightLinePersonController:
    def __init__(self, target, walk_speed, start_delay=0.0, arrival_threshold=0.2):
        self._person = None
        self._target = list(target)
        self._walk_speed = walk_speed
        self._start_delay = max(0.0, float(start_delay))
        # Distance (in stage units) at which the person is considered to have
        # arrived. omni.anim.people's own arrival logic is tuned for meters; in a
        # non-meter (e.g. centimeter) scene that threshold is effectively
        # unreachable, so the character oscillates / "wanders" around the goal
        # forever. Once within this threshold we stop commanding motion so it
        # settles into idle instead. Size it to the scene units.
        self._arrival_threshold = max(0.0, float(arrival_threshold))
        self._elapsed = 0.0
        self._arrived = False

    def initialize(self, person):
        self._person = person

    def update(self, dt):
        if self._person is None:
            return
        self._elapsed += dt
        if self._elapsed < self._start_delay:
            self._person.update_target_position(self._person.state.position, 0.0)
            return
        # Latch arrival so the character holds position and idles cleanly rather
        # than fighting an arrival threshold it cannot satisfy at this scale.
        if not self._arrived:
            position = self._person.state.position
            distance = float(
                np.linalg.norm(np.asarray(self._target) - np.asarray(position))
            )
            if distance <= self._arrival_threshold:
                self._arrived = True
        if self._arrived:
            self._person.update_target_position(self._person.state.position, 0.0)
            return
        self._person.update_target_position(self._target, self._walk_speed)

    def update_state(self, state):
        pass

    def start(self):
        pass

    def stop(self):
        pass

    def reset(self):
        pass


def sample_patch_on_cube_negative_x_face(patch_cfg):
    """Sample a square patch center + size on the -X face of the target cube.

    Y/Z center ranges come from patch.y_range / patch.z_range when provided,
    otherwise they are derived from the cube bounds (inset by half the patch
    size). Size is sampled uniformly in patch.size_range. Seeds are taken from
    the config but can be overridden by the MESH_PLANE_*_SEED env vars.
    """
    size_seed = int(os.environ.get(
        "MESH_PLANE_SIZE_SEED",
        str(_get(patch_cfg, "size_seed", 1234)),
    ))
    position_seed = int(os.environ.get(
        "MESH_PLANE_POSITION_SEED",
        str(_get(patch_cfg, "position_seed", 5678)),
    ))
    size_rng = np.random.default_rng(size_seed)
    position_rng = np.random.default_rng(position_seed)

    min_size, max_size = [float(value) for value in _get(patch_cfg, "size_range", [0.5, 1.0])]
    patch_size = float(size_rng.uniform(min_size, max_size))
    half_patch = patch_size * 0.5

    cube_center = [float(value) for value in _get(patch_cfg, "cube_center")]
    cube_half_extent = [float(value) * 0.5 for value in _get(patch_cfg, "cube_scale")]
    bounds_min = [cube_center[index] - cube_half_extent[index] for index in range(3)]
    bounds_max = [cube_center[index] + cube_half_extent[index] for index in range(3)]

    # Patch center sampling ranges: explicit overrides or derived from cube face.
    y_range = _get(patch_cfg, "y_range")
    z_range = _get(patch_cfg, "z_range")
    y_lo, y_hi = (
        [float(v) for v in y_range]
        if y_range is not None
        else (bounds_min[1] + half_patch, bounds_max[1] - half_patch)
    )
    z_lo, z_hi = (
        [float(v) for v in z_range]
        if z_range is not None
        else (bounds_min[2] + half_patch, bounds_max[2] - half_patch)
    )
    if y_hi < y_lo or z_hi < z_lo:
        raise RuntimeError(
            f"Sampled patch size {patch_size:.3f} does not fit within the "
            f"{_get(patch_cfg, 'target_prim_name', 'cube')} -X face sampling "
            f"ranges: y=[{y_lo}, {y_hi}], z=[{z_lo}, {z_hi}]."
        )

    x = bounds_min[0] - float(_get(patch_cfg, "negative_x_face_offset", 0.0))
    y = float(position_rng.uniform(y_lo, y_hi))
    z = float(position_rng.uniform(z_lo, z_hi))

    return {
        "center": [x, y, z],
        "size": [patch_size, patch_size],
        "bounds_min": bounds_min,
        "bounds_max": bounds_max,
        "size_seed": size_seed,
        "position_seed": position_seed,
    }


def wait_for_stage(timeout_s: float = 20.0):
    app = omni.kit.app.get_app()
    deadline = time.time() + timeout_s

    while time.time() < deadline:
        app.update()
        stage = omni.usd.get_context().get_stage()
        if stage is not None and stage.GetPseudoRoot().IsValid():
            return stage
        time.sleep(0.1)

    raise RuntimeError("Timed out waiting for USD stage to load")


def wait_for_stage_updates(frame_count: int = 30):
    for _ in range(frame_count):
        simulation_app.update()


def count_lights(stage):
    light_type_names = {
        "DomeLight",
        "DistantLight",
        "DiskLight",
        "RectLight",
        "SphereLight",
        "CylinderLight",
    }
    return sum(
        1
        for prim in stage.Traverse()
        if prim.GetTypeName() in light_type_names
    )


def apply_uniform_scale(stage, prim_path, scale):
    """Apply a uniform scale to an existing xformable prim without clobbering
    any translate/rotate ops the loaded asset already carries.

    Works whether the prim is locally defined (reference mode) or composed in
    from a sublayer -- in the latter case this authors an override scale op in
    the current (root) edit layer on top of the sublayer's opinion.
    """
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        raise RuntimeError(f"Cannot scale missing prim: {prim_path}")
    xformable = UsdGeom.Xformable(prim)
    scale_op = None
    for op in xformable.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeScale:
            scale_op = op
            break
    if scale_op is None:
        # AddScaleOp appends to xformOpOrder; keep the existing ops (e.g. a
        # translate/rotate from the sublayer) ahead of the new scale.
        scale_op = xformable.AddScaleOp()
    scale_op.Set(Gf.Vec3d(float(scale), float(scale), float(scale)))


def find_sublayer_scene_roots(stage, scene_usd_path, pre_existing_top_level):
    """Identify the root prim(s) of a scene that was composed in as a sublayer.

    A sublayer is merged into the root, so there is no single wrapper prim. We
    prefer the sublayer's defaultPrim, otherwise any top-level prim that appeared
    after sublayering.
    """
    layer = Sdf.Layer.FindOrOpen(scene_usd_path)
    if layer is not None and layer.defaultPrim:
        return [Sdf.Path.absoluteRootPath.AppendChild(layer.defaultPrim).pathString]
    roots = [
        child.GetPath().pathString
        for child in stage.GetPseudoRoot().GetChildren()
        if child.GetPath().pathString not in pre_existing_top_level
    ]
    if not roots:
        raise RuntimeError(
            f"Could not identify the sublayered scene root: {scene_usd_path}. "
            "Set a defaultPrim on the scene USD, or use scene.load_mode: reference."
        )
    return roots


def add_collision_to_subtree(stage, root_path, approximation="none"):
    """Apply a static collider to every Mesh under root_path.

    Many environment USDs ship as visual-only meshes with no PhysX colliders, so
    a dynamic body (the drone) falls straight through. This applies CollisionAPI
    (static -- no RigidBodyAPI) to each mesh. 'none' = exact triangle mesh, which
    is correct for static geometry; 'convexHull'/'convexDecomposition' are cheaper
    but approximate.

    Instanced meshes (instance proxies) are read-only and cannot carry authored
    colliders, so they are counted and skipped. Returns (applied, skipped_instanced).
    """
    root = stage.GetPrimAtPath(root_path)
    if not root or not root.IsValid():
        return 0, 0
    applied = 0
    skipped_instanced = 0
    # Traverse into instance proxies so we can *see* (and report) instanced
    # meshes even though we cannot author colliders onto them.
    for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        if prim.IsInstanceProxy():
            skipped_instanced += 1
            continue
        UsdPhysics.CollisionAPI.Apply(prim)
        mesh_collision = UsdPhysics.MeshCollisionAPI.Apply(prim)
        mesh_collision.CreateApproximationAttr().Set(approximation)
        applied += 1
    return applied, skipped_instanced


def sublayer_scene(stage, scene_usd_path):
    root_layer = stage.GetRootLayer()
    if scene_usd_path not in root_layer.subLayerPaths:
        root_layer.subLayerPaths.append(scene_usd_path)
    wait_for_stage_updates()


def create_textured_mesh_plane(
    stage,
    prim_path,
    material_path,
    center,
    orientation_deg,
    size,
    albedo_path,
    texture_scale,
):
    half_x = float(size[0]) * 0.5
    half_y = float(size[1]) * 0.5
    center = [float(value) for value in center]
    orientation_deg = [float(value) for value in orientation_deg]

    mesh = UsdGeom.Mesh.Define(stage, prim_path)
    mesh.CreatePointsAttr(
        [
            Gf.Vec3f(-half_x, -half_y, 0.0),
            Gf.Vec3f(half_x, -half_y, 0.0),
            Gf.Vec3f(half_x, half_y, 0.0),
            Gf.Vec3f(-half_x, half_y, 0.0),
        ]
    )
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateDoubleSidedAttr(True)
    mesh.CreateNormalsAttr([Gf.Vec3f(0.0, 0.0, 1.0)] * 4)
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)

    xformable = UsdGeom.Xformable(mesh.GetPrim())
    xformable.ClearXformOpOrder()
    xformable.AddTranslateOp().Set(Gf.Vec3d(*center))
    xformable.AddRotateXYZOp().Set(Gf.Vec3f(*orientation_deg))

    # UV corners span the whole quad, so one albedo image fills the patch.
    primvars = UsdGeom.PrimvarsAPI(mesh.GetPrim())
    st = primvars.CreatePrimvar(
        "st",
        Sdf.ValueTypeNames.TexCoord2fArray,
        UsdGeom.Tokens.faceVarying,
    )
    # UVs are remapped (rotated 90 deg vs. the trivial corner order) so the
    # texture reads upright from the drone's view. The quad is authored in local
    # XY then rotated [0, 90, 0] onto the cube's -X face; that rotation sends
    # local X (U) to world -Z and local Y (V) to world +Y, which renders the
    # image rotated 90 deg to the left. Assigning U along world -Y (screen
    # right) and V along world +Z (screen up) corrects the orientation.
    st.Set(
        [
            Gf.Vec2f(1.0, 1.0),
            Gf.Vec2f(1.0, 0.0),
            Gf.Vec2f(0.0, 0.0),
            Gf.Vec2f(0.0, 1.0),
        ]
    )
    st.SetIndices([0, 1, 2, 3])

    material_kwargs = {
        "prim_path": material_path,
        "name": "track_scene_patch_omnipbr",
        "color": np.array([2.0, 2.0, 2.0]),
        "texture_scale": [1.0, 1.0],
        "texture_translate": [0.0, 0.0],
    }
    if albedo_path:
        material_kwargs["texture_path"] = albedo_path
    OmniPBR(**material_kwargs)

    material = UsdShade.Material.Get(stage, material_path)
    material_prim = material.GetPrim()
    for child in material_prim.GetChildren():
        if child.GetTypeName() != "Shader":
            continue

        shader = UsdShade.Shader(child)
        shader.CreateInput("project_uvw", Sdf.ValueTypeNames.Bool).Set(False)
        shader.CreateInput("world_or_object", Sdf.ValueTypeNames.Bool).Set(False)
        shader.CreateInput("uv_space_index", Sdf.ValueTypeNames.Int).Set(0)
        shader.CreateInput("texture_scale", Sdf.ValueTypeNames.Float2).Set(Gf.Vec2f(*texture_scale))
        shader.CreateInput("texture_translate", Sdf.ValueTypeNames.Float2).Set(Gf.Vec2f(0.0, 0.0))
        shader.CreateInput("texture_rotate", Sdf.ValueTypeNames.Float).Set(0.0)

    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)
    return mesh


class ConfigurableTrackScene:
    def __init__(self, config):
        self.config = config
        scene_cfg = _get(config, "scene", {})
        person_cfg = _get(config, "person", {})
        camera_cfg = _get(config, "camera", {})

        scene_usd_path = (
            _get(scene_cfg, "usd_path", "")
            or os.environ.get("SCENE_USD_PATH", "")
            or os.environ.get("ISAAC_SIM_GUI", "")
        )
        self.scene_usd_path = os.path.expanduser(scene_usd_path)
        self.scene_load_mode = str(_get(scene_cfg, "load_mode", "sublayer")).strip().lower()
        # Uniform scale applied to the loaded scene. Use to reconcile a scene
        # authored in different units (e.g. 0.01 turns a centimeter scene into
        # meters so the meter-scale drone/person match). Actor positions in this
        # config are world-space and must be expressed in the *scaled* units.
        self.scene_scale = float(_get(scene_cfg, "scale", 1.0))
        # Add a static collider to every mesh in the loaded scene. Use when the
        # scene asset has no PhysX colliders and the drone falls through it.
        self.scene_add_collision = bool(_get(scene_cfg, "add_collision", False))
        self.scene_collision_approximation = str(
            _get(scene_cfg, "collision_approximation", "none")
        )
        if not self.scene_usd_path:
            raise RuntimeError(
                "Set scene.usd_path in the config (or SCENE_USD_PATH / ISAAC_SIM_GUI)."
            )
        if self.scene_load_mode not in ("sublayer", "reference"):
            raise RuntimeError("scene.load_mode must be 'sublayer' or 'reference'.")

        self.person_name = _get(person_cfg, "name", "person1")
        self.person_character = _get(person_cfg, "character", "original_male_adult_construction_05")
        self.person_start = [float(v) for v in _get(person_cfg, "start", [0.0, 0.0, 0.0])]
        self.person_target = [float(v) for v in _get(person_cfg, "target", [5.0, 0.0, 0.0])]
        self.person_yaw = float(_get(person_cfg, "yaw", 0.0))
        self.person_walk_speed = float(_get(person_cfg, "walk_speed", 1.0))
        self.person_start_delay = float(_get(person_cfg, "start_delay", 0.0))
        # Distance (stage units) at which the person stops at its target. Size to
        # the scene units (e.g. ~20 for a centimeter scene) to avoid wandering.
        self.person_arrival_threshold = float(_get(person_cfg, "arrival_threshold", 0.2))
        # Uniform scale for the spawned character. Use to match a scene authored
        # in non-meter units (e.g. 100 for a centimeter scene). Visual/navmesh
        # only -- safe to scale.
        self.person_scale = float(_get(person_cfg, "scale", 1.0))

        self.camera_eye = [float(v) for v in _get(camera_cfg, "eye", [8.0, 8.0, 5.0])]
        camera_target = _get(camera_cfg, "target")
        self.camera_target = (
            [float(v) for v in camera_target] if camera_target is not None else self.person_start
        )

        for extension in PEOPLE_EXTENSIONS:
            enable_extension(extension)
            simulation_app.update()

        # Import Pegasus people classes only after the animation/people extensions
        # are enabled. The Person module imports replicator-agent/anim APIs.
        from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
        from pegasus.simulator.logic.people.person import Person
        from pegasus.simulator.ogn.api.spawn_multirotor import spawn_px4_multirotor_node
        from pegasus.simulator.ogn.api.spawn_rtx_lidar import add_rtx_lidar_subgraph
        from pegasus.simulator.ogn.api.spawn_zed_camera import add_zed_stereo_camera_subgraph

        # Use a fresh writable stage and compose the user's scene into it. Opening
        # a Nucleus scene as the root layer can make people/animation commands
        # edit a generated metrics layer, which has crashed Kit in Isaac Sim 5.1.
        omni.usd.get_context().new_stage()
        self.stage = wait_for_stage()
        self.pg = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        if self.scene_load_mode == "sublayer":
            carb.log_warn(f"Sublayering scene USD to preserve authored lighting: {self.scene_usd_path}")
            pre_existing_top_level = {
                child.GetPath().pathString
                for child in self.stage.GetPseudoRoot().GetChildren()
            }
            sublayer_scene(self.stage, self.scene_usd_path)
            scene_roots = find_sublayer_scene_roots(
                self.stage, self.scene_usd_path, pre_existing_top_level
            )
        else:
            carb.log_warn(f"Referencing scene USD at /World/stage: {self.scene_usd_path}")
            self.pg.load_asset(self.scene_usd_path, "/World/stage")
            wait_for_stage_updates()
            scene_roots = ["/World/stage"]

        if self.scene_scale != 1.0:
            for scene_root in scene_roots:
                carb.log_warn(f"Scaling scene root {scene_root} by {self.scene_scale}.")
                apply_uniform_scale(self.stage, scene_root, self.scene_scale)
            wait_for_stage_updates()

        if self.scene_add_collision:
            applied = 0
            skipped_instanced = 0
            for scene_root in scene_roots:
                root_applied, root_skipped = add_collision_to_subtree(
                    self.stage, scene_root, self.scene_collision_approximation
                )
                applied += root_applied
                skipped_instanced += root_skipped
            carb.log_warn(
                f"Added '{self.scene_collision_approximation}' colliders to {applied} "
                f"mesh prim(s) across {scene_roots}."
            )
            if skipped_instanced:
                carb.log_warn(
                    f"Skipped {skipped_instanced} INSTANCED mesh(es) -- instance proxies "
                    "cannot carry authored colliders."
                )
            wait_for_stage_updates()

        light_count = count_lights(self.stage)
        carb.log_warn(f"Scene contains {light_count} USD light prim(s) after loading.")
        if light_count == 0:
            carb.log_warn("No authored lights found in the composed scene; the viewport may look dark.")

        self._maybe_create_patch()

        self.timeline = omni.timeline.get_timeline_interface()
        self.timeline.stop()

        self._maybe_spawn_drone(
            spawn_px4_multirotor_node,
            add_zed_stereo_camera_subgraph,
            add_rtx_lidar_subgraph,
        )

        carb.log_warn(
            f"Spawning person '{self.person_name}' using character "
            f"'{self.person_character}' at {self.person_start}"
        )
        self.person_controller = StraightLinePersonController(
            self.person_target,
            self.person_walk_speed,
            self.person_start_delay,
            self.person_arrival_threshold,
        )
        self.person = Person(
            self.person_name,
            self.person_character,
            init_pos=self.person_start,
            init_yaw=self.person_yaw,
            controller=self.person_controller,
        )
        add_update_semantics(self.person.character_skel_root, "person")
        if self.person_scale != 1.0:
            # Scale the spawned character's root xform about its spawn point. The
            # translate op carrying init_pos is preserved, so the person stays put
            # but renders person_scale times larger.
            carb.log_warn(
                f"Scaling person '{self.person_name}' by {self.person_scale} "
                f"at {self.person._stage_prefix}"
            )
            apply_uniform_scale(self.stage, self.person._stage_prefix, self.person_scale)
        self.person.update_target_position(self.person_start, 0.0)
        set_camera_view(eye=self.camera_eye, target=self.camera_target)

        self.person.update_target_position(self.person_start, 0.0)
        play_default = str(_get(scene_cfg, "play_sim_on_start", False)).lower() == "true"
        self.play_on_start = os.environ.get(
            "PLAY_SIM_ON_START", str(play_default)
        ).lower() == "true"

    def _maybe_create_patch(self):
        patch_cfg = _get(self.config, "patch", {})
        if not bool(_get(patch_cfg, "enabled", False)):
            return

        albedo_path = os.environ.get(
            "MESH_PLANE_ALBEDO_PATH",
            _get(patch_cfg, "albedo_path", ""),
        )
        sample = sample_patch_on_cube_negative_x_face(patch_cfg)
        orientation_deg = _get(patch_cfg, "orientation_deg", [0.0, 90.0, 0.0])
        texture_scale = _get(patch_cfg, "texture_scale", [1.0, 1.0])
        prim_path = _get(patch_cfg, "prim_path", "/World/EvalPatch/TexturedMeshPlane")
        material_path = _get(patch_cfg, "material_path", "/World/Looks/EvalPatchOmniPBR")
        carb.log_warn(
            f"Creating mesh plane '{prim_path}' with OmniPBR albedo map: "
            f"{albedo_path or '<none>'}; target="
            f"{_get(patch_cfg, 'target_prim_name', 'cube')}, "
            f"bounds_min={sample['bounds_min']}, bounds_max={sample['bounds_max']}, "
            f"center={sample['center']}, orientation_deg={orientation_deg}, "
            f"size={sample['size']}, size_seed={sample['size_seed']}, "
            f"position_seed={sample['position_seed']}"
        )
        mesh_plane = create_textured_mesh_plane(
            self.stage,
            prim_path,
            material_path,
            sample["center"],
            orientation_deg,
            sample["size"],
            albedo_path,
            texture_scale,
        )
        add_update_semantics(mesh_plane.GetPrim(), "patch_plane")

    def _maybe_spawn_drone(
        self,
        spawn_px4_multirotor_node,
        add_zed_stereo_camera_subgraph,
        add_rtx_lidar_subgraph,
    ):
        drone_cfg = _get(self.config, "drone", {})
        if not bool(_get(drone_cfg, "enabled", False)):
            return

        drone_prim = _get(drone_cfg, "prim", "/World/spawned_iris/base_link")
        drone_robot_name = _get(drone_cfg, "robot_name", "robot_1")
        drone_vehicle_id = int(_get(drone_cfg, "vehicle_id", 1))
        drone_domain_id = int(_get(drone_cfg, "domain_id", 1))
        drone_usd = os.path.expanduser(_get(drone_cfg, "usd", ""))
        drone_init_pos = [float(v) for v in _get(drone_cfg, "init_pos", [0.0, 0.0, 0.07])]
        drone_init_orient = [float(v) for v in _get(drone_cfg, "init_orient", [0.0, 0.0, 0.0, 1.0])]

        carb.log_warn(
            f"Spawning Pegasus PX4 Iris '{drone_robot_name}' at {drone_init_pos} "
            f"on prim '{drone_prim}' using {drone_usd}"
        )
        graph_handle = spawn_px4_multirotor_node(
            pegasus_node_name="PX4Multirotor_TrackScene",
            drone_prim=drone_prim,
            robot_name=drone_robot_name,
            vehicle_id=drone_vehicle_id,
            domain_id=drone_domain_id,
            usd_file=drone_usd,
            init_pos=drone_init_pos,
            init_orient=drone_init_orient,
        )

        if bool(_get(drone_cfg, "enable_zed", False)):
            add_zed_stereo_camera_subgraph(
                parent_graph_handle=graph_handle,
                drone_prim=drone_prim,
                robot_name=drone_robot_name,
                camera_name="ZEDCamera",
                camera_offset=[0.2, 0.0, -0.05],
                camera_rotation_offset=[0.0, 0.0, 0.0],
                enable_instance_segmentation=bool(_get(drone_cfg, "enable_zed_instance_segmentation", False)),
                enable_semantic_segmentation=bool(_get(drone_cfg, "enable_zed_semantic_segmentation", False)),
                enable_bbox_2d=bool(_get(drone_cfg, "enable_zed_bbox_2d", False)),
            )

        if bool(_get(drone_cfg, "enable_ouster", False)):
            add_rtx_lidar_subgraph(
                parent_graph_handle=graph_handle,
                drone_prim=drone_prim,
                robot_name=drone_robot_name,
                lidar_name="OusterOS1",
                lidar_config="ouster_os1",
                lidar_offset=[0.0, 0.0, 0.025],
                lidar_rotation_offset=[0.0, 0.0, 0.0],
                min_range=0.75,
            )

        drone_scale = float(_get(drone_cfg, "scale", 1.0))
        if drone_scale != 1.0:
            # Scale the drone xform (and its attached sensors) about its spawn
            # point. NOTE: the Iris is a PhysX/PX4 vehicle -- scaling the visual
            # mesh does NOT rescale its mass/inertia or the PX4 flight dynamics,
            # so a scaled drone will look right but its free-flight behavior will
            # be off. Use only when the drone is a (mostly) static camera
            # platform; keep drone.scale = 1.0 if it must fly realistically.
            carb.log_warn(
                f"Scaling drone '{drone_robot_name}' by {drone_scale} at "
                f"{drone_prim}. Flight dynamics are NOT rescaled (visual only)."
            )
            apply_uniform_scale(self.stage, drone_prim, drone_scale)

    def run(self):
        if self.play_on_start:
            self.timeline.play()

        app = omni.kit.app.get_app()
        while simulation_app.is_running():
            if self.timeline.is_playing():
                world = World.instance()
                if world is not None and hasattr(world, "_scene"):
                    world.step(render=True)
                    if world is not self.world:
                        self.world = world
                        self.pg._world = world
                else:
                    app.update()
            else:
                app.update()


def main():
    try:
        config = load_config()
        ConfigurableTrackScene(config).run()
    except Exception as exc:
        carb.log_error(str(exc))
        simulation_app.close()
        raise


if __name__ == "__main__":
    main()
