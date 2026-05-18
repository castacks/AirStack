#!/usr/bin/env python
"""
Load the CyLab hallway evaluation scene, spawn a Pegasus PX4 Iris drone using
the same OmniGraph helper as example_one_px4_pegasus_launch_script.py, add a
textured USD mesh plane, and spawn a built-in Pegasus/NVIDIA person that walks.

Required:
    Set HARDCODED_SCENE_USD_PATH below, or use SCENE_USD_PATH=/path/to/scene.usd.
    If neither is set, the script uses AirStack's ISAAC_SIM_GUI value.

Optional:
    SCENE_LOAD_MODE=sublayer
    PERSON_NAME=person1
    PERSON_CHARACTER=original_male_adult_construction_05
    PERSON_START="0,0,0"
    PERSON_TARGET="5,0,0"
    PERSON_YAW=0.0
    PERSON_WALK_SPEED=1.0
    DRONE_ENABLED=true
    DRONE_INIT_POS="-6.8,-6.3,0.07"
    DRONE_VEHICLE_ID=1
    DRONE_DOMAIN_ID=1
    DRONE_ENABLE_ZED=true
    DRONE_ENABLE_OUSTER=true
    MESH_PLANE_ALBEDO_PATH=/path/to/albedo.png

Streaming:
    Set ISAAC_SIM_HEADLESS=true to run headless with WebRTC streaming on port 49100.
    Connect via the isaac-sim-webrtc-viewer browser UI.
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
from isaacsim.core.api.materials.omni_pbr import OmniPBR
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.viewports import set_camera_view
from omni.isaac.core.world import World
from pxr import Gf, Sdf, UsdGeom, UsdShade

if _HEADLESS:
    simulation_app.set_setting("/app/window/drawMouse", True)
    enable_extension("omni.kit.livestream.webrtc")


# ---------------------------------------------------------------------------
# Hardcode your CyLab hallway/person/mesh-plane settings here.
#
# If HARDCODED_SCENE_USD_PATH is empty, the script falls back to SCENE_USD_PATH,
# then AirStack's ISAAC_SIM_GUI.
# Environment variables still override person settings and the albedo path.
# ---------------------------------------------------------------------------
HARDCODED_SCENE_USD_PATH = "omniverse://airlab-nucleus.andrew.cmu.edu/Users/ruiwang3/Rui_Office_Exp5.usd"
HARDCODED_SCENE_LOAD_MODE = "sublayer"

HARDCODED_PERSON_NAME = "person1"
HARDCODED_PERSON_CHARACTER = "original_male_adult_construction_05"

# Tune these for the hallway scene. Z is usually 0.0 if the floor is at origin.
HARDCODED_PERSON_START = [-9.0, -10.5, 0.0]
HARDCODED_PERSON_TARGET = [-4.4, -10.5, 0.0]
HARDCODED_PERSON_YAW = 1.5708
HARDCODED_PERSON_WALK_SPEED = 1.0

HARDCODED_CAMERA_EYE = [8.0, 8.0, 5.0]
HARDCODED_CAMERA_TARGET = HARDCODED_PERSON_START

HARDCODED_MESH_PLANE_ENABLED = True
HARDCODED_MESH_PLANE_PATH = "/World/CyLabPatchEval/TexturedMeshPlane"
HARDCODED_MESH_PLANE_MATERIAL_PATH = "/World/Looks/CyLabPatchEvalOmniPBR"
HARDCODED_MESH_PLANE_CENTER = [-4.335, -9.425, 1.363]
HARDCODED_MESH_PLANE_ORIENTATION_DEG = [90.0, 0.0, 0.0]
HARDCODED_MESH_PLANE_SIZE = [1.0, 1.0]
# Use a path mounted into the Isaac Sim container, or an omniverse:// URL.
# HARDCODED_MESH_PLANE_ALBEDO_PATH = "omniverse://airlab-nucleus.andrew.cmu.edu/Users/ruiwang3/sim-patch.png"
HARDCODED_MESH_PLANE_ALBEDO_PATH = ""
HARDCODED_MESH_PLANE_TEXTURE_SCALE = [1.0, 1.0]

HARDCODED_DRONE_ENABLED = True
HARDCODED_DRONE_PRIM = "/World/spawned_iris/base_link"
HARDCODED_DRONE_ROBOT_NAME = "robot_1"
HARDCODED_DRONE_VEHICLE_ID = 1
HARDCODED_DRONE_DOMAIN_ID = 1
HARDCODED_DRONE_USD = (
    "~/.local/share/ov/data/documents/Kit/shared/exts/pegasus.simulator/"
    "pegasus/simulator/assets/Robots/Iris/iris.usd"
)
HARDCODED_DRONE_INIT_POS = [-6.8, -6.3, 0.07]
# yaw = -90 deg
HARDCODED_DRONE_INIT_ORIENT = [0.0, 0.0, -0.7071, 0.7071]
HARDCODED_DRONE_ENABLE_ZED = True
HARDCODED_DRONE_ENABLE_OUSTER = True


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
    def __init__(self, target, walk_speed):
        self._person = None
        self._target = list(target)
        self._walk_speed = walk_speed

    def initialize(self, person):
        self._person = person

    def update(self, dt):
        if self._person is not None:
            self._person.update_target_position(self._target, self._walk_speed)

    def update_state(self, state):
        pass

    def start(self):
        pass

    def stop(self):
        pass

    def reset(self):
        pass


def parse_vec3(value: str, default):
    if not value:
        return list(default)

    parts = [part.strip() for part in value.split(",")]
    if len(parts) != 3:
        raise ValueError(f"Expected three comma-separated values, got: {value!r}")

    return [float(part) for part in parts]


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
    st.Set(
        [
            Gf.Vec2f(0.0, 0.0),
            Gf.Vec2f(1.0, 0.0),
            Gf.Vec2f(1.0, 1.0),
            Gf.Vec2f(0.0, 1.0),
        ]
    )
    st.SetIndices([0, 1, 2, 3])

    material_kwargs = {
        "prim_path": material_path,
        "name": "cylab_hallway_eval_omnipbr",
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


class CyLabHallwayEvalScene:
    def __init__(self):
        scene_usd_path = (
            HARDCODED_SCENE_USD_PATH
            or os.environ.get("SCENE_USD_PATH", "")
            or os.environ.get("ISAAC_SIM_GUI", "")
        )
        self.scene_usd_path = os.path.expanduser(scene_usd_path)
        self.scene_load_mode = os.environ.get(
            "SCENE_LOAD_MODE",
            HARDCODED_SCENE_LOAD_MODE,
        ).strip().lower()
        if not self.scene_usd_path:
            raise RuntimeError(
                "Set HARDCODED_SCENE_USD_PATH, SCENE_USD_PATH, or ISAAC_SIM_GUI to the scene USD."
            )
        if self.scene_load_mode not in ("sublayer", "reference"):
            raise RuntimeError("SCENE_LOAD_MODE must be 'sublayer' or 'reference'.")

        self.person_name = os.environ.get("PERSON_NAME", HARDCODED_PERSON_NAME)
        self.person_character = os.environ.get(
            "PERSON_CHARACTER",
            HARDCODED_PERSON_CHARACTER,
        )
        self.person_start = parse_vec3(os.environ.get("PERSON_START"), HARDCODED_PERSON_START)
        self.person_target = parse_vec3(os.environ.get("PERSON_TARGET"), HARDCODED_PERSON_TARGET)
        self.person_yaw = float(os.environ.get("PERSON_YAW", str(HARDCODED_PERSON_YAW)))
        self.person_walk_speed = float(
            os.environ.get("PERSON_WALK_SPEED", str(HARDCODED_PERSON_WALK_SPEED))
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
            sublayer_scene(self.stage, self.scene_usd_path)
        else:
            carb.log_warn(f"Referencing scene USD at /World/stage: {self.scene_usd_path}")
            self.pg.load_asset(self.scene_usd_path, "/World/stage")
            wait_for_stage_updates()

        light_count = count_lights(self.stage)
        carb.log_warn(f"Scene contains {light_count} USD light prim(s) after loading.")
        if light_count == 0:
            carb.log_warn("No authored lights found in the composed scene; the viewport may look dark.")

        if HARDCODED_MESH_PLANE_ENABLED:
            albedo_path = os.environ.get(
                "MESH_PLANE_ALBEDO_PATH",
                HARDCODED_MESH_PLANE_ALBEDO_PATH,
            )
            carb.log_warn(
                f"Creating mesh plane '{HARDCODED_MESH_PLANE_PATH}' with OmniPBR "
                f"albedo map: {albedo_path or '<none>'}"
            )
            create_textured_mesh_plane(
                self.stage,
                HARDCODED_MESH_PLANE_PATH,
                HARDCODED_MESH_PLANE_MATERIAL_PATH,
                HARDCODED_MESH_PLANE_CENTER,
                HARDCODED_MESH_PLANE_ORIENTATION_DEG,
                HARDCODED_MESH_PLANE_SIZE,
                albedo_path,
                HARDCODED_MESH_PLANE_TEXTURE_SCALE,
            )

        self.timeline = omni.timeline.get_timeline_interface()
        self.timeline.stop()

        drone_enabled = os.environ.get(
            "DRONE_ENABLED",
            str(HARDCODED_DRONE_ENABLED),
        ).lower() == "true"
        if drone_enabled:
            drone_prim = os.environ.get("DRONE_PRIM", HARDCODED_DRONE_PRIM)
            drone_robot_name = os.environ.get("DRONE_ROBOT_NAME", HARDCODED_DRONE_ROBOT_NAME)
            drone_vehicle_id = int(os.environ.get("DRONE_VEHICLE_ID", HARDCODED_DRONE_VEHICLE_ID))
            drone_domain_id = int(os.environ.get("DRONE_DOMAIN_ID", HARDCODED_DRONE_DOMAIN_ID))
            drone_usd = os.path.expanduser(os.environ.get("DRONE_USD", HARDCODED_DRONE_USD))
            drone_init_pos = parse_vec3(
                os.environ.get("DRONE_INIT_POS"),
                HARDCODED_DRONE_INIT_POS,
            )

            carb.log_warn(
                f"Spawning Pegasus PX4 Iris '{drone_robot_name}' at {drone_init_pos} "
                f"on prim '{drone_prim}' using {drone_usd}"
            )
            graph_handle = spawn_px4_multirotor_node(
                pegasus_node_name="PX4Multirotor_CyLabHallway",
                drone_prim=drone_prim,
                robot_name=drone_robot_name,
                vehicle_id=drone_vehicle_id,
                domain_id=drone_domain_id,
                usd_file=drone_usd,
                init_pos=drone_init_pos,
                init_orient=HARDCODED_DRONE_INIT_ORIENT,
            )

            enable_zed = os.environ.get(
                "DRONE_ENABLE_ZED",
                str(HARDCODED_DRONE_ENABLE_ZED),
            ).lower() == "true"
            if enable_zed:
                add_zed_stereo_camera_subgraph(
                    parent_graph_handle=graph_handle,
                    drone_prim=drone_prim,
                    robot_name=drone_robot_name,
                    camera_name="ZEDCamera",
                    camera_offset=[0.2, 0.0, -0.05],
                    camera_rotation_offset=[0.0, 0.0, 0.0],
                )

            enable_ouster = os.environ.get(
                "DRONE_ENABLE_OUSTER",
                str(HARDCODED_DRONE_ENABLE_OUSTER),
            ).lower() == "true"
            if enable_ouster:
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

        carb.log_warn(
            f"Spawning person '{self.person_name}' using character "
            f"'{self.person_character}' at {self.person_start}"
        )
        self.person_controller = StraightLinePersonController(
            self.person_target,
            self.person_walk_speed,
        )
        self.person = Person(
            self.person_name,
            self.person_character,
            init_pos=self.person_start,
            init_yaw=self.person_yaw,
            controller=self.person_controller,
        )
        self.person.update_target_position(self.person_target, self.person_walk_speed)
        set_camera_view(eye=HARDCODED_CAMERA_EYE, target=HARDCODED_CAMERA_TARGET)

        self.world.reset()
        self.person.update_target_position(self.person_target, self.person_walk_speed)

    def run(self):
        self.timeline.play()
        self.person.update_target_position(self.person_target, self.person_walk_speed)

        app = omni.kit.app.get_app()
        while simulation_app.is_running():
            world = World.instance()
            if world is not None and hasattr(world, "_scene"):
                world.step(render=True)
                if world is not self.world:
                    self.world = world
                    self.pg._world = world
            else:
                app.update()

        self.timeline.stop()
        simulation_app.close()


def main():
    try:
        CyLabHallwayEvalScene().run()
    except Exception as exc:
        carb.log_error(str(exc))
        simulation_app.close()
        raise


if __name__ == "__main__":
    main()
