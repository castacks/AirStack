"""Office bench runtime. Starts physics and cameras; never arms the vehicle."""
import os
import sys
import json
import time
from pathlib import Path
import socket

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
RUNTIME = ROOT / "robot/ros_ws/ws2_runtime"
RUNTIME.mkdir(exist_ok=True)
os.environ["ROS_DOMAIN_ID"] = "1"
os.environ["PX4_PHYSICS_HZ"] = "200"
os.environ["PX4_RENDERING_HZ"] = "30"
from isaacsim import SimulationApp
HEADLESS=os.environ.get('WS2_HEADLESS','1')!='0'
app = SimulationApp({"headless": HEADLESS, "width": 1280, "height": 720,
                     "window_width": 1440, "window_height": 900,
                     "renderer": "RaytracedLighting"})
import numpy as np
import omni.usd
import omni.timeline
from pxr import Usd, UsdGeom, UsdPhysics, UsdLux, UsdShade, Sdf, Gf
from isaacsim.core.api import World
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.core.utils.extensions import enable_extension
from conditions import validate,PATCH_HEIGHT_M
from layout_summary import describe
for name in ("isaacsim.ros2.bridge", "pegasus.simulator"):
    enable_extension(name)
for _ in range(10):
    app.update()
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.ogn.api.spawn_multirotor import spawn_px4_multirotor_node
from pegasus.simulator.ogn.api.spawn_zed_camera import add_zed_stereo_camera_subgraph
from pegasus.simulator.ogn.python.nodes.OgnPegasusMultirotorNodeBase import drone_sim_dict
from physics_state import install
install()
from scipy.spatial.transform import Rotation

OFFICE = os.environ.get("WS2_OFFICE_USD", "/tmp/ws2_assets/Isaac/4.5/Isaac/Environments/Office/office.usd")
PATCH = os.environ.get("WS2_PATCH_TEXTURE", str(HERE / "assets/learned_patch.png"))
PATCH_KIND = os.environ.get("WS2_PATCH_KIND", "Rui learned FCRN patch")
import hashlib
PATCH_SHA256 = hashlib.sha256(Path(PATCH).read_bytes()).hexdigest()
episode_path=os.environ.get('WS2_EPISODE_CONFIG')
episode=json.loads(Path(episode_path).read_text()) if episode_path else {}
pg = PegasusInterface()
pg._world_settings["physics_dt"] = 1/200
pg._world_settings["rendering_dt"] = 1/30
world = World(physics_dt=1/200, rendering_dt=1/30, stage_units_in_meters=1.)
pg._world = world
stage = omni.usd.get_context().get_stage()
omni.timeline.get_timeline_interface().stop()
environment = stage.DefinePrim("/World/Office", "Xform")
environment.GetReferences().AddReference(OFFICE)
UsdGeom.Xformable(environment).AddRotateZOp().Set(-90.)
for p in Usd.PrimRange(environment, Usd.TraverseInstanceProxies()):
    if p.IsA(UsdGeom.Mesh) and not p.IsInstanceProxy():
        UsdPhysics.CollisionAPI.Apply(p)
        UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr("none")

sources = {"plant": "/Root/SM_Plant8", "column": "/Root/SM_ColumnA13", "plant2": "/Root/SM_Plant8"}
sources.update({f'{kind}_{i}':path for kind,path in [('plant','/Root/SM_Plant8'),('column','/Root/SM_ColumnA13')] for i in range(2,6)})
original = Usd.Stage.Open(OFFICE)
for name, src in sources.items():
    dst = stage.DefinePrim("/World/Office/WS2_" + name, "Xform")
    dst.GetReferences().AddReference(OFFICE, src)
    if dst.IsInstance():
        dst.SetInstanceable(False)
    for p in Usd.PrimRange(dst):
        if p.IsA(UsdGeom.Mesh):
            UsdPhysics.CollisionAPI.Apply(p)
            UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr("none")
move_path = "/World/Office/SM_Plant7_463"
original_move = UsdGeom.Xformable(stage.GetPrimAtPath(move_path)).GetLocalTransformation()
source_matrices = {n: UsdGeom.Xformable(original.GetPrimAtPath(p)).GetLocalTransformation() for n,p in sources.items()}
dome = UsdLux.DomeLight.Define(stage, "/World/WS2Light")
dome.CreateIntensityAttr(1800.)
fill = UsdLux.SphereLight.Define(stage, "/World/WS2Fill")
fill.CreateIntensityAttr(10000.)
fill.CreateRadiusAttr(2.)
UsdLux.ShadowAPI.Apply(fill.GetPrim()).CreateShadowEnableAttr(False)
UsdGeom.Xformable(fill).AddTranslateOp().Set(Gf.Vec3d(-2, 0, 2.5))

# A non-colliding surface decal, backed by the unchanged column collider.
quad = UsdGeom.Mesh.Define(stage, "/World/PatchSurface")
quad.CreateFaceVertexCountsAttr([4])
quad.CreateFaceVertexIndicesAttr([0,1,2,3])
quad.CreateDoubleSidedAttr(True)
quad.CreateSubdivisionSchemeAttr("none")
UsdGeom.PrimvarsAPI(quad).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex).Set([(0,0),(1,0),(1,1),(0,1)])
material = UsdShade.Material.Define(stage, "/World/PatchMaterial")
shader = UsdShade.Shader.Define(stage, "/World/PatchMaterial/Surface")
shader.CreateIdAttr("UsdPreviewSurface")
shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(.85)
shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(1.)
material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
texture = UsdShade.Shader.Define(stage, "/World/PatchMaterial/Texture")
texture.CreateIdAttr("UsdUVTexture")
texture.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(PATCH))
texture.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
texture.CreateInput("scale", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(1,1,1,1))
texture.CreateInput("bias", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(0,0,0,0))
texture.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
reader = UsdShade.Shader.Define(stage, "/World/PatchMaterial/UV")
reader.CreateIdAttr("UsdPrimvarReader_float2")
reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
texture.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), "result")
shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(texture.ConnectableAPI(), "rgb")
UsdShade.MaterialBindingAPI.Apply(quad.GetPrim()).Bind(material)

condition = None
def apply_condition(raw):
    global condition
    c = validate(raw)
    # Layout edits are for stopped/grounded vehicle between trials. Never move a
    # collider into a flying aircraft just to make the demo look adversarial.
    if drone_sim_dict and condition and (c["layout"],c["layout_seed"]) != (condition["layout"],condition["layout_seed"]):
        state = next(iter(drone_sim_dict.values()))["multirotor"].state
        if float(state.position[2]) > .35:
            raise ValueError("land before changing layout")
    active = c["layout"] != "stock"
    catalog=json.loads((HERE/"layouts.json").read_text())
    placement=catalog[c["layout"]][c["layout_seed"]] if active else {}
    delta=placement.get("move",[0,0,0])
    matrix = Gf.Matrix4d(original_move)
    if active:
        matrix.SetTranslateOnly(matrix.ExtractTranslation() + Gf.Vec3d(*delta))
    UsdGeom.Xformable(stage.GetPrimAtPath(move_path)).MakeMatrixXform().Set(matrix)
    displacements=placement
    for name in sources:
        p = stage.GetPrimAtPath("/World/Office/WS2_"+name)
        enabled=active and name in placement
        p.SetActive(enabled)
        if enabled:
            m = Gf.Matrix4d(source_matrices[name]);m.SetTranslateOnly(m.ExtractTranslation()+Gf.Vec3d(*displacements[name]))
            UsdGeom.Xformable(p).MakeMatrixXform().Set(m)
    dome.GetIntensityAttr().Set(c["light"])
    fill.GetIntensityAttr().Set(c["light"] * 5.5)
    if active:
        p = stage.GetPrimAtPath("/World/Office/WS2_column")
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default","render"])
        b = cache.ComputeWorldBound(p).ComputeAlignedRange()
        x = float(b.GetMin()[0]) - .006
        y = float((b.GetMin()[1]+b.GetMax()[1])/2)
        half = c["patch_size"]/2; z = PATCH_HEIGHT_M
        quad.GetPointsAttr().Set([(x,y-half,z-half),(x,y+half,z-half),(x,y+half,z+half),(x,y-half,z+half)])
        if c['patch_enabled']:
            UsdGeom.Imageable(quad).MakeVisible()
        else:
            UsdGeom.Imageable(quad).MakeInvisible()
    else:
        UsdGeom.Imageable(quad).MakeInvisible()
    condition=c
    print('[WS2] Condition applied: '+json.dumps(c),flush=True)

apply_condition(episode.get('condition',{"name":"Office clean", "layout":"furnished_a"}))
iris = "/isaac-sim/.local/share/ov/data/documents/Kit/shared/exts/pegasus.simulator/pegasus/simulator/assets/Robots/Iris/iris.usd"
graph=spawn_px4_multirotor_node(drone_prim="/World/base_link",usd_file=iris,robot_name="robot_1",vehicle_id=1,domain_id=1,
                              init_pos=episode.get('spawn',[-4.,0.,.07]),init_orient=[0.,0.,0.,1.])
add_zed_stereo_camera_subgraph(parent_graph_handle=graph,drone_prim="/World/base_link",robot_name="robot_1",camera_name="ZEDCamera",
                              camera_offset=[.2,0.,-.05],camera_rotation_offset=[0.,0.,0.])
set_camera_view(eye=np.array([-8.,-3.,2.3]),target=np.array([2.,0.,1.2]))
import omni.replicator.core as rep
from PIL import Image as PilImage
observer=UsdGeom.Camera.Define(stage,'/World/WS2Observer')
observer.CreateClippingRangeAttr().Set(Gf.Vec2f(.05,500))
observer.CreateFocalLengthAttr().Set(18.)
observer_transform=UsdGeom.Xformable(observer).AddTransformOp()
render_product=rep.create.render_product(str(observer.GetPath()),(960,540))
observer_rgb=rep.AnnotatorRegistry.get_annotator('rgb');observer_rgb.attach([render_product])
view={'mode':'follow','distance':2.5,'height':1.5,'azimuth':180.}
observer_eye=np.array([-8.,-3.,3.]);observer_target=np.array([0.,0.,1.])
world.reset()
from scene_oracle import SceneOracle
oracle=SceneOracle(stage)
omni.timeline.get_timeline_interface().play()
last_status=0.
camera_mode="follow"
operator_paused=False
telemetry=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
telemetry_target=(socket.gethostbyname("robot-desktop"),9877)
telemetry_registered=False
guard=None
last_preview=0.
def publish_state(dt):
    global guard
    vehicle=next(iter(drone_sim_dict.values()))["multirotor"]
    state=vehicle.state
    oracle.update(float(world.current_time),state.position)
    tilt=float(np.degrees(np.arccos(np.clip(Rotation.from_quat(state.attitude).apply([0,0,1])[2],-1,1))))
    telemetry.sendto(json.dumps({"sim_time":float(world.current_time),"position":state.position.tolist(),
                     "attitude_xyzw":state.attitude.tolist(),"velocity":state.linear_velocity.tolist(),
                     "oracle":oracle.status}).encode(),telemetry_target)
    if tilt>20 or (state.position[2]>.2 and np.linalg.norm(state.linear_velocity)>.9):
        guard={"tilt":tilt,"position":state.position.tolist(),"velocity":state.linear_velocity.tolist(),"oracle":oracle.status}
        (RUNTIME/"flight_guard.json").write_text(json.dumps(guard))
print("[WS2] Office ready. Physics running; vehicle remains disarmed.",flush=True)
while app.is_running():
    if guard:
        omni.timeline.get_timeline_interface().pause()
        app.update()
        continue
    world.step(render=True)
    if drone_sim_dict and not telemetry_registered:
        world.add_physics_callback("ws2_ground_truth",publish_state)
        telemetry_registered=True
    command_path=RUNTIME/"scene_command.json"
    if command_path.exists():
        try:
            command=json.loads(command_path.read_text());command_path.unlink()
            if command.get('oracle_validation'):
                if episode.get('fault')!='contact_probe':raise ValueError('contact probe requires explicit validation episode')
                probe_position=next(iter(drone_sim_dict.values()))['multirotor'].state.position
                probe=UsdGeom.Cube.Define(stage,'/World/WS2OracleProbe');probe.CreateSizeAttr(.2)
                UsdGeom.Xformable(probe).AddTranslateOp().Set(Gf.Vec3d(*map(float,probe_position+np.array([.1,0,0]))))
                UsdPhysics.CollisionAPI.Apply(probe.GetPrim())
            if "condition" in command:
                apply_condition(command["condition"])
            if "camera" in command:
                camera_mode=command["camera"]
                view['mode']=camera_mode
            if 'paused' in command:
                operator_paused=bool(command['paused'])
                (omni.timeline.get_timeline_interface().pause if operator_paused else omni.timeline.get_timeline_interface().play)()
            if "eye" in command:
                camera_mode="fixed";set_camera_view(eye=np.array(command["eye"]),target=np.array(command["target"]))
            if "capture" in command:
                from omni.kit.viewport.utility import get_active_viewport,capture_viewport_to_file
                world.step(render=True);world.step(render=True)
                capture_viewport_to_file(get_active_viewport(),str(RUNTIME/Path(command["capture"]).name))
            reply={"id":command.get("id"),"ok":True,"condition":condition,"sim_time":float(world.current_time)}
        except Exception as exc:
            reply={"id":command.get("id") if 'command' in locals() else None,"ok":False,"error":str(exc)}
            print("[WS2] "+str(exc),flush=True)
        (RUNTIME/"scene_reply.json").write_text(json.dumps(reply))
    state=None
    if drone_sim_dict:
        state=next(iter(drone_sim_dict.values()))["multirotor"].state
        try:
            if (RUNTIME/'view_settings.json').exists():view.update(json.loads((RUNTIME/'view_settings.json').read_text()))
        except (OSError,ValueError):pass
        camera_mode=view['mode']
        if camera_mode!='free':
            center=np.array(state.position if camera_mode=='follow' else [0.,0.,0.])
            angle=np.radians(view['azimuth']);d=view['distance']
            observer_eye=center+np.array([d*np.cos(angle),d*np.sin(angle),view['height']])
            observer_target=center+np.array([0.,0.,.3])
            observer_transform.Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*map(float,observer_eye)),Gf.Vec3d(*map(float,observer_target)),Gf.Vec3d(0,0,1)).GetInverse())
            if not HEADLESS:set_camera_view(eye=observer_eye,target=observer_target)
    if time.monotonic()-last_preview>.125:
        pixels=observer_rgb.get_data()
        if pixels is not None and getattr(pixels,'size',0):
            target=RUNTIME/'live_view.jpg';tmp=target.with_suffix('.tmp.jpg')
            PilImage.fromarray(np.asarray(pixels)[:,:,:3]).save(tmp,quality=85);tmp.replace(target)
            meta={'wall_time':time.time(),'sim_time':float(world.current_time),'view':view,'headless':HEADLESS}
            temp=RUNTIME/'live_view.tmp.json';temp.write_text(json.dumps(meta));temp.replace(RUNTIME/'live_view.json')
        last_preview=time.monotonic()
    if time.monotonic()-last_status>.1:
        data={"sim_time":float(world.current_time),"condition":condition,"patch_kind":PATCH_KIND,
              "patch_sha256":PATCH_SHA256,"patch_corners_world":[list(p) for p in quad.GetPointsAttr().Get() or []],
              "realized_layout":json.loads((HERE/"layouts.json").read_text()).get(condition["layout"],[{}]*8)[condition["layout_seed"]],
              "position":None if state is None else state.position.tolist(),
              "attitude_xyzw":None if state is None else state.attitude.tolist(),
              "velocity":None if state is None else state.linear_velocity.tolist(),"camera":camera_mode,
              "oracle":oracle.status,'paused':operator_paused,'view':view,'headless':HEADLESS,
              'layout_description':describe(condition),
              'active_added_obstacles':[n for n in sources if stage.GetPrimAtPath('/World/Office/WS2_'+n).IsActive()]}
        temp=RUNTIME/"scene_status.tmp";temp.write_text(json.dumps(data));temp.replace(RUNTIME/"scene_status.json")
        last_status=time.monotonic()
app.close()
