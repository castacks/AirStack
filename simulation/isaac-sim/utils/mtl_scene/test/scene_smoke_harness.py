"""Smoke harness: run launch_scripts/search_mission_scene.py against REAL pxr
(usd-core or Isaac's) with fake Kit / Pegasus / OmniGraph modules, then fly a
fake vehicle and check the rendered camera's world pose against the command.

Run as a subprocess by test_scene_script.py (it stubs sys.modules).
"""
import math, os, sys, types, runpy
from pxr import Usd, UsdGeom, Gf

from pathlib import Path
REPO = str(Path(__file__).resolve().parents[5])
LS = REPO + "/simulation/isaac-sim/launch_scripts"
STAGE = Usd.Stage.CreateInMemory()
UsdGeom.Xform.Define(STAGE, "/World")

def mod(name, **kw):
    m = types.ModuleType(name); m.__dict__.update(kw); sys.modules[name] = m; return m

# carb
mod("carb", log_warn=lambda m: print("WARN", m), log_info=lambda m: None, log_error=lambda m: print("ERR", m))
# omni.usd / omni.kit.app
subs = []
class _Stream:
    def create_subscription_to_pop(self, fn, name=""): subs.append(fn); return object()
class _App:
    def get_update_event_stream(self): return _Stream()
omni = mod("omni"); omni.usd = mod("omni.usd", get_context=lambda: types.SimpleNamespace(get_stage=lambda: STAGE))
omni.kit = mod("omni.kit"); omni.kit.app = mod("omni.kit.app", get_app=lambda: _App())
# og
class Attr:
    def __init__(self, path): self.path = path; self.value = 0.0
    def is_valid(self): return True
ATTRS, EDITS = {}, []
class Controller:
    class Keys: CREATE_NODES="c"; SET_VALUES="s"; CONNECT="x"
    @staticmethod
    def edit(g, cmds): EDITS.append((g, cmds))
    @staticmethod
    def attribute(p): return ATTRS.setdefault(p, Attr(p))
    @staticmethod
    def get(a): return a.value
    @staticmethod
    def set(a, v): a.value = v
og = mod("omni.graph.core", Controller=Controller); omni.graph = mod("omni.graph"); omni.graph.core = og
# pegasus
mod("pegasus"); mod("pegasus.simulator")
mod("pegasus.simulator.params", SIMULATION_ENVIRONMENTS={"Default Environment": "default.usd"})
VEH = {}
class VM:
    vehicles = VEH
    @staticmethod
    def get_vehicle_manager(): return VM
mod("pegasus.simulator.logic"); mod("pegasus.simulator.logic.vehicle_manager", VehicleManager=VM)
mod("gps_utils", DEFAULT_WORLD_ORIGIN=(38.7, -9.1, 0.0))

class FakeTimeline:
    t = 0.0
    def get_current_time(self): return self.t

class PegasusApp:
    """Mimics the real call order: attrs, gps, post_scene_prep, _position_scale, spawn, post_spawn."""
    def __init__(self, *, env_url, drone_configs=(), world_gps_origin=None, enable_camera=True, enable_lidar=True, **kw):
        self.drone_configs = list(drone_configs); self.timeline = FakeTimeline()
        assert world_gps_origin is not None
        self.post_scene_prep(STAGE)
        self._position_scale = 1.0
        for c in self.drone_configs: self.spawn_drone(c)
        self.post_spawn(STAGE)
    def spawn_drone(self, cfg):
        i = cfg["domain_id"]; p = cfg.get("prim", f"/World/drone{i}/base_link")
        x = UsdGeom.Xform.Define(STAGE, p); x.AddTranslateOp().Set(Gf.Vec3d(cfg["x_m"], cfg["y_m"], cfg["z_m"]))
        x.AddOrientOp().Set(Gf.Quatf(1, 0, 0, 0)); return p
    def run(self): APP.append(self)
APP = []
mod("pegasus_app", create_simulation_app=lambda: None, PegasusApp=PegasusApp)
sys.modules["isaacsim"] = mod("isaacsim")

os.environ.pop("FLEET_CONFIG_FILE", None)
os.environ["FLEET_CONFIG_FILE"] = "/root/AirStack/config/fleets/mtl_search_fleet.yaml"
sys.argv = [LS + "/search_mission_scene.py"]
runpy.run_path(LS + "/search_mission_scene.py", run_name="__main__")
app = APP[0]
print("gimbals", [g["robot"] for g in app.gimbals], "graphs", len(EDITS))
for prim in ["/World/MTL/Ground", "/World/MTL/SearchArea", "/World/MTL/Targets", "/World/drone1/base_link/camera_gimbal/camera"]:
    assert STAGE.GetPrimAtPath(prim).IsValid(), prim

sys.path.insert(0, REPO + "/simulation/isaac-sim/utils")
from mtl_scene import gimbal as mg
g = app.gimbals[1]
# vehicle flying at (10,20,30), yawed 40 deg, pitched 10 deg
bq = mg.quat_from_euler_zyx(0.0, math.radians(10), math.radians(40))
VEH["/World/drone2/base_link"] = types.SimpleNamespace(state=types.SimpleNamespace(position=[10, 20, 30], attitude=list(bq)))
cmd = (0.0, math.radians(45), math.radians(90))  # look north, 45 down
for a, v in zip(g["cmd_paths"], cmd): ATTRS.setdefault(a, Attr(a)).value = v
for k in range(60):
    app.timeline.t = 0.05 * k
    for fn in subs: fn(None)
cam = STAGE.GetPrimAtPath(g["camera_path"])
M = UsdGeom.Xformable(cam).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
view = M.TransformDir(Gf.Vec3d(0, 0, -1)).GetNormalized(); up = M.TransformDir(Gf.Vec3d(0, 1, 0)).GetNormalized()
pos = M.ExtractTranslation()
s = math.sqrt(0.5)
print("view", view, "up", up, "pos", pos, "state", [round(math.degrees(v), 2) for v in g["axis"].state])
assert (view - Gf.Vec3d(0, s, -s)).GetLength() < 1e-6
assert (up - Gf.Vec3d(0, s, s)).GetLength() < 1e-6
off = mg.quat_rotate(bq, (0.10, 0, -0.08))
assert (pos - Gf.Vec3d(10 + off[0], 20 + off[1], 30 + off[2])).GetLength() < 1e-6
st = [ATTRS[p].value for p in g["state_paths"]]
assert all(abs(a - b) < 1e-9 for a, b in zip(st, cmd)), st
# slew: fresh command 90 deg away takes > 0.7 s at 120 deg/s
for a, v in zip(g["cmd_paths"], (0.0, math.radians(45), math.radians(0))): ATTRS[a].value = v
app.timeline.t += 0.25
for fn in subs: fn(None)
print("after 0.25 s yaw", round(math.degrees(g["axis"].state[2]), 2))
assert abs(math.degrees(g["axis"].state[2]) - 60.0) < 1e-6
# robot_1 has received nothing -> parked at initial pitch
g1 = app.gimbals[0]
print("robot_1 parked", [round(math.degrees(v), 1) for v in g1["axis"].state])
sets = dict(v for e in EDITS for v in e[1].get("s", []))
print({k: v for k, v in sets.items() if "topicName" in k or "domain" in k or "frameSkip" in k})
print("SMOKE OK")
