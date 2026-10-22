"""Disaster City (TEEX) digital twin with N PX4 drones.

The twin is a finished, self-lit, georeferenced USD built by
`scene_gen/disaster_city/` (metres, Z-up, /World as defaultPrim, its own sun +
sky under /World/Environment, colliders authored). This launcher references it
unchanged under /World/stage — no rescale, no blanket colliders, no extra dome
light — and spawns the drones the same way `example_multi_drone_scene_import.py`
does (PX4 node + ZED stereo, robot_<i> on ROS domain <i>), so the normal robot
stack flies them.

    ISAAC_SIM_SCRIPT_NAME=disaster_city_launch_script.py \
    SPAWN_CONFIGS='[{"x_m":3,"y_m":-335,"z_m":55.6}, ...]' \
    airstack up isaac-sim

Env:
  DISASTER_CITY_USD  stage to load: a container path or an omniverse:// URL (e.g. the
                     Nucleus copy, for an OSMO pod with no data mount). Default: the
                     mounted data dir's recon/disaster_city.usda (the mount is
                     `DISASTER_CITY_DATA` in docker-compose.yaml)
  SPAWN_CONFIGS      JSON list of {x_m, y_m, z_m[, orient, domain_id]} in TWIN WORLD metres.
                     The ground is NOT at z=0 (z is WGS84 ellipsoidal height, ~55 m on the
                     north field), so z_m is required: there is no terrain raycast.
  ISAAC_SIM_HEADLESS / ISAAC_SIM_LIVESTREAM   as in the example launcher
  SNAP_DIR           if set, a top-down view over B01 (where the survivors are) is written
                     there every 10 sim seconds — a time-lapse of the mission.

PX4's simulated GPS stays on the stack-wide Lisbon anchor (gps_utils), because
map_anchor_node and the GCS relay convert with that same constant; the twin's
WGS84 frame is applied by whoever needs real lat/lon (scene_gen/disaster_city/georef.py).
"""

import json
import os

from isaacsim import SimulationApp

_LIVESTREAM = os.environ.get("ISAAC_SIM_LIVESTREAM", "").lower() == "true"
_ARGS = ["--/renderer/multiGpu/enabled=false"]
if _LIVESTREAM:
    simulation_app = SimulationApp(launch_config={
        "width": 1280, "height": 720, "window_width": 1920, "window_height": 1080,
        "headless": True, "hide_ui": False, "renderer": "RaytracedLighting",
        "display_options": 3286, "extra_args": _ARGS})
else:
    simulation_app = SimulationApp(launch_config={
        "headless": os.getenv("ISAAC_SIM_HEADLESS", "false").lower() == "true",
        "extra_args": _ARGS})

import carb
import omni.client
import omni.kit.app
import omni.timeline
import omni.usd

if _LIVESTREAM:
    from isaacsim.core.utils.extensions import enable_extension
    _port = int(os.environ.get("ISAAC_SIM_LIVESTREAM_UDP_PORT", "49099"))
    for key in ("fixedHostPort", "minHostPort", "maxHostPort"):
        simulation_app.set_setting(f"/app/livestream/{key}", _port)
    simulation_app.set_setting("/app/livestream/enabled", True)
    enable_extension("omni.kit.livestream.webrtc")

# Pegasus probes the NVIDIA asset root (<root>/Isaac and <root>/NVIDIA) when it is
# IMPORTED, and fails the import if the probe does. Nothing here loads an NVIDIA
# asset — the twin is self-contained and the Iris ships inside the extension — so
# point the probe at an empty local skeleton and the launcher runs with no Nucleus.
_ASSET_STUB = "/tmp/disaster_city_asset_root"
for _d in ("Isaac", "NVIDIA"):
    os.makedirs(os.path.join(_ASSET_STUB, _d), exist_ok=True)
carb.settings.get_settings().set("/persistent/isaac/asset_root/default", _ASSET_STUB)

# pymavlink select() fails on fds >= 1024, which DDS-over-TCP (LARGE_DATA) reaches
# with a few robots; poll() has no such limit (same patch as the example launcher).
import select as _select
import time as _time
from pymavlink import mavutil as _mavutil


def _mavfile_select_poll(self, timeout):
    if self.fd is None:
        _time.sleep(min(timeout, 0.5))
        return True
    poller = _select.poll()
    poller.register(self.fd, _select.POLLIN)
    try:
        return len(poller.poll(timeout * 1000)) > 0
    except OSError:
        return False


_mavutil.mavfile.select = _mavfile_select_poll

ext_manager = omni.kit.app.get_app().get_extension_manager()
for ext in ("omni.graph.core", "omni.graph.action", "omni.graph.action_nodes",
            "omni.graph.scriptnode", "pegasus.simulator"):
    if not ext_manager.is_extension_enabled(ext):
        ext_manager.set_extension_enabled_immediate(ext, True)

import importlib.util
import sys

from omni.isaac.core.world import World
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.ogn.api.spawn_multirotor import spawn_px4_multirotor_node
from pegasus.simulator.ogn.api.spawn_zed_camera import add_zed_stereo_camera_subgraph

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.normpath(os.path.join(_HERE, "..", "utils")))
from gps_utils import set_gps_origins  # noqa: E402
from scene_prep import dedupe_physics_scenes  # noqa: E402

TWIN_USD = os.environ.get("DISASTER_CITY_USD", "").strip() or \
    "/isaac-sim/disaster_city_data/recon/disaster_city.usda"
DRONE_USD = "~/.local/share/ov/data/documents/Kit/shared/exts/pegasus.simulator/pegasus/simulator/assets/Robots/Iris/iris.usd"
ZED_PITCH_DEG = float(os.environ.get("ZED_PITCH_DEG", "").strip() or 0.0)
ZED_WIDTH = int(os.environ.get("ZED_WIDTH", "").strip() or 720)
ZED_HEIGHT = int(os.environ.get("ZED_HEIGHT", "").strip() or 450)
SNAP_DIR = os.environ.get("SNAP_DIR", "").strip()
SNAP_PERIOD_S = 10.0
SNAP_EYE, SNAP_AT = (28.0, -415.0, 128.0), (28.0, -415.0, 57.0)   # ~80 m wide over B01 and V01-V11


def drone_configs():
    raw = os.environ.get("SPAWN_CONFIGS", "").strip()
    if not raw:
        raise SystemExit("SPAWN_CONFIGS is required: the twin's ground is ~55 m up, "
                         "so there is no safe default spawn height.")
    cfgs = json.loads(raw)
    for i, c in enumerate(cfgs, start=1):
        if "z_m" not in c:
            raise SystemExit(f"SPAWN_CONFIGS[{i - 1}] has no z_m (twin world z, ground ~55 m)")
        c.setdefault("domain_id", i)
        c.setdefault("orient", [0.0, 0.0, 0.0, 1.0])
    return cfgs


def main():
    if TWIN_USD.startswith("omniverse://"):
        if omni.client.stat(TWIN_USD)[0] != omni.client.Result.OK:
            raise SystemExit(f"twin USD not reachable on Nucleus: {TWIN_USD} (OMNI_USER / OMNI_PASS?)")
    elif not os.path.exists(TWIN_USD):
        raise SystemExit(f"twin USD not found in the container: {TWIN_USD} "
                         "(set DISASTER_CITY_DATA so docker-compose mounts the data dir)")
    cfgs = drone_configs()
    set_gps_origins(cfgs)

    timeline = omni.timeline.get_timeline_interface()
    pg = PegasusInterface()
    pg._world = World(**pg._world_settings)
    timeline.stop()
    pg.load_environment(TWIN_USD)        # references the twin's defaultPrim (/World) at /World/stage
    stage = omni.usd.get_context().get_stage()
    for _ in range(100):
        omni.kit.app.get_app().update()
        if stage.GetPrimAtPath("/World/stage").IsValid():
            break
    dedupe_physics_scenes(stage)

    for c in cfgs:
        i = c["domain_id"]
        handle = spawn_px4_multirotor_node(
            pegasus_node_name=f"PX4Multirotor_{i}", drone_prim=f"/World/drone{i}/base_link",
            robot_name=f"robot_{i}", vehicle_id=i, domain_id=i, usd_file=DRONE_USD,
            init_pos=[c["x_m"], c["y_m"], c["z_m"]], init_orient=c["orient"])
        add_zed_stereo_camera_subgraph(
            parent_graph_handle=handle, drone_prim=f"/World/drone{i}/base_link",
            robot_name=f"robot_{i}", camera_name="ZEDCamera", camera_offset=[0.21, 0.0, 0.05],
            camera_rotation_offset=[0.0, ZED_PITCH_DEG, 0.0],
            frame_width=ZED_WIDTH, frame_height=ZED_HEIGHT)
        print(f"[disaster_city] robot_{i} (domain {i}) at world "
              f"({c['x_m']:.1f}, {c['y_m']:.1f}, {c['z_m']:.1f})", flush=True)

    snapshots = None
    if SNAP_DIR:
        spec = importlib.util.spec_from_file_location(
            "snapshots", os.path.join(_HERE, "..", "utils", "snapshots.py"))
        snapshots = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(snapshots)
        snapshots.place_camera(stage, SNAP_EYE, SNAP_AT, focal_mm=18.0)
        snapshots.hide_decorations()
    print(f"[disaster_city] twin {TWIN_USD}, {len(cfgs)} drone(s); PX4 booting", flush=True)

    timeline.play()
    import omni.kit.viewport.utility as vp
    world, next_snap = pg.world, 0.0
    while simulation_app.is_running():
        world.step(render=True)
        if snapshots is not None and world.current_time >= next_snap:
            path = os.path.join(SNAP_DIR, f"t{int(world.current_time):05d}.png")
            vp.capture_viewport_to_file(vp.get_active_viewport(), path)
            next_snap = world.current_time + SNAP_PERIOD_S
    timeline.stop()
    simulation_app.close()


if __name__ == "__main__":
    main()
