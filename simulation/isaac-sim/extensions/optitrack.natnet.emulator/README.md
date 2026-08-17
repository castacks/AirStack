# OptiTrack NatNet Emulator (Isaac Sim Extension)

Python NatNet **server** emulator for AirStack simulation and integration testing with [`natnet_ros2`](../../../../robot/ros_ws/src/perception/natnet_ros2/).

The extension has two layers:

1. **Transport + protocol** (`optitrack.natnet.emulator.server`) — UDP NatNet server, ctypes wire types, MODELDEF cache, frame streaming. Importable outside Isaac Sim (unit tests, host-side integration).
2. **Isaac integration** (`optitrack.natnet.emulator.isaac`) — stage-driven `/World/NatNetInterface` config prim, pose sampling on physics steps, Kit UI editor, and Pegasus launch-script helpers.

## Layout

```
optitrack.natnet.emulator/
├── config/extension.toml          # Kit manifest (server module + UI entry point)
├── schema/schema.usda             # Typed NatNet interface attribute definitions
├── setup.py
├── docs/                          # (legacy design notes — see docs/simulation/isaac_sim/natnet_emulator.md)
├── test/                          # Co-located unit tests (proxied by tests/sim/)
└── optitrack/natnet/emulator/
    ├── defaults.py                # Reference Drone → prim bindings for tests
    ├── server/                    # NatNet UDP server (transport + protocol)
    │   ├── natnet_server.py       # Base server, queue, MODELDEF cache
    │   ├── natnet_unicast_server.py
    │   ├── natnet_data_types.py
    │   ├── natnet_model_types.py
    │   └── natnet_server_types.py
    └── isaac/                     # Isaac Sim wrapper (Kit + USD)
        ├── config.py              # Pure-Python NatNetInterfaceConfig model
        ├── usd_bindings.py        # Author/read interface prims on a stage
        ├── catalog.py             # Config → sDataDescriptions (MODELDEF)
        ├── frames.py              # Prim poses → sFrameOfMocapData
        ├── manager.py             # NatNetServerManager (lifecycle + sampling)
        ├── scene_setup.py         # Pegasus launch helpers (start_drone_natnet_server)
        └── ui_extension.py        # Docked editor panel (NatNetEmulatorExtension)
```

## Responsibilities

| Layer | Role |
|-------|------|
| **Server** | UDP transport; `NAT_CONNECT` / `NAT_SERVERINFO`; `NAT_REQUEST_MODELDEF`; `NAT_KEEPALIVE`; `NAT_ECHOREQUEST` / `NAT_ECHORESPONSE`; `NAT_FRAMEOFDATA` on the **data port** (1511). MODELDEF stored as packed bytes via `set_model_def_payload()`. Frames enqueued with `enqueue_mocap_data()`. |
| **Isaac wrapper** | Authors and reads the NatNet interface config prim; builds MODELDEF from scene config; samples tracked prim world poses each physics step; calls `flush_mocap_data()` synchronously (background timer disabled — see below). |
| **`defaults.py`** | Hardcoded `Drone` → `/World/base_link` binding for legacy tests; production paths use the stage prim via `scene_setup.build_drone_config()`. |

The server does **not** own prim-path bindings. The Isaac layer calls `set_model_def_payload(catalog.pack())` after building `sDataDescriptions` from the interface config.

## Stage-driven config prim

Configuration lives on a USD prim (conventionally `/World/NatNetInterface`) with `natnet:*` attributes:

- Server: IP, unicast/multicast mode, command/data ports, publish rate, NatNet version, up-axis, optional pose noise.
- Bodies: multi-apply `natnet:body:<key>:*` fields mapping rigid-body name / streaming ID → target prim path.

`NatNetServerManager` scans the stage, resyncs the catalog when the prim changes, and streams one rigid body per configured target. Missing prims emit **lost** bodies (NaN position, tracking-invalid bit clear) until the target appears — important for Pegasus drones spawned on first Play.

**Up axis:** default `Z` passes Isaac/USD world poses through unchanged (matches `natnet_ros2`). Set `Y` to emulate a Y-up Motive room.

## Streaming model (Isaac)

Inside Kit, the server's background `_data_update_loop` is **disabled** (`auto_stream = False`) because the GIL-starved daemon thread does not reliably transmit frames. Instead, each physics step:

1. `NatNetServerManager.sample_once()` reads prim poses and `enqueue_mocap_data(frame)`.
2. `NatNetUnicastServer.flush_mocap_data()` sends immediately on the physics-step thread.

Outside Isaac (host unit tests), `auto_stream=True` uses the timer-driven loop.

Default Docker sim IP: **`172.31.0.200`** (Isaac container on the AirStack bridge network).

## Enabling in AirStack

**Robot:** `LAUNCH_NATNET=true` in `.env` → `natnet_ros2` in perception bringup. Configure Motive/emulator IP in [`natnet_config.yaml`](../../../../robot/ros_ws/src/perception/natnet_ros2/config/natnet_config.yaml).

**Isaac Sim:** set `ISAAC_SIM_SCRIPT_NAME` to a NatNet launch script (NatNet always starts — no `LAUNCH_NATNET` gate in the script):

| Script | Use |
|--------|-----|
| `example_one_px4_pegasus_natnet_launch_script.py` | Single drone + static `Target` |
| `example_multi_px4_pegasus_natnet_launch_script.py` | `NUM_ROBOTS` drones + shared `Target` (system tests with NatNet use this even for `NUM_ROBOTS=1`) |

Baseline Pegasus scripts (`example_one_px4_pegasus_launch_script.py`, `example_multi_px4_pegasus_launch_script.py`) have **no** NatNet integration.

Convenience bundle for NatNet + external-vision PX4 SITL:

```bash
airstack up --env-file overrides/isaac-optitrack-simulation.env
```

See [optitrack-development skill](../../../../.agents/skills/optitrack-development/SKILL.md) for wire-protocol details, libNatNet 4.4 unicast quirks, and debugging.

## Usage

### Server only (no Kit)

```python
from optitrack.natnet.emulator import NatNetUnicastServer, make_default_drone_catalog
from optitrack.natnet.emulator.isaac.frames import BodySample, build_frame

server = NatNetUnicastServer(local_interface="172.31.0.200")
server.set_model_def_payload(make_default_drone_catalog().pack())
server.start()

frame = build_frame(0, [BodySample(1, (0, 0, 1), (0, 0, 0, 1))])
server.enqueue_mocap_data(frame)
server.flush_mocap_data()
```

### Isaac launch script

```python
from isaacsim.core.utils.extensions import enable_extension

# Register the extension with Kit before importing from it.
enable_extension("optitrack.natnet.emulator")

from optitrack.natnet.emulator.isaac import start_drone_natnet_server

# Keep a reference to the manager for the sim lifetime.
manager = start_drone_natnet_server(
    stage,
    drones=[("Drone", 1, "/World/drone1/base_link/body")],
    server_ip="172.31.0.200",
)
```

### Kit UI

The extension registers **Window → NatNet Emulator** — a docked panel to create/edit the interface prim, start/stop the server, and view live body readouts. The same `NatNetServerManager` backs both the UI and launch-script paths.

## Protocol notes (unicast, libNatNet 4.4)

| Port | Traffic |
|------|---------|
| **1510** | Command: `NAT_CONNECT`, `NAT_REQUEST_MODELDEF`, keepalives, echo |
| **1511** | Data: `NAT_FRAMEOFDATA` — **must** be sent from a socket bound to the data port |

Frames sent from the command socket are silently dropped by libNatNet. Every frame payload must include the 4-byte end-of-data tag expected by the C SDK unpacker.

Full handshake layouts and sniffing workflow: [optitrack-development skill](../../../../.agents/skills/optitrack-development/SKILL.md).

## Tests

| Tier | Mark | What |
|------|------|------|
| Unit | `unit` | Serializers, protocol, config, USD authoring, catalog, pose sampling, server lifecycle, scene setup |
| Integration | `integration` | Host emulator → robot `natnet_ros2` pose Hz |

Co-located tests live in `test/`. Pytest discovers them via thin proxies in [`tests/sim/optitrack_natnet_emulator/`](../../../../tests/sim/optitrack_natnet_emulator/).

```bash
# Unit (no Docker / no SDK)
pytest tests/sim/optitrack_natnet_emulator/ -m unit -v

# Integration (robot container + NatNet SDK)
pytest tests/integration/natnet/ -m integration -v
```

Representative unit modules: `test_unicast_protocol.py`, `test_pose_streaming.py`, `test_interface_authoring.py`, `test_server_lifecycle.py`, `test_scene_setup.py`.

## Reference material

- User guide: [`docs/simulation/isaac_sim/natnet_emulator.md`](../../../../docs/simulation/isaac_sim/natnet_emulator.md)
- Robot client: [`natnet_ros2/README.md`](../../../../robot/ros_ws/src/perception/natnet_ros2/README.md)
- Integration tier: [`tests/integration/natnet/README.md`](../../../../tests/integration/natnet/README.md)

OptiTrack SDK sample headers may exist locally under `NatNetClientSDK/` for wire-format reference; they are **not** redistributed by AirStack (proprietary license).
