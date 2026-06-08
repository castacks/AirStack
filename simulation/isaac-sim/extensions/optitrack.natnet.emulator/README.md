# OptiTrack NatNet Emulator (Isaac Sim Extension)

Python NatNet **server** emulator for AirStack simulation and integration testing with [`natnet_ros2`](../../../../robot/ros_ws/src/perception/natnet_ros2/).

## Layout

```
optitrack.natnet.emulator/
├── config/extension.toml       # Isaac Sim extension manifest (stub)
├── setup.py
├── optitrack/natnet/emulator/
│   ├── defaults.py             # Test / Isaac reference constants (not used by server)
│   └── server/                 # NatNet UDP server (transport + protocol)
│       ├── natnet_server.py
│       ├── natnet_unicast_server.py
│       ├── natnet_data_types.py
│       ├── natnet_model_types.py
│       └── natnet_server_types.py
└── NatNetClientSDK/            # Reference SDK only (not shipped in wheel)
```

## Responsibilities

| Layer | Role |
|-------|------|
| **Server** (`optitrack.natnet.emulator.server`) | UDP transport, `NAT_CONNECT` / `NAT_SERVERINFO`, MODELDEF **wire cache**, frame relay via `enqueue_mocap_data()` |
| **Isaac wrapper** (planned) | Build catalog from scene config, `set_model_def_payload(catalog.pack())`, sample prims → `enqueue_mocap_data()` |
| **`defaults.py`** | Hardcoded Drone → `/World/base_link` binding for tests and future Isaac wrapper; **not imported by the server** |

The server stores MODELDEF as packed **bytes** (`_model_def_payload`). It does not own prim-path bindings or ctypes catalog copies. The Isaac wrapper (or integration tests) call `set_model_def_payload()` after building `sDataDescriptions`.

## Usage (development)

```python
from optitrack.natnet.emulator import NatNetUnicastServer, TransmissionType, make_default_drone_catalog

server = NatNetUnicastServer(local_interface="172.31.0.200")
# Default Drone MODELDEF is loaded on init; override from Isaac wrapper:
# server.set_model_def_payload(make_default_drone_catalog().pack())
server.start()
# Wrapper enqueues pre-built frames:
# server.enqueue_mocap_data(frame)
```

Default Docker sim IP: `172.31.0.200` (Isaac container on AirStack bridge network).

## Protocol notes

For libNatNet 4.4 **unicast**, the client uses a single UDP socket; server replies and frames go to the client's `NAT_CONNECT` source endpoint on the **command port** (1510). Handlers: `NAT_CONNECT`, `NAT_REQUEST_MODELDEF`, `NAT_KEEPALIVE`. See [optitrack-development skill](../../../../.agents/skills/optitrack-development/SKILL.md).

## Tests

| Tier | Mark | Location |
|------|------|----------|
| Unit (serializers, protocol, payload cache) | `unit` | `test/` + proxy in `tests/sim/optitrack_natnet_emulator/` |
| Integration (natnet_ros2 loopback Hz) | `integration`, `natnet` | `tests/integration/natnet/test_natnet_integration.py` |

```bash
pytest tests/sim/optitrack_natnet_emulator/ -m unit -v
pytest tests/integration/natnet/ -m natnet -v   # requires robot container + NatNet SDK
```

## Reference material

Wire-format reference lives in [`NatNetClientSDK/`](NatNetClientSDK/README.md) (OptiTrack SDK samples; not redistributed by AirStack).
