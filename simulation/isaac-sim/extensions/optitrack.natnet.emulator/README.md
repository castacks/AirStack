# OptiTrack NatNet Emulator (Isaac Sim Extension)

Python NatNet **server** emulator for AirStack simulation and integration testing with [`natnet_ros2`](../../../../robot/ros_ws/src/perception/natnet_ros2/).

## Layout

```
optitrack.natnet.emulator/
├── config/extension.toml       # Isaac Sim extension manifest (stub)
├── setup.py
├── optitrack/natnet/emulator/
│   └── server/                 # NatNet UDP server (transport + protocol)
│       ├── natnet_server.py
│       ├── natnet_unicast_server.py
│       ├── natnet_data_types.py
│       └── natnet_server_types.py
└── NatNetClientSDK/            # Reference SDK only (not shipped in wheel)
```

## Responsibilities

| Layer | Role |
|-------|------|
| **Server** (`optitrack.natnet.emulator.server`) | UDP transport, `NAT_CONNECT` / `NAT_SERVERINFO`, frame relay via `enqueue_mocap_data()` |
| **Isaac wrapper** (planned) | Build `sFrameOfMocapData` from sim poses; register rigid-body model catalog |

## Usage (development)

```python
from optitrack.natnet.emulator import NatNetUnicastServer, TransmissionType

server = NatNetUnicastServer(local_interface="172.31.0.200")
server.start()
# Wrapper enqueues pre-built frames:
# server.enqueue_mocap_data(frame)
```

Default Docker sim IP: `172.31.0.200` (Isaac container on AirStack bridge network).

## Protocol notes

For libNatNet 4.4 **unicast**, the client uses a single UDP socket; server replies and frames go to the client's `NAT_CONNECT` source endpoint on the **command port** (1510). See [optitrack-development skill](../../../../.agents/skills/optitrack-development/SKILL.md).

## Reference material

Wire-format reference lives in [`NatNetClientSDK/`](NatNetClientSDK/README.md) (OptiTrack SDK samples; not redistributed by AirStack).
