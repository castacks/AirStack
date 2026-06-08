# NatNet ↔ robot autonomy integration

Drives the Python NatNet wire-protocol emulator against `natnet_ros2_node`
running with the robot autonomy stack. First resident of the
[`integration`](../README.md) tier.

Marks: `integration` (tier) + `natnet` (scenario).

## What it verifies

1. Emulator serves the default Drone MODELDEF and streams frames with rigid-body `ID=1`.
2. `natnet_ros2_node` connects via unicast, parses MODELDEF, and publishes
   `/{ROBOT_NAME}/perception/optitrack/Drone` at ≥ 5 Hz.

## Requirements

- A NatNet UDP server (`NatNetUnicastServer`) on the host — started by the test.
- `natnet_ros2_node` (OptiTrack NatNet SDK) built in the robot container.
- Docker bridge routing from container → host gateway.

The robot container is provided by the `robot_autonomy_stack` fixture (see the
[tier README](../README.md)). The test **skips** when `natnet_ros2_node` is not
built (the SDK is license-gated, fetched via `airstack setup --natnet`).

## Running

```bash
# Reuse an existing robot container:
AUTOLAUNCH=false airstack up robot-desktop
pytest tests/integration/natnet/ -m natnet -v

# Or let the harness bring the container up/down:
pytest tests/integration/natnet/ -m natnet --run-integration -v
```

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│ Host (pytest)                                                │
│  NatNetUnicastServer — MODELDEF bytes cache + frame queue   │
└───────────────────────────┬─────────────────────────────────┘
                            │ UDP (docker bridge gateway IP)
┌───────────────────────────▼─────────────────────────────────┐
│ Robot container (autonomy stack)                             │
│  natnet_ros2_node (libNatNet 4.4 unicast)                   │
│  → /{ROBOT_NAME}/perception/optitrack/Drone                 │
└─────────────────────────────────────────────────────────────┘
```

**Catalog ownership:** The server holds a MODELDEF **wire cache** only. Scene
semantics (prim paths, body names/IDs) belong in the future Isaac Sim wrapper,
which calls `set_model_def_payload()`. See
[`defaults.py`](../../../simulation/isaac-sim/extensions/optitrack.natnet.emulator/optitrack/natnet/emulator/defaults.py)
for hardcoded Drone reference constants used in tests.

## Future: Isaac-wrapped variant + liveliness

Today the NatNet **server** is the host emulator. Once the Isaac-sim emulator
wrapper emits NatNet frames from the simulator, an Isaac-wrapped variant will be
added in this directory, and the gated pose-rate check can additionally surface
as a conditional sentinel in
[`../../system/test_liveliness.py`](../../system/test_liveliness.py) (run only
when `LAUNCH_NATNET=true`).

## libNatNet 4.4 unicast — verified wire contract

The emulator is validated against the **real `libNatNet.so`** (not just the Python
`NatNetClient`) with a minimal C probe that registers `SetFrameReceivedCallback`
and `NatNet_SetLogCallback`. All of the following must hold for the SDK to deliver
frames to the callback:

| Requirement | Why |
|-------------|-----|
| `NAT_CONNECT` → `sSender_Server` (279 B), name `Motive` | libNatNet reads `Motive 3.1 / NatNet 4.4` |
| `NAT_ECHOREQUEST` → `NAT_ECHORESPONSE` (16 B) | Prevents libNatNet assert |
| Frame ends with a **4-byte end-of-data tag** after `params` | libNatNet's frame unpacker reads it; without it the unpacked size mismatches `nDataBytes` and **every frame is silently dropped** |
| `NAT_FRAMEOFDATA` sent from the **data port** (source port == `data_port`) | libNatNet routes unicast frames by the server's data port. Frames sent from the **command** port are treated as command traffic and dropped — no error, no callback |
| `NAT_KEEPALIVE` gets **no reply** | An echo reply makes libNatNet log `Received unrecognized message Message=10` |

With these in place the C probe reports `Server: Motive 3.1.0.0 NatNet 4.4.0.0`,
`data descriptions: 1`, and **~74 Hz** of frame callbacks.

> The lenient Python `NatNetClient` accepts frames *without* the end-of-data tag
> and *on the command port*, which is why it appeared to work while libNatNet did
> not. Always validate against the C SDK.

Rebuild `natnet_ros2` in the robot container after any adapter changes:
`docker exec airstack-robot-desktop-1 bash -lc 'bws --packages-select natnet_ros2'`
