# NatNet ↔ robot autonomy integration

Drives the Python NatNet wire-protocol emulator against `natnet_ros2_node`
running with the robot autonomy stack. First resident of the
[`integration`](../README.md) tier.

Marks: `integration` (tier) + `natnet` (scenario).

## What it verifies

Two variants, both ending at `/{ROBOT_NAME}/perception/optitrack/Drone` ≥ 5 Hz:

1. **Raw server** (`test_natnet_ros2_receives_drone_pose_hz`) — hand-built frames
   via `NatNetUnicastServer` (no USD): the minimal end-to-end wire check.
2. **Isaac wrapper** (`test_natnet_ros2_receives_isaac_wrapper_pose_hz`) — the full
   new data path: `NatNetServerManager` builds the catalog from a
   `NatNetInterfaceConfig` and samples a moving prim's world pose off an in-memory
   USD stage (`sample_once`, exactly what the in-sim physics-step callback does),
   streaming real frames to the robot client. Skips without `usd-core` (`pxr`).
   Exact pose-value fidelity is covered hermetically by the package's
   `test_pose_streaming.py` loopback.

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

## Liveliness sentinel (sim end-to-end)

The integration tier drives the emulator **host-side** (no sim/GPU). The matching
in-sim check is a conditional sentinel in
[`../../system/test_liveliness.py`](../../system/test_liveliness.py):
`TestLiveliness::test_natnet_pose_alive` asserts
`/{robot}/perception/optitrack/<body>` is live per robot. It is **gated on
`LAUNCH_NATNET=true`** (skips otherwise), so normal liveliness runs are unaffected.
Override the body name with `NATNET_BODY_NAME` (default `Drone`).

> The sentinel passes once the Isaac emulator actually streams in-sim — i.e. a
> Pegasus launch script authors a `NatNetInterface` prim and calls
> `NatNetServerManager.start_from_stage()` (the scripting entry point). Until that
> sim auto-start is wired, run the **integration** variants above for robot-level
> coverage.

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
