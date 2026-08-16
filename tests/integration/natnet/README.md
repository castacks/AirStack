# NatNet ↔ robot autonomy integration

Host-side NatNet wire-protocol tests that drive the Python emulator against
`natnet_ros2_node` in a real robot container. First resident of the
[`integration`](../README.md) tier (no sim, no GPU).

Mark: `integration`. Filter this scenario with `tests/integration/natnet/`.

For the **in-sim** end-to-end check (Isaac emulator + full stack), see
[Liveliness sentinel](#liveliness-sentinel-sim-end-to-end) below and
[`tests/system/test_liveliness.py`](../../system/test_liveliness.py).

## What it verifies

Three variants in [`test_natnet_integration.py`](test_natnet_integration.py).
All start a host-side `NatNetUnicastServer`, launch `natnet_ros2_node` in the
robot container pointed at the Docker bridge gateway, and assert a sustained
pose stream at **≥ 5 Hz** on the configured topic(s), e.g.:

- `/{ROBOT_NAME}/perception/optitrack/drone/pose_cov` (wait for first message)
- `/{ROBOT_NAME}/perception/optitrack/drone` (Hz sample)

| Test | Path |
|------|------|
| **`test_natnet_ros2_receives_drone_pose_hz`** | Hand-built `sFrameOfMocapData` frames enqueued on a raw `NatNetUnicastServer` (no USD). Minimal wire + SDK check. |
| **`test_natnet_ros2_receives_isaac_wrapper_pose_hz`** | Full Isaac data path: in-memory USD stage, `NatNetInterfaceConfig`, `author_interface`, `NatNetServerManager.sample_once()` on a moving prim — same sampling logic as the in-sim physics-step callback. Skips without `usd-core` (`pxr`). Pose-value fidelity is covered hermetically by the emulator's `test_pose_streaming.py` loopback. |
| **`test_natnet_ros2_multi_body_drone_and_target`** | Two bodies (drone id 1 + target id 100) with distinct relative topics; asserts both pose streams and that the target's `pose_cov` topic is **absent** (`body_pose_cov=false`). Exercises the multi-body profile + per-body `pose`/`pose_cov` toggles. |

These tests **do not** start the full perception bringup or `LAUNCH_NATNET`; they
exec `natnet_ros2_node` directly with the flattened per-body params
(`body_names`/`body_ids`/`body_topics`/`body_pose`/`body_pose_cov`) and no MAVROS bridge.

## Requirements

- Docker daemon (robot-desktop container reachable from pytest).
- **`natnet_ros2_node` built** in the robot image (OptiTrack NatNet SDK is
  license-gated — run `airstack setup --natnet`, then
  `bws --packages-select natnet_ros2` in the container). Tests **skip** if the
  node binary is missing.
- Host-side emulator package on `PYTHONPATH` (the test adds
  `simulation/isaac-sim/extensions/optitrack.natnet.emulator` — not pip-installed
  on the host).
- Ephemeral UDP ports on the host gateway IP (Docker default route as seen from
  inside the container).

The robot container comes from the shared **`robot_autonomy_stack`** fixture in
[`tests/conftest.py`](../../conftest.py) (see the [integration tier README](../README.md)).

## Running

```bash
# 1. One-time: NatNet SDK + build natnet_ros2 in the robot image
airstack setup --natnet   # or NATNET_ACCEPT_LICENSE=1 airstack setup --natnet
docker exec airstack-robot-desktop-1 bash -lc 'bws --packages-select natnet_ros2'

# 2a. Reuse an existing robot container (fast local iteration):
AUTOLAUNCH=false airstack up robot-desktop
pytest tests/integration/natnet/ -m integration -v

# 2b. Let the harness bring the container up/down:
pytest tests/integration/natnet/ -m integration -v
```
On CI / PR (write access): `/pytest -m integration`

## Architecture

```
┌──────────────────────────────────────────────────────────────┐
│ Host (pytest)                                                │
│  NatNetUnicastServer @ docker bridge gateway IP              │
│    • raw variant: hand-built frame queue                     │
│    • Isaac variant: NatNetServerManager.sample_once(USD)     │
└────────────────────────────┬─────────────────────────────────┘
                             │ UDP unicast (cmd + data ports)
┌────────────────────────────▼─────────────────────────────────┐
│ Robot container (robot-desktop)                              │
│  natnet_ros2_node (libNatNet 4.4 client)                     │
│  → /{ROBOT_NAME}/{body topic}[/pose_cov] per configured body │
└──────────────────────────────────────────────────────────────┘
```

**In sim (liveliness tier):** the server runs inside the Isaac Sim container
(`172.31.0.200` by default). Use a NatNet Pegasus launch script
(`example_one_px4_pegasus_natnet_launch_script.py` or
`example_multi_px4_pegasus_natnet_launch_script.py`); `natnet_ros2` in the
robot stack connects via `natnet_config.yaml` (`server_ip` → emulator IP).

**Catalog / MODELDEF:** The server holds a MODELDEF **wire cache** only
(`set_model_def_payload()`). Scene semantics (body names, streaming IDs, target
prim paths) come from the Isaac layer (`NatNetInterfaceConfig`, USD interface
prim, or launch-script `build_drone_config`). See the
[emulator README](../../../simulation/isaac-sim/extensions/optitrack.natnet.emulator/README.md).

## Liveliness sentinel (sim end-to-end)

The integration tier proves **robot client + host emulator** without Isaac.
The matching **system** check is
`TestLiveliness::test_natnet_pose_alive` in
[`test_liveliness.py`](../../system/test_liveliness.py):

- **Gated on `LAUNCH_NATNET=true`** (skipped otherwise — normal liveliness runs
  are unaffected).
- Asserts `/{robot_n}/{natnet pose topic}/pose_cov` ≥ 5 Hz per robot (the drone
  body's configured topic — default `perception/optitrack/drone`).
- Override the checked topic with `NATNET_POSE_TOPIC` (default
  `perception/optitrack/drone`). The sim body name (`NATNET_BODY_NAME`, default
  `Drone`) is decoupled from the published topic, which the robot profile sets.

Sim auto-start: set `ISAAC_SIM_SCRIPT_NAME` to a NatNet launch script and
`LAUNCH_NATNET=true` on the robot. Convenience bundle:
`airstack up --env-file overrides/isaac-natnet-vision.env` (NatNet script +
PX4 external-vision SITL profile).

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

Full handshake notes and sniffing workflow:
[optitrack-development skill](../../../.agents/skills/optitrack-development/SKILL.md).

## After changing natnet_ros2 or the emulator

Rebuild in the robot container:

```bash
docker exec airstack-robot-desktop-1 bash -lc 'bws --packages-select natnet_ros2'
```

Unit tests (protocol, serializers, Isaac wrapper loopback):

```bash
pytest tests/sim/optitrack_natnet_emulator/ -m unit -v
```
