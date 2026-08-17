# NatNet Emulator (OptiTrack Simulation)

The `optitrack.natnet.emulator` Isaac Sim extension lets you test the full
[`natnet_ros2`](../../../robot/ros_ws/src/perception/natnet_ros2/README.md)
perception stack in simulation without a physical OptiTrack system. It runs a
Motive-compatible NatNet UDP server inside Isaac Sim, streams rigid-body poses
sampled from USD prim world transforms, and presents the same wire protocol
that the real Motive software uses.

## How it works

```
Isaac Sim (physics step)
    ↓  sample prim world pose
NatNetServerManager (/World/NatNetInterface USD prim)
    ↓  encode sFrameOfMocapData (NatNet 4.1 wire format)
NatNetUnicastServer  ──UDP 1510/1511──► natnet_ros2_node (robot container)
                                            ↓
                                    /robot_N/perception/optitrack/{body}
                                    /robot_N/interface/mavros/vision_pose/pose
```

Configuration lives on a `/World/NatNetInterface` USD prim with `natnet:*`
attributes. Because it is USD, the config **persists when you save the stage** —
re-opening a `.usd` file restores the catalog and server settings without
re-running any script.

Each physics step the extension:

1. Reads the world transform of each tracked prim.
2. Packs a `sFrameOfMocapData` frame (one `sRigidBodyData` entry per body).
3. Flushes the frame immediately on the physics-step thread (no background timer).

Bodies whose target prim is missing emit a **lost** frame (NaN position,
tracking-invalid bit clear) until the prim appears — this handles Pegasus drones
that are spawned on the first Play tick.

Optional **sensor noise** (`pose_noise_std_meters`, `pose_noise_rotation_deg`)
adds Gaussian position and orientation perturbation to simulate real OptiTrack
measurement uncertainty.

---

## Using the pre-built launch scripts

The easiest way to start is with the provided Pegasus launch scripts. Set
`ISAAC_SIM_SCRIPT_NAME` in your environment or use the convenience override:

```bash
# Single drone, NatNet emulator + PX4 flying on external vision
airstack up --env-file overrides/isaac-optitrack-simulation.env
```

`overrides/isaac-optitrack-simulation.env` sets:

| Variable | Value |
|---|---|
| `NUM_ROBOTS` | `1` |
| `LAUNCH_NATNET` | `true` |
| `PX4_PARAM_SET` | `external-vision` |
| `ISAAC_SIM_SCRIPT_NAME` | `example_one_px4_pegasus_natnet_launch_script.py` |

`PX4_PARAM_SET` selects `simulation/isaac-sim/docker/px4-params/<name>.env`, whose
`PX4_PARAM_*` entries PX4's rcS applies at boot. It defaults to `default`, which is empty,
so every other Isaac Sim run keeps PX4's firmware defaults. Add a file there to save your
own parameter set.

### Available NatNet launch scripts

| Script | Use case |
|---|---|
| `example_one_px4_pegasus_natnet_launch_script.py` | Single drone + static `Target` body |
| `example_multi_px4_pegasus_natnet_launch_script.py` | `NUM_ROBOTS` drones + shared `Target` body |

Both scripts set up GPS origins (via `gps_utils.py`) so the GCS datum matches
PX4, author the NatNet interface, and play the simulation automatically.

!!! note "Baseline scripts have no NatNet"
    `example_one_px4_pegasus_launch_script.py` and
    `example_multi_px4_pegasus_launch_script.py` do **not** include NatNet. Use
    the `*_natnet_*` variants above when you need mocap simulation.

### Body naming

| `NUM_ROBOTS` | Body names streamed |
|---|---|
| 1 | `Drone`, `Target` |
| N > 1 | `Drone1`, `Drone2`, …, `DroneN`, `Target` |

### Changing which body is streamed

The streamed body name and streaming id are **constants in the launch script**
(`NATNET_BODY_NAME` / `NATNET_BODY_ID` / `NATNET_TARGET_NAME`), not environment
variables. They must match a body entry in the robot's profile in
[`natnet_config.yaml`](../../../robot/ros_ws/src/perception/natnet_ros2/config/natnet_config.yaml),
which is the only place the client reads its bodies from — that is what lets each robot
in a multi-robot scene track a different body.

To retarget, edit **both** together:

| Where | What |
|---|---|
| `example_one_px4_pegasus_natnet_launch_script.py` | `NATNET_BODY_NAME`, `NATNET_BODY_ID` |
| `natnet_config.yaml` → `robots.<robot_name>.bodies[]` | `rigid_body_name`, `id` |

!!! warning "A mismatch fails silently"
    The NatNet client filters incoming frames by **numeric id**. If the ids disagree, the
    client connects, the emulator streams, and the pose topic never publishes — with no
    error on either side. When debugging a silent stream, check the id first.

---

## Adding NatNet to your own launch script

Call `author_drone_natnet_interface` after your Pegasus drones are spawned and
before you start the timeline. The extension builds the server from the prim on
Play.

```python
from isaacsim.core.utils.extensions import enable_extension

# Enable the NatNet emulator extension through Kit extension manager.
enable_extension("optitrack.natnet.emulator")

from optitrack.natnet.emulator.isaac import (
    author_drone_natnet_interface,
    author_static_target,
    DEFAULT_TARGET_PATH,
    DEFAULT_TARGET_STREAMING_ID,
)

stage = omni.usd.get_context().get_stage()

# Optional: add a static target body the robot can navigate toward.
author_static_target(stage, DEFAULT_TARGET_PATH, position=(2.0, 0.0, 1.0))

# One entry per drone: (rigid_body_name, streaming_id, target_prim_path)
drones = [
    ("Drone", 1, "/World/drone1/base_link/body"),
]

author_drone_natnet_interface(
    stage,
    drones=drones,
    server_ip="172.31.0.200",   # Isaac container IP on AirStack bridge network
    pose_noise_enabled=True,
    pose_noise_std_meters=0.0005,
    pose_noise_rotation_deg=0.05,
)
```

For multiple drones, add one tuple per drone:

```python
drones = [
    ("Drone1", 1, "/World/drone1/base_link/body"),
    ("Drone2", 2, "/World/drone2/base_link/body"),
    ("Drone3", 3, "/World/drone3/base_link/body"),
]
```

`author_drone_natnet_interface` also accepts the static target as a body — include
it explicitly if you want it:

```python
from optitrack.natnet.emulator.isaac import DEFAULT_TARGET_STREAMING_ID, DEFAULT_TARGET_PATH

drones = [
    ("Drone", 1, "/World/drone1/base_link/body"),
    ("Target", DEFAULT_TARGET_STREAMING_ID, DEFAULT_TARGET_PATH),
]
```

---

## Using the Kit UI panel

The extension registers a docked panel under **Window → NatNet Interface** in
the Isaac Sim menu bar (appears alongside the Pegasus panel).

### Opening the panel

Open Isaac Sim, load your scene, then go to **Window → NatNet Interface**. The
panel docks next to the Property panel in the bottom-right.

### Panel controls

| Button | Action |
|---|---|
| **Create Interface** | Author a fresh `/World/NatNetInterface` prim with current settings |
| **Save** | Push the form fields into the USD prim on the stage |
| **Load from Stage** | Pull the existing prim's values back into the form |
| **Print config** | Log the current config to the console |

**The server's lifetime follows the simulation:** Play builds it from the prim,
Stop shuts it down. The panel's `Server:` label reports which state it is in.

When edits take effect after **Save**:

| Setting | Takes effect |
|---|---|
| Bodies — added, removed, renamed, retargeted | Next frame; the server re-reads the interface as it samples |
| `upAxis`, pose noise | Next frame |
| `serverIp`, ports, `mode` | Next **Play**; these are bound when the server is built |

!!! warning "Restart the robot stack after each Play"
    Clients register with the server instance they connect to, and `natnet_ros2`
    handshakes only until its first success. A client connected during an earlier
    run is unknown to the server built by the next Play and receives no frames; the
    console shows `[Command Handler] Ignoring message N from unregistered client`.

    Assume one client connection per server lifetime: restart the robot container
    after each Stop → Play cycle.

### Server settings

| Field | Default | Description |
|---|---|---|
| Server enabled | `true` | Uncheck to stop the server starting on Play |
| Server IP | `172.31.0.200` | IP the UDP socket binds to (Isaac container address) |
| Mode | `unicast` | `unicast` for direct; `multicast` for broadcast |
| Command port | `1510` | NatNet command channel |
| Data port | `1511` | NatNet data channel (frame stream) |
| Publish rate (Hz) | `120` | Target frame rate |
| Up axis | `Z` | `Z` passes poses through unchanged; `Y` re-axes for Y-up Motive |
| Pose noise enabled | `true` | Add Gaussian noise to simulate real sensor uncertainty |
| Pose noise std (m) | `0.0005` | Position noise std dev (0.5 mm, matching OptiTrack spec) |
| Pose noise rotation (deg) | `0.05` | Orientation noise std dev |

### Adding tracked bodies

1. In the Stage tree, **select the prim** you want to track (e.g. `/World/drone1/base_link/body`).
2. Click **Add body (from selection)** in the panel.
3. Fill in the **rigid body name** (must match the `rigid_body_name` in `natnet_config.yaml`) and **streaming ID**.
4. Click **Save**. The body starts streaming on the next frame.

Each body row shows a live readout of the prim's current world position with a
colour-coded status indicator:

- Green dot — server running, prim found, pose valid
- Grey dot — prim found but server not running
- Red — prim missing or NaN position

### Persistence

After configuring the panel, save your USD stage (**File → Save**). The
`natnet:*` attributes are written into the `.usd` file. Re-opening the stage
restores the full catalog automatically — no script or panel interaction needed
unless you want to change the config.

---

## Configuration reference

`author_drone_natnet_interface` and `build_drone_config` accept these keyword
arguments (all optional):

| Parameter | Default | Description |
|---|---|---|
| `server_ip` | `"172.31.0.200"` | IP to bind the UDP server to |
| `mode` | `"unicast"` | `"unicast"` or `"multicast"` |
| `command_port` | `1510` | NatNet command port |
| `data_port` | `1511` | NatNet data port |
| `publish_rate` | `120.0` | Frame streaming rate (Hz) |
| `up_axis` | `"Z"` | Axis convention (`"Z"` or `"Y"`) |
| `pose_noise_enabled` | `True` | Enable sensor noise |
| `pose_noise_std_meters` | `0.0005` | Position noise std dev (m) |
| `pose_noise_rotation_deg` | `0.05` | Orientation noise std dev (degrees) |

---

## Troubleshooting

**`natnet_ros2` connects but no pose topics appear**

- Check that `rigid_body_name` in `natnet_config.yaml` matches exactly what the
  emulator is streaming (case-sensitive). Run `ros2 topic list` inside the robot
  container and look for `/robot_N/perception/optitrack/...`.

**Server starts but no data arrives in `natnet_ros2`**

- Confirm the server IP matches the Isaac container's address (`172.31.0.200`
  on the AirStack bridge). Check with `docker network inspect airstack_network`.
- The NatNet data port (1511) must be bound to the *data* socket — frames sent
  from the command socket are silently dropped by libNatNet 4.4.

**Data stopped after stopping and replaying the simulation**

- Expected: Stop destroys the server, so the client's registration goes with it,
  and `natnet_ros2` does not re-handshake after its first successful connect. The
  console shows `Ignoring message N from unregistered client`. Restart the robot
  container to force a fresh `NAT_CONNECT`. See the warning under
  [Panel controls](#panel-controls).

**Emulator streams but `vision_pose` is empty**

- `vision_pose` forwarding requires `vision_pose.enabled: true` in the robot's
  `natnet_config.yaml` profile and `SITL_PARAM_PROFILE=px4-vision` so PX4
  accepts external vision instead of GPS.

**Body shows red / NaN in the UI panel**

- The target prim doesn't exist yet. This is normal before pressing Play (Pegasus
  spawns the drone `base_link` prim on the first physics tick). After Play the
  indicator should turn green within one frame.
