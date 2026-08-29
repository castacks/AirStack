# Deploying to Hardware

!!! danger "Safety first"
    This tutorial ends with a real drone spinning real propellers. **Keep propellers off
    until Step 8**, and arm only after every check in this page has passed. Fly with a
    safety pilot holding an RC transmitter with a working manual override / kill switch,
    where you are permitted to fly, per local regulations.

This tutorial takes one path from a bench-top Jetson to a first autonomous flight: a Jetson
Orin running the `l4t` profile with the `full_default` stack, connected to a PX4 flight
controller over serial. Other platforms (e.g. ModalAI VOXL2 via the `voxl` profile) follow
the same shape — see [Autonomy Modes](../robot/autonomy_modes.md).

## 1. Prerequisites

- A Jetson ORIN AGX/NX flashed with JetPack (L4T, Ubuntu 22.04) — the tested platform per the [Installation Guide](installation/index.md).
- A PX4-based flight controller wired to a Jetson UART — default `/dev/ttyTHS4` at
  115200 baud; *verify the UART and baud on your hardware, wiring varies by carrier board*.
- You have completed [Getting Started](../getting_started/index.md) in simulation.
- An RC transmitter bound to the flight controller for manual takeover.

**Check:** on the Jetson, `ls -l /dev/ttyTHS4` shows the serial device (or note the device your FCU is actually wired to — you'll need it in Step 4).

## 2. Install AirStack on the Jetson

Follow the [Installation Guide](installation/index.md) — in short:

```bash
git clone --recursive -j8 git@github.com:castacks/AirStack.git
cd AirStack
./airstack.sh setup                          # adds `airstack` to PATH, installs Docker if needed
docker compose --profile l4t pull robot-l4t  # pull the Jetson image
```

Also set `COMPOSE_PROFILES=l4t` in the repo's `.env` so the desktop defaults (`desktop,isaac-sim`) are not active on the Jetson.

**Check:** `docker images | grep robot-l4t` lists the pulled image.

## 3. Set the robot's identity

Real-robot profiles resolve `ROBOT_NAME` and `ROS_DOMAIN_ID` from the **OS hostname**
via the [robot name map](../robot/docker/robot_identity.md). Name the device `robot-<N>`:

```bash
sudo hostnamectl set-hostname robot-1
```

!!! warning "The fallback is silent"
    A hostname that doesn't match the map does **not** error — the default map's catch-all
    resolves it to `ROBOT_NAME=unknown_robot`, `ROS_DOMAIN_ID=0`, and the symptoms only
    surface later (topics under `/unknown_robot`, the `zed-l4t` container on domain 1
    unable to see the stack). Check the mapping now.

**Check:** the resolver maps your hostname as expected (re-verified in Step 6):

```bash
python3 robot/docker/robot_name_map/resolve_robot_name.py $(hostname) \
    robot/docker/robot_name_map/default_robot_name_map.yaml   # → ROBOT_NAME=robot_1 / ROS_DOMAIN_ID=1
```

## 4. Configure the FCU connection

The `robot-l4t` service (`robot/docker/docker-compose.yaml`) runs privileged with
`network_mode: host`, so the Jetson's serial devices are visible inside the container,
and it sets the MAVROS connection for you: `FCU_URL=${FCU_URL:-/dev/ttyTHS4:115200}`
and `TGT_SYSTEM=1`. If your FCU uses a different UART or baud rate, set `FCU_URL` in `.env`
(e.g. `FCU_URL=/dev/ttyTHS0:921600`) — a set `FCU_URL` environment variable is used directly by
`interface.launch.py` instead of deriving a simulation UDP URL (see [Robot Interface](../robot/autonomy/interface/index.md)).

**Check:** `.env` reflects your serial device and baud rate if they differ from the default. MAVROS connectivity itself is verified in Step 6.

## 5. Choose the stack (topology)

The `l4t` profile defaults to the **`full_default`** stack — every autonomy module runs on
the Jetson, no ground station required. The alternative for compute-constrained vehicles is
the **`lite_offload_global`** split (lite modules onboard via `l4t_lite`, global planning on
a ground host via `offboard`) — see [Autonomy Modes](../robot/autonomy_modes.md).
**For a first flight, stay with `full_default`** — one machine, one container, nothing to bridge.

**Check:** you have not set `AIRSTACK_STACK_DIR` or `--stack`, so the default applies.

## 6. Bench test — PROPS OFF

!!! danger "Propellers must be removed for this entire step"

Power the FCU and bring the stack up on the Jetson, then verify in order:

```bash
airstack --profile l4t up && airstack status                              # containers running
docker exec airstack-robot-l4t-1 bash -c 'echo "$ROBOT_NAME / $ROS_DOMAIN_ID"'          # robot_1 / 1
docker exec airstack-robot-l4t-1 bash -c "ros2 node list"                               # stack nodes present
docker exec airstack-robot-l4t-1 bash -c "ros2 topic echo /robot_1/interface/mavros/state --once"      # connected: true
docker exec airstack-robot-l4t-1 bash -c "ros2 topic hz /robot_1/sensors/front_stereo/left/image_rect" # sensors ticking
```

The sensor topic above is the ZED front-stereo stream from the `zed-l4t` container —
substitute your own if your payload differs. Debug the bringup with `airstack logs robot-l4t` /
`airstack connect robot-l4t`. Finally, on a laptop on the same network, start the field GCS (`gcs-real`, host-networked):

```bash
COMPOSE_PROFILES=deploy airstack up
```

**Check:** MAVROS reports `connected: true`, sensor topics tick at a steady rate, and the
robot appears in the GCS 3D view ([Operating the GCS](../gcs/usage/user_interface.md)).

## 7. HITL rehearsal (recommended)

Before flying, rehearse with the *real Jetson and real container* against a simulator:
[Hardware-In-The-Loop Simulation](HITL/index.md). HITL exercises the exact `robot-l4t`
image, identity resolution, networking, and GCS link you just configured — with zero flight risk.

**Check:** the full Takeoff → Navigate → Land flow works end-to-end in HITL.

## 8. First flight checks

Only now, with every previous check green, install propellers.

- **Safety monitor:** the `full_default` stack runs `drone_safety_monitor`, which watches
  the state estimate and pauses the trajectory controller if it times out (it also accepts
  `pause` / `resume` / `rewind` on its `command` topic). Confirm it is in `ros2 node list`.
- **Manual override:** confirm the safety pilot can take over and kill motors from the RC
  transmitter at any time. *AirStack does not configure this — set up and test your FCU's
  RC failsafe / kill switch per its documentation, and verify on your hardware.*
- **Telemetry:** GCS link live, odometry sane (a stationary drone shows a stationary pose), GPS fix acquired if you rely on it.
- **First command:** from the GCS Robot Tasks panel, send a **Takeoff** to a low altitude,
  let it hover, then **Land**. Expand to Navigate missions only after a clean hover.

## Congratulations

Your robot has gone from a bench-top Jetson to an autonomous first flight. From here you can
grow the mission: waypoint routes and geofences from the GCS, split-stack topologies, and multi-robot fleets.

## See Also

- [Robot Identity](../robot/docker/robot_identity.md) — hostname → name/domain mapping in depth
- [Autonomy Modes](../robot/autonomy_modes.md) — profiles, stacks, and split topologies
- [HITL Testing](HITL/index.md) — the pre-flight rehearsal setup
- [Operating the GCS](../gcs/usage/user_interface.md) — commanding and monitoring from Foxglove
- [Data Offloading](data_offloading/index.md) — getting your flight data off the vehicle
