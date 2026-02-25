# Autonomy Modes

AirStack supports three **autonomy modes** that control which planning modules are launched inside a robot container.
The same Docker images and launch files are used in all cases — the mode is just an environment variable.

| Mode | `AUTONOMY_MODE` | Launched on | What runs |
|---|---|---|---|
| **All Onboard** | `onboard_all` | Robot or desktop | Every autonomy module: interface, sensors, perception, local planning, global planning, behavior |
| **Onboard Local** | `onboard_local` | Robot (physical) | Interface, sensors, perception, local planning, behavior — **no** global planner |
| **Offboard Global** | `offboard_global` | Ground station (`robot-offboard` service) | Global planner only, one container per robot |

---

## Mode 1 — All Onboard (default)

All planning runs on the robot (or desktop in simulation). This is the default for development and for fully autonomous real-robot deployments.

```
┌─────────────────────────┐
│  Robot / Desktop        │
│  AUTONOMY_MODE=         │
│    onboard_all          │
│  ─────────────────────  │
│  interface              │
│  sensors                │
│  perception             │
│  local planning         │
│  global planning        │
│  behavior               │
└─────────────────────────┘
```

### Simulation (desktop)

```bash
# .env (defaults are already correct)
COMPOSE_PROFILES=desktop
AUTONOMY_MODE=onboard_all
NUM_ROBOTS=1
```

```bash
airstack up
```

### Multiple simulated robots

```bash
NUM_ROBOTS=3 airstack up
```

Each replica gets a unique `ROBOT_NAME` (`robot_1`, `robot_2`, `robot_3`) and `ROS_DOMAIN_ID` (1, 2, 3)
automatically from the [`robot_name_map`](../robot/docker/robot_identity.md).

### Real robot — fully autonomous (Jetson)

```bash
# On the Jetson, in your .env or shell:
AUTONOMY_MODE=onboard_all airstack --profile l4t up
```

### Real robot — fully autonomous (VOXL)

```bash
AUTONOMY_MODE=onboard_all airstack --profile voxl up
```

---

## Mode 2 — Onboard Local + Offboard Global

Local planning (perception, obstacle avoidance, trajectory control) runs on the robot.
Global planning runs on the ground station alongside the GCS, one container per robot.
Use this when the robot has limited compute but you still want reactive local behaviour.

```
┌──────────────────────┐        ┌──────────────────────────────┐
│  Robot (Jetson/VOXL) │        │  Ground Station              │
│  AUTONOMY_MODE=      │        │                              │
│    onboard_local     │◄──────►│  robot-offboard × N replicas │
│  ────────────────────│  LAN / │  AUTONOMY_MODE=              │
│  interface           │  radio │    offboard_global           │
│  sensors             │        │  ──────────────────────────  │
│  perception          │        │  global planning (robot_1)   │
│  local planning      │        │  global planning (robot_2)   │
│  behavior            │        │  ...                         │
└──────────────────────┘        │                              │
                                │  gcs (1×)                    │
                                │  ──────────────────────────  │
                                │  Foxglove / rqt / operator   │
                                └──────────────────────────────┘
```

### On each physical robot

**Jetson:**
```bash
AUTONOMY_MODE=onboard_local airstack --profile l4t up
```

**VOXL:**
```bash
AUTONOMY_MODE=onboard_local airstack --profile voxl up
```

### On the ground station

Set `NUM_ROBOTS` to match the number of robots in the field, then bring up the offboard and GCS services:

```bash
# .env on the ground station
COMPOSE_PROFILES=offboard
NUM_ROBOTS=3          # one robot-offboard replica per robot
AUTONOMY_MODE=onboard_all  # ignored for robot-offboard (it hardcodes offboard_global)
```

```bash
airstack --profile offboard up   # starts robot-offboard x3 + gcs
```

!!! note "Networking"
    The ground station and robots must share a network (LAN or radio bridge) and use the same
    `ROS_DOMAIN_ID` per robot pair. The `robot_name_map` assigns domain ID from the replica index,
    so `robot-offboard-1` uses domain ID 1, matching onboard `robot_1`.

---

## Changing mode at runtime

`AUTONOMY_MODE` can be overridden without editing `.env`:

```bash
AUTONOMY_MODE=onboard_local airstack up          # shell override
airstack --env-file my_override.env up           # env-file override
```

It can also be passed directly as a ROS 2 launch argument if you are launching manually:

```bash
ros2 launch autonomy_bringup robot.launch.xml mode:=onboard_local sim:=false
```

---

## Summary of compose profiles

| Profile | Services started | Typical use |
|---|---|---|
| `desktop` | `robot-desktop`, `isaac-sim` / `simple-sim`, `gcs` | Dev / simulation |
| `simple` | `simple-robot`, `simple-sim`, `gcs` | Lightweight sim without Isaac |
| `l4t` | `robot-l4t`, `zed-l4t` | Jetson onboard |
| `voxl` | `robot-voxl-onboard` | VOXL onboard |
| `offboard` | `robot-offboard` (×N replicas), `gcs` | Ground station — real-world missions |
| `hitl` / `deploy` | `gcs-real` | GCS on a laptop connected directly to robot LAN |
