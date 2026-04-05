# Onboard/Offboard Distributed Computing 

AirStack uses a **role** system to control which planning modules launch inside each container.
The role is hardcoded per compose service — no environment variables need to be set by hand.

| Role | Value | What runs |
|---|---|---|
| **Full** | `full` | Every autonomy module: interface, sensors, perception, local planning, global planning, behavior |
| **Onboard** | `onboard` | Lite modules only: interface, sensors, perception, local planning, behavior — no global planner |
| **Offboard** | `offboard` | Global planner only — runs on the GCS paired with onboard robots |

---

## Compose profiles

Profiles are split into **deployment** and **simulator** categories.

**Deployment profiles:**

| Profile | Machine | Services started | Role(s) |
|---|---|---|---|
| `desktop` | Dev desktop | `robot-desktop` + `gcs` | `full` |
| `desktop_split` | Dev desktop | `robot-desktop-onboard` + `robot-offboard` + `gcs` | `onboard` + `offboard` |
| `l4t` | Jetson | `robot-l4t` + `zed-l4t` | `full` |
| `l4t_lite` | Jetson | `robot-l4t-onboard` + `zed-l4t` | `onboard` |
| `voxl` | VOXL2 | `robot-voxl-onboard` | `onboard` (always) |
| `offboard` | Ground station | `robot-offboard` ×N + `gcs-real` | `offboard` |

**Simulator profiles (mutually exclusive, `desktop`/`desktop_split` only):**

| Profile | Simulator |
|---|---|
| `isaac-sim` | NVIDIA Isaac Sim (Pegasus) |
| `airsim` | Microsoft AirSim (UE4) |
| `simple` | Simple Sim |

Only one simulator profile can be active at a time. `airstack up` will error if multiple are set.

---

## Profile: `desktop` (default)

Standard simulation and development. All autonomy runs in one container per simulated robot.
Combine with a simulator profile.

```
Dev desktop
├── simulator (isaac-sim / airsim / simple)
├── robot-desktop × N   [role: full]
└── gcs
```

```bash
# Isaac Sim (set in .env: COMPOSE_PROFILES="desktop,isaac-sim"):
airstack up

# AirSim:
COMPOSE_PROFILES="desktop,airsim" airstack up

# Multiple simulated robots:
NUM_ROBOTS=3 airstack up
```

Each replica gets a unique `ROBOT_NAME` (`robot_1`, `robot_2`, `robot_3`) and `ROS_DOMAIN_ID` (1, 2, 3)
automatically from the [`robot_name_map`](../robot/docker/robot_identity.md).

---

## Profile: `desktop_split`

Simulates the onboard/offboard split on a single developer machine.
`robot-desktop-onboard` acts as the simulated onboard computer (lite modules only).
`robot-offboard` acts as the GCS containers (global planning only).
Use this to debug the split configuration and domain bridge without needing physical hardware.

```
Dev desktop
├── simulator (isaac-sim / airsim / simple)
├── robot-desktop-onboard × N   [role: onboard, ROS_DOMAIN_ID = 1..N]
├── robot-offboard × N          [role: offboard, ROS_DOMAIN_ID = 0]
└── gcs                         [domain 0]
```

```bash
COMPOSE_PROFILES="desktop_split,isaac-sim" airstack up

# Or:
airstack --profile desktop_split --profile isaac-sim up
```

!!! note "Domain isolation"
    Onboard containers run on `ROS_DOMAIN_ID` 1, 2, 3… (one per robot).
    All offboard containers and the GCS share `ROS_DOMAIN_ID=0`.
    A `domain_bridge` node inside each `robot-offboard` container bridges only the
    necessary topics across the domain boundary to avoid flooding the radio link.

---

## Profile: `l4t` (Jetson, fully autonomous)

All autonomy runs on the Jetson. Use when the Jetson has sufficient compute to run
global planning onboard, or when no GCS is available.

```bash
# On the Jetson:
airstack --profile l4t up
```

---

## Profile: `l4t_lite` + `offboard` (Jetson with GCS offboard)

Lite modules run on the Jetson; global planning runs on the ground station.

```bash
# On the Jetson:
airstack --profile l4t_lite up

# On the ground station (set NUM_ROBOTS to match fleet size):
NUM_ROBOTS=3 airstack --profile offboard up
```

---

## Profile: `voxl` + `offboard`

VOXL2 always runs in onboard-only (lite) mode — it does not have sufficient compute
for global planning. Global planning must always run on the GCS.

```bash
# On the VOXL2:
airstack --profile voxl up

# On the ground station:
NUM_ROBOTS=3 airstack --profile offboard up
```

---

## Launching manually (without AUTOLAUNCH)

If `AUTOLAUNCH=false`, containers start idle. Launch manually inside the container:

```bash
# Full role (desktop or l4t):
ros2 launch autonomy_bringup robot.launch.xml role:=full sim:=false

# Onboard role (VOXL, l4t_lite, desktop_split onboard):
ros2 launch autonomy_bringup robot.launch.xml role:=onboard sim:=false

# Offboard role (GCS):
ros2 launch autonomy_bringup robot.launch.xml role:=offboard sim:=false
```

`desktop_bringup` wraps the above and adds RViz (only when `sim:=true`):

```bash
ros2 launch desktop_bringup robot.launch.xml role:=full sim:=true
```
