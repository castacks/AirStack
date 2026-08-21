# Onboard/Offboard Distributed Computing

AirStack uses **stacks** ([docs/development/stacks.md](../development/stacks.md))
to control which autonomy modules launch inside each container. Each compose
service carries a default stack — no environment variables need to be set by
hand. (The legacy `AUTONOMY_ROLE` role system was removed; a set
`AUTONOMY_ROLE` is a preflight error.)

| Stack | What runs |
|---|---|
| **`full_default`** | Every autonomy module: interface, sensors, perception, local planning, global planning, behavior, logging — the default when no stack is selected |
| **`lite_default`** | Lite modules only: interface, sensors, perception, local planning, behavior — no global planner |
| **`lite_offload_global:onboard`** | The lite set on the vehicle, bridged to an offboard global half per the stack's `bridge.yaml` |
| **`lite_offload_global:offboard`** | Global planner + world model only — runs on the GCS paired with onboard robots |

---

## Compose profiles

Profiles are split into **deployment** and **simulator** categories.

**Deployment profiles:**

| Profile | Machine | Services started | Default stack(s) |
|---|---|---|---|
| `desktop` | Dev desktop | `robot-desktop` + `gcs` | `full_default` |
| `desktop_split` | Dev desktop | `robot-desktop-onboard` + `robot-offboard` + `gcs` | `lite_default` + `lite_offload_global:offboard` |
| `l4t` | Jetson | `robot-l4t` + `zed-l4t` | `full_default` |
| `l4t_lite` | Jetson | `robot-l4t-onboard` + `zed-l4t` | `lite_default` |
| `voxl` | VOXL2 | `robot-voxl-onboard` | `lite_default` (compute-constrained) |
| `offboard` | Ground station | `robot-offboard` ×N + `gcs-real` | `lite_offload_global:offboard` |

The hardware-profile defaults are redefinable per deployment (env /
`--env-file` / `--stack`).

**Simulator profiles (mutually exclusive, `desktop`/`desktop_split` only):**

| Profile | Simulator |
|---|---|
| `isaac-sim` | NVIDIA Isaac Sim (Pegasus) |
| `ms-ms-airsim` | Microsoft AirSim (legacy) (UE4) |
| `simple` | Simple Sim |

Only one simulator profile can be active at a time. `airstack up` will error if multiple are set.

---

## Profile: `desktop` (default)

Standard simulation and development. All autonomy runs in one container per simulated robot.
Combine with a simulator profile.

```
Dev desktop
├── simulator (isaac-sim / ms-airsim / simple)
├── robot-desktop × N   [stack: full_default]
└── gcs
```

```bash
# Isaac Sim (set in .env: COMPOSE_PROFILES="desktop,isaac-sim"):
airstack up

# Microsoft AirSim (legacy):
COMPOSE_PROFILES="desktop,ms-airsim" airstack up

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
├── simulator (isaac-sim / ms-airsim / simple)
├── robot-desktop-onboard × N   [stack: lite_default, ROS_DOMAIN_ID = 1..N]
├── robot-offboard × N          [stack: lite_offload_global:offboard, ROS_DOMAIN_ID = 0]
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
    The DDS router bridges only the topics listed in the split stack's
    `bridge.yaml` across the domain boundary to avoid flooding the radio
    link — generate its config first:
    `python3 tools/gen_dds_router.py stacks/lite_offload_global/bridge.yaml`
    (or `airstack fleet generate <fleet>`).

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
# Full stack (desktop or l4t) — also the default with no stack args:
ros2 launch autonomy_bringup robot.launch.xml sim:=false \
    stack_dir:=/root/AirStack/stacks/full_default

# Lite stack (VOXL, l4t_lite, desktop_split onboard):
ros2 launch autonomy_bringup robot.launch.xml sim:=false \
    stack_dir:=/root/AirStack/stacks/lite_default

# Offboard half of the split stack (GCS):
ros2 launch autonomy_bringup robot.launch.xml sim:=false \
    stack_dir:=/root/AirStack/stacks/lite_offload_global stack_entry:=offboard
```

`desktop_bringup` wraps the above and adds RViz (only when `sim:=true`);
the stack selection flows through the `AIRSTACK_STACK_DIR` /
`AIRSTACK_STACK_ENTRY` env vars:

```bash
ros2 launch desktop_bringup robot.launch.xml sim:=true
```
