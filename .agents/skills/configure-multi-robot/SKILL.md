---
name: configure-multi-robot
description: Configure, name, and isolate multiple robots in AirStack — fleet files (config/fleets/, airstack up --fleet) first, legacy NUM_ROBOTS second. Use whenever launching multi-robot, multiple robots, swarm, or fleet scenarios; mixing different stacks/vehicles per robot (heterogeneous fleets, split placement via hosts:); setting ROBOT_NAME; debugging cross-robot topic collisions; choosing a ROS_DOMAIN_ID; or namespacing topics, TF frames, and DDS bridges across robots.
license: Apache-2.0
metadata:
  author: AirLab CMU
  repository: AirStack
---

# Skill: Configure Multi-Robot Setup

## When to Use

Reach for this skill any time you:

- Spawn more than one robot in simulation (`--fleet <name>` or legacy `NUM_ROBOTS > 1`)
- Need robots that differ (stack, vehicle, or offboard placement) — a **heterogeneous fleet**
- Deploy multiple physical aircraft (VOXL, Jetson, etc.)
- Debug topic collisions, missing topics on `/robot_2/...`, or "two robots talking on the same topic"
- Write a new launch file or YAML config that hardcodes a topic path
- Vary `--num-robots` in the system test suite (`tests/`)
- Add a node that subscribes to or publishes a robot-specific topic
- Set up the `desktop_split` / `offboard` profile where global planning runs on a separate machine

If you only ever touch one robot, you can usually skip this skill — but the moment a second drone enters the picture, every assumption about hardcoded `/drone1/...` topic names becomes a bug.

## Prerequisites

- Familiarity with the AirStack docker-compose layout (`.env`, `robot/docker/`, `simulation/{isaac-sim,ms-airsim}/`)
- Basic understanding of ROS 2 namespaces and TF frame names
- You have already read [`docs/robot/docker/robot_identity.md`](../../../docs/robot/docker/robot_identity.md), or are willing to as you go — that file is the canonical reference for the resolution mechanism

## Fleet-First: Declare the Whole Deployment in One File

Since RFC #380 P6, the preferred way to run multiple robots is a **fleet file**
(`config/fleets/*.yaml`): who exists, which vehicle each flies, which stack
each runs, and which ground host runs each split stack's offboard half. Full
guide: [`docs/development/fleets.md`](../../../docs/development/fleets.md).

```bash
airstack fleet list                              # what exists + shape
airstack up --fleet sim_one_default --sim isaac  # 1 robot, today's defaults
airstack up --fleet sim_three_mixed --sim isaac  # heterogeneous: 3 robots, 3 stacks + a split
```

What `--fleet <name>` does:

- validates the fleet (named errors), exports `FLEET_CONFIG_FILE` (container
  path), and **derives `NUM_ROBOTS`** from the robot count (explicit env
  `NUM_ROBOTS` still wins, with a banner)
- on Isaac, switches an untouched-default `ISAAC_SIM_SCRIPT_NAME` to the
  generic fleet spawner `fleet_spawn.py` (spawns/scene/sensors from the fleet
  + vehicle files)
- **homogeneous** fleets (same vehicle + stack everywhere) keep
  `deploy.replicas`; each replica resolves its own entry via
  `tools/fleet/resolve_fleet.py` in `.bashrc` (opt-in: only when
  `FLEET_CONFIG_FILE` is set)
- **heterogeneous** fleets get generated per-robot services
  (`airstack fleet generate <fleet>` →
  `.airstack/generated/docker-compose.fleet.yaml`, auto-included; the `fleet`
  compose profile replaces `desktop`)
- a robot with `hosts: {offboard: gcs}` on a split stack gets its `onboard`
  entry point, and the named ground host gets a service running the same
  stack with `AIRSTACK_STACK_ENTRY=offboard` — the declared successor of the
  `desktop_split` / `offboard` profiles

Test harness: `airstack test -m liveliness --fleet sim_three_mixed ...`
passes `FLEET_CONFIG_FILE` + the derived `NUM_ROBOTS`; without `--fleet`,
`--num-robots` behaves exactly as before.

Everything below — the legacy `NUM_ROBOTS` + `robot_name_map` path — remains
the default without a fleet and is still fully supported; the topic/TF
namespacing rules and pitfalls apply identically under both paths.

## How ROBOT_NAME Flows Through the Stack (Legacy Path)

`ROBOT_NAME` is **not** a single static value. It is computed per container at shell start by `robot/docker/.bashrc` and propagated into every ROS launch substitution. When `FLEET_CONFIG_FILE` is set, a fleet branch in `.bashrc` resolves the whole fleet entry first (name, domain, stack placement, vehicle — pre-set env still wins per variable, and failures fall back to the legacy resolver below). The legacy chain:

```
.env  (ROBOT_NAME_MAP_CONFIG_FILE, NUM_ROBOTS)
  │
  ▼
docker-compose.yaml  (ROBOT_NAME_SOURCE=container_name | hostname,
                      deploy.replicas: ${NUM_ROBOTS:-1})
  │
  ▼
robot/docker/.bashrc  (runs on container shell start)
  │
  ├─ ROBOT_NAME already set in env?    →  KEEP IT, skip resolution entirely
  │  (guard: `if [ -z "${ROBOT_NAME:-}" ]`; lets an override/compose pin the name)
  │
  ├─ ROBOT_NAME_SOURCE=container_name  →  resolve `hostname` back to docker container name
  │                                      (e.g. `airstack-robot-desktop-1`)
  ├─ ROBOT_NAME_SOURCE=hostname        →  use OS hostname directly (`robot-1` on real HW)
  │
  ▼
robot/docker/robot_name_map/resolve_robot_name.py
  applies regex rules from $ROBOT_NAME_MAP_CONFIG_FILE
  │
  ▼
exports ROBOT_NAME=robot_<N>     (e.g. robot_1)
        ROS_DOMAIN_ID=<N>        (e.g. 1)
  │
  ▼
ros2 launch reads $(env ROBOT_NAME) → topic remappings, push_ros_namespace,
                                       MAVROS FCU URLs, DDS allowlists, etc.
```

The default mapping rule in [`robot/docker/robot_name_map/default_robot_name_map.yaml`](../../../robot/docker/robot_name_map/default_robot_name_map.yaml):

```yaml
- pattern: '.*robot-.*(\d+)'
  robot: 'robot_{1}'
  domain_id: '{1}'
- pattern: '.*'           # catch-all
  robot: 'unknown_robot'  # must be a valid ROS token (no hyphen) or launch fails
  domain_id: '0'
```

So `airstack-robot-desktop-1` → `ROBOT_NAME=robot_1`, `ROS_DOMAIN_ID=1`. Replica `2` → `robot_2`, domain `2`. Etc. **The container name is the source of truth in simulation.**

The top-level [`autonomy_bringup/launch/robot.launch.xml`](../../../robot/ros_ws/src/autonomy_bringup/launch/robot.launch.xml) then pushes this name as the root namespace for every node it spawns:

```xml
<push_ros_namespace namespace="$(env ROBOT_NAME)" />
```

Every layer-bringup launch file underneath inherits that namespace, and every cross-robot remap uses `/$(env ROBOT_NAME)/...` to reach back out to the absolute path.

## Configuring a Single Robot

The default config in `.env` already runs one robot. You almost never need to set `ROBOT_NAME` directly; instead let the resolver compute it:

```bash
# .env
NUM_ROBOTS="1"
ROBOT_NAME_MAP_CONFIG_FILE="default_robot_name_map.yaml"
```

```bash
airstack up
docker exec airstack-robot-desktop-1 bash -c 'echo $ROBOT_NAME $ROS_DOMAIN_ID'
# robot_1 1
```

If you need a non-default name (custom hostname scheme on a physical robot, or you want `drone_alpha` instead of `robot_1`), you have two options:

1. **Write a mapping YAML** in `robot/docker/robot_name_map/` and point `ROBOT_NAME_MAP_CONFIG_FILE` at it. Preferred when the name should be derived from the machine (hostname/container) — keeps the resolver in charge of `ROS_DOMAIN_ID` co-assignment.
2. **Rename the device** so the default map resolves it. On real hardware
   (`ROBOT_NAME_SOURCE=hostname`) the OS hostname *is* the identity, so
   `hostnamectl set-hostname robot-1` is a complete, one-time fix — and it scales to a
   fleet, since `robot-2` and `robot-3` then resolve on their own.

!!! danger "Setting `ROBOT_NAME` in an env file does nothing"
    No compose service declares `ROBOT_NAME` or `ROS_DOMAIN_ID` in its `environment:`
    block, and Docker Compose only injects a variable into a container if some service
    names it there. Putting `ROBOT_NAME=robot_1` in an override `.env` sets it for
    **compose's own interpolation**, not for the container — `.bashrc` sees it unset,
    the map lookup runs anyway, and there is no error. The robot simply comes up under
    the resolved name instead of yours.

    `overrides/l4t-px4-realrobot.env` used to ship `ROBOT_NAME` / `ROS_DOMAIN_ID` on
    this basis; they never had any effect and have been removed. Use a hostname or a
    map file instead.

    The general lesson applies to **any** deployment knob: it needs a declaration in
    the service's `environment:` *and* a consumer that reads it. Always
    [verify](#verification-commands) rather than assuming.

**Never hardcode `ROBOT_NAME` on a service in compose either.** `robot-desktop` and
friends are reused for every replica, so a pinned name there would collapse all robots
onto one name and domain and silently break multi-robot. Identity must come from
something that differs per container — the container name in sim, the device hostname on
real hardware — or from a map rule that derives it.

For a one-off override (e.g. ad hoc debugging), pass it to the shell directly, which
does work because `docker exec -e` sets it in the process environment:

```bash
docker exec -e ROBOT_NAME=robot_5 -e ROS_DOMAIN_ID=5 -it airstack-robot-desktop-1 bash
```

## Launching Multiple Robots (Legacy `NUM_ROBOTS` Path)

Prefer `airstack up --fleet <name>` (above). Without a fleet, AirStack launches multiple robots as **replicas of the same container**, not as multiple namespaces inside one container — which is also why replicas can only ever be *identical* robots (heterogeneous fleets need the generated per-robot services). Look at [`robot/docker/docker-compose.yaml`](../../../robot/docker/docker-compose.yaml):

```yaml
robot-desktop:
  ...
  deploy:
    replicas: ${NUM_ROBOTS:-1}
```

So `NUM_ROBOTS=3 airstack up` produces **three** robot containers (`airstack-robot-desktop-1`, `-2`, `-3`), each with its own `ROBOT_NAME` and its own `ROS_DOMAIN_ID`. Each container runs the full autonomy stack independently. Cross-robot communication, when needed, goes through the DDS router (see the shared allowlist [`autonomy_bringup/config/dds_router.yaml`](../../../robot/ros_ws/src/autonomy_bringup/config/dds_router.yaml)) which bridges allowlisted topics from each per-robot domain into a shared GCS domain.

```bash
airstack up --sim isaac --robots 3   # sets NUM_ROBOTS and the multi-drone Isaac script together
docker ps --format '{{.Names}}' | grep robot-desktop
# airstack-robot-desktop-1
# airstack-robot-desktop-2
# airstack-robot-desktop-3
```

The simulator side has to spawn matching vehicles — see [Sim-Side Robot Spawning](#sim-side-robot-spawning).

### Full vs. lite vs. split topologies (stacks)

Topology is selected by **stack** — the legacy `AUTONOMY_ROLE` role dispatch was removed (a set `AUTONOMY_ROLE` is now a preflight error): `--stack full_default` (the no-stack default) runs everything on the machine, `--stack lite_default` runs the lite set, `--stack lite_offload_global:onboard|:offboard` is the split pair, and a fleet entry's `hosts: {offboard: <ground>}` declares the split *placement* (see Fleet-First above).

| Stack | What runs onboard | What runs offboard | When to use |
|-------|-------------------|--------------------|-------------|
| `full_default` | interface, sensors, perception, local, **global**, behavior, logging | nothing | Sim/dev desktop, autonomous Jetson with enough compute, single-machine deployments |
| `lite_default` | interface, sensors, perception, local, behavior | nothing (no global anywhere) | Compute-constrained vehicle flying task-driven missions |
| `lite_offload_global` (`:onboard` + `:offboard`) | interface, sensors, perception, local, behavior | global planning + mapping | VOXL / lite Jetson where global planning is offloaded to a ground station; `desktop_split` profile for debugging the split |

The split is significant for multi-robot: with `lite_offload_global`, **one offboard container is launched per robot** (also via `replicas: ${NUM_ROBOTS}`), all on `ROS_DOMAIN_ID=0`, and each bridges into its own per-robot onboard domain via the DDS-router config generated from the stack's `bridge.yaml` (`python3 tools/gen_dds_router.py stacks/lite_offload_global/bridge.yaml` — the generated allowlist deliberately drops the legacy split's `set_trajectory_mode` crossing, doctor hard gate #2). See [`docs/robot/autonomy_modes.md`](../../../docs/robot/autonomy_modes.md) for the profile matrix.

## Topic and TF Namespacing

### Topics

Every cross-module topic must be prefixed with `/$(env ROBOT_NAME)/...`. The standard topics used across the stack (also catalogued in `AGENTS.md` under "Standard Topic Patterns"):

| Topic Pattern | Type | Purpose |
|---|---|---|
| `/{robot_name}/odometry` | `nav_msgs/Odometry` | Robot state estimate |
| `/{robot_name}/odometry_conversion/odometry` | `nav_msgs/Odometry` | Reframed odometry into AirStack frames |
| `/{robot_name}/global_plan` | `nav_msgs/Path` | Global waypoint path |
| `/{robot_name}/trajectory_controller/trajectory_override` | `airstack_msgs/TrajectoryOverride` | Direct trajectory commands |
| `/{robot_name}/trajectory_controller/trajectory_segment_to_add` | `airstack_msgs/TrajectorySegment` | Planned segment |
| `/{robot_name}/trajectory_controller/look_ahead` | `geometry_msgs/PointStamped` | Look-ahead point |
| `/{robot_name}/interface/mavros/local_position/odom` | `nav_msgs/Odometry` | MAVROS-published odom |
| `/{robot_name}/tasks/takeoff` | `task_msgs/action/TakeoffTask` | Takeoff action server |
| `/{robot_name}/tasks/land` | `task_msgs/action/LandTask` | Landing action server |

Pattern in launch XML — do this in **every** new module:

```xml
<remap from="odometry" to="/$(env ROBOT_NAME)/odometry_conversion/odometry" />
<remap from="global_plan" to="/$(env ROBOT_NAME)/global_plan" />
```

In node code, subscribe/publish using **relative names** (e.g. `odometry`) and let the launch file remap. Never write `self.create_subscription(..., "/drone1/odometry", ...)`.

### TF frames

TF frames in AirStack are **also** namespaced under the robot, but the namespacing happens because TF in ROS 2 honors the publishing node's namespace. The top-level launch pushes `$(env ROBOT_NAME)` as namespace, so a node publishing `base_link` ends up with the resolved frame `robot_1/base_link`.

Standard frame names you will see (per robot):

- `{robot_name}/base_link` — body-fixed frame
- `{robot_name}/base_link_stabilized` — yaw-only-rotated body frame
- `{robot_name}/odom` — odometry origin
- `{robot_name}/look_ahead_point_stabilized` — controller look-ahead

Two static frames are **shared** across robots:

- `world` — global root
- `map` — global map frame, anchored at `world`

The static `world` → `map` broadcaster is launched once per robot inside [`robot.launch.xml`](../../../robot/ros_ws/src/autonomy_bringup/launch/robot.launch.xml) — multiple robots publish the identical transform, which TF accepts as redundant. Do not rename `map` per-robot; many global planners and the GCS assume `map` is the shared global frame.

If you write a node that hardcodes a TF frame string, prefer relative frame IDs (`base_link`, `odom`) over absolute ones — the namespace prefix gets added automatically. If you must use an absolute name, build it from the env var:

```python
robot_name = os.environ["ROBOT_NAME"]
self.target_frame = f"{robot_name}/base_link"
```

## Sim-Side Robot Spawning

Both simulators read `NUM_ROBOTS` from the environment and spawn matching vehicles named `robot_1`, `robot_2`, … so the names line up with what the resolver assigns to robot containers.

### Microsoft AirSim (legacy)

[`simulation/ms-airsim/config/generate_settings.py`](../../../simulation/ms-airsim/config/generate_settings.py) reads `NUM_ROBOTS` and renders [`settings.json.j2`](../../../simulation/ms-airsim/config/settings.json.j2) into AirSim's `settings.json`. The Jinja loop produces one `Vehicles.robot_<i>` block per robot, each with its own `TcpPort` (`4561 + i`), `ControlPortLocal` (`24541 + i`), and spawn offset (`Y = (i-1) * spawn_spacing`):

```jinja
{% for i in range(1, num_robots + 1) %}
"robot_{{ i }}": {
  "VehicleType": "PX4Multirotor",
  "TcpPort": {{ 4560 + i }},
  ...
  "Y": {{ (i - 1) * spawn_spacing }}
}
{% endfor %}
```

The `ms-airsim` container's `entrypoint.sh` (in `simulation/ms-airsim/docker/`) loops `for i in $(seq 1 "$NUM_ROBOTS")` to start one PX4 SITL instance per vehicle. AirSim binds them via the per-robot TCP ports.

### Isaac Sim (Pegasus)

With a fleet, [`fleet_spawn.py`](../../../simulation/isaac-sim/launch_scripts/fleet_spawn.py) is selected automatically: spawn positions come from each robot's `spawn:`, the scene from `sim.scene`, and sensor toggles from the vehicle manifests (any `lidar*` sensor enables the RTX lidar subgraph — the per-vehicle `ENABLE_LIDAR` equivalent). The legacy path:

[`simulation/isaac-sim/launch_scripts/example_multi_px4_pegasus_launch_script.py`](../../../simulation/isaac-sim/launch_scripts/example_multi_px4_pegasus_launch_script.py) reads `NUM_ROBOTS` and calls `spawn_drone(i)` in a loop. Each drone is created with `robot_name=f"robot_{index}"`, `vehicle_id=index`, `domain_id=index`, and an X offset for spacing:

```python
NUM_ROBOTS = int(os.environ.get("NUM_ROBOTS", "1"))
...
for i in range(1, NUM_ROBOTS + 1):
    spawn_drone(i)
```

To use the multi-drone launcher, either launch with `airstack up --sim isaac --robots N` (which selects it automatically) or set in `.env`:

```
ISAAC_SIM_SCRIPT_NAME="example_multi_px4_pegasus_launch_script.py"
```

(The default `example_one_px4_pegasus_launch_script.py` only spawns one; `airstack up` preflight rejects `NUM_ROBOTS>1` with a single-drone script.)

### Test harness

The `airstack_env` fixture in [`tests/conftest.py`](../../../tests/conftest.py) parametrizes tests over `(sim, num_robots, iteration)` and sets:

```python
env_overrides = {
    "NUM_ROBOTS": str(num_robots),
    ...
}
```

Tests that act on robots iterate `n=1..num_robots` and address them as `/robot_{n}/...` directly (see `_takeoff_one_robot` in `tests/system/test_takeoff_hover_land.py`). The test sets `ROS_DOMAIN_ID=n` for each per-robot subprocess (`domain_id=n` in `ros2_exec(...)`), matching what the resolver assigned inside the container. **If you write a new test that talks to a robot, follow this same `domain_id=n` + `/robot_{n}/...` pattern.**

CLI passthrough:

```bash
airstack test -m takeoff_hover_land --sim msairsim --num-robots 1,3 -v
airstack test -m liveliness --sim isaacsim --fleet sim_three_mixed -v   # fleet-first
```

With `--fleet`, the fixture sets `FLEET_CONFIG_FILE`, derives `NUM_ROBOTS`
from the fleet, and pins `ISAAC_SIM_SCRIPT_NAME=fleet_spawn.py` on Isaac;
`env["fleet"]` carries the fleet name for tests that need it.

## Common Pitfalls

### 1. Hardcoding the robot name in topics

```xml
<!-- WRONG: only works for the first robot, breaks for /robot_2/... -->
<remap from="odometry" to="/drone1/odometry" />
<remap from="odometry" to="/robot_1/odometry" />

<!-- RIGHT -->
<remap from="odometry" to="/$(env ROBOT_NAME)/odometry" />
```

```python
# WRONG
self.create_subscription(Odometry, "/robot_1/odometry", cb, 10)

# RIGHT (let launch remap a relative name)
self.create_subscription(Odometry, "odometry", cb, 10)
```

### 2. Forgetting `allow_substs="true"` on YAML param files

If your `config/*.yaml` references `$(env ROBOT_NAME)` or other substitutions, you must opt in:

```xml
<param from="$(find-pkg-share my_pkg)/config/my_pkg.yaml" allow_substs="true" />
```

Without `allow_substs="true"`, the substitution string is loaded literally and the node sees `frame_id: "$(env ROBOT_NAME)/base_link"` instead of `frame_id: "robot_1/base_link"`.

### 3. Two robots sharing one ROS_DOMAIN_ID

If two robots share a domain, every topic collides — both `/robot_1/odometry` publishers will be visible to both subscribers, and DDS will sometimes deliver crossed data. The default `robot_name_map` derives the domain from the robot index, so this only happens if you:

- Hardcode `ROS_DOMAIN_ID` in compose to the same value for two replicas
- Use a hostname that doesn't match any rule and falls through to the catch-all (both robots get `unknown_robot`, domain `0`)

Always verify after starting:

```bash
for c in $(docker ps --format '{{.Names}}' | grep robot-desktop); do
  echo "$c: $(docker exec $c bash -c 'echo $ROBOT_NAME $ROS_DOMAIN_ID')"
done
```

### 4. Running multiple `airstack up` instances without isolating domains

Two developers on the same LAN running `airstack up` will see each other's robots if `ROS_DOMAIN_ID` happens to match. The `airstack_network` bridge in compose isolates the **container** network but DDS multicast can still leak over the host's actual network depending on the discovery config. If you are sharing a LAN, set a distinct `ROBOT_NAME_MAP_CONFIG_FILE` that maps to a non-overlapping domain range (e.g. one developer uses domains 1-3, another 11-13).

### 5. Test harness defaulting to 1 robot

`tests/conftest.py` defaults to `--num-robots 1,3`. If you wrote a test that assumes exactly one or exactly three robots, restrict the parametrization in your own test or guard with `pytest.skip(...)`. Don't rely on the harness picking your expected count.

### 6. Forgetting to pass `NUM_ROBOTS` to the simulator container

Both Isaac Sim and AirSim read `NUM_ROBOTS` themselves at startup. `airstack up` and the test harness propagate it for you, but if you start the simulator alone (e.g. `docker compose up isaac-sim`), the simulator will spawn 1 drone regardless of how many robot containers you started. Always set `NUM_ROBOTS` at the top-level invocation, not after the simulator is already running.

### 7. Hardcoded TF frame `base_link` from outside the namespace

A node running outside the robot namespace (e.g. a GCS node, or something launched from `gcs/`) cannot just look up `base_link` — it needs the full `robot_1/base_link`. Build the frame name from the robot you mean to reach:

```python
target_frame = f"{robot_name}/base_link"
```

### 8. `push_ros_namespace` with an absolute remap

This is a common foot-gun:

```xml
<push_ros_namespace namespace="$(env ROBOT_NAME)" />
<node ...>
  <remap from="odometry" to="/odometry" />   <!-- absolute! escapes the namespace -->
</node>
```

Either keep the remap relative (`to="odometry"`) so it joins the namespace, or write the full path explicitly (`to="/$(env ROBOT_NAME)/odometry"`).

### 9. Real robots and the `unknown_robot` fallback

On VOXL/Jetson the service uses `ROBOT_NAME_SOURCE=hostname`, so the **OS hostname** is what gets mapped — not a compose replica index. The stock `default_robot_name_map.yaml` only matches `robot-<n>`, so a device named `airlab-jetson-42` falls through to the catch-all and comes up as **`ROBOT_NAME=unknown_robot`, domain `0`** (with a map that has *no* catch-all, the resolver instead exits non-zero and `ROBOT_NAME` is left unset — same confusing "empty namespace" symptom). This is the usual "why is my real robot `unknown_robot`?" report.

Pick whichever fix matches your topology (see [Configuring a Single Robot](#configuring-a-single-robot)):

- **Quickest, no config:** rename the device — `hostnamectl set-hostname robot-1`. The default map resolves it to `robot_1` on domain 1, and a fleet named `robot-2`, `robot-3`, … resolves the same way with nothing further to maintain.
- **Hostnames you can't change:** ship a mapping YAML matching them and point `ROBOT_NAME_MAP_CONFIG_FILE` at it. Needs no code change — the variable is already forwarded and `robot_name_map/` is bind-mounted into the container — and keeps the resolver co-assigning `ROS_DOMAIN_ID`.

Setting `ROBOT_NAME` in an override env file is **not** an option: nothing declares it in
compose, so it never reaches the container. See the danger note under
[Configuring a Single Robot](#configuring-a-single-robot).

Verify on the device — do this every time, especially after pinning `ROBOT_NAME`, since
a pin that never reached the container fails silently:

```bash
docker exec <container> bash -c 'echo "$(hostname) -> ROBOT_NAME=$ROBOT_NAME ROS_DOMAIN_ID=$ROS_DOMAIN_ID"'
```

If it still reports `unknown_robot` after you set `ROBOT_NAME`, the variable did not
reach the container. Check that the service (or the base compose file it extends)
declares it in `environment:` — see the warning under
[Configuring a Single Robot](#configuring-a-single-robot).

## Pre-Merge Checklist

Before merging a change that touches anything robot-namespaced:

- [ ] No `/drone1/...` or `/robot_1/...` literals in any code, config, or launch file you added or modified — search with `grep -rn '/robot_[0-9]\|/drone[0-9]' <changed paths>`
- [ ] Every cross-module topic uses `$(env ROBOT_NAME)` (in launch files) or a relative name remapped at launch time (in node code)
- [ ] Every YAML config file that references `$(env ...)` is loaded with `allow_substs="true"`
- [ ] TF frames in node code are either relative (`base_link`, `odom`) or built from `os.environ["ROBOT_NAME"]`
- [ ] If you added a new module to a layer bringup, you tested it with `NUM_ROBOTS=2` and confirmed both robots' namespaces look identical under `ros2 node list`
- [ ] If you added a sim launch script, it reads `NUM_ROBOTS` and spawns vehicles named `robot_1`, `robot_2`, … with matching `vehicle_id` / `domain_id`
- [ ] If you added a system test that addresses a robot, it loops over `range(1, num_robots + 1)` and uses `domain_id=n` in `ros2_exec(...)`
- [ ] DDS router allowlists in `autonomy_bringup/config/dds_router.yaml` (or the split stack's `bridge.yaml`) include any new cross-domain topic your module exposes — otherwise it will not appear on the GCS
- [ ] Verified end-to-end: `NUM_ROBOTS=3 airstack up`, then `docker exec airstack-robot-desktop-2 bash -c 'ros2 topic list | grep robot_2'` shows the same topics that `airstack-robot-desktop-1` shows under `robot_1`

## Verification Commands

Quick checks while debugging:

```bash
# Confirm each container resolved a distinct (ROBOT_NAME, ROS_DOMAIN_ID)
for c in $(docker ps --format '{{.Names}}' | grep robot-desktop); do
  docker exec "$c" bash -c 'echo "$(hostname) -> ROBOT_NAME=$ROBOT_NAME ROS_DOMAIN_ID=$ROS_DOMAIN_ID"'
done

# Each robot's nodes (run on the matching domain)
docker exec -e ROS_DOMAIN_ID=1 airstack-robot-desktop-1 bash -c \
  "source /opt/ros/jazzy/setup.bash && ros2 node list"
docker exec -e ROS_DOMAIN_ID=2 airstack-robot-desktop-2 bash -c \
  "source /opt/ros/jazzy/setup.bash && ros2 node list"

# Robots talking to each other? Use the GCS domain (0) and check the router-bridged topics
docker exec airstack-gcs-1 bash -c \
  "source /opt/ros/jazzy/setup.bash && ROS_DOMAIN_ID=0 ros2 topic list | grep -E 'robot_[0-9]+'"

# TF tree for one robot
docker exec -e ROS_DOMAIN_ID=1 airstack-robot-desktop-1 bash -c \
  "source /opt/ros/jazzy/setup.bash && ros2 run tf2_tools view_frames"
```

## References

- [`docs/development/fleets.md`](../../../docs/development/fleets.md) — fleets: hierarchy, file tour, split placement, migration table (fleet-first path)
- [`config/fleets/`](../../../config/fleets/) — `sim_one_default.yaml` (parity with legacy), `sim_three_mixed.yaml` (heterogeneous + split)
- [`tools/fleet/resolve_fleet.py`](../../../tools/fleet/resolve_fleet.py) — fleet-entry resolver (`--table` to inspect, `--validate` to check)
- [`docs/robot/docker/robot_identity.md`](../../../docs/robot/docker/robot_identity.md) — canonical reference for the legacy resolution mechanism
- [`docs/robot/autonomy_modes.md`](../../../docs/robot/autonomy_modes.md) — profile matrix (`desktop`, `desktop_split`, `voxl`, `l4t`, `offboard`)
- [`robot/docker/robot_name_map/`](../../../robot/docker/robot_name_map/) — mapping YAMLs and `resolve_robot_name.py`
- [`robot/ros_ws/src/autonomy_bringup/launch/robot.launch.xml`](../../../robot/ros_ws/src/autonomy_bringup/launch/robot.launch.xml) — top-level `push_ros_namespace`
- [`robot/ros_ws/src/autonomy_bringup/config/dds_router.yaml`](../../../robot/ros_ws/src/autonomy_bringup/config/dds_router.yaml) — cross-domain allowlist pattern
- [`simulation/ms-airsim/config/generate_settings.py`](../../../simulation/ms-airsim/config/generate_settings.py) and [`settings.json.j2`](../../../simulation/ms-airsim/config/settings.json.j2)
- [`simulation/isaac-sim/launch_scripts/example_multi_px4_pegasus_launch_script.py`](../../../simulation/isaac-sim/launch_scripts/example_multi_px4_pegasus_launch_script.py)
- [`tests/conftest.py`](../../../tests/conftest.py) — `airstack_env` fixture and `--num-robots` parametrization

## Related Skills

- [integrate-module-into-layer](../integrate-module-into-layer) — every remap in your module must use `$(env ROBOT_NAME)`
- [write-launch-file](../write-launch-file) — patterns for env substitution and namespace pushing
- [test-in-simulation](../test-in-simulation) — multi-robot test scenarios
- [debug-module](../debug-module) — diagnosing topic/namespace issues
