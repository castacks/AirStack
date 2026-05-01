---
name: configure-multi-robot
description: Configure, name, and isolate multiple robots in AirStack. Use whenever launching multi-robot, multiple robots, swarm, or fleet scenarios; setting ROBOT_NAME; debugging cross-robot topic collisions; choosing a ROS_DOMAIN_ID; or namespacing topics, TF frames, and DDS bridges across robots.
license: Apache-2.0
metadata:
  author: AirLab CMU
  repository: AirStack
---

# Skill: Configure Multi-Robot Setup

## When to Use

Reach for this skill any time you:

- Spawn more than one robot in simulation (`NUM_ROBOTS > 1`)
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

## How ROBOT_NAME Flows Through the Stack

`ROBOT_NAME` is **not** a single static value. It is computed per container at shell start by `robot/docker/.bashrc` and propagated into every ROS launch substitution. The full chain:

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
  robot: 'unknown-robot'
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

If you need a non-default name (custom hostname scheme on a physical robot, or you want `drone_alpha` instead of `robot_1`), write a new mapping YAML in `robot/docker/robot_name_map/` and point `ROBOT_NAME_MAP_CONFIG_FILE` at it. Do **not** hardcode `ROBOT_NAME=...` in compose unless you know what you are doing — it bypasses the resolver and you lose `ROS_DOMAIN_ID` co-assignment.

For a one-off override (e.g. ad hoc debugging):

```bash
docker exec -e ROBOT_NAME=robot_5 -e ROS_DOMAIN_ID=5 -it airstack-robot-desktop-1 bash
```

## Launching Multiple Robots

AirStack launches multiple robots as **replicas of the same container**, not as multiple namespaces inside one container. Look at [`robot/docker/docker-compose.yaml`](../../../robot/docker/docker-compose.yaml):

```yaml
robot-desktop:
  ...
  deploy:
    replicas: ${NUM_ROBOTS:-1}
```

So `NUM_ROBOTS=3 airstack up` produces **three** robot containers (`airstack-robot-desktop-1`, `-2`, `-3`), each with its own `ROBOT_NAME` and its own `ROS_DOMAIN_ID`. Each container runs the full autonomy stack independently. Cross-robot communication, when needed, goes through the DDS router (see [`onboard_all/config/dds_router.yaml`](../../../robot/ros_ws/src/autonomy_bringup/onboard_all/config/dds_router.yaml)) which bridges allowlisted topics from each per-robot domain into a shared GCS domain.

```bash
NUM_ROBOTS=3 airstack up
docker ps --format '{{.Names}}' | grep robot-desktop
# airstack-robot-desktop-1
# airstack-robot-desktop-2
# airstack-robot-desktop-3
```

The simulator side has to spawn matching vehicles — see [Sim-Side Robot Spawning](#sim-side-robot-spawning).

### `onboard_all` vs `onboard_local_offboard_global`

[`autonomy_bringup`](../../../robot/ros_ws/src/autonomy_bringup/) ships two layouts, selected by the `role` arg / `AUTONOMY_ROLE` env var:

| Variant | Role values | What runs onboard | What runs offboard | When to use |
|--------|-------------|-------------------|--------------------|-------------|
| `onboard_all` | `role:=full` | interface, sensors, perception, local, **global**, behavior | nothing | Sim/dev desktop, autonomous Jetson with enough compute, single-machine deployments |
| `onboard_local_offboard_global` | `role:=onboard` (lite) + `role:=offboard` (GCS) | interface, sensors, perception, local, behavior | global planning + mapping | VOXL / lite Jetson where global planning is offloaded to a ground station; `desktop_split` profile for debugging the split |

The split is significant for multi-robot: with `onboard_local_offboard_global`, **one offboard container is launched per robot** (also via `replicas: ${NUM_ROBOTS}`), all on `ROS_DOMAIN_ID=0`, and each bridges into its own per-robot onboard domain via the domain bridge config in `onboard_local_offboard_global/config/dds_router.yaml`. See [`docs/robot/autonomy_modes.md`](../../../docs/robot/autonomy_modes.md) for the profile matrix.

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

[`simulation/isaac-sim/launch_scripts/example_multi_px4_pegasus_launch_script.py`](../../../simulation/isaac-sim/launch_scripts/example_multi_px4_pegasus_launch_script.py) reads `NUM_ROBOTS` and calls `spawn_drone(i)` in a loop. Each drone is created with `robot_name=f"robot_{index}"`, `vehicle_id=index`, `domain_id=index`, and an X offset for spacing:

```python
NUM_ROBOTS = int(os.environ.get("NUM_ROBOTS", "1"))
...
for i in range(1, NUM_ROBOTS + 1):
    spawn_drone(i)
```

To use the multi-drone launcher, set in `.env`:

```
ISAAC_SIM_SCRIPT_NAME="example_multi_px4_pegasus_launch_script.py"
```

(The default `example_one_px4_pegasus_launch_script.py` only spawns one.)

### Test harness

The `airstack_env` fixture in [`tests/conftest.py`](../../../tests/conftest.py) parametrizes tests over `(sim, num_robots, iteration)` and sets:

```python
env_overrides = {
    "NUM_ROBOTS": str(num_robots),
    ...
}
```

Tests that act on robots iterate `n=1..num_robots` and address them as `/robot_{n}/...` directly (see `_takeoff_one_robot` in `tests/test_takeoff_hover_land.py`). The test sets `ROS_DOMAIN_ID=n` for each per-robot subprocess (`domain_id=n` in `ros2_exec(...)`), matching what the resolver assigned inside the container. **If you write a new test that talks to a robot, follow this same `domain_id=n` + `/robot_{n}/...` pattern.**

CLI passthrough:

```bash
airstack test -m takeoff_hover_land --sim msairsim --num-robots 1,3 -v
```

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
- Use a hostname that doesn't match any rule and falls through to the catch-all (both robots get `unknown-robot`, domain `0`)

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

### 9. Hostname doesn't match any rule on real robots

On VOXL/Jetson with `ROBOT_NAME_SOURCE=hostname`, the device hostname must match a rule in the mapping YAML. If `hostname` returns `airlab-jetson-42` and your config only matches `robot-N`, the resolver exits non-zero and `ROBOT_NAME` is unset — the autonomy stack will then launch with empty namespaces and break in confusing ways. Either rename the device or extend the mapping config.

## Pre-Merge Checklist

Before merging a change that touches anything robot-namespaced:

- [ ] No `/drone1/...` or `/robot_1/...` literals in any code, config, or launch file you added or modified — search with `grep -rn '/robot_[0-9]\|/drone[0-9]' <changed paths>`
- [ ] Every cross-module topic uses `$(env ROBOT_NAME)` (in launch files) or a relative name remapped at launch time (in node code)
- [ ] Every YAML config file that references `$(env ...)` is loaded with `allow_substs="true"`
- [ ] TF frames in node code are either relative (`base_link`, `odom`) or built from `os.environ["ROBOT_NAME"]`
- [ ] If you added a new module to a layer bringup, you tested it with `NUM_ROBOTS=2` and confirmed both robots' namespaces look identical under `ros2 node list`
- [ ] If you added a sim launch script, it reads `NUM_ROBOTS` and spawns vehicles named `robot_1`, `robot_2`, … with matching `vehicle_id` / `domain_id`
- [ ] If you added a system test that addresses a robot, it loops over `range(1, num_robots + 1)` and uses `domain_id=n` in `ros2_exec(...)`
- [ ] DDS router allowlists in `onboard_all/config/dds_router.yaml` (or the split equivalent) include any new cross-domain topic your module exposes — otherwise it will not appear on the GCS
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

- [`docs/robot/docker/robot_identity.md`](../../../docs/robot/docker/robot_identity.md) — canonical reference for the resolution mechanism
- [`docs/robot/autonomy_modes.md`](../../../docs/robot/autonomy_modes.md) — profile matrix (`desktop`, `desktop_split`, `voxl`, `l4t`, `offboard`)
- [`docs/tutorials/multi_robot_simulation.md`](../../../docs/tutorials/multi_robot_simulation.md) — hands-on walkthrough
- [`robot/docker/robot_name_map/`](../../../robot/docker/robot_name_map/) — mapping YAMLs and `resolve_robot_name.py`
- [`robot/ros_ws/src/autonomy_bringup/launch/robot.launch.xml`](../../../robot/ros_ws/src/autonomy_bringup/launch/robot.launch.xml) — top-level `push_ros_namespace`
- [`robot/ros_ws/src/autonomy_bringup/onboard_all/config/dds_router.yaml`](../../../robot/ros_ws/src/autonomy_bringup/onboard_all/config/dds_router.yaml) — cross-domain allowlist pattern
- [`simulation/ms-airsim/config/generate_settings.py`](../../../simulation/ms-airsim/config/generate_settings.py) and [`settings.json.j2`](../../../simulation/ms-airsim/config/settings.json.j2)
- [`simulation/isaac-sim/launch_scripts/example_multi_px4_pegasus_launch_script.py`](../../../simulation/isaac-sim/launch_scripts/example_multi_px4_pegasus_launch_script.py)
- [`tests/conftest.py`](../../../tests/conftest.py) — `airstack_env` fixture and `--num-robots` parametrization

## Related Skills

- [integrate-module-into-layer](../integrate-module-into-layer) — every remap in your module must use `$(env ROBOT_NAME)`
- [write-launch-file](../write-launch-file) — patterns for env substitution and namespace pushing
- [test-in-simulation](../test-in-simulation) — multi-robot test scenarios
- [debug-module](../debug-module) — diagnosing topic/namespace issues
