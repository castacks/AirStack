# DDS Router — Cross-Domain Topic Bridging

AirStack's onboard/offboard split (described in [Autonomy Modes](../autonomy_modes.md)) requires ROS 2 nodes running on different DDS domain IDs to exchange a controlled set of topics. AirStack uses [eProsima DDS Router](https://github.com/eProsima/DDS-Router) to bridge those topics without merging entire domains. We choose this over the native ROS 2 Domain Bridge due to the convenience of bridging services and actions.

---

## How it fits into the system

```
┌─────────────────────────────────────────┐     ┌──────────────────────────────────┐
│  Onboard container  (ROS_DOMAIN_ID = N) │     │  GCS / offboard (ROS_DOMAIN_ID 0)│
│                                         │     │                                  │
│  interface · sensors · perception       │◄───►│  global planner · GCS UI         │
│  local planner · behavior               │     │                                  │
│                          ▲              │     │       ▲                          │
│                          │ DDS Router   │     │       │ DDS Router               │
└──────────────────────────┼────────-─────┘     └───────┼──────────────────────────┘
                           │                            │
                           └──── only the topics in ────┘
                                    the allowlist
```

Each `robot-offboard` container runs one DDS Router instance. It participates in **two local DDS domains simultaneously** (one per `participants` entry in the config) and relays only the topics listed in `allowlist`.

---

## ROS 2 topic naming conventions

eProsima DDS Router operates at the DDS level, so ROS 2 topic names must use their DDS equivalents:

| Prefix | Meaning |
|---|---|
| `rt/` | Standard ROS 2 topic (publisher / subscriber) |
| `rq/<name>Request` | ROS 2 service — request side |
| `rr/<name>Reply` | ROS 2 service — reply side |
| `rs/` | Service handled as a single entity (less common) |
| `<action_topic>/_action/status` | Action — goal status |
| `<action_topic>/_action/feedback` | Action — execution feedback |
| `<action_topic>/_action/send_goal` | Action — initiate goal (service) |
| `<action_topic>/_action/get_result` | Action — retrieve result (service) |
| `<action_topic>/_action/cancel_goal` | Action — cancel goal (service) |

All topics are **bidirectional** by default.

---

## Launch file: `interpolate_dds_router.launch.py`

**Location:** [`robot/ros_ws/src/autonomy_bringup/launch/interpolate_dds_router.launch.py`](../../../../robot/ros_ws/src/autonomy_bringup/launch/interpolate_dds_router.launch.py)

DDS Router consumes a plain YAML file, but the router configs in AirStack need runtime values (domain IDs, robot names) and shared base configs. `interpolate_dds_router.launch.py` adds three capabilities on top of plain YAML before handing the file to `ddsrouter`:

### 1 — Variable substitution

The launch file recognises the same `$(...)` token syntax used in ROS 2 XML launch files:

| Token | Resolved from | Error if missing? |
|---|---|---|
| `$(env VAR_NAME)` | Shell environment variable | Yes — `RuntimeError` |
| `$(var VAR_NAME)` | `args` launch argument (space-separated `key:=value` pairs) | Yes — `RuntimeError` |
| `$(find-pkg-share PKG)` | `ament_index` share directory of `PKG` | Yes — `RuntimeError` |

**Example** — calling the launch file from XML:

```xml
<include file="$(find-pkg-share autonomy_bringup)/launch/interpolate_dds_router.launch.py">
  <arg name="config_file"
       value="$(find-pkg-share autonomy_bringup)/onboard_all/config/dds_router.yaml" />
  <arg name="args" value="gcs_domain:=0" />
</include>
```

### 2 — Config inheritance via `extends:`

A YAML config may declare a top-level `extends:` key pointing to a base config file. The base is loaded first (chains are supported), and then the extending file's keys are **deep-merged** on top:

- **Dict keys** — merged recursively; extending file wins on collision.
- **List values** — base entries kept; extending file entries appended (duplicates skipped).
- `extends:` itself is stripped from the final output.

### 3 — Merge-control YAML tags

Two special YAML tags give fine-grained control when inheritance is used:

| Tag | Applied to | Effect |
|---|---|---|
| `!override` | list or dict | Replaces the base value entirely instead of merging |
| `!reset` | any value | Removes the key from the merged result entirely |

```yaml
# Replace the entire inherited allowlist rather than appending to it:
allowlist: !override
  - name: "rt/my_robot/only_this_topic"

# Remove a key inherited from the base:
some_key: !reset
```

---

## Config files

### `onboard_all/config/dds_router.yaml` — base config

**Location:** [`robot/ros_ws/src/autonomy_bringup/onboard_all/config/dds_router.yaml`](../../../../robot/ros_ws/src/autonomy_bringup/onboard_all/config/dds_router.yaml)

Used when the robot role is `full` or `onboard` and both robot and GCS share the same physical machine or are bridged by this router.

**Participants:**

| Name | Kind | Domain |
|---|---|---|
| `robot` | local | `$(env ROS_DOMAIN_ID)` — the robot's own domain |
| `gcs` | local | `$(var gcs_domain)` — supplied at launch time |

**Allowed topics (abridged):**

| Topic / Service |
|---|
| `rt/<ROBOT_NAME>/odometry_conversion/odometry` |
| `rt/<ROBOT_NAME>/interface/mavros/global_position/raw/fix` |
| `rt/<ROBOT_NAME>/behavior/behavior_tree_commands` |
| `rt/<ROBOT_NAME>/behavior/behavior_tree_graphviz` |
| `rq+rr/<ROBOT_NAME>/interface/robot_command` |
| `rq+rr/<ROBOT_NAME>/trajectory_controller/set_trajectory_mode` |
| `rq+rr/<ROBOT_NAME>/takeoff_landing_planner/set_takeoff_landing_command` |
| `rq+rr/<ROBOT_NAME>/behavior/global_plan_toggle` |
| `rt/<ROBOT_NAME>/bag_record/bag_recording_status` |
| `rt/<ROBOT_NAME>/bag_record/set_recording_status` |
| `rt/<ROBOT_NAME>/fixed_trajectory_generator/fixed_trajectory_command` |


---

### `onboard_local_offboard_global/config/dds_router.yaml` — extended config

**Location:** [`robot/ros_ws/src/autonomy_bringup/onboard_local_offboard_global/config/dds_router.yaml`](../../../../robot/ros_ws/src/autonomy_bringup/onboard_local_offboard_global/config/dds_router.yaml)

Used for the `desktop_split`, `l4t_lite + offboard`, and `voxl + offboard` deployment profiles where local planning runs onboard and global planning runs on the GCS.

This config **inherits from `onboard_all`** via `extends:` and appends additional topics:

```yaml
extends: "$(find-pkg-share autonomy_bringup)/onboard_all/config/dds_router.yaml"
```

**Additional topics (appended to the base allowlist):**

| Topic |
|---|
| `rt/<ROBOT_NAME>/sensors/front_stereo/left/image_rect` |
| `rt/<ROBOT_NAME>/sensors/front_stereo/left/camera_info` |
| `rt/<ROBOT_NAME>/sensors/front_stereo/right/image_rect` |
| `rt/<ROBOT_NAME>/sensors/front_stereo/right/camera_info` |
| `rt/<ROBOT_NAME>/global_plan` |


The stereo image topics let the global planner on the GCS observe the robot's environment. `global_plan` carries the resulting path back to the onboard local planner.

---

## Adding a new bridged topic

1. Decide which config applies (`onboard_all` for all roles, `onboard_local_offboard_global` for split-only).
2. Add the topic to the appropriate `allowlist`, using the correct DDS prefix (`rt/`, `rq/`, `rr/`, etc.) and the `$(env ROBOT_NAME)` substitution for the robot namespace.
3. If overriding inherited list entries is needed, use the `!override` tag on the list.
