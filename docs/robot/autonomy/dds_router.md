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
| `$(var VAR_NAME)` | `dds_router_args` launch argument (space-separated `key:=value` pairs) | Yes — `RuntimeError` |
| `$(find-pkg-share PKG)` | `ament_index` share directory of `PKG` | Yes — `RuntimeError` |

**Example** — calling the launch file from XML:

```xml
<include file="$(find-pkg-share autonomy_bringup)/launch/interpolate_dds_router.launch.py">
  <arg name="dds_router_config_file"
       value="$(find-pkg-share autonomy_bringup)/config/dds_router.yaml" />
  <arg name="dds_router_args" value="gcs_domain:=0" />
</include>
```

!!! note "Renamed launch arguments (RFC #379 §4)"
    The arguments were renamed from the generic `config_file` / `args` to the
    prefixed `dds_router_config_file` / `dds_router_args` — generic launch
    configurations leak across sibling includes in the same launch scope. The
    old names still work as **deprecated aliases** (the prefixed name wins
    when both are set); update external callers when convenient.

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

### `autonomy_bringup/config/dds_router.yaml` — shared allowlist

**Location:** [`robot/ros_ws/src/autonomy_bringup/config/dds_router.yaml`](../../../../robot/ros_ws/src/autonomy_bringup/config/dds_router.yaml)

Selected by the `full_*` stacks and `lite_default` (their entry files pass it
to `interpolate_dds_router.launch.py`). Historically this lived at
`onboard_all/config/` under the removed AUTONOMY_ROLE dispatch.

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

### Split-stack router config — generated from `bridge.yaml`

The split stack (`stacks/lite_offload_global`) does NOT use a hand-written
router config: its [`bridge.yaml`](../../../../stacks/lite_offload_global/bridge.yaml)
is the authoritative boundary document, and `tools/gen_dds_router.py`
generates `.airstack/generated/dds_router.lite_offload_global.yaml` from it
(loaded by the stack's `onboard` entry).

The generated config replaced the legacy split's committed
`onboard_local_offboard_global/config/dds_router.yaml` (removed with the
AUTONOMY_ROLE dispatch) — deliberately minus the `set_trajectory_mode`
crossing that config carried: command authority stays onboard
(`airstack doctor` hard gate #2).

---

## Adding a new bridged topic

1. Decide where it belongs: the shared `autonomy_bringup/config/dds_router.yaml` allowlist (full/lite stacks), or the split stack's `bridge.yaml` (then regenerate with `tools/gen_dds_router.py`; the doctor hard gate rejects control-setpoint / trajectory-group names).
2. For the shared allowlist, add the topic using the correct DDS prefix (`rt/`, `rq/`, `rr/`, etc.) and the `$(env ROBOT_NAME)` substitution for the robot namespace.
3. If overriding inherited list entries is needed, use the `!override` tag on the list.
