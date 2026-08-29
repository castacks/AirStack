# Robot Configuration

Configure robot identity, stack selection, and module parameters for AirStack deployment.

## Where Configuration Actually Lives

There is no single configuration file — settings live at the level they affect:

| Level | Location | What it controls |
| ----- | -------- | ---------------- |
| Compose / containers | top-level `.env` | Image tags, `NUM_ROBOTS`, `AUTOLAUNCH`, sim selection, bag recording |
| Stack (launch topology) | `stacks/<name>/launch/*.launch.xml` | Which modules run and how their topics are wired (launch args, remaps) |
| Module parameters | each package's `config/*.yaml` (`robot/ros_ws/src/<layer>/.../<package>/config/`) | Algorithm-specific ROS 2 parameters |
| Fleet era (RFC #380) | `config/vehicles/` and `config/fleets/` | Vehicle definitions; who exists, which vehicle, which stack, which ground hosts |
| Robot identity | `robot/docker/robot_name_map/` | Container/hostname → `ROBOT_NAME` + `ROS_DOMAIN_ID` mapping |

## Stack Selection

The autonomy topology is selected by a **stack** — stacks are the only dispatch
(the legacy `AUTONOMY_ROLE` role dispatch was removed). `airstack up --stack
<name>[:<entry>]` exports `AIRSTACK_STACK_DIR` (the container path of the stack
folder, `/root/AirStack/stacks/<name>`) and `AIRSTACK_STACK_ENTRY` (the entry
launch file name, default `stack`). Unset, the trunk reference stack
`full_default` is used.

See [Stacks](../../development/stacks.md) for the reference stacks and how to
create your own.

## Environment Variables

Key variables in the top-level `.env` file (compose-level configuration):

```bash
# Launch Configuration
AUTOLAUNCH="true"           # false = spawn idle containers with no launch command

# Multi-robot
NUM_ROBOTS="1"              # Number of robot containers (compose replicas)

# Robot identity mapping (name → ROBOT_NAME + ROS_DOMAIN_ID)
ROBOT_NAME_MAP_CONFIG_FILE="default_robot_name_map.yaml"

# Logging
RECORD_BAGS="false"         # Start the bag recorder node (see Logging docs)
```

Stack selection (`AIRSTACK_STACK_DIR`, `AIRSTACK_STACK_ENTRY`) and fleet
selection (`FLEET_CONFIG_FILE`) are exported by `airstack up --stack` /
`--fleet` rather than set by hand in `.env`.

`ROBOT_NAME` and `ROS_DOMAIN_ID` are **not** set in `.env` — each container
resolves them at startup from `ROBOT_NAME_SOURCE` and the mapping config; see
[Robot Identity](../docker/robot_identity.md).

The full table of variables forwarded into the robot containers is in the
[Docker guide](../docker/index.md#environment-variables).

## ROS 2 Parameters

Module-specific parameters live in YAML files in each module package's own
`config/` directory (`robot/ros_ws/src/<layer>/.../<package>/config/`); stacks
override them via launch arguments in their entry files
(`stacks/<name>/launch/*.launch.xml`).

**Example** (`robot/ros_ws/src/sensors/lidar_point_cloud_filter/config/lidar_point_cloud_filter.yaml`):
```yaml
/**:
  ros__parameters:
    near_range_m: 0.75
    input_topic: "/$(env ROBOT_NAME)/sensors/ouster/point_cloud_raw"
    output_topic: "/$(env ROBOT_NAME)/sensors/ouster/point_cloud"
    qos_reliable: true
```

## Robot Identity

Each robot requires unique configuration for multi-robot scenarios.

See: [Robot Identity Guide](../docker/robot_identity.md)

**Key Settings**:

- **ROBOT_NAME**: Namespace for all topics (`/robot_1/...`)
- **ROS_DOMAIN_ID**: Isolate ROS 2 communication (0-101)

Both are resolved at container startup by
`robot/docker/robot_name_map/resolve_robot_name.py` from the mapping file
selected by `ROBOT_NAME_MAP_CONFIG_FILE`.

## See Also

- [Robot Identity](../docker/robot_identity.md) - Configuring robot identification
- [Stacks](../../development/stacks.md) - Stack folders and entry launch files
- [Fleets](../../development/fleets.md) - Fleet files, vehicles, and placement
- [Autonomy Modes](../autonomy_modes.md) - Different operation modes
- [Integration Checklist](../autonomy/integration_checklist.md) - Module configuration requirements
