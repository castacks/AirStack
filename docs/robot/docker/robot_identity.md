# Robot Identity

Each robot container needs a unique **robot name** and a unique **ROS domain ID** so that multiple robots can operate on the same network without their ROS 2 traffic colliding.

AirStack handles this automatically. The `ROBOT_NAME_SOURCE` environment variable (set in `docker-compose.yaml` on a per-service basis) tells the container's `.bashrc` *where* to obtain a name to look up, and a YAML mapping config file then resolves it to both `ROBOT_NAME` and `ROS_DOMAIN_ID`.

## How It Works

```
.env
  └─ ROBOT_NAME_MAP_CONFIG_FILE=<yaml config>   (e.g. default_robot_name_map.yaml)

docker-compose.yaml
  └─ ROBOT_NAME_SOURCE=<strategy>
        │
        ▼
  .bashrc (runs at shell start)
        │
        ├─ "container_name"  →  resolve Docker container hostname to a name
        ├─ "hostname"                 →  use the OS hostname directly
        └─ (anything else)            →  fall back to "unknown_robot" / domain 0
              │
              ▼
        resolve_robot_name.py <name> <config_file>
              │
              ▼
        ROBOT_NAME=robot_<N>        (e.g. robot_1)
        ROS_DOMAIN_ID=<N>           (e.g. 1)
```

In both active strategies, the resolved name is passed to `resolve_robot_name.py` along with the config file specified by `ROBOT_NAME_MAP_CONFIG_FILE`. The script matches the name against regex rules in the YAML file and outputs `ROBOT_NAME` and `ROS_DOMAIN_ID`, which the `.bashrc` captures via `eval`:

```bash
eval "$(resolve_robot_name.py "$name_to_map" "$ROBOT_NAME_MAP_CONFIG_FILE")"
export ROBOT_NAME
export ROS_DOMAIN_ID
```

All ROS 2 topic namespaces are prefixed with `/$ROBOT_NAME`, so robot 1 publishes on `/robot_1/odometry` and robot 2 publishes on `/robot_2/odometry`.

## Mapping Config File

The mapping config is a YAML file under `robot/docker/robot_name_map/`. The file to use is set by `ROBOT_NAME_MAP_CONFIG_FILE` in the project's `.env` file (default: `default_robot_name_map.yaml`).

Each rule has a `pattern` (Python `re.fullmatch` regex), a `robot` template, and a `domain_id` template. Capture groups are referenced as `{1}`, `{2}`, etc. Rules are evaluated top-down; the first match wins.

```yaml
mappings:
  - pattern: '.*robot-.*(\ d+)'
    robot: 'robot_{1}'
    domain_id: '{1}'

  - pattern: '.*'
    robot: 'unknown-robot'
    domain_id: '0'
```

To customize the mapping for your deployment, create a new YAML file in `robot/docker/robot_name_map/` and set `ROBOT_NAME_MAP_CONFIG_FILE` in your `.env` to point to it.

## `ROBOT_NAME_SOURCE` Values

### `container_name` — simulation / desktop

Used by the **`desktop`** and **`simple`** profiles.

In simulation, Docker Compose names containers after the service, appending a replica number (e.g. `airstack-robot-1`, `airstack-robot-2`). The `.bashrc` resolves the container's hostname back to its Docker name:

```bash
name_to_map=$(host $(host $(hostname) | awk '{print $NF}') | awk '{print $NF}' | awk -F . '{print $1}')
```

This technique requires **Docker Engine ≥ 29**. The resolved container name is then passed to the mapping script.

Because simulation robots get their identity from the container name (which is controlled entirely by the host via `docker compose`), you can spin up multiple replicas with a single command:

```bash
NUM_ROBOTS=3 docker compose --profile desktop up
# → containers: airstack-robot-1, airstack-robot-2, airstack-robot-3
# → ROBOT_NAME:   robot_1,         robot_2,          robot_3
# → ROS_DOMAIN_ID: 1,              2,                3
```

In addition to `ROBOT_NAME` and `ROS_DOMAIN_ID`, the desktop profile also computes MAVLink UDP ports and the MAVRos FCU URL so each simulated drone connects to the correct PX4 instance:

```bash
export OFFBOARD_PORT=$((OFFBOARD_BASE_PORT + ROS_DOMAIN_ID))
export ONBOARD_PORT=$((ONBOARD_BASE_PORT + ROS_DOMAIN_ID))
export FCU_URL="udp://:$OFFBOARD_PORT@172.31.0.200:$ONBOARD_PORT"
export TGT_SYSTEM=$((1 + ROS_DOMAIN_ID))
```

### `hostname` — real hardware (VOXL, Jetson)

Used by the **`voxl`** and **`l4t`** profiles.

On physical robots the container runs with `network_mode: host`, so the container's hostname is the same as the device's OS hostname. The hostname is passed directly to the mapping script:

```bash
name_to_map=$(hostname)
```

By convention, devices are named `robot-<N>` (e.g. `robot-1`), which the default mapping config resolves to `ROBOT_NAME=robot_<N>` and `ROS_DOMAIN_ID=<N>`. If your hostnames follow a different scheme, customize the mapping config file accordingly.

The serial port for MAVLink (`/dev/ttyTHS4`) is hardcoded for real-robot profiles since there is only one flight controller per device:

```bash
export FCU_URL="/dev/ttyTHS4:115200"
```

!!! warning "Hostname convention is required"
    The hostname must match a rule in the mapping config file. If no rule matches, the script exits with an error and `ROBOT_NAME` / `ROS_DOMAIN_ID` will be unset. Make sure every physical robot has a hostname that matches a rule before deployment.

### Fallback (anything else)

If `ROBOT_NAME_SOURCE` is unset or set to an unrecognized value, the `.bashrc` falls back to:

```
ROBOT_NAME=unknown_robot
ROS_DOMAIN_ID=0
```

A warning is printed to the shell. This is safe for single-robot testing but **will cause collisions** if two containers with this fallback are on the same network.

## Summary Table

| Profile | `ROBOT_NAME_SOURCE` | Identity source | Multi-robot safe? |
|---|---|---|---|
| `desktop` | `container_name` | Docker container name (host-assigned) | Yes — scale with `NUM_ROBOTS` |
| `simple` | `container_name` | Docker container name | Yes |
| `voxl` | `hostname` | OS hostname of the device | Yes — set distinct hostnames |
| `l4t` | `hostname` | OS hostname of the device | Yes — set distinct hostnames |
| _(fallback)_ | _(other)_ | Hardcoded defaults | **No** |

## Overriding Manually

If you need to override the robot identity for testing, you can set the variables directly before starting a shell session inside the container:

```bash
docker exec -e ROBOT_NAME=robot_5 -e ROS_DOMAIN_ID=5 -it <container> bash
```

Or add them to your project's `.env` file and pass them through in `docker-compose.yaml`.
