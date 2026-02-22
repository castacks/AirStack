# Robot Identity

Each robot container needs a unique **robot name** and a unique **ROS domain ID** so that multiple robots can operate on the same network without their ROS 2 traffic colliding.

AirStack handles this automatically. The `ROBOT_NAME_SOURCE` environment variable (set in `docker-compose.yaml` on a per-service basis) tells the container's `.bashrc` *where* to look up the robot's number, and from that number it derives both `ROBOT_NAME` and `ROS_DOMAIN_ID`.

## How It Works

```
docker-compose.yaml
  └─ ROBOT_NAME_SOURCE=<strategy>
        │
        ▼
  .bashrc (runs at shell start)
        │
        ├─ "container_name"  →  parse name from Docker's container hostname
        ├─ "hostname"        →  parse number from OS hostname
        └─ (anything else)   →  fall back to "unknown_robot" / domain 0
              │
              ▼
        ROBOT_NAME=robot_<N>        (e.g. robot_1)
        ROS_DOMAIN_ID=<N>           (e.g. 1)
```

The number `<N>` is the index of the robot (1-based). All ROS 2 topic namespaces are prefixed with `/$ROBOT_NAME`, so robot 1 publishes on `/robot_1/odometry` and robot 2 publishes on `/robot_2/odometry`.

## `ROBOT_NAME_SOURCE` Values

### `container_name` — simulation / desktop

Used by the **`desktop`** and **`simple`** profiles.

In simulation, Docker Compose names containers after the service, appending a replica number (e.g. `airstack-robot-1`, `airstack-robot-2`). The `.bashrc` resolves the container's hostname back to a name and extracts the trailing `robot-<N>` segment:

```bash
container_name=$(host $(host $(hostname) | awk '{print $NF}') | awk '{print $NF}' | awk -F . '{print $1}')
export ROBOT_NAME=$(echo "$container_name" | sed 's/.*\(robot-[0-9]*\)$/\1/' | sed 's#-#_#')
export ROS_DOMAIN_ID=$(echo "$ROBOT_NAME" | awk -F'_' '{print $NF}')
```

This technique requires **Docker Engine ≥ 29**.

Because simulation robots get their identity from the container name (which is controlled entirely by the host via `docker compose`), you can spin up multiple replicas with a single command:

```bash
NUM_ROBOTS=3 docker compose --profile desktop up
# → containers: airstack-robot-1, airstack-robot-2, airstack-robot-3
# → ROBOT_NAME:   robot_1,         robot_2,          robot_3
# → ROS_DOMAIN_ID: 1,              2,                3
```

In addition to `ROBOT_NAME` and `ROS_DOMAIN_ID`, the desktop profile also computes MAVLink UDP ports and the MAVRos FCU URL from the robot index so each simulated drone connects to the correct PX4 instance:

```bash
export OFFBOARD_PORT=$((OFFBOARD_BASE_PORT + ROS_DOMAIN_ID))
export ONBOARD_PORT=$((ONBOARD_BASE_PORT + ROS_DOMAIN_ID))
export FCU_URL="udp://:$OFFBOARD_PORT@172.31.0.200:$ONBOARD_PORT"
export TGT_SYSTEM=$((1 + ROS_DOMAIN_ID))
```

### `hostname` — real hardware (VOXL, Jetson)

Used by the **`voxl`** and **`l4t`** profiles.

On physical robots the container runs with `network_mode: host`, so the container's hostname is the same as the device's OS hostname. By convention, devices are named `robot-<N>` (e.g. `robot-1`, `robot-03`). The `.bashrc` extracts the number with:

```bash
num=$(hostname | awk -F'-' '{print $2}')
num=$((num))   # strip leading zeros
export ROBOT_NAME="robot_$num"
export ROS_DOMAIN_ID=$num
```

The serial port for MAVLink (`/dev/ttyTHS4`) is hardcoded for real-robot profiles since there is only one flight controller per device:

```bash
export FCU_URL="/dev/ttyTHS4:115200"
```

!!! warning "Hostname convention is required"
    If the device hostname does not follow the `robot-<N>` pattern, `num` will be `0` and `ROBOT_NAME` / `ROS_DOMAIN_ID` will both be set to `ERROR`. Make sure every physical robot is given a compliant hostname before deployment.

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
