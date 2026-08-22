# GCS Docker Configuration

The Ground Control Station is containerized using Docker Compose, following the same patterns as the robot and simulation components.

## File Structure

```
gcs/docker/
├── gcs-base-docker-compose.yaml  # Shared base service: command, env, GPU, mounts
├── docker-compose.yaml           # gcs (sim/dev) + gcs-real (field) services
├── Dockerfile.gcs                # Image: ROS 2 Jazzy desktop + Foxglove Studio
│                                 #   + foxglove_bridge + eProsima DDS Router
│                                 #   + mosquitto/gstreamer (TAK tooling deps)
├── .bashrc                       # Bash config mounted into the container
└── Foxglove/                     # Foxglove Studio app state (mounted to
                                  #   /root/.config/Foxglove, gitignored)
```

The Foxglove **extensions and layout template** live one level up in `gcs/foxglove_extensions/` (mounted into the container) — not under `docker/`.

## Services and profiles

Both services extend the same `gcs-base` definition:

| Service | Profiles | Networking | Use case |
|---------|----------|------------|----------|
| `gcs` | `desktop`, `desktop_split` | `airstack_network` bridge; publishes `2222:22` (ssh) and `8766:8765` (Foxglove bridge) | Sim/dev alongside robot-desktop containers |
| `gcs-real` | `hitl`, `deploy`, `offboard` | `network_mode: host` (DDS directly on the LAN) | Field laptop / ground station next to real or HITL robots |

`gcs-real` additionally mounts `$HOME/bags` at `/bags` for mission recordings on the host.

## Container startup

The `gcs-base` `command:` runs, in order:

```yaml
command: >
  bash -c "
  service ssh restart;
  python3 /root/AirStack/gcs/foxglove_extensions/install.py;
  python3 /root/AirStack/gcs/foxglove_extensions/render_layout.py;
  tmux new -d -s bringup;
  if [ $$AUTOLAUNCH = 'true' ]; then
    tmux send-keys -t bringup:0.0 'bws &&  sws; ros2 launch desktop_bringup gcs.launch.xml' ENTER;
  fi;
  sleep infinity"
```

1. **`install.py`** walks `gcs/foxglove_extensions/` and installs every extension directory (a `package.json` + `dist/extension.js`) into Foxglove's user-extensions dir (`/root/.foxglove-studio/extensions`) — that's how the Robot Tasks / Waypoint Editor / Polygon Editor panels appear.
2. **`render_layout.py`** expands the single-robot template `airstack_default.json` into `/root/airstack_layout_num_robots_<N>.json` for the current `NUM_ROBOTS` (see [GCS Foxglove Visualization](../foxglove.md) for the import flow).
3. A tmux session named **`bringup`** is created; with `AUTOLAUNCH=true` it builds the workspace and launches `desktop_bringup gcs.launch.xml` (Foxglove Studio GUI, `foxglove_bridge` on port 8765, gossip bridge, `gcs_visualizer`, `action_relay`).

Attach to the session with `airstack connect gcs` (or `tmux attach -t bringup` inside the container).

## Environment Variables

| Variable | Description | Default |
|----------|-------------|---------|
| `AUTOLAUNCH` | Auto-start the GCS bringup in tmux | `false` in the compose file (the `airstack` CLI sets it `true` for full-stack launches) |
| `NUM_ROBOTS` | Legacy robot count: sizes the rendered layout and the per-robot action relays | `1` |
| `FLEET_CONFIG_FILE` | Fleet-first roster (RFC #380): when set, `action_relay` derives robot names/domains from the fleet file instead of `NUM_ROBOTS` | unset |
| `ROBOT_RELAY_MAP` | `name:domain,...` override for custom robot→domain mappings (wins over both of the above) | unset |
| `RECORD_BAGS` | Enable bag recording | unset |
| `DISPLAY` | Host X11 display for the Foxglove Studio window | inherited |

**Example overrides:**

```bash
# Don't auto-launch (for development)
AUTOLAUNCH=false airstack up gcs

# Three robots: three relays + a 3-tab layout
NUM_ROBOTS=3 airstack up --sim isaac --robots 3
```

## Networking

In the sim/dev profiles the GCS joins the `airstack_network` bridge (172.31.0.0/24) and talks ROS 2 DDS to the robot containers and the sim. The GCS itself runs on `ROS_DOMAIN_ID=0`; `action_relay` and the gossip bridge are the components that cross into the per-robot domains (robot N = domain N) and the gossip domain (99).

## Accessing the GCS

### Foxglove (primary)

- The Foxglove Studio **desktop app** opens on the host's X display when the bringup launches.
- From the **host**, open your own Foxglove and connect to `ws://localhost:8766` (the published bridge port).
- Import the rendered layout `/root/airstack_layout_num_robots_<N>.json` — full walkthrough in [GCS Foxglove Visualization](../foxglove.md).

### tmux / shell

```bash
airstack connect gcs          # tmux attach to the bringup session
docker exec -it airstack-gcs-1 bash
```

### SSH

```bash
ssh -p 2222 root@localhost    # password: airstack
```

## Development Workflow

### Interactive Development

```bash
# Start GCS without auto-launch
AUTOLAUNCH=false airstack up gcs

# Inside the container
airstack connect gcs
bws --packages-select gcs_visualizer     # build one package
sws
ros2 launch desktop_bringup gcs.launch.xml
```

### Building GCS Packages

```bash
# Build from host
docker exec airstack-gcs-1 bash -c "bws --packages-select action_relay"

# Build with debug symbols
docker exec airstack-gcs-1 bash -c "bws --packages-select action_relay --cmake-args '-DCMAKE_BUILD_TYPE=Debug'"
```

### Viewing Logs

```bash
airstack logs gcs             # tmux output is mirrored to docker logs
docker logs -f airstack-gcs-1
```

## Troubleshooting

**Foxglove window won't display:**

- Check `DISPLAY` on the host: `echo $DISPLAY`
- Allow X11 connections: `xhost +local:docker`
- Verify the X11 socket mount in `gcs-base-docker-compose.yaml`

**Host Foxglove won't connect:**

- Use port **8766** from the host (8765 is the in-container port)
- Verify nothing else holds the port: `sudo lsof -i :8766`
- Check the bridge node is up: `airstack logs gcs`

**Custom panels show "Unknown panel type":**

- `install.py` runs on container start — restart the container after editing an extension, or re-run `python3 /root/AirStack/gcs/foxglove_extensions/install.py` and restart Foxglove

**ROS 2 communication issues:**

- The GCS lives on domain 0; robot topics reach it via the DDS router / relays, not raw discovery
- Check all containers are on `airstack_network`
- `docker exec airstack-gcs-1 bash -c "ros2 topic list"`

**SSH connection refused:**

- Wait a few seconds after container launch for `sshd`
- Check the port mapping: `docker port airstack-gcs-1`

## See Also

- [GCS Overview](../index.md) - Main GCS documentation
- [GCS Foxglove Visualization](../foxglove.md) - Layouts, visualizer, extending markers
- [Docker Workflow](../../development/beginner/airstack-cli/docker_usage.md) - General Docker operations
- [Robot Docker Configuration](../../robot/docker/index.md) - Robot container setup
