# Microsoft AirSim (legacy)

[Microsoft AirSim (legacy)](https://microsoft.github.io/AirSim/) is an open-source simulator for drones built on Unreal Engine, with built-in PX4 SITL integration.

<video controls muted loop playsinline preload="metadata" style="max-width: 100%;">
  <source src="../../assets/media/ms_airsim_demo.mp4" type="video/mp4">
</video>
*The AirStack autonomy stack flying `TakeoffTask` + `FixedTrajectoryTask` patterns in the AirSimNH neighborhood scene (`airstack up --sim airsim --robots 2 --scene neighborhood`).*

## Overview

Microsoft AirSim (legacy) provides an alternative simulation backend for AirStack, offering:

- **PX4 SITL integration** - Native MAVLink lockstep with PX4 autopilot
- **Unreal Engine environments** - Photorealistic scenes (UE 4.27)
- **Depth camera simulation** - Used by DROAN for obstacle avoidance
- **Lightweight setup** - Pre-built binary environments, no Omniverse required

**Trade-offs vs Isaac Sim:**

- Simpler setup (no NVIDIA Omniverse)
- PX4 integration via standard MAVLink (same as real hardware)
- Archived project (no new features, but stable)
- UE 4.27 only (older engine)

## Project status

Microsoft archived AirSim, which is why AirStack labels it "legacy": it remains a stable, supported simulation backend here, but the upstream project receives no new features. For a maintained successor, see [Project AirSim](https://github.com/iamaisim/ProjectAirSim) (UE5, new API).

## Quick Start

### 1. Scene (auto-fetched on first launch)

If `MS_AIRSIM_BINARY_PATH` is unset, the container's entrypoint auto-downloads the selected scene (Blocks by default, ~200 MB) into `simulation/ms-airsim/assets/scenes/` inside the `airsim` tmux window on first launch. Progress and any errors are visible there.

The easiest way to pick a scene is the launch flag — it maps a shortname to the right UE4 binary and, when the scene isn't downloaded yet, asks before fetching it (see [Simulation Scenes](../scenes.md) for the full catalog):

```bash
airstack up --sim airsim --scene neighborhood
```

To pre-fetch (e.g. before CI) or pick a different scene, run the helper directly:

```bash
./simulation/ms-airsim/assets/scenes/fetch_scene.sh              # blocks (default)
./simulation/ms-airsim/assets/scenes/fetch_scene.sh airsimnh     # or: abandonedpark,
                                                                 # landscapemountains, zhangjiajie,
                                                                 # africasavannah, msbuild2018
```

To use a scene that isn't one of the presets, extract it yourself into `simulation/ms-airsim/assets/scenes/` and set `MS_AIRSIM_BINARY_PATH` to its `.sh` path inside the container.

Scenes are pulled from the [AirSim Linux releases](https://github.com/microsoft/AirSim/releases/tag/v1.8.1).

### 2. Launch Microsoft AirSim (legacy) + Robot

```bash
airstack up --sim airsim
```

(Equivalently: `airstack up --env-file overrides/ms-airsim.env`, which sets the same compose profiles and URDF.)

To build or pull the images first, see [Docker reference → Image Management](docker.md#image-management).

To attach to the container's tmux session (window layout and startup sequence are detailed in the [Docker reference](docker.md#accessing-the-container)):

```bash
airstack connect ms-airsim
```

## Architecture

```
┌────────────────────────────────┐     ┌──────────────────────┐
│     MS-AirSim Container        │     │   Robot Container    │
│       (172.31.0.200)           │     │                      │
│                                │     │                      │
│   AirSim Binary (UE4)          │     │  MAVROS              │
│      ▲    │                    │     │    ▲                 │
│      │    │ TCP 4560+i         │     │    │                 │
│      │    ▼ (lockstep)         │     │    │                 │
│   PX4 SITL ×N ─── MAVLink UDP ─┼─────┼─► 24540+i/24580+i    │
│      ▲                         │     │    │                 │
│      │                         │     │    ▼                 │
│   Bridge Node ×N ─── ROS 2 DDS─┼─────┼─► Perception → DROAN │
│   (stereo RGB + depth + info)  │     │                      │
└────────────────────────────────┘     └──────────────────────┘
            airstack_network (172.31.0.0/24)
```

**Data flow:**

1. AirSim simulates physics and renders stereo RGB + depth for each vehicle
2. N PX4 SITL instances run in lockstep with AirSim via TCP (port `4560+i`)
3. MAVROS in the robot container connects to PX4 SITL via MAVLink UDP (offboard `14540 + ROS_DOMAIN_ID`, see `interface.launch.py`; ports `24540+i`/`24580+i` in `settings.json` are AirSim's own PX4 control channel)
4. Bridge nodes (one per robot) publish stereo RGB + depth + camera_info to ROS 2 topics
5. `disparity_expansion` converts depth to disparity for DROAN

## Configuration

### settings.json

Located at `simulation/ms-airsim/config/settings.json`, mounted into the container at `~/Documents/AirSim/settings.json`.

Key settings:

| Setting | Value | Purpose |
|---------|-------|---------|
| `SimMode` | `Multirotor` | Drone simulation |
| `ClockType` | `SteppableClock` | Lockstep with PX4 |
| `VehicleType` | `PX4Multirotor` | PX4 SITL vehicle |
| `TcpPort` | `4560 + i` | PX4 lockstep connection (per robot `i`) |
| `ControlPortLocal` | `24540 + i` | AirSim MAVLink proxy local port (deliberately offset from `14540+i` so the proxy doesn't intercept PX4 ↔ MAVROS traffic) |
| `ControlPortRemote` | `24580 + i` | AirSim MAVLink proxy remote port |

`settings.json` is generated at container start from [`settings.json.j2`](https://github.com/castacks/AirStack/blob/main/simulation/ms-airsim/config/settings.json.j2) via [`generate_settings.py`](https://github.com/castacks/AirStack/blob/main/simulation/ms-airsim/config/generate_settings.py), which expands per-robot port offsets, spawn positions, and camera parameters.

### Cameras

The default configuration is a forward-facing **stereo pair** (left + right) plus aligned depth:

| Property | Default | `.env` override |
|----------|---------|-----------------|
| Resolution | 480×300 | `AIRSIM_CAM_WIDTH`, `AIRSIM_CAM_HEIGHT` |
| FOV | 90° | `AIRSIM_CAM_FOV` |
| Baseline (2 × Y offset) | 0.12 m | `AIRSIM_CAM_Y` |
| Forward (X) offset | 0.4 m | `AIRSIM_CAM_X` |
| Vertical (Z) offset | 0 m | `AIRSIM_CAM_Z` |
| Pitch | 0° | `AIRSIM_CAM_PITCH` |

Cameras are defined per vehicle in the generated `settings.json` under `Vehicles.robot_<i>.Cameras`. Override the `.env` variables and restart the container to regenerate `settings.json` (see [Docker reference → Settings Generation](docker.md#settings-generation)).

### Bridge node parameters

Declared (with these defaults) in `simulation/ms-airsim/ros_ws/src/ms_airsim_ros_bridge/ms_airsim_ros_bridge/bridge_node.py`; the entrypoint starts each bridge with `ros2 run ... --ros-args -p robot_name:=robot_<i>`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `ms_airsim_ip` | `127.0.0.1` | AirSim API address (same container, so localhost) |
| `publish_rate` | `15.0` | Camera / depth publish rate (Hz) |
| `robot_name` | `robot_1` | ROS 2 topic namespace (set per-instance by the entrypoint to `robot_<i>`) |
| `clock_rate` | `50.0` | `/clock` publish rate (Hz) |

### Environment variables

Container-level environment variables (`AUTOLAUNCH`, `NUM_ROBOTS`, `SIM_IP`, `MS_AIRSIM_*`) are documented in the [Docker reference → Environment Variables](docker.md#environment-variables).

## Published ROS 2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/{robot_name}/sensors/front_stereo/left/image_rect` | `sensor_msgs/Image` | Left RGB image |
| `/{robot_name}/sensors/front_stereo/left/camera_info` | `sensor_msgs/CameraInfo` | Left camera intrinsics |
| `/{robot_name}/sensors/front_stereo/right/image_rect` | `sensor_msgs/Image` | Right RGB image |
| `/{robot_name}/sensors/front_stereo/right/camera_info` | `sensor_msgs/CameraInfo` | Right camera intrinsics |
| `/{robot_name}/sensors/front_stereo/depth` | `sensor_msgs/Image` | Depth image (32FC1, meters) |
| `/clock` | `rosgraph_msgs/Clock` | Simulation clock from AirSim |

## Troubleshooting

**Bridge can't connect to Microsoft AirSim (legacy):**

- Ensure the AirSim binary is running (`airsim` tmux window) and `settings.json` is loaded
- The entrypoint retries until the AirSim API is ready; check for connection errors in the container logs
- Check that the bridge node's `ms_airsim_ip` parameter matches where AirSim is running (default: `127.0.0.1` — same container)

**No depth images:**

- Verify the camera names in the generated `settings.json` are `front_left` / `front_right` — the names the bridge node requests images by
- Check AirSim console for rendering errors
- Echo the topic: `ros2 topic echo /robot_1/sensors/front_stereo/depth --once`

**MAVROS won't connect:**

- Verify `SIM_IP=172.31.0.200` is set in `.env` (default)
- Ensure PX4 SITL has started (look for `[mavlink]` output in the `robot_<i>_px4` tmux window)
- Check port configuration: offboard `14540 + ROS_DOMAIN_ID` (see Data flow above); AirSim's own control channel uses `24540+i`/`24580+i`

For container-level issues (UE4 binary won't launch, GPU/Vulkan access, PX4 lockstep connection, DDS topic visibility across containers), see the [Docker reference → Troubleshooting](docker.md#troubleshooting).

## See Also

- [Docker Configuration](docker.md) — container reference: services, env vars, networking, tmux layout, startup sequence
- [Simulation Scenes](../scenes.md) — scene catalog and fetch helper
- [Simulation Overview](../index.md) — choosing between simulators
