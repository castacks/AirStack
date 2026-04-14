# Microsoft AirSim (legacy)

[Microsoft AirSim (legacy)](https://microsoft.github.io/AirSim/) is an open-source simulator for drones built on Unreal Engine, with built-in PX4 SITL integration.

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

## Quick Start

### 1. Download an environment

Download a pre-built environment from the [AirSim Linux releases](https://github.com/microsoft/AirSim/releases/tag/v1.8.1) and extract it into `simulation/ms-airsim/environments/`:

```bash
cd simulation/ms-airsim
mkdir -p environments && cd environments
wget https://github.com/microsoft/AirSim/releases/download/v1.8.1/AbandonedPark.zip
unzip AbandonedPark.zip
```

### 2. Launch Microsoft AirSim (legacy) + Robot

```bash
airstack up --env-file overrides/ms-airsim.env
```

To build the images first:

```bash
airstack image-build --profile ms-airsim
```

The container runs `1 + 2*NUM_ROBOTS` tmux windows:
- **Window 0**: AirSim binary (Unreal Engine rendering)
- **Windows 1..N**: one PX4 SITL instance per robot
- **Windows N+1..2N**: one ROS 2 bridge node per robot (depth + stereo RGB + camera_info)

To attach to the tmux session:

```bash
airstack connect ms-airsim
```

A video is below:

<iframe src="https://private-user-images.githubusercontent.com/4943471/574307425-90c3d9d0-f53a-46a6-a11d-f3690f682e51.mp4?jwt=eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzI1NiJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3NzU2NjY4NTIsIm5iZiI6MTc3NTY2NjU1MiwicGF0aCI6Ii80OTQzNDcxLzU3NDMwNzQyNS05MGMzZDlkMC1mNTNhLTQ2YTYtYTExZC1mMzY5MGY2ODJlNTEubXA0P1gtQW16LUFsZ29yaXRobT1BV1M0LUhNQUMtU0hBMjU2JlgtQW16LUNyZWRlbnRpYWw9QUtJQVZDT0RZTFNBNTNQUUs0WkElMkYyMDI2MDQwOCUyRnVzLWVhc3QtMSUyRnMzJTJGYXdzNF9yZXF1ZXN0JlgtQW16LURhdGU9MjAyNjA0MDhUMTY0MjMyWiZYLUFtei1FeHBpcmVzPTMwMCZYLUFtei1TaWduYXR1cmU9YjM4NjEyODA5Nzg0MzY2ODNmZDlkYzlmODNmMmJhNzkwODA2YWViYjU3Nzc2YjA4ZDhiZThjY2MzNWUyYWViYSZYLUFtei1TaWduZWRIZWFkZXJzPWhvc3QifQ.N11vdWatLn7F3mLDdctHIu3p9eZzO52M9oVnnjwW88M" allowfullscreen width="1920" height="500"></iframe>


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
3. MAVROS in the robot container connects to PX4 SITL via MAVLink UDP (offboard `24540+i`, onboard `24580+i`)
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
| `ControlPortLocal` | `24540 + i` | AirSim MAVLink proxy local port (moved off `14540+i` so it doesn't intercept PX4 ↔ MAVROS traffic) |
| `ControlPortRemote` | `24580 + i` | AirSim MAVLink proxy remote port |

`settings.json` is generated at container start from [`settings.json.j2`](https://github.com/.../simulation/ms-airsim/config/settings.json.j2) via [`generate_settings.py`](https://github.com/.../simulation/ms-airsim/config/generate_settings.py), which expands per-robot port offsets, spawn positions, and camera parameters.

### Cameras

The default configuration is a forward-facing **stereo pair** (left + right) plus aligned depth:

| Property | Default | `.env` override |
|----------|---------|-----------------|
| Resolution | 480×300 | `AIRSIM_CAM_WIDTH`, `AIRSIM_CAM_HEIGHT` |
| FOV | 110° | `AIRSIM_CAM_FOV` |
| Baseline (2 × Y offset) | 0.12 m | `AIRSIM_CAM_Y` |
| Forward (X) offset | 0.4 m | `AIRSIM_CAM_X` |
| Pitch | 0° | `AIRSIM_CAM_PITCH` |

Cameras are defined per vehicle in the generated `settings.json` under `Vehicles.drone<i>.Cameras`.

### Bridge node parameters

Located at `simulation/ms-airsim/ros_ws/src/ms_airsim_ros_bridge/config/bridge.yaml`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `ms_airsim_ip` | `127.0.0.1` | AirSim API address (same container, so localhost) |
| `publish_rate` | `15.0` | Camera / depth publish rate (Hz) |
| `robot_name` | `robot_1` | ROS 2 topic namespace (set per-instance by the entrypoint to `robot_<i>`) |
| `clock_rate` | `50.0` | `/clock` publish rate (Hz) |

### Environment variables

| Variable | Default | Description |
|----------|---------|-------------|
| `SIM_IP` | `172.31.0.200` | Simulation container IP |
| `MS_AIRSIM_ENV_DIR` | `simulation/ms-airsim/environments` | Host path to extracted AirSim environment |
| `MS_AIRSIM_BINARY_PATH` | `/ms-airsim-env/LinuxNoEditor/Blocks.sh` | Path to binary inside container |

## Published ROS 2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/{robot_name}/sensors/front_stereo/left/image_rect` | `sensor_msgs/Image` | Left RGB image |
| `/{robot_name}/sensors/front_stereo/left/camera_info` | `sensor_msgs/CameraInfo` | Left camera intrinsics |
| `/{robot_name}/sensors/front_stereo/right/image_rect` | `sensor_msgs/Image` | Right RGB image |
| `/{robot_name}/sensors/front_stereo/right/camera_info` | `sensor_msgs/CameraInfo` | Right camera intrinsics |
| `/{robot_name}/sensors/front_stereo/depth` | `sensor_msgs/Image` | Depth image (32FC1, meters) |
| `/clock` | `rosgraph_msgs/Clock` | Simulation clock from AirSim |

### Project status

Microsoft archived AirSim. For a maintained successor, see [Project AirSim](https://github.com/iamaisim/ProjectAirSim) (UE5, new API — integration planned as a future AirStack feature).

## Troubleshooting

**Bridge can't connect to Microsoft AirSim (legacy):**

- Ensure the AirSim binary is running and `settings.json` is loaded
- Check that `ms_airsim_ip` parameter matches where AirSim is running

**No depth images:**

- Verify the camera name in `settings.json` matches `bridge.yaml`
- Check AirSim console for rendering errors

**MAVROS won't connect:**

- Verify `SIM_IP=172.31.0.200` is set in `.env` (default)
- Ensure PX4 SITL has started (check AirSim console for MAVLink messages)
- Check port configuration: offboard=24540+i, onboard=24580+i
