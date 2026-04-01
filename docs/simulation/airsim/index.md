# AirSim

AirSim (Microsoft) is an open-source simulator for drones built on Unreal Engine, with built-in PX4 SITL integration.

## Overview

AirSim provides an alternative simulation backend for AirStack, offering:

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

### 1. Download an AirSim environment

Download a pre-built environment from the [AirSim releases](https://github.com/microsoft/AirSim/releases) and extract it into `simulation/airsim/environments/`:

```bash
cd simulation/airsim
mkdir -p environments && cd environments
wget https://github.com/microsoft/AirSim/releases/download/v1.8.1-linux/Blocks.zip
unzip Blocks.zip
```

### 2. Set environment variables

Edit `.env` at the repository root:

```bash
SIM_IP="172.31.0.201"
AIRSIM_ENV_DIR="simulation/airsim/environments"
AIRSIM_BINARY_PATH="/airsim_env/LinuxNoEditor/Blocks.sh"
```

### 3. Launch AirSim + Robot

```bash
# Option A: pass profiles as flags
airstack up --profile airsim --profile desktop

# Option B: set profiles in .env (COMPOSE_PROFILES="airsim,desktop") then just:
airstack up
```

To build the images first:

```bash
airstack image-build --profile airsim
```

The container runs two tmux windows:
- **Window 0**: AirSim binary (Unreal Engine rendering)
- **Window 1**: ROS 2 bridge node (depth + camera_info publisher)

To attach to the tmux session:

```bash
airstack connect airsim
```

## Architecture

```
┌──────────────────────┐     ┌──────────────────────┐
│   AirSim Container   │     │   Robot Container     │
│   (172.31.0.201)     │     │                       │
│                      │     │                       │
│  AirSim Binary       │     │  MAVROS ◄──── MAVLink UDP ────► PX4 SITL
│    │                 │     │    │                  │
│    ▼                 │     │    ▼                  │
│  PX4 SITL            │     │  Interface Layer      │
│    (TCP lockstep)    │     │    │                  │
│                      │     │    ▼                  │
│  Bridge Node ────────ROS 2 DDS────►  Perception    │
│    (depth +          │     │    │                  │
│     camera_info)     │     │    ▼                  │
│                      │     │  DROAN (disparity     │
│                      │     │   expansion + planner)│
└──────────────────────┘     └──────────────────────┘
         airstack_network (172.31.0.0/24)
```

**Data flow:**

1. AirSim simulates physics and renders depth images
2. PX4 SITL runs in lockstep with AirSim via TCP (port 4560)
3. Robot container connects to PX4 via MAVROS (UDP 14540/14580)
4. Bridge node publishes depth + camera_info to ROS 2 topics
5. `disparity_expansion` converts depth to disparity for DROAN

## Configuration

### settings.json

Located at `simulation/airsim/config/settings.json`, mounted into the container at `~/Documents/AirSim/settings.json`.

Key settings:

| Setting | Value | Purpose |
|---------|-------|---------|
| `SimMode` | `Multirotor` | Drone simulation |
| `ClockType` | `SteppableClock` | Lockstep with PX4 |
| `VehicleType` | `PX4Multirotor` | PX4 SITL vehicle |
| `TcpPort` | `4560` | PX4 lockstep connection |
| `ControlPortLocal` | `14540` | MAVLink offboard port |
| `ControlPortRemote` | `14580` | MAVLink onboard port |

### Camera

The default configuration uses a single forward-facing depth camera:

- Resolution: 480x300
- FOV: 90 degrees
- Position: 20cm forward, 10cm up from body center

To modify camera settings, edit `settings.json` under `Vehicles.drone1.Cameras.front_camera`.

### Bridge node parameters

Located at `simulation/airsim/ros_ws/src/airsim_depth_bridge/config/bridge.yaml`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `airsim_ip` | `127.0.0.1` | AirSim API address |
| `camera_name` | `front_camera` | AirSim camera name |
| `vehicle_name` | `drone1` | AirSim vehicle name |
| `publish_rate` | `15.0` | Depth publish rate (Hz) |
| `robot_name` | `robot_1` | ROS 2 topic namespace |

### Environment variables

| Variable | Default | Description |
|----------|---------|-------------|
| `SIM_IP` | `172.31.0.200` | Simulation container IP. Set to `172.31.0.201` for AirSim |
| `AIRSIM_ENV_DIR` | `simulation/airsim/environments` | Host path to extracted AirSim environment |
| `AIRSIM_BINARY_PATH` | `/airsim_env/LinuxNoEditor/Blocks.sh` | Path to binary inside container |

## Published ROS 2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/{robot_name}/sensors/front_camera/depth` | `sensor_msgs/Image` | Depth image (32FC1, meters) |
| `/{robot_name}/sensors/front_camera/camera_info` | `sensor_msgs/CameraInfo` | Camera intrinsics (P matrix) |

## Adding Stereo Cameras

To switch from depth-only to stereo (for full disparity-based DROAN), add a second camera to `settings.json`:

```json
"Cameras": {
  "front_left": {
    "CaptureSettings": [{"ImageType": 0, "Width": 480, "Height": 300, "FOV_Degrees": 90}],
    "X": 0.2, "Y": -0.06, "Z": -0.1
  },
  "front_right": {
    "CaptureSettings": [{"ImageType": 0, "Width": 480, "Height": 300, "FOV_Degrees": 90}],
    "X": 0.2, "Y": 0.06, "Z": -0.1
  }
}
```

This creates a stereo pair with 12cm baseline (matching ZED cameras). The bridge node would need to be extended to publish rectified stereo images.

### Project status

Microsoft archived AirSim. For a maintained successor, see [Project AirSim](https://github.com/iamaisim/ProjectAirSim) (UE5, new API — integration planned as a future AirStack feature).

## Troubleshooting

**Bridge can't connect to AirSim:**

- Ensure the AirSim binary is running and `settings.json` is loaded
- Check that `airsim_ip` parameter matches where AirSim is running

**No depth images:**

- Verify the camera name in `settings.json` matches `bridge.yaml`
- Check AirSim console for rendering errors

**MAVROS won't connect:**

- Verify `SIM_IP=172.31.0.201` is set in `.env`
- Ensure PX4 SITL has started (check AirSim console for MAVLink messages)
- Check port configuration: offboard=14540, onboard=14580
