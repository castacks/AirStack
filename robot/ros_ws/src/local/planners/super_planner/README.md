# SUPER Planner

Adapter that connects the [SUPER planner](https://github.com/ZJU-FAST-Lab/SUPER) to AirStack's trajectory controller. SUPER runs in its own Docker container (ROS Humble); this node bridges it to AirStack (ROS Jazzy) over a TCP JSON sidecar, avoiding any ROS distro or message-type compatibility issues.

## Architecture

```
AirStack (Jazzy)                         SUPER container (Humble)
─────────────────────────────────────    ────────────────────────
/{robot}/odometry_conversion/odometry ─┐
/{robot}/global_plan              ─────┤  super_adapter_node.py
                                        ├──── TCP JSON (port 8768) ───► super_ros_sidecar.py
trajectory_override ◄──────────────────┘                                      │
  → trajectory_controller                                               fsm_node (SUPER)
```

## Prerequisites

1. SUPER Docker image available: `super_planner:1` (or `ghcr.io/kay-000/safe-benchmark/super_planner:latest`)
2. The sidecar launch script from the benchmark repo: `workspace/benchmark/integrations/aerial_nav/agent_hooks/super.sh`

## Usage

**Step 1 — Start the SUPER container and sidecar:**
```bash
bash /home/kayla/workspace/benchmark/integrations/aerial_nav/agent_hooks/super.sh
```

This starts the container with `--network=host`, launches `fsm_node`, and starts the TCP sidecar on port 8768.

**Step 2 — Launch AirStack with the SUPER planner:**
```bash
# Inside the robot container
ros2 launch super_planner super_planner.launch.xml
```

Or add it to your bringup in place of DROAN.

## Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `odometry` | `nav_msgs/Odometry` | subscribed (remapped to `/{robot}/odometry_conversion/odometry`) |
| `global_plan` | `nav_msgs/Path` | subscribed (remapped to `/{robot}/global_plan`) |
| `trajectory_override` | `airstack_msgs/TrajectoryXYZVYaw` | published (remapped to `/{robot}/trajectory_controller/trajectory_override`) |
| `point_cloud` | `sensor_msgs/PointCloud2` | subscribed (remapped to `/{robot}/sensors/ouster/point_cloud`) |

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `sidecar_host` | `127.0.0.1` | SUPER sidecar host |
| `sidecar_port` | `8768` | SUPER sidecar TCP port |
| `max_speed` | `2.0` | Max speed clamp sent to trajectory controller (m/s) |
| `step_hz` | `10.0` | Polling rate for sidecar step requests (Hz) |
| `max_lidar_points` | `512` | Max points forwarded to sidecar per step (stride-downsampled) |

## Building

```bash
# Inside the robot container
bws --packages-select super_planner
```
