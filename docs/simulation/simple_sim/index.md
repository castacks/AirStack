# Simple Sim

Simple Sim is AirStack's lightweight kinematic simulator: a single C++ ROS 2
node that flies a simplified drone model through one OpenGL-rendered mesh
world — **no PX4, no MAVROS, no Isaac Sim**. It is actively used by core
maintainer John Keller for fast planner/perception iteration.

## Quick Start

```bash
airstack up --sim simple
```

This starts two containers:

- **`simple-sim`** — the simulator ([`simulation/simple-sim/`](https://github.com/castacks/AirStack/tree/main/simulation/simple-sim)). On first start it downloads the world mesh (`models/download.sh`), colcon-builds its small workspace, then runs `ros2 launch sim sim.launch.xml` on `ROS_DOMAIN_ID=1`.
- **`airstack-simple-robot-1`** — the autonomy stack. The `simple-robot` compose service extends `robot-desktop` with `SIM_TYPE=simple`, which makes the interface layer **skip MAVROS** (`interface_bringup/launch/interface.launch.py`); everything else in the stack launches as usual.

`--sim simple` deliberately drops the `desktop` profile: `simple-robot`
*replaces* `robot-desktop` (both would otherwise claim `robot_1` on domain 1),
and no GCS container is started.

## How it works

The sim node (`MavrosMockNode`, launched as `/sim`) **impersonates the MAVROS
surface** the autonomy stack talks to, so the stack runs unmodified minus
MAVROS itself:

| Direction | Interface |
|---|---|
| Serves | `/robot_1/interface/mavros/set_mode`, `.../cmd/arming`, `.../cmd/takeoff` |
| Subscribes | `/robot_1/interface/mavros/setpoint_raw/attitude` (attitude + thrust setpoints) |
| Publishes | `/robot_1/interface/mavros/state`, `/robot_1/interface/mavros/local_position/odom`, `/clock` |
| Publishes | `/robot_1/sensors/front_stereo/{left,right}/image_rect` + `camera_info` (OpenGL-rendered stereo pair of the FBX world) |

Static TFs for the stereo pair (`base_link` → camera links → optical frames)
are published by the sim's launch file. After a takeoff service call the sim
briefly runs in a fast-forward mode to skip the stack's post-takeoff wait.

**Single robot only:** all topics and services are hardcoded to `robot_1` on
`ROS_DOMAIN_ID=1`. There is no multi-robot support.

## When to use it

- Fast iteration on planning / control / stereo-perception code — startup is a
  small colcon build plus a mesh load, not an Isaac Sim boot.
- Machines without an Isaac-class GPU or Omniverse credentials. (The container
  still needs OpenGL: an X display is mounted in, and the compose file
  reserves an NVIDIA GPU via the nvidia container runtime.)
- Not for: PX4/MAVROS behavior, LiDAR, physics fidelity, multi-robot, or
  final validation — use [Isaac Sim](../isaac_sim/index.md) for those.

## Smoke test

A dedicated system test verifies the simple-sim bring-up (containers,
`/clock`, mock-MAVROS odometry reaching the robot, and the `SIM_TYPE=simple`
sentinel nodes — MAVROS intentionally absent):

```bash
airstack test -m simple_sim --sim simplesim --num-robots 1 -v
```

## See Also

- [Docker Configuration](docker.md) — the `simple-sim` container in detail
- [Isaac Sim](../isaac_sim/index.md) — high-fidelity alternative
- [Simulation Overview](../index.md)
