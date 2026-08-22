# Simple Sim Docker Configuration

The simulator runs in one container defined in
[`simulation/simple-sim/docker/docker-compose.yaml`](https://github.com/castacks/AirStack/blob/main/simulation/simple-sim/docker/docker-compose.yaml).

## Service

| Property | Value |
|---|---|
| Service / container name | `simple-sim` |
| Compose profile | `simple` (activated by `airstack up --sim simple`) |
| Image | `osrf/ros:jazzy-desktop-full` + GLFW/GLM/Assimp and dev tools (`Dockerfile.sim`) |
| Network | `airstack_network`, fixed IP **172.31.0.200** — the same `SIM_IP` address every sim service binds, so only one simulator can run at a time |
| Paired robot service | `simple-robot` (in `robot/docker/docker-compose.yaml`): extends `robot-desktop` with `SIM_TYPE=simple`, same `simple` profile |

## Startup sequence

The container command runs, inside a tmux session named `sim`:

1. `cd /models && ./download.sh` — fetch the world mesh on first start
2. `colcon build --symlink-install` in `/root/ros_ws` (the workspace is
   bind-mounted from `simulation/simple-sim/ros_ws`, so it builds at container
   start, not at image build)
3. `ROS_DOMAIN_ID=1 ros2 launch sim sim.launch.xml`

Expect the sim's topics (`/clock`, mock-MAVROS state/odom, stereo images —
see the [overview](index.md)) to appear only after the build finishes.

## Requirements

- **X display + OpenGL:** the sim renders the stereo pair with GLFW; the
  compose file mounts `~/.Xauthority` and `/tmp/.X11-unix` from the host.
- **NVIDIA container runtime:** the service reserves one GPU
  (`deploy.resources.reservations.devices`). The rendering itself is plain
  OpenGL — far below Isaac Sim's requirements — but as written the compose
  file will not start without the nvidia runtime.

## Working with the container

```bash
airstack connect simple-sim      # shell in; `tmux a -t sim` for the sim pane
airstack logs simple-sim         # tmux output mirrored to docker logs

# Rebuild the sim package after editing simulation/simple-sim/ros_ws/src/sim/
docker exec simple-sim bash -c "cd ~/ros_ws && colcon build --symlink-install --packages-select sim"

# See what the sim is publishing (domain 1)
docker exec simple-sim bash -c "source /opt/ros/jazzy/setup.bash && ROS_DOMAIN_ID=1 ros2 topic list"
```

Scene parameters (camera FOV/resolution/baseline, world mesh path, scale,
offsets) are plain ROS parameters in
`simulation/simple-sim/ros_ws/src/sim/launch/sim.launch.xml`.

## Troubleshooting

- **No topics after several minutes** — the startup colcon build failed or the
  model download stalled: `airstack logs simple-sim`.
- **GLFW / display errors** — no usable X display in the container: check
  `echo $DISPLAY` on the host and `xhost +local:docker`.
- **Robot sees nothing** — the stack must run as `robot_1` on domain 1 (the
  sim hardcodes both); verify the robot container is
  `airstack-simple-robot-1` and was started via `airstack up --sim simple`,
  not alongside a `desktop`-profile robot.

## Smoke test

```bash
airstack test -m simple_sim --sim simplesim --num-robots 1 -v
```

## See Also

- [Simple Sim Overview](index.md)
- [Isaac Sim Docker](../isaac_sim/docker.md)
