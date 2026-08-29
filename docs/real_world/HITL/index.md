<!-- verified against: gcs/docker/docker-compose.yaml (gcs-real: profiles hitl/deploy/offboard, network_mode: host, $HOME/bags mount);
     robot/docker/docker-compose.yaml (robot-l4t: profile l4t, network_mode: host, ROBOT_NAME_SOURCE=hostname, AIRSTACK_STACK_DIR default full_default, FCU_URL default /dev/ttyTHS4:115200, autolaunch robot.launch.xml sim:=false; robot-desktop SIM_IP=${SIM_IP:-172.31.0.200});
     simulation/isaac-sim/docker/docker-compose.yaml (isaac-sim on airstack_network bridge at fixed 172.31.0.200);
     robot/ros_ws/src/interface/interface_bringup/launch/interface.launch.py (FCU_URL env wins; else udp://:<14540+N>@$SIM_IP:<14580+N>);
     .env (COMPOSE_PROFILES default "desktop,isaac-sim"); airstack.sh (cmd_up, --profile passthrough, --stack, --no-autolaunch, ready/status/logs/connect);
     docs/robot/autonomy_modes.md; docs/robot/docker/robot_identity.md; docs/robot/autonomy/dds_router.md; docs/real_world/installation/index.md;
     common/fastdds.xml (UDP-only, shared memory disabled); tests/sensor_probes.py (front_stereo topic names);
     simulation/isaac-sim/launch_scripts/pegasus_app.py (drone domain_id = N convention) -->

# Hardware-In-The-Loop Simulation

Hardware-in-the-loop (HITL) testing runs the real onboard compute — a Jetson flying the same `robot-l4t` container it will fly in the field — against a simulator on a desktop machine, before any propellers spin. The desktop runs Isaac Sim (and optionally the GCS); the Jetson runs the autonomy stack; they talk ROS 2 over the LAN.

## Prerequisites

- A desktop set up per [Getting Started](../../getting_started/index.md), with the Isaac Sim image built or pulled.
- One or more Jetson ORIN AGX/NX set up per the [Installation Guide](../installation/index.md), with the `robot-l4t` image pulled and the repo cloned.
- All machines on the same LAN (we test with everything wired into one router). Verify each machine can `ping` the others.
- Each Jetson's hostname set to the `robot-<N>` convention (e.g. `hostnamectl set-hostname robot-1`) so [robot identity resolution](../../robot/docker/robot_identity.md) assigns `ROBOT_NAME=robot_1` / `ROS_DOMAIN_ID=1` — the fallback is silent, so check this first.

## Desktop: simulator (+ GCS)

The `hitl` compose profile selects `gcs-real` — the field variant of the GCS that runs with `network_mode: host` so its DDS participants sit directly on the LAN (see [GCS Docker Configuration](../../gcs/docker/index.md)). Combine it with the `isaac-sim` profile, overriding the `.env` default (`desktop,isaac-sim`) so no `robot-desktop` containers start on the desktop:

```bash
COMPOSE_PROFILES="hitl,isaac-sim" airstack up
```

This starts `isaac-sim` and `gcs-real` only. If you don't want a ground station, use `COMPOSE_PROFILES="isaac-sim" airstack up` for the simulator alone.

## Jetson: robot stack

On each Jetson, use the `l4t` profile (same command as the [Installation Guide](../installation/index.md); on a Jetson clone, also set `COMPOSE_PROFILES=l4t` in `.env` so the desktop default `desktop,isaac-sim` profiles are not active):

```bash
airstack --profile l4t up
```

`robot-l4t` runs with `network_mode: host` and autolaunches `robot.launch.xml sim:=false` with the `full_default` stack. To pick a different [stack](../../development/stacks.md), pass `--stack` (e.g. `airstack --profile l4t up --stack lite_default`), or add `--no-autolaunch` to start the container idle and launch manually.

## Networking: what has to line up

The old failure modes here are all networking. Check each of these:

- **Domain IDs.** Isaac Sim publishes each drone's topics on `ROS_DOMAIN_ID = N` (drone `domain_id` convention in the Pegasus launch scripts), and the Jetson's hostname must resolve to the same domain via the [robot name map](../../robot/docker/robot_identity.md) — `robot-1` → domain 1 matches sim drone 1. Verify inside the container: `docker exec airstack-robot-l4t-1 bash -c 'echo "$ROBOT_NAME / $ROS_DOMAIN_ID"'`.
- **DDS transport.** All AirStack containers load `common/fastdds.xml`, which forces plain UDP and disables shared memory — required for topics to cross container and machine boundaries at all.
- **Sim container is on a Docker bridge.** The `isaac-sim` container joins the internal `airstack_network` bridge at the fixed address `172.31.0.200`, while the Jetson containers are host-networked on the LAN. DDS multicast discovery does not traverse the desktop's Docker bridge NAT by itself, and the repo does not ship a peer configuration for this topology. If the Jetson does not see sim topics (`ros2 topic list` empty apart from local nodes), you will need to configure cross-machine discovery yourself — e.g. a Fast DDS initial-peers profile or a discovery server pointing at the desktop's LAN IP — and verify it; this step is not currently automated or tested in CI.
- **Flight controller connection.** `robot-l4t` defaults to a real flight controller on serial (`FCU_URL=/dev/ttyTHS4:115200`) — with a physical FCU wired to the Jetson, MAVROS talks to real hardware while sensors come from the sim. If instead you want MAVROS to reach the PX4 SITL instance inside Isaac Sim, `FCU_URL` is env-overridable (unset, `interface_bringup`'s `interface.launch.py` computes `udp://:<14540+N>@$SIM_IP:<14580+N>`), but note `SIM_IP`'s default `172.31.0.200` is only reachable from the desktop's own Docker network, and the compose files do not publish PX4's UDP ports to the LAN — this cross-machine SITL path is untested; expect to forward those ports yourself.
- **GCS visibility.** With the `full_default` stack, the robot runs a [DDS router](../../robot/autonomy/dds_router.md) that bridges an allowlist of its topics into domain 0, where the host-networked `gcs-real` listens.

## Verification

```bash
airstack status                                           # containers up on each machine
docker exec airstack-robot-l4t-1 bash -c "ros2 node list"     # stack nodes present
docker exec airstack-robot-l4t-1 bash -c "ros2 topic hz /robot_1/sensors/front_stereo/left/image_rect"
```

Once the scene is playing in Isaac Sim, sensor topics on the Jetson should tick at a steady rate. On the desktop, `gcs-real` launches Foxglove Studio (host networking: connect your own Foxglove to `ws://localhost:8765`) — sensor and odometry panels streaming live data confirm the LAN link end to end. See [GCS Foxglove Visualization](../../gcs/foxglove.md). Use `airstack logs robot-l4t` / `airstack connect robot-l4t` to debug the bringup session.

!!! note "Tested configuration"
    The demos below were recorded on an earlier AirStack release (raw `docker compose`, RViz-based verification). The commands on this page reflect the current CLI and compose profiles but this exact multi-machine topology is not covered by CI — treat the discovery and SITL caveats above as things to verify on your own network.

## Demos

Screen recording of the desktop machine:
<iframe src="https://drive.google.com/file/d/1sNkEattgDyBAI9xFPVQsXn8sRi6gYQSG/view?usp=sharing" width="840" height="480" allow="autoplay" allowfullscreen="allowfullscreen"></iframe>

Screen recording of the Jetson:
<iframe src="https://drive.google.com/file/d/19S8Yceq8t2FPubN8Mly003Up1AbLzCA8/view?usp=sharing" width="840" height="480" allow="autoplay" allowfullscreen="allowfullscreen"></iframe>
