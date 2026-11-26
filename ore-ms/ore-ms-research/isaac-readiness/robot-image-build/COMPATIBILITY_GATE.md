# Compatibility gate

## Source and Compose checks completed offline

- Resolved the single-robot desktop/livestream configuration using Compose without reading service env files. Confirmed the intended context, Dockerfile and arguments, unchanged robot and GCS tags and pinned Isaac digest, one robot replica, and single-GPU reservations on Isaac and robot. These declarations do not measure GPU placement. The expected simulator IP is `172.31.0.200` on the AirStack network.
- Verified the runtime mounts include the host robot workspace, common ROS packages, stacks and robot `.bashrc`. The latter runs colcon through `autolaunch` and launches `desktop_bringup robot.launch.xml` using the `full_default` stack by default. Bash, Python and XML syntax checks passed for the selected launch inputs.
- Reviewed the MAVROS launch namespace binding and PX4 SITL defaults. The robot launch adds `interface` and supplies an empty MAVROS namespace to avoid a repeated `mavros` segment. With the default `robot_1` in domain 1, the source computes `udp://:14541@172.31.0.200:14581` and target system 2. The Isaac launcher selects domain 1. Record the actual identity, ports and topics during integration.
- The existing four-case startup fixture verified digest propagation, `--no-build`, unchanged robot/GCS references and no launch fallback on a simulated error. These checks establish the wiring and parsing contract.

## Static checks after a candidate exists

In the Stage 1 shell, use the local candidate and its evidence directory. This probe starts an isolated shell with no network or launch services. It does not exercise the mounted workspace.

```bash
test "$(docker image inspect --format '{{.Os}}/{{.Architecture}}' "$candidate")" = linux/amd64
docker run --rm --network none --entrypoint /bin/bash "$candidate" -ec '
  test "$ROS_DISTRO" = jazzy
  test -r /opt/ros/jazzy/setup.bash
  source /opt/ros/jazzy/setup.bash
  command -v ros2 colcon tmux Xvfb
  test -x /usr/sbin/sshd
  test -x /opt/ros/jazzy/lib/mavros/mavros_node
  ros2 pkg prefix mavros
  dpkg-query -W ros-jazzy-mavros ros-jazzy-mavros-extras
  dpkg-query -W
  python3 -m pip freeze --all
' | tee "$out/package-inventory.txt"
```

## Bounded runtime acceptance after separate approval

Use a clean mounted workspace at the frozen source and a grounded, disarmed simulator. Use the one-robot configuration and qualified Isaac image `airlab-docker.andrew.cmu.edu/airstack/airstack@sha256:ba2402ec4f389fad6552d79e87871131c587d090963d1d894539016904f3e697`. Use an isolated integration configuration selecting the exact temporary candidate. Preserve the production image references and no-build safeguard.

1. Record image and source identities and first startup logs. Require mounted colcon success, working launch packages and sentinel nodes, and a passing bounded `./airstack.sh ready --json`. Preserve its first result. The existing readiness check uses bounded container, clock, node and PX4 waits.
2. Capture one 60-second observation after readiness. Require advancing robot-domain `/clock`, connected/disarmed `/robot_1/interface/mavros/state`, and fresh odometry on `/robot_1/interface/mavros/local_position/odom`. Verify canonical topic names and correct simulated time basis.
3. Verify dynamic TF for the active odometry frame pair at representative odometry timestamps, static transforms where needed and no persistent stale TF or clock-reset warnings. Record the actual frame IDs and sampled timestamps.
4. On failure, preserve evidence and stop. The prior `ore_proj` restart is outside default startup. Consider separately authorized recovery only after an observed clock/TF failure. A successful build or package probe does not establish these runtime properties.
