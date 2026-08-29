# Docker Workflow Guide

This guide covers practical Docker operations for AirStack development. For concepts, see [Key Concepts](../key_concepts.md).

## Quick Reference

```bash
# Start/stop
airstack up              # Start all services
airstack up robot-desktop        # Start only robot
airstack down            # Stop all services

# Manage
airstack status          # Show running containers
airstack connect robot   # Connect to container
airstack logs robot      # View logs

# Pull/build images
docker compose pull      # Pull from registry
docker compose build     # Build from scratch
```

## Pull Images

The AirLab docker registry (`airlab-docker.andrew.cmu.edu`) is public — no `docker login` is needed to pull:

```bash
cd AirStack/

## Pull the images in the docker compose file
docker compose pull
```

The available image tags are listed [here](https://airlab-docker.andrew.cmu.edu/harbor/projects/2/repositories/airstack/artifacts-tab).

Pushing images still requires an AirLab account (`docker login airlab-docker.andrew.cmu.edu` first).

## Build Images

For an overview of build-time options (`BASE_IMAGE`, `ROS_DISTRO`, platform profiles), see [Docker Build Profiles](../../intermediate/docker-build-profiles.md). For runtime container operations, continue below.

```bash
# Build all images from scratch
docker compose build

# Build specific service
docker compose build robot
```

### Isaac Sim

Start a bash shell in the Isaac Sim container:

```bash
# if the isaac container is already running, execute a bash shell in it
airstack connect isaac-sim  # or equivalently: docker exec -it isaac-sim bash
```

Within the isaac-sim Docker container, the alias `runapp` launches Isaac Sim.
The `--path` argument can be passed with a path to a `.usd` file to load a scene.

It can also be run in headless mode (`airstack up --sim isaac --headless`) and accessed remotely via WebRTC streaming — see [Isaac Sim Docker → Accessing Isaac Sim](../../../simulation/isaac_sim/docker.md) for the current access methods. (The Omniverse Streaming Client and `runheadless.native.sh` have been discontinued upstream.)

The container also has the isaacsim ROS2 package within that can be launched with `ros2 launch isaacsim run_isaacsim.launch.py`.

### Robot

Start a bash shell in a robot container, e.g. for robot_1:

```bash
airstack connect robot  # or equivalently: docker exec -it airstack-robot-desktop-1 bash
```

To launch more than one robot, use `--robots` — it sets `NUM_ROBOTS` **and** keeps the Isaac launch script consistent (a plain `NUM_ROBOTS=2 airstack up` with the single-drone default script is rejected by preflight, since only one drone would exist in sim):

```bash
airstack up --sim isaac --robots 2
airstack connect robot-1  # to connect to robot 1
airstack connect robot-2  # to connect to robot 2
```

### Launch flags and readiness

`airstack up` accepts intent flags that derive the coordinated env-var sets for you (they override `.env` for that run without editing it). For example:

```bash
airstack up --sim isaac --robots 2   # simulator profile + matching URDF + multi-drone script
airstack up --no-autolaunch          # idle containers, no tmux launch (development)
airstack up --headless --wait        # no sim window; block until flight-ready
```

The full flags table (`--play`/`--no-play`, `--dry-run`, `--stack`, `--fleet`, `--scene`, ...) is in the [CLI reference](index.md#airstack-up-flags).

Every `up` prints the resolved launch config and saves it to `.airstack/runs/<timestamp>/effective_config.env`. Preflight validates the resolved values (one simulator profile, URDF pairing, robot-count/script consistency, missing images by name) before compose runs; `AIRSTACK_SKIP_PREFLIGHT=1` downgrades errors to warnings.

`airstack up` returns as soon as containers start — workspaces may still be building and the sim loading. To wait for actual flight-readiness (containers → sim `/clock` → per-robot autonomy nodes → PX4 connected + EKF armable):

```bash
airstack ready          # staged progress, per-gate diagnostics
airstack ready --json   # machine-readable (last line), exit 0 when ready
```

With `AUTOLAUNCH` enabled (the default), each robot container launches the autonomy stack in a tmux session. To attach to the session within the docker container, e.g. to inspect output, run `tmux a`.

The following commands are available within the robot container:

```bash
# in robot docker
cws  # cleans workspace
bws  # builds workspace
bws --packages-select [your_packages] # builds only desired packages
sws  # sources workspace
ros2 launch autonomy_bringup robot.launch.xml  # top-level launch (dispatches the selected stack)
```

These aliases are defined in `AirStack/robot/docker/.bashrc`.

Each robot has `ROS_DOMAIN_ID` set to its ID number. `ROBOT_NAME` is set to `robot_$ROS_DOMAIN_ID`.

### Ground Control Station

Currently the ground control station uses the same image as the robot container. This may change in the future.

Start a bash shell in a robot container:

```bash
airstack connect gcs  # or equivalently: docker exec -it gcs bash
```

The available aliases within the container are currently the same.

On the GCS `ROS_DOMAIN_ID` is set to 0.

## SSH into Robots

The containers mimic the robots' onboard computers on the same network. Therefore we intend to interface with the robots through ssh.

The `gcs` and robot containers are set up with an ssh daemon, so you can ssh into the containers using the IP address.

You can get the IP address of each container by running the following command:

```bash
docker inspect -f '{{range.NetworkSettings.Networks}}{{.IPAddress}}{{end}}' [CONTAINER-NAME]
```

Then ssh in, for example (containers get addresses on the `172.31.0.0/24` bridge network; use the `docker inspect` command above to find the actual IP):

```bash
ssh root@172.31.0.5
```

The ssh password is `airstack`.

## Automated Testing

Automated testing is handled by the pytest-based test harness: run `airstack test -m <mark>` (e.g. `airstack test -m unit -v`) in a containerized runner. See the [Testing docs](../../intermediate/testing/index.md) and [`tests/README.md`](../../../../tests/README.md) for the full mark reference and options.

## Docker Compose Variable Overrides

As mentioned above, the `airstack` CLI is a wrapper around Docker Compose.
Therefore, it supports [variable interpolation](https://docs.docker.com/compose/how-tos/environment-variables/variable-interpolation/) in the `docker-compose.yaml` file, allowing you to adjust project settings by modifying environment variables.

For settings that have a launch-intent flag, prefer the flag — it derives
the env vars for you (see the [flag reference](index.md)): `--no-play`
instead of `PLAY_SIM_ON_START=false`, `--no-autolaunch` instead of
`AUTOLAUNCH=false`, `--robots N` instead of `NUM_ROBOTS=N`, plus `--sim`,
`--headless`, `--scene`, `--stack`, and `--fleet`.

Environment variables remain the mechanism for everything **without** a
flag. For example, to run a custom Isaac Sim launch script (they live in
`simulation/isaac-sim/launch_scripts/`):

```bash
ISAAC_SIM_SCRIPT_NAME=your_launch_script.py airstack up
```

A list of all available environment variables is in the default `.env` file in the project root directory, which allows specifying all the variables in one place.
When no `--env-file` argument is passed to `docker compose`, it automatically uses this default `.env` file.

The default `.env` file is reproduced below:

```bash
--8<-- ".env"
```

To override the default `.env` file, you can pass the `--env-file` argument with the syntax `airstack --env-file [env_file] up` (or `docker compose --env-file [env_file] up -d`).

Multiple `--env-file` arguments can be passed to compose overriding sets of `variables.env` files.
All subsequent `--env-file` arguments override the previous ones, allowing you to layer configurations.
