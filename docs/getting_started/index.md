# Getting Started

!!! tip "On Mac, Windows, or no GPU?"

    This page assumes a Linux desktop with an NVIDIA GPU. If you don't have
    one, [AirStack on OSMO](../tutorials/airstack_on_osmo.md) is the
    recommended remote development path — you only need an SSH key, the
    `osmo` CLI, and VS Code or Cursor. No local Docker, no NVIDIA drivers,
    no `airstack install`.

!!! warning ""

    AirStack is currently in ALPHA and only meant for internal usage. 

    We'd really appreciate your feedback and contributions to improve this project for everyone! 
    Please join our #airstack channel on Slack to contribute or ask questions.

    You will need to have an account with AirLab to access the AirLab Docker registry, Nucleus server, and other resources.
    The API and functionality are not stable and are subject to change. 


By the end of this tutorial, you will have the autonomy stack running on your machine.

## Requirements

The Docker images take about 25GB; we recommend at least 100GB of free disk space.

Check the hardware requirements for the NVIDIA Isaac Sim [here](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html).
An NVIDIA RTX 3070 is the minimum GPU; a GeForce RTX 4080 or better is recommended for the best performance.

AirStack is tested on Ubuntu 22.04 and 24.04. 

## Clone
```bash
git clone --recursive -j8 git@github.com:castacks/AirStack.git
cd AirStack
```

## Install and Setup

```bash
./airstack.sh install  # installs docker, docker-compose, and the NVIDIA Container Toolkit
./airstack.sh setup  # this lets you use the `airstack` command and sets up your keys
source ~/.bashrc  # OR ~/.zshrc. applies settings to enable `airstack` command
```

## Docker Images

Now you have two options on how to proceed. You can build the docker image from scratch or pull the existing image on the airlab docker registry. Building the image from scratch can be useful if you would like to add new dependencies or add new custom functionality. For most users just pulling the existing image will be more conveninent and fast since it doesn't require access to the Nvidia registry.

<details open> <summary>Option 1: Pull From the Airlab Docker Registry (Preferred)</summary>
To use the AirLab Docker registry do the following

```bash
cd AirStack/
docker login airlab-docker.andrew.cmu.edu
## <Enter your andrew id (without @andrew.cmu.edu)>
## <Enter your andrew password>

## Pull the images in the docker compose file
airstack images pull
```

The images will be pulled from the server automatically. This might take a while since the images are large.

</details>

<details><summary>Option 2: Build Docker Images From Scratch</summary>

1.  First, gain access to NVIDIA NGC Containers by following <a href="https://docs.nvidia.com/launchpad/ai/base-command-coe/latest/bc-coe-docker-basics-step-02.html">these instructions</a>.

2. Then:

    ```bash
    cd AirStack/
    airstack images build
    ```

If you have permission you can push updated images to the docker server.

```bash
airstack images push
```

</details>

## Launch

```bash
airstack up # This will launch the robot, ground control station, and isaac sim
```

This launches the Isaac Sim scene specified by `ISAAC_SIM_SCRIPT_NAME` in `AirStack/.env` (default: a single drone in the Pegasus default environment). By default the sim comes up **playing** (`PLAY_SIM_ON_START="true"` in `.env`) — launch with `airstack up --no-play` to come up paused and press **Play** in the Isaac Sim window yourself.

Containers start immediately, but the ROS 2 workspace still builds and PX4 still boots in the background. To wait until the drone is actually ready to fly:

```bash
airstack ready   # or: airstack up --wait
```

Useful variants (see `airstack help up`):

```bash
airstack up --sim airsim          # MS AirSim instead of Isaac Sim
airstack up --sim isaac --robots 3  # multi-robot (auto-selects the multi-drone scene script)
```

## Move Robot

Foxglove opens automatically in the GCS with the AirStack layout already loaded — no manual import or connection needed (see [GCS Foxglove Visualization](../gcs/foxglove.md)). Press `Takeoff` in the Robot Tasks panel, then `Navigate`, like in this video:

<iframe width="560" height="315" src="https://www.youtube.com/embed/EAKsHzNIU2I?si=zQUFq8fPst2BIIMz" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

!!! note
    If you instead connect a Foxglove running on the host, the bridge is exposed on host port `8766` (container port `8765`): connect to `ws://localhost:8766`.


## Shutdown

To shutdown and remove docker containers:

```bash
airstack down # This will stop and remove the docker containers
```

Congratulations! You did it.

**Next:** the [Modular AirStack Walkthrough](modular_airstack.md) — fly a reference stack, add a module from the [catalog](../modules/index.md), build your own stack, and scale to a fleet. 
