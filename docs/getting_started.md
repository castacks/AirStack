# Getting Started

By the end of this tutorial, you will have the autonomy stack running on your machine.

## Requirements

You need at least 25GB free to install the Docker image.

Have an NVIDIA GPU >= RTX 3070 to run Isaac Sim locally.

## Setup

### Clone

```
git clone --recursive -j8 git@github.com:castacks/AirStack.git
```

### Docker

Follow [NVIDIA's instructions](https://docs.nvidia.com/ai-enterprise/deployment/vmware/latest/docker.html) for installing Docker to be compatible with NVIDIA GPUs, including adding the NVIDIA Container Toolkit.
Make sure `docker-compose-plugin` is also installed with Docker.

## Configure

Run `./configure.sh` and follow the instructions in the prompts to do an initial configuration of the repo.

## Docker Images

Now you have two options on how to proceed. You can build the docker image from scratch or pull the existing image on the airlab docker registry. Building the image from scratch can be useful if you would like to add new dependencies or add new custom functionality. For most users just pulling the existing image will be more conveninent and fast since it doesn't require access to the Nvidia registry.

<details open> <summary>Option 1: Pull From the Airlab Registry (Preferred)</summary>
To use the AirLab Docker registry do the following

```bash
cd AirStack/
docker login airlab-storage.andrew.cmu.edu:5001
## <Enter your andrew id (without @andrew.cmu.edu)>
## <Enter your andrew password>

## Pull the images in the docker compose file
docker compose pull
```

The images will be pulled from the server automatically. This might take a while since the images are large.

</details>

<details><summary>Option 2: Build Docker Images From Scratch</summary>

1.  Download the Ascent Spirit SITL software package by running this script (pip3 is required):

    ```
    cd AirStack/
    bash simulation/isaac-sim/installation/download_sitl.bash
    ```

2.  Next, gain access to NVIDIA NGC Containers by following <a href="https://docs.nvidia.com/launchpad/ai/base-command-coe/latest/bc-coe-docker-basics-step-02.html">these instructions</a>.

    Then:

    ```bash
    cd AirStack/
    docker compose build  # build the images locally
    ```

If you have permission you can push updated images to the docker server.

```bash
docker compose push
```

</details>

## Launch

```bash
xhost +  # allow docker access to X-Server

# Make sure you are in the AirStack directory.

# Start docker compose services. This launches Isaac Sim and the robots.
#  You can append `--scale robot=[NUM_ROBOTS]` for more robots, default is 1
docker compose up -d
```

This will automatically launch and play the Isaac scene specified under `AirStack/.env` (default is the Fire Academy).

## Move Robot

Find the RQT GUI window. Hit `Takeoff`, then hit `Publish` in the trajectory window like in this video:

<iframe src="https://drive.google.com/file/d/1eF9mVqvIthb2NKyWrrZmk7dR8zTGBtmx/preview?usp=sharing&t=52" width="840" height="480" allow="autoplay" allowfullscreen="allowfullscreen"></iframe>

Note you can also use the `ros2 topic pub` command to move the robot. For example, to fly to a position:

```bash
# start another terminal in docker container
docker exec -it airstack-robot-1 bash
# in docker
# FLY TO POSITION. Put whatever position you want
ros2 topic pub /robot_1/interface/mavros/setpoint_position/local geometry_msgs/PoseStamped \
    "{ header: { stamp: { sec: 0, nanosec: 0 }, frame_id: 'base_link' }, \
    pose: { position: { x: 10.0, y: 0.0, z: 20.0 }, orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 } } }" -1
```

## Shutdown

To shutdown and remove docker containers:

```bash
docker compose down
```
