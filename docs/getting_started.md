# Getting Started

!!! warning ""

    AirStack is currently in ALPHA and only meant for internal usage. 
    You will need to have an account with AirLab to access the AirLab Docker registry, Nucleus server, and other resources.
    The API and functionality are not stable and are subject to change. 


By the end of this tutorial, you will have the autonomy stack running on your machine.

## Requirements

You need at least 25GB free to install the Docker image.

Check the hardware requirements for the NVIDIA Isaac Sim [here](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html).
A GPU of GeForce RTX 4080 or higher is recommended for the best performance.

AirStack is primarily tested on Ubuntu 22.04. 

## Clone
```bash
git clone --recursive -j8 git@github.com:castacks/AirStack.git
cd AirStack
```

## Install and Setup

```bash
./airstack.sh install  # installs docker and docker-compose
./airstack.sh setup  # this lets you use the `airstack` command and sets up your keys
```

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

1.  Download the Ascent Spirit SITL software package by running this script:

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
xhost +     # This is needed every system boot to allow Docker to access the X server
airstack up # This will launch the robot, ground control station, and isaac sim
```

This will automatically launch and play the Isaac scene specified under `AirStack/.env` (default is the Fire Academy).

## Move Robot

Find the RQT GUI window. Hit `Arm and Takeoff`, then hit `Global Plan` in the trajectory window like in this video:

<iframe src="https://drive.google.com/file/d/1XYgSUTU5tf6e6sOuStYJXIs2SK3XL7g6/preview?usp=sharing&t=0" width="840" height="480" allow="autoplay" allowfullscreen="allowfullscreen"></iframe>

You can also switch to `Fixed Trajectory` mode and hit `Publish` on the bottom right to fly a predefined trajectory.

## Shutdown

To shutdown and remove docker containers:

```bash
airstack down # This will stop and remove the docker containers
```

Congratulations! You did it. Keep reading the Developer Guide to learn how to work with AirStack.