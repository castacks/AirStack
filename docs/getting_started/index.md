# Getting Started

!!! warning ""

    AirStack is currently in ALPHA and only meant for internal usage. 

    We'd really appreciate your feedback and contributions to improve this project for everyone! 
    Please join our #airstack channel on Slack to contribute or ask questions.

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
airstack image-pull
```

The images will be pulled from the server automatically. This might take a while since the images are large.

</details>

<details><summary>Option 2: Build Docker Images From Scratch</summary>

1.  First, gain access to NVIDIA NGC Containers by following <a href="https://docs.nvidia.com/launchpad/ai/base-command-coe/latest/bc-coe-docker-basics-step-02.html">these instructions</a>.

2. Then:

    ```bash
    cd AirStack/
    airstack image-build
    ```

If you have permission you can push updated images to the docker server.

```bash
airstack image-push
```

</details>

## Launch

```bash
airstack up # This will launch the robot, ground control station, and isaac sim
```

This will automatically launch and play the Isaac scene specified under `AirStack/.env` (default is the Fire Academy).

## Move Robot

Find the RViz window, `Takeoff`, then `Navigate` and `Explore` in the trajectory window like in this video:

<iframe width="560" height="315" src="https://www.youtube.com/embed/EAKsHzNIU2I?si=zQUFq8fPst2BIIMz" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>


## Shutdown

To shutdown and remove docker containers:

```bash
airstack down # This will stop and remove the docker containers
```

Congratulations! You did it. 
