# Getting Started
Welcome to the AirLab Autonomy Stack. By the end of this guide, you will have the autonomy stack running on your machine.

## Requirements

You need at least 25GB free to install the Docker image.

Have an NVIDIA GPU >= RTX 3070 to run Isaac Sim locally.

## Setup
### Clone
```
git clone --recursive -j8 git@github.com:castacks/AirStack.git
```
### Docker
Install [Docker Desktop](https://docs.docker.com/desktop/install/ubuntu/). This should come installed with docker compose.

### Running.
Now you have two options on how to proceed. You can build the docker image from scratch or pull the existing image on the airlab docker registry. Building the image from scratch can be  useful if you would like to add new dependencies or add new custom functionality. For most users just pulling the existing image will be more conveninent and fast since it doesn't require access to the Nvidia registry.

### Option 1 (Preferred): Use the Airlab Docker registry

To use the AirLab docker registry do the following
```bash
cd AirStack/docker/
docker login airlab-storage.andrew.cmu.edu:5001
## <Enter your andrew id (without @andrew.cmu.edu)>
## <Enter your andrew password>

## Pull the images in the docker compose file
docker compose pull 
```
When you execute docker compose pull in the next step the image will be pulled from the server automatically. This might take a while since the image is large.


### Option 2: Setup from scratch
#### Omniverse
Install the Omniverse launcher download from this link:

```
wget https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage
```

Follow these instructions to setup Nucleus : https://airlab.slite.com/app/docs/X8dZ8w5S3GP9tw


#### Build and run the Docker image
 Gain access to NVIDIA NGC Containers by following [these instructions](https://docs.nvidia.com/launchpad/ai/base-command-coe/latest/bc-coe-docker-basics-step-02.html)

```bash
cd AirStack/docker/
docker compose build  # build the images locally 
```

## SITL (Required until we add to docker image)
If you are using the Ascent Spirit drone download the SITL software packages from this link:
https://drive.google.com/file/d/1UxgezaTrHe4WJ28zsVeRhv1VYfOU5VK8/view?usp=drive_link

Then unzip the file AscentAeroSystemsSITLPackage.zip in this folder:

```
cd AirStack/simulation/AscentAeroSystems
unzip ~/Downloads/AscentAeroSystemsSITLPackage.zip -d .
```

## Configure 

Follow the instructions in `docker/isaac-sim/omni_pass.env` to configure the required settings for your Omniverse Nucelus Server token.
To generate a token, follow the NVIDIA docs [here](https://docs.omniverse.nvidia.com/nucleus/latest/config-and-info/api_tokens.html).


## Launch

Launch the docker services:

```bash

xhost +  # allow docker access to X-Server

# Make sure you are in the AirStack/docker directory.

# Start docker compose services, 
# can append `--scale robot=[NUM_ROBOTS]` for more robots, default is 1
docker compose up -d 
# view running containers
docker ps -a
# attach to an existing container
docker attach docker-robot-1

# in docker
bws && sws ## build workspace and source workspace. these are aliases in ~/.bashrc
ros2 launch robot_bringup launch_robot.yaml
```

Launch Isaac Sim in headless mode:

```bash
# in another terminal under AirStack/docker
docker compose exec isaac-sim bash
# within docker
./runheadless.native.sh
```

Connect to Isaac Sim via [Omniverse Streaming Client](https://docs.omniverse.nvidia.com/streaming-client/latest/user-manual.html):
```bash
# get the IP of the isaac-sim container
docker inspect -f '{{range.NetworkSettings.Networks}}{{.IPAddress}}{{end}}' isaac-sim
```
Paste the outputted IP into the Omniverse Streaming Client.

## Move Robot (THIS NEEDS UPDATES)

```bash
# start another terminal in docker container
docker compose exec robot-1 bash

# in docker
# set drone mode to GUIDED
ros2 service call /robot1/controls/mavros/set_mode mavros_msgs/SetMode "custom_mode: 'GUIDED'"
# ARM
ros2 service call /robot1/controls/mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: True}"
# TAKEOFF
ros2 service call /robot1/controls/mavros/cmd/takeoff mavros_msgs/srv/CommandTOL "{altitude: 5}"
# FLY TO POSITION. Put whatever position you want
ros2 topic pub /controls/mavros/setpoint_position/local geometry_msgs/PoseStamped \
    "{ header: { stamp: { sec: 0, nanosec: 0 }, frame_id: 'base_link' }, \
    pose: { position: { x: 10.0, y: 0.0, z: 20.0 }, orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 } } }" -1
```
## Shutdown

To pause containers:
```bash
docker compose stop
```

To shutdown and remove docker containers:
```bash
docker compose down
```