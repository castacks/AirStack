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

### Omniverse
Install the Omniverse launcher download from this link:

```
wget https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage
```

Follow these instructions to setup Nucleus : https://airlab.slite.com/app/docs/X8dZ8w5S3GP9tw

### SITL
If you are using the Ascent Spirit drone download the SITL software packages from this link:
https://drive.google.com/file/d/1UxgezaTrHe4WJ28zsVeRhv1VYfOU5VK8/view?usp=drive_link

Then unzip the file AscentAeroSystemsSITLPackage.zip in this folder:

```
cd AirStack/simulation/AscentAeroSystems
unzip ~/Downloads/AscentAeroSystemsSITLPackage.zip -d .
```

### Docker
- Install [Docker Desktop](https://docs.docker.com/desktop/install/ubuntu/). This should come installed with docker compose.
- Gain access to NVIDIA NGC Containers by following [these instructions](https://docs.nvidia.com/launchpad/ai/base-command-coe/latest/bc-coe-docker-basics-step-02.html)

## Build and run the Docker image

```bash
cd AirStack/docker/
## build the image, it is named airlab-autonomy-dev:latest
docker compose --profile build build
## start docker compose service/container
docker compose up -d
```

## Launch

Launch autonomy stack controls package:

```bash
# start a new terminal in docker container
docker compose exec airstack_dev bash

# in docker
bws && sws ## build workspace and source workspace. these are aliases in ~/.bashrc
ros2 launch robot_bringup launch_robot.yaml
```

Launch simulator (Isaac Sim and Ascent SITL):

```bash
xhost +  ## allow Docker access to Linux X-Server
# start another terminal in docker container
docker compose exec airstack_dev bash

# in docker
ISAACSIM_PYTHON simulation/launch_sim.py
```

## Move Robot

```bash
# start another terminal in docker container
docker compose exec airstack_dev bash

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

## Setup Storage Tools Server Locally

### Clone and install

``` bash
git clone https://github.com/castacks/storage_tools_server
cd storage_tools_server
python -m venv venv
. venv/bin/activate
pip install -r requirements.txt
```

### Configure

Edit the `config/config.yaml` file to match your configuration.

### REQUIRED UPDATES

- `upload_dir` is the location for uploads.  This must be readable and writeable by the user running the Server.
- `volume_root` sets the prefix for all entries in the `volume_map`.  This must be readable and writeable by the user running the Server.
- `volume_map` is a mapping from project name to `volume_root/{path}`.  All projects must have a mapping.

### Set Environment and Run

- `CONFIG` is the full path to the `config.yaml` in use.  By default, the app will use `$PWD/config/config.yaml`
- `PORT` is the same port as define in the optional setup. The default is 8091.

``` bash
export CONFIG=$PWD/config/config.yaml
export PORT=8091

gunicorn -k gevent -w 1 -b "0.0.0.0:${PORT}" --timeout 120 "server.app:app"
```

Open a web browser to http://localhost:8091 (or the PORT you set). The default user is `admin` and the default password is `NodeNodeDevices`.

### Create an API Key for your robot

- Log into the Server
- Go to Configure -> Keys
- Enter a name for the device key in the "Add a new key name" field.
- Click "Generate Key"

## Set up Storage Tools Device on your Robot

### Install Requirements

- [Docker Compose](https://docs.docker.com/compose/install/standalone/)

### Clone Device Repo

```bash
cd /opt 
git clone https://github.com/castacks/storage_tools_device
cd stroage_tools_device
```

### Update the config.yaml

Update `config/config.yaml` to match your environment.  Things you should update:

- `API_KEY_TOKEN`.  The api key that your admin gave you, or the key that you set up in the [Server Setup](#create-an-api-key-for-your-robot)
- `watch`.  The list of directories that have your robot's files.

Update the `env.sh` to match your system.

- `CONFIG_FILE`.  If you have multiple config files, make sure `CONFIG_FILE` points to the one you want to use.
- `DATA_DIR`. This is the top level data directory that all of the `watch` dirs share.  For example, if you `watch` directories are `/mnt/data/processor_1` and `/mnt/data/processor_2`, set the `DATA_DIR` to `/mnt/data`.  

### Build and Run

``` bash
cd /opt/storage_tools_device
. env.sh
docker compose up --build
```