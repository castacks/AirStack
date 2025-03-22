# Getting Started

Welcome to the AirLab Autonomy Stack. By the end of this guide, you will have the autonomy stack running on your machine.

## Requirements

You need at least 25GB free to install the Docker image.

Have an NVIDIA GPU >= RTX 3070 to run Isaac Sim locally.

## Setup

### Clone

```bash
git clone --recursive -j8 git@github.com:castacks/AirStack.git
```

### Omniverse

Install the Omniverse launcher download from this link:

```bash

wget https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage
```

Follow these instructions to setup Nucleus : [https://airlab.slite.com/app/docs/X8dZ8w5S3GP9tw](https://airlab.slite.com/app/docs/X8dZ8w5S3GP9tw)

### SITL

If you are using the Ascent Spirit drone download the SITL software packages from this link:
[https://drive.google.com/file/d/1UxgezaTrHe4WJ28zsVeRhv1VYfOU5VK8/view?usp=drive_link](https://drive.google.com/file/d/1UxgezaTrHe4WJ28zsVeRhv1VYfOU5VK8/view?usp=drive_link)

Then unzip the file AscentAeroSystemsSITLPackage.zip in this folder:

```bash
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

### Set Environment and Run

- `CONFIG` is the full path to the `config.yaml` in use.  By default, the app will use `$PWD/config/config.yaml`
- `PORT` is the same port as define in the optional setup. The default is 8091.

``` bash
export CONFIG=$PWD/config/config.yaml
export PORT=8091

gunicorn -k gevent -w 1 -b "0.0.0.0:${PORT}" --timeout 120 "server.app:app"
```

Open a web browser to [http://localhost:8091](http://localhost:8091) (or the PORT you set). The default user is `admin` and the default password is `NodeNodeDevices`.

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

### Update the config.env

Update the `config.env` to match your system.

- Required
  - `DATA_DIR`. This is the top level data directory that all of the `watch` dirs share.  For example, if you `watch` directories are `/mnt/data/processor_1` and `/mnt/data/processor_2`, set the `DATA_DIR` to `/mnt/data`.  
- Optional
  - `CONFIG_PORT`. The HTTP port to edit the configuration.  Navigate to http://YOUR_DEVICE_IP:CONFIG_PORT to edit the configurations and view online status. The default port is  8811.

### Build and Run

This sets up the environment and configures the image to start on boot.

``` bash
cd /opt/storage_tools_device

# run in foreground 
docker compose --env-file config.env up --build --remove-orphans

# run in background
docker compose --env-file config.env up --build --remove-orphans -d
```

### Configure the Device

Navigate to http://YOUR_DEVICE_IP:CONFIG_PORT to edit the configurations and view online status. The default port is  8811.

### Device Configuration

You must update all *Required* fields.  

- **Project Name**:  Name of the project. If this is empty, the server will ask you to fill in the name.
- **Robot Name**: (Required) Name of this robot. This should be unique within the project.
- **API Key Token**: (Required) The API_KEY_TOKEN for this robot. Your admin can provide this to you. If you are running your own upload server, it is set in the Config->Keys page.
- **Watch**: (Required) The list of directories to be watched.  Any file with a matching suffix (see Include Suffix) will be uploaded. These all must be in the same subdirectory as `DATA_DIR` from the `config.env` file.
- **Servers**:  List of potential servers.  If this is empty, the Device will search for servers using ZeroConf.
- **Local Time Zone**: The time zone that the logs were recorded in.
- **Threads**: The number of threads used for uploading from this device to the server.
- **Include Suffix**: The list of included suffixes.  Any file in the Watch directory (see above) that matches one of these suffixes will be uploaded to the server.
- **Watch Interval**: How long to wait in seconds before attempting to connect to server again.

Press [Save] to commit the changes to the device.  

Press [Refresh] to refesh the page with the on device settings.
