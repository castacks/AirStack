
AirStack is designed for multi-robot development, and is setup to run multiple robots in simulation.

To mimic interacting with real world robots, we use Docker Compose to manage Docker containers that isolate the simulation, each robot, and the ground control station.


The details of the docker compose setup is in `AirStack/docker/docker-compose.yaml`.

In essence, the compose file launches:

- Isaac Sim
- ground control station
- robots

all get created on the same default Docker bridge network. 
This lets them communicate with ROS2 on the same network.


Each robot has its own ROS_DOMAIN_ID.

## Start and Stop
Start
```bash
docker compose up -d --scale robot=[NUM_ROBOTS]

# see running containers
docker ps -a
```

Stop
```bash
docker compose down
```


## SSH into Robots
The containers mimic the robots' onboard computers on the same network. Therefore we intend to interface with the robots through ssh.

The `ground-control-station` and `docker-robot-*` containers are setup with ssh daemon, so you can ssh into the containers using the IP address.

You can get the IP address of each container by running the following command:

```bash
docker inspect -f '{{range.NetworkSettings.Networks}}{{.IPAddress}}{{end}}' [CONTAINER-NAME]
```
Then ssh in, for example:
```bash
ssh root@172.18.0.6
```

The ssh password is `airstack`.


## Container Details


### Isaac Sim
Start a bash shell in the Isaac Sim container:
```bash
docker exec -it isaac-sim bash
```


`runapp` launches Isaac Sim.
The `--path` argument can be passed with a path to a `.usd` file to load a scene.

It can also be run in headless mode with `./runheadless.native.sh` to stream to Omniverse Streaming Client or `./runheadless.webrtc.sh` to stream to a web browser.

The container also has the isaacsim ROS2 package within that can be launched with `ros2 launch isaacsim run_isaacsim.launch.py`. 

### Robot
Start a bash shell in a robot container, e.g. for robot_1:
```bash
docker exec -it docker-robot-1 bash
```

```bash
# in robot docker
cws  # cleans workspace
bws  # builds workspace
sws  # sources workspace
ros2 launch robot_bringup robot.launch.xml  # top-level launch 
```

These aliases are in the `~/.bashrc` file.

Each robot has `ROS_DOMAIN_ID` set to its ID number. `ROBOT_NAME` is set to `robot_$ROS_DOMAIN_ID`.

### Ground Control Station
Currently the ground control station uses the same image as the robot container. This may change in the future.

Start a bash shell in a robot container:
```bash
docker exec -it ground-control-station bash
```

The commands are currently the same.

`ROS_DOMAIN_ID` is set to 0.