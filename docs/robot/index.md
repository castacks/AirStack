# Robot

## Directory Structure
Underneath `AirStack/robot`, there are these directories:
- `docker/`: Contains files related to building and launching the robot Docker container.
- `installation/`: Contains files related to installing the robot software on a physical robot (TODO).
- `ros_ws/`: Contains the ROS 2 workspace for the robot.

## Launch Structure
Each high-level module under `ros_ws/` has a `[module]_bringup` package that contains the launch files for that module. The launch files are located in the `launch` directory of the `[module]_bringup` package. The launch files are named `*.launch.(xml/yaml/py)` and can be launched with `ros2 launch <module_name>_bringup <module_name>.launch.(xml/yaml/py)`.

At a high level, the launch files are organized as follows:


```
- robot_bringup/: robot.launch.xml
    - autonomy_bringup/: autonomy.launch.xml
        - interface_bringup/: interface.launch.xml
        - sensors_bringup/: sensors.launch.xml
        - perception_bringup/: perception.launch.xml
        - local_bringup/: local.launch.xml
        - global_bringup/: global.launch.xml
        - behavior_bringup/: behavior.launch.xml
```

## Configuration

### Desktop vs Jetson
If you look at the `robot/docker/docker-compose.yaml` file, you'll see it contains two services. `robot` is meant for x86-64 desktop development whereas `robot_l4t` is meant to run on NVIDIA Jetson devices. Both extend a base service in `robot/docker/robot-base-docker-compose.yaml`.

## Common Topics
| Topic                          | Type              | Description                                                                                                                             |
| -------------------------------| ------------------| ---------------------------------------------------------------------------------------------------------------------------|
| `/$ROBOT_NAME/odometry`        | [nav_msgs/Odometry](https://docs.ros.org/en/rolling/p/nav_msgs/interfaces/msg/Odometry.html) | Best estimate of robot odometry
| `/$ROBOT_NAME/global_plan`     | [nav_msgs/Path](https://docs.ros.org/en/rolling/p/nav_msgs/interfaces/msg/Path.html)     | Current target global trajectory for the robot to follow. See [global planning](4_global/planning.md) for more details.

### Rough System Diagram
![AirStack System Diagram](airstack_system_diagram.png)