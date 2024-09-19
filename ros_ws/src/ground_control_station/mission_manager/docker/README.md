# Debugging this node with Docker

```
docker build -t airstack-gs -f ros_ws/src/ground_control_station/mission_manager/docker/Dockerfile .
```

```
docker run -it --rm --name airstack_groundstation --mount type=bind,source="$(pwd)",target=/root/AirStack airstack-gs /bin/bash
```
