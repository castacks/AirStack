## Overview

The details of the docker containers setup is in the `docker-compose.yaml` file in the `AirStack` directory.

Isaac Sim, the ground control station, and robots all get created on the same default Docker bridge network.
This lets them communicate with ROS2 on the same network.

Each robot has its own ROS_DOMAIN_ID.

## SSH into Robots

The `gcs` and `docker-robot` containers are setup with ssh daemon, so you can ssh into the containers using the IP address.

You can get the IP address of each container by running the following command:

```bash
docker inspect -f '{{range.NetworkSettings.Networks}}{{.IPAddress}}{{end}}' [CONTAINER-NAME]
```

Then ssh in, for example:

```bash
ssh root@172.18.0.6
```

The ssh password is `airstack`.
