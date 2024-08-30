```
# make sure in this directory
# build the image, it is named airlab-autonomy-dev:latest
docker compose --profile build build

# start docker compose service
docker compose up -d 

# enter a bash session
docker compose exec airstack_dev bash

# launch the isaacsim with two drones
cd ~/Airstack/ros_ws
source install/setup.bash
ros2 launch central ascent_isaac_two_drones.xml
# wait until you see the odometry appear in rviz then do the following in the RQT GUI
Click the "Offboard" button, then the "Arm" button, then the "Takeoff" button
# After a few seconds the drone should takeoff
Click publish on the fixed trajectory part of the GUI to send different trajectory patterns that the drone will follow
# After killing the ros2 launch, run the following to kill the SITL, mavproxy, and mavros that is running in the background, this will be handled automatically in the future
tmux kill-server

# stop service
docker compose stop 
```