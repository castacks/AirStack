Common top-level files for sub workspaces.

`ros_packages/` contains common ROS packages that are used between different machines, such as ground control station and the robot.
This folder is bind-mounted into the containers at `/root/AirStack/robot/ros_ws/src/common` (robot) and `/root/AirStack/gcs/ros_ws/src/common` (GCS).
