Common top-level files for sub workspaces.

[`human_detector_model/`](human_detector_model/README.md) is a standalone,
tunable human-detection surrogate for planner experiments, with Python and JSONL
interfaces. It is excluded from colcon discovery.

`ros_packages/` contains common ROS packages that are used between different machines, such as ground control station and the robot.
This folder is bind-mounted into the containers at `/root/AirStack/robot/ros_ws/src/common` (robot) and `/root/AirStack/gcs/ros_ws/src/common` (GCS).
