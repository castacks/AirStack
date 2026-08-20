# Local Planning

Part of the local planner is the Waypoint Manager.

The Waypoint Manager subscribes to the global waypoints and the drone's current position and publishes the next waypoint to the local planner.

We plan for this baseline to be DROAN.

The [External Vision Planner Bridge](../../../../../robot/ros_ws/src/local/planners/mononav_bridge/README.md)
also connects camera-based GPU planners such as MonoNav and Collision-avoidance to the
trajectory controller. Their model dependencies stay in separate containers while one
shared ROS 2 bridge handles camera/pose transport, visualization, pause, and trajectory
override commands.
