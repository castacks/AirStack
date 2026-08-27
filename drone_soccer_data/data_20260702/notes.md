# Notes for 20260702

- bags/drone2_volleyball_20260702_163925: Random manual flight
- bags/drone2_volleyball_20260702_185306: Position command to 0, -2, 1.0.
- bags/drone2_volleyball_20260702_185819: Position command to 0, -2, 1.8.
- bags/drone2_volleyball_20260702_190427: Manual flight with ball starting around y=-2. Mat in the middle so affects ball rolling. Not very accurate.
- bags/drone2_volleyball_20260702_191012: Manual flight, ball went into net in +x direction.
- bags/drone2_volleyball_20260702_191334: Manual flight, ball went into net in +x direction.
- bags/drone2_volleyball_20260702_191550: Manual flight, ball went into net in -x direction.

- bags/drone2_soccerball_20260702_192953: Manual flight, ball went into net in +x direction.

## Future test

- More principled testing with trajectory waypoints
- Add a trajectory waypoint stack to replace/modify `swarm_commander.py` (Make our own ground controller / interface)
- Better recording pipeline, better rosbag visualization script
- Record downward camera rostopic


develop branch on airstack
- plug in external controller

bypass the pd loop and let px4 handle position -> to velocity input