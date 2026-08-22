# Static Transforms

Static transforms pin down the fixed geometric relationships between frames — where sensors sit on the body, and how each robot's map frame relates to the world — so that every module can transform data into a common frame without per-module configuration.

## Frame Conventions

Each robot has its own **map** frame that represents the starting position of the robot.
The **map** frame is expected to be in ENU (East-North-Up) convention. 

The robot is in the **base_link** frame.
