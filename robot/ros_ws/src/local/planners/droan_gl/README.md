# DROAN GL

## Overview

This is an implementation of DROAN based local planner using OpenGL to do the disparity expansion and collision checking on the GPU. In the original version of DROAN, the disparity expansion expanded obstacles as flat surfaces facing the camera because it was expanding using the camera facing bounding box of a sphere instead of the sphere itself so that it could be implemented efficiently on the CPU. This version expands directly using the sphere.

Here is an example of the expansion. The red points are the raw disparity, black is the foreground expansion and white is the background expansion.

![expansion](images/expansion.png)

Trajectories are generated using snaps in different directions. Collision checking is done on a list of foreground and background disparity expansions. The current one is always used and new ones are added to the list based distance and how much the drone has rotated. Below is an image showing the trajectory library with collision free trajectories shown as green, trajectories with collisions shown as red and trajectories going into unseen areas as grey. This is a marker array which can be configured in rviz to show or not show the different classes of trajectories, collision free, in collision and unseen, and also show the individual points on the trajectories. The green arrows are the camera positions of the foreground and background disparity images used for collision checking. The cyan line is the path from the global planner and the blue line which overlaps the cyan line is the portion of the path that the local planner is using to score the trajectories. All of the foreground and background disparities from the list used in collision checking are visualized in the pointcloud, darker colors are foreground, lighter colors are background. The disparity expansion visualization can be toggled off since it significantly increased the CPU usage.

![trajectories](images/trajectories.png)

## Parameters

`target_frame` - The frame that visualizations and collision checking happen in.

`look_ahead_frame` - The frame where trajectories are planned from.

`rewind_info_frame` - The frame where the rewind info text is published. When a rewind is triggered, a text marker is published which shows the time duration left for the rewind and/or the remaining distance of the rewind.

`visualize` - Whether or not to publish visualizations of the expanded disparity. Visualizing significantly increased CPU usage.

`expansion_radius` - How much obstacles are expanded in meters.

`seen_radius` - The distance in meters around that drone that is considered to be seen.

`ht` - The total length in seconds of a trajectory.

`dt` - The increment in seconds for generating points along the trajectory.

`downsample_scale` - The factor by which the input disparity image will be downsampled.

`graph_nodes` - The number of expanded foreground and background disparity image pairs to do collision checking on.

`graph_distance_threshold` - The distance in meters after which a new node will be added to the list.
`graph_angle_threshold` - The angle in degrees that the drone has to yaw before a new node will be added to the list.

Rewinds are triggered by two events. When all trajectories are in collision from length of time the drone will rewind for a set duraiton. When the drone is considered to be staionary for too long, the drone will rewind for some distance or duration, whichever is satisfied first.

`all_in_collision_duration_threshold` - The amount of time in seconds all trajectories need to be in collision for a rewind to be triggered.

`all_in_collision_rewind_duration` - The amount of time in seconds the rewind will last for.

`stationary_history_duration` - The amount of time in seconds that the drone has to be stationary for a rewind to be triggered.

`stationary_distance_threshold` - If the drone moves less than this distance, it will be considered stationary.

`stationary_rewind_distance` - The distance in meters that the drone should rewind.

`stationary_rewind_duration` - The time in seconds that the drone should rewind. The rewind will end when either the distance or duration is met.
