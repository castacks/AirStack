# DROAN Local Planner


This is part of a set of packages that performs local obstacle avoidance from stereo depth. The method is based on the publication ["DROAN - Disparity-space representation for obstacle avoidance."](https://www.ri.cmu.edu/app/uploads/2018/01/root.pdf). Essentially, the local world model makes a C-space expansion around detected obstacles in the disparity image. The local planner then plans a path by scoring the best trajectory from a trajectory library in this expanded space.

The packages are
 
 1. [disparity_expansion](../../a_world_models/disparity_expansion/README.md)   (takes in stereo disparity and expands it to a 3D point cloud)
 2. [disparity_graph](../../a_world_models/disparity_graph/README.md)        (creates a graph where each node is a drone pose and 3D point cloud observation)
 3. [disparity_graph_cost_map](../../a_world_models/disparity_graph_cost_map/README.md)  (creates a cost map from the disparity graph)
 4. droan_local_planner  (this package. uses the cost map to plan a path)


DROAN local planner takes the global plan, and finds the global plan's closest point to the drone.
DROAN trims the global plan to be from that point to the end of the global plan.

Consequently, DROAN currently does NOT use a waypoint manager, and is NOT guaranteed to reach every waypoint on the global plan, especially if the global plan loops back on itself.
Ideally in the future we will use a proper waypoint manager.