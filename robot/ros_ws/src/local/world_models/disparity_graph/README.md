# Disparity Graph

Maintains a sliding graph of keyframe disparity images and their camera poses for occupancy queries by disparity-based cost maps. A single disparity frame only covers the current camera view; keeping a rolling window of past (pose, expanded-disparity) keyframes gives the local planner spatial memory of obstacles the camera is no longer looking at.

Used by [`disparity_graph_cost_map`](../disparity_graph_cost_map/README.md), which serves collision costs to the [DROAN local planner](../../planners/droan_local_planner/README.md). Expanded disparity inputs come from [`disparity_expansion`](../disparity_expansion/README.md).

Contact: Andrew Jong

Docs TODO. Help appreciated.
