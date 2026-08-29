# Disparity Graph Cost Map

A cost map plugin backed by a [disparity graph](../disparity_graph/README.md): it answers "what is the collision cost of this 3D point?" by projecting the query point into the expanded disparity keyframes stored in the graph. The [DROAN local planner](../../planners/droan_local_planner/README.md) loads it via its `cost_map` parameter (`disparity_graph_cost_map::DisparityGraphCostMap`) to score candidate trajectories.

Contact: Andrew Jong

Docs TODO. Help appreciated.
