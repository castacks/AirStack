# Local World Model

Local world models give the local planner a fast, short-range obstacle representation built directly from sensor data — cheaper and lower-latency than the global map, at the cost of limited spatial extent. AirStack's local world model is disparity-based:

- [**Disparity Expansion**](../../../../../robot/ros_ws/src/local/world_models/disparity_expansion/README.md) — C-space expansion of stereo disparity images by the robot radius
- [**Disparity Graph**](../../../../../robot/ros_ws/src/local/world_models/disparity_graph/README.md) — rolling window of expanded-disparity keyframes with their camera poses
- [**Disparity Graph Cost Map**](../../../../../robot/ros_ws/src/local/world_models/disparity_graph_cost_map/README.md) — cost-map plugin answering collision-cost queries for the DROAN local planner

The GPU planner `droan_gl` performs the expansion and graph internally on the GPU; the CPU pipeline uses these packages as separate nodes.
