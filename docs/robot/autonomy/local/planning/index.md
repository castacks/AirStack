# Local Planning

Local planners turn the coarse global plan into short, collision-free trajectories, reacting to obstacles the global map is too slow or too coarse to capture. Every AirStack local planner serves the same seam — the `NavigateTask` action at `/{robot_name}/tasks/navigate` in, trajectories to the [trajectory controller](../controls/index.md) out — so the controllers, safety monitor, takeoff/landing pipeline and GCS are planner-agnostic, and swapping planners is one include line in a stack's entry launch file.

AirStack's default local planner is **MIGHTY**, and the **DROAN** reactive planner is available as an alternative. Both ship as modules:

| Planner | Module | Stacks | World model | Sensor input | Compute |
|---|---|---|---|---|---|
| [**MIGHTY**](../../../../modules/mighty.md) (`mighty_bridge` + `mighty_node`) — Hermite-spline planner over convex safe corridors (MIT ACL, RA-L 2026) | [asm_mighty](https://github.com/castacks/asm_mighty) | `full_default` (default), `lite_default`, `lite_offload_global` | sliding voxel map with explicit unknown space ([acl-mapping](../world_model/index.md)) | 3D lidar cloud (`sensors/ouster/point_cloud`) | CPU |
| [**DROAN GL**](../../../../modules/droan.md) (`droan_gl`) — GPU disparity-space reactive planner (OpenGL shaders) | [asm_droan](https://github.com/castacks/asm_droan) | `full_droan`, `full_macvo` | disparity graph, held on the GPU | stereo disparity image | GPU |
| [**DROAN Local Planner**](../../../../modules/droan.md) (`droan_local_planner`) — the CPU implementation | [asm_droan](https://github.com/castacks/asm_droan) | `full_droan_cpu` | `disparity_expansion` → `disparity_graph` → `disparity_graph_cost_map` nodes | stereo disparity (or depth) image | CPU |

**Choosing:** MIGHTY holds a persistent map with tunable clearance margins and takes the full 360° lidar, so it is the choice where clutter is dense and clearance matters. DROAN keeps no map and needs only a stereo/depth camera, so it suits depth-camera-only vehicles and minimal compute budgets; its accumulated collision votes cannot be erased by looking again, which can turn cluttered pockets into hover states. The judged side-by-side evaluation behind the default is in the [asm_mighty README](https://github.com/castacks/asm_mighty#why-mighty-replaced-droan).

The trajectory controller's **look-ahead point** couples planner and controller: DROAN plans forward from it and appends `trajectory_segment_to_add` segments; MIGHTY replans anchored on the vehicle and publishes receding-horizon `trajectory_override` trajectories. Both are the [trajectory group (§5)](../../interface_conventions.md#5-trajectory-group-the-trajectory-controllers-contract-onboard-only) interchange.

Specialized maneuvers are handled by the [Takeoff Landing Planner](../../../../../robot/ros_ws/src/local/planners/takeoff_landing_planner/README.md), and DROAN's candidate trajectories come from the trunk [Trajectory Library](../../../../../robot/ros_ws/src/local/planners/trajectory_library/README.md).
