std_msgs/Header header
bool result                          # replanning success/failure
float64 successful_factor            # time allocation factor that succeeded
float64 total_replanning_ms          # total replanning wall-clock time
# Wrapper-stage breakdown of total_replanning_ms (housekeeping + global_planning + local_outer + append + final_housekeeping ~= total).
# local_outer_ms is the wrapper around STSFC generation + the parallel MIQP factor sweep; the difference
# (local_outer_ms - cvx_decomp_ms - local_traj_ms) is the parallel-sweep coordination overhead.
float64 housekeeping_ms              # initial state copy + readiness checks
float64 local_outer_ms               # wrapper around STSFC + parallel MIQP sweep (incl. sweep coord)
float64 append_ms                    # append optimized trajectory to plan
float64 final_housekeeping_ms        # final cleanup + counter reset
float64 global_planning_ms           # global path search (HGP solver internal time only — does NOT include update_map_ms and other generateGlobalPath setup)
float64 update_map_ms                # map update inside generateGlobalPath: rebuilds HGP voxel grid + KD-trees + dyn-obstacle inflation
float64 hgp_static_jps_ms            # static JPS sub-step
float64 hgp_check_path_ms            # path validity check
float64 hgp_dynamic_astar_ms         # dynamic A* sub-step
float64 hgp_recover_path_ms          # path recovery sub-step
float64 cvx_decomp_ms                # convex decomposition
float64 local_traj_ms                # Gurobi local trajectory optimization
float64 safe_paths_ms                # safe path computation
float64 safety_check_ms              # safety check
float64 yaw_sequence_ms              # yaw sequence generation
float64 yaw_fitting_ms               # yaw fitting
