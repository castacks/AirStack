# Full trajectory message for robust tracking with replan support
std_msgs/Header header
uint32 trajectory_id          # Increments on each replan, used to detect new trajectories
Goal[] goals                  # Array of trajectory points with p, v, a, yaw, dyaw
float64 dt                    # Time step between goals (should be 0.01 for 100Hz)
