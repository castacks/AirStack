std_msgs/Header header 

# Initial condition information:
geometry_msgs/PoseStamped[] goal_poses # List of goal poses for multi-goal planning

# Hyperparameters
float32 max_planning_seconds # seconds

# Constraints and objectives:
float32 maximum_range # budget
geometry_msgs/Polygon[] stay_within_constraints # Stay within these polygons. If empty, no constraints are applied
geometry_msgs/Polygon[] keep_out_constraints # Keep out of these polygons. If empty, no constraints are applied