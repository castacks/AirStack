std_msgs/Header header 

# Initial condition information:
geometry_msgs/Pose start_pose #Plan from position and orientation
geometry_msgs/Vector3 wind_speed #Wind speed in m/s
float32 max_planning_time #seconds
float32 maximum_range #budget

#Constraints and objectives:
float32 desired_speed #Desired flight speed in m/s
geometry_msgs/Polygon search_bounds 

#Prior information:
SearchPrior[] search_priors 
KeepOutZone[] keep_out_zones

# Replan flag
bool clear_tree

# Scenario
uint32 scenario # To match up with the correct plan