# Use this message to command the outer loop to track
# a trajectory generated from a high-level trajectory planner.

std_msgs/Header header

# Current time-slice of desired trajectory
geometry_msgs/Vector3 p # position
geometry_msgs/Vector3 v # velocity
geometry_msgs/Vector3 a # acceleration
geometry_msgs/Vector3 j # jerk

float64 yaw # angle as defined in Sec. III of https://arxiv.org/pdf/2103.06372.pdf  
# In general, it is will not be the rXYZ yaw Euler angle of the UAV (unless the drone is in a hover condition).
# This is due to the fact that q_psi (see link above) already has some rXYZ yaw on it.
# See also the paper https://link.springer.com/chapter/10.1007/978-3-030-28619-4_20
float64 dyaw #d{psi}/dt

bool power # true if motors should be able to spin

# Trajectory tracking mode constants
uint8 MODE_POSITION_CONTROL     = 0
uint8 MODE_VELOCITY_CONTROL     = 1
uint8 MODE_ACCELERATION_CONTROL = 2

# Trajectory tracking mode for x/y and z components.
# The default is POSITION control, which uses position and velocity error
# to calculate the control effort. VELOCITY control only uses vel error.
# ACCELERATION mode does not use tracking error and could be used to provide
# a control signal computed from something other than the default PID cntrl.
uint8 mode_xy
uint8 mode_z

