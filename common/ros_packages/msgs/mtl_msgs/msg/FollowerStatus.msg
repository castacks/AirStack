# FollowerStatus — mtl_trajectory_follower progress along the active SearchPlan.

uint8 IDLE=0        # passthrough of the nominal tracking point
uint8 INGRESS=1     # flying to the track start at mission altitude
uint8 SEARCH=2      # carrot pursuit along the track, gimbal on schedule
uint8 COMPLETE=3    # reached the end of the track; holding
uint8 ABORTED=4     # cancelled, state-estimate timeout, or no odometry

std_msgs/Header header
string plan_id
uint8 state
string state_name
float64 progress_m              # arc length reached (projection, interpolated)
float64 total_m
float64 remaining_m
float64 cross_track_error_m     # distance from the vehicle to the track
float64 elapsed_s               # sim time since the plan was accepted
int32 track_index
geometry_msgs/Point carrot      # current setpoint [map]
geometry_msgs/Point aim_point   # boresight target the gimbal is commanded at [map]
geometry_msgs/Vector3 gimbal_cmd   # commanded (roll, pitch, yaw) [rad, earth frame]
