# SearchPlan — one agent's flight-ready MTL sortie, published atomically.
#
# Everything the follower and the metrics logger need to agree on in ONE
# message (latched / transient-local), so the track, the boresight schedule and
# the arc-length index can never be received out of step.
#
# Frame: header.frame_id is the robot's odometry frame ("map"): its PX4/MAVROS
# local origin = where the robot spawned. world_ENU = map + map_origin_in_world.

std_msgs/Header header
string plan_id                  # "<scenario>/<agent>/<run_id>"
string run_id
string scenario_name
string agent_name
int32 agent_index
bool start_mission              # false = preview only; follower stays idle

# --- platform / sensor model the plan was made for ---------------------------
bool single_axis_gimbal         # 1-DOF cross-track mount (true) or 2-DOF pan/tilt
float64 mount_tilt_rad          # forward bracket tilt tau (single-axis only)
float64 fov_rad                 # full cone angle
float64 speed_mps
float64 min_turn_radius_m
float64 altitude_m
float64 dt_s                    # planner sample period
float64 gimbal_max_rad          # 1-DOF travel from the mount axis
float64 gimbal_rate_rad_s       # 1-DOF slew-rate limit
float64 pitch_nudge_max_rad     # |delta p| bound of the scheduler (5 deg; 0 for the curve planner)
string gimbal_law               # single-axis gimbal law the follower flies:
                                #   "open_loop"  replay the planned cross-track angle (roll + phi), tilt fixed
                                #   "aim_point"  aim at the planned ground point from the actual pose (+/-5 deg pitch nudge)
                                #   ""           = "aim_point" (a planner that names none, e.g. TIGRIS)
geometry_msgs/Vector3 map_origin_in_world   # world ENU of the map origin (the agent's home)

# --- the track: N samples, all arrays share one index --------------------------
airstack_msgs/TrajectoryXYZVYaw trajectory  # position [map], planned speed, ENU yaw
geometry_msgs/Point[] boresight             # scheduled ground point per sample [map]
float64[] arc_length_m                      # flown ground-track arc length
float64[] time_s                            # planner timeline
float64[] planned_gimbal_phi_rad            # 1-DOF cross-track angle (+ right; crossAngle = roll + phi); 0 in 2-DOF mode
float64[] planned_pitch_rad                 # planned airframe pitch (look angle nudge)
float64[] planned_roll_rad                  # planned airframe roll (empty = 0; the open_loop law replays roll + phi)

int32[] serviced_cells          # global cell indices the gimbal is scheduled to observe
float64 planned_length_m
float64 budget_m
