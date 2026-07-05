# Gossip-protocol peer state broadcast.
# Each robot publishes one of these on /gossip/peers at its own rate.

# Identity
string robot_name

# Monotonic per-origin message id; (robot_name, seq) dedups relayed copies
uint32 seq

# Current GPS position from interface/mavros/global_position/raw/fix
# gps_fix.header.stamp carries the NavSatFix timestamp
sensor_msgs/NavSatFix gps_fix

# Heading in degrees clockwise from North (0-360), from compass_hdg
float64 heading

# Current navigation goal – last pose in the global planner's published path.
# All-zero header stamp signals that no plan is available yet (null waypoint).
geometry_msgs/PoseStamped waypoint

# Gossip metadata
uint8 SOURCE_DIRECT  = 0
uint8 SOURCE_RELAYED = 1
uint8 source      # SOURCE_DIRECT or SOURCE_RELAYED
uint8 relay_hops  # number of relay hops taken
float64 relay_lat # last-hop transmitter position (valid when source==SOURCE_RELAYED)
float64 relay_lon

# Arbitrary typed payloads – any number, any ROS message type
PeerProfilePayload[] payloads
