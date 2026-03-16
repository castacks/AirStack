# Gossip-protocol peer state broadcast.
# Each robot publishes one of these on /gossip/peers at its own rate.

# Identity
string robot_name

# Current pose (position + orientation) extracted from odometry_conversion/odometry
# pose.header.stamp carries the odometry timestamp
geometry_msgs/PoseStamped pose

# Current navigation goal – last pose in the global planner's published path.
# All-zero header stamp signals that no plan is available yet (null waypoint).
geometry_msgs/PoseStamped waypoint

# Gossip metadata
uint8 SOURCE_DIRECT  = 0
uint8 SOURCE_RELAYED = 1
uint8 source      # how this message reached us (unused in Phase 1, reserved)
uint8 relay_hops  # number of relay hops (unused in Phase 1, reserved)

# Arbitrary typed payloads – any number, any ROS message type
PeerProfilePayload[] payloads
