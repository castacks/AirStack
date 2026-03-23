# A single typed payload carried inside a PeerProfile.
# payload_type holds the fully-qualified ROS message type string,
# e.g. "nav_msgs/msg/OccupancyGrid". payload_data holds the
# serialized bytes produced by rclpy.serialization.serialize_message().
# stamp is the time the source topic was last received — receivers can use
# this to detect stale payloads independently of the gossip message timestamp.
builtin_interfaces/Time stamp
string  payload_type
uint8[] payload_data
