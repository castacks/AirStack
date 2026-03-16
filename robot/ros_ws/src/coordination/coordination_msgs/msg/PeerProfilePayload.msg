# A single typed payload carried inside a PeerProfile.
# payload_type holds the fully-qualified ROS message type string,
# e.g. "nav_msgs/msg/OccupancyGrid". payload_data holds the
# serialized bytes produced by rclpy.serialization.serialize_message().
string  payload_type
uint8[] payload_data
