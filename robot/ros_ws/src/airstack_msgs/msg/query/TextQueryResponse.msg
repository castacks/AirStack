# This message type is used for the response of a text query from a robot semantic query

std_msgs/Header header      # Header for the message, usually contains timestamp and frame_id
string tag_name             # The tag associated with the query or response
sensor_msgs/NavSatFix[] geofence  # Array of GPS fixes (NavSatFix) representing the geofence
