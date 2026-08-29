# Message describing a 3-dimensional rectangular box.
# Origin is defined as the corner of the box with the minimum x, y, and z values
# in the cartesian coordinate system provided by header.frame.
# Size gives the x, y, and z dimensions of the box.

std_msgs/Header header
geometry_msgs/Vector3 origin
geometry_msgs/Vector3 size
