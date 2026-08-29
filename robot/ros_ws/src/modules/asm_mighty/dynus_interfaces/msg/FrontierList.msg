# Structured frontier dump intended for external selectors (e.g. VLM nodes).
# Published alongside the visualization MarkerArray, but with stable schema.

std_msgs/Header header          # frame_id is the map frame
dynus_interfaces/Frontier[] frontiers
