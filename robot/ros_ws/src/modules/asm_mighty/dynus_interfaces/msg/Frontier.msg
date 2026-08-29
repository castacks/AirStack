# A single frontier produced by MIGHTY's frontier manager.
# Coordinates are in the map frame (see FrontierList.header.frame_id).

uint8 STATE_ACTIVE=0
uint8 STATE_DORMANT=1
uint8 STATE_VISITED=2
uint8 STATE_INVALIDATED=3

uint64 id                       # stable identifier across cycles
uint8  state                    # STATE_* constant above
geometry_msgs/Point centroid    # cluster centroid (x, y; z is the goal plane)
float64 size_m2                 # cluster area
uint64  size_cells              # cluster size in grid cells
geometry_msgs/Point aabb_min    # cluster bounding box, lower corner
geometry_msgs/Point aabb_max    # cluster bounding box, upper corner
float64 cached_utility          # last utility score from the selector
