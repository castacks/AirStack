std_msgs/Header header

uint8 POLYGON_PRIOR = 1 # must be convex polygon
uint8 LINE_SEG_PRIOR = 2
uint8 POINT_PRIOR = 3
uint8 grid_prior_type

geometry_msgs/Polygon points_list # polygon, line seg, or list of point
float32[] value # initial value for the polygon, line seg, or point

# Optional values
float32[] priority # Higher values mean higher priority. If empty then the priority is set to 1.0
uint8[] sensor_model_id # the type of sensor model for that region type. Default is 0