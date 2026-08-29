uint32 class_id
string class_name
float32 confidence

# (x_min, y_min), (x_max, y_max) define the bounding box of the detection in pixel coordinates
float32 x_min 
float32 y_min
float32 x_max
float32 y_max

# ground truth unique identifier and position of the detected object (Simulation only) 
uint32 object_id
float32 x_pos
float32 y_pos
float32 z_pos
