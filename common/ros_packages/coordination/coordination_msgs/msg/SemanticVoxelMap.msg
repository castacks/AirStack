# Sender-owned semantic map or lossless update, never the receiver's fused map.
# cloud frame: world_xy_agl (global ENU XY, ground-relative Z).
# Fields: x,y,z,sim_0,...,sim_N; sim_i corresponds to labels[i].
string source_robot
bool is_delta
uint64 base_revision
string[] labels
float32 voxel_size
sensor_msgs/PointCloud2 cloud
