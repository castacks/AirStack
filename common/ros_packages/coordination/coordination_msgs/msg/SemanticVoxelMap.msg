# Full sender-owned semantic map snapshot, not the receiver's fused map.
# cloud frame: world_xy_agl (global ENU XY, ground-relative Z).
# Fields: x,y,z,sim_0,...,sim_N; sim_i corresponds to labels[i].
string[] labels
float32 voxel_size
sensor_msgs/PointCloud2 cloud
