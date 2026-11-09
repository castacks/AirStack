# Sender-owned semantic map or lossless update, never the receiver's fused map.
# cloud frame: world_xy_agl (global ENU XY, ground-relative Z).
# Fields: x,y,z,sim_0,...,sim_N; sim_i corresponds to labels[i].
# Reusable-feature mode labels identify RGB, count and full encoder feature
# channels (feature:<encoder-space-hash>:<index>), rather than query scores.
string source_robot
bool is_delta
uint64 base_revision
string[] labels
float32 voxel_size
sensor_msgs/PointCloud2 cloud
# Empty representation means the existing expanded/score cloud. Native PCA
# maps carry RGB/count + coefficients in cloud; basis is sent on full resyncs.
string representation
string feature_space
string basis_id
uint32 basis_rows
uint32 basis_cols
uint32 decoded_dimension
float32[] pca_basis
