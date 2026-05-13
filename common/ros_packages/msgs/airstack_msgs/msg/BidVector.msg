string robot_name

# Per-ray-group bid rows. Each i is one ray group this robot sees; multiple
# rows may share a label (one per distinct group). Receivers run
# is_same_target() between own groups and peer rows to decide whether two
# entries with the same label are competing for the same physical target or
# pointing at different ones.
string[]  labels
float64[] values        # bid (= -min_dist_to_robot). higher = closer = wins.

# Ray-group descriptor, parallel arrays to labels[]/values[].
# Frame: publisher's local 'map' (gossip translates origin_x/y/z to global ENU
# before sending; receivers convert back via global_enu_to_local_batch).
float64[] origin_x
float64[] origin_y
float64[] origin_z
float64[] dir_x         # unit vector (FLU)
float64[] dir_y
float64[] dir_z
int32[]   num_rays
float64[] avg_score
