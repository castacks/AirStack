# MIN3D University 2 release intake

University 2 is a same-rig ground-floor–outdoor loop. The release lists
RealSense RGB and IMU, FLIR thermal, Livox and Velodyne LiDAR, shared
calibration, and a sequence-specific Riegl survey map. The reference
download is a LAS point cloud. The paper describes map-based evaluation
and shows an approximate route sketch. The reviewed release lists no
timestamped pose reference for this sequence.
[Paper, sections 3.2–3.4](https://doi.org/10.1007/s41064-023-00260-0),
[pinned release](https://github.com/3DOM-FBK/MIN3D/tree/e9103ae4b407018bfa9c4454bc47be5c14e71948#university-dataset).

The paper documents a central ROS clock and device-side synchronization
for RealSense streams and for Livox with its own IMU. Physical timing
between the standalone FLIR and the RealSense IMU remains to be
qualified. A camera-independent metric comparison would require
localizing LiDAR against the survey map and validating pose support and
timing through the transition before scoring camera estimates.

All six bounded requests for the calibration, RGB, thermal, Livox, LAS
map, and RealSense IMU failed TLS verification from this host. Diagnostic
requests with verification disabled reached an HTTP 503 page. The AZON
catalog linked to the same calibration endpoint. No input payloads were
available to check timestamp coverage or establish RGB tracking loss
and return. No estimator or map registration was attempted.
